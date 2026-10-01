# =============================================================
# drone_flight.py — 드론 비행 (docs/PLAN.md 2-2, Pegasus Simulator 방식)
#
#   제어기 → 로터 4 개 목표 각속도 ω (rad/s) → 이차 추력 모델 → PhysX 외력 (매 physics step, PHYSICS_PRE_STEP 콜백)
#     - 로터 i 강체에 자기 z 축 방향 추력 Fᵢ = k·ωᵢ² (로터 위치에서)
#     - body 에 반토크 Σ c·ωᵢ²·dirᵢ (body z 축) + 선형 공기저항 −D·v_body
#   제어기: 기하 제어기 (Pegasus 예제 NonlinearController 와 같은 식, Lee/Mellinger). 2단계 2번에서 PX4 로 바꾼다
#     F_des = −Kp·e_p − Kd·e_v − Ki·∫e_p + m·g·ẑ + m·a_ref → 몸체를 F_des 방향으로 기울이고 u₁ = F_des·Z_B
#     τ = −Kr·e_R − Kw·(ω − ω_des) (ω_des = 목표 jerk 투영),  (u₁, τ) → 할당 행렬 역행렬 → ω²  (음수는 0, 최대 ω 넘으면 비율 유지 축소)
#   Pegasus 와 다른 점 (2026-10-01 코드 비교, PLAN 2-2): 게인을 PhysX 질량·관성으로 계산(Pegasus 는 m 1.5 고정, 세 축 같은 Kr·Kw),
#     제어기 출력을 같은 step 에 적용(Pegasus 는 backend 를 vehicle.update 끝에 갱신해 한 step 늦음), physics 1/120 s (Pegasus 1/250 s),
#     yaw 목표 = 시작 yaw 고정. 센서(IMU·GPS 등)는 없음
#   모드: static (목표 고정), hover (목표 + 축별 사인파 합, seed), trajectory (미구현 → 에러)
#   release(): 모터 정지 (ω = 0 → 추력 0. 실물: 잡은 뒤 모터를 멈춤). 공기저항은 계속
#   프로펠러 회전 (보여 주기용, prop_spin): Pegasus handle_propeller_visual 과 같이 실제 ω 와 무관하게
#     추력 ≥ 0.1 N 이면 prop_visual_speed, 0 < 추력 < 0.1 N 이면 prop_idle_speed, 0 이면 정지 (관절 속도를 매 step 덮어씀)
#
# 설정: drone_*.yaml 의 flight. 질량·관성은 PhysX 값을 읽어 게인을 계산한다 (Kp = m·ω², Kr = I·ω² …)
# SimulationApp 을 만든 뒤에 import 할 것.
# =============================================================

import math

import numpy as np

import drone as dr

G = 9.81
MODES = ("static", "hover", "trajectory")
FLIGHT_KEYS = ("rotors", "rot_dir", "rotor_constant", "rolling_moment_coefficient", "max_rotor_velocity", "linear_drag",
               "prop_joints", "prop_visual_speed", "prop_idle_speed", "controller", "hover")
VISUAL_FORCE = 0.1   # N, Pegasus 프로펠러 표시 기준


def quat_to_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def vee(M):
    return np.array([M[2, 1], M[0, 2], M[1, 0]])


class Reference:
    """목표 위치·속도·가속도·jerk (world). static: p0 고정. hover: p0 + 축별 사인파 합 (진폭 합 = amplitude)."""

    def __init__(self, mode, p0, hover_cfg=None, seed=0):
        if mode not in MODES:
            raise ValueError(f"mode 는 {MODES} 중 하나: {mode}")
        if mode == "trajectory":
            raise NotImplementedError("trajectory 모드는 아직 구현하지 않음 (PLAN 2단계: 나중에)")
        self.mode, self.p0 = mode, np.asarray(p0, dtype=float)
        self.params = {"mode": mode, "p0": self.p0.tolist(), "seed": int(seed)}
        if mode == "hover":
            rng = np.random.default_rng(seed)
            n = int(hover_cfg["n_sines"])
            amp = np.asarray(hover_cfg["amplitude"], dtype=float)
            lo, hi = (float(x) for x in hover_cfg["freq_range"])
            self.A = np.repeat(amp[:, None] / n, n, axis=1)                  # (3, n)
            self.f = rng.uniform(lo, hi, size=(3, n))
            self.phi = rng.uniform(0, 2 * math.pi, size=(3, n))
            self.params.update(amplitude=amp.tolist(), freq=self.f.tolist(), phase=self.phi.tolist())

    def __call__(self, t):
        if self.mode == "static":
            return self.p0.copy(), np.zeros(3), np.zeros(3), np.zeros(3)
        w = 2 * math.pi * self.f
        arg = w * t + self.phi
        # 시작 순간 목표가 p0 이 되도록 sin(φ) 만큼 뺌 (튀지 않게)
        p = self.p0 + (self.A * (np.sin(arg) - np.sin(self.phi))).sum(axis=1)
        v = (self.A * w * np.cos(arg)).sum(axis=1)
        a = (-self.A * w * w * np.sin(arg)).sum(axis=1)
        j = (-self.A * w ** 3 * np.cos(arg)).sum(axis=1)
        return p, v, a, j


class GeometricController:
    """Pegasus NonlinearController.update 와 같은 식 (yaw 목표 = 시작 yaw 고정, yaw rate 목표 0).
    다른 점: Kp·Kd·Ki 는 세 축 같은 스칼라(Pegasus 도 세 축 같은 값), Kr·Kw 는 축별 관성 비례 (Pegasus 는 세 축 같은 값)."""

    def __init__(self, mass, inertia_diag, ccfg):
        p, a = ccfg["position"], ccfg["attitude"]
        self.m = mass
        self.Kp = mass * p["omega"] ** 2
        self.Kd = 2 * p["zeta"] * mass * p["omega"]
        self.Ki = mass * p["ki"]
        I = np.asarray(inertia_diag, dtype=float)
        self.Kr = I * a["omega"] ** 2
        self.Kw = 2 * a["zeta"] * I * a["omega"]
        self.integral = np.zeros(3)
        self.yaw = 0.0

    def reset(self, yaw):
        self.integral[:] = 0.0
        self.yaw = float(yaw)

    def update(self, dt, p, v, R, w_world, ref):
        p_ref, v_ref, a_ref, j_ref = ref
        ep, ev = p - p_ref, v - v_ref
        self.integral += ep * dt
        F_des = -self.Kp * ep - self.Kd * ev - self.Ki * self.integral + np.array([0.0, 0.0, self.m * G]) + self.m * a_ref
        Z_B = R[:, 2]
        u1 = float(F_des @ Z_B)
        Z_des = F_des / np.linalg.norm(F_des)
        X_c = np.array([math.cos(self.yaw), math.sin(self.yaw), 0.0])
        Y_des = np.cross(Z_des, X_c)
        Y_des /= np.linalg.norm(Y_des)
        X_des = np.cross(Y_des, Z_des)
        R_des = np.column_stack([X_des, Y_des, Z_des])
        e_R = 0.5 * vee(R_des.T @ R - R.T @ R_des)
        # 목표 각속도: jerk 를 X_b–Y_b 평면에 투영 (Pegasus 식 (7), [Mellinger & Kumar]). yaw rate 목표 0
        hw = (self.m / u1) * (j_ref - (Z_des @ j_ref) * Z_des)
        w_des = np.array([-(hw @ Y_des), hw @ X_des, 0.0])
        e_w = R.T @ w_world - w_des                        # 몸체 좌표계 각속도 (Pegasus state.angular_velocity 와 같음)
        tau = -self.Kr * e_R - self.Kw * e_w
        return u1, tau


class DroneFlight:
    """드론 비행 (physics 콜백). start() 전에 재생(play) 되어 있어야 한다. 콜백 예외는 check() 에서 다시 올린다."""

    def __init__(self, cfg, info, mode="static", seed=0, target=None):
        from isaacsim.core.experimental.prims import RigidPrim

        if "flight" not in cfg:
            raise KeyError(f"{cfg['path']}: flight 항목이 없습니다")
        fc = cfg["flight"]
        for k in FLIGHT_KEYS:
            if k not in fc:
                raise KeyError(f"{cfg['path']}: 'flight.{k}' 항목이 없습니다")
        n = len(fc["rotors"])
        if len(fc["rot_dir"]) != n:
            raise ValueError("flight.rot_dir 길이 != rotors")
        self.fc, self.info = fc, info
        path = info["path"]
        self.body = RigidPrim(info["body_path"])
        self.rotors = RigidPrim([dr._sub(path, r) for r in fc["rotors"]])
        self.links = RigidPrim(info["bodies"])
        self.k = float(fc["rotor_constant"])
        self.c = float(fc["rolling_moment_coefficient"])
        self.dir = np.asarray(fc["rot_dir"], dtype=float)
        self.w_max = float(fc["max_rotor_velocity"])
        self.drag = np.asarray(fc["linear_drag"], dtype=float)

        self.mass, self.inertia = self._mass_properties()
        self.ctrl = GeometricController(self.mass, np.diag(self.inertia), fc["controller"])
        p, q = self._body_pose()
        self.ref = Reference(mode, p if target is None else target, fc["hover"], seed)

        self.spin = None
        if info["prop_spin"]:
            from isaacsim.core.experimental.prims import Articulation

            self.art = Articulation(path)
            names = list(self.art.dof_names)
            joints = [j.split("/")[-1] for j in fc["prop_joints"]]
            missing = [j for j in joints if j not in names]
            if missing:
                raise RuntimeError(f"프로펠러 관절 DOF 가 없음: {missing} (DOF: {names})")
            self.spin = np.array([names.index(j) for j in joints])

        self.armed, self.t, self.t_release = False, 0.0, None
        self.omega = np.zeros(n)
        self.forces = np.zeros(n)
        self.prop_cmd = np.zeros(n)
        self.u1, self.tau = 0.0, np.zeros(3)
        self._cb, self.cb_error = None, None

    # ── 질량·관성 (PhysX) ──
    def _body_pose(self):
        p, q = (x.numpy().reshape(-1) for x in self.body.get_world_poses())
        return p, q

    def _mass_properties(self):
        """전체 질량, body 좌표계 기준 전체 무게중심 둘레 관성 (링크 관성 + 평행축)."""
        m = self.links.get_masses().numpy().reshape(-1)
        pos, quat = (x.numpy() for x in self.links.get_world_poses())
        com_l = self.links.get_coms()[0].numpy().reshape(-1, 3)
        Il = self.links.get_inertias().numpy().reshape(-1, 3, 3)
        pb, qb = self._body_pose()
        Rb = quat_to_R(qb)
        Rs = [quat_to_R(q) for q in quat]
        com_w = np.array([p + R @ c for p, R, c in zip(pos, Rs, com_l)])
        c_tot = (m[:, None] * com_w).sum(axis=0) / m.sum()
        I = np.zeros((3, 3))
        for mi, R, Ii, cw in zip(m, Rs, Il, com_w):
            Rr = Rb.T @ R
            r = Rb.T @ (cw - c_tot)
            I += Rr @ Ii @ Rr.T + mi * (r @ r * np.eye(3) - np.outer(r, r))
        return float(m.sum()), I

    # ── 명령 ──
    def arm(self, target=None):
        """모터 시동 + 현재 자세의 yaw 로 제어 시작 (target 을 주면 static·hover 의 기준 위치를 바꿈)."""
        p, q = self._body_pose()
        R = quat_to_R(q)
        self.ctrl.reset(math.atan2(R[1, 0], R[0, 0]))
        if target is not None:
            self.ref.p0 = np.asarray(target, dtype=float)
            self.ref.params["p0"] = self.ref.p0.tolist()
        self.t, self.t_release, self.armed = 0.0, None, True

    def release(self):
        """모터 정지 (ω = 0)."""
        self.armed = False
        self.t_release = self.t

    def start(self):
        from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager

        if self._cb is None:
            self._cb = SimulationManager.register_callback(self._pre_step, event=SimulationEvent.PHYSICS_PRE_STEP)

    def stop(self):
        from isaacsim.core.simulation_manager import SimulationManager

        if self._cb is not None:
            SimulationManager.deregister_callback(self._cb)
            self._cb = None

    def check(self):
        if self.cb_error is not None:
            e, self.cb_error = self.cb_error, None
            raise RuntimeError(f"드론 비행 콜백 에러: {e!r}") from e

    # ── 할당 (Pegasus force_and_torques_to_velocities) ──
    def allocate(self, u1, tau, rotor_rel):
        A = np.vstack([np.full(len(self.dir), self.k),
                       rotor_rel[:, 1] * self.k,
                       -rotor_rel[:, 0] * self.k,
                       self.c * self.dir])
        w2 = np.linalg.pinv(A) @ np.array([u1, tau[0], tau[1], tau[2]])
        w2 = np.maximum(w2, 0.0)
        if w2.max() > self.w_max ** 2:
            w2 *= self.w_max ** 2 / w2.max()
        return np.sqrt(w2)

    # ── physics 스텝 ──
    def _pre_step(self, dt, context):
        try:
            self._step(float(dt))
        except Exception as e:  # noqa: BLE001
            self.cb_error = e

    def _step(self, dt):
        self.t += dt
        p, q = self._body_pose()
        R = quat_to_R(q)
        v, w = (x.numpy().reshape(-1) for x in self.body.get_velocities())
        rp, rq = (x.numpy() for x in self.rotors.get_world_poses())
        if self.armed:
            ref = self.ref(self.t)
            self.u1, self.tau = self.ctrl.update(dt, p, v, R, w, ref)
            self.omega = self.allocate(self.u1, self.tau, (rp - p) @ R)
        else:
            self.u1, self.tau = 0.0, np.zeros(3)
            self.omega = np.zeros(len(self.dir))
        self.forces = self.k * self.omega ** 2
        if self.forces.any():
            f_world = np.array([F * quat_to_R(qq)[:, 2] for F, qq in zip(self.forces, rq)])
            self.rotors.apply_forces_and_torques_at_pos(forces=f_world, positions=rp, local_frame=False)
        yaw_torque = float((self.c * self.omega ** 2 * self.dir).sum()) * R[:, 2]
        drag = R @ (-self.drag * (R.T @ v))
        self.body.apply_forces_and_torques_at_pos(forces=drag.reshape(1, 3), torques=yaw_torque.reshape(1, 3),
                                                  positions=p.reshape(1, 3), local_frame=False)
        if self.spin is not None:
            spd = np.where(self.forces >= VISUAL_FORCE, self.fc["prop_visual_speed"],
                           np.where(self.forces > 0, self.fc["prop_idle_speed"], 0.0))
            self.prop_cmd = spd * self.dir
            self.art.set_dof_velocities(self.prop_cmd.reshape(1, -1), dof_indices=self.spin)

    def prop_velocities(self):
        if self.spin is None:
            return None
        return self.art.get_dof_velocities().numpy()[0][self.spin]

    def hover_omega(self):
        """정지 호버에 필요한 로터 각속도 (질량 / 4 로터 균등) [rad/s]."""
        return math.sqrt(self.mass * G / (len(self.dir) * self.k))
