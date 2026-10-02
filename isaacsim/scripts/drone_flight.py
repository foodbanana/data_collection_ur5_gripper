# =============================================================
# drone_flight.py — 드론 비행 (docs/PLAN.md 2-2, Pegasus Simulator 방식)
#
#   제어기 → 로터 4 개 목표 각속도 ω (rad/s) → 이차 추력 모델 → PhysX 외력 (매 physics step, PHYSICS_PRE_STEP 콜백)
#     - 로터 i 강체에 자기 z 축 방향 추력 Fᵢ = k·ωᵢ² (로터 위치에서)
#     - body 에 반토크 Σ c·ωᵢ²·dirᵢ (body z 축) + 선형 공기저항 −D·v_body
#   제어기 (flight.backend):
#     geometric — 기하 제어기 = Pegasus 예제 NonlinearController 그대로 (식·게인·한 step 지연, Lee/Mellinger)
#     px4       — PX4 SITL (px4_bridge.PX4Bridge: 가상 센서 → PX4 → 모터 명령, lockstep) + 명령 drone_cmd.PX4Commander
#                 (arm() = Offboard 위치 목표 + arm 요청, release() = kill). 추력·반토크·공기저항·프로펠러 표시는 같은 코드
#                 가상 센서(IMU·flow)에 주는 속도·각속도 = 자세·위치 차분 (PhysX 가 보고하는 속도 대신, px4.sensor_kinematics):
#                   그리퍼에 잡혀 접촉이 걸리면 PhysX 각속도가 실제 자세 변화와 맞지 않음 (자세는 0.5° 안인데 보고 각속도 평균 11~19 °/s)
#                   → 가짜 자이로로 PX4 자세 추정이 1.5 s 에 15° 틀어지고 발산 (2026-10-02 ulog). 실물 IMU 는 실제 움직임만 잼
#                 px4.motor_dynamics: 모터 1차 지연 (PX4 Gazebo motor_model 과 같음, Pegasus 에는 없음):
#                   ω ← ω + (ω_cmd − ω)·(1 − e^(−dt/τ)), τ = 올릴 때 tau_up, 내릴 때 tau_down. geometric 은 Pegasus 그대로 (지연 없음)
#     F_des = −Kp·e_p − Kd·e_v − Ki·∫e_p + m·g·ẑ + m·a_ref → 몸체를 F_des 방향으로 기울이고 u₁ = F_des·Z_B
#     τ = −Kr·e_R − Kw·(ω − ω_des) (ω_des = 목표 jerk 투영),  (u₁, τ) → 할당 행렬 역행렬 → ω²  (음수는 0, 최대 ω 넘으면 비율 유지 축소)
#   Pegasus 와 같게 (2026-10-01 사용자 결정): 게인은 Pegasus 값을 우리 기체 질량·관성 비율로 환산(설정 scale_to_airframe,
#     Pegasus Iris 와 같은 응답), 매 step 먼저 지난 step 에 계산한 ω 를 적용하고
#     그다음 제어기를 돌림 (Pegasus vehicle.update: thrusters ← backend.input_reference() → 힘 적용 → backend.update)
#   Pegasus 와 다른 점: 질량 m 은 PhysX 값 (Pegasus 는 1.5 고정), physics 1/120 s (Pegasus 1/250 s, 로봇 drive 튜닝 기준),
#     yaw 목표 = 시작 yaw 고정 (Pegasus 는 궤적 파일), 센서(IMU·GPS 등) 없음
#   모드: static (목표 고정), hover (목표 + 축별 사인파 합, seed), trajectory (미구현 → 에러)
#   release(): 모터 정지 (ω = 0 → 추력 0. 실물: 잡은 뒤 모터를 멈춤). 공기저항은 계속
#   프로펠러 회전 (보여 주기용, prop_spin): Pegasus handle_propeller_visual 과 같이 실제 ω 와 무관하게
#     추력 ≥ 0.1 N 이면 prop_visual_speed, 0 < 추력 < 0.1 N 이면 prop_idle_speed, 0 이면 정지 (관절 속도를 매 step 덮어씀)
#
# 설정: drone_*.yaml 의 flight. 질량·관성은 PhysX 값, 게인은 설정값 (세 축 [x, y, z]) × 기체 비율 (scale_to_airframe)
# SimulationApp 을 만든 뒤에 import 할 것.
# =============================================================

import math
import os

import numpy as np

import drone as dr

G = 9.81
MODES = ("static", "hover", "trajectory")
BACKENDS = ("geometric", "px4")
FLIGHT_KEYS = ("backend", "rotors", "rot_dir", "rotor_constant", "rolling_moment_coefficient", "max_rotor_velocity", "linear_drag",
               "prop_joints", "prop_visual_speed", "prop_idle_speed", "controller", "hover")
VISUAL_FORCE = 0.1   # N, Pegasus 프로펠러 표시 기준


def quat_to_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def vee(M):
    return np.array([M[2, 1], M[0, 2], M[1, 0]])


def _log_so3(R):
    """회전 행렬 → 회전 벡터 (rad)."""
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    th = math.acos(c)
    w = vee(R - R.T) / 2.0
    if th < 1e-9:
        return w
    return w * (th / math.sin(th))


class FastRigid:
    """매 physics 스텝 읽기·쓰기용: RigidPrim 의 physics tensor view 를 직접 쓴다 (PLAN 3-1).
    RigidPrim.get_world_poses / apply_forces_and_torques_at_pos 는 호출마다 warp 배열을 여러 개 새로 만들고 복사해
    드론 스텝 하나에 Python 시간이 크게 든다 (프로파일: warp copy 가 물리 스텝당 18 번). 읽는 값은 같은 view 데이터 (float32).
    view 는 재생 때 다시 만들어질 수 있어 매번 prim 에서 가져온다."""

    def __init__(self, prim):
        import warp as wp

        self.prim, self.n = prim, len(prim)
        self._wp = wp
        self._cache = None

    @property
    def view(self):
        v = self.prim._physics_rigid_body_view
        if v is None:
            raise RuntimeError(f"physics tensor view 없음 (재생 전?): {self.prim.paths}")
        return v

    def poses(self):
        """(위치 (N, 3), 자세 쿼터니언 wxyz (N, 4)). view 는 xyzw."""
        d = self.view.get_transforms().numpy()
        return d[:, :3].copy(), d[:, [6, 3, 4, 5]]       # 복사: CPU 장치면 numpy 가 view 버퍼를 공유할 수 있음

    def velocities(self):
        """(선속도 (N, 3), 각속도 (N, 3)) world."""
        d = self.view.get_velocities().numpy()
        return d[:, :3].copy(), d[:, 3:6].copy()

    def apply_global(self, forces=None, torques=None, positions=None):
        """world 좌표 힘·토크 (positions: 힘 작용점 world). RigidPrim.apply_forces_and_torques_at_pos(local_frame=False) 와 같음."""
        wp, view = self._wp, self.view
        if self._cache is None or self._cache[0] is not view:      # view 가 바뀌면 (재생 다시) 장치·인덱스·버퍼 다시
            dev = view.get_transforms().device
            idx = wp.array(np.arange(self.n, dtype=np.int32), dtype=wp.int32, device=dev)
            bufs = [wp.zeros((self.n, 3), dtype=wp.float32, device=dev) for _ in range(3)]
            self._cache = (view, idx, bufs, dev.is_cpu)
        _, idx, bufs, on_cpu = self._cache

        def arr(x, buf):
            if x is None:
                return None
            x = np.asarray(x, dtype=np.float32).reshape(self.n, 3)
            if on_cpu:
                buf.numpy()[:] = x                                # CPU: numpy 가 버퍼를 공유 → 새 배열 없이 씀
            else:
                buf.assign(x)
            return buf

        view.apply_forces_and_torques_at_position(arr(forces, bufs[0]), arr(torques, bufs[1]), arr(positions, bufs[2]),
                                                  indices=idx, is_global=True)


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
    게인은 설정값 (축별 [x, y, z], Pegasus 는 np.diag). scale_to_airframe 이면 Kp·Kd·Ki × m/m_ref, Kr·Kw × I/I_ref (축별),
    질량 m·관성 I 는 PhysX 값."""

    def __init__(self, mass, inertia_diag, ccfg):
        self.m = mass
        for k in ("Kp", "Kd", "Ki", "Kr", "Kw"):
            v = np.asarray(ccfg[k], dtype=float)
            if v.shape != (3,):
                raise ValueError(f"flight.controller.{k} 는 [x, y, z] 세 값: {ccfg[k]}")
            setattr(self, k, v)
        if "scale_to_airframe" not in ccfg:
            raise KeyError("flight.controller.scale_to_airframe 항목이 없습니다 (true/false 명시)")
        if ccfg["scale_to_airframe"]:
            s_m = mass / float(ccfg["reference_mass"])
            s_I = np.asarray(inertia_diag, dtype=float) / np.asarray(ccfg["reference_inertia"], dtype=float)
            self.Kp, self.Kd, self.Ki = self.Kp * s_m, self.Kd * s_m, self.Ki * s_m
            self.Kr, self.Kw = self.Kr * s_I, self.Kw * s_I
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
    """드론 비행 (physics 콜백). start() 전에 재생(play) 되어 있어야 한다. 콜백 예외는 check() 에서 다시 올린다.
    check() 는 매 루프(simulation_app.update() 뒤) 부른다 (px4: 명령 송수신)."""

    def __init__(self, cfg, info, mode="static", seed=0, target=None, backend=None, report_dir=None, position_source=None):
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
        self.backend = fc["backend"] if backend is None else backend
        if self.backend not in BACKENDS:
            raise ValueError(f"flight.backend 는 {BACKENDS} 중 하나: {self.backend}")
        path = info["path"]
        self.body = RigidPrim(info["body_path"])
        self.rotors = RigidPrim([dr._sub(path, r) for r in fc["rotors"]])
        self.links = RigidPrim(info["bodies"])
        self.body_io, self.rotors_io = FastRigid(self.body), FastRigid(self.rotors)   # 매 스텝 읽기·쓰기
        self.k =float(fc["rotor_constant"])
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

        self.px4, self.cmd = None, None
        if self.backend == "px4":
            self._init_px4(cfg, n, seed, p, report_dir, position_source)

        self.armed, self.t, self.t_release = False, 0.0, None
        self.t_arm = 0.0
        if self.px4 is None:
            self.motor_tau = None
        self.omega = np.zeros(n)        # 이번 step 에 적용한 ω
        self.omega_cmd = np.zeros(n)    # 제어기가 낸 ω (다음 step 에 적용)
        self.forces = np.zeros(n)
        self.prop_cmd = np.zeros(n)
        self.u1, self.tau = 0.0, np.zeros(3)
        self._cb, self.cb_error = None, None

    def _init_px4(self, cfg, n, seed, p_start, report_dir, position_source):
        import yaml

        import drone_cmd
        import px4_bridge
        import test_scene as ts

        if "px4" not in cfg:
            raise KeyError(f"{cfg['path']}: flight.backend px4 인데 'px4' 항목이 없습니다")
        px4c = dict(cfg["px4"])
        for k in ("sitl_config", "airframe", "input_offset", "input_scaling", "zero_position_armed", "params",
                  "rotor_geometry_from_sim", "rate_gain_scaling", "downward_sensor_position", "motor_dynamics",
                  "sensor_overrides", "sensor_kinematics"):
            if k not in px4c:
                raise KeyError(f"{cfg['path']}: 'px4.{k}' 항목이 없습니다")
        sp = px4c["sitl_config"] if os.path.isabs(px4c["sitl_config"]) else os.path.join(ts.SIM_ROOT, px4c["sitl_config"])
        with open(sp, encoding="utf-8") as f:
            sitl = yaml.safe_load(f)
        self.sitl = sitl
        # 위치 정보 방식 → 켤 센서 + PX4 파라미터 (우선순위: 위치 방식 < 기체 자동값 < 드론 설정 px4.params)
        self.position_source = sitl["position_source"] if position_source is None else position_source
        if self.position_source not in sitl["position_sources"]:
            raise ValueError(f"position_source '{self.position_source}' 가 {sp} position_sources 에 없음")
        src = sitl["position_sources"][self.position_source]
        sensors = {k: (dict(v) if isinstance(v, dict) else v) for k, v in sitl["sensors"].items()}
        for k, v in (px4c["sensor_overrides"] or {}).items():                  # 기체별 센서 덮어쓰기 (한 단계 깊이)
            if k not in sensors:
                raise KeyError(f"px4.sensor_overrides.{k} 는 {sp} sensors 에 없는 항목")
            if isinstance(v, dict):
                sensors[k].update(v)
            else:
                sensors[k] = v
        self.sensors_cfg = sensors
        if px4c["sensor_kinematics"] not in ("pose_difference", "physx_velocity"):
            raise ValueError(f"px4.sensor_kinematics 는 pose_difference | physx_velocity: {px4c['sensor_kinematics']}")
        self.sensor_kin = px4c["sensor_kinematics"]
        self._prev_pose = None
        md = px4c["motor_dynamics"]
        self.motor_tau = (float(md["tau_up"]), float(md["tau_down"])) if md["enabled"] else None
        for k, sk in (("gps", "gps"), ("rangefinder", "rangefinder"), ("optical_flow", "optical_flow"), ("mocap", "mocap")):
            sensors[sk]["enabled"] = bool(src[k])
        self.px4_auto_params = self._px4_airframe_params(px4c)
        px4c["params"] = {**(src["params"] or {}), **self.px4_auto_params, **(px4c["params"] or {})}
        self.px4_params = px4c["params"]
        self.down_sensor = np.asarray(px4c["downward_sensor_position"], dtype=float)
        self.ground_max = max(float(sensors["rangefinder"]["max_distance"]), float(sensors["optical_flow"]["max_distance"]))
        self.ground, self.ground_hit = None, None
        self.px4 = px4_bridge.PX4Bridge(sitl, px4c, sensors, self.w_max, n, seed, report_dir or ts.REPORT_DIR)
        # PX4 local 원점 = EKF 초기화 위치 = 시작 위치 (바닥에 놓인 드론)
        self.cmd = drone_cmd.PX4Commander(int(sitl["offboard_port"]) + int(sitl["instance"]), p_start,
                                          sitl["commander"]["setpoint_hz"], sitl["commander"]["retry_sec"],
                                          command_port=int(sitl["command_port"]) + int(sitl["instance"]),
                                          disable_streams=sitl["commander"]["disable_streams"])

    def _px4_airframe_params(self, px4c):
        """우리 기체에 맞춘 PX4 파라미터 (설정에서 켠 것만).
        rotor_geometry_from_sim: CA_ROTORi_PX/PY = sim 로터 위치 (body 원점 기준, FRD).
        rate_gain_scaling: 각속도 루프 전체 게인 MC_{ROLL,PITCH,YAW}RATE_K = (I / I_ref) ÷ (팔 / 팔_ref) (축별).
          각가속도 = 토크 / I, 최대 토크 ∝ 팔 길이 (roll: 로터 |y| 평균, pitch: |x| 평균, yaw: 반토크라 팔 무관)
          → 기준 기체와 같은 각속도 응답. K 는 P·I·D 를 함께 곱함 (PX4 MC_*RATE_K 설명)"""
        out = {}
        p, q = self._body_pose()
        R = quat_to_R(q)
        rp = self.rotors.get_world_poses()[0].numpy()
        rel = (rp - p) @ R                                   # body FLU
        frd = np.column_stack([rel[:, 0], -rel[:, 1]])
        if px4c["rotor_geometry_from_sim"]:
            for i, (x, y) in enumerate(frd):
                out[f"CA_ROTOR{i}_PX"] = round(float(x), 4)
                out[f"CA_ROTOR{i}_PY"] = round(float(y), 4)
        rs = px4c["rate_gain_scaling"]
        if rs["enabled"]:
            I = np.diag(self.inertia)
            I_ref = np.asarray(rs["reference_inertia"], dtype=float)
            arm = np.abs(frd).mean(axis=0)                     # [|x|, |y|]
            arm_ref = np.asarray(rs["reference_arm_xy"], dtype=float)
            k = (I / I_ref) / np.array([arm[1] / arm_ref[1], arm[0] / arm_ref[0], 1.0])
            for name, v in zip(("MC_ROLLRATE_K", "MC_PITCHRATE_K", "MC_YAWRATE_K"), k):
                out[name] = round(float(v), 4)
        return out

    def _ground_distance(self, p, R):
        """하향 센서 광축(body −z) 방향 PhysX raycast, 드론 자기 충돌 형상은 건너뜀 → (거리, 맞은 충돌 형상 경로) 또는 (None, None)."""
        import carb
        from omni.physx import get_physx_scene_query_interface

        origin = p + R @ self.down_sensor
        d = -R[:, 2]
        own = self.info["path"] + "/"
        best = [None, None]

        def report(hit):
            if not hit.collision.startswith(own) and (best[0] is None or hit.distance < best[0]):
                best[0], best[1] = float(hit.distance), hit.collision
            return True

        get_physx_scene_query_interface().raycast_all(carb.Float3(*origin), carb.Float3(*d), self.ground_max + 0.5, report)
        return best[0], best[1]

    # ── 질량·관성 (PhysX) ──
    def _body_pose(self):
        p, q = self.body_io.poses()
        return p[0], q[0]

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
        """모터 시동 + 현재 자세의 yaw 로 제어 시작 (target 을 주면 static·hover 의 기준 위치를 바꿈).
        geometric: 바로 제어 시작 (시간 t 를 0 으로). px4: Offboard 위치 목표 + arm 요청 (실제 arm 은 PX4 가 판단, armed 로 확인)."""
        p, q = self._body_pose()
        R = quat_to_R(q)
        yaw = math.atan2(R[1, 0], R[0, 0])
        if target is not None:
            self.ref.p0 = np.asarray(target, dtype=float)
            self.ref.params["p0"] = self.ref.p0.tolist()
        self.t_release = None
        if self.backend == "px4":
            # PX4 local 좌표 원점을 world 에 맞춤: 바닥에 멈춰 있는 지금 실제 위치 − PX4 추정 위치 만큼 옮김
            #   (EKF 원점 = PX4 부팅 때 위치. 실물에서 이륙 전 드론 위치를 world 에 등록하는 것과 같음)
            if not self.px4.armed and self.cmd.estimate_enu is not None:
                self.cmd.origin = self.cmd.origin + (p - self.cmd.estimate_enu)
                self.cmd.estimate_enu = p.copy()
            self.t_arm = self.t
            self.cmd.fly_to(self.ref.p0, yaw)
            return
        self.ctrl.reset(yaw)
        self.t, self.t_arm, self.armed = 0.0, 0.0, True

    def release(self):
        """모터 정지. geometric: ω = 0 (한 step 지연 때문에 바로 다음 step 은 지난 ω, 그다음부터 0, Pegasus 와 같은 순서).
        px4: kill 명령 (공중 강제 disarm, 실물과 같음) → PX4 가 모터 명령을 0 으로."""
        if self.backend == "px4":
            self.cmd.kill()
        else:
            self.armed = False
        self.t_release = self.t

    def start(self):
        from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager

        if self._cb is None:
            if self.px4 is not None:
                self.px4.start()
            self._cb = SimulationManager.register_callback(self._pre_step, event=SimulationEvent.PHYSICS_PRE_STEP)

    def stop(self):
        from isaacsim.core.simulation_manager import SimulationManager

        if self._cb is not None:
            SimulationManager.deregister_callback(self._cb)
            self._cb = None
        if self.px4 is not None:
            self.px4.stop()
            self.cmd.close()

    def check(self):
        if self.cb_error is not None:
            e, self.cb_error = self.cb_error, None
            raise RuntimeError(f"드론 비행 콜백 에러: {e!r}") from e
        if self.cmd is not None:
            if self.ref.mode == "hover" and self.cmd.target is not None and not self.cmd.external:
                self.cmd.goto(self.ref(self.t - self.t_arm)[0])
            self.cmd.tick(self.t)

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
        v, w = (x[0] for x in self.body_io.velocities())
        rp, rq = self.rotors_io.poses()
        # Pegasus 순서: 지난 step 에 제어기가 낸 ω 를 먼저 적용 (한 step 지연) → 그다음 제어기 갱신
        if self.motor_tau is None:
            self.omega = self.omega_cmd
        else:
            tau = np.where(self.omega_cmd > self.omega, self.motor_tau[0], self.motor_tau[1])
            self.omega = self.omega + (self.omega_cmd - self.omega) * (1.0 - np.exp(-dt / tau))
        self.forces = self.k * self.omega ** 2
        if self.forces.any():
            f_world = np.array([F * quat_to_R(qq)[:, 2] for F, qq in zip(self.forces, rq)])
            self.rotors_io.apply_global(forces=f_world, positions=rp)
        yaw_torque = float((self.c * self.omega ** 2 * self.dir).sum()) * R[:, 2]
        drag = R @ (-self.drag * (R.T @ v))
        self.body_io.apply_global(forces=drag, torques=yaw_torque, positions=p)
        if self.spin is not None:
            spd = np.where(self.forces >= VISUAL_FORCE, self.fc["prop_visual_speed"],
                           np.where(self.forces > 0, self.fc["prop_idle_speed"], 0.0))
            self.prop_cmd = spd * self.dir
            self.art.set_dof_velocities(self.prop_cmd.reshape(1, -1), dof_indices=self.spin)
        if self.px4 is not None:
            if self.px4.needs_ground:
                self.ground, self.ground_hit = self._ground_distance(p, R)
            v_s, w_s = v, R.T @ w
            if self.sensor_kin == "pose_difference":
                if self._prev_pose is not None:
                    p0, R0 = self._prev_pose
                    v_s = (p - p0) / dt
                    w_s = _log_so3(R0.T @ R) / dt               # 몸체 좌표계 각속도
                self._prev_pose = (p.copy(), R.copy())
            self.sensor_v, self.sensor_w = v_s, w_s
            self.physx_w = R.T @ w
            self.omega_cmd = self.px4.step(dt, p, v_s, R, w_s, self.ground)
            self.armed = self.px4.armed
        elif self.armed:
            self.u1, self.tau = self.ctrl.update(dt, p, v, R, w, self.ref(self.t))
            self.omega_cmd = self.allocate(self.u1, self.tau, (rp - p) @ R)
        else:
            self.u1, self.tau = 0.0, np.zeros(3)
            self.omega_cmd = np.zeros(len(self.dir))

    def prop_velocities(self):
        if self.spin is None:
            return None
        return self.art.get_dof_velocities().numpy()[0][self.spin]

    def hover_omega(self):
        """정지 호버에 필요한 로터 각속도 (질량 / 4 로터 균등) [rad/s]."""
        return math.sqrt(self.mass * G / (len(self.dir) * self.k))
