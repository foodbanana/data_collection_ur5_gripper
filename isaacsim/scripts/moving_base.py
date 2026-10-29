# =============================================================
# moving_base.py — 움직이는 베이스 (PLAN 8단계). 고정 베이스 씬 (drone_scene.py) 은 이 모듈을 쓰지 않는다
#
#   LinearTrajectory : 직선 왕복 궤적 (사다리꼴 속도: 가속 → 등속 → 감속 → 멈춤 → 되돌아옴). 시각 → (거리, 속도, 가속도)
#   KinematicBase    : 베이스 USD (예 Husky) 를 씬에 넣고 메모리 stage 에서만 kinematic 으로 바꾼 뒤,
#                      물리 스텝마다 (PHYSICS_PRE_STEP) 최상위 prim 의 USD 변환을 궤적 위치로 옮긴다.
#                      PhysX 는 이것을 kinematic target 으로 받아 베이스 속도를 안다 (순간이동 아님 → 위에 붙은 팔이 관성력을 받음).
#                      바퀴는 이동 거리 / 반지름 만큼 도는 표시만 (바퀴 물리·바닥 접촉 없음)
#   SineTrajectory   : 6 축 사인파 q_k(t) = e(t)·s·A_k·sin(ω_k t + φ_k) (Stewart, 5 차 시작 램프, ω·위상은 seed 로)
#   StewartBase      : Stewart 윗판 (kinematic 강체) 을 SineTrajectory 자세로 (USD translate + orient), 다리 메시는 역기구학으로
#   make_base(cfg)   : 설정 kind (linear | stewart) 로 위 둘 중 하나
#
# 확인 (2026-10-28 실험): 최상위 prim translate 를 0.2 m/s 로 옮기면 차체·바퀴 강체 위치가 명령과 같고
#   PhysX 가 보고하는 선속도도 0.200 m/s.
#   바퀴 강체 prim 의 회전을 USD 로 바꾸면 PhysX 가 그 충돌 형상을 지우고 다시 만들어 tensor view 가 무효가 된다
#   → 바퀴 링크 아래 보이는 메시 prim (설정 wheels.visual, 물리 없음) 만 돌린다
#   Stewart 윗판 (부모가 강체가 아닌 kinematic 강체) 은 translate + orient 를 USD 로 바꿔도 괜찮다 (2026-10-28 실험:
#   위치·roll 이 명령과 같고 PhysX 선속도·각속도도 맞음, tensor view 유지)
# SimulationApp 을 만든 뒤에 import 할 것.
# =============================================================

import math
import os

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_ROOT = os.path.dirname(HERE)
CONFIG_DIR = os.path.join(SIM_ROOT, "config")
DEFAULT_BASE_CONFIG = os.path.join(CONFIG_DIR, "base_husky.yaml")

KINDS = ("linear", "stewart")
BASE_KEYS = ("usd", "physics_variant", "prim_name", "body", "base_link_height", "wheels", "trajectory")
STEWART_KEYS = ("usd", "prim_name", "body", "geometry", "legs", "imu", "motion")
WHEEL_KEYS = ("links", "radius", "axis", "visual")
TRAJ_KEYS = ("direction", "distance", "max_speed", "accel", "dwell")
AXES = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}


def _abs(path):
    return path if os.path.isabs(path) else os.path.join(SIM_ROOT, path)


def load_base_config(path=None):
    path = path or DEFAULT_BASE_CONFIG
    with open(path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if cfg.get("kind") not in KINDS:
        raise KeyError(f"{path}: 'kind' 는 {KINDS} 중 하나: {cfg.get('kind')}")
    cfg["path"] = path
    if cfg["kind"] == "stewart":
        for k in STEWART_KEYS:
            if k not in cfg:
                raise KeyError(f"{path}: '{k}' 항목이 없습니다")
        import design_stewart as dst        # 부착점·회전·프리셋 식을 설계 스크립트와 같은 코드로

        dst.load(path)                       # geometry·motion 항목 확인
        for k in ("length_min", "length_max"):
            if cfg["legs"].get(k) is None:
                raise KeyError(f"{path}: 'legs.{k}' 가 비어 있음 (design_stewart.py 결과로 채울 것)")
        return cfg
    for k in BASE_KEYS:
        if k not in cfg:
            raise KeyError(f"{path}: '{k}' 항목이 없습니다")
    for k in WHEEL_KEYS:
        if k not in cfg["wheels"]:
            raise KeyError(f"{path}: 'wheels.{k}' 항목이 없습니다")
    for k in TRAJ_KEYS:
        if k not in cfg["trajectory"]:
            raise KeyError(f"{path}: 'trajectory.{k}' 항목이 없습니다")
    if cfg["wheels"]["axis"] not in AXES:
        raise ValueError(f"{path}: wheels.axis 는 {tuple(AXES)} 중 하나: {cfg['wheels']['axis']}")
    return cfg


class LinearTrajectory:
    """0 → distance → 0 → … 직선 왕복. 구간마다 사다리꼴 속도 (거리가 짧으면 삼각형), 양 끝에서 dwell 초 멈춤.

    sample(t) → (s, v, a): 시작점에서 direction 방향으로 간 거리 [m], 속도 [m/s], 가속도 [m/s²] (t < 0 이면 시작점에 멈춤)
    """

    def __init__(self, distance, max_speed, accel, dwell):
        if not (distance > 0 and max_speed > 0 and accel > 0 and dwell >= 0):
            raise ValueError(f"궤적 값이 잘못됨: distance {distance}, max_speed {max_speed}, accel {accel}, dwell {dwell}")
        self.distance, self.accel, self.dwell = float(distance), float(accel), float(dwell)
        # 사다리꼴이 되려면 가속 거리 2 × v²/(2a) ≤ distance. 아니면 삼각형 (최고 속도 = √(a·d))
        self.v_peak = min(float(max_speed), math.sqrt(self.accel * self.distance))
        self.t_acc = self.v_peak / self.accel
        self.t_cruise = (self.distance - self.v_peak * self.t_acc) / self.v_peak
        self.t_move = 2 * self.t_acc + self.t_cruise
        self.period = 2 * (self.t_move + self.dwell)

    def _leg(self, tau):
        """한 구간 (0 → distance) 안에서 시각 tau 의 (s, v, a)."""
        a, ta, tc = self.accel, self.t_acc, self.t_cruise
        if tau <= 0:
            return 0.0, 0.0, 0.0
        if tau < ta:
            return 0.5 * a * tau * tau, a * tau, a
        if tau < ta + tc:
            return 0.5 * a * ta * ta + self.v_peak * (tau - ta), self.v_peak, 0.0
        if tau < self.t_move:
            r = self.t_move - tau
            return self.distance - 0.5 * a * r * r, a * r, -a
        return self.distance, 0.0, 0.0

    def sample(self, t):
        if t <= 0:
            return 0.0, 0.0, 0.0
        tp = t % self.period
        half = self.t_move + self.dwell
        if tp < half:
            return self._leg(tp)
        s, v, a = self._leg(tp - half)
        return self.distance - s, -v, -a


def _yaw_quat(yaw):
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


class _StepDriver:
    """물리 스텝마다 (PHYSICS_PRE_STEP) self._apply(t) 를 부른다. t = start() 뒤 sim 시각 (이번 스텝이 끝나는 시각)."""

    def _init_driver(self):
        self.t = 0.0
        self._t0 = None
        self._cb = None
        self._err = None

    def start(self):
        if self._cb is not None:
            return
        from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager

        self._t0 = SimulationManager.get_simulation_time()
        self.t = 0.0
        self._cb = SimulationManager.register_callback(self._pre_step, event=SimulationEvent.PHYSICS_PRE_STEP)

    def stop(self):
        if self._cb is not None:
            from isaacsim.core.simulation_manager import SimulationManager

            SimulationManager.deregister_callback(self._cb)
            self._cb = None

    def _pre_step(self, dt, context):
        # 이번 스텝이 끝나는 시각의 자세를 kinematic target 으로. 콜백 안 예외는 묻힐 수 있어 check() 에서 다시 올린다
        try:
            from isaacsim.core.simulation_manager import SimulationManager

            self.t = SimulationManager.get_simulation_time() + dt - self._t0
            self._apply(self.t)
        except Exception as e:  # noqa: BLE001
            self._err = e

    def check(self):
        if self._err is not None:
            err, self._err = self._err, None
            raise RuntimeError(f"{type(self).__name__} 물리 콜백 실패: {err}") from err

    def go_start(self):
        """리셋: 멈추고 궤적 시작 자세 (t = 0) 로 순간이동 (USD). 다음 start() 에서 t = 0 부터 다시 움직인다."""
        self.stop()
        self.t = 0.0
        self._apply(0.0)


class KinematicBase(_StepDriver):
    """베이스 USD 를 /World/<prim_name> 에 넣고 kinematic 궤적으로 움직인다.

    start_xy, start_yaw_deg: base_link 시작 위치 (world, 바닥 위 높이는 설정 base_link_height)·방향.
    순서: add() (재생 전) → 재생 → start() → 루프마다 check() → stop()
    """

    def __init__(self, cfg, start_xy=(0.0, 0.0), start_yaw_deg=0.0, static=False):
        self.cfg = cfg
        self.static = bool(static)                           # True = 서 있음 (--base-motion static: 궤적 시작 자세에 그대로)
        self.path = f"/World/{cfg['prim_name']}"
        self.body_path = f"{self.path}/{cfg['body']}"
        self.imu_path = f"{self.body_path}/imu_link"
        self.wheel_paths = [f"{self.path}/{w}" for w in cfg["wheels"]["links"]]
        self.joint_names = [f"{w.split('/')[-1]}_joint" for w in cfg["wheels"]["links"]]   # URDF 바퀴 관절 이름
        self.wheel_visual_paths = [f"{w}/{cfg['wheels']['visual']}" for w in self.wheel_paths]
        self.wheel_r = float(cfg["wheels"]["radius"])
        self.wheel_axis = np.array(AXES[cfg["wheels"]["axis"]])
        tc = cfg["trajectory"]
        d = np.asarray(tc["direction"], dtype=float)
        if d.shape != (3,) or np.linalg.norm(d) < 1e-9:
            raise ValueError(f"trajectory.direction 이 잘못됨: {tc['direction']}")
        self.yaw = math.radians(start_yaw_deg)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        self.dir_world = Rz @ (d / np.linalg.norm(d))        # 궤적 방향 (world)
        self.p0 = np.array([start_xy[0], start_xy[1], float(cfg["base_link_height"])])
        self.traj = LinearTrajectory(tc["distance"], tc["max_speed"], tc["accel"], tc["dwell"])
        self.seed = None                                     # 리셋 seed (기록용, reseed)
        self._init_driver()
        self._t_op = None
        self._wheel_ops = []

    # ── 씬 구성 (재생 전) ──
    def add(self, stage):
        """reference + 시작 자세 + 메모리 stage 에서만 kinematic 으로 (USD 파일은 그대로)."""
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.experimental.prims import XformPrim
        from pxr import Gf, Usd, UsdGeom, UsdPhysics

        usd = _abs(self.cfg["usd"])
        if not os.path.isfile(usd):
            raise FileNotFoundError(f"베이스 USD 가 없습니다: {usd}")
        stage_utils.add_reference_to_stage(usd, self.path, variants=[("Physics", self.cfg["physics_variant"])])
        XformPrim(self.path, reset_xform_op_properties=True).set_world_poses(
            positions=[self.p0.tolist()], orientations=[_yaw_quat(self.yaw).tolist()])

        body = stage.GetPrimAtPath(self.body_path)
        if not body.IsValid() or not body.HasAPI(UsdPhysics.RigidBodyAPI):
            raise RuntimeError(f"차체 강체가 없습니다: {self.body_path}")
        if not stage.GetPrimAtPath(self.imu_path).IsValid():
            raise RuntimeError(f"imu_link 가 없음: {self.imu_path}")
        # articulation 을 풀어 강체 묶음으로: articulation root 를 떼고, 모든 강체 kinematic, 관절 (root_joint·바퀴·고정) 끔
        for s in list(body.GetAppliedSchemas()):
            if "ArticulationRoot" in s or s == "PhysxArticulationAPI":
                body.RemoveAppliedSchema(s)
        n_body = n_joint = 0
        for p in Usd.PrimRange(stage.GetPrimAtPath(self.path)):
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(p).CreateKinematicEnabledAttr().Set(True)
                n_body += 1
            if p.IsA(UsdPhysics.Joint):
                UsdPhysics.Joint(p).CreateJointEnabledAttr().Set(False)
                n_joint += 1

        top = UsdGeom.Xformable(stage.GetPrimAtPath(self.path))
        self._t_op = self._op(top, UsdGeom.XformOp.TypeTranslate, self.path)
        self._wheel_ops = []
        for w in self.wheel_visual_paths:
            xf = UsdGeom.Xformable(stage.GetPrimAtPath(w))
            if not xf.GetPrim().IsValid():
                raise RuntimeError(f"바퀴 메시 prim 이 없습니다: {w}")
            if xf.GetPrim().HasAPI(UsdPhysics.RigidBodyAPI) or xf.GetPrim().HasAPI(UsdPhysics.CollisionAPI):
                raise RuntimeError(f"{w}: 보이기만 하는 prim 이어야 함 (강체·충돌이 있으면 PhysX 가 형상을 다시 만듦)")
            op = self._op(xf, UsdGeom.XformOp.TypeOrient, w)
            q = op.Get()
            self._wheel_ops.append((op, Gf.Quatd(q.GetReal(), *q.GetImaginary()), type(q)))
        return {"path": self.path, "kinematic_bodies": n_body, "joints_disabled": n_joint}

    @staticmethod
    def _op(xf, op_type, path):
        ops = [op for op in xf.GetOrderedXformOps() if op.GetOpType() == op_type]
        if len(ops) != 1:
            raise RuntimeError(f"{path}: {op_type} xform op 가 하나가 아님 ({len(ops)})")
        return ops[0]

    # ── 궤적 ──
    def _sample(self, t):
        return (0.0, 0.0, 0.0) if self.static else self.traj.sample(t)

    def target(self, t):
        """시각 t (start 뒤 sim s) 의 base_link (위치, 속도, 가속도) world 와 바퀴 회전각 [rad]."""
        s, v, a = self._sample(t)
        return self.p0 + s * self.dir_world, v * self.dir_world, a * self.dir_world, s / self.wheel_r

    def reseed(self, seed):
        """리셋: 직선 궤적은 정해진 하나라 seed 로 바뀌는 값이 없음 (기록용으로만 받음)."""
        self.seed = int(seed)

    def motion_params(self):
        """episode.json 에 남길 베이스 움직임 값."""
        if self.static:
            return {"kind": "linear", "preset": "static", "seed": self.seed}
        return {"kind": "linear", "preset": None, "seed": self.seed, **{k: self.cfg["trajectory"][k] for k in TRAJ_KEYS},
                "direction_world": np.round(self.dir_world, 6).tolist(), "period": round(self.traj.period, 4)}

    def kinematics(self, t):
        """시각 t 의 차체 (base_link) (위치, 선속도, 선가속도, 회전 R, 각속도, 각가속도) world. 직선 이동이라 회전은 처음 yaw 그대로."""
        p, v, a, _ = self.target(t)
        z = np.zeros(3)
        return p, v, a, _rz(self.yaw), z, z

    def joint_states(self, t):
        """(관절 이름, 위치, 속도): 바퀴 4 개 회전각 [rad]·각속도 [rad/s] (네 바퀴 같음, 미끄러짐 없음)."""
        s, v, _ = self._sample(t)
        n = len(self.joint_names)
        return self.joint_names, [s / self.wheel_r] * n, [v / self.wheel_r] * n

    def _apply(self, t):
        from pxr import Gf

        p, _, _, ang = self.target(t)
        self._t_op.Set(Gf.Vec3d(*p.tolist()))
        h = 0.5 * ang
        spin = Gf.Quatd(math.cos(h), Gf.Vec3d(*(math.sin(h) * self.wheel_axis).tolist()))
        for op, q0, qtype in self._wheel_ops:
            q = q0 * spin
            op.Set(qtype(q.GetReal(), *q.GetImaginary()))


# ── Stewart 플랫폼 (6 축 사인파) ──
SINE_AXES = ("x", "y", "z", "roll", "pitch", "yaw")
DIFF_H = 1e-4                                    # [s] 궤적 식의 수치 미분 간격 (각가속도·다리 속도)


class SineTrajectory:
    """q_k(t) = e(t) · s · A_k · sin(ω_k t + φ_k),  ω_k = c_k · Ω_k,  c_k ~ U(omega_scale_range),  φ_k ~ U(0, 2π)  (seed 로).
    e(t) = 10u³ − 15u⁴ + 6u⁵, u = clamp(t / ramp_time, 0, 1) → 램프 시작·끝에서 위치·속도·가속도가 이어짐.
    sample(t) → (q, q̇, q̈) 각각 (6,) 배열, 순서 SINE_AXES. omega_scale=(a, b) 로 범위를 덮어씀 (lab 재현 (1, 1))."""

    def __init__(self, motion, preset, seed=0, omega_scale=None):
        import design_stewart as dst

        if preset not in motion["presets"]:
            raise ValueError(f"움직임 프리셋 '{preset}' 이 없음 (있는 것: {sorted(motion['presets'])})")
        amp = dst.preset_amplitude(motion, preset)
        lo, hi = omega_scale or motion["omega_scale_range"]
        if not (0 < lo <= hi):
            raise ValueError(f"omega_scale 범위가 잘못됨: {lo}, {hi}")
        rng = np.random.default_rng(seed)
        self.preset, self.seed = preset, int(seed)
        self.ramp_time = float(motion["ramp_time"])
        self.amp = np.array([amp[k] for k in SINE_AXES])
        self.omega = np.array([float(motion["base_omega"][k]) * rng.uniform(lo, hi) for k in SINE_AXES])
        self.phase = np.array([rng.uniform(0.0, 2 * np.pi) for _ in SINE_AXES])
        self.collect = bool(motion["presets"][preset].get("collect", True))

    def params(self):
        """episode.json 에 남길 값."""
        return {"preset": self.preset, "seed": self.seed, "ramp_time": self.ramp_time, "axes": list(SINE_AXES),
                "amplitude": self.amp.round(6).tolist(), "omega": self.omega.round(6).tolist(), "phase": self.phase.round(6).tolist()}

    def _ramp(self, t):
        T = self.ramp_time
        if t <= 0:
            return 0.0, 0.0, 0.0
        if t >= T:
            return 1.0, 0.0, 0.0
        u = t / T                    # 5 차 (smootherstep): 시작·끝에서 위치·속도·가속도가 이어짐 (IMU 계단 없음)
        return 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5, 30 * u * u * (1 - u) ** 2 / T, 60 * u * (1 - u) * (1 - 2 * u) / (T * T)

    def sample(self, t):
        e, de, dde = self._ramp(t)
        arg = self.omega * t + self.phase
        s, ds, dds = self.amp * np.sin(arg), self.amp * self.omega * np.cos(arg), -self.amp * self.omega ** 2 * np.sin(arg)
        return e * s, de * s + e * ds, dde * s + 2 * de * ds + e * dds


def _quat_z_to(d):
    """+z 를 단위 벡터 d 로 돌리는 쿼터니언 (w, x, y, z). d = −z 근처 아님 (다리는 항상 위쪽)."""
    d = d / np.linalg.norm(d)
    c = float(d[2])
    if c < -1 + 1e-9:
        return (0.0, 1.0, 0.0, 0.0)
    w = math.sqrt((1 + c) / 2)
    return (w, -d[1] / (2 * w), d[0] / (2 * w), 0.0)


def _rz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class StewartBase(_StepDriver):
    """Stewart 플랫폼 USD (build_stewart_usd.py) 를 /World/<prim_name> 에 넣고, 윗판 (kinematic 강체) 을 SineTrajectory 로 움직인다.
    윗판 원점 = 회전 중심 = 윗판 윗면 가운데 (= 로봇 base 자리). 최상위 prim 은 바닥 (start_xy, z 0), 방향 start_yaw_deg 에 고정.
    다리 메시 (보이기만) 는 매 스텝 역기구학으로 방향·로드 위치를 맞춘다. 다리 길이가 legs 범위를 넘으면:
    수집 프리셋은 에러 (check() 에서), collect: false 프리셋 (lab) 은 기록만 (leg_over).
    순서: add() (재생 전) → 재생 → start() → 루프마다 check() → stop()"""

    def __init__(self, cfg, preset, seed=0, start_xy=(0.0, 0.0), start_yaw_deg=0.0, omega_scale=None):
        import design_stewart as dst

        self.cfg = cfg
        self.path = f"/World/{cfg['prim_name']}"
        self.body_path = f"{self.path}/{cfg['body']}"
        self.imu_path = f"{self.body_path}/imu_link"
        g = cfg["geometry"]
        self.height = float(g["height"])
        self.B, self.P = dst.anchors(g)              # B: 최상위 prim 좌표 (아랫판 윗면), P: 윗판 좌표
        self.lmin, self.lmax = float(cfg["legs"]["length_min"]), float(cfg["legs"]["length_max"])
        self.omega_scale = omega_scale
        self.traj = SineTrajectory(cfg["motion"], preset, seed, omega_scale)
        self.yaw = math.radians(start_yaw_deg)
        self.R0 = _rz(self.yaw)
        self.p0 = np.array([start_xy[0], start_xy[1], 0.0])
        self._rot = dst.rot
        self.leg_over = 0.0                          # 다리 길이가 범위를 넘은 최대량 [m] (lab 기록용)
        self.joint_names = [f"leg_{i + 1}" for i in range(6)]
        self._init_driver()
        self._ops = None

    def add(self, stage):
        """reference + 최상위 prim 위치 (바닥). 윗판은 USD 에서 이미 kinematic. 재생 전에 부를 것."""
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.experimental.prims import XformPrim
        from pxr import UsdGeom, UsdPhysics

        usd = _abs(self.cfg["usd"])
        if not os.path.isfile(usd):
            raise FileNotFoundError(f"베이스 USD 가 없습니다: {usd} (build_stewart_usd.py 로 만들 것)")
        stage_utils.add_reference_to_stage(usd, self.path)
        XformPrim(self.path, reset_xform_op_properties=True).set_world_poses(
            positions=[self.p0.tolist()], orientations=[_yaw_quat(self.yaw).tolist()])
        top = stage.GetPrimAtPath(self.body_path)
        if not (top.IsValid() and top.HasAPI(UsdPhysics.RigidBodyAPI) and UsdPhysics.RigidBodyAPI(top).GetKinematicEnabledAttr().Get()):
            raise RuntimeError(f"윗판이 kinematic 강체가 아님: {self.body_path}")
        if not stage.GetPrimAtPath(self.imu_path).IsValid():
            raise RuntimeError(f"imu_link 가 없음: {self.imu_path}")
        op = KinematicBase._op
        tx = UsdGeom.Xformable(top)
        self._ops = {"top_t": op(tx, UsdGeom.XformOp.TypeTranslate, self.body_path),
                     "top_o": op(tx, UsdGeom.XformOp.TypeOrient, self.body_path), "cyl_o": [], "rod_t": [], "rod_o": []}
        for i in range(6):
            cyl = UsdGeom.Xformable(stage.GetPrimAtPath(f"{self.path}/legs/leg_{i + 1}/cylinder"))
            rod = UsdGeom.Xformable(stage.GetPrimAtPath(f"{self.path}/legs/leg_{i + 1}/rod"))
            for x, name in ((cyl, "cylinder"), (rod, "rod")):
                if not x.GetPrim().IsValid():
                    raise RuntimeError(f"다리 {i + 1} {name} prim 이 없음")
                if x.GetPrim().HasAPI(UsdPhysics.RigidBodyAPI) or x.GetPrim().HasAPI(UsdPhysics.CollisionAPI):
                    raise RuntimeError(f"다리 {i + 1} {name}: 보이기만 하는 prim 이어야 함")
            self._ops["cyl_o"].append(op(cyl, UsdGeom.XformOp.TypeOrient, str(cyl.GetPath())))
            self._ops["rod_t"].append(op(rod, UsdGeom.XformOp.TypeTranslate, str(rod.GetPath())))
            self._ops["rod_o"].append(op(rod, UsdGeom.XformOp.TypeOrient, str(rod.GetPath())))
        return {"path": self.path, "body": self.body_path, "motion": self.traj.params()}

    # ── 자세 ──
    def local_pose(self, t):
        """최상위 prim 좌표의 윗판 (위치 c, 회전 R) 과 q, q̇, q̈."""
        q, dq, ddq = self.traj.sample(t)
        c = np.array([q[0], q[1], self.height + q[2]])
        R = self._rot(np.array(q[3]), np.array(q[4]), np.array(q[5]))
        return c, R, q, dq, ddq

    @staticmethod
    def omega_body(q, dq):
        """ZYX 오일러 각속도 → 윗판 좌표 각속도."""
        r, p = q[3], q[4]
        dr, dp, dy = dq[3], dq[4], dq[5]
        return np.array([dr - dy * math.sin(p),
                         dp * math.cos(r) + dy * math.sin(r) * math.cos(p),
                         -dp * math.sin(r) + dy * math.cos(r) * math.cos(p)])

    def target(self, t):
        """시각 t 의 윗판 회전 중심 (위치, 선속도, 선가속도, 회전 R, 각속도) world."""
        c, R, q, dq, ddq = self.local_pose(t)
        Rw = self.R0 @ R
        return (self.p0 + self.R0 @ c, self.R0 @ dq[:3], self.R0 @ ddq[:3], Rw, Rw @ self.omega_body(q, dq))

    def reseed(self, seed):
        """리셋: 같은 프리셋으로 ω·위상을 새 seed 로 다시 뽑는다 (움직이지 않을 때만 부를 것. t = 0 은 항상 가운데 자세)."""
        if self._cb is not None:
            raise RuntimeError("움직이는 중에는 reseed 할 수 없음 (stop() 먼저)")
        self.traj = SineTrajectory(self.cfg["motion"], self.traj.preset, seed, self.omega_scale)

    def motion_params(self):
        return {"kind": "stewart", **self.traj.params()}

    def kinematics(self, t):
        """시각 t 의 윗판 회전 중심 (위치, 선속도, 선가속도, 회전 R, 각속도, 각가속도) world.
        각가속도는 각속도의 중심 차분 (궤적이 식이라 h = 1e-4 s 에서 오차 ~1e-8)."""
        p, v, a, R, w = self.target(t)
        h = DIFF_H
        alpha = (self.target(t + h)[4] - self.target(t - h)[4]) / (2 * h)
        return p, v, a, R, w, alpha

    def joint_states(self, t):
        """(관절 이름, 위치, 속도): 다리 6 개 길이 [m]·늘어나는 속도 [m/s] (중심 차분)."""
        h = DIFF_H
        dL = (self.legs(t + h)[0] - self.legs(t - h)[0]) / (2 * h)
        return self.joint_names, self.legs(t)[0].tolist(), dL.tolist()

    def legs(self, t):
        """(다리 길이 (6,), 윗판 부착점 (6, 3), 단위 방향 (6, 3)) 최상위 prim 좌표."""
        c, R, *_ = self.local_pose(t)
        top = (R @ self.P.T).T + c
        v = top - self.B
        L = np.linalg.norm(v, axis=1)
        return L, top, v / L[:, None]

    def _apply(self, t):
        from pxr import Gf

        c, R, q, *_ = self.local_pose(t)
        self._ops["top_t"].Set(Gf.Vec3d(*c.tolist()))
        h = [0.5 * q[3], 0.5 * q[4], 0.5 * q[5]]
        qx = Gf.Quatd(math.cos(h[0]), Gf.Vec3d(math.sin(h[0]), 0, 0))
        qy = Gf.Quatd(math.cos(h[1]), Gf.Vec3d(0, math.sin(h[1]), 0))
        qz = Gf.Quatd(math.cos(h[2]), Gf.Vec3d(0, 0, math.sin(h[2])))
        self._ops["top_o"].Set(qz * qy * qx)                    # Rz·Ry·Rx
        L, top, d = self.legs(t)
        over = max(float((self.lmin - L).max()), float((L - self.lmax).max()), 0.0)
        if over > 0:
            self.leg_over = max(self.leg_over, over)
            if self.traj.collect:
                raise RuntimeError(f"다리 길이 {L.round(3).tolist()} 가 범위 {self.lmin} ~ {self.lmax} 밖 (t {t:.2f}, 프리셋 {self.traj.preset})")
        for i in range(6):
            qc, qr = _quat_z_to(d[i]), _quat_z_to(-d[i])
            self._ops["cyl_o"][i].Set(Gf.Quatd(float(qc[0]), Gf.Vec3d(*map(float, qc[1:]))))
            self._ops["rod_t"][i].Set(Gf.Vec3d(*top[i].tolist()))
            self._ops["rod_o"][i].Set(Gf.Quatd(float(qr[0]), Gf.Vec3d(*map(float, qr[1:]))))


def make_base(cfg, start_xy=(0.0, 0.0), start_yaw_deg=0.0, motion=None, seed=0, omega_scale=None):
    """설정 kind 로 베이스를 만든다. linear (Husky): motion None = 설정 trajectory, "static" = 서 있음.
    stewart: motion = 프리셋 이름 필수 (static 프리셋 = 진폭 0)."""
    if cfg["kind"] == "linear":
        if motion not in (None, "static"):
            raise ValueError(f"linear 베이스 (Husky) 의 움직임은 설정 trajectory (motion 없음) 또는 static (서 있음): {motion}")
        return KinematicBase(cfg, start_xy, start_yaw_deg, static=motion == "static")
    if motion is None:
        raise ValueError("stewart 베이스는 motion (프리셋 이름: small·mid·big·lab) 이 필요")
    return StewartBase(cfg, motion, seed, start_xy, start_yaw_deg, omega_scale)
