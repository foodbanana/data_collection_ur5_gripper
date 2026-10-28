# =============================================================
# moving_base.py — 움직이는 베이스 (PLAN 8단계). 고정 베이스 씬 (drone_scene.py) 은 이 모듈을 쓰지 않는다
#
#   LinearTrajectory : 직선 왕복 궤적 (사다리꼴 속도: 가속 → 등속 → 감속 → 멈춤 → 되돌아옴). 시각 → (거리, 속도, 가속도)
#   KinematicBase    : 베이스 USD (예 Husky) 를 씬에 넣고 메모리 stage 에서만 kinematic 으로 바꾼 뒤,
#                      물리 스텝마다 (PHYSICS_PRE_STEP) 최상위 prim 의 USD 변환을 궤적 위치로 옮긴다.
#                      PhysX 는 이것을 kinematic target 으로 받아 베이스 속도를 안다 (순간이동 아님 → 위에 붙은 팔이 관성력을 받음).
#                      바퀴는 이동 거리 / 반지름 만큼 도는 표시만 (바퀴 물리·바닥 접촉 없음)
#
# 확인 (2026-10-28 실험): 최상위 prim translate 를 0.2 m/s 로 옮기면 차체·바퀴 강체 위치가 명령과 같고
#   PhysX 가 보고하는 선속도도 0.200 m/s.
#   바퀴 강체 prim 의 회전을 USD 로 바꾸면 PhysX 가 그 충돌 형상을 지우고 다시 만들어 tensor view 가 무효가 된다
#   → 바퀴 링크 아래 보이는 메시 prim (설정 wheels.visual, 물리 없음) 만 돌린다
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

BASE_KEYS = ("usd", "physics_variant", "prim_name", "body", "base_link_height", "wheels", "trajectory")
WHEEL_KEYS = ("links", "radius", "axis", "visual")
TRAJ_KEYS = ("direction", "distance", "max_speed", "accel", "dwell")
AXES = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}


def _abs(path):
    return path if os.path.isabs(path) else os.path.join(SIM_ROOT, path)


def load_base_config(path=None):
    path = path or DEFAULT_BASE_CONFIG
    with open(path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
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
    cfg["path"] = path
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


class KinematicBase:
    """베이스 USD 를 /World/<prim_name> 에 넣고 kinematic 궤적으로 움직인다.

    start_xy, start_yaw_deg: base_link 시작 위치 (world, 바닥 위 높이는 설정 base_link_height)·방향.
    순서: add() (재생 전) → 재생 → start() → 루프마다 check() → stop()
    """

    def __init__(self, cfg, start_xy=(0.0, 0.0), start_yaw_deg=0.0):
        self.cfg = cfg
        self.path = f"/World/{cfg['prim_name']}"
        self.body_path = f"{self.path}/{cfg['body']}"
        self.wheel_paths = [f"{self.path}/{w}" for w in cfg["wheels"]["links"]]
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
        self.t = 0.0
        self._t0 = None
        self._cb = None
        self._err = None
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
    def target(self, t):
        """시각 t (start 뒤 sim s) 의 base_link (위치, 속도, 가속도) world 와 바퀴 회전각 [rad]."""
        s, v, a = self.traj.sample(t)
        return self.p0 + s * self.dir_world, v * self.dir_world, a * self.dir_world, s / self.wheel_r

    def _apply(self, t):
        from pxr import Gf

        p, _, _, ang = self.target(t)
        self._t_op.Set(Gf.Vec3d(*p.tolist()))
        h = 0.5 * ang
        spin = Gf.Quatd(math.cos(h), Gf.Vec3d(*(math.sin(h) * self.wheel_axis).tolist()))
        for op, q0, qtype in self._wheel_ops:
            q = q0 * spin
            op.Set(qtype(q.GetReal(), *q.GetImaginary()))

    # ── 물리 스텝 콜백 ──
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
        # 이번 스텝이 끝나는 시각의 위치를 kinematic target 으로. 콜백 안 예외는 묻힐 수 있어 check() 에서 다시 올린다
        try:
            from isaacsim.core.simulation_manager import SimulationManager

            self.t = SimulationManager.get_simulation_time() + dt - self._t0
            self._apply(self.t)
        except Exception as e:  # noqa: BLE001
            self._err = e

    def check(self):
        if self._err is not None:
            err, self._err = self._err, None
            raise RuntimeError(f"KinematicBase 물리 콜백 실패: {err}") from err
