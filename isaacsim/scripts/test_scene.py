# =============================================================
# test_scene.py — 1-4/1-5/1-6 공용 테스트 씬
#
#   physics scene(PhysX) + ground plane + 로봇 USD reference, 최상위 prim z = ROBOT_BASE_Z
#   로봇 USD 원본은 수정하지 않는다.
#
# SimulationApp 을 만든 뒤에 import 할 것 (isaacsim 모듈은 app 시작 후에만 import 가능).
# =============================================================

import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_ROOT = os.path.dirname(HERE)  # ~/data_collection_ur5_gripper/isaacsim
DEFAULT_USD = os.path.join(SIM_ROOT, "assets/robots/ur5_rh_p12_d435i/ur5_rh_p12_d435i.usda")
REPORT_DIR = os.path.join(SIM_ROOT, "reports")
CONFIG_DIR = os.path.join(SIM_ROOT, "config")

ROBOT_PATH = "/World/ur5_rh_p12_d435i"
ROBOT_BASE_Z = 0.762  # m, 테이블 상판 높이. 0 이면 관절 0 자세에서 팔이 ground plane 에 걸친다
PHYSICS_DT = 1.0 / 120.0

ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
GRIPPER_DRIVE = "rh_r1_joint"
GRIPPER_MIMIC = ["rh_r2", "rh_l1", "rh_l2"]
GRIPPER_JOINTS = [GRIPPER_DRIVE] + GRIPPER_MIMIC


DEFAULT_CONFIG = os.path.join(CONFIG_DIR, "drive_gains.yaml")
BASE_MODES = ("fixed", "floating")
MOUNT_PATH = f"{ROBOT_PATH}/Geometry/robot_mount"


def load_config(path=DEFAULT_CONFIG):
    import yaml

    with open(path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    for key in ("gravity_ff", "articulation", "home_pose", "arm", "gripper", "mimic"):
        if key not in cfg:
            raise KeyError(f"{path}: '{key}' 항목이 없습니다")
    return cfg


def articulation_roots(stage):
    from pxr import Usd, UsdPhysics

    return [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PATH))
            if p.HasAPI(UsdPhysics.ArticulationRootAPI)]


def _apply_articulation_settings(prim, art_cfg):
    from pxr import PhysxSchema

    api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
    api.CreateSolverPositionIterationCountAttr().Set(int(art_cfg["solver_position_iterations"]))
    api.CreateSolverVelocityIterationCountAttr().Set(int(art_cfg["solver_velocity_iterations"]))
    api.CreateEnabledSelfCollisionsAttr().Set(bool(art_cfg["enabled_self_collisions"]))
    api.CreateSleepThresholdAttr().Set(float(art_cfg["sleep_threshold"]))


def _make_fixed_base(stage):
    """ArticulationRootAPI 를 robot_mount → 최상위 prim 으로 옮긴다 (메모리 stage 에만, USD 원본은 그대로).
    root_joint(최상위 prim ↔ robot_mount)가 articulation 안으로 들어와 fixed base 가 된다.
    원본처럼 robot_mount 에 root 가 있으면 root_joint 가 articulation 밖 구속이 되어 floating base 로 풀린다."""
    from pxr import UsdPhysics

    mount = stage.GetPrimAtPath(MOUNT_PATH)
    top = stage.GetPrimAtPath(ROBOT_PATH)
    mount.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    for s in list(mount.GetAppliedSchemas()):
        if "ArticulationRoot" in s or s == "PhysxArticulationAPI":  # NewtonArticulationRootAPI 포함
            mount.RemoveAppliedSchema(s)
    UsdPhysics.ArticulationRootAPI.Apply(top)
    return top


def build_test_scene(usd_path, physics_variant="physx", base_z=ROBOT_BASE_Z, base="fixed", config=None, ground=True,
                     light=False):
    """새 stage 에 테스트 씬을 만들고 (stage, Articulation, info) 를 돌려준다. 재생은 하지 않는다.

    base="fixed":    ArticulationRootAPI 를 최상위 prim 으로 옮겨 fixed base (1~7단계)
    base="floating": USD 원본 그대로 (root 는 robot_mount, root_joint 는 외부 구속). 8단계에서 다시 결정
    config: load_config() 결과. articulation 설정을 새 root 에 적는다.
    ground: False 면 ground plane 을 만들지 않는다 (중력 시험에서 팔이 바닥에 닿지 않게)
    light: True 면 Dome Light 를 추가한다 (GUI 로 볼 때. 조명이 없으면 뷰포트가 비어 보임). 물리에는 영향 없음
    """
    import isaacsim.core.experimental.utils.stage as stage_utils
    from isaacsim.core.experimental.objects import DomeLight, GroundPlane
    from isaacsim.core.experimental.prims import Articulation, XformPrim
    from isaacsim.core.simulation_manager import SimulationManager

    if not os.path.isfile(usd_path):
        raise FileNotFoundError(f"로봇 USD 가 없습니다: {usd_path}")
    if base not in BASE_MODES:
        raise ValueError(f"base 는 {BASE_MODES} 중 하나: {base}")
    if config is None:
        config = load_config()

    stage_utils.create_new_stage()
    stage_utils.set_stage_up_axis("Z")
    stage_utils.set_stage_units(meters_per_unit=1.0, kilograms_per_unit=1.0)
    stage = stage_utils.get_current_stage(backend="usd")
    stage_utils.define_prim("/World", "Xform")

    # isaacsim.physics.newton 이 켜져 있으면 Newton 으로 자동 전환될 수 있다 → PhysX 명시
    if not SimulationManager.switch_physics_engine("physx"):
        raise RuntimeError("PhysX 로 전환하지 못했습니다")
    engine = SimulationManager.get_active_physics_engine()
    if engine != "physx":
        raise RuntimeError(f"physics engine 이 physx 가 아닙니다: {engine}")
    SimulationManager.setup_simulation(dt=PHYSICS_DT)

    if ground:
        GroundPlane("/World/GroundPlane")
    if light:
        DomeLight("/World/DomeLight").set_intensities(1000)
    # Physics variant 는 기본 선택이 없으므로 명시
    stage_utils.add_reference_to_stage(usd_path=usd_path, path=ROBOT_PATH, variants=[("Physics", physics_variant)])
    # 배치 규칙(1-4): 위치는 최상위 prim Transform 으로만 지정 (root_joint 가 이 위치에 고정)
    XformPrim(ROBOT_PATH, reset_xform_op_properties=True).set_world_poses(positions=[0.0, 0.0, base_z])

    info = {"base": base, "asset_roots": articulation_roots(stage)}
    if info["asset_roots"] != [MOUNT_PATH]:
        raise RuntimeError(f"로봇 USD 의 ArticulationRootAPI 위치가 예상({MOUNT_PATH})과 다름: {info['asset_roots']}")
    root_prim = _make_fixed_base(stage) if base == "fixed" else stage.GetPrimAtPath(MOUNT_PATH)
    _apply_articulation_settings(root_prim, config["articulation"])
    info["root"] = str(root_prim.GetPath())

    return stage, Articulation(ROBOT_PATH), info


def verify_articulation(stage, robot, info, config):
    """재생 후 호출. articulation 설정이 실제로 적용됐는지 읽어서 확인한다. (문제 목록, 로그 줄) 을 돌려준다."""
    from pxr import PhysxSchema

    problems, lines = [], []
    roots = articulation_roots(stage)
    lines.append(f"ArticulationRootAPI: {roots} (USD 원본: {info['asset_roots']})")
    if roots != [info["root"]]:
        problems.append(f"root 가 {info['root']} 하나가 아님: {roots}")

    fixed = bool(robot._physics_articulation_view.shared_metatype.fixed_base)  # PhysX 가 실제로 만든 articulation
    lines.append(f"PhysX fixed_base = {fixed}")
    if fixed != (info["base"] == "fixed"):
        problems.append(f"fixed_base={fixed} 인데 base={info['base']}")

    api = PhysxSchema.PhysxArticulationAPI(stage.GetPrimAtPath(info["root"]))
    got = {
        "solver_position_iterations": api.GetSolverPositionIterationCountAttr().Get(),
        "solver_velocity_iterations": api.GetSolverVelocityIterationCountAttr().Get(),
        "enabled_self_collisions": api.GetEnabledSelfCollisionsAttr().Get(),
        "sleep_threshold": api.GetSleepThresholdAttr().Get(),
    }
    for k, v in got.items():
        want = config["articulation"][k]
        lines.append(f"{k}: {v} (설정 {want})")
        if v != want:
            problems.append(f"{k}={v} != 설정 {want}")
    return problems, lines


def dof_index_map(robot):
    """이름 → DOF index. 필요한 관절이 없으면 에러 (조용한 fallback 금지)."""
    names = list(robot.dof_names)
    missing = [n for n in ARM_JOINTS + GRIPPER_JOINTS if n not in names]
    if missing:
        raise RuntimeError(f"DOF 에 없는 관절: {missing} (DOF: {names})")
    return {n: i for i, n in enumerate(names)}


PAYLOAD_PATH = "/World/Payload"
PAYLOAD_LINK = "rh_p12_rn_base"


def find_link_path(stage, name):
    from pxr import Usd

    for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PATH)):
        if prim.GetName() == name:
            return str(prim.GetPath())
    raise KeyError(f"{name} 링크가 없음")


def add_payload(stage, mass_kg, local_pos):
    """드론 무게 흉내: 충돌 없는 강체(mass_kg)를 그리퍼 base 링크에 FixedJoint 로 붙인다 (재생 전에 호출).
    articulation 밖의 강체라서 로봇 중력 보상(get_dof_gravity_compensation_forces)에는 포함되지 않는다.
    local_pos: rh_p12_rn_base 좌표계에서 드론 무게중심 위치 [m]."""
    from pxr import Gf, UsdGeom, UsdPhysics

    link = find_link_path(stage, PAYLOAD_LINK)
    body = UsdGeom.Xform.Define(stage, PAYLOAD_PATH).GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(body)
    mass = UsdPhysics.MassAPI.Apply(body)
    mass.CreateMassAttr().Set(float(mass_kg))
    i = 0.4 * mass_kg * 0.05 ** 2  # 반지름 5 cm 구 정도의 관성 (정적 시험이라 영향 작음)
    mass.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(i, i, i))
    mass.CreateCenterOfMassAttr().Set(Gf.Vec3f(0, 0, 0))
    joint = UsdPhysics.FixedJoint.Define(stage, PAYLOAD_PATH + "_joint")
    joint.CreateBody0Rel().SetTargets([link])
    joint.CreateBody1Rel().SetTargets([PAYLOAD_PATH])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(x) for x in local_pos]))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    # 이 조인트가 없으면 PhysX 가 드론 강체를 articulation 의 새 링크로 흡수해 중력 보상 계산에 드론 무게가 들어간다
    joint.CreateExcludeFromArticulationAttr().Set(True)
    return link


def snap_payload(link_path, local_pos):
    """재생 중 로봇 자세를 순간이동시킨 뒤, 드론 강체도 링크에 맞는 위치로 옮긴다 (조인트가 튀지 않게)."""
    from isaacsim.core.experimental.prims import RigidPrim

    link = RigidPrim(link_path)
    pos, quat = (x.numpy()[0] for x in link.get_world_poses())
    w, x, y, z = quat
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    p = pos + R @ np.asarray(local_pos, dtype=float)
    body = RigidPrim(PAYLOAD_PATH)
    body.set_world_poses(positions=p.reshape(1, 3), orientations=quat.reshape(1, 4))
    body.set_velocities(np.zeros((1, 3)), np.zeros((1, 3)))
