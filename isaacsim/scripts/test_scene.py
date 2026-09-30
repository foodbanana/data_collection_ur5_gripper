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
    """설정 파일을 읽는다. 비교 시험용으로 'file.yaml@articulation.self_collision=true' 처럼
    '@점.경로=값' 을 붙이면 그 값만 바꿔 읽는다 (파일은 그대로)."""
    import yaml

    path, *overrides = str(path).split("@")
    with open(path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    for ov in overrides:
        key, _, val = ov.partition("=")
        node, parts = cfg, key.split(".")
        for k in parts[:-1]:
            node = node[k]
        if parts[-1] not in node:
            raise KeyError(f"설정 덮어쓰기 대상이 없음: {key}")
        node[parts[-1]] = yaml.safe_load(val)
    for key in ("gravity_ff", "articulation", "collision_shapes", "contact", "home_pose", "arm", "gripper", "mimic"):
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
    api.CreateEnabledSelfCollisionsAttr().Set(bool(art_cfg["self_collision"]))
    api.CreateSleepThresholdAttr().Set(float(art_cfg["sleep_threshold"]))


def _apply_filter_pairs(stage, pairs):
    """충돌 제외 쌍을 씬 레이어에 적는다 (UsdPhysics.FilteredPairsAPI, 메모리 stage 에만)."""
    from pxr import UsdPhysics

    for a, b in pairs:
        pa, pb = stage.GetPrimAtPath(find_link_path(stage, a)), find_link_path(stage, b)
        UsdPhysics.FilteredPairsAPI.Apply(pa).CreateFilteredPairsRel().AddTarget(pb)


def _link_collision_meshes(stage, body):
    """링크(강체) 자신의 충돌 메시 prim 경로 (자식 링크 것은 제외, instance proxy 포함)."""
    from pxr import Usd, UsdGeom, UsdPhysics

    out = []
    for p in Usd.PrimRange(body, Usd.TraverseInstanceProxies()):
        if not (p.HasAPI(UsdPhysics.CollisionAPI) and p.IsA(UsdGeom.Mesh)):
            continue
        b = p
        while b and not b.HasAPI(UsdPhysics.RigidBodyAPI):
            b = b.GetParent()
        if b and b.GetPath() == body.GetPath():
            out.append(str(p.GetPath()))
    return out


def _deinstance_link_meshes(stage, link):
    """링크 자신의 충돌 메시 prim 들. instance 로 들어온 메시는 그 instance 를 풀어야(instanceable=false) 속성을 쓸 수 있다
    → 해당 링크 부분만 푼다 (메모리 stage 에만)."""
    body = stage.GetPrimAtPath(find_link_path(stage, link))
    paths = _link_collision_meshes(stage, body)
    if not paths:
        raise RuntimeError(f"{link} 에 충돌 메시가 없음")
    for path in paths:
        a = stage.GetPrimAtPath(path)
        while a and a.GetPath() != body.GetPath():
            if a.IsInstance():
                a.SetInstanceable(False)
            a = a.GetParent()
    prims = [stage.GetPrimAtPath(p) for p in paths]
    for p in prims:
        if p.IsInstanceProxy():
            raise RuntimeError(f"instance 를 풀지 못함: {p.GetPath()}")
    return prims


def _apply_convex_decomposition(stage, cd):
    """지정 링크의 충돌 메시를 convexDecomposition 으로 (메모리 stage 에만). 적용한 prim 경로 목록을 돌려줌."""
    from pxr import PhysxSchema, UsdPhysics

    if not cd["enabled"]:
        return []
    applied = []
    for link in cd["links"]:
        for prim in _deinstance_link_meshes(stage, link):
            path = str(prim.GetPath())
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr().Set("convexDecomposition")
            api = PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(prim)
            api.CreateMaxConvexHullsAttr().Set(int(cd["max_convex_hulls"]))
            api.CreateHullVertexLimitAttr().Set(int(cd["hull_vertex_limit"]))
            api.CreateVoxelResolutionAttr().Set(int(cd["voxel_resolution"]))
            api.CreateErrorPercentageAttr().Set(float(cd["error_percentage"]))
            api.CreateShrinkWrapAttr().Set(bool(cd["shrink_wrap"]))
            applied.append(path)
    return applied


def mimic_targets(stage):
    """그리퍼 관절 → mimic 기준 관절 이름 (NewtonMimicAPI 가 없으면 None)."""
    out = {}
    for j in GRIPPER_MIMIC:
        p = stage.GetPrimAtPath(find_link_path(stage, j))
        t = p.GetRelationship("newton:mimicJoint").GetTargets() if p.HasAPI("NewtonMimicAPI") else []
        out[j] = t[0].name if t else None
    return out


MATERIAL_ROOT = "/World/PhysicsMaterials"
FINGER_MATERIAL_PATH = f"{MATERIAL_ROOT}/finger"
OBJECT_MATERIAL_PATH = f"{MATERIAL_ROOT}/object"   # 시험 물체·드론이 bind_object_material 로 쓴다
COMBINE_MODES = ("average", "min", "multiply", "max")


def define_physics_material(stage, path, m):
    """physics 재질 (UsdPhysics.MaterialAPI + PhysxMaterialAPI 합치는 방식). m: 설정의 *_material dict."""
    from pxr import PhysxSchema, UsdPhysics, UsdShade

    for k in ("friction_combine", "restitution_combine"):
        if m[k] not in COMBINE_MODES:
            raise ValueError(f"{k} 는 {COMBINE_MODES} 중 하나: {m[k]}")
    mat = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr().Set(float(m["static_friction"]))
    api.CreateDynamicFrictionAttr().Set(float(m["dynamic_friction"]))
    api.CreateRestitutionAttr().Set(float(m["restitution"]))
    px = PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim())
    px.CreateFrictionCombineModeAttr().Set(m["friction_combine"])
    px.CreateRestitutionCombineModeAttr().Set(m["restitution_combine"])
    return mat


def bind_physics_material(prim, mat):
    """physics 용 material binding (자식 prim 의 binding 보다 강하게)."""
    from pxr import UsdShade

    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, bindingStrength=UsdShade.Tokens.strongerThanDescendants,
                                                  materialPurpose="physics")


def bind_object_material(stage, prim):
    from pxr import UsdShade

    bind_physics_material(prim, UsdShade.Material(stage.GetPrimAtPath(OBJECT_MATERIAL_PATH)))


def _apply_contact(stage, contact):
    """손가락 재질·접촉 거리, 시험 물체 재질 정의 (메모리 stage 에만)."""
    fm = define_physics_material(stage, FINGER_MATERIAL_PATH, contact["finger_material"])
    define_physics_material(stage, OBJECT_MATERIAL_PATH, contact["object_material"])
    co, ro = contact["finger_contact_offset"], contact["finger_rest_offset"]
    for link in contact["finger_links"]:
        bind_physics_material(stage.GetPrimAtPath(find_link_path(stage, link)), fm)
        if co is None and ro is None:
            continue
        from pxr import PhysxSchema

        for prim in _deinstance_link_meshes(stage, link):
            api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            if co is not None:
                api.CreateContactOffsetAttr().Set(float(co))
            if ro is not None:
                api.CreateRestOffsetAttr().Set(float(ro))


def bound_physics_material(prim):
    """prim 에 실제로 적용되는 physics 재질 경로 (조상 binding 포함, 없으면 '')."""
    from pxr import UsdShade

    mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial(materialPurpose="physics")
    return str(mat.GetPath()) if mat else ""


WRIST_CAMERA_CONFIG = os.path.join(CONFIG_DIR, "wrist_camera.yaml")
WRIST_CAMERA_NAME = "wrist_color_camera"
CAMERA_TILT_PIVOT = "wrist_camera_mount"   # xacro cam_tilt 가 이 프레임의 x 축으로 돈다


def load_camera_config(path=WRIST_CAMERA_CONFIG):
    import yaml

    with open(path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    for key in ("frame", "resolution", "intrinsics", "distortion", "horizontal_aperture", "clipping_range"):
        if key not in cfg:
            raise KeyError(f"{path}: '{key}' 항목이 없습니다")
    if any(abs(float(x)) > 0 for x in cfg["distortion"]):
        raise ValueError(f"{path}: 왜곡 계수가 0 이 아님 {cfg['distortion']} — sim 카메라는 왜곡 없는 핀홀")
    return cfg


def _rx(angle):
    from pxr import Gf

    return Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), float(np.degrees(angle))))


def camera_local_matrix(stage, optical_path, tilt):
    """optical frame 기준 Camera prim 의 local 행렬 (USD 행벡터 규약).
    tilt [rad]: xacro cam_tilt 흉내 — wrist_camera_mount 를 자기 x 축으로 돌린 것과 같은 자세 (씬 레이어 비교용).
    optical(+z 전방, +y 아래) → USD camera(−z 전방, +y 위) 는 x 축 180°."""
    from pxr import Usd, UsdGeom

    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    opt = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(optical_path))
    mount = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(find_link_path(stage, CAMERA_TILT_PIVOT)))
    a = opt * mount.GetInverse()            # optical → mount (행벡터: p_mount = p_opt · a)
    return _rx(np.pi) * a * _rx(tilt) * a.GetInverse()


def add_wrist_camera(stage, cam_cfg, tilt=0.0):
    """손목 color 카메라 Camera prim 을 optical frame 아래에 만든다 (메모리 stage 에만). prim 경로를 돌려준다."""
    from pxr import Gf, UsdGeom

    optical = find_link_path(stage, cam_cfg["frame"])
    path = f"{optical}/{WRIST_CAMERA_NAME}"
    cam = UsdGeom.Camera.Define(stage, path)
    w, h = (int(x) for x in cam_cfg["resolution"])
    k = cam_cfg["intrinsics"]
    pix = float(cam_cfg["horizontal_aperture"]) / w
    cam.CreateProjectionAttr().Set("perspective")
    cam.CreateFocalLengthAttr().Set(float(k["fx"]) * pix)
    cam.CreateHorizontalApertureAttr().Set(w * pix)
    cam.CreateVerticalApertureAttr().Set(h * pix)
    # aperture offset 부호: 가로는 −, 세로는 +. 2026-09-30 check_wrist_camera.py 표식으로 확인
    #   (가로를 + 로 두면 렌더링이 OpenCV 투영보다 2 × (cx − w/2) = 24.5 px 왼쪽에 찍힘)
    # + 0.5: OpenCV 는 픽셀 (i, j) 중심이 (i, j), 렌더러 영상 평면은 (i + 0.5, j + 0.5) → 주점을 0.5 px 옮겨 맞춤
    cam.CreateHorizontalApertureOffsetAttr().Set(-(float(k["cx"]) + 0.5 - w / 2) * pix)
    cam.CreateVerticalApertureOffsetAttr().Set((float(k["cy"]) + 0.5 - h / 2) * pix)
    cam.CreateClippingRangeAttr().Set(Gf.Vec2f(*[float(x) for x in cam_cfg["clipping_range"]]))
    _set_matrix(cam.GetPrim(), camera_local_matrix(stage, optical, tilt))
    return path


def _set_matrix(prim, m):
    """prim 의 변환을 행렬 op 하나로 (다른 코드가 translate/orient op 로 바꿔 놓아도 다시 행렬 하나로)."""
    from pxr import UsdGeom

    UsdGeom.Xformable(prim).MakeMatrixXform().Set(m)


def set_wrist_camera_tilt(stage, cam_path, tilt):
    """재생 중에도 Camera prim 자세만 바꿔 tilt 비교 (카메라 몸체·충돌 형상은 그대로)."""
    prim = stage.GetPrimAtPath(cam_path)
    _set_matrix(prim, camera_local_matrix(stage, str(prim.GetParent().GetPath()), tilt))


def filtered_pairs(stage):
    from pxr import Usd, UsdPhysics

    out = set()
    for p in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PATH)):
        if p.HasAPI(UsdPhysics.FilteredPairsAPI):
            for t in UsdPhysics.FilteredPairsAPI(p).GetFilteredPairsRel().GetTargets():
                out.add(frozenset((p.GetName(), str(t).split("/")[-1])))
    return out


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
    _apply_filter_pairs(stage, config["articulation"]["collision_filter_pairs"])
    info["convex_decomposition"] = _apply_convex_decomposition(stage, config["collision_shapes"]["convex_decomposition"])
    _apply_contact(stage, config["contact"])
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
        "self_collision": api.GetEnabledSelfCollisionsAttr().Get(),
        "sleep_threshold": api.GetSleepThresholdAttr().Get(),
    }
    cd = config["collision_shapes"]["convex_decomposition"]
    cd_got = []
    for link in (cd["links"] if cd["enabled"] else []):
        from pxr import UsdPhysics

        for path in _link_collision_meshes(stage, stage.GetPrimAtPath(find_link_path(stage, link))):
            cd_got.append((link, UsdPhysics.MeshCollisionAPI(stage.GetPrimAtPath(path)).GetApproximationAttr().Get()))
    lines.append(f"convex_decomposition: enabled={cd['enabled']} " + ", ".join(f"{l}={a}" for l, a in cd_got))
    if cd["enabled"] and (not cd_got or any(a != "convexDecomposition" for _, a in cd_got)):
        problems.append(f"convex decomposition 이 적용되지 않음: {cd_got}")
    fm_got = []
    for link in config["contact"]["finger_links"]:
        for path in _link_collision_meshes(stage, stage.GetPrimAtPath(find_link_path(stage, link))):
            fm_got.append((link, bound_physics_material(stage.GetPrimAtPath(path))))
    lines.append("finger physics material: " + ", ".join(f"{l}={m or '(없음)'}" for l, m in fm_got))
    if not fm_got or any(m != FINGER_MATERIAL_PATH for _, m in fm_got):
        problems.append(f"손가락 충돌 형상에 재질 {FINGER_MATERIAL_PATH} 이 적용되지 않음: {fm_got}")
    got_mimic = mimic_targets(stage)
    lines.append("gripper mimic: " + ", ".join(f"{j}→{t}" for j, t in got_mimic.items()))
    if got_mimic != {j: GRIPPER_DRIVE for j in GRIPPER_MIMIC}:
        problems.append(f"mimic 이 모두 {GRIPPER_DRIVE} 를 따르지 않음: {got_mimic}")
    want_pairs = {frozenset(p) for p in config["articulation"]["collision_filter_pairs"]}
    got_pairs = filtered_pairs(stage)
    lines.append(f"collision_filter_pairs: {sorted(tuple(sorted(p)) for p in got_pairs)} (설정 {len(want_pairs)} 쌍)")
    if got_pairs != want_pairs:
        problems.append(f"충돌 제외 쌍이 설정과 다름: {got_pairs} != {want_pairs}")
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
