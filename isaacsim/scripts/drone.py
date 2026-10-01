# =============================================================
# drone.py — 2단계 드론 공용 모듈 (docs/PLAN.md 2단계)
#
#   add_drone(): 드론 USD 를 reference 하고 씬 레이어에서 덮어쓴다 (원본 USD 는 그대로)
#     - 축소(scale), 링크 질량(link_masses), 관절 고정(lock_joints → FixedJoint), 저장된 초기 속도 0, 물체 재질, 위치
#   grasp_width(): 잡는 곳(손가락 폭 slab)에서 충돌 형상의 잡는 방향 폭
#   설정: isaacsim/config/drone_*.yaml
#     drone_iris.yaml   — 3DR Iris (PX4 기본 기체 외형) 0.75 배, 데모 기본값
#     drone_simple.yaml — 기본 도형 간이 드론 (build_simple_drone.py 가 같은 yaml 로 USD 를 만든다)
#
# 좌표: "드론 좌표계" = 드론 최상위 prim 의 world 위치·자세 (축소는 반영된 m 단위)
# SimulationApp 을 만든 뒤에 import 할 것.
# =============================================================

import os

import numpy as np

import test_scene as ts

DEFAULT_DRONE_CONFIG = os.path.join(ts.CONFIG_DIR, "drone_iris.yaml")
DRONE_PATH = "/World/Drone"
LOCK_SUFFIX = "_fixed"
REQUIRED_KEYS = ("usd", "name", "mass", "grasp")
GRASP_KEYS = ("approach", "close_axis", "center_xy")
FINGER_HALF_WIDTH = 0.013   # m, 손가락 폭의 절반 (TCP 좌표계 r2·l2 x 범위 ±13 mm, PLAN 2-1). 잡는 폭을 잴 slab


def load_drone_config(path=DEFAULT_DRONE_CONFIG):
    import yaml

    with open(path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    for key in REQUIRED_KEYS:
        if key not in cfg:
            raise KeyError(f"{path}: '{key}' 항목이 없습니다")
    for key in GRASP_KEYS:
        if key not in cfg["grasp"]:
            raise KeyError(f"{path}: 'grasp.{key}' 항목이 없습니다")
    cfg.setdefault("body_link", "")          # "" = 최상위 prim 이 강체 (간이 드론)
    cfg.setdefault("scale", 1.0)
    cfg.setdefault("link_masses", None)
    cfg.setdefault("lock_joints", [])
    cfg.setdefault("zero_initial_velocity", False)
    cfg.setdefault("articulation", None)
    if cfg["link_masses"] is not None and abs(sum(cfg["link_masses"].values()) - cfg["mass"]) > 1e-9:
        raise ValueError(f"{path}: link_masses 합계 {sum(cfg['link_masses'].values())} != mass {cfg['mass']}")
    if not cfg["scale"] > 0:
        raise ValueError(f"{path}: scale 은 양수: {cfg['scale']}")
    usd = cfg["usd"] if os.path.isabs(cfg["usd"]) else os.path.join(ts.SIM_ROOT, cfg["usd"])
    if not os.path.isfile(usd):
        raise FileNotFoundError(f"드론 USD 가 없습니다: {usd} (간이 드론이면 build_simple_drone.py 로 빌드)")
    cfg["usd_abs"] = usd
    cfg["path"] = path
    return cfg


def _sub(path, rel):
    return f"{path}/{rel}" if rel else path


def body_path(cfg, path=DRONE_PATH):
    return _sub(path, cfg["body_link"])


def rigid_bodies(stage, path=DRONE_PATH):
    from pxr import Usd, UsdPhysics

    return [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(path)) if p.HasAPI(UsdPhysics.RigidBodyAPI)]


def collision_prims(stage, path=DRONE_PATH):
    from pxr import Usd, UsdPhysics

    return [p for p in Usd.PrimRange(stage.GetPrimAtPath(path)) if p.HasAPI(UsdPhysics.CollisionAPI)]


def _lock_joint(stage, jp):
    """원래 관절을 비활성화하고 같은 두 링크·같은 프레임으로 FixedJoint.
    (관절 한계 0/0 은 충격에 0.79° 움직임, 2026-10-01 Quadcopter 시험 → FixedJoint 는 articulation 안에서 완전히 고정)"""
    from pxr import UsdPhysics

    src = UsdPhysics.Joint(jp)
    fj = UsdPhysics.FixedJoint.Define(stage, f"{jp.GetPath()}{LOCK_SUFFIX}")
    for get, create in ((src.GetBody0Rel, fj.CreateBody0Rel), (src.GetBody1Rel, fj.CreateBody1Rel)):
        targets = get().GetTargets()
        if len(targets) != 1:
            raise RuntimeError(f"{jp.GetPath()} body 대상이 1 개가 아님: {targets}")
        create().SetTargets(targets)
    for name in ("LocalPos0", "LocalRot0", "LocalPos1", "LocalRot1"):
        getattr(fj, f"Create{name}Attr")().Set(getattr(src, f"Get{name}Attr")().Get())
    jp.SetActive(False)


def add_drone(stage, cfg, position, path=DRONE_PATH, prop_spin=False):
    """드론을 path 에 넣고 덮어쓴다 (메모리 stage 에만). 재생 전에 호출. 정보 dict 를 돌려준다.
    prop_spin: True 면 flight.prop_joints 는 고정하지 않고 회전 관절로 둔다 (보여 주기용 프로펠러 회전, drone_flight)"""
    import isaacsim.core.experimental.utils.stage as stage_utils
    from isaacsim.core.experimental.prims import XformPrim
    from pxr import Gf, PhysxSchema, Usd, UsdPhysics

    stage_utils.add_reference_to_stage(usd_path=cfg["usd_abs"], path=path)
    top = stage.GetPrimAtPath(path)
    bodies = rigid_bodies(stage, path)
    bp = body_path(cfg, path)
    if bp not in bodies:
        raise RuntimeError(f"body_link '{cfg['body_link']}' 가 강체가 아님 (강체: {bodies})")
    if not collision_prims(stage, path):
        raise RuntimeError(f"드론에 충돌 형상이 없음: {path}")
    roots = [str(p.GetPath()) for p in Usd.PrimRange(top) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if len(bodies) > 1 and roots != [path]:
        raise RuntimeError(f"강체가 여럿이면 articulation root 가 최상위 prim 하나여야 함: {roots}")

    if cfg["link_masses"] is not None:
        named = {_sub(path, k): v for k, v in cfg["link_masses"].items()}
        if set(named) != set(bodies):
            raise RuntimeError(f"link_masses 링크 {sorted(named)} != 드론 강체 {bodies}")
        for p, m in named.items():
            api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(p))
            api.CreateMassAttr().Set(float(m))
            api.CreateDensityAttr().Set(0.0)
            # 원본에 저장된 관성·주축이 있으면 질량과 맞지 않으므로 지워서 PhysX 가 형상에서 다시 계산하게 한다
            for a in ("physics:diagonalInertia", "physics:principalAxes"):
                attr = stage.GetPrimAtPath(p).GetAttribute(a)
                if attr and attr.HasAuthoredValue():
                    attr.Set(Gf.Vec3f(0, 0, 0) if a.endswith("Inertia") else Gf.Quatf(0, 0, 0, 0))

    spin = []
    if prop_spin:
        if "flight" not in cfg or not cfg["flight"].get("prop_joints"):
            raise KeyError(f"{cfg['path']}: prop_spin 을 켜려면 flight.prop_joints 가 필요")
        spin = list(cfg["flight"]["prop_joints"])
        for j in spin:
            if not stage.GetPrimAtPath(_sub(path, j)).IsA(UsdPhysics.RevoluteJoint):
                raise RuntimeError(f"프로펠러 관절이 revolute 가 아님: {_sub(path, j)}")
    for j in cfg["lock_joints"]:
        if j in spin:
            continue
        jp = stage.GetPrimAtPath(_sub(path, j))
        if not jp.IsValid() or not jp.IsA(UsdPhysics.Joint):
            raise RuntimeError(f"고정할 관절이 없음: {_sub(path, j)}")
        _lock_joint(stage, jp)

    if cfg["zero_initial_velocity"]:
        for p in bodies:
            rb = UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(p))
            rb.CreateVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
            rb.CreateAngularVelocityAttr().Set(Gf.Vec3f(0, 0, 0))

    if cfg["articulation"] is not None:
        if roots != [path]:
            raise RuntimeError(f"articulation 설정이 있는데 최상위 prim 이 articulation root 가 아님: {roots}")
        PhysxSchema.PhysxArticulationAPI.Apply(top).CreateSleepThresholdAttr().Set(
            float(cfg["articulation"]["sleep_threshold"]))

    if not stage.GetPrimAtPath(ts.OBJECT_MATERIAL_PATH).IsValid():
        raise RuntimeError(f"물체 재질 {ts.OBJECT_MATERIAL_PATH} 이 없음 (build_test_scene 또는 define_physics_material 먼저)")
    ts.bind_object_material(stage, top)

    xp = XformPrim(path, reset_xform_op_properties=True)
    xp.set_world_poses(positions=[list(map(float, position))])
    s = float(cfg["scale"])
    xp.set_local_scales([[s, s, s]])
    return {"path": path, "body_path": bp, "bodies": bodies, "articulation": bool(roots), "prop_spin": bool(spin)}


def drone_frame(stage, path=DRONE_PATH):
    """드론 좌표계 (R, t): 최상위 prim world 변환에서 축소를 뺀 회전·위치 (USD 값, 재생 중이면 Fabric 이 아닌 USD 기준)."""
    from pxr import Usd, UsdGeom

    m = UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = np.array(m.ExtractTranslation())
    R = np.array(m.ExtractRotationMatrix()).T          # USD 행벡터 → 열벡터 규약
    R = R / np.linalg.norm(R, axis=0)                   # 축소 제거
    return R, t


def collision_points(stage, prims, R, t):
    """충돌 형상들을 드론 좌표계 trimesh 목록으로 (메시는 원래 면, 프리미티브는 convex hull)."""
    import trimesh
    from pxr import Usd, UsdGeom

    import collision_geom as cg

    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    out = []
    for prim in prims:
        pts, faces, is_mesh = cg._shape(prim)
        M = cg.mat_np(cache.GetLocalToWorldTransform(prim))
        w = (np.c_[pts, np.ones(len(pts))] @ M)[:, :3]
        local = (w - t) @ R
        out.append(trimesh.Trimesh(local, faces, process=False) if is_mesh else trimesh.convex.convex_hull(local))
    return out


def grasp_width(stage, cfg, path=DRONE_PATH, half_width=FINGER_HALF_WIDTH):
    """잡는 곳(center_xy)을 지나는 손가락 폭 slab 안에서 body 링크 충돌 형상의 잡는 방향 폭 [m] 과 (최소, 최대) 좌표.
    slab 방향 = approach × close_axis (손가락 폭 방향)."""
    from pxr import Usd, UsdPhysics

    a = np.array(cfg["grasp"]["approach"], dtype=float)
    c = np.array(cfg["grasp"]["close_axis"], dtype=float)
    n = np.cross(a, c)
    n /= np.linalg.norm(n)
    center = np.array([*cfg["grasp"]["center_xy"], 0.0], dtype=float)
    bp = body_path(cfg, path)
    prims = [p for p in collision_prims(stage, path) if _owner(p) == bp]
    R, t = drone_frame(stage, path)
    # 면을 slab |(x − center)·n| ≤ half_width 로 자른 영역의 끝점 = slab 안 꼭짓점 + 모서리와 두 경계 평면의 교점
    #   (trimesh.slice_plane 은 shapely 가 필요해 직접 계산)
    lo, hi = np.inf, -np.inf
    for mesh in collision_points(stage, prims, R, t):
        v = mesh.vertices
        d = (v - center) @ n
        pts = [v[np.abs(d) <= half_width]]
        e = mesh.edges_unique
        da, db = d[e[:, 0]], d[e[:, 1]]
        for b in (-half_width, half_width):
            cross = (da - b) * (db - b) < 0
            s = (b - da[cross]) / (db[cross] - da[cross])
            pts.append(v[e[cross, 0]] + s[:, None] * (v[e[cross, 1]] - v[e[cross, 0]]))
        pts = np.vstack(pts)
        if len(pts):
            w = pts @ c
            lo, hi = min(lo, float(w.min())), max(hi, float(w.max()))
    if not np.isfinite(lo):
        raise RuntimeError("잡는 곳 slab 안에 body 충돌 형상이 없음 (grasp.center_xy 확인)")
    return hi - lo, (lo, hi)


def _owner(prim):
    from pxr import UsdPhysics

    p = prim
    while p and p.IsValid() and not p.HasAPI(UsdPhysics.RigidBodyAPI):
        p = p.GetParent()
    return str(p.GetPath()) if p and p.IsValid() else None
