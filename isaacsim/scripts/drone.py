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
REQUIRED_KEYS = ("usd", "name", "mass", "grasp", "grasp_marker")
GRASP_KEYS = ("approach", "close_axis", "center_xy")
MARKER_KEYS = ("color", "roughness", "half_width", "z_max", "bottom_inner", "offset", "grid")
MARKER_NAME = "grasp_marker"        # body 링크 아래 표시 prim 이름 (_left / _right), 재질은 {DRONE_PATH}/Looks/grasp_marker
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
    gm = cfg["grasp_marker"]
    if gm is not None:
        for key in MARKER_KEYS:
            if key not in gm:
                raise KeyError(f"{path}: 'grasp_marker.{key}' 항목이 없습니다 (끄려면 grasp_marker: null)")
        if not (gm["half_width"] > 0 and gm["bottom_inner"] >= 0 and gm["offset"] > 0 and 0 < gm["grid"] <= gm["half_width"]):
            raise ValueError(f"{path}: grasp_marker 범위가 잘못됨: {gm}")
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
    marker = add_grasp_marker(stage, cfg, path) if cfg["grasp_marker"] is not None else None
    return {"path": path, "body_path": bp, "bodies": bodies, "articulation": bool(roots), "prop_spin": bool(spin),
            "grasp_marker": marker}


def _hull_strip(pts2, s, z_top, y_in, n):
    """단면 점 pts2 (y, z) 의 볼록 껍질을 따라 s 쪽 (닫는 방향 부호) 옆면 z = z_top 에서 바깥 모서리를 돌아 아랫면 s·y = y_in 까지의
    선을 호 길이로 n 개 같은 간격으로 뽑는다. (점 (n, 2), 바깥 법선 (n, 2), 호 길이 [m], 껍질 꼭짓점) 또는 범위가 없으면 None"""
    from scipy.spatial import ConvexHull

    hv = pts2[ConvexHull(pts2).vertices]                                 # 반시계 방향
    loop = np.vstack([hv, hv[:1]])

    def cross(f):
        """껍질 변 중 f(점) 부호가 바뀌는 곳: (변 번호 + 비율, 교점) 목록"""
        out = []
        for i in range(len(hv)):
            fa, fb = f(loop[i]), f(loop[i + 1])
            if fa == fb or fa * fb > 0:
                continue
            u = fa / (fa - fb)
            out.append((i + u, loop[i] + u * (loop[i + 1] - loop[i])))
        return out

    tops = [c for c in cross(lambda p: p[1] - z_top) if c[1][0] * s > 0]
    bots = [c for c in cross(lambda p: p[0] * s - y_in) if c[1][1] < z_top]
    if not tops or not bots:
        return None
    (ua, pa), (ub, pb) = max(tops, key=lambda c: c[1][0] * s), min(bots, key=lambda c: c[1][1])
    m = len(hv)
    ext = int(np.argmax(hv[:, 0] * s))                                   # 이 쪽 가장 바깥 꼭짓점을 지나는 방향으로 걷는다

    def walk(u0, u1, step):
        idx, i = [], (int(np.floor(u0)) + (1 if step > 0 else 0)) % m
        while True:
            if (step > 0 and (i - u0) % m > (u1 - u0) % m) or (step < 0 and (u0 - i) % m > (u0 - u1) % m):
                break
            idx.append(i)
            i = (i + step) % m
        return idx

    for step in (1, -1):
        mid = walk(ua, ub, step)
        if ext in mid:
            break
    else:
        return None
    P = np.vstack([pa, hv[mid], pb])
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    L = np.r_[0, np.cumsum(seg)]
    u = np.linspace(0, L[-1], n)
    Q = np.column_stack([np.interp(u, L, P[:, 0]), np.interp(u, L, P[:, 1])])
    T = np.gradient(Q, axis=0)
    N = np.column_stack([T[:, 1], -T[:, 0]])
    N /= np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-12)
    c = hv.mean(axis=0)
    N *= np.where(((Q - c) * N).sum(1) >= 0, 1.0, -1.0)[:, None]         # 껍질 안쪽에서 바깥으로
    return Q, N, float(L[-1]), hv


def add_grasp_marker(stage, cfg, path=DRONE_PATH):
    """잡는 곳 표시 (빨간 테이프, 설정 grasp_marker). 실물에서 테이프를 옆면 위쪽에서 아랫면 가장자리로 감아 팽팽하게 붙인 모양.
    손가락 폭 띠 |x − center_x| ≤ half_width 를 grid 간격 단면으로 잘라 (x 마다), 단면 (y, z) 의 볼록 껍질을 따라 옆면 z = z_max 에서
    바깥 모서리를 돌아 아랫면 |y| = bottom_inner 까지를 한 장으로 잇는다 → 구멍·홈 위는 건너고 돌기는 덮는다 (테이프처럼). x 방향은 단면마다
    따로라 허리의 오목한 곡선은 따라간다. offset 만큼 바깥으로 띄운 메시 2 개 (쪽마다 하나) 를 body 링크 아래에 만든다 (충돌·질량 없음).
    축소·위치를 적용한 뒤 (재생 전) 호출. 정보 dict 를 돌려준다."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade
    from scipy.spatial import cKDTree

    import collision_geom as cg

    gm = cfg["grasp_marker"]
    bp = body_path(cfg, path)
    body = stage.GetPrimAtPath(bp)
    prims = [p for p in collision_prims(stage, path) if _owner(p) == bp]
    meshes = [p for p in prims if p.IsA(UsdGeom.Mesh)]
    if not meshes or len(meshes) != len(prims):
        raise RuntimeError(f"grasp_marker: body 충돌 형상이 메시가 아님 ({[str(p.GetPath()) for p in prims]}). 끄려면 grasp_marker: null")
    n_coll = len(collision_prims(stage, path))
    R, t = drone_frame(stage, path)
    c_ax = np.array(cfg["grasp"]["close_axis"], dtype=float)
    a_ax = np.array(cfg["grasp"]["approach"], dtype=float)
    w_ax = np.cross(c_ax, a_ax)                                          # 손가락 폭 방향 (B 가 회전이 되게)
    B = np.column_stack([w_ax, c_ax, a_ax])                              # 드론 좌표계 → (폭, 닫는 방향, 접근) 좌표
    center = np.array([*cfg["grasp"]["center_xy"], 0.0]) @ B
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    M_body = cg.mat_np(cache.GetLocalToWorldTransform(body))
    hw, z1, y_in = float(gm["half_width"]), float(gm["z_max"]), float(gm["bottom_inner"])
    off, g = float(gm["offset"]), float(gm["grid"])
    vs, fs = [], []
    for prim in meshes:
        pts, faces, _ = cg._shape(prim)
        M = cg.mat_np(cache.GetLocalToWorldTransform(prim))
        fs.append(faces + sum(len(v) for v in vs))
        vs.append(((np.c_[pts, np.ones(len(pts))] @ M)[:, :3] - t) @ R @ B - center)       # (폭, 닫는 방향, 접근)
    mesh = cg.trimesh.Trimesh(np.vstack(vs), np.vstack(fs), process=False)
    xs = np.linspace(-hw, hw, int(round(2 * hw / g)) + 1)
    sections = []
    for x in xs:
        segs = cg.trimesh.intersections.mesh_plane(mesh, plane_normal=[1.0, 0, 0], plane_origin=[x, 0, 0])
        if len(segs) == 0:
            raise RuntimeError(f"grasp_marker: x = {x * 1000:.1f} mm 단면이 비었음 (half_width·grasp.center_xy 확인)")
        sections.append(segs[:, :, 1:])                                  # (선분 수, 2, 2) (y, z)

    info = {"color": [float(x) for x in gm["color"]], "half_width": hw, "z_max": z1, "bottom_inner": y_in, "grid": g, "sides": {}}
    M_inv = np.linalg.inv(M_body)
    mat = UsdShade.Material.Define(stage, f"{path}/Looks/{MARKER_NAME}")
    sh = UsdShade.Shader.Define(stage, f"{path}/Looks/{MARKER_NAME}/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*map(float, gm["color"])))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(gm["roughness"]))
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    for side, s in (("left", 1.0), ("right", -1.0)):
        # 단면마다 길이가 조금 달라 먼저 재고, 가장 긴 것 기준 grid 간격 개수로 모두 같은 수로 뽑는다 (이웃 단면을 사각형으로 잇기)
        lens = []
        for sec in sections:
            r = _hull_strip(sec.reshape(-1, 2), s, z1, y_in, 2)
            if r is None:
                raise RuntimeError(f"grasp_marker: {side} 단면에서 z_max·bottom_inner 사이 껍질을 찾지 못함 (값 확인)")
            lens.append(r[2])
        n = int(np.ceil(max(lens) / g)) + 1
        rows, bridge = [], 0.0
        for x, sec in zip(xs, sections):
            Q, N, _, _ = _hull_strip(sec.reshape(-1, 2), s, z1, y_in, n)
            # 껍질이 표면에서 떨어진 최대 거리 = 건넌 홈 깊이 (선분 위를 0.1 mm 간격으로 채운 점까지. 끝점만 쓰면 점 간격만큼 크게 나옴)
            k = np.linspace(0, 1, max(2, int(np.ceil(np.linalg.norm(sec[:, 1] - sec[:, 0], axis=1).max() / 1e-4)) + 1))
            dense = (sec[:, :1] + k[None, :, None] * (sec[:, 1:] - sec[:, :1])).reshape(-1, 2)
            bridge = max(bridge, float(cKDTree(dense).query(Q)[0].max()))
            Qo = Q + N * off
            rows.append(np.column_stack([np.full(n, x), Qo]))
        P = np.stack(rows)                                               # (x 수, n, 3)
        verts = P.reshape(-1, 3)
        Fall = []
        for i in range(len(xs) - 1):
            for j in range(n - 1):
                q = [i * n + j, (i + 1) * n + j, (i + 1) * n + j + 1, i * n + j + 1]
                nq = np.cross(verts[q[1]] - verts[q[0]], verts[q[3]] - verts[q[0]])
                outv = verts[q].mean(0) - np.array([verts[q].mean(0)[0], *sections[i].reshape(-1, 2).mean(0)])
                Fall.append(q if nq @ outv > 0 else q[::-1])
        qv = verts[np.array(Fall)]
        area = float(0.5 * (np.linalg.norm(np.cross(qv[:, 1] - qv[:, 0], qv[:, 2] - qv[:, 0]), axis=1)
                            + np.linalg.norm(np.cross(qv[:, 2] - qv[:, 0], qv[:, 3] - qv[:, 0]), axis=1)).sum())
        w = ((verts + center) @ B.T) @ R.T + t                           # world
        local = (np.c_[w, np.ones(len(w))] @ M_inv)[:, :3]               # body 링크 좌표
        gp = UsdGeom.Mesh.Define(stage, f"{bp}/{MARKER_NAME}_{side}")
        gp.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in local])
        gp.CreateFaceVertexCountsAttr([4] * len(Fall))
        gp.CreateFaceVertexIndicesAttr([int(i) for f in Fall for i in f])
        gp.CreateOrientationAttr(UsdGeom.Tokens.rightHanded)
        gp.CreateDoubleSidedAttr(False)
        gp.CreateSubdivisionSchemeAttr("none")
        UsdShade.MaterialBindingAPI.Apply(gp.GetPrim()).Bind(mat)
        # 실물 테이프: 폭 2 × half_width, 길이 = 단면 껍질 길이 (옆면 위 끝 → 아랫면 안쪽 끝)
        info["sides"][side] = {"faces": len(Fall), "area_cm2": round(area * 1e4, 2),
                               "length_mm": [round(min(lens) * 1000, 1), round(max(lens) * 1000, 1)],
                               "max_bridge_mm": round(bridge * 1000, 1),
                               "close_axis_range_mm": [round(float(verts[:, 1].min()) * 1000, 1), round(float(verts[:, 1].max()) * 1000, 1)],
                               "z_range_mm": [round(float(verts[:, 2].min()) * 1000, 1), round(float(verts[:, 2].max()) * 1000, 1)]}
    if len(collision_prims(stage, path)) != n_coll:
        raise RuntimeError("grasp_marker 를 만든 뒤 충돌 형상 수가 바뀜 (표시는 보이기만 해야 함)")
    return info


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
