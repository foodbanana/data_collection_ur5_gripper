#!/usr/bin/env python3
# =============================================================
# check_self_collision.py  (docs/PLAN.md 1-6, self-collision 켜기 전 점검)
#
# 여러 자세에서 로봇 링크 쌍별 충돌 형상 간 거리를 계산해, 겹치거나 가까운(기본 2 mm 이내) 쌍을 보여준다.
#   - 충돌 형상: USD 의 CollisionAPI prim (메시는 PhysX 와 같게 convex hull, 박스·원기둥은 그 형상)
#   - 거리: 각 형상의 볼록 껍질 표면을 SPACING 간격으로 표본화 → 최소 점간 거리 (오차 ≈ 표본 간격).
#           한쪽 표본이 다른 쪽 껍질 안에 있으면 겹침으로 보고 파고든 깊이를 음수로 표시
#   - 관절로 직접 연결된 쌍은 USD 에서 이미 충돌이 꺼져 있음 (joint collisionEnabled = false) → 표시만
#   - 모든 자세에서 가까운 쌍 = 충돌 제외 쌍 후보
#   - B 시험(gravity_worst_cases.yaml) 자세도 검사: 무작위 자세라 실물에서 불가능한(몸에 파고든) 자세가 섞였는지
#   - [5] --physx-poses: 무작위 자세를 기하 계산으로 분류(가능 / convex hull 때문에만 겹침 / 실제 겹침 / 판정 불가)한 뒤,
#         convex decomposition 을 끈 씬과 켠 씬에서 PhysX 가 실제 충돌 형상으로 보고하는 파고듦을 센다
#         (기하 계산은 convex hull 만 알기 때문에 decomposition 효과는 PhysX 접촉 보고로 확인)
#
# 실행:
#   ~/isaacsim/python.sh ~/data_collection_ur5_gripper/isaacsim/scripts/check_self_collision.py --headless
# =============================================================

import argparse
import datetime
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_scene as ts  # noqa: E402

parser = argparse.ArgumentParser(description="self-collision 사전 점검: 링크 쌍별 충돌 형상 거리")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--usd", default=ts.DEFAULT_USD)
parser.add_argument("--config", default=ts.DEFAULT_CONFIG)
parser.add_argument("--near-mm", type=float, default=2.0, help="이 거리 이내면 '가까움'")
parser.add_argument("--spacing-mm", type=float, default=1.0, help="표면 표본 간격")
parser.add_argument("--cases", default=os.path.join(ts.CONFIG_DIR, "gravity_worst_cases.yaml"))
parser.add_argument("--skip-distances", action="store_true", help="[1]~[4] 거리 계산을 건너뜀 (오래 걸림)")
parser.add_argument("--physx-poses", action="store_true", help="[5] 무작위 자세 PhysX 접촉 비교 (decomposition 끔/켬)")
parser.add_argument("--n-valid", type=int, default=302, help="[5] 가능한 자세를 이만큼 모을 때까지 뽑음 (compute_gain_seed 와 같게)")
parser.add_argument("--reuse-classes", default=None,
                    help="[5] 저장해 둔 자세 분류(pose_classes_*.npz)를 다시 씀 (기하 분류 약 3 분 생략). 'latest' 면 가장 최근 파일")
parser.add_argument("--cd-variant", action="append", default=[],
                    help="[5] decomposition 설정 변형 추가 비교, 예: 'max_convex_hulls=64,voxel_resolution=2000000' (여러 번 가능)")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import numpy as np  # noqa: E402
import trimesh  # noqa: E402
import yaml  # noqa: E402
from isaacsim.core.experimental.prims import RigidPrim  # noqa: E402
from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402
from scipy.spatial import ConvexHull, cKDTree  # noqa: E402

WRIST_GROUP = {"wrist_3_link", "wrist_mount", "gripper_bracket", "rh_p12_rn_base", "rh_p12_rn_r1", "rh_p12_rn_r2",
               "rh_p12_rn_l1", "rh_p12_rn_l2", "wrist_camera_link"}
HOME = [0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0]
GRIP_OPEN, GRIP_CLOSED = 0.0, 1.1351


def named_poses():
    h = np.array(HOME)
    poses = {
        "home": h,
        "zero(팔 수평)": np.zeros(6),
        "home wrist_2=0 (공구 옆)": h + [0, 0, 0, 0, math.pi / 2, 0],
        "home wrist_1=-π (공구 앞)": h + [0, 0, 0, -math.pi / 2, 0, 0],
        "home wrist_1=0 (공구 뒤, 팔 쪽)": h + [0, 0, 0, math.pi / 2, 0, 0],
        "home wrist_3=π/2": h + [0, 0, 0, 0, 0, math.pi / 2],
        "elbow 접음 (2.6 rad)": np.array([0, -math.pi / 2, 2.6, -math.pi / 2, -math.pi / 2, 0]),
    }
    return poses


def body_of(prim):
    p = prim
    while p and p.IsValid() and not p.HasAPI(UsdPhysics.RigidBodyAPI):
        p = p.GetParent()
    return p if p and p.IsValid() else None


def mat_np(m):
    return np.array([[m[i][j] for j in range(4)] for i in range(4)])


def shape_points(prim):
    """prim 로컬 좌표의 형상 꼭짓점 (볼록 껍질을 만들 점들)."""
    if prim.IsA(UsdGeom.Mesh):
        return np.array(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=float)
    if prim.IsA(UsdGeom.Cube):
        s = float(UsdGeom.Cube(prim).GetSizeAttr().Get()) / 2
        return np.array([[x, y, z] for x in (-s, s) for y in (-s, s) for z in (-s, s)])
    if prim.IsA(UsdGeom.Cylinder):
        c = UsdGeom.Cylinder(prim)
        r, h, ax = float(c.GetRadiusAttr().Get()), float(c.GetHeightAttr().Get()), c.GetAxisAttr().Get()
        a = np.linspace(0, 2 * math.pi, 64, endpoint=False)
        ring = np.stack([r * np.cos(a), r * np.sin(a)], axis=1)
        pts = []
        for z in (-h / 2, h / 2):
            for u, v in ring:
                pts.append({"X": (z, u, v), "Y": (u, z, v), "Z": (u, v, z)}[ax])
        return np.array(pts)
    raise TypeError(f"지원하지 않는 충돌 형상: {prim.GetPath()} ({prim.GetTypeName()})")


def collect_shapes(stage):
    """링크 이름 → 링크 좌표계의 볼록 껍질 목록. 링크 prim 경로도 돌려줌."""
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    shapes, link_paths = {}, {}
    root = stage.GetPrimAtPath(ts.ROBOT_PATH)
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if prim.GetAttribute("physics:collisionEnabled").Get() is False:
            continue
        body = body_of(prim)
        if body is None:
            continue
        rel = mat_np(cache.GetLocalToWorldTransform(prim) * cache.GetLocalToWorldTransform(body).GetInverse())
        pts = shape_points(prim)
        pts_link = (np.c_[pts, np.ones(len(pts))] @ rel)[:, :3]  # USD 행벡터 규약
        hull = trimesh.convex.convex_hull(pts_link)
        name = body.GetName()
        shapes.setdefault(name, []).append(hull)
        link_paths[name] = str(body.GetPath())
    return shapes, link_paths


def joint_pairs(stage):
    """관절로 직접 연결돼 USD 에서 충돌이 꺼진 쌍 (질량 없는 프레임은 가장 가까운 강체로)."""
    out = set()
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ts.ROBOT_PATH), Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdPhysics.Joint):
            continue
        if prim.GetAttribute("physics:collisionEnabled").Get():
            continue
        ends = []
        for r in ("physics:body0", "physics:body1"):
            t = prim.GetRelationship(r).GetTargets()
            b = body_of(stage.GetPrimAtPath(t[0])) if t else None
            ends.append(b.GetName() if b else None)
        if None not in ends:
            out.add(frozenset(ends))
    return out


def sample(hull, spacing):
    n = int(min(60000, max(200, hull.area / (spacing ** 2))))
    pts, _ = trimesh.sample.sample_surface_even(hull, n)
    return np.vstack([pts, hull.vertices])


def pair_distance(A, B):
    """A, B: (표본점 world, 껍질 방정식 world) 리스트. 최소 거리 [m], 겹치면 음수 (파고든 깊이)."""
    best = math.inf
    for pa, eqa in A:
        for pb, eqb in B:
            depth = 0.0
            for p, eq in ((pa, eqb), (pb, eqa)):
                s = p @ eq[:, :3].T + eq[:, 3]  # 면 방정식 값, 모두 음수면 안쪽
                inside = np.all(s < 0, axis=1)
                if inside.any():
                    depth = max(depth, float((-s[inside].max(axis=1)).max()))
            if depth > 0:
                best = min(best, -depth)
                continue
            d, _ = cKDTree(pb).query(pa, k=1)
            best = min(best, float(d.min()))
    return best


def physx_penetrations(cfg, poses, pen_mm=0.5):
    """cfg 로 씬을 만들고 각 자세로 순간이동한 뒤 첫 physics 스텝의 PhysX 접촉 보고에서 파고듦(separation < −pen_mm)이 있는
    링크 쌍을 모은다. 반환: 자세별 {(a, b): 최소 separation [m]}, 적용된 decomposition prim, 스텝당 계산 시간 [ms]."""
    from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
    from omni.physx import get_physx_simulation_interface
    from pxr import PhysicsSchemaTools, PhysxSchema

    stage, robot, info = ts.build_test_scene(args.usd, base="fixed", config=cfg, ground=False)
    for p in Usd.PrimRange(stage.GetPrimAtPath(ts.ROBOT_PATH)):
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr().Set(0.0)
    app_utils.play()
    simulation_app.update()
    problems, _ = ts.verify_articulation(stage, robot, info, cfg)
    if problems:
        raise RuntimeError(f"articulation 확인 실패: {problems}")
    idx = ts.dof_index_map(robot)
    sim = get_physx_simulation_interface()
    box = {"armed": False, "pairs": None}

    def post(dt, ctx):
        if box["armed"]:
            headers, data = sim.get_contact_report()[:2]
            pairs = {}
            for h in headers:
                seps = [data[h.contact_data_offset + k].separation for k in range(h.num_contact_data)]
                if seps and min(seps) < -pen_mm / 1000:
                    a = str(PhysicsSchemaTools.intToSdfPath(h.actor0)).split("/")[-1]
                    b = str(PhysicsSchemaTools.intToSdfPath(h.actor1)).split("/")[-1]
                    k = tuple(sorted((a, b)))
                    pairs[k] = min(pairs.get(k, 0.0), min(seps))
            box["pairs"], box["armed"] = pairs, False

    cb = SimulationManager.register_callback(post, event=SimulationEvent.PHYSICS_POST_STEP)
    out = []
    import time as _t

    wall, steps = 0.0, 0
    try:
        for q6 in poses:
            q = np.zeros(robot.num_dofs, dtype=np.float32)
            for n, v in zip(ts.ARM_JOINTS, q6):
                q[idx[n]] = v
            robot.set_dof_positions(q)
            robot.set_dof_velocities(np.zeros_like(q))
            robot.set_dof_position_targets(q)
            box["armed"], box["pairs"] = True, None
            while box["armed"]:
                w0 = _t.perf_counter()
                simulation_app.update()
                wall += _t.perf_counter() - w0
                steps += 1
            out.append(box["pairs"])
    finally:
        SimulationManager.deregister_callback(cb)
        app_utils.stop()
        simulation_app.update()
    return out, info.get("convex_decomposition", []), 1000 * wall / max(1, steps)


def _set_pose_fn(robot, idx):
    def set_pose(q6):
        q = np.zeros(robot.num_dofs, dtype=np.float32)
        for n, v in zip(ts.ARM_JOINTS, q6):
            q[idx[n]] = v
        for _ in range(2):
            robot.set_dof_positions(q)
            robot.set_dof_velocities(np.zeros_like(q))
            simulation_app.update()
        robot.set_dof_positions(q)
    return set_pose


def _geo_scene():
    geo_cfg = ts.load_config(args.config)
    geo_cfg["articulation"]["self_collision"] = False  # 기하 계산 때는 접촉으로 링크가 밀리지 않게
    stage, robot, info = ts.build_test_scene(args.usd, base="fixed", config=geo_cfg, ground=False)
    app_utils.play()
    simulation_app.update()
    return stage, robot, ts.dof_index_map(robot)


def physx_pose_section(log):
    """[5] 무작위 자세 분류 + PhysX 접촉 비교 (decomposition 끔/켬/변형) + 놓친 실제 겹침의 원래 메시 깊이."""
    import glob
    import time

    import collision_geom as cg

    if args.reuse_classes:
        path = (sorted(glob.glob(os.path.join(ts.REPORT_DIR, "pose_classes_*.npz")))[-1]
                if args.reuse_classes == "latest" else args.reuse_classes)
        z = np.load(path)
        groups = {g: [q for q in z[g]] for g in ("hull_only", "real", "valid")}
        log(f"[5] 자세 분류 다시 씀: {path} (가능 {len(groups['valid'])}, convex hull 때문에만 {len(groups['hull_only'])}, "
            f"실제 겹침 {len(groups['real'])})")
    else:
        stage, robot, idx = _geo_scene()
        rng = np.random.default_rng(0)  # compute_gain_seed.py 와 같은 시드·순서
        lim = np.array([2 * math.pi, 2 * math.pi, math.pi, 2 * math.pi, 2 * math.pi, 2 * math.pi])
        r = cg.classify_random_poses(stage, _set_pose_fn(robot, idx), rng, lim, args.n_valid, first=(HOME, np.zeros(6)))
        app_utils.stop()
        simulation_app.update()
        groups = {g: [q for q, _ in r[g]] for g in ("hull_only", "real", "valid")}
        path = os.path.join(ts.REPORT_DIR, f"pose_classes_{datetime.datetime.now():%Y%m%d_%H%M%S}.npz")
        os.makedirs(ts.REPORT_DIR, exist_ok=True)
        np.savez(path, **{g: np.array(v) for g, v in groups.items()}, unknown=np.array([q for q, _ in r["unknown"]]))
        log("")
        log(f"[5] 무작위 자세 {r['draws']} 개 기하 분류: 가능 {len(r['valid'])}, convex hull 때문에만 겹침 {len(r['hull_only'])}, "
            f"실제 겹침 {len(r['real'])}, 판정 불가 {len(r['unknown'])}  (저장: {path})")

    variants = [("decomposition 끔", {"enabled": False}), ("decomposition 켬", {"enabled": True})]
    for v in args.cd_variant:
        kv = {k: yaml.safe_load(x) for k, x in (item.split("=") for item in v.split(","))}
        variants.append((f"켬 {v}", {"enabled": True, **kv}))
    allq = [q for g in groups.values() for q in g]
    res = {}
    for label, over in variants:
        cfg = ts.load_config(args.config)
        cfg["articulation"]["self_collision"] = True
        for k, x in over.items():
            if k not in cfg["collision_shapes"]["convex_decomposition"]:
                raise KeyError(f"convex_decomposition 에 없는 항목: {k}")
            cfg["collision_shapes"]["convex_decomposition"][k] = x
        t0 = time.perf_counter()
        pens, applied, ms = physx_penetrations(cfg, allq)
        res[label] = dict(pens=pens, applied=applied, sec=time.perf_counter() - t0, ms=ms,
                          cd=dict(cfg["collision_shapes"]["convex_decomposition"]))

    log(f"  PhysX 접촉 보고로 파고듦(separation < −0.5 mm, 순간이동 직후 첫 스텝)이 있는 자세 수")
    log(f"  {'분류':12s} {'자세 수':>6s} " + " ".join(f"{k:>24s}" for k in res))
    split = {}
    k0 = 0
    for g, items in groups.items():
        n = len(items)
        split[g] = (k0, k0 + n)
        log(f"  {g:12s} {n:6d} " + " ".join(f"{sum(bool(p) for p in res[l]['pens'][k0:k0 + n]):>24d}" for l in res))
        k0 += n
    a, b = split["real"]
    off = res["decomposition 끔"]["pens"]
    log(f"  {'놓침(실제)':12s} {'':6s} " + " ".join(
        f"{sum(1 for i in range(a, b) if off[i] and not res[l]['pens'][i]):>24d}" for l in res))
    log(f"  {'스텝당 ms':12s} {'':6s} " + " ".join(f"{res[l]['ms']:>24.2f}" for l in res))
    for label, rr in res.items():
        log(f"  {label}: 설정 {rr['cd']}, 적용 prim {len(rr['applied'])} 개, 소요 {rr['sec']:.1f} s")

    # 놓친 실제 겹침: 끔에서는 보고되던 쌍이 켬(각 변형)에서 사라진 자세 → 원래 메시 기준 깊이
    stage, robot, idx = _geo_scene()
    shapes, link_paths = cg.collect(stage)
    names = sorted(shapes)
    from isaacsim.core.experimental.prims import RigidPrim

    view = RigidPrim([link_paths[n] for n in names])
    set_pose = _set_pose_fn(robot, idx)
    for label in [l for l in res if l != "decomposition 끔"]:
        on = res[label]["pens"]
        lost = [(i, pr) for i in range(a, b) if off[i] and not on[i] for pr in off[i]]
        log("")
        log(f"  [{label}] 놓친 실제 겹침 자세 {len(set(i for i, _ in lost))} 개: 링크 쌍 / PhysX(끔) separation / 원래 메시 깊이")
        for i, (pa, pb) in lost:
            set_pose(allq[i])
            T = cg.link_transforms(view, names)
            ds = [cg.raw_depth(sa, T[pa], sb, T[pb]) for sa in shapes[pa] for sb in shapes[pb]]
            d = None if any(x is None for x in ds) else max(ds)
            log(f"    자세 {i - a:3d} {np.round(allq[i], 2).tolist()}  {pa}-{pb}  {off[i][(pa, pb)] * 1000:+.2f} mm  "
                f"{'판정 불가' if d is None else f'{d * 1000:.2f} mm'}")
    app_utils.stop()
    simulation_app.update()


def main():
    L = []

    def log(m=""):
        print(m, flush=True)
        L.append(m)

    if args.physx_poses and args.skip_distances:
        physx_pose_section(log)
        _write(L)
        return
    near = args.near_mm / 1000
    spacing = args.spacing_mm / 1000
    cfg = ts.load_config(args.config)
    stage, robot, info = ts.build_test_scene(args.usd, base="fixed", config=cfg, ground=False, light=not args.headless)
    shapes, link_paths = collect_shapes(stage)
    jp = joint_pairs(stage)
    names = sorted(shapes)
    log(f"check_self_collision  {datetime.datetime.now().isoformat(timespec='seconds')}  가까움 < {args.near_mm} mm, 표본 간격 {args.spacing_mm} mm")
    log(f"충돌 형상이 있는 링크 ({len(names)}): " + ", ".join(f"{n}×{len(shapes[n])}" for n in names))
    log(f"관절로 직접 연결(USD 에서 충돌 꺼짐): " + ", ".join(sorted("-".join(sorted(p)) for p in jp)))

    # 링크 좌표계 표본·방정식은 한 번만
    local = {n: [(sample(h, spacing), h) for h in shapes[n]] for n in names}

    app_utils.play()
    simulation_app.update()
    idx = ts.dof_index_map(robot)
    links = RigidPrim([link_paths[n] for n in names])

    def world_geom():
        pos, quat = (x.numpy() for x in links.get_world_poses())
        out = {}
        for k, n in enumerate(names):
            w, x, y, z = quat[k]
            R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                          [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                          [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
            T = np.eye(4)
            T[:3, :3], T[:3, 3] = R, pos[k]
            geo = []
            for pts, hull in local[n]:
                hw = hull.copy()
                hw.apply_transform(T)
                eq = ConvexHull(hw.vertices).equations
                geo.append((pts @ R.T + pos[k], eq))
            out[n] = geo
        return out

    def set_pose(q6, grip):
        q = np.zeros(robot.num_dofs, dtype=np.float32)
        for n, v in zip(ts.ARM_JOINTS, q6):
            q[idx[n]] = v
        for n in ts.GRIPPER_JOINTS:
            q[idx[n]] = grip
        for _ in range(2):
            robot.set_dof_positions(q)
            robot.set_dof_velocities(np.zeros_like(q))
            robot.set_dof_position_targets(q)
            simulation_app.update()

    def scan(q6, grip):
        set_pose(q6, grip)
        g = world_geom()
        res = {}
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                res[(a, b)] = pair_distance(g[a], g[b])
        return res

    poses = named_poses()
    all_res = {}
    for pname, q in poses.items():
        for gname, gv in (("열림", GRIP_OPEN), ("닫힘", GRIP_CLOSED)):
            all_res[f"{pname} / 그리퍼 {gname}"] = scan(q, gv)

    def fmt_mm(d):
        return f"{d * 1000:+7.2f}"

    log("")
    log(f"[1] 자세별 가까운 쌍 (< {args.near_mm} mm, 음수 = 겹침 깊이). J = 관절로 직접 연결(이미 충돌 꺼짐), W = 손목 쪽 쌍")
    for key, res in all_res.items():
        close = sorted([(d, p) for p, d in res.items() if d < near])
        log(f"  {key}: {len(close)} 쌍")
        for d, (a, b) in close:
            tag = ("J" if frozenset((a, b)) in jp else " ") + ("W" if a in WRIST_GROUP and b in WRIST_GROUP else " ")
            log(f"    {tag} {a:20s} {b:20s} {fmt_mm(d)} mm")

    log("")
    log("[2] 손목 쪽 쌍 전체 거리 (mm, 자세별 최소·최대)")
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if a in WRIST_GROUP and b in WRIST_GROUP:
                ds = [res[(a, b)] for res in all_res.values()]
                tag = "J" if frozenset((a, b)) in jp else " "
                log(f"  {tag} {a:20s} {b:20s} 최소 {fmt_mm(min(ds))}  최대 {fmt_mm(max(ds))}")

    always = [(a, b) for (a, b) in all_res[next(iter(all_res))]
              if all(res[(a, b)] < near for res in all_res.values()) and frozenset((a, b)) not in jp]
    sometimes = sorted({p for res in all_res.values() for p, d in res.items() if d < near and frozenset(p) not in jp}
                       - set(always))
    log("")
    log(f"[3] 충돌 제외 쌍 후보 (관절 연결 아님, 모든 자세에서 < {args.near_mm} mm): {len(always)}")
    for a, b in always:
        log(f"    - [{a}, {b}]")
    log(f"    일부 자세에서만 가까운 쌍 (제외하지 않음, 켜면 실제로 닿을 수 있음): {len(sometimes)}")
    for a, b in sometimes:
        which = [k for k, res in all_res.items() if res[(a, b)] < near]
        log(f"    · {a} - {b}: {', '.join(which)}")

    # B 시험 자세
    if os.path.isfile(args.cases):
        cases = yaml.safe_load(open(args.cases, encoding="utf-8"))
        log("")
        log(f"[4] B 시험 자세 (gravity_worst_cases.yaml): 관절 연결이 아닌 쌍이 겹치면 실물에서 불가능한 자세")
        seen = {}
        for c in cases["cases"]:
            key = tuple(round(x, 4) for x in c["pose"])
            seen.setdefault(key, []).append(f"{c['joint']}@{c['offset_cm']}cm")
        for key, who in seen.items():
            res = scan(np.array(key), GRIP_OPEN)
            bad = sorted([(d, p) for p, d in res.items() if d < 0 and frozenset(p) not in jp])
            log(f"  {', '.join(who)}: " + ("겹침 없음" if not bad else
                                           "겹침 " + ", ".join(f"{a}-{b} {fmt_mm(d)} mm" for d, (a, b) in bad)))

    app_utils.stop()
    simulation_app.update()
    if args.physx_poses:
        physx_pose_section(log)
    _write(L)


def _write(L):
    os.makedirs(ts.REPORT_DIR, exist_ok=True)
    path = os.path.join(ts.REPORT_DIR, f"check_self_collision_{datetime.datetime.now():%Y%m%d_%H%M%S}.txt")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")
    print(f"report: {path}", flush=True)


if __name__ == "__main__":
    code = 0
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        code = 1
    finally:
        simulation_app.close(exit_code=code)
    sys.exit(code)
