# =============================================================
# collision_geom.py — 로봇 충돌 형상 기하 계산 (자기 충돌 점검·자세 거르기 공용, docs/PLAN.md 1-6)
#
#   - 충돌 형상: USD CollisionAPI prim. 메시는 PhysX 와 같게 convex hull, 박스·원기둥은 그 형상
#   - 링크 좌표계로 한 번 계산해 두고, 자세마다 링크 world pose 만 곱한다
#   - hull_overlap(): 빠른 겹침 판정 (껍질 꼭짓점 + 성긴 표면 표본이 상대 껍질 안에 있는지)
#   - raw_overlap(): 원래 메시(볼록 근사 전)로 겹침 판정 → "충돌 형상(convex hull) 때문에만 겹치는" 경우 구분
#
# SimulationApp 을 만든 뒤에 import 할 것.
# =============================================================

import math

import numpy as np
import trimesh
from pxr import Usd, UsdGeom, UsdPhysics
from scipy.spatial import ConvexHull

import test_scene as ts


def body_of(prim):
    p = prim
    while p and p.IsValid() and not p.HasAPI(UsdPhysics.RigidBodyAPI):
        p = p.GetParent()
    return p if p and p.IsValid() else None


def mat_np(m):
    return np.array([[m[i][j] for j in range(4)] for i in range(4)])


def quat_to_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _shape(prim):
    """(꼭짓점, 면 또는 None, 메시인지). prim 로컬 좌표."""
    if prim.IsA(UsdGeom.Mesh):
        m = UsdGeom.Mesh(prim)
        pts = np.array(m.GetPointsAttr().Get(), dtype=float)
        counts = np.array(m.GetFaceVertexCountsAttr().Get())
        idx = np.array(m.GetFaceVertexIndicesAttr().Get())
        faces = []
        k = 0
        for c in counts:  # 다각형 → 부채꼴 삼각형
            for j in range(1, c - 1):
                faces.append([idx[k], idx[k + j], idx[k + j + 1]])
            k += c
        return pts, np.array(faces), True
    if prim.IsA(UsdGeom.Cube):
        s = float(UsdGeom.Cube(prim).GetSizeAttr().Get()) / 2
        return np.array([[x, y, z] for x in (-s, s) for y in (-s, s) for z in (-s, s)]), None, False
    if prim.IsA(UsdGeom.Cylinder):
        c = UsdGeom.Cylinder(prim)
        r, h, ax = float(c.GetRadiusAttr().Get()), float(c.GetHeightAttr().Get()), c.GetAxisAttr().Get()
        a = np.linspace(0, 2 * math.pi, 64, endpoint=False)
        pts = []
        for z in (-h / 2, h / 2):
            for u, v in zip(r * np.cos(a), r * np.sin(a)):
                pts.append({"X": (z, u, v), "Y": (u, z, v), "Z": (u, v, z)}[ax])
        return np.array(pts), None, False
    raise TypeError(f"지원하지 않는 충돌 형상: {prim.GetPath()} ({prim.GetTypeName()})")


class Shape:
    def __init__(self, hull, raw, is_mesh, fast_spacing):
        self.hull = hull            # trimesh, 링크 좌표계 convex hull
        self.raw = raw              # trimesh, 링크 좌표계 원래 메시 (프리미티브는 hull 과 같음)
        self.is_mesh = is_mesh
        n = int(min(4000, max(100, hull.area / fast_spacing ** 2)))
        pts, _ = trimesh.sample.sample_surface_even(hull, n)
        self.fast_pts = np.vstack([pts, hull.vertices])
        self.eq = ConvexHull(hull.vertices).equations  # 링크 좌표계 면 방정식 n·x + d (안쪽이 음수)


def collect(stage, fast_spacing=0.003):
    """링크 이름 → [Shape], 링크 이름 → 링크 prim 경로."""
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    shapes, link_paths = {}, {}
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ts.ROBOT_PATH), Usd.TraverseInstanceProxies()):
        if not prim.HasAPI(UsdPhysics.CollisionAPI) or prim.GetAttribute("physics:collisionEnabled").Get() is False:
            continue
        body = body_of(prim)
        if body is None:
            continue
        rel = mat_np(cache.GetLocalToWorldTransform(prim) * cache.GetLocalToWorldTransform(body).GetInverse())
        pts, faces, is_mesh = _shape(prim)
        pl = (np.c_[pts, np.ones(len(pts))] @ rel)[:, :3]  # USD 행벡터 규약
        hull = trimesh.convex.convex_hull(pl)
        raw = trimesh.Trimesh(pl, faces, process=True) if is_mesh else hull
        shapes.setdefault(body.GetName(), []).append(Shape(hull, raw, is_mesh, fast_spacing))
        link_paths[body.GetName()] = str(body.GetPath())
    return shapes, link_paths


def joint_pairs(stage):
    """관절로 직접 연결돼 USD 에서 충돌이 꺼진 쌍."""
    out = set()
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ts.ROBOT_PATH), Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdPhysics.Joint) or prim.GetAttribute("physics:collisionEnabled").Get():
            continue
        ends = []
        for r in ("physics:body0", "physics:body1"):
            t = prim.GetRelationship(r).GetTargets()
            b = body_of(stage.GetPrimAtPath(t[0])) if t else None
            ends.append(b.GetName() if b else None)
        if None not in ends:
            out.add(frozenset(ends))
    return out


def link_transforms(link_prim_view, names):
    """RigidPrim(링크 경로 목록) → 이름별 (R, t)."""
    pos, quat = (x.numpy() for x in link_prim_view.get_world_poses())
    return {n: (quat_to_R(quat[k]), pos[k]) for k, n in enumerate(names)}


def _eq_world(shape, R, t):
    n = shape.eq[:, :3] @ R.T
    return np.c_[n, shape.eq[:, 3] - n @ t]


def hull_overlap(sa, Ta, sb, Tb):
    """두 형상의 convex hull 이 겹치면 파고든 깊이 추정 [m] (>0), 아니면 0. 빠른 판정."""
    Ra, ta = Ta
    Rb, tb = Tb
    depth = 0.0
    for p, s, T in ((sa.fast_pts @ Ra.T + ta, sb, Tb), (sb.fast_pts @ Rb.T + tb, sa, Ta)):
        eq = _eq_world(s, *T)
        v = p @ eq[:, :3].T + eq[:, 3]
        inside = np.all(v < 0, axis=1)
        if inside.any():
            depth = max(depth, float((-v[inside].max(axis=1)).max()))
    return depth


def raw_overlap(sa, Ta, sb, Tb, spacing=0.002):
    """원래 메시 기준 겹침: True / False / None(판정 불가: 메시가 닫혀 있지 않음).
    한쪽이라도 프리미티브면 hull 이 곧 형상이므로 hull 판정과 같게 본다."""
    if not (sa.is_mesh and sb.is_mesh):
        return hull_overlap(sa, Ta, sb, Tb) > 0
    if not (sa.raw.is_watertight and sb.raw.is_watertight):
        return None
    for s1, T1, s2, T2 in ((sa, Ta, sb, Tb), (sb, Tb, sa, Ta)):
        n = int(min(20000, max(300, s1.raw.area / spacing ** 2)))
        p, _ = trimesh.sample.sample_surface_even(s1.raw, n)
        R1, t1 = T1
        R2, t2 = T2
        pw = p @ R1.T + t1
        local2 = (pw - t2) @ R2  # s2 링크 좌표계로
        if s2.raw.contains(local2).any():
            return True
    return False


def self_collisions(shapes, T, skip_pairs, depth_min=0.0):
    """관절 연결이 아닌 링크 쌍 중 hull 이 겹치는 것: [(a, b, 깊이)]."""
    names = sorted(shapes)
    out = []
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if frozenset((a, b)) in skip_pairs:
                continue
            d = max(hull_overlap(sa, T[a], sb, T[b]) for sa in shapes[a] for sb in shapes[b])
            if d > depth_min:
                out.append((a, b, d))
    return out


def classify_random_poses(stage, set_pose, rng, lim, n_valid, first=(), max_draw=None):
    """무작위 자세(앞에 first 자세들)를 뽑아 자기 충돌로 분류한다. set_pose(q6) 는 자세를 맞추는 함수.
    반환 dict: valid, hull_only, real, unknown (각각 [(q, [(a, b, 깊이)])]), draws, pair_count.
      hull_only = 충돌 형상(convex hull)으로는 겹치지만 원래 메시로는 겹치지 않음
      real      = 원래 메시로도 겹침,  unknown = 원래 메시가 닫혀 있지 않아 판정 불가"""
    from isaacsim.core.experimental.prims import RigidPrim

    shapes, link_paths = collect(stage)
    names = sorted(shapes)
    skip = joint_pairs(stage)
    view = RigidPrim([link_paths[n] for n in names])
    out = {"valid": [], "hull_only": [], "real": [], "unknown": [], "pair_count": {}, "draws": 0,
           "watertight": {n: [s.raw.is_watertight for s in shapes[n] if s.is_mesh] for n in names}}
    cands = [np.asarray(q, dtype=float) for q in first]
    max_draw = max_draw or 20 * n_valid
    while len(out["valid"]) < n_valid and out["draws"] < max_draw:
        q = cands.pop(0) if cands else rng.uniform(-lim, lim)
        out["draws"] += 1
        set_pose(q)
        T = link_transforms(view, names)
        coll = self_collisions(shapes, T, skip)
        if not coll:
            out["valid"].append((q, []))
            continue
        verdicts = []
        for a, b, d in coll:
            out["pair_count"][(a, b)] = out["pair_count"].get((a, b), 0) + 1
            v = [raw_overlap(sa, T[a], sb, T[b]) for sa in shapes[a] for sb in shapes[b]]
            verdicts.append(True if any(x is True for x in v) else (None if any(x is None for x in v) else False))
        key = ("hull_only" if all(v is False for v in verdicts)
               else "real" if any(v is True for v in verdicts) else "unknown")
        out[key].append((q, coll))
    return out
