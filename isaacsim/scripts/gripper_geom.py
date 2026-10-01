# =============================================================
# gripper_geom.py — RH-P12-RN(A) 손가락 기구학·파지 기하 (docs/PLAN.md 1-6, 2-4)
#
#   URDF(rh_p12_rn_a.urdf.xacro) 값으로 손가락 링크 위치를 계산 (그리퍼 base 좌표계, +z = 손가락 끝 방향, y = 닫는 방향).
#   1-6 grasp_tests.py 에서 옮김 (식은 그대로). 2-4 드론 파지 높이 계산도 같이 쓴다.
#   **이 그리퍼 전용** (로봇·그리퍼를 바꾸면 새로 만들어야 함, config/robot_*.yaml 주석 참고)
#
# SimulationApp 을 만든 뒤에 import 할 것 (collision_geom 이 pxr 를 씀).
# =============================================================

import math

import numpy as np

import collision_geom as cg

GRIP_CLOSED = 1.1351
CLEAR = 0.002                      # m, 물체와 손가락·그리퍼 몸체 사이 여유
PAD_TOL = 0.0001                   # m, 파지면 = 안쪽 끝에서 이 거리 안의 면
TCP_Z = 0.11                       # m, rh_p12_rn_tcp 의 그리퍼 base 좌표계 z (URDF, 축 방향은 base 와 같음)


def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


# 손가락 기구학 (URDF rh_p12_rn_a.urdf.xacro 값). 그리퍼 base 좌표계에서 링크 (R, t). r2·l2 는 base 와 평행하게 움직인다
R1_ORIGIN, L1_ORIGIN = np.array([0, 0.008, 0.048]), np.array([0, -0.008, 0.048])
R2_IN_R1, L2_IN_L1 = np.array([0, 0.0493634, 0.0285]), np.array([0, -0.0493634, 0.0285])


def finger_T(q):
    return {"rh_p12_rn_r1": (Rx(q), R1_ORIGIN),
            "rh_p12_rn_r2": (np.eye(3), R1_ORIGIN + Rx(q) @ R2_IN_R1),
            "rh_p12_rn_l1": (Rx(-q), L1_ORIGIN),
            "rh_p12_rn_l2": (np.eye(3), L1_ORIGIN + Rx(-q) @ L2_IN_L1)}


def verts(shapes, link, T=None, which="raw"):
    v = np.vstack([getattr(s, which).vertices for s in shapes[link]])
    if T is None:
        return v
    R, t = T
    return v @ R.T + t


def pad_patch(mesh, d, tol=PAD_TOL):
    """mesh(링크 좌표계)에서 방향 d 로 가장 바깥 면(끝에서 tol 이내, 면 법선이 d 와 10° 이내). 넓이·범위·기울기."""
    s = float((mesh.vertices @ d).max())
    near = mesh.vertices @ d >= s - tol
    f = np.all(near[mesh.faces], axis=1) & (mesh.face_normals @ d >= math.cos(math.radians(10)))
    if not f.any():
        raise RuntimeError("파지면을 찾지 못함 (안쪽 끝에 평평한 면이 없음)")
    area = float(mesh.area_faces[f].sum())
    pts = mesh.vertices[np.unique(mesh.faces[f])]
    n = (mesh.face_normals[f] * mesh.area_faces[f, None]).sum(axis=0)
    ang = math.degrees(math.acos(np.clip(n @ d / max(np.linalg.norm(n), 1e-12), -1, 1))) if area > 0 else float("nan")
    return dict(support=s, area=area, x=(float(pts[:, 0].min()), float(pts[:, 0].max())),
                z=(float(pts[:, 2].min()), float(pts[:, 2].max())), tilt_deg=ang)


def pad_depths(shape, d, band=0.002, step=0.0005):
    """파지면 쪽 넓은 범위(convex hull 에서 안쪽 끝 band 이내 면의 x·z 범위)에 격자를 두고 d 반대 방향 광선으로
    안쪽 끝 평면에서 원래 메시·convex hull 표면까지 깊이 [m] 를 잰다. 반환 (x, z 격자, 원래 깊이, hull 깊이)."""
    s = float((shape.hull.vertices @ d).max())
    near = shape.hull.vertices @ d >= s - band
    f = np.all(near[shape.hull.faces], axis=1)
    pts = shape.hull.vertices[np.unique(shape.hull.faces[f])]
    xs = np.arange(pts[:, 0].min() + step / 2, pts[:, 0].max(), step)
    zs = np.arange(pts[:, 2].min() + step / 2, pts[:, 2].max(), step)
    X, Z = np.meshgrid(xs, zs)
    o = np.zeros((X.size, 3))
    o[:, 0], o[:, 2] = X.ravel(), Z.ravel()
    o += d * (s + 0.001) - d * (d @ o.T)[:, None]   # 안쪽 끝 평면보다 1 mm 바깥에서 출발
    dirs = np.tile(-d, (len(o), 1))
    out = []
    for mesh in (shape.raw, shape.hull):
        loc, idx, _ = mesh.ray.intersects_location(o, dirs, multiple_hits=False)
        depth = np.full(len(o), np.nan)
        depth[idx] = np.linalg.norm(loc - o[idx], axis=1) - 0.001
        out.append(depth.reshape(X.shape))
    return X, Z, out[0], out[1]


def inner_gap(shapes, q):
    """손가락 각도 q 에서 오른쪽 손가락 안쪽 끝 y, 왼쪽 손가락 안쪽 끝 y (그리퍼 base 좌표계)."""
    T = finger_T(q)
    return (float(verts(shapes, "rh_p12_rn_r2", T["rh_p12_rn_r2"])[:, 1].min()),
            float(verts(shapes, "rh_p12_rn_l2", T["rh_p12_rn_l2"])[:, 1].max()))


def grasp_geometry(shapes, dims):
    """박스(dims)를 잡을 때: 닿는 각도 q_c, 파지면 높이, 박스 윗면 깊이 z_top (그리퍼 base 좌표계, +z = 손가락 끝 방향)."""
    lx, ly, h = dims
    gap = lambda q: inner_gap(shapes, q)[0] - inner_gap(shapes, q)[1]  # noqa: E731
    g_open, g_closed = gap(0.0), gap(GRIP_CLOSED)
    if not (g_closed < ly < g_open):
        raise ValueError(f"박스 폭 {ly * 1000:.1f} mm 가 손가락 사이 범위({g_closed * 1000:.1f}~{g_open * 1000:.1f} mm) 밖")
    lo, hi = 0.0, GRIP_CLOSED
    for _ in range(50):  # gap 은 q 에 대해 감소
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if gap(mid) > ly else (lo, mid)
    qc = 0.5 * (lo + hi)
    T = finger_T(qc)
    # 파지면 높이: 오른쪽 손가락 원래 메시에서 안쪽 끝 면
    raw_r2 = cg.trimesh.util.concatenate([s.raw for s in shapes["rh_p12_rn_r2"]])
    raw_r2.apply_translation(T["rh_p12_rn_r2"][1])
    pad = pad_patch(raw_r2, np.array([0, -1.0, 0]))
    # 박스 윗면 위(|x| < lx/2, |y| < ly/2)로 들어오는 그리퍼 몸체·r1·l1 의 가장 깊은 점 (닫히는 동안 전부)
    zb = -1.0
    for q in np.linspace(0.0, qc, 25):
        Tq = finger_T(q)
        for link, Tl in (("rh_p12_rn_base", (np.eye(3), np.zeros(3))), ("rh_p12_rn_r1", Tq["rh_p12_rn_r1"]),
                         ("rh_p12_rn_l1", Tq["rh_p12_rn_l1"])):
            v = verts(shapes, link, Tl)
            m = (np.abs(v[:, 0]) < lx / 2 + CLEAR) & (np.abs(v[:, 1]) < ly / 2 + CLEAR)
            if m.any():
                zb = max(zb, float(v[m, 2].max()))
    z_top = zb + CLEAR
    engaged = (pad["z"][1] - max(z_top, pad["z"][0])) / (pad["z"][1] - pad["z"][0])
    if engaged < 0.5:
        raise ValueError(f"박스 윗면({z_top * 1000:.1f} mm)이 파지면({pad['z'][0] * 1000:.1f}~{pad['z'][1] * 1000:.1f} mm)을 "
                         f"절반도 덮지 못함 (손가락·몸체가 먼저 닿음)")
    if z_top + h < pad["z"][1]:
        raise ValueError(f"박스 높이 {h * 1000:.0f} mm 가 파지면 끝까지 닿지 않음")
    ri, li = inner_gap(shapes, qc)
    return dict(qc=qc, pad_z=pad["z"], z_top=z_top, engaged=engaged, grip_z=0.5 * (max(z_top, pad["z"][0]) + pad["z"][1]),
                inner=(ri, li), g_open=g_open, g_closed=g_closed)


def grip_lever(q, h=1e-4):
    """rh_r1 을 1 rad 닫을 때 파지면이 안쪽으로 움직이는 거리 [m] (가상일: 수직력 합 × 이 값 = 전달된 관절 토크)."""
    return float(finger_T(q - h)["rh_p12_rn_r2"][1][1] - finger_T(q + h)["rh_p12_rn_r2"][1][1]) / (2 * h)


def plan_grasp_from_below(shapes, pts, half_width=0.013, cell=0.002, clear=CLEAR, h_step=0.0005, min_engaged=0.5):
    """그리퍼가 아래에서 위로(+z) 다가가 물체 옆면을 y 방향으로 잡을 때의 TCP 높이 (PLAN 2-4).

    pts: 물체 표면 점 (N, 3), **그리퍼 정렬 물체 좌표계** (x = 그리퍼 x, y = 닫는 방향, z = 접근 방향 = 그리퍼 +z,
         원점 = 손가락 사이 가운데가 지나갈 축 위). shapes: collision_geom.collect 의 링크별 충돌 형상.
    높이 h (TCP 의 물체 좌표계 z) 마다:
      닿는 각도 q_c = 손가락 안쪽 면 사이가 파지면 높이 띠·손가락 폭 slab(|x| ≤ half_width) 안 물체 폭과 같아지는 각도
      여유 = 0 ~ q_c 로 닫는 동안 그리퍼 base·r1·l1 꼭짓점이 그 (x, y) 칸의 물체 가장 낮은 점보다 아래인 거리 (최솟값)
    여유 ≥ clear 이고 파지면이 물체에 min_engaged 이상 걸리는 가장 높은 h 를 고른다. dict 를 돌려준다 (못 찾으면 에러)."""
    pts = np.asarray(pts, dtype=float)
    slab = pts[np.abs(pts[:, 0]) <= half_width]
    if len(slab) == 0:
        raise ValueError("손가락 폭 slab 안에 물체 점이 없음")
    # 물체 아랫면 높이 지도 (칸별 최저 z)
    ij = np.floor(pts[:, :2] / cell).astype(int)
    bottom = {}
    for (i, j), z in zip(map(tuple, ij), pts[:, 2]):
        if z < bottom.get((i, j), np.inf):
            bottom[(i, j)] = z
    raw_r2 = cg.trimesh.util.concatenate([s.raw for s in shapes["rh_p12_rn_r2"]])
    pad = pad_patch(raw_r2, np.array([0, -1.0, 0]))             # r2 링크 좌표계 파지면 (z 범위)
    q_tab = np.linspace(0.0, GRIP_CLOSED, 1201)                 # 손가락 사이 간격 표 (q 에 대해 감소)
    gap_tab = np.array([np.subtract(*inner_gap(shapes, q)) for q in q_tab])
    if np.any(np.diff(gap_tab) > 0):
        raise RuntimeError("손가락 사이 간격이 q 에 대해 감소하지 않음 (기구학 확인)")
    g_open, g_closed = float(gap_tab[0]), float(gap_tab[-1])

    def contact(h):
        base_z0 = h - TCP_Z                                      # 물체 좌표계에서 그리퍼 base 원점 z
        q = 0.0
        for _ in range(4):                                       # 파지면 높이가 q 에 따라 바뀌므로 몇 번 반복
            pz = finger_T(q)["rh_p12_rn_r2"][1][2] + np.array(pad["z"]) + base_z0
            band = slab[(slab[:, 2] >= pz[0]) & (slab[:, 2] <= pz[1])]
            if len(band) == 0:
                return None
            width = float(band[:, 1].max() - band[:, 1].min())
            if not g_closed < width < g_open:
                return None
            q = float(np.interp(width, gap_tab[::-1], q_tab[::-1]))
        side = slab[(slab[:, 1] > band[:, 1].max() - 0.003) | (slab[:, 1] < band[:, 1].min() + 0.003)]
        zr = (side[:, 2].min(), side[:, 2].max())               # 옆면이 있는 높이 범위
        engaged = (min(pz[1], zr[1]) - max(pz[0], zr[0])) / (pz[1] - pz[0])
        return dict(q=q, width=width, pad_z=pz.tolist(), center_y=float(band[:, 1].max() + band[:, 1].min()) / 2,
                    engaged=float(engaged))

    def clearance(h, qc):
        base_z0 = h - TCP_Z
        worst = np.inf
        for q in np.linspace(0.0, qc, 12):
            T = finger_T(q)
            for link, Tl in (("rh_p12_rn_base", (np.eye(3), np.zeros(3))), ("rh_p12_rn_r1", T["rh_p12_rn_r1"]),
                             ("rh_p12_rn_l1", T["rh_p12_rn_l1"])):
                v = verts(shapes, link, Tl, which="hull") + np.array([0.0, 0.0, base_z0])
                for (i, j), z in zip(map(tuple, np.floor(v[:, :2] / cell).astype(int)), v[:, 2]):
                    b = bottom.get((i, j))
                    if b is not None:
                        worst = min(worst, b - z)
        return worst

    z_hi = float(slab[:, 2].max())
    for h in np.arange(z_hi + TCP_Z, float(pts[:, 2].min()) - 0.05, -h_step):
        c = contact(h)
        if c is None or c["engaged"] < min_engaged:
            continue
        cl = clearance(h, c["q"])
        if cl >= clear:
            c.update(h=float(h), clearance=float(cl), g_open=g_open, g_closed=g_closed)
            return c
    raise ValueError("아래에서 잡을 높이를 찾지 못함 (그리퍼 몸체가 물체 아랫면에 닿거나 폭이 손가락 범위 밖)")
