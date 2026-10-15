#!/usr/bin/env python3
# =============================================================
# wrist_mount_cad.py — UR5 플랜지 ↔ RH-P12-RN-UR 브래킷 사이에 끼우는 손목 카메라 (D435i) 마운트 CAD (CadQuery)
#
#   좌표 = URDF wrist_mount 좌표계 [mm]: 원점 = UR 플랜지 면 중심, +z = 그리퍼 쪽 (공구 축), +y = 카메라 탭 쪽, x = 옆.
#   이 좌표 그대로 STL 을 내므로 URDF 에서 회전 없이 쓴다.
#
#   플랜지·브래킷과 맞물리는 부분은 원래 마운트 ("Realsense Support UniversalRobots", Niko Bonomi 2022, meshes/mount/wrist_mount.stl)
#   에서 잰 값 그대로 (2026-10-15 STL 단면으로 확인):
#     판 두께 5.0 (z 0 ~ 5), 플랜지 쪽 테두리 높이 6.45 (z −6.45 ~ 0, 안지름 63.5 = UR 플랜지 Ø63 을 감쌈),
#     ISO 9409-1-50-4-M6: Ø6.5 × 4 (피치원 Ø50, 45° 방향), 핀 구멍 Ø6.5 (피치원 위, 탭 반대쪽 −y), 가운데 Ø34
#   카메라 탭만 바꾼다 (docs/PLAN.md "손목 카메라 마운트 후보", compare_wrist_camera.py):
#     카메라 면 기준점 = (CAM_S, 57.5, 5), 그 점을 지나는 x 축으로 tilt 만큼 공구 축 쪽으로 기울임 (xacro wrist_camera_mount + cam_tilt 와 같은 정의)
#     D435i 뒷면 M3 구멍 2 개 (간격 45) 가 그 면에서 x' = ±22.5. 몸체 (90 × 25 × 25) 는 면에서 그리퍼 쪽으로, 위쪽 (y') 이 바깥
#     M3 구멍 (원래 마운트와 같게, 2026-10-15 STL 단면으로 잼): 카메라 면에서 6.0 mm 까지 Ø3.4 (나사 자리), 그 뒤 Ø5.8 카운터보어,
#       로봇 쪽 입구에 1 mm × 45° 모따기 (Ø5.8 → Ø7.8). 구멍 축은 카메라 면에 수직 (기울인 버전도 나사 자리 6.0 mm 가 같다)
#     권장 나사 M3 × 8 → 카메라에 2 mm 들어감. D435i 뒷면 M3 최대 삽입 깊이 3 mm, 권장 토크 0.4 Nm
#     (RealSense D400 Series Datasheet Mar 2026, Figure 10-9. RGB 렌즈 = 삼각대 구멍 중심선에서 17.5 + 15 = 32.5 mm, Figure 4-7 / Table 4-20)
#
# 실행 (레포 밖 venv, CadQuery):  ~/cad/venv/bin/python isaacsim/ur5_rh_p12_description/cad/wrist_mount_cad.py [--tilts 0 7 10]
# 결과: cad/out/wrist_mount_t<기울기>.step, .stl, preview_t<기울기>.png
# =============================================================

import argparse
import math
import os

import cadquery as cq
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")

# 맞물림 (원래 마운트에서 잰 값)
PLATE_T = 5.0          # 판 두께
RIM_H = 6.45           # 플랜지 쪽 테두리 높이
POCKET_D = 63.5        # 플랜지를 감싸는 안지름
DISK_R = 37.5          # 판 바깥 반지름
BORE_D = 34.0          # 가운데 구멍
BOLT_PCD, BOLT_D = 50.0, 6.5
# 카메라
CAM_S = -32.5          # 옆 이동 (렌즈를 그리퍼 중심선에, compare_wrist_camera.py 자동 계산 −32.5)
CAM_D = 57.5           # 공구 축에서 카메라 면 기준점까지
M3_D, M3_CB_D, M3_KEEP = 3.4, 5.8, 6.0    # 나사 자리 6.0 (원래 마운트)
M3_CHAMFER = 1.0      # 로봇 쪽 입구 모따기 45°
M3_PITCH = 45.0
BODY = (90.0, 25.0, 25.0)   # D435i 몸체 (가로, 세로, 깊이)
# 탭: 원래 마운트의 탭 (카메라 구멍 기준 x −34.8 ~ +50.0, y 50.4 ~ 63.4) 을 CAM_S 만큼 옮긴 것
TAB_X = (-57.3 + CAM_S, 27.5 + CAM_S)
TAB_Y = (50.4, 63.4)


def hull2d(pts):
    """2D 볼록 껍질 (monotone chain), 반시계."""
    pts = sorted(set(map(tuple, np.round(pts, 6))))
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, hi = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(hi) >= 2 and cross(hi[-2], hi[-1], p) <= 0:
            hi.pop()
        hi.append(p)
    return lo[:-1] + hi[:-1]


def cam_frame(tilt_deg):
    """카메라 면 좌표계 (cq.Plane): 원점 (CAM_S, CAM_D, PLATE_T), x' = x, z' = 면 법선 (그리퍼 쪽, 공구 축 쪽으로 tilt)."""
    t = math.radians(tilt_deg)
    z = (0.0, -math.sin(t), math.cos(t))
    return cq.Plane(origin=(CAM_S, CAM_D, PLATE_T), xDir=(1, 0, 0), normal=z)


def build(tilt_deg):
    circ = [(DISK_R * math.cos(a), DISK_R * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 180, endpoint=False)]
    rect = [(TAB_X[0], TAB_Y[0]), (TAB_X[1], TAB_Y[0]), (TAB_X[1], TAB_Y[1]), (TAB_X[0], TAB_Y[1])]
    outline = hull2d(circ + rect)
    top = PLATE_T + 15.0                                   # 탭이 기울어 올라가는 높이까지 넉넉히
    body = cq.Workplane("XY").workplane(offset=-RIM_H).polyline(outline).close().extrude(RIM_H + PLATE_T)
    tab = cq.Workplane("XY").workplane(offset=-RIM_H).center(sum(TAB_X) / 2, sum(TAB_Y) / 2) \
        .rect(TAB_X[1] - TAB_X[0], TAB_Y[1] - TAB_Y[0]).extrude(RIM_H + top)
    part = body.union(tab)
    P = cam_frame(tilt_deg)
    # 카메라 면 위 (그리퍼 쪽) 는 비운다: 탭 위 전체 + 카메라 몸체 자리 (여유 2 mm)
    big = cq.Workplane(P).center(0, 0).rect(BODY[0] + 4 + 80, BODY[1] + 4 + 30).extrude(60)
    part = part.cut(big.intersect(cq.Workplane("XY").workplane(offset=-RIM_H).center(sum(TAB_X) / 2, sum(TAB_Y) / 2)
                                  .rect(TAB_X[1] - TAB_X[0] + 0.02, TAB_Y[1] - TAB_Y[0] + 0.02).extrude(RIM_H + top + 1)))
    part = part.cut(cq.Workplane(P).rect(BODY[0] + 4, BODY[1] + 4).extrude(BODY[2] + 10))
    # 플랜지 쪽 자리 (Ø63.5, 깊이 6.45), 가운데 구멍, 볼트·핀 구멍
    part = part.cut(cq.Workplane("XY").workplane(offset=-RIM_H).circle(POCKET_D / 2).extrude(RIM_H))
    part = part.cut(cq.Workplane("XY").workplane(offset=-RIM_H - 1).circle(BORE_D / 2).extrude(RIM_H + PLATE_T + 2))
    r = BOLT_PCD / 2
    holes = [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a))) for a in (45, 135, 225, 315)] + [(0.0, -r)]
    part = part.cut(cq.Workplane("XY").workplane(offset=-RIM_H - 1).pushPoints(holes).circle(BOLT_D / 2).extrude(RIM_H + PLATE_T + 2))
    # M3: 면에서 뒤로 관통 Ø3.4, 면에서 M3_KEEP 뒤부터 Ø5.8
    m3 = [(-M3_PITCH / 2, 0.0), (M3_PITCH / 2, 0.0)]
    part = part.cut(cq.Workplane(P).workplane(offset=1).pushPoints(m3).circle(M3_D / 2).extrude(-40))
    part = part.cut(cq.Workplane(P).workplane(offset=-M3_KEEP).pushPoints(m3).circle(M3_CB_D / 2).extrude(-40))
    # 입구 모따기: 구멍 축과 같은 축의 원뿔. 축이 로봇 쪽 면 (z = −RIM_H) 을 지나는 점에서 지름 Ø5.8 + 2 × M3_CHAMFER
    d = -P.zDir                                            # 카메라 면 → 로봇 쪽
    for xy in m3:
        c = P.toWorldCoords(xy)
        k = (-RIM_H - c.z) / d.z                           # 축이 로봇 쪽 면을 지나는 거리
        p_in = c + d * (k - M3_CHAMFER)
        cone = cq.Solid.makeCone(M3_CB_D / 2, M3_CB_D / 2 + M3_CHAMFER + 3.0, M3_CHAMFER + 3.0, p_in, d)
        part = part.cut(cq.Workplane().add(cone))
    return part


def check_hole(part, tilt_deg):
    """구멍 축을 따라 0.05 mm 간격으로, 반지름 2.3 (Ø3.4 와 Ø5.8 사이) 에 재질이 있는 구간 = 나사 자리, 반지름 3.4 (Ø5.8 밖) 가 비는 구간 = 모따기."""
    P = cam_frame(tilt_deg)
    d = -P.zDir
    sol = part.val()
    out = []
    for xy in ((-M3_PITCH / 2, 0.0), (M3_PITCH / 2, 0.0)):
        c = P.toWorldCoords(xy)
        side = P.xDir
        seat = [s for s in np.arange(0.025, 15, 0.05) if sol.isInside(c + d * s + side * 2.3)]
        cham = [s for s in np.arange(0.025, 15, 0.05) if not sol.isInside(c + d * s + side * 3.4)
                and sol.isInside(c + d * (s - 0.5) + side * 3.4)]
        out.append(f"x' {xy[0]:+.1f}: 나사 자리 0 ~ {max(seat) + 0.025:.2f} mm" + (f", 모따기 시작 {min(cham):.2f} mm" if cham else ""))
    return "; ".join(out)


def camera_box(tilt_deg):
    P = cam_frame(tilt_deg)
    # 몸체: 면에서 z' 0 ~ 25, x' ±45, y' −12.5 ~ +12.5 (구멍이 몸체 세로 가운데)
    return cq.Workplane(P).rect(BODY[0], BODY[1]).extrude(BODY[2])


def preview(part, tilt_deg, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(14, 5))
    for i, (elev, azim, title) in enumerate(((25, -60, "iso"), (0, 0, "side: from +x (y-z, tilt)"), (90, -90, "from gripper (+z)"))):
        ax = fig.add_subplot(1, 3, i + 1, projection="3d")
        for obj, col, a in ((part, "#4a6fa5", 0.9), (camera_box(tilt_deg), "#d9822b", 0.35)):
            v, f = obj.val().tessellate(0.2)
            tri = np.array([[(p.x, p.y, p.z) for p in (v[a_], v[b_], v[c_])] for a_, b_, c_ in f])
            ax.add_collection3d(Poly3DCollection(tri, facecolor=col, edgecolor="none", alpha=a))
        ax.set_xlim(-95, 45); ax.set_ylim(-45, 95); ax.set_zlim(-20, 120)
        ax.set_box_aspect((1, 1, 1)); ax.view_init(elev, azim)
        ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z (gripper)")
        ax.set_title(f"tilt {tilt_deg:g} deg - {title}")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tilts", type=float, nargs="+", default=[0.0, 7.0, 10.0])
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    for t in args.tilts:
        part = build(t)
        name = f"wrist_mount_t{t:g}"
        cq.exporters.export(part, os.path.join(OUT, name + ".step"))
        cq.exporters.export(part, os.path.join(OUT, name + ".stl"), tolerance=0.05, angularTolerance=0.1)
        preview(part, t, os.path.join(OUT, f"preview_t{t:g}.png"))
        bb = part.val().BoundingBox()
        print(f"  {name}: " + check_hole(part, t))
        print(f"{name}: 부피 {part.val().Volume() / 1000:.1f} cm3, 크기 x {bb.xmin:.1f}~{bb.xmax:.1f}, y {bb.ymin:.1f}~{bb.ymax:.1f}, "
              f"z {bb.zmin:.1f}~{bb.zmax:.1f} mm, 고체 {part.val().isValid()}")


if __name__ == "__main__":
    main()
