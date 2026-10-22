#!/usr/bin/env python3
# =============================================================
# third_view_mount_cad.py — third view 카메라 (D456) 받침대 CAD (CadQuery, docs/PLAN.md 6단계 "third view 받침대")
#
#   카메라 자세는 sim 설정에서 읽는다 (config/third_view_camera.yaml position·look_at, scene_drone.yaml table.top_z) → 고정 각도 받침대.
#   좌표 = 받침대 좌표계 [mm]: 원점 = 카메라 아랫면 1/4-20 자리를 테이블 상판으로 수직으로 내린 점, +x = 카메라가 보는 수평 방향, +z = 위.
#   카메라 고정 = 뒷면 M4 × 2 (간격 95 mm, 몸체 세로 가운데, 최대 체결 깊이 4 mm. D456 Datasheet 337029-017 Figure 10-14).
#     아랫면은 비운다 (D456 USB-C 단자가 아랫면, 1/4-20 에서 37 mm 옆) → 카메라 뒷면 판보다 앞 (카메라 쪽) 에는 아무것도 없다.
#   카메라 뒷면 판 (두께 PLATE_T) ← 수직 기둥 (윗면이 판 뒷면을 따라 기울어짐) ← 옆·뒤 리브 ← 바닥판 (Ø5.5 × 4, 카운터보어. M5 볼트·목재 나사)
#   바닥판 윗면 홈: 원점 십자 (= 1/4-20 자리 아래) + 화살표 (+x, 카메라가 보는 방향). 설치 때 로봇 base 기준 위치는 실행 출력에 나옴
#   카메라 프레임 값 (나사 자리 기준, D455 = D456 외형): scripts/build_d455_usd.py 출력 (realsense2_description)
#
# 실행 (레포 밖 venv, CadQuery):  ~/cad/venv/bin/python isaacsim/cad/third_view_mount_cad.py [--screw-len 8]
# 결과: isaacsim/cad/out/third_view_mount.step, .stl, preview.png
# =============================================================

import argparse
import math
import os

import cadquery as cq
import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_ROOT = os.path.dirname(HERE)
OUT = os.path.join(HERE, "out")

# 카메라 (나사 자리 = 아랫면 1/4-20 기준, x 앞·y 왼쪽·z 위, mm)
LENS = (11.15, -11.5, 14.5)          # RGB optical frame (build_d455_usd.py)
BODY_X, BODY_Y, BODY_Z = (-10.2, 15.8), 62.0, (0.0, 29.0)   # 몸체 26 × 124 × 29
M4_Y, M4_Z, M4_DEPTH_MAX = 47.5, 14.5, 4.0                  # 뒷면 M4 (간격 95, 세로 가운데), 최대 체결 깊이
M4_CLEAR = 4.5                       # M4 구멍 지름
USB_BELOW = 60.0                     # 아랫면 아래 비울 깊이 (USB-C 플러그·케이블)
# 받침대
PLATE_T = 5.0                        # 카메라 뒷면 판 두께
PLATE_W = 110.0                      # 판 가로 (M4 ±47.5 + 7.5)
POST_W = 40.0                        # 기둥 폭 (y)
POST_INSET = 1.0                     # 기둥이 판 아래·위 끝에서 들어온 거리 (x)
RIB_T, RIB_H, RIB_L = 6.0, 60.0, 30.0
BASE_T = 7.0
BASE_X, BASE_Y = (-80.0, 35.0), 50.0      # 바닥판 x 범위, y ±
HOLE_D, CB_D, CB_DEPTH, HOLE_INSET = 5.5, 10.5, 3.5, 10.0
MARK_W, MARK_D = 1.2, 0.6                 # 표시 홈 폭·깊이


def camera_pose():
    tv = yaml.safe_load(open(os.path.join(SIM_ROOT, "config", "third_view_camera.yaml"), encoding="utf-8"))
    sc = yaml.safe_load(open(os.path.join(SIM_ROOT, "config", "scene_drone.yaml"), encoding="utf-8"))
    top = float(sc["table"]["top_z"])
    p = np.asarray(tv["position"], float)
    f = np.asarray(tv["look_at"], float) - p
    f /= np.linalg.norm(f)
    pitch = math.asin(f[2])
    yaw = math.atan2(f[1], f[0])
    # 카메라 축 (world): x = f, y = 왼쪽 수평, z = 위 (drone_scene.optical_look_at 과 같은 roll 0)
    left = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    R = np.column_stack([f, left, np.cross(f, left)])
    screw_w = p - R @ (np.array(LENS) / 1000)
    return dict(pitch=pitch, yaw=yaw, screw_base=(screw_w - [0, 0, top]) * 1000, lens_base=(p - [0, 0, top]) * 1000)


def build(cp):
    th = cp["pitch"]
    h = cp["screw_base"][2]
    xC = (math.cos(th), 0.0, math.sin(th))
    zC = (-math.sin(th), 0.0, math.cos(th))
    C = cq.Plane(origin=(0, 0, h), xDir=xC, normal=zC)          # 카메라 좌표계 (local x, y, z = 카메라 앞, 왼쪽, 위)
    back = BODY_X[0]
    # 카메라 뒷면 판
    plate = cq.Workplane(C).center(back - PLATE_T / 2, 0).rect(PLATE_T, PLATE_W).extrude(BODY_Z[1] - BODY_Z[0])
    # 판 아래·위 끝의 수평 위치 (받침대 x)
    x_lo = (back - PLATE_T) * math.cos(th)                           # 판 뒷면 아래 끝
    x_hi = x_lo - (BODY_Z[1] - BODY_Z[0]) * math.sin(th)             # 판 뒷면 위 끝
    px0, px1 = x_hi + POST_INSET, x_lo - POST_INSET
    post = cq.Workplane("XY").center((px0 + px1) / 2, 0).rect(px1 - px0, POST_W).extrude(h + 60)
    # 리브: 뒤 (−x, 가운데) + 옆 (±y), 바닥에서 RIB_H
    back_rib = (cq.Workplane("XZ").polyline([(px0, 0), (px0 - RIB_L, 0), (px0, RIB_H)]).close()
                .extrude(RIB_T / 2, both=True))
    support = post.union(back_rib)
    for s in (1, -1):
        y0 = s * POST_W / 2
        support = support.union(cq.Workplane("YZ").polyline([(y0, 0), (y0 + s * RIB_L, 0), (y0, RIB_H)]).close()
                                .extrude(RIB_T / 2, both=True).translate(((px0 + px1) / 2, 0, 0)))
    # 판 뒷면 평면보다 앞 (카메라 쪽) 은 전부 비움 → 기둥 윗면이 판 뒷면을 따라 기울어짐, 아랫면 USB 자리도 비음
    behind = cq.Workplane(C).center(back - PLATE_T - 500, 0).rect(1000, 1000).extrude(500, both=True)
    support = support.intersect(behind)
    base = cq.Workplane("XY").center(sum(BASE_X) / 2, 0).rect(BASE_X[1] - BASE_X[0], 2 * BASE_Y).extrude(BASE_T)
    part = base.union(support).union(plate)
    # M4 구멍 (카메라 앞 방향 축)
    for y in (M4_Y, -M4_Y):
        c = C.toWorldCoords((back - PLATE_T - 1, y, M4_Z))
        part = part.cut(cq.Workplane().add(cq.Solid.makeCylinder(M4_CLEAR / 2, PLATE_T + 2, c, cq.Vector(*xC))))
    # 바닥판 구멍·카운터보어
    holes = [(x, y) for x in (BASE_X[0] + HOLE_INSET, BASE_X[1] - HOLE_INSET) for y in (BASE_Y - HOLE_INSET, -BASE_Y + HOLE_INSET)]
    part = part.cut(cq.Workplane("XY").workplane(offset=-1).pushPoints(holes).circle(HOLE_D / 2).extrude(BASE_T + 2))
    part = part.cut(cq.Workplane("XY").workplane(offset=BASE_T - CB_DEPTH).pushPoints(holes).circle(CB_D / 2).extrude(CB_DEPTH + 1))
    # 표시 홈: 원점 십자 + 화살표 (+x)
    z0 = BASE_T - MARK_D
    marks = (cq.Workplane("XY").workplane(offset=z0).rect(12, MARK_W).extrude(1)
             .union(cq.Workplane("XY").workplane(offset=z0).rect(MARK_W, 12).extrude(1))
             .union(cq.Workplane("XY").workplane(offset=z0).center(15, 0).rect(18, MARK_W).extrude(1))
             .union(cq.Workplane("XY").workplane(offset=z0).polyline([(30, 0), (23, 4), (23, -4)]).close().extrude(1)))
    part = part.cut(marks)
    return part, C, (px0, px1)


def camera_box(C, below=0.0):
    """카메라 몸체 (below > 0 이면 아랫면 아래 그만큼 = USB 자리)."""
    z0, z1 = (BODY_Z[0] - below, BODY_Z[0]) if below else BODY_Z
    P = cq.Plane(origin=C.toWorldCoords((0, 0, z0)), xDir=C.xDir, normal=C.zDir)
    return cq.Workplane(P).center(sum(BODY_X) / 2, 0).rect(BODY_X[1] - BODY_X[0], 2 * BODY_Y).extrude(z1 - z0)


def preview(part, C, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(15, 5))
    for i, (elev, azim, title) in enumerate(((20, -125, "iso"), (0, -90, "side (x-z)"), (0, 180, "back (y-z)"))):
        ax = fig.add_subplot(1, 3, i + 1, projection="3d")
        for obj, col, a in ((part, "#4a6fa5", 0.9), (camera_box(C), "#d9822b", 0.4)):
            v, fcs = obj.val().tessellate(0.3)
            tri = np.array([[(p.x, p.y, p.z) for p in (v[a_], v[b_], v[c_])] for a_, b_, c_ in fcs])
            ax.add_collection3d(Poly3DCollection(tri, facecolor=col, edgecolor="none", alpha=a))
        ax.set_xlim(-130, 90); ax.set_ylim(-110, 110); ax.set_zlim(0, 220)
        ax.set_box_aspect((1, 1, 1)); ax.view_init(elev, azim)
        ax.set_xlabel("x (view dir)"); ax.set_ylabel("y"); ax.set_zlabel("z")
        ax.set_title(title)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--screw-len", type=float, default=8.0, help="카메라 M4 나사 길이 [mm] (머리 아래)")
    a = ap.parse_args()
    cp = camera_pose()
    part, C, (px0, px1) = build(cp)
    sol = part.val()
    if not sol.isValid():
        raise RuntimeError("고체가 올바르지 않음")
    # 확인: 카메라 몸체·아랫면 아래 USB 자리와 겹치지 않음, 나사 체결 깊이, 렌즈 위치
    for name, box in (("카메라 몸체", camera_box(C)), ("아랫면 아래 USB 자리", camera_box(C, USB_BELOW))):
        v = part.intersect(box).val().Volume() if part.intersect(box).vals() else 0.0
        if v > 1e-3:
            raise RuntimeError(f"받침대가 {name} 과 겹침 ({v:.2f} mm³)")
    ins = a.screw_len - PLATE_T
    if not 0 < ins <= M4_DEPTH_MAX:
        raise ValueError(f"M4 × {a.screw_len:g}: 카메라에 {ins:g} mm 들어감 (0 ~ {M4_DEPTH_MAX:g} mm 여야 함)")
    th = cp["pitch"]
    lens_m = np.array([0.0, 0.0, cp["screw_base"][2]]) + np.array(LENS)[0] * np.array([math.cos(th), 0, math.sin(th)]) \
        + np.array(LENS)[1] * np.array([0, 1, 0]) + np.array(LENS)[2] * np.array([-math.sin(th), 0, math.cos(th)])
    if abs(lens_m[2] - cp["lens_base"][2]) > 0.1:
        raise RuntimeError(f"렌즈 높이 {lens_m[2]:.2f} ↔ 설정 {cp['lens_base'][2]:.2f} mm")
    os.makedirs(OUT, exist_ok=True)
    cq.exporters.export(part, os.path.join(OUT, "third_view_mount.step"))
    cq.exporters.export(part, os.path.join(OUT, "third_view_mount.stl"), tolerance=0.05, angularTolerance=0.1)
    preview(part, C, os.path.join(OUT, "preview.png"))
    bb = sol.BoundingBox()
    yaw = cp["yaw"]
    to_base = lambda x: cp["screw_base"][:2] + x * np.array([math.cos(yaw), math.sin(yaw)])   # noqa: E731  받침대 x 축 위 점 → base xy
    tip = to_base(30.0)
    print(f"third_view_mount: 부피 {sol.Volume() / 1000:.1f} cm³, 크기 x {bb.xmin:.1f} ~ {bb.xmax:.1f}, y {bb.ymin:.1f} ~ {bb.ymax:.1f}, "
          f"z {bb.zmin:.1f} ~ {bb.zmax:.1f} mm, 고체 {sol.isValid()}")
    print(f"  카메라: 올려봄 {math.degrees(th):.2f}°, 보는 방향 (base) {math.degrees(yaw):.2f}°, 1/4-20 자리 높이 {cp['screw_base'][2]:.1f} mm, "
          f"렌즈 높이 {lens_m[2]:.1f} mm (설정 {cp['lens_base'][2]:.1f}), 기둥 x {px0:.1f} ~ {px1:.1f} mm")
    print(f"  카메라 나사: M4 × {a.screw_len:g} (판 {PLATE_T:g} mm → 카메라에 {ins:g} mm, 최대 {M4_DEPTH_MAX:g}), 바닥판: Ø{HOLE_D} × 4 + 카운터보어 Ø{CB_D} 깊이 {CB_DEPTH}")
    print(f"  설치 (로봇 base 기준 mm): 원점 십자 = ({cp['screw_base'][0]:.1f}, {cp['screw_base'][1]:.1f}), "
          f"화살표 끝 (+30 mm) = ({tip[0]:.1f}, {tip[1]:.1f}). 화살표가 base +x 에서 {math.degrees(yaw):.1f}° (왼쪽 +) 를 향함")
    print(f"  결과: {OUT}/third_view_mount.step, .stl, preview.png")


if __name__ == "__main__":
    main()
