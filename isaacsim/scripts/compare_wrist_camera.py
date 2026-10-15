#!/usr/bin/env python3
# =============================================================
# compare_wrist_camera.py — 손목 카메라 장착 후보를 sim 영상으로 비교 (docs/PLAN.md 5-B 뒤 "대량 수집 전 준비" 2 번)
#
#   로봇 USD·xacro·URDF 는 그대로. 렌더링 Camera prim 의 자세만 후보 값으로 옮겨 찍는다 (1-7 tilt 비교와 같은 방식, 메모리 stage 만).
#   후보 = xacro 의 wrist_camera_mount 프레임을 자기 좌표계에서 옮기고 돌린 것:
#     mount x = 옆 (영상 가로), y = 공구 축에서 바깥 (지금 57.5 mm), z = 공구 축 방향 (그리퍼 끝 쪽 +)
#     s  옆 이동 [m]   ("c" = 잡는 자세에서 TCP 가 영상 가로 가운데에 오도록 자동 계산)
#     d  공구 축에서의 거리 [m] (지금 0.0575)
#     b  뒤로 물리기 [m] (손목 쪽, mount −z)
#     t  기울기 [°] (mount x 축, + = 공구 축 쪽, xacro cam_tilt 와 같음)
#   원래 자리의 카메라 몸체·마운트 탭은 겉모습만 감춘다 (뒤로 물린 후보에서 화면을 가리지 않게). 부딪힘은 따로 본다
#   장면 (기하 제어기 드론, 기준 위치 고정): 시작 자세 under_drone / 잡기 10 cm 전 / 잡는 자세 열림 / 닫힘
#   잰 것: 드론·손가락이 화면에서 차지하는 %, 드론이 화면 테두리에 걸리는지, 잡는 곳 (TCP 목표) 과 TCP 의 영상 좌표
#
# 실행: ~/isaacsim/python.sh isaacsim/scripts/compare_wrist_camera.py --headless
#   후보 바꾸기: --cand "C0:0,0.0575,0,0" "C1:c,0.0575,0,0" ...   (이름:s,d,b,t)
# 결과: isaacsim/reports/compare_wrist_camera_<시각>/ (report.txt, 후보 × 장면 모음 grid.png, 장면별 PNG)
# =============================================================

import argparse
import datetime
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "teleop"))

DEFAULT_CANDS = [
    "C0:0,0.0575,0,0",       # 지금 (내려받은 마운트)
    "C1:c,0.0575,0,0",       # 옆으로만 (렌즈를 그리퍼 중심선에)
    "C2:c,0.0575,0.02,0",    # + 20 mm 뒤로
    "C3:c,0.0575,0.04,0",    # + 40 mm 뒤로
    "C4:c,0.0575,0,10",      # + 10° 기울임
    "C5:c,0.0575,0.02,5",    # + 20 mm 뒤로 + 5°
    "C6:c,0.0575,0,15",      # + 15° 기울임
]

parser = argparse.ArgumentParser(description="손목 카메라 장착 후보 비교 (렌더링 카메라만 옮김)")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--cand", nargs="+", default=DEFAULT_CANDS, help="이름:s,d,b,t (s 는 m 또는 c, d·b m, t °)")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

import isaacsim.core.experimental.utils.semantics as sem_utils  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from isaacsim.core.experimental.prims import RigidPrim  # noqa: E402
from isaacsim.sensors.experimental.rtx import CameraSensor  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402
from pxr import Gf, Usd, UsdGeom  # noqa: E402

import collision_geom as cg  # noqa: E402
import drone as dr  # noqa: E402
import drone_scene as ds  # noqa: E402
import gripper_geom as gg  # noqa: E402
import ik_model as km  # noqa: E402
import test_scene as ts  # noqa: E402

GRIP_CLOSED = 1.1351
TCP_Z = 0.11
D_NOW = 0.0575
PRE_BELOW = 0.10
SETTLE = 8


def mat_np(m):
    return np.array([[m[i][j] for j in range(4)] for i in range(4)]).T


def Rx(a):
    c, s = math.cos(a), math.sin(a)
    T = np.eye(4)
    T[1:3, 1:3] = [[c, -s], [s, c]]
    return T


def Tr(v):
    T = np.eye(4)
    T[:3, 3] = v
    return T


FLIP = np.diag([1.0, -1.0, -1.0, 1.0])     # optical (+z 전방, +y 아래) ↔ USD camera (−z 전방, +y 위)


def parse_cand(text):
    name, vals = text.split(":")
    s, d, b, t = vals.split(",")
    return dict(name=name, s=None if s == "c" else float(s), d=float(d), b=float(b), t=float(t))


def M_of(c, s):
    """mount 좌표계에서 후보 mount 로: 옮긴 뒤 (s, d − 지금, −b) 자기 x 축으로 t 회전 (xacro origin xyz → rpy 와 같은 순서)."""
    return Tr([s, c["d"] - D_NOW, -c["b"]]) @ Rx(math.radians(c["t"]))


def render(sensor):
    for _ in range(SETTLE):
        simulation_app.update()
    for _ in range(200):
        if sensor.has_data():
            break
        simulation_app.update()
    rgb, _ = sensor.get_data("rgb")
    seg, info = sensor.get_data("semantic_segmentation")
    return rgb.numpy()[..., :3].astype(np.uint8), seg.numpy()[..., 0], info["idToLabels"]


def mask(seg, id2l, name):
    ids = [int(k) for k, v in id2l.items() if name in str(v.get("class", "") if isinstance(v, dict) else v)]
    return np.isin(seg, ids)


def main():
    cands = [parse_cand(t) for t in args.cand]
    stamp = f"{datetime.datetime.now():%Y%m%d_%H%M%S}"
    out = os.path.join(ts.REPORT_DIR, f"compare_wrist_camera_{stamp}")
    os.makedirs(out, exist_ok=True)
    lines = []

    def L(m=""):
        print(m, flush=True)
        lines.append(m)

    s = ds.build_scene(flight="geometric", init_pose="under_drone")
    st = s.stage
    cam_cfg = s.cam_cfg
    k = cam_cfg["intrinsics"]
    K = np.array([[k["fx"], 0, k["cx"]], [0, k["fy"], k["cy"]], [0, 0, 1.0]])
    W, H = (int(x) for x in cam_cfg["resolution"])
    L(f"compare_wrist_camera {stamp}: 후보 {len(cands)} 개, 영상 {W}x{H}, 세로 시야각 {math.degrees(2 * math.atan(H / 2 / k['fy'])):.1f}°")

    # 라벨·감추기
    sem_utils.add_labels(s.drone_info["path"], labels="drone")
    sem_utils.add_labels(ts.find_link_path(st, "rh_p12_rn_r1"), labels="finger_r")
    sem_utils.add_labels(ts.find_link_path(st, "rh_p12_rn_l1"), labels="finger_l")
    hidden = 0
    for link in ("wrist_camera_link", "wrist_mount"):
        for p in Usd.PrimRange(st.GetPrimAtPath(ts.find_link_path(st, link))):
            if p.IsA(UsdGeom.Mesh) or p.IsA(UsdGeom.Gprim):
                if "collision" in str(p.GetPath()).lower():
                    continue
                UsdGeom.Imageable(p).MakeInvisible()
                hidden += 1
    L(f"  원래 카메라 몸체·마운트 겉모습 {hidden} 개를 감춤 (렌더링만)")
    sensor = CameraSensor(s.wrist_cam, resolution=(H, W), annotators=["rgb", "semantic_segmentation"])

    # 고정 상대 변환 (USD)
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    W_of = lambda path: mat_np(cache.GetLocalToWorldTransform(st.GetPrimAtPath(path)))  # noqa: E731
    cam_link_path = ts.find_link_path(st, "wrist_camera_link")
    opt_path = ts.find_link_path(st, cam_cfg["frame"])
    mount_path = ts.find_link_path(st, ts.CAMERA_TILT_PIVOT)
    oim = np.linalg.inv(W_of(mount_path)) @ W_of(opt_path)          # optical in mount
    link_opt = np.linalg.inv(W_of(cam_link_path)) @ W_of(opt_path)  # optical in camera link
    cam_link = RigidPrim(cam_link_path)
    base = RigidPrim(ts.find_link_path(st, ts.PAYLOAD_LINK))

    def T_opt0():
        p, q = (x.numpy()[0] for x in cam_link.get_world_poses())
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = cg.quat_to_R(q), p
        return T @ link_opt

    def set_cam(M):
        L_ = np.linalg.inv(oim) @ M @ oim @ FLIP                    # Camera prim local (optical 부모 기준)
        UsdGeom.Xformable(st.GetPrimAtPath(s.wrist_cam)).MakeMatrixXform().Set(Gf.Matrix4d(L_.T.tolist()))
        return T_opt0() @ np.linalg.inv(oim) @ M @ oim              # 후보 optical (world)

    def project(T, p):
        q = np.linalg.inv(T) @ np.append(p, 1.0)
        return None if q[2] <= 0 else (K[0, 0] * q[0] / q[2] + K[0, 2], K[1, 1] * q[1] / q[2] + K[1, 2])

    # 잡는 곳 (grasp_demo 와 같은 계산)
    body = s.drone_info["body_path"]
    R_d, t_d = dr.drone_frame(st, body)
    prims = [p for p in dr.collision_prims(st, s.drone_info["path"]) if dr._owner(p) == body]
    meshes = dr.collision_points(st, prims, R_d, t_d)
    pts = np.vstack([m.vertices for m in meshes] + [cg.trimesh.sample.sample_surface(m, 200000)[0] for m in meshes
                                                     if m.faces is not None and len(m.faces)])
    gcfg = yaml.safe_load(open(os.path.join(ts.SIM_ROOT, "config", "grasp_demo.yaml"), encoding="utf-8"))
    x_g = np.asarray(gcfg["tool_x_in_drone"], float)
    z_g = np.array([0.0, 0.0, 1.0])
    R_dg = np.column_stack([x_g, np.cross(z_g, x_g), z_g])
    shapes, _ = cg.collect(st)
    plan = gg.plan_grasp_from_below(shapes, pts @ R_dg)
    tcp_in_drone = R_dg @ np.array([0.0, plan["center_y"], plan["h"]])
    for _ in range(60):
        ds.step(s)
    dp, dq = s.flight._body_pose()
    R_dw = cg.quat_to_R(dq)
    grasp_p, grasp_R = dp + R_dw @ tcp_in_drone, R_dw @ R_dg
    L(f"  드론 (world) {np.round(dp, 4).tolist()}, 잡는 곳 TCP {np.round(grasp_p, 4).tolist()}")

    # 장면 자세: IK (URDF, 로봇 base = 테이블 상판)
    model = km.ArmModel()
    base_z = float(s.scene_cfg["table"]["top_z"])
    to_base = lambda p: np.asarray(p) - np.array([0, 0, base_z])  # noqa: E731

    def solve(q0, p_w, R_w):
        q = np.array(q0, float)
        for _ in range(2000):
            stp = km.ik_step(model, q, q, to_base(p_w), R_w, max_dq=0.02, max_lead=1.0)
            q = stp.q
            if stp.pos_err < 1e-5 and stp.rot_err < 1e-4:
                return q
        raise RuntimeError(f"IK 수렴 안 함 (위치 {stp.pos_err * 1000:.2f} mm, 자세 {math.degrees(stp.rot_err):.3f}°)")

    q_start = s.init_pose.copy()
    q_grasp = solve(q_start, grasp_p, grasp_R)
    q_pre = solve(q_grasp, grasp_p - grasp_R[:, 2] * PRE_BELOW, grasp_R)
    scenes = [("start", q_start, False), ("pre_10cm", q_pre, False), ("grasp_open", q_grasp, False), ("grasp_closed", q_grasp, True)]

    # TCP 를 영상 가로 가운데에 두는 옆 이동 (잡는 자세, 기울기·뒤로 물림 반영)
    def auto_s(c):
        s.drive.reset_pose(q_grasp, 0.0)
        for _ in range(10):
            ds.step(s)
        bp, bq = (x.numpy()[0] for x in base.get_world_poses())
        tcp = bp + cg.quat_to_R(bq) @ np.array([0, 0, TCP_Z])
        sv = 0.0
        for _ in range(20):                 # 1 차원 뉴턴
            u0 = project(T_opt0() @ np.linalg.inv(oim) @ M_of(c, sv) @ oim, tcp)[0]
            u1 = project(T_opt0() @ np.linalg.inv(oim) @ M_of(c, sv + 1e-4) @ oim, tcp)[0]
            sv -= (u0 - K[0, 2]) / ((u1 - u0) / 1e-4)
            if abs(u0 - K[0, 2]) < 0.05:
                break
        return sv

    for c in cands:
        c["s_val"] = auto_s(c) if c["s"] is None else c["s"]
    L("  후보 (mount 좌표계: s 옆, d 공구 축에서 거리, b 뒤로, t 공구 축 쪽 기울기)")
    for c in cands:
        L(f"    {c['name']}: s {c['s_val'] * 1000:+.1f} mm{' (자동)' if c['s'] is None else ''}, d {c['d'] * 1000:.1f} mm, b {c['b'] * 1000:.0f} mm, t {c['t']:g}°")

    res = {}
    for sname, q, closed in scenes:
        s.drive.reset_pose(q, 0.0)
        for _ in range(20):
            ds.step(s)
        if closed:
            s.drive.set_gripper_goal(GRIP_CLOSED)
            for _ in range(int(3.0 / ts.PHYSICS_DT)):
                ds.step(s)
        bp, bq = (x.numpy()[0] for x in base.get_world_poses())
        tcp = bp + cg.quat_to_R(bq) @ np.array([0, 0, TCP_Z])
        dpp, _ = s.flight._body_pose()
        gp = dpp + R_dw @ tcp_in_drone
        for c in cands:
            T = set_cam(M_of(c, c["s_val"]))
            rgb, seg, id2l = render(sensor)
            md = mask(seg, id2l, "drone")
            fr, fl = (100 * mask(seg, id2l, n).mean() for n in ("finger_r", "finger_l"))
            edge = bool(md[0, :].any() or md[-1, :].any() or md[:, 0].any() or md[:, -1].any())
            if md.any():
                vs, us = np.nonzero(md)
                cen = (us.mean(), vs.mean())
            else:
                cen = None
            res[(c["name"], sname)] = dict(rgb=rgb, drone=100 * md.mean(), edge=edge, cen=cen, fr=fr, fl=fl,
                                           tcp=project(T, tcp), grasp=project(T, gp))
            Image.fromarray(rgb).save(os.path.join(out, f"{c['name']}_{sname}.png"))
        if closed:
            s.drive.set_gripper_goal(0.0)
            for _ in range(int(2.5 / ts.PHYSICS_DT)):
                ds.step(s)

    def uv(p):
        if p is None:
            return "뒤"
        inside = 0 <= p[0] < W and 0 <= p[1] < H
        return f"({p[0]:.0f}, {p[1]:.0f}){'' if inside else ' 밖'}"

    L("")
    L(f"결과: 드론 % (테두리에 걸림 *) / 드론 중심 영상 좌표 / 잡는 곳 영상 좌표 / TCP 영상 좌표 / 손가락 오른쪽·왼쪽 %   (영상 가운데 = ({K[0, 2]:.0f}, {K[1, 2]:.0f}))")
    for sname, *_ in scenes:
        L(f"  --- {sname}")
        for c in cands:
            r = res[(c["name"], sname)]
            cen = "-" if r["cen"] is None else f"({r['cen'][0]:.0f}, {r['cen'][1]:.0f})"
            L(f"    {c['name']}: 드론 {r['drone']:5.1f} %{'*' if r['edge'] else ' '} 중심 {cen:12s} 잡는 곳 {uv(r['grasp']):16s} TCP {uv(r['tcp']):16s} "
              f"손가락 {r['fr']:4.1f} / {r['fl']:4.1f} %")

    # 모음 그림: 행 = 후보, 열 = 장면, 절반 크기, 잡는 곳 (초록 +) 과 TCP (빨강 x) 표시
    sw, sh = W // 2, H // 2
    grid = Image.new("RGB", (sw * len(scenes) + 90, sh * len(cands) + 24), (255, 255, 255))
    dr_ = ImageDraw.Draw(grid)
    for j, (sname, *_) in enumerate(scenes):
        dr_.text((90 + j * sw + 4, 4), sname, fill=(0, 0, 0))
    for i, c in enumerate(cands):
        dr_.text((4, 24 + i * sh + 4), f"{c['name']}\ns {c['s_val'] * 1000:+.0f}\nb {c['b'] * 1000:.0f}\nt {c['t']:g}", fill=(0, 0, 0))
        for j, (sname, *_) in enumerate(scenes):
            r = res[(c["name"], sname)]
            im = Image.fromarray(r["rgb"]).resize((sw, sh))
            d2 = ImageDraw.Draw(im)
            for p, col, mk in ((r["grasp"], (0, 220, 0), "+"), (r["tcp"], (230, 0, 0), "x")):
                if p is not None and 0 <= p[0] < W and 0 <= p[1] < H:
                    x, y = p[0] / 2, p[1] / 2
                    if mk == "+":
                        d2.line([(x - 6, y), (x + 6, y)], fill=col, width=2)
                        d2.line([(x, y - 6), (x, y + 6)], fill=col, width=2)
                    else:
                        d2.line([(x - 5, y - 5), (x + 5, y + 5)], fill=col, width=2)
                        d2.line([(x - 5, y + 5), (x + 5, y - 5)], fill=col, width=2)
            d2.line([(sw / 2 - 4, sh / 2), (sw / 2 + 4, sh / 2)], fill=(255, 255, 0))
            d2.line([(sw / 2, sh / 2 - 4), (sw / 2, sh / 2 + 4)], fill=(255, 255, 0))
            grid.paste(im, (90 + j * sw, 24 + i * sh))
    grid.save(os.path.join(out, "grid.png"))
    L(f"\n모음: {os.path.join(out, 'grid.png')} (초록 + = 잡는 곳, 빨강 x = TCP, 노랑 + = 영상 가운데)")
    with open(os.path.join(out, "report.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


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
