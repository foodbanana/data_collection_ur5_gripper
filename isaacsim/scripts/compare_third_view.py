#!/usr/bin/env python3
# =============================================================
# compare_third_view.py — third view 카메라 위치 후보를 sim 영상으로 비교 (docs/PLAN.md 6단계 "수집 전 확인")
#
#   씬·설정 파일은 그대로. third view 카메라 (모양 + 렌더링 카메라) 를 후보 위치로 옮겨 찍는다 (메모리 stage 만).
#   intrinsics 는 third_view_camera.yaml (실물 D456 640x480). 받침대 기둥은 지금 설정 위치에 남으므로 겉모습만 감춘다
#   후보 = 로봇 base 기준 (테이블 상판 위) 렌즈 위치 x, y, z [m] + 보는 점
#     보는 점 "m" = 시작 자세 TCP 와 잡는 곳의 가운데 (시작 ~ 잡기 전체가 화면에 들어오게), 숫자 3 개면 그 점 (base 기준)
#   장면 (기하 제어기 드론, 기준 위치 고정): 시작 자세 under_drone / 잡기 10 cm 전 / 잡는 자세 열림 / 닫힘
#   잰 것: 드론·그리퍼 (base + 손가락 4 개)·손가락이 화면에서 차지하는 %, 화면 테두리에 걸리는지,
#          드론이 로봇에 가려진 비율 (로봇을 감추고 한 번 더 찍어 비교), 잡는 곳·TCP 의 영상 좌표
#
# 실행: ~/isaacsim/python.sh isaacsim/scripts/compare_third_view.py --headless
#   후보 바꾸기: --cand "P1:0.6,-0.48,0.15,m" "Q:1.0,-0.4,0.3,0.6,0,0.6" ...   (이름:x,y,z,보는 점)
# 결과: isaacsim/reports/compare_third_view_<시각>/ (report.txt, 후보 × 장면 모음 grid.png, 장면별 PNG)
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
    "P0:0.05,-0.45,0.138,0.55,0,0.588",   # 지금 설정 (third_view_camera.yaml)
    "P1:0.6,-0.48,0.15,m",                # 옆 (잡는 방향과 수직)
    "P2:0.6,-0.48,0.45,m",                # 옆 높게
    "P3:1.3,-0.4,0.15,m",                 # 앞 대각선 (로봇 맞은편)
    "P4:1.4,0,0.15,m",                    # 앞 (정면)
    "P5:0.3,0.45,0.15,m",                 # 반대 옆 (P0 쪽 대칭)
]

parser = argparse.ArgumentParser(description="third view 카메라 위치 후보 비교 (카메라만 옮김)")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--cand", nargs="+", default=DEFAULT_CANDS, help="이름:x,y,z,보는 점 (base 기준 m. 보는 점 = m 또는 x,y,z)")
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
PRE_BELOW = 0.10
SETTLE = 8
GRIPPER_LINKS = ("rh_p12_rn_base", "rh_p12_rn_r1", "rh_p12_rn_r2", "rh_p12_rn_l1", "rh_p12_rn_l2")
FINGER_LINKS = ("rh_p12_rn_r1", "rh_p12_rn_r2", "rh_p12_rn_l1", "rh_p12_rn_l2")


def parse_cand(text):
    name, vals = text.split(":")
    v = vals.split(",")
    if len(v) == 4 and v[3] == "m":
        look = None
    elif len(v) == 6:
        look = np.array([float(x) for x in v[3:]])
    else:
        raise ValueError(f"후보 형식: 이름:x,y,z,m 또는 이름:x,y,z,lx,ly,lz → {text}")
    return dict(name=name, pos=np.array([float(x) for x in v[:3]]), look=look)


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
    out = os.path.join(ts.REPORT_DIR, f"compare_third_view_{stamp}")
    os.makedirs(out, exist_ok=True)
    lines = []

    def L(m=""):
        print(m, flush=True)
        lines.append(m)

    s = ds.build_scene(flight="geometric", init_pose="under_drone")
    st = s.stage
    c = s.tv_cfg
    k = c["intrinsics"]
    K = np.array([[k["fx"], 0, k["cx"]], [0, k["fy"], k["cy"]], [0, 0, 1.0]])
    W, H = (int(x) for x in c["resolution"])
    base_z = float(s.scene_cfg["table"]["top_z"])
    to_w = lambda p: np.asarray(p, float) + np.array([0, 0, base_z])   # noqa: E731  base → world
    to_b = lambda p: np.asarray(p, float) - np.array([0, 0, base_z])   # noqa: E731
    L(f"compare_third_view {stamp}: 후보 {len(cands)} 개, 영상 {W}x{H}, 가로 {math.degrees(2 * math.atan(W / 2 / k['fx'])):.1f}° "
      f"세로 {math.degrees(2 * math.atan(H / 2 / k['fy'])):.1f}° ({os.path.basename(s.scene_cfg['third_view_camera_config'])})")

    # 라벨
    sem_utils.add_labels(s.drone_info["path"], labels="drone")
    for n in GRIPPER_LINKS:
        sem_utils.add_labels(ts.find_link_path(st, n), labels="finger" if n in FINGER_LINKS else "gripper_body")
    # 받침대 기둥·바닥판은 지금 설정 위치에 고정 → 겉모습만 감춤
    for name in ("mount_plate", "mount_post"):
        UsdGeom.Imageable(st.GetPrimAtPath(f"{ds.THIRD_VIEW_ROOT}/{name}")).MakeInvisible()
    L("  받침대 겉모습을 감춤 (렌더링만, 충돌 형상은 지금 설정 위치에 그대로)")
    sensor = CameraSensor(s.third_cam, resolution=(H, W), annotators=["rgb", "semantic_segmentation"])

    # 카메라 링크 → optical 고정 변환 (drone_scene.add_third_view_camera 와 같은 계산)
    link = st.GetPrimAtPath(ds.THIRD_VIEW_LINK)
    opt = next(p for p in Usd.PrimRange(link) if p.GetName() == c["model"]["optical_frame"])
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    T_lo = cg.mat_np(cache.GetLocalToWorldTransform(opt) * cache.GetLocalToWorldTransform(link).GetInverse()).T

    def place(pos_w, look_w):
        T = ds.optical_look_at(pos_w, look_w)
        ts._set_matrix(link, Gf.Matrix4d((T @ np.linalg.inv(T_lo)).T.tolist()))
        return T

    def project(T, p):
        q = np.linalg.inv(T) @ np.append(p, 1.0)
        return None if q[2] <= 0 else (K[0, 0] * q[0] / q[2] + K[0, 2], K[1, 1] * q[1] / q[2] + K[1, 2])

    # 잡는 곳 (grasp_demo·compare_wrist_camera 와 같은 계산)
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

    model = km.ArmModel()

    def solve(q0, p_w, R_w):
        q = np.array(q0, float)
        for _ in range(2000):
            stp = km.ik_step(model, q, q, to_b(p_w), R_w, max_dq=0.02, max_lead=1.0)
            q = stp.q
            if stp.pos_err < 1e-5 and stp.rot_err < 1e-4:
                return q
        raise RuntimeError(f"IK 수렴 안 함 (위치 {stp.pos_err * 1000:.2f} mm, 자세 {math.degrees(stp.rot_err):.3f}°)")

    q_start = s.init_pose.copy()
    q_grasp = solve(q_start, grasp_p, grasp_R)
    q_pre = solve(q_grasp, grasp_p - grasp_R[:, 2] * PRE_BELOW, grasp_R)
    scenes = [("start", q_start, False), ("pre_10cm", q_pre, False), ("grasp_open", q_grasp, False), ("grasp_closed", q_grasp, True)]
    tcp_start_w = to_w(model.fk(q_start)[:3, 3])
    mid_b = to_b((tcp_start_w + grasp_p) / 2)
    L(f"  잡는 곳 TCP (base) {np.round(to_b(grasp_p), 3).tolist()}, 시작 TCP (base) {np.round(to_b(tcp_start_w), 3).tolist()}, "
      f"보는 점 m = {np.round(mid_b, 3).tolist()}")
    for cd in cands:
        cd["look_b"] = mid_b if cd["look"] is None else cd["look"]
        d = cd["look_b"] - cd["pos"]
        cd["dist"] = float(np.linalg.norm(d))
        cd["pitch"] = math.degrees(math.atan2(d[2], math.hypot(d[0], d[1])))
        cd["yaw"] = math.degrees(math.atan2(d[1], d[0]))
    L("  후보 (base 기준 m: 렌즈 → 보는 점, 거리, 방향 = base +x 에서 왼쪽 + 으로 돈 각, 올려다본 각)")
    for cd in cands:
        L(f"    {cd['name']}: {np.round(cd['pos'], 3).tolist()} → {np.round(cd['look_b'], 3).tolist()}  "
          f"{cd['dist']:.2f} m, 방향 {cd['yaw']:+.0f}°, 올려봄 {cd['pitch']:+.0f}°")

    robot = UsdGeom.Imageable(st.GetPrimAtPath(ts.ROBOT_PATH))
    base = RigidPrim(ts.find_link_path(st, ts.PAYLOAD_LINK))
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
        for cd in cands:
            T = place(to_w(cd["pos"]), to_w(cd["look_b"]))
            rgb, seg, id2l = render(sensor)
            robot.MakeInvisible()
            _, seg0, id2l0 = render(sensor)
            robot.MakeVisible()
            md, md0 = mask(seg, id2l, "drone"), mask(seg0, id2l0, "drone")
            mg, mf = mask(seg, id2l, "gripper_body") | mask(seg, id2l, "finger"), mask(seg, id2l, "finger")
            edge = lambda m: bool(m[0, :].any() or m[-1, :].any() or m[:, 0].any() or m[:, -1].any())  # noqa: E731
            res[(cd["name"], sname)] = dict(
                rgb=rgb, drone=100 * md.mean(), drone_edge=edge(md0),
                hidden=100 * (1 - md.sum() / md0.sum()) if md0.any() else float("nan"),
                grip=100 * mg.mean(), grip_edge=edge(mg), finger=100 * mf.mean(),
                tcp=project(T, tcp), grasp=project(T, gp))
            Image.fromarray(rgb).save(os.path.join(out, f"{cd['name']}_{sname}.png"))
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
    L("결과: 드론 % (테두리에 걸림 *) / 로봇에 가려진 드론 % / 그리퍼 (몸통 + 손가락) % (*) / 손가락 % / 잡는 곳·TCP 영상 좌표"
      f"   (영상 가운데 = ({K[0, 2]:.0f}, {K[1, 2]:.0f}))")
    for sname, *_ in scenes:
        L(f"  --- {sname}")
        for cd in cands:
            r = res[(cd["name"], sname)]
            L(f"    {cd['name']}: 드론 {r['drone']:5.1f} %{'*' if r['drone_edge'] else ' '} 가려짐 {r['hidden']:5.1f} %  "
              f"그리퍼 {r['grip']:4.1f} %{'*' if r['grip_edge'] else ' '} 손가락 {r['finger']:4.2f} %  "
              f"잡는 곳 {uv(r['grasp']):16s} TCP {uv(r['tcp'])}")

    # 모음 그림: 행 = 후보, 열 = 장면, 절반 크기, 잡는 곳 (초록 +) 과 TCP (빨강 x)
    sw, sh = W // 2, H // 2
    grid = Image.new("RGB", (sw * len(scenes) + 110, sh * len(cands) + 24), (255, 255, 255))
    d_ = ImageDraw.Draw(grid)
    for j, (sname, *_) in enumerate(scenes):
        d_.text((110 + j * sw + 4, 4), sname, fill=(0, 0, 0))
    for i, cd in enumerate(cands):
        p = cd["pos"]
        d_.text((4, 24 + i * sh + 4), f"{cd['name']}\n({p[0]:.2f}, {p[1]:.2f},\n {p[2]:.2f})\n{cd['dist']:.2f} m\nyaw {cd['yaw']:+.0f}\nup {cd['pitch']:+.0f}",
                fill=(0, 0, 0))
        for j, (sname, *_) in enumerate(scenes):
            r = res[(cd["name"], sname)]
            im = Image.fromarray(r["rgb"]).resize((sw, sh))
            d2 = ImageDraw.Draw(im)
            for pt, col, mk in ((r["grasp"], (0, 220, 0), "+"), (r["tcp"], (230, 0, 0), "x")):
                if pt is not None and 0 <= pt[0] < W and 0 <= pt[1] < H:
                    x, y = pt[0] / 2, pt[1] / 2
                    if mk == "+":
                        d2.line([(x - 6, y), (x + 6, y)], fill=col, width=2)
                        d2.line([(x, y - 6), (x, y + 6)], fill=col, width=2)
                    else:
                        d2.line([(x - 5, y - 5), (x + 5, y + 5)], fill=col, width=2)
                        d2.line([(x - 5, y + 5), (x + 5, y - 5)], fill=col, width=2)
            grid.paste(im, (110 + j * sw, 24 + i * sh))
    grid.save(os.path.join(out, "grid.png"))
    L(f"\n모음: {os.path.join(out, 'grid.png')} (초록 + = 잡는 곳, 빨강 x = TCP)")
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
