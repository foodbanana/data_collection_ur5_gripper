#!/usr/bin/env python3
# =============================================================
# compare_drone_targets.py — 드론 기준 위치 후보 거르기 (docs/PLAN.md 6단계 "드론 기준 위치 목록")
#
#   VLA 데이터의 드론 위치를 다양하게 하려고 기준 위치 (사인파 중심) 를 여러 점으로 둘 때, 후보 격자 (로봇 base 기준) 를 점마다 잰다.
#   씬·설정 파일은 그대로 (메모리 stage 만). 팔 시작 자세 = under_drone, 드론 자세 = 수평·yaw 0 (리셋과 같음)
#   잰 것:
#     1 팔이 닿나: 시작 자세에서 잡는 자세 (grasp_demo 와 같은 TCP) 와 그 10 cm 아래 (접근) 의 URDF IK (teleop/ik_model.py) 수렴,
#       관절 한계 여유, 조작성 (특이점), 그 두 자세에서 자기 충돌 (충돌 형상 hull)·테이블 여유 (팔을 순간이동해 PhysX 링크 자세로 봄)
#       + 여유 상자: 기준 위치 ± --margin 상자 꼭짓점 8 개에서 잡는 자세 IK (리셋 흔들림 + 사인파 진폭)
#     2 카메라에 보이나: 드론 외곽 상자 (충돌 형상, 프로펠러 포함) 를 시작 자세 손목 카메라·third view (기본 = 설정 위치) 에 투영
#       → 가운데가 화면 안인지, 꼭짓점 중 화면 안 비율. third view 는 여유 상자만큼 넓힌 외곽 상자로도
#     3 이륙 경로: 테이블 이륙 지점에서 수직 상승 → 수평 이동 선분과 시작 자세 팔 (충돌 형상 꼭짓점) 의 최소 거리
#   그림: 시작 자세 손목 카메라·third view (드론은 기본 위치) 에 후보 점을 겹쳐 그림 (초록 = 다 통과, 노랑 = 시야만 문제, 빨강 = 팔이 못 닿음·충돌)
#
# 실행: ~/isaacsim/python.sh isaacsim/scripts/compare_drone_targets.py --headless [--x 0.45 0.55 0.65 0.75 --y ... --z ... --margin 0.08 0.08 0.065]
# 결과: isaacsim/reports/compare_drone_targets_<시각>/ (report.txt, targets.csv, wrist_start.png, third_view_<후보>.png)
# =============================================================

import argparse
import datetime
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "teleop"))

parser = argparse.ArgumentParser(description="드론 기준 위치 후보 거르기 (base 기준 격자)")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--x", type=float, nargs="+", default=[0.45, 0.55, 0.65, 0.75], help="base 기준 x 후보 [m]")
parser.add_argument("--y", type=float, nargs="+", default=[-0.20, -0.10, 0.0, 0.10, 0.20], help="base 기준 y 후보 [m]")
parser.add_argument("--z", type=float, nargs="+", default=[0.65, 0.75, 0.85], help="base 기준 z 후보 [m] (지금 기본 위치 0.738)")
parser.add_argument("--margin", type=float, nargs=3, default=[0.08, 0.08, 0.065],
                    help="여유 상자 반폭 [m] = 리셋 흔들림 ±0.05 + 사인파 진폭 (기본 mid 0.03·0.03·0.015)")
parser.add_argument("--third-view", nargs="+", default=["cfg"],
                    help="third view 후보: cfg (third_view_camera.yaml 의 위치·보는 점) 또는 이름:x,y,z (base, 보는 점은 기본 위치 잡기와 시작 TCP 의 가운데)")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

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

PRE_BELOW = 0.10
SETTLE = 8
CRIT = {"limit_deg": 5.0,          # 관절 한계 여유 최소
        "manip": 0.01,             # 조작성 최소 (특이점)
        "table_mm": 20.0,          # 팔 링크와 테이블 상판 사이 최소
        "path_m": 0.25,            # 이륙 경로와 팔 사이 최소 (드론 반폭 약 0.18 + 여유)
        "wrist_in": 0.5}           # 시작 자세 손목 카메라: 외곽 상자 꼭짓점이 화면 안 비율 (참고)
NO_TABLE = ("robot_mount", "base_link", "base_link_inertia")   # 테이블에 닿아 있는 링크 (판정에서 뺌)


def solve(model, q0, p_b, R_b, iters=3000):
    q = np.array(q0, float)
    for _ in range(iters):
        stp = km.ik_step(model, q, q, p_b, R_b, max_dq=0.02, max_lead=1.0)
        q = stp.q
        if stp.pos_err < 1e-5 and stp.rot_err < 1e-4:
            return q, stp
    return None, stp


def render(sensor):
    for _ in range(SETTLE):
        simulation_app.update()
    for _ in range(200):
        if sensor.has_data():
            break
        simulation_app.update()
    rgb, _ = sensor.get_data("rgb")
    return rgb.numpy()[..., :3].astype(np.uint8)


def main():
    stamp = f"{datetime.datetime.now():%Y%m%d_%H%M%S}"
    out = os.path.join(ts.REPORT_DIR, f"compare_drone_targets_{stamp}")
    os.makedirs(out, exist_ok=True)
    lines = []

    def L(m=""):
        print(m, flush=True)
        lines.append(m)

    s = ds.build_scene(flight="geometric", init_pose="under_drone")
    st = s.stage
    base_z = float(s.scene_cfg["table"]["top_z"])
    to_w = lambda p: np.asarray(p, float) + np.array([0, 0, base_z])   # noqa: E731
    to_b = lambda p: np.asarray(p, float) - np.array([0, 0, base_z])   # noqa: E731
    nominal_b = to_b(s.drone_pos)
    takeoff_xy = np.array(s.scene_cfg["px4"]["takeoff_xy"], float)
    margin = np.array(args.margin, float)
    L(f"compare_drone_targets {stamp}: 기본 위치 (base) {np.round(nominal_b, 3).tolist()}, 여유 상자 ±{margin.tolist()} m, 기준 {CRIT}")

    # 잡는 곳 (grasp_demo 와 같은 계산, 드론 좌표계)
    body = s.drone_info["body_path"]
    R_d, t_d = dr.drone_frame(st, body)
    prims_body = [p for p in dr.collision_prims(st, s.drone_info["path"]) if dr._owner(p) == body]
    meshes = dr.collision_points(st, prims_body, R_d, t_d)
    pts = np.vstack([m.vertices for m in meshes] + [cg.trimesh.sample.sample_surface(m, 200000)[0] for m in meshes])
    gcfg = yaml.safe_load(open(os.path.join(ts.SIM_ROOT, "config", "grasp_demo.yaml"), encoding="utf-8"))
    x_g = np.asarray(gcfg["tool_x_in_drone"], float)
    z_g = np.array([0.0, 0.0, 1.0])
    R_dg = np.column_stack([x_g, np.cross(z_g, x_g), z_g])
    shapes, link_paths = cg.collect(st)
    plan = gg.plan_grasp_from_below(shapes, pts @ R_dg)
    tcp_in_drone = R_dg @ np.array([0.0, plan["center_y"], plan["h"]])
    # 드론 외곽 상자 (body 좌표계 = 드론 좌표계, 프로펠러 포함)
    allm = dr.collision_points(st, dr.collision_prims(st, s.drone_info["path"]), R_d, t_d)
    allv = np.vstack([m.vertices for m in allm])
    lo, hi = allv.min(0), allv.max(0)
    L(f"  드론 외곽 상자 (드론 좌표계, mm) {np.round(lo * 1000).tolist()} ~ {np.round(hi * 1000).tolist()}, 잡는 TCP {np.round(tcp_in_drone * 1000, 1).tolist()} mm")

    def corners(c_w, grow=np.zeros(3)):
        return np.array([[c_w[0] + (hi if i else lo)[0] + (grow[0] if i else -grow[0]),
                          c_w[1] + (hi if j else lo)[1] + (grow[1] if j else -grow[1]),
                          c_w[2] + (hi if k else lo)[2] + (grow[2] if k else -grow[2])]
                         for i in (0, 1) for j in (0, 1) for k in (0, 1)])

    # 카메라: 손목 (시작 자세, 렌더링 Camera prim 의 world 자세), third view 후보
    model = km.ArmModel()
    q_start = s.init_pose.copy()
    for _ in range(20):
        ds.step(s)
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    W_of = lambda path: cg.mat_np(cache.GetLocalToWorldTransform(st.GetPrimAtPath(path))).T   # noqa: E731  열벡터 규약
    cam_link_path = ts.find_link_path(st, "wrist_camera_link")
    opt_path = ts.find_link_path(st, s.cam_cfg["frame"])
    link_opt = np.linalg.inv(W_of(cam_link_path)) @ W_of(opt_path)
    p_, q_ = (x.numpy()[0] for x in RigidPrim(cam_link_path).get_world_poses())
    T_cl = np.eye(4)
    T_cl[:3, :3], T_cl[:3, 3] = cg.quat_to_R(q_), p_
    T_wrist = T_cl @ link_opt
    kw = s.cam_cfg["intrinsics"]
    Kw = np.array([[kw["fx"], 0, kw["cx"]], [0, kw["fy"], kw["cy"]], [0, 0, 1.0]])
    Ww, Hw = (int(x) for x in s.cam_cfg["resolution"])
    kt = s.tv_cfg["intrinsics"]
    Kt = np.array([[kt["fx"], 0, kt["cx"]], [0, kt["fy"], kt["cy"]], [0, 0, 1.0]])
    Wt, Ht = (int(x) for x in s.tv_cfg["resolution"])
    tcp_start_w = to_w(model.fk(q_start)[:3, 3])
    grasp_nom_w = s.drone_pos + tcp_in_drone
    look_w = (tcp_start_w + grasp_nom_w) / 2
    tvs = []
    for txt in args.third_view:
        if txt == "cfg":
            pos_w = np.asarray(s.tv_cfg["position"], float)
            tvs.append(("cfg", pos_w, ds.optical_look_at(pos_w, np.asarray(s.tv_cfg["look_at"], float))))
            continue
        name, v = txt.split(":")
        pos_w = to_w([float(x) for x in v.split(",")])
        tvs.append((name, pos_w, ds.optical_look_at(pos_w, look_w)))

    def project(T, K, p):
        q = np.linalg.inv(T) @ np.append(p, 1.0)
        return None if q[2] <= 0 else np.array([K[0, 0] * q[0] / q[2] + K[0, 2], K[1, 1] * q[1] / q[2] + K[1, 2]])

    def inside(T, K, W, H, P):
        uv = [project(T, K, p) for p in P]
        return float(np.mean([u is not None and 0 <= u[0] < W and 0 <= u[1] < H for u in uv]))

    # 시작 자세 팔 꼭짓점 (이륙 경로 거리용)
    names = sorted(shapes)
    links = RigidPrim([link_paths[n] for n in names])

    def arm_vertices():
        T = cg.link_transforms(links, names)
        return np.vstack([sh.hull.vertices @ T[n][0].T + T[n][1] for n in names for sh in shapes[n]]), T

    v_start, _ = arm_vertices()
    skip = cg.joint_pairs(st)

    # 팔 순간이동 동안 드론은 멀리 (팔이 드론에 닿지 않게)
    f = s.flight
    away = np.array([0.6, 0.0, 3.0])
    f.release()
    f.teleport(away, yaw=0.0)
    f.arm(away)
    for _ in range(10):
        ds.step(s)

    def pose_check(q):
        s.drive.reset_pose(q, 0.0)
        for _ in range(3):
            ds.step(s)
        V, T = arm_vertices()
        sc = cg.self_collisions(shapes, T, skip, depth_min=0.001)
        tz = min(float((np.vstack([sh.hull.vertices for sh in shapes[n]]) @ T[n][0].T + T[n][1])[:, 2].min())
                 for n in names if n not in NO_TABLE)
        return sc, (tz - base_z) * 1000

    lo_j, hi_j = model.lo.copy(), model.hi.copy()
    rows = []
    grid = [(x, y, z) for z in args.z for x in args.x for y in args.y]
    L(f"  후보 {len(grid)} 점 (x {args.x}, y {args.y}, z {args.z})")
    for k, (x, y, z) in enumerate(grid):
        c_b = np.array([x, y, z])
        c_w = to_w(c_b)
        r = {"name": f"x{x:+.2f}_y{y:+.2f}_z{z:.2f}", "b": c_b}
        g_b = c_b + tcp_in_drone
        q_g, stp = solve(model, q_start, g_b, R_dg)
        r["ik"] = q_g is not None
        why = []
        if q_g is None:
            why.append(f"IK 안 됨 ({stp.pos_err * 1000:.0f} mm)")
        else:
            q_p, stp_p = solve(model, q_g, g_b - R_dg[:, 2] * PRE_BELOW, R_dg)
            if q_p is None:
                why.append("접근 자세 IK 안 됨")
            lim = min(float(np.degrees(np.minimum(q - lo_j, hi_j - q).min())) for q in [q_g] + ([q_p] if q_p is not None else []))
            r["limit_deg"], r["manip"] = lim, stp.manip
            if lim < CRIT["limit_deg"]:
                why.append(f"관절 한계 여유 {lim:.1f}°")
            if stp.manip < CRIT["manip"]:
                why.append(f"특이점 가까움 {stp.manip:.3f}")
            sc_t = []
            for qq in [q_g] + ([q_p] if q_p is not None else []):
                sc, tmm = pose_check(qq)
                sc_t.append((sc, tmm))
            r["table_mm"] = min(t for _, t in sc_t)
            r["self"] = sorted({f"{a}-{b}" for sc, _ in sc_t for a, b, _ in sc})
            if r["self"]:
                why.append("자기 충돌 " + ",".join(r["self"]))
            if r["table_mm"] < CRIT["table_mm"]:
                why.append(f"테이블 {r['table_mm']:.0f} mm")
            # 여유 상자 꼭짓점
            ok_c = 0
            for sx in (-1, 1):
                for sy in (-1, 1):
                    for sz in (-1, 1):
                        qc, _ = solve(model, q_g, g_b + margin * np.array([sx, sy, sz]), R_dg)
                        ok_c += qc is not None
            r["box_ik"] = ok_c
            if ok_c < 8:
                why.append(f"여유 상자 IK {ok_c}/8")
        # 시야
        P = corners(c_w)
        r["wrist_in"] = inside(T_wrist, Kw, Ww, Hw, P)
        cu = project(T_wrist, Kw, c_w)
        r["wrist_center"] = cu is not None and 0 <= cu[0] < Ww and 0 <= cu[1] < Hw
        r["tv"] = {}
        view_why = []
        for name, _, T in tvs:
            a_in = inside(T, Kt, Wt, Ht, P)
            b_in = inside(T, Kt, Wt, Ht, corners(c_w, margin))
            r["tv"][name] = (a_in, b_in)
            if b_in < 1.0:
                view_why.append(f"{name} 여유 상자 {b_in * 100:.0f} %")
        # 이륙 경로 (수직 상승 → 수평 이동)
        a0, a1 = np.array([*takeoff_xy, base_z + 0.06]), np.array([*takeoff_xy, c_w[2]])
        seg = np.vstack([a0 + (a1 - a0) * u for u in np.linspace(0, 1, 40)] + [a1 + (c_w - a1) * u for u in np.linspace(0, 1, 60)])
        d = np.min(np.linalg.norm(v_start[None, :, :] - seg[:, None, :], axis=2))
        r["path_m"] = float(d)
        if d < CRIT["path_m"]:
            why.append(f"이륙 경로–팔 {d:.2f} m")
        r["why"], r["view_why"] = why, view_why
        r["ok"] = not why
        rows.append(r)
        print(f"  [{k + 1}/{len(grid)}] {r['name']}: {'OK' if r['ok'] else 'X'} {why} {view_why}", flush=True)
    s.drive.reset_pose(q_start, 0.0)

    # 표
    L("")
    L("결과 (base m): 팔 (IK·한계 여유°·조작성·테이블 mm·여유 상자 IK) / 손목 시작 화면 (가운데·꼭짓점 %) / third view 꼭짓점 % (기준 위치·여유 상자) / 이륙 경로–팔 m")
    for r in rows:
        tv = "  ".join(f"{n} {a * 100:3.0f}/{b * 100:3.0f}" for n, (a, b) in r["tv"].items())
        arm = (f"한계 {r.get('limit_deg', float('nan')):5.1f}° 조작 {r.get('manip', float('nan')):.3f} 테이블 {r.get('table_mm', float('nan')):5.0f} "
               f"상자 {r.get('box_ik', 0)}/8") if r["ik"] else "IK 안 됨"
        L(f"  {'O' if r['ok'] else 'X'}{'v' if r['view_why'] else ' '} {r['name']}: {arm} | 손목 {'가운데' if r['wrist_center'] else '밖   '} "
          f"{r['wrist_in'] * 100:3.0f} % | {tv} | 경로 {r['path_m']:.2f}" + (f"  ← {'; '.join(r['why'] + r['view_why'])}" if r["why"] or r["view_why"] else ""))
    n_ok = sum(r["ok"] for r in rows)
    n_all = sum(r["ok"] and not r["view_why"] for r in rows)
    L(f"\n  팔·경로 통과 {n_ok}/{len(rows)}, third view ({', '.join(n for n, *_ in tvs)}) 여유 상자까지 다 보이는 것 포함 {n_all}/{len(rows)}")
    with open(os.path.join(out, "targets.csv"), "w", encoding="utf-8") as fcsv:
        fcsv.write("name,x_b,y_b,z_b,ok,ik,limit_deg,manip,table_mm,box_ik,wrist_center,wrist_in," +
                   ",".join(f"{n}_in,{n}_box_in" for n, *_ in tvs) + ",path_m,why\n")
        for r in rows:
            fcsv.write(",".join(map(str, [r["name"], *np.round(r["b"], 3), r["ok"], r["ik"], round(r.get("limit_deg", float("nan")), 2),
                                          round(r.get("manip", float("nan")), 4), round(r.get("table_mm", float("nan")), 1), r.get("box_ik", 0),
                                          r["wrist_center"], round(r["wrist_in"], 2),
                                          *[round(v, 2) for n, _, _ in tvs for v in r["tv"][n]], round(r["path_m"], 3),
                                          '"' + "; ".join(r["why"] + r["view_why"]) + '"'])) + "\n")

    # 그림: 드론을 기본 위치로 되돌리고 손목·third view 에 후보 점
    f.release()
    f.teleport(s.drone_pos, yaw=0.0)
    f.arm(s.drone_pos)
    for _ in range(60):
        ds.step(s)

    def overlay(rgb, T, K, W, H, path, title):
        im = Image.fromarray(rgb)
        dd = ImageDraw.Draw(im)
        for r in rows:
            u = project(T, K, to_w(r["b"]))
            if u is None or not (0 <= u[0] < W and 0 <= u[1] < H):
                continue
            col = (0, 200, 0) if r["ok"] and not r["view_why"] else ((230, 180, 0) if r["ok"] else (220, 0, 0))
            rad = 3 + 3 * args.z.index(r["b"][2]) if r["b"][2] in args.z else 4
            dd.ellipse([u[0] - rad, u[1] - rad, u[0] + rad, u[1] + rad], outline=col, width=2)
        dd.text((6, 6), title, fill=(0, 0, 0))
        im.save(path)

    wsens = CameraSensor(s.wrist_cam, resolution=(Hw, Ww), annotators=["rgb"])
    overlay(render(wsens), T_wrist, Kw, Ww, Hw, os.path.join(out, "wrist_start.png"),
            "wrist start: green ok / yellow third-view edge / red arm; circle size = z (small low)")
    link = st.GetPrimAtPath(ds.THIRD_VIEW_LINK)
    opt = next(p for p in Usd.PrimRange(link) if p.GetName() == s.tv_cfg["model"]["optical_frame"])
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    T_lo = cg.mat_np(cache.GetLocalToWorldTransform(opt) * cache.GetLocalToWorldTransform(link).GetInverse()).T
    for name in ("mount_plate", "mount_post"):
        UsdGeom.Imageable(st.GetPrimAtPath(f"{ds.THIRD_VIEW_ROOT}/{name}")).MakeInvisible()
    tsens = CameraSensor(s.third_cam, resolution=(Ht, Wt), annotators=["rgb"])
    for name, _, T in tvs:
        ts._set_matrix(link, Gf.Matrix4d((T @ np.linalg.inv(T_lo)).T.tolist()))
        overlay(render(tsens), T, Kt, Wt, Ht, os.path.join(out, f"third_view_{name}.png"),
                f"third view {name}: green ok / yellow edge / red arm; circle size = z")
    L(f"그림: {out}/wrist_start.png, third_view_*.png")
    with open(os.path.join(out, "report.txt"), "w", encoding="utf-8") as fr:
        fr.write("\n".join(lines) + "\n")


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
