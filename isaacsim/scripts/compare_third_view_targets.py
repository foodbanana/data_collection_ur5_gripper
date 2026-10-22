#!/usr/bin/env python3
# =============================================================
# compare_third_view_targets.py — third view 카메라 위치를 드론 기준 위치 목록 전체로 고르기 (docs/PLAN.md 6단계 "수집 전 확인")
#
#   compare_third_view.py 는 기본 위치 하나만 본다. 여기서는 scene_drone.yaml drone_targets_base (25 점) 전체 + 여유 상자
#   (리셋 흔들림 reset.drone_offset + 사인파 프리셋 진폭) + 드론 크기가 화면에 들어오는지로 후보 격자를 거른 뒤, 상위 후보만 렌더한다.
#   씬·설정 파일은 그대로 (메모리 stage 만). 팔 시작 자세 = under_drone, 드론 = 기하 제어기 (수평·yaw 0, 리셋과 같음)
#
#   1 계산 (렌더 없음, 후보마다):
#     - 보는 방향 = 자동: 화면에 넣을 점 (점마다 여유 상자만큼 넓힌 드론 외곽 상자 꼭짓점 + 시작 자세 그리퍼) 이 화면 가운데에 오게
#     - 테두리 여유 [px] (가장 바깥 점과 화면 끝, 음수 = 잘림), 화면 가로·세로 중 쓰는 비율
#     - 드론 크기 = 외곽 상자를 투영한 사각형이 화면에서 차지하는 % (가장 먼 점 ~ 가장 가까운 점)
#     - 이륙 경로 (이륙 지점 수직 상승 → 점으로 수평 이동) 와 카메라·받침대 기둥 거리, 테이블 위인지
#   2 렌더 (통과 상위 --render 개 + --cand): 대표 점 (영역 꼭짓점 + 가운데) 마다 시작 자세 / 잡는 자세 닫힘
#     - 드론 (테이프 포함) %, 로봇에 가려진 드론 %, 빨간 테이프 % (가려짐), 손가락 두 쪽 (r·l) 이 따로 보이나, 테두리에 걸림
#
# 실행: ~/isaacsim/python.sh isaacsim/scripts/compare_third_view_targets.py --headless
#   격자 바꾸기 --x 1.0 1.2 --y 0 -0.3 --z 0.2 0.4 (base 기준 렌즈 위치 m), 직접 후보 --cand "Q2:1.1,-0.3,0.20",
#   여유 --preset small|mid|big (기본 big), 렌더 개수 --render 6 (0 = 계산만)
# 결과: isaacsim/reports/compare_third_view_targets_<시각>/ (report.txt, cands.csv, grid_start.png, grid_grasp.png, cover_<후보>.png, 장면별 PNG)
# =============================================================

import argparse
import datetime
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "teleop"))

parser = argparse.ArgumentParser(description="third view 카메라 위치를 드론 기준 위치 목록 전체로 고르기")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--x", type=float, nargs="+", default=[1.0, 1.1, 1.2, 1.3, 1.4], help="렌즈 x 후보 (base m)")
parser.add_argument("--y", type=float, nargs="+", default=[0.0, -0.15, -0.30, -0.45], help="렌즈 y 후보 (base m)")
parser.add_argument("--z", type=float, nargs="+", default=[0.15, 0.25, 0.35, 0.45], help="렌즈 z 후보 (base m = 테이블 상판 위 높이)")
parser.add_argument("--cand", nargs="*", default=["Q1:1.1,0,0.20", "Q2:1.1,-0.3,0.20"],
                    help="격자 밖 후보 이름:x,y,z (base m, 보는 방향은 자동). 항상 렌더한다")
parser.add_argument("--preset", default="big", help="여유 상자에 넣을 사인파 프리셋 (드론 설정 flight.hover_presets). none = 리셋 흔들림만 (사인파가 가끔 테두리를 넘는 것은 허용)")
parser.add_argument("--box", choices=["hull", "body"], default="hull",
                    help="화면에 넣을 드론 상자: hull = 프로펠러 포함 외곽, body = 몸체 충돌 형상만 (프로펠러는 테두리에서 잘려도 됨)")
parser.add_argument("--start", choices=["on", "off"], default="on", help="시작 자세 그리퍼도 화면에 넣나")
parser.add_argument("--border-px", type=float, default=8.0, help="통과: 가장 바깥 점과 화면 끝 최소 [px]")
parser.add_argument("--path-m", type=float, default=0.25, help="통과: 이륙 경로와 카메라·기둥 최소 [m] (드론 반폭 약 0.18 + 여유)")
parser.add_argument("--render", type=int, default=6, help="렌더할 통과 상위 후보 수 (0 = 계산만)")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

import isaacsim.core.experimental.utils.semantics as sem_utils  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
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
SETTLE = 8
WRIST_BELOW = 0.15          # 시작 자세 그리퍼: TCP 와 그 아래 (그리퍼 몸통·손목 쪽) 를 화면에 넣음 [m]
CAM_HALF = 0.07             # 카메라 몸체 반길이 (D456 124 mm) — 이륙 경로 거리에서 뺌
FINGER_MIN_PX = 30          # 손가락 한 쪽이 "보인다" 최소 픽셀
FINGERS = {"rh_p12_rn_r1": "finger_r", "rh_p12_rn_r2": "finger_r", "rh_p12_rn_l1": "finger_l", "rh_p12_rn_l2": "finger_l",
           "rh_p12_rn_base": "gripper_body"}


def render(sensor, annotators=("rgb", "semantic_segmentation")):
    for _ in range(SETTLE):
        simulation_app.update()
    for _ in range(200):
        if sensor.has_data():
            break
        simulation_app.update()
    rgb, _ = sensor.get_data("rgb")
    rgb = rgb.numpy()[..., :3].astype(np.uint8)
    if "semantic_segmentation" not in annotators:
        return rgb, None, None
    seg, info = sensor.get_data("semantic_segmentation")
    return rgb, seg.numpy()[..., 0], info["idToLabels"]


def mask(seg, id2l, *names):
    ids = [int(k) for k, v in id2l.items()
           if any(n in str(v.get("class", "") if isinstance(v, dict) else v) for n in names)]
    return np.isin(seg, ids)


def seg_dist(P, a, b):
    """점들 P (N,3) 와 선분 a-b 의 최소 거리."""
    ab = b - a
    t = np.clip(((P - a) @ ab) / max(float(ab @ ab), 1e-12), 0.0, 1.0)
    return float(np.min(np.linalg.norm(P - (a + t[:, None] * ab), axis=1)))


def main():
    stamp = f"{datetime.datetime.now():%Y%m%d_%H%M%S}"
    out = os.path.join(ts.REPORT_DIR, f"compare_third_view_targets_{stamp}")
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

    # 드론 기준 위치 목록·여유 상자
    targets = {n: np.asarray(v, float) for n, v in s.scene_cfg["drone_targets_base"].items()}
    iface = yaml.safe_load(open(os.path.join(ts.SIM_ROOT, "config", "ros2_iface.yaml"), encoding="utf-8"))
    presets = s.drone_cfg["flight"]["hover_presets"]
    if args.preset != "none" and args.preset not in presets:
        raise ValueError(f"--preset {args.preset} 이 드론 설정 hover_presets 에 없음 ({list(presets)}, 또는 none)")
    amp = np.zeros(3) if args.preset == "none" else np.asarray(presets[args.preset]["amplitude"], float)
    margin = float(iface["reset"]["drone_offset"]) + amp
    takeoff_xy = np.array(s.scene_cfg["px4"]["takeoff_xy"], float)
    tbl = s.scene_cfg["table"]
    t_lo = np.array(tbl["center_xy"], float) - np.array(tbl["size"], float) / 2
    t_hi = np.array(tbl["center_xy"], float) + np.array(tbl["size"], float) / 2
    L(f"compare_third_view_targets {stamp}: 영상 {W}x{H}, 가로 {math.degrees(2 * math.atan(W / 2 / k['fx'])):.1f}° "
      f"세로 {math.degrees(2 * math.atan(H / 2 / k['fy'])):.1f}°, 기준 위치 {len(targets)} 점, "
      f"여유 상자 ±{np.round(margin, 3).tolist()} m (흔들림 {iface['reset']['drone_offset']} + {args.preset} 진폭), "
      f"이륙 지점 (base) {takeoff_xy.tolist()}, 테이블 x {t_lo[0]:.2f} ~ {t_hi[0]:.2f}, y {t_lo[1]:.2f} ~ {t_hi[1]:.2f}")

    # 드론 외곽 상자·잡는 곳 (compare_drone_targets 와 같은 계산)
    body = s.drone_info["body_path"]
    R_d, t_d = dr.drone_frame(st, body)
    prims_body = [p for p in dr.collision_prims(st, s.drone_info["path"]) if dr._owner(p) == body]
    meshes = dr.collision_points(st, prims_body, R_d, t_d)
    pts = np.vstack([m.vertices for m in meshes] + [cg.trimesh.sample.sample_surface(m, 200000)[0] for m in meshes
                                                     if m.faces is not None and len(m.faces)])
    gcfg = yaml.safe_load(open(os.path.join(ts.SIM_ROOT, "config", "grasp_demo.yaml"), encoding="utf-8"))
    x_g = np.asarray(gcfg["tool_x_in_drone"], float)
    z_g = np.array([0.0, 0.0, 1.0])
    R_dg = np.column_stack([x_g, np.cross(z_g, x_g), z_g])
    shapes, _ = cg.collect(st)
    plan = gg.plan_grasp_from_below(shapes, pts @ R_dg)
    tcp_in_drone = R_dg @ np.array([0.0, plan["center_y"], plan["h"]])
    allv = np.vstack([m.vertices for m in dr.collision_points(st, dr.collision_prims(st, s.drone_info["path"]), R_d, t_d)])
    lo, hi = allv.min(0), allv.max(0)
    bodyv = np.vstack([m.vertices for m in meshes])
    lo_n, hi_n = (bodyv.min(0), bodyv.max(0)) if args.box == "body" else (lo, hi)
    L(f"  드론 외곽 상자 (드론 좌표계 mm) {np.round(lo * 1000).tolist()} ~ {np.round(hi * 1000).tolist()}, "
      f"화면에 넣을 상자 ({args.box}) {np.round(lo_n * 1000).tolist()} ~ {np.round(hi_n * 1000).tolist()}")

    def corners(c_w, grow=np.zeros(3), lo=lo, hi=hi):
        return np.array([[c_w[0] + (hi[0] + grow[0] if i else lo[0] - grow[0]),
                          c_w[1] + (hi[1] + grow[1] if j else lo[1] - grow[1]),
                          c_w[2] + (hi[2] + grow[2] if kk else lo[2] - grow[2])]
                         for i in (0, 1) for j in (0, 1) for kk in (0, 1)])

    model = km.ArmModel()
    q_start = s.init_pose.copy()
    F = model.fk(q_start)
    tcp_start_w = to_w(F[:3, 3])
    grip_start_w = np.array([tcp_start_w, tcp_start_w - F[:3, 2] * WRIST_BELOW])
    need = np.vstack([corners(to_w(p), margin, lo_n, hi_n) for p in targets.values()] + ([grip_start_w] if args.start == "on" else []))
    L(f"  화면에 넣을 점 {len(need)} 개 = 점마다 ({args.box} 상자 + 여유) 꼭짓점 8 개"
      + (f" + 시작 자세 그리퍼 (TCP {np.round(to_b(tcp_start_w), 3).tolist()} 와 {WRIST_BELOW * 100:.0f} cm 아래)" if args.start == "on" else ""))

    def project(T, P):
        q = (np.linalg.inv(T) @ np.c_[P, np.ones(len(P))].T).T
        if np.any(q[:, 2] <= 0):
            return None
        return np.c_[K[0, 0] * q[:, 0] / q[:, 2] + K[0, 2], K[1, 1] * q[:, 1] / q[:, 2] + K[1, 2]]

    def aim(pos_w, P):
        """P 가 화면 가운데에 오는 보는 점 (world) 과 optical 자세."""
        d = P - pos_w
        yaw = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
        pit = np.arctan2(d[:, 2], np.hypot(d[:, 0], d[:, 1]))
        y_c, p_c = (yaw.min() + yaw.max()) / 2, (pit.min() + pit.max()) / 2
        for _ in range(6):
            look = pos_w + np.array([math.cos(p_c) * math.cos(y_c), math.cos(p_c) * math.sin(y_c), math.sin(p_c)])
            T = ds.optical_look_at(pos_w, look)
            uv = project(T, P)
            if uv is None:
                return look, T
            y_c -= math.atan(((uv[:, 0].min() + uv[:, 0].max()) / 2 - K[0, 2]) / K[0, 0])
            p_c -= math.atan(((uv[:, 1].min() + uv[:, 1].max()) / 2 - K[1, 2]) / K[1, 1])
        look = pos_w + np.array([math.cos(p_c) * math.cos(y_c), math.cos(p_c) * math.sin(y_c), math.sin(p_c)])
        return look, ds.optical_look_at(pos_w, look)

    def evaluate(name, pos_b):
        pos_w = to_w(pos_b)
        look_w, T = aim(pos_w, need)
        uv = project(T, need)
        r = {"name": name, "pos": np.asarray(pos_b, float), "look_b": to_b(look_w), "T": T}
        d = look_w - pos_w
        r["yaw"] = math.degrees(math.atan2(d[1], d[0]))
        r["pitch"] = math.degrees(math.atan2(d[2], math.hypot(d[0], d[1])))
        if uv is None:
            r.update(border=-1e9, use_w=float("nan"), use_h=float("nan"), d_min=float("nan"), d_max=float("nan"))
        else:
            r["border"] = float(np.min(np.c_[uv[:, 0], W - uv[:, 0], uv[:, 1], H - uv[:, 1]]))
            r["use_w"] = float((uv[:, 0].max() - uv[:, 0].min()) / W)
            r["use_h"] = float((uv[:, 1].max() - uv[:, 1].min()) / H)
            sizes = []
            for p in targets.values():
                q = project(T, corners(to_w(p)))
                q = np.clip(q, [0, 0], [W, H])
                sizes.append(100 * (q[:, 0].max() - q[:, 0].min()) * (q[:, 1].max() - q[:, 1].min()) / (W * H))
            r["d_min"], r["d_max"] = float(min(sizes)), float(max(sizes))
        dists = [float(np.linalg.norm(to_w(p) - pos_w)) for p in targets.values()]
        r["dist"] = (min(dists), max(dists))
        # 이륙 경로 ↔ 카메라 몸체 (점, 반길이 뺌)·받침대 기둥 (상판 → 렌즈 아래)
        post_a, post_b = np.array([pos_w[0], pos_w[1], base_z]), pos_w - np.array([0, 0, 0.02])
        path_min = 1e9
        for p in targets.values():
            c_w = to_w(p)
            a0, a1 = np.array([*takeoff_xy, base_z + 0.06]), np.array([*takeoff_xy, c_w[2]])
            seg = np.vstack([a0 + (a1 - a0) * u for u in np.linspace(0, 1, 40)] + [a1 + (c_w - a1) * u for u in np.linspace(0, 1, 60)])
            path_min = min(path_min, float(np.min(np.linalg.norm(seg - pos_w, axis=1))) - CAM_HALF, seg_dist(seg, post_a, post_b))
        r["path_m"] = path_min
        r["on_table"] = bool(np.all(r["pos"][:2] >= t_lo) and np.all(r["pos"][:2] <= t_hi))
        why = []
        if uv is None:
            why.append("넣을 점이 카메라 뒤")
        elif r["border"] < args.border_px:
            why.append("잘림" if r["border"] < 0 else f"테두리 {r['border']:.0f} px")
        if path_min < args.path_m:
            why.append(f"이륙 경로 {path_min:.2f} m")
        r["why"], r["ok"] = why, not why
        return r

    grid = [(f"c{x:.2f}_{y:+.2f}_{z:.2f}", (x, y, z)) for x in args.x for y in args.y for z in args.z]
    extra = []
    for txt in args.cand or []:
        n, v = txt.split(":")
        extra.append((n, tuple(float(t) for t in v.split(","))))
    rows = [evaluate(n, p) for n, p in grid + extra]
    extra_names = {n for n, _ in extra}
    rows.sort(key=lambda r: (not r["ok"], -r["d_min"] if r["ok"] else -r["border"]))

    def row_text(r):
        return (f"  {'O' if r['ok'] else 'X'} {r['name']:18s} 렌즈 {np.round(r['pos'], 2).tolist()} 보는 점 {np.round(r['look_b'], 2).tolist()} "
                f"방향 {r['yaw']:+4.0f}° 올려봄 {r['pitch']:+3.0f}° | 테두리 {r['border']:6.0f} px, 화면 사용 가로 {r['use_w'] * 100:3.0f} % 세로 {r['use_h'] * 100:3.0f} % | "
                f"드론 {r['d_min']:4.1f} ~ {r['d_max']:4.1f} % (거리 {r['dist'][0]:.2f} ~ {r['dist'][1]:.2f} m) | 이륙 경로 {r['path_m']:5.2f} m"
                f"{'' if r['on_table'] else ' | 테이블 밖'}" + (f"  ← {', '.join(r['why'])}" if r["why"] else ""))

    L("")
    L(f"1 계산 결과 (통과 = 테두리 ≥ {args.border_px:.0f} px, 이륙 경로 ≥ {args.path_m} m; 통과는 가장 먼 점의 드론 크기 순).  "
      f"드론 % = 외곽 상자 투영 사각형 (렌더 드론 % 의 약 2 ~ 3 배)")
    for r in rows:
        L(row_text(r))
    n_ok = sum(r["ok"] for r in rows)
    L(f"  통과 {n_ok}/{len(rows)}")
    with open(os.path.join(out, "cands.csv"), "w", encoding="utf-8") as fc:
        fc.write("name,x_b,y_b,z_b,look_x,look_y,look_z,yaw_deg,pitch_deg,ok,border_px,use_w,use_h,drone_min_pct,drone_max_pct,"
                 "dist_min,dist_max,path_m,on_table,why\n")
        for r in rows:
            fc.write(",".join(map(str, [r["name"], *np.round(r["pos"], 3), *np.round(r["look_b"], 3), round(r["yaw"], 1), round(r["pitch"], 1),
                                        r["ok"], round(r["border"], 1), round(r["use_w"], 3), round(r["use_h"], 3), round(r["d_min"], 2),
                                        round(r["d_max"], 2), round(r["dist"][0], 3), round(r["dist"][1], 3), round(r["path_m"], 3),
                                        r["on_table"], '"' + "; ".join(r["why"]) + '"'])) + "\n")

    chosen = [r for r in rows if r["ok"] and r["name"] not in extra_names][:args.render] + [r for r in rows if r["name"] in extra_names]
    if args.render <= 0:
        chosen = []
    if not chosen:
        with open(os.path.join(out, "report.txt"), "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        return

    # 2 렌더 ------------------------------------------------------------
    # 대표 점: 높이마다 x 최소·최대 × y 최소·최대 + 기본 위치에 가장 가까운 점
    reps = []
    for z in sorted({float(p[2]) for p in targets.values()}):
        at = {n: p for n, p in targets.items() if abs(p[2] - z) < 1e-6}
        xs, ys = [p[0] for p in at.values()], [p[1] for p in at.values()]
        for xx in (min(xs), max(xs)):
            for yy in (min(ys), max(ys)):
                n = min(at, key=lambda m: np.linalg.norm(at[m][:2] - [xx, yy]))
                if n not in reps:
                    reps.append(n)
    nom = to_b(s.drone_pos)
    centre = min(targets, key=lambda m: np.linalg.norm(targets[m] - nom))
    if centre not in reps:
        reps.append(centre)
    L("")
    L(f"2 렌더: 후보 {[r['name'] for r in chosen]}, 대표 점 {reps}, 장면 = 시작 자세 / 잡는 자세 닫힘 (드론 = 점 그대로, 흔들림 없음)")

    sem_utils.add_labels(s.drone_info["path"], labels="drone")
    tape = [f"{body}/{dr.MARKER_NAME}_{side}" for side in ("left", "right")]
    has_tape = all(st.GetPrimAtPath(p).IsValid() for p in tape)
    if has_tape:
        for p in tape:
            sem_utils.add_labels(p, labels="tape")
    else:
        L("  드론 설정에 grasp_marker 가 없음 → 테이프 % 는 재지 않음")
    for n, lab in FINGERS.items():
        sem_utils.add_labels(ts.find_link_path(st, n), labels=lab)
    for name in ("mount_plate", "mount_post"):
        UsdGeom.Imageable(st.GetPrimAtPath(f"{ds.THIRD_VIEW_ROOT}/{name}")).MakeInvisible()
    L("  받침대 겉모습을 감춤 (렌더링만, 충돌 형상은 지금 설정 위치에 그대로)")
    sensor = CameraSensor(s.third_cam, resolution=(H, W), annotators=["rgb", "semantic_segmentation"])
    link = st.GetPrimAtPath(ds.THIRD_VIEW_LINK)
    opt = next(p for p in Usd.PrimRange(link) if p.GetName() == c["model"]["optical_frame"])
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    T_lo = cg.mat_np(cache.GetLocalToWorldTransform(opt) * cache.GetLocalToWorldTransform(link).GetInverse()).T

    def place(T):
        ts._set_matrix(link, Gf.Matrix4d((T @ np.linalg.inv(T_lo)).T.tolist()))

    def solve(q0, p_b, R_b):
        q = np.array(q0, float)
        for _ in range(3000):
            stp = km.ik_step(model, q, q, p_b, R_b, max_dq=0.02, max_lead=1.0)
            q = stp.q
            if stp.pos_err < 1e-5 and stp.rot_err < 1e-4:
                return q
        raise RuntimeError(f"IK 수렴 안 함 (위치 {stp.pos_err * 1000:.2f} mm)")

    robot = UsdGeom.Imageable(st.GetPrimAtPath(ts.ROBOT_PATH))
    f = s.flight

    def move_drone(p_w):
        f.release()
        f.teleport(p_w, yaw=0.0)
        f.arm(p_w)
        for _ in range(60):
            ds.step(s)

    def shoot(tag):
        res = {}
        for r in chosen:
            place(r["T"])
            rgb, seg, id2l = render(sensor)
            robot.MakeInvisible()
            _, seg0, id2l0 = render(sensor)
            robot.MakeVisible()
            md, md0 = mask(seg, id2l, "drone", "tape"), mask(seg0, id2l0, "drone", "tape")
            mt, mt0 = mask(seg, id2l, "tape"), mask(seg0, id2l0, "tape")
            fr, fl = mask(seg, id2l, "finger_r"), mask(seg, id2l, "finger_l")
            mg = fr | fl | mask(seg, id2l, "gripper_body")
            edge = lambda m: bool(m[0, :].any() or m[-1, :].any() or m[:, 0].any() or m[:, -1].any())  # noqa: E731
            res[r["name"]] = dict(
                rgb=rgb, drone=100 * md.mean(), drone_edge=edge(md0),
                hidden=100 * (1 - md.sum() / md0.sum()) if md0.any() else float("nan"),
                tape=100 * mt.mean(), tape_hidden=100 * (1 - mt.sum() / mt0.sum()) if mt0.any() else float("nan"),
                fr=int(fr.sum()), fl=int(fl.sum()), grip_edge=edge(mg))
            Image.fromarray(rgb).save(os.path.join(out, f"{r['name']}_{tag}.png"))
        return res

    results = {}
    for n in reps:
        p_w = to_w(targets[n])
        s.drive.set_gripper_goal(0.0)
        s.drive.reset_pose(q_start, 0.0)
        for _ in range(10):
            ds.step(s)
        move_drone(p_w)
        print(f"  {n}: 시작 자세", flush=True)
        results[(n, "start")] = shoot(f"{n}_start")
        dp, dq = f._body_pose()
        R_dw = cg.quat_to_R(dq)
        q_g = solve(q_start, to_b(dp + R_dw @ tcp_in_drone), R_dw @ R_dg)
        s.drive.reset_pose(q_g, 0.0)
        for _ in range(20):
            ds.step(s)
        s.drive.set_gripper_goal(GRIP_CLOSED)
        for _ in range(int(3.0 / ts.PHYSICS_DT)):
            ds.step(s)
        print(f"  {n}: 잡는 자세 닫힘", flush=True)
        results[(n, "grasp")] = shoot(f"{n}_grasp")
        s.drive.set_gripper_goal(0.0)
        for _ in range(int(2.5 / ts.PHYSICS_DT)):
            ds.step(s)

    L("")
    L("결과 (대표 점마다): 드론 % (* 테두리에 걸림) / 로봇에 가려진 드론 % / 테이프 % (가려짐 %) / 손가락 r·l px (둘 다 "
      f"≥ {FINGER_MIN_PX} 이면 따로 보임 o) / 그리퍼 테두리 *")
    for scene in ("start", "grasp"):
        L(f"  --- {scene}")
        for r in chosen:
            L(f"    {r['name']}")
            for n in reps:
                x = results[(n, scene)][r["name"]]
                L(f"      {n:14s} 드론 {x['drone']:4.1f} %{'*' if x['drone_edge'] else ' '} 가려짐 {x['hidden']:5.1f} % | 테이프 {x['tape']:5.2f} % "
                  f"(가려짐 {x['tape_hidden']:5.1f} %) | 손가락 r {x['fr']:5d} l {x['fl']:5d} "
                  f"{'o' if min(x['fr'], x['fl']) >= FINGER_MIN_PX else 'x'}{' 그리퍼*' if x['grip_edge'] else ''}")
    L("")
    L("요약 (후보마다, 대표 점 전체): 드론 % 최소 ~ 최대 / 잡을 때 가려짐 최대 % / 잡을 때 테이프 최소 % / 손가락 따로 보임 점 수 / 테두리에 걸린 장면 수")
    for r in chosen:
        g = [results[(n, "grasp")][r["name"]] for n in reps]
        a = g + [results[(n, "start")][r["name"]] for n in reps]
        L(f"  {r['name']:18s} 드론 {min(x['drone'] for x in a):4.1f} ~ {max(x['drone'] for x in a):4.1f} % | 가려짐 최대 {max(x['hidden'] for x in g):5.1f} % | "
          f"테이프 최소 {min(x['tape'] for x in g):5.2f} % | 손가락 따로 {sum(min(x['fr'], x['fl']) >= FINGER_MIN_PX for x in g)}/{len(g)} | "
          f"테두리 {sum(x['drone_edge'] or x['grip_edge'] for x in a)}/{len(a)}   ({row_text(r).strip()[2:60].strip()})")

    # 모음 그림: 행 = 후보, 열 = 대표 점
    sw, sh = W // 2, H // 2
    for scene in ("start", "grasp"):
        im_g = Image.new("RGB", (110 + sw * len(reps), 24 + sh * len(chosen)), (255, 255, 255))
        dd = ImageDraw.Draw(im_g)
        for j, n in enumerate(reps):
            dd.text((110 + j * sw + 4, 4), n, fill=(0, 0, 0))
        for i, r in enumerate(chosen):
            p = r["pos"]
            dd.text((4, 24 + i * sh + 4), f"{r['name']}\n({p[0]:.2f}, {p[1]:.2f},\n {p[2]:.2f})\nyaw {r['yaw']:+.0f}\nup {r['pitch']:+.0f}",
                    fill=(0, 0, 0))
            for j, n in enumerate(reps):
                im_g.paste(Image.fromarray(results[(n, scene)][r["name"]]["rgb"]).resize((sw, sh)), (110 + j * sw, 24 + i * sh))
        im_g.save(os.path.join(out, f"grid_{scene}.png"))

    # 덮는 범위 그림: 시작 자세 + 드론 기본 위치에서, 점마다 여유 상자 (외곽 상자 + 여유) 투영 사각형
    s.drive.reset_pose(q_start, 0.0)
    for _ in range(10):
        ds.step(s)
    move_drone(s.drone_pos)
    cs = CameraSensor(s.third_cam, resolution=(H, W), annotators=["rgb"])
    for r in chosen:
        place(r["T"])
        rgb, _, _ = render(cs, ("rgb",))
        im = Image.fromarray(rgb)
        d2 = ImageDraw.Draw(im)
        for n, p in targets.items():
            q = project(r["T"], corners(to_w(p), margin, lo_n, hi_n))
            col = (0, 160, 255) if abs(p[2] - 0.70) < 1e-6 else (255, 140, 0)
            d2.rectangle([q[:, 0].min(), q[:, 1].min(), q[:, 0].max(), q[:, 1].max()], outline=col, width=1)
            u = project(r["T"], to_w(p)[None])[0]
            d2.ellipse([u[0] - 2, u[1] - 2, u[0] + 2, u[1] + 2], fill=col)
        for u in project(r["T"], grip_start_w):
            d2.ellipse([u[0] - 3, u[1] - 3, u[0] + 3, u[1] + 3], outline=(230, 0, 0), width=2)
        d2.text((6, 6), f"{r['name']}: margin boxes (blue z0.70, orange z0.80), red = start gripper", fill=(255, 255, 255))
        im.save(os.path.join(out, f"cover_{r['name']}.png"))
    L(f"\n그림: {out}/grid_start.png, grid_grasp.png, cover_<후보>.png")
    with open(os.path.join(out, "report.txt"), "w", encoding="utf-8") as fw:
        fw.write("\n".join(lines) + "\n")


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
