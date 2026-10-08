#!/usr/bin/env python3
# =============================================================
# check_rc.py  (docs/PLAN.md 5단계 5-A A-2, docs/drone_teleop.md)
#
# 조종기에 넘기기 시험: PX4 SITL 드론을 Offboard 로 띄운 뒤 PX4Commander.handover() 로 수동 모드 (POSCTL) 에 넘기고,
# 스틱 값 (MANUAL_CONTROL, 명령 링크 UDP 14540) 으로 움직인다. 씬: ground plane + 드론 (로봇 없음).
# 스틱은 조종기 대신 스크립트가 만든 값이다 (직렬·조종기 없이 돈다). 조종기 설정의 px4_params 는 그대로 넣는다.
#   R0 연결·이륙: 가운데 스틱 값을 보내는 채로 Offboard 이륙 (지금 흐름이 그대로 되는지)
#   R1 넘기기: handover() 뒤 HANDOVER_SEC 안에 POSCTL, 그 뒤 HOLD_SEC 동안 넘긴 순간의 위치에서 벗어난 거리
#   R2 축별 스틱: pitch·roll·throttle·yaw 를 차례로 +PUSH 만큼 PUSH_SEC 밀었다 놓음 → 움직인 방향 (drone body 기준), 속도, 놓은 뒤 멈춤
#   R3 되돌리기: takeback() 뒤 Offboard, 그 자리에 머묾 (조종기 신호가 끊겼을 때의 동작)
#   R4 (기록) 스틱 값 송신이 끊김: POSCTL 에서 MANUAL_CONTROL 을 LOSS_SEC 동안 멈추면 PX4 가 어떻게 하는지, 다시 보내고 takeback() 하면 돌아오는지 (판정 없음)
#   R5 kill: 수동 모드에서 모터 정지
#
# 실행:
#   ~/isaacsim/python.sh ~/data_collection_ur5_gripper/isaacsim/scripts/check_rc.py --headless
# 결과: 콘솔, isaacsim/reports/check_rc_<시각>.txt·csv, PX4 콘솔·ulog (reports/px4_<시각>/)
# =============================================================

import argparse
import datetime
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_scene as ts  # noqa: E402

parser = argparse.ArgumentParser(description="조종기에 넘기기 시험 (PLAN 5단계 5-A)")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--drone-config", default=os.path.join(ts.CONFIG_DIR, "drone_iris.yaml"))
parser.add_argument("--rc-config", default=os.path.join(ts.CONFIG_DIR, "rc_input.yaml"))
parser.add_argument("--config", default=ts.DEFAULT_CONFIG, help="물체 재질(contact.object_material)을 읽을 yaml")
parser.add_argument("--physics-hz", type=float, default=120.0)
parser.add_argument("--start-z", type=float, default=0.08, help="드론 시작 높이 (body 원점, 바닥에 내려앉음)")
parser.add_argument("--takeoff-alt", type=float, default=1.0)
parser.add_argument("--px4-param", action="append", default=[], metavar="NAME=VALUE", help="비교용: PX4 파라미터 덮어쓰기 (여러 번 가능)")
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--report-dir", default=ts.REPORT_DIR)
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
import numpy as np  # noqa: E402
from isaacsim.core.experimental.objects import DomeLight, GroundPlane  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402

import drone as dr  # noqa: E402
import drone_flight as fl  # noqa: E402
import rc_input  # noqa: E402

CONNECT_SEC, TAKEOFF_SEC, TAKEOFF_TOL = 60.0, 30.0, 0.05
SETTLE_SEC = 8.0
HANDOVER_SEC, HOLD_SEC, HOLD_TOL = 3.0, 10.0, 0.10
PUSH, PUSH_SEC, REST_SEC = 0.6, 2.0, 5.0
MOVE_MIN, YAW_MIN_DEG, STOP_TOL = 0.03, 5.0, 0.02
LOSS_SEC = 2.5             # 송신을 멈추는 시간 (PX4 COM_RC_LOSS_T 0.5 s 뒤 failsafe. 길면 착륙해 버린다)

LINES, RESULTS = [], []


def log(msg=""):
    print(msg, flush=True)
    LINES.append(msg)


def result(no, title, ok, why=""):
    RESULTS.append((no, title, "PASS" if ok else "FAIL"))
    log(f"  => {'PASS' if ok else 'FAIL'}" + (f" ({why})" if why else ""))


def build(rc_cfg):
    cfg = dr.load_drone_config(args.drone_config)
    cfg["px4"]["params"] = {**(cfg["px4"]["params"] or {}), **rc_cfg["px4_params"]}
    for kv in args.px4_param:
        k, v = kv.split("=", 1)
        cfg["px4"]["params"][k.strip()] = float(v) if "." in v else int(v)
    config = ts.load_config(args.config)
    stage_utils.create_new_stage()
    stage_utils.set_stage_up_axis("Z")
    stage_utils.set_stage_units(meters_per_unit=1.0, kilograms_per_unit=1.0)
    stage = stage_utils.get_current_stage(backend="usd")
    stage_utils.define_prim("/World", "Xform")
    if not SimulationManager.switch_physics_engine("physx"):
        raise RuntimeError("PhysX 로 전환하지 못했습니다")
    SimulationManager.setup_simulation(dt=1.0 / args.physics_hz)
    GroundPlane("/World/GroundPlane")
    DomeLight("/World/DomeLight").set_intensities(1000)
    ts.define_physics_material(stage, ts.OBJECT_MATERIAL_PATH, config["contact"]["object_material"])
    info = dr.add_drone(stage, cfg, position=(0.0, 0.0, args.start_z), prop_spin=False)
    app_utils.play()
    simulation_app.update()
    return cfg, info


def pose(f):
    p, q = f._body_pose()
    Rm = fl.quat_to_R(q)
    return p.copy(), Rm, math.atan2(Rm[1, 0], Rm[0, 0]), math.degrees(math.acos(np.clip(Rm[2, 2], -1, 1)))


ROWS = []


def run(f, sec, until=None):
    """sim 시간 sec 동안 (until(f) 가 참이면 먼저 끝). 끝난 이유가 until 이면 True."""
    t_end = f.t + sec
    while f.t < t_end - 1e-9:
        simulation_app.update()
        f.check()
        p, _, yaw, tilt = pose(f)
        if not np.all(np.isfinite(p)):
            raise RuntimeError(f"드론 위치 NaN (t = {f.t:.3f} s)")
        ROWS.append((f.t, *p, yaw, tilt, *(f.cmd.sticks or (np.nan,) * 4), int(f.cmd.want_manual), int(f.px4.armed), f.cmd.mode or ""))
        if until is not None and until(f):
            return True
    return False


def rows(t0):
    return np.array([r[:6] for r in ROWS if r[0] >= t0], dtype=float)


def statustext(f, since=0.0):
    for t, sev, text in f.cmd.messages:
        if t >= since:
            log(f"    PX4 [{t:7.2f} s, sev {sev}] {text}")


def tests(cfg, info):
    f = fl.DroneFlight(cfg, info, mode="static", seed=args.seed, backend="px4", report_dir=args.report_dir)
    c = f.cmd
    log(f"PX4 파라미터 덮어쓰기 {f.px4_params}")
    wall0 = time.monotonic()
    try:
        # R0
        log("\n[R0] 연결·이륙 (가운데 스틱 값을 보내는 채로 Offboard)")
        f.start()
        if not run(f, CONNECT_SEC, until=lambda f: f.px4.status == "lockstep"):
            result("R0", "연결·이륙", False, f"lockstep 안 됨 ({f.px4.status})")
            return f
        run(f, CONNECT_SEC, until=lambda f: c.connected)
        c.set_sticks(0.0, 0.0, 0.0, 0.0)
        run(f, 5.0)
        target = pose(f)[0] + np.array([0.0, 0.0, args.takeoff_alt])
        f.arm(target)
        armed = run(f, CONNECT_SEC, until=lambda f: f.px4.armed)

        def arrived(f):
            return c.mode == "OFFBOARD" and c.estimate_enu is not None and np.linalg.norm(c.estimate_enu - target) < TAKEOFF_TOL

        t0 = f.t
        ok = armed and run(f, TAKEOFF_SEC, until=arrived)
        log(f"  armed {f.px4.armed}, 모드 {c.mode}, 이륙 {f.t - t0:.2f} s, 실제 − 목표 {np.round((pose(f)[0] - target) * 1000, 1).tolist()} mm")
        result("R0", "연결·이륙", ok, "" if ok else "이륙 못 함")
        if not ok:
            statustext(f)
            return f
        run(f, SETTLE_SEC)

        # R1
        log(f"\n[R1] 넘기기 (handover → POSCTL {HANDOVER_SEC:.0f} s 안, 그 뒤 {HOLD_SEC:.0f} s 가운데 스틱)")
        p0, _, yaw0, _ = pose(f)
        t0 = f.t
        c.handover()
        sw = run(f, HANDOVER_SEC, until=lambda f: c.mode == "POSCTL")
        t_sw = f.t - t0
        run(f, HOLD_SEC)
        d = rows(t0)
        dev = np.linalg.norm(d[:, 1:4] - p0, axis=1)
        log(f"  모드 {c.mode} ({t_sw:.2f} s 뒤), 넘긴 위치에서 최대 {dev.max() * 1000:.1f} mm (축별 최대 "
            f"{np.round(np.abs(d[:, 1:4] - p0).max(axis=0) * 1000, 1).tolist()} mm), 끝 {dev[-1] * 1000:.1f} mm, "
            f"yaw 변화 최대 {np.degrees(np.abs(d[:, 4] - yaw0).max()):.2f}°, 기울기 최대 {d[:, 5].max():.2f}°, 위치 목표 송신 {c.target is not None}")
        why = [] if sw else [f"POSCTL 로 안 바뀜 (모드 {c.mode})"]
        if dev.max() > HOLD_TOL:
            why.append(f"{dev.max() * 1000:.0f} mm 벗어남")
        if c.target is not None:
            why.append("위치 목표를 계속 보냄")
        statustext(f, t0)
        result("R1", "넘기기", not why, "; ".join(why))
        if not sw:
            return f

        # R2
        log(f"\n[R2] 축별 스틱 +{PUSH} 를 {PUSH_SEC:.0f} s 밀었다 놓고 {REST_SEC:.0f} s (움직임은 밀기 시작할 때의 드론 방향 기준: 앞·오른쪽·위, yaw 는 오른쪽 회전 +)")
        why = []
        for k, name in enumerate(rc_input.AXES):
            p0, R0, yaw0, _ = pose(f)
            t0 = f.t
            s = [0.0] * 4
            s[k] = PUSH
            c.set_sticks(*s)
            run(f, PUSH_SEC)
            c.set_sticks(0.0, 0.0, 0.0, 0.0)
            run(f, REST_SEC)
            d = rows(t0)
            rel = (d[:, 1:4] - p0) @ R0                                    # body FLU
            move = np.column_stack([rel[:, 0], -rel[:, 1], rel[:, 2]])       # 앞, 오른쪽, 위
            dyaw = -np.degrees(np.unwrap(d[:, 4]) - yaw0)                    # 오른쪽 회전 +
            tt = d[:, 0] - t0
            speed = np.linalg.norm(np.gradient(d[:, 1:4], d[:, 0], axis=0), axis=1)
            rest = np.linalg.norm(d[-1, 1:4] - d[tt >= REST_SEC + PUSH_SEC - 1.0][0, 1:4])
            log(f"  {name:8s}: 끝 위치 (앞, 오른쪽, 위) {np.round(move[-1] * 1000, 1).tolist()} mm, yaw {dyaw[-1]:+.1f}°, 최고 속도 {speed.max():.3f} m/s, "
                f"기울기 최대 {d[:, 5].max():.2f}°, 마지막 1 s 이동 {rest * 1000:.1f} mm")
            if name == "yaw":
                if not dyaw[-1] > YAW_MIN_DEG:
                    why.append(f"yaw 가 오른쪽으로 안 돎 ({dyaw[-1]:+.1f}°)")
            else:
                i = {"pitch": 0, "roll": 1, "throttle": 2}[name]
                if not (move[-1, i] > MOVE_MIN and abs(move[-1, i]) == np.abs(move[-1]).max()):
                    why.append(f"{name}: 기대 방향으로 안 움직임 {np.round(move[-1], 3).tolist()}")
            if rest > STOP_TOL:
                why.append(f"{name}: 놓은 뒤 안 멈춤 ({rest * 1000:.0f} mm)")
        result("R2", "축별 스틱", not why, "; ".join(why))

        # R3
        log("\n[R3] 되돌리기 (takeback → Offboard, 그 자리)")
        p0 = pose(f)[0]
        t0 = f.t
        c.takeback()
        sw = run(f, HANDOVER_SEC, until=lambda f: c.mode == "OFFBOARD")
        t_sw = f.t - t0
        run(f, 5.0)
        dev = np.linalg.norm(rows(t0)[:, 1:4] - p0, axis=1)
        log(f"  모드 {c.mode} ({t_sw:.2f} s 뒤), 되돌린 위치에서 최대 {dev.max() * 1000:.1f} mm, 끝 {dev[-1] * 1000:.1f} mm")
        statustext(f, t0)
        ok = sw and dev.max() <= HOLD_TOL
        result("R3", "되돌리기", ok, "" if ok else f"모드 {c.mode}, {dev.max() * 1000:.0f} mm")

        # R4 (기록)
        log(f"\n[R4] (기록) POSCTL 에서 스틱 값 송신을 {LOSS_SEC} s 멈췄다가 다시 보내고 takeback")
        c.handover()
        run(f, HANDOVER_SEC, until=lambda f: c.mode == "POSCTL")
        run(f, 2.0)
        p0 = pose(f)[0]
        t0 = f.t
        c.sticks = None                                 # 시험용: MANUAL_CONTROL 을 더 보내지 않음 (PX4Commander 가 멈춘 경우)
        modes = []

        def watch(sec):
            t_end = f.t + sec
            while f.t < t_end - 1e-9:
                run(f, 0.05)
                if not modes or modes[-1][1] != c.mode:
                    modes.append((round(f.t - t0, 2), c.mode))

        watch(LOSS_SEC)
        p1 = pose(f)[0]
        c.set_sticks(0.0, 0.0, 0.0, 0.0)
        c.takeback()
        watch(6.0)
        log(f"  모드 변화 (송신을 멈춘 뒤 s) {modes}, armed {f.px4.armed}")
        log(f"  멈춘 동안 위치 변화 (x, y, z) {np.round((p1 - p0).astype(float) * 1000, 1).tolist()} mm, takeback 6 s 뒤 {np.round((pose(f)[0] - p0).astype(float) * 1000, 1).tolist()} mm")
        statustext(f, t0)
        result("R4", "송신 끊김 (기록)", True)

        # R5
        log("\n[R5] kill (수동 모드에서)")
        if c.mode == "OFFBOARD" and f.px4.armed:
            c.handover()
            run(f, HANDOVER_SEC, until=lambda f: c.mode == "POSCTL")
            run(f, 2.0)
        mode0, z0 = c.mode, pose(f)[0][2]
        t0 = f.t
        f.release()
        off = run(f, 1.0, until=lambda f: not f.px4.armed)
        t_off = f.t - t0
        run(f, 0.4)
        drop = z0 - pose(f)[0][2]
        log(f"  kill 전 모드 {mode0}, 높이 {z0:.3f} m, disarm {t_off * 1000:.0f} ms, 0.4 s 뒤 {drop * 1000:.0f} mm 낙하, 모터 ω 최대 {f.omega.max():.1f} rad/s")
        ok = mode0 == "POSCTL" and off and drop > 0.2
        result("R5", "kill", ok, "" if ok else f"모드 {mode0}, disarm {off}, 낙하 {drop * 1000:.0f} mm")
    finally:
        wall = time.monotonic() - wall0
        log(f"\nsim {f.t:.1f} s / 실제 {wall:.1f} s (실시간 비율 {f.t / wall:.2f})")
        f.stop()
    return f


def write_report(n_expected):
    log("")
    log("=" * 50)
    for no, title, tag in RESULTS:
        log(f"  {no}. {title:16s} {tag}")
    n_pass = sum(t == "PASS" for _, _, t in RESULTS)
    log(f"  {n_pass}/{n_expected} PASS")
    os.makedirs(args.report_dir, exist_ok=True)
    stamp = f"{datetime.datetime.now():%Y%m%d_%H%M%S}"
    path = os.path.join(args.report_dir, f"check_rc_{stamp}.txt")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(LINES) + "\n")
    with open(os.path.join(args.report_dir, f"check_rc_{stamp}.csv"), "w", encoding="utf-8") as fh:
        fh.write("t,x,y,z,yaw,tilt_deg,pitch,roll,throttle,yaw_stick,manual,armed,mode\n")
        for r in ROWS:
            fh.write(",".join(f"{v:.6g}" for v in r[:10]) + f",{r[10]},{r[11]},{r[12]}\n")
    print(f"report: {path}", flush=True)
    return n_pass == n_expected and len(RESULTS) == n_expected


if __name__ == "__main__":
    ok = False
    try:
        rc_cfg = rc_input.load_config(args.rc_config)
        cfg, info = build(rc_cfg)
        log(f"드론 설정: {cfg['path']}, 조종기 설정: {rc_cfg['path']}")
        tests(cfg, info)
    except Exception as e:  # 중간에 멈춰도 리포트는 남긴다
        import traceback
        log("")
        log(f"[ERROR] {type(e).__name__}: {e}")
        log(traceback.format_exc())
    finally:
        ok = write_report(6)
        simulation_app.close(exit_code=0 if ok else 1)
    sys.exit(0 if ok else 1)
