#!/usr/bin/env python3
# =============================================================
# check_px4.py  (docs/PLAN.md 2단계 2번 P-2·P-3·P-6)
#
# PX4 SITL 드론 비행 시험 (드론 설정의 flight.backend 와 무관하게 px4). 씬: ground plane + 드론 (로봇 없음). 바닥에서 시작 → PX4 Offboard 로 이륙
#   X0 연결: PX4 실행 → heartbeat → lockstep (모터 명령 수신) → 명령 링크 연결 → arm (PX4 사전 점검 통과)
#   X1 이륙: 목표 (시작 + 위로 takeoff_alt) 까지 PX4 추정 위치가 5 cm 안에 TAKEOFF_SEC 안에 들어옴 (PX4 가 도착했다고 봄).
#      실제 위치 − 목표 (= 추정 오차·표류, flow 는 절대 위치가 없어 이륙 중 밀린 만큼 남음) 는 기록
#   X2 호버 HOVER_SEC: 실제 위치 흔들림 (목표 기준 RMS·최대, 축별 범위), 기울기, PX4 추정 위치 − 실제 위치 (기록, 판정 없음)
#   X3 계단 +STEP x: overshoot, 2 cm 안 정착 시간, 최종 오차 (기록, 판정 없음)
#   X4 kill (공중 강제 정지): disarm 뒤 모터가 멈추면(모터 지연이 있으면 5 τ_down 뒤) RELEASE_FIT_SEC 동안 수직 가속도 ≈ −g,
#      0.5 s 뒤 모터 ω < 1 rad/s (모터 지연이 있으면 지수적으로 감속)
#   (X5 반복 차이, X6 flow 표류는 P-3 이후)
#
# 실행:
#   ~/isaacsim/python.sh ~/data_collection_ur5_gripper/isaacsim/scripts/check_px4.py --headless \
#       [--drone-config config/drone_iris_pegasus.yaml] [--physics-hz 250]
#   GUI 로 보기: --headless 빼고 --view (이륙 → 호버 → kill 장면, 실제 시간 속도. 창을 닫으면 종료)
# 결과: 콘솔, isaacsim/reports/check_px4_<시각>.txt, 시계열 csv, PX4 콘솔·ulog (reports/px4_<시각>/)
# =============================================================

import argparse
import datetime
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_scene as ts  # noqa: E402

parser = argparse.ArgumentParser(description="PX4 SITL 드론 비행 시험 (PLAN 2단계 2번)")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--drone-config", default=os.path.join(ts.CONFIG_DIR, "drone_iris_pegasus.yaml"))
parser.add_argument("--config", default=ts.DEFAULT_CONFIG, help="물체 재질(contact.object_material)을 읽을 yaml")
parser.add_argument("--physics-hz", type=float, default=250.0, help="physics 주기 (Pegasus 250, 로봇 씬 120)")
parser.add_argument("--start-z", type=float, default=0.10, help="드론 시작 높이 (body 원점, 바닥에 내려앉음)")
parser.add_argument("--takeoff-alt", type=float, default=1.0, help="이륙 목표 높이 (시작 위치 위로) [m]")
parser.add_argument("--prop-spin", choices=("on", "off"), default="off")
parser.add_argument("--body-inertia", type=float, nargs=3, default=None, metavar=("IXX", "IYY", "IZZ"),
                    help="진단용: 드론 body 링크 관성 대각 [kg·m²] 덮어쓰기 (기본: USD/PhysX 값)")
parser.add_argument("--position-source", default=None, choices=("mocap", "flow", "gps"),
                    help="위치 정보 방식 (기본: px4_sitl.yaml 의 position_source)")
parser.add_argument("--motor-lag", choices=("on", "off"), default=None, help="비교용: px4.motor_dynamics.enabled 덮어쓰기")
parser.add_argument("--sensor-compat", choices=("on", "off"), default=None,
                    help="비교용: 센서 pegasus_compat 덮어쓰기 (on = Pegasus 코드 그대로)")
parser.add_argument("--px4-param", action="append", default=[], metavar="NAME=VALUE",
                    help="비교용: PX4 파라미터 덮어쓰기 (여러 번 가능, 드론 설정 px4.params 보다 우선)")
parser.add_argument("--seed", type=int, default=0, help="센서 잡음 seed")
parser.add_argument("--report-dir", default=ts.REPORT_DIR)
parser.add_argument("--view", action="store_true", help="GUI: 시험 대신 이륙 → 호버 → kill 을 실제 시간으로")
parser.add_argument("--kill-after", type=float, default=20.0, help="--view: arm 뒤 kill 까지 [s]")
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

CONNECT_SEC = 60.0          # sim s, arm 까지 (PX4 부팅·EKF 수렴·사전 점검)
TAKEOFF_SEC, TAKEOFF_TOL = 30.0, 0.05
HOVER_SEC = 30.0
STEP, STEP_SEC, STEP_BAND = 0.10, 15.0, 0.02
RELEASE_FIT_SEC, FALL_SEC, FALL_ACC_PCT = 0.15, 1.5, 5.0


class Report:
    def __init__(self):
        self.lines, self.results = [], []

    def log(self, msg=""):
        print(msg, flush=True)
        self.lines.append(msg)

    def section(self, no, title):
        self.log("")
        self.log(f"[{no}] {title}")

    def result(self, no, title, ok, why=""):
        self.results.append((no, title, "PASS" if ok else "FAIL"))
        self.log(f"  => {'PASS' if ok else 'FAIL'}" + (f" ({why})" if why else ""))


R = Report()
HIST = []           # PX4 모터 명령 (매 메시지)


def build():
    cfg = dr.load_drone_config(args.drone_config)
    if args.motor_lag is not None:
        cfg["px4"]["motor_dynamics"]["enabled"] = args.motor_lag == "on"
    if args.sensor_compat is not None:
        cfg["px4"]["sensor_overrides"] = {**(cfg["px4"]["sensor_overrides"] or {}), "pegasus_compat": args.sensor_compat == "on"}
    for kv in args.px4_param:
        k, v = kv.split("=", 1)
        cfg["px4"]["params"] = {**(cfg["px4"]["params"] or {}), k.strip(): float(v) if "." in v else int(v)}
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
    info = dr.add_drone(stage, cfg, position=(0.0, 0.0, args.start_z), prop_spin=args.prop_spin == "on")
    if args.body_inertia is not None:
        from pxr import Gf, UsdPhysics

        UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(info["body_path"])).CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*args.body_inertia))
    app_utils.play()
    simulation_app.update()
    return cfg, info


class Log:
    def __init__(self):
        self.rows = []

    def add(self, f):
        p, q = f._body_pose()
        Rm = fl.quat_to_R(q)
        c = f.cmd
        self.rows.append(dict(
            t=f.t, p=p.copy(), tilt=math.degrees(math.acos(np.clip(Rm[2, 2], -1, 1))),
            est=np.full(3, np.nan) if c.estimate_enu is None else c.estimate_enu.copy(),
            target=np.full(3, np.nan) if c.target is None else c.target.copy(),
            omega=f.omega.copy(), u=f.px4.u.copy(), armed=f.px4.armed, mode=c.mode or "",
            ground=np.nan if getattr(f, "ground", None) is None else f.ground))

    def arr(self, key, t0=-1e9):
        return np.array([r[key] for r in self.rows if r["t"] >= t0])

    def write_csv(self, path):
        with open(path, "w", encoding="utf-8") as fh:
            fh.write("t,x,y,z,tilt_deg,est_x,est_y,est_z,tgt_x,tgt_y,tgt_z,w0,w1,w2,w3,u0,u1,u2,u3,ground,armed,mode\n")
            for r in self.rows:
                vals = [r["t"], *r["p"], r["tilt"], *r["est"], *r["target"], *r["omega"], *r["u"], r["ground"]]
                fh.write(",".join(f"{v:.6g}" for v in vals) + f",{int(r['armed'])},{r['mode']}\n")


def run(f, sec, log=None, until=None):
    """sim 시간 sec 동안 (until(f) 가 참이면 먼저 끝). 끝난 이유가 until 이면 True."""
    t_end = f.t + sec
    while f.t < t_end - 1e-9:
        simulation_app.update()
        f.check()
        if log is not None:
            log.add(f)
        if not np.all(np.isfinite(f._body_pose()[0])):
            raise RuntimeError(f"드론 위치 NaN (t = {f.t:.3f} s)")
        if until is not None and until(f):
            return True
    return False


def statustext(f, since=0.0):
    for t, sev, text in f.cmd.messages:
        if t >= since:
            R.log(f"    PX4 [{t:7.2f} s, sev {sev}] {text}")


def connect(f, log):
    R.section("X0", f"연결 (PX4 실행 → lockstep → 명령 링크 → arm, {CONNECT_SEC:.0f} s 안)")
    f.start()
    wall0 = time.monotonic()
    ok = run(f, 5.0, until=lambda f: f.px4.status == "lockstep")
    R.log(f"  lockstep 시작 sim {f.t:.2f} s (실제 {time.monotonic() - wall0:.1f} s), 상태 {f.px4.status}")
    if not ok:
        # PX4 부팅은 실제 시간이 걸림 → heartbeat 전에는 sim 이 PX4 를 기다리지 않음
        ok = run(f, CONNECT_SEC, until=lambda f: f.px4.status == "lockstep")
    if not ok:
        R.result("X0", "연결", False, f"lockstep 안 됨 (상태 {f.px4.status})")
        return False
    t_lock = f.t
    run(f, CONNECT_SEC, until=lambda f: f.cmd.connected)
    R.log(f"  명령 링크 heartbeat: {f.cmd.connected}, 모드 {f.cmd.mode}")
    run(f, 5.0)                                   # EKF 초기화 여유
    target = f._body_pose()[0] + np.array([0.0, 0.0, args.takeoff_alt])
    f.arm(target)
    t_arm_req = f.t
    ok = run(f, CONNECT_SEC, log, until=lambda f: f.px4.armed)
    R.log(f"  arm 요청 sim {t_arm_req:.2f} s → armed {f.px4.armed} (sim {f.t:.2f} s), 모드 {f.cmd.mode}, "
          f"모터 명령 수 {f.px4.n_actuator}, lockstep 뒤 {f.t - t_lock:.1f} s")
    statustext(f)
    R.result("X0", "연결", ok, "" if ok else "arm 되지 않음 (PX4 메시지 참고)")
    return ok


def tests(cfg, info):
    f = fl.DroneFlight(cfg, info, mode="static", seed=args.seed, backend="px4", report_dir=args.report_dir,
                         position_source=args.position_source)
    R.log(f"PhysX 질량 {f.mass:.4f} kg, 관성 대각 {np.round(np.diag(f.inertia), 6).tolist()}, 호버 ω (계산) {f.hover_omega():.1f} rad/s"
          f" → u ≈ {(f.hover_omega() - f.px4.zero_armed[0]) / f.px4.scaling[0] - f.px4.offset[0]:.3f}")
    R.log(f"physics {args.physics_hz:.0f} Hz, PX4 airframe {cfg['px4']['airframe']}, 센서 pegasus_compat "
          f"{f.sensors_cfg['pegasus_compat']}, 모터 지연 τ {f.motor_tau}, PX4 실행 폴더 {f.px4.run_dir}")
    R.log(f"위치 정보 {f.position_source}, PX4 파라미터 덮어쓰기 {f.px4_params}")
    log = Log()
    f.px4.record = True
    wall0 = time.monotonic()
    try:
        if not connect(f, log):
            return f, log
        target = f.cmd.target.copy()

        # X1
        R.section("X1", f"이륙 → 목표 {np.round(target, 3).tolist()} (PX4 추정 {TAKEOFF_TOL * 100:.0f} cm 안, {TAKEOFF_SEC:.0f} s 안)")
        t0 = f.t

        def arrived(f):
            e = f.cmd.estimate_enu
            return f.cmd.mode == "OFFBOARD" and e is not None and np.linalg.norm(e - target) < TAKEOFF_TOL

        ok = run(f, TAKEOFF_SEC, log, until=arrived)
        p_now = f._body_pose()[0]
        R.log(f"  도착 {f.t - t0:.2f} s, PX4 추정 − 목표 {np.linalg.norm(f.cmd.estimate_enu - target) * 1000:.1f} mm, "
              f"실제 − 목표 {np.round((p_now - target) * 1000, 1).tolist()} mm, 모드 {f.cmd.mode}")
        R.result("X1", "이륙", ok, "" if ok else "목표에 못 감")
        if not ok:
            statustext(f, t0)
            return f, log
        run(f, 5.0, log)

        # X2
        R.section("X2", f"호버 {HOVER_SEC:.0f} s")
        t0 = f.t
        run(f, HOVER_SEC, log)
        p, tgt, est = log.arr("p", t0), log.arr("target", t0), log.arr("est", t0)
        e = (p - tgt) * 1000
        en = np.linalg.norm(e, axis=1)
        ee = np.linalg.norm(est - p, axis=1) * 1000
        om = log.arr("omega", t0)
        R.log(f"  실제 위치 − 목표: RMS {np.sqrt((en ** 2).mean()):.1f} mm, 최대 {en.max():.1f} mm, 평균 (mm) {np.round(e.mean(axis=0), 1).tolist()}")
        R.log(f"  흔들림 범위 (축별, mm) {np.round(np.ptp(p, axis=0) * 1000, 1).tolist()}, 표준편차 {np.round(p.std(axis=0) * 1000, 2).tolist()}")
        R.log(f"  기울기 최대 {log.arr('tilt', t0).max():.2f}°, PX4 추정 − 실제: RMS {np.sqrt(np.nanmean(ee ** 2)):.1f} mm, 최대 {np.nanmax(ee):.1f} mm")
        R.log(f"  로터 ω 평균 {np.round(om.mean(axis=0), 1).tolist()} rad/s (계산 호버 {f.hover_omega():.1f}), "
              f"표준편차 {np.round(om.std(axis=0), 1).tolist()}")
        R.result("X2", "호버 (기록)", True)

        # X3
        R.section("X3", f"계단 +{STEP * 100:.0f} cm x, {STEP_SEC:.0f} s")
        x0 = f._body_pose()[0][0]
        f.cmd.goto(target + np.array([STEP, 0.0, 0.0]))
        t0 = f.t
        run(f, STEP_SEC, log)
        t, x = log.arr("t", t0) - t0, log.arr("p", t0)[:, 0] - x0
        over = (x.max() - STEP) / STEP * 100
        out = np.nonzero(np.abs(x - STEP) > STEP_BAND)[0]
        settle = 0.0 if len(out) == 0 else (float(t[out[-1] + 1]) if out[-1] + 1 < len(t) else float("inf"))
        R.log(f"  overshoot {over:.1f}%, {STEP_BAND * 100:.0f} cm 안 정착 {settle:.2f} s, 최종 오차 {abs(x[-1] - STEP) * 1000:.1f} mm, "
              f"기울기 최대 {log.arr('tilt', t0).max():.2f}°")
        R.result("X3", "계단 (기록)", True)

        # X4
        R.section("X4", "kill (공중 강제 정지)")
        f.release()
        t0 = f.t
        run(f, FALL_SEC, log)
        t, z = log.arr("t", t0) - t0, log.arr("p", t0)[:, 2]
        armed = log.arr("armed", t0)
        t_off = t[np.argmax(~armed)] if (~armed).any() else float("nan")
        spin_down = 5 * f.motor_tau[1] if f.motor_tau is not None else 2.0 / args.physics_hz
        m = (t >= t_off + spin_down) & (t <= t_off + spin_down + RELEASE_FIT_SEC)
        acc = 2 * np.polyfit(t[m], z[m], 2)[0] if m.sum() > 5 else float("nan")
        om_after = log.arr("omega", t0 + t_off + 0.5).max() if np.isfinite(t_off) else float("nan")
        R.log(f"  disarm 까지 {t_off * 1000:.0f} ms, 모터 감속 {spin_down * 1000:.0f} ms 뒤 {RELEASE_FIT_SEC} s 수직 가속도 {acc:.3f} m/s² "
              f"(−g = {-fl.G}), 0.5 s 뒤 ω 최대 {om_after:.3f}")
        why = []
        if not np.isfinite(acc) or abs(acc + fl.G) > FALL_ACC_PCT / 100 * fl.G:
            why.append(f"가속도 {acc:.2f}")
        if not (om_after < 1.0):
            why.append(f"ω 가 0 으로 가지 않음 ({om_after:.2f})")
        R.result("X4", "kill", not why, "; ".join(why))
        statustext(f, t0)
    finally:
        wall = time.monotonic() - wall0
        R.log("")
        HIST.extend(f.px4.history)
        R.log(f"sim {f.t:.1f} s / 실제 {wall:.1f} s (실시간 비율 {f.t / wall:.2f}), 모터 명령 {f.px4.n_actuator} 개")
        f.stop()
    return f, log


def view(cfg, info):
    from isaacsim.core.rendering_manager import ViewportManager

    ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[2.5, -2.5, 1.8], target=[0.0, 0.0, 0.8])
    f = fl.DroneFlight(cfg, info, mode="static", seed=args.seed, backend="px4", report_dir=args.report_dir,
                         position_source=args.position_source)
    log = Log()
    if connect(f, log):
        t_arm = f.t
        wall0, sim0 = time.monotonic(), f.t
        print(f"[view] 이륙·호버 → arm 뒤 {args.kill_after:.0f} s 에 kill. 창을 닫으면 종료", flush=True)
        killed = False
        while simulation_app.is_running():
            simulation_app.update()
            f.check()
            if not killed and f.t - t_arm >= args.kill_after:
                f.release()
                killed = True
                print(f"[view] kill (t = {f.t:.2f} s)", flush=True)
            ahead = (f.t - sim0) - (time.monotonic() - wall0)
            if ahead > 0:
                time.sleep(ahead)
    f.stop()


def write_report(n_expected, log):
    R.log("")
    R.log("=" * 50)
    for no, title, tag in R.results:
        R.log(f"  {no}. {title:12s} {tag}")
    n_pass = sum(t == "PASS" for _, _, t in R.results)
    R.log(f"  {n_pass}/{n_expected} PASS")
    os.makedirs(args.report_dir, exist_ok=True)
    stamp = f"{datetime.datetime.now():%Y%m%d_%H%M%S}"
    path = os.path.join(args.report_dir, f"check_px4_{stamp}.txt")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(R.lines) + "\n")
    if log is not None and log.rows:
        log.write_csv(os.path.join(args.report_dir, f"check_px4_{stamp}.csv"))
    if HIST:
        np.savetxt(os.path.join(args.report_dir, f"check_px4_{stamp}_actuator.csv"), np.array(HIST, dtype=float), delimiter=",",
                   header="t,armed,u0,u1,u2,u3", comments="", fmt="%.6g")
    print(f"report: {path}", flush=True)
    return n_pass == n_expected and len(R.results) == n_expected


if __name__ == "__main__":
    ok, log = False, None
    try:
        cfg, info = build()
        R.log(f"드론 설정: {cfg['path']}")
        if args.view and not args.headless:
            view(cfg, info)
            ok = True
        else:
            _, log = tests(cfg, info)
    except Exception as e:  # 중간에 멈춰도 리포트는 남긴다
        import traceback
        R.log("")
        R.log(f"[ERROR] {type(e).__name__}: {e}")
        R.log(traceback.format_exc())
    finally:
        if not args.view or args.headless:
            ok = write_report(5, log)
        simulation_app.close(exit_code=0 if ok else 1)
    sys.exit(0 if ok else 1)
