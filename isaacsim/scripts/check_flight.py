#!/usr/bin/env python3
# =============================================================
# check_flight.py  (docs/PLAN.md 2-2)
#
# 드론 비행(drone_flight.DroneFlight, Pegasus 방식: 로터 4 개 추력 + 기하 제어기) 시험. 드론 설정 backend 와 무관하게 기하 제어기 (PX4 는 check_px4.py). 씬: ground plane + 드론 (로봇 없음)
#   F1 정지 호버 (static): 목표 = 시작 위치, FLY_SEC → 처음 SETTLE_SEC 뒤 위치 오차·기울기, 로터 ω 가 호버 값 근처
#      (무게중심이 조금만 치우쳐도 자세 적분항이 없는 Pegasus 제어기는 몸체가 살짝 기운 채 버티며 옆으로 밀리고,
#       위치 적분항(Ki/Kp ≈ 0.15 /s)이 수 초에 걸쳐 없앤다 → 처음 SETTLE_SEC 는 수렴 구간으로 제외)
#   F2 계단: 목표를 +x 5 cm, 그다음 +z 5 cm → overshoot < 15%, STEP_SEC 뒤 오차 < 2 mm. 2 mm 안 정착 시간·이동 중 기울기는 기록
#      (실물처럼 기울어서 이동)
#      (계단에서는 적분이 쌓여 약 10% overshoot 후 느리게 돌아옴 = Pegasus PID 특성. static·hover 에는 계단이 없음)
#   F3 hover: 목표에 사인파 합 (seed) → 추적 오차, 실제 흔들림 범위
#   F4 모터 정지 (release): ω = 0 → 처음 RELEASE_FIT_SEC 동안 수직 가속도 ≈ −g, 프로펠러 표시 정지, 바닥에 떨어져도 NaN 없음
#   --prop-spin on: 프로펠러 관절을 풀고 보여 주기용으로 돌림 → 비행 중 관절 속도 = ±prop_visual_speed, 정지 후 0 도 확인
#
# 실행:
#   ~/isaacsim/python.sh ~/data_collection_ur5_gripper/isaacsim/scripts/check_flight.py --headless [--prop-spin on]
#   GUI 로 보기: --headless 빼고 --view [--mode hover] [--prop-spin on] [--release-after 8]
#     (시험 없이 공중 1 m 에서 비행 → release-after 초 뒤 모터 정지 → 낙하, 실제 시간 속도. 창을 닫으면 종료)
#
# 시간: simulation_app.update() 한 번에 physics 가 여러 step (6.1.0 기본 1/60 s = 2 step) 진행될 수 있어
#   시간은 비행 콜백이 센 sim 시간(DroneFlight.t)으로 잰다 (grasp_tests·tune_drives 와 같은 방식)
# 결과: 콘솔과 isaacsim/reports/check_flight_<날짜시간>.txt. 하나라도 FAIL 이면 종료 코드 1.
# =============================================================

import argparse
import datetime
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_scene as ts  # noqa: E402

parser = argparse.ArgumentParser(description="드론 비행 시험 (PLAN 2-2)")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--drone-config", default=None, help="드론 설정 yaml (기본 config/drone_iris.yaml)")
parser.add_argument("--config", default=ts.DEFAULT_CONFIG, help="물체 재질(contact.object_material)을 읽을 yaml")
parser.add_argument("--prop-spin", choices=("on", "off"), default="off",
                    help="프로펠러를 보여 주기용으로 돌림 (on: 프로펠러 관절을 고정하지 않음)")
parser.add_argument("--seed", type=int, default=0, help="hover 사인파 seed")
parser.add_argument("--report-dir", default=ts.REPORT_DIR)
parser.add_argument("--view", action="store_true", help="GUI: 시험 대신 비행 → 모터 정지 장면을 실제 시간으로 보여 줌")
parser.add_argument("--mode", default="hover", choices=("static", "hover", "trajectory"), help="--view 의 비행 모드")
parser.add_argument("--release-after", type=float, default=8.0, help="--view: 모터 정지까지 시간 [s]")
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

START = (0.0, 0.0, 1.0)     # m, 시작 위치 (body)
# Pegasus PID 는 위치 적분이 느려(Kp/Ki ≈ 6.7 s) 시작 처짐·무게중심 치우침에 따른 오차가 수십 초에 걸쳐 0 으로 감 (2026-10-01)
FLY_SEC, SETTLE_SEC = 30.0, 20.0
STEP = 0.05                 # m, F2 계단
STEP_SEC = 12.0
HOVER_SEC, HOVER_SKIP = 20.0, 2.0   # hover 판정에서 처음 HOVER_SKIP 은 제외 (목표가 static → hover 로 바뀌는 순간)
RELEASE_FIT_SEC = 0.15
FALL_SEC = 2.0
CRIT = {"static_err_mm": 2.0, "static_tilt_deg": 0.5, "hover_omega_pct": 5.0, "overshoot_pct": 15.0, "final_err_mm": 2.0,
        "settle_band_mm": 2.0, "hover_rms_mm": 5.0, "fall_acc_pct": 5.0, "prop_vel_tol": 1.0}


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


def build():
    cfg = dr.load_drone_config(args.drone_config or dr.DEFAULT_DRONE_CONFIG)
    config = ts.load_config(args.config)
    stage_utils.create_new_stage()
    stage_utils.set_stage_up_axis("Z")
    stage_utils.set_stage_units(meters_per_unit=1.0, kilograms_per_unit=1.0)
    stage = stage_utils.get_current_stage(backend="usd")
    stage_utils.define_prim("/World", "Xform")
    if not SimulationManager.switch_physics_engine("physx"):
        raise RuntimeError("PhysX 로 전환하지 못했습니다")
    SimulationManager.setup_simulation(dt=ts.PHYSICS_DT)
    GroundPlane("/World/GroundPlane")
    if not args.headless:
        DomeLight("/World/DomeLight").set_intensities(1000)
    ts.define_physics_material(stage, ts.OBJECT_MATERIAL_PATH, config["contact"]["object_material"])
    info = dr.add_drone(stage, cfg, position=START, prop_spin=args.prop_spin == "on")
    app_utils.play()
    simulation_app.update()
    return cfg, info


class Log:
    def __init__(self):
        self.rows = []

    def add(self, f):
        p, q = f._body_pose()
        Rm = fl.quat_to_R(q)
        tilt = math.degrees(math.acos(np.clip(Rm[2, 2], -1, 1)))
        pr = f.ref(f.t)[0] if f.armed else np.full(3, np.nan)
        pv = f.prop_velocities()
        self.rows.append(dict(t=f.t, p=p.copy(), ref=pr, tilt=tilt, omega=f.omega.copy(), forces=f.forces.copy(),
                              prop=None if pv is None else pv.copy(), cmd=f.prop_cmd.copy()))

    def arr(self, key, t0=-1.0):
        return np.array([r[key] for r in self.rows if r["t"] >= t0])


def run(f, sec, log=None):
    t_end = f.t + sec
    while f.t < t_end - 1e-9:
        simulation_app.update()
        f.check()
        if log is not None:
            log.add(f)
        p = f._body_pose()[0]
        if not np.all(np.isfinite(p)):
            raise RuntimeError(f"드론 위치 NaN (t = {f.t:.3f} s)")


def prop_check(f, log, t0, why):
    """비행 중 프로펠러 관절 속도가 명령(±visual_speed)과 같은지."""
    if f.spin is None:
        return
    prop, cmd = log.arr("prop", t0), log.arr("cmd", t0)
    err = float(np.abs(prop - cmd).max())
    R.log(f"  프로펠러 관절 속도 {np.round(prop[-1], 2).tolist()} rad/s (명령 {np.round(cmd[-1], 1).tolist()}), 최대 차이 {err:.3f}")
    if err > CRIT["prop_vel_tol"]:
        why.append(f"프로펠러 속도 차이 {err:.2f} rad/s")


def tests(cfg, info):
    f = fl.DroneFlight(cfg, info, mode="static", target=START, backend="geometric")
    R.log(f"PhysX 질량 {f.mass:.4f} kg, 관성 대각 (kg·m², 전체 무게중심 둘레) {np.round(np.diag(f.inertia), 6).tolist()}")
    sc = "기체 비율로 환산" if cfg["flight"]["controller"]["scale_to_airframe"] else "그대로"
    R.log(f"게인 (Pegasus 값 {sc}) Kp {np.round(f.ctrl.Kp, 3).tolist()}, Kd {np.round(f.ctrl.Kd, 3).tolist()}, "
          f"Ki {np.round(f.ctrl.Ki, 3).tolist()}, Kr {np.round(f.ctrl.Kr, 4).tolist()}, Kw {np.round(f.ctrl.Kw, 4).tolist()}, "
          f"Kw·dt/I {np.round(f.ctrl.Kw * ts.PHYSICS_DT / np.diag(f.inertia), 3).tolist()}")
    R.log(f"호버 로터 각속도 (계산) {f.hover_omega():.1f} rad/s, 최대 {f.w_max:.0f}. 프로펠러 표시 {'켬' if f.spin is not None else '끔'}")
    f.start()
    f.arm()

    # F1
    R.section("F1", f"정지 호버 static ({FLY_SEC:.0f} s, 처음 {SETTLE_SEC:.0f} s 수렴 구간 제외)")
    log = Log()
    run(f, FLY_SEC, log)
    p, ref = log.arr("p", SETTLE_SEC), log.arr("ref", SETTLE_SEC)
    err = np.linalg.norm(p - ref, axis=1) * 1000
    tilt = log.arr("tilt", SETTLE_SEC)
    om = log.arr("omega", SETTLE_SEC)
    dev = abs(om.mean() / f.hover_omega() - 1) * 100
    R.log(f"  위치 오차 최대 {err.max():.3f} mm, 평균 {err.mean():.3f} mm, 기울기 최대 {tilt.max():.4f}°")
    R.log(f"  로터 ω 평균 {np.round(om.mean(axis=0), 1).tolist()} rad/s (계산 호버 값과 {dev:.2f}% 차이)")
    why = []
    if err.max() > CRIT["static_err_mm"]:
        why.append(f"오차 {err.max():.2f} mm")
    if tilt.max() > CRIT["static_tilt_deg"]:
        why.append(f"기울기 {tilt.max():.3f}°")
    if dev > CRIT["hover_omega_pct"]:
        why.append(f"ω {dev:.1f}%")
    prop_check(f, log, SETTLE_SEC, why)
    R.result("F1", "정지 호버", not why, "; ".join(why))

    # F2
    R.section("F2", f"계단 +{STEP * 100:.0f} cm (x, 그다음 z), 각 {STEP_SEC:.0f} s")
    why = []
    for axis in (0, 2):
        p_start = f._body_pose()[0]
        target = f.ref.p0.copy()
        target[axis] += STEP
        f.ref.p0 = target
        t0 = f.t
        log = Log()
        run(f, STEP_SEC, log)
        t = log.arr("t") - t0
        x = log.arr("p")[:, axis] - p_start[axis]
        over = (x.max() - STEP) / STEP * 100
        out = np.abs(x - STEP) > CRIT["settle_band_mm"] / 1000
        last_out = np.nonzero(out)[0]
        settle = 0.0 if not out.any() else (float(t[last_out[-1] + 1]) if last_out[-1] + 1 < len(t) else float("inf"))
        tilt = log.arr("tilt").max()
        R.log(f"  {'xyz'[axis]}: overshoot {over:.2f}%, {CRIT['settle_band_mm']:.0f} mm 안 정착 {settle:.2f} s, "
              f"이동 중 기울기 최대 {tilt:.2f}°, 최종 오차 {abs(x[-1] - STEP) * 1000:.3f} mm")
        if over > CRIT["overshoot_pct"]:
            why.append(f"{'xyz'[axis]} overshoot {over:.1f}%")
        if abs(x[-1] - STEP) * 1000 > CRIT["final_err_mm"]:
            why.append(f"{'xyz'[axis]} {STEP_SEC:.0f} s 뒤 오차 {abs(x[-1] - STEP) * 1000:.2f} mm")
    R.result("F2", "계단", not why, "; ".join(why))

    # F3
    R.section("F3", f"hover (seed {args.seed}, {HOVER_SEC:.0f} s)")
    f.ref = fl.Reference("hover", f.ref.p0, cfg["flight"]["hover"], args.seed)
    f.t = 0.0
    log = Log()
    run(f, HOVER_SEC, log)
    p, ref = log.arr("p", HOVER_SKIP), log.arr("ref", HOVER_SKIP)
    e = (p - ref) * 1000
    rms = float(np.sqrt((np.linalg.norm(e, axis=1) ** 2).mean()))
    R.log(f"  사인파 주파수 (Hz) x {np.round(f.ref.f[0], 3).tolist()}, y {np.round(f.ref.f[1], 3).tolist()}, "
          f"z {np.round(f.ref.f[2], 3).tolist()}")
    R.log(f"  목표 흔들림 범위 (mm) {np.round(np.ptp(ref, axis=0) * 1000, 1).tolist()}, 실제 {np.round(np.ptp(p, axis=0) * 1000, 1).tolist()}")
    R.log(f"  추적 오차 RMS {rms:.2f} mm, 최대 {np.linalg.norm(e, axis=1).max():.2f} mm, 기울기 최대 {log.arr('tilt', HOVER_SKIP).max():.2f}°, 샘플 {len(p)}")
    why = [] if rms <= CRIT["hover_rms_mm"] else [f"RMS {rms:.2f} mm"]
    prop_check(f, log, HOVER_SKIP, why)
    R.result("F3", "hover", not why, "; ".join(why))

    # F4
    R.section("F4", "모터 정지 (release)")
    f.release()
    log = Log()
    run(f, FALL_SEC, log)
    t, z = log.arr("t") - f.t_release, log.arr("p")[:, 2]
    m = t <= RELEASE_FIT_SEC
    acc = 2 * np.polyfit(t[m], z[m], 2)[0]
    om = log.arr("omega")
    R.log(f"  처음 {RELEASE_FIT_SEC} s 수직 가속도 {acc:.3f} m/s² (−g = {-fl.G}), 정지 후 ω 최대 {om.max():.3f}, "
          f"최종 높이 {z[-1] * 1000:.1f} mm")
    why = []
    if abs(acc + fl.G) > CRIT["fall_acc_pct"] / 100 * fl.G:
        why.append(f"가속도 {acc:.2f}")
    if om.max() > 0:
        why.append("ω 가 0 이 아님")
    if f.spin is not None:
        pv = np.abs(log.arr("prop")[-1]).max()
        R.log(f"  정지 후 프로펠러 관절 속도 최대 {pv:.4f} rad/s")
        if pv > CRIT["prop_vel_tol"]:
            why.append(f"프로펠러가 멈추지 않음 ({pv:.2f})")
    R.result("F4", "모터 정지", not why, "; ".join(why))
    f.stop()


def view(cfg, info):
    from isaacsim.core.rendering_manager import ViewportManager

    ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[0.9, -0.9, START[2] + 0.4], target=list(START))
    f = fl.DroneFlight(cfg, info, mode=args.mode, seed=args.seed, target=START, backend="geometric")
    f.start()
    f.arm()
    print(f"[view] {args.mode} 비행 → {args.release_after:.0f} s 뒤 모터 정지. 창을 닫으면 종료", flush=True)
    released = False
    wall0, sim0 = time.monotonic(), f.t
    while simulation_app.is_running():
        simulation_app.update()
        f.check()
        if not released and f.t >= args.release_after:
            f.release()
            released = True
            print(f"[view] 모터 정지 (t = {f.t:.2f} s)", flush=True)
        ahead = (f.t - sim0) - (time.monotonic() - wall0)   # 실제 시간 속도로
        if ahead > 0:
            time.sleep(ahead)
    f.stop()


def write_report(n_expected):
    R.log("")
    R.log("=" * 50)
    for no, title, tag in R.results:
        R.log(f"  {no}. {title:12s} {tag}")
    n_pass = sum(t == "PASS" for _, _, t in R.results)
    R.log(f"  {n_pass}/{n_expected} PASS")
    os.makedirs(args.report_dir, exist_ok=True)
    path = os.path.join(args.report_dir, f"check_flight_{datetime.datetime.now():%Y%m%d_%H%M%S}.txt")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(R.lines) + "\n")
    print(f"report: {path}", flush=True)
    return n_pass == n_expected and len(R.results) == n_expected


if __name__ == "__main__":
    ok = False
    try:
        cfg, info = build()
        R.log(f"드론 설정: {cfg['path']}, prop_spin {args.prop_spin}")
        if args.view and not args.headless:
            view(cfg, info)
            ok = True
        else:
            tests(cfg, info)
    except Exception as e:  # 중간에 멈춰도 리포트는 남긴다
        import traceback
        R.log("")
        R.log(f"[ERROR] {type(e).__name__}: {e}")
        R.log(traceback.format_exc())
    finally:
        if not args.view or args.headless:
            ok = write_report(4)
        simulation_app.close(exit_code=0 if ok else 1)
    sys.exit(0 if ok else 1)
