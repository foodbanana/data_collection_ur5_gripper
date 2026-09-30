#!/usr/bin/env python3
# =============================================================
# tune_drives.py  (docs/PLAN.md 1-5)
#
# drive 설정 파일(들)로 A~D 시험을 돌리고 나란히 비교한다. 테스트 씬은 1-4 와 같다 (fixed base, z = 0.762 m).
#   A. 스텝 응답: 홈 자세에서 팔 관절별 +0.05 rad (판정), +0.2·+1.0 rad (참고: 50 Hz 텔레옵 명령 사이 변화는 최대 0.063 rad)
#   B. 중력 유지: gravity_worst_cases.yaml(--cases)의 관절별 최악 자세 + 드론 1.5 kg (무게중심 공구 축에서 3·5 cm) → 정상상태 오차
#                드론 무게는 그리퍼 base 링크에 매 스텝 외력으로 가한다 (gravity_ff 가 보상하지 않음). 씬은 설정마다 한 번. 바닥 없음
#   C. 텔레옵식 추종: 50 Hz zero-order hold 명령으로 부드러운 궤적 → 추종 오차와 지연
#   D. 그리퍼 열림·닫힘 시간 (정의: 명령 순간부터 위치 변화가 멈출 때(최종값의 2% 이내)까지). 목표값 이동 방식
#   E. 홈 자세에서 가만히 3 초: 명령 위치에서 벗어남·떨림(peak-to-peak), 최대 관절 속도, NaN, physics 스텝당 계산 시간
#   F. 빈손으로 그리퍼 끝까지 닫기: 닫힌 뒤 2 초 동안 손가락 떨림 (self-collision 을 켜면 손가락끼리 닿은 상태)
#
# 설정 덮어쓰기: 'drive_gains.yaml@articulation.self_collision=true' 처럼 쓰면 그 값만 바꿔 비교 (test_scene.load_config)
#
# 설정: A~C 는 --configs (기본: 기본·ff·공식 세 개), D 는 --d-configs (기본: drive_gains.yaml)
# GUI 로 보기: --headless 를 빼고 --realtime 을 주면 sim 시간을 실제 시간에 맞춰 천천히 돌린다
#
# 실행:
#   ~/isaacsim/python.sh ~/data_collection_ur5_gripper/isaacsim/scripts/tune_drives.py --headless
#   GUI: ~/isaacsim/python.sh .../tune_drives.py --tests ACD --configs isaacsim/config/drive_gains.yaml --realtime
#
# 단위: rad (리포트의 각도 오차는 degree)
# =============================================================

import argparse
import datetime
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_scene as ts  # noqa: E402

parser = argparse.ArgumentParser(description="drive 튜닝 시험 A~D (PLAN 1-5)")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--usd", default=ts.DEFAULT_USD)
parser.add_argument("--physics-variant", default="physx")
parser.add_argument("--configs", nargs="+", help="A~C 에 쓸 설정 파일들",
                    default=[ts.DEFAULT_CONFIG, os.path.join(ts.CONFIG_DIR, "drive_gains_ff.yaml"),
                             os.path.join(ts.CONFIG_DIR, "drive_gains_official.yaml")])
parser.add_argument("--d-configs", nargs="+", default=[ts.DEFAULT_CONFIG], help="D(그리퍼)에 쓸 설정 파일들")
parser.add_argument("--realtime", action="store_true", help="sim 시간을 실제 시간에 맞춤 (GUI 로 눈으로 볼 때)")
parser.add_argument("--cases", default=os.path.join(ts.CONFIG_DIR, "gravity_worst_cases.yaml"))
parser.add_argument("--tests", default="ABCD", help="A~F 중 고르기 (E·F 는 D 와 같은 설정 목록 --d-configs 를 씀)")
args, _ = parser.parse_known_args()
# 같은 파일을 상대·절대 경로로 주면 두 번 돌지 않도록 절대경로로 통일
def _abs(p):
    path, *ov = p.split("@")
    return "@".join([os.path.abspath(path)] + ov)


args.configs = [_abs(p) for p in args.configs]
args.d_configs = [_abs(p) for p in args.d_configs]


def disp(p):
    """리포트 표시 이름: 파일 이름 + 덮어쓴 값 (self_collision 은 짧게)."""
    path, *ov = p.split("@")
    tags = ["selfcol ON" if o == "articulation.self_collision=true" else
            ("selfcol off" if o == "articulation.self_collision=false" else o) for o in ov]
    return os.path.basename(path) + (f" [{', '.join(tags)}]" if tags else "")

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

import time  # noqa: E402

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager  # noqa: E402
from robot_drive import RobotDrive  # noqa: E402

# ── 시험 조건 ──
STEPS = [(0.05, 1.5), (0.2, 2.0), (1.0, 3.0)]   # (스텝 크기 rad, 기록 시간 s). 판정은 JUDGE_STEP, 나머지는 참고
JUDGE_STEP = 0.05
SETTLE_BEFORE = 1.0                   # s
B_RUN = 2.5                           # s, 마지막 B_AVG 초 평균으로 정상상태 오차
B_AVG = 0.3
C_AMP, C_FREQ, C_DUR, C_RAMP, C_CMD_HZ = 0.3, 0.5, 6.0, 1.0, 50.0
C_EVAL_FROM = 1.5                     # s, 램프 이후부터 평가
D_WIN = 4.0                           # s
GRIP_CLOSED = 1.1351
HOLD_AT = 0.6                         # rad, "물체를 잡은" 위치 (rh_r1_joint 상한을 임시로 여기로 낮춰 손가락을 막음)
HOLD_MOVE = 0.005                     # rad, 열기 명령 후 이만큼 움직이면 "움직이기 시작"

# ── 합격 기준 (PLAN 1-5) ──
CRIT = {"overshoot_pct": 2.0, "settle_s": 0.5, "ss_deg": 0.1, "lag_ms": 40.0, "vel": math.pi * 1.02,
        "grip_s": (2.2 * 0.9, 2.2 * 1.1), "grip_delay_s": 0.1}


class Recorder:
    """physics 스텝마다 (t, q, qd) 기록 + 명령 스케줄러 (pre-step). 콜백 예외는 저장 후 check() 에서 올림."""

    def __init__(self, robot):
        self.robot = robot
        self.t = 0.0
        self.on = False
        self.rows = []
        self.cmd = None
        self.err = None
        self.cb = [SimulationManager.register_callback(self._pre, event=SimulationEvent.PHYSICS_PRE_STEP, order=-1),
                   SimulationManager.register_callback(self._post, event=SimulationEvent.PHYSICS_POST_STEP)]

    def _pre(self, dt, ctx):
        try:
            if self.cmd is not None:
                self.cmd(self.t)
        except Exception as e:  # noqa: BLE001
            self.err = e

    def _post(self, dt, ctx):
        try:
            self.t += dt
            if self.on:
                self.rows.append((self.t, self.robot.get_dof_positions().numpy()[0].copy(),
                                  self.robot.get_dof_velocities().numpy()[0].copy()))
        except Exception as e:  # noqa: BLE001
            self.err = e

    def start(self):
        self.rows, self.on = [], True
        return self.t

    def stop(self):
        self.on = False
        t = np.array([r[0] for r in self.rows])
        q = np.array([r[1] for r in self.rows])
        qd = np.array([r[2] for r in self.rows])
        return t, q, qd

    def close(self):
        for c in self.cb:
            SimulationManager.deregister_callback(c)

    def check(self):
        if self.err is not None:
            raise RuntimeError(f"기록 콜백 오류: {self.err!r}")


class Ctx:
    pass


def open_scene(cfg, ground=True, payload_pos=None, payload_kg=None):
    c = Ctx()
    c.stage, c.robot, c.info = ts.build_test_scene(args.usd, args.physics_variant, base="fixed", config=cfg,
                                                   ground=ground, light=not args.headless)
    c.link = ts.add_payload(c.stage, payload_kg, payload_pos) if payload_pos is not None else None
    if not args.headless:
        from isaacsim.core.utils.viewports import set_camera_view

        set_camera_view(eye=[1.6, 1.4, 1.6], target=[0.1, 0.1, 0.9], camera_prim_path="/OmniverseKit_Persp")
    app_utils.play()
    simulation_app.update()
    problems, _ = ts.verify_articulation(c.stage, c.robot, c.info, cfg)
    if problems:
        raise RuntimeError(f"articulation 확인 실패: {problems}")
    c.drive = RobotDrive(c.robot, cfg)
    c.drive.apply()
    c.drive.start()
    c.rec = Recorder(c.robot)
    c.wall, c.steps = 0.0, 0
    return c


def close_scene(c):
    c.drive.stop()
    c.rec.close()
    app_utils.stop()
    simulation_app.update()


def run_for(c, sec):
    t_end = c.rec.t + sec
    wall0, sim0 = time.monotonic(), c.rec.t
    while c.rec.t < t_end - 1e-9:
        n0, w0 = c.rec.t, time.perf_counter()
        simulation_app.update()
        c.wall += time.perf_counter() - w0
        c.steps += int(round((c.rec.t - n0) / ts.PHYSICS_DT))
        c.rec.check()
        c.drive.check()
        if args.realtime:
            ahead = (c.rec.t - sim0) - (time.monotonic() - wall0)
            if ahead > 0:
                time.sleep(ahead)


def reset(c, q6, gripper=0.0, settle=SETTLE_BEFORE):
    c.drive.reset_pose(q6, gripper)
    run_for(c, settle)


def settle_time(t, y, y_final, band):
    """y 가 y_final ± band 안에 마지막으로 들어온 시각 (t[0] 기준). 끝까지 안 들어오면 nan."""
    out = np.abs(y - y_final) > band
    if not out.any():
        return 0.0
    last = np.where(out)[0][-1]
    return float(t[last + 1] - t[0]) if last + 1 < len(t) else float("nan")


# ── A ──
def test_a(c, home, ai):
    res = []
    for j, name in enumerate(ts.ARM_JOINTS):
        for step, dur in STEPS:
            reset(c, home)
            tgt = np.array(home, dtype=float)
            tgt[j] += step
            q_start = c.robot.get_dof_positions().numpy()[0][ai].copy()
            t0 = c.rec.start()
            c.drive.set_arm_targets(tgt)
            run_for(c, dur)
            t, q, qd = c.rec.stop()
            t = t - t0
            y = (q[:, ai[j]] - q_start[j]) / (tgt[j] - q_start[j])
            i10 = np.argmax(y >= 0.1)
            i90 = np.argmax(y >= 0.9)
            rise = float(t[i90] - t[i10]) if y.max() >= 0.9 else float("nan")
            over = max(0.0, float(y.max() - 1.0) * 100)
            ts2 = settle_time(t, q[:, ai[j]], tgt[j], 0.02 * abs(step))
            tail = t >= t[-1] - 0.2
            ss = math.degrees(abs(q[tail, ai[j]].mean() - tgt[j]))
            others = [k for k in range(6) if k != j]
            coup = math.degrees(np.abs(q[:, ai[others]] - np.array(home)[others]).max())
            vmax = float(np.abs(qd[:, ai[j]]).max())
            nan = not (np.isfinite(q).all() and np.isfinite(qd).all())
            res.append(dict(joint=name, step=step, rise=rise, over=over, settle=ts2, ss=ss, coup=coup, vmax=vmax,
                            nan=nan))
    return res


def a_pass(r):
    return ((not r["nan"]) and r["over"] < CRIT["overshoot_pct"] and r["ss"] < CRIT["ss_deg"]
            and r["vmax"] <= CRIT["vel"] and r["settle"] < CRIT["settle_s"])


# ── B ──
def test_b(cfg, cases):
    """드론 무게는 매 physics 스텝 그리퍼 base 링크의 (TCP + offset) 점에 가하는 외력 (0, 0, −m·g) 으로 흉내낸다.
    articulation 밖의 힘이라 gravity_ff 가 보상하지 않는다. 씬은 설정마다 한 번만 만들고 경우마다 자세만 바꾼다
    (예전: 경우마다 씬을 새로 만들고 별도 강체를 FixedJoint 로 붙임 → 느림). 정적 시험이라 드론 관성은 무시."""
    from isaacsim.core.experimental.prims import RigidPrim

    res = []
    pay_kg, pay_pos = cases["payload_kg"], np.array(cases["payload_pos"], dtype=float)
    c = open_scene(cfg, ground=False)
    try:
        ai = np.array([c.drive.idx[n] for n in ts.ARM_JOINTS])
        link = RigidPrim(ts.find_link_path(c.stage, ts.PAYLOAD_LINK))
        force = np.array([[0.0, 0.0, -pay_kg * 9.81]])
        state = {"local": pay_pos}

        def push(t):  # pre-step: 링크 현재 자세에서 드론 무게중심 위치에 무게를 가함
            pos, quat = (x.numpy()[0] for x in link.get_world_poses())
            w, x, y, z = quat
            R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                          [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                          [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
            p = pos + R @ state["local"]
            link.apply_forces_and_torques_at_pos(forces=force, positions=p.reshape(1, 3), local_frame=False)

        c.rec.cmd = push
        for case in cases["cases"]:
            state["local"] = pay_pos + np.array([case["offset"][0], case["offset"][1], 0.0])
            pose = np.array(case["pose"], dtype=float)
            c.drive.reset_pose(pose)
            run_for(c, B_RUN - B_AVG)
            c.rec.start()
            run_for(c, B_AVG)
            t, q, qd = c.rec.stop()
            err = np.degrees(np.abs(q[:, ai].mean(axis=0) - pose))
            j = ts.ARM_JOINTS.index(case["joint"])
            res.append(dict(joint=case["joint"], off=case["offset_cm"], tau=case["tau"], err_j=float(err[j]),
                            err_max=float(err.max()), err_all=err, nan=not np.isfinite(q).all()))
        c.rec.cmd = None
    finally:
        close_scene(c)
    return res


# ── C ──
def test_c(c, home, ai):
    reset(c, home)
    phase = np.arange(6) * math.pi / 3

    def ref(t):
        ramp = np.clip(t / C_RAMP, 0, 1)
        ramp = ramp * ramp * (3 - 2 * ramp)
        return np.array(home) + C_AMP * ramp * np.sin(2 * math.pi * C_FREQ * t + phase)

    t0 = c.rec.t
    state = {"next": 0.0}

    def cmd(t):  # 50 Hz zero-order hold
        tr = t - t0
        if tr >= state["next"] - 1e-9:
            c.drive.set_arm_targets(ref(state["next"]))
            state["next"] += 1.0 / C_CMD_HZ

    c.rec.start()
    c.rec.cmd = cmd
    run_for(c, C_DUR)
    c.rec.cmd = None
    t, q, _ = c.rec.stop()
    t = t - t0
    m = t >= C_EVAL_FROM
    res = []
    for j, name in enumerate(ts.ARM_JOINTS):
        r_cont = np.array([ref(x)[j] for x in t[m]])
        e = q[m, ai[j]] - r_cont
        lags = np.arange(-0.02, 0.3001, 0.001)
        rms_l = [np.sqrt(np.mean((q[m, ai[j]] - np.array([ref(x - L)[j] for x in t[m]])) ** 2)) for L in lags]
        res.append(dict(joint=name, rms=math.degrees(np.sqrt(np.mean(e ** 2))), emax=math.degrees(np.abs(e).max()),
                        lag=1000 * float(lags[int(np.argmin(rms_l))])))
    return res


# ── D ──
def test_d(c, home, gi):
    out = {}
    reset(c, home, 0.0, settle=1.5)
    for label, target in (("close", GRIP_CLOSED), ("open", 0.0)):
        q0 = float(c.robot.get_dof_positions().numpy()[0][gi])
        t0 = c.rec.start()
        c.drive.set_gripper_goal(target)
        run_for(c, D_WIN)
        t, q, qd = c.rec.stop()
        y = q[:, gi]
        final = float(y[-20:].mean())
        dur = settle_time(t - t0, y, final, 0.02 * abs(final - q0))
        out[label] = dict(time=dur, start=q0, final=final, vmax=float(np.abs(qd[:, gi]).max()))

    # 잡은 상태에서 열기: collider 가 아직 없으므로(1-6 전) 관절 상한을 HOLD_AT 으로 낮춰 물체처럼 막는다 (메모리에서만)
    lo, hi = (x.numpy().copy() for x in c.robot.get_dof_limits())
    try:
        hi2 = hi.copy()
        hi2[0, gi] = HOLD_AT
        c.robot.set_dof_limits(lo, hi2)
        reset(c, home, 0.0, settle=0.5)
        c.drive.set_gripper_goal(GRIP_CLOSED)
        run_for(c, (HOLD_AT / c.drive.gripper.velocity) + 1.5)  # 막힐 때까지 + 목표값이 끝(1.1351)까지 가도록
        q_hold = float(c.robot.get_dof_positions().numpy()[0][gi])
        sp_hold = c.drive.gripper.setpoint
        t0 = c.rec.start()
        c.drive.set_gripper_goal(0.0)
        run_for(c, 1.5)
        t, q, _ = c.rec.stop()
        moved = np.where(q_hold - q[:, gi] > HOLD_MOVE)[0]
        delay = float(t[moved[0]] - t0) if len(moved) else float("nan")
        out["hold_open"] = dict(q_hold=q_hold, setpoint_hold=sp_hold, delay=delay,
                                old_delay=(sp_hold - q_hold) / c.drive.gripper.velocity)
    finally:
        c.robot.set_dof_limits(lo, hi)
    return out


# ── E ──
def test_e(c, home, ai, gi):
    reset(c, home, 0.0, settle=1.0)
    w0, s0 = c.wall, c.steps
    t0 = c.rec.start()
    run_for(c, 3.0)
    t, q, qd = c.rec.stop()
    t = t - t0
    last = t >= 1.0
    cmd = np.array(home)
    dev = np.degrees(np.abs(q[:, ai] - cmd).max(axis=0))
    p2p = np.degrees(q[last][:, ai].max(axis=0) - q[last][:, ai].min(axis=0))
    return dict(dev=float(dev.max()), dev_joint=ts.ARM_JOINTS[int(dev.argmax())], p2p=float(p2p.max()),
                p2p_joint=ts.ARM_JOINTS[int(p2p.argmax())], vmax=float(np.abs(qd[:, ai]).max()),
                grip_dev=math.degrees(float(np.abs(q[:, gi]).max())),
                nan=not (np.isfinite(q).all() and np.isfinite(qd).all()),
                ms_per_step=1000 * (c.wall - w0) / max(1, c.steps - s0))


# ── F ──
def test_f(c, home, gi, idx):
    reset(c, home, 0.0, settle=0.5)
    c.drive.set_gripper_goal(GRIP_CLOSED)
    run_for(c, GRIP_CLOSED / c.drive.gripper.velocity + 1.0)
    t0 = c.rec.start()
    run_for(c, 2.0)
    t, q, _ = c.rec.stop()
    fi = [idx[n] for n in ts.GRIPPER_JOINTS]
    p2p = np.degrees(q[:, fi].max(axis=0) - q[:, fi].min(axis=0))
    return dict(final_deg=math.degrees(float(q[-1, gi])), p2p=float(p2p.max()),
                nan=not np.isfinite(q).all())


def fmt(x, nd=3):
    return "nan" if x is None or (isinstance(x, float) and math.isnan(x)) else f"{x:.{nd}f}"


def main():
    L = []

    def log(m=""):
        print(m, flush=True)
        L.append(m)

    log(f"tune_drives  {datetime.datetime.now().isoformat(timespec='seconds')}  physics dt = {ts.PHYSICS_DT:.5f} s")
    cases = yaml.safe_load(open(args.cases, encoding="utf-8")) if "B" in args.tests else None
    results = {}
    d_like = any(k in args.tests for k in "DEF")
    all_paths = list(dict.fromkeys(list(args.configs) + (list(args.d_configs) if d_like else [])))
    for path in all_paths:
        cfg = ts.load_config(path)
        in_abc, in_d = path in args.configs, (d_like and path in args.d_configs)
        name = disp(path)
        log("")
        log(f"===== {name} (gravity_ff = {cfg['gravity_ff']}) =====")
        R = results[name] = {}
        home = [float(x) for x in cfg["home_pose"]]
        if (in_abc and any(k in args.tests for k in "AC")) or in_d:
            c = open_scene(cfg)
            try:
                ai = np.array([c.drive.idx[n] for n in ts.ARM_JOINTS])
                gi = c.drive.idx[ts.GRIPPER_DRIVE]
                if cfg["gravity_ff"]:
                    reset(c, home, settle=0.2)
                    log(f"  gravity_ff 동작 확인: {c.drive.ff_steps} 스텝 적용, 홈 자세 ff 토크 = "
                        + ", ".join(f"{x:.2f}" for x in c.drive.last_ff) + " Nm")
                if in_abc and "A" in args.tests:
                    R["A"] = test_a(c, home, ai)
                if in_abc and "C" in args.tests:
                    R["C"] = test_c(c, home, ai)
                if in_d and "E" in args.tests:
                    R["E"] = test_e(c, home, ai, gi)
                if in_d and "D" in args.tests:
                    R["D"] = test_d(c, home, gi)
                if in_d and "F" in args.tests:
                    R["F"] = test_f(c, home, gi, c.drive.idx)
                R["wall_ms_per_step"] = 1000 * c.wall / max(1, c.steps)
            finally:
                close_scene(c)
        if in_abc and "B" in args.tests:
            R["B"] = test_b(cfg, cases)

    names = [disp(p) for p in args.configs]
    d_names = [disp(p) for p in args.d_configs]
    col = lambda s: f"{s:>34s}"  # noqa: E731

    if "A" in args.tests:
        log("")
        log(f"[A] 스텝 응답: 상승(10→90%) s / overshoot % / 2% 정착 s / 정상상태 오차 ° / 최대속도 rad/s / 다른 관절 흔들림 °")
        log(f"    판정은 {JUDGE_STEP} rad 스텝 (P/F), 0.2·1.0 rad 는 참고 (p/f 소문자)")
        log(f"  {'joint':20s} {'step':>5s} " + " ".join(col(n) for n in names))
        for k in range(len(results[names[0]]["A"])):
            r0 = results[names[0]]["A"][k]
            cells = []
            for n in names:
                r = results[n]["A"][k]
                tag = "P" if a_pass(r) else "F"
                tag = tag if r["step"] == JUDGE_STEP else tag.lower()
                cells.append(col(f"{fmt(r['rise'], 2)}/{fmt(r['over'], 1)}/{fmt(r['settle'], 2)}/{fmt(r['ss'], 3)}/"
                                 f"{fmt(r['vmax'], 2)}/{fmt(r['coup'], 2)} {tag}"))
            log(f"  {r0['joint']:20s} {r0['step']:5.2f} " + " ".join(cells))

    if "B" in args.tests:
        log("")
        log("[B] 중력 유지 (드론 1.5 kg, 최악 자세·방향): 대상 관절 오차 ° / 전 관절 최대 오차 °  [기준 < 0.1°]")
        log(f"  {'joint':20s} {'off':>4s} {'τ Nm':>6s} " + " ".join(col(n) for n in names))
        for k in range(len(results[names[0]]["B"])):
            r0 = results[names[0]]["B"][k]
            cells = []
            for n in names:
                r = results[n]["B"][k]
                ok = (not r["nan"]) and r["err_max"] < CRIT["ss_deg"]
                cells.append(col(f"{r['err_j']:.3f} / {r['err_max']:.3f} {'P' if ok else 'F'}"))
            log(f"  {r0['joint']:20s} {r0['off']:3d}cm {r0['tau']:6.2f} " + " ".join(cells))

    if "C" in args.tests:
        log("")
        log(f"[C] 추종 ({C_CMD_HZ:.0f} Hz ZOH, 진폭 {C_AMP} rad, {C_FREQ} Hz): RMS ° / 최대 ° / 지연 ms  [지연 기준 < 40 ms]")
        log(f"  {'joint':20s} " + " ".join(col(n) for n in names))
        for k in range(6):
            cells = []
            for n in names:
                r = results[n]["C"][k]
                cells.append(col(f"{r['rms']:.3f} / {r['emax']:.3f} / {r['lag']:.0f} "
                                 f"{'P' if r['lag'] < CRIT['lag_ms'] else 'F'}"))
            log(f"  {results[names[0]]['C'][k]['joint']:20s} " + " ".join(cells))

    if "D" in args.tests:
        lo, hi = CRIT["grip_s"]
        log("")
        log(f"[D] 그리퍼: 시간 s (최종 rad, 최대속도 rad/s)  [목표 약 2.2 s, {lo:.2f}~{hi:.2f}]")
        log(f"  {'':26s} " + " ".join(col(n) for n in d_names))
        for label in ("close", "open"):
            cells = []
            for n in d_names:
                r = results[n]["D"][label]
                ok = lo <= r["time"] <= hi
                cells.append(col(f"{fmt(r['time'], 2)} ({r['final']:.3f}, {r['vmax']:.3f}) {'P' if ok else 'F'}"))
            log(f"  {label:26s} " + " ".join(cells))
        log(f"  잡은 상태에서 열기 (상한 {HOLD_AT} rad 로 막음): 명령 → 움직이기 시작({HOLD_MOVE} rad)까지 지연 [기준 < {CRIT['grip_delay_s']} s]")
        cells = []
        for n in d_names:
            r = results[n]["D"]["hold_open"]
            ok = r["delay"] < CRIT["grip_delay_s"]
            cells.append(col(f"{fmt(r['delay'], 3)} s {'P' if ok else 'F'}"))
        log(f"  {'hold_open':26s} " + " ".join(cells))
        for n in d_names:
            r = results[n]["D"]["hold_open"]
            log(f"    {n}: 막힌 위치 {r['q_hold']:.3f} rad, 그때 목표값 {r['setpoint_hold']:.3f} rad "
                f"→ 이전 방식(목표값에서 이어 감)이면 지연 약 {r['old_delay']:.2f} s")

    if "E" in args.tests:
        log("")
        log("[E] 홈 자세 가만히 3 초: 명령 위치에서 최대 벗어남 ° / 떨림 peak-to-peak ° (마지막 2 초) / 최대 속도 rad/s / 그리퍼 벗어남 ° / 스텝당 계산 ms")
        log(f"    [떨림 기준 < 0.01°, NaN 없음]")
        for n in d_names:
            r = results[n]["E"]
            ok = (not r["nan"]) and r["p2p"] < 0.01
            log(f"  {n:40s} {r['dev']:.4f} ({r['dev_joint']}) / {r['p2p']:.5f} ({r['p2p_joint']}) / {r['vmax']:.4f} / "
                f"{r['grip_dev']:.3f} / {r['ms_per_step']:.2f} ms {'P' if ok else 'F'}")
    if "F" in args.tests:
        log("")
        log("[F] 빈손으로 그리퍼 끝까지 닫기 → 닫힌 뒤 2 초: 최종 rh_r1 각도 ° / 손가락 4 관절 떨림 peak-to-peak °  [떨림 기준 < 0.01°]")
        for n in d_names:
            r = results[n]["F"]
            ok = (not r["nan"]) and r["p2p"] < 0.01
            log(f"  {n:40s} {r['final_deg']:.3f}° / {r['p2p']:.5f}° {'P' if ok else 'F'}")
    log("")
    log("[계산 시간] 설정별 전체 시험의 physics 스텝당 평균 wall time (headless, 렌더링 포함 app.update 기준)")
    for n in list(results):
        if "wall_ms_per_step" in results[n]:
            log(f"  {n:40s} {results[n]['wall_ms_per_step']:.2f} ms/step")

    os.makedirs(ts.REPORT_DIR, exist_ok=True)
    path = os.path.join(ts.REPORT_DIR, f"tune_drives_{datetime.datetime.now():%Y%m%d_%H%M%S}.txt")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")
    print(f"report: {path}", flush=True)


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
