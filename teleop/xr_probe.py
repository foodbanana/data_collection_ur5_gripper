#!/usr/bin/env python3
# =============================================================
# xr_probe.py — PICO 4 Ultra 컨트롤러 입력 확인 (docs/PLAN.md 5-B B-0, sim 없이)
#
#   XRoboToolkit PC-Service → Python 바인딩 (xrobotoolkit_sdk) 으로 들어오는 값을 재서
#   텔레옵 노드 (B-2) 를 만들기 전에 다음을 숫자로 남긴다:
#     1. 주기·간격 흔들림 (헤드셋 타임스탬프), PC 시각 − 헤드셋 타임스탬프 (지연 + 시계 차이)
#     2. 가만히 있을 때 흔들림 (위치 mm, 자세 °)
#     3. 축 방향: 앞(몸에서 멀어지게)·오른쪽·위로 옮겼을 때 어느 축이 어느 부호로 변하나, 오른손/왼손 좌표계
#     4. 회전: 끝 들기·오른쪽 돌리기·손목 비틀기의 회전축 (world·컨트롤러 좌표계)
#     5. 버튼: trigger·grip 범위, B·조이스틱
#     6. 끊김: 컨트롤러를 숨김 / 내려놓음 / 헤드셋을 벗음 → 값이 멈추나, 0 이 되나, 계속 오나
#
#   바인딩 동작 (소스 bindings/py_bindings.cpp, 2026-10-12 확인): 헤드셋 데이터가 올 때마다 SDK 스레드가
#   전역 변수를 "최신값" 으로 덮어쓰고, get_* 는 그 값을 복사해 돌려준다 → 새 데이터가 안 와도 옛 값이 계속 나온다.
#   한 번도 안 왔으면 전부 0 (서비스가 없어도 init() 은 에러 없이 끝남). 그래서 신선도는 get_time_stamp_ns() 증가로 본다.
#
#   측정 방식: 안내마다 "자세를 잡은 뒤 오른쪽 컨트롤러 A 버튼 (또는 터미널 Enter)" 을 누르면, 누르기 직전 0.3 s 평균을 그 자세로 쓴다
#   (A-0 조종기 확인에서 시간 창 방식은 일찍 움직이면 틀렸다). 끊김 시험은 A 로 시작하고 정해진 시간 뒤 삐 소리
#
# 실행 (PC-Service 실행 중, 헤드셋 앱 WORKING, Controller 추적 켬·Data/Control Send):
#   ~/xrobotoolkit/venv/bin/python teleop/xr_probe.py
#   헤드셋을 쓴 채 터미널 안내를 보려면 PICO 패스스루로 모니터를 보거나, 다른 사람이 안내를 읽어 주면 편하다
#   --skip-loss : 끊김 시험 (헤드셋 벗기 포함) 을 건너뜀
# 결과: teleop/reports/xr_probe_<시각>/ (report.txt, summary.json, raw.csv, events.csv). git 제외
# =============================================================

import argparse
import csv
import datetime
import json
import os
import select
import sys
import threading
import time

try:
    import numpy as np
except ImportError:
    sys.exit("[ERROR] numpy 를 import 할 수 없음 (시스템 python3-numpy 가 venv 에 보여야 함: venv 를 --system-site-packages 로 만들었는지 확인)")
try:
    import xrobotoolkit_sdk as xrt
except ImportError:
    sys.exit("[ERROR] xrobotoolkit_sdk 를 import 할 수 없음. ~/xrobotoolkit/venv/bin/python 으로 실행할 것")

REPORT_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "reports")

POLL_SEC = 0.001          # 바인딩 값을 읽는 간격 (헤드셋 주기보다 충분히 빠르게. 새 타임스탬프일 때만 기록)
MARK_WINDOW = 0.3         # [s] A / Enter 를 누르기 직전 이만큼을 평균해 그 자세로 쓴다
STILL_SEC = 10.0          # [s] 가만히 측정
CONNECT_TIMEOUT = 5.0     # [s] 시작할 때 새 데이터가 이 안에 와야 함
STALE_GAP = 0.1           # [s] 헤드셋 타임스탬프가 이보다 오래 안 바뀌면 "끊김" 구간으로 셈

POSE_KEYS = ("px", "py", "pz", "qx", "qy", "qz", "qw")      # 바인딩 순서 (README: x, y, z, qx, qy, qz, qw)
FIELDS = (["pc_mono_ns", "pc_wall_ns", "dev_ts_ns"]
          + [f"r_{k}" for k in POSE_KEYS] + [f"l_{k}" for k in POSE_KEYS] + [f"h_{k}" for k in POSE_KEYS]
          + ["r_trig", "r_grip", "r_a", "r_b", "r_ax", "r_ay", "r_click",
             "l_trig", "l_grip", "l_x", "l_y", "l_ax", "l_ay", "l_click"])
COL = {k: i for i, k in enumerate(FIELDS)}
R_POS, R_QUAT = slice(COL["r_px"], COL["r_pz"] + 1), slice(COL["r_qx"], COL["r_qw"] + 1)


# ---------------------------------------------------------------- 읽기
def read_once():
    """바인딩 값 한 벌. 타임스탬프를 앞뒤로 읽어 같을 때만 쓴다 (읽는 도중 새 데이터가 들어오면 다시)."""
    for _ in range(5):
        ts0 = xrt.get_time_stamp_ns()
        r, l, h = xrt.get_right_controller_pose(), xrt.get_left_controller_pose(), xrt.get_headset_pose()
        rax, lax = xrt.get_right_axis(), xrt.get_left_axis()
        row = [time.monotonic_ns(), time.time_ns(), ts0, *r, *l, *h,
               xrt.get_right_trigger(), xrt.get_right_grip(), int(xrt.get_A_button()), int(xrt.get_B_button()),
               rax[0], rax[1], int(xrt.get_right_axis_click()),
               xrt.get_left_trigger(), xrt.get_left_grip(), int(xrt.get_X_button()), int(xrt.get_Y_button()),
               lax[0], lax[1], int(xrt.get_left_axis_click())]
        if xrt.get_time_stamp_ns() == ts0:
            return row
    return row


class Recorder:
    """바인딩을 POLL_SEC 간격으로 읽어, 헤드셋 타임스탬프가 바뀐 것만 쌓는다 (rows). latest = 가장 최근 읽은 값 (같은 값 포함)."""

    def __init__(self):
        self.rows, self.latest = [], None
        self.error = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._t = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._t.start()

    def _run(self):
        last_ts = None
        try:
            while not self._stop.is_set():
                row = read_once()
                with self._lock:
                    self.latest = row
                    if row[2] != last_ts:
                        self.rows.append(row)
                        last_ts = row[2]
                time.sleep(POLL_SEC)
        except Exception as e:  # noqa: BLE001
            self.error = e

    def check(self):
        if self.error is not None:
            raise RuntimeError(f"바인딩 읽기 에러: {self.error!r}") from self.error

    def snapshot(self):
        with self._lock:
            return list(self.latest) if self.latest else None, len(self.rows)

    def rows_between(self, t0_ns, t1_ns):
        with self._lock:
            return [r for r in self.rows if t0_ns <= r[0] <= t1_ns]

    def stop(self):
        self._stop.set()
        self._t.join(timeout=1.0)


# ---------------------------------------------------------------- 회전 (numpy 만)
def quat_to_R(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_rotvec(R):
    c = np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)
    ang = np.arccos(c)
    if ang < 1e-9:
        return np.zeros(3)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if np.pi - ang < 1e-6:                                    # 180° 근처: 대각 성분에서 축
        a = np.sqrt(np.maximum((np.diag(R) + 1) / 2, 0))
        return ang * a / np.linalg.norm(a)
    return ang * v / (2 * np.sin(ang))


def mean_quat(qs):
    qs = np.asarray(qs, dtype=float)
    ref = qs[0]
    qs = np.where((qs @ ref < 0)[:, None], -qs, qs)          # 부호 맞춤 (q 와 −q 는 같은 자세)
    m = qs.mean(axis=0)
    return m / np.linalg.norm(m)


def axis_name(v, names=("x", "y", "z")):
    i = int(np.argmax(np.abs(v)))
    return f"{'+' if v[i] >= 0 else '-'}{names[i]}", float(abs(v[i]) / (np.linalg.norm(v) + 1e-12))


# ---------------------------------------------------------------- 입력 (A 버튼 / Enter)
def stdin_enter():
    """터미널에서 Enter 가 눌렸으면 True (막지 않음). stdin 이 터미널이 아니면 항상 False."""
    if not sys.stdin.isatty():
        return False
    if select.select([sys.stdin], [], [], 0)[0]:
        sys.stdin.readline()
        return True
    return False


def say(msg, beep=True):
    print(("\a" if beep else "") + "\n>>> " + msg, flush=True)


def wait_mark(rec, msg, timeout=120.0):
    """A 버튼 (누르는 순간) 또는 Enter 를 기다린다 → 누른 시각 [monotonic ns]."""
    say(msg + "   [자세를 잡은 뒤 A 버튼 또는 Enter]")
    snap, _ = rec.snapshot()
    a_prev = snap[COL["r_a"]] if snap else 0
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        rec.check()
        if stdin_enter():
            return time.monotonic_ns()
        snap, _ = rec.snapshot()
        a = snap[COL["r_a"]] if snap else 0
        if a and not a_prev:
            return time.monotonic_ns()
        a_prev = a
        time.sleep(0.005)
    raise TimeoutError(f"{timeout:g} s 안에 A / Enter 가 없음: {msg}")




# ---------------------------------------------------------------- 단계
TRANSLATIONS = [  # (이름, 안내)
    ("forward", "앞으로: 기준 자세에서, 컨트롤러를 몸에서 멀어지는 방향(정면)으로 약 20 cm 옮기세요. 자세(기울기)는 그대로"),
    ("right", "오른쪽으로: 기준 자세에서, 컨트롤러를 오른쪽으로 약 20 cm 옮기세요. 자세는 그대로"),
    ("up", "위로: 기준 자세에서, 컨트롤러를 위로 약 20 cm 올리세요. 자세는 그대로"),
]
ROTATIONS = [
    ("pitch_up", "끝 들기: 기준 자세에서, 위치는 그대로 두고 컨트롤러 앞쪽 끝을 위로 약 45° 드세요"),
    ("yaw_right", "오른쪽 돌리기: 기준 자세에서, 위치는 그대로 두고 컨트롤러 앞쪽 끝을 오른쪽으로 약 45° 돌리세요"),
    ("roll_cw", "손목 비틀기: 기준 자세에서, 위치는 그대로 두고 손목을 시계 방향(내 쪽에서 볼 때)으로 약 45° 비트세요"),
]
BUTTONS = [  # (이름, 열, 안내, 기준값, 횟수)
    ("trigger", "r_trig", "검지 트리거를 끝까지 눌렀다 떼기를 3 번", 0.5, 3),
    ("grip", "r_grip", "중지 그립(손잡이 옆 버튼)을 끝까지 쥐었다 놓기를 3 번", 0.5, 3),
    ("B", "r_b", "B 버튼을 2 번", 0.5, 2),
    ("stick_click", "r_click", "조이스틱을 눌러(클릭) 2 번", 0.5, 2),
    ("stick_up", "r_ay", "조이스틱을 위로 밀었다 놓기를 2 번", 0.5, 2),
    ("stick_right", "r_ax", "조이스틱을 오른쪽으로 밀었다 놓기를 2 번", 0.5, 2),
]
LOSS = [  # (이름, 안내, 시간 [s])
    ("hide", "숨기기: A 를 누른 뒤 컨트롤러를 등 뒤로 숨겨 삐 소리가 날 때까지 (8 s) 그대로, 삐 소리 뒤 다시 앞으로", 8.0),
    ("put_down", "내려놓기: A 를 누른 뒤 컨트롤러를 책상에 내려놓고 손을 떼세요. 삐 소리 (15 s) 뒤 다시 집기", 15.0),
    ("headset_off", "헤드셋 벗기: A 를 누른 뒤 헤드셋을 벗어 내려놓고 컨트롤러를 들고 천천히 움직이세요. 삐 소리 (15 s) 뒤 다시 쓰기", 15.0),
]


def count_presses(vals, thr):
    on = np.abs(np.asarray(vals, dtype=float)) > thr          # 조이스틱 축은 방향에 따라 음수일 수 있음
    return int(np.sum(on[1:] & ~on[:-1]))


def run(args):
    out = os.path.join(REPORT_ROOT, "xr_probe_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
    events, summary = [], {"started": datetime.datetime.now().isoformat(timespec="seconds"), "args": vars(args)}

    def event(name, t0, t1=None, **kw):
        events.append({"name": name, "t0_ns": t0, "t1_ns": t1 if t1 is not None else t0, **kw})

    xrt.init()
    rec = Recorder()
    rec.start()
    try:
        # ---- 0. 연결: 새 데이터가 오고 자세가 0 이 아닌지 (조용히 0 을 쓰지 않는다)
        say("연결 확인 중 (오른쪽 컨트롤러를 헤드셋 앞에 들고 있으세요)", beep=False)
        t_end = time.monotonic() + CONNECT_TIMEOUT
        while True:
            rec.check()
            snap, n = rec.snapshot()
            if snap and n >= 10 and snap[2] != 0 and np.linalg.norm(snap[R_QUAT]) > 0.5:
                break
            if time.monotonic() > t_end:
                why = ("값이 전부 0 (서비스 연결 안 됨 또는 헤드셋 데이터 없음)" if not snap or snap[2] == 0
                       else f"새 데이터 {n} 개 / 쿼터니언 크기 {np.linalg.norm(snap[R_QUAT]):.3f}")
                raise RuntimeError(f"{CONNECT_TIMEOUT:g} s 안에 헤드셋 데이터가 오지 않음: {why}. "
                                   "PC-Service 실행·헤드셋 앱 WORKING·Data/Control Send·컨트롤러 추적을 확인")
            time.sleep(0.05)
        print("    OK: 데이터 들어옴", flush=True)

        # ---- 1. 가만히
        say(f"가만히: 오른쪽 컨트롤러를 헤드셋이 보는 앞쪽 책상 위에 내려놓거나 최대한 가만히 들고, A / Enter. 그 뒤 {STILL_SEC:g} s 측정")
        wait_mark(rec, "준비되면")
        t0 = time.monotonic_ns()
        time.sleep(STILL_SEC)
        t1 = time.monotonic_ns()
        event("still", t0, t1)
        print("    끝", flush=True)

        # ---- 2. 축 (이동)
        say("이제부터는 정면(로봇 모니터 쪽)을 보고, 오른쪽 컨트롤러를 편하게 앞에 든 '기준 자세' 를 매번 같은 곳에서 시작합니다", beep=False)
        for name, msg in TRANSLATIONS:
            ta = wait_mark(rec, f"[{name}] 기준 자세에서")
            tb = wait_mark(rec, msg)
            event(name, ta, tb, kind="translation")
        for name, msg in ROTATIONS:
            ta = wait_mark(rec, f"[{name}] 기준 자세에서")
            tb = wait_mark(rec, msg)
            event(name, ta, tb, kind="rotation")

        # ---- 3. 버튼 (횟수가 차거나 20 s)
        for name, col, msg, thr, n_need in BUTTONS:
            say(f"[{name}] {msg}")
            t0 = time.monotonic_ns()
            while True:
                rec.check()
                vals = [r[COL[col]] for r in rec.rows_between(t0, time.monotonic_ns())]
                n = count_presses(vals, thr) if len(vals) > 1 else 0
                if n >= n_need or time.monotonic_ns() - t0 > 20e9:
                    break
                time.sleep(0.02)
            event(name, t0, time.monotonic_ns(), kind="button", col=col, presses=n, need=n_need)
            print(f"    눌림 {n} / {n_need}" + ("" if n >= n_need else "  (시간 초과)"), flush=True)

        # ---- 4. 끊김
        if not args.skip_loss:
            for name, msg, dur in LOSS:
                t0 = wait_mark(rec, msg)
                time.sleep(dur)
                say(f"[{name}] 끝: 원래대로 (컨트롤러를 앞으로 / 헤드셋을 다시 쓰기)")
                t1 = time.monotonic_ns()
                # 다시 데이터가 정상으로 올 때까지 (최대 60 s) 기다려 복구 시간도 잰다
                # 복구 = 삐 소리 뒤 새 데이터가 다시 5 개 이상 들어옴. 시각은 그 첫 데이터 (안 끊겼으면 거의 0)
                t_back = None
                while time.monotonic_ns() - t1 < 60e9:
                    rows = rec.rows_between(t1, time.monotonic_ns())
                    if len(rows) >= 5:
                        t_back = rows[0][0]
                        break
                    time.sleep(0.05)
                event(name, t0, t1, kind="loss", recovered_ns=t_back)
                print("    복구됨" if t_back else "    60 s 안에 복구 안 됨", flush=True)
                if name == "headset_off":
                    wait_mark(rec, "헤드셋을 다시 쓰고 연결이 돌아왔으면")
    except KeyboardInterrupt:
        summary["interrupted"] = True
        print("\n[중단] Ctrl+C: 지금까지 받은 데이터로 결과를 저장합니다", flush=True)
    finally:
        rec.stop()
        xrt.close()

    rows = np.asarray(rec.rows, dtype=float)
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "raw.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(FIELDS)
        w.writerows(rec.rows)
    with open(os.path.join(out, "events.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=sorted({k for e in events for k in e}))
        w.writeheader()
        w.writerows(events)
    report, summary = analyze(rows, events, summary)
    with open(os.path.join(out, "report.txt"), "w", encoding="utf-8") as f:
        f.write(report)
    with open(os.path.join(out, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=1)
    print("\n" + report)
    print(f"결과: {out}")


# ---------------------------------------------------------------- 분석
def analyze(rows, events, summary):
    L = []
    ev = {e["name"]: e for e in events}
    n_zero = int(np.sum(rows[:, 2] == 0)) if len(rows) else 0
    rows = rows[rows[:, 2] != 0] if len(rows) else rows      # 타임스탬프 0 = 아직 데이터가 오기 전 (바인딩 초기값). 분석에서 뺀다
    summary["zero_timestamp_samples"] = n_zero
    if len(rows) < 2:
        return "데이터 없음\n", summary

    # 1. 주기 (가만히 구간, 없으면 전체)
    seg = rows if "still" not in ev else rows[(rows[:, 0] >= ev["still"]["t0_ns"]) & (rows[:, 0] <= ev["still"]["t1_ns"])]
    d_dev = np.diff(seg[:, 2]) / 1e6
    d_pc = np.diff(seg[:, 0]) / 1e6
    lag = (seg[:, 1] - seg[:, 2]) / 1e6
    qn = np.linalg.norm(rows[:, R_QUAT], axis=1)
    per = {"dev_interval_ms": {"median": float(np.median(d_dev)), "p5": float(np.percentile(d_dev, 5)),
                               "p95": float(np.percentile(d_dev, 95)), "max": float(d_dev.max())},
           "rate_hz": float(1000 / np.median(d_dev)),
           "arrival_interval_ms": {"median": float(np.median(d_pc)), "p95": float(np.percentile(d_pc, 95)), "max": float(d_pc.max())},
           "pc_wall_minus_dev_ms": {"median": float(np.median(lag)), "std": float(np.std(lag)),
                                    "p5": float(np.percentile(lag, 5)), "p95": float(np.percentile(lag, 95))},
           "quat_norm": {"min": float(qn.min()), "max": float(qn.max())}, "n_samples": int(len(rows))}
    summary["timing"] = per
    L += ["== 1. 주기 (가만히 구간)",
          f"  헤드셋 타임스탬프 간격 [ms]: 중앙값 {per['dev_interval_ms']['median']:.2f} (≈ {per['rate_hz']:.1f} Hz), "
          f"5% {per['dev_interval_ms']['p5']:.2f}, 95% {per['dev_interval_ms']['p95']:.2f}, 최대 {per['dev_interval_ms']['max']:.2f}",
          f"  PC 도착 간격 [ms]: 중앙값 {per['arrival_interval_ms']['median']:.2f}, 95% {per['arrival_interval_ms']['p95']:.2f}, 최대 {per['arrival_interval_ms']['max']:.2f}",
          f"  PC 시각 − 헤드셋 타임스탬프 [ms]: 중앙값 {per['pc_wall_minus_dev_ms']['median']:.1f}, 표준편차 {per['pc_wall_minus_dev_ms']['std']:.1f}"
          " (지연 + 두 기기 시계 차이. 시계 차이를 몰라 지연 자체는 아님. 표준편차는 지연 흔들림에 가깝다)",
          f"  쿼터니언 크기: {per['quat_norm']['min']:.4f} ~ {per['quat_norm']['max']:.4f} (전체 {per['n_samples']} 개,"
          f" 이와 별도로 데이터가 오기 전 타임스탬프 0 인 샘플 {n_zero} 개는 뺌)", ""]

    # 2. 가만히 흔들림
    if "still" in ev:
        p = seg[:, R_POS]
        q0 = mean_quat(seg[:, R_QUAT])
        R0 = quat_to_R(q0)
        angs = [np.degrees(np.linalg.norm(R_to_rotvec(quat_to_R(q) @ R0.T))) for q in seg[:, R_QUAT]]
        st = {"pos_std_mm": (p.std(axis=0) * 1000).tolist(), "pos_ptp_mm": (np.ptp(p, axis=0) * 1000).tolist(),
              "rot_std_deg": float(np.std(angs)), "rot_max_deg": float(np.max(angs))}
        summary["still"] = st
        L += ["== 2. 가만히 흔들림",
              f"  위치 표준편차 [mm] x {st['pos_std_mm'][0]:.2f}, y {st['pos_std_mm'][1]:.2f}, z {st['pos_std_mm'][2]:.2f}"
              f" (최대-최소 {st['pos_ptp_mm'][0]:.2f}, {st['pos_ptp_mm'][1]:.2f}, {st['pos_ptp_mm'][2]:.2f})",
              f"  자세 흔들림 [°] 표준편차 {st['rot_std_deg']:.3f}, 최대 {st['rot_max_deg']:.3f}", ""]

    # 3·4. 축
    from_mark = {}
    trans, rots = {}, {}

    def pose_near(t_ns):
        sel = rows[(rows[:, 0] <= t_ns) & (rows[:, 0] >= t_ns - MARK_WINDOW * 1e9)]
        if len(sel) < 3:
            return None
        return sel[:, R_POS].mean(axis=0), mean_quat(sel[:, R_QUAT])

    L += ["== 3. 이동 → 축 (bindings 좌표계, 위치 단위는 m 로 가정)"]
    for name, _ in TRANSLATIONS:
        if name not in ev:
            continue
        a, b = pose_near(ev[name]["t0_ns"]), pose_near(ev[name]["t1_ns"])
        if a is None or b is None:
            L.append(f"  {name}: 표시 직전 데이터 부족")
            continue
        d = b[0] - a[0]
        ax, purity = axis_name(d)
        trans[name] = {"delta_m": d.tolist(), "axis": ax, "purity": purity, "dist_m": float(np.linalg.norm(d))}
        L.append(f"  {name:8s}: Δ = ({d[0]:+.3f}, {d[1]:+.3f}, {d[2]:+.3f}) m, |Δ| {np.linalg.norm(d):.3f} → {ax} "
                 f"(그 축 비중 {purity:.2f})")
    if all(k in trans for k in ("forward", "right", "up")):
        f_, r_, u_ = (np.asarray(trans[k]["delta_m"]) for k in ("forward", "right", "up"))
        f_, r_, u_ = (v / np.linalg.norm(v) for v in (f_, r_, u_))
        hand = float(np.dot(np.cross(r_, u_), -f_))            # 오른손 좌표계 (x 오른쪽, y 위, z 뒤) 이면 +1
        ortho = float(max(abs(np.dot(f_, r_)), abs(np.dot(f_, u_)), abs(np.dot(r_, u_))))
        trans["handedness"] = "오른손" if hand > 0 else "왼손"
        L.append(f"  좌표계: (오른쪽 × 위) · 뒤 = {hand:+.2f} → {trans['handedness']} 좌표계 (OpenXR 는 오른손: +x 오른쪽, +y 위, −z 앞 /"
                 f" Unity 는 왼손: +z 앞). 세 방향 사이 cos 최대 {ortho:.2f} (0 에 가까울수록 직교)")
    summary["translation"] = trans
    L += ["", "== 4. 회전 → 축 (world = bindings 좌표계, local = 컨트롤러 자기 좌표계)"]
    for name, _ in ROTATIONS:
        if name not in ev:
            continue
        a, b = pose_near(ev[name]["t0_ns"]), pose_near(ev[name]["t1_ns"])
        if a is None or b is None:
            L.append(f"  {name}: 표시 직전 데이터 부족")
            continue
        Ra, Rb = quat_to_R(a[1]), quat_to_R(b[1])
        w_ = R_to_rotvec(Rb @ Ra.T)
        l_ = R_to_rotvec(Ra.T @ Rb)
        wax, wp = axis_name(w_)
        lax_, lp = axis_name(l_)
        rots[name] = {"angle_deg": float(np.degrees(np.linalg.norm(w_))), "world_rotvec": w_.tolist(), "world_axis": wax,
                      "world_purity": wp, "local_rotvec": l_.tolist(), "local_axis": lax_, "local_purity": lp,
                      "pos_drift_m": float(np.linalg.norm(b[0] - a[0]))}
        L.append(f"  {name:9s}: {rots[name]['angle_deg']:5.1f}°, world 축 {wax} ({wp:.2f}), local 축 {lax_} ({lp:.2f}),"
                 f" 그동안 위치 변화 {rots[name]['pos_drift_m'] * 1000:.0f} mm")
    summary["rotation"] = rots

    # 5. 버튼
    L += ["", "== 5. 버튼"]
    btn = {}
    for name, col, _, thr, n_need in BUTTONS:
        if name not in ev:
            continue
        e = ev[name]
        sel = rows[(rows[:, 0] >= e["t0_ns"]) & (rows[:, 0] <= e["t1_ns"]), COL[col]]
        btn[name] = {"col": col, "presses": e["presses"], "need": n_need,
                     "min": float(sel.min()) if len(sel) else None, "max": float(sel.max()) if len(sel) else None}
        L.append(f"  {name:12s} ({col}): 눌림 {e['presses']} / {n_need}, 값 범위 {btn[name]['min']} ~ {btn[name]['max']}")
    summary["buttons"] = btn

    # 6. 끊김
    loss = {}
    if any(k in ev for k, _, _ in LOSS):
        L += ["", "== 6. 끊김 (구간 = A 부터 삐 소리까지)"]
    for name, _, dur in LOSS:
        if name not in ev:
            continue
        e = ev[name]
        sel = rows[(rows[:, 0] >= e["t0_ns"]) & (rows[:, 0] <= e["t1_ns"])]
        span = (e["t1_ns"] - e["t0_ns"]) / 1e9
        d = np.diff(sel[:, 2]) / 1e9 if len(sel) > 1 else np.array([span])
        gaps = d[d > STALE_GAP]
        p = sel[:, R_POS] if len(sel) else np.zeros((0, 3))
        frozen = int(np.sum(np.all(np.diff(p, axis=0) == 0, axis=1))) if len(p) > 1 else 0
        zeros = int(np.sum(np.all(sel[:, R_POS.start:R_QUAT.stop] == 0, axis=1))) if len(sel) else 0
        rec_s = (e["recovered_ns"] - e["t1_ns"]) / 1e9 if e.get("recovered_ns") else None
        loss[name] = {"span_s": span, "n_samples": int(len(sel)), "rate_hz": float(len(sel) / span),
                      "max_gap_s": float(d.max()) if len(d) else span, "n_gaps_over": int(len(gaps)),
                      "pos_unchanged_steps": frozen, "all_zero_samples": zeros,
                      "pos_range_mm": (np.ptp(p, axis=0) * 1000).tolist() if len(p) else None,
                      "recover_after_s": rec_s}
        L.append(f"  {name:11s}: {span:.1f} s 동안 새 데이터 {len(sel)} 개 ({loss[name]['rate_hz']:.1f} Hz), 최대 간격 {loss[name]['max_gap_s']:.3f} s,"
                 f" {STALE_GAP * 1000:.0f} ms 넘는 간격 {len(gaps)} 번, 위치가 그대로인 연속 샘플 {frozen}, 전부 0 인 샘플 {zeros},"
                 f" 위치 범위 [mm] {np.round(loss[name]['pos_range_mm'], 1).tolist() if loss[name]['pos_range_mm'] else '-'},"
                 f" 삐 뒤 첫 새 데이터까지 {('%.2f s' % rec_s) if rec_s is not None else '60 s 안에 없음'}")
    summary["loss"] = loss
    return "\n".join(L) + "\n", summary


def main():
    ap = argparse.ArgumentParser(description="PICO 컨트롤러 입력 확인 (B-0)")
    ap.add_argument("--skip-loss", action="store_true", help="끊김 시험 (숨기기·내려놓기·헤드셋 벗기) 을 건너뜀")
    args = ap.parse_args()
    try:
        run(args)
    except (RuntimeError, TimeoutError) as e:
        sys.exit(f"\n[ERROR] {e}")


if __name__ == "__main__":
    main()
