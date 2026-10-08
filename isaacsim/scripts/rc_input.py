#!/usr/bin/env python3
# =============================================================
# rc_input.py — 실물 조종기 입력 읽기 (docs/PLAN.md 5단계 5-A, docs/drone_teleop.md)
#
#   Radiolink 조종기 → 수신기 → Raspberry Pi Pico (MicroPython) → USB 직렬. Pico 가 한 줄씩 보낸다:
#       RC,0,0,1006,1002,995,996,200,200,200,1800
#     머리말 "RC" + 정수 10 개. 3~6 번째 값이 스틱 4 축 (2026-10-08 사용자 확인). 나머지 값의 뜻·범위·주기는 A-0 에서 확인
#   parse_line: 한 줄 → 정수 N_VALUES 개. 형식이 다르면 ValueError (조용히 넘기지 않음)
#   RcReader  : 직렬을 스레드로 읽어 (받은 시각, 값) 을 쌓는다. 형식이 다른 줄은 따로 모은다
#   RcMap     : 값 10 개 → 스틱 (pitch, roll, throttle, yaw) −1 ~ 1. 신호 끊김 (상태 값)·범위 밖이면 None + 이유 (config/rc_input.yaml)
#   RcInput   : RcReader + RcMap. sim 이 루프마다 poll() (줄이 안 오면 None + 이유)
#
#   A-0 입력 확인 (이 파일을 직접 실행, sim 없이):
#       python3 isaacsim/scripts/rc_input.py                 # 안내에 따라 스틱·스위치를 움직인다 (약 2 분)
#       python3 isaacsim/scripts/rc_input.py --watch          # 값만 계속 보기 (Ctrl+C 로 끝)
#     안내마다 기다렸다가 (--wait) 그 뒤 --hold 초 동안의 값을 잰다 → 어느 값이 어느 스틱·방향인지, 범위, 줄 주기,
#     조종기를 껐을 때 출력. 결과는 isaacsim/reports/rc_probe_<시각>/ (report.txt, summary.json, raw.csv)
#   권한: /dev/ttyACM* 는 root:dialout → 계정을 dialout 그룹에 넣거나 (sudo usermod -aG dialout $USER, 다시 로그인) sudo 로 실행
# =============================================================

import argparse
import datetime
import json
import os
import sys
import threading
import time

HEADER = "RC"
N_VALUES = 10
DEFAULT_PORT = "/dev/ttyACM0"
REPORT_ROOT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "reports")


def parse_line(line):
    """'RC,v1,...,v10' → [int] * 10. 형식이 다르면 ValueError."""
    parts = line.strip().split(",")
    if parts[0] != HEADER:
        raise ValueError(f"머리말이 {HEADER!r} 가 아님: {line!r}")
    if len(parts) != 1 + N_VALUES:
        raise ValueError(f"값이 {N_VALUES} 개가 아님 ({len(parts) - 1} 개): {line!r}")
    try:
        return [int(p) for p in parts[1:]]
    except ValueError:
        raise ValueError(f"정수가 아닌 값: {line!r}") from None


class RcReader:
    """직렬을 스레드로 읽는다. samples = [(받은 시각 time.monotonic(), 값 10 개)], bad = [(시각, 줄, 이유)].
    표준 라이브러리 (termios) 만 쓴다: Isaac Sim 의 python 에는 pyserial 이 없다."""

    def __init__(self, port, baudrate=115200):
        import termios
        import tty

        self.fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            tty.setraw(self.fd)                       # 줄 편집·echo·문자 변환 끔 (받은 바이트 그대로)
            attr = termios.tcgetattr(self.fd)
            speed = getattr(termios, f"B{int(baudrate)}", None)
            if speed is None:
                raise ValueError(f"지원하지 않는 baudrate: {baudrate}")
            attr[4] = attr[5] = speed
            attr[2] |= termios.CLOCAL | termios.CREAD
            termios.tcsetattr(self.fd, termios.TCSANOW, attr)
        except Exception:
            os.close(self.fd)
            raise
        self.samples, self.bad = [], []
        self.error = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        import termios

        termios.tcflush(self.fd, termios.TCIFLUSH)    # 열기 전에 쌓인 옛 줄을 버림
        self._thread.start()

    def _run(self):
        import select

        buf, first = b"", True
        try:
            while not self._stop.is_set():
                if not select.select([self.fd], [], [], 0.2)[0]:
                    continue
                chunk = os.read(self.fd, 4096)
                if not chunk:
                    raise OSError("직렬 장치가 닫힘 (뽑힘?)")
                t = time.monotonic()
                buf += chunk
                *raws, buf = buf.split(b"\n")
                for raw in raws:
                    if first:                         # 버퍼를 비운 직후의 첫 줄은 중간부터일 수 있음
                        first = False
                        continue
                    line = raw.decode("ascii", errors="replace").strip()
                    if not line:
                        continue
                    try:
                        self.samples.append((t, parse_line(line)))
                    except ValueError as e:
                        self.bad.append((t, line, str(e)))
        except Exception as e:  # noqa: BLE001  (장치가 뽑힘 등 → 읽는 쪽에서 다시 던짐)
            self.error = e

    def check(self):
        if self.error is not None:
            raise RuntimeError(f"직렬 읽기 에러: {self.error!r}") from self.error

    def latest(self):
        return self.samples[-1] if self.samples else None

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        os.close(self.fd)


AXES = ("pitch", "roll", "throttle", "yaw")       # PX4Commander.set_sticks 순서


def load_config(path):
    import yaml

    with open(path, encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    for k in ("port", "baudrate", "stale_sec", "lost_flags", "axes", "range_margin", "deadzone", "handover_center", "px4_params"):
        if k not in cfg:
            raise KeyError(f"{path}: '{k}' 항목이 없습니다")
    used = set()
    for name in AXES:
        if name not in cfg["axes"]:
            raise KeyError(f"{path}: axes.{name} 이 없습니다")
        ax = cfg["axes"][name]
        if not (1 <= ax["index"] <= N_VALUES) or ax["index"] in used or ax["index"] in cfg["lost_flags"]:
            raise ValueError(f"{path}: axes.{name}.index {ax['index']} 가 범위 밖이거나 겹침")
        used.add(ax["index"])
        if not (ax["min"] < ax["center"] < ax["max"]) or ax["sign"] not in (1, -1):
            raise ValueError(f"{path}: axes.{name} 은 min < center < max, sign ±1: {ax}")
    cfg["path"] = path
    return cfg


class RcMap:
    """직렬 값 10 개 → 스틱 (pitch, roll, throttle, yaw) −1 ~ 1 (가운데 0). 신호 끊김·범위 밖이면 (None, 이유)."""

    def __init__(self, cfg):
        self.cfg = cfg

    def sticks(self, values):
        c = self.cfg
        if any(values[i - 1] != 0 for i in c["lost_flags"]):
            return None, f"조종기 신호 끊김 (상태 값 {[values[i - 1] for i in c['lost_flags']]})"
        out = []
        for name in AXES:
            ax = c["axes"][name]
            v = values[ax["index"] - 1]
            if not (ax["min"] - c["range_margin"] <= v <= ax["max"] + c["range_margin"]):
                return None, f"{name} 값 {v} 가 범위 {ax['min']} ~ {ax['max']} 밖"
            x = (v - ax["center"]) / ((ax["max"] - ax["center"]) if v >= ax["center"] else (ax["center"] - ax["min"]))
            x = max(-1.0, min(1.0, x)) * ax["sign"]
            out.append(0.0 if abs(x) < c["deadzone"] else x)
        return tuple(out), None

    def centered(self, sticks):
        return all(abs(x) <= self.cfg["handover_center"] for x in sticks)


class RcInput:
    """조종기 입력 한 벌: 직렬 읽기 (RcReader) + 환산 (RcMap). poll() → (스틱 또는 None, 이유).
    줄이 stale_sec (실제 시간) 넘게 안 오면 입력 없음."""

    def __init__(self, cfg, port=None):
        self.cfg, self.map = cfg, RcMap(cfg)
        self.reader = RcReader(cfg["port"] if port is None else port, cfg["baudrate"])
        self.reader.start()

    def poll(self):
        self.reader.check()
        last = self.reader.latest()
        del self.reader.samples[:-1]              # 최신 줄만 쓴다 (142.9 Hz 로 쌓임)
        if last is None:
            return None, "받은 줄 없음"
        age = time.monotonic() - last[0]
        if age > self.cfg["stale_sec"]:
            return None, f"줄이 {age:.2f} s 동안 안 옴 (Pico·USB)"
        return self.map.sticks(last[1])

    def close(self):
        self.reader.close()


# ── A-0 입력 확인 ──
STICK_STEPS = [          # (이름, 안내)
    ("center", "두 스틱을 모두 가운데에 두세요 (스로틀 스틱이 안 돌아오면 손으로 가운데에)"),
    ("left_up", "왼쪽 스틱을 위로 끝까지 밀고 그대로"),
    ("left_down", "왼쪽 스틱을 아래로 끝까지 당기고 그대로"),
    ("left_left", "왼쪽 스틱을 왼쪽 끝까지 밀고 그대로 (위아래는 가운데)"),
    ("left_right", "왼쪽 스틱을 오른쪽 끝까지 밀고 그대로 (위아래는 가운데)"),
    ("right_up", "오른쪽 스틱을 위로 끝까지 밀고 그대로"),
    ("right_down", "오른쪽 스틱을 아래로 끝까지 당기고 그대로"),
    ("right_left", "오른쪽 스틱을 왼쪽 끝까지 밀고 그대로"),
    ("right_right", "오른쪽 스틱을 오른쪽 끝까지 밀고 그대로"),
]


def median(xs):
    s = sorted(xs)
    return s[len(s) // 2]


def window(reader, t0, t1):
    return [(t, v) for t, v in reader.samples if t0 <= t < t1]


def countdown(reader, text, wait):
    print(f"\n▶ {text}", flush=True)
    t_end = time.monotonic() + wait
    while True:
        left = t_end - time.monotonic()
        if left <= 0:
            break
        last = reader.latest()
        vals = " ".join(f"{x:5d}" for x in last[1]) if last else "(받은 줄 없음)"
        print(f"\r   {left:3.0f} s 뒤 측정  | {vals}", end="", flush=True)
        reader.check()
        time.sleep(0.1)
    print("\r   측정 중 ...".ljust(100), end="", flush=True)


def measure(reader, text, wait, hold):
    """안내 → wait 초 기다림 → hold 초 동안 받은 값. 돌려줌: {"n", "median", "min", "max"} (받은 줄이 없으면 n = 0)."""
    countdown(reader, text, wait)
    t0 = time.monotonic()
    time.sleep(hold)
    reader.check()
    rows = [v for _, v in window(reader, t0, t0 + hold)]
    if not rows:
        print("\r   받은 줄 없음".ljust(100))
        return {"n": 0}
    cols = list(zip(*rows))
    res = {"n": len(rows), "median": [median(c) for c in cols], "min": [min(c) for c in cols], "max": [max(c) for c in cols]}
    print("\r   중앙값: " + " ".join(f"{x:5d}" for x in res["median"]) + f"   ({len(rows)} 줄)".ljust(20))
    return res


def rate_stats(samples):
    if len(samples) < 2:
        return None
    dt = sorted(b[0] - a[0] for a, b in zip(samples, samples[1:]))
    span = samples[-1][0] - samples[0][0]
    return {"hz": (len(samples) - 1) / span, "dt_median_ms": dt[len(dt) // 2] * 1e3, "dt_max_ms": dt[-1] * 1e3}


def probe(reader, out_dir, wait, hold, switch_sec, off_sec):
    lines = []

    def say(s=""):
        print(s, flush=True)
        lines.append(s)

    t_begin = time.monotonic()
    print(f"조종기를 켜세요. 안내가 나오면 그 자세를 만들고 **'중앙값' 줄이 나올 때까지 유지**하세요 (안내 {wait:.0f} s 뒤부터 {hold:.0f} s 동안 잽니다).")
    steps = {name: measure(reader, text, wait, hold) for name, text in STICK_STEPS}
    for name, res in steps.items():
        if res["n"] == 0:
            raise RuntimeError(f"'{name}' 단계에서 받은 줄이 없음 (조종기·수신기·Pico 연결 확인)")

    # 스위치: 값이 바뀐 열과 그 열이 가진 값들
    countdown(reader, f"스위치·다이얼을 하나씩 천천히 끝에서 끝으로 움직이세요 ({switch_sec:.0f} s 동안, 스틱은 가운데)", wait)
    t0 = time.monotonic()
    time.sleep(switch_sec)
    reader.check()
    sw_rows = [v for _, v in window(reader, t0, t0 + switch_sec)]
    print("\r   끝".ljust(100))

    # 조종기 끔: 줄이 계속 오는지, 값이 어떻게 되는지
    countdown(reader, f"조종기 전원을 끄세요 ({off_sec:.0f} s 동안 지켜봄)", wait)
    t0 = time.monotonic()
    time.sleep(off_sec)
    reader.check()
    off = window(reader, t0, t0 + off_sec)
    print("\r   끝. 조종기를 다시 켜도 됩니다".ljust(100))
    t_finish = time.monotonic()

    # ── 정리 ──
    c = steps["center"]["median"]
    say("\n================ 결과 ================")
    say(f"값 번호:           " + " ".join(f"{i + 1:5d}" for i in range(N_VALUES)))
    for name, _ in STICK_STEPS:
        say(f"{name:12s} 중앙값 " + " ".join(f"{x:5d}" for x in steps[name]["median"]))
    say()
    say("스틱 동작별로 가운데에서 가장 많이 바뀐 값 (번호는 1 부터):")
    axes = {}
    for name, _ in STICK_STEPS[1:]:
        d = [m - c0 for m, c0 in zip(steps[name]["median"], c)]
        k = max(range(N_VALUES), key=lambda i: abs(d[i]))
        others = [(i + 1, d[i]) for i in range(N_VALUES) if i != k and abs(d[i]) > 0.2 * abs(d[k])]
        axes[name] = {"value_index": k + 1, "delta": d[k], "value": steps[name]["median"][k], "also_moved": others}
        say(f"  {name:12s} → {k + 1} 번째 값 {c[k]} → {steps[name]['median'][k]} ({d[k]:+d})"
            + (f"   같이 움직인 값: {others}" if others else ""))
    say()
    jitter = [hi - lo for lo, hi in zip(steps["center"]["min"], steps["center"]["max"])]
    say("가운데에 둔 동안 흔들림 (최대 − 최소): " + " ".join(f"{x:5d}" for x in jitter))

    sw = {}
    if sw_rows:
        cols = list(zip(*sw_rows))
        for i, col in enumerate(cols):
            if max(col) - min(col) > max(20, 3 * jitter[i]):
                sw[i + 1] = {"min": min(col), "max": max(col), "levels": sorted(set(round(x, -1) for x in col))[:12]}
    say("스위치 단계에서 바뀐 값: " + (", ".join(f"{k} 번째 ({v['min']} ~ {v['max']}, 단계 {v['levels']})" for k, v in sw.items()) or "없음"))

    on = [s for s in reader.samples if s[0] < t0 - wait]
    r_on, r_off = rate_stats(on), rate_stats(off)
    say()
    say(f"줄 주기 (조종기 켬): " + (f"{r_on['hz']:.1f} Hz, 간격 중앙값 {r_on['dt_median_ms']:.1f} ms, 최대 {r_on['dt_max_ms']:.1f} ms" if r_on else "잴 수 없음"))
    if not off:
        say(f"조종기를 끈 동안: 줄이 오지 않음 ({off_sec:.0f} s)")
    else:
        cols = list(zip(*[v for _, v in off]))
        last_on = on[-1][1] if on else None
        say(f"조종기를 끈 동안: 줄 {len(off)} 개" + (f" ({r_off['hz']:.1f} Hz)" if r_off else ""))
        say("  끄기 전 마지막 값 " + (" ".join(f"{x:5d}" for x in last_on) if last_on else "-"))
        say("  끈 동안 마지막 값 " + " ".join(f"{x:5d}" for x in off[-1][1]))
        say("  끈 동안 최소      " + " ".join(f"{min(col):5d}" for col in cols))
        say("  끈 동안 최대      " + " ".join(f"{max(col):5d}" for col in cols))
    say(f"형식이 다른 줄: {len(reader.bad)} 개" + ("".join(f"\n  {line!r}: {why}" for _, line, why in reader.bad[:10])))
    say(f"받은 줄 전체: {len(reader.samples)} 개, {t_finish - t_begin:.0f} s")

    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "report.txt"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")
    with open(os.path.join(out_dir, "summary.json"), "w", encoding="utf-8") as fh:
        json.dump({"steps": steps, "axes": axes, "center_jitter": jitter, "switches": sw, "rate_on": r_on, "rate_off": r_off,
                   "off_lines": len(off), "bad_lines": [(line, why) for _, line, why in reader.bad]}, fh, ensure_ascii=False, indent=1)
    with open(os.path.join(out_dir, "raw.csv"), "w", encoding="utf-8") as fh:
        fh.write("t," + ",".join(f"v{i + 1}" for i in range(N_VALUES)) + "\n")
        for t, v in reader.samples:
            fh.write(f"{t - t_begin:.4f}," + ",".join(map(str, v)) + "\n")
    if os.environ.get("SUDO_UID"):          # sudo 로 돌렸으면 결과 파일을 원래 사용자 것으로
        uid, gid = int(os.environ["SUDO_UID"]), int(os.environ["SUDO_GID"])
        for path in (out_dir, *(os.path.join(out_dir, f) for f in os.listdir(out_dir))):
            os.chown(path, uid, gid)
    print(f"\n저장: {out_dir}")


def watch(reader):
    print("값 번호: " + " ".join(f"{i + 1:5d}" for i in range(N_VALUES)) + "    (Ctrl+C 로 끝)")
    n = 0
    while True:
        reader.check()
        last = reader.latest()
        if last is not None and len(reader.samples) != n:
            n = len(reader.samples)
            print("\r         " + " ".join(f"{x:5d}" for x in last[1]) + f"   줄 {n}, 형식 다름 {len(reader.bad)}", end="", flush=True)
        time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser(description="조종기 직렬 입력 확인 (PLAN 5-A A-0)")
    ap.add_argument("--port", default=DEFAULT_PORT, help="직렬 장치 (꽂는 순서와 무관한 이름: /dev/serial/by-id/...)")
    ap.add_argument("--watch", action="store_true", help="값만 계속 출력")
    ap.add_argument("--wait", type=float, default=5.0, help="안내 뒤 측정까지 기다리는 시간 [s]")
    ap.add_argument("--hold", type=float, default=2.0, help="측정 시간 [s]")
    ap.add_argument("--switch-sec", type=float, default=30.0, help="스위치 단계 길이 [s]")
    ap.add_argument("--off-sec", type=float, default=10.0, help="조종기를 끈 채 지켜보는 시간 [s]")
    a = ap.parse_args()
    try:
        reader = RcReader(a.port)
    except Exception as e:  # noqa: BLE001
        print(f"직렬 장치를 열 수 없음: {e}\n  권한이면: sudo usermod -aG dialout $USER (다시 로그인) 또는 sudo 로 실행", file=sys.stderr)
        return 2
    reader.start()
    try:
        if a.watch:
            watch(reader)
        else:
            stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            probe(reader, os.path.join(REPORT_ROOT, f"rc_probe_{stamp}"), a.wait, a.hold, a.switch_sec, a.off_sec)
    except KeyboardInterrupt:
        print()
    finally:
        reader.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
