#!/usr/bin/env python3
# =============================================================
# check_xr_teleop.py — 텔레옵 노드 (xr_teleop_node.py) 를 헤드셋 없이 시험 (docs/PLAN.md 5-B B-2·B-3)
#
#   컨트롤러 입력을 스크립트가 만들어 (ScriptedInput) 진짜 노드 코드 (XrTeleop) 에 넣고, 실행 중인 sim 에서 결과를 잰다.
#   TCP 위치·자세는 /joint_states 의 FK (ik_model, sim 과 0.03 mm 이내) 로 본다. 만든 컨트롤러는 좌표계 축에서 35° 돌아가 있고
#   끝이 20° 들려 있다 (B-0 처럼 "앞" 이 축과 나란하지 않은 경우)
#     (아래 "텔레옵 버튼" = 설정 teleop_button, 지금 Y)
#     1 수직 거부      끝을 80° 든 채 텔레옵 버튼 → 명령이 나가지 않음
#     2 시작           텔레옵 버튼 → 명령이 나가고, 손을 안 움직이면 TCP 가 그대로
#     3 앞으로 10 cm   → TCP base +x 10 cm × 배율 (scale)
#     4 위 8 cm·오른쪽 5 cm → TCP +z 8 cm, −y 5 cm (× 배율)
#     5 왼쪽으로 20° 돌림 → TCP 가 base z 축으로 +20°, 위치 그대로
#     5b 손목 비틀기 ±25° → TCP 가 25° 돌고 보호 정지 없음 (손목 마운트 ↔ 팔뚝 자기 충돌이 없는 시작 자세인지)
#     6 그리퍼         트리거 1.0 → /gripper/target 1150, 0.5 → 그대로 (히스테리시스), 0.0 → 0
#     7 명령 간격      보낸 간격 (wall) 중앙값 ≈ 1 / rate_hz, stamp 0 없음·뒤로 가지 않음
#     8 멈춤·다시 시작 텔레옵 버튼 → 명령 멈춤. 손을 30 cm 옮긴 뒤 텔레옵 버튼 → TCP 가 뛰지 않음
#     9 리셋           텔레옵 중 /sim/reset → 노드가 스스로 멈추고, 리셋 뒤 팔이 시작 자세에 그대로
#    10 보호 정지      전체 동안 /protective_stop true 0 번
#    11 버튼 요청      설정 buttons 의 버튼 (A·B·X) 을 0.5 s 씩 누름 → /teleop/request 에 그 요청이 한 번씩 (누르고 있어도 한 번), 텔레옵은 그대로
#                     (녹화 도구가 받아서 하는 일 (리셋·녹화·모터 정지) 은 이 시험에 없다)
#
# 실행 (sim 을 먼저 띄운다. 기하 제어기 드론이면 리셋이 약 2 s):
#   source /opt/ros/jazzy/setup.bash
#   ~/isaacsim/python.sh isaacsim/scripts/sim_ros2.py --headless --flight geometric --init-pose under_drone   # 다른 터미널
#   python3 teleop/check_xr_teleop.py
# 결과: teleop/reports/check_xr_teleop_<시각>.txt (git 제외). 하나라도 FAIL 이면 종료 코드 1
# =============================================================

import argparse
import datetime
import math
import os
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import ik_model as km  # noqa: E402
import xr_teleop_node as tn  # noqa: E402

LINES, RESULTS = [], []


def log(s=""):
    print(s, flush=True)
    LINES.append(s)


def result(name, ok, detail):
    RESULTS.append((name, bool(ok)))
    log(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")


def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


class ScriptedInput:
    """만든 컨트롤러. 조작자 기준 (앞, 오른쪽, 위) 로 위치를, yaw·pitch 로 자세를 준다. 읽을 때마다 타임스탬프가 증가한다."""

    def __init__(self, teleop_button, heading_deg=35.0, pitch_deg=20.0):
        self.tb = teleop_button
        self.h = math.radians(heading_deg)
        self.lock = threading.Lock()
        self.fru = np.zeros(3)             # 앞·오른쪽·위 [m]
        self.yaw, self.pitch = 0.0, math.radians(pitch_deg)   # yaw: 위 축으로 왼쪽 + [rad], pitch: 끝을 듦 + [rad]
        self.roll = 0.0                    # 손목 비틀기: 조작자가 볼 때 시계 방향 + [rad] (컨트롤러 앞 축 둘레)
        self.trigger, self.a = 0.0, False   # a: 텔레옵 버튼을 누름
        self.buttons = {}                  # 그 밖의 버튼 {'B': True, ...}
        Rh = Ry(self.h)
        self.fwd = Rh @ np.array([0.0, 0.0, -1.0])
        self.right = np.cross(self.fwd, np.array([0.0, 1.0, 0.0]))
        self.origin = np.array([0.1, 1.2, -0.3])

    def read(self):
        with self.lock:
            pos = self.origin + self.fru[0] * self.fwd + self.fru[1] * self.right + self.fru[2] * np.array([0.0, 1.0, 0.0])
            R = Ry(self.h + self.yaw) @ Rx(self.pitch) @ Rz(-self.roll)
            return tn.XrSample(time.monotonic_ns(), pos, km.R_to_quat(R), self.trigger, {**self.buttons, self.tb: self.a})

    def set(self, **kw):
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, v)


def main():
    ap = argparse.ArgumentParser(description="텔레옵 노드 시험 (만든 컨트롤러 입력, 실행 중인 sim)")
    ap.add_argument("--config", default=tn.DEFAULT_CONFIG)
    ap.add_argument("--report-dir", default=os.path.join(HERE, "reports"))
    args = ap.parse_args()
    cfg = tn.load_config(args.config)

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger

    rclpy.init()
    model = km.ArmModel()
    n = Node("check_xr_teleop")
    st = {"q": None, "cmd": [], "grip_t": None, "pstop_true": 0, "pstop": None, "req": []}

    def on_js(m):
        d = dict(zip(m.name, m.position))
        st["q"] = np.array([d[k] for k in km.ARM_JOINTS])

    def on_cmd(m):
        st["cmd"].append((time.monotonic(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, np.array(m.position)))

    def on_ps(m):
        st["pstop"] = m.data
        st["pstop_true"] += int(m.data)

    t = cfg["topics"]
    n.create_subscription(JointState, t["joint_states"], on_js, 10)
    n.create_subscription(JointState, t["joint_command"], on_cmd, 1000)
    n.create_subscription(JointState, "/gripper/target", lambda m: st.__setitem__("grip_t", m.position[0]), 10)
    n.create_subscription(Bool, t["protective_stop"], on_ps, 10)
    n.create_subscription(String, t["request"], lambda m: st["req"].append(m.data), 10)
    reset_cli = n.create_client(Trigger, "/sim/reset")

    def pump(sec, fn=None):
        """sec 동안 시험 노드를 돌린다. fn(u) (u = 0 → 1, 최소 jerk) 로 만든 컨트롤러를 움직인다."""
        t0 = time.monotonic()
        while True:
            u = min((time.monotonic() - t0) / sec, 1.0)
            if fn is not None:
                fn(10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5)
            rclpy.spin_once(n, timeout_sec=0.004)
            if u >= 1.0:
                return

    def reset():
        f = reset_cli.call_async(Trigger.Request())
        t0 = time.monotonic()
        while not f.done():
            rclpy.spin_once(n, timeout_sec=0.05)
            if time.monotonic() - t0 > 240:
                raise RuntimeError("/sim/reset 응답 없음 (240 s)")
        if not f.result().success:
            raise RuntimeError(f"/sim/reset 실패: {f.result().message[:300]}")

    log(f"check_xr_teleop  {datetime.datetime.now().isoformat(timespec='seconds')}  설정 {cfg['path']}")
    t0 = time.monotonic()
    while st["q"] is None or not reset_cli.service_is_ready():
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.monotonic() - t0 > 20:
            sys.exit("[ERROR] sim 이 없음 (/joint_states·/sim/reset). sim_ros2.py 를 먼저 띄울 것")
    reset()
    pump(1.0)
    q_init = st["q"].copy()
    T_init = model.fk(q_init)

    inp = ScriptedInput(cfg["teleop_button"])
    node_log = []
    tele = tn.XrTeleop(cfg, inp, log=lambda m: (node_log.append(m), print("    " + m.replace("\a", ""), flush=True)))
    stop = threading.Event()
    th = threading.Thread(target=lambda: tele.run(stop.is_set), daemon=True)
    th.start()
    pump(1.0)

    def press_a():
        inp.set(a=True)
        pump(0.15)
        inp.set(a=False)
        pump(0.15)

    def tcp():
        return model.fk(st["q"])

    try:
        # 1 수직 거부
        n0 = len(st["cmd"])
        inp.set(pitch=math.radians(80))
        press_a()
        pump(1.0)
        result("1 수직 거부", len(st["cmd"]) == n0 and not tele.engaged, f"끝을 80° 든 채 텔레옵 버튼 → 명령 {len(st['cmd']) - n0} 개 (기준 0)")
        inp.set(pitch=math.radians(20))

        # 2 시작
        press_a()
        pump(1.5)
        d = np.linalg.norm(tcp()[:3, 3] - T_init[:3, 3])
        result("2 시작", tele.engaged and len(st["cmd"]) - n0 > 50 and d < 1e-3,
               f"텔레옵 버튼 → 명령 {len(st['cmd']) - n0} 개 (1.8 s), 손을 안 움직였을 때 TCP 이동 {d * 1000:.2f} mm (기준 < 1)")
        i_start = n0

        # 3 앞으로 10 cm
        lead = [0.0]

        def track():
            if st["cmd"]:
                lead[0] = max(lead[0], float(np.max(np.abs(st["cmd"][-1][2] - st["q"]))))

        pump(2.0, lambda u: (inp.set(fru=np.array([0.10 * u, 0, 0])), track()))
        pump(1.5)
        dp = tcp()[:3, 3] - T_init[:3, 3]
        k = float(cfg["scale"])                # 손 이동 × 배율 = TCP 이동
        result("3 앞으로 10 cm", np.linalg.norm(dp - k * np.array([0.10, 0, 0])) < 3e-3,
               f"TCP 이동 (base) [{dp[0] * 1000:+.1f}, {dp[1] * 1000:+.1f}, {dp[2] * 1000:+.1f}] mm (기준 [{100 * k:+.0f}, 0, 0] ± 3, 배율 {k:g}), 이동 중 명령 − 측정 최대 {math.degrees(lead[0]):.2f}°")

        # 4 위 8 cm, 오른쪽 5 cm
        pump(2.0, lambda u: inp.set(fru=np.array([0.10, 0.05 * u, 0.08 * u])))
        pump(1.5)
        dp = tcp()[:3, 3] - T_init[:3, 3]
        result("4 위 8 cm·오른쪽 5 cm", np.linalg.norm(dp - k * np.array([0.10, -0.05, 0.08])) < 3e-3,
               f"TCP 이동 (base) [{dp[0] * 1000:+.1f}, {dp[1] * 1000:+.1f}, {dp[2] * 1000:+.1f}] mm (기준 [{100 * k:+.0f}, {-50 * k:+.0f}, {80 * k:+.0f}] ± 3)")

        # 5 왼쪽으로 20° 돌림
        p_before = tcp()[:3, 3].copy()
        pump(2.0, lambda u: inp.set(yaw=math.radians(20) * u))
        pump(1.5)
        T = tcp()
        w = km.R_to_rotvec(T[:3, :3] @ T_init[:3, :3].T)
        dpos = np.linalg.norm(T[:3, 3] - p_before)
        result("5 왼쪽으로 20° 돌림", abs(math.degrees(w[2]) - 20) < 1.0 and np.linalg.norm(w[:2]) < math.radians(1.0) and dpos < 3e-3,
               f"TCP 회전 벡터 (base) [{math.degrees(w[0]):+.2f}, {math.degrees(w[1]):+.2f}, {math.degrees(w[2]):+.2f}]° (기준 [0, 0, +20] ± 1), 위치 변화 {dpos * 1000:.2f} mm (기준 < 3)")

        # 5b 손목 비틀기 ±25° (2026-10-12: 처음 시작 자세에서는 한쪽으로 10° 만 비틀어도 손목 마운트가 팔뚝에 닿아 보호 정지)
        T_a = tcp()
        pump(1.0, lambda u: inp.set(roll=math.radians(25) * u))
        pump(1.0)
        w_cw = km.R_to_rotvec(tcp()[:3, :3] @ T_a[:3, :3].T)
        pump(2.0, lambda u: inp.set(roll=math.radians(25 - 50 * u)))
        pump(1.0)
        w_ccw = km.R_to_rotvec(tcp()[:3, :3] @ T_a[:3, :3].T)
        pump(1.0, lambda u: inp.set(roll=math.radians(-25 + 25 * u)))
        pump(1.0)
        a_cw, a_ccw = math.degrees(np.linalg.norm(w_cw)), math.degrees(np.linalg.norm(w_ccw))
        result("5b 손목 비틀기 ±25°", abs(a_cw - 25) < 1.5 and abs(a_ccw - 25) < 1.5 and st["pstop_true"] == 0 and tele.engaged,
               f"TCP 회전 {a_cw:.1f}° / {a_ccw:.1f}° (기준 25 ± 1.5), 보호 정지 {st['pstop_true']} 번, 텔레옵 {'중' if tele.engaged else '멈춤'}")

        # 6 그리퍼
        inp.set(trigger=1.0)
        pump(1.0)
        g1 = st["grip_t"]
        inp.set(trigger=0.5)
        pump(0.7)
        g2 = st["grip_t"]
        inp.set(trigger=0.0)
        pump(1.0)
        g3 = st["grip_t"]
        result("6 그리퍼", g1 == 1150.0 and g2 == 1150.0 and g3 == 0.0, f"/gripper/target: 트리거 1.0 → {g1}, 0.5 → {g2}, 0.0 → {g3} (기준 1150, 1150, 0)")

        # 7 명령 간격
        c = st["cmd"][i_start:]
        dw = np.diff([x[0] for x in c]) * 1000
        stamps = np.array([x[1] for x in c])
        dq = np.max(np.abs(np.diff(np.array([x[2] for x in c]), axis=0)))
        ds = np.diff(stamps) * 1000
        ok7 = abs(np.median(dw) - 1000 / cfg["rate_hz"]) < 2 and np.all(stamps > 0) and np.all(ds >= 0) and dq <= tele.max_dq + 1e-9
        result("7 명령 간격", ok7,
               f"{len(c)} 개, 받은 간격 (wall) 중앙값 {np.median(dw):.2f} ms·95 % {np.percentile(dw, 95):.2f}·최대 {dw.max():.2f} (기준 중앙값 {1000 / cfg['rate_hz']:.2f} ± 2), "
               f"stamp 0 인 것 {int(np.sum(stamps <= 0))} 개, stamp 간격 최대 {ds.max():.1f} ms (40 ms 넘은 것 {int(np.sum(ds > 40))} 번), 뒤로 간 것 {int(np.sum(ds < 0))} 번, "
               f"한 주기 관절 변화 최대 {dq:.4f} rad (한계 {tele.max_dq:.4f})")

        # 8 멈춤·다시 시작
        press_a()
        n1 = len(st["cmd"])
        p_stop = tcp()[:3, 3].copy()
        pump(1.0, lambda u: inp.set(fru=np.array([0.10 - 0.30 * u, 0.05, 0.08])))
        stopped = len(st["cmd"]) - n1
        press_a()
        pump(1.5)
        jump = np.linalg.norm(tcp()[:3, 3] - p_stop)
        result("8 멈춤·다시 시작", stopped == 0 and tele.engaged and jump < 2e-3,
               f"텔레옵 버튼 → 멈춘 1 s 동안 명령 {stopped} 개 (기준 0). 손을 30 cm 옮긴 뒤 다시 → TCP 이동 {jump * 1000:.2f} mm (기준 < 2)")

        # 9 리셋
        reset()
        pump(0.5)
        n2 = len(st["cmd"])
        pump(2.0)
        err = math.degrees(float(np.max(np.abs(st["q"] - q_init))))
        result("9 리셋", not tele.engaged and len(st["cmd"]) == n2 and err < 1.0 and st["pstop"] is False,
               f"텔레옵 중 /sim/reset → 노드 텔레옵 {'중' if tele.engaged else '멈춤'}, 리셋 뒤 2 s 동안 명령 {len(st['cmd']) - n2} 개 (기준 0), "
               f"팔이 시작 자세에서 {err:.3f}° (기준 < 1), 보호 정지 {st['pstop']}")

        # 11 버튼 요청 (리셋 뒤 텔레옵이 멈춘 상태에서. 요청은 텔레옵 중이 아니어도 나가야 한다)
        got = {}
        btns = sorted(cfg["buttons"].values())
        for btn in btns:
            n3 = len(st["req"])
            inp.set(buttons={btn: True})
            pump(0.5)
            inp.set(buttons={})
            pump(0.3)
            got[btn] = st["req"][n3:]
        want = {b: [r for r, bb in cfg["buttons"].items() if bb == b] for b in btns}
        result("11 버튼 요청", got == want and not tele.engaged,
               f"0.5 s 누름 → /teleop/request: {', '.join(f'{b} {got[b]}' for b in btns)} (기준 {', '.join(f'{b} {want[b]}' for b in btns)}), "
               f"텔레옵 {'중' if tele.engaged else '멈춤 그대로'}")

        # 10 보호 정지
        result("10 보호 정지", st["pstop_true"] == 0, f"/protective_stop true {st['pstop_true']} 번 (기준 0)")
    finally:
        stop.set()
        th.join(timeout=2.0)
        tele.shutdown()

    ok = all(r for _, r in RESULTS)
    log(f"\n결과: {sum(r for _, r in RESULTS)}/{len(RESULTS)} PASS")
    os.makedirs(args.report_dir, exist_ok=True)
    path = os.path.join(args.report_dir, f"check_xr_teleop_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(LINES) + "\n\n[노드 출력]\n" + "\n".join(m.replace("\a", "") for m in node_log) + "\n")
    print(f"리포트: {path}", flush=True)
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
