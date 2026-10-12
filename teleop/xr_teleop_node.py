#!/usr/bin/env python3
# =============================================================
# xr_teleop_node.py — PICO 4 Ultra 컨트롤러로 팔·그리퍼 텔레옵 (docs/PLAN.md 5-B B-2)
#
#   컨트롤러 자세 (XRoboToolkit 바인딩, 90 Hz) → TCP 목표 → 차분 IK (ik_model.py, URDF) → /joint_command, /gripper/command
#   발행하는 것은 이 두 토픽뿐이다 (CLAUDE.md 원칙 2). sim·실물 공용 (실물은 --stamp wall)
#
#   조작 (2026-10-12 결정):
#     - 매번 같은 처음 자세로 컨트롤러를 들고 **A 버튼** → 시작. 그 순간의 컨트롤러 자세와 TCP 자세 (/joint_states 의 지금 관절각) 가 기준.
#       컨트롤러 앞쪽 (자기 좌표계 −z) 을 수평면에 내린 방향 = 로봇 base +x, 위 = base +z. 손 이동·회전의 변화량을 TCP 가 따라간다
#     - **A 를 다시** 누르면 멈춤 (명령을 더 보내지 않는다 → 팔은 마지막 목표에 머문다). 또 누르면 그 자리에서 기준을 새로 잡아 시작
#     - 트리거: close_above 를 넘으면 닫기 (1150), open_below 아래면 열기 (0). 텔레옵 중에만
#   손목이 테이블에 가까워지면 (min_wrist_height) 더 내려가는 목표는 보내지 않고 알린다 (2026-10-12: 손을 7 cm 내리자 wrist_2 가 테이블에 닿아 보호 정지)
#   시작을 거부하는 경우 (이유를 출력): 컨트롤러 끝이 수직에 가까움, /joint_states·/clock 이 안 옴, 보호 정지 중
#   스스로 멈추는 경우 (이유를 출력): 측정 관절각이 관절 목표와 크게 다름 (/sim/reset 의 순간이동, 보호 정지), /protective_stop true,
#     /joint_states 가 끊김. 계속 보내면 리셋 직후 팔이 옛 목표로 뛴다
#   추적 상실·헤드셋 연결 끊김은 자동으로 판정하지 않는다 (조작자가 보고 그 에피소드를 버린다). 타임스탬프가 멈추면 알림만 한다
#   명령은 텔레옵 중에만 보낸다 → 녹화에서 첫 명령 = A 를 누른 순간 (stage1 은 첫 명령 이전 구간을 뺀다)
#
# 실행 (ROS 2 를 source 한 터미널. PC-Service 실행 중, 헤드셋 앱 WORKING·Send. **녹화를 시작하기 전에 띄워 둔다**):
#   source /opt/ros/jazzy/setup.bash
#   ~/xrobotoolkit/venv/bin/python teleop/xr_teleop_node.py            # sim (stamp = sim time)
#   --config teleop/xr_teleop.yaml   --stamp sim|wall (설정 덮어쓰기)   --urdf <빌드한 URDF>
# 시험 (헤드셋 없이, 만든 입력으로): python3 teleop/check_xr_teleop.py
# =============================================================

import argparse
import math
import os
import sys
import threading
import time

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import ik_model as km  # noqa: E402

DEFAULT_CONFIG = os.path.join(HERE, "xr_teleop.yaml")
REQUIRED = ("rate_hz", "stamp", "topics", "hand", "scale", "rotation", "max_start_elevation_deg", "max_joint_speed", "max_lead",
            "damping", "limit_margin", "warn_pos_err", "warn_manip", "min_wrist_height", "gripper", "xr_connect_timeout", "xr_stale_warn",
            "state_stale", "desync", "status_sec")
XR_UP = np.array([0.0, 1.0, 0.0])                 # 바인딩 좌표계: 오른손, +y 위 (B-0)
CTRL_FORWARD = np.array([0.0, 0.0, -1.0])         # 컨트롤러 자기 좌표계의 앞 (B-0: +x 오른쪽, +y 위, −z 앞)


def load_config(path):
    with open(path, encoding="utf-8") as f:
        c = yaml.safe_load(f)
    for k in REQUIRED:
        if k not in c:
            raise KeyError(f"{path}: '{k}' 항목이 없습니다")
    for k in ("joint_states", "joint_command", "gripper_command", "clock", "protective_stop"):
        if k not in c["topics"]:
            raise KeyError(f"{path}: 'topics.{k}' 항목이 없습니다")
    for k in ("close_above", "open_below", "raw_open", "raw_close"):
        if k not in c["gripper"]:
            raise KeyError(f"{path}: 'gripper.{k}' 항목이 없습니다")
    if c["stamp"] not in ("sim", "wall"):
        raise ValueError(f"{path}: stamp 는 sim 또는 wall: {c['stamp']}")
    if c["hand"] != "right":
        raise ValueError(f"{path}: hand 는 지금 right 만 (A 버튼·오른쪽 트리거): {c['hand']}")
    if not c["gripper"]["open_below"] < c["gripper"]["close_above"]:
        raise ValueError(f"{path}: gripper.open_below < close_above 여야 함")
    if c["rate_hz"] < 50:
        raise ValueError(f"{path}: rate_hz 는 50 이상 (데이터셋 25 Hz 의 두 배): {c['rate_hz']}")
    c["path"] = path
    return c


# ---------------------------------------------------------------- 회전
def quat_to_R(q):
    """쿼터니언 (x, y, z, w) → 회전행렬."""
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def heading_map(R_ctrl, max_elev_deg):
    """컨트롤러 자세 → (M, 고도각 [°]). M = 바인딩 좌표계 벡터를 로봇 base 좌표계로 옮기는 회전 (3x3).
    컨트롤러 앞쪽을 수평면에 내린 방향 → base +x, 그 왼쪽 → +y, 위 (+y_xr) → +z. 끝이 수직에 가까우면 M = None."""
    f = R_ctrl @ CTRL_FORWARD
    elev = math.degrees(math.asin(max(-1.0, min(1.0, float(f @ XR_UP)))))
    if abs(elev) > max_elev_deg:
        return None, elev
    fh = f - (f @ XR_UP) * XR_UP
    fh /= np.linalg.norm(fh)
    right = np.cross(fh, XR_UP)
    return np.array([fh, -right, XR_UP]), elev


# ---------------------------------------------------------------- 입력
class XrSample:
    __slots__ = ("ts_ns", "pos", "quat", "trigger", "a")

    def __init__(self, ts_ns, pos, quat, trigger, a):
        self.ts_ns, self.pos, self.quat, self.trigger, self.a = int(ts_ns), np.asarray(pos, float), np.asarray(quat, float), float(trigger), bool(a)


class XrInput:
    """XRoboToolkit 바인딩 (오른손). 바인딩은 최신값만 준다: 새 데이터가 안 와도 옛 값이 나오고, 한 번도 안 왔으면 전부 0 (B-0)."""

    def __init__(self):
        try:
            import xrobotoolkit_sdk as xrt
        except ImportError as e:
            raise RuntimeError("xrobotoolkit_sdk 를 import 할 수 없음. ~/xrobotoolkit/venv/bin/python 으로 실행할 것 (teleop/build_xr_binding.sh)") from e
        self.xrt = xrt
        xrt.init()

    def read(self):
        x = self.xrt
        for _ in range(5):                       # 읽는 도중 새 데이터가 들어오면 다시 (타임스탬프가 앞뒤로 같을 때만 쓴다)
            ts = x.get_time_stamp_ns()
            p = x.get_right_controller_pose()
            s = XrSample(ts, p[:3], p[3:7], x.get_right_trigger(), x.get_A_button())
            if x.get_time_stamp_ns() == ts:
                break
        return s

    def close(self):
        self.xrt.close()


def wait_xr(inp, timeout):
    """헤드셋 데이터가 실제로 오는지: 타임스탬프가 증가하고 쿼터니언 크기가 1 근처. 아니면 에러 (값 0 으로 시작하지 않는다)."""
    t_end = time.monotonic() + timeout
    first, n_new, s = None, 0, None
    while time.monotonic() < t_end:
        s = inp.read()
        if s.ts_ns != 0 and abs(np.linalg.norm(s.quat) - 1.0) < 0.05:
            if first is None:
                first = s.ts_ns
            elif s.ts_ns != first:
                n_new += 1
                first = s.ts_ns
                if n_new >= 5:
                    return
        time.sleep(0.02)
    why = "값이 전부 0 (PC-Service 가 꺼져 있거나, 헤드셋이 연결되지 않았거나, 헤드셋 앱에서 Send 를 누르지 않음)" if s is None or s.ts_ns == 0 else \
        f"타임스탬프가 증가하지 않음 또는 쿼터니언 크기 {np.linalg.norm(s.quat):.3f} (헤드셋 앱 Send·컨트롤러 추적 확인)"
    raise RuntimeError(f"{timeout:g} s 안에 헤드셋 데이터가 오지 않음: {why}")


# ---------------------------------------------------------------- 노드
class XrTeleop:
    """제어 루프 (자기 스레드, wall rate_hz) + ROS 구독 (rclpy spin 스레드). inp: read() → XrSample."""

    def __init__(self, cfg, inp, urdf=km.DEFAULT_URDF, log=lambda m: print(m, flush=True)):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Float64

        self.cfg, self.inp, self.log = cfg, inp, log
        self.model = km.ArmModel(urdf)
        self.JointState, self.Float64 = JointState, Float64
        self.node = Node("xr_teleop")
        self.lock = threading.Lock()
        self.q_meas, self.t_js = None, None          # 최신 /joint_states (팔 6 관절, ARM_JOINTS 순서), 받은 wall 시각
        self.clock_msg, self.t_clock = None, None    # 최신 /clock (builtin Time), 받은 wall 시각
        self.pstop = False
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)   # 최신 값만 쓴다
        t = cfg["topics"]
        self.node.create_subscription(JointState, t["joint_states"], self._on_js, latest)
        self.node.create_subscription(Clock, t["clock"], self._on_clock, latest)
        self.node.create_subscription(Bool, t["protective_stop"], self._on_pstop, latest)
        self.pub_arm = self.node.create_publisher(JointState, t["joint_command"], 10)
        self.pub_grip = self.node.create_publisher(Float64, t["gripper_command"], 10)
        self.dt = 1.0 / float(cfg["rate_hz"])
        self.max_dq = float(cfg["max_joint_speed"]) * self.dt
        # 상태
        self.engaged = False
        self.q_cmd = None
        self.ref = None                              # (p_ctrl0, R_ctrl0, M, T_tcp0)
        self.grip_closed = False
        self.a_prev = True                           # 시작할 때 A 가 눌려 있어도 그것을 누름으로 보지 않는다
        self.xr_ts, self.t_xr = None, time.monotonic()
        self.n_sent = 0
        self.last = None                             # 마지막 IkStep
        self._warn_t = {}
        self._t_status = 0.0
        self._stop = threading.Event()
        self._spin = threading.Thread(target=self._spin_loop, daemon=True)
        self._rclpy = rclpy

    # ── ROS 콜백 (spin 스레드) ──
    def _on_js(self, m):
        d = dict(zip(m.name, m.position))
        try:
            q = np.array([d[n] for n in km.ARM_JOINTS], dtype=float)
        except KeyError as e:
            self._warn("js_name", f"/joint_states 에 관절 {e} 이 없음 (받은 이름 {list(m.name)})", 5.0)
            return
        with self.lock:
            self.q_meas, self.t_js = q, time.monotonic()

    def _on_clock(self, m):
        with self.lock:
            self.clock_msg, self.t_clock = m.clock, time.monotonic()

    def _on_pstop(self, m):
        with self.lock:
            self.pstop = bool(m.data)

    def _spin_loop(self):
        from rclpy.executors import SingleThreadedExecutor
        ex = SingleThreadedExecutor()
        ex.add_node(self.node)
        while not self._stop.is_set() and self._rclpy.ok():
            ex.spin_once(timeout_sec=0.05)

    # ── 도우미 ──
    def _warn(self, key, msg, every=2.0):
        now = time.monotonic()
        if now - self._warn_t.get(key, -1e9) >= every:
            self._warn_t[key] = now
            self.log(f"[xr_teleop] 알림: {msg}")

    def _state(self):
        with self.lock:
            return (None if self.q_meas is None else self.q_meas.copy()), self.t_js, self.clock_msg, self.t_clock, self.pstop

    def _state_problem(self, now, t_js, t_clock, pstop):
        stale = float(self.cfg["state_stale"])
        if t_js is None or now - t_js > stale:
            return f"/joint_states 가 {stale:g} s 넘게 안 옴 (sim·로봇 드라이버 확인)"
        if self.cfg["stamp"] == "sim" and (t_clock is None or now - t_clock > stale):
            return f"/clock 이 {stale:g} s 넘게 안 옴 (stamp: sim. 실물이면 --stamp wall)"
        if pstop:
            return "보호 정지 중 (/protective_stop true). 리셋 뒤 다시 시작"
        return None

    def _stamp(self, clock_msg):
        if self.cfg["stamp"] == "sim":
            return clock_msg
        return self.node.get_clock().now().to_msg()

    # ── 시작·멈춤 ──
    def _engage(self, s, q_meas):
        M, elev = heading_map(quat_to_R(s.quat), float(self.cfg["max_start_elevation_deg"]))
        if M is None:
            self.log(f"\a[xr_teleop] 시작 거부: 컨트롤러 끝이 수평에서 {elev:+.0f}° (한계 ±{self.cfg['max_start_elevation_deg']:g}°). "
                     "컨트롤러를 앞으로 향하게 든 뒤 A")
            return
        self.q_cmd = q_meas.copy()
        self.ref = (s.pos.copy(), quat_to_R(s.quat), M, self.model.fk(self.q_cmd))
        self.engaged, self.last = True, None
        self._t_status = time.monotonic()
        p = self.ref[3][:3, 3]
        self.log(f"\a[xr_teleop] 시작: TCP (base) [{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}] m, 컨트롤러 끝 고도 {elev:+.0f}°. A 를 다시 누르면 멈춤")

    def _disengage(self, why):
        self.engaged, self.ref = False, None
        self.log(f"\a[xr_teleop] 멈춤: {why} (보낸 명령 누적 {self.n_sent} 개). A 를 누르면 지금 자세에서 다시 시작")

    # ── 한 주기 ──
    def tick(self):
        now = time.monotonic()
        s = self.inp.read()
        if s.ts_ns != self.xr_ts:
            self.xr_ts, self.t_xr = s.ts_ns, now
        elif now - self.t_xr > float(self.cfg["xr_stale_warn"]):
            self._warn("xr_stale", f"헤드셋 데이터가 {now - self.t_xr:.1f} s 째 그대로 (연결 확인. 팔은 마지막 목표에 머문다)", 2.0)
        a_edge = s.a and not self.a_prev
        self.a_prev = s.a
        q_meas, t_js, clock_msg, t_clock, pstop = self._state()
        problem = self._state_problem(now, t_js, t_clock, pstop)

        if not self.engaged:
            if a_edge:
                if problem:
                    self.log(f"\a[xr_teleop] 시작 거부: {problem}")
                elif abs(np.linalg.norm(s.quat) - 1.0) > 0.05:
                    self.log(f"\a[xr_teleop] 시작 거부: 컨트롤러 자세가 올바르지 않음 (쿼터니언 크기 {np.linalg.norm(s.quat):.3f})")
                else:
                    self._engage(s, q_meas)
            if not self.engaged:
                return

        # 텔레옵 중
        if a_edge and self.last is not None:          # 시작한 그 주기의 A 는 멈춤이 아니다
            return self._disengage("A 버튼")
        if problem:
            return self._disengage(problem)
        dev = float(np.max(np.abs(q_meas - self.q_cmd)))
        if dev > float(self.cfg["desync"]):
            return self._disengage(f"측정 관절각이 관절 목표와 {math.degrees(dev):.1f}° 다름 (리셋으로 팔이 옮겨졌거나 팔이 멈춤)")

        p0, R0, M, T0 = self.ref
        p_t = T0[:3, 3] + float(self.cfg["scale"]) * (M @ (s.pos - p0))
        R_t = (M @ (quat_to_R(s.quat) @ R0.T) @ M.T) @ T0[:3, :3] if self.cfg["rotation"] else T0[:3, :3]
        st = km.ik_step(self.model, self.q_cmd, q_meas, p_t, R_t, max_dq=self.max_dq, damping=float(self.cfg["damping"]),
                        max_lead=float(self.cfg["max_lead"]), limit_margin=float(self.cfg["limit_margin"]))
        # 테이블 보호: 손목 관절 원점이 한계 높이 아래로 더 내려가는 목표는 보내지 않는다 (지난 목표를 그대로 보냄)
        z_new = float(self.model.joint_origins(st.q)[3:, 2].min())
        if z_new < float(self.cfg["min_wrist_height"]) and z_new < float(self.model.joint_origins(self.q_cmd)[3:, 2].min()):
            self._warn("table", f"손목이 테이블에 가까움 (관절 원점 높이 {z_new * 100:.1f} cm < {float(self.cfg['min_wrist_height']) * 100:g} cm). "
                                "더 내려가지 않는다. 손을 올릴 것")
            st.q = self.q_cmd.copy()
        self.q_cmd, self.last = st.q, st
        m = self.JointState()
        m.header.stamp = self._stamp(clock_msg)
        m.name = list(km.ARM_JOINTS)
        m.position = self.q_cmd.tolist()
        self.pub_arm.publish(m)
        self.n_sent += 1

        g = self.cfg["gripper"]
        want = True if s.trigger > float(g["close_above"]) else False if s.trigger < float(g["open_below"]) else self.grip_closed
        if want != self.grip_closed or self.n_sent == 1:
            self.grip_closed = want
            self.pub_grip.publish(self.Float64(data=float(g["raw_close"] if want else g["raw_open"])))

        lim = [x for x in st.limited if x.endswith(":limit")]
        if lim:
            self._warn("limit", f"관절 한계에 걸림: {', '.join(x.split(':')[0] for x in lim)}")
        if st.manip < float(self.cfg["warn_manip"]):
            self._warn("manip", f"특이점 근처 (지표 {st.manip:.4f})")
        if st.pos_err > float(self.cfg["warn_pos_err"]):
            self._warn("lag", f"TCP 목표가 팔보다 {st.pos_err * 100:.0f} cm 앞서 있음 (손이 팔보다 빠르거나 팔이 갈 수 없는 곳)")
        if now - self._t_status >= float(self.cfg["status_sec"]):
            self._t_status = now
            self.log(f"[xr_teleop] 텔레옵 중: 명령 누적 {self.n_sent} 개, TCP 목표 오차 {st.pos_err * 1000:.1f} mm / {math.degrees(st.rot_err):.1f}°, "
                     f"특이점 지표 {st.manip:.3f}, 그리퍼 {'닫기' if self.grip_closed else '열기'}")

    # ── 실행 ──
    def start(self):
        self._spin.start()

    def run(self, should_stop=lambda: False):
        """rate_hz 로 tick. 한 주기를 넘기면 밀린 만큼 몰아 보내지 않고 다음 주기로 간다."""
        self.start()
        nxt = time.monotonic()
        while not should_stop() and self._rclpy.ok():
            self.tick()
            nxt += self.dt
            d = nxt - time.monotonic()
            if d > 0:
                time.sleep(d)
            else:
                nxt = time.monotonic()

    def shutdown(self):
        self._stop.set()
        if self._spin.is_alive():
            self._spin.join(timeout=1.0)
        self.node.destroy_node()


def main():
    ap = argparse.ArgumentParser(description="PICO 4 Ultra 컨트롤러 팔 텔레옵 노드 (PLAN 5-B B-2)")
    ap.add_argument("--config", default=DEFAULT_CONFIG)
    ap.add_argument("--stamp", choices=("sim", "wall"), default=None, help="/joint_command stamp (설정 덮어쓰기. sim = /clock, wall = PC 시각)")
    ap.add_argument("--urdf", default=km.DEFAULT_URDF)
    args = ap.parse_args()
    cfg = load_config(args.config)
    if args.stamp:
        cfg["stamp"] = args.stamp

    import rclpy
    try:
        inp = XrInput()
    except RuntimeError as e:
        sys.exit(f"[ERROR] {e}")
    try:
        wait_xr(inp, float(cfg["xr_connect_timeout"]))
    except RuntimeError as e:
        inp.close()                              # SDK 스레드를 닫지 않고 끝내면 프로세스가 abort 한다
        sys.exit(f"\n[ERROR] {e}")
    rclpy.init()
    node = XrTeleop(cfg, inp, urdf=args.urdf)
    print(f"[xr_teleop] 준비: {cfg['rate_hz']:g} Hz, stamp {cfg['stamp']}, 배율 {cfg['scale']:g}, 회전 {'따라감' if cfg['rotation'] else '안 함'}. "
          "같은 처음 자세로 컨트롤러를 앞으로 향하게 들고 A 버튼 → 시작 / 다시 A → 멈춤. 트리거 = 그리퍼. Ctrl+C 로 종료", flush=True)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        inp.close()
        if rclpy.ok():
            rclpy.shutdown()
        print(f"\n[xr_teleop] 종료 (보낸 명령 {node.n_sent} 개)", flush=True)


if __name__ == "__main__":
    main()
