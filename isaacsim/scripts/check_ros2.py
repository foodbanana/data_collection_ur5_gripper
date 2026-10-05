#!/usr/bin/env python3
# =============================================================
# check_ros2.py  (docs/PLAN.md 3-8)
#
# sim ROS 2 인터페이스 자동 검사. 시스템 python3 (source /opt/ros/jazzy/setup.bash) 로 실행하면
# sim_ros2.py --headless 를 직접 띄우고, ROS 2 로 명령을 보내고 받아 **stamp(sim time) 기준**으로 판정한다.
#   A. 기하 제어기 드론 (드론 위치가 일정 → 명령 재생이 항상 같음)
#      1 토픽 주기  2 같은 시계  3 stage1 방식 이미지 읽기  4 팔 명령 응답  5 이미지 지연  6 파지 신호  7 보호 정지
#   B. PX4 드론 (실제 사용 조건)
#      8 리셋 (goto 2 회 + 잡은 뒤 kill → PX4 재시작 1 회)  9 RTF
#   받는 쪽은 별도 스레드 executor (처리가 밀려 도착 시각이 늦게 찍히는 문제를 피함, 판정은 stamp 기준)
#   기준 명령: isaacsim/config/ros2_check/commands_success.csv (만든 방법은 같은 폴더 README.md). 계단·테이블 충돌 명령은 여기서 만든다
#
# 실행:
#   source /opt/ros/jazzy/setup.bash
#   python3 isaacsim/scripts/check_ros2.py [--part A|B|all]      # 약 8~10 분, 리포트 isaacsim/reports/check_ros2_<시각>/
# =============================================================

import argparse
import csv
import datetime
import json
import os
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Trigger

SCRIPTS = os.path.dirname(os.path.abspath(__file__))
SIM_ROOT = os.path.dirname(SCRIPTS)
REPO = os.path.dirname(SIM_ROOT)
REPORT_DIR = os.path.join(SIM_ROOT, "reports")
GRASP_CSV = os.path.join(SIM_ROOT, "config", "ros2_check", "commands_success.csv")
PYTHON_SH = os.path.expanduser("~/isaacsim/python.sh")
ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
HOME = np.array([0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])      # drive_gains.yaml home_pose
CMD_HZ = 60.0

CRIT = {
    "rate_rel": 0.01,           # 주기 ±1 %
    "step50_ms": 80.0,          # 계단 응답 50 % 도달 (보간 33 ms + drive)
    "image_lag_frames": 1,      # 이미지가 팔 움직임보다 늦는 프레임 수 최대
    "present_mid": (150.0, 1100.0),
    "home_deg": 1.0,
    "open_raw": 20.0,
    "rtf": 0.9,
}


def stamp(s):
    return s.sec + s.nanosec * 1e-9


# ── sim 프로세스 ──
class Sim:
    def __init__(self, args, log_path):
        self.log_path = log_path
        self.log = open(log_path, "w", encoding="utf-8")
        cmd = [PYTHON_SH, os.path.join(SCRIPTS, "sim_ros2.py"), "--headless", "--duration", "3000", *args]
        self.proc = subprocess.Popen(cmd, cwd=REPO, stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)

    def wait_ready(self, timeout=240.0):
        t_end = time.time() + timeout
        while time.time() < t_end:
            txt = open(self.log_path, encoding="utf-8", errors="replace").read()
            if "[sim_ros2] 준비" in txt:
                return
            if "[ERROR]" in txt or self.proc.poll() is not None:
                raise RuntimeError(f"sim 이 시작하지 못함 (로그 {self.log_path})")
            time.sleep(1.0)
        raise RuntimeError(f"sim 준비가 {timeout:g} s 안에 안 됨 (로그 {self.log_path})")

    def stop(self):
        if self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGINT)
            try:
                self.proc.wait(timeout=60)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
                self.proc.wait()
        self.log.close()


# ── ROS 2 (받는 쪽 스레드) ──
class Ros(Node):
    def __init__(self):
        super().__init__("check_ros2")
        rel = QoSProfile(depth=500, reliability=ReliabilityPolicy.RELIABLE)
        self.lock = threading.Lock()
        self.clk = None
        self.clk_wall = []                  # (wall, sim)
        self.js, self.cam, self.grip, self.ps = [], {"wrist": [], "third_view": []}, {"present": [], "target": []}, []
        self.keep_images = None             # None 이 아니면 (카메라 이름) 의 이미지를 (stamp, 회색 작은 이미지) 로 저장
        self.images = []
        self.last_img = {}
        self.create_subscription(Clock, "/clock", self._on_clock, rel)
        self.create_subscription(JointState, "/joint_states", self._on_js, rel)
        self.create_subscription(Image, "/cam/wrist/color/image_raw", lambda m: self._on_img("wrist", m), rel)
        self.create_subscription(Image, "/cam/third_view/color/image_raw", lambda m: self._on_img("third_view", m), rel)
        self.create_subscription(JointState, "/gripper/joint_states", lambda m: self._on_grip("present", m), rel)
        self.create_subscription(JointState, "/gripper/target", lambda m: self._on_grip("target", m), rel)
        self.create_subscription(Bool, "/protective_stop", self._on_ps, rel)
        self.pub_arm = self.create_publisher(JointState, "/joint_command", 10)
        self.pub_grip = self.create_publisher(Float64, "/gripper/command", 10)
        self.reset_cli = self.create_client(Trigger, "/sim/reset")

    def _on_clock(self, m):
        with self.lock:
            self.clk = stamp(m.clock)
            self.clk_wall.append((time.time(), self.clk))

    def _on_js(self, m):
        with self.lock:
            self.js.append((stamp(m.header.stamp), list(m.name), np.array(m.position), np.array(m.velocity)))

    def _on_img(self, name, m):
        st = stamp(m.header.stamp)
        with self.lock:
            self.cam[name].append(st)
            self.last_img[name] = m
            if self.keep_images == name:
                a = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height, m.step)[:, :m.width * 3]
                g = a.reshape(m.height, m.width, 3).astype(np.float32).mean(axis=2)[::4, ::4]
                self.images.append((st, g))

    def _on_grip(self, k, m):
        with self.lock:
            self.grip[k].append((stamp(m.header.stamp), list(m.name), float(m.position[0])))

    def _on_ps(self, m):
        with self.lock:
            self.ps.append((self.clk, bool(m.data)))

    # 보내기·기다리기 (메인 스레드)
    def now(self):
        with self.lock:
            return self.clk

    def wait_sim(self, sec):
        t0 = self.now()
        while self.now() < t0 + sec:
            time.sleep(0.002)

    def send_arm(self, q):
        m = JointState()
        m.name = ARM
        m.position = [float(x) for x in q]
        self.pub_arm.publish(m)

    def send_grip(self, raw):
        m = Float64()
        m.data = float(raw)
        self.pub_grip.publish(m)

    def play(self, rows):
        """rows: [(t, q6, gripper_raw)] 를 sim time 기준 CMD 주기로. 그리퍼는 값이 바뀔 때. 시작 sim 시각을 돌려준다."""
        t0 = self.now() + 0.2
        last_g = None
        for t, q, g in rows:
            while self.now() < t0 + t:
                time.sleep(0.001)
            self.send_arm(q)
            if g != last_g:
                self.send_grip(g)
                last_g = g
        return t0

    def reset(self, timeout=300.0):
        if not self.reset_cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/sim/reset 서비스가 없음")
        fut = self.reset_cli.call_async(Trigger.Request())
        t_end = time.time() + timeout
        while not fut.done():
            if time.time() > t_end:
                raise RuntimeError(f"/sim/reset 응답이 {timeout:g} s 안에 없음")
            time.sleep(0.05)
        r = fut.result()
        return r.success, json.loads(r.message)


# ── 명령 만들기 ──
def hold_rows(q, sec, t0=0.0):
    return [(t0 + i / CMD_HZ, np.asarray(q, dtype=float), 0.0) for i in range(1, int(sec * CMD_HZ) + 1)]


def grasp_rows():
    with open(GRASP_CSV, encoding="utf-8") as f:
        return [(float(r["t"]), np.array([float(r[j]) for j in ARM]), float(r["gripper_goal_raw"])) for r in csv.DictReader(f)]


def table_rows():
    """shoulder_lift 를 6 s 동안 1.4 rad 내림 (손가락 끝이 테이블에 닿음, 3-7)."""
    out = []
    for i in range(1, int(8 * CMD_HZ) + 1):
        t = i / CMD_HZ
        u = min(t / 6.0, 1.0)
        q = HOME.copy()
        q[1] += 1.4 * u * u * (3 - 2 * u)
        out.append((t, q, 0.0))
    return out


# ── 판정 도우미 ──
class Report:
    def __init__(self, out_dir):
        self.out_dir, self.lines, self.results = out_dir, [], []

    def log(self, m=""):
        print(m, flush=True)
        self.lines.append(m)

    def item(self, name, ok, detail):
        self.results.append((name, bool(ok)))
        self.log(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")


def js_in(ros, t0, t1):
    with ros.lock:
        return [x for x in ros.js if t0 <= x[0] <= t1]


def rate_of(stamps):
    s = np.sort(np.asarray(stamps))
    if len(s) < 3:
        return 0.0, 0, float("nan")
    gaps = np.diff(s)
    return (len(s) - 1) / (s[-1] - s[0]), int(np.sum(gaps > 1.5 * np.median(gaps))), float(gaps.max())


# ── A. 기하 제어기 ──
def part_a(ros, R):
    R.log("== A. 기하 제어기 드론")
    ros.wait_sim(2.0)
    # 1·2 주기·같은 시계 (가만히 10 s)
    t0 = ros.now()
    ros.wait_sim(10.0)
    t1 = ros.now()
    with ros.lock:
        js = [x[0] for x in ros.js if t0 <= x[0] <= t1]
        clk = [s for _, s in ros.clk_wall if t0 <= s <= t1]
        cams = {k: [s for s in v if t0 <= s <= t1] for k, v in ros.cam.items()}
        grip = {k: [x[0] for x in v if t0 <= x[0] <= t1] for k, v in ros.grip.items()}
        ps = [s for s, _ in ros.ps if s is not None and t0 <= s <= t1]
    ok, parts = True, []
    for name, st, hz in [("/joint_states", js, 120.0), ("/clock", clk, 120.0), *[(f"카메라 {k}", v, 30.0) for k, v in cams.items()],
                         *[(f"그리퍼 {k}", v, 30.0) for k, v in grip.items()]]:
        r, n_gap, gmax = rate_of(st)
        good = abs(r - hz) <= CRIT["rate_rel"] * hz and n_gap == 0
        ok &= good
        parts.append(f"{name} {r:.2f} Hz (끊김 {n_gap})")
    ps_hz = (len(ps) - 1) / (t1 - t0) if len(ps) > 1 else 0.0          # Bool 은 stamp 없음 → 받은 때의 /clock 기준 개수
    ok &= abs(ps_hz - 30.0) <= 1.0
    parts.append(f"/protective_stop 약 {ps_hz:.1f} Hz")
    R.item("1 토픽 주기 (sim 기준)", ok, ", ".join(parts))
    js_set = set(np.round(js, 6))
    miss = {k: sum(1 for s in v if round(s, 6) not in js_set) for k, v in {**cams, **{f"grip_{k}": v for k, v in grip.items()}}.items()}
    R.item("2 같은 시계 (카메라·그리퍼 stamp ∈ /joint_states stamp)", all(v == 0 for v in miss.values()), f"안 맞는 개수 {miss}")

    # 3 stage1 방식 이미지 읽기
    from cv_bridge import CvBridge

    with ros.lock:
        m = ros.last_img["third_view"]
    raw = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
    bgr = CvBridge().imgmsg_to_cv2(m, desired_encoding="bgr8")
    same = m.encoding == "rgb8" and np.array_equal(bgr, raw[..., ::-1]) and (m.width, m.height) == (640, 480)
    R.item("3 stage1 방식 이미지 읽기 (imgmsg_to_cv2 bgr8)", same,
           f"encoding {m.encoding}, {m.width}x{m.height}, frame_id {m.header.frame_id}, bgr8 = rgb 채널 뒤집기 {same}")
    import cv2
    cv2.imwrite(os.path.join(R.out_dir, "third_view.png"), bgr)

    # 4·5 계단 응답 + 이미지 지연
    with ros.lock:
        ros.images, ros.keep_images = [], "third_view"
    step = 0.03            # 텔레옵 한 번 변화 수준 (0.2 rad 을 한 번에 주면 위치 오차가 5° 를 넘어 보호 정지가 걸림 = 설계대로)
    q1 = HOME.copy()
    q1[0] += step
    rows = hold_rows(HOME, 1.0) + hold_rows(q1, 2.0, t0=1.0)
    t_play = ros.play(rows)
    t_step = t_play + 1.0 + 1.0 / CMD_HZ                                   # 계단 명령을 보낸 sim 시각
    ros.wait_sim(0.5)
    with ros.lock:
        ros.keep_images = None
        imgs = list(ros.images)
    seg = [(t, p[0]) for t, n, p, _ in js_in(ros, t_step - 0.5, t_step + 2.0)]
    tt, qq = np.array([x[0] for x in seg]), np.array([x[1] for x in seg])
    q_pre = qq[tt < t_step].mean()
    move = np.abs(qq - q_pre)
    with ros.lock:
        ps_s = [v for s_, v in ros.ps if s_ is not None and t_step - 1.0 <= s_ <= t_step + 2.0]
    reached = bool(np.any(move >= 0.5 * step)) and bool(np.any(move >= np.radians(0.2)))
    i50 = int(np.argmax(move >= 0.5 * step))
    i_on = int(np.argmax(move >= np.radians(0.2)))
    t50 = (tt[i50] - t_step) * 1000
    over = (qq.max() - (q_pre + step)) / step * 100
    final = (qq[-1] - q_pre) / step * 100
    R.item(f"4 팔 명령 응답 (계단 {step} rad)", reached and t50 <= CRIT["step50_ms"] and not any(ps_s) and abs(final - 100) < 5,
           f"50 % 도달 {t50:.0f} ms (기준 ≤ {CRIT['step50_ms']:.0f}), 움직이기 시작 {(tt[i_on] - t_step) * 1000:.0f} ms, "
           f"오버슈트 {over:.1f} %, 2 s 뒤 {final:.1f} %, 보호 정지 {any(ps_s)}")
    its = np.array([s for s, _ in imgs])
    diffs = [float(np.abs(imgs[k][1] - imgs[k - 1][1]).mean()) for k in range(1, len(imgs))]
    base = [d for s, d in zip(its[1:], diffs) if s < t_step - 0.1]
    thr = max(3 * (max(base) if base else 0.0), 0.5)
    after = [(s, d) for s, d in zip(its[1:], diffs) if s >= t_step - 0.1]
    img_on = next((s for s, d in after if d > thr), None)
    if img_on is None:
        R.item("5 이미지 지연", False, f"이미지 변화를 못 찾음 (기준 차이 {thr:.2f})")
    else:
        lag = img_on - tt[i_on]
        frames = int(np.floor(lag * 30.0 + 1e-6))
        R.item("5 이미지 지연 (팔 움직임 → third view 이미지 변화)", 0 <= frames <= CRIT["image_lag_frames"],
               f"팔 움직임 시작 stamp {tt[i_on]:.4f} s, 이미지 변화 첫 stamp {img_on:.4f} s → {lag * 1000:.0f} ms ({frames} 프레임 뒤), "
               f"정지 때 이미지 차이 최대 {max(base) if base else 0:.2f}")
    ros.play(hold_rows(HOME, 2.0))

    # 6·7 파지 신호, 재생 중 보호 정지 안 걸림
    ok_r, res = ros.reset()
    if not ok_r:
        raise RuntimeError(f"A 리셋 실패: {res}")
    t_g = ros.play(grasp_rows())
    ros.wait_sim(1.0)
    t_rh = ros.now()
    ros.wait_sim(5.0)                                                     # 잡은 채 5 s: RTF (기하 제어기 드론이 잡힌 채 계속 날려 함)
    with ros.lock:
        cw = [(w, s_) for w, s_ in ros.clk_wall if s_ >= t_rh]
    rtf_held = (cw[-1][1] - cw[0][1]) / (cw[-1][0] - cw[0][0])
    t_e = ros.now()
    with ros.lock:
        pres = [v for s, _, v in ros.grip["present"] if s >= t_e - 1.0]
        targ = [v for s, _, v in ros.grip["target"] if s >= t_e - 1.0]
        ps_g = [v for s, v in ros.ps if s is not None and t_g <= s <= t_e]
    lo, hi = CRIT["present_mid"]
    R.item("6 파지 신호 (grasp 재생)", lo <= pres[-1] <= hi and targ[-1] == 1150.0 and np.ptp(pres) < 5.0,
           f"present {pres[-1]:.1f} (기준 {lo:g}~{hi:g}, 마지막 1 s 변화 {np.ptp(pres):.1f}), target {targ[-1]:.0f}, "
           f"잡은 채 RTF {rtf_held:.3f} (기록)")
    ok_r, res = ros.reset()
    t_h = ros.play(table_rows())
    ros.wait_sim(1.0)
    with ros.lock:
        ps_t = [(s, v) for s, v in ros.ps if s is not None and s >= t_h]
    hit = next((s for s, v in ps_t if v), None)
    ok_r2, res2 = ros.reset()
    ros.wait_sim(0.5)
    with ros.lock:
        ps_after = [v for s, v in ros.ps[-5:]]
    ok7 = (not any(ps_g)) and hit is not None and ok_r2 and not any(ps_after)
    R.item("7 보호 정지", ok7, f"grasp 재생 중 true {sum(ps_g)} 번 (기준 0), 테이블 충돌 → true "
           f"{'t ' + format(hit - t_h, '.2f') + ' s' if hit else '안 됨'}, 리셋 뒤 {ps_after} (기준 모두 false)")


# ── B. PX4 ──
def check_after_reset(ros, ok, res):
    ros.wait_sim(0.3)
    with ros.lock:
        q = ros.js[-1][2]
        pres = ros.grip["present"][-1][2]
        ps = ros.ps[-1][1] if ros.ps else None
    err = float(np.degrees(np.abs(q - HOME)).max())
    good = ok and err <= CRIT["home_deg"] and pres <= CRIT["open_raw"] and ps is False
    return good, f"{res.get('drone')} {res.get('duration_s')} s, 드론 오차 {res.get('drone_err_mm')} mm, 팔 홈 {err:.3f}°, 그리퍼 {pres:.1f}, 보호 정지 {ps}" \
        + ("" if ok else f", 실패 {res.get('error')}")


def part_b(ros, R):
    R.log("== B. PX4 드론")
    ros.wait_sim(2.0)
    rows, ok_all = [], True
    for k in range(2):
        ok, res = ros.reset()
        good, d = check_after_reset(ros, ok, res)
        good &= res.get("drone") == "goto"
        ok_all &= good
        rows.append(f"goto {k + 1}: {d}")
    ros.play(grasp_rows())
    ros.wait_sim(1.0)
    with ros.lock:
        present = ros.grip["present"][-1][2]
    kill = subprocess.run([sys.executable, os.path.join(SCRIPTS, "drone_cmd.py"), "kill"], capture_output=True, text=True, timeout=20)
    if kill.returncode != 0:
        raise RuntimeError(f"drone_cmd kill 실패: {kill.stderr}")
    ros.wait_sim(3.0)
    ok, res = ros.reset()
    good, d = check_after_reset(ros, ok, res)
    good &= res.get("drone") == "teleport_restart_takeoff"
    ok_all &= good
    rows.append(f"잡기 재생 (present {present:.0f}) → kill → {d}")
    R.item("8 리셋 (goto 2 회 + kill → PX4 재시작 1 회)", ok_all, " | ".join(rows))
    # 9 RTF (가만히 20 s, /clock 을 받은 wall 시각 기준)
    t0 = ros.now()
    ros.wait_sim(20.0)
    with ros.lock:
        cw = [(w, s) for w, s in ros.clk_wall if s >= t0]
    rtf = (cw[-1][1] - cw[0][1]) / (cw[-1][0] - cw[0][0])
    R.item("9 RTF (헤드리스, PX4, 카메라 2 대)", rtf >= CRIT["rtf"], f"{rtf:.3f} (기준 ≥ {CRIT['rtf']})")


def run_part(name, sim_args, fn, R, out_dir):
    sim = Sim(sim_args, os.path.join(out_dir, f"sim_{name}.log"))
    ros, ex = None, None
    try:
        ros = Ros()
        ex = SingleThreadedExecutor()
        ex.add_node(ros)
        threading.Thread(target=ex.spin, daemon=True).start()
        sim.wait_ready()
        t_end = time.time() + 30
        while ros.now() is None:
            if time.time() > t_end:
                raise RuntimeError("/clock 을 받지 못함")
            time.sleep(0.1)
        fn(ros, R)
    except Exception as e:  # noqa: BLE001
        R.item(f"{name} 진행", False, f"{type(e).__name__}: {e} (sim 로그 {sim.log_path})")
    finally:
        if ex is not None:
            ex.shutdown()
        if ros is not None:
            ros.destroy_node()
        sim.stop()


def main():
    p = argparse.ArgumentParser(description="sim ROS 2 인터페이스 자동 검사 (PLAN 3-8)")
    p.add_argument("--part", choices=("A", "B", "all"), default="all")
    a = p.parse_args()
    if os.environ.get("ROS_DISTRO") != "jazzy":
        sys.exit("source /opt/ros/jazzy/setup.bash 먼저")
    out_dir = os.path.join(REPORT_DIR, f"check_ros2_{datetime.datetime.now():%Y%m%d_%H%M%S}")
    os.makedirs(out_dir)
    R = Report(out_dir)
    R.log(f"check_ros2  {datetime.datetime.now().isoformat(timespec='seconds')}  기준 {CRIT}")
    rclpy.init()
    try:
        if a.part in ("A", "all"):
            run_part("A", ["--flight", "geometric"], part_a, R, out_dir)
        if a.part in ("B", "all"):
            run_part("B", [], part_b, R, out_dir)
    finally:
        rclpy.shutdown()
    R.log("")
    for name, ok in R.results:
        R.log(f"  {'PASS' if ok else 'FAIL'}  {name}")
    n = sum(ok for _, ok in R.results)
    R.log(f"  {n}/{len(R.results)} PASS   리포트 {out_dir}")
    with open(os.path.join(out_dir, "report.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(R.lines) + "\n")
    sys.exit(0 if n == len(R.results) else 1)


if __name__ == "__main__":
    main()
