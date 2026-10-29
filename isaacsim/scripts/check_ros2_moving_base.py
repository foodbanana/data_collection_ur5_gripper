#!/usr/bin/env python3
# =============================================================
# check_ros2_moving_base.py  (docs/PLAN.md 8단계 "움직이는 베이스 수집 연결" R2·R3)
#
# 움직이는 베이스 sim ROS 2 인터페이스 자동 검사. 시스템 python3 (source /opt/ros/jazzy/setup.bash) 로 실행하면
# sim_ros2_moving_base.py --headless 를 베이스마다 직접 띄우고, ROS 2 로 받고 보내 **stamp (sim time) 기준**으로 판정한다.
# 받는 쪽 노드·보고서·주기 계산은 check_ros2.py 의 것을 그대로 쓴다 (고정 베이스 검사 1·2·4 번과 같은 기준).
#   1 토픽 주기 (베이스가 움직이는 동안)   2 같은 시계 (카메라·그리퍼 stamp ∈ /joint_states stamp)
#   4 팔 명령 응답 (지금 자세에서 shoulder_pan 계단 0.03 rad, 50 % 도달 ≤ 80 ms, 보호 정지 없음)   + RTF (참고)
#   베이스 토픽 (R3, sim 을 --imu-compare 로 띄움):
#   5 /base/imu 120 Hz·/base/odom·/base/joint_states 30 Hz, stamp ∈ /joint_states stamp (같은 sim time)
#   6 IMU 두 방법 일치: 궤적 계산값 (/base/imu) vs Isaac IMU 센서 (/base/imu_isaac), 같은 stamp 끼리
#   7 IMU 물리 확인: 비력 z 평균 ≈ |g|, Husky 는 각속도 0·등속·정지 구간 비력 = (0, 0, |g|)
#   8 odom 일관성: 위치 차분 ≈ twist (child → odom 좌표계로 돌려서), joint_states 이름·개수
#
# 실행:
#   source /opt/ros/jazzy/setup.bash
#   python3 isaacsim/scripts/check_ros2_moving_base.py [--base husky stewart] [--base-motion small]   # 리포트 isaacsim/reports/check_ros2_moving_base_<시각>/
# =============================================================

import argparse
import datetime
import os
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState

import check_ros2 as c2

SCRIPT = os.path.join(c2.SCRIPTS, "sim_ros2_moving_base.py")
READY = "[sim_ros2_moving_base] 준비"
G = 9.81
BASE_HZ = {"imu": 120.0, "odom": 30.0, "joint_states": 30.0}
JOINTS = {"husky": 4, "stewart": 6}
CRIT_IMU = {"gyro_max": 0.005,     # [rad/s] 두 방법 차이 최대 (임시: 처음 재 보고 정함)
            "acc_rms": 0.02,       # [m/s²] 두 방법 차이 RMS
            "acc_max": 0.15,       # [m/s²] 두 방법 차이 최대. Husky 사다리꼴 궤적은 가속도가 계단 (0 ↔ 0.1) 이라
                                   #   Isaac 센서 (속도 차분) 가 계단에서 한 스텝 늦어 그 한 샘플만 0.1 차이 (2026-10-29 측정)
            "ori_deg": 0.1,        # [°] 자세 차이 최대
            "match": 0.95,         # 짝지어진 비율 최소 (가장 가까운 stamp, match_tol 이내)
            "match_tol": 1e-4,     # [s] Isaac 센서 시각은 float32 라 소수 6 자리가 가끔 다름 (물리 스텝 8.3 ms 보다 훨씬 작게)
            "g_mean": 0.05,        # [m/s²] 비력 z 평균 − |g|
            "odom_v": 0.002}       # [m/s] 위치 차분 − twist 최대


class Sim(c2.Sim):
    """check_ros2.Sim 과 같고 띄우는 스크립트·준비 줄만 다르다."""

    def __init__(self, args, log_path):
        self.log_path = log_path
        self.log = open(log_path, "w", encoding="utf-8")
        cmd = [c2.PYTHON_SH, SCRIPT, "--headless", "--duration", "3000", "--seed", "0", *args]
        self.proc = subprocess.Popen(cmd, cwd=c2.REPO, stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)

    def wait_ready(self, timeout=240.0):
        t_end = time.time() + timeout
        while time.time() < t_end:
            txt = open(self.log_path, encoding="utf-8", errors="replace").read()
            if READY in txt:
                return
            if "[ERROR]" in txt or self.proc.poll() is not None:
                raise RuntimeError(f"sim 이 시작하지 못함 (로그 {self.log_path})")
            time.sleep(1.0)
        raise RuntimeError(f"sim 준비가 {timeout:g} s 안에 안 됨 (로그 {self.log_path})")


def imu_row(m):
    q = m.orientation
    return (c2.stamp(m.header.stamp), np.array([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]),
            np.array([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]), np.array([q.w, q.x, q.y, q.z]))


class BaseRos(c2.Ros):
    """check_ros2.Ros + 베이스 토픽."""

    def __init__(self):
        super().__init__()
        rel = QoSProfile(depth=500, reliability=ReliabilityPolicy.RELIABLE)
        self.base = {"imu": [], "isaac_imu": [], "odom": [], "joint_states": []}
        self.create_subscription(Imu, "/base/imu", lambda m: self._add("imu", imu_row(m)), rel)
        self.create_subscription(Imu, "/base/imu_isaac", lambda m: self._add("isaac_imu", imu_row(m)), rel)
        self.create_subscription(Odometry, "/base/odom", lambda m: self._add("odom", self._odom_row(m)), rel)
        self.create_subscription(JointState, "/base/joint_states",
                                 lambda m: self._add("joint_states", (c2.stamp(m.header.stamp), list(m.name), list(m.position))), rel)

    def _add(self, k, row):
        with self.lock:
            self.base[k].append(row)

    @staticmethod
    def _odom_row(m):
        p, q, v, w = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear, m.twist.twist.angular
        return (c2.stamp(m.header.stamp), np.array([p.x, p.y, p.z]), np.array([q.w, q.x, q.y, q.z]),
                np.array([v.x, v.y, v.z]), np.array([w.x, w.y, w.z]), m.header.frame_id, m.child_frame_id)


def R_wxyz(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def check_base_topics(ros, R, tag, base, t0, t1, js):
    with ros.lock:
        rows = {k: [r for r in v if t0 <= r[0] <= t1] for k, v in ros.base.items()}
    # 5 주기·같은 시계
    js_set = set(np.round(js, 6))
    ok, parts = True, []
    for k, hz in BASE_HZ.items():
        st = [r[0] for r in rows[k]]
        r, n_gap, _ = c2.rate_of(st)
        miss = sum(1 for s in st if round(s, 6) not in js_set)
        good = abs(r - hz) <= c2.CRIT["rate_rel"] * hz and n_gap == 0 and miss == 0
        ok &= good
        parts.append(f"{k} {r:.2f} Hz (끊김 {n_gap}, stamp 안 맞음 {miss})")
    R.item(f"[{tag}] 5 베이스 토픽 주기·같은 시계", ok, ", ".join(parts))

    # 6 두 방법 일치 (같은 stamp)
    c = CRIT_IMU
    ist = np.array([r[0] for r in rows["isaac_imu"]])
    pairs, off = [], []
    if len(ist):
        for r in rows["imu"]:
            j = int(np.argmin(np.abs(ist - r[0])))
            if abs(ist[j] - r[0]) <= c["match_tol"]:
                pairs.append((r, rows["isaac_imu"][j]))
                off.append(abs(ist[j] - r[0]))
    frac = len(pairs) / max(len(rows["imu"]), 1)
    if pairs:
        dg = np.array([a[1] - b[1] for a, b in pairs])
        da = np.array([a[2] - b[2] for a, b in pairs])
        dq = np.array([np.degrees(2 * np.arccos(min(1.0, abs(float(np.dot(a[3], b[3])))))) for a, b in pairs])
        g_max, a_rms, a_max, q_max = float(np.abs(dg).max()), float(np.sqrt((da ** 2).sum(axis=1).mean())), float(np.abs(da).max()), float(dq.max())
        acc_amp = float(np.abs(np.array([a[2] for a, _ in pairs]) - [0, 0, G]).max())
        gyro_amp = float(np.abs(np.array([a[1] for a, _ in pairs])).max())
    else:
        g_max = a_rms = a_max = q_max = acc_amp = gyro_amp = float("nan")
    R.item(f"[{tag}] 6 IMU 두 방법 일치 (궤적 계산 vs Isaac IMU 센서)",
           frac >= c["match"] and g_max <= c["gyro_max"] and a_rms <= c["acc_rms"] and a_max <= c["acc_max"] and q_max <= c["ori_deg"],
           f"짝 {len(pairs)}/{len(rows['imu'])} ({frac:.0%}, stamp 차 최대 {max(off, default=float('nan')) * 1e6:.1f} µs), 각속도 차 최대 {g_max:.5f} rad/s (크기 최대 {gyro_amp:.4f}), "
           f"비력 차 RMS {a_rms:.4f}·최대 {a_max:.4f} m/s² (비력 − (0,0,g) 최대 {acc_amp:.4f}), 자세 차 최대 {q_max:.4f}°  기준 {c}")

    # 7 물리 확인
    acc = np.array([r[2] for r in rows["imu"]])
    gyro = np.array([r[1] for r in rows["imu"]])
    fz = float(acc[:, 2].mean())
    ok7 = abs(fz - G) <= c["g_mean"]
    extra = ""
    if base == "husky":
        flat = np.abs(acc[:, :2]).max(axis=1) < 1e-9          # 가감속 없는 구간 (등속·정지)
        ok7 &= float(np.abs(gyro).max()) == 0.0 and flat.any() and bool(np.allclose(acc[flat], [0, 0, G], atol=0.01))
        extra = f", 각속도 최대 {np.abs(gyro).max():.2e}, 등속·정지 샘플 {int(flat.sum())}/{len(flat)} 비력 {acc[flat].mean(axis=0).round(4).tolist() if flat.any() else None}"
    R.item(f"[{tag}] 7 IMU 물리 확인", ok7, f"비력 z 평균 {fz:.4f} (|g| {G}){extra}")

    # 8 odom 일관성
    od = rows["odom"]
    t = np.array([r[0] for r in od])
    P = np.array([r[1] for r in od])
    v_fd = (P[2:] - P[:-2]) / (t[2:] - t[:-2])[:, None]
    v_tw = np.array([R_wxyz(r[2]) @ r[3] for r in od[1:-1]])
    dv = float(np.abs(v_fd - v_tw).max())
    names = rows["joint_states"][-1][1] if rows["joint_states"] else []
    frames = {(r[5], r[6]) for r in od}
    ok8 = dv <= c["odom_v"] and len(names) == JOINTS[base] and len(frames) == 1
    R.item(f"[{tag}] 8 odom 일관성·joint_states", ok8,
           f"위치 차분 − twist 최대 {dv * 1000:.3f} mm/s (기준 {c['odom_v'] * 1000:.1f}), 속도 크기 최대 {np.abs(v_tw).max():.4f} m/s, "
           f"frame {frames}, joint_states {names}")


def check_base(ros, R, tag):
    ros.wait_sim(2.0)
    t0, w0 = ros.now(), time.time()
    ros.wait_sim(10.0)
    t1, w1 = ros.now(), time.time()
    with ros.lock:
        js = [x[0] for x in ros.js if t0 <= x[0] <= t1]
        clk = [s for _, s in ros.clk_wall if t0 <= s <= t1]
        cams = {k: [s for s in v if t0 <= s <= t1] for k, v in ros.cam.items()}
        grip = {k: [x[0] for x in v if t0 <= x[0] <= t1] for k, v in ros.grip.items()}
        ps = [s for s, _ in ros.ps if s is not None and t0 <= s <= t1]
    ok, parts = True, []
    for name, st, hz in [("/joint_states", js, 120.0), ("/clock", clk, 120.0), *[(f"카메라 {k}", v, 30.0) for k, v in cams.items()],
                         *[(f"그리퍼 {k}", v, 30.0) for k, v in grip.items()]]:
        r, n_gap, _ = c2.rate_of(st)
        good = abs(r - hz) <= c2.CRIT["rate_rel"] * hz and n_gap == 0
        ok &= good
        parts.append(f"{name} {r:.2f} Hz (끊김 {n_gap})")
    ps_hz = (len(ps) - 1) / (t1 - t0) if len(ps) > 1 else 0.0
    ok &= abs(ps_hz - 30.0) <= 1.0
    parts.append(f"/protective_stop 약 {ps_hz:.1f} Hz")
    R.item(f"[{tag}] 1 토픽 주기 (sim 기준, 베이스 움직이는 중)", ok, ", ".join(parts))
    js_set = set(np.round(js, 6))
    miss = {k: sum(1 for s in v if round(s, 6) not in js_set) for k, v in {**cams, **{f"grip_{k}": v for k, v in grip.items()}}.items()}
    R.item(f"[{tag}] 2 같은 시계 (카메라·그리퍼 stamp ∈ /joint_states stamp)", all(v == 0 for v in miss.values()), f"안 맞는 개수 {miss}")
    check_base_topics(ros, R, tag, tag.split(":")[0], t0, t1, js)

    # 4 팔 명령 응답: 지금 자세 (시작 자세) 에서 계단
    with ros.lock:
        last = ros.js[-1]
    idx = [last[1].index(n) for n in c2.ARM]
    q0 = np.asarray(last[2])[idx]
    step = 0.03
    q1 = q0.copy()
    q1[0] += step
    rows = c2.hold_rows(q0, 1.0) + c2.hold_rows(q1, 2.0, t0=1.0)
    t_play = ros.play(rows)
    t_step = t_play + 1.0 + 1.0 / c2.CMD_HZ
    ros.wait_sim(0.5)
    seg = [(t, np.asarray(p)[idx][0]) for t, n, p, _ in c2.js_in(ros, t_step - 0.5, t_step + 2.0)]
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
    R.item(f"[{tag}] 4 팔 명령 응답 (시작 자세에서 계단 {step} rad)",
           reached and t50 <= c2.CRIT["step50_ms"] and not any(ps_s) and abs(final - 100) < 5,
           f"50 % 도달 {t50:.0f} ms (기준 ≤ {c2.CRIT['step50_ms']:.0f}), 움직이기 시작 {(tt[i_on] - t_step) * 1000:.0f} ms, "
           f"오버슈트 {over:.1f} %, 2 s 뒤 {final:.1f} %, 보호 정지 {any(ps_s)}")
    R.log(f"  [{tag}] RTF (참고, 헤드리스) {(t1 - t0) / (w1 - w0):.3f}")


def run_base(base, motion, R, out_dir):
    tag = base if motion is None else f"{base}:{motion}"
    args = ["--base", base, "--imu-compare"] + ([] if motion is None else ["--base-motion", motion])
    sim = Sim(args, os.path.join(out_dir, f"sim_{tag.replace(':', '_')}.log"))
    ros, ex = None, None
    try:
        ros = BaseRos()
        ex = SingleThreadedExecutor()
        ex.add_node(ros)
        threading.Thread(target=ex.spin, daemon=True).start()
        sim.wait_ready()
        t_end = time.time() + 30
        while ros.now() is None:
            if time.time() > t_end:
                raise RuntimeError("/clock 을 받지 못함")
            time.sleep(0.1)
        check_base(ros, R, tag)
    except Exception as e:  # noqa: BLE001
        R.item(f"[{tag}] 진행", False, f"{type(e).__name__}: {e} (sim 로그 {sim.log_path})")
    finally:
        if ex is not None:
            ex.shutdown()
        if ros is not None:
            ros.destroy_node()
        sim.stop()


def main():
    p = argparse.ArgumentParser(description="움직이는 베이스 sim ROS 2 인터페이스 자동 검사 (PLAN 8단계 R2·R3)")
    p.add_argument("--base", nargs="+", default=["husky", "stewart"], choices=("husky", "stewart"))
    p.add_argument("--base-motion", default="small", help="Stewart 프리셋")
    a = p.parse_args()
    if os.environ.get("ROS_DISTRO") != "jazzy":
        sys.exit("source /opt/ros/jazzy/setup.bash 먼저")
    out_dir = os.path.join(c2.REPORT_DIR, f"check_ros2_moving_base_{datetime.datetime.now():%Y%m%d_%H%M%S}")
    os.makedirs(out_dir)
    R = c2.Report(out_dir)
    R.log(f"check_ros2_moving_base  {datetime.datetime.now().isoformat(timespec='seconds')}  기준 {c2.CRIT}")
    rclpy.init()
    try:
        for b in a.base:
            run_base(b, a.base_motion if b == "stewart" else None, R, out_dir)
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
