#!/usr/bin/env python3
# =============================================================
# measure_stamp_offset.py   (docs/PLAN.md 4단계 "실물 stamp 동기화 문제 (D1)")
#
# 카메라 이미지의 header.stamp 와 그리퍼 (/gripper/joint_states) 의 header.stamp 가 서로 얼마나 어긋나 있는지 잰다.
# 변환 (stage1) 은 stamp 로 프레임을 맞추므로, 이 차이만큼 데이터셋의 한 프레임 안에서 이미지와 그리퍼 값이 어긋난다.
#
# 방법: 그리퍼를 여러 번 조금씩 움직였다 멈춘다 (예 50 → 300 → 550 → 800 → 1050 → 800 → … raw).
#   - 그리퍼 쪽: present 값의 속도 (차분) 가 켜지는 시각·꺼지는 시각 (움직임 시작·정지, stamp 기준)
#   - 카메라 쪽: 이미지가 변하는 속도가 켜지는 시각·꺼지는 시각 (stamp 기준)
#       이미지 변화량 m(t) = (이미지 − 열린 상태 이미지) 를 (닫힌 상태 이미지 − 열린 상태 이미지) 방향으로 투영한 값 (0 = 열림, 1 = 닫힘)
#   - 차이 = 카메라 시각 − 그리퍼 시각.  **양수 = 이미지 stamp 가 그리퍼 stamp 보다 늦다** (같은 순간의 장면이 더 늦은 stamp 를 달고 있음)
#   시작·정지 시각은 속도가 "그 움직임의 보통 속도의 절반" 을 지나는 순간을 표본 사이 보간으로 구한다 (기준값을 따로 정하지 않음,
#   카메라·그리퍼 모두 약 30 Hz 라 같은 차분을 써서 차분 때문에 생기는 치우침이 상쇄된다).
#   방향을 바꾼 직후의 "시작" 은 따로 센다: 기구 유격 (backlash) 만큼 모터가 먼저 돌아 이미지가 늦게 변하므로 시각 차이로 보면 안 된다.
#
# 준비: 그리퍼를 움직이지 않게 고정하고, 재려는 카메라가 **손가락이 크게 보이게** 향하게 한다. 배경·조명은 가만히 (다른 움직임이 있으면 틀어진다).
#       카메라 (5_cameras.sh) 와 그리퍼 노드 (3_gripper_node.sh) 가 떠 있어야 한다. 그리퍼 teleop 은 끌 것 (명령이 겹친다).
# 실행 (시스템 python3 + ROS):
#   source /opt/ros/jazzy/setup.bash
#   python3 measure_stamp_offset.py [--cycles 6] [--goals 50 300 550 800 1050] [--pause 1.0] [--cameras wrist third_view]
# 결과: 화면 + inspect_out/stamp_offset_<시각>/ (report.txt, data.npz, 그림 plot_<카메라>.png (matplotlib 가 있으면))
# =============================================================

import os
import sys
import json
import time
import argparse
from datetime import datetime

import numpy as np

from camera_config import load_cameras, ROLES

GRIPPER_TOPIC = '/gripper/joint_states'
COMMAND_TOPIC = '/gripper/command'
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DOWN = 4                      # 이미지 축소 (가로세로 1/4, 회색조): 변화 시각만 보면 되므로


def stamp_sec(s):
    return s.sec + s.nanosec * 1e-9


class Probe:
    def __init__(self, cameras, roles):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image, JointState
        from std_msgs.msg import Float64

        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node('measure_stamp_offset')
        self.img = {r: [] for r in roles}        # (stamp, 받은 시각, 축소 회색조 이미지)
        self.grip = []                           # (stamp, 받은 시각, present raw)
        self.cmds = []                           # (보낸 시각, goal raw)
        self.Float64 = Float64
        rel = QoSProfile(depth=300, reliability=ReliabilityPolicy.RELIABLE)
        for r in roles:
            self.node.create_subscription(Image, cameras['roles'][r]['topic'], lambda m, r=r: self._on_image(r, m), rel)
        # 그리퍼는 발행자와 같은 reliability 로 (발행자가 best effort 인데 RELIABLE 로 받으면 연결이 안 된다)
        t_end = time.monotonic() + 5.0
        while not self.node.get_publishers_info_by_topic(GRIPPER_TOPIC):
            if time.monotonic() > t_end:
                raise RuntimeError(f"{GRIPPER_TOPIC} 발행자가 없음 (그리퍼 노드가 떠 있는지 확인)")
            rclpy.spin_once(self.node, timeout_sec=0.1)
        pubs = self.node.get_publishers_info_by_topic(GRIPPER_TOPIC)
        ok = all(p.qos_profile.reliability == ReliabilityPolicy.RELIABLE for p in pubs)
        self.node.create_subscription(JointState, GRIPPER_TOPIC, self._on_grip,
                                      QoSProfile(depth=300, reliability=ReliabilityPolicy.RELIABLE if ok else ReliabilityPolicy.BEST_EFFORT))
        self.pub = self.node.create_publisher(Float64, COMMAND_TOPIC, 10)

    def _on_image(self, role, m):
        if m.encoding not in ('rgb8', 'bgr8'):
            raise RuntimeError(f"{role}: 이미지 encoding {m.encoding} (rgb8 / bgr8 만)")
        a = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step // 3, 3)[:, :m.width]
        self.img[role].append((stamp_sec(m.header.stamp), time.time(), a[::DOWN, ::DOWN].astype(np.float32).mean(axis=2)))

    def _on_grip(self, m):
        self.grip.append((stamp_sec(m.header.stamp), time.time(), float(m.position[0])))

    def spin(self, sec):
        t_end = time.monotonic() + sec
        while time.monotonic() < t_end:
            self.rclpy.spin_once(self.node, timeout_sec=0.01)

    def command(self, raw):
        m = self.Float64()
        m.data = float(raw)
        self.pub.publish(m)
        self.cmds.append((time.time(), float(raw)))

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def speed(t, x):
    """중심 차분 속도 (앞뒤 표본). 양 끝은 0"""
    v = np.zeros_like(x)
    v[1:-1] = (x[2:] - x[:-2]) / (t[2:] - t[:-2])
    return v


def crossing(t, y, level, i_from, step):
    """i_from 에서 step (+1 / −1) 방향으로 가며 y 가 level 아래로 처음 내려가는 순간을 표본 사이 선형 보간으로. 없으면 None"""
    i = i_from
    while 0 <= i + step < len(y):
        a, b = y[i], y[i + step]
        if a >= level > b:
            return float(t[i] + (t[i + step] - t[i]) * (a - level) / (a - b))
        i += step
    return None


def analyze(role, t_c, frames, t_g, p, goals_seq, log):
    """→ dict(start=[…], stop=[…], start_rev=[…]) [s]. 카메라 − 그리퍼"""
    lo, hi = min(goals_seq), max(goals_seq)
    v_p = np.abs(speed(t_g, p))
    still = v_p < 0.05 * np.percentile(v_p, 95)
    # 열린 / 닫힌 기준 이미지: 그리퍼가 양 끝 근처에서 멈춰 있는 동안의 평균
    p_at_c = np.interp(t_c, t_g, p)
    still_c = np.interp(t_c, t_g, still.astype(float)) > 0.99
    tol = 0.04 * (hi - lo)
    open_m, closed_m = still_c & (np.abs(p_at_c - p_at_c[still_c].min()) < tol), still_c & (np.abs(p_at_c - p_at_c[still_c].max()) < tol)
    if open_m.sum() < 5 or closed_m.sum() < 5:
        raise RuntimeError(f"{role}: 열림 / 닫힘 기준 이미지를 못 만듦 (정지 프레임 {open_m.sum()} / {closed_m.sum()})")
    I_open, I_closed = frames[open_m].mean(axis=0), frames[closed_m].mean(axis=0)
    d = I_closed - I_open
    roi = np.abs(d) > 0.3 * np.abs(d).max()
    noise = float(frames[open_m].std(axis=0)[roi].mean())
    log(f"  {role}: 프레임 {len(t_c)}, 손가락이 바꾸는 영역 {100 * roi.mean():.1f} % 화면, 그 영역의 밝기 변화 평균 {np.abs(d[roi]).mean():.1f} (0~255), "
        f"정지 때 흔들림 {noise:.2f}")
    if roi.mean() < 0.005 or np.abs(d[roi]).mean() < 10 * noise:
        log(f"  [주의] {role}: 손가락이 화면에 작거나 변화가 약함 → 결과가 흔들릴 수 있음 (카메라를 손가락에 더 가까이·정면으로)")
    m = ((frames - I_open)[:, roi] * d[roi]).sum(axis=1) / (d[roi] ** 2).sum()
    v_m = np.abs(speed(t_c, m))

    out = {'start': [], 'stop': [], 'start_rev': [], 'm': m}
    # 움직임 구간: 그리퍼 속도가 켜져 있는 연속 구간
    moving = v_p > 0.3 * np.percentile(v_p, 95)
    edges = np.flatnonzero(np.diff(moving.astype(int)))
    segs = [(a + 1, b) for a, b in zip(edges[::2], edges[1::2])] if not moving[0] else []
    prev_dir = 0
    for a, b in segs:
        if b - a < 4:
            continue
        direction = int(np.sign(p[b] - p[a]))
        level_p = 0.5 * np.median(v_p[a:b + 1])
        mid = (a + b) // 2
        ts_p, te_p = crossing(t_g, v_p, level_p, mid, -1), crossing(t_g, v_p, level_p, mid, +1)
        if ts_p is None or te_p is None:
            continue
        # 카메라: 같은 구간 (± 0.4 s) 에서, 구간 앞 1/3 · 뒤 1/3 의 보통 속도의 절반을 지나는 순간
        ia, ib = np.searchsorted(t_c, ts_p), np.searchsorted(t_c, te_p)
        if ib - ia < 6 or ia < 12 or ib > len(t_c) - 12:
            continue
        third = max(2, (ib - ia) // 3)
        lvl_s, lvl_e = 0.5 * np.median(v_m[ia + 1:ia + 1 + third]), 0.5 * np.median(v_m[ib - third:ib])
        rest = np.median(v_m[max(0, ia - 12):ia - 3])                      # 움직이기 전의 변화 속도 (잡음)
        if min(lvl_s, lvl_e) < 3 * rest:
            continue                                                       # 이 구간은 이미지 변화가 잡음에 묻힘
        ts_m, te_m = crossing(t_c, v_m, lvl_s, ia + third, -1), crossing(t_c, v_m, lvl_e, ib - third, +1)
        if ts_m is not None and abs(ts_m - ts_p) < 0.4:
            out['start' if direction == prev_dir else 'start_rev'].append(ts_m - ts_p)
        if te_m is not None and abs(te_m - te_p) < 0.4:
            out['stop'].append(te_m - te_p)
        prev_dir = direction
    return out


def stats(x):
    x = np.asarray(x) * 1000.0
    return f"{x.mean():+6.1f} ms (표준편차 {x.std():4.1f}, 중앙값 {np.median(x):+6.1f}, {x.min():+.0f} ~ {x.max():+.0f}, n {len(x)})" if len(x) else "측정 없음"


def main():
    ap = argparse.ArgumentParser(description='카메라 stamp 와 그리퍼 stamp 의 차이를 잰다 (그리퍼를 조금씩 움직였다 멈추며)')
    ap.add_argument('--cycles', type=int, default=6, help='왕복 횟수 (기본 6, 약 15 s 씩)')
    ap.add_argument('--goals', type=float, nargs='+', default=[50, 300, 550, 800, 1050], help='그리퍼 목표 raw 단계 (오름차순)')
    ap.add_argument('--pause', type=float, default=1.0, help='[s] 한 단계 움직인 뒤 멈춰 있는 시간')
    ap.add_argument('--move-time', type=float, default=0.9, help='[s] 한 단계 움직이는 데 주는 시간')
    ap.add_argument('--cameras', nargs='+', default=list(ROLES), choices=ROLES, help='잴 카메라 역할')
    args = ap.parse_args()
    goals = sorted(args.goals)
    if len(goals) < 3 or goals[0] < 0 or goals[-1] > 1150:
        sys.exit('--goals 는 0~1150 사이 3 개 이상')

    cams = load_cameras()
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_dir = os.path.join(SCRIPT_DIR, 'inspect_out', f'stamp_offset_{stamp}')
    os.makedirs(out_dir)
    lines = []

    def log(msg=''):
        print(msg, flush=True)
        lines.append(msg)

    pr = Probe(cams, args.cameras)
    try:
        log(f"measure_stamp_offset {stamp}: 카메라 {args.cameras}, 단계 {goals}, 왕복 {args.cycles} 번, 멈춤 {args.pause} s")
        pr.spin(2.0)
        for r in args.cameras:
            if len(pr.img[r]) < 20:
                raise RuntimeError(f"{cams['roles'][r]['topic']} 이미지를 못 받음 (2 s 에 {len(pr.img[r])} 장). 카메라가 떠 있는지 확인")
        if len(pr.grip) < 20:
            raise RuntimeError(f"{GRIPPER_TOPIC} 를 못 받음 (2 s 에 {len(pr.grip)} 개)")
        pr.command(goals[0])
        pr.spin(3.0)
        seq = goals[1:] + goals[-2::-1]                        # 올라갔다 내려오는 한 왕복
        n = args.cycles * len(seq)
        log(f"그리퍼를 움직입니다 ({n} 번, 약 {n * (args.move_time + args.pause):.0f} s). 카메라·그리퍼·배경을 건드리지 마세요")
        for k in range(n):
            pr.command(seq[k % len(seq)])
            pr.spin(args.move_time + args.pause)
        pr.spin(1.0)
    finally:
        pr.close()

    t_g = np.array([x[0] for x in pr.grip])
    p = np.array([x[2] for x in pr.grip])
    order = np.argsort(t_g)
    t_g, p = t_g[order], p[order]
    lat = {GRIPPER_TOPIC: np.median([x[1] - x[0] for x in pr.grip])}
    log(f"\n그리퍼: {len(t_g)} 개, {1 / np.median(np.diff(t_g)):.1f} Hz, 간격 최대 {np.diff(t_g).max() * 1000:.0f} ms, present {p.min():.0f} ~ {p.max():.0f} raw")
    res, save = {}, {'t_g': t_g, 'p': p, 'cmd_t': np.array([c[0] for c in pr.cmds]), 'cmd_goal': np.array([c[1] for c in pr.cmds])}
    for r in args.cameras:
        t_c = np.array([x[0] for x in pr.img[r]])
        frames = np.stack([x[2] for x in pr.img[r]])
        order = np.argsort(t_c)
        t_c, frames = t_c[order], frames[order]
        dt = np.diff(t_c)
        lat[cams['roles'][r]['topic']] = np.median([x[1] - x[0] for x in pr.img[r]])
        log(f"{r}: {len(t_c)} 장, {1 / np.median(dt):.1f} Hz, 간격 최대 {dt.max() * 1000:.0f} ms, 빠진 프레임 {int((dt > 1.5 * np.median(dt)).sum())} 곳")
        res[r] = analyze(r, t_c, frames, t_g, p, goals, log)
        save[f't_{r}'], save[f'm_{r}'] = t_c, res[r].pop('m')
    np.savez(os.path.join(out_dir, 'data.npz'), **save)

    log("\n── 결과: 카메라 stamp − 그리퍼 stamp (양수 = 이미지 stamp 가 늦다) ──")
    summary = {}
    for r in args.cameras:
        both = res[r]['start'] + res[r]['stop']
        log(f"{r}")
        log(f"  움직임 시작 (같은 방향으로 이어 갈 때) : {stats(res[r]['start'])}")
        log(f"  움직임 정지                           : {stats(res[r]['stop'])}")
        log(f"  → 시작 + 정지                         : {stats(both)}")
        log(f"  (참고) 방향을 바꾼 직후의 시작        : {stats(res[r]['start_rev'])}   ← 기구 유격이 섞여 시각 차이로 보지 않음")
        summary[r] = {k: [round(float(x) * 1000, 1) for x in v] for k, v in res[r].items()}
    log("\n── 참고: 받은 시각 − header.stamp (중앙값). stamp 가 '찍은 순간' 인지 '도착한 순간' 인지 가늠 ──")
    for t, v in lat.items():
        log(f"  {t:36s} {v * 1000:+7.1f} ms")
    log("  (sim 은 stamp 가 sim time 이라 이 값은 의미 없음)")
    with open(os.path.join(out_dir, 'report.txt'), 'w') as f:
        f.write('\n'.join(lines) + '\n')
    with open(os.path.join(out_dir, 'result.json'), 'w') as f:
        json.dump(summary, f, indent=1)

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        for r in args.cameras:
            fig, ax = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
            t0 = t_g[0]
            ax[0].plot(t_g - t0, (p - p.min()) / (p.max() - p.min()), '.-', ms=3, label='gripper present (0~1)')
            ax[0].plot(save[f't_{r}'] - t0, save[f'm_{r}'], '.-', ms=3, label=f'{r} image change m')
            ax[0].legend(); ax[0].set_ylabel('position')
            ax[1].plot(t_g - t0, np.abs(speed(t_g, (p - p.min()) / (p.max() - p.min()))), label='gripper speed')
            ax[1].plot(save[f't_{r}'] - t0, np.abs(speed(save[f't_{r}'], save[f'm_{r}'])), label=f'{r} image speed')
            ax[1].legend(); ax[1].set_ylabel('speed [1/s]'); ax[1].set_xlabel('stamp [s]')
            fig.tight_layout(); fig.savefig(os.path.join(out_dir, f'plot_{r}.png'), dpi=110); plt.close(fig)
    except ImportError:
        pass
    log(f"\n저장: {out_dir}")


if __name__ == '__main__':
    main()
