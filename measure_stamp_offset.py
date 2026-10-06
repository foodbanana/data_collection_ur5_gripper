#!/usr/bin/env python3
# =============================================================
# measure_stamp_offset.py   (docs/PLAN.md 4단계 "실물 stamp 동기화 문제 (D1)")
#
# 카메라 이미지의 header.stamp 와 그리퍼 (/gripper/joint_states) 의 header.stamp 가 서로 얼마나 어긋나 있는지 잰다.
# 변환 (stage1) 은 stamp 로 프레임을 맞추므로, 이 차이만큼 데이터셋의 한 프레임 안에서 이미지와 그리퍼 값이 어긋난다.
#
# 방법: 그리퍼를 여러 번 조금씩 움직였다 멈춘다 (예 50 → 300 → 550 → 800 → 1050 → 800 → … raw).
#   - 이미지 변화량 m(t) = (이미지 − 열린 상태 이미지) 를 (닫힌 상태 이미지 − 열린 상태 이미지) 방향으로 투영한 값 (0 = 열림, 1 = 닫힘)
#   - 손가락 모양은 그리퍼 위치로 정해지므로 m(t) = F(p(t − τ)) 여야 한다 (p = present, τ = 두 stamp 의 차이).
#     한 방향으로 움직이는 구간마다, 그리퍼 위치 곡선을 시간축으로 τ 만큼 밀어 가며 m 과 가장 잘 겹치는 τ 를 찾는다
#     (F 는 구간마다 다항식으로 맞춤. 속도가 변하는 곳 (가속·감속·정지) 이 τ 를 정해 준다)
#   - **양수 = 이미지 stamp 가 그리퍼 stamp 보다 늦다**
#   닫는 방향과 여는 방향을 따로 보여 준다. 시각 차이라면 방향과 무관해야 한다 → 두 값이 크게 다르면 시각이 아닌 것 (기구가 방향에 따라
#   다르게 따라옴 등) 이 섞인 것이라 평균을 그대로 믿으면 안 된다.
#   방향을 바꾼 뒤 첫 걸음과 양 끝 걸음은 쓰지 않는다 (기구 유격, 끝 멈춤).
# 정확도 (2026-10-06): 정답이 0 ms 인 sim 에서 닫는 방향 +6 ms, 여는 방향 −11 ms (평균 −3 ms) → 방법 자체의 치우침 ±10 ms 수준.
#   실물 (RH-P12-RN + D435I) 첫 측정: 닫는 방향 −51 ms, 여는 방향 0 ms (방향마다 6 번이 ±4 ms 로 일치) → "−50 ~ 0 ms 사이" 까지만 말할 수 있었다.
#   그리퍼는 느리고 (전체 2 s) 방향에 따라 다르게 따라와서 정밀 측정에 맞지 않는다. 팔 ↔ 카메라는 실물 팔로 따로 잴 것 (PLAN D1)
#
# 준비: 그리퍼를 움직이지 않게 고정하고, 재려는 카메라가 **손가락이 크게 보이게** 향하게 한다. 배경·조명은 가만히 (다른 움직임이 있으면 틀어진다).
#       카메라 (5_cameras.sh) 와 그리퍼 노드 (3_gripper_node.sh) 가 떠 있어야 한다. 그리퍼 teleop 은 끌 것 (명령이 겹친다).
# 실행 (시스템 python3 + ROS):
#   source /opt/ros/jazzy/setup.bash
#   python3 measure_stamp_offset.py [--cycles 6] [--goals 50 300 550 800 1050] [--move-time 2.2] [--pause 0.8] [--cameras wrist third_view]
#   python3 measure_stamp_offset.py --analyze <data.npz> --range 280 820      # 저장된 측정을 다시 분석
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


def image_change(role, t_c, frames, t_g, p, log):
    """이미지 변화량 m(t): 0 = 열린 상태, 1 = 닫힌 상태 (손가락이 바꾸는 영역에서 두 기준 이미지의 차이 방향으로 투영)"""
    v_p = np.abs(speed(t_g, p))
    still = v_p < 0.05 * np.percentile(v_p, 95)
    p_at_c = np.interp(t_c, t_g, p)
    still_c = np.interp(t_c, t_g, still.astype(float)) > 0.99
    tol = 0.04 * (p.max() - p.min())
    open_m = still_c & (np.abs(p_at_c - p_at_c[still_c].min()) < tol)
    closed_m = still_c & (np.abs(p_at_c - p_at_c[still_c].max()) < tol)
    if open_m.sum() < 5 or closed_m.sum() < 5:
        raise RuntimeError(f"{role}: 열림 / 닫힘 기준 이미지를 못 만듦 (정지 프레임 {open_m.sum()} / {closed_m.sum()})")
    I_open, I_closed = frames[open_m].mean(axis=0), frames[closed_m].mean(axis=0)
    d = I_closed - I_open
    roi = np.abs(d) > 0.3 * np.abs(d).max()
    noise = float(frames[open_m].std(axis=0)[roi].mean())
    log(f"  {role}: 손가락이 바꾸는 영역 {100 * roi.mean():.1f} % 화면, 그 영역의 밝기 변화 평균 {np.abs(d[roi]).mean():.1f} (0~255), 정지 때 흔들림 {noise:.2f}")
    if roi.mean() < 0.03 or np.abs(d[roi]).mean() < 10 * noise:
        log(f"  [주의] {role}: 손가락이 화면에 작거나 변화가 약함 → 결과가 흔들릴 수 있음 (카메라를 손가락에 더 가까이·정면으로, 배경은 가만히)")
    return ((frames - I_open)[:, roi] * d[roi]).sum(axis=1) / (d[roi] ** 2).sum()


def align_runs(t_g, p, t_c, m, lo, hi, deg=4, taus=np.arange(-0.15, 0.1501, 0.002)):
    """한 방향으로 움직이는 구간 (그리퍼 위치가 lo~hi 인 동안) 마다 m(t) = F(p(t − τ)) 가 가장 잘 맞는 τ. → [(방향 ±1, τ [s])]"""
    out = []
    pc = np.interp(t_c, t_g, p)
    slope = np.interp(t_c, t_g, np.gradient(np.convolve(p, np.ones(9) / 9, mode='same'), t_g))
    idx = np.flatnonzero((pc > lo) & (pc < hi))
    if len(idx) == 0:
        return out
    for seg in np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1):
        if len(seg) < 25:
            continue
        d = np.sign(np.median(slope[seg]))
        if d == 0 or np.mean(slope[seg] * d < -0.05 * np.abs(slope[seg]).max()) > 0.02:
            continue                                         # 반대 방향으로 가는 표본이 있으면 한 방향 구간이 아님
        a, b = seg[0] - 6, seg[-1] + 6                       # 양 끝을 조금 넓힘 (τ 만큼 밀어도 표본이 있게)
        if a < 0 or b >= len(t_c):
            continue
        tt, mm = t_c[a:b], m[a:b]
        cost = []
        for tau in taus:
            pp = np.interp(tt - tau, t_g, p)
            x = (pp - pp.mean()) / (pp.std() + 1e-9)
            cost.append(np.sum((np.polyval(np.polyfit(x, mm, deg), x) - mm) ** 2))
        cost = np.asarray(cost)
        k = int(cost.argmin())
        if 0 < k < len(taus) - 1:                            # 최솟값 근처 포물선 보간. 탐색 범위 끝에 걸린 구간은 버림
            y0, y1, y2 = cost[k - 1:k + 2]
            out.append((int(d), float(taus[k] + 0.5 * (y0 - y2) / (y0 - 2 * y1 + y2) * (taus[1] - taus[0]))))
    return out


def stats(x):
    x = np.asarray(x) * 1000.0
    return (f"{x.mean():+6.1f} ms (표준편차 {x.std():4.1f}, 평균의 오차 ±{x.std() / np.sqrt(len(x)):.1f}, {x.min():+.0f} ~ {x.max():+.0f}, n {len(x)})"
            if len(x) else "측정 없음")


def report(cameras, series, t_g, p, lo, hi, log):
    """series: {역할: (t_c, m)}. 방향별 τ 를 출력하고 {역할: {close: [ms…], open: […]}} 를 돌려준다"""
    log(f"\n── 결과: 카메라 stamp − 그리퍼 stamp (양수 = 이미지 stamp 가 늦다). 그리퍼 위치 {lo:g} ~ {hi:g} raw 구간 ──")
    summary = {}
    for r in cameras:
        t_c, m = series[r]
        res = align_runs(t_g, p, t_c, m, lo, hi)
        close, opn = [t for d, t in res if d > 0], [t for d, t in res if d < 0]
        log(f"{r}")
        log(f"  닫는 방향 : {stats(close)}")
        log(f"  여는 방향 : {stats(opn)}")
        log(f"  → 전체    : {stats(close + opn)}")
        if close and opn and abs(np.mean(close) - np.mean(opn)) > 0.02:
            a, b = sorted((np.mean(close) * 1000, np.mean(opn) * 1000))
            log(f"  [주의] 방향에 따라 {b - a:.0f} ms 다름 → 시각 차이가 아닌 것이 섞였다. \"{a:+.0f} ~ {b:+.0f} ms 사이\" 까지만 말할 수 있다")
        summary[r] = {'close': [round(x * 1000, 1) for x in close], 'open': [round(x * 1000, 1) for x in opn]}
    log("  (방법의 정확도: 정답이 0 ms 인 sim 에서 닫는 방향 +6, 여는 방향 −11 ms)")
    return summary


def main():
    ap = argparse.ArgumentParser(description='카메라 stamp 와 그리퍼 stamp 의 차이를 잰다 (그리퍼를 조금씩 움직였다 멈추며)')
    ap.add_argument('--cycles', type=int, default=6, help='왕복 횟수 (기본 6)')
    ap.add_argument('--goals', type=float, nargs='+', default=[50, 300, 550, 800, 1050], help='그리퍼 목표 raw 단계 (오름차순)')
    ap.add_argument('--move-time', type=float, default=2.2, help='[s] 한 단계 움직이는 데 주는 시간 (실물 그리퍼는 250 raw 에 약 1.9 s)')
    ap.add_argument('--pause', type=float, default=0.8, help='[s] 한 단계 움직인 뒤 멈춰 있는 시간')
    ap.add_argument('--cameras', nargs='+', default=list(ROLES), choices=ROLES, help='잴 카메라 역할')
    ap.add_argument('--range', type=float, nargs=2, default=None, metavar=('LO', 'HI'),
                    help='분석에 쓸 그리퍼 위치 구간 raw (기본: 둘째 목표 − 20 ~ 끝에서 둘째 목표 + 20. 양 끝 걸음과 방향을 바꾼 뒤 첫 걸음을 뺀다)')
    ap.add_argument('--analyze', default=None, metavar='DATA_NPZ', help='측정하지 않고 저장된 data.npz 를 다시 분석 (--range 필요)')
    args = ap.parse_args()

    lines = []

    def log(msg=''):
        print(msg, flush=True)
        lines.append(msg)

    if args.analyze:
        if args.range is None:
            sys.exit('--analyze 에는 --range LO HI 가 필요')
        d = np.load(args.analyze)
        roles = [k[2:] for k in d.keys() if k.startswith('m_')]
        log(f"다시 분석: {args.analyze}, 카메라 {roles}")
        report(roles, {r: (d[f't_{r}'], d[f'm_{r}']) for r in roles}, d['t_g'], d['p'], *args.range, log)
        return

    goals = sorted(args.goals)
    if len(goals) < 4 or goals[0] < 0 or goals[-1] > 1150:
        sys.exit('--goals 는 0~1150 사이 4 개 이상')
    lo, hi = args.range if args.range else (goals[1] - 20, goals[-2] + 20)

    cams = load_cameras()
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_dir = os.path.join(SCRIPT_DIR, 'inspect_out', f'stamp_offset_{stamp}')
    os.makedirs(out_dir)

    pr = Probe(cams, args.cameras)
    try:
        log(f"measure_stamp_offset {stamp}: 카메라 {args.cameras}, 단계 {goals}, 왕복 {args.cycles} 번, 한 단계 {args.move_time} + {args.pause} s")
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
    if p.max() < goals[-1] - 40 or p.min() > goals[0] + 40:
        raise RuntimeError(f"그리퍼가 목표 범위 ({goals[0]:g} ~ {goals[-1]:g}) 까지 못 감 (present {p.min():.0f} ~ {p.max():.0f}). "
                           f"손가락 사이에 물체가 있는지 확인하거나 --goals 를 줄일 것")
    series, save = {}, {'t_g': t_g, 'p': p, 'cmd_t': np.array([c[0] for c in pr.cmds]), 'cmd_goal': np.array([c[1] for c in pr.cmds])}
    for r in args.cameras:
        t_c = np.array([x[0] for x in pr.img[r]])
        frames = np.stack([x[2] for x in pr.img[r]])
        order = np.argsort(t_c)
        t_c, frames = t_c[order], frames[order]
        dt = np.diff(t_c)
        lat[cams['roles'][r]['topic']] = np.median([x[1] - x[0] for x in pr.img[r]])
        log(f"{r}: {len(t_c)} 장, {1 / np.median(dt):.1f} Hz, 간격 최대 {dt.max() * 1000:.0f} ms, 빠진 프레임 {int((dt > 1.5 * np.median(dt)).sum())} 곳")
        series[r] = (t_c, image_change(r, t_c, frames, t_g, p, log))
        save[f't_{r}'], save[f'm_{r}'] = series[r]
    np.savez(os.path.join(out_dir, 'data.npz'), **save)

    summary = report(args.cameras, series, t_g, p, lo, hi, log)
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
            t0, pn = t_g[0], (p - p.min()) / (p.max() - p.min())
            ax[0].plot(t_g - t0, pn, '.-', ms=3, label='gripper present (0~1)')
            ax[0].plot(series[r][0] - t0, series[r][1], '.-', ms=3, label=f'{r} image change m')
            ax[0].legend(); ax[0].set_ylabel('position')
            ax[1].plot(t_g - t0, np.abs(speed(t_g, pn)), label='gripper speed')
            ax[1].plot(series[r][0] - t0, np.abs(speed(*series[r])), label=f'{r} image speed')
            ax[1].legend(); ax[1].set_ylabel('speed [1/s]'); ax[1].set_xlabel('stamp [s]')
            fig.tight_layout(); fig.savefig(os.path.join(out_dir, f'plot_{r}.png'), dpi=110); plt.close(fig)
    except ImportError:
        pass
    log(f"\n저장: {out_dir}")


if __name__ == '__main__':
    main()
