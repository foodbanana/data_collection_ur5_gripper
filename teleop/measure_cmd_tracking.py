#!/usr/bin/env python3
# =============================================================
# measure_cmd_tracking.py — 녹화한 bag 에서 팔 명령 (/joint_command) 과 측정 관절 (/joint_states) 의 지연·오차를 잰다
#   (docs/PLAN.md 5단계 완료 기준 "명령과 실제 관절 사이 지연·오차". sim·실물 bag 공용, 두 토픽의 header.stamp 로 맞춘다)
#
#   1 오차 (데이터셋 그대로): 측정 관절각 − 그 시각 직전의 최신 명령 (zero-order hold = 데이터셋 action 과 같은 규칙)
#   2 지연: 명령을 τ 만큼 늦춰 (선형 보간) 측정값과 가장 잘 겹치는 τ (움직이는 구간, 0 ~ --max-lag, 1 ms 간격)
#   3 지연을 뺀 오차: τ 만큼 늦춘 명령과 측정값의 차이 = 따라가는 정확도
#   관절 [°] 와 TCP (URDF FK, teleop/ik_model.py) 위치 [mm]·자세 [°]. 움직이는 구간 = 명령 관절 속도 최댓값이 --moving-dps 넘는 시각
#   구간: 첫 명령 ~ 마지막 명령 (텔레옵을 멈춘 뒤는 뺀다)
#
# 실행 (시스템 python3 + ROS):
#   source /opt/ros/jazzy/setup.bash
#   python3 teleop/measure_cmd_tracking.py bags/<bag> [bags/<bag> ...]   [--max-lag 0.3] [--moving-dps 2]
# 결과: 터미널 + teleop/reports/measure_cmd_tracking_<시각>.txt (git 제외)
# =============================================================

import argparse
import datetime
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import ik_model as km  # noqa: E402

CMD_TOPIC, JS_TOPIC = "/joint_command", "/joint_states"
LINES = []


def log(s=""):
    print(s, flush=True)
    LINES.append(s)


def read_bag(path):
    """→ (명령 stamp [s], 명령 (N, 6)), (측정 stamp, 측정 (M, 6)). 관절 순서 = ik_model.ARM_JOINTS. 같은 stamp 의 명령은 마지막 것."""
    import rclpy.serialization
    import rosbag2_py
    from rosidl_runtime_py.utilities import get_message

    reader = None
    for sid in ("mcap", "sqlite3"):
        try:
            r = rosbag2_py.SequentialReader()
            r.open(rosbag2_py.StorageOptions(uri=path, storage_id=sid), rosbag2_py.ConverterOptions("cdr", "cdr"))
            reader = r
            break
        except Exception:
            continue
    if reader is None:
        raise RuntimeError(f"bag 을 열 수 없음: {path}")
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for t in (CMD_TOPIC, JS_TOPIC):
        if t not in types:
            raise RuntimeError(f"{path}: {t} 토픽이 없음")
    cls = {t: get_message(types[t]) for t in (CMD_TOPIC, JS_TOPIC)}
    out = {CMD_TOPIC: [], JS_TOPIC: []}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in out:
            continue
        m = rclpy.serialization.deserialize_message(data, cls[topic])
        d = dict(zip(m.name, m.position))
        try:
            q = [d[n] for n in km.ARM_JOINTS]
        except KeyError as e:
            raise RuntimeError(f"{path}: {topic} 에 관절 {e} 이 없음 (이름 {list(m.name)})")
        out[topic].append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, q))
    res = []
    for topic in (CMD_TOPIC, JS_TOPIC):
        if not out[topic]:
            raise RuntimeError(f"{path}: {topic} 메시지가 0 개")
        if any(t <= 0 for t, _ in out[topic]):
            raise RuntimeError(f"{path}: {topic} 에 stamp 0 인 메시지가 있음")
        out[topic].sort(key=lambda x: x[0])
        t = np.array([x[0] for x in out[topic]])
        q = np.array([x[1] for x in out[topic]], dtype=float)
        if topic == CMD_TOPIC:                         # 같은 stamp 가 여러 개면 마지막 (sim 은 루프마다 최신 명령 하나를 쓴다)
            keep = np.append(np.diff(t) > 0, True)
            t, q = t[keep], q[keep]
        res.append((t, q))
    return res


def zoh(t_src, q_src, t):
    """t 직전 (같음 포함) 의 최신 값. 데이터셋 action 과 같은 규칙."""
    i = np.searchsorted(t_src, t, side="right") - 1
    return q_src[np.clip(i, 0, len(t_src) - 1)]


def lerp(t_src, q_src, t):
    return np.stack([np.interp(t, t_src, q_src[:, j]) for j in range(q_src.shape[1])], axis=1)


def tcp_err(model, qa, qb):
    """→ 위치 차이 [mm], 자세 차이 [°] (각 샘플)."""
    dp, dr = np.zeros(len(qa)), np.zeros(len(qa))
    for k in range(len(qa)):
        A, B = model.fk(qa[k]), model.fk(qb[k])
        dp[k] = np.linalg.norm(A[:3, 3] - B[:3, 3]) * 1000
        dr[k] = math.degrees(np.linalg.norm(km.R_to_rotvec(A[:3, :3] @ B[:3, :3].T)))
    return dp, dr


def stats(x):
    return f"RMS {np.sqrt(np.mean(x ** 2)):.3f}, 95 % {np.percentile(np.abs(x), 95):.3f}, 최대 {np.max(np.abs(x)):.3f}"


def main():
    ap = argparse.ArgumentParser(description="bag 의 /joint_command ↔ /joint_states 지연·오차")
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--max-lag", type=float, default=0.3, help="찾을 지연의 최댓값 [s]")
    ap.add_argument("--moving-dps", type=float, default=2.0, help="움직이는 구간 기준: 명령 관절 속도 최댓값 [°/s]")
    ap.add_argument("--report-dir", default=os.path.join(HERE, "reports"))
    args = ap.parse_args()

    model = km.ArmModel()
    names = [n.replace("_joint", "") for n in km.ARM_JOINTS]
    log(f"measure_cmd_tracking  {datetime.datetime.now().isoformat(timespec='seconds')}")
    log(f"기준: 움직임 = 명령 관절 속도 최댓값 > {args.moving_dps:g} °/s, 지연 탐색 0 ~ {args.max_lag * 1000:.0f} ms")

    allE, allM, allS = [], [], []          # 전체 합산용: (오차 zoh, 움직임 여부), 지연 탐색용 (t, q, tc, qc)
    for path in args.bags:
        (tc, qc), (tj, qj) = read_bag(path)
        sel = (tj >= tc[0]) & (tj <= tc[-1])
        t, q = tj[sel], qj[sel]
        vc = np.gradient(qc, tc, axis=0)                   # 명령 관절 속도
        v_at = zoh(tc, vc, t)
        moving = np.degrees(np.max(np.abs(v_at), axis=1)) > args.moving_dps
        e0 = q - zoh(tc, qc, t)
        allE.append(e0); allM.append(moving); allS.append((t, q, tc, qc))
        log(f"\n=== {os.path.basename(os.path.normpath(path))}: 명령 {len(tc)} 개 ({(tc[-1] - tc[0]):.1f} s), 측정 {len(t)} 개, 움직이는 비율 {moving.mean() * 100:.0f} %, "
            f"명령 관절 속도 최대 {np.degrees(np.abs(vc).max()):.1f} °/s")

    E, M = np.concatenate(allE), np.concatenate(allM)

    # 지연: 움직이는 구간에서 RMS 가 가장 작은 τ (모든 관절 합, 관절별)
    lags = np.arange(0.0, args.max_lag + 1e-9, 0.001)
    rms_all, rms_j = [], []
    for tau in lags:
        es = [q - lerp(tc, qc, t - tau) for t, q, tc, qc in allS]
        e = np.concatenate(es)[M]
        rms_all.append(np.sqrt(np.mean(e ** 2)))
        rms_j.append(np.sqrt(np.mean(e ** 2, axis=0)))
    rms_all, rms_j = np.array(rms_all), np.array(rms_j)
    tau = lags[int(np.argmin(rms_all))]
    tau_j = lags[np.argmin(rms_j, axis=0)]

    def tcp_for(e_list_fn):
        dps, drs, ms = [], [], []
        for (t, q, tc, qc), m in zip(allS, allM):
            qa, qb = q, e_list_fn(t, tc, qc)
            dp, dr = tcp_err(model, qa, qb)
            dps.append(dp); drs.append(dr); ms.append(m)
        return np.concatenate(dps), np.concatenate(drs), np.concatenate(ms)

    log("\n[1] 오차 = 측정 − 최신 명령 (zero-order hold, 데이터셋 action 과 같은 규칙)")
    for lab, mask in (("움직이는 중", M), ("멈춘 중", ~M)):
        if mask.sum() == 0:
            continue
        log(f"  {lab} ({mask.sum()} 개)")
        for j, n in enumerate(names):
            log(f"    {n:14s} [°] {stats(np.degrees(E[mask, j]))}")
    dp0, dr0, mm = tcp_for(lambda t, tc, qc: zoh(tc, qc, t))
    for lab, mask in (("움직이는 중", mm), ("멈춘 중", ~mm)):
        if mask.sum():
            log(f"  TCP {lab}: 위치 [mm] {stats(dp0[mask])} / 자세 [°] {stats(dr0[mask])}")

    log(f"\n[2] 지연 (움직이는 구간에서 명령을 늦춰 측정과 가장 잘 겹치는 시간)")
    log(f"  모든 관절 합: {tau * 1000:.0f} ms (RMS {math.degrees(rms_all[0]):.3f}° → {math.degrees(rms_all.min()):.3f}°)")
    log("  관절별: " + ", ".join(f"{n} {tj_ * 1000:.0f} ms" for n, tj_ in zip(names, tau_j)))
    if tau >= args.max_lag - 0.002:
        log(f"  주의: 지연이 탐색 끝 ({args.max_lag * 1000:.0f} ms) 에 걸림 → --max-lag 를 늘려 다시")

    log(f"\n[3] 지연 {tau * 1000:.0f} ms 를 뺀 오차 = 측정 − {tau * 1000:.0f} ms 전 명령 (선형 보간). 따라가는 정확도")
    Es = np.concatenate([q - lerp(tc, qc, t - tau) for t, q, tc, qc in allS])
    for lab, mask in (("움직이는 중", M), ("멈춘 중", ~M)):
        if mask.sum() == 0:
            continue
        log(f"  {lab}")
        for j, n in enumerate(names):
            log(f"    {n:14s} [°] {stats(np.degrees(Es[mask, j]))}")
    dp1, dr1, mm = tcp_for(lambda t, tc, qc: lerp(tc, qc, t - tau))
    for lab, mask in (("움직이는 중", mm), ("멈춘 중", ~mm)):
        if mask.sum():
            log(f"  TCP {lab}: 위치 [mm] {stats(dp1[mask])} / 자세 [°] {stats(dr1[mask])}")

    os.makedirs(args.report_dir, exist_ok=True)
    p = os.path.join(args.report_dir, f"measure_cmd_tracking_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")
    with open(p, "w", encoding="utf-8") as f:
        f.write("\n".join(LINES) + "\n")
    print(f"\n리포트: {p}")


if __name__ == "__main__":
    main()
