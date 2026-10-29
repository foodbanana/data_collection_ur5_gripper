#!/usr/bin/env python3
# =============================================================
# design_stewart.py — Stewart 플랫폼 크기 설계 (PLAN 8단계 S2). Isaac Sim 없이 (numpy·yaml, 시스템 python3)
#
# config/base_stewart.yaml 의 크기·부착 각도·움직임 프리셋으로, 프리셋마다 무작위 위상·주파수를 많이 돌려
#   다리 길이 범위·신축 비율·다리 기울기·특이점 여유 (Jacobian 조건수)·다리끼리 거리, 로봇 base·드론 높이를 계산한다.
#   수집 프리셋 (collect 가 false 가 아닌 것) 이 모두 들어가는 다리 길이 (length_min / max, 여유 포함) 를 제안한다.
#
# 실행 (레포 최상위):
#   python3 isaacsim/scripts/design_stewart.py --config isaacsim/config/base_stewart.yaml
#   옵션: --height H --base-radius R --top-radius R (설정 덮어쓰기, 비교용), --trials N, --seconds S, --seed N, --margin 0.03
# =============================================================

import argparse
import math
import os

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(os.path.dirname(HERE), "config", "base_stewart.yaml")
DRONE_BASE = np.array([0.60, 0.0, 0.738])     # scene_husky.yaml drone_base 와 같은 상대 위치 (Stewart 씬도 같게 쓸 예정)
AXES = ("x", "y", "z", "roll", "pitch", "yaw")


def load(path):
    with open(path, encoding="utf-8") as f:
        c = yaml.safe_load(f)
    for k in ("geometry", "legs", "imu", "motion"):
        if k not in c:
            raise KeyError(f"{path}: '{k}' 항목이 없습니다")
    g = c["geometry"]
    for k in ("height", "base_radius", "top_radius", "base_plate", "top_plate", "base_angles_deg", "top_angles_deg"):
        if k not in g:
            raise KeyError(f"{path}: 'geometry.{k}' 항목이 없습니다")
    if len(g["base_angles_deg"]) != 6 or len(g["top_angles_deg"]) != 6:
        raise ValueError("부착 각도는 6 개씩")
    m = c["motion"]
    for k in ("ramp_time", "omega_scale_range", "base_omega", "presets"):
        if k not in m:
            raise KeyError(f"{path}: 'motion.{k}' 항목이 없습니다")
    return c


def preset_amplitude(m, name):
    """프리셋의 축별 진폭 × scale. amplitude 가 다른 프리셋 이름이면 그 진폭을 쓴다."""
    p = m["presets"][name]
    amp = p["amplitude"]
    if isinstance(amp, str):
        if amp not in m["presets"] or isinstance(m["presets"][amp]["amplitude"], str):
            raise ValueError(f"프리셋 {name}: amplitude '{amp}' 를 찾을 수 없음")
        amp = m["presets"][amp]["amplitude"]
    missing = [k for k in AXES if k not in amp]
    if missing:
        raise KeyError(f"프리셋 {name}: 진폭이 없는 축 {missing}")
    return {k: float(p["scale"]) * float(amp[k]) for k in AXES}


def rot(r, p, y):
    """Rz(yaw) · Ry(pitch) · Rx(roll). r, p, y 는 같은 모양의 배열 → (..., 3, 3)."""
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    R = np.empty(np.shape(r) + (3, 3))
    R[..., 0, 0], R[..., 0, 1], R[..., 0, 2] = cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr
    R[..., 1, 0], R[..., 1, 1], R[..., 1, 2] = sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr
    R[..., 2, 0], R[..., 2, 1], R[..., 2, 2] = -sp, cp * sr, cp * cr
    return R


def anchors(g):
    """B (아랫판 부착점, world, 아랫판 윗면), P (윗판 부착점, 윗판 좌표 = 회전 중심 기준, 윗판 아랫면)."""
    ab, at = np.radians(g["base_angles_deg"]), np.radians(g["top_angles_deg"])
    zb = float(g["base_plate"]["thickness"])
    zt = -float(g["top_plate"]["thickness"])
    B = np.stack([g["base_radius"] * np.cos(ab), g["base_radius"] * np.sin(ab), np.full(6, zb)], axis=1)
    P = np.stack([g["top_radius"] * np.cos(at), g["top_radius"] * np.sin(at), np.full(6, zt)], axis=1)
    return B, P


def ramp(t, T):
    u = np.clip(t / T, 0.0, 1.0)
    return 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5      # moving_base.SineTrajectory 와 같은 5 차 램프


def sample_motion(m, amp, t, rng, omega_scale=None):
    """한 에피소드 궤적 q[k] (len(t),). omega_scale 을 주면 c_k 고정 (lab 재현 [1, 1] 등)."""
    lo, hi = omega_scale or m["omega_scale_range"]
    e = ramp(t, float(m["ramp_time"]))
    q = {}
    for k in AXES:
        w = float(m["base_omega"][k]) * rng.uniform(lo, hi)
        q[k] = e * amp[k] * np.sin(w * t + rng.uniform(0, 2 * np.pi))
    return q


def leg_vectors(B, P, height, q):
    """(N, 6, 3) 다리 벡터 (아랫판 부착점 → 윗판 부착점)."""
    R = rot(q["roll"], q["pitch"], q["yaw"])                               # (N, 3, 3)
    c = np.stack([q["x"], q["y"], height + q["z"]], axis=1)                 # (N, 3)
    top = np.einsum("nij,kj->nki", R, P) + c[:, None, :]                   # (N, 6, 3)
    return top - B[None], R, c


def jacobian_cond(B, P, height, R, c):
    """다리 길이의 자세 미분 (6x6, 이동 3 + 작은 회전 3) 의 조건수 (클수록 특이점 가까움)."""
    out = np.empty(len(c))
    for n in range(len(c)):
        top = (R[n] @ P.T).T + c[n]
        u = top - B
        u /= np.linalg.norm(u, axis=1, keepdims=True)
        rP = (R[n] @ P.T).T
        J = np.hstack([u, np.cross(rP, u)])
        out[n] = np.linalg.cond(J)
    return out


def seg_dist(p1, q1, p2, q2):
    """선분 p1-q1, p2-q2 사이 최소 거리 (근사 없이)."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    c, b = d1 @ r, d1 @ d2
    den = a * e - b * b
    s = np.clip((b * f - c * e) / den, 0, 1) if den > 1e-12 else 0.0
    t = (b * s + f) / e
    if t < 0:
        t, s = 0.0, np.clip(-c / a, 0, 1)
    elif t > 1:
        t, s = 1.0, np.clip((b - c) / a, 0, 1)
    return float(np.linalg.norm((p1 + s * d1) - (p2 + t * d2)))


def analyze(c, name, trials, seconds, rng, dt=0.02):
    g, m = c["geometry"], c["motion"]
    amp = preset_amplitude(m, name)
    B, P = anchors(g)
    t = np.arange(0.0, seconds, dt)
    lo, hi, tilt_max, cond_max, gap_min, rel_max, vrel_max = np.inf, 0.0, 0.0, 0.0, np.inf, np.zeros(3), 0.0
    for _ in range(trials):
        q = sample_motion(m, amp, t, rng)
        V, R, cen = leg_vectors(B, P, float(g["height"]), q)
        L = np.linalg.norm(V, axis=2)
        lo, hi = min(lo, L.min()), max(hi, L.max())
        tilt_max = max(tilt_max, float(np.degrees(np.arccos(np.clip(V[..., 2] / L, -1, 1))).max()))
        idx = np.linspace(0, len(t) - 1, 40).astype(int)                  # 비싼 값은 일부 시각에서만
        cond_max = max(cond_max, float(jacobian_cond(B, P, float(g["height"]), R[idx], cen[idx]).max()))
        for n in idx[::4]:
            top = B + V[n]
            for i in range(6):
                for j in range(i + 1, 6):
                    if (B[i] == B[j]).all():
                        continue
                    gap_min = min(gap_min, seg_dist(B[i], top[i], B[j], top[j]))
        rel = np.einsum("nji,nj->ni", R, DRONE_BASE[None] - (cen - [0, 0, float(g["height"])])) - DRONE_BASE   # 로봇 base 기준 드론 이동
        rel_max = np.maximum(rel_max, np.abs(rel).max(axis=0))
        vrel_max = max(vrel_max, float(np.linalg.norm(np.diff(rel, axis=0), axis=1).max() / dt))
    return dict(name=name, amp=amp, lo=lo, hi=hi, tilt=tilt_max, cond=cond_max, gap=gap_min, rel=rel_max, vrel=vrel_max)


def main():
    ap = argparse.ArgumentParser(description="Stewart 플랫폼 크기 설계 (PLAN 8단계 S2)")
    ap.add_argument("--config", default=DEFAULT_CONFIG)
    ap.add_argument("--height", type=float, default=None)
    ap.add_argument("--base-radius", type=float, default=None)
    ap.add_argument("--top-radius", type=float, default=None)
    ap.add_argument("--trials", type=int, default=200, help="프리셋마다 무작위 에피소드 수")
    ap.add_argument("--seconds", type=float, default=60.0, help="에피소드 길이 [s]")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--margin", type=float, default=0.03, help="제안 다리 길이 여유 [m] (양쪽)")
    a = ap.parse_args()

    c = load(a.config)
    g = c["geometry"]
    for k, v in (("height", a.height), ("base_radius", a.base_radius), ("top_radius", a.top_radius)):
        if v is not None:
            g[k] = v
    B, P = anchors(g)
    L0 = np.linalg.norm(P + [0, 0, g["height"]] - B, axis=1)
    print(f"[design_stewart] {a.config}")
    print(f"  윗판 높이 {g['height']} m, 부착 반지름 아랫판 {g['base_radius']} / 윗판 {g['top_radius']} m, "
          f"판 두께 {g['base_plate']['thickness']} / {g['top_plate']['thickness']} m")
    print(f"  가운데 자세 다리 길이 {L0.min():.3f} ~ {L0.max():.3f} m, 기울기 "
          f"{np.degrees(np.arccos((P[:, 2] + g['height'] - B[:, 2]) / L0)).max():.1f}°")
    print(f"  무작위 {a.trials} 에피소드 × {a.seconds:g} s / 프리셋, ω × {c['motion']['omega_scale_range']}, 램프 {c['motion']['ramp_time']} s")
    print(f"  로봇 base 높이 (가운데) {g['height']:.3f} m, 드론 (base 기준 {DRONE_BASE.tolist()}) world 높이 {g['height'] + DRONE_BASE[2]:.3f} m")
    print("")
    print(f"  {'프리셋':6s} {'다리 길이 [m]':>17s} {'신축':>6s} {'비율':>5s} {'기울기':>7s} {'조건수':>7s} {'다리 간격':>8s}  드론 상대 흔들림 x/y/z [cm], 속도")
    rng = np.random.default_rng(a.seed)
    rows = [analyze(c, n, a.trials, a.seconds, rng) for n in c["motion"]["presets"]]
    for r in rows:
        print(f"  {r['name']:6s} {r['lo']:7.3f} ~ {r['hi']:6.3f} {r['hi'] - r['lo']:6.3f} {r['hi'] / r['lo']:5.2f} {r['tilt']:6.1f}° "
              f"{r['cond']:7.1f} {r['gap'] * 100:6.1f} cm  {r['rel'][0] * 100:4.1f}/{r['rel'][1] * 100:4.1f}/{r['rel'][2] * 100:4.1f}, "
              f"{r['vrel'] * 100:4.1f} cm/s")
    collect = [r for r in rows if c["motion"]["presets"][r["name"]].get("collect", True)]
    lo = min(r["lo"] for r in collect) - a.margin
    hi = max(r["hi"] for r in collect) + a.margin
    print("")
    print(f"  제안 (수집 프리셋 {[r['name'] for r in collect]} 이 들어가게, 여유 ±{a.margin} m): "
          f"legs.length_min {lo:.3f}, length_max {hi:.3f} → 신축 {hi - lo:.3f} m, 비율 {hi / lo:.2f}")
    print("    참고: 실제 리니어 액추에이터는 비율 약 2 이하 (실린더 안에 로드가 들어감). 조건수가 크면 특이점 근처, 다리 간격이 작으면 다리끼리 닿음")
    for r in rows:
        if not c["motion"]["presets"][r["name"]].get("collect", True) and (r["lo"] < lo or r["hi"] > hi):
            print(f"    {r['name']}: 제안 범위 밖 ({r['lo']:.3f} ~ {r['hi']:.3f}) → 씬에서 경고만 (수집 안 하는 프리셋)")


if __name__ == "__main__":
    main()
