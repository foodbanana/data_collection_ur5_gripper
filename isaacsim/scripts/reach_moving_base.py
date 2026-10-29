#!/usr/bin/env python3
# =============================================================
# reach_moving_base.py — 움직이는 베이스에서 드론을 잡을 수 있는 시간 비율·시작 자세 충돌 여유 (PLAN 8단계 S6)
#   Isaac Sim 없이 (numpy·yaml + teleop/ik_model.py URDF 기구학, 시스템 python3). 물리·렌더·자기 충돌은 보지 않는다 (기구학만)
#
#   드론은 world 에 고정 (기준 위치 목록 25 점 = scene_drone.yaml drone_targets_base, 리셋 흔들림 ± reset 값, 수평·yaw 0),
#   로봇 base 는 베이스 움직임 (moving_base: Stewart 프리셋 / Husky 궤적) 대로 움직인다. 시각마다 base 좌표계에서 본
#   "잡는 자세" (grasp_demo 와 같은 TCP·방향, 드론이 base 에 대해 기운 만큼 같이 기움) 를 IK 로 푼다 (이전 시각 해에서 이어서).
#     닿음      : IK 수렴 (위치 1 mm, 회전 0.01 rad) + 관절 한계 여유 ≥ 5° + 조작성 ≥ 0.01 (compare_drone_targets.py 와 같은 기준)
#     잡기 창   : 그 시각부터 손가락이 드론에 닿기까지 (기본 0.6 s) 계속 닿고, 잡는 점이 base 기준으로 3 cm 안에 머묾
#     충돌 여유 : 시작 자세 (under_drone) 그대로 있을 때 팔 (관절 원점·TCP 를 반지름으로 부풀린 점) 과 드론 외곽 상자 사이 최소 거리
#                 (텔레옵 전, 리셋 직후 팔이 드론에 부딪히는지. lab 프리셋에서 실제로 부딪힘, 2026-10-28)
#   잡는 TCP·드론 외곽 상자는 compare_drone_targets.py 가 드론 메시로 잰 값 (리포트 20261022_190652) 을 설정 상수로 둔다
#
# 실행 (레포 최상위):
#   python3 isaacsim/scripts/reach_moving_base.py --scene-config isaacsim/config/scene_stewart.yaml --base-motion small mid big lab
#   python3 isaacsim/scripts/reach_moving_base.py --scene-config isaacsim/config/scene_husky.yaml          # Husky (궤적 하나)
#   옵션: --seeds N (점마다 ω·위상·흔들림 seed 수), --seconds S, --dt S, --close-time S, --hold-cm C, --omega-scale a b
# =============================================================

import argparse
import math
import os
import sys
import time

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(SIM_ROOT), "teleop"))

import ik_model as km  # noqa: E402
import moving_base as mb  # noqa: E402

# compare_drone_targets.py 리포트 20261022_190652 (드론 메시에서 잰 값, 드론 좌표계 = body, mm → m)
DRONE_BOX = (np.array([-0.191, -0.180, -0.050]), np.array([0.201, 0.178, 0.035]))
TCP_IN_DRONE = np.array([0.0, -0.0013, -0.0045])
CRIT = {"pos": 1e-3, "rot": 1e-2, "limit_deg": 5.0, "manip": 0.01}
ARM_RADIUS = 0.0             # 충돌 여유: 팔 점 (관절 원점·TCP·TCP 위 3 cm) 을 부풀리는 반지름. 실제 sim 결과로 맞춤 (2026-10-29):
                             #   moving_base_scene --check seed 0, 드론 = 기본 위치에서 lab 은 t 약 5.0 ~ 5.3 s 에 팔이 드론을 밀었고 big 은 안 닿음
                             #   → 반지름 0: lab 이 0 아래로 가는 시각 5.04 s, big 최소 +2.4 cm (0.03 이상이면 big 도 닿는다고 틀리게 나옴)
CLEAR_WARN = 0.02            # 이보다 작으면 위험으로 표시 [m]


def grasp_frame(tool_x):
    x = np.asarray(tool_x, float)
    z = np.array([0.0, 0.0, 1.0])
    return np.column_stack([x, np.cross(z, x), z])


def solve(model, q0, p, R, iters):
    q = np.array(q0, float)
    stp = None
    for _ in range(iters):
        stp = km.ik_step(model, q, q, p, R, max_dq=0.05, max_lead=1.0)
        q = stp.q
        if stp.pos_err < CRIT["pos"] * 0.1 and stp.rot_err < CRIT["rot"] * 0.1:
            break
    return q, stp


def box_dist(points, c, R):
    """점들 (base 좌표) 과 드론 외곽 상자 (중심 c, 회전 R, base 좌표) 사이 최소 거리 (안이면 음수)."""
    lo, hi = DRONE_BOX
    local = (points - c) @ R                         # 드론 좌표
    d_out = np.linalg.norm(np.maximum(np.maximum(lo - local, local - hi), 0), axis=1)
    d_in = np.minimum(local - lo, hi - local).min(axis=1)
    return float(np.where(d_out > 0, d_out, -d_in).min())


def base_motion(scene_cfg, base_cfg, motion, seed, omega_scale):
    """시각 → (로봇 base 이동 Δp (3,), 회전 R (3x3)) world, 기준 = 움직임 가운데의 로봇 base. moving_base 와 같은 식."""
    mount = np.asarray(scene_cfg["mount"]["xyz"], float)
    myaw = math.radians(float(scene_cfg["mount"]["yaw_deg"]))
    Rm = mb._rz(myaw)
    if base_cfg["kind"] == "stewart":
        b = mb.StewartBase(base_cfg, motion, seed=seed, omega_scale=omega_scale)
        H = b.height

        def f(t):
            c, R, *_ = b.local_pose(t)
            p = c + R @ mount
            return p - (np.array([0, 0, H]) + mount), R @ Rm
        return f, b.traj.params()
    tr = base_cfg["trajectory"]
    k = mb.LinearTrajectory(tr["distance"], tr["max_speed"], tr["accel"], tr["dwell"])
    d = np.asarray(tr["direction"], float)
    d = mb._rz(-myaw) @ (d / np.linalg.norm(d))      # 베이스 yaw = −마운트 yaw (moving_base_scene 과 같음)

    def g(t):
        s = k.sample(t)[0]
        return (s - 0.5 * float(tr["distance"])) * d, np.eye(3)
    return g, {"trajectory": tr}


def analyze(model, q_start, targets, jitter, scene_cfg, base_cfg, motion, a, R_dg):
    lo_j, hi_j = model.lo, model.hi
    ts_ = np.arange(0.0, a.seconds, a.dt)
    n_close = max(1, int(round(a.close_time / a.dt)))
    # 시작 자세 팔 점 (base 좌표, 고정): 관절 원점 6 + TCP + TCP 위 3 cm (손가락 끝 쪽)
    T0 = model.fk(q_start)
    arm_pts = np.vstack([model.joint_origins(q_start), T0[:3, 3], T0[:3, 3] + T0[:3, 2] * 0.03])
    reach_all, win_all, clear_min, worst = [], [], np.inf, None
    for name, c0 in targets.items():
        for si in range(a.seeds):
            seed = a.seed0 + si
            rng = np.random.default_rng(10_000 + seed)
            d_w = c0 + rng.uniform(-1, 1, 3) * jitter          # 리셋 흔들림 (world 고정 드론, 가운데 base 기준)
            f, _ = base_motion(scene_cfg, base_cfg, motion, seed, a.omega_scale)
            q = q_start.copy()
            ok, gp = np.zeros(len(ts_), bool), np.zeros((len(ts_), 3))
            first = True
            for i, t in enumerate(ts_):
                dp, R = f(t)
                Rt = R.T
                c_b = Rt @ (d_w - dp)                          # base 좌표의 드론 중심
                R_b = Rt                                       # 드론 (수평) 이 base 에 대해 기운 회전
                p_g = c_b + R_b @ TCP_IN_DRONE
                R_g = R_b @ R_dg
                gp[i] = p_g
                cl = box_dist(arm_pts, c_b, R_b) - ARM_RADIUS
                if cl < clear_min:
                    clear_min, worst = cl, (name, seed, round(float(t), 2))
                q_try, stp = solve(model, q, p_g, R_g, 3000 if first else a.iters)
                lim = float(np.degrees(np.minimum(q_try - lo_j, hi_j - q_try).min()))
                good = (stp.pos_err < CRIT["pos"] and stp.rot_err < CRIT["rot"] and lim >= CRIT["limit_deg"] and stp.manip >= CRIT["manip"])
                ok[i] = good
                if good:
                    q, first = q_try, False
            win = np.zeros(len(ts_), bool)
            for i in range(len(ts_) - n_close):
                seg = slice(i, i + n_close + 1)
                if ok[seg].all() and np.linalg.norm(gp[seg] - gp[i], axis=1).max() <= a.hold_cm / 100:
                    win[i] = True
            m = ts_ >= a.skip                                  # 램프·드론 자리 잡기 구간 뺌
            reach_all.append(ok[m].mean())
            win_all.append(win[m][: max(1, m.sum() - n_close)].mean())
    return np.array(reach_all), np.array(win_all), clear_min, worst


def main():
    ap = argparse.ArgumentParser(description="움직이는 베이스 잡기 시간 비율·충돌 여유 (PLAN 8단계 S6)")
    ap.add_argument("--scene-config", default=os.path.join(SIM_ROOT, "config", "scene_stewart.yaml"))
    ap.add_argument("--base-motion", nargs="+", default=None, help="Stewart 프리셋 (여러 개). Husky 는 안 줌")
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--seed0", type=int, default=0)
    ap.add_argument("--seconds", type=float, default=40.0)
    ap.add_argument("--dt", type=float, default=0.1)
    ap.add_argument("--skip", type=float, default=5.0, help="앞 구간 뺌 [s] (램프)")
    ap.add_argument("--iters", type=int, default=60, help="이어서 풀 때 IK 반복")
    ap.add_argument("--close-time", type=float, default=0.6,
                    help="그리퍼가 열림에서 드론에 닿기까지 [s]. 완전히 닫는 데 2.2 s 지만 잡힐 때 state 0.18 ~ 0.28 (첫 소량 수집) → 약 1/4 지점에서 닿음")
    ap.add_argument("--hold-cm", type=float, default=3.0, help="잡기 창: 닫는 동안 잡는 점이 base 기준 머무를 범위 [cm]")
    ap.add_argument("--omega-scale", type=float, nargs=2, default=None, help="ω 배율 범위 덮어쓰기 (예 1 1 = 기준값 고정)")
    a = ap.parse_args()

    scene_cfg = yaml.safe_load(open(a.scene_config, encoding="utf-8"))
    base_cfg = mb.load_base_config(os.path.join(SIM_ROOT, scene_cfg["base_config"]))
    fixed = yaml.safe_load(open(os.path.join(SIM_ROOT, scene_cfg["fixed_scene_config"]), encoding="utf-8"))
    ros2 = yaml.safe_load(open(os.path.join(SIM_ROOT, "config", "ros2_iface.yaml"), encoding="utf-8"))
    gd = yaml.safe_load(open(os.path.join(SIM_ROOT, "config", "grasp_demo.yaml"), encoding="utf-8"))
    targets = {k: np.asarray(v, float) for k, v in fixed["drone_targets_base"].items()}
    jitter = float(ros2["reset"]["drone_offset"])
    q_start = np.asarray(fixed["init_poses"][scene_cfg["init_pose"]], float)
    R_dg = grasp_frame(gd["tool_x_in_drone"])
    model = km.ArmModel()
    motions = a.base_motion if base_cfg["kind"] == "stewart" else [None]
    if base_cfg["kind"] == "stewart" and not motions:
        ap.error("Stewart 는 --base-motion 이 필요")
    print(f"[reach_moving_base] {a.scene_config} (kind {base_cfg['kind']}), 기준 위치 {len(targets)} 점 × seed {a.seeds}, "
          f"흔들림 ±{jitter} m, {a.seconds:g} s (앞 {a.skip:g} s 뺌), dt {a.dt}, 닫는 시간 {a.close_time} s, 머묾 {a.hold_cm} cm"
          + (f", ω × {a.omega_scale}" if a.omega_scale else ""))
    print(f"  닿음 = IK (위치 < {CRIT['pos'] * 1000:.0f} mm, 회전 < {CRIT['rot']} rad) + 한계 여유 ≥ {CRIT['limit_deg']}° + 조작성 ≥ {CRIT['manip']};"
          f" 충돌 여유 = 시작 자세 팔 (관절 원점·TCP·손가락 끝 점) ↔ 드론 외곽 상자")
    print("")
    print(f"  {'프리셋':8s} {'닿는 시간 %':>22s} {'잡기 창 %':>22s} {'충돌 여유 최소':>14s}   가장 가까운 순간")
    print(f"  {'':8s} {'평균 (최소 ~ 최대)':>22s} {'평균 (최소 ~ 최대)':>22s}")
    for mo in motions:
        t0 = time.time()
        r, w, cl, worst = analyze(model, q_start, targets, jitter, scene_cfg, base_cfg, mo, a, R_dg)
        print(f"  {str(mo or 'husky'):8s} {r.mean() * 100:8.1f} ({r.min() * 100:5.1f} ~ {r.max() * 100:5.1f}) "
              f"{w.mean() * 100:8.1f} ({w.min() * 100:5.1f} ~ {w.max() * 100:5.1f}) {cl * 100:10.1f} cm   {worst}  ({time.time() - t0:.0f} s)",
              flush=True)
    print("")
    print(f"  충돌 여유 < {CLEAR_WARN * 100:.0f} cm 는 위험 (시작 자세 그대로 둘 때 팔이 드론에 닿을 수 있음, 반지름 {ARM_RADIUS} 은 sim 결과로 맞춘 값)")
    print("  참고: 기구학만 (자기 충돌·테이블·카메라 시야·드론 흔들림 (hover) 은 안 봄). 충돌 여유가 음수면 시작 자세 그대로 둘 때 팔이 드론 상자 안으로 들어감")


if __name__ == "__main__":
    main()
