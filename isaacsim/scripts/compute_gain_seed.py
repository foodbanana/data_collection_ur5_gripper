#!/usr/bin/env python3
# =============================================================
# compute_gain_seed.py  (docs/PLAN.md 1-5)
#
# drive gain 시작값 계산. 테스트 씬에서 질량행렬과 중력 토크를 직접 읽는다.
#   팔:     K = I·ω², D = 2·ζ·I·ω (ζ = 1)
#           I = 질량행렬 대각값 (무작위 자세 중 최대 → 어느 자세에서도 ζ ≥ 1, overshoot 없음)
#           ω = max(정착시간 기준 ω, 중력 처짐 기준 ω). 중력은 드론 1.5 kg, 무게중심이 공구 축에서 5 cm 벗어난 최악 방향
#   gravity_ff 설정(drive_gains_ff.yaml)용: ω = W_FF (정착시간 기준), 드론 무게만 남는 부하로 예상 처짐 출력
#   B 시험용 관절별 최악 경우를 isaacsim/config/gravity_worst_cases.yaml 로 저장
#   그리퍼: K ≥ 파지 토크 / 최소 남은 오차 (물체에 닿으면 힘 한계로 누르는 상태), D = 2·√(K·I) (임계감쇠)
#           I = mimic 으로 묶인 4개 DOF 질량행렬 부분합 (네 조인트가 같이 움직이므로)
#
# 실행:
#   ~/isaacsim/python.sh ~/data_collection_ur5_gripper/isaacsim/scripts/compute_gain_seed.py --headless
#
# 단위: 모두 rad 기준 (Nm/rad, Nm·s/rad). tensor API 단위와 같다.
# =============================================================

import argparse
import datetime
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_scene as ts  # noqa: E402

parser = argparse.ArgumentParser(description="drive gain 시작값 계산 (PLAN 1-5)")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--usd", default=ts.DEFAULT_USD)
parser.add_argument("--physics-variant", default="physx")
parser.add_argument("--config", default=ts.DEFAULT_CONFIG)
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import numpy as np  # noqa: E402

# ── 설계 기준 ──
HOME = [0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0]
ZERO = [0.0] * 6                    # 팔을 수평으로 뻗은 자세 = shoulder_lift 중력 최악
SETTLE_DESIGN = 0.3                 # s, 합격 기준 0.5 s 에 여유. 임계감쇠 2% 정착 ≈ 5.83/ω
ERR_DESIGN = math.radians(0.05)     # rad, 합격 기준 0.1° 의 절반
PAYLOAD_KG = 1.5                    # 드론 무게 (그리퍼 base 링크에 더함, 메모리에서만)
PAYLOAD_LINK = "rh_p12_rn_base"
PAYLOAD_POS = (0.0, 0.0, 0.11)      # m, TCP 위치 (rh_p12_rn_base 좌표계, URDF rh_p12_rn_tcp_joint) = 드론 무게중심
N_SAMPLES = 300                     # 관절별 최악 중력 토크를 찾기 위한 무작위 자세 수
OFFSETS = (0.0, 0.03, 0.05)         # m, 드론 무게중심이 공구 축에서 벗어난 거리 (최악 방향)
OFFSET_DESIGN = 0.05                # m, gain 설계에 쓰는 오프셋
OFFSET_PROBE = 0.05                 # m, 선형성으로 방향별 토크를 구할 때 쓰는 탐침 오프셋
W_LAG = 100.0                       # rad/s, C 추종 지연 < 40 ms 기준. 1회차 후 70 추가, 2회차 실측(ω 70 → 42 ms, 지연 ≈ 2.47/ω + 10.7 ms) 후 100
W_FF = 30.0                         # rad/s, gravity_ff 설정의 ω (정착시간 기준 20~40 의 가운데)
SEED = 0
GRIP_FORCE_N = 20.0                 # 400 mA 임시 파지력 (사양 선형 환산)
GRIP_LEVER_M = 0.057                # r1 축 → r2 축 거리 (URDF), 손가락 2개가 한 drive 에 묶임
GRIP_ERR_MIN = 0.05                 # rad, 물체에 닿았을 때 목표까지 남는 최소 오차 (얇은 물체 가정)
GRIP_MAX_VEL = 1.1351 / 2.2         # rad/s, 실물 열림·닫힘 약 2.2 s (임시)
ARM_MAX_VEL = math.pi               # rad/s, UR5 관절 180°/s


def set_pose(robot, idx, arm_q):
    q = np.zeros(robot.num_dofs, dtype=np.float32)
    for n, v in zip(ts.ARM_JOINTS, arm_q):
        q[idx[n]] = v
    robot.set_dof_positions(q)
    robot.set_dof_velocities(np.zeros_like(q))
    simulation_app.update()
    robot.set_dof_positions(q)       # 한 스텝 사이 처짐을 되돌림
    robot.set_dof_velocities(np.zeros_like(q))


def read_state(robot, idx):
    M = robot.get_mass_matrices().numpy()[0]
    g = robot.get_dof_gravity_compensation_forces().numpy()[0]
    arm_I = np.array([M[idx[n], idx[n]] for n in ts.ARM_JOINTS])
    gi = [idx[n] for n in ts.GRIPPER_JOINTS]
    grip_I = float(M[np.ix_(gi, gi)].sum())
    arm_g = np.array([abs(g[idx[n]]) for n in ts.ARM_JOINTS])
    return arm_I, grip_I, arm_g


def main():
    lines = []

    def log(msg=""):
        print(msg, flush=True)
        lines.append(msg)

    log(f"compute_gain_seed  {datetime.datetime.now().isoformat(timespec='seconds')}")
    log(f"robot USD: {args.usd}  (variant {args.physics_variant}, base z = {ts.ROBOT_BASE_Z} m)")

    config = ts.load_config(args.config)
    stage, robot, info = ts.build_test_scene(args.usd, args.physics_variant, base="fixed", config=config,
                                               light=not args.headless)
    app_utils.play()
    simulation_app.update()
    problems, vlines = ts.verify_articulation(stage, robot, info, config)
    for line in vlines:
        log(f"  {line}")
    if problems:  # floating base 면 질량행렬에 베이스 6자유도가 섞여 값이 틀린다
        raise RuntimeError(f"articulation 설정 확인 실패: {problems}")
    idx = ts.dof_index_map(robot)
    mv = robot.get_dof_max_velocities().numpy()[0]
    me = robot.get_dof_max_efforts().numpy()[0]
    log("  현재 DOF 한계 (USD 에서 온 값, tensor 단위): " +
        ", ".join(f"{n}: vel {mv[i]:.4g} rad/s, effort {me[i]:.4g} Nm" for n, i in idx.items()))
    M_shape = robot.get_mass_matrices().numpy().shape
    if M_shape[1:] != (robot.num_dofs, robot.num_dofs):
        raise RuntimeError(f"질량행렬 모양 {M_shape} 가 DOF 수 {robot.num_dofs} 와 맞지 않음 (fixed base 아님?)")

    # 자세 목록: 홈, 관절 0, 무작위 N_SAMPLES 개
    rng = np.random.default_rng(SEED)
    lim = np.array([2 * math.pi, 2 * math.pi, math.pi, 2 * math.pi, 2 * math.pi, 2 * math.pi])
    poses = [np.array(HOME), np.array(ZERO)] + [rng.uniform(-lim, lim) for _ in range(N_SAMPLES)]

    # 드론 무게: 그리퍼 base 링크에 +1.5 kg 을 더하고 합친 무게중심을 옮긴다 (메모리에서만).
    # 중력 토크는 무게중심 위치에 선형 → 무게중심을 TCP, TCP+δx, TCP+δy 세 곳에 두고 재면
    # 공구 축에서 r 만큼 벗어난 모든 방향의 최악값을 정확히 구할 수 있다: max|τ| = |τ_TCP| + r·‖B‖
    li = robot.get_link_indices(PAYLOAD_LINK).numpy()
    m0 = robot.get_link_masses(link_indices=li).numpy().copy()
    c0_pos, c0_rot = (x.numpy().copy() for x in robot.get_link_coms(link_indices=li))
    m_base = float(m0.reshape(-1)[0])

    def sweep(payload_offset):
        if payload_offset is None:
            robot.set_link_masses(m0, link_indices=li)
            robot.set_link_coms(positions=c0_pos, orientations=c0_rot, link_indices=li)
        else:
            p = np.array(PAYLOAD_POS) + np.array(payload_offset)
            c = (m_base * c0_pos.reshape(3) + PAYLOAD_KG * p) / (m_base + PAYLOAD_KG)
            robot.set_link_masses(m0 + PAYLOAD_KG, link_indices=li)
            robot.set_link_coms(positions=c.reshape(c0_pos.shape), orientations=c0_rot, link_indices=li)
        Is, Gs, gIs = [], [], []
        for q in poses:
            set_pose(robot, idx, q)
            I_q, gI_q, _ = read_state(robot, idx)
            g = robot.get_dof_gravity_compensation_forces().numpy()[0]
            Gs.append([g[idx[n]] for n in ts.ARM_JOINTS])  # 부호 유지
            Is.append(I_q)
            gIs.append(gI_q)
        return np.array(Is), np.array(Gs), np.array(gIs)

    I_all, G_none, gI_all = sweep(None)
    I_pay, G_tcp, _ = sweep((0.0, 0.0, 0.0))
    _, G_x, _ = sweep((OFFSET_PROBE, 0.0, 0.0))
    _, G_y, _ = sweep((0.0, OFFSET_PROBE, 0.0))
    sweep(None)  # 원래대로
    app_utils.stop()

    B = np.stack([(G_x - G_tcp) / OFFSET_PROBE, (G_y - G_tcp) / OFFSET_PROBE], axis=-1)  # (포즈, 관절, 2) Nm/m
    Bn = np.linalg.norm(B, axis=-1)

    def worst(r, base):
        """base: 무게중심이 TCP 에 있을 때의 토크 (포즈, 관절). r 만큼 최악 방향으로 옮겼을 때 |τ| 와 그 방향."""
        tau = np.abs(base) + r * Bn
        k = np.argmax(tau, axis=0)
        cases = []
        for j in range(6):
            b = B[k[j], j]
            sgn = 1.0 if base[k[j], j] >= 0 else -1.0
            d = (r * sgn * b / np.linalg.norm(b)) if np.linalg.norm(b) > 1e-9 else np.zeros(2)
            cases.append({"pose": poses[k[j]], "offset": d, "tau": float(tau[k[j], j])})
        return tau.max(axis=0), cases

    I_arm = np.maximum(I_all.max(axis=0), I_pay.max(axis=0))  # 드론을 단 경우까지 포함한 최대 관성 → 어느 경우에도 ζ ≥ 1
    tau_r = {r: worst(r, G_tcp) for r in OFFSETS}                  # 로봇 + 드론 (gravity_ff 없음)
    pay_r = {r: worst(r, G_tcp - G_none) for r in OFFSETS}         # 드론 무게만 (gravity_ff 켜면 남는 부하)
    g_max = tau_r[OFFSET_DESIGN][0]

    w_settle = 5.83 / SETTLE_DESIGN
    w_grav = np.sqrt(g_max / (ERR_DESIGN * I_arm))
    w = np.maximum(np.maximum(w_settle, w_grav), W_LAG)
    K = I_arm * w ** 2
    D = 2.0 * I_arm * w
    K_ff = I_arm * W_FF ** 2
    D_ff = 2.0 * I_arm * W_FF

    log("")
    log(f"설계 기준: ω ≥ {W_LAG:.0f} rad/s (추종 지연), 정착 {SETTLE_DESIGN} s → ω_settle = {w_settle:.1f} rad/s, 중력 처짐 ≤ {math.degrees(ERR_DESIGN):.2f}°, "
        f"드론 {PAYLOAD_KG} kg (무게중심 공구 축에서 {OFFSET_DESIGN * 100:.0f} cm, 최악 방향), ζ = 1, physics dt = {ts.PHYSICS_DT:.5f} s")
    log(f"  자세: 홈, 관절 0, 무작위 {N_SAMPLES} 개. I = 자세 중 최대 질량행렬 대각값 (드론 무게 포함 경우까지)")
    log("")
    log("관절별 최악 중력 토크 |τg| (Nm): 로봇 자체 / 로봇+드론(무게중심 오프셋 0·3·5 cm) / 드론만(오프셋 0·3·5 cm)")
    for i, n in enumerate(ts.ARM_JOINTS):
        log(f"  {n:20s} 로봇 {np.abs(G_none[:, i]).max():6.2f} | "
            + " ".join(f"{tau_r[r][0][i]:6.2f}" for r in OFFSETS) + " | "
            + " ".join(f"{pay_r[r][0][i]:5.2f}" for r in OFFSETS))

    log("")
    log("[drive_gains.yaml] gravity_ff 없음, 높은 stiffness (I: kg·m², ω: rad/s, K: Nm/rad, D: Nm·s/rad)")
    log(f"  {'joint':20s} {'I_max':>8s} {'τg_max':>8s} {'ω_grav':>7s} {'ω':>6s} {'K':>10s} {'D':>9s} {'ω·dt':>5s}  근거")
    for i, n in enumerate(ts.ARM_JOINTS):
        why = "중력 처짐" if w_grav[i] >= max(w_settle, W_LAG) else ("추종 지연" if W_LAG > w_settle else "정착시간")
        log(f"  {n:20s} {I_arm[i]:8.4f} {g_max[i]:8.2f} {w_grav[i]:7.1f} {w[i]:6.1f} {K[i]:10.1f} {D[i]:9.2f} "
            f"{w[i] * ts.PHYSICS_DT:5.2f}  {why}")

    log("")
    log(f"[drive_gains_ff.yaml] gravity_ff 켬, ω = {W_FF:.0f} rad/s (정착시간 기준). 드론 무게는 보상 안 함 → 예상 처짐")
    log(f"  {'joint':20s} {'K':>10s} {'D':>9s}  " + "  ".join(f"처짐@{r * 100:.0f}cm" for r in OFFSETS))
    for i, n in enumerate(ts.ARM_JOINTS):
        log(f"  {n:20s} {K_ff[i]:10.1f} {D_ff[i]:9.2f}  "
            + "  ".join(f"{math.degrees(pay_r[r][0][i] / K_ff[i]):8.3f}°" for r in OFFSETS))

    # B 시험용 최악 경우 (관절 × 오프셋)
    case_lines = [
        "# 자동 생성: compute_gain_seed.py (" + datetime.datetime.now().isoformat(timespec="seconds") + ")",
        "# B 시험(tune_drives.py)용 관절별 최악 중력 경우. 드론 1.5 kg, 무게중심 = TCP + offset (rh_p12_rn_base 좌표계, 공구 축에 수직)",
        "# pose: 팔 6관절 [rad], offset: [x, y] m, tau: 로봇+드론 중력 토크 크기 [Nm]",
        f"payload_kg: {PAYLOAD_KG}",
        f"payload_pos: {list(PAYLOAD_POS)}",
        "cases:",
    ]
    for r in OFFSETS[1:]:
        for j, n in enumerate(ts.ARM_JOINTS):
            c = tau_r[r][1][j]
            case_lines.append(
                f"  - {{joint: {n}, offset_cm: {r * 100:.0f}, pose: {[round(float(x), 4) for x in c['pose']]}, "
                f"offset: {[round(float(x), 4) for x in c['offset']]}, tau: {c['tau']:.2f}}}")
    case_path = os.path.join(ts.CONFIG_DIR, "gravity_worst_cases.yaml")
    with open(case_path, "w", encoding="utf-8") as f:
        f.write("\n".join(case_lines) + "\n")
    log("")
    log(f"B 시험용 최악 경우: {case_path}")

    gI_home, gI_zero = gI_all[0], gI_all[1]

    grip_I = max(gI_home, gI_zero)
    grip_tau = 2 * GRIP_FORCE_N * GRIP_LEVER_M
    grip_K = max(grip_tau / GRIP_ERR_MIN, grip_I * w_settle ** 2)
    grip_D = 2.0 * math.sqrt(grip_K * grip_I)
    log("")
    log("그리퍼 rh_r1_joint (mimic 4 DOF 묶음)")
    log(f"  I = {grip_I:.3e} kg·m²,  임시 maxForce τ = 2 × {GRIP_FORCE_N} N × {GRIP_LEVER_M} m = {grip_tau:.2f} Nm")
    log(f"  K = max(τ / {GRIP_ERR_MIN} rad, I·ω_settle²) = {grip_K:.1f} Nm/rad,  D = 2√(K·I) = {grip_D:.4f} Nm·s/rad")
    log(f"  목표값 이동 속도(profile_velocity) = {GRIP_MAX_VEL:.4f} rad/s, 관절 속도 한계는 USD 값 유지 (6.5 rad/s)")

    log("")
    log("# ---- drive_gains_ff.yaml (gravity_ff: true) 팔 ----")
    for i, n in enumerate(ts.ARM_JOINTS):
        log(f"  {n}: {{stiffness: {K_ff[i]:.1f}, damping: {D_ff[i]:.2f}}}")
    log("# ---- drive_gains.yaml 시작값 ----")
    for i, n in enumerate(ts.ARM_JOINTS):
        log(f"  {n}: {{stiffness: {K[i]:.1f}, damping: {D[i]:.2f}, max_velocity: {ARM_MAX_VEL:.4f}}}")
    log(f"  {ts.GRIPPER_DRIVE}: {{stiffness: {grip_K:.1f}, damping: {grip_D:.4f}, max_force: {grip_tau:.2f}, "
        f"profile_velocity: {GRIP_MAX_VEL:.4f}, max_velocity: 6.5}}")

    os.makedirs(ts.REPORT_DIR, exist_ok=True)
    path = os.path.join(ts.REPORT_DIR, f"gain_seed_{datetime.datetime.now():%Y%m%d_%H%M%S}.txt")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"report: {path}", flush=True)


if __name__ == "__main__":
    code = 0
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        code = 1
    finally:
        simulation_app.close(exit_code=code)
    sys.exit(code)
