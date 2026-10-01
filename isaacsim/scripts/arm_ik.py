# =============================================================
# arm_ik.py — 차분 IK (docs/PLAN.md 2-4, 5단계 SpaceMouse·PICO 텔레옵에서도 씀)
#
#   PhysX articulation Jacobian (get_jacobian_matrices, world 좌표계) 의 그리퍼 base 링크 행에서
#   TCP(= base 원점 + R·tcp_offset) Jacobian 을 만들고, damped least squares 로 팔 6 관절 변화량을 구한다:
#     e = [p* − p, log(R* Rᵀ)],  Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e,  |Δq| ≤ max_dq (관절별, 비율 유지)
#   관절 목표 = 이전 관절 목표 + Δq (텔레옵처럼 매 제어 주기 호출). 측정 관절각에 더하면 drive 중력 처짐(약 0.03°)만큼
#   계속 모자라 TCP 가 약 0.6 mm 못 감 (2026-10-01) → 명령에 누적해 처짐을 저절로 보상. 시작·순간이동 뒤에는 reset()
#   **PhysX Jacobian 의 선속도 행은 링크 원점이 아니라 링크 무게중심 기준** (2026-10-01 확인: 관절을 순간이동으로 1 mrad 씩
#   움직인 TCP 변화와 비교, 원점 기준으로 쓰면 축이 수평인 관절에서 약 3 cm/rad 차이 = 그리퍼 base 무게중심 위치) →
#   TCP 까지의 팔 r = R·(tcp_offset − com_local)
#
# SimulationApp 을 만든 뒤에 import 할 것.
# =============================================================

import numpy as np
from scipy.spatial.transform import Rotation

import collision_geom as cg


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


class DiffIK:
    def __init__(self, robot, link_path, link_name, arm_i, tcp_offset, damping=0.02, max_lead=0.05):
        from isaacsim.core.experimental.prims import RigidPrim

        names = list(robot.link_names)
        if link_name not in names:
            raise KeyError(f"{link_name} 이 articulation 링크에 없음: {names}")
        self.robot, self.arm_i = robot, np.asarray(arm_i)
        self.k = names.index(link_name)
        if self.k == 0:
            raise ValueError("root 링크는 fixed base Jacobian 에 행이 없음")
        self.link = RigidPrim(link_path)
        self.offset = np.asarray(tcp_offset, dtype=float)
        self.com = self.link.get_coms()[0].numpy().reshape(-1)[:3]   # 링크 좌표계 무게중심 (Jacobian 선속도 기준점)
        self.lam = float(damping)
        self.q_cmd = None
        # 와인드업 방지: 관절 목표가 측정값보다 max_lead [rad] 이상 앞서지 않게 (팔이 막혔을 때 목표가 끝없이 쌓이는 것 방지,
        #   2026-10-01 wrist_1 이 막힌 채 목표가 10 rad 넘게 쌓임). 중력 처짐(약 0.0006 rad)보다 충분히 큼
        self.max_lead = float(max_lead)

    def link_pose(self):
        p, q = (x.numpy().reshape(-1) for x in self.link.get_world_poses())
        return p, cg.quat_to_R(q)

    def tcp_pose(self):
        p, R = self.link_pose()
        return p + R @ self.offset, R

    def jacobian(self):
        """TCP Jacobian (6 × 팔 관절), world 좌표계 [v; ω]. fixed base: 링크 k 는 행 k−1 (root 제외)."""
        J = self.robot.get_jacobian_matrices().numpy()[0, self.k - 1][:, self.arm_i]
        p, R = self.link_pose()
        r = R @ (self.offset - self.com)
        Jv = J[:3] - skew(r) @ J[3:]          # v_tcp = v_com + ω × r = v_com − [r]× ω
        return np.vstack([Jv, J[3:]])

    def error(self, p_t, R_t):
        p, R = self.tcp_pose()
        e_rot = Rotation.from_matrix(R_t @ R.T).as_rotvec()
        return np.concatenate([np.asarray(p_t, dtype=float) - p, e_rot])

    def reset(self, q_cmd=None):
        """관절 목표 누적 시작값 (기본 측정 관절각)."""
        self.q_cmd = (self.robot.get_dof_positions().numpy()[0][self.arm_i].copy() if q_cmd is None
                      else np.asarray(q_cmd, dtype=float).copy())

    def step(self, p_t, R_t, max_dq):
        """목표 TCP 위치·자세로 가는 팔 관절 목표 (이전 목표 + Δq). (q 목표, 위치 오차 m, 자세 오차 rad) 를 돌려준다."""
        if self.q_cmd is None:
            raise RuntimeError("DiffIK.reset() 을 먼저 호출")
        e = self.error(p_t, R_t)
        J = self.jacobian()
        dq = J.T @ np.linalg.solve(J @ J.T + self.lam ** 2 * np.eye(6), e)
        m = np.max(np.abs(dq) / max_dq)
        if m > 1:
            dq /= m
        q_meas = self.robot.get_dof_positions().numpy()[0][self.arm_i]
        self.q_cmd = np.clip(self.q_cmd + dq, q_meas - self.max_lead, q_meas + self.max_lead)
        return self.q_cmd.copy(), float(np.linalg.norm(e[:3])), float(np.linalg.norm(e[3:]))
