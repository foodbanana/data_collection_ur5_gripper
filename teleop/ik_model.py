#!/usr/bin/env python3
# =============================================================
# ik_model.py — URDF 기반 팔 기구학 + 차분 IK 한 스텝 (docs/PLAN.md 5-B B-1). numpy 와 표준 라이브러리만 쓴다
#
#   텔레옵 노드는 Isaac Sim 밖 (그리고 실물 PC) 에서 돌아야 하므로 PhysX Jacobian (isaacsim/scripts/arm_ik.py) 을 쓸 수 없다
#   → 빌드한 URDF (isaacsim/ur5_rh_p12_description/out/) 의 관절 원점·축을 읽어 직접 계산한다
#     (pinocchio 는 최신판이 numpy 2 를 요구해 venv 에서 시스템 numpy 1.26 을 가리므로 쓰지 않음, 2026-10-12 결정)
#
#   ArmModel(urdf, base, tip, arm_joints)
#     fk(q)        → 4x4, base 좌표계의 tip 자세. q = 팔 6 관절 [rad] (arm_joints 순서)
#     jacobian(q)  → 6 x 6 [v; ω], base 좌표계, 점 = tip 원점 (기하 Jacobian)
#     limits       → (lo, hi) [rad]
#     joint_origins(q) → 6 x 3, 팔 관절 원점 위치 (base 좌표계)
#   ik_step(model, q_cmd, q_meas, p_target, R_target, ...)  → 새 관절 목표 (IkStep)
#     e = [p* − p, log(R* Rᵀ)],  Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e,  |Δq| ≤ max_dq (관절별, 비율 유지)
#     관절 목표 = 이전 목표 + Δq  (arm_ik.py 와 같은 규칙)
#     **오차는 명령 관절각의 FK 기준** (arm_ik.py 는 측정 기준): 텔레옵 노드는 sim 밖이라 명령 → 실제 팔 반영에
#       약 50 ~ 80 ms (보간 33 ms + drive) 가 걸리고, 그동안 측정 기준 오차를 계속 더하면 목표를 지나친다.
#       측정값은 와인드업 방지 (목표가 측정보다 max_lead 넘게 앞서지 않게) 에만 쓴다
#     관절 한계: 한계 − limit_margin 안으로 자르고 표시 (sim 은 한계 밖 /joint_command 를 받으면 에러로 종료)
#
# 단독 실행 (자기 점검): python3 teleop/ik_model.py [--urdf <경로>]
#   체인 요약, 홈 자세 FK, Jacobian 을 수치 미분과 비교, 무작위 목표로 ik_step 수렴
# =============================================================

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
DEFAULT_URDF = os.path.join(REPO, "isaacsim", "ur5_rh_p12_description", "out", "ur5_rh_p12_d435i.urdf")
ARM_JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")
BASE_LINK = "robot_mount"           # 로봇 base (sim: 로봇 최상위 prim = root_joint 위치, 데이터셋 "로봇 base 기준")
TIP_LINK = "rh_p12_rn_tcp"          # 손가락 사이 (rh_p12_rn_base 에서 z 0.11)
HOME = (0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0)   # isaacsim/config/drive_gains.yaml home_pose


# ---------------------------------------------------------------- 변환
def rpy_to_R(r, p, y):
    """URDF rpy (고정축 x → y → z): R = Rz(y) Ry(p) Rx(r)."""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle_R(axis, q):
    a = np.asarray(axis, dtype=float)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * (K @ K)


def make_T(R, p):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, p
    return T


def R_to_quat(R):
    """회전행렬 → 쿼터니언 (x, y, z, w), w ≥ 0. Shepperd 방법: 가장 큰 성분으로 나눠 어느 각도에서나 안정적."""
    t = np.trace(R)
    d = np.array([t, R[0, 0], R[1, 1], R[2, 2]])
    k = int(np.argmax(d))
    if k == 0:
        w = math.sqrt(1 + t) / 2
        q = np.array([(R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w])
    else:
        i, j, l = k - 1, k % 3, (k + 1) % 3
        s = math.sqrt(max(1 + 2 * R[i, i] - t, 0.0)) / 2
        q = np.zeros(4)
        q[i] = s
        q[j] = (R[j, i] + R[i, j]) / (4 * s)
        q[l] = (R[l, i] + R[i, l]) / (4 * s)
        q[3] = (R[l, j] - R[j, l]) / (4 * s)
    q /= np.linalg.norm(q)
    return q if q[3] >= 0 else -q


def R_to_rotvec(R):
    """회전행렬 → 회전 벡터 (축 × 각도, 0 ~ π). 쿼터니언을 거쳐 atan2 로 각도를 구해 180° 근처에서도 정밀도가 유지된다."""
    q = R_to_quat(R)
    s = float(np.linalg.norm(q[:3]))
    if s < 1e-12:
        return np.zeros(3)
    return (2 * math.atan2(s, q[3]) / s) * q[:3]


# ---------------------------------------------------------------- 모델
class ArmModel:
    """URDF 의 base → tip 직렬 체인. 고정 관절은 원점 변환만, 회전 관절은 arm_joints 에 있어야 한다 (모르는 가동 관절은 에러)."""

    def __init__(self, urdf=DEFAULT_URDF, base=BASE_LINK, tip=TIP_LINK, arm_joints=ARM_JOINTS):
        if not os.path.isfile(urdf):
            raise FileNotFoundError(f"URDF 없음: {urdf} (isaacsim/ur5_rh_p12_description/scripts/build_urdf.sh 로 빌드)")
        self.urdf, self.base, self.tip, self.arm_joints = urdf, base, tip, tuple(arm_joints)
        root = ET.parse(urdf).getroot()
        by_child = {}
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            ax = j.find("axis")
            lim = j.find("limit")
            by_child[j.find("child").get("link")] = {
                "name": j.get("name"), "type": j.get("type"), "parent": j.find("parent").get("link"),
                "T": make_T(rpy_to_R(*rpy), np.array(xyz)),
                "axis": np.array([float(v) for v in (ax.get("xyz") if ax is not None else "1 0 0").split()]),
                "lo": float(lim.get("lower")) if lim is not None and lim.get("lower") else None,
                "hi": float(lim.get("upper")) if lim is not None and lim.get("upper") else None}
        # tip 에서 base 까지 거꾸로
        chain, link = [], tip
        while link != base:
            if link not in by_child:
                raise ValueError(f"{tip} 에서 {base} 까지 체인이 이어지지 않음 ({link} 의 부모 관절 없음)")
            chain.append(by_child[link])
            link = chain[-1]["parent"]
        chain.reverse()
        self.chain, names = [], []
        for j in chain:
            if j["type"] == "fixed":
                self.chain.append((j["T"], None, None))
            elif j["type"] in ("revolute", "continuous"):
                if j["name"] not in self.arm_joints:
                    raise ValueError(f"체인에 arm_joints 밖의 가동 관절: {j['name']}")
                n = np.linalg.norm(j["axis"])
                self.chain.append((j["T"], self.arm_joints.index(j["name"]), j["axis"] / n))
                names.append(j["name"])
            else:
                raise ValueError(f"지원하지 않는 관절 형식: {j['name']} ({j['type']})")
        if sorted(names) != sorted(self.arm_joints):
            raise ValueError(f"체인의 가동 관절 {names} 가 arm_joints {list(self.arm_joints)} 와 다름")
        js = {by_child[c]["name"]: by_child[c] for c in by_child}
        self.lo = np.array([js[n]["lo"] for n in self.arm_joints], dtype=float)
        self.hi = np.array([js[n]["hi"] for n in self.arm_joints], dtype=float)

    @property
    def limits(self):
        return self.lo.copy(), self.hi.copy()

    def _frames(self, q):
        """각 가동 관절의 (base 좌표계 축, 관절 원점) 과 tip 자세."""
        q = np.asarray(q, dtype=float)
        if q.shape != (len(self.arm_joints),) or not np.all(np.isfinite(q)):
            raise ValueError(f"관절각은 유한한 {len(self.arm_joints)} 개여야 함: {q}")
        T = np.eye(4)
        axes = []
        for To, idx, ax in self.chain:
            T = T @ To
            if idx is not None:
                axes.append((idx, T[:3, :3] @ ax, T[:3, 3].copy()))
                T = T @ make_T(axis_angle_R(ax, q[idx]), np.zeros(3))
        return axes, T

    def fk(self, q):
        return self._frames(q)[1]

    def joint_origins(self, q):
        """팔 관절 원점의 위치 (6 x 3, base 좌표계, arm_joints 순서). 텔레옵 노드가 손목 높이를 볼 때 쓴다."""
        P = np.zeros((len(self.arm_joints), 3))
        for idx, _, o in self._frames(q)[0]:
            P[idx] = o
        return P

    def jacobian(self, q):
        axes, T = self._frames(q)
        p = T[:3, 3]
        J = np.zeros((6, len(self.arm_joints)))
        for idx, z, o in axes:
            J[:3, idx] = np.cross(z, p - o)
            J[3:, idx] = z
        return J


# ---------------------------------------------------------------- 차분 IK 한 스텝
class IkStep:
    __slots__ = ("q", "pos_err", "rot_err", "limited", "manip")

    def __init__(self, q, pos_err, rot_err, limited, manip):
        self.q, self.pos_err, self.rot_err, self.limited, self.manip = q, pos_err, rot_err, limited, manip

    def __repr__(self):
        return (f"IkStep(q={np.round(self.q, 4).tolist()}, pos_err={self.pos_err * 1000:.2f} mm, "
                f"rot_err={math.degrees(self.rot_err):.2f}°, limited={self.limited}, manip={self.manip:.4f})")


def ik_step(model, q_cmd, q_meas, p_target, R_target, max_dq=0.05, damping=0.02, max_lead=0.05, limit_margin=0.05):
    """목표 TCP 자세 (base 좌표계) 로 가는 다음 관절 목표.
    q_cmd: 지금 관절 목표 (지난 스텝 결과), q_meas: 측정 관절각 (와인드업 방지용).
    max_dq [rad]: 한 스텝 관절 변화 한계 (관절별, 비율 유지). max_lead [rad]: 목표가 측정보다 앞설 수 있는 최대.
    limit_margin [rad]: 관절 한계 안쪽 여유. limited = 한계나 와인드업으로 잘렸는지 (관절 이름 목록).
    manip = sqrt(det(J Jᵀ)) (0 에 가까우면 특이점)."""
    q_cmd, q_meas = np.asarray(q_cmd, dtype=float), np.asarray(q_meas, dtype=float)
    p_target, R_target = np.asarray(p_target, dtype=float), np.asarray(R_target, dtype=float)
    if not (np.all(np.isfinite(p_target)) and np.all(np.isfinite(R_target))):
        raise ValueError("목표 자세가 유한하지 않음")
    T = model.fk(q_cmd)
    e = np.concatenate([p_target - T[:3, 3], R_to_rotvec(R_target @ T[:3, :3].T)])
    J = model.jacobian(q_cmd)
    JJt = J @ J.T
    dq = J.T @ np.linalg.solve(JJt + damping ** 2 * np.eye(6), e)
    m = float(np.max(np.abs(dq)) / max_dq)
    if m > 1:
        dq /= m
    q_new = q_cmd + dq
    limited = []
    lo_lead, hi_lead = q_meas - max_lead, q_meas + max_lead
    lo_lim, hi_lim = model.lo + limit_margin, model.hi - limit_margin
    for i, n in enumerate(model.arm_joints):
        if q_new[i] < lo_lead[i] or q_new[i] > hi_lead[i]:
            q_new[i] = min(max(q_new[i], lo_lead[i]), hi_lead[i])
            limited.append(f"{n}:lead")
        if q_new[i] < lo_lim[i] or q_new[i] > hi_lim[i]:
            q_new[i] = min(max(q_new[i], lo_lim[i]), hi_lim[i])
            limited.append(f"{n}:limit")
    manip = float(math.sqrt(max(np.linalg.det(JJt), 0.0)))
    return IkStep(q_new, float(np.linalg.norm(e[:3])), float(np.linalg.norm(e[3:])), limited, manip)


# ---------------------------------------------------------------- 자기 점검
def _self_check(urdf):
    m = ArmModel(urdf)
    print(f"URDF {urdf}")
    print(f"체인 {BASE_LINK} → {TIP_LINK}: 원점 변환 {len(m.chain)} 개, 가동 관절 {list(m.arm_joints)}")
    print(f"관절 한계 [rad]: {np.round(m.lo, 4).tolist()} ~ {np.round(m.hi, 4).tolist()}")
    T = m.fk(HOME)
    print(f"홈 자세 TCP (base 기준) [m]: {np.round(T[:3, 3], 4).tolist()}, 공구 축(z) {np.round(T[:3, 2], 3).tolist()}")
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(200):
        q = rng.uniform(-math.pi, math.pi, 6)
        J = m.jacobian(q)
        Jn = np.zeros((6, 6))
        T0 = m.fk(q)
        for i in range(6):
            dq = np.zeros(6)
            dq[i] = 1e-6
            T1 = m.fk(q + dq)
            Jn[:3, i] = (T1[:3, 3] - T0[:3, 3]) / 1e-6
            Jn[3:, i] = R_to_rotvec(T1[:3, :3] @ T0[:3, :3].T) / 1e-6
        worst = max(worst, float(np.max(np.abs(J - Jn))))
    print(f"Jacobian vs 수치 미분 (무작위 200 자세): 최대 차이 {worst:.2e}")
    ok = 0
    for k in range(50):
        q0 = np.array(HOME) + rng.uniform(-0.4, 0.4, 6)
        Tt = m.fk(q0 + rng.uniform(-0.15, 0.15, 6))
        q = q0.copy()
        for _ in range(200):
            s = ik_step(m, q, q, Tt[:3, 3], Tt[:3, :3], max_lead=10.0)
            q = s.q
            if s.pos_err < 1e-5 and s.rot_err < 1e-4:
                ok += 1
                break
    print(f"ik_step 수렴 (홈 근처 무작위 목표 50 개, 200 스텝 안에 0.01 mm·0.006°): {ok} / 50")
    return worst < 1e-4 and ok >= 48


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="URDF 팔 기구학 자기 점검")
    ap.add_argument("--urdf", default=DEFAULT_URDF)
    a = ap.parse_args()
    sys.exit(0 if _self_check(a.urdf) else 1)
