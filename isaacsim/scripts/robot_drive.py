# =============================================================
# robot_drive.py — drive 설정 적용, 그리퍼 목표값 이동, 중력 보상 feedforward (docs/PLAN.md 1-5)
#
#   설정 파일(isaacsim/config/drive_gains*.yaml)의 값을 tensor API 로 적용한다 (rad 기준 단위).
#   USD 원본은 수정하지 않는다. 2·3단계 씬 스크립트와 3단계 sim 그리퍼 브리지도 이 모듈을 쓴다.
#
#   그리퍼 속도는 관절 속도 제한이 아니라 **목표값 이동**으로 맞춘다 (step_toward, GripperProfile).
#   rh_r1_joint 에 관절 속도 제한을 걸면 mimic 과 함께 점성 저항처럼 동작해 열리지 못한다 (2026-09-30 확인).
#   목표값이 일정 속도로 goal 까지 움직이므로, 물체에 닿으면 위치 오차가 커져 maxForce 로 누른다 (실물 Profile Velocity 와 같은 원리).
#
#   사용:
#     drive = RobotDrive(robot, cfg)   # 재생(play) 후
#     drive.apply()                    # gain·한계·armature 적용 후 읽어서 확인
#     drive.start()                    # physics 스텝 콜백 등록 (그리퍼 목표값 이동, gravity_ff)
#     drive.set_gripper_goal(1.1351)   # 목표값이 profile_velocity 로 이동
#     ...
#     drive.stop()                     # 콜백 해제 (stage 를 바꾸기 전에 호출)
#
# SimulationApp 을 만든 뒤에 import 할 것 (step_toward, GripperProfile 은 Isaac Sim 없이도 쓸 수 있음).
# =============================================================

import numpy as np

import test_scene as ts

REL_TOL = 1e-3


def step_toward(current, goal, max_step):
    """current 를 goal 쪽으로 최대 max_step 만큼 옮긴 값 (지나치지 않음). 가속 구간 없음 (등속)."""
    d = goal - current
    if abs(d) <= max_step:
        return goal
    return current + max_step * (1.0 if d > 0 else -1.0)


class GripperProfile:
    """그리퍼 목표값 이동 (실물 Profile Velocity 흉내). 매 physics 스텝 update(dt) → 이번 스텝의 drive 목표값.
    3단계 sim 그리퍼 브리지도 이 클래스를 쓴다 (goal 은 /gripper/command raw → rad 변환 후 넣음).

    goal 이 바뀌면(열기↔닫기) 이동을 **현재 실제 손가락 위치에서 새로 시작**한다 (실물 Dynamixel 도 현재 위치에서 새 궤적 시작).
    이전 목표값에서 이어 가면, 물체를 잡고 있다가 열 때 목표값이 물체 위치까지 되돌아오는 동안 손가락이 멈춰 있는 지연이 생긴다."""

    def __init__(self, velocity, setpoint=0.0):
        if velocity <= 0:
            raise ValueError(f"profile_velocity 는 양수여야 함: {velocity}")
        self.velocity = float(velocity)
        self.setpoint = float(setpoint)
        self.goal = float(setpoint)

    def set_goal(self, goal, current_position):
        """goal 이 바뀌면 목표값을 실제 위치(current_position)로 옮기고 거기서부터 이동. 같은 goal 반복이면 그대로."""
        goal = float(goal)
        if goal != self.goal:
            self.setpoint = float(current_position)
            self.goal = goal

    def jump(self, value):
        """목표값과 goal 을 즉시 value 로 (리셋용)."""
        self.setpoint = self.goal = float(value)

    def update(self, dt):
        self.setpoint = step_toward(self.setpoint, self.goal, self.velocity * dt)
        return self.setpoint


class RobotDrive:
    def __init__(self, robot, cfg):
        self.robot = robot
        self.cfg = cfg
        self.idx = ts.dof_index_map(robot)
        self.n = robot.num_dofs
        self.gravity_ff = bool(cfg["gravity_ff"])
        self.arm_i = np.array([self.idx[j] for j in ts.ARM_JOINTS])
        self.grip_i = self.idx[ts.GRIPPER_DRIVE]
        self.mimic_i = np.array([self.idx[j] for j in cfg["mimic"]])
        if sorted(cfg["mimic"]) != sorted(ts.GRIPPER_MIMIC):
            raise ValueError(f"설정의 mimic {cfg['mimic']} 이 {ts.GRIPPER_MIMIC} 과 다름")
        self._cb = None
        g = cfg["gripper"][ts.GRIPPER_DRIVE]
        if g.get("profile_velocity") is None:
            raise ValueError(f"gripper.{ts.GRIPPER_DRIVE}.profile_velocity 값이 비어 있음")
        self.gripper = GripperProfile(g["profile_velocity"])
        self.last_ff = np.zeros(6)
        self.ff_steps = 0
        self.cb_error = None

        # 설정값을 DOF 순서 벡터로 (null 이 있으면 에러)
        per_joint = dict(cfg["arm"])
        per_joint[ts.GRIPPER_DRIVE] = cfg["gripper"][ts.GRIPPER_DRIVE]
        keys = ("stiffness", "damping", "max_force", "max_velocity", "armature")
        self.vec = {k: np.zeros(self.n, dtype=np.float32) for k in keys}
        for name, p in per_joint.items():
            if name not in self.idx:
                raise KeyError(f"설정의 관절 {name} 이 DOF 에 없음")
            for k in keys:
                if p.get(k) is None:
                    raise ValueError(f"{name}.{k} 값이 비어 있음")
                self.vec[k][self.idx[name]] = float(p[k])
        # mimic: gain 0, armature 0. 힘·속도 한계는 USD 값 유지 (mimic 이 구동 조인트를 따라가도록)
        cur_eff = robot.get_dof_max_efforts().numpy()[0]
        cur_vel = robot.get_dof_max_velocities().numpy()[0]
        for i in self.mimic_i:
            self.vec["max_force"][i] = cur_eff[i]
            self.vec["max_velocity"][i] = cur_vel[i]
        self.max_force_arm = self.vec["max_force"][self.arm_i]

    def apply(self):
        """gain·힘 한계·속도 한계·armature 를 적용하고 읽어서 확인한다. 문제가 있으면 에러."""
        r = self.robot
        r.set_dof_gains(self.vec["stiffness"], self.vec["damping"])
        r.set_dof_max_efforts(self.vec["max_force"])
        r.set_dof_max_velocities(self.vec["max_velocity"])
        r.set_dof_armatures(self.vec["armature"])

        got = {
            "stiffness": r.get_dof_gains()[0].numpy()[0],
            "damping": r.get_dof_gains()[1].numpy()[0],
            "max_force": r.get_dof_max_efforts().numpy()[0],
            "max_velocity": r.get_dof_max_velocities().numpy()[0],
            "armature": r.get_dof_armatures().numpy()[0],
        }
        bad = []
        for k, want in self.vec.items():
            diff = np.abs(got[k] - want) > REL_TOL * np.maximum(1.0, np.abs(want))
            bad += [f"{k}[{list(self.idx)[i]}]={got[k][i]:.4g} != {want[i]:.4g}" for i in np.where(diff)[0]]
        for i in self.mimic_i:
            if got["stiffness"][i] != 0 or got["damping"][i] != 0:
                bad.append(f"mimic {list(self.idx)[i]} 에 gain 이 있음")
        if bad:
            raise RuntimeError("drive 설정 적용 확인 실패: " + "; ".join(bad))
        return got

    # ── physics 스텝 콜백: 그리퍼 목표값 이동 + (gravity_ff) 중력 보상 ──
    def start(self):
        if self._cb is not None:
            return
        self.gripper.jump(float(self.robot.get_dof_positions().numpy()[0][self.grip_i]))
        self._set_gripper_targets(self.gripper.setpoint)
        from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager

        self._cb = SimulationManager.register_callback(self._pre_step, event=SimulationEvent.PHYSICS_PRE_STEP)

    def stop(self):
        if self._cb is not None:
            from isaacsim.core.simulation_manager import SimulationManager

            SimulationManager.deregister_callback(self._cb)
            self._cb = None

    def _pre_step(self, dt, context):
        # physics 콜백 안의 예외는 묻힐 수 있으므로 저장해 두고 check() 에서 다시 올린다
        try:
            if self.gripper.setpoint != self.gripper.goal:
                self._set_gripper_targets(self.gripper.update(dt))
            if not self.gravity_ff:
                return
            # 로봇 링크 질량만으로 계산된 중력 토크 → 잡은 물체(별도 강체)의 무게는 포함되지 않음
            g = self.robot.get_dof_gravity_compensation_forces().numpy()[0]
            ff = np.clip(g[self.arm_i], -self.max_force_arm, self.max_force_arm)
            eff = np.zeros(self.n, dtype=np.float32)
            eff[self.arm_i] = ff
            self.robot.set_dof_efforts(eff)
            self.last_ff = ff
            self.ff_steps += 1
        except Exception as e:  # noqa: BLE001
            self.cb_error = e

    def check(self):
        if self.cb_error is not None:
            raise RuntimeError(f"drive 콜백 오류: {self.cb_error!r}")

    # ── 명령 ──
    def set_arm_targets(self, q6):
        self.robot.set_dof_position_targets(np.asarray(q6, dtype=np.float32), dof_indices=self.arm_i)

    def set_gripper_goal(self, rad):
        """그리퍼 goal. 목표값은 physics 스텝마다 profile_velocity 로 goal 까지 이동한다 (start() 필요)."""
        if self._cb is None:
            raise RuntimeError("start() 전에는 그리퍼 목표값 이동이 동작하지 않음")
        self.gripper.set_goal(rad, float(self.robot.get_dof_positions().numpy()[0][self.grip_i]))

    def _set_gripper_targets(self, rad):
        self.robot.set_dof_position_targets(float(rad), dof_indices=self.grip_i)

    def _set_gripper_now(self, rad):
        self.gripper.jump(rad)
        self._set_gripper_targets(float(rad))

    def reset_pose(self, q6, gripper=0.0):
        """순간이동으로 자세를 맞추고 목표도 같게 (시험 준비용)."""
        q = self.robot.get_dof_positions().numpy()[0].copy()
        q[self.arm_i] = q6
        for i in [self.grip_i, *self.mimic_i]:
            q[i] = gripper
        self.robot.set_dof_positions(q)
        self.robot.set_dof_velocities(np.zeros_like(q))
        self.set_arm_targets(q6)
        self._set_gripper_now(gripper)
