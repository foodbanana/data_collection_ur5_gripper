#!/usr/bin/env python3
# =============================================================
# sim_reset.py  (docs/PLAN.md 3-6)
#
# 에피소드 리셋 서비스 /sim/reset (std_srvs/Trigger). 서비스 콜백 안에서 sim 을 실제 시간 속도로 계속 돌리며 리셋하고 끝나면 응답한다
# (응답 message = JSON: 시드, 새 드론 호버 위치, 드론 처리 방법, 걸린 sim 시간). 순서와 값은 ros2_iface.yaml reset 머리말.
#   드론 (PX4): 날고 있으면 새 호버 위치로 이동 명령만 (실물과 같음).
#               kill 상태면 (잡은 뒤 모터 정지) 이륙 지점으로 순간이동 → **PX4 를 새로 띄움** → drone_scene.px4_takeoff 로 재이륙
#               (2026-10-05: 처음엔 순간이동 뒤 EKF 수렴을 기다렸으나, 드론이 팔 위로 떨어지면 EKF 가 가속도계 편향을 크게 잘못 잡아
#                "Preflight Fail: High Accelerometer Bias" 로 60 s 넘게 arm 거부. 실물도 추락한 드론은 재부팅한다)
#   드론 (기하): 날고 있으면 기준 위치만 바꿈, 꺼져 있으면 새 위치로 순간이동 후 다시 켬
#   팔: 홈 자세까지 최소 jerk 궤적을 ArmBridge 보간기로 (명령과 같은 경로, 30 Hz 계단 없음)
#   보호 정지: 리셋 시작에 해제. 리셋 동안 접촉력 기준만 끈다 (테이블을 누른 채 정지했으면 접촉이 남아 바로 다시 걸림, 2026-10-05).
#             위치 오차·관절 속도 기준은 켠 채로
# 실패하면 success=False + 이유를 응답하고 sim 은 계속 돈다. 다음 리셋이 성공할 때까지 팔·그리퍼 명령을 무시 (안전, 재시도 가능)
#   (처음엔 sim 을 멈췄으나 텔레옵 중 PX4 까지 다시 띄워야 해 바꿈, 2026-10-05)
# =============================================================

import json

import numpy as np


def min_jerk(q0, q1, T, t):
    u = min(max(t / T, 0.0), 1.0)
    s = 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5
    return q0 + s * (q1 - q0)


class EpisodeReset:
    def __init__(self, s, cfg, node, arm, grip, stop, step, seed=0, realtime=True):
        """step: sim 을 한 루프 돌리는 함수 (sim_ros2 의 실제 시간 맞춤 포함). arm/grip/stop: ArmBridge/GripperBridge/ProtectiveStop."""
        from isaacsim.core.simulation_manager import SimulationManager
        from std_srvs.srv import Trigger

        c = cfg["reset"]
        for k in ("service", "drone_offset", "arm_max_speed", "gripper_open_timeout", "drone_timeout"):
            if k not in c:
                raise KeyError(f"{cfg['path']}: 'reset.{k}' 항목이 없습니다")
        self.c, self.s, self.arm, self.grip, self.stop, self.step = c, s, arm, grip, stop, step
        self._sm = SimulationManager
        self.seed0, self.realtime = int(seed), bool(realtime)
        self.count = 0
        self.home = np.asarray(s.home, dtype=float)
        self.nominal = np.asarray(s.scene_cfg["drone_pos"], dtype=float)
        self.srv = node.create_service(Trigger, c["service"], self._on_reset)
        self.n_cb = 0
        self.last = None
        self.release_pending = False      # True 면 메인 루프가 다음 spin 뒤 명령 무시를 푼다 (리셋 중 쌓인 명령을 버리려고)
        self.failed = False               # 마지막 리셋 실패 → 다음 리셋 성공까지 명령 무시

    # ── 기다리기 ──
    def _run_until(self, cond, timeout, what):
        t0 = self.s.flight.t
        while not cond():
            self.step()
            if self.stop.stopped:
                raise RuntimeError(f"리셋 중 보호 정지: {self.stop.reason}")
            if self.s.flight.t - t0 > timeout:
                raise RuntimeError(f"리셋: {what} 가 {timeout:g} s 안에 안 됨")

    def _run_for(self, sec):
        t0 = self.s.flight.t
        self._run_until(lambda: self.s.flight.t - t0 >= sec, sec + 1.0, f"{sec:g} s 대기")

    # ── 단계 ──
    def _open_gripper(self):
        self.s.drive.set_gripper_goal(0.0)
        self.grip.goal_raw = 0.0
        self._run_until(lambda: self.grip.present_raw() < 20.0, self.c["gripper_open_timeout"], "그리퍼 열림")

    def _arm_home(self):
        v = self.s.robot._physics_articulation_view
        q0 = v.get_dof_positions().numpy()[0][self.arm.arm_i].astype(float)
        T = max(float(np.abs(self.home - q0).max()) / float(self.c["arm_max_speed"]) * 1.875, 1.0)
        dt_loop = self.arm.interp_T
        t0 = self.s.flight.t
        while self.s.flight.t - t0 < T:
            self.arm.follow(min_jerk(q0, self.home, T, self.s.flight.t - t0 + dt_loop))   # 다음 루프 시각의 궤적 점
            self.step()
            if self.stop.stopped:
                raise RuntimeError(f"리셋 중 보호 정지: {self.stop.reason}")
        self.arm.follow(self.home)
        self._run_for(0.5)
        q = v.get_dof_positions().numpy()[0][self.arm.arm_i]
        err = float(np.degrees(np.abs(q - self.home)).max())
        if err > 1.0:
            raise RuntimeError(f"리셋: 팔이 홈에 못 감 (최대 {err:.2f}°)")
        return T, err

    def _flying(self):
        f = self.s.flight
        if f.backend == "px4":
            return bool(f.px4.armed and f.cmd.landed == "in_air")
        return bool(f.armed)

    def _drone_before_arm(self, goal):
        """팔을 움직이기 전: kill 상태 드론은 그리퍼·팔에서 치워 둔다 (순간이동). 처리 방법 이름을 돌려준다."""
        f = self.s.flight
        if self._flying():
            return "goto"
        if f.backend == "px4":
            tc = self.s.scene_cfg["table"]
            spawn = np.array([*self.s.px4_cfg["takeoff_xy"], float(tc["top_z"]) + float(self.s.px4_cfg["spawn_above_table"])])
            f.teleport(spawn, yaw=0.0)
            f.restart_px4()                      # kill·낙하로 망가진 EKF 대신 새 PX4 (실물의 재부팅). 팔이 홈으로 가는 동안 부팅
            return "teleport_restart_takeoff"
        f.teleport(goal, yaw=0.0)
        return "teleport"

    def _drone_after_arm(self, how, goal):
        import drone_scene as ds

        f = self.s.flight
        f.ref.p0 = goal.copy()
        f.ref.params["p0"] = goal.tolist()
        if f.backend == "px4":
            if how == "goto":
                f.cmd.goto(goal)
                tol = float(self.s.px4_cfg["arrive_tol"])
                self._run_until(lambda: f.cmd.estimate_enu is not None and np.linalg.norm(f.cmd.estimate_enu - goal) < tol,
                                self.c["drone_timeout"], "드론 새 위치 도착")
                self._run_for(float(self.s.px4_cfg["settle_time"]))
            else:
                self.s.drone_pos, self.s.waypoints = goal.copy(), []
                f.cmd.cli_ready = False
                ds.px4_takeoff(self.s, realtime=self.realtime)
        else:
            if how == "teleport":
                f.arm(goal)
            self._run_for(2.0)
        return float(np.linalg.norm(f._body_pose()[0] - goal))

    # ── 서비스 ──
    def _on_reset(self, request, response):
        self.n_cb += 1
        t0 = self.s.flight.t
        seed = self.seed0 + self.count
        self.count += 1
        rng = np.random.default_rng(seed)
        goal = self.nominal + rng.uniform(-1.0, 1.0, 3) * float(self.c["drone_offset"])
        res = {"reset": self.count, "seed": seed, "drone_offset": float(self.c["drone_offset"]), "drone_goal": np.round(goal, 4).tolist(),
               "t_start": round(t0, 3)}
        self.arm.hold = self.grip.hold = True
        self.stop.ignore_contact = True          # 보호 정지가 테이블을 누른 자세에서 걸렸으면 접촉이 남아 있음 → 리셋 동안 접촉 기준만 끔
        try:
            self.stop.clear()
            self.arm.seg = None
            how = self._drone_before_arm(goal)  # kill 상태 드론은 먼저 치움 (빈손 그리퍼에 떨어져 손가락 사이에 끼면 그리퍼가 안 열림, 2026-10-05 check_ros2)
            self._open_gripper()
            T, err = self._arm_home()
            dist = self._drone_after_arm(how, goal)
            res.update(ok=True, drone=how, arm_home_s=round(T, 2), arm_err_deg=round(err, 3), drone_err_mm=round(dist * 1000, 1),
                       duration_s=round(self.s.flight.t - t0, 2), drone_pos=np.round(self.s.flight._body_pose()[0], 4).tolist())
            response.success = True
        except Exception as e:  # noqa: BLE001
            f = self.s.flight
            res.update(ok=False, error=str(e), duration_s=round(self.s.flight.t - t0, 2),
                       px4_messages=[m for _, _, m in f.cmd.messages[-5:]] if f.backend == "px4" else None)
            response.success = False
        finally:
            # 명령 무시는 아직 풀지 않는다: 이 콜백을 부른 spin 이 리셋 중 쌓인 명령을 마저 꺼내며 버린 뒤 메인 루프가 푼다 (release())
            self.release_pending = True
            self.stop.ignore_contact = False
        self.failed = not response.success
        response.message = json.dumps(res, ensure_ascii=False)
        self.last = res
        print(f"[sim_reset] {response.message}", flush=True)
        return response

    def release(self):
        """메인 루프: spin 뒤 부른다. 리셋이 성공으로 끝났으면 팔·그리퍼 명령 받기를 다시 시작 (실패면 계속 무시)."""
        if self.release_pending:
            if not self.failed:
                self.arm.hold = self.grip.hold = False
            self.release_pending = False

    def check(self):
        """리셋 실패는 응답·failed 로 알린다 (sim 은 계속). spin 이 모든 브리지에 부르는 자리만 맞춤."""

    def close(self):
        pass
