#!/usr/bin/env python3
# =============================================================
# sim_reset.py  (docs/PLAN.md 3-6)
#
# 에피소드 리셋 서비스 /sim/reset (std_srvs/Trigger). 서비스 콜백 안에서 sim 을 실제 시간 속도로 계속 돌리며 리셋하고 끝나면 응답한다
# (응답 message = JSON: 시드, 새 드론 호버 위치 (world 와 로봇 base 기준), 걸린 sim 시간, PX4 로그 폴더, "sim" = 에피소드 메타데이터용 고정 정보 (4-2)). 리셋 결과 = 에피소드 시작 상태 (팔 홈, 그리퍼 열림, 드론은 팔 위 호버).
#
# **항상 같은 경로 (순간이동)** (2026-10-05 사용자 결정):
#   1. 팔·그리퍼 명령 무시, 보호 정지 해제
#   2. 드론: PX4 — 날고 있으면 kill → 이륙 지점(테이블 위)으로 순간이동 → **PX4 새로 띄움** (매 에피소드 같은 깨끗한 상태: EKF·편향·적분값)
#            기하 — 모터 끄고 새 호버 위치로 순간이동
#   3. 팔·그리퍼를 홈·열림으로 순간이동 (RobotDrive.reset_pose: 위치·속도·목표를 한 번에. 리셋은 데이터에 안 들어가므로 실물처럼 움직일 필요 없음)
#   4. 드론: PX4 는 drone_scene.px4_takeoff 로 재이륙해 새 호버 위치로 (약 30 s), 기하는 그 자리에서 다시 켬
#   새 호버 위치 = 씬 drone_pos + 균등분포 ±drone_offset (x, y, z 각각), 시드 = --seed + 리셋 횟수
# 처음 (2026-10-05) 에는 날고 있는 드론은 이동 명령만 (약 7 s), 팔은 최소 jerk 궤적으로 돌렸으나 갈래마다 예외가 생김
#   (kill 된 드론이 손가락 사이에 끼어 그리퍼가 안 열림, 보호 정지 뒤 테이블 접촉이 남음, 팔 위로 떨어진 드론의 EKF 가속도계 편향이 망가져
#    60 s 넘게 arm 거부) → 한 경로로. PX4 재시작은 실물의 재부팅과 같음
# 실패하면 success=False + 이유를 응답하고 sim 은 계속 돈다. 다음 리셋이 성공할 때까지 팔·그리퍼 명령을 무시 (안전, 재시도 가능)
#
# 드론 모터 정지 서비스 /sim/drone_kill (std_srvs/Trigger, PLAN 4-1): 잡은 뒤 드론 모터를 끈다 (실물 시나리오: 잡는다 → 모터 정지 → 버틴다).
#   녹화 도구(record_toggle.py --sim)의 k 키가 부른다. 제어기와 무관하게 DroneFlight.release() (PX4 = kill, 기하 = ω 0).
#   응답 message = JSON (sim 시각). 이미 꺼져 있으면 success=False. 다시 켜는 것은 /sim/reset
# =============================================================

import json

import numpy as np


class EpisodeReset:
    def __init__(self, s, cfg, node, arm, grip, stop, step, seed=0, realtime=True):
        """step: sim 을 한 루프 돌리는 함수 (sim_ros2 의 실제 시간 맞춤 포함). arm/grip/stop: ArmBridge/GripperBridge/ProtectiveStop."""
        from isaacsim.core.simulation_manager import SimulationManager
        from std_srvs.srv import Trigger

        c = cfg["reset"]
        for k in ("service", "kill_service", "drone_offset", "kill_timeout"):
            if k not in c:
                raise KeyError(f"{cfg['path']}: 'reset.{k}' 항목이 없습니다")
        self.c, self.s, self.arm, self.grip, self.stop, self.step = c, s, arm, grip, stop, step
        self.seed0, self.realtime = int(seed), bool(realtime)
        self.count = 0
        self.home = np.asarray(s.home, dtype=float)
        self.nominal = np.asarray(s.drone_pos, dtype=float).copy()      # 리셋 기준 위치 = 시작할 때의 드론 위치 (--drone-pos 또는 씬 설정 drone_pos)
        # 로봇 base (최상위 prim = robot_mount, world 와 축이 같고 테이블 상판 높이만큼 위). 메타데이터 위치값은 base 기준 (CLAUDE.md)
        self.base = np.array([0.0, 0.0, float(s.scene_cfg["table"]["top_z"])])
        self.now = SimulationManager.get_simulation_time    # sim time (/clock 과 같은 시계). flight.t 는 기하 제어기를 다시 켤 때 0 으로 돌아간다
        self.srv = node.create_service(Trigger, c["service"], self._on_reset)
        self.kill_srv = node.create_service(Trigger, c["kill_service"], self._on_kill)
        self.n_cb = 0
        self.last = None
        self.sim_info = None              # 에피소드 메타데이터용 고정 정보 (sim_ros2.episode_info). 리셋 응답의 "sim" 에 넣는다 (PLAN 4-2)
        self.release_pending = False      # True 면 메인 루프가 다음 spin 뒤 명령 무시를 푼다 (리셋 중 쌓인 명령을 버리려고)
        self.failed = False               # 마지막 리셋 실패 → 다음 리셋 성공까지 명령 무시

    def _run_until(self, cond, timeout, what):
        t0 = self.s.flight.t
        while not cond():
            self.step()
            if self.s.flight.t - t0 > timeout:
                raise RuntimeError(f"리셋: {what} 가 {timeout:g} s 안에 안 됨")

    # ── 단계 ──
    def _drone_off_and_away(self, goal):
        """드론 모터를 끄고 (PX4: kill 확인) 치운다. PX4 는 이륙 지점으로 순간이동 + 재시작, 기하는 새 호버 위치로 순간이동."""
        f = self.s.flight
        if f.backend == "px4":
            if f.px4.armed:
                f.release()                                 # kill (공중 강제 disarm)
                self._run_until(lambda: not f.px4.armed, float(self.c["kill_timeout"]), "PX4 kill (disarm)")
            tc = self.s.scene_cfg["table"]
            spawn = np.array([*self.s.px4_cfg["takeoff_xy"], float(tc["top_z"]) + float(self.s.px4_cfg["spawn_above_table"])])
            f.teleport(spawn, yaw=0.0)
            f.restart_px4()                                 # 매 에피소드 새 PX4 (부팅은 다음 단계 동안)
        else:
            f.armed = False
            f.teleport(goal, yaw=0.0)

    def _arm_home(self):
        """팔·그리퍼를 홈·열림으로 순간이동하고 보간 기준·그리퍼 브리지 goal 을 맞춘다."""
        self.s.drive.reset_pose(self.home, gripper=0.0)
        self.arm.set_now(self.home)                         # 보간·속도 차분 기준도 새 자세로 (순간이동 거리가 속도로 잡히지 않게)
        self.stop.resync(self.home)
        self.grip.goal_raw = 0.0
        self.step()
        v = self.s.robot._physics_articulation_view
        q = v.get_dof_positions().numpy()[0][self.arm.arm_i]
        err = float(np.degrees(np.abs(q - self.home)).max())
        if err > 1.0 or self.grip.present_raw() > 20.0:
            raise RuntimeError(f"리셋: 팔·그리퍼 순간이동 확인 실패 (팔 최대 {err:.2f}°, 그리퍼 {self.grip.present_raw():.1f} raw)")
        return err

    def _drone_up(self, goal):
        import drone_scene as ds

        f = self.s.flight
        f.ref.p0 = goal.copy()
        f.ref.params["p0"] = goal.tolist()
        if f.backend == "px4":
            self.s.drone_pos, self.s.waypoints = goal.copy(), []
            f.cmd.cli_ready = False
            ds.px4_takeoff(self.s, realtime=self.realtime)
        else:
            f.arm(goal)
            t0 = f.t
            self._run_until(lambda: f.t - t0 >= 2.0, 3.0, "기하 제어기 드론 안정")
        return float(np.linalg.norm(f._body_pose()[0] - goal))

    # ── 서비스 ──
    def _on_reset(self, request, response):
        self.n_cb += 1
        t0 = self.now()
        seed = self.seed0 + self.count
        self.count += 1
        rng = np.random.default_rng(seed)
        goal = self.nominal + rng.uniform(-1.0, 1.0, 3) * float(self.c["drone_offset"])
        res = {"reset": self.count, "seed": seed, "drone_offset": float(self.c["drone_offset"]), "drone_goal": np.round(goal, 4).tolist(),
               "t_start": round(t0, 3)}
        self.arm.hold = self.grip.hold = True
        try:
            self.stop.clear()
            self.arm.seg = None
            self._drone_off_and_away(goal)
            err = self._arm_home()
            dist = self._drone_up(goal)
            if self.stop.stopped:
                raise RuntimeError(f"리셋 중 보호 정지: {self.stop.reason}")
            f = self.s.flight
            res.update(ok=True, arm_err_deg=round(err, 3), drone_err_mm=round(dist * 1000, 1), duration_s=round(self.now() - t0, 2),
                       drone_pos=np.round(f._body_pose()[0], 4).tolist(), t_end=round(self.now(), 3),
                       drone_goal_base=np.round(goal - self.base, 4).tolist(), drone_pos_base=np.round(f._body_pose()[0] - self.base, 4).tolist(),
                       px4_log=f.px4.launcher.run_dir if f.backend == "px4" else None, sim=self.sim_info)
            response.success = True
        except Exception as e:  # noqa: BLE001
            f = self.s.flight
            res.update(ok=False, error=str(e), duration_s=round(self.now() - t0, 2),
                       px4_messages=[m for _, _, m in f.cmd.messages[-5:]] if f.backend == "px4" else None)
            response.success = False
        finally:
            # 명령 무시는 아직 풀지 않는다: 이 콜백을 부른 spin 이 리셋 중 쌓인 명령을 마저 꺼내며 버린 뒤 메인 루프가 푼다 (release())
            self.release_pending = True
        self.failed = not response.success
        response.message = json.dumps(res, ensure_ascii=False)
        self.last = res
        print(f"[sim_reset] {response.message}", flush=True)
        return response

    def _drone_on(self):
        f = self.s.flight
        return bool(f.px4.armed) if f.backend == "px4" else bool(f.armed)

    def _on_kill(self, request, response):
        res = {"t": round(self.now(), 3), "backend": self.s.flight.backend}
        if self._drone_on():
            self.s.flight.release()
            response.success = True
        else:
            res["error"] = "드론 모터가 이미 꺼져 있음"
            response.success = False
        response.message = json.dumps(res, ensure_ascii=False)
        print(f"[sim_reset] drone_kill {response.message}", flush=True)
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
