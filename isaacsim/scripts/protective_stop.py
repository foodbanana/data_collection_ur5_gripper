#!/usr/bin/env python3
# =============================================================
# protective_stop.py  (docs/PLAN.md 3-7)
#
# UR 보호 정지 흉내 (Cat 2: 감속 후 정지 유지). 물리 스텝마다 (PHYSICS_POST_STEP) 세 가지를 검사한다 (ros2_iface.yaml protective_stop):
#   - 링크 접촉력 [N]: 손가락이 아닌 링크는 PhysX net contact force (모든 물체: 테이블·바닥·받침대·드론·자기 링크),
#     손가락 링크는 environment_paths (테이블·바닥·받침대) 와의 접촉만 (드론을 쥐는 힘·손가락끼리 닿는 힘 제외)
#   - 관절 목표 − 측정 [°] (팔 6 관절): drive 목표(articulation tensor view) − 측정 관절각
#   - 관절 속도 [°/s] (팔 6 관절): 관절각 차분 (PhysX 보고 관절 속도는 실제보다 작음, docs/arm_command_interpolation.md 5장)
#   mode "on": 하나라도 기준을 넘으면 정지 → 팔 목표를 그 순간 관절각에 고정, ArmBridge 가 이후 /joint_command 를 무시, 해제는 clear() (리셋 서비스)
#   mode "measure": 정지하지 않고 최댓값만 기록 (정상 동작에서 기준의 여유 확인용)
#   /protective_stop (std_msgs/Bool) 은 1/30 s 배수 시각에 계속 발행 (그리퍼 토픽과 같은 스텝)
# 접촉 보고: 씬을 재생하기 전에 enable_contact_report(stage) 를 불러야 한다 (PhysxContactReportAPI, 임계 0)
# =============================================================

import numpy as np

MODES = ("on", "measure")


def robot_links(stage):
    """로봇 강체 링크 prim 경로 (grasp_demo._robot_links 와 같음)."""
    from pxr import Usd, UsdPhysics

    import test_scene as ts

    return [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(ts.ROBOT_PATH)) if p.HasAPI(UsdPhysics.RigidBodyAPI)]


def enable_contact_report(stage):
    """로봇 링크에 접촉 보고 켜기 (재생 전, 씬 레이어)."""
    from pxr import PhysxSchema

    for p in robot_links(stage):
        PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(p)).CreateThresholdAttr().Set(0.0)


class ProtectiveStop:
    def __init__(self, s, cfg, node, mode="on", publish_hz=30.0):
        from isaacsim.core.experimental.prims import RigidPrim
        from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
        from std_msgs.msg import Bool

        import test_scene as ts

        if mode not in MODES:
            raise ValueError(f"보호 정지 mode 는 {MODES} 중 하나: {mode}")
        c = cfg["protective_stop"]
        for k in ("topic", "contact_force_n", "tracking_error_deg", "joint_speed_dps", "finger_links", "environment_paths"):
            if k not in c:
                raise KeyError(f"{cfg['path']}: 'protective_stop.{k}' 항목이 없습니다")
        self.s, self.mode, self._sm, self._Bool = s, mode, SimulationManager, Bool
        self.lim = {"force": float(c["contact_force_n"]), "error": float(c["tracking_error_deg"]), "speed": float(c["joint_speed_dps"])}
        links = robot_links(s.stage)
        names = [p.split("/")[-1] for p in links]
        missing = [n for n in c["finger_links"] if n not in names]
        if missing:
            raise KeyError(f"protective_stop.finger_links 가 로봇 링크에 없음: {missing} (링크 {names})")
        env = list(c["environment_paths"])
        bad = [p for p in env if not s.stage.GetPrimAtPath(p).IsValid()]
        if bad:
            raise KeyError(f"protective_stop.environment_paths 에 없는 prim: {bad}")
        body = [i for i, n in enumerate(names) if n not in c["finger_links"]]
        fing = [i for i, n in enumerate(names) if n in c["finger_links"]]
        self.link_names = [names[i] for i in body] + [names[i] for i in fing]
        self.contacts = RigidPrim([links[i] for i in body], max_contact_count=64)    # 필터 없음 = 모든 물체와 순 접촉력
        # 손가락 view 는 직접 만든다: RigidPrim(contact_filter_paths=...) 는 필터 수가 prim 수와 같으면 prim 마다 필터를 하나씩
        #   짝지어 버린다 (손가락 4 개 · 환경 4 개 → 각 손가락이 환경 하나만 봄, 2026-10-05 테이블 충돌을 못 잡음)
        sim_view = SimulationManager._physics_sim_view__warp
        self.finger_view = sim_view.create_rigid_contact_view([links[i] for i in fing], filter_patterns=[env for _ in fing],
                                                              max_contact_data_count=256)
        if self.contacts._physics_rigid_contact_view is None or self.finger_view is None or not self.finger_view.check():
            raise RuntimeError("보호 정지 접촉 view 를 만들지 못함 (재생 전?)")
        self.arm_i = np.asarray(s.drive.arm_i)
        self.joints = list(ts.ARM_JOINTS)
        self.dt = ts.PHYSICS_DT
        self.hz = float(publish_hz)
        self.pub = node.create_publisher(Bool, c["topic"], max(10, int(round(float(cfg["publish_queue_sec"]) * self.hz))))
        self.stopped, self.reason, self.t_stop = False, None, None
        self.max = {"force": (0.0, None, None), "error": (0.0, None, None), "speed": (0.0, None, None)}   # (값, 링크·관절, sim t)
        v = s.robot._physics_articulation_view
        self._q_prev = v.get_dof_positions().numpy()[0][self.arm_i].astype(float)
        self.error = None
        self.n_cb = 0                          # spin() 이 세는 콜백 수 (구독 없음)
        self._last_k = None
        self._cb = SimulationManager.register_callback(self._post_step, event=SimulationEvent.PHYSICS_POST_STEP)

    def _post_step(self, dt, context):
        try:
            t = self._sm.get_simulation_time()
            v = self.s.robot._physics_articulation_view
            q = v.get_dof_positions().numpy()[0][self.arm_i].astype(float)
            qd = (q - self._q_prev) / self.dt
            self._q_prev = q
            qt = v.get_dof_position_targets().numpy()[0][self.arm_i]
            f_body = np.linalg.norm(self.contacts._physics_rigid_contact_view.get_net_contact_forces(self.dt).numpy().reshape(-1, 3), axis=1)
            f_fing = np.linalg.norm(self.finger_view.get_contact_force_matrix(self.dt).numpy(), axis=2).sum(axis=1)
            f = np.concatenate([f_body, f_fing])
            vals = {"force": (float(f.max()), self.link_names[int(f.argmax())]),
                    "error": (float(np.degrees(np.abs(qt - q)).max()), self.joints[int(np.abs(qt - q).argmax())]),
                    "speed": (float(np.degrees(np.abs(qd)).max()), self.joints[int(np.abs(qd).argmax())])}
            if not self.stopped:                # 정지 뒤 값(정지 순간의 큰 접촉 등)은 정상 동작 최댓값에 넣지 않음
                for k, (x, where) in vals.items():
                    if x > self.max[k][0]:
                        self.max[k] = (x, where, t)
                over = [f"{k} {x:.1f} ({where}) > {self.lim[k]:g}" for k, (x, where) in vals.items() if x > self.lim[k]]
                if over and self.mode == "on":
                    self.stopped, self.reason, self.t_stop = True, "; ".join(over), t
                    self.s.drive.set_arm_targets(q)          # 그 순간 관절각에 고정 (Cat 2: 정지 유지)
            k = round(t * self.hz)
            if abs(t - k / self.hz) <= 1e-6 and k != self._last_k:
                self._last_k = k
                m = self._Bool()
                m.data = bool(self.stopped)
                self.pub.publish(m)
        except Exception as e:  # noqa: BLE001
            self.error = self.error or e

    def resync(self, q):
        """팔을 순간이동한 뒤 (리셋): 관절 속도 차분 기준을 새 자세로 (안 맞추면 순간이동 거리 / dt 가 속도로 잡혀 정지가 걸림)."""
        self._q_prev = np.asarray(q, dtype=float).copy()

    def clear(self):
        """보호 정지 해제 (리셋 서비스). 팔 목표는 호출한 쪽이 정한다."""
        self.stopped, self.reason, self.t_stop = False, None, None

    def summary(self):
        lim = self.lim
        parts = [f"{k} 최대 {x:.1f} ({where}, t {t:.2f}) / 기준 {lim[k]:g}" if where else f"{k} 최대 0"
                 for k, (x, where, t) in self.max.items()]
        st = f"정지 t {self.t_stop:.2f}: {self.reason}" if self.stopped else "정지 아님"
        return f"보호 정지 [{self.mode}] {st} | " + ", ".join(parts)

    def check(self):
        if self.error is not None:
            e, self.error = self.error, None
            raise RuntimeError(f"보호 정지 검사 에러: {e}") from e

    def close(self):
        from isaacsim.core.simulation_manager import SimulationManager

        if self._cb is not None:
            SimulationManager.deregister_callback(self._cb)
            self._cb = None
