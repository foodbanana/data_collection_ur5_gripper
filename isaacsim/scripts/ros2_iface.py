#!/usr/bin/env python3
# =============================================================
# ros2_iface.py  (docs/PLAN.md 3단계)
#
# sim ROS 2 인터페이스 모듈. SimulationApp 을 만든 뒤 import 해서 쓴다.
#   app_config(cfg, headless)     : SimulationApp 설정 (렌더 anti_aliasing 명시). SimulationApp 을 만들기 전에 (이 모듈은 isaacsim 을 import 하지 않음)
#   set_dlss_mode(cfg) / check_render_settings(cfg) : DLSS 모드는 stage 를 만든 뒤 적용, 재생 후 다시 읽어 확인 (다르면 에러)
#   enable_ros2(cfg)              : isaacsim.ros2.bridge 를 켜고 rclpy 가 시스템 ROS 2 (cfg ros_distro) 인지 확인
#   set_camera_tick_rate(paths, r): 카메라 prim 에 OmniSensorAPI + omni:sensor:tickRate (6.0 부터 카메라 주기는 이것으로, multi-tick rendering)
#   add_camera_publishers(s, cfg) : OmniGraph Camera Helper (rgb) + Camera Info Helper. stamp = sim time (useSystemTime False)
#   ArmBridge(s, cfg, node)       : /clock·/joint_states 발행 (물리 스텝마다, PHYSICS_POST_STEP), /joint_command 구독 → 팔 drive 목표 (3-2)
#   GripperBridge(s, cfg, node)   : /gripper/command 구독 → GripperProfile, /gripper/joint_states·/gripper/target 30 Hz 같은 stamp (3-3)
#   spin(rclpy, node, bridges)    : 매 루프 쌓인 구독 메시지를 모두 처리하고 브리지 에러를 올린다
# 공식 방식 조사·결정: PLAN 3단계
# =============================================================

import os

import numpy as np
import yaml

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_ROS2_CONFIG = os.path.join(os.path.dirname(SCRIPT_DIR), "config", "ros2_iface.yaml")
CAMERA_GRAPH = "/ROS2Cameras"
CAMERA_NAMES = ("wrist", "third_view")
AA_MODES = {3: "DLSS", 4: "DLAA"}               # RTX 실시간 렌더러가 지원하는 것만 (0·1·2 는 렌더러가 3 으로 되돌림)
DLSS_MODES = {0: "Performance", 1: "Balanced", 2: "Quality", 3: "Auto"}
SETTING_AA, SETTING_DLSS = "/rtx/post/aa/op", "/rtx/post/dlss/execMode"


def load_ros2_config(path=None):
    path = path or DEFAULT_ROS2_CONFIG
    with open(path, encoding="utf-8") as f:
        c = yaml.safe_load(f)
    for k in ("ros_distro", "node_name", "loop_hz", "publish_queue_sec", "render", "arm", "gripper", "cameras"):
        if k not in c:
            raise KeyError(f"{path}: '{k}' 항목이 없습니다")
    r = c["render"]
    if r.get("anti_aliasing") not in AA_MODES:
        raise ValueError(f"{path}: render.anti_aliasing 는 {AA_MODES} 중 하나: {r.get('anti_aliasing')}")
    if r.get("dlss_exec_mode") not in DLSS_MODES:
        raise ValueError(f"{path}: render.dlss_exec_mode 는 {DLSS_MODES} 중 하나: {r.get('dlss_exec_mode')}")
    for k in ("clock_topic", "joint_states_topic", "joint_command_topic", "command_interp_time"):
        if k not in c["arm"]:
            raise KeyError(f"{path}: 'arm.{k}' 항목이 없습니다")
    for k in ("command_topic", "joint_states_topic", "target_topic", "joint_name", "publish_hz"):
        if k not in c["gripper"]:
            raise KeyError(f"{path}: 'gripper.{k}' 항목이 없습니다")
    if not float(c["arm"]["command_interp_time"]) > 0:
        raise ValueError(f"{path}: arm.command_interp_time 는 양수 [s]: {c['arm']['command_interp_time']}")
    if "tick_rate" not in c["cameras"]:
        raise KeyError(f"{path}: 'cameras.tick_rate' 항목이 없습니다")
    for name in CAMERA_NAMES:
        for k in ("topic", "info_topic", "frame_id"):
            if k not in c["cameras"].get(name, {}):
                raise KeyError(f"{path}: 'cameras.{name}.{k}' 항목이 없습니다")
    c["path"] = path
    return c


def app_config(cfg, headless):
    """SimulationApp 설정: 렌더 안티에일리어싱을 Isaac Sim 기본값에 맡기지 않고 설정 파일 값으로."""
    return {"headless": bool(headless), "anti_aliasing": int(cfg["render"]["anti_aliasing"])}


def set_dlss_mode(cfg):
    """DLSS 모드 적용. 새 stage 를 만들면 기본값(apps/isaacsim.exp.base.kit)으로 돌아가므로 stage 를 만든 뒤에 부른다."""
    import carb

    carb.settings.get_settings().set(SETTING_DLSS, int(cfg["render"]["dlss_exec_mode"]))


def check_render_settings(cfg):
    """실제 렌더 설정을 읽어 설정 파일과 다르면 에러 (재생 후, 카메라 렌더가 돈 뒤 부른다). 메타데이터용 dict 를 돌려준다."""
    import carb

    st = carb.settings.get_settings()
    got = {"anti_aliasing": st.get(SETTING_AA), "dlss_exec_mode": st.get(SETTING_DLSS)}
    want = {k: int(cfg["render"][k]) for k in got}
    if got != want:
        raise RuntimeError(f"렌더 설정이 설정 파일과 다름: 실제 {got}, 설정 {want} ({cfg['path']})")
    return {"anti_aliasing": AA_MODES[got["anti_aliasing"]], "dlss_exec_mode": DLSS_MODES[got["dlss_exec_mode"]]}


def enable_ros2(cfg):
    """bridge 를 켜고 rclpy 를 import 해 돌려준다. 시스템 ROS 2 를 source 하지 않았거나 distro 가 다르면 에러
    (bridge 는 source 가 없으면 내장 Jazzy 라이브러리로 뜨는데, 그러면 외부 터미널과 환경이 달라질 수 있어 막는다)."""
    import isaacsim.core.experimental.utils.app as app_utils
    import omni.kit.app

    distro = os.environ.get("ROS_DISTRO")
    if distro != cfg["ros_distro"]:
        raise RuntimeError(f"ROS_DISTRO={distro} (기대 {cfg['ros_distro']}). python.sh 실행 전에 "
                           f"source /opt/ros/{cfg['ros_distro']}/setup.bash")
    app_utils.enable_extension("isaacsim.ros2.bridge")
    omni.kit.app.get_app().update()
    import rclpy

    expect = f"/opt/ros/{cfg['ros_distro']}/"
    if not rclpy.__file__.startswith(expect):
        raise RuntimeError(f"rclpy 가 시스템 ROS 2 가 아님: {rclpy.__file__} (기대 {expect}...)")
    return rclpy


def set_camera_tick_rate(paths, rate):
    """카메라 prim 에 OmniSensorAPI 를 적용하고 omni:sensor:tickRate [Hz] (0 = 매 프레임 렌더). 자세(xform)는 건드리지 않는다."""
    from isaacsim.sensors.experimental.rtx import RtxCamera

    for p in paths:
        RtxCamera(p, tick_rate=float(rate), reset_xform_op_properties=False)


def pub_depth(cfg, hz):
    """발행 큐 길이 [메시지 수] = publish_queue_sec 분량. 받는 쪽 (녹화기 등) 이 잠깐 못 받는 동안 발행 큐 (keep_last) 가 넘치면
    그 메시지는 RELIABLE 이어도 버려진다 (새 노드가 뜨는 순간·녹화기가 뜬 직후 0.4~0.7 s, PLAN 4단계 A1)."""
    return max(10, int(round(float(cfg["publish_queue_sec"]) * float(hz))))


def add_camera_publishers(s, cfg):
    """씬 s 의 손목·third view 카메라를 Camera Helper (rgb) + Camera Info Helper 로 발행 (재생 중에 호출).
    해상도는 카메라 설정 yaml (1-7, 실물 640x480). 그래프 경로를 돌려준다."""
    import omni.graph.core as og
    import omni.kit.app
    import usdrt.Sdf

    cc = cfg["cameras"]
    keys = og.Controller.Keys
    nodes, conns, vals = [("OnTick", "omni.graph.action.OnTick")], [], []
    for name, path, cam_cfg in (("wrist", s.wrist_cam, s.cam_cfg), ("third_view", s.third_cam, s.tv_cfg)):
        w, h = (int(x) for x in cam_cfg["resolution"])
        rp, rgb, info = f"{name}_rp", f"{name}_rgb", f"{name}_info"
        nodes += [(rp, "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                  (rgb, "isaacsim.ros2.bridge.ROS2CameraHelper"),
                  (info, "isaacsim.ros2.bridge.ROS2CameraInfoHelper")]
        conns += [("OnTick.outputs:tick", f"{rp}.inputs:execIn"),
                  (f"{rp}.outputs:execOut", f"{rgb}.inputs:execIn"),
                  (f"{rp}.outputs:execOut", f"{info}.inputs:execIn"),
                  (f"{rp}.outputs:renderProductPath", f"{rgb}.inputs:renderProductPath"),
                  (f"{rp}.outputs:renderProductPath", f"{info}.inputs:renderProductPath")]
        vals += [(f"{rp}.inputs:cameraPrim", [usdrt.Sdf.Path(path)]),
                 (f"{rp}.inputs:width", w), (f"{rp}.inputs:height", h),
                 (f"{rgb}.inputs:type", "rgb"), (f"{rgb}.inputs:topicName", cc[name]["topic"]),
                 (f"{rgb}.inputs:frameId", cc[name]["frame_id"]), (f"{rgb}.inputs:useSystemTime", False),
                 (f"{rgb}.inputs:queueSize", pub_depth(cfg, cc["tick_rate"])),
                 (f"{info}.inputs:topicName", cc[name]["info_topic"]),
                 (f"{info}.inputs:frameId", cc[name]["frame_id"]), (f"{info}.inputs:useSystemTime", False),
                 (f"{info}.inputs:queueSize", pub_depth(cfg, cc["tick_rate"]))]
    graph, _, _, _ = og.Controller.edit(
        {"graph_path": CAMERA_GRAPH, "evaluator_name": "push",
         "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND},
        {keys.CREATE_NODES: nodes, keys.CONNECT: conns, keys.SET_VALUES: vals})
    og.Controller.evaluate_sync(graph)
    omni.kit.app.get_app().update()
    return CAMERA_GRAPH


def sim_stamp(t):
    """sim time [s] → (sec, nanosec)."""
    sec = int(t)
    nsec = int(round((t - sec) * 1e9))
    if nsec >= 1_000_000_000:
        sec, nsec = sec + 1, nsec - 1_000_000_000
    return sec, nsec


def spin(rclpy, node, bridges):
    """매 루프 (update 뒤): 쌓인 구독 메시지를 모두 처리한 뒤 브리지 에러를 올린다.
    rclpy spin_once 는 메시지가 쌓여 있어도 콜백 1 번 → 빈 호출 1 번을 번갈아 한다 (1, 0, 1, 0, …, 2026-10-05 확인).
    빈 호출 한 번에 멈추면 루프마다 1 개만 처리해 60 Hz 명령의 절반이 큐에서 밀려 버려진다 → 빈 호출이 두 번 연속이면 멈춤."""
    empty = 0
    for _ in range(400):
        before = sum(b.n_cb for b in bridges)
        rclpy.spin_once(node, timeout_sec=0.0)
        empty = empty + 1 if sum(b.n_cb for b in bridges) == before else 0
        if empty >= 2:
            break
    for b in bridges:
        b.check()


class ArmBridge:
    """팔 ROS 2 인터페이스 (실물 UR 드라이버와 같은 토픽·형식, ros2_iface.yaml arm 머리말).
    - 물리 스텝마다 (PHYSICS_POST_STEP 콜백) /clock 과 /joint_states 를 같은 sim time stamp 로 발행
      (관절 값은 articulation tensor view 에서 직접: 실험용 API 는 호출마다 warp 배열을 만들어 느림, docs/sim_performance.md 6장)
      velocity 는 **관절각 차분** (q − q_이전) / dt: PhysX 가 보고하는 관절 속도는 실제 움직임보다 작다
      (2026-10-05 측정: 실제 대비 shoulder_pan 0.85, wrist_1 0.61, wrist_3 0.42 배, 멈춘 관절도 1~3 °/s, docs/arm_command_interpolation.md 5장)
    - /joint_command 는 spin() (매 루프) 에서 받아 새 목표로. 물리 스텝마다 (PHYSICS_PRE_STEP) 지금 목표에서 새 목표까지
      command_interp_time 동안 직선 보간 (docs/arm_command_interpolation.md). 잘못된 명령은 에러
    콜백 안 예외는 저장해 두고 check() 에서 다시 올린다."""

    def __init__(self, s, cfg, node):
        from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import JointState

        import test_scene as ts

        self.s, self.cfg, self._np = s, cfg, np
        self._sm = SimulationManager
        self._Clock, self._JointState = Clock, JointState
        self.names = list(ts.ARM_JOINTS)
        self.arm_i = np.asarray(s.drive.arm_i)
        lo, hi = (x.numpy()[0][self.arm_i] for x in s.robot.get_dof_limits())
        self.lo, self.hi = lo.astype(float), hi.astype(float)
        a = cfg["arm"]
        self.node = node
        d = pub_depth(cfg, 1.0 / ts.PHYSICS_DT)
        self.pub_clock = self.node.create_publisher(Clock, a["clock_topic"], d)
        self.pub_js = self.node.create_publisher(JointState, a["joint_states_topic"], d)
        self.sub_cmd = self.node.create_subscription(JointState, a["joint_command_topic"], self._on_command, 10)
        self.error = None
        self.n_js, self.n_cmd, self.n_cb, self.n_ignored = 0, 0, 0, 0
        self.stop = None                  # protective_stop.ProtectiveStop (정지 중이면 명령 무시)
        self.interp_T = float(a["command_interp_time"])
        self.cmd_per_loop = []            # 루프(spin)마다 받은 /joint_command 수 (명령이 고르게 오는지)
        self.seg = None                   # 보간 구간 (q_start, q_goal, t_start). None = 보간 안 함 (목표 그대로)
        self.hold = False                 # True 면 /joint_command 무시 (리셋 중. 리셋은 팔을 순간이동한 뒤 set_now())
        self._q_now = None                # 지난 물리 스텝에 넣은 관절 목표
        self._cb_pre = SimulationManager.register_callback(self._pre_step, event=SimulationEvent.PHYSICS_PRE_STEP)
        self._q_prev = self._view().get_dof_positions().numpy()[0][self.arm_i].astype(float)   # 속도 = 관절각 차분
        self.last_cmd = None              # (sim t, q [rad]) 마지막으로 적용한 명령
        self._cb = SimulationManager.register_callback(self._post_step, event=SimulationEvent.PHYSICS_POST_STEP)

    def _view(self):
        v = self.s.robot._physics_articulation_view
        if v is None:
            raise RuntimeError("articulation tensor view 없음 (재생 전?)")
        return v

    def _target_at(self, t):
        q0, q1, t0 = self.seg
        u = min(max((t - t0) / self.interp_T, 0.0), 1.0)
        return q0 + u * (q1 - q0)

    def _pre_step(self, dt, context):
        """물리 스텝 직전: 보간 구간의 이번 스텝 목표를 drive 에 (이번 스텝이 끝나는 시각 기준)."""
        try:
            if self.seg is None:
                return
            if self.stop is not None and self.stop.stopped:      # 보호 정지: 보간 멈춤 (정지가 관절각에 고정해 둠)
                self.seg = None
                return
            q = self._target_at(self._sm.get_simulation_time() + float(dt))
            if self._q_now is not None and np.array_equal(q, self._q_now):
                return
            self.s.drive.set_arm_targets(q)
            self._q_now = q
            if q is self.seg[1] or np.array_equal(q, self.seg[1]):
                self.seg = None                                   # 도착: 더 쓸 필요 없음
        except Exception as e:  # noqa: BLE001
            self.error = self.error or e

    def _post_step(self, dt, context):
        try:
            sec, nsec = sim_stamp(self._sm.get_simulation_time())
            c = self._Clock()
            c.clock.sec, c.clock.nanosec = sec, nsec
            self.pub_clock.publish(c)
            v = self._view()
            q = v.get_dof_positions().numpy()[0][self.arm_i].astype(float)
            qd = (q - self._q_prev) / float(dt)
            self._q_prev = q
            tau = v.get_dof_projected_joint_forces().numpy()[0][self.arm_i]
            m = self._JointState()
            m.header.stamp.sec, m.header.stamp.nanosec = sec, nsec
            m.name = self.names
            m.position, m.velocity, m.effort = q.tolist(), qd.tolist(), tau.tolist()
            self.pub_js.publish(m)
            self.n_js += 1
        except Exception as e:  # noqa: BLE001
            self.error = self.error or e

    def _on_command(self, msg):
        np = self._np
        self.n_cb += 1
        try:
            names, pos = list(msg.name), list(msg.position)
            if len(names) != len(pos):
                raise ValueError(f"/joint_command name {len(names)} 개, position {len(pos)} 개")
            if sorted(names) != sorted(self.names) or len(set(names)) != len(names):
                raise ValueError(f"/joint_command 관절 이름이 팔 6 관절과 다름: {names} (기대 {self.names})")
            by = dict(zip(names, pos))
            q = np.array([by[n] for n in self.names], dtype=float)
            if not np.all(np.isfinite(q)):
                raise ValueError(f"/joint_command 값이 유한하지 않음: {q.tolist()}")
            out = [f"{n} {v:+.4f} (한계 {a:+.4f} ~ {b:+.4f})" for n, v, a, b in zip(self.names, q, self.lo, self.hi) if not a <= v <= b]
            if out:
                raise ValueError("/joint_command 가 관절 한계 밖: " + ", ".join(out))
            if (self.stop is not None and self.stop.stopped) or self.hold:
                self.n_ignored += 1           # 보호 정지·리셋 중: 검사는 하되 적용하지 않음
                return
            t = self._sm.get_simulation_time()
            if self.seg is not None:
                q_start = self._target_at(t)  # 지금 보간 중인 목표에서 새로 시작
            elif self._q_now is not None:
                q_start = self._q_now
            else:
                v = self._view()
                q_start = v.get_dof_position_targets().numpy()[0][self.arm_i].astype(float)
            self.seg = (np.asarray(q_start, dtype=float), q, t)
            self.n_cmd += 1
            self.last_cmd = (self._sm.get_simulation_time(), q)
        except Exception as e:  # noqa: BLE001
            self.error = self.error or e

    def check(self):
        if self.error is not None:
            e, self.error = self.error, None
            raise RuntimeError(f"ROS 2 팔 인터페이스 에러: {e}") from e

    def close(self):
        from isaacsim.core.simulation_manager import SimulationManager

        if self._cb is not None:
            SimulationManager.deregister_callback(self._cb)
            self._cb = None
        if self._cb_pre is not None:
            SimulationManager.deregister_callback(self._cb_pre)
            self._cb_pre = None

    def set_now(self, q):
        """밖에서 (리셋의 순간이동) 팔 자세·목표를 바꿨을 때: 보간 기준과 속도 차분 기준을 새 자세로
        (안 맞추면 다음 스텝 /joint_states velocity 에 순간이동 거리 / dt 가 나감)."""
        self.seg = None
        self._q_now = np.asarray(q, dtype=float)
        self._q_prev = np.asarray(q, dtype=float).copy()


class GripperBridge:
    """sim 그리퍼 브리지 (실물 rh_gripper_node 의 sim 버전, ros2_iface.yaml gripper 머리말).
    - /gripper/command (Float64 raw) → goal latch → RobotDrive.set_gripper_goal (GripperProfile 이 물리 스텝마다 목표값 이동)
    - 물리 스텝 뒤 sim time 이 1/publish_hz 배수일 때 /gripper/joint_states (present raw) 와 /gripper/target (goal raw) 을 같은 stamp 로"""

    def __init__(self, s, cfg, node):
        from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64

        import test_scene as ts

        g = cfg["gripper"]
        self.s, self._sm, self._JointState = s, SimulationManager, JointState
        self.name = str(g["joint_name"])
        self.hz = float(g["publish_hz"])
        self.upper, self.raw_max = ts.GRIPPER_UPPER, ts.GRIPPER_RAW_MAX
        self.grip_i = int(s.drive.grip_i)
        self.pub_present = node.create_publisher(JointState, g["joint_states_topic"], pub_depth(cfg, self.hz))
        self.pub_target = node.create_publisher(JointState, g["target_topic"], pub_depth(cfg, self.hz))
        self.sub_cmd = node.create_subscription(Float64, g["command_topic"], self._on_command, 10)
        if abs(s.drive.gripper.goal) > 1e-9:
            raise RuntimeError(f"그리퍼가 열림(0)으로 시작하지 않음: goal {s.drive.gripper.goal} rad")
        self.goal_raw = 0.0                   # 실물 노드: 시작 시 Goal Position 0 (열림)
        self.error = None
        self.n_pub, self.n_cmd, self.n_cb, self.n_ignored = 0, 0, 0, 0
        self.hold = False                     # True 면 명령 무시 (리셋 중)
        self._last_k = None
        self._cb = SimulationManager.register_callback(self._post_step, event=SimulationEvent.PHYSICS_POST_STEP)

    def present_raw(self):
        v = self.s.robot._physics_articulation_view
        if v is None:
            raise RuntimeError("articulation tensor view 없음 (재생 전?)")
        q = float(v.get_dof_positions().numpy()[0][self.grip_i])
        return min(max(q * self.raw_max / self.upper, 0.0), self.raw_max)      # 실물 노드처럼 0~1150

    def _post_step(self, dt, context):
        try:
            t = self._sm.get_simulation_time()
            k = round(t * self.hz)
            if abs(t - k / self.hz) > 1e-6 or k == self._last_k:      # 1/publish_hz 배수 시각에만 (카메라 stamp 와 같은 스텝)
                return
            self._last_k = k
            sec, nsec = sim_stamp(t)
            for pub, val in ((self.pub_present, self.present_raw()), (self.pub_target, self.goal_raw)):
                m = self._JointState()
                m.header.stamp.sec, m.header.stamp.nanosec = sec, nsec
                m.name = [self.name]
                m.position = [float(val)]
                pub.publish(m)
            self.n_pub += 1
        except Exception as e:  # noqa: BLE001
            self.error = self.error or e

    def _on_command(self, msg):
        import math

        self.n_cb += 1
        try:
            v = float(msg.data)
            if not math.isfinite(v) or not 0.0 <= v <= self.raw_max:
                raise ValueError(f"/gripper/command 는 0 ~ {self.raw_max:g} (raw): {v}")
            if self.hold:
                self.n_ignored += 1
                return
            self.s.drive.set_gripper_goal(v * self.upper / self.raw_max)
            self.goal_raw = v
            self.n_cmd += 1
        except Exception as e:  # noqa: BLE001
            self.error = self.error or e

    def check(self):
        if self.error is not None:
            e, self.error = self.error, None
            raise RuntimeError(f"ROS 2 그리퍼 브리지 에러: {e}") from e

    def close(self):
        from isaacsim.core.simulation_manager import SimulationManager

        if self._cb is not None:
            SimulationManager.deregister_callback(self._cb)
            self._cb = None
