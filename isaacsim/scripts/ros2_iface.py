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
#   ArmBridge(s, cfg, rclpy)      : /clock·/joint_states 발행 (물리 스텝마다, PHYSICS_POST_STEP), /joint_command 구독 → 팔 drive 목표 (3-2)
# 공식 방식 조사·결정: PLAN 3단계
# =============================================================

import os

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
    for k in ("ros_distro", "node_name", "loop_hz", "render", "arm", "cameras"):
        if k not in c:
            raise KeyError(f"{path}: '{k}' 항목이 없습니다")
    r = c["render"]
    if r.get("anti_aliasing") not in AA_MODES:
        raise ValueError(f"{path}: render.anti_aliasing 는 {AA_MODES} 중 하나: {r.get('anti_aliasing')}")
    if r.get("dlss_exec_mode") not in DLSS_MODES:
        raise ValueError(f"{path}: render.dlss_exec_mode 는 {DLSS_MODES} 중 하나: {r.get('dlss_exec_mode')}")
    for k in ("clock_topic", "joint_states_topic", "joint_command_topic"):
        if k not in c["arm"]:
            raise KeyError(f"{path}: 'arm.{k}' 항목이 없습니다")
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
                 (f"{info}.inputs:topicName", cc[name]["info_topic"]),
                 (f"{info}.inputs:frameId", cc[name]["frame_id"]), (f"{info}.inputs:useSystemTime", False)]
    graph, _, _, _ = og.Controller.edit(
        {"graph_path": CAMERA_GRAPH, "evaluator_name": "push",
         "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND},
        {keys.CREATE_NODES: nodes, keys.CONNECT: conns, keys.SET_VALUES: vals})
    og.Controller.evaluate_sync(graph)
    omni.kit.app.get_app().update()
    return CAMERA_GRAPH


class ArmBridge:
    """팔 ROS 2 인터페이스 (실물 UR 드라이버와 같은 토픽·형식, ros2_iface.yaml arm 머리말).
    - 물리 스텝마다 (PHYSICS_POST_STEP 콜백) /clock 과 /joint_states 를 같은 sim time stamp 로 발행
      (관절 값은 articulation tensor view 에서 직접: 실험용 API 는 호출마다 warp 배열을 만들어 느림, docs/sim_performance.md 6장)
    - /joint_command 는 spin() (매 루프) 에서 받아 바로 팔 drive 목표로 (zero-order hold). 잘못된 명령은 에러
    콜백 안 예외는 저장해 두고 check() 에서 다시 올린다."""

    def __init__(self, s, cfg, rclpy):
        import numpy as np
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
        self.node = rclpy.create_node(cfg["node_name"])
        self.pub_clock = self.node.create_publisher(Clock, a["clock_topic"], 10)
        self.pub_js = self.node.create_publisher(JointState, a["joint_states_topic"], 10)
        self.sub_cmd = self.node.create_subscription(JointState, a["joint_command_topic"], self._on_command, 10)
        self._rclpy = rclpy
        self.error = None
        self.n_js, self.n_cmd, self._n_cb = 0, 0, 0
        self.last_cmd = None              # (sim t, q [rad]) 마지막으로 적용한 명령
        self._cb = SimulationManager.register_callback(self._post_step, event=SimulationEvent.PHYSICS_POST_STEP)

    def _view(self):
        v = self.s.robot._physics_articulation_view
        if v is None:
            raise RuntimeError("articulation tensor view 없음 (재생 전?)")
        return v

    def _post_step(self, dt, context):
        try:
            t = self._sm.get_simulation_time()
            sec = int(t)
            nsec = int(round((t - sec) * 1e9))
            if nsec >= 1_000_000_000:
                sec, nsec = sec + 1, nsec - 1_000_000_000
            c = self._Clock()
            c.clock.sec, c.clock.nanosec = sec, nsec
            self.pub_clock.publish(c)
            v = self._view()
            q = v.get_dof_positions().numpy()[0][self.arm_i]
            qd = v.get_dof_velocities().numpy()[0][self.arm_i]
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
        self._n_cb += 1
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
            self.s.drive.set_arm_targets(q)
            self.n_cmd += 1
            self.last_cmd = (self._sm.get_simulation_time(), q)
        except Exception as e:  # noqa: BLE001
            self.error = self.error or e

    def spin(self):
        """매 루프 (update 뒤) 호출: 쌓인 명령을 모두 처리 (spin_once 는 콜백 하나씩, 더 없으면 멈춤)."""
        for _ in range(100):
            before = self._n_cb
            self._rclpy.spin_once(self.node, timeout_sec=0.0)
            if self._n_cb == before:
                break
        self.check()

    def check(self):
        if self.error is not None:
            e, self.error = self.error, None
            raise RuntimeError(f"ROS 2 팔 인터페이스 에러: {e}") from e

    def close(self):
        from isaacsim.core.simulation_manager import SimulationManager

        if self._cb is not None:
            SimulationManager.deregister_callback(self._cb)
            self._cb = None
        self.node.destroy_node()
