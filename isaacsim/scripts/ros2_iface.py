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
    for k in ("ros_distro", "render", "cameras"):
        if k not in c:
            raise KeyError(f"{path}: '{k}' 항목이 없습니다")
    r = c["render"]
    if r.get("anti_aliasing") not in AA_MODES:
        raise ValueError(f"{path}: render.anti_aliasing 는 {AA_MODES} 중 하나: {r.get('anti_aliasing')}")
    if r.get("dlss_exec_mode") not in DLSS_MODES:
        raise ValueError(f"{path}: render.dlss_exec_mode 는 {DLSS_MODES} 중 하나: {r.get('dlss_exec_mode')}")
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
