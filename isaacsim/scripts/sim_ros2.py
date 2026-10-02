#!/usr/bin/env python3
# =============================================================
# sim_ros2.py  (docs/PLAN.md 3단계)
#
# sim ROS 2 인터페이스 실행: 드론 파지 씬(drone_scene) + 실물과 같은 ROS 2 토픽. 실제 시간 속도로 돈다 (텔레옵).
#   발행: /clock, /joint_states (팔 6 관절, 물리 스텝마다 120 Hz), /cam/{wrist,third_view}/color/image_raw·camera_info (30 Hz),
#         /gripper/joint_states (present raw)·/gripper/target (goal raw) (30 Hz, 같은 stamp)
#   구독: /joint_command (팔 6 관절 목표 [rad]), /gripper/command (raw 0 / 1150)
#   (/base/imu·리셋·보호 정지는 3-5 ~ 3-7 에서 추가)
#   토픽·주기·렌더 설정: isaacsim/config/ros2_iface.yaml. stamp 는 모두 sim time
#
# 실행 (ROS 2 를 source 한 터미널):
#   source /opt/ros/jazzy/setup.bash
#   ~/isaacsim/python.sh isaacsim/scripts/sim_ros2.py                    # GUI (메인 뷰포트 하나, 프로펠러 회전 켬). 창을 닫으면 종료
#   ~/isaacsim/python.sh isaacsim/scripts/sim_ros2.py --headless --duration 60
#   인자: --flight px4|geometric --position-source --drone-pos x y z --mode static|hover --seed --init-pose q1..q6
#         --prop-spin on|off (기본 GUI on, headless off) --duration [sim s] (headless 는 없으면 Ctrl+C 까지)
#   손목·third view 화면은 ROS 토픽으로 본다 (예: ros2 run rqt_image_view rqt_image_view). 카메라 뷰포트 창은 렌더 비용이 커서 띄우지 않음
# =============================================================

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_scene as ts  # noqa: E402

import numpy as np  # noqa: E402

import ros2_iface as ri  # noqa: E402

STATUS_SEC = 10.0      # 상태 출력 주기 (sim s)


def parse():
    p = argparse.ArgumentParser(description="sim ROS 2 인터페이스 (PLAN 3단계)")
    p.add_argument("--headless", action="store_true")
    p.add_argument("--duration", type=float, default=None, help="[sim s] 이만큼 돌고 끝냄 (없으면 GUI 창을 닫거나 Ctrl+C 까지)")
    p.add_argument("--flight", default=None, choices=("px4", "geometric"), help="드론 제어기 (기본 드론 설정)")
    p.add_argument("--position-source", default=None, choices=("mocap", "flow", "gps"), help="px4 위치 정보 (기본 px4_sitl.yaml)")
    p.add_argument("--drone-pos", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"), help="드론 위치 [m] world")
    p.add_argument("--mode", default="static", choices=("static", "hover"), help="드론 비행 모드")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--init-pose", type=float, nargs=6, default=None, metavar="Q", help="팔 시작 자세 [rad] (기본 home_pose)")
    p.add_argument("--prop-spin", choices=("on", "off"), default=None, help="프로펠러 회전 표시 (기본 GUI on, headless off)")
    p.add_argument("--ros2-config", default=None)
    p.add_argument("--report-dir", default=ts.REPORT_DIR)
    args, _ = p.parse_known_args()
    if args.prop_spin is None:
        args.prop_spin = "off" if args.headless else "on"
    return args


def main():
    args = parse()
    cfg = ri.load_ros2_config(args.ros2_config)
    loop_hz = float(cfg["loop_hz"])
    n_sub = (1.0 / ts.PHYSICS_DT) / loop_hz
    if abs(n_sub - round(n_sub)) > 1e-9:
        raise ValueError(f"loop_hz {loop_hz} 는 물리 {1.0 / ts.PHYSICS_DT:g} Hz 의 약수여야 함")
    from isaacsim import SimulationApp

    app = SimulationApp(ri.app_config(cfg, args.headless))
    ok, rclpy, node, bridges = False, None, None, []
    try:
        import drone_scene as ds
        from isaacsim.core.rendering_manager import RenderingManager, ViewportManager

        rclpy = ri.enable_ros2(cfg)
        rclpy.init()

        def setup(s):
            RenderingManager.set_dt(1.0 / loop_hz)          # 앱 루프 = 카메라 주기 (물리는 120 Hz 그대로)
            ri.set_dlss_mode(cfg)                           # stage 를 만든 뒤
            ri.set_camera_tick_rate([s.wrist_cam, s.third_cam], cfg["cameras"]["tick_rate"])

        s = ds.build_scene(drone_pos=args.drone_pos, mode=args.mode, seed=args.seed, prop_spin=args.prop_spin == "on",
                           init_pose=args.init_pose, extra_setup=setup, flight=args.flight,
                           position_source=args.position_source, report_dir=args.report_dir, realtime=not args.headless)
        ri.add_camera_publishers(s, cfg)
        node = rclpy.create_node(cfg["node_name"])
        arm = ri.ArmBridge(s, cfg, node)
        grip = ri.GripperBridge(s, cfg, node)
        bridges = [arm, grip]
        ds.run(s, 0.5)                                       # 카메라 렌더가 한 번 돈 뒤 렌더 설정 확인
        render = ri.check_render_settings(cfg)
        if not args.headless:
            ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[1.9, -1.6, 1.7], target=[0.4, 0.0, 1.1])
        print(f"[sim_ros2] 준비: 드론 {s.backend} ({s.flight.ref.mode}), 렌더 {render}, loop {loop_hz:g} Hz, "
              f"발행 {cfg['arm']['clock_topic']} {cfg['arm']['joint_states_topic']} {cfg['gripper']['joint_states_topic']} {cfg['gripper']['target_topic']}, "
              f"구독 {cfg['arm']['joint_command_topic']} {cfg['gripper']['command_topic']}, "
              f"카메라 {[cfg['cameras'][n]['topic'] for n in ri.CAMERA_NAMES]}", flush=True)

        t0, wall0 = s.flight.t, time.monotonic()
        t_status, w_status, n_status = t0, wall0, 0
        while ds._running():
            ds.step(s)
            ri.spin(rclpy, node, bridges)
            ahead = (s.flight.t - t0) - (time.monotonic() - wall0)
            if ahead > 0:
                time.sleep(ahead)                           # 실제 시간 속도 (텔레옵)
            if s.flight.t - t_status >= STATUS_SEC:
                w = time.monotonic()
                cmd = "없음" if arm.last_cmd is None else f"{np.round(arm.last_cmd[1], 3).tolist()} (t {arm.last_cmd[0]:.2f})"
                print(f"[sim_ros2] t {s.flight.t:7.1f} s, 최근 {STATUS_SEC:g} s RTF {(s.flight.t - t_status) / (w - w_status):.3f}, "
                      f"/joint_states {arm.n_js - n_status} 개, /joint_command 누적 {arm.n_cmd} 개, 마지막 명령 {cmd}, "
                      f"그리퍼 present {grip.present_raw():.0f} target {grip.goal_raw:.0f} (명령 누적 {grip.n_cmd})", flush=True)
                t_status, w_status, n_status = s.flight.t, w, arm.n_js
            if args.duration is not None and s.flight.t - t0 >= args.duration:
                break
        ok = True
    except KeyboardInterrupt:
        print("[sim_ros2] Ctrl+C 로 종료", flush=True)
        ok = True
    except Exception as e:  # noqa: BLE001
        import traceback
        print(f"[ERROR] {type(e).__name__}: {e}", flush=True)
        traceback.print_exc()
    finally:
        for b in bridges:
            b.close()
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        app.close(exit_code=0 if ok else 1)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
