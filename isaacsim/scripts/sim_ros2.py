#!/usr/bin/env python3
# =============================================================
# sim_ros2.py  (docs/PLAN.md 3단계)
#
# sim ROS 2 인터페이스 실행: 드론 파지 씬(drone_scene) + 실물과 같은 ROS 2 토픽. 실제 시간 속도로 돈다 (텔레옵).
#   발행: /clock, /joint_states (팔 6 관절, 물리 스텝마다 120 Hz), /cam/{wrist,third_view}/color/image_raw·camera_info (30 Hz),
#         /gripper/joint_states (present raw)·/gripper/target (goal raw) (30 Hz, 같은 stamp)
#   구독: /joint_command (팔 6 관절 목표 [rad]), /gripper/command (raw 0 / 1150)
#         /protective_stop (std_msgs/Bool, 30 Hz) — 보호 정지 흉내 (3-7, protective_stop.py)
#   서비스: /sim/reset (std_srvs/Trigger) — 에피소드 리셋 (3-6, sim_reset.py). 끝날 때까지 sim 을 돌린 뒤 응답 (message = JSON)
#   (/base/imu 는 8단계부터)
#   토픽·주기·렌더 설정: isaacsim/config/ros2_iface.yaml. stamp 는 모두 sim time
#
# 실행 (ROS 2 를 source 한 터미널):
#   source /opt/ros/jazzy/setup.bash
#   ~/isaacsim/python.sh isaacsim/scripts/sim_ros2.py                    # GUI (메인 뷰포트 하나, 프로펠러 회전 켬). 창을 닫으면 종료
#   ~/isaacsim/python.sh isaacsim/scripts/sim_ros2.py --headless --duration 60
#   인자: --flight px4|geometric --position-source --drone-pos x y z --mode static|hover --seed --init-pose q1..q6
#         --prop-spin on|off (기본 GUI on, headless off) --duration [sim s] (headless 는 없으면 Ctrl+C 까지)
#         --protective-stop on|measure (기본 on. measure = 정지하지 않고 최댓값만 기록, 기준 여유 확인용)
#         --reset-offset [m] (리셋 때 드론 호버 위치 랜덤 범위 ±값, 기본 ros2_iface.yaml reset.drone_offset. 0 = 항상 씬 drone_pos.
#                             PX4 는 그래도 호버 흔들림 1~3 cm, 위치까지 거의 고정하려면 --flight geometric)
#   손목·third view 화면은 ROS 토픽으로 본다 (예: ros2 run rqt_image_view rqt_image_view). 카메라 뷰포트 창은 렌더 비용이 커서 띄우지 않음
# =============================================================

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_scene as ts  # noqa: E402

import numpy as np  # noqa: E402

import protective_stop as ps  # noqa: E402
import sim_reset as sr  # noqa: E402
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
    p.add_argument("--protective-stop", choices=("on", "measure"), default="on", help="보호 정지 (measure = 최댓값만 기록)")
    p.add_argument("--reset-offset", type=float, default=None,
                   help="리셋 드론 위치 랜덤 범위 ±[m] (기본 ros2_iface.yaml reset.drone_offset, 0 = 항상 기준 위치)")
    p.add_argument("--ros2-config", default=None)
    p.add_argument("--report-dir", default=ts.REPORT_DIR)
    args, _ = p.parse_known_args()
    if args.prop_spin is None:
        args.prop_spin = "off" if args.headless else "on"
    return args


def main():
    args = parse()
    cfg = ri.load_ros2_config(args.ros2_config)
    if args.reset_offset is not None:
        if args.reset_offset < 0:
            raise ValueError(f"--reset-offset 는 0 이상 [m]: {args.reset_offset}")
        cfg["reset"]["drone_offset"] = args.reset_offset
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
            ps.enable_contact_report(s.stage)              # 보호 정지 접촉력 (재생 전)

        s = ds.build_scene(drone_pos=args.drone_pos, mode=args.mode, seed=args.seed, prop_spin=args.prop_spin == "on",
                           init_pose=args.init_pose, extra_setup=setup, flight=args.flight,
                           position_source=args.position_source, report_dir=args.report_dir, realtime=not args.headless)
        ri.add_camera_publishers(s, cfg)
        node = rclpy.create_node(cfg["node_name"])
        arm = ri.ArmBridge(s, cfg, node)
        grip = ri.GripperBridge(s, cfg, node)
        stop = ps.ProtectiveStop(s, cfg, node, mode=args.protective_stop, publish_hz=cfg["gripper"]["publish_hz"])
        arm.stop = stop
        pace = {"t0": None, "w0": None}

        def step_paced():
            """한 루프 + 실제 시간 맞춤 (메인 루프·리셋 서비스 공용)."""
            ds.step(s)
            if pace["t0"] is not None:
                ahead = (s.flight.t - pace["t0"]) - (time.monotonic() - pace["w0"])
                if ahead > 0:
                    time.sleep(ahead)                       # 실제 시간 속도 (텔레옵)

        reset = sr.EpisodeReset(s, cfg, node, arm, grip, stop, step_paced, seed=args.seed)
        bridges = [arm, grip, stop, reset]
        ds.run(s, 0.5)                                       # 카메라 렌더가 한 번 돈 뒤 렌더 설정 확인
        render = ri.check_render_settings(cfg)
        if not args.headless:
            ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[1.9, -1.6, 1.7], target=[0.4, 0.0, 1.1])
        print(f"[sim_ros2] 준비: 드론 {s.backend} ({s.flight.ref.mode}), 렌더 {render}, loop {loop_hz:g} Hz, "
              f"발행 {cfg['arm']['clock_topic']} {cfg['arm']['joint_states_topic']} {cfg['gripper']['joint_states_topic']} {cfg['gripper']['target_topic']}, "
              f"구독 {cfg['arm']['joint_command_topic']} {cfg['gripper']['command_topic']}, "
              f"서비스 {cfg['reset']['service']} (드론 위치 ±{cfg['reset']['drone_offset']:g} m), "
              f"카메라 {[cfg['cameras'][n]['topic'] for n in ri.CAMERA_NAMES]}", flush=True)

        t0, wall0 = s.flight.t, time.monotonic()
        pace["t0"], pace["w0"] = t0, wall0
        t_status, w_status, n_status = t0, wall0, 0
        while ds._running():
            step_paced()
            n_before = arm.n_cb
            ri.spin(rclpy, node, bridges)                   # (리셋 서비스는 이 안에서 끝날 때까지 돈다)
            reset.release()                                 # 리셋이 끝났으면 명령 받기 재개 (리셋 중 쌓인 명령은 방금 spin 이 버림)
            if arm.n_cmd:                                   # 첫 명령 뒤부터: 루프마다 받은 명령 수
                arm.cmd_per_loop.append(arm.n_cb - n_before)
            if s.flight.t - t_status >= STATUS_SEC:
                w = time.monotonic()
                cmd = "없음" if arm.last_cmd is None else f"{np.round(arm.last_cmd[1], 3).tolist()} (t {arm.last_cmd[0]:.2f})"
                print(f"[sim_ros2] t {s.flight.t:7.1f} s, 최근 {STATUS_SEC:g} s RTF {(s.flight.t - t_status) / (w - w_status):.3f}, "
                      f"/joint_states {arm.n_js - n_status} 개, /joint_command 누적 {arm.n_cmd} 개, 마지막 명령 {cmd}, "
                      f"그리퍼 present {grip.present_raw():.0f} target {grip.goal_raw:.0f} (명령 누적 {grip.n_cmd})", flush=True)
                print(f"[sim_ros2]   {stop.summary()}, 정지 중 무시한 팔 명령 {arm.n_ignored} 개", flush=True)
                if reset.failed:
                    print("[sim_ros2]   마지막 리셋 실패 → 팔·그리퍼 명령 무시 중. /sim/reset 을 다시 호출하세요", flush=True)
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
        if len(bridges) >= 3:                               # 끝날 때 (Ctrl+C 포함) 보호 정지 요약
            print(f"[sim_ros2] 끝: {bridges[2].summary()}, 정지 중 무시한 팔 명령 {bridges[0].n_ignored} 개", flush=True)
            c = np.bincount(bridges[0].cmd_per_loop) if bridges[0].cmd_per_loop else []
            print(f"[sim_ros2] 루프마다 받은 /joint_command 수 분포 (0 개, 1 개, 2 개, …): {list(c)}", flush=True)
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
