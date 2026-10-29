#!/usr/bin/env python3
# =============================================================
# sim_ros2_moving_base.py  (docs/PLAN.md 8단계 "움직이는 베이스 수집 연결")
#
# 움직이는 베이스 씬 (moving_base_scene: Husky 직선 왕복 / Stewart 6 축 사인파 위 UR5) + 고정 베이스 sim_ros2.py 와 같은 ROS 2 토픽.
# sim_ros2.py·drone_scene.py·sim_reset.py 는 고치지 않고 부품 (ros2_iface 브리지, protective_stop) 을 그대로 import 해서 쓴다.
#   발행: /clock, /joint_states (120 Hz), /cam/{wrist,third_view}/color/image_raw·camera_info (30 Hz),
#         /gripper/joint_states·/gripper/target (30 Hz), /protective_stop (30 Hz),
#         /base/imu (120 Hz)·/base/odom·/base/joint_states (30 Hz) (moving_base_ros2.BaseBridge, 베이스 궤적 정답값, ros2_iface.yaml base)
#   구독: /joint_command, /gripper/command   (텔레옵 노드·녹화 도구는 고정 베이스와 같은 것)
#   지금 (R3): 베이스는 시작하자마자 움직임, 드론은 기하 제어기 (world 고정 호버). 아직 없음: /sim/reset·/sim/drone_kill, PX4 드론 (R4)
#   토픽·주기·렌더 설정: isaacsim/config/ros2_iface.yaml (고정 베이스와 공용). 보호 정지 대상만 메모리에서 이 씬에 맞게 바꿈 (테이블 없음)
#
# 실행 (ROS 2 를 source 한 터미널):
#   source /opt/ros/jazzy/setup.bash
#   ~/isaacsim/python.sh isaacsim/scripts/sim_ros2_moving_base.py --base husky
#   ~/isaacsim/python.sh isaacsim/scripts/sim_ros2_moving_base.py --base stewart --base-motion small
#   인자: --base husky|stewart (필수), --base-motion small|mid|big|lab (Stewart 필수, Husky 는 안 줌), --scene-config <yaml> (기본 scene_<base>.yaml),
#         --seed (안 주면 시작 시각), --headless --duration [sim s], --prop-spin on|off, --protective-stop on|measure, --ros2-config,
#         --imu-compare (검사용: Isaac IMU 센서 값을 /base/imu_isaac 로 같이 발행)
#   검사: python3 isaacsim/scripts/check_ros2_moving_base.py
# =============================================================

import argparse
import copy
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_scene as ts  # noqa: E402

import numpy as np  # noqa: E402

import protective_stop as ps  # noqa: E402
import ros2_iface as ri  # noqa: E402

STATUS_SEC = 10.0
SEED_AUTO_MOD = 1_000_000
SCENES = {"husky": "scene_husky.yaml", "stewart": "scene_stewart.yaml"}
TAG = "[sim_ros2_moving_base]"


def parse():
    p = argparse.ArgumentParser(description="움직이는 베이스 sim ROS 2 인터페이스 (PLAN 8단계)", allow_abbrev=False)
    p.add_argument("--base", required=True, choices=tuple(SCENES))
    p.add_argument("--base-motion", default=None, help="Stewart 움직임 프리셋 (small·mid·big·lab). Husky 는 안 줌 (궤적은 base_husky.yaml)")
    p.add_argument("--scene-config", default=None, help="기본 isaacsim/config/scene_<base>.yaml")
    p.add_argument("--seed", type=int, default=None, help="베이스 움직임 seed (안 주면 시작 시각)")
    p.add_argument("--headless", action="store_true")
    p.add_argument("--duration", type=float, default=None, help="[sim s] headless 에서 이만큼 돌고 끝 (없으면 Ctrl+C)")
    p.add_argument("--prop-spin", choices=("on", "off"), default=None, help="기본 GUI on, headless off")
    p.add_argument("--protective-stop", choices=ps.MODES, default="on")
    p.add_argument("--ros2-config", default=None)
    p.add_argument("--imu-compare", action="store_true", help="검사용: Isaac IMU 센서를 imu_link 에 달고 /base/imu_isaac 로 발행 (궤적 계산값과 비교)")
    args = p.parse_args()
    if args.prop_spin is None:
        args.prop_spin = "off" if args.headless else "on"
    if args.base == "stewart" and args.base_motion is None:
        p.error("--base stewart 는 --base-motion (small·mid·big·lab) 이 필요")
    if args.base == "husky" and args.base_motion is not None:
        p.error("--base husky 는 --base-motion 을 받지 않음 (궤적은 base_husky.yaml trajectory)")
    if args.scene_config is None:
        args.scene_config = os.path.join(ts.CONFIG_DIR, SCENES[args.base])
    return args


def protective_stop_config(cfg, s):
    """보호 정지 대상 (손가락이 닿으면 안 되는 환경) 을 이 씬에 맞게: 고정 씬의 테이블 대신 바닥·third view 받침대·베이스 충돌체.
    ros2_iface.yaml 은 그대로 두고 메모리 사본만 바꾼다."""
    import moving_base_scene as mbs

    c = copy.deepcopy(cfg)
    env = ["/World/GroundPlane", "/World/ThirdView/mount_plate", "/World/ThirdView/mount_post"] + mbs.base_collision_prims(s.stage, s.base)
    missing = [p for p in env if not s.stage.GetPrimAtPath(p).IsValid()]
    if missing:
        raise RuntimeError(f"보호 정지 대상 prim 이 없음: {missing}")
    c["protective_stop"]["environment_paths"] = env
    return c


def main():
    args = parse()
    args.seed_auto = args.seed is None
    if args.seed_auto:
        args.seed = int(time.time()) % SEED_AUTO_MOD
        print(f"{TAG} seed {args.seed} (--seed 를 안 줘서 시작 시각으로 정함. 같은 조건을 다시 만들려면 --seed {args.seed})", flush=True)
    cfg = ri.load_ros2_config(args.ros2_config)
    loop_hz = float(cfg["loop_hz"])
    n_sub = (1.0 / ts.PHYSICS_DT) / loop_hz
    if abs(n_sub - round(n_sub)) > 1e-9:
        raise ValueError(f"loop_hz {loop_hz} 는 물리 {1.0 / ts.PHYSICS_DT:g} Hz 의 약수여야 함")
    from isaacsim import SimulationApp

    app = SimulationApp(ri.app_config(cfg, args.headless))
    ok, rclpy, node, bridges, s = False, None, None, [], None
    try:
        import drone_scene as ds
        import moving_base_ros2 as mbr
        import moving_base_scene as mbs
        from isaacsim.core.rendering_manager import RenderingManager, ViewportManager
        from isaacsim.core.simulation_manager import SimulationManager

        rclpy = ri.enable_ros2(cfg)
        rclpy.init()

        def setup(s):
            RenderingManager.set_dt(1.0 / loop_hz)          # 앱 루프 = 카메라 주기 (물리는 120 Hz 그대로)
            ri.set_dlss_mode(cfg)                           # stage 를 만든 뒤
            ri.set_camera_tick_rate([s.wrist_cam, s.third_cam], cfg["cameras"]["tick_rate"])
            ps.enable_contact_report(s.stage)              # 보호 정지 접촉력 (재생 전)
            return mbr.add_isaac_imu(s) if args.imu_compare else None      # 검사용 IMU 센서 (재생 전) → s.pre_play

        s = mbs.build_scene(args.scene_config, prop_spin=args.prop_spin == "on", seed=args.seed, pre_play=setup,
                            base_motion=args.base_motion)
        ri.add_camera_publishers(s, cfg)
        node = rclpy.create_node(cfg["node_name"])
        arm = ri.ArmBridge(s, cfg, node)
        grip = ri.GripperBridge(s, cfg, node)
        stop = mbr.MovingProtectiveStop(s, protective_stop_config(cfg, s), node, mode=args.protective_stop, publish_hz=cfg["gripper"]["publish_hz"])
        arm.stop = stop
        base = mbr.BaseBridge(s, cfg, node, isaac_imu=s.pre_play)
        bridges = [arm, grip, stop, base]
        sim_time = SimulationManager.get_simulation_time
        pace = {"t0": None, "w0": None}

        def step_paced():
            ds._update()
            mbs.check_all(s)                                # 베이스·drive·드론 콜백 에러
            if pace["t0"] is not None:
                ahead = (sim_time() - pace["t0"]) - (time.monotonic() - pace["w0"])
                if ahead > 0:
                    time.sleep(ahead)                       # 실제 시간 속도 (텔레옵)

        for _ in range(int(0.5 * loop_hz)):                 # 카메라 렌더가 한 번 돈 뒤 렌더 설정 확인
            step_paced()
        render = ri.check_render_settings(cfg)
        mbs.start_base(s)                                   # R2: 시작하자마자 베이스가 움직임 (R4 부터는 리셋 끝에)
        if not args.headless:
            dz = s.base_z - float(s.fixed_cfg["table"]["top_z"])          # GUI 시점을 이 씬의 로봇 base 높이로
            ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[ds.GUI_EYE[0], ds.GUI_EYE[1], ds.GUI_EYE[2] + dz],
                                            target=[ds.GUI_TARGET[0], ds.GUI_TARGET[1], ds.GUI_TARGET[2] + dz])
        motion = s.base.traj.params() if s.kind == "stewart" else s.base_cfg["trajectory"]
        print(f"{TAG} 준비: 베이스 {args.base} ({motion}), seed {args.seed}{' (시작 시각)' if args.seed_auto else ''}, "
              f"로봇 base 높이 {s.base_z:.3f} m, 드론 geometric (world {np.round(s.drone_pos, 3).tolist()}), 렌더 {render}, loop {loop_hz:g} Hz, "
              f"발행 {cfg['arm']['clock_topic']} {cfg['arm']['joint_states_topic']} {cfg['gripper']['joint_states_topic']} {cfg['gripper']['target_topic']} "
              f"{[cfg['cameras'][n]['topic'] for n in ri.CAMERA_NAMES]} {[cfg['base'][k] for k in ('imu_topic', 'odom_topic', 'joint_states_topic')]}"
              f"{' ' + cfg['base']['isaac_imu_topic'] if args.imu_compare else ''}, "
              f"IMU 위치 (베이스 강체 기준) {np.round(base.r_imu, 4).tolist()}, 중력 {np.round(base.g, 3).tolist()}, "
              f"구독 {cfg['arm']['joint_command_topic']} {cfg['gripper']['command_topic']}", flush=True)

        t0, wall0 = sim_time(), time.monotonic()
        pace["t0"], pace["w0"] = t0, wall0
        t_status, w_status, n_status = t0, wall0, 0
        while ds._running():
            step_paced()
            ri.spin(rclpy, node, bridges)
            t = sim_time()
            if t - t_status >= STATUS_SEC:
                w = time.monotonic()
                cmd = "없음" if arm.last_cmd is None else f"{np.round(arm.last_cmd[1], 3).tolist()} (t {arm.last_cmd[0]:.2f})"
                p_b = s.base.target(s.base.t)[0]
                print(f"{TAG} t {t:7.1f} s, 최근 {STATUS_SEC:g} s RTF {(t - t_status) / (w - w_status):.3f}, /joint_states {arm.n_js - n_status} 개, "
                      f"베이스 {np.round(p_b, 3).tolist()}, /joint_command 누적 {arm.n_cmd} 개, 마지막 명령 {cmd}, "
                      f"그리퍼 present {grip.present_raw():.0f} target {grip.goal_raw:.0f}", flush=True)
                print(f"{TAG}   {stop.summary()}, 정지 중 무시한 팔 명령 {arm.n_ignored} 개, {base.summary()}", flush=True)
                t_status, w_status, n_status = t, w, arm.n_js
            if args.duration is not None and t - t0 >= args.duration:
                break
        ok = True
    except KeyboardInterrupt:
        print(f"{TAG} Ctrl+C 로 종료", flush=True)
        ok = True
    except Exception as e:  # noqa: BLE001
        import traceback
        print(f"[ERROR] {type(e).__name__}: {e}", flush=True)
        traceback.print_exc()
    finally:
        if len(bridges) >= 3:
            print(f"{TAG} 끝: {bridges[2].summary()}, 정지 중 무시한 팔 명령 {bridges[0].n_ignored} 개", flush=True)
        if s is not None and getattr(s, "base", None) is not None:
            s.base.stop()
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
