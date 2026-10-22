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
#   --drone-rc on: 실물 조종기 (Pico USB 직렬) 로 PX4 드론을 조종 (5단계 5-A, drone_rc.py, config/rc_input.yaml). 조종 입력은 토픽이 아니고 녹화되지 않는다
#   토픽·주기·렌더 설정: isaacsim/config/ros2_iface.yaml. stamp 는 모두 sim time
#
# 실행 (ROS 2 를 source 한 터미널):
#   source /opt/ros/jazzy/setup.bash
#   ~/isaacsim/python.sh isaacsim/scripts/sim_ros2.py                    # GUI (메인 뷰포트 하나, 프로펠러 회전 켬). 창을 닫으면 종료
#   ~/isaacsim/python.sh isaacsim/scripts/sim_ros2.py --headless --duration 60
#   인자: --flight px4|geometric --position-source --drone-pos x y z --mode static|hover --hover small|mid|big --seed (안 주면 시작 시각) --init-pose q1..q6 | <이름> (예: under_drone)
#   --mode hover --hover <프리셋>: 리셋마다 사인파 주파수·위상을 새로 뽑아 이륙 뒤 기준 위치 둘레를 움직임 (조종자 없이 수집). 잡은 뒤에도 계속 움직이고
#     (실물 드론처럼, 2026-10-16 사용자 결정) 모터 정지 (X) 때 멈춤. --drone-rc on 과 같이 쓰면 에러 (PLAN 6단계 "드론 사인파")
#         --prop-spin on|off (기본 GUI on, headless off) --duration [sim s] (headless 는 없으면 Ctrl+C 까지)
#         --drone-rc on|off (기본 off) --rc-port <직렬 장치> (기본 rc_input.yaml port)
#         --protective-stop on|measure (기본 on. measure = 정지하지 않고 최댓값만 기록, 기준 여유 확인용)
#         --drone-targets (리셋마다 드론 기준 위치를 씬 drone_targets_base 목록 25 점에서 섞어 한 바퀴씩 고름, PLAN 6단계. --drone-pos 와 같이 쓰면 에러)
#         --reset-offset [m] (리셋 때 드론 호버 위치 랜덤 범위 ±값, 기본 ros2_iface.yaml reset.drone_offset. 0 = 항상 씬 drone_pos.
#                             PX4 는 그래도 호버 흔들림 1~3 cm, 위치까지 거의 고정하려면 --flight geometric)
#   손목·third view 화면은 ROS 토픽으로 본다 (예: ros2 run rqt_image_view rqt_image_view). 카메라 뷰포트 창은 렌더 비용이 커서 띄우지 않음
# =============================================================

import argparse
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_scene as ts  # noqa: E402

import numpy as np  # noqa: E402

import protective_stop as ps  # noqa: E402
import sim_reset as sr  # noqa: E402
import ros2_iface as ri  # noqa: E402

STATUS_SEC = 10.0      # 상태 출력 주기 (sim s)
SEED_AUTO_MOD = 1_000_000   # --seed 를 안 주면 시작 시각 (초) % 이 값 (sim_reset.HOVER_SEED_OFFSET 보다 작게)


def _init_pose_action():
    import drone_scene as ds          # 인자 검사만 쓴다 (Isaac Sim 모듈은 build_scene 안에서 import)
    return ds.InitPoseAction


def parse():
    p = argparse.ArgumentParser(description="sim ROS 2 인터페이스 (PLAN 3단계)")
    p.add_argument("--headless", action="store_true")
    p.add_argument("--duration", type=float, default=None, help="[sim s] 이만큼 돌고 끝냄 (없으면 GUI 창을 닫거나 Ctrl+C 까지)")
    p.add_argument("--flight", default=None, choices=("px4", "geometric"), help="드론 제어기 (기본 드론 설정)")
    p.add_argument("--position-source", default=None, choices=("mocap", "flow", "gps"), help="px4 위치 정보 (기본 px4_sitl.yaml)")
    p.add_argument("--drone-pos", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"), help="드론 위치 [m] world")
    p.add_argument("--mode", default="static", choices=("static", "hover"), help="드론 비행 모드")
    p.add_argument("--drone-targets", action="store_true",
                   help="리셋마다 드론 기준 위치를 씬 drone_targets_base 목록에서 (섞어 한 바퀴씩) 고름 (--drone-pos 와 같이 쓰지 않음)")
    p.add_argument("--hover", default=None, help="hover 모드 사인파 프리셋 (드론 설정 flight.hover_presets: small·mid·big). hover 모드면 필수")
    p.add_argument("--seed", type=int, default=None,
                   help="리셋 순서·흔들림·사인파 seed (재현·평가·검사용). 안 주면 시작 시각으로 정하고 터미널·episode.json 에 남김 (수집할 때 세션마다 달라지게)")
    p.add_argument("--init-pose", nargs="+", default=None, metavar="Q", action=_init_pose_action(),
                   help="팔 시작 자세: 관절각 6 개 [rad] 또는 씬 설정 init_poses 의 이름 (예: under_drone). 기본 home_pose. /sim/reset 도 이 자세로 되돌린다")
    p.add_argument("--prop-spin", choices=("on", "off"), default=None, help="프로펠러 회전 표시 (기본 GUI on, headless off)")
    p.add_argument("--protective-stop", choices=("on", "measure"), default="on", help="보호 정지 (measure = 최댓값만 기록)")
    p.add_argument("--reset-offset", type=float, default=None,
                   help="리셋 드론 위치 랜덤 범위 ±[m] (기본 ros2_iface.yaml reset.drone_offset, 0 = 항상 기준 위치)")
    p.add_argument("--drone-rc", choices=("on", "off"), default="off", help="실물 조종기로 PX4 드론 조종 (PLAN 5-A)")
    p.add_argument("--rc-config", default=os.path.join(ts.CONFIG_DIR, "rc_input.yaml"))
    p.add_argument("--rc-port", default=None, help="조종기 직렬 장치 (기본 rc_input.yaml port)")
    p.add_argument("--ros2-config", default=None)
    p.add_argument("--report-dir", default=ts.REPORT_DIR)
    args, _ = p.parse_known_args()
    if args.prop_spin is None:
        args.prop_spin = "off" if args.headless else "on"
    return args


def _git_state():
    """레포 commit 과 수정 여부 (에피소드 메타데이터: 어떤 코드·설정으로 모은 데이터인지)."""
    repo = os.path.dirname(ts.SIM_ROOT)
    def git(*a):
        r = subprocess.run(["git", "-C", repo, *a], capture_output=True, text=True)
        if r.returncode != 0:
            raise RuntimeError(f"git {' '.join(a)} 실패: {r.stderr.strip()}")
        return r.stdout.strip()
    return {"commit": git("rev-parse", "HEAD"), "branch": git("rev-parse", "--abbrev-ref", "HEAD"),
            "dirty": bool(git("status", "--porcelain", "--untracked-files=no"))}


def wrist_camera_mount_info(stage, cam_cfg):
    """로봇 USD 에서 손목 카메라 장착을 읽는다 (설정 파일에 따로 적지 않음 → USD 와 어긋나지 않게). 2026-10-15 마운트 t10 부터.
    cam_tilt_deg = wrist_mount → wrist_camera_mount 의 x 축 회전 (xacro mount_variant 의 기울기, + = 공구 축 쪽),
    optical_in_wrist_mount = 렌즈 (color optical frame) 위치 [m], mount_mesh = 마운트 겉모습 메시 이름 (예: wrist_mount_t10)."""
    from pxr import Usd, UsdGeom

    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    W = lambda path: np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))).T  # noqa: E731  (열벡터 규약)
    wm = ts.find_link_path(stage, "wrist_mount")
    inv = np.linalg.inv(W(wm))
    R = (inv @ W(ts.find_link_path(stage, ts.CAMERA_TILT_PIVOT)))[:3, :3]
    opt = (inv @ W(ts.find_link_path(stage, cam_cfg["frame"])))[:3, 3]
    meshes = sorted({p.GetName() for p in Usd.PrimRange(stage.GetPrimAtPath(wm)) if p.GetName().startswith("wrist_mount_")
                     and "collision" not in p.GetName()})
    return {"cam_tilt_deg": round(float(np.degrees(np.arctan2(R[2, 1], R[1, 1]))), 3),
            "optical_in_wrist_mount": [round(float(x), 5) for x in opt],
            "mount_mesh": meshes[0] if meshes else "wrist_mount (original)"}


def episode_info(s, cfg, args, render, stop_mode):
    """에피소드 메타데이터용 고정 정보 (PLAN 4-2). /sim/reset 응답의 "sim" → record_toggle 의 episode.json → stage1 meta.json.
    경로는 레포 기준, 위치는 world [m] (로봇 base 는 robot_base_world)."""
    repo = os.path.dirname(ts.SIM_ROOT)
    rel = lambda p: os.path.relpath(os.path.abspath(p), repo)
    g = s.drive_cfg["gripper"]
    (gname, gc), = g.items()
    c = s.drive_cfg["contact"]
    ps_c = cfg["protective_stop"]
    f = s.flight
    return {
        "git": _git_state(),
        "physics_hz": round(1.0 / ts.PHYSICS_DT), "loop_hz": float(cfg["loop_hz"]),
        "render": render,
        "robot": {"config": rel(s.robot_cfg["path"]), "base_world": [0.0, 0.0, float(s.scene_cfg["table"]["top_z"])],
                  "home_pose": [float(x) for x in s.home], "init_pose": [float(x) for x in s.init_pose], "init_pose_name": s.init_pose_name,
                  "command_interp_time": float(cfg["arm"]["command_interp_time"])},
        "gripper": {"joint": gname, "max_force_nm": float(gc["max_force"]), "profile_velocity": float(gc["profile_velocity"]),
                    "finger_friction": [float(c["finger_material"]["static_friction"]), float(c["finger_material"]["dynamic_friction"])]},
        "drone": {"config": rel(s.drone_cfg["path"]), "backend": f.backend, "mode": f.ref.mode,
                  "hover_preset": f.ref.preset,                                # hover 사인파 프리셋 이름 (static 이면 null). 뽑힌 값은 리셋 응답 drone_ref
                  "position_source": f.position_source if f.backend == "px4" else None,
                  "nominal_pos": [float(x) for x in s.drone_pos],            # 리셋 기준 위치 (--drone-pos 또는 씬 설정). 리셋마다 ± reset.drone_offset
                  "targets": sorted(s.scene_cfg["drone_targets_base"]) if args.drone_targets else None,   # --drone-targets 목록 이름 (고른 점은 리셋 응답 drone_target)
                  "seed_base": int(args.seed), "seed_auto": bool(args.seed_auto),     # seed_auto: --seed 를 안 줘서 시작 시각으로 정함
                  "prop_spin": args.prop_spin == "on",
                  "grasp_marker": s.drone_info["grasp_marker"],             # 잡는 곳 빨간 테이프 (색·크기·면적, 없으면 null)
                  "rc_teleop": args.drone_rc == "on"},              # 사람이 조종기로 드론을 움직였는지 (조종 입력 자체는 녹화하지 않음)
        "cameras": {"tick_rate": float(cfg["cameras"]["tick_rate"]),
                    "wrist": {"prim": str(s.wrist_cam), "config": rel(ts.WRIST_CAMERA_CONFIG),
                              **wrist_camera_mount_info(s.stage, s.cam_cfg),          # cam_tilt_deg 등은 로봇 USD 에서 읽음
                              "intrinsics": s.cam_cfg["intrinsics"]},
                    "third_view": {"prim": str(s.third_cam), "config": rel(ts._abs(s.scene_cfg["third_view_camera_config"])),
                                   "position": [float(x) for x in s.tv_cfg["position"]], "look_at": [float(x) for x in s.tv_cfg["look_at"]],
                                   "intrinsics": s.tv_cfg["intrinsics"], "model": s.tv_cfg["model"]["usd"],
                                   "real_distortion": s.tv_cfg["real_distortion"]}},   # 실물 렌즈 왜곡 (sim 렌더링에는 없음, 기록만)
        "protective_stop": {"mode": stop_mode, "contact_force_n": float(ps_c["contact_force_n"]),
                            "tracking_error_deg": float(ps_c["tracking_error_deg"]), "joint_speed_dps": float(ps_c["joint_speed_dps"])},
    }


def main():
    args = parse()
    args.seed_auto = args.seed is None
    if args.seed_auto:
        args.seed = int(time.time()) % SEED_AUTO_MOD             # 시작 시각 (초). hover seed (+1 000 000) 와 겹치지 않게 이보다 작게
        print(f"[sim_ros2] seed {args.seed} (--seed 를 안 줘서 시작 시각으로 정함. 같은 조건을 다시 만들려면 --seed {args.seed})", flush=True)
    if args.drone_targets and args.drone_pos is not None:
        raise ValueError("--drone-targets 와 --drone-pos 는 같이 쓸 수 없음 (기준 위치를 어디서 고를지 둘 중 하나)")
    if args.mode == "hover" and args.drone_rc == "on":
        raise ValueError("--mode hover 와 --drone-rc on 은 같이 쓸 수 없음 (드론을 움직이는 쪽이 둘이 됨: 사인파 목표 ↔ 조종기)")
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
    ok, rclpy, node, bridges, rc = False, None, None, [], None
    try:
        import drone_scene as ds
        from isaacsim.core.rendering_manager import RenderingManager, ViewportManager

        rclpy = ri.enable_ros2(cfg)
        rclpy.init()
        rc_cfg = None
        if args.drone_rc == "on":
            import rc_input

            rc_cfg = rc_input.load_config(args.rc_config)

        def setup(s):
            RenderingManager.set_dt(1.0 / loop_hz)          # 앱 루프 = 카메라 주기 (물리는 120 Hz 그대로)
            ri.set_dlss_mode(cfg)                           # stage 를 만든 뒤
            ri.set_camera_tick_rate([s.wrist_cam, s.third_cam], cfg["cameras"]["tick_rate"])
            ps.enable_contact_report(s.stage)              # 보호 정지 접촉력 (재생 전)

        s = ds.build_scene(drone_pos=args.drone_pos, mode=args.mode, hover=args.hover, seed=args.seed, prop_spin=args.prop_spin == "on",
                           init_pose=ds.parse_init_pose(args.init_pose), extra_setup=setup, flight=args.flight,
                           position_source=args.position_source, report_dir=args.report_dir, realtime=not args.headless,
                           px4_params=None if rc_cfg is None else rc_cfg["px4_params"])
        if rc_cfg is not None:
            import drone_rc

            rc = drone_rc.DroneRc(s.flight, rc_cfg, args.rc_port)
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

        targets = None
        if args.drone_targets:
            tb = s.scene_cfg.get("drone_targets_base")
            if not tb:
                raise KeyError("--drone-targets: 씬 설정에 drone_targets_base 목록이 없거나 비었음")
            top = float(s.scene_cfg["table"]["top_z"])
            targets = [(n, np.asarray(p, dtype=float) + np.array([0.0, 0.0, top])) for n, p in tb.items()]
        reset = sr.EpisodeReset(s, cfg, node, arm, grip, stop, step_paced, seed=args.seed, targets=targets)
        bridges = [arm, grip, stop, reset]
        ds.run(s, 0.5)                                       # 카메라 렌더가 한 번 돈 뒤 렌더 설정 확인
        render = ri.check_render_settings(cfg)
        reset.sim_info = episode_info(s, cfg, args, render, args.protective_stop)
        if not args.headless:
            ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[1.9, -1.6, 1.7], target=[0.4, 0.0, 1.1])
        print(f"[sim_ros2] 준비: seed {args.seed}{' (시작 시각)' if args.seed_auto else ''}, 드론 {s.backend} ({s.flight.ref.mode}), 렌더 {render}, loop {loop_hz:g} Hz, "
              f"발행 {cfg['arm']['clock_topic']} {cfg['arm']['joint_states_topic']} {cfg['gripper']['joint_states_topic']} {cfg['gripper']['target_topic']}, "
              f"구독 {cfg['arm']['joint_command_topic']} {cfg['gripper']['command_topic']}, "
              f"서비스 {cfg['reset']['service']} (드론 위치 {'목록 ' + str(len(targets)) + ' 점' if targets else '기준 위치'} ±{cfg['reset']['drone_offset']:g} m), "
              f"카메라 {[cfg['cameras'][n]['topic'] for n in ri.CAMERA_NAMES]}", flush=True)

        t0, wall0 = s.flight.t, time.monotonic()
        pace["t0"], pace["w0"] = t0, wall0
        t_status, w_status, n_status = t0, wall0, 0
        while ds._running():
            step_paced()
            if rc is not None:
                rc.update()
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
                if rc is not None:
                    print(f"[sim_ros2]   {rc.summary()}", flush=True)
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
        if rc is not None:
            print(f"[sim_ros2] 끝: {rc.summary()}", flush=True)
            rc.close()
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
