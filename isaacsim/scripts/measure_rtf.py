#!/usr/bin/env python3
# =============================================================
# measure_rtf.py  (docs/PLAN.md 3-1)
#
# real-time factor(RTF = sim 시간 / wall 시간) 와 카메라 발행 주기를 잰다. 실제 시간 맞춤(sleep) 없이 최대 속도로 돌린다
#   → RTF ≥ 1 이면 실제 시간으로 돌릴 수 있다 (텔레옵 조작감).
#   씬 = drone_scene (테이블·로봇·카메라 2대·드론). 카메라는 Camera Helper 로 발행, 받는 쪽은 별도 프로세스 topic_rate.py
#   카메라 주기: wall 기준(받는 쪽 도착 간격)과 sim 기준(header.stamp 간격) 둘 다
#
# 실행 (ROS 2 를 source 한 터미널):
#   source /opt/ros/jazzy/setup.bash
#   ~/isaacsim/python.sh isaacsim/scripts/measure_rtf.py --headless [--flight px4|geometric] [--loop-hz 120] [--tick-rate 30] [--cameras on|off]
#   --loop-hz : 앱 루프(렌더) 주기. 물리는 120 Hz 고정 → loop 60 이면 update 한 번에 물리 2 스텝
#   --tick-rate : 카메라 prim omni:sensor:tickRate (0 = 매 프레임 렌더). 기본 ros2_iface.yaml
#   --cameras off : 카메라 발행 없이 (물리·드론만의 기준선)
#   --anti-aliasing 3|4 : SimulationApp anti_aliasing (3 DLSS, 4 DLAA = 원래 해상도). 없으면 ros2_iface.yaml render (3 DLSS).
#     RTX 실시간 렌더러는 DLSS·DLAA 만 지원 (0 끔·1 TAA·2 FXAA 를 주면 렌더러가 3 으로 되돌림, omni.rtx.settings.core "Invalid AA Mode")
#   --dlss-mode 0|1|2|3 : /rtx/post/dlss/execMode (0 Performance, 1 Balanced, 2 Quality, 3 Auto). 없으면 ros2_iface.yaml render (0).
#     새 stage 를 만들 때 기본값으로 돌아가므로 stage 를 만든 뒤 적용. 적용이 안 되면 에러
#   --arm-motion : 측정 구간 동안 팔을 움직임 (shoulder_pan·wrist_3 사인파, 움직일 때 잔상 확인용)
#   --save-images : 받는 쪽이 발행된 이미지를 PNG 로 (측정 직전 정지 1 장 + 측정 중 2 장) → <리포트>/images/
# =============================================================

import argparse
import datetime
import json
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_scene as ts  # noqa: E402

import numpy as np  # noqa: E402

import ros2_iface as ri  # noqa: E402


def parse():
    p = argparse.ArgumentParser(description="RTF·카메라 주기 측정 (PLAN 3-1)")
    p.add_argument("--headless", action="store_true")
    p.add_argument("--flight", default=None, choices=("px4", "geometric"), help="드론 제어기 (기본 드론 설정)")
    p.add_argument("--loop-hz", type=float, default=120.0, help="앱 루프(렌더) 주기 [Hz]. 물리 120 Hz 의 약수")
    p.add_argument("--tick-rate", type=float, default=None, help="카메라 tickRate [Hz] (0 = 매 프레임). 기본 ros2_iface.yaml")
    p.add_argument("--cameras", choices=("on", "off"), default="on")
    p.add_argument("--camera-windows", action="store_true", help="GUI: 손목·third view 뷰포트 창도 띄움 (drone_scene GUI 와 같게)")
    p.add_argument("--anti-aliasing", type=int, choices=(3, 4), default=None, help="3 DLSS, 4 DLAA (기본 ros2_iface.yaml render)")
    p.add_argument("--dlss-mode", type=int, choices=range(4), default=None, help="DLSS 0 Performance, 1 Balanced, 2 Quality, 3 Auto (기본 ros2_iface.yaml render)")
    p.add_argument("--init-pose", type=float, nargs=6, default=None, metavar="Q",
                   help="팔 시작 자세 [rad] (기본 home_pose). 예: 손목 카메라가 드론을 봄 -0.1888 -0.7854 0.9599 1.3963 -1.5708 0.1888")
    p.add_argument("--prop-spin", choices=("on", "off"), default="off", help="프로펠러를 보여 주기용으로 돌림 (drone_scene 과 같음)")
    p.add_argument("--arm-motion", action="store_true", help="측정 중 팔을 사인파로 움직임")
    p.add_argument("--save-images", action="store_true", help="발행된 카메라 이미지를 PNG 로 (정지 1 + 움직임 2)")
    p.add_argument("--profile", action="store_true", help="측정 구간을 cProfile 로 (우리 Python 코드 시간, RTF 는 느려짐) → profile.txt")
    p.add_argument("--seconds", type=float, default=20.0, help="측정 구간 sim 시간 [s]")
    p.add_argument("--warmup", type=float, default=3.0, help="측정 전 sim 시간 [s]")
    p.add_argument("--ros2-config", default=None)
    p.add_argument("--report-dir", default=ts.REPORT_DIR)
    args, _ = p.parse_known_args()
    return args


# --arm-motion: 관절 이름 → (진폭 rad, 주파수 Hz). 최대 속도 = 2π·f·A (shoulder_pan 0.75 rad/s, wrist_3 1.5 rad/s, 텔레옵 수준)
ARM_MOTION = {"shoulder_pan_joint": (0.4, 0.3), "wrist_3_joint": (0.6, 0.4)}
# --save-images: 측정 시작 기준 sim 시각 [s] → 이름 (음수 = 측정 직전, 팔 정지)
SAVE_AT = {-0.5: "static", 2.0: "moving_a", 5.0: "moving_b"}


def start_receiver(cfg, out, save_dir=None, save_at=()):
    """시스템 python3 로 topic_rate.py (python.sh 환경변수를 물려받지 않게 깨끗한 환경 + ROS source)."""
    topics = " ".join(f"{cfg['cameras'][n]['topic']}:sensor_msgs/msg/Image" for n in ri.CAMERA_NAMES)
    save = f"--save-dir {save_dir} --save-at {' '.join(f'{t:.4f}' for t in save_at)}" if save_dir else ""
    cmd = (f"source /opt/ros/{cfg['ros_distro']}/setup.bash && exec python3 {ri.SCRIPT_DIR}/topic_rate.py "
           f"{topics} --out {out} {save}")     # --save-at 는 개수가 정해지지 않아 토픽 뒤에
    env = {k: os.environ[k] for k in ("HOME", "USER", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION") if k in os.environ}
    env["PATH"] = "/usr/bin:/bin"
    proc = subprocess.Popen(["bash", "-c", cmd], env=env, stdout=subprocess.PIPE, text=True)
    if proc.stdout.readline().strip() != "topic_rate ready":
        raise RuntimeError("topic_rate.py 가 시작하지 못함")
    return proc


def rates(samples, w0, w1):
    """측정 구간(wall w0~w1)에 도착한 메시지 → (개수, wall Hz, sim Hz, stamp 간격 최대 [ms], 100 ms 넘는 간격 수)."""
    a = np.array([x for x in samples if w0 <= x[0] <= w1])
    if len(a) < 2:
        return len(a), 0.0, 0.0, float("nan"), 0
    wall_hz = (len(a) - 1) / (a[-1, 0] - a[0, 0])
    sim_span = a[-1, 1] - a[0, 1]
    sim_hz = (len(a) - 1) / sim_span if sim_span > 0 else float("inf")
    gaps = np.diff(a[:, 1])
    return len(a), wall_hz, sim_hz, float(gaps.max() * 1000), int((gaps > 0.1).sum())


def main():
    args = parse()
    from isaacsim import SimulationApp

    cfg = ri.load_ros2_config(args.ros2_config)
    for key, val in (("anti_aliasing", args.anti_aliasing), ("dlss_exec_mode", args.dlss_mode)):
        if val is not None:
            cfg["render"][key] = val                           # 비교용 덮어쓰기
    app = SimulationApp(ri.app_config(cfg, args.headless))
    ok = False
    recv = None
    try:
        import drone_scene as ds
        from isaacsim.core.rendering_manager import RenderingManager

        tick = cfg["cameras"]["tick_rate"] if args.tick_rate is None else args.tick_rate
        ri.enable_ros2(cfg)
        n_sub = 120.0 / args.loop_hz
        if abs(n_sub - round(n_sub)) > 1e-9:
            raise ValueError(f"--loop-hz 는 물리 120 Hz 의 약수: {args.loop_hz}")

        from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager

        # 물리 구간 시간: 같은 order 의 콜백은 등록 순서대로 불린다 (SimulationManager.register_callback 문서).
        #   _pre 는 drive·드론 콜백보다 먼저(재생 전), _mid 는 그 뒤(build_scene 후) 등록
        #   → _pre→_mid = 우리 Python 콜백, _mid→_post = PhysX 스텝
        phys = {"t": None, "tm": None, "sum": 0.0, "py": 0.0}

        def _pre(dt, ctx):
            phys["t"] = time.perf_counter()

        def _mid(dt, ctx):
            phys["tm"] = time.perf_counter()
            if phys["t"] is not None:
                phys["py"] += phys["tm"] - phys["t"]

        def _post(dt, ctx):
            if phys["t"] is not None:
                phys["sum"] += time.perf_counter() - phys["t"]

        def setup(s):
            SimulationManager.register_callback(_pre, event=SimulationEvent.PHYSICS_PRE_STEP)
            SimulationManager.register_callback(_post, event=SimulationEvent.PHYSICS_POST_STEP)
            RenderingManager.set_dt(1.0 / args.loop_hz)
            ri.set_dlss_mode(cfg)                               # stage 를 만든 뒤 (새 stage 는 기본값으로 돌아감)
            if args.cameras == "on":
                ri.set_camera_tick_rate([s.wrist_cam, s.third_cam], tick)

        s = ds.build_scene(flight=args.flight, extra_setup=setup, report_dir=args.report_dir, init_pose=args.init_pose,
                           prop_spin=args.prop_spin == "on")
        SimulationManager.register_callback(_mid, event=SimulationEvent.PHYSICS_PRE_STEP)
        if args.cameras == "on":
            ri.add_camera_publishers(s, cfg)
        if args.camera_windows:
            if args.headless:
                raise ValueError("--camera-windows 는 GUI 에서만")
            ds.open_camera_windows(s)
        out_dir = os.path.join(args.report_dir, f"measure_rtf_{datetime.datetime.now():%Y%m%d_%H%M%S}")
        os.makedirs(out_dir, exist_ok=True)
        rate_json = os.path.join(out_dir, "topics.json")
        img_dir, save_abs = None, {}
        if args.save_images:
            if args.cameras != "on":
                raise ValueError("--save-images 는 카메라 발행이 켜져 있어야 함")
            img_dir = os.path.join(out_dir, "images")
            t_meas = SimulationManager.get_simulation_time() + args.warmup    # 카메라 stamp 와 같은 시계
            save_abs = {round(t_meas + dt, 2): name for dt, name in SAVE_AT.items()}   # 파일 이름이 .2f
        if args.cameras == "on":
            recv = start_receiver(cfg, rate_json, img_dir, list(save_abs))
        ds.run(s, args.warmup)
        render = ri.check_render_settings(cfg)                 # 다르면 에러
        q_home = s.robot.get_dof_positions().numpy()[0][s.drive.arm_i].copy()
        motion = [(ts.ARM_JOINTS.index(j), a, f) for j, (a, f) in ARM_MOTION.items()] if args.arm_motion else []
        dts, sim_steps = [], []
        phys["sum"] = phys["py"] = 0.0
        prof = None
        if args.profile:
            import cProfile
            prof = cProfile.Profile()
            prof.enable()
        t0, w0, w0_epoch = s.flight.t, time.monotonic(), time.time()
        while s.flight.t < t0 + args.seconds - 1e-9:
            a, ts0 = time.monotonic(), s.flight.t
            if motion:
                q = q_home.copy()
                for i, amp, f in motion:
                    q[i] += amp * np.sin(2 * np.pi * f * (s.flight.t - t0))
                s.drive.set_arm_targets(q)
            ds.step(s)
            dts.append(time.monotonic() - a)
            sim_steps.append(s.flight.t - ts0)
        wall = time.monotonic() - w0
        if prof is not None:
            import io
            import pstats
            prof.disable()
            buf = io.StringIO()
            st = pstats.Stats(prof, stream=buf)
            st.sort_stats("tottime").print_stats(45)
            st.sort_stats("cumulative").print_stats(ri.SCRIPT_DIR, 40)
            with open(os.path.join(out_dir, "profile.txt"), "w", encoding="utf-8") as f:
                f.write(buf.getvalue())
        w1_epoch = time.time()
        rtf = (s.flight.t - t0) / wall
        dts = np.array(dts) * 1000
        lines = [f"measure_rtf  {datetime.datetime.now().isoformat(timespec='seconds')}",
                 f"headless {args.headless}, 카메라 창 {args.camera_windows}, 드론 {s.backend}, loop {args.loop_hz:g} Hz (RenderingManager dt {RenderingManager.get_dt() * 1000:.2f} ms), "
                 f"물리 120 Hz, update 당 sim {np.mean(sim_steps) * 1000:.2f} ms, 카메라 {args.cameras} tickRate {tick:g}",
                 f"렌더: {render['anti_aliasing']} {render['dlss_exec_mode'] if render['anti_aliasing'] == 'DLSS' else ''}, "
                 f"팔 움직임 {'on' if motion else 'off'}, 프로펠러 회전 {args.prop_spin}",
                 f"RTF {rtf:.3f}  (sim {s.flight.t - t0:.1f} s / wall {wall:.1f} s), update wall 평균 {dts.mean():.2f} ms, "
                 f"p95 {np.percentile(dts, 95):.2f} ms, 최대 {dts.max():.1f} ms",
                 f"  update 당 물리 구간 {phys['sum'] * 1000 / len(dts):.2f} ms (우리 Python 콜백 {phys['py'] * 1000 / len(dts):.2f} ms + "
                 f"PhysX {(phys['sum'] - phys['py']) * 1000 / len(dts):.2f} ms), "
                 f"나머지(렌더·앱·우리 루프) {dts.mean() - phys['sum'] * 1000 / len(dts):.2f} ms"]
        if recv is not None:
            recv.terminate()
            recv.wait(timeout=10)
            recv = None
            with open(rate_json, encoding="utf-8") as f:
                rec = json.load(f)
            for name in ri.CAMERA_NAMES:
                n, wall_hz, sim_hz, gap, n_gap = rates(rec[cfg["cameras"][name]["topic"]], w0_epoch, w1_epoch)
                lines.append(f"  카메라 {name}: {n} 장, wall {wall_hz:.1f} Hz, sim {sim_hz:.1f} Hz, stamp 간격 최대 {gap:.1f} ms, 100 ms 넘는 간격 {n_gap} 번")
        if img_dir is not None:
            for t_abs, name in save_abs.items():
                for cam in ri.CAMERA_NAMES:
                    src = os.path.join(img_dir, f"{cfg['cameras'][cam]['topic'].strip('/').replace('/', '_')}_{t_abs:.2f}.png")
                    if not os.path.exists(src):
                        raise RuntimeError(f"이미지 저장 안 됨: {src}")
                    os.replace(src, os.path.join(img_dir, f"{cam}_{name}.png"))
            lines.append(f"  이미지: {img_dir}/ ({', '.join(SAVE_AT.values())})")
        for ln in lines:
            print(ln, flush=True)
        with open(os.path.join(out_dir, "report.txt"), "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        ok = True
    except Exception as e:  # noqa: BLE001
        import traceback
        print(f"[ERROR] {type(e).__name__}: {e}", flush=True)
        traceback.print_exc()
    finally:
        if recv is not None:
            recv.kill()
        app.close(exit_code=0 if ok else 1)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
