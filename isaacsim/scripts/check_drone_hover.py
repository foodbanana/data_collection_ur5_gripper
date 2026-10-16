#!/usr/bin/env python3
# =============================================================
# check_drone_hover.py  (docs/PLAN.md 6단계 "드론 사인파·기준 위치 범위")
#
# 드론 사인파 hover (sim_ros2.py --mode hover --hover <프리셋>, PX4) 자동 검사. 시스템 python3 (source /opt/ros/jazzy/setup.bash) 로 실행하면
# sim_ros2.py --headless 를 직접 띄우고 (check_ros2.py 의 Sim·Ros 를 씀), 리셋·그리퍼·모터 정지 서비스와 drone_cmd.py 명령 포트 (status = PX4 목표·추정 위치) 로 판정한다.
#   H1 --mode hover 와 --drone-rc on 을 같이 주면 시작하지 않음 (에러)
#   H2 리셋 응답 drone_ref: 프리셋 진폭, 주파수가 범위 안, seed. 이륙이 끝난 뒤 목표가 사인파로 움직이고 (범위·속도가 프리셋 안) 드론이 따라감
#   H3 drone_cmd.py goto: 기준 위치만 옮기고 사인파는 계속
#   H4 그리퍼 닫기 명령에는 사인파가 계속 (실물 드론처럼, 2026-10-16 사용자 결정), 모터 정지 → 목표 멈춤·응답 hover_freeze = kill
#   H5 다음 리셋: 주파수·위상이 새로 뽑히고 (앞 리셋과 다름) 멈춤이 풀려 다시 움직임
#   H6 --drone-targets: 리셋마다 목록 (씬 drone_targets_base) 에서 다른 점을 고르고, 기준 위치 = 그 점 ± drone_offset, 응답에 이름·위치 기록
#
# 실행:
#   source /opt/ros/jazzy/setup.bash
#   python3 isaacsim/scripts/check_drone_hover.py [--hover mid]      # 약 5 분, 리포트 isaacsim/reports/check_drone_hover_<시각>/
# =============================================================

import argparse
import datetime
import json
import os
import socket
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger

import check_ros2 as cr

CRIT = {
    "moving_frac": 0.3,         # 10 s 동안 목표가 움직인 범위 ≥ 프리셋 진폭 × 이 값 (적어도 한 축)
    "speed_margin": 1.1,        # 목표 속도 ≤ 리셋 응답 max_speed × 이 값 (축별)
    "track_rms_mm": 30.0,       # PX4 추정 위치 − 목표 RMS
    "goto_mm": 5.0,             # goto 뒤 목표 평균이 옮긴 만큼 (± 이 값) 움직임
}
SAMPLE_SEC = 10.0


def cli(req, port, timeout=3.0):
    """drone_cmd.py 명령 포트 (UDP) 로 JSON 명령을 보내고 답을 받는다."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    try:
        s.sendto(json.dumps(req).encode(), ("127.0.0.1", port))
        return json.loads(s.recvfrom(4096)[0].decode())
    finally:
        s.close()


def sample(ros, port, sec, dt=0.1):
    """sim 시간 sec 동안 dt 마다 (sim 시각, 목표, 추정 위치)."""
    out, t_end = [], ros.now() + sec
    while ros.now() < t_end:
        r = cli({"cmd": "status"}, port)
        if r.get("target") is not None and r.get("estimate") is not None:
            out.append((ros.now(), np.array(r["target"]), np.array(r["estimate"])))
        ros.wait_sim(dt)
    if len(out) < 10:
        raise RuntimeError(f"상태 표본이 너무 적음 ({len(out)} 개)")
    t = np.array([x[0] for x in out])
    return t, np.array([x[1] for x in out]), np.array([x[2] for x in out])


def motion(t, tg, est):
    ptp = np.ptp(tg, axis=0)
    v = np.abs(np.diff(tg, axis=0) / np.maximum(np.diff(t), 1e-6)[:, None]).max(axis=0)
    rms = float(np.sqrt((np.linalg.norm(est - tg, axis=1) ** 2).mean()))
    return ptp, v, rms


def main():
    p = argparse.ArgumentParser(description="드론 사인파 hover 자동 검사 (PLAN 6단계)")
    p.add_argument("--hover", default="mid", help="프리셋 (드론 설정 flight.hover_presets)")
    a = p.parse_args()
    if os.environ.get("ROS_DISTRO") != "jazzy":
        sys.exit("source /opt/ros/jazzy/setup.bash 먼저")
    with open(os.path.join(cr.SIM_ROOT, "config", "drone_iris.yaml"), encoding="utf-8") as f:
        preset = yaml.safe_load(f)["flight"]["hover_presets"][a.hover]
    with open(os.path.join(cr.SIM_ROOT, "config", "px4_sitl.yaml"), encoding="utf-8") as f:
        sitl = yaml.safe_load(f)
    port = int(sitl["command_port"]) + int(sitl["instance"])
    amp = np.array(preset["amplitude"], dtype=float)
    out_dir = os.path.join(cr.REPORT_DIR, f"check_drone_hover_{datetime.datetime.now():%Y%m%d_%H%M%S}")
    os.makedirs(out_dir)
    R = cr.Report(out_dir)
    R.log(f"check_drone_hover  {datetime.datetime.now().isoformat(timespec='seconds')}  프리셋 {a.hover} {preset}  기준 {CRIT}")

    # H1 조종기와 같이 주면 에러 (sim 을 띄우기 전에 멈춤)
    r = subprocess.run([cr.PYTHON_SH, os.path.join(cr.SCRIPTS, "sim_ros2.py"), "--headless", "--mode", "hover", "--hover", a.hover,
                        "--drone-rc", "on"], cwd=cr.REPO, capture_output=True, text=True, timeout=120)
    txt = r.stdout + r.stderr
    R.item("H1 --mode hover + --drone-rc on 은 시작하지 않음", r.returncode != 0 and "같이 쓸 수 없음" in txt,
           f"종료 코드 {r.returncode}, 에러 문구 {'있음' if '같이 쓸 수 없음' in txt else '없음'}")

    rclpy.init()
    sim = cr.Sim(["--mode", "hover", "--hover", a.hover, "--drone-targets"], os.path.join(out_dir, "sim.log"))
    with open(os.path.join(cr.SIM_ROOT, "config", "ros2_iface.yaml"), encoding="utf-8") as f:
        offset = float(yaml.safe_load(f)["reset"]["drone_offset"])
    with open(os.path.join(cr.SIM_ROOT, "config", "scene_drone.yaml"), encoding="utf-8") as f:
        sc = yaml.safe_load(f)
    tlist = {n: np.array(p, dtype=float) + np.array([0, 0, float(sc["table"]["top_z"])]) for n, p in sc["drone_targets_base"].items()}
    ros, ex = None, None
    try:
        ros = cr.Ros()
        kill_cli = ros.create_client(Trigger, "/sim/drone_kill")
        ex = SingleThreadedExecutor()
        ex.add_node(ros)
        threading.Thread(target=ex.spin, daemon=True).start()
        sim.wait_ready()
        while ros.now() is None:
            time.sleep(0.1)

        # H2 리셋 → 사인파
        ok, res1 = ros.reset()
        ref1 = res1.get("drone_ref") or {}
        lo, hi = preset["freq_range"]
        f1 = np.array(ref1.get("freq", []), dtype=float)
        good_ref = ok and ref1.get("preset") == a.hover and np.allclose(ref1.get("amplitude", []), amp) \
            and f1.size > 0 and bool(np.all((f1 >= lo) & (f1 <= hi)))
        t, tg, est = sample(ros, port, SAMPLE_SEC)
        ptp, v, rms = motion(t, tg, est)
        vmax = np.array(ref1.get("max_speed", [0, 0, 0]), dtype=float)
        moving = bool(np.any(ptp >= amp * CRIT["moving_frac"]))
        speed_ok = bool(np.all(v <= vmax * CRIT["speed_margin"] + 1e-3))
        R.item("H2 리셋 → 사인파 (응답 drone_ref, 목표가 움직임, 속도, 추적)", good_ref and moving and speed_ok and rms <= CRIT["track_rms_mm"] / 1000,
               f"리셋 {ok} {res1.get('duration_s')} s, 프리셋 {ref1.get('preset')}, 주파수 {np.round(f1, 3).tolist()} Hz, seed {ref1.get('seed')}, "
               f"목표 범위 {np.round(ptp * 1000, 1).tolist()} mm (진폭 {np.round(amp * 1000, 1).tolist()}), "
               f"목표 속도 최대 {np.round(v * 1000, 1).tolist()} mm/s (상한 {np.round(vmax * 1000, 1).tolist()}), 추적 RMS {rms * 1000:.1f} mm")

        # H3 goto = 기준 위치만 옮김
        p0 = np.array(ref1["p0"], dtype=float)
        shift = np.array([0.03, 0.0, 0.0])
        g = cli({"cmd": "goto", "pos": (p0 + shift).tolist()}, port)
        ros.wait_sim(4.0)
        t2, tg2, est2 = sample(ros, port, SAMPLE_SEC)
        d_mean = (tg2.mean(axis=0) - tg.mean(axis=0))
        moved = abs(d_mean[0] - shift[0]) * 1000 <= CRIT["goto_mm"] + np.ptp(tg[:, 0]) * 1000 / 2
        still_moving = bool(np.any(np.ptp(tg2, axis=0) >= amp * CRIT["moving_frac"]))
        st = cli({"cmd": "status"}, port)
        R.item("H3 drone_cmd goto = 기준 위치만 옮기고 사인파 계속", bool(g.get("ok")) and moved and still_moving and not st.get("external"),
               f"goto ok {g.get('ok')}, 목표 평균 이동 {np.round(d_mean * 1000, 1).tolist()} mm (옮긴 것 {np.round(shift * 1000, 1).tolist()}), "
               f"뒤 범위 {np.round(np.ptp(tg2, axis=0) * 1000, 1).tolist()} mm, external {st.get('external')}")

        # H4 그리퍼 닫기 → 계속 움직임, 모터 정지 → 멈춤
        ros.send_grip(1150.0)
        ros.wait_sim(0.5)
        t3, tg3, est3 = sample(ros, port, 4.0)
        still_moving = float(np.ptp(tg3, axis=0).max())
        fut = kill_cli.call_async(Trigger.Request())
        t_end = time.time() + 10
        while not fut.done() and time.time() < t_end:
            time.sleep(0.05)
        kres = json.loads(fut.result().message) if fut.done() else {}
        hf = kres.get("hover_freeze") or {}
        ros.wait_sim(1.0)
        st = cli({"cmd": "status"}, port)                   # kill 하면 PX4 목표를 비움 (위치 목표를 더 보내지 않음)
        R.item("H4 그리퍼 닫기에는 사인파 계속, 모터 정지 → 사인파 멈춤 (hover_freeze = kill)",
               still_moving * 1000 > 5.0 and hf.get("reason") == "kill" and st.get("target") is None and not st.get("armed"),
               f"닫은 뒤 4 s 목표 범위 최대 {still_moving * 1000:.1f} mm, 모터 정지 뒤 목표 {st.get('target')}·armed {st.get('armed')}, hover_freeze {hf}")

        # H5 다음 리셋: 새로 뽑힘, 다시 움직임
        ros.wait_sim(2.0)
        ok2, res2 = ros.reset()
        ref2 = res2.get("drone_ref") or {}
        f2 = np.array(ref2.get("freq", []), dtype=float)
        t4, tg4, est4 = sample(ros, port, SAMPLE_SEC)
        ptp4, v4, rms4 = motion(t4, tg4, est4)
        new = ok2 and f2.shape == f1.shape and not np.allclose(f1, f2) and ref2.get("seed") != ref1.get("seed")
        R.item("H5 다음 리셋: 사인파 새로 뽑힘, 멈춤 풀림", new and bool(np.any(ptp4 >= amp * CRIT["moving_frac"])) and rms4 <= CRIT["track_rms_mm"] / 1000,
               f"리셋 {ok2} {res2.get('duration_s')} s, 주파수 {np.round(f2, 3).tolist()} Hz, seed {ref2.get('seed')} (앞 {ref1.get('seed')}), "
               f"목표 범위 {np.round(ptp4 * 1000, 1).tolist()} mm, 추적 RMS {rms4 * 1000:.1f} mm")
        # H6 기준 위치 목록
        why = []
        for tag, res in (("1", res1), ("2", res2)):
            tg_ = res.get("drone_target") or {}
            nm = tg_.get("name")
            goal = np.array(res.get("drone_goal", [np.nan] * 3), dtype=float)
            if nm not in tlist:
                why.append(f"리셋 {tag} 목록 점 없음 ({nm})")
                continue
            if not np.allclose(tg_.get("center"), tlist[nm], atol=1e-4):
                why.append(f"리셋 {tag} 위치가 목록과 다름")
            if np.abs(goal - tlist[nm]).max() > offset + 1e-6:
                why.append(f"리셋 {tag} 기준 위치가 ±{offset} 밖")
            if not np.allclose((res.get("drone_ref") or {}).get("p0", [np.nan] * 3), goal, atol=1e-4):
                why.append(f"리셋 {tag} drone_ref.p0 ≠ drone_goal")
        n1, n2 = (res1.get("drone_target") or {}).get("name"), (res2.get("drone_target") or {}).get("name")
        if n1 == n2:
            why.append("두 리셋이 같은 점")
        R.item("H6 --drone-targets: 리셋마다 목록에서 다른 점, 기준 위치 = 점 ± drone_offset, 기록", not why,
               f"리셋 1 {n1} → 기준 {np.round(np.array(res1.get('drone_goal')) - tlist.get(n1, 0), 3).tolist() if n1 in tlist else '-'} m, "
               f"리셋 2 {n2}, 목록 {len(tlist)} 점, ±{offset} m" + (f" | {'; '.join(why)}" if why else ""))
    except Exception as e:  # noqa: BLE001
        R.item("진행", False, f"{type(e).__name__}: {e} (sim 로그 {sim.log_path})")
    finally:
        if ex is not None:
            ex.shutdown()
        if ros is not None:
            ros.destroy_node()
        sim.stop()
        rclpy.shutdown()
    R.log("")
    for name, ok in R.results:
        R.log(f"  {'PASS' if ok else 'FAIL'}  {name}")
    n = sum(ok for _, ok in R.results)
    R.log(f"  {n}/{len(R.results)} PASS   리포트 {out_dir}")
    with open(os.path.join(out_dir, "report.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(R.lines) + "\n")
    sys.exit(0 if n == len(R.results) else 1)


if __name__ == "__main__":
    main()
