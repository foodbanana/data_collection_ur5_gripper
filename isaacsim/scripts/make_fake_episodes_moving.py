#!/usr/bin/env python3
# =============================================================
# make_fake_episodes_moving.py  (docs/PLAN.md 8단계 "움직이는 베이스 수집 연결" R5)
#
# 움직이는 베이스 녹화 → 변환 → 병합 → 검수를 텔레옵 장치 없이 끝까지 돌린다 (고정 베이스 make_fake_episodes.py 의 움직이는 베이스판).
# 시스템 python3 (source /opt/ros/jazzy/setup.bash). sim_ros2_moving_base.py --headless 를 직접 띄우고 (기하 제어기 드론, 리셋 위치 고정),
# 녹화는 record_toggle (--base moving 과 같은 RosLink(base="moving")), 명령 재생·드론 정지는 make_fake_episodes.record_episode 그대로.
#   A. static 베이스 (--base-motion static, 팔 시작 자세 = 홈): 고정 명령 재생으로 드론을 실제로 잡는 에피소드 --success 개
#      → stage1 (--base-imu topic --base-pose odom) → 병합 → inspect_dataset_v21.py --load
#      (흔들리는 베이스에서는 고정 명령으로 잡을 수 없어 검수의 파지 신호 항목을 볼 수 없음)
#   B. 흔들리는 베이스 (Stewart --base-motion small / Husky 설정 궤적): 시작 자세를 유지하는 에피소드 1 개 → stage1 만
#      → base_imu.npy·base_pose.npy 가 실제로 변하는지 (병합 안 함: 파지 신호 없음)
# 완료 기준 (하나라도 FAIL 이면 종료 코드 1):
#   1 A 에피소드가 v2.1 데이터셋 하나로 병합 (에피소드 수, merge_manifest base_imu = topic)
#   2 meta/info.json observation.base_imu (6,), 검수 통과 (inspect --load)
#   3 A: episode.json base = moving·리셋 응답 base, base_imu ≈ 상수 (0, 0, 0, 0, 0, |g|) (서 있으니), base_pose 원점
#   4 B: base_imu 가 변함 (비력 범위 > 0.01 m/s², z 평균 ≈ |g|), base_pose 가 움직임 (> 5 mm), frame (odom, 베이스 강체)
#
# 실행:
#   source /opt/ros/jazzy/setup.bash
#   python3 isaacsim/scripts/make_fake_episodes_moving.py [--base stewart|husky] [--success 2] [--cleanup]
#   --cleanup: 끝난 뒤 이 실행이 만든 bag·중간 파일만 지움 (데이터셋·리포트는 남김. bag 30 s ≈ 1.5 GB)
#   리포트·로그: isaacsim/reports/fake_moving_<시각>/ , 데이터셋: lerobot_dataset_v21/local/fake_moving_<시각>/
# =============================================================

import argparse
import csv
import datetime
import json
import os
import shutil
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, REPO)
sys.path.insert(0, HERE)

import make_fake_episodes as mfe  # noqa: E402
import record_toggle as rt  # noqa: E402
from camera_config import load_cameras  # noqa: E402
from check_ros2 import ARM, GRASP_CSV, HOME  # noqa: E402
from check_ros2_moving_base import Sim  # noqa: E402

G = 9.81
TASK_NAME = "fake_moving"
HOLD_SEC = 12.0
MOVING = {"stewart": ["--base-motion", "small"], "husky": []}
CHILD = {"stewart": "top_plate", "husky": "base_link"}


def record_session(sim_args, plan, out_dir, tag, log):
    """sim 을 띄워 plan [(kind, csv)] 대로 녹화. → [(kind, bag 이름)]"""
    sim, link, rec, bags = None, None, None, []
    try:
        log(f"sim 띄우는 중 ({tag}: {' '.join(sim_args)})...")
        sim = Sim(sim_args, os.path.join(out_dir, f"sim_{tag}.log"))
        sim.wait_ready()
        link = rt.RosLink(True, load_cameras(), base="moving")
        rec = rt.BagRecorder(TASK_NAME, link)
        problems = link.check_topics(wait=30.0)
        if problems:
            raise RuntimeError(f"sim 토픽 문제: {problems}")
        for k, (kind, csv_path) in enumerate(plan):
            log(f"── {tag} 에피소드 {k + 1}/{len(plan)}: {kind}")
            bags.append((kind, mfe.record_episode(kind, rec, link, csv_path, out_dir, k)))
    finally:
        if rec is not None:
            if rec.recording:
                rec.stop()
            rec.close()
        if link is not None:
            link.close()
        if sim is not None:
            sim.stop()
    return bags


def stage1(bag, out_dir, log):
    log(f"stage1: {bag}")
    mfe.run(["python3", os.path.join(REPO, "lerobot_stage1_extract_bag.py"), os.path.join(rt.BAG_DIR, bag),
             "--arm-action", "command", "--base-imu", "topic", "--base-pose", "odom", "--overwrite"], os.path.join(out_dir, f"stage1_{bag}.log"))
    d = os.path.join(REPO, "bag_lerobot_intermediate", bag)
    return {"meta": json.load(open(os.path.join(d, "meta.json"))), "imu": np.load(os.path.join(d, "base_imu.npy")),
            "pose": np.load(os.path.join(d, "base_pose.npy"))}


def main():
    ap = argparse.ArgumentParser(description="움직이는 베이스 가짜 에피소드 → 병합·검수 (PLAN 8단계 R5)")
    ap.add_argument("--base", default="stewart", choices=tuple(MOVING))
    ap.add_argument("--success", type=int, default=2, help="A (static 베이스) 드론 잡기 에피소드 수 (2 이상)")
    ap.add_argument("--task", default="pick up the drone")
    ap.add_argument("--cleanup", action="store_true", help="끝난 뒤 이 실행이 만든 bag·중간 파일을 지움")
    args = ap.parse_args()
    if args.success < 2:
        sys.exit("--success 는 2 이상 (여러 에피소드 병합 확인)")
    if os.environ.get("ROS_DISTRO") != "jazzy":
        sys.exit("source /opt/ros/jazzy/setup.bash 먼저")

    stamp = f"{datetime.datetime.now():%Y%m%d_%H%M%S}"
    out_dir = os.path.join(REPO, "isaacsim", "reports", f"fake_moving_{stamp}")
    os.makedirs(out_dir)
    report = []

    def log(msg):
        print(msg, flush=True)
        report.append(msg)

    hold_csv = os.path.join(out_dir, "commands_hold.csv")          # B: 시작 자세 (홈) 유지
    with open(hold_csv, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", *ARM, "gripper_goal_raw"])
        for i in range(1, int(HOLD_SEC * 60) + 1):
            w.writerow([f"{i / 60:.6f}", *[f"{x:.6f}" for x in HOME], "0.000000"])
    common = ["--base", args.base, "--flight", "geometric", "--reset-offset", "0", "--init-pose", *[f"{x:.6f}" for x in HOME]]
    log(f"make_fake_episodes_moving {stamp}: 베이스 {args.base}, A static 잡기 {args.success} 개, B 흔들림 1 개  (리포트 {out_dir})")

    bags_a = record_session(common + ["--base-motion", "static"], [("success", GRASP_CSV)] * args.success, out_dir, "A_static", log)
    bags_b = record_session(common + MOVING[args.base], [("hold", hold_csv)], out_dir, "B_moving", log)
    made = [b for _, b in bags_a + bags_b]

    res_a = {b: stage1(b, out_dir, log) for _, b in bags_a}
    res_b = {b: stage1(b, out_dir, log) for _, b in bags_b}

    manifest = os.path.join(out_dir, "merge_fake_moving.txt")
    with open(manifest, "w", encoding="utf-8") as f:
        f.write("# make_fake_episodes_moving.py: A (static 베이스) 에피소드\n" + "\n".join(res_a) + "\n")
    repo_id = f"local/fake_moving_{stamp}"
    root = os.path.join(REPO, "lerobot_dataset_v21", repo_id)
    log(f"병합: {len(res_a)} 개 → {root}")
    mfe.run(["python", os.path.join(REPO, "lerobot_merge_episodes_v21.py"), "--manifest", manifest,
             "--intermediate-dir", os.path.join(REPO, "bag_lerobot_intermediate"), "--repo-id", repo_id, "--task", args.task],
            os.path.join(out_dir, "merge.log"), conda=True)
    inspect_log = os.path.join(out_dir, "inspect.log")
    inspect_ok = True
    try:
        mfe.run(["python", os.path.join(REPO, "inspect_dataset_v21.py"), root, "--load"], inspect_log, conda=True)
    except RuntimeError:
        inspect_ok = False
    inspect_txt = open(inspect_log, encoding="utf-8", errors="replace").read()

    info = json.load(open(os.path.join(root, "meta", "info.json")))
    mm = json.load(open(os.path.join(root, "meta", "merge_manifest.json")))
    results = []

    def item(name, ok, detail):
        results.append(bool(ok))
        log(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")

    log("")
    got = sorted(e["source"] for e in mm["episodes"])
    item("1 A 에피소드 병합", got == sorted(res_a) and info["total_episodes"] == len(res_a) and mm.get("base_imu") == "topic",
         f"데이터셋 에피소드 {info['total_episodes']} 개 ({info['total_frames']} 프레임), merge_manifest base_imu {mm.get('base_imu')}")
    item("2 info.json·검수", info["features"].get("observation.base_imu", {}).get("shape") == [6] and inspect_ok,
         f"observation.base_imu {info['features'].get('observation.base_imu', {}).get('shape')}, "
         + " / ".join(ln.split(":")[0] for ln in inspect_txt.splitlines() if ln.startswith("[")))
    rows = []
    ok3 = True
    for b, r in res_a.items():
        ep = r["meta"]["episode"]
        dev = float(np.abs(r["imu"] - [0, 0, 0, 0, 0, G]).max())
        pose = float(np.abs(r["pose"][:, :3]).max()) * 1000
        good = (ep.get("base") == "moving" and (ep.get("sim_reset") or {}).get("base") is not None and dev < 1e-3 and pose < 1.0
                and r["meta"]["base_pose"]["child_frame"] == CHILD[args.base])
        ok3 &= good
        rows.append(f"{b}: base_imu − 상수 최대 {dev:.2e}, base_pose 최대 {pose:.3f} mm, episode.json base {ep.get('base')}, "
                    f"리셋 응답 base {(ep.get('sim_reset') or {}).get('base', {}).get('preset')}")
    item("3 A static: 메타데이터·base_imu 상수·base_pose 원점", ok3, " | ".join(rows))
    rows = []
    ok4 = True
    for b, r in res_b.items():
        acc_rng = float(np.ptp(r["imu"][:, 3:], axis=0).max())
        fz = float(r["imu"][:, 5].mean())
        move = float(np.ptp(r["pose"][:, :3], axis=0).max()) * 1000
        bp = r["meta"]["base_pose"]
        good = acc_rng > 0.01 and abs(fz - G) < 0.05 and move > 5.0 and bp["frame"] == "odom" and bp["child_frame"] == CHILD[args.base]
        ok4 &= good
        rows.append(f"{b}: 프레임 {len(r['imu'])}, 비력 범위 최대 {acc_rng:.4f} m/s², 비력 z 평균 {fz:.4f}, 각속도 범위 "
                    f"{np.round(np.ptp(r['imu'][:, :3], axis=0), 4).tolist()}, base_pose 이동 범위 {move:.1f} mm, frame ({bp['frame']}, {bp['child_frame']}), "
                    f"IMU {r['meta']['base_imu_stats']['hz_median']} Hz")
    item("4 B 흔들림: base_imu·base_pose 가 변함", ok4, " | ".join(rows))
    log("")
    for ln in inspect_txt.splitlines():
        if ln.startswith("[") or " | " in ln or ln.startswith("병합 기록"):
            log("  " + ln)

    if args.cleanup:
        for b in made:
            for d in (os.path.join(rt.BAG_DIR, b), os.path.join(rt.DISCARD_DIR, b), os.path.join(REPO, "bag_lerobot_intermediate", b)):
                if os.path.isdir(d):
                    shutil.rmtree(d)
                    log(f"지움: {d}")
    else:
        log(f"남김 (bag·중간 파일): {made}")
    n_ok = sum(results)
    log(f"\n  {n_ok}/{len(results)} PASS   데이터셋 {root}   리포트 {out_dir}")
    with open(os.path.join(out_dir, "report.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(report) + "\n")
    sys.exit(0 if n_ok == len(results) else 1)


if __name__ == "__main__":
    main()
