#!/usr/bin/env python3
# =============================================================
# make_fake_episodes.py  (docs/PLAN.md 4-5)
#
# 텔레옵 장치 없이 녹화 → 변환 → 병합 → 검수 파이프라인을 끝까지 돌려 4단계 완료 기준을 확인한다.
# 시스템 python3 (source /opt/ros/jazzy/setup.bash). sim_ros2.py --headless 를 직접 띄우고 (기하 제어기 드론, 리셋 위치 고정:
# 고정 명령 재생이 매번 잡히게), 사람이 record_toggle 로 하는 일을 같은 함수로 자동으로 한다.
#   에피소드 하나 = /sim/reset → 녹화 시작 → 명령 재생 (replay_commands.py) → (성공이면 드론 모터 정지) → 녹화 끝
#     success  : 드론 잡기 명령 (config/ros2_check/commands_success.csv) → 병합에 들어가야 함
#     pstop    : 테이블 충돌 명령 → 보호 정지가 걸려 meta.json 에 기록되고 병합에서 빠져야 함
#     discard  : 드론 잡기를 녹화한 뒤 bags/_discarded/ 로 버림 (사람이 d 키) → 변환하지 않으므로 병합에 안 들어감
#   그다음 남은 bag 마다 stage1 (--arm-action command --base-imu const) → 병합 (conda lerobot_v2) → inspect_dataset_v21.py --load
# 완료 기준 (PASS / FAIL, 하나라도 FAIL 이면 종료 코드 1):
#   1 가짜 에피소드 여러 개가 v2.1 데이터셋 하나로 병합됨 (성공 에피소드 수 = 데이터셋 에피소드 수, 보호 정지는 제외 기록, 버린 것은 없음)
#   2 meta/info.json: codebase_version v2.1, 카메라 키 2 개, observation.base_imu (6,)
#   3 검수 통과 (그리퍼 action {0, 1}, state 0~1, 이상치 0, 파지 신호, LeRobotDataset 로드)
#
# 실행:
#   source /opt/ros/jazzy/setup.bash
#   python3 isaacsim/scripts/make_fake_episodes.py [--success 3] [--pstop 1] [--discard 1] [--use-running-sim]
#   리포트·로그: isaacsim/reports/fake_episodes_<시각>/ , 데이터셋: lerobot_dataset_v21/local/fake_<시각>/
# =============================================================

import argparse
import csv
import datetime
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, REPO)
sys.path.insert(0, HERE)

import record_toggle as rt  # noqa: E402
from camera_config import load_cameras  # noqa: E402
from check_ros2 import ARM, GRASP_CSV, Sim, table_rows  # noqa: E402

CONDA_SH = os.path.expanduser("~/miniconda3/etc/profile.d/conda.sh")
CONDA_ENV = "lerobot_v2"
TASK_NAME = "fake"


def run(cmd, log_path, conda=False):
    """명령을 돌리고 출력을 로그 파일에. 실패하면 RuntimeError (로그 끝 포함). conda=True 면 lerobot_v2 환경 (ROS PYTHONPATH 없이)"""
    if conda:
        if not os.path.isfile(CONDA_SH):
            raise RuntimeError(f"conda 가 없음: {CONDA_SH}")
        cmd = ["bash", "-c", f'source "{CONDA_SH}" && conda activate {CONDA_ENV} && env -u PYTHONPATH python "$@"', "_", *cmd[1:]]
    with open(log_path, "w", encoding="utf-8") as f:
        r = subprocess.run(cmd, cwd=REPO, stdout=f, stderr=subprocess.STDOUT)
    if r.returncode != 0:
        tail = "".join(open(log_path, encoding="utf-8", errors="replace").readlines()[-8:])
        raise RuntimeError(f"{' '.join(cmd[:3])} … 실패 (종료 코드 {r.returncode}, 로그 {log_path})\n{tail}")


def wait_wall(link, sec):
    t0 = time.monotonic()
    while time.monotonic() - t0 < sec:
        link.spin(0.05)


def record_episode(kind, rec, link, csv_path, out_dir, k):
    """→ bag 폴더 이름. kind: success | pstop | discard"""
    if not rt.reset_then_record(rec, link):
        raise RuntimeError(f"{kind} {k}: 리셋·녹화 시작 실패")
    bag = os.path.basename(rec.out_dir)
    run(["python3", os.path.join(HERE, "replay_commands.py"), csv_path, "--tail", "1", "--out", os.path.join(out_dir, f"replay_{bag}.json")],
        os.path.join(out_dir, f"replay_{bag}.log"))
    if kind != "pstop":
        rec.kill_drone()                     # 잡은 뒤 드론 모터 정지 (실물 시나리오)
        wait_wall(link, 3.0)
    rec.stop()
    if kind == "discard":
        rec.discard_last()
    return bag


def main():
    ap = argparse.ArgumentParser(description="가짜 에피소드 → 병합된 v2.1 데이터셋 (4단계 완료 기준 확인)")
    ap.add_argument("--success", type=int, default=3, help="드론 잡기 에피소드 수")
    ap.add_argument("--pstop", type=int, default=1, help="테이블 충돌 (보호 정지) 에피소드 수")
    ap.add_argument("--discard", type=int, default=1, help="녹화 뒤 버리는 에피소드 수")
    ap.add_argument("--use-running-sim", action="store_true", help="sim 을 띄우지 않고 이미 떠 있는 sim_ros2.py 를 쓴다")
    ap.add_argument("--task", default="pick up the drone")
    args = ap.parse_args()
    if args.success < 2:
        sys.exit("--success 는 2 이상 (여러 에피소드 병합 확인)")

    stamp = f"{datetime.datetime.now():%Y%m%d_%H%M%S}"
    out_dir = os.path.join(REPO, "isaacsim", "reports", f"fake_episodes_{stamp}")
    os.makedirs(out_dir)
    report = []

    def log(msg):
        print(msg, flush=True)
        report.append(msg)

    # 테이블 충돌 명령 CSV (check_ros2 와 같은 명령)
    table_csv = os.path.join(out_dir, "commands_table.csv")
    with open(table_csv, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", *ARM, "gripper_goal_raw"])
        for t, q, g in table_rows():
            w.writerow([f"{t:.6f}", *[f"{x:.6f}" for x in q], f"{g:.6f}"])

    plan = [("success", GRASP_CSV)] * args.success + [("pstop", table_csv)] * args.pstop + [("discard", GRASP_CSV)] * args.discard
    log(f"make_fake_episodes {stamp}: 성공 {args.success}, 보호 정지 {args.pstop}, 버림 {args.discard}  (리포트 {out_dir})")

    sim, link, rec, bags = None, None, None, []
    try:
        if not args.use_running_sim:
            log("sim 띄우는 중 (헤드리스, 기하 제어기 드론, 리셋 위치 고정)...")
            sim = Sim(["--flight", "geometric", "--reset-offset", "0"], os.path.join(out_dir, "sim.log"))
            sim.wait_ready()
        os.makedirs(rt.BAG_DIR, exist_ok=True)
        link = rt.RosLink(True, load_cameras())
        rec = rt.BagRecorder(TASK_NAME, link)
        problems = link.check_topics(wait=30.0)
        if problems:
            raise RuntimeError(f"sim 토픽 문제: {problems}")
        for k, (kind, csv_path) in enumerate(plan):
            log(f"── 에피소드 {k + 1}/{len(plan)}: {kind}")
            bags.append((kind, record_episode(kind, rec, link, csv_path, out_dir, k)))
    finally:
        if rec is not None:
            if rec.recording:
                rec.stop()
            rec.close()
        if link is not None:
            link.close()
        if sim is not None:
            sim.stop()

    # ── 변환: 남은 bag 마다 stage1 ──
    kept = [(kind, b) for kind, b in bags if kind != "discard"]
    inter = os.path.join(REPO, "bag_lerobot_intermediate")
    for kind, b in kept:
        log(f"stage1: {b} ({kind})")
        run(["python3", os.path.join(REPO, "lerobot_stage1_extract_bag.py"), os.path.join(rt.BAG_DIR, b),
             "--arm-action", "command", "--base-imu", "const", "--overwrite"], os.path.join(out_dir, f"stage1_{b}.log"))
    metas = {b: json.load(open(os.path.join(inter, b, "meta.json"))) for _, b in kept}

    # ── 병합·검수 (conda lerobot_v2) ──
    manifest = os.path.join(out_dir, "merge_fake.txt")
    with open(manifest, "w", encoding="utf-8") as f:
        f.write("# make_fake_episodes.py 가 만든 병합 목록 (보호 정지 에피소드 포함: 병합이 빼야 함)\n" + "\n".join(b for _, b in kept) + "\n")
    repo_id = f"local/fake_{stamp}"
    root = os.path.join(REPO, "lerobot_dataset_v21", repo_id)
    log(f"병합: {len(kept)} 개 → {root}")
    run(["python", os.path.join(REPO, "lerobot_merge_episodes_v21.py"), "--manifest", manifest, "--intermediate-dir", inter,
         "--repo-id", repo_id, "--task", args.task], os.path.join(out_dir, "merge.log"), conda=True)
    inspect_log = os.path.join(out_dir, "inspect.log")
    inspect_ok = True
    try:
        run(["python", os.path.join(REPO, "inspect_dataset_v21.py"), root, "--load"], inspect_log, conda=True)
    except RuntimeError:
        inspect_ok = False
    inspect_txt = open(inspect_log, encoding="utf-8", errors="replace").read()

    # ── 완료 기준 ──
    info = json.load(open(os.path.join(root, "meta", "info.json")))
    mm = json.load(open(os.path.join(root, "meta", "merge_manifest.json")))
    want_in = sorted(b for kind, b in kept if kind == "success")
    want_out = sorted(b for kind, b in kept if kind == "pstop")
    got_in = sorted(e["source"] for e in mm["episodes"])
    got_out = sorted(e["source"] for e in mm["excluded"])
    discarded = [b for kind, b in bags if kind == "discard"]
    disc_ok = all(os.path.isdir(os.path.join(rt.DISCARD_DIR, b)) and not os.path.isdir(os.path.join(rt.BAG_DIR, b)) for b in discarded)
    ps_meta_ok = all(bool((metas[b].get("protective_stop") or {}).get("occurred")) == (kind == "pstop") for kind, b in kept)
    results = []

    def item(name, ok, detail):
        results.append(ok)
        log(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")

    log("")
    item("1 여러 에피소드가 v2.1 데이터셋 하나로 병합", got_in == want_in and got_out == want_out and info["total_episodes"] == len(want_in) and disc_ok and ps_meta_ok,
         f"데이터셋 에피소드 {info['total_episodes']} 개 ({info['total_frames']} 프레임) = 성공 {len(want_in)} 개, "
         f"보호 정지 {len(want_out)} 개는 meta.json 에 기록되고 병합에서 제외 ({got_out == want_out and ps_meta_ok}), 버린 {len(discarded)} 개는 _discarded 에 ({disc_ok})")
    feats = info["features"]
    cams = sorted(k for k, v in feats.items() if v["dtype"] == "video")
    item("2 meta/info.json", info["codebase_version"] == "v2.1" and cams == ["observation.images.third_view", "observation.images.wrist"]
         and feats.get("observation.base_imu", {}).get("shape") == [6],
         f"codebase_version {info['codebase_version']}, 카메라 키 {cams}, observation.base_imu {feats.get('observation.base_imu', {}).get('shape')}")
    item("3 검수 (inspect_dataset_v21.py --load)", inspect_ok, " / ".join(ln.split(":")[0] for ln in inspect_txt.splitlines() if ln.startswith("[")) or "출력 없음")
    log("")
    for ln in inspect_txt.splitlines():
        if ln.startswith("[") or " | " in ln or ln.startswith("병합 기록"):
            log("  " + ln)
    n_ok = sum(results)
    log(f"\n  {n_ok}/{len(results)} PASS   데이터셋 {root}   리포트 {out_dir}")
    with open(os.path.join(out_dir, "report.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(report) + "\n")
    sys.exit(0 if n_ok == len(results) else 1)


if __name__ == "__main__":
    main()
