#!/usr/bin/env python3
# =============================================================
# lerobot_merge_episodes_v21.py  [conda(lerobot_v2) 환경에서 실행]
#
# 여러 개의 1단계 중간 파일(bag_lerobot_intermediate/<bag>/)을 매니페스트로 받아
# LeRobotDataset 하나로 병합한다. (lerobot 0.3.3 -> CODEBASE_VERSION "v2.1")
#
# 방식: 병합 대상들의 parquet/mp4를 이어붙이는 대신,
#       중간 파일에서 데이터셋 하나를 create() 하고
#       에피소드마다 add_frame -> save_episode 를 반복한다.
#       episode_index/index/task_index/stats/info.json 총계는 lerobot 이 계산.
#
# 스키마·검사·프레임 추가는 lerobot_stage2_build_dataset_v21.py 와 같은 공용 코드 (lerobot_v21_common.py).
#   1) 입력: 매니페스트(폴더 목록) 또는 --all (중간 파일 폴더 전체)
#   2) create() 는 한 번, save_episode() 는 에피소드마다
#   3) create 전에 전 에피소드 사전검사 (실패 시 즉시 중단, 조용한 fallback 없음).
#      fps·이미지 크기·팔 action 종류 (command / next_state)·base_imu 방식이 에피소드끼리 다르면 중단
#   4) 보호 정지가 걸린 에피소드는 기본 제외 (--include-protective-stop 으로 포함). 제외한 것은 출력과 merge_manifest.json 에 남김
#   5) 앞뒤 정지 구간 자르기는 명시 옵션 --trim-idle start|end|both (기본 안 자름).
#      정지 = action (팔·그리퍼 명령) 과 state (측정값) 가 모두 첫 / 마지막 프레임 값에서 그대로인 구간 (팔 --idle-eps-deg). --trim-margin [s] 만큼은 남김
#   6) meta/merge_manifest.json 에 어떤 폴더가 어떤 episode_index 로 갔는지, 자른 프레임, 에피소드 요약 기록
#
# 실행:
#   conda activate lerobot_v2
#   python lerobot_merge_episodes_v21.py \
#       --manifest merge_ur5_gripper_drone.txt   (또는 --all) \
#       [--intermediate-dir bag_lerobot_intermediate] \
#       [--repo-id foodbanana/ur5_gripper_drone] \
#       [--task "pick up the drone"] [--root <출력폴더>] \
#       [--include-protective-stop] [--trim-idle start|end|both] [--trim-margin 0.5] [--idle-eps-deg 0.1]
# =============================================================

import os
import sys
import json
import argparse
from datetime import datetime, timezone

import numpy as np

from lerobot.datasets.lerobot_dataset import LeRobotDataset, CODEBASE_VERSION

import lerobot_v21_common as common


def parse_manifest(path):
    """매니페스트에서 폴더 이름 목록 읽기. '#' 주석/빈 줄 무시."""
    names = []
    with open(path) as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith('#'):
                continue
            names.append(line)
    if not names:
        print(f"[오류] 매니페스트에 유효한 폴더가 하나도 없음: {path}")
        sys.exit(1)
    return names


def main():
    ap = argparse.ArgumentParser()
    src = ap.add_mutually_exclusive_group()
    src.add_argument('--manifest', default=None, help='병합 대상 폴더 목록 파일 (기본 merge_ur5_gripper_drone.txt)')
    src.add_argument('--all', action='store_true', help='중간 파일 폴더 아래 전부 (매니페스트 대신)')
    ap.add_argument('--intermediate-dir', default='bag_lerobot_intermediate',
                    help='중간 파일들이 모여있는 폴더')
    ap.add_argument('--repo-id', default='foodbanana/ur5_gripper_drone',
                    help="병합 데이터셋 repo id ('user/name')")
    ap.add_argument('--task', default='pick up the drone',
                    help='언어 명령 (모든 에피소드 공통)')
    ap.add_argument('--root', default=None,
                    help='출력 폴더 (기본: <intermediate상위>/lerobot_dataset_v21/<repo_id>)')
    ap.add_argument('--include-protective-stop', action='store_true',
                    help='보호 정지가 걸린 에피소드도 넣는다 (기본 제외)')
    ap.add_argument('--trim-idle', choices=('start', 'end', 'both'), default=None,
                    help='앞 / 뒤 / 양쪽 정지 구간을 자른다 (기본 안 자름)')
    ap.add_argument('--trim-margin', type=float, default=0.5, help='[s] 자를 때 남길 정지 구간 (기본 0.5)')
    ap.add_argument('--idle-eps-deg', type=float, default=0.1, help='[deg] 팔 명령이 이만큼 안 변하면 정지 (기본 0.1)')
    args = ap.parse_args()

    # ── v2.1 형식 확인 ──
    print(f"lerobot CODEBASE_VERSION = {CODEBASE_VERSION}")
    if CODEBASE_VERSION != 'v2.1':
        print(f"[오류] 이 환경 lerobot 은 v2.1 이 아니라 '{CODEBASE_VERSION}'. "
              f"lerobot 0.3.3(lerobot_v2)에서 실행하세요.")
        sys.exit(1)

    intermediate_root = os.path.abspath(os.path.expanduser(args.intermediate_dir))
    if not os.path.isdir(intermediate_root):
        print(f"[오류] 중간 파일 폴더 없음: {intermediate_root}")
        sys.exit(1)
    if args.all:
        names = [n for n in os.listdir(intermediate_root) if os.path.isdir(os.path.join(intermediate_root, n))]
        if not names:
            print(f"[오류] 중간 파일 폴더가 비어 있음: {intermediate_root}")
            sys.exit(1)
        source = '--all'
    else:
        manifest = os.path.expanduser(args.manifest or 'merge_ur5_gripper_drone.txt')
        if not os.path.isfile(manifest):
            print(f"[오류] 매니페스트 파일 없음: {manifest}")
            sys.exit(1)
        names = parse_manifest(manifest)
        source = manifest
    # 중복 제거 후 이름순 정렬 -> episode_index 결정적
    names = sorted(set(names))
    print(f"\n=== 병합 대상 {len(names)}개 (이름순, {source}) ===")
    for name in names:
        print(f"  - {name}")

    # ── Phase 1: 전 에피소드 사전검사 (실패 시 즉시 중단) ──
    print("\n=== Phase 1: 사전 검사 ===")
    eps = np.radians(args.idle_eps_deg)
    ref, infos, excluded = None, [], []
    for name in names:
        d = os.path.join(intermediate_root, name)
        if not os.path.isdir(d):
            print(f"[중단] 중간 파일 폴더가 없음: {d}")
            sys.exit(1)
        try:
            info = common.check_episode(name, d, ref)
        except ValueError as e:
            print(f"[중단] {name}: {e}")
            print("       조용한 fallback 없음. 문제를 고치거나 매니페스트에서 제외 후 다시 실행.")
            sys.exit(1)
        ref = ref or {k: info[k] for k in ('fps', 'hwc', 'arm_action', 'base_imu')}
        lead, trail = common.idle_frames(info['data']['actions'], info['data']['states'], eps)
        info['idle'] = (lead, trail)
        note = f"프레임 {info['N']}, 정지 앞 {lead} / 뒤 {trail} 프레임, " \
               f"그리퍼 state {info['grip_state_range'][0]:.3f}~{info['grip_state_range'][1]:.3f}, " \
               f"파지 신호 {'있음' if info['grasp_signal'] else '없음'}"
        if info['protective_stop'] and not args.include_protective_stop:
            print(f"  [제외] {name}: 보호 정지가 걸린 에피소드 (넣으려면 --include-protective-stop). {note}")
            excluded.append({'source': name, 'reason': 'protective_stop'})
            continue
        print(f"  [OK] {name}: {note}" + (" [보호 정지]" if info['protective_stop'] else ""))
        infos.append(info)
    if not infos:
        print("[중단] 병합할 에피소드가 없음")
        sys.exit(1)

    # 자를 구간 [start, end)
    fps = int(round(ref['fps']))
    margin = int(round(args.trim_margin * fps))
    for info in infos:
        lead, trail = info['idle']
        start = max(0, lead - margin) if args.trim_idle in ('start', 'both') else 0
        end = info['N'] - max(0, trail - margin) if args.trim_idle in ('end', 'both') else info['N']
        if end - start < 2:
            print(f"[중단] {info['name']}: 자르고 나면 프레임이 {end - start} 개 (전 구간이 정지)")
            sys.exit(1)
        info['range'] = (start, end)
    total_frames = sum(i['range'][1] - i['range'][0] for i in infos)
    print(f"사전검사 통과. 에피소드 {len(infos)}개 (제외 {len(excluded)}개), 총 프레임 {total_frames}, fps {fps}, "
          f"팔 action {ref['arm_action']}, base_imu {ref['base_imu']}, 정지 구간 자르기 {args.trim_idle or '안 함'}")

    # ── 출력 위치 ──
    if args.root:
        root = os.path.expanduser(args.root)
    else:
        collection = os.path.dirname(intermediate_root)   # .../data_collection_ur5_gripper
        root = os.path.join(collection, 'lerobot_dataset_v21', args.repo_id)
    print(f"출력: {root}")
    if os.path.exists(root) and os.listdir(root):
        print(f"\n[중단] 출력 폴더가 이미 있고 비어있지 않음:\n  {root}")
        print("기존 걸 지우거나 --repo-id / --root 로 다른 위치를 지정하세요.")
        sys.exit(1)

    # ── Phase 2: 빌드 (create 1회, save_episode 는 에피소드마다) ──
    print("\n=== Phase 2: 병합 빌드 ===")
    dataset = LeRobotDataset.create(repo_id=args.repo_id, fps=fps, features=common.make_features(infos[0]),
                                    root=root, robot_type='ur5', use_videos=True)
    manifest_record = []
    for ep_idx, info in enumerate(infos):
        start, end = info['range']
        print(f"[{ep_idx}] {info['name']}  (프레임 {end - start}" + (f", 원래 {info['N']} 중 {start}~{end}" if (start, end) != (0, info['N']) else "") + ")")
        common.add_episode(dataset, info, args.task, start, end)
        manifest_record.append({
            'episode_index': ep_idx, 'source': info['name'], 'frames': end - start, 'source_frames': info['N'],
            'frame_range': [start, end], 'idle_frames': {'start': info['idle'][0], 'end': info['idle'][1]},
            'mode': info['mode'], 'grasp_signal': info['grasp_signal'], 'grip_state_max_while_closing': info['grip_state_max_while_closing'],
            'protective_stop': info['protective_stop'], 'joint_command': info['joint_command'], 'drone_kill': info['drone_kill'],
        })

    # ── Phase 3: 추적 기록 ──
    rec = {
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'repo_id': args.repo_id,
        'task': args.task,
        'fps': fps,
        'arm_action': ref['arm_action'], 'base_imu': ref['base_imu'],
        'trim_idle': args.trim_idle, 'trim_margin_s': args.trim_margin, 'idle_eps_deg': args.idle_eps_deg,
        'total_episodes': len(infos),
        'total_frames': total_frames,
        'episodes': manifest_record,
        'excluded': excluded,
    }
    meta_dir = os.path.join(root, 'meta')
    os.makedirs(meta_dir, exist_ok=True)
    with open(os.path.join(meta_dir, 'merge_manifest.json'), 'w') as f:
        json.dump(rec, f, indent=2, ensure_ascii=False)

    print("\n-- 완료 --")
    print(f"데이터셋 위치: {root}")
    print(f"  에피소드 {len(infos)}개, 총 프레임 {total_frames}, fps {fps}")
    print(f"  추적 기록: {os.path.join(meta_dir, 'merge_manifest.json')}")
    print(f"검수: python3 inspect_dataset_v21.py {root}")


if __name__ == '__main__':
    main()
