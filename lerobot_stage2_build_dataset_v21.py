#!/usr/bin/env python3
# =============================================================
# lerobot_stage2_build_dataset_v21.py  [2단계 — conda(lerobot_v2) 환경에서 실행]
#
# 1단계(lerobot_stage1_extract_bag.py)가 만든 중간 파일 하나를 읽어서
# LeRobotDataset 으로 조립한다. (lerobot 0.3.3 기준 → CODEBASE_VERSION "v2.1")
# 에피소드 여러 개를 하나로 합치려면 lerobot_merge_episodes_v21.py (같은 공용 코드 lerobot_v21_common.py).
#
# lerobot 0.3.3 API 주의 (v3.0 / 0.6.1 과 다른 점):
#   1) add_frame: 'task'를 frame dict에서 빼서 인자로 넘김
#   2) finalize() 없음. save_episode()가 인코딩까지 끝냄
#   3) 시작 시 CODEBASE_VERSION 검증 (v2.1 이 아니면 중단)
#
# 입력(중간 파일, schema_version 2):  <intermediate>/<bag이름>/
#     states.npy (N, 7), actions.npy (N, 7), base_imu.npy (N, 6), timestamps.npy (N,)
#     wrist/000000.png ..., third_view/000000.png ..., meta.json
#
# 출력: LeRobotDataset (v2.1 형식; meta/info.json 의 codebase_version == "v2.1")
#     기본 저장 위치: <intermediate 상위>/lerobot_dataset_v21/<repo_id>
#     observation.state (7,)   = 팔 6관절 (rad) + 그리퍼 present / 1150 (0~1)
#     action (7,)              = 팔 6관절 목표 (rad) + 그리퍼 goal / 1150 ({0, 1})
#     observation.base_imu (6,) = 각속도 xyz + 선가속도 xyz
#     observation.images.wrist / observation.images.third_view (480,640,3)
#     task = "pick up the drone" (--task 로 변경)
#
# 실행:
#   conda activate lerobot_v2
#   python lerobot_stage2_build_dataset_v21.py <중간파일폴더> \
#       [--repo-id taeunglee/ur5_gripper_drone] \
#       [--task "pick up the drone"] [--root <출력폴더>]
#
#   # ROS2 PYTHONPATH를 확실히 배제하고 싶으면:
#   #   env -u PYTHONPATH python lerobot_stage2_build_dataset_v21.py ...
# =============================================================

import os
import sys
import argparse

from lerobot.datasets.lerobot_dataset import LeRobotDataset, CODEBASE_VERSION

import lerobot_v21_common as common


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('in_dir', help='1단계 중간 파일 폴더 (states.npy 등이 있는 곳)')
    ap.add_argument('--repo-id', default='taeunglee/ur5_gripper_drone',
                    help="데이터셋 repo id ('user/name' 형식)")
    ap.add_argument('--task', default='pick up the drone',
                    help='언어 명령 (에피소드 task)')
    ap.add_argument('--root', default=None,
                    help='데이터셋 저장 폴더 (기본: 중간폴더 상위/lerobot_dataset_v21/<repo_id>)')
    args = ap.parse_args()

    print(f"lerobot CODEBASE_VERSION = {CODEBASE_VERSION}")
    if CODEBASE_VERSION != 'v2.1':
        print(f"[오류] 이 환경의 lerobot은 v2.1이 아니라 '{CODEBASE_VERSION}' 형식을 만든다.")
        print("       lerobot 0.3.3 이 설치된 lerobot_v2 환경에서 실행하세요.")
        sys.exit(1)

    in_dir = os.path.abspath(os.path.expanduser(args.in_dir)).rstrip('/')
    if not os.path.isdir(in_dir):
        print(f"[오류] 폴더가 아님: {in_dir}")
        sys.exit(1)
    try:
        info = common.check_episode(os.path.basename(in_dir), in_dir)
    except ValueError as e:
        print(f"[오류] {in_dir}: {e}")
        sys.exit(1)
    fps = int(round(info['fps']))

    print("=== 2단계: LeRobotDataset 생성 (v2.1) ===")
    print(f"입력: {in_dir}")
    print(f"프레임 수: {info['N']}, fps: {fps}, 이미지 {info['hwc']}, 팔 action: {info['arm_action']}, base_imu: {info['base_imu']}")
    print(f"task: {args.task!r}")

    if args.root:
        root = os.path.expanduser(args.root)
    else:
        parent = os.path.dirname(in_dir)                 # .../bag_lerobot_intermediate
        collection = os.path.dirname(parent)             # .../data_collection_ur5_gripper
        root = os.path.join(collection, 'lerobot_dataset_v21', args.repo_id)
    print(f"출력: {root}")
    if os.path.exists(root) and os.listdir(root):
        print(f"\n[오류] 출력 폴더가 이미 있고 비어있지 않음:\n  {root}")
        print("기존 걸 지우거나 --repo-id / --root 로 다른 위치를 지정하세요.")
        sys.exit(1)

    dataset = LeRobotDataset.create(repo_id=args.repo_id, fps=fps, features=common.make_features(info),
                                    root=root, robot_type='ur5', use_videos=True)
    print("프레임 추가 중...")
    common.add_episode(dataset, info, args.task)

    print("\n-- 완료 --")
    print(f"데이터셋 위치: {root}")
    print(f"  에피소드 1개, 프레임 {info['N']}개, fps {fps}")
    print(f"  features: {', '.join(common.make_features(info))}")
    print(f"검수: python3 inspect_dataset_v21.py {root}")


if __name__ == '__main__':
    main()
