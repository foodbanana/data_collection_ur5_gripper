#!/usr/bin/env python3
# =============================================================
# lerobot_stage2_build_dataset_v30.py  [2단계 — conda(lerobot_v1) 환경에서 실행]
#
# 1단계(lerobot_stage1_extract_bag.py)가 만든 중간 파일을 읽어서
# LeRobotDataset 으로 조립한다. (lerobot 0.6.1 기준)
#
# 입력(중간 파일):  <intermediate>/<bag이름>/
#     states.npy       (N, 7)  = [팔6관절(rad) + 그리퍼1 present(raw 0~1150)]
#     actions.npy      (N, 7)  = [팔6관절 states[i+1](rad) + 그리퍼1 target[i](raw {0,1150})]
#     timestamps.npy   (N,)
#     head/000000.png ...      (d456)
#     third_view/000000.png ...(d435i)
#     meta.json
#
# 출력: LeRobotDataset (기본 v3.0 형식; 필요시 나중에 v2.1로 변환)
#     기본 저장 위치: <intermediate 상위>/lerobot_dataset_v30/<repo_id>
#
# 설계:
#     observation.state (7,)          = states[i]
#     action (7,)                     = actions[i]   (1단계에서 완성: 팔=states[i+1], 그리퍼=/gripper/target 의 target[i])
#     observation.images.head (480,640,3)       = d456
#     observation.images.third_view (480,640,3) = d435i
#     task = "pick up the drone" (--task 로 변경)
#
# 실행:
#   conda activate lerobot_v1
#   python lerobot_stage2_build_dataset_v30.py <중간파일폴더> \
#       [--repo-id taeunglee/ur5_gripper_drone] \
#       [--task "pick up the drone"] \
#       [--fps 25] [--root <출력폴더>]
# 예:
#   python lerobot_stage2_build_dataset_v30.py \
#     ~/data_collection_ur5_gripper/bag_lerobot_intermediate/20260916_141816_test_1_fake_node \
#     --task "pick up the drone"
# =============================================================

import os
import sys
import json
import argparse

import numpy as np
from PIL import Image

from lerobot.datasets.lerobot_dataset import LeRobotDataset


def load_intermediate(in_dir):
    """1단계 중간 파일을 읽어 반환."""
    states = np.load(os.path.join(in_dir, 'states.npy'))        # (N, 7)
    actions_path = os.path.join(in_dir, 'actions.npy')
    if not os.path.isfile(actions_path):
        # 옛 1단계 출력(states[i+1] 방식)으로 조용히 대체하지 않음
        print(f"[오류] actions.npy 가 없음: {in_dir}")
        print("       최신 lerobot_stage1_extract_bag.py 로 1단계를 다시 실행하세요.")
        sys.exit(1)
    actions = np.load(actions_path)                             # (N, 7)
    if actions.shape[0] != states.shape[0]:
        print(f"[오류] states({states.shape}) 와 actions({actions.shape}) 프레임 수가 다름.")
        sys.exit(1)
    stamps = np.load(os.path.join(in_dir, 'timestamps.npy'))    # (N,)
    with open(os.path.join(in_dir, 'meta.json')) as f:
        meta = json.load(f)
    return states, actions, stamps, meta


def read_png(path):
    """png 를 (H, W, 3) uint8 RGB numpy 로 읽기.
    1단계에서 cv2(BGR)로 저장했으므로 PIL 로 열면 RGB 로 바르게 읽힌다
    (cv2.imwrite 는 BGR 배열을 파일에는 표준 RGB 로 저장하기 때문)."""
    img = Image.open(path).convert('RGB')
    return np.asarray(img, dtype=np.uint8)   # (H, W, 3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('in_dir', help='1단계 중간 파일 폴더 (states.npy 등이 있는 곳)')
    ap.add_argument('--repo-id', default='taeunglee/ur5_gripper_drone',
                    help="데이터셋 repo id ('user/name' 형식)")
    ap.add_argument('--task', default='pick up the drone',
                    help='언어 명령 (에피소드 task)')
    ap.add_argument('--fps', type=int, default=None,
                    help='fps (기본: meta.json 값 사용)')
    ap.add_argument('--root', default=None,
                    help='데이터셋 저장 폴더 (기본: 중간폴더 상위/lerobot_dataset_v30/<repo_id>)')
    args = ap.parse_args()

    in_dir = os.path.expanduser(args.in_dir).rstrip('/')
    if not os.path.isdir(in_dir):
        print(f"폴더가 아님: {in_dir}")
        sys.exit(1)

    states, actions, stamps, meta = load_intermediate(in_dir)
    N = states.shape[0]
    state_dim = states.shape[1]
    action_dim = actions.shape[1]
    fps = args.fps if args.fps is not None else int(round(meta.get('fps', 25)))

    print(f"=== 2단계: LeRobotDataset 생성 ===")
    print(f"입력: {in_dir}")
    print(f"프레임 수: {N}, state 차원: {state_dim}, fps: {fps}")
    print(f"task: {args.task!r}")

    head_dir = os.path.join(in_dir, 'head')
    third_dir = os.path.join(in_dir, 'third_view')

    # 이미지 크기 확인 (첫 장으로)
    first_head = read_png(os.path.join(head_dir, '000000.png'))
    H, W, C = first_head.shape
    print(f"이미지 크기: {H}x{W}x{C}")

    # ── 저장 위치 ──
    if args.root:
        root = os.path.expanduser(args.root)
    else:
        parent = os.path.dirname(in_dir)                 # .../bag_lerobot_intermediate
        collection = os.path.dirname(parent)             # .../data_collection_ur5_gripper
        root = os.path.join(collection, 'lerobot_dataset_v30', args.repo_id)
    print(f"출력: {root}")

    # 이미 존재하면 경고 (create 는 기존 폴더에 덮어쓰기 안 함)
    if os.path.exists(root) and os.listdir(root):
        print(f"\n[주의] 출력 폴더가 이미 있고 비어있지 않음:\n  {root}")
        print("기존 걸 지우거나 --repo-id / --root 로 다른 위치를 지정하세요.")
        sys.exit(1)

    # ── feature 정의 ──
    state_names = meta.get('state_names',
                           ['shoulder_pan','shoulder_lift','elbow',
                            'wrist_1','wrist_2','wrist_3','gripper'])
    features = {
        'observation.state': {
            'dtype': 'float32',
            'shape': (state_dim,),
            'names': state_names,
        },
        'action': {
            'dtype': 'float32',
            'shape': (action_dim,),
            'names': meta.get('action_names', state_names),
        },
        'observation.images.head': {
            'dtype': 'video',
            'shape': (H, W, C),
            'names': ['height', 'width', 'channels'],
        },
        'observation.images.third_view': {
            'dtype': 'video',
            'shape': (H, W, C),
            'names': ['height', 'width', 'channels'],
        },
    }

    # ── 데이터셋 생성 ──
    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=fps,
        features=features,
        root=root,
        robot_type='ur5',
        use_videos=True,
    )

    # ── 프레임 추가 ──
    #   action 은 1단계가 완성한 actions.npy 를 그대로 사용
    #   (팔=states[i+1], 그리퍼=target[i]. 여기서 다시 유도하지 않음)
    print("프레임 추가 중...")
    for i in range(N):
        state_i = states[i].astype(np.float32)
        action_i = actions[i].astype(np.float32)

        head_img = read_png(os.path.join(head_dir, f'{i:06d}.png'))
        third_img = read_png(os.path.join(third_dir, f'{i:06d}.png'))

        dataset.add_frame({
            'observation.state': state_i,
            'action': action_i,
            'observation.images.head': head_img,
            'observation.images.third_view': third_img,
            'task': args.task,
        })

        if (i + 1) % 100 == 0 or i == N - 1:
            print(f"  {i+1}/{N}")

    # ── 에피소드 저장 + 마무리 ──
    print("에피소드 저장(비디오 인코딩) 중...")
    dataset.save_episode()
    dataset.finalize()

    print("\n-- 완료 --")
    print(f"데이터셋 위치: {root}")
    print(f"  에피소드 1개, 프레임 {N}개, fps {fps}")
    print(f"  features: observation.state(7), action(7), "
          f"observation.images.head, observation.images.third_view")
    print(f"  task: {args.task!r}")
    print(f"\n확인: meta/info.json 의 codebase_version 을 보면 형식(v3.0 등)을 알 수 있음.")


if __name__ == '__main__':
    main()
