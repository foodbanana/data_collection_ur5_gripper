#!/usr/bin/env python3
# =============================================================
# lerobot_stage1_extract_bag.py   [1단계 — ROS2 환경에서 실행]
#
# ros2 bag(mcap) 을 읽어서, 25Hz(기본)로 리샘플링한 중간 파일로 추출한다.
# 2단계(lerobot_stage2_build_dataset.py, conda 환경)가 이 중간 파일을
# LeRobotDataset v2.1 로 만든다.
#
# 2단계로 나눈 이유:
#   - rosbag2_py 는 ROS2(시스템 파이썬)에만 있음
#   - lerobot 은 conda 환경에만 있음
#   - 한 스크립트에서 둘 다 import 하면 충돌 → 분리
#
# 동기화 방식:
#   - 각 토픽의 header.stamp(센서 생성 시각) 기준
#   - 공통 구간을 25Hz 격자로 나누고, 각 격자 시점마다
#     "그 시점 이전의 가장 최근 메시지"(zero-order hold)를 선택
#
# action 생성 (여기 한 곳에서 완성 → 2단계 v21/v30 은 actions.npy 를 그대로 사용):
#   - 팔 6관절  : states[i+1] 의 팔 (freedrive 시연이라 명령 신호가 없어 다음 프레임에서 유도)
#                 마지막 프레임은 states[N-1] 의 팔 복제
#   - 그리퍼    : /gripper/target 의 target[i] (프레임 i 시점의 실제 goal 명령, raw {0, 1150})
#   ※ /gripper/target 이 없는 bag(옛 그리퍼 노드로 수집)은 에러로 중단. fallback 없음.
#
# 출력(중간 파일):  <데이터수집폴더>/bag_lerobot_intermediate/<bag이름>/
#     states.npy       (N, 7)  = [팔6관절(rad) + 그리퍼1 present(raw 0~1150)]   ← observation
#     actions.npy      (N, 7)  = [팔6관절 states[i+1](rad) + 그리퍼1 target[i](raw)] ← action
#     timestamps.npy   (N,)    = 각 프레임 기준 시각(초)
#     head/000000.png ...      = d456  (손목)
#     third_view/000000.png ...= d435i (외부 고정)
#     meta.json                = fps, 프레임수, 관절이름 등
#
# 실행:
#   source /opt/ros/jazzy/setup.bash
#   python3 lerobot_stage1_extract_bag.py <bag폴더> [--fps 25] [--out <출력폴더>]
# 예:
#   python3 lerobot_stage1_extract_bag.py \
#     ~/data_collection_ur5_gripper/bags/20260916_141816_test_1_fake_node --fps 25
# =============================================================

import os
import sys
import json
import argparse

import numpy as np

import rclpy.serialization
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

from cv_bridge import CvBridge
import cv2


# ── 토픽 이름 ──
TOPIC_ARM   = '/joint_states'
TOPIC_GRIP  = '/gripper/joint_states'          # 그리퍼 present (observation)
TOPIC_GRIP_TARGET = '/gripper/target'          # 그리퍼 goal, 30Hz 상시 발행 (action)
TOPIC_HEAD  = '/d456/d456/color/image_raw'    # head camera (손목)
TOPIC_THIRD = '/d435i/d435i/color/image_raw'  # third view (외부 고정)

# ── UR5 관절 순서(이 순서로 state 앞 6칸을 채움) ──
UR5_JOINT_ORDER = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# ── 중간 파일 출력 폴더 이름 (데이터수집 폴더 바로 아래) ──
INTERMEDIATE_DIRNAME = 'bag_lerobot_intermediate'


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def open_reader(bag_path):
    for storage_id in ('mcap', 'sqlite3'):
        try:
            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id),
                rosbag2_py.ConverterOptions(
                    input_serialization_format='cdr',
                    output_serialization_format='cdr'),
            )
            return reader
        except Exception:
            continue
    raise RuntimeError(f"bag을 열 수 없음: {bag_path}")


def load_all_messages(bag_path):
    reader = open_reader(bag_path)
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    msg_cls = {}
    def cls_of(topic):
        typ = type_map[topic]
        if typ not in msg_cls:
            msg_cls[typ] = get_message(typ)
        return msg_cls[typ]

    wanted = {TOPIC_ARM, TOPIC_GRIP, TOPIC_GRIP_TARGET, TOPIC_HEAD, TOPIC_THIRD}
    buckets = {t: [] for t in wanted}

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in wanted:
            continue
        msg = rclpy.serialization.deserialize_message(data, cls_of(topic))
        if hasattr(msg, 'header'):
            ts = stamp_to_sec(msg.header.stamp)
        else:
            ts = t_ns * 1e-9
        buckets[topic].append((ts, msg))

    for t in buckets:
        buckets[t].sort(key=lambda x: x[0])
    return buckets


def latest_at(sorted_times, query_t):
    idx = np.searchsorted(sorted_times, query_t, side='right') - 1
    if idx < 0:
        idx = 0
    return idx


def reorder_arm(msg):
    name_to_pos = dict(zip(msg.name, msg.position))
    out = []
    for j in UR5_JOINT_ORDER:
        if j not in name_to_pos:
            raise KeyError(f"관절 '{j}' 가 /joint_states 에 없음. 실제: {list(msg.name)}")
        out.append(float(name_to_pos[j]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag_path', help='ros2 bag 폴더 경로')
    ap.add_argument('--fps', type=float, default=25.0, help='목표 fps (기본 25)')
    ap.add_argument('--out', default=None,
                    help=f'출력 상위 폴더 (기본: 데이터수집폴더/{INTERMEDIATE_DIRNAME}/)')
    args = ap.parse_args()

    bag_path = os.path.expanduser(args.bag_path).rstrip('/')
    if not os.path.isdir(bag_path):
        print(f"폴더가 아님: {bag_path}")
        sys.exit(1)

    bag_name = os.path.basename(bag_path)

    # 기본 출력 위치: bag 이 있는 곳(.../bags/)의 부모 = 데이터수집 폴더, 그 아래 intermediate
    if args.out:
        out_root = os.path.expanduser(args.out)
    else:
        bags_dir = os.path.dirname(bag_path)                 # .../data_collection_ur5_gripper/bags
        collection_dir = os.path.dirname(bags_dir)           # .../data_collection_ur5_gripper
        out_root = os.path.join(collection_dir, INTERMEDIATE_DIRNAME)

    out_dir = os.path.join(out_root, bag_name)
    head_dir = os.path.join(out_dir, 'head')
    third_dir = os.path.join(out_dir, 'third_view')

    print(f"=== 추출: {bag_name} ===")
    print(f"목표 fps: {args.fps}")
    print(f"출력 위치: {out_dir}")
    print("bag 읽는 중...")
    buckets = load_all_messages(bag_path)

    n_arm = len(buckets[TOPIC_ARM])
    n_grip = len(buckets[TOPIC_GRIP])
    n_target = len(buckets[TOPIC_GRIP_TARGET])
    n_head = len(buckets[TOPIC_HEAD])
    n_third = len(buckets[TOPIC_THIRD])
    print(f"  /joint_states        : {n_arm}")
    print(f"  /gripper/joint_states: {n_grip}")
    print(f"  /gripper/target      : {n_target}")
    print(f"  head (d456)          : {n_head}")
    print(f"  third_view (d435i)   : {n_third}")

    # 그리퍼 action 소스가 없으면 중단 (states[i+1] 로 조용히 대체하지 않음)
    if n_target == 0:
        print("[오류] 이 bag에는 /gripper/target이 없습니다. "
              "새 그리퍼 노드로 다시 수집한 bag을 사용하세요.")
        sys.exit(1)

    if min(n_arm, n_grip, n_head, n_third) == 0:
        print("토픽 중 하나가 비어있음. 중단.")
        sys.exit(1)

    os.makedirs(head_dir, exist_ok=True)
    os.makedirs(third_dir, exist_ok=True)

    arm_t   = [t for t, _ in buckets[TOPIC_ARM]]
    grip_t  = [t for t, _ in buckets[TOPIC_GRIP]]
    target_t = [t for t, _ in buckets[TOPIC_GRIP_TARGET]]
    head_t  = [t for t, _ in buckets[TOPIC_HEAD]]
    third_t = [t for t, _ in buckets[TOPIC_THIRD]]

    t_start = max(arm_t[0], grip_t[0], target_t[0], head_t[0], third_t[0])
    t_end   = min(arm_t[-1], grip_t[-1], target_t[-1], head_t[-1], third_t[-1])
    if t_end <= t_start:
        print("공통 시간 구간이 없음. 중단.")
        sys.exit(1)

    dt = 1.0 / args.fps
    grid = np.arange(t_start, t_end, dt)
    print(f"공통 구간: {t_end - t_start:.2f}s -> {len(grid)} 프레임 @ {args.fps}Hz")

    arm_ts_np   = np.asarray(arm_t, dtype=np.float64)
    grip_ts_np  = np.asarray(grip_t, dtype=np.float64)
    target_ts_np = np.asarray(target_t, dtype=np.float64)
    head_ts_np  = np.asarray(head_t, dtype=np.float64)
    third_ts_np = np.asarray(third_t, dtype=np.float64)

    bridge = CvBridge()
    states = []
    grip_targets = []
    stamps = []

    for i, qt in enumerate(grid):
        ai = latest_at(arm_ts_np, qt)
        arm_pos = reorder_arm(buckets[TOPIC_ARM][ai][1])

        gi = latest_at(grip_ts_np, qt)
        grip_msg = buckets[TOPIC_GRIP][gi][1]
        grip_pos = float(grip_msg.position[0]) if len(grip_msg.position) > 0 else 0.0

        # 그리퍼 goal (present 와 같은 tick/같은 stamp 로 발행되므로 같은 방식으로 샘플링)
        gti = latest_at(target_ts_np, qt)
        target_msg = buckets[TOPIC_GRIP_TARGET][gti][1]
        if len(target_msg.position) == 0:
            print(f"[오류] /gripper/target 메시지에 position 이 비어있음 (프레임 {i}). 중단.")
            sys.exit(1)
        grip_targets.append(float(target_msg.position[0]))

        states.append(arm_pos + [grip_pos])
        stamps.append(float(qt))

        hi = latest_at(head_ts_np, qt)
        ti = latest_at(third_ts_np, qt)
        head_img = bridge.imgmsg_to_cv2(buckets[TOPIC_HEAD][hi][1],
                                        desired_encoding='bgr8')
        third_img = bridge.imgmsg_to_cv2(buckets[TOPIC_THIRD][ti][1],
                                         desired_encoding='bgr8')
        cv2.imwrite(os.path.join(head_dir,  f'{i:06d}.png'), head_img)
        cv2.imwrite(os.path.join(third_dir, f'{i:06d}.png'), third_img)

        if (i + 1) % 50 == 0 or i == len(grid) - 1:
            print(f"  프레임 {i+1}/{len(grid)}")

    states = np.asarray(states, dtype=np.float32)
    stamps = np.asarray(stamps, dtype=np.float64)

    # ── action (N, 7) 완성 ──
    #   팔 6관절 : states[i+1] 의 팔. 마지막 프레임은 states[N-1] 의 팔 복제
    #   그리퍼   : target[i] (raw 그대로, 정규화 안 함)
    actions = np.empty_like(states)
    actions[:-1, :6] = states[1:, :6]
    actions[-1, :6] = states[-1, :6]
    actions[:, 6] = np.asarray(grip_targets, dtype=np.float32)

    np.save(os.path.join(out_dir, 'states.npy'), states)
    np.save(os.path.join(out_dir, 'actions.npy'), actions)
    np.save(os.path.join(out_dir, 'timestamps.npy'), stamps)

    meta = {
        'bag_name': bag_name,
        'fps': args.fps,
        'num_frames': int(states.shape[0]),
        'state_dim': int(states.shape[1]),
        'state_names': UR5_JOINT_ORDER + ['gripper'],
        'action_dim': int(actions.shape[1]),
        'action_names': UR5_JOINT_ORDER + ['gripper'],
        'gripper_action_source': TOPIC_GRIP_TARGET,
        'gripper_raw_range': [0, 1150],
        'image_keys': {
            'head': 'd456 (wrist)',
            'third_view': 'd435i (external fixed)',
        },
        'image_size': [480, 640, 3],
        'note': 'state=[arm6(rad)+gripper1 present(raw)]; action: 팔=states[i+1], 그리퍼=target[i]',
    }
    with open(os.path.join(out_dir, 'meta.json'), 'w') as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    print("\n-- 추출 완료 --")
    print(f"출력 폴더: {out_dir}")
    print(f"  states.npy      : {states.shape}")
    print(f"  actions.npy     : {actions.shape}")
    print(f"  timestamps.npy  : {stamps.shape}")
    print(f"  head/*.png      : {len(grid)} 장")
    print(f"  third_view/*.png: {len(grid)} 장")
    print(f"  meta.json")
    print(f"\nstate  예시(첫 프레임): {states[0].tolist()}")
    print(f"action 예시(첫 프레임): {actions[0].tolist()}")
    print(f"그리퍼 action 값 종류 : {sorted(set(actions[:, 6].tolist()))}")


if __name__ == '__main__':
    main()