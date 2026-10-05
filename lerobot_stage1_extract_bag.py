#!/usr/bin/env python3
# =============================================================
# lerobot_stage1_extract_bag.py   [1단계 — ROS2 환경에서 실행]   (sim·실물 공용, docs/PLAN.md 4-3)
#
# ros2 bag(mcap) 을 읽어서, 25Hz(기본)로 리샘플링한 중간 파일로 추출한다.
# 2단계(lerobot_stage2_build_dataset_v21.py, conda 환경)가 이 중간 파일을
# LeRobotDataset v2.1 로 만든다.
#
# 2단계로 나눈 이유:
#   - rosbag2_py 는 ROS2(시스템 파이썬)에만 있음
#   - lerobot 은 conda 환경에만 있음
#   - 한 스크립트에서 둘 다 import 하면 충돌 → 분리
#
# 입력: record_toggle.py 로 녹화한 bag 폴더 (episode.json 포함). 카메라 토픽은 config/cameras.yaml (/cam/<역할>/color/image_raw)
#
# 동기화 방식:
#   - 각 토픽의 header.stamp(센서 생성 시각. sim 은 sim time) 기준
#   - 공통 구간을 25Hz 격자로 나누고, 각 격자 시점마다
#     "그 시점 이전의 가장 최근 메시지"(zero-order hold)를 선택
#   - 고른 메시지가 격자 시각보다 --max-age (기본 66 ms = 카메라 2 프레임) 넘게 오래됐으면 에러
#     (이미지가 빠진 bag 에서 옛 이미지 + 새 관절 프레임이 조용히 만들어지는 것을 막는다)
#
# 데이터셋 스키마 (CLAUDE.md, 0단계 확정):
#   state  = [팔 6관절 측정값 (rad), 그리퍼 present / 1150 (0~1, 0 = 열림)]
#   action = [팔 6관절 목표 (rad), 그리퍼 goal / 1150 ({0.0, 1.0})]
#     팔 action 은 --arm-action 으로 정한다 (필수, 의미가 달라 섞으면 안 됨):
#       command    : /joint_command 의 그 시점 직전 최신 명령 (텔레옵·sim). stamp 는 보내는 쪽이 넣는다 (0 이면 에러).
#                    첫 명령 이전 구간은 에피소드에서 뺀다 (값을 지어내지 않음, 버린 프레임 수는 meta.json dropped_frames_at_start)
#       next_state : states[i+1] 의 팔 (freedrive 시연: 명령 신호가 없어 다음 프레임에서 유도). 마지막 프레임은 복제
#     그리퍼 action = /gripper/target (실행된 goal, raw 0 / 1150). 없으면 에러
#   base_imu = [각속도 xyz (rad/s), 선가속도 xyz (m/s²)] — --base-imu 로 정한다 (필수):
#       const : 고정 베이스. /base/imu 가 없는 bag 에서 상수 [0, 0, 0, 0, 0, 9.81] (bag 에 /base/imu 가 있으면 에러)
#       topic : /base/imu 를 격자 구간 (t − 1/fps, t] 평균으로 (8단계부터)
#
# 출력(중간 파일):  <데이터수집폴더>/bag_lerobot_intermediate/<bag이름>/
#     states.npy       (N, 7)
#     actions.npy      (N, 7)
#     base_imu.npy     (N, 6)
#     timestamps.npy   (N,)    = 각 프레임 기준 시각(초)
#     wrist/000000.png ...      = /cam/wrist/color/image_raw
#     third_view/000000.png ... = /cam/third_view/color/image_raw
#     meta.json                = fps, 프레임수, 옵션, 카메라 역할 → 장치, 녹화 정보(episode.json), 보호 정지, 명령 간격 통계
#
# 실행:
#   source /opt/ros/jazzy/setup.bash
#   python3 lerobot_stage1_extract_bag.py <bag폴더> --arm-action command --base-imu const [--fps 25] [--out <출력폴더>] [--overwrite]
# =============================================================

import os
import sys
import json
import shutil
import argparse

import numpy as np

import rclpy.serialization
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

from cv_bridge import CvBridge
import cv2

from camera_config import load_cameras, ROLES


# ── 토픽 이름 ──
TOPIC_ARM   = '/joint_states'
TOPIC_ARM_CMD = '/joint_command'                # 팔 관절 목표 (--arm-action command 의 action)
TOPIC_GRIP  = '/gripper/joint_states'          # 그리퍼 present (observation)
TOPIC_GRIP_TARGET = '/gripper/target'          # 그리퍼 goal, 30Hz 상시 발행 (action)
TOPIC_IMU = '/base/imu'                         # 베이스 IMU (8단계부터)
TOPIC_PSTOP = '/protective_stop'                # sim 보호 정지 흉내 (std_msgs/Bool, header 없음)
TOPIC_CLOCK = '/clock'                          # sim time (header 없는 토픽의 시각을 sim time 으로 바꾸는 데 씀)

# ── UR5 관절 순서(이 순서로 state 앞 6칸을 채움) ──
UR5_JOINT_ORDER = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

GRIPPER_RAW_MAX = 1150.0                        # 그리퍼 raw 범위 0 ~ 1150 → 0 ~ 1
BASE_IMU_CONST = [0.0, 0.0, 0.0, 0.0, 0.0, 9.81]   # 고정 베이스: 각속도 0, 선가속도 (0, 0, +g) (IMU 는 정지 시 위쪽 +g)
CMD_GAP_WARN = 0.040                            # /joint_command 간격이 이보다 길면 "늦은 명령" 으로 센다 (데이터셋 한 프레임)

# ── 중간 파일 출력 폴더 이름 (데이터수집 폴더 바로 아래) ──
INTERMEDIATE_DIRNAME = 'bag_lerobot_intermediate'
SCHEMA_VERSION = 2                              # 2 = 0단계 스키마 (wrist/third_view, 그리퍼 0~1, base_imu). 1 = 옛 형식 (head, raw)


def fail(msg):
    print(f"[오류] {msg}")
    sys.exit(1)


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


def load_all_messages(bag_path, wanted):
    """→ ({토픽: [(header.stamp 초, 메시지)] stamp 순}, {header 없는 토픽: [(bag 도착 시각 초, 메시지)]}, bag 에 있는 토픽 집합)"""
    reader = open_reader(bag_path)
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    msg_cls = {}
    def cls_of(topic):
        typ = type_map[topic]
        if typ not in msg_cls:
            msg_cls[typ] = get_message(typ)
        return msg_cls[typ]

    stamped = {}
    unstamped = {}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in wanted:
            continue
        msg = rclpy.serialization.deserialize_message(data, cls_of(topic))
        if hasattr(msg, 'header'):
            stamped.setdefault(topic, []).append((stamp_to_sec(msg.header.stamp), msg))
        else:
            unstamped.setdefault(topic, []).append((t_ns * 1e-9, msg))

    for t in stamped:
        stamped[t].sort(key=lambda x: x[0])
    return stamped, unstamped, set(type_map)


def latest_at(sorted_times, query_t):
    """query_t 이전(같음 포함)의 가장 최근 메시지 index. 없으면 -1"""
    return int(np.searchsorted(sorted_times, query_t, side='right')) - 1


def reorder_arm(msg, topic):
    name_to_pos = dict(zip(msg.name, msg.position))
    out = []
    for j in UR5_JOINT_ORDER:
        if j not in name_to_pos:
            raise KeyError(f"관절 '{j}' 가 {topic} 에 없음. 실제: {list(msg.name)}")
        out.append(float(name_to_pos[j]))
    return out


def interval_stats(times):
    """메시지 간격 통계 [ms] (텔레옵 명령이 끊긴 에피소드를 찾는 용도)"""
    d = np.diff(np.asarray(times, dtype=np.float64))
    return {'count': int(len(times)), 'hz_median': round(float(1.0 / np.median(d)), 2),
            'interval_median_ms': round(float(np.median(d) * 1000), 2), 'interval_max_ms': round(float(d.max() * 1000), 2),
            f'intervals_over_{CMD_GAP_WARN * 1000:.0f}ms': int((d > CMD_GAP_WARN + 1e-9).sum())}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag_path', help='ros2 bag 폴더 경로 (record_toggle.py 로 녹화, episode.json 포함)')
    ap.add_argument('--arm-action', required=True, choices=('command', 'next_state'),
                    help='팔 action: command = /joint_command (텔레옵·sim), next_state = states[i+1] (freedrive)')
    ap.add_argument('--base-imu', required=True, choices=('const', 'topic'),
                    help='base_imu: const = 고정 베이스 상수 [0,0,0,0,0,9.81], topic = /base/imu 구간 평균')
    ap.add_argument('--fps', type=float, default=25.0, help='목표 fps (기본 25)')
    ap.add_argument('--max-age', type=float, default=0.066,
                    help='[s] 격자 시각과 고른 메시지 stamp 의 차이 한계 (기본 0.066 = 카메라 2 프레임). 넘으면 에러')
    ap.add_argument('--skip-start', type=float, default=0.0,
                    help='[s] 녹화 시작 뒤 이만큼은 에피소드에서 뺀다 (기본 0). 녹화기가 뜨는 순간 메시지가 빠진 bag 을 살릴 때')
    ap.add_argument('--out', default=None,
                    help=f'출력 상위 폴더 (기본: 데이터수집폴더/{INTERMEDIATE_DIRNAME}/)')
    ap.add_argument('--overwrite', action='store_true', help='출력 폴더가 이미 있으면 지우고 다시 만든다 (없으면 에러)')
    args = ap.parse_args()

    bag_path = os.path.expanduser(args.bag_path).rstrip('/')
    if not os.path.isdir(bag_path):
        fail(f"폴더가 아님: {bag_path}")

    bag_name = os.path.basename(bag_path)

    # 기본 출력 위치: bag 이 있는 곳(.../bags/)의 부모 = 데이터수집 폴더, 그 아래 intermediate
    if args.out:
        out_root = os.path.expanduser(args.out)
    else:
        bags_dir = os.path.dirname(os.path.abspath(bag_path))   # .../data_collection_ur5_gripper/bags
        collection_dir = os.path.dirname(bags_dir)               # .../data_collection_ur5_gripper
        out_root = os.path.join(collection_dir, INTERMEDIATE_DIRNAME)
    out_dir = os.path.join(out_root, bag_name)

    # 녹화 정보 (record_toggle.py 가 남김): 모드 (sim / real), 카메라 역할 → 장치, sim 리셋 응답
    ep_path = os.path.join(bag_path, 'episode.json')
    if not os.path.isfile(ep_path):
        fail(f"episode.json 이 없음: {ep_path} (record_toggle.py 로 녹화한 bag 만 변환)")
    with open(ep_path) as f:
        episode = json.load(f)

    cams = load_cameras()
    cam_topics = {role: cams['roles'][role]['topic'] for role in ROLES}
    use_cmd = args.arm_action == 'command'

    # 카메라 역할 → 장치: 실물 = cameras.yaml 의 모델·시리얼, sim = 리셋 응답의 Camera prim 등
    sim = (episode.get('sim_reset') or {}).get('sim') if episode['mode'] == 'sim' else None
    if episode['mode'] == 'sim':
        if sim is None:
            fail("episode.json 에 sim 정보 (sim_reset.sim) 가 없음 (4-2 이후 sim 으로 녹화한 bag 만 변환)")
        cameras = {role: {'topic': cam_topics[role], 'device': 'sim', **sim['cameras'][role]} for role in ROLES}
    else:
        cameras = {role: {'topic': cam_topics[role], 'device': 'real', **cams['roles'][role]['real']} for role in ROLES}
    kills = [k['t'] for k in episode.get('drone_kill', [])]

    print(f"=== 추출: {bag_name} ({episode['mode']}) ===")
    print(f"목표 fps: {args.fps}, 팔 action: {args.arm_action}, base_imu: {args.base_imu}, 메시지 나이 한계: {args.max_age * 1000:.0f} ms")
    print(f"출력 위치: {out_dir}")
    print("bag 읽는 중...")
    wanted = {TOPIC_ARM, TOPIC_ARM_CMD, TOPIC_GRIP, TOPIC_GRIP_TARGET, TOPIC_IMU, TOPIC_PSTOP, TOPIC_CLOCK, *cam_topics.values()}
    buckets, unstamped, bag_topics = load_all_messages(bag_path, wanted)

    # ── 필요한 토픽 확인 (없으면 중단. 다른 토픽으로 조용히 대체하지 않음) ──
    state_topics = [TOPIC_ARM, TOPIC_GRIP, TOPIC_GRIP_TARGET, *cam_topics.values()]
    need = state_topics + ([TOPIC_ARM_CMD] if use_cmd else []) + ([TOPIC_IMU] if args.base_imu == 'topic' else [])
    for t in need:
        print(f"  {t:36s}: {len(buckets.get(t, []))}")
    missing = [t for t in need if not buckets.get(t)]
    if missing:
        fail(f"bag 에 메시지가 없는 토픽: {missing}")
    if args.base_imu == 'const' and TOPIC_IMU in bag_topics:
        fail(f"bag 에 {TOPIC_IMU} 가 있는데 --base-imu const 를 줌. --base-imu topic 으로 변환하세요")

    times = {t: np.asarray([s for s, _ in buckets[t]], dtype=np.float64) for t in need}
    zero = [t for t in need if np.any(times[t] <= 0.0)]
    if zero:
        fail(f"header.stamp 가 0 인 메시지가 있는 토픽: {zero} (보내는 쪽이 stamp 를 넣어야 함. sim 텔레옵은 sim time)")

    # ── 에피소드 구간 ──
    #   상태 토픽이 모두 있는 구간. command 면 첫 /joint_command 부터 (그 앞은 "직전 최신 명령" 이 없음)
    t_state_start = max(times[t][0] for t in state_topics)
    if args.skip_start < 0:
        fail(f"--skip-start 는 0 이상: {args.skip_start}")
    t_start = max(t_state_start + args.skip_start, times[TOPIC_ARM_CMD][0] if use_cmd else -np.inf)
    t_end = min(times[t][-1] for t in state_topics)
    if t_end <= t_start:
        fail(f"공통 시간 구간이 없음 (상태 {t_state_start:.3f} ~ {t_end:.3f} s"
             + (f", /joint_command {times[TOPIC_ARM_CMD][0]:.3f} ~ {times[TOPIC_ARM_CMD][-1]:.3f} s. 명령 stamp 의 시계가 다른지 확인" if use_cmd else "") + ")")

    dt = 1.0 / args.fps
    grid = np.arange(t_start, t_end, dt)
    dropped_head = int(np.floor((t_start - t_state_start) * args.fps + 1e-9))
    print(f"구간: {t_start:.3f} ~ {t_end:.3f} s ({t_end - t_start:.2f} s) -> {len(grid)} 프레임 @ {args.fps}Hz"
          + (f"  (녹화 시작 뒤 {t_start - t_state_start:.2f} s = {dropped_head} 프레임은 뺌: "
             f"{'첫 명령 이전' if use_cmd and times[TOPIC_ARM_CMD][0] >= t_state_start + args.skip_start else '--skip-start'})" if dropped_head else ""))

    if os.path.exists(out_dir):
        if not args.overwrite:
            fail(f"출력 폴더가 이미 있음: {out_dir} (다시 만들려면 --overwrite)")
        shutil.rmtree(out_dir)
    img_dirs = {role: os.path.join(out_dir, role) for role in ROLES}
    for d in img_dirs.values():
        os.makedirs(d)

    def pick(topic, qt, i):
        """격자 시각 qt 직전의 최신 메시지. 없거나 --max-age 보다 오래됐으면 에러. → (메시지, 나이 [s])"""
        k = latest_at(times[topic], qt)
        if k < 0:
            fail(f"프레임 {i} (t {qt:.3f}): {topic} 에 이 시각 이전 메시지가 없음")
        age = qt - times[topic][k]
        if age > args.max_age + 1e-9:
            fail(f"프레임 {i} (t {qt:.3f}): {topic} 의 최신 메시지가 {age * 1000:.0f} ms 전 것 (한계 {args.max_age * 1000:.0f} ms). "
                 f"메시지가 빠진 bag (카메라 best effort·프레임 드롭 등). 한계를 바꾸려면 --max-age, "
                 f"녹화 시작 직후에만 빠졌으면 --skip-start (지금 에피소드 시작 뒤 {qt - t_start:.2f} s)")
        return buckets[topic][k][1], age

    bridge = CvBridge()
    states, arm_cmds, grip_targets = [], [], []
    max_age = {t: 0.0 for t in state_topics}

    for i, qt in enumerate(grid):
        arm_msg, a = pick(TOPIC_ARM, qt, i)
        max_age[TOPIC_ARM] = max(max_age[TOPIC_ARM], a)
        grip_msg, a = pick(TOPIC_GRIP, qt, i)
        max_age[TOPIC_GRIP] = max(max_age[TOPIC_GRIP], a)
        # 그리퍼 goal (present 와 같은 tick/같은 stamp 로 발행되므로 같은 방식으로 샘플링)
        target_msg, a = pick(TOPIC_GRIP_TARGET, qt, i)
        max_age[TOPIC_GRIP_TARGET] = max(max_age[TOPIC_GRIP_TARGET], a)
        if len(grip_msg.position) == 0 or len(target_msg.position) == 0:
            fail(f"프레임 {i}: 그리퍼 메시지에 position 이 비어있음")
        states.append(reorder_arm(arm_msg, TOPIC_ARM) + [float(grip_msg.position[0])])
        grip_targets.append(float(target_msg.position[0]))

        if use_cmd:
            # 팔 명령은 텔레옵 주기로 오고, 텔레옵이 멈추면 로봇이 마지막 명령을 유지한다 → 나이 한계 없이 직전 최신 명령
            k = latest_at(times[TOPIC_ARM_CMD], qt)
            if k < 0:
                fail(f"프레임 {i} (t {qt:.3f}): 이 시각 이전 {TOPIC_ARM_CMD} 가 없음")
            arm_cmds.append(reorder_arm(buckets[TOPIC_ARM_CMD][k][1], TOPIC_ARM_CMD))

        for role in ROLES:
            img_msg, a = pick(cam_topics[role], qt, i)
            max_age[cam_topics[role]] = max(max_age[cam_topics[role]], a)
            img = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            if (img.shape[1], img.shape[0]) != cams['resolution']:
                fail(f"프레임 {i}: {cam_topics[role]} 크기 {img.shape[1]}x{img.shape[0]} (기대 {cams['resolution'][0]}x{cams['resolution'][1]})")
            cv2.imwrite(os.path.join(img_dirs[role], f'{i:06d}.png'), img)

        if (i + 1) % 50 == 0 or i == len(grid) - 1:
            print(f"  프레임 {i+1}/{len(grid)}")

    states = np.asarray(states, dtype=np.float64)
    grip_targets = np.asarray(grip_targets, dtype=np.float64)
    stamps = np.asarray(grid, dtype=np.float64)
    N = len(grid)

    # ── 그리퍼: raw (0~1150) → 0~1. 범위·값 종류가 스키마와 다르면 중단 ──
    if states[:, 6].min() < 0.0 or states[:, 6].max() > GRIPPER_RAW_MAX:
        fail(f"그리퍼 present 가 raw 범위 0~{GRIPPER_RAW_MAX:g} 밖: {states[:, 6].min():.1f} ~ {states[:, 6].max():.1f}")
    bad = sorted(set(grip_targets.tolist()) - {0.0, GRIPPER_RAW_MAX})
    if bad:
        fail(f"{TOPIC_GRIP_TARGET} 에 0 / {GRIPPER_RAW_MAX:g} 가 아닌 값: {bad[:5]} (그리퍼 action 은 이진)")
    states[:, 6] /= GRIPPER_RAW_MAX

    # ── action (N, 7) ──
    actions = np.empty_like(states)
    if use_cmd:
        actions[:, :6] = np.asarray(arm_cmds, dtype=np.float64)
    else:
        actions[:-1, :6] = states[1:, :6]
        actions[-1, :6] = states[-1, :6]
    actions[:, 6] = grip_targets / GRIPPER_RAW_MAX

    # ── base_imu (N, 6) ──
    if args.base_imu == 'const':
        base_imu = np.tile(np.asarray(BASE_IMU_CONST), (N, 1))
    else:
        imu_t = times[TOPIC_IMU]
        imu_v = np.asarray([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                             m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for _, m in buckets[TOPIC_IMU]])
        base_imu = np.empty((N, 6))
        for i, qt in enumerate(grid):
            sel = (imu_t > qt - dt) & (imu_t <= qt)          # zero-order hold 대신 구간 평균 (IMU 는 수백 Hz)
            if not sel.any():
                fail(f"프레임 {i} (t {qt:.3f}): 구간 ({qt - dt:.3f}, {qt:.3f}] 에 {TOPIC_IMU} 메시지가 없음")
            base_imu[i] = imu_v[sel].mean(axis=0)

    # ── 보호 정지 (sim). Bool 에 header 가 없어 bag 도착 시각 → /clock 으로 sim time 환산 ──
    pstop = None
    if TOPIC_PSTOP in bag_topics:
        ps_msgs, ck = unstamped.get(TOPIC_PSTOP, []), unstamped.get(TOPIC_CLOCK, [])
        if not ps_msgs or not ck:
            fail(f"bag 에 {TOPIC_PSTOP} 가 있는데 메시지가 없거나 {TOPIC_CLOCK} 이 없음 (보호 정지 시각을 sim time 으로 바꿀 수 없음)")
        ck_wall = np.asarray([w for w, _ in ck])
        ck_sim = np.asarray([stamp_to_sec(m.clock) for _, m in ck])
        true_wall = [w for w, m in ps_msgs if m.data]
        pstop = {'occurred': bool(true_wall), 'messages': len(ps_msgs)}
        if true_wall:
            t_ps = float(np.interp(true_wall[0], ck_wall, ck_sim))
            pstop.update(t=round(t_ps, 3), t_from_start=round(t_ps - t_start, 3),
                         frame=int(np.clip(np.ceil((t_ps - t_start) * args.fps), 0, N - 1)), in_episode=bool(t_start <= t_ps <= t_end))

    np.save(os.path.join(out_dir, 'states.npy'), states.astype(np.float32))
    np.save(os.path.join(out_dir, 'actions.npy'), actions.astype(np.float32))
    np.save(os.path.join(out_dir, 'base_imu.npy'), base_imu.astype(np.float32))
    np.save(os.path.join(out_dir, 'timestamps.npy'), stamps)

    meta = {
        'schema_version': SCHEMA_VERSION,
        'bag_name': bag_name,
        'mode': episode['mode'],
        'fps': args.fps,
        'num_frames': N,
        't_start': round(float(t_start), 6), 't_end': round(float(stamps[-1]), 6),
        'state_dim': 7, 'state_names': UR5_JOINT_ORDER + ['gripper'],
        'action_dim': 7, 'action_names': UR5_JOINT_ORDER + ['gripper'],
        'arm_action': args.arm_action,
        'arm_action_source': TOPIC_ARM_CMD if use_cmd else 'states[i+1]',
        'gripper_action_source': TOPIC_GRIP_TARGET,
        'gripper_scale': f'raw / {GRIPPER_RAW_MAX:g} (0 = 열림, 1 = 닫힘)',
        'base_imu': args.base_imu, 'base_imu_names': ['gyro_x', 'gyro_y', 'gyro_z', 'accel_x', 'accel_y', 'accel_z'],
        'image_keys': list(ROLES), 'image_size': [cams['resolution'][1], cams['resolution'][0], 3],
        'cameras': cameras,
        'max_age_limit_ms': round(args.max_age * 1000, 1),
        'max_age_ms': {t: round(a * 1000, 1) for t, a in max_age.items()},
        'skip_start_s': args.skip_start,
        'dropped_frames_at_start': dropped_head,      # 녹화 시작 ~ 에피소드 시작 (첫 명령 이전 또는 --skip-start)
        'joint_command': interval_stats(times[TOPIC_ARM_CMD]) if use_cmd else None,
        'protective_stop': pstop,
        'drone_kill': [{'t': t, 't_from_start': round(t - t_start, 3), 'frame': int(np.clip(np.ceil((t - t_start) * args.fps), 0, N - 1))} for t in kills],
        'episode': episode,
    }
    with open(os.path.join(out_dir, 'meta.json'), 'w') as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    print("\n-- 추출 완료 --")
    print(f"출력 폴더: {out_dir}")
    print(f"  states.npy {states.shape}, actions.npy {actions.shape}, base_imu.npy {base_imu.shape}, timestamps.npy {stamps.shape}")
    print(f"  {', '.join(f'{r}/*.png' for r in ROLES)}: 각 {N} 장, meta.json")
    print(f"state  예시(첫 프레임): {np.round(states[0], 4).tolist()}")
    print(f"action 예시(첫 프레임): {np.round(actions[0], 4).tolist()}")
    print(f"그리퍼 state 범위 {states[:, 6].min():.3f} ~ {states[:, 6].max():.3f}, action 값 종류 {sorted(set(actions[:, 6].tolist()))}")
    print(f"고른 메시지 나이 최대 [ms]: " + ", ".join(f"{t} {a * 1000:.1f}" for t, a in max_age.items()))
    if use_cmd:
        print(f"/joint_command: {meta['joint_command']}")
    if pstop is not None:
        print(f"보호 정지: {pstop}")
    if kills:
        print(f"드론 모터 정지: {meta['drone_kill']}")


if __name__ == '__main__':
    main()
