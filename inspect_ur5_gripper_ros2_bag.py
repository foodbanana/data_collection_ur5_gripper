#!/usr/bin/env python3
# =============================================================
# inspect_bag.py
# ros2 bag(mcap) 내부를 안전하게 들여다보는 검사 스크립트.
# 3GB 전체를 읽지 않고, 각 토픽에서 앞쪽 몇 개만 샘플로 확인한다.
#
# 확인 항목:
#   - 각 토픽의 타입, 첫 메시지 값(관절 배열/그리퍼 position/이미지 규격)
#   - 각 토픽의 타임스탬프 몇 개 (동기화 설계 참고용)
#   - 카메라 첫 프레임을 png로 저장 (눈으로 확인)
#     저장 위치: <이 스크립트 폴더>/inspect_out/<bag이름>_<검사시각>/
#     (bags/ 안에는 실제 bag만 남도록 bags/ 밖에 저장)
#
# 실행:
#   source /opt/ros/jazzy/setup.bash
#   python3 inspect_bag.py <bag폴더경로>
# 예:
#   python3 inspect_bag.py ~/data_collection_ur5_gripper/bags/20260916_141816_test_1_fake_node
# =============================================================

import sys
import os

import rclpy.serialization
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


# 토픽별로 몇 개까지 샘플을 볼지
SAMPLES_PER_TOPIC = 3
# 안전장치: bag 앞쪽 이 시간(초)까지만 훑고 종료.
# 이벤트성 토픽(/gripper/command 등)이 뒤쪽에만 있어도 bag 끝까지 읽지 않게 한다.
MAX_SCAN_SEC = 60.0
# 이미지 토픽은 첫 프레임 1장만 png로 저장
SAVE_IMAGE_TOPICS = [
    '/d435i/d435i/color/image_raw',
    '/d456/d456/color/image_raw',
]
# 이미지 미리보기 출력 상위 폴더 (bags/ 밖: bags/* 일괄 처리 시 bag으로 오인되지 않게)
INSPECT_OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'inspect_out')


def open_reader(bag_path):
    """bag 폴더를 열어 rosbag2 reader 반환. storage_id는 자동 감지 시도."""
    # mcap 우선 시도, 실패하면 sqlite3
    for storage_id in ('mcap', 'sqlite3'):
        try:
            reader = rosbag2_py.SequentialReader()
            storage_options = rosbag2_py.StorageOptions(
                uri=bag_path, storage_id=storage_id)
            converter_options = rosbag2_py.ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr')
            reader.open(storage_options, converter_options)
            return reader
        except Exception:
            continue
    raise RuntimeError(f"bag을 열 수 없음: {bag_path}")


def main():
    if len(sys.argv) < 2:
        print("사용법: python3 inspect_bag.py <bag폴더경로>")
        sys.exit(1)

    bag_path = os.path.expanduser(sys.argv[1])
    if not os.path.isdir(bag_path):
        print(f"폴더가 아님: {bag_path}")
        sys.exit(1)

    print(f"=== bag 검사: {bag_path} ===\n")

    reader = open_reader(bag_path)

    # 토픽 목록 + 타입 매핑
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    # 토픽별 전체 메시지 개수 (메타데이터에서 읽음 — bag 본문은 읽지 않음)
    try:
        msg_count = {ti.topic_metadata.name: ti.message_count
                     for ti in reader.get_metadata().topics_with_message_count}
    except Exception:
        msg_count = {}  # 못 읽으면 개수 미상으로 처리

    print("── 토픽 목록 ──")
    for name, typ in type_map.items():
        cnt = msg_count.get(name)
        cnt_str = f"{cnt}개" if cnt is not None else "개수 미상"
        print(f"  {name}   [{typ}]   ({cnt_str})")
    print()

    # 종료 판정용 목표 샘플 수: 메시지가 3개 미만인 토픽(이벤트성)은 있는 만큼만 요구
    target_count = {name: min(SAMPLES_PER_TOPIC, msg_count.get(name, SAMPLES_PER_TOPIC))
                    for name in type_map}
    # 실제로 메시지가 있는 이미지 토픽만 저장 완료를 기다린다
    image_topics = [n for n in SAVE_IMAGE_TOPICS
                    if n in type_map and target_count[n] > 0]

    # 메시지 클래스 캐시
    msg_class_cache = {}
    def get_msg_class(type_str):
        if type_str not in msg_class_cache:
            msg_class_cache[type_str] = get_message(type_str)
        return msg_class_cache[type_str]

    # 토픽별 샘플 카운트/이미지 저장 플래그
    seen_count = {name: 0 for name in type_map}
    image_saved = {name: False for name in type_map}

    # 이미지 저장용 (cv_bridge는 필요할 때만 import)
    # 출력 폴더: inspect_out/<bag이름>_<검사시각>
    import datetime
    bridge = None
    bag_name = os.path.basename(bag_path.rstrip('/'))
    now_stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    out_dir = os.path.join(INSPECT_OUT_DIR, f'{bag_name}_{now_stamp}')

    print("── 각 토픽 샘플 ──")
    first_t_ns = None
    while reader.has_next():
        topic, data, t_ns = reader.read_next()  # t_ns: 녹화 타임스탬프(ns)

        if first_t_ns is None:
            first_t_ns = t_ns
        if (t_ns - first_t_ns) > MAX_SCAN_SEC * 1e9:
            print(f"\n[안전장치] 앞쪽 {MAX_SCAN_SEC:.0f}초 구간 도달 — 검사 조기 종료")
            break

        # 아직 샘플이 부족한 토픽만 처리
        need_sample = seen_count[topic] < SAMPLES_PER_TOPIC
        need_image = (topic in SAVE_IMAGE_TOPICS) and (not image_saved[topic])
        if not need_sample and not need_image:
            # 모든 토픽이 충분히 모였는지 검사
            if all(seen_count[n] >= target_count[n] for n in type_map) and \
               all(image_saved[n] for n in image_topics):
                break
            continue

        msg_cls = get_msg_class(type_map[topic])
        msg = rclpy.serialization.deserialize_message(data, msg_cls)

        if need_sample:
            seen_count[topic] += 1
            print(f"\n[{topic}]  샘플 #{seen_count[topic]}  (bag_time={t_ns} ns)")
            _print_msg_brief(topic, msg)

        if need_image:
            if bridge is None:
                from cv_bridge import CvBridge
                import cv2
                bridge = CvBridge()
                os.makedirs(out_dir, exist_ok=True)
            import cv2
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            fname = topic.strip('/').replace('/', '_') + '_first.png'
            fpath = os.path.join(out_dir, fname)
            cv2.imwrite(fpath, cv_img)
            image_saved[topic] = True
            print(f"      → 첫 프레임 저장: {fpath}  (shape={cv_img.shape})")

    print("\n── 검사 완료 ──")
    # 샘플을 다 못 본 토픽 요약 (녹화가 안 된 건지, 뒤쪽에만 있는 건지 구분용)
    for name in type_map:
        if seen_count[name] < SAMPLES_PER_TOPIC:
            cnt = msg_count.get(name)
            cnt_str = f"{cnt}개" if cnt is not None else "개수 미상"
            print(f"  샘플 부족: {name}  (확인 {seen_count[name]}개 / bag 전체 {cnt_str})")
    if any(image_saved.get(n) for n in SAVE_IMAGE_TOPICS):
        print(f"이미지 미리보기 폴더: {out_dir}")


def _print_msg_brief(topic, msg):
    """메시지 타입별로 핵심만 출력."""
    tname = type(msg).__name__

    # header.stamp 있으면 출력
    if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
        s = msg.header.stamp
        print(f"      header.stamp = {s.sec}.{s.nanosec:09d}")

    if tname == 'JointState':
        print(f"      name     = {list(msg.name)}")
        print(f"      position = {[round(p, 4) for p in msg.position]}")
        if len(msg.velocity) > 0:
            print(f"      velocity = {[round(v, 4) for v in msg.velocity]}")
        else:
            print(f"      velocity = [] (비어있음)")

    elif tname == 'Float64':
        print(f"      data = {msg.data}")

    elif tname == 'Image':
        print(f"      {msg.width}x{msg.height}, encoding={msg.encoding}, "
              f"step={msg.step}, data_len={len(msg.data)}")

    else:
        # 기타 타입은 대략적으로
        print(f"      (타입 {tname}, 상세 생략)")


if __name__ == '__main__':
    main()