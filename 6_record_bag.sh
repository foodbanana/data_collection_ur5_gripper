#!/bin/bash
# =============================================================
# 6_record_bag.sh   [터미널 6]
# rosbag2 녹화 토글 (압축 없음) — 실행하면 대기 상태, r 키로 녹화 시작/종료
# 전제: 필요한 노드들(로봇팔/그리퍼/카메라)이 실행 중이고 토픽이 발행되는 상태
# 사용: ./6_record_bag.sh            → 날짜시간만  (예: 20260916_143022)
#       ./6_record_bag.sh pick       → 날짜시간_라벨 (예: 20260916_143022_pick)
# 키:   r = 녹화 시작 / 녹화 종료 (r~r 구간 1개 = bag 1개 = 에피소드 1개)
#       d = 방금 저장한 에피소드 버리기 → bags/_discarded/ 로 이동 (y 로 확인)
#       q = 종료 (녹화 중이면 안전 종료 후 종료)
# 저장: ~/data_collection_ur5_gripper/bags/<이름>/
# ※ 녹화 토픽 목록은 record_toggle.py 의 TOPICS 에 있음
# =============================================================

source /opt/ros/jazzy/setup.bash
source ~/realsense_ws/install/setup.bash   # 카메라 토픽 타입 인식용 underlay

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec python3 "${SCRIPT_DIR}/record_toggle.py" "$@"
