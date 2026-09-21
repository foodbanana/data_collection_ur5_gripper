#!/bin/bash
# =============================================================
# 2_arm_freedrive.sh   [터미널 2]
# freedrive 활성화 노드 실행
# 전제: 터미널1 드라이버 실행 중 + 펜던트 External Control 재생(▶) 상태
# 종료: Enter (freedrive 해제 + scaled_joint_trajectory_controller 원복)
# =============================================================

source /opt/ros/jazzy/setup.bash
source ~/ur_freedrive_ws/install/setup.bash

echo "[2] freedrive 토글 노드 실행..."
echo "    실행되면 freedrive ON (손으로 팔을 움직일 수 있음)"
echo "    Enter 를 누르면 freedrive 해제 + 원복 후 종료"
echo ""

ros2 run ur5_teleop freedrive_toggle
