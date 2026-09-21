#!/bin/bash
# =============================================================
# 4_gripper_teleop.sh   [터미널 4]
# 그리퍼 키보드 제어 → /gripper/command 발행
# 전제: 터미널3 (3_gripper_node.sh) 실행 중이어야 실제로 그리퍼가 움직임
# 사용: 실행되면 뜨는 키 안내(0/1/2 등)에 따라 그리퍼 여닫기
# 종료: q (또는 Ctrl+C)
# =============================================================

source /opt/ros/jazzy/setup.bash
source ~/rh_gripper_ros2_ws/install/setup.bash

echo "[4] 그리퍼 teleop (키보드 제어) 실행..."
echo "    발행 토픽: /gripper/command"
echo "    키 안내는 아래에 표시됨. 키를 눌러 그리퍼를 여닫으세요."
echo "    (데이터 수집 중 한 손은 로봇팔, 한 손은 이 키보드)"
echo ""

ros2 run rh_p12_rn_ros2 rh_gripper_teleop
