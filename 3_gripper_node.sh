#!/bin/bash
# =============================================================
# 3_gripper_node.sh   [터미널 3]
# 그리퍼 노드 실행 → /gripper/joint_states 발행 (그리퍼 present, 30Hz)
#                  + /gripper/target 발행 (그리퍼 goal, 30Hz, 같은 tick·같은 stamp)
#                  + /gripper/command 구독 (명령 받으면 실제로 움직임)
# 전제: 그리퍼 - U2D2 - 노트북 USB 연결 + 파워서플라이 24V ON
#       /dev/ttyUSB0 인식 확인 (ls -l /dev/ttyUSB0)
# 주의: 이 노드는 상태를 읽고 명령을 받으면 움직이지만, 스스로 움직이지 않음.
#       실제 여닫기는 4_gripper_teleop.sh (키보드 제어)로 명령을 줘야 함.
# =============================================================

source /opt/ros/jazzy/setup.bash
source ~/rh_gripper_ros2_ws/install/setup.bash

echo "[3] 그리퍼 노드 실행..."
echo "    발행 토픽: /gripper/joint_states (그리퍼 present, sensor_msgs/JointState, 30Hz)"
echo "               /gripper/target       (그리퍼 goal,    sensor_msgs/JointState, 30Hz)"
echo "    구독 토픽: /gripper/command (std_msgs/Float64, raw 0~1150)"
echo "    포트     : /dev/ttyUSB0"
echo ""
echo "    ※ 이 터미널은 켜둔 채로, 그리퍼 제어는 4_gripper_teleop.sh 사용"
echo ""

ros2 run rh_p12_rn_ros2 rh_gripper_node
