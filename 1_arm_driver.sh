#!/bin/bash
# =============================================================
# 1_arm_driver.sh   [터미널 1]
# UR5 드라이버(ur_control.launch.py) 실행
# → 실행 후, 로봇 티치펜던트에서 External Control 프로그램 재생(▶) 해야 함
# =============================================================

# ── 사용자 환경에 맞게 수정 ──
ROBOT_IP="192.168.56.10"
CALIB="${HOME}/my_robot_calibration.yaml"

source /opt/ros/jazzy/setup.bash

echo "[1] UR5 드라이버 실행..."
echo "    로봇 IP     : ${ROBOT_IP}"
echo "    캘리브레이션: ${CALIB}"
echo ""
echo "    >>> 드라이버가 뜨면 티치펜던트에서 <<<"
echo "    >>> External Control 프로그램을 재생(▶) 하세요 <<<"
echo ""

ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5 \
  robot_ip:=${ROBOT_IP} \
  kinematics_params_file:="${CALIB}"
