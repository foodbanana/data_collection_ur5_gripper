#!/bin/bash
# =============================================================
# 5_cameras.sh   [터미널 5]
# RealSense 카메라 2대 실행 (d435i + d456), color만 640x480@30
#   ※ depth / infra / IMU 는 dual_camera.launch.py 에서 꺼져 있음 (enable_depth: False 등)
# 전제: 두 카메라 모두 USB 3.x 포트에 연결
#       (확인: rs-enumerate-devices | grep -iE "serial number|usb type")
# =============================================================

source /opt/ros/jazzy/setup.bash
source ~/realsense_ws/install/setup.bash

echo "[5] RealSense 카메라 2대 실행..."
echo "    d435i serial: 843112074130"
echo "    d456  serial: 252122301126"
echo ""
echo "    발행 토픽(color):"
echo "      /d435i/d435i/color/image_raw"
echo "      /d456/d456/color/image_raw"
echo ""

ros2 launch realsense_dual_camera dual_camera.launch.py \
  d435i_serial:=_843112074130 \
  d456_serial:=_252122301126
