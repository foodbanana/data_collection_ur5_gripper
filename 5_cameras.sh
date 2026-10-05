#!/bin/bash
# =============================================================
# 5_cameras.sh   [터미널 5]
# RealSense 카메라 2대 실행, color만 640x480@30
#   ※ depth / infra / IMU 는 dual_camera.launch.py 에서 꺼져 있음 (enable_depth: False 등)
# 전제: 두 카메라 모두 USB 3.x 포트에 연결
#       (확인: rs-enumerate-devices | grep -iE "serial number|usb type")
#
# 카메라 역할 ↔ 모델·시리얼은 config/cameras.yaml 한 곳 (지금 wrist = D435i, third_view = D456). 이 스크립트는 거기서 읽는다.
# ※ 토픽 이름 = 역할 (docs/PLAN.md 0-2, 4단계): 녹화·변환은
#      /cam/wrist/color/image_raw , /cam/third_view/color/image_raw   (+ camera_info, QoS RELIABLE)
#   만 받는다. realsense_dual_camera 런치 (~/realsense_ws, 이 레포 밖) 가 이 이름으로 발행하도록 고쳐야 한다:
#      realsense2_camera 노드마다 camera_namespace:=cam , camera_name:=wrist / third_view , serial_no:=_<시리얼>
#   옛 이름 (/d435i/d435i/..., /d456/d456/...) 으로 발행하면 6_record_bag.sh 가 "발행자 없음" 으로 녹화를 시작하지 않는다.
# =============================================================

source /opt/ros/jazzy/setup.bash
source ~/realsense_ws/install/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
serial_of() {
  python3 -c "import sys; sys.path.insert(0, '${SCRIPT_DIR}'); from camera_config import load_cameras; r = load_cameras()['roles']['$1']['real']; print(r['model'], r['serial'])"
}
read -r WRIST_MODEL WRIST_SERIAL <<< "$(serial_of wrist)"
read -r THIRD_MODEL THIRD_SERIAL <<< "$(serial_of third_view)"
if [ -z "${WRIST_SERIAL}" ] || [ -z "${THIRD_SERIAL}" ]; then
  echo "[오류] config/cameras.yaml 에서 카메라 시리얼을 읽지 못함"; exit 1
fi

echo "[5] RealSense 카메라 2대 실행..."
echo "    wrist      : ${WRIST_MODEL} serial ${WRIST_SERIAL}"
echo "    third_view : ${THIRD_MODEL} serial ${THIRD_SERIAL}"
echo ""
echo "    발행해야 하는 토픽(color):"
echo "      /cam/wrist/color/image_raw"
echo "      /cam/third_view/color/image_raw"
echo ""

# 런치 인자 이름 (d435i_serial / d456_serial) 은 지금 런치 파일 기준: wrist = D435i, third_view = D456.
# 카메라 구성이 바뀌면 (예: third_view 도 D435i) 런치 파일의 인자 이름과 함께 고칠 것
ros2 launch realsense_dual_camera dual_camera.launch.py \
  d435i_serial:=_${WRIST_SERIAL} \
  d456_serial:=_${THIRD_SERIAL}
