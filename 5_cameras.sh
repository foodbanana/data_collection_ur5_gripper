#!/bin/bash
# =============================================================
# 5_cameras.sh   [터미널 5]
# RealSense 카메라 2대 실행, color만 640x480@30 (depth / infra / IMU 꺼짐)
# 전제: 두 카메라 모두 USB 3.x 포트에 연결
#       (확인: rs-enumerate-devices | grep -iE "serial number|usb type")
#
# 카메라 역할 ↔ 모델·시리얼은 config/cameras.yaml 한 곳 (지금 wrist = D435i, third_view = D456).
# 런치 파일은 이 레포의 launch/cameras.launch.py (cameras.yaml 을 읽어 역할마다 노드 하나: namespace cam, name <역할>)
#   → 토픽 = 역할 (docs/PLAN.md 0-2, 4단계): /cam/wrist/color/image_raw , /cam/third_view/color/image_raw (+ camera_info)
#   녹화·변환은 이 이름만 받는다. 카메라를 바꾸면 cameras.yaml 의 model / serial 만 고친다.
# ※ 예전 런치 (~/realsense_ws 의 realsense_dual_camera, 토픽 /d435i/d435i/..., /d456/d456/...) 는 쓰지 않는다.
#   ~/realsense_ws 는 realsense2_camera 패키지를 쓰려고 source 만 한다
# ※ 주기 확인은 camera_info 로: ros2 topic hz /cam/wrist/color/camera_info
#   (image_raw 를 ros2 topic hz 로 재면 best effort 수신 + 파이썬 처리 때문에 실제보다 낮게 나온다)
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
echo "    발행 토픽(color):"
echo "      /cam/wrist/color/image_raw"
echo "      /cam/third_view/color/image_raw"
echo ""

ros2 launch "${SCRIPT_DIR}/launch/cameras.launch.py"
