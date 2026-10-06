# =============================================================
# cameras.launch.py   (5_cameras.sh 가 실행. docs/PLAN.md 0-2, 4단계)
#
# RealSense 카메라를 역할 이름으로 띄운다. 역할 ↔ 모델·시리얼은 config/cameras.yaml 한 곳에서 읽는다.
#   역할마다 realsense2_camera 노드 하나: namespace = cam, name = <역할>
#   → 토픽 /cam/<역할>/color/image_raw , /cam/<역할>/color/camera_info   (녹화·변환이 받는 이름)
#   color 만 640x480 @ 30 Hz. depth / infra / IMU 는 끔
# 실행: ros2 launch <이 파일 경로>      (보통 ./5_cameras.sh)
# =============================================================

import os
import sys

from launch import LaunchDescription
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from camera_config import load_cameras  # noqa: E402


def generate_launch_description():
    cams = load_cameras()
    w, h = cams['resolution']
    common_params = {
        'enable_color': True,
        'rgb_camera.color_profile': f'{w}x{h}x30',
        'enable_depth': False,
        'enable_infra1': False,
        'enable_infra2': False,
        'enable_gyro': False,
        'enable_accel': False,
    }
    nodes = []
    for role, r in cams['roles'].items():
        if r['topic'] != f'/cam/{role}/color/image_raw':
            raise RuntimeError(f"config/cameras.yaml 의 토픽 형식이 이 런치와 다름: {r['topic']} (런치는 /cam/<역할>/color/image_raw 로 발행)")
        nodes.append(Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='cam',
            name=role,
            # serial_no 앞의 '_' = 숫자로만 된 시리얼을 문자열로 읽게 하는 realsense-ros 규칙
            parameters=[dict(common_params, serial_no='_' + r['real']['serial'])],
            output='screen',
        ))
    return LaunchDescription(nodes)
