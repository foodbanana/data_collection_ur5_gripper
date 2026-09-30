#!/usr/bin/env python3
# =============================================================
# fake_gripper_cameras_sim.py
#
# [5단계 스모크 테스트 전용 — Isaac Sim 씬과 함께 실행]
#
# 목적:
#   지금 Isaac Sim 은 /joint_states (진짜 UR5) 만 발행한다.
#   실물 rosbag 은 토픽 6개로 되어 있고, 변환 코드(lerobot_stage1_extract_bag.py)는
#   그 중 5개(/joint_states, /gripper/joint_states, /gripper/target,
#   두 카메라)를 읽는다.
#   이 노드는 sim 에 없는 나머지 토픽을 "가짜로" 발행해서,
#   실물과 동일한 6-토픽 bag 을 만들 수 있게 한다.
#   → 그러면 기존 변환/검수 코드를 (수정 없이) sim bag 에 그대로 돌려볼 수 있다.
#
# 발행 토픽 (실물과 이름·타입 동일):
#   /gripper/joint_states          sensor_msgs/JointState  present(raw 0~1150)  30Hz
#   /gripper/target                sensor_msgs/JointState  goal(raw {0,1150})   30Hz
#   /gripper/command               std_msgs/Float64        참고용(변환 X)       30Hz
#   /d435i/d435i/color/image_raw   sensor_msgs/Image       third_view 640x480   30Hz
#   /d456/d456/color/image_raw     sensor_msgs/Image       head       640x480   30Hz
#
# ★ 핵심 — sim time:
#   변환 코드는 각 토픽 header.stamp 기준으로 25Hz 리샘플하며,
#   공통 구간 t_start=max(첫 stamp), t_end=min(마지막 stamp) 을 잡는다.
#   sim /joint_states 는 sim time(예: sec 14)이므로, 이 가짜 토픽도 반드시
#   sim time 으로 stamp 를 찍어야 공통 구간이 겹친다.
#   → 이 노드는 반드시 use_sim_time:=true 로 실행할 것 (아래 실행 예시).
#     그러면 self.get_clock().now() 가 /clock(sim time) 을 따라간다.
#
# 실행 (Isaac Sim 이 Play 로 /clock 을 발행 중이어야 함):
#   source /opt/ros/jazzy/setup.bash
#   python3 fake_gripper_cameras_sim.py --ros-args -p use_sim_time:=true
#
# 종료: Ctrl+C
#
# 스모크 테스트 시나리오(그리퍼 값):
#   기본은 open(0). --close-after N 초 뒤부터 target=1150(닫기 명령),
#   present 는 물체에 막힌 것처럼 CLAMP_PRESENT(742)에서 멈추도록 흉내낸다.
#   → inspect_parquet.py 의 "파지 신호(action=1150인데 state=742에서 멈춤)" 검수까지 재현 가능.
# =============================================================

import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float64


# ── 실물과 동일한 토픽 이름 ──
TOPIC_GRIP_PRESENT = '/gripper/joint_states'
TOPIC_GRIP_TARGET  = '/gripper/target'
TOPIC_GRIP_COMMAND = '/gripper/command'
TOPIC_CAM_THIRD    = '/d435i/d435i/color/image_raw'   # 외부 고정 (third_view)
TOPIC_CAM_HEAD     = '/d456/d456/color/image_raw'      # 손목 (head)

PUBLISH_HZ = 30.0          # 실물 그리퍼/카메라와 동일
IMG_W, IMG_H = 640, 480    # 실물 color 해상도

GRIP_OPEN  = 0.0
GRIP_CLOSE = 1150.0
CLAMP_PRESENT = 742.0      # 닫기 명령 시 present 가 "물체에 막혀" 멈추는 값(파지 흉내)


class FakeGripperCameras(Node):
    def __init__(self, close_after, grasp):
        super().__init__('fake_gripper_cameras_sim')

        self.close_after = close_after
        self.grasp = grasp

        # 이미지는 큰 메시지라 SENSOR_DATA(best effort) QoS 가 자연스럽지만,
        # ros2 bag record 는 reliable 도 무리 없이 받으므로 기본 depth 10 reliable 로 둔다.
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)

        self.pub_present = self.create_publisher(JointState, TOPIC_GRIP_PRESENT, qos)
        self.pub_target  = self.create_publisher(JointState, TOPIC_GRIP_TARGET, qos)
        self.pub_command = self.create_publisher(Float64,    TOPIC_GRIP_COMMAND, qos)
        self.pub_third   = self.create_publisher(Image,      TOPIC_CAM_THIRD, qos)
        self.pub_head    = self.create_publisher(Image,      TOPIC_CAM_HEAD, qos)

        self.dt = 1.0 / PUBLISH_HZ
        self.t = 0.0
        self.timer = self.create_timer(self.dt, self.tick)

        # 미리 만들어 둔 더미 프레임 버퍼 (매 tick 색만 살짝 바꿔 흐름이 보이게)
        self._third_base = self._make_frame(base_color=(60, 90, 140))   # 파란톤
        self._head_base  = self._make_frame(base_color=(140, 90, 60))   # 주황톤

        use_sim = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.get_logger().info(
            f'가짜 그리퍼+카메라 발행 시작 ({PUBLISH_HZ:.0f}Hz). '
            f'use_sim_time={use_sim}. '
            f'close_after={close_after}s, grasp={grasp}. Ctrl+C 로 종료.'
        )
        if not use_sim:
            self.get_logger().warn(
                'use_sim_time 이 False 입니다! sim bag 스모크 테스트라면 반드시 '
                '--ros-args -p use_sim_time:=true 로 실행하세요. '
                '(안 그러면 stamp 가 벽시계라 변환 시 공통 구간이 안 겹칩니다.)'
            )

    def _make_frame(self, base_color):
        img = np.zeros((IMG_H, IMG_W, 3), dtype=np.uint8)
        img[:, :] = base_color
        return img

    def _now_stamp(self):
        # use_sim_time:=true 이면 이 clock 이 /clock(sim time) 을 따라간다.
        return self.get_clock().now().to_msg()

    def _fill_image_msg(self, base, moving_x):
        stamp = self._now_stamp()
        frame = base.copy()
        # 흐름이 눈에 보이도록 세로 밝은 띠 하나를 좌우로 움직임(디버그용, 없어도 무방)
        x = int(moving_x) % IMG_W
        frame[:, max(0, x - 3):x + 3] = (255, 255, 255)

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.height = IMG_H
        msg.width = IMG_W
        msg.encoding = 'bgr8'          # Stage1 이 desired_encoding='bgr8' 로 읽음
        msg.is_bigendian = 0
        msg.step = IMG_W * 3
        msg.data = frame.tobytes()
        return msg

    def tick(self):
        stamp = self._now_stamp()

        # ── 그리퍼 명령 스케줄 ──
        # close_after 초 전: 열림(0), 이후: 닫기(1150)
        closing = (self.close_after >= 0.0) and (self.t >= self.close_after)
        target_val = GRIP_CLOSE if closing else GRIP_OPEN

        # present: 닫는 중이고 grasp=True 면 물체에 막혀 CLAMP_PRESENT 에서 멈춤(파지 흉내),
        #          grasp=False 면 target 을 그대로 따라감(끝까지 닫힘)
        if closing:
            present_val = CLAMP_PRESENT if self.grasp else GRIP_CLOSE
        else:
            present_val = GRIP_OPEN

        # /gripper/joint_states (present)
        m_present = JointState()
        m_present.header.stamp = stamp
        m_present.name = ['gripper']
        m_present.position = [float(present_val)]
        self.pub_present.publish(m_present)

        # /gripper/target (goal)
        m_target = JointState()
        m_target.header.stamp = stamp
        m_target.name = ['gripper']
        m_target.position = [float(target_val)]
        self.pub_target.publish(m_target)

        # /gripper/command (참고용, Float64)
        m_cmd = Float64()
        m_cmd.data = float(target_val)
        self.pub_command.publish(m_cmd)

        # ── 카메라 2대 ──
        moving = self.t * 120.0   # 밝은 띠 이동 속도(디버그용)
        self.pub_third.publish(self._fill_image_msg(self._third_base, moving))
        self.pub_head.publish(self._fill_image_msg(self._head_base, moving + 320))

        self.t += self.dt


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--close-after', type=float, default=3.0,
                    help='이 초 이후부터 그리퍼 target=1150(닫기). 음수면 계속 열림. 기본 3.0')
    ap.add_argument('--no-grasp', action='store_true',
                    help='설정 시 닫을 때 present 가 끝까지(1150) 닫힘. '
                         '기본(미설정)은 물체에 막힌 것처럼 742 에서 멈춤(파지 흉내).')
    # rclpy 가 --ros-args 를 처리하므로, 알 수 없는 인자는 통과시킨다.
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = FakeGripperCameras(close_after=args.close_after,
                              grasp=(not args.no_grasp))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
