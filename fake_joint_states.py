#!/usr/bin/env python3
# =============================================================
# fake_joint_states.py
# 실제 UR5 드라이버가 내보내는 /joint_states 를 흉내 내는 가짜 노드.
# 로봇팔이 없을 때 전체 6-토픽 파이프라인(녹화+변환)을 테스트하기 위함.
#
# 실제 UR /joint_states 와 최대한 동일하게:
#   토픽 : /joint_states
#   타입 : sensor_msgs/JointState
#   name : UR5 관절 6개 (실제 순서)
#   position/velocity : 천천히 sine 으로 움직이는 가짜 값
#   header.stamp : 매 발행 시각
#
# 실행:  python3 fake_joint_states.py
# 종료:  Ctrl+C
#
# ※ 나중에 진짜 로봇이 준비되면 이 노드를 끄고 실제 드라이버를 켜면 된다.
#    (토픽 이름/타입/관절 이름이 같으므로 변환 코드는 그대로 동작)
# =============================================================

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


# 실제 UR5 관절 이름 (순서도 실제와 동일하게)
UR5_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# 각 관절이 왔다갔다 하는 중심값(rad) — 대략 흔한 자세
CENTERS = [0.0, -1.57, 1.0, -1.57, 0.0, 0.0]
# 각 관절의 흔들림 크기(rad) — 작게 살살
AMPLITUDES = [0.3, 0.2, 0.3, 0.2, 0.3, 0.4]
# 각 관절의 주기(초) — 서로 다르게 해서 자연스럽게
PERIODS = [8.0, 10.0, 6.0, 7.0, 9.0, 5.0]

PUBLISH_HZ = 125.0   # 실제 UR 드라이버와 비슷한 주기


class FakeJointStates(Node):
    def __init__(self):
        super().__init__('fake_joint_states')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.t = 0.0
        self.dt = 1.0 / PUBLISH_HZ
        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            f'가짜 /joint_states 발행 시작 ({PUBLISH_HZ:.0f}Hz, 관절 6개). '
            f'실제 UR 드라이버 대용. Ctrl+C 로 종료.'
        )

    def tick(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(UR5_JOINT_NAMES)

        positions = []
        velocities = []
        for i in range(6):
            w = 2.0 * math.pi / PERIODS[i]
            # 위치 = 중심 + 진폭*sin(w t)
            pos = CENTERS[i] + AMPLITUDES[i] * math.sin(w * self.t)
            # 속도 = 위치의 시간미분 = 진폭*w*cos(w t)
            vel = AMPLITUDES[i] * w * math.cos(w * self.t)
            positions.append(pos)
            velocities.append(vel)

        msg.position = positions
        msg.velocity = velocities
        # effort 는 실제 UR도 채우지만 여기선 비워둠(변환에 안 씀)

        self.pub.publish(msg)
        self.t += self.dt


def main(args=None):
    rclpy.init(args=args)
    node = FakeJointStates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()