#!/usr/bin/env python3
# =============================================================
# replay_commands.py  (docs/PLAN.md 3단계)
#
# 팔·그리퍼 명령 CSV (grasp_demo.py 가 남기는 commands_<케이스>.csv: t, 팔 6 관절 [rad], gripper_goal_raw) 를
# 실행 중인 sim_ros2 에 ROS 2 로 재생한다 (텔레옵 대신). 시스템 python3 (source /opt/ros/jazzy/setup.bash).
#   /joint_command (sensor_msgs/JointState, 팔 6 관절) 매 줄, /gripper/command (std_msgs/Float64) 는 값이 바뀔 때
#   시각은 sim time 기준 (/clock): 시작 = 첫 /clock + --start-delay, 줄의 t 만큼 지난 뒤 보냄 (RTF 와 무관하게 sim 기준 같은 궤적)
#   재생하는 동안 /gripper/joint_states·/gripper/target 을 받아 끝에 요약 (그리퍼 닫은 뒤 present 최종값·target)
#   시작 전에 sim 노드 (--sim-node) 의 구독이 두 명령 토픽에 연결될 때까지 기다린다 (안 되면 에러). 연결 전에 보내기 시작하면 sim 이 앞부분을 못 받다가
#     나중 명령부터 받아 팔 목표가 뛴다 (2026-10-08 make_fake_episodes: 처음 1.7 s 를 못 받고 36° 뛰어 보호 정지)
# 실행:
#   python3 isaacsim/scripts/replay_commands.py isaacsim/reports/grasp_demo_<시각>/commands_success.csv [--start-delay 1] [--tail 3]
#   --out result.json : 요약을 JSON 으로
# =============================================================

import argparse
import csv
import json
import sys

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def stamp(s):
    return s.sec + s.nanosec * 1e-9


def main():
    p = argparse.ArgumentParser(description="명령 CSV 를 sim_ros2 에 ROS 2 로 재생")
    p.add_argument("csv")
    p.add_argument("--start-delay", type=float, default=1.0, help="[sim s] 첫 /clock 뒤 재생 시작까지")
    p.add_argument("--tail", type=float, default=3.0, help="[sim s] 마지막 줄 뒤 더 기다림")
    p.add_argument("--clock-timeout", type=float, default=30.0, help="[wall s] /clock 기다리는 시간")
    p.add_argument("--sim-node", default="isaac_sim", help="sim 의 ROS 2 노드 이름 (ros2_iface.yaml node_name). 이 노드의 구독이 연결된 뒤 시작")
    p.add_argument("--connect-timeout", type=float, default=15.0, help="[wall s] sim 구독 연결을 기다리는 시간")
    p.add_argument("--out", default=None, help="요약 JSON")
    a = p.parse_args()
    with open(a.csv, encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    need = ["t", *ARM, "gripper_goal_raw"]
    if not rows or any(k not in rows[0] for k in need):
        sys.exit(f"CSV 열이 {need} 이어야 함: {a.csv}")

    rclpy.init()
    n = Node("replay_commands")
    clk = [None]
    grip = {"present": [], "target": []}
    n.create_subscription(Clock, "/clock", lambda m: clk.__setitem__(0, stamp(m.clock)), 10)
    n.create_subscription(JointState, "/gripper/joint_states", lambda m: grip["present"].append((stamp(m.header.stamp), m.position[0])), 100)
    n.create_subscription(JointState, "/gripper/target", lambda m: grip["target"].append((stamp(m.header.stamp), m.position[0])), 100)
    p_arm = n.create_publisher(JointState, "/joint_command", 10)
    p_grip = n.create_publisher(Float64, "/gripper/command", 10)

    import time
    w_end = time.time() + a.clock_timeout
    while clk[0] is None:
        rclpy.spin_once(n, timeout_sec=0.05)
        if time.time() > w_end:
            sys.exit(f"/clock 을 {a.clock_timeout} s 안에 받지 못함 (sim_ros2 실행 중?)")

    def connected(pub, topic):
        infos = n.get_subscriptions_info_by_topic(topic)
        return any(i.node_name == a.sim_node for i in infos) and pub.get_subscription_count() >= len(infos)

    w_end = time.time() + a.connect_timeout
    while not (connected(p_arm, "/joint_command") and connected(p_grip, "/gripper/command")):
        rclpy.spin_once(n, timeout_sec=0.05)
        if time.time() > w_end:
            seen = {t: [i.node_name for i in n.get_subscriptions_info_by_topic(t)] for t in ("/joint_command", "/gripper/command")}
            sys.exit(f"sim 노드 '{a.sim_node}' 의 명령 구독이 {a.connect_timeout} s 안에 연결되지 않음 (보이는 구독자 {seen})")

    def spin_until(t):
        while clk[0] < t:
            rclpy.spin_once(n, timeout_sec=0.002)

    t0 = clk[0] + a.start_delay
    spin_until(t0)
    last_g, t_close, sent = None, None, 0
    for r in rows:
        spin_until(t0 + float(r["t"]))
        m = JointState()
        # stamp = 보낸 순간의 sim time (stage1 이 header.stamp 로 팔 action 을 맞춘다. 텔레옵 장치도 같은 시계로 stamp 를 넣어야 함)
        m.header.stamp.sec, m.header.stamp.nanosec = int(clk[0]), min(int(round((clk[0] - int(clk[0])) * 1e9)), 999999999)
        m.name = ARM
        m.position = [float(r[j]) for j in ARM]
        p_arm.publish(m)
        sent += 1
        g = float(r["gripper_goal_raw"])
        if g != last_g:
            mg = Float64()
            mg.data = g
            p_grip.publish(mg)
            if g > 0 and t_close is None:
                t_close = clk[0]
            last_g = g
    t_last = clk[0]
    spin_until(t_last + a.tail)

    res = {"csv": a.csv, "rows_sent": sent, "sim_start": t0, "sim_end": t_last, "gripper_close_t": t_close}
    if grip["present"]:
        res["present_final"] = grip["present"][-1][1]
        res["target_final"] = grip["target"][-1][1] if grip["target"] else None
    print("REPLAY " + json.dumps(res, ensure_ascii=False), flush=True)
    if a.out:
        with open(a.out, "w", encoding="utf-8") as f:
            json.dump(res, f, ensure_ascii=False, indent=1)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
