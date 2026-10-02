#!/usr/bin/env python3
# =============================================================
# topic_rate.py  (docs/PLAN.md 3단계)
#
# 외부 프로세스(시스템 python3, source /opt/ros/jazzy/setup.bash)에서 토픽을 받아 도착 시각(wall)과 header.stamp(sim time)를 기록.
# sim 프로세스 밖에서 재야 실제 받는 쪽 주기가 나온다 (ros2 topic hz 는 wall 기준만).
#   python3 isaacsim/scripts/topic_rate.py --out rate.json --duration 60 \
#       /cam/wrist/color/image_raw:sensor_msgs/msg/Image /clock:rosgraph_msgs/msg/Clock
#   SIGTERM/SIGINT 또는 --duration 이 지나면 JSON 을 쓰고 끝난다: {topic: [[wall, stamp], ...]} (stamp 없는 메시지는 /clock 의 clock)
#   --save-dir D --save-at T1 T2 ... : 이미지 토픽에서 stamp ≥ Ti 인 첫 이미지를 PNG 로 (받은 그대로, 녹화될 이미지와 같음)
# =============================================================

import argparse
import importlib
import json
import signal
import time

import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data


def msg_class(type_name):
    pkg, kind, name = type_name.split("/")
    return getattr(importlib.import_module(f"{pkg}.{kind}"), name)


def stamp_of(msg):
    st = msg.header.stamp if hasattr(msg, "header") else msg.clock
    return st.sec + st.nanosec * 1e-9


def save_image(msg, path):
    """sensor_msgs/Image (rgb8·bgr8) → PNG. 다른 encoding 은 에러."""
    import cv2

    if msg.encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"저장할 수 없는 encoding: {msg.encoding}")
    img = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    cv2.imwrite(path, img[..., ::-1] if msg.encoding == "rgb8" else img)


def main():
    p = argparse.ArgumentParser(description="토픽 도착 시각·stamp 기록")
    p.add_argument("topics", nargs="+", metavar="TOPIC:TYPE")
    p.add_argument("--out", required=True)
    p.add_argument("--duration", type=float, default=600.0, help="[s] wall")
    p.add_argument("--qos", choices=("best_effort", "reliable"), default="reliable",
                   help="받는 쪽 QoS (best_effort 는 큰 이미지가 UDP 조각 손실로 버려질 수 있음)")
    p.add_argument("--save-dir", default=None, help="이미지 PNG 저장 폴더")
    p.add_argument("--save-at", type=float, nargs="*", default=[], help="저장할 sim 시각 [s] (stamp ≥ 이 값인 첫 이미지)")
    args = p.parse_args()

    rclpy.init()
    node = Node("topic_rate")
    rec = {}
    qos = qos_profile_sensor_data if args.qos == "best_effort" else QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    subs = []
    for spec in args.topics:
        topic, type_name = spec.split(":")
        rec[topic] = []

        pending = sorted(args.save_at) if args.save_dir else []

        def cb(msg, topic=topic, pending=pending):
            st = stamp_of(msg)
            rec[topic].append((time.time(), st))
            while pending and st >= pending[0]:
                save_image(msg, os.path.join(args.save_dir, f"{topic.strip('/').replace('/', '_')}_{pending.pop(0):.2f}.png"))

        subs.append(node.create_subscription(msg_class(type_name), topic, cb, qos))
    stop = {"flag": False}
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.update(flag=True))
    t_end = time.time() + args.duration
    print("topic_rate ready", flush=True)
    while not stop["flag"] and time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(rec, f)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
