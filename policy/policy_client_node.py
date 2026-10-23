#!/usr/bin/env python3
# =============================================================
# policy_client_node.py  (docs/openpi_finetuning.md 8장, docs/PLAN.md 7단계 sim 폐루프)
#
# openpi 정책 서버 (scripts/serve_policy.py) 로 sim (또는 실물) 을 움직이는 노드. 텔레옵 노드 자리를 대신한다:
#   받기  /joint_states (팔 6), /gripper/joint_states (present raw), 카메라 2 대 (rgb8 640x480), /clock, /protective_stop
#   보내기 /joint_command (sensor_msgs/JointState, stamp = sim time), /gripper/command (std_msgs/Float64, 0 / 1150, 바뀔 때)
#   관측 = 학습 데이터셋과 같은 모양: state (7) = 팔 6 [rad] + present / 1150, base = third view, wrist = 손목, prompt
#   chunk (10, 7) 를 받아 replan_steps 칸을 control_hz 간격 (sim time) 으로 실행. 추론하는 동안 지나간 칸은 건너뛰고,
#   칸 사이는 publish_hz 로 선형 보간. 보내는 목표는 텔레옵과 같은 안전 한계 (속도·측정 대비 앞섬·관절 한계) 로 자른다
#   그리퍼: 출력 > threshold → 닫기 (1150), 아니면 열기 (0)
#   멈춤: 보호 정지, 측정 관절각이 목표와 desync 넘게 다름, 관측이 오래됨 (에러), --duration 이 지남
#
# 실행 (sim_ros2.py 와 정책 서버를 먼저 띄운다. 시스템 ROS + openpi-client 가 든 venv):
#   source /opt/ros/jazzy/setup.bash
#   ~/openpi_client/venv/bin/python policy/policy_client_node.py --reset --duration 40
#   --episodes N : 리셋 → 실행을 N 번.   --auto-kill S : 닫기 명령 S 초 뒤 그리퍼가 물체에 막혀 있으면 (present < 0.9) 드론 모터 정지
#   venv 만들기 (한 번): python3 -m venv --system-site-packages ~/openpi_client/venv
#                        ~/openpi_client/venv/bin/pip install -e ~/openpi/packages/openpi-client
# =============================================================

import argparse
import json
import math
import os
import sys
import threading
import time
from collections import deque

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, "teleop"))
import ik_model as km  # noqa: E402  (관절 이름·한계를 텔레옵과 같은 URDF 에서)

REQUIRED = ("server", "prompt", "control_hz", "replan_steps", "publish_hz", "obs_max_age", "gripper", "max_joint_speed",
            "max_lead", "limit_margin", "desync", "topics")
TOPICS = ("clock", "joint_states", "gripper_state", "wrist_image", "third_view_image", "protective_stop", "joint_command",
          "gripper_command", "reset_service", "kill_service")


def load_config(path):
    with open(path) as f:
        c = yaml.safe_load(f)
    missing = [k for k in REQUIRED if k not in c] + [f"topics.{k}" for k in TOPICS if k not in c.get("topics", {})]
    missing += [f"gripper.{k}" for k in ("threshold", "raw_open", "raw_close", "present_scale") if k not in c.get("gripper", {})]
    if missing:
        raise ValueError(f"{path}: 항목이 없음: {missing}")
    if not (1 <= int(c["replan_steps"])):
        raise ValueError(f"{path}: replan_steps 는 1 이상: {c['replan_steps']}")
    if float(c["max_lead"]) >= float(c["desync"]):
        raise ValueError(f"{path}: max_lead ({c['max_lead']}) 는 desync ({c['desync']}) 보다 작아야 함")
    return c


def t_of(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class PolicyClient:
    def __init__(self, cfg, policy, log=lambda m: print(m, flush=True)):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Image, JointState
        from std_msgs.msg import Bool, Float64
        from std_srvs.srv import Trigger

        self.cfg, self.policy, self.log = cfg, policy, log
        self.JointState, self.Float64, self.Trigger = JointState, Float64, Trigger
        self._rclpy = rclpy
        self.node = Node("policy_client")
        lo, hi = km.ArmModel(km.DEFAULT_URDF).limits
        m = float(cfg["limit_margin"])
        self.lo, self.hi = lo + m, hi - m
        self.lock = threading.Lock()
        self.clock = None                            # 최신 sim time [s]
        # 받은 메시지를 짧게 모아 둔다: 카메라는 렌더·전송 때문에 /joint_states 보다 1 ~ 2 프레임 늦게 도착하므로,
        # 이미지 시각을 기준으로 그 시각의 관절값·그리퍼를 고른다 (stage1 이 같은 시각끼리 짝지은 것과 같게)
        self.js = deque(maxlen=120)                  # (stamp, q 6), 120 Hz → 1 s
        self.grip = deque(maxlen=30)                 # (stamp, present raw), 30 Hz → 1 s
        self.img = {"wrist": deque(maxlen=4), "third_view": deque(maxlen=4)}   # (stamp, HxWx3 uint8)
        self.pstop = False
        self.err = None                              # 콜백에서 찾은 입력 문제 (spin 스레드에서 던지면 그 스레드만 죽으므로 메인에서 던진다)
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        t = cfg["topics"]
        self.node.create_subscription(Clock, t["clock"], self._on_clock, latest)
        self.node.create_subscription(JointState, t["joint_states"], self._on_js, latest)
        self.node.create_subscription(JointState, t["gripper_state"], self._on_grip, latest)
        self.node.create_subscription(Image, t["wrist_image"], lambda msg: self._on_img("wrist", msg), latest)
        self.node.create_subscription(Image, t["third_view_image"], lambda msg: self._on_img("third_view", msg), latest)
        self.node.create_subscription(Bool, t["protective_stop"], self._on_pstop, latest)
        self.pub_arm = self.node.create_publisher(JointState, t["joint_command"], 10)
        self.pub_grip = self.node.create_publisher(Float64, t["gripper_command"], 10)
        self.cli_reset = self.node.create_client(Trigger, t["reset_service"])
        self.cli_kill = self.node.create_client(Trigger, t["kill_service"])
        self._stop = threading.Event()
        self._spin = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin.start()

    # ── ROS 콜백 (spin 스레드) ──
    def _on_clock(self, msg):
        with self.lock:
            self.clock = t_of(msg.clock)

    def _on_js(self, msg):
        d = dict(zip(msg.name, msg.position))
        missing = [n for n in km.ARM_JOINTS if n not in d]
        if missing:
            self.err = f"/joint_states 에 관절 {missing} 이 없음 (받은 이름 {list(msg.name)})"
            return
        with self.lock:
            self.js.append((t_of(msg.header.stamp), np.array([d[n] for n in km.ARM_JOINTS], dtype=float)))

    def _on_grip(self, msg):
        with self.lock:
            self.grip.append((t_of(msg.header.stamp), float(msg.position[0])))

    def _on_img(self, role, msg):
        if msg.encoding != "rgb8":
            self.err = f"{role} 이미지 encoding 이 rgb8 이 아님: {msg.encoding} (학습 데이터는 rgb8)"
            return
        a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3).copy()
        with self.lock:
            self.img[role].append((t_of(msg.header.stamp), a))

    def _on_pstop(self, msg):
        with self.lock:
            self.pstop = bool(msg.data)

    def _spin_loop(self):
        from rclpy.executors import SingleThreadedExecutor
        ex = SingleThreadedExecutor()
        ex.add_node(self.node)
        while not self._stop.is_set() and self._rclpy.ok():
            ex.spin_once(timeout_sec=0.05)

    # ── 도우미 ──
    def now(self):
        with self.lock:
            return self.clock

    def wait_sim_until(self, t_target, poll=0.002):
        while not self._stop.is_set():
            now = self.now()
            if now is not None and now >= t_target:
                return now
            time.sleep(poll)
        raise KeyboardInterrupt

    def wait_ready(self, timeout=60.0):
        """모든 관측이 들어오고, sim 이 두 명령 토픽을 구독할 때까지 (연결 전에 보내면 앞부분을 못 받아 목표가 뛴다, replay_commands.py)."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            with self.lock:
                have = {"clock": self.clock is not None, "joint_states": bool(self.js), "gripper": bool(self.grip),
                        "wrist": bool(self.img["wrist"]), "third_view": bool(self.img["third_view"])}
            subs = {"joint_command": self.pub_arm.get_subscription_count() > 0,
                    "gripper_command": self.pub_grip.get_subscription_count() > 0}
            if self.err:
                raise RuntimeError(self.err)
            if all(have.values()) and all(subs.values()):
                # 막 받기 시작했을 때는 이미지 (늦게 도착) 보다 앞선 관절값이 아직 없을 수 있다 → 짝지을 수 있을 때까지
                try:
                    self.observe()
                    return
                except RuntimeError as e:
                    last_obs_err = str(e)
                    if time.monotonic() - t0 > 10.0:
                        raise RuntimeError(f"관측을 짝지을 수 없음: {last_obs_err}") from e
            time.sleep(0.2)
        raise RuntimeError(f"{timeout:g} s 안에 준비 안 됨: 관측 {have}, 명령 구독자 {subs} (sim_ros2.py 가 떠 있는지)")

    def call(self, client, name, timeout):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"서비스 {name} 가 없음 (sim_ros2.py 가 떠 있는지)")
        fut = client.call_async(self.Trigger.Request())
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > timeout:
                raise RuntimeError(f"서비스 {name} 응답이 {timeout:g} s 안에 안 옴")
            time.sleep(0.05)
        return fut.result()

    def observe(self):
        """학습 데이터셋과 같은 관측. 기준 시각 t_obs = 두 카메라 중 늦은 쪽의 최신 이미지 시각 (그보다 늦지 않은 것끼리 고름).
        고른 메시지가 t_obs 보다 obs_max_age 넘게 오래되면 에러. 돌려줌: obs, t_obs, 지금 측정 관절각, 지금 present (0~1), 보호 정지."""
        if self.err:
            raise RuntimeError(self.err)
        with self.lock:
            js, grip, pstop = list(self.js), list(self.grip), self.pstop
            img = {k: list(v) for k, v in self.img.items()}
        t_obs = min(v[-1][0] for v in img.values())

        def pick(buf, name):
            m = [x for x in buf if x[0] <= t_obs + 1e-6]
            if not m:
                raise RuntimeError(f"{name}: t_obs {t_obs:.3f} 이전 메시지가 없음 (가장 오래된 것 {buf[0][0]:.3f})")
            return m[-1]

        sel = {"joint_states": pick(js, "joint_states"), "gripper": pick(grip, "gripper"),
               "wrist": pick(img["wrist"], "wrist"), "third_view": pick(img["third_view"], "third_view")}
        old = {k: round(t_obs - v[0], 3) for k, v in sel.items() if t_obs - v[0] > float(self.cfg["obs_max_age"])}
        if old:
            raise RuntimeError(f"관측이 기준 시각보다 오래됨 [s]: {old} (기준 {self.cfg['obs_max_age']})")
        sc = float(self.cfg["gripper"]["present_scale"])
        state = np.concatenate([sel["joint_states"][1], [sel["gripper"][1] / sc]]).astype(np.float32)
        obs = {"observation/state": state, "observation/base_image": sel["third_view"][1],
               "observation/wrist_image": sel["wrist"][1], "prompt": self.cfg["prompt"]}
        return obs, t_obs, js[-1][1].copy(), grip[-1][1] / sc, pstop

    def safe_target(self, q_target, q_cmd, q_meas, dt):
        """텔레옵 ik_step 과 같은 한계: 한 번에 max_joint_speed·dt, 측정 ± max_lead, 관절 한계 안쪽."""
        max_dq = float(self.cfg["max_joint_speed"]) * dt
        q = q_cmd + np.clip(q_target - q_cmd, -max_dq, max_dq)
        lead = float(self.cfg["max_lead"])
        q = np.clip(q, q_meas - lead, q_meas + lead)
        return np.clip(q, self.lo, self.hi)

    def send_arm(self, q, t_sim):
        msg = self.JointState()
        sec = int(math.floor(t_sim))
        msg.header.stamp.sec, msg.header.stamp.nanosec = sec, int(round((t_sim - sec) * 1e9)) % 1_000_000_000
        msg.name = list(km.ARM_JOINTS)
        msg.position = [float(x) for x in q]
        self.pub_arm.publish(msg)

    def send_grip(self, closed):
        g = self.cfg["gripper"]
        self.pub_grip.publish(self.Float64(data=float(g["raw_close"] if closed else g["raw_open"])))

    # ── 에피소드 ──
    def _infer(self):
        """관측 → 정책. 돌려줌 (chunk (H, 7), t_obs, 걸린 wall 시간)."""
        obs, t_obs, _, _, _ = self.observe()
        w0 = time.monotonic()
        act = np.asarray(self.policy.infer(obs)["actions"], dtype=float)
        if act.ndim != 2 or act.shape[1] != 7 or not np.all(np.isfinite(act)):
            raise RuntimeError(f"정책 출력이 (H, 7) 유한값이 아님: {act.shape}")
        return act, t_obs, time.monotonic() - w0

    def warmup(self):
        """서버의 첫 추론은 JIT 컴파일로 십수 초 걸린다 → 에피소드 전에 한 번 (로봇은 움직이지 않음)."""
        _, _, w = self._infer()
        self.log(f"[policy] 첫 추론 (서버 컴파일 포함) {w:.1f} s")

    def run_episode(self, duration, auto_kill):
        """chunk 칸 i = sim t_obs + i·dt_c 의 명령 (데이터셋 action[t] = 프레임 t 직전 최신 명령).
        publish_hz 로 지금 시각의 칸 위치를 계산해 선형 보간한 목표를 보낸다. 지금 chunk 의 replan_steps 칸에 이르면
        다음 추론을 뒤에서 시작하고 (그동안 지금 chunk 를 계속 실행), 결과가 오면 바꾼다. chunk 끝에 닿으면 마지막 칸을 유지."""
        cfg = self.cfg
        dt_c, dt_p, K = 1.0 / float(cfg["control_hz"]), 1.0 / float(cfg["publish_hz"]), int(cfg["replan_steps"])
        thr = float(cfg["gripper"]["threshold"])
        _, _, q_meas, _, _ = self.observe()
        q_cmd, closed = q_meas.copy(), False
        self.send_grip(False)
        t_start = self.now()
        t_closed, killed = None, False
        st = {"chunks": 0, "infer_wall": [], "lag_steps": [], "limited": 0, "sent": 0, "held": 0, "stop": "duration"}
        self.log(f"[policy] 에피소드 시작: sim t {t_start:.2f}, {duration:g} s, {K} 칸마다 다시 물음 / {cfg['control_hz']:g} Hz")
        act, t_obs, w = self._infer()
        st["chunks"] += 1
        st["infer_wall"].append(w)
        box, worker = {}, None                       # 뒤에서 도는 다음 추론

        def bg():
            try:
                box["res"] = self._infer()
            except Exception as e:                   # noqa: BLE001  (메인 루프에서 다시 던진다)
                box["err"] = e

        t_next = self.now()
        while True:
            t_next += dt_p
            now = self.wait_sim_until(t_next)
            if now - t_start >= duration:
                break
            with self.lock:
                pstop, q_meas = self.pstop, self.js[-1][1].copy()
            if pstop:
                st["stop"] = "protective_stop"
                break
            dev = float(np.max(np.abs(q_meas - q_cmd)))
            if dev > float(cfg["desync"]):
                st["stop"] = f"desync {math.degrees(dev):.1f} deg"
                break
            if worker is not None and not worker.is_alive():   # 다음 chunk 도착
                worker = None
                if "err" in box:
                    raise box["err"]
                act, t_obs, w = box.pop("res")
                st["chunks"] += 1
                st["infer_wall"].append(w)
                st["lag_steps"].append((now - t_obs) / dt_c)
            x = (now - t_obs) / dt_c                 # 지금 chunk 안 위치 [칸]
            if worker is None and x >= K:
                worker = threading.Thread(target=bg, daemon=True)
                worker.start()
            last = act.shape[0] - 1
            if x >= last:
                st["held"] += 1                      # 다음 chunk 가 늦음 → 마지막 칸 유지
            xc = min(max(x, 0.0), last)
            i0 = min(int(math.floor(xc)), last)
            i1 = min(i0 + 1, last)
            a = act[i0] + (xc - i0) * (act[i1] - act[i0])
            q_new = self.safe_target(a[:6], q_cmd, q_meas, dt_p)
            st["limited"] += int(not np.allclose(q_new, a[:6], atol=1e-6))
            q_cmd = q_new
            self.send_arm(q_cmd, now)
            st["sent"] += 1
            want = bool(act[i0, 6] > thr)
            if want != closed:
                closed = want
                self.send_grip(closed)
                t_closed = now if closed else None
                self.log(f"[policy] 그리퍼 {'닫기' if closed else '열기'} (에피소드 t {now - t_start:.2f} s, 출력 {act[i0, 6]:.2f})")
            if auto_kill is not None and closed and not killed and t_closed is not None and now - t_closed >= auto_kill:
                with self.lock:
                    present = self.grip[-1][1] / float(cfg["gripper"]["present_scale"])
                if present < 0.9:
                    r = self.call(self.cli_kill, cfg["topics"]["kill_service"], 10.0)
                    killed = True
                    self.log(f"[policy] 드론 모터 정지 (그리퍼 present {present:.2f} = 물체에 막힘): {r.success} {r.message[:80]}")
        if worker is not None:
            worker.join(timeout=10.0)
        _, _, _, present, _ = self.observe()
        iw = np.array(st["infer_wall"])
        lag = np.array(st["lag_steps"]) if st["lag_steps"] else np.array([np.nan])
        res = {"stop": st["stop"], "sim_s": round(self.now() - t_start, 2), "chunks": st["chunks"], "sent": st["sent"],
               "infer_wall_median_s": round(float(np.median(iw)), 3), "infer_wall_max_s": round(float(np.max(iw)), 3),
               "chunk_arrive_step_median": round(float(np.median(lag)), 2), "held_frac": round(st["held"] / max(st["sent"], 1), 3),
               "limited_frac": round(st["limited"] / max(st["sent"], 1), 3),
               "gripper_closed": closed, "gripper_present_end": round(present, 3), "drone_killed": killed}
        self.log(f"[policy] 에피소드 끝: {json.dumps(res, ensure_ascii=False)}")
        return res

    def shutdown(self):
        self._stop.set()
        self._spin.join(timeout=1.0)
        self.node.destroy_node()


def main():
    ap = argparse.ArgumentParser(description="openpi 정책 서버로 sim 을 움직이는 클라이언트 노드")
    ap.add_argument("--config", default=os.path.join(HERE, "policy_client.yaml"))
    ap.add_argument("--host", default=None, help="정책 서버 주소 (설정 덮어쓰기)")
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--duration", type=float, required=True, help="[sim s] 에피소드 길이 (데이터 에피소드 12 ~ 30 s)")
    ap.add_argument("--episodes", type=int, default=1)
    ap.add_argument("--reset", action="store_true", help="에피소드마다 먼저 /sim/reset (PX4 약 24 s, 기하 제어기 약 2 s)")
    ap.add_argument("--auto-kill", type=float, default=None, help="[sim s] 닫기 명령 뒤 이만큼 지나 그리퍼가 막혀 있으면 드론 모터 정지")
    args = ap.parse_args()
    if args.episodes > 1 and not args.reset:
        ap.error("--episodes 2 이상은 --reset 과 같이 (리셋 없이 이어 하면 다음 에피소드가 앞 결과에서 시작)")
    cfg = load_config(args.config)
    host = args.host or cfg["server"]["host"]
    port = args.port or int(cfg["server"]["port"])

    import rclpy
    from openpi_client import websocket_client_policy
    print(f"[policy] 정책 서버 {host}:{port} 연결 중 ...", flush=True)
    policy = websocket_client_policy.WebsocketClientPolicy(host=host, port=port)
    print(f"[policy] 서버 metadata: {policy.get_server_metadata()}", flush=True)
    rclpy.init()
    pc = PolicyClient(cfg, policy)
    results = []
    try:
        pc.wait_ready()
        pc.warmup()
        for ep in range(args.episodes):
            if args.reset:
                print(f"[policy] 에피소드 {ep + 1}/{args.episodes}: /sim/reset ...", flush=True)
                r = pc.call(pc.cli_reset, cfg["topics"]["reset_service"], 120.0)
                if not r.success:
                    raise RuntimeError(f"/sim/reset 실패: {r.message[:300]}")
                info = json.loads(r.message)
                print(f"[policy] 리셋 완료: 드론 기준 위치 (base) {info.get('drone_goal_base')}, 목록 점 "
                      f"{(info.get('drone_target') or {}).get('name')}", flush=True)
                t_reset = pc.now()
                pc.wait_sim_until(t_reset + 0.2)     # 리셋 뒤 관측이 새로 들어오게
            results.append(pc.run_episode(args.duration, args.auto_kill))
    except KeyboardInterrupt:
        print("[policy] 중단 (Ctrl+C)", flush=True)
    finally:
        pc.shutdown()
        rclpy.shutdown()
    if len(results) > 1:
        print(f"[policy] 요약: 그리퍼 막힘 (파지 신호) {sum(r['gripper_closed'] and r['gripper_present_end'] < 0.9 for r in results)}"
              f"/{len(results)}, 멈춤 이유 {[r['stop'] for r in results]}", flush=True)


if __name__ == "__main__":
    main()
