# =============================================================
# drone_cmd.py — PX4 드론 명령 (pymavlink, docs/PLAN.md 2단계 2번 P-5). 실물 PX4 드론과 같은 MAVLink 명령
#
#   PX4Commander: PX4 SITL 의 offboard API 링크 (PX4 → UDP 14540 + instance) 로 붙는다
#     - 위치 목표는 Offboard 모드 SET_POSITION_TARGET_LOCAL_NED (위치 + yaw) 를 계속 보냄 (PX4 는 2 Hz 이상 끊기지 않아야 유지)
#     - fly_to(target): setpoint 를 먼저 보내고 → Offboard 모드 요청 → arm 요청 (이루어질 때까지 retry_sec 마다 재요청).
#       바닥에서 이륙도 같은 방식 (목표를 위로)
#     - goto(target, yaw) = 목표 바꿈, hold() = 현재 추정 위치를 목표로, land() = AUTO.LAND, kill() = 공중 강제 정지
#       (MAV_CMD_COMPONENT_ARM_DISARM, param2 21196 → 모터 즉시 정지. 실물: 잡은 뒤 모터를 끔)
#   좌표: world ENU (m, 로봇 base 기준 world) ↔ PX4 local NED. PX4 local 원점 = EKF 가 초기화된 위치 = 드론 시작 위치 (origin_enu)
#     NED = (y − oy, x − ox, −(z − oz)), yaw_ned = π/2 − yaw_enu. PX4 가 추정한 위치(estimate_enu)는 실제 위치와 다를 수 있음 (실물과 같음)
#   시간: tick(t) 를 sim 시간으로 부름 (씬 루프). lockstep 이라 PX4 시계 = sim 시간
#
#   다른 터미널에서 명령 (CLI, 이 파일을 직접 실행): 씬 안 PX4Commander 의 명령 포트(px4_sitl.yaml command_port, UDP localhost)로
#     JSON 명령을 보내고 답을 받는다. PX4 Offboard 는 위치 목표를 끊김 없이 받아야 해서, 목표는 씬 안 PX4Commander 가 계속 보내고
#     CLI 는 그 목표를 바꿈 (실물에서 보조 컴퓨터가 setpoint 를 보내고 사람이 높은 수준 명령을 주는 구성과 같음)
#       python3 isaacsim/scripts/drone_cmd.py status
#       python3 isaacsim/scripts/drone_cmd.py goto 0.6 0.1 1.5 [--yaw-deg 0]
#       python3 isaacsim/scripts/drone_cmd.py hold | land | kill
#     CLI 로 goto·hold 하면 씬의 hover 사인파 목표는 멈춤 (external). 씬이 이륙·waypoint 를 마치기 전(status 의 ready false)에는
#     이동 명령을 거부 (kill 은 안전을 위해 항상 받음)
# =============================================================

import json
import math
import os
import socket
import sys

import numpy as np

PYDEPS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), ".pydeps")
SITL_CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "px4_sitl.yaml")
mavutil = M = None


def _load_mavlink():
    """pymavlink 은 PX4Commander 를 만들 때만 (CLI 는 필요 없음)."""
    global mavutil, M
    if mavutil is None:
        if PYDEPS not in sys.path:
            sys.path.insert(0, PYDEPS)
        from pymavlink import mavutil as mu
        mavutil, M = mu, mu.mavlink
MAIN_MODE = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO", 5: "ACRO", 6: "OFFBOARD", 7: "STABILIZED", 8: "RATTITUDE"}
AUTO_SUB = {1: "READY", 2: "TAKEOFF", 3: "LOITER", 4: "MISSION", 5: "RTL", 6: "LAND", 8: "FOLLOW", 9: "PRECLAND"}
POS_YAW_MASK = 0b0000_1001_1111_1000     # 위치 + yaw 만 사용 (속도·가속도·yaw rate 무시)
KILL_MAGIC = 21196


def mode_name(custom_mode):
    main, sub = (custom_mode >> 16) & 0xFF, (custom_mode >> 24) & 0xFF
    name = MAIN_MODE.get(main, f"main{main}")
    if main == 4:
        name += "." + AUTO_SUB.get(sub, f"sub{sub}")
    return name


class CommandServer:
    """다른 터미널 CLI 의 JSON 명령 (UDP localhost). poll() 은 막지 않음."""

    def __init__(self, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind(("127.0.0.1", int(port)))
        except OSError as e:
            raise RuntimeError(f"드론 명령 포트 {port} 를 열 수 없음 (다른 씬이 쓰는 중?): {e}") from e
        self.sock.setblocking(False)

    def poll(self):
        out = []
        while True:
            try:
                data, addr = self.sock.recvfrom(4096)
            except BlockingIOError:
                return out
            try:
                out.append((json.loads(data.decode()), addr))
            except ValueError:
                self.reply(addr, {"ok": False, "error": "JSON 이 아님"})

    def reply(self, addr, obj):
        self.sock.sendto(json.dumps(obj, ensure_ascii=False).encode(), addr)

    def close(self):
        self.sock.close()


class PX4Commander:
    def __init__(self, port, origin_enu, setpoint_hz=20.0, retry_sec=1.0, target_system=1, command_port=None, disable_streams=()):
        _load_mavlink()
        # 연결되면 끌 스트림 (px4_sitl.yaml commander.disable_streams). 이름이 틀리면 에러
        #   메시지 ID 는 MAVLink 2 common 에서 찾는다 (M 은 처음 import 때 v1 dialect 일 수 있고 ODOMETRY 등이 없음. PX4 는 MAVLink 2)
        from pymavlink.dialects.v20 import common as mav2

        self.disable_ids = []
        for name in disable_streams:
            mid = getattr(mav2, f"MAVLINK_MSG_ID_{name}", None)
            if mid is None:
                raise ValueError(f"모르는 MAVLink 메시지 이름: {name}")
            self.disable_ids.append(int(mid))
        self._streams_sent = False
        self.conn = mavutil.mavlink_connection(f"udpin:0.0.0.0:{int(port)}")
        self.server = CommandServer(command_port) if command_port is not None else None
        self.external = False       # CLI 가 목표를 바꿨음 (씬의 hover 목표 갱신 멈춤)
        self.cli_ready = False      # 씬이 이륙·waypoint 를 마친 뒤 True (그 전에는 이동 명령 거부, status·kill 은 항상)
        self.cli_log = []           # (t, 명령, 결과)
        self.origin = np.asarray(origin_enu, dtype=float)
        self.sp_period, self.retry = 1.0 / float(setpoint_hz), float(retry_sec)
        self.sys = int(target_system)
        self.target, self.yaw = None, 0.0
        self.want_offboard, self.want_armed = False, False
        self.armed, self.mode, self.landed = False, None, None
        self.estimate_enu, self.estimate_yaw = None, None
        self.messages = []          # (t, 심각도, 글) PX4 STATUSTEXT
        self.acks = []              # (t, command, result)
        self.t, self._t_sp, self._t_req = 0.0, -1e9, -1e9
        self.connected = False

    def reset_link(self):
        """PX4 를 새로 띄운 뒤: 연결·상태를 처음처럼 (스트림 끄기도 새 PX4 에 다시 보냄). 소켓은 그대로."""
        self.connected, self._streams_sent = False, False
        self.armed, self.mode, self.landed = False, None, None
        self.estimate_enu, self.estimate_yaw = None, None
        self.target, self.want_offboard, self.want_armed = None, False, False
        self._t_sp, self._t_req = -1e9, -1e9

    def close(self):
        self.conn.close()
        if self.server is not None:
            self.server.close()

    # ── CLI 명령 ──
    def status(self):
        f = lambda a: None if a is None else [round(float(x), 4) for x in a]  # noqa: E731
        return {"t": round(self.t, 3), "connected": self.connected, "mode": self.mode, "armed": self.armed, "landed": self.landed,
                "estimate": f(self.estimate_enu), "target": f(self.target), "yaw_deg": round(math.degrees(self.yaw), 2),
                "external": self.external, "ready": self.cli_ready}

    def _handle_cli(self, req):
        cmd = req.get("cmd")
        if cmd == "status":
            return {"ok": True, **self.status()}
        if cmd in ("goto", "hold", "land") and not self.cli_ready:
            return {"ok": False, "error": "씬이 아직 이륙·waypoint 중 (ready 가 true 가 된 뒤 명령)", **self.status()}
        if cmd in ("goto", "hold") and not (self.armed and self.mode == "OFFBOARD"):
            return {"ok": False, "error": f"비행 중(Offboard, armed)이 아님: 모드 {self.mode}, armed {self.armed}", **self.status()}
        if cmd == "goto":
            pos = req.get("pos")
            if not (isinstance(pos, list) and len(pos) == 3):
                return {"ok": False, "error": "goto 는 pos [x, y, z] (world m)"}
            yaw = req.get("yaw_deg")
            self.goto([float(v) for v in pos], None if yaw is None else math.radians(float(yaw)))
            self.external = True
        elif cmd == "hold":
            self.hold()
            self.external = True
        elif cmd == "land":
            self.land()
            self.external = True
        elif cmd == "kill":
            self.kill()
            self.external = True
        else:
            return {"ok": False, "error": f"모르는 명령: {cmd} (goto, hold, land, kill, status)"}
        return {"ok": True, **self.status()}

    # ── 좌표 ──
    def enu_to_ned(self, p):
        d = np.asarray(p, dtype=float) - self.origin
        return np.array([d[1], d[0], -d[2]])

    def ned_to_enu(self, n):
        return self.origin + np.array([n[1], n[0], -n[2]])

    # ── 수신 ──
    def poll(self):
        while True:
            m = self.conn.recv_match(blocking=False)
            if m is None:
                return
            if m.get_srcSystem() != self.sys:
                continue
            k = m.get_type()
            if k == "HEARTBEAT" and m.get_srcComponent() == 1:
                self.connected = True
                self.armed = bool(m.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
                self.mode = mode_name(m.custom_mode)
            elif k == "LOCAL_POSITION_NED":
                self.estimate_enu = self.ned_to_enu([m.x, m.y, m.z])
            elif k == "ATTITUDE":
                self.estimate_yaw = math.pi / 2 - m.yaw
            elif k == "EXTENDED_SYS_STATE":
                self.landed = {0: "undefined", 1: "on_ground", 2: "in_air", 3: "takeoff", 4: "landing"}.get(m.landed_state)
            elif k == "STATUSTEXT":
                self.messages.append((round(self.t, 3), int(m.severity), m.text))
            elif k == "COMMAND_ACK":
                self.acks.append((round(self.t, 3), int(m.command), int(m.result)))
                if m.command == M.MAV_CMD_SET_MESSAGE_INTERVAL and m.result != M.MAV_RESULT_ACCEPTED:
                    raise RuntimeError(f"PX4 가 스트림 끄기(SET_MESSAGE_INTERVAL)를 거부: result {m.result}")

    # ── 송신 ──
    def _command(self, cmd, *params):
        p = list(params) + [0.0] * (7 - len(params))
        self.conn.mav.command_long_send(self.sys, 1, cmd, 0, *p)

    def _send_setpoint(self):
        n = self.enu_to_ned(self.target)
        self.conn.mav.set_position_target_local_ned_send(int(self.t * 1000), self.sys, 1, M.MAV_FRAME_LOCAL_NED, POS_YAW_MASK,
                                                         n[0], n[1], n[2], 0, 0, 0, 0, 0, 0, math.pi / 2 - self.yaw, 0)

    def _send_gcs_heartbeat(self):
        self.conn.mav.heartbeat_send(M.MAV_TYPE_GCS, M.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    # ── 명령 ──
    def fly_to(self, target_enu, yaw_enu=0.0):
        """Offboard 위치 목표 + arm (이륙 포함). 이루어질 때까지 tick() 이 재요청."""
        self.goto(target_enu, yaw_enu)
        self.want_offboard, self.want_armed = True, True

    def goto(self, target_enu, yaw_enu=None):
        self.target = np.asarray(target_enu, dtype=float)
        if yaw_enu is not None:
            self.yaw = float(yaw_enu)

    def hold(self):
        if self.estimate_enu is None:
            raise RuntimeError("PX4 위치 추정을 아직 받지 못함")
        self.goto(self.estimate_enu.copy())

    def land(self):
        self.want_offboard = False
        self.target = None
        self._command(M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 6)

    def kill(self):
        """공중 강제 정지 (force disarm). 모터 즉시 정지."""
        self.want_offboard, self.want_armed = False, False
        self.target = None
        self._command(M.MAV_CMD_COMPONENT_ARM_DISARM, 0, KILL_MAGIC)

    def tick(self, t):
        """sim 시간 t 로 매 루프 호출: 수신, setpoint 송신, 모드·arm 재요청."""
        self.t = float(t)
        self.poll()
        if self.connected and not self._streams_sent:
            for mid in self.disable_ids:
                self._command(M.MAV_CMD_SET_MESSAGE_INTERVAL, mid, -1)
            self._streams_sent = True
        if self.server is not None:
            for req, addr in self.server.poll():
                res = self._handle_cli(req)
                self.cli_log.append((round(self.t, 3), req, res.get("ok"), res.get("error")))
                self.server.reply(addr, res)
        if self.target is not None and self.t - self._t_sp >= self.sp_period:
            self._send_setpoint()
            self._t_sp = self.t
        if self.t - self._t_req >= self.retry:
            self._send_gcs_heartbeat()
            if self.connected and self.want_offboard and self.target is not None and self.mode != "OFFBOARD":
                self._command(M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6, 0)
            elif self.connected and self.want_armed and self.mode == "OFFBOARD" and not self.armed:
                self._command(M.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            self._t_req = self.t


# ── CLI (다른 터미널) ──
def main():
    import argparse

    import yaml

    with open(SITL_CONFIG, encoding="utf-8") as fh:
        sitl = yaml.safe_load(fh)
    ap = argparse.ArgumentParser(description="실행 중인 sim 의 PX4 드론에 명령 (world 좌표 m, 로봇 base 기준)")
    ap.add_argument("--port", type=int, default=int(sitl["command_port"]) + int(sitl["instance"]))
    ap.add_argument("--timeout", type=float, default=3.0, help="답 기다리는 시간 [s, 실제 시간]")
    sub = ap.add_subparsers(dest="cmd", required=True)
    g = sub.add_parser("goto", help="위치 목표 (x y z, world m)")
    g.add_argument("x", type=float)
    g.add_argument("y", type=float)
    g.add_argument("z", type=float)
    g.add_argument("--yaw-deg", type=float, default=None, help="yaw [deg, world +x 기준 반시계] (기본: 그대로)")
    for c, h in (("hold", "지금 추정 위치에 멈춤"), ("land", "착륙 (AUTO.LAND)"), ("kill", "공중 강제 정지 (모터 즉시 정지)"),
                 ("status", "모드·arm·추정 위치·목표")):
        sub.add_parser(c, help=h)
    a = ap.parse_args()
    req = {"cmd": a.cmd}
    if a.cmd == "goto":
        req["pos"] = [a.x, a.y, a.z]
        if a.yaw_deg is not None:
            req["yaw_deg"] = a.yaw_deg
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(a.timeout)
    sock.sendto(json.dumps(req).encode(), ("127.0.0.1", a.port))
    try:
        data, _ = sock.recvfrom(4096)
    except socket.timeout:
        print(f"답 없음 ({a.timeout} s): PX4 드론 씬이 실행 중인지 확인 (명령 포트 {a.port})", file=sys.stderr)
        return 2
    res = json.loads(data.decode())
    print(json.dumps(res, ensure_ascii=False, indent=1))
    return 0 if res.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
