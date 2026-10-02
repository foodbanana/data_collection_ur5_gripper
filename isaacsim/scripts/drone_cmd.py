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
# =============================================================

import math
import os
import sys

import numpy as np

PYDEPS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), ".pydeps")
if PYDEPS not in sys.path:
    sys.path.insert(0, PYDEPS)
from pymavlink import mavutil  # noqa: E402

M = mavutil.mavlink
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


class PX4Commander:
    def __init__(self, port, origin_enu, setpoint_hz=20.0, retry_sec=1.0, target_system=1):
        self.conn = mavutil.mavlink_connection(f"udpin:0.0.0.0:{int(port)}")
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

    def close(self):
        self.conn.close()

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
