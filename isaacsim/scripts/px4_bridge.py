# =============================================================
# px4_bridge.py — PX4 SITL ↔ Isaac Sim MAVLink 연결 (docs/PLAN.md 2단계 2번, Pegasus Simulator 방식)
#
#   출처: livealive7/PegasusSimulator b1256ca logic/backends/px4_mavlink_backend.py, tools/px4_launch_tool.py
#         (BSD-3-Clause, Copyright (c) 2023 Marcelo Jacinto, 라이선스 isaacsim/assets/drones/iris/LICENSE.PegasusSimulator)
#
#   PX4Launcher: PX4 SITL 을 subprocess 로 실행 (Pegasus 와 같은 명령:
#     PX4_SIM_MODEL=<airframe> build/px4_sitl_default/bin/px4 ROMFS/px4fmu_common -s rcS -i <instance> -d).
#     작업 폴더(rootfs: 파라미터·ulog 로그)는 리포트 폴더 안 새 폴더 (Pegasus 는 임시 폴더) → 비행 로그(.ulg)가 남음.
#     파라미터 덮어쓰기는 PX4 rcS 의 환경 변수 PX4_PARAM_<이름>=<값>
#   PX4Bridge: TCP 4560 (sim = 서버, PX4 가 접속), lockstep. 매 physics step (Pegasus PX4MavlinkBackend.update 순서):
#     1. (DroneFlight) 지난 step 에 받은 모터 명령으로 추력 적용
#     2. 첫 모터 명령을 받은 뒤부터는 HIL_ACTUATOR_CONTROLS 가 올 때까지 기다림 (lockstep, 시간 제한 넘으면 에러)
#     3. heartbeat (실제 시간 1 Hz), HIL_SENSOR (IMU·지자기·기압, 이번 step 새 값만 fields_updated 비트), HIL_GPS (새 값일 때)
#        시각 time_usec = sim 시간 누적 (PX4 는 이 시각으로 자기 시계를 돌림)
#     4. 센서 갱신 (다음 step 에 보냄. Pegasus 는 센서 콜백이 backend 뒤에 등록돼 보내는 값이 한 step 전 상태 → 같은 순서)
#   모터 명령 → ω: armed(mode 의 SAFETY_ARMED 비트) 이면 ωᵢ = (uᵢ + offsetᵢ)·scalingᵢ + zero_position_armedᵢ, 아니면 0.
#     [0, ω_max] 로 자름 (Pegasus QuadraticThrustCurve 와 같음). 로터 순서 = PX4 모터 순서 (Pegasus Iris 와 같음)
#   Pegasus 와 다른 점: HIL_SENSOR 온도 인자에 Pegasus 는 GPS 고도(mm 정수)를 넣음(인자 순서 오류) → 기압계 온도,
#     PX4 무응답이면 에러로 중단 (Pegasus 는 무한 대기), 연결 전 단계 상태를 status 로 노출
#
# pymavlink 은 isaacsim/.pydeps/ (pip install --target, ~/isaacsim 에 설치하지 않음)
# =============================================================

import atexit
import datetime
import os
import subprocess
import sys
import time

import numpy as np

PYDEPS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), ".pydeps")
if PYDEPS not in sys.path:
    sys.path.insert(0, PYDEPS)
from pymavlink import mavutil  # noqa: E402

import px4_sensors as ps  # noqa: E402

SENSOR_ACCEL, SENSOR_GYRO, SENSOR_MAG, SENSOR_BARO = 0b111, 0b111000, 0b111000000, 0b1101000000000
ARMED_FLAG = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED   # 128


def _die_with_parent():
    """PX4 자식 프로세스: 부모(Isaac Sim)가 죽으면 커널이 SIGTERM (Linux prctl PR_SET_PDEATHSIG).
    simulation_app.close() 는 빠른 종료라 atexit 이 돌지 않아 PX4 가 남는 일이 있었음 (2026-10-02)."""
    import ctypes
    import signal

    ctypes.CDLL("libc.so.6", use_errno=True).prctl(1, signal.SIGTERM)    # PR_SET_PDEATHSIG = 1


def running_px4(binary, instance):
    """실행 파일이 binary 이고 -i instance 인 프로세스 pid 목록 (/proc/*/cmdline)."""
    pids = []
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            with open(f"/proc/{d}/cmdline", "rb") as fh:
                argv = fh.read().split(b"\0")
        except OSError:
            continue
        if argv and argv[0].decode(errors="ignore") == binary and b"-i" in argv:
            i = argv.index(b"-i")
            if i + 1 < len(argv) and argv[i + 1] == str(instance).encode():
                pids.append(int(d))
    return pids


class PX4Launcher:
    def __init__(self, px4_dir, airframe, instance, run_dir, params=None):
        px4_dir = os.path.expanduser(px4_dir)
        self.binary = os.path.join(px4_dir, "build/px4_sitl_default/bin/px4")
        self.romfs = os.path.join(px4_dir, "ROMFS/px4fmu_common")
        self.rcs = os.path.join(self.romfs, "init.d-posix/rcS")
        for f in (self.binary, self.rcs):
            if not os.path.exists(f):
                raise FileNotFoundError(f"PX4 SITL 파일이 없습니다: {f} (~/PX4-Autopilot 에서 make px4_sitl_default none)")
        af = os.path.join(self.romfs, "init.d-posix/airframes")
        if not any(n.split("_", 1)[-1] == airframe for n in os.listdir(af)):
            raise ValueError(f"PX4 airframe '{airframe}' 가 {af} 에 없습니다")
        self.airframe, self.instance = airframe, int(instance)
        self.run_dir = run_dir
        self.params = dict(params or {})
        self.proc, self.log = None, None

    def launch(self):
        os.makedirs(self.run_dir, exist_ok=False)
        env = dict(os.environ)
        env["PX4_SIM_MODEL"] = self.airframe
        for k, v in self.params.items():
            env[f"PX4_PARAM_{k}"] = str(v)
        self.log = open(os.path.join(self.run_dir, "px4_console.log"), "w")
        # 같은 instance 의 PX4 가 이미 돌고 있으면 새 PX4 는 "server already running" 으로 바로 끝남 → 먼저 알림
        other = running_px4(self.binary, self.instance)
        if other:
            raise RuntimeError(f"PX4 instance {self.instance} 가 이미 실행 중 (pid {other}). 다른 sim 이 쓰는 중이 아니면 kill {' '.join(map(str, other))}")
        self.proc = subprocess.Popen([self.binary, self.romfs, "-s", self.rcs, "-i", str(self.instance), "-d"],
                                     cwd=self.run_dir, env=env, stdin=subprocess.DEVNULL, stdout=self.log,
                                     stderr=subprocess.STDOUT, preexec_fn=_die_with_parent)
        atexit.register(self.kill)              # 정상 종료 때 정리 (Isaac Sim close 는 atexit 를 건너뛸 수 있어 PDEATHSIG 가 주 안전장치)

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def kill(self):
        if self.proc is not None:
            if self.proc.poll() is None:
                self.proc.terminate()
                try:
                    self.proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
                    self.proc.wait()
            self.proc = None
        if self.log is not None:
            self.log.close()
            self.log = None


class PX4Bridge:
    """physics step 마다 step() 을 부른다. PX4 가 arm 하면 모터 각속도(rad/s) 를 돌려준다."""

    def __init__(self, sitl, px4c, sensors_cfg, w_max, n_rotors, seed, report_dir):
        self.sitl, self.c = sitl, px4c
        self.sensors_cfg, self.seed = sensors_cfg, seed
        self.w_max, self.n = float(w_max), int(n_rotors)
        self.offset = np.asarray(px4c["input_offset"], dtype=float)
        self.scaling = np.asarray(px4c["input_scaling"], dtype=float)
        self.zero_armed = np.asarray(px4c["zero_position_armed"], dtype=float)
        for name, a in (("input_offset", self.offset), ("input_scaling", self.scaling), ("zero_position_armed", self.zero_armed)):
            if a.shape != (self.n,):
                raise ValueError(f"flight.px4.{name} 길이 {a.shape} != 로터 {self.n}")
        stamp = f"{datetime.datetime.now():%Y%m%d_%H%M%S}"
        self.run_dir = os.path.join(report_dir, f"px4_{stamp}")
        self.port = int(sitl["sim_port"]) + int(sitl["instance"])
        self.launcher = PX4Launcher(sitl["px4_dir"], px4c["airframe"], sitl["instance"], self.run_dir, px4c.get("params"))
        self.conn = None
        self.sensors, self.dt = None, None
        self.t_us = 0
        self.heartbeat, self.first_actuator = False, False
        self.armed, self.u = False, np.zeros(self.n)
        self.omega = np.zeros(self.n)
        self.pending = None
        self.last = {"imu": None, "baro": None, "mag": None}
        self.last_hb_wall = 0.0
        self.n_actuator = 0
        self.record = False         # True 면 모터 명령마다 (sim s, armed, u...) 를 history 에 (진동 분석용)
        self.history = []

    @property
    def status(self):
        if not self.heartbeat:
            return "waiting_heartbeat"
        return "lockstep" if self.first_actuator else "waiting_actuator"

    def start(self):
        self.conn = mavutil.mavlink_connection(f"tcpin:localhost:{self.port}")   # 먼저 열고 PX4 를 띄움
        if self.sitl["autolaunch"]:
            self.launcher.launch()

    def stop(self):
        if self.conn is not None:
            self.conn.close()
            self.conn = None
        if self.sitl["autolaunch"]:
            self.launcher.kill()

    # ── 수신 ──
    def _handle(self, msg):
        if msg.get_type() == "HIL_ACTUATOR_CONTROLS":
            self.first_actuator = True
            self.n_actuator += 1
            self.armed = bool(msg.mode & ARMED_FLAG)
            self.u = np.asarray(msg.controls[: self.n], dtype=float)
            if self.record:
                self.history.append((self.t_us * 1e-6, self.armed, *self.u))
            if self.armed:
                w = (self.u + self.offset) * self.scaling + self.zero_armed
                self.omega = np.clip(w, 0.0, self.w_max)
            else:
                self.omega = np.zeros(self.n)
            return True
        return False

    def _poll(self):
        if not self.first_actuator:
            while True:
                m = self.conn.recv_match(blocking=False)
                if m is None:
                    return
                self._handle(m)
        deadline = time.monotonic() + float(self.sitl["lockstep_timeout"])
        while True:
            m = self.conn.recv_match(blocking=True, timeout=0.5)
            if m is not None and self._handle(m):
                return
            if self.sitl["autolaunch"] and not self.launcher.alive():
                raise RuntimeError(f"PX4 프로세스가 끝났습니다 (로그 {self.run_dir}/px4_console.log)")
            if time.monotonic() > deadline:
                raise RuntimeError(f"PX4 모터 명령이 {self.sitl['lockstep_timeout']} s 동안 오지 않음 (lockstep 끊김)")

    # ── 송신 ──
    def _send_sensors(self):
        d = self.pending
        fields = 0
        if d["imu"] is not None:
            fields |= SENSOR_ACCEL | SENSOR_GYRO
            self.last["imu"] = d["imu"]
        if d["mag"] is not None:
            fields |= SENSOR_MAG
            self.last["mag"] = d["mag"]
        if d["baro"] is not None:
            fields |= SENSOR_BARO
            self.last["baro"] = d["baro"]
        imu, mag, baro = self.last["imu"], self.last["mag"], self.last["baro"]
        acc, gyro = (imu["acc"], imu["gyro"]) if imu else (np.zeros(3), np.zeros(3))
        m = mag["mag"] if mag else np.zeros(3)
        b = baro or {"abs_pressure_hpa": 0.0, "pressure_alt": 0.0, "temperature": 0.0}
        self.conn.mav.hil_sensor_send(self.t_us, *map(float, acc), *map(float, gyro), *map(float, m),
                                      float(b["abs_pressure_hpa"]), 0.0, float(b["pressure_alt"]), float(b["temperature"]),
                                      fields)
        r = d.get("range")
        if r is not None:
            # 범위 밖: max 초과 값 + 신호 품질 0% (MAVLink signal_quality 1 → PX4 0%, 0 은 "모름")
            valid = r["distance"] is not None
            cur = int(round(r["distance"] * 100)) if valid else int(round(r["max"] * 100)) + 1
            cov = max(1, int(round((r["std"] * 100) ** 2)))                    # cm²
            self.conn.mav.distance_sensor_send(int(self.t_us / 1000), int(round(r["min"] * 100)), int(round(r["max"] * 100)),
                                               cur, mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, 0,
                                               mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, cov,
                                               signal_quality=100 if valid else 1)
        fl = d.get("flow")
        if fl is not None:
            nan = float("nan")
            self.conn.mav.hil_optical_flow_send(self.t_us, 0, int(round(fl["span"] * 1e6)), float(fl["integrated"][0]),
                                                float(fl["integrated"][1]), nan, nan, nan, 0, int(fl["quality"]), 0, -1.0)
        mc = d.get("mocap")
        if mc is not None:
            cov = [float("nan")] + [0.0] * 20                   # 분산 모름 → PX4 는 EKF2_EVP/EVA_NOISE 사용 (EKF2_EV_NOISE_MD 1)
            self.conn.mav.vision_position_estimate_send(self.t_us, *map(float, mc["pos_ned"]), *map(float, mc["euler"]), cov, 0)
        g = d["gps"]
        if g is not None:
            vn = g["vel_ned"]
            self.conn.mav.hil_gps_send(self.t_us, int(g["fix_type"]), int(g["lat_deg"] * 1e7), int(g["lon_deg"] * 1e7),
                                       int(g["alt"] * 1000), int(g["eph"]), int(g["epv"]), int(g["speed"] * 100),
                                       int(vn[0] * 100), int(vn[1] * 100), int(vn[2] * 100), 0, int(g["satellites"]))

    @property
    def needs_ground(self):
        return bool(self.sensors_cfg["rangefinder"]["enabled"] or self.sensors_cfg["optical_flow"]["enabled"])

    def step(self, dt, p, v, R, w_body, ground=None):
        """이번 step 상태로 PX4 와 주고받고, 다음 step 에 적용할 ω 를 돌려준다. ground: 하향 센서 거리 [m] 또는 None."""
        if self.dt is None:
            self.dt = dt
            self.sensors = ps.SensorSuite(self.sensors_cfg, dt, self.seed)
        elif abs(dt - self.dt) > 1e-9:
            raise RuntimeError(f"physics dt 가 바뀜 {self.dt} → {dt} (센서 주기는 시작 dt 기준)")
        if self.sitl["autolaunch"] and not self.launcher.alive():
            raise RuntimeError(f"PX4 프로세스가 끝났습니다 (로그 {self.run_dir}/px4_console.log)")
        if not self.heartbeat:
            if self.conn.wait_heartbeat(blocking=False) is not None:
                self.heartbeat = True
        if self.heartbeat:
            self._poll()
            now = time.monotonic()
            if now - self.last_hb_wall > 1.0:
                self.conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GENERIC, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                self.last_hb_wall = now
            self.t_us += int(round(dt * 1e6))
            if self.pending is not None:
                self._send_sensors()
        self.pending = self.sensors.update(p, v, R, w_body, ground)
        return self.omega.copy()
