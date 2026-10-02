# =============================================================
# px4_sensors.py — PX4 SITL 로 보내는 가상 센서 (docs/PLAN.md 2단계 2번 P-2, Pegasus Simulator 방식)
#
#   출처: livealive7/PegasusSimulator b1256ca logic/sensors/{imu,barometer,magnetometer,gps}.py
#         (BSD-3-Clause, Copyright (c) 2023 Marcelo Jacinto, 라이선스 isaacsim/assets/drones/iris/LICENSE.PegasusSimulator)
#         식은 PX4 sitl_gazebo 플러그인과 같음 (Pegasus 원본 주석)
#   입력 상태 (Isaac Sim 관례): p, v = body 위치·속도 (world ENU), R = body(FLU) → world 회전, w = 각속도 (body FLU)
#   출력 (PX4 관례): body FRD, world NED. IMU 가속도 = 속도 차분 − g → 그리퍼가 미는 힘도 그대로 잡힘
#
#   pegasus_compat: true 면 Pegasus 코드의 동작을 그대로 (버그 포함, P-2 기준선). false 면 PX4 sitl_gazebo 원래 식대로 고침 (P-4):
#     - 자이로 bias 상관 시간: Pegasus 는 가속도계 값(300 s)을 씀 → 자이로 값(1000 s)
#     - 가속도계 bias: Pegasus 는 rand(0~1 균등)로 갱신하고 측정에 더하지 않음 → randn 으로 갱신하고 더함
#     - 기압 잡음: Pegasus 는 Box-Muller 에 randn 을 넣음 → 표준 정규분포 1 Pa (randn), 표류 = 누적값
#     - GPS bias: Pegasus 는 매 갱신 bias/τ 를 뺌 (dt 빠짐) → bias·dt/τ. GPS 속도 잡음: Pegasus 는 꺼짐 → 켬
#   실내 위치 (PLAN 2단계 2번 결정, Pegasus 에 없음): 하향 거리 센서 + optical flow
#     RangeFinder: 드론 아래 방향 PhysX raycast 거리 (DroneFlight 가 계산, 드론 자기 충돌 형상 제외 → 팔·그리퍼·테이블에 맞음)
#       + 가우시안 잡음, 범위 밖이면 무효. MAVLink DISTANCE_SENSOR 는 cm 정수 (실물 드라이버와 같음)
#     OpticalFlow: PX4 EKF2 의 flow 모델 (ekf2 optical_flow_fusion.cpp predictFlow, EKF2.cpp 가 pixel_flow·delta_angle 부호를 뒤집어 받음)
#       에 맞춰 몸체 FRD 에서 pixel_flow 각속도 = ω_xy + (−v_y, v_x) / 거리 (v = 몸체 속도, 거리 = 광축 방향 표면까지) 를 적분해 보냄 + 잡음.
#       (회전 +ω_x 는 화면을 +Y 로, 오른쪽 이동 +v_y 는 −Y 로 움직임 → 반대 부호. 2026-10-02 처음에 (v_y, −v_x) 로 넣어 EKF 가 발산)
#       맞은 표면은 정지했다고 가정 (위로 올라오는 그리퍼는 주로 거리 변화로 나타남). 거리 범위 밖이면 quality 0.
#       flow 모듈 자체 자이로는 없음 (integrated gyro NaN → PX4 가 기체 IMU 로 보정, PMW3901 류 실물과 같음)
#     MotionCapture (기본, 2026-10-02 결정 A): 실내 모션캡처 = world 좌표의 body 위치·자세 + 잡음, 주기·지연 (아래를 보지 않음 → 팔 영향 없음)
#       → MAVLink VISION_POSITION_ESTIMATE (NED 위치, FRD 몸체 roll·pitch·yaw). world = 모션캡처 좌표 = PX4 local 좌표 (원점 같음)
#   Pegasus 와 다른 점 (compat 이어도): 난수는 seed 를 받는 np.random.Generator (재현용, Pegasus 는 전역 np.random),
#     센서 주기는 physics step 정수배로 셈 (Pegasus 는 시간 누적 비교)
# =============================================================

import math

import numpy as np

from pegasus_geo_mag import GRAVITY_VECTOR, get_mag_declination, get_mag_inclination, get_mag_strength, reprojection


def enu_to_ned(x):
    return np.array([x[1], x[0], -x[2]])


def flu_to_frd(x):
    return np.array([x[0], -x[1], -x[2]])


def _bias_sigma(random_walk, tau, dt):
    """이산 시간 bias 과정 표준편차 [Maybeck 4-114] (Pegasus·sitl_gazebo 와 같은 식)."""
    return math.sqrt(-random_walk * random_walk * tau / 2.0 * (math.exp(-2.0 * dt / tau) - 1.0))


class RateGate:
    """physics step 을 세어 update_rate 마다 True. 첫 호출은 항상 True (Pegasus _first_update 와 같음)."""

    def __init__(self, rate, dt):
        self.n = max(1, int(round(1.0 / (rate * dt))))
        self.period = self.n * dt
        self.k = 0

    def __call__(self):
        due = self.k % self.n == 0
        self.k += 1
        return due


class IMU:
    def __init__(self, c, dt, rng, compat):
        g, a = c["gyroscope"], c["accelerometer"]
        self.g_nd, self.g_rw, self.g_tau = g["noise_density"], g["random_walk"], g["bias_correlation_time"]
        self.a_nd, self.a_rw, self.a_tau = a["noise_density"], a["random_walk"], a["bias_correlation_time"]
        self.gate = RateGate(c["update_rate"], dt)
        self.rng, self.compat = rng, compat
        self.g_bias, self.a_bias = np.zeros(3), np.zeros(3)
        self.v_prev = None

    def update(self, p, v, R, w):
        if not self.gate():
            return None
        dt = self.gate.period
        rng = self.rng
        tau_g = self.a_tau if self.compat else self.g_tau
        s_g = self.g_nd / math.sqrt(dt)
        sb_g = _bias_sigma(self.g_rw, tau_g, dt)
        self.g_bias = math.exp(-dt / tau_g) * self.g_bias + sb_g * rng.standard_normal(3)
        gyro = w + s_g * rng.standard_normal(3) + self.g_bias

        s_a = self.a_nd / math.sqrt(dt)
        sb_a = _bias_sigma(self.a_rw, self.a_tau, dt)
        v_prev = np.zeros(3) if self.v_prev is None else self.v_prev   # Pegasus: 이전 속도 초기값 0
        acc_world = (v - v_prev) / dt - GRAVITY_VECTOR
        self.v_prev = v.copy()
        acc = R.T @ acc_world
        if self.compat:
            self.a_bias = math.exp(-dt / self.a_tau) * self.a_bias + sb_a * rng.random(3)
            acc = acc + s_a * rng.standard_normal(3)
        else:
            self.a_bias = math.exp(-dt / self.a_tau) * self.a_bias + sb_a * rng.standard_normal(3)
            acc = acc + s_a * rng.standard_normal(3) + self.a_bias
        return {"gyro": flu_to_frd(gyro), "acc": flu_to_frd(acc)}


class Barometer:
    T_MSL, P_MSL, LAPSE, RHO_MSL, ABS_ZERO = 288.15, 101325.0, 0.0065, 1.225, -273.15

    def __init__(self, c, dt, rng, compat, origin_alt):
        self.gate = RateGate(c["update_rate"], dt)
        self.drift_rate = c["drift_pa_per_sec"]
        self.rng, self.compat, self.origin_alt = rng, compat, origin_alt
        self.z0, self.drift, self._y2 = None, 0.0, None

    def _noise(self):
        if not self.compat:
            return float(self.rng.standard_normal())
        # Pegasus 그대로: Box-Muller 극좌표 방식에 균등 난수 대신 randn 을 넣음
        if self._y2 is not None:
            y, self._y2 = self._y2, None
            return y
        w = 1.0
        while w >= 1.0:
            x1 = 2.0 * self.rng.standard_normal() - 1.0
            x2 = 2.0 * self.rng.standard_normal() - 1.0
            w = x1 * x1 + x2 * x2
        w = math.sqrt(-2.0 * math.log(w) / w)
        self._y2 = x2 * w
        return x1 * w

    def update(self, p):
        if not self.gate():
            return None
        dt = self.gate.period
        if self.z0 is None:
            self.z0 = p[2]
        alt = self.origin_alt + (p[2] - self.z0)
        T = self.T_MSL - self.LAPSE * alt
        pressure = self.P_MSL / (self.T_MSL / T) ** 5.2561
        n = self._noise()
        self.drift += self.drift_rate * dt
        # Pegasus 그대로: 측정 기압에 누적 표류 대신 표류율(Pa/s)을 더함 (기본 0 이라 영향 없음)
        p_noisy = pressure + n + (self.drift_rate if self.compat else self.drift)
        rho = self.RHO_MSL / (self.T_MSL / T) ** 4.256
        p_alt = alt - (n + self.drift) / (np.linalg.norm(GRAVITY_VECTOR) * rho)
        return {"abs_pressure_hpa": p_noisy * 0.01, "pressure_alt": p_alt, "temperature": T + self.ABS_ZERO}


class Magnetometer:
    def __init__(self, c, dt, rng, origin):
        self.nd, self.rw, self.tau = c["noise_density"], c["random_walk"], c["bias_correlation_time"]
        self.gate = RateGate(c["update_rate"], dt)
        self.rng, self.origin = rng, origin
        self.bias = np.zeros(3)

    def update(self, p, R):
        if not self.gate():
            return None
        dt = self.gate.period
        lat, lon = reprojection(p, math.radians(self.origin[0]), math.radians(self.origin[1]))
        lat, lon = math.degrees(lat), math.degrees(lon)
        dec, inc = math.radians(get_mag_declination(lat, lon)), math.radians(get_mag_inclination(lat, lon))
        s = 0.01 * get_mag_strength(lat, lon)                        # gauss
        H = s * math.cos(inc)
        field_ned = np.array([H * math.cos(dec), H * math.sin(dec), math.tan(inc) * H])
        field_enu = enu_to_ned(field_ned)                            # NED ↔ ENU 는 같은 변환
        body = flu_to_frd(R.T @ field_enu)
        sd = self.nd / math.sqrt(dt)
        sb = _bias_sigma(self.rw, self.tau, dt)
        self.bias = math.exp(-dt / self.tau) * self.bias + sb * self.rng.standard_normal(3)
        return {"mag": body + sd * self.rng.standard_normal(3) + self.bias}


class GPS:
    def __init__(self, c, dt, rng, compat, origin):
        self.c, self.gate = c, RateGate(c["update_rate"], dt)
        self.rng, self.compat, self.origin = rng, compat, origin
        self.bias = np.zeros(3)

    def update(self, p, v):
        if not self.gate():
            return None
        dt, c, rng = self.gate.period, self.c, self.rng
        sq = math.sqrt(dt)
        rw = np.array([c["xy_random_walk"], c["xy_random_walk"], c["z_random_walk"]]) * sq * rng.standard_normal(3)
        n_pos = np.array([c["xy_noise_density"], c["xy_noise_density"], c["z_noise_density"]]) * sq * rng.standard_normal(3)
        n_vel = np.array([c["vxy_noise_density"], c["vxy_noise_density"], c["vz_noise_density"]]) * sq * rng.standard_normal(3)
        tau = c["correlation_time"]
        self.bias = self.bias + rw * dt - self.bias * ((1.0 / tau) if self.compat else (dt / tau))
        lat, lon = reprojection(p + n_pos + self.bias, math.radians(self.origin[0]), math.radians(self.origin[1]))
        vel = v if self.compat else v + n_vel
        return {"lat_deg": math.degrees(lat), "lon_deg": math.degrees(lon),
                "alt": p[2] + self.origin[2] - n_pos[2] + self.bias[2],
                "vel_ned": enu_to_ned(vel), "speed": float(np.linalg.norm(vel[:2])),
                "fix_type": c["fix_type"], "eph": c["eph"], "epv": c["epv"], "satellites": c["satellites_visible"]}


class RangeFinder:
    def __init__(self, c, dt, rng):
        self.min, self.max, self.std = float(c["min_distance"]), float(c["max_distance"]), float(c["noise_std"])
        self.gate = RateGate(c["update_rate"], dt)
        self.rng = rng

    def update(self, d):
        if not self.gate():
            return None
        if d is None or not (self.min <= d <= self.max):
            return {"distance": None, "min": self.min, "max": self.max, "std": self.std}
        return {"distance": max(self.min, d + self.std * float(self.rng.standard_normal())), "min": self.min, "max": self.max,
                "std": self.std}


class OpticalFlow:
    def __init__(self, c, dt, rng):
        self.min, self.max, self.std = float(c["min_distance"]), float(c["max_distance"]), float(c["noise_std"])
        self.gate = RateGate(c["update_rate"], dt)
        self.dt, self.rng = dt, rng
        self.acc, self.span, self.valid = np.zeros(2), 0.0, True

    def update(self, v_body_flu, w_body_flu, d):
        """매 step 적분, 주기마다 내보냄. 그 사이 한 번이라도 거리 범위 밖이면 quality 0."""
        v, w = flu_to_frd(v_body_flu), flu_to_frd(w_body_flu)
        if d is None or not (self.min <= d <= self.max):
            self.valid = False
        else:
            rate = w[:2] + np.array([-v[1], v[0]]) / d
            self.acc += (rate + self.std * self.rng.standard_normal(2)) * self.dt
        self.span += self.dt
        if not self.gate():
            return None
        out = {"integrated": self.acc.copy(), "span": self.span, "quality": 255 if self.valid else 0}
        self.acc, self.span, self.valid = np.zeros(2), 0.0, True
        return out


class MotionCapture:
    """주기마다 latency 전 실제 자세를 잡음과 함께 내보냄 (latency 는 physics step 단위로 반올림)."""

    def __init__(self, c, dt, rng):
        self.gate = RateGate(c["update_rate"], dt)
        self.lag = int(round(float(c["latency"]) / dt))
        self.p_std, self.a_std = float(c["position_noise_std"]), float(c["angle_noise_std"])
        self.rng, self.hist = rng, []

    def update(self, p, R):
        self.hist.append((p.copy(), R.copy()))
        if len(self.hist) > self.lag + 1:
            self.hist.pop(0)
        if not self.gate():
            return None
        p0, R0 = self.hist[0]                                   # lag step 전 (처음에는 있는 만큼)
        T = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])      # ENU → NED
        Rn = T @ R0 @ np.diag([1.0, -1.0, -1.0])                # FRD 몸체 → NED
        euler = np.array([math.atan2(Rn[2, 1], Rn[2, 2]), -math.asin(max(-1.0, min(1.0, Rn[2, 0]))), math.atan2(Rn[1, 0], Rn[0, 0])])
        return {"pos_ned": T @ p0 + self.p_std * self.rng.standard_normal(3),
                "euler": euler + self.a_std * self.rng.standard_normal(3)}


class SensorSuite:
    """설정 px4.sensors → 센서 묶음. update() 는 매 physics step 호출, 이번 step 에 새로 나온 값만 dict 로."""

    def __init__(self, c, dt, seed):
        self.compat = bool(c["pegasus_compat"])
        rng = np.random.default_rng(seed)
        origin = tuple(float(x) for x in c["origin_lat_lon_alt"])
        self.imu = IMU(c["imu"], dt, rng, self.compat)
        self.baro = Barometer(c["barometer"], dt, rng, self.compat, origin[2])
        self.mag = Magnetometer(c["magnetometer"], dt, rng, origin)
        self.gps = GPS(c["gps"], dt, rng, self.compat, origin) if c["gps"]["enabled"] else None
        self.range = RangeFinder(c["rangefinder"], dt, rng) if c["rangefinder"]["enabled"] else None
        self.flow = OpticalFlow(c["optical_flow"], dt, rng) if c["optical_flow"]["enabled"] else None
        self.mocap = MotionCapture(c["mocap"], dt, rng) if c["mocap"]["enabled"] else None

    @property
    def needs_ground(self):
        return self.range is not None or self.flow is not None

    def update(self, p, v, R, w, ground=None):
        """ground: 하향 센서에서 맞은 표면까지 거리 [m] (없으면 None)."""
        out = {"imu": self.imu.update(p, v, R, w), "baro": self.baro.update(p), "mag": self.mag.update(p, R)}
        out["gps"] = self.gps.update(p, v) if self.gps is not None else None
        out["range"] = self.range.update(ground) if self.range is not None else None
        out["flow"] = self.flow.update(R.T @ v, w, ground) if self.flow is not None else None
        out["mocap"] = self.mocap.update(p, R) if self.mocap is not None else None
        return out
