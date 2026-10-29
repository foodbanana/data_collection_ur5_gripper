# =============================================================
# moving_base_ros2.py — 움직이는 베이스 sim ROS 2 부품 (docs/PLAN.md 8단계 "움직이는 베이스 수집 연결")
#
#   sim_ros2_moving_base.py 가 import 한다. 고정 베이스 부품 (ros2_iface, protective_stop, sim_reset) 은 고치지 않고 상속·설정만 바꿔 쓴다.
#   MovingProtectiveStop : protective_stop.ProtectiveStop 에서 손가락 아닌 링크 접촉 view 만 sim view 로 직접 만든다
#                          (floating 로봇은 재생 중 로봇 링크로 RigidPrim 을 만들면 PhysX 가 로봇을 다시 읽어 view 가 무효, 2026-10-29)
#   BaseBridge           : /base/imu·/base/odom·/base/joint_states (ros2_iface.yaml base). 값은 베이스 궤적 식에서 계산한 정답값 (R3)
#   add_isaac_imu(s)     : 검사용 Isaac IMU 센서 (재생 전). BaseBridge 가 /base/imu_isaac 로 같이 발행 → 궤적 계산값과 비교
#   (R4 에서) MovingEpisodeReset : /sim/reset·/sim/drone_kill
# SimulationApp 을 만든 뒤에 import 할 것.
# =============================================================

import numpy as np

import protective_stop as ps
import ros2_iface as ri

BASE_KEYS = ("imu_topic", "imu_hz", "imu_frame_id", "odom_topic", "odom_hz", "odom_frame_id",
             "joint_states_topic", "joint_states_hz", "isaac_imu_topic")
ISAAC_IMU_NAME = "isaac_imu"


class MovingProtectiveStop(ps.ProtectiveStop):
    def _body_contact_view(self, paths):
        from isaacsim.core.simulation_manager import SimulationManager

        view = SimulationManager._physics_sim_view__warp.create_rigid_contact_view(list(paths), max_contact_data_count=256)
        return view if view is not None and view.check() else None


def quat_wxyz(R):
    """회전 행렬 → 쿼터니언 (w, x, y, z), w ≥ 0."""
    t = np.trace(R)
    if t > 0:
        s = 2.0 * np.sqrt(t + 1.0)
        q = np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
        q = np.zeros(4)
        q[0] = (R[k, j] - R[j, k]) / s
        q[1 + i] = 0.25 * s
        q[1 + j] = (R[j, i] + R[i, j]) / s
        q[1 + k] = (R[k, i] + R[i, k]) / s
    q /= np.linalg.norm(q)
    return q if q[0] >= 0 else -q


def imu_offset(stage, base):
    """베이스 강체 좌표계에서 imu_link 위치 (3,). imu_link 는 강체와 같은 방향이어야 함 (다르면 에러)."""
    from pxr import Usd, UsdGeom

    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    M_body = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(base.body_path))
    M_imu = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(base.imu_path))
    rel = M_imu * M_body.GetInverse()                    # USD 는 행벡터: world = local · M
    R = np.array([[rel[i][j] for j in range(3)] for i in range(3)])
    if not np.allclose(R, np.eye(3), atol=1e-6):
        raise RuntimeError(f"{base.imu_path} 가 베이스 강체와 방향이 다름 (지금 코드는 같은 방향만): {R.round(4).tolist()}")
    return np.array(rel.ExtractTranslation())


def gravity():
    """씬 PhysicsScene 의 중력 벡터 world [m/s²]."""
    from isaacsim.core.simulation_manager import SimulationManager

    scenes = SimulationManager.get_physics_scenes()
    if len(scenes) != 1:
        raise RuntimeError(f"PhysicsScene 이 하나가 아님 ({len(scenes)})")
    return np.array(scenes[0].get_gravity(), dtype=float)


def add_isaac_imu(s):
    """검사용 Isaac IMU 센서를 imu_link 아래에 (재생 전에 부를 것). 필터 없음 (창 1)."""
    from isaacsim.sensors.experimental.physics import IMU, IMUSensor

    return IMUSensor(IMU.create(f"{s.base.imu_path}/{ISAAC_IMU_NAME}", translations=[[0.0, 0.0, 0.0]]))


class BaseBridge:
    """움직이는 베이스 ROS 2 발행 (ros2_iface.yaml base 머리말). 물리 스텝 뒤 (PHYSICS_POST_STEP) sim time 이 1/hz 배수일 때 발행.
    - 값은 베이스 궤적 식 (moving_base kinematics·joint_states) 의 이번 스텝 끝 시각 값 = PhysX 에 넣은 kinematic 자세와 같은 시각.
      베이스가 서 있으면 (start 전·stop 뒤) 그 자리 자세, 속도·가속도 0
    - IMU: a_imu = a + α × (R r) + ω × (ω × (R r)),  비력 f = Rᵀ (a_imu − g),  각속도 Rᵀ ω   (r = 강체 좌표계 imu_link 위치)
    - isaac_imu 를 주면 (add_isaac_imu, --imu-compare) Isaac IMU 센서 값을 같은 시각에 isaac_imu_topic 으로 (stamp = 센서 읽은 sim time)
    콜백 안 예외는 저장해 두고 check() 에서 다시 올린다."""

    def __init__(self, s, cfg, node, isaac_imu=None):
        from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, JointState

        import test_scene as ts

        c = cfg.get("base")
        if c is None:
            raise KeyError(f"{cfg['path']}: 'base' 항목이 없습니다")
        for k in BASE_KEYS:
            if k not in c:
                raise KeyError(f"{cfg['path']}: 'base.{k}' 항목이 없습니다")
        phys_hz = 1.0 / ts.PHYSICS_DT
        self.hz = {k: float(c[f"{k}_hz"]) for k in ("imu", "odom", "joint_states")}
        for k, hz in self.hz.items():
            n = phys_hz / hz
            if not hz > 0 or abs(n - round(n)) > 1e-9:
                raise ValueError(f"base.{k}_hz {hz} 는 물리 {phys_hz:g} Hz 의 약수여야 함")
        self.s, self.c, self._sm = s, c, SimulationManager
        self._Imu, self._Odometry, self._JointState = Imu, Odometry, JointState
        self.base = s.base
        self.r_imu = imu_offset(s.stage, s.base)
        self.g = gravity()
        p0, _, _, R0, _, _ = self.base.kinematics(0.0)
        self.p0, self.R0 = p0, R0                     # odom 원점 = 시작 자세
        self.child_frame = s.base_cfg["body"].split("/")[-1]
        self.pub = {"imu": node.create_publisher(Imu, c["imu_topic"], ri.pub_depth(cfg, self.hz["imu"])),
                    "odom": node.create_publisher(Odometry, c["odom_topic"], ri.pub_depth(cfg, self.hz["odom"])),
                    "joint_states": node.create_publisher(JointState, c["joint_states_topic"], ri.pub_depth(cfg, self.hz["joint_states"]))}
        self.isaac_imu = isaac_imu
        if isaac_imu is not None:
            self.pub["isaac_imu"] = node.create_publisher(Imu, c["isaac_imu_topic"], ri.pub_depth(cfg, self.hz["imu"]))
        self.n = {k: 0 for k in self.pub}
        self._last_k = {k: None for k in self.hz}
        self.error = None
        self.n_cb = 0                                 # 구독 없음 (ri.spin 규약)
        self._cb = SimulationManager.register_callback(self._post_step, event=SimulationEvent.PHYSICS_POST_STEP)

    # ── 계산 ──
    def state(self):
        """(이번 스텝 끝 베이스 시각, 움직이는 중인지, kinematics 6 개)."""
        b = self.base
        moving = b._cb is not None
        k = b.kinematics(b.t)
        if not moving:
            z = np.zeros(3)
            k = (k[0], z, z, k[3], z, z)
        return b.t, moving, k

    def imu_values(self, k):
        """(각속도 (3,) IMU 좌표계, 비력 (3,) IMU 좌표계, 자세 R world)."""
        p, v, a, R, w, alpha = k
        rw = R @ self.r_imu
        a_imu = a + np.cross(alpha, rw) + np.cross(w, np.cross(w, rw))
        return R.T @ w, R.T @ (a_imu - self.g), R

    def _due(self, key, t):
        hz = self.hz[key]
        k = int(round(t * hz))
        if abs(t - k / hz) > 1e-6 or k == self._last_k[key]:
            return False
        self._last_k[key] = k
        return True

    # ── 발행 ──
    def _imu_msg(self, sec, nsec, gyro, acc, R):
        m = self._Imu()
        m.header.stamp.sec, m.header.stamp.nanosec = sec, nsec
        m.header.frame_id = self.c["imu_frame_id"]
        q = quat_wxyz(R)
        m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = map(float, q)
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = map(float, gyro)
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = map(float, acc)
        return m                                      # 공분산 0 = 모름 (잡음 없음)

    def _post_step(self, dt, context):
        try:
            t = self._sm.get_simulation_time()
            due = {k: self._due(k, t) for k in self.hz}
            if not any(due.values()):
                return
            sec, nsec = ri.sim_stamp(t)
            tb, _, k = self.state()
            if due["imu"]:
                self.pub["imu"].publish(self._imu_msg(sec, nsec, *self.imu_values(k)))
                self.n["imu"] += 1
                if self.isaac_imu is not None:
                    r = self.isaac_imu.get_sensor_reading(read_gravity=True)
                    if not r.is_valid:
                        raise RuntimeError("Isaac IMU 값이 유효하지 않음")
                    Rq = _R_wxyz(r.orientation_w, r.orientation_x, r.orientation_y, r.orientation_z)   # C++ 값 (orientation_w … z)
                    s2, n2 = ri.sim_stamp(r.time)
                    self.pub["isaac_imu"].publish(self._imu_msg(
                        s2, n2, (r.angular_velocity_x, r.angular_velocity_y, r.angular_velocity_z),
                        (r.linear_acceleration_x, r.linear_acceleration_y, r.linear_acceleration_z), Rq))
                    self.n["isaac_imu"] += 1
            if due["odom"]:
                p, v, _, R, w, _ = k
                m = self._Odometry()
                m.header.stamp.sec, m.header.stamp.nanosec = sec, nsec
                m.header.frame_id, m.child_frame_id = self.c["odom_frame_id"], self.child_frame
                po, Ro = self.R0.T @ (p - self.p0), self.R0.T @ R
                q = quat_wxyz(Ro)
                m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = map(float, po)
                m.pose.pose.orientation.w, m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z = map(float, q)
                vc, wc = R.T @ v, R.T @ w                 # twist = child 좌표계
                m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z = map(float, vc)
                m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z = map(float, wc)
                self.pub["odom"].publish(m)
                self.n["odom"] += 1
            if due["joint_states"]:
                names, pos, vel = self.base.joint_states(tb)
                if self.base._cb is None:
                    vel = [0.0] * len(names)
                m = self._JointState()
                m.header.stamp.sec, m.header.stamp.nanosec = sec, nsec
                m.name, m.position, m.velocity = list(names), [float(x) for x in pos], [float(x) for x in vel]
                self.pub["joint_states"].publish(m)
                self.n["joint_states"] += 1
        except Exception as e:  # noqa: BLE001
            self.error = self.error or e

    def summary(self):
        return "베이스 발행 " + ", ".join(f"{k} {n}" for k, n in self.n.items())

    def check(self):
        if self.error is not None:
            e, self.error = self.error, None
            raise RuntimeError(f"ROS 2 베이스 인터페이스 에러: {e}") from e

    def close(self):
        if self._cb is not None:
            self._sm.deregister_callback(self._cb)
            self._cb = None


def _R_wxyz(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
