# sim ROS 2 인터페이스 (토픽·주기·구조)

작성: 2026-10-05 · 3단계 완료 시점 (`docs/PLAN.md` 3단계), 브랜치 `isaacsim_v6.1.0`

Isaac Sim 의 드론 파지 씬이 실물과 같은 ROS 2 토픽으로 동작하도록 만든 인터페이스를 정리한다.
녹화(`record_toggle.py`)·변환(stage1 → LeRobot v2.1)은 sim 과 실물이 같은 코드를 쓴다 (CLAUDE.md 원칙 1).

---

## 1. 한눈에 보기

| 항목 | 내용 |
|------|------|
| 실행 | `~/isaacsim/python.sh isaacsim/scripts/sim_ros2.py` (GUI) / `--headless --duration <s>` |
| 프로세스 | sim_ros2 (Isaac Sim, ROS 2 노드 `isaac_sim`) + PX4 SITL (sim 이 자식 프로세스로 띄움) |
| 주기 | 앱 루프(렌더) 30 Hz = 카메라 주기, 물리 120 Hz (루프 한 번에 물리 4 스텝) |
| 시각 | 모든 `header.stamp` = **sim time** (`/clock` 과 같은 시계). 카메라·그리퍼 stamp 는 `/joint_states` stamp 와 정확히 같은 물리 스텝 |
| 속도 | 실제 시간 속도로 돈다 (텔레옵). RTF 대기 0.96, 드론을 잡은 채 0.69 (`docs/sim_performance.md`) |
| 설정 | `isaacsim/config/ros2_iface.yaml` (토픽 이름·주기·렌더·보호 정지·리셋) |
| 검사 | `python3 isaacsim/scripts/check_ros2.py` → 9/9 PASS (약 8 분) |

---

## 2. 구조

```text
 EXTERNAL ROS 2 NODES                         sim_ros2.py  (Isaac Sim 6.1.0, ROS 2 node "isaac_sim")
                                              app loop 30 Hz (render) | physics 120 Hz | real-time paced
                                              +-----------------------------------------------------------------+
 +----------------------+  /joint_command     |  ArmBridge                                                      |
 | Teleop device (st.5) |  JointState ~60 Hz  |    last cmd per loop -> linear interp 33 ms                     |
 |  or                  |-------------------->|    (every physics step) -> UR5 arm drive targets                |
 | replay_commands.py   |  /gripper/command   |  GripperBridge                                                  |
 |                      |  Float64, on change |    raw 0..1150 -> GripperProfile -> rh_r1_joint                 |
 +----------------------+-------------------->|  ProtectiveStop (contact / tracking error / speed,              |
                                              |    every physics step; stop = hold arm)                         |
 +----------------------+  /sim/reset         |  EpisodeReset                                                   |
 | record_toggle (st.4) |  std_srvs/Trigger   |    kill drone -> teleport + PX4 restart                         |
 |  or user CLI         |-------------------->|    -> teleport arm home -> re-takeoff (~24 s)                   |
 +----------------------+<-- JSON response ---|                                                                 |
                                              |  OmniGraph Camera Helper x2 (RTX, DLSS Performance)             |
                                              |                                                                 |
                                              |  PhysX: table + UR5 + RH-P12-RN(A) + D435i x2 + drone           |
                                              +-+---------------------------------------------------------------+
                                                |
                                                | /clock                          Clock      120 Hz
                                                | /joint_states                   JointState 120 Hz (arm 6)
                                                | /gripper/joint_states           JointState  30 Hz (now)
                                                | /gripper/target                 JointState  30 Hz (goal)
                                                | /protective_stop                Bool        30 Hz
                                                | /cam/wrist/color/image_raw      Image       30 Hz (+info)
                                                | /cam/third_view/color/image_raw Image       30 Hz (+info)
                                                v
 +----------------------+                +-------------------+
 | rqt_image_view (view)|<-- cameras ----| record_toggle.py  |  ros2 bag
 +----------------------+                | (step 4)          |  -> stage1 25 Hz
                                         +-------------------+  -> LeRobot v2.1

 DRONE (inside sim_ros2 + PX4 SITL child process)
 +--------------------------------+  TCP 4560 (lockstep)                        +---------------+
 | DroneFlight + PX4Bridge        |  HIL_SENSOR 120 Hz ------------------------>| PX4 SITL      |
 |  rotor thrust k*w^2 -> PhysX   |  VISION_POSITION ~100 Hz ------------------>| v1.16.0 -i 0  |
 |  forces (every physics step)   |<- HIL_ACTUATOR 120 Hz ----------------------|               |
 +--------------------------------+                                             |               |
 | PX4Commander (offboard)        |  UDP 14540                                  |               |
 |                                |  SET_POSITION_TARGET 20 Hz ---------------->|               |
 |                                |<- LOCAL_POSITION 30 Hz, --------------------|               |
 |                                |   ATTITUDE 60 Hz, HEARTBEAT 1Hz             |               |
 +--------------------------------+                                             +---------------+
          ^
          |  UDP 14600 JSON: goto / hold / land / kill / status
 +----------------------+
 | drone_cmd.py (CLI)   |
 +----------------------+
```

- 화살표 위 글자 = 토픽(또는 링크) 이름, 메시지 타입, 주기. 주기는 **sim 시간 기준** (실제 시간 기준은 RTF 만큼 느림)
- 왼쪽 위 텔레옵 장치는 5단계에서 만든다. 지금은 `replay_commands.py` 가 명령 CSV 를 `/joint_command`·`/gripper/command` 로 재생한다
- `record_toggle.py` 의 `/sim/reset` 호출 연동과 새 토픽 녹화는 4단계 작업
- PX4 쪽 링크는 ROS 2 가 아니라 MAVLink (sim 내부). 드론의 위치 정보는 실내 모션캡처를 가정 (`docs/drone_flight.md` 13장)

---

## 3. ROS 2 토픽

| 토픽 | 타입 | 방향 | 주기 (sim) | 내용 | 데이터셋 |
|------|------|------|-----------|------|----------|
| `/clock` | rosgraph_msgs/Clock | sim → | 120 Hz (물리 스텝마다) | sim time | — |
| `/joint_states` | sensor_msgs/JointState | sim → | 120 Hz (물리 스텝마다) | 팔 6 관절 (`shoulder_pan_joint` … `wrist_3_joint`), position [rad]·velocity [rad/s] (관절각 차분)·effort [Nm] | `observation.state[0:6]` |
| `/joint_command` | sensor_msgs/JointState | → sim | 텔레옵 주기 (일정하게, 예 60 Hz) | 팔 6 관절 목표 position [rad]. 이름·개수·유한값·관절 한계가 틀리면 sim 이 에러로 중단. **`header.stamp` = 보낸 순간의 sim time (보내는 쪽이 넣음, stage1 이 이 stamp 로 맞춤)** | `action[0:6]` |
| `/gripper/command` | std_msgs/Float64 | → sim | 이벤트 (바뀔 때) | raw 0 (열림) ~ 1150 (닫힘). 범위 밖이면 에러 | — |
| `/gripper/joint_states` | sensor_msgs/JointState | sim → | 30 Hz | `name ['rh_p12_rn']`, position = present raw (0~1150) | `observation.state[6]` (/1150) |
| `/gripper/target` | sensor_msgs/JointState | sim → | 30 Hz (present 와 같은 stamp) | position = 실행된 goal raw (0 / 1150) | `action[6]` (/1150) |
| `/protective_stop` | std_msgs/Bool | sim → | 30 Hz | 보호 정지 중이면 true (해제는 `/sim/reset`) | 에피소드 메타데이터 (4단계) |
| `/cam/wrist/color/image_raw` | sensor_msgs/Image | sim → | 30 Hz | 손목 D435i color 640x480 `rgb8`, frame_id `wrist_camera_color_optical_frame` | `observation.images.wrist` |
| `/cam/wrist/color/camera_info` | sensor_msgs/CameraInfo | sim → | 30 Hz | intrinsics (1-7, 실물 camera_info 값) | — |
| `/cam/third_view/color/image_raw` | sensor_msgs/Image | sim → | 30 Hz | third view color 640x480 `rgb8`, frame_id `third_view_camera_color_optical_frame` | `observation.images.third_view` |
| `/cam/third_view/color/camera_info` | sensor_msgs/CameraInfo | sim → | 30 Hz | intrinsics | — |
| `/base/imu` | sensor_msgs/Imu | — | **8단계부터** | 고정 베이스에서는 sim·실물 모두 발행 안 함. stage1 `--base-imu const` | `observation.base_imu` |

- **QoS**: 기본 (RELIABLE, depth 10). 카메라는 **받는 쪽도 RELIABLE** 로 받을 것 (best effort 면 640x480 이미지가 UDP 조각 손실로 통째로 버려짐, `docs/sim_performance.md` 5장)
- **stamp**: 모두 sim time. 카메라 이미지는 stamp 시각의 물리 상태 (팔이 움직이기 시작한 stamp = 이미지가 바뀐 첫 stamp, 0 프레임 지연, check_ros2 5 번)
- `ros2 topic hz` 는 기본이 wall 시간 기준. sim 기준은 `ros2 topic hz --use-sim-time <토픽>`

### 서비스

| 서비스 | 타입 | 동작 | 걸리는 시간 |
|--------|------|------|-------------|
| `/sim/reset` | std_srvs/Trigger | 에피소드 리셋 = 에피소드 시작 상태로: 드론 kill → 이륙 지점으로 순간이동 + PX4 재시작 → 팔·그리퍼 홈·열림 순간이동 → 재이륙 → 새 호버 위치 (기준 위치 ± 5 cm, 시드 기록). 응답 message = JSON. 실패하면 `success=False` (sim 은 계속, 다음 성공까지 명령 무시) | 약 24 s (sim) |
| `/sim/drone_kill` | std_srvs/Trigger | 드론 모터 정지 (잡은 뒤. PX4 = kill, 기하 제어기 = 로터 ω 0). 응답 message = JSON (sim 시각). 이미 꺼져 있으면 `success=False`. 다시 켜는 것은 `/sim/reset`. `record_toggle.py --sim` 의 `k` 키가 부르고 시각을 `episode.json` 에 기록 | 즉시 |

### sim 안 링크 (ROS 2 아님)

| 링크 | 프로토콜 | 내용 |
|------|----------|------|
| sim ↔ PX4 | MAVLink, TCP 4560 (lockstep) | sim → PX4: HIL_SENSOR (IMU·기압·지자기, 물리 스텝마다 120 Hz), VISION_POSITION_ESTIMATE (모션캡처, 설정 100 Hz, 지연 20 ms). PX4 → sim: HIL_ACTUATOR_CONTROLS (모터 명령, 120 Hz, 이 명령이 와야 sim 이 다음 스텝으로 = lockstep) |
| PX4Commander ↔ PX4 | MAVLink, UDP 14540 (offboard) | → PX4: SET_POSITION_TARGET 20 Hz, 모드·arm·kill 명령. ← PX4: LOCAL_POSITION_NED 30 Hz, ATTITUDE 60 Hz, HEARTBEAT 1 Hz, STATUSTEXT, COMMAND_ACK (안 쓰는 HIGHRES_IMU·ATTITUDE_QUATERNION·ODOMETRY 는 끔) |
| drone_cmd.py → sim | JSON, UDP 14600 (localhost) | 다른 터미널 CLI: goto / hold / land / kill / status |

---

## 4. sim 과 실물의 차이 (데이터셋에 영향 없는 것 포함)

| 항목 | 실물 | sim | 영향 |
|------|------|-----|------|
| `/joint_states` effort | 관절 전류 [A] (UR 드라이버) | 관절 토크 [Nm] (PhysX projected joint force) | stage1 은 effort 를 안 씀 |
| `/joint_states` 주기 | 125 Hz | 120 Hz (물리 스텝) | stage1 이 25 Hz 로 리샘플 |
| `/gripper/command` 범위 밖 | 노드가 0~1150 으로 clamp | 에러로 중단 | 텔레옵은 0 / 1150 만 보냄 |
| 카메라 토픽 이름 | `/d435i/d435i/...`, `/d456/d456/...` (지금 실물 스크립트) | `/cam/wrist/...`, `/cam/third_view/...` | 4단계에서 실물·stage1 을 `/cam/...` 로 맞춤 |
| 카메라 렌더 | RealSense 원래 해상도 | DLSS Performance (절반 해상도 렌더 후 업스케일, DLAA 대비 평균 차이 1/255 이하) | `docs/sim_performance.md` 7장 |
| camera_info fy | 618.956 (fx 618.551) | fy = fx (렌더러가 정사각 픽셀만) | stage1 은 camera_info 를 안 씀 |
| 팔 명령 → 움직임 | servoJ (lookahead 로 부드럽게) | 선형 보간 33 ms + drive (50 % 도달 약 50 ms) | `docs/arm_command_interpolation.md` |
| 보호 정지 | UR 안전 시스템 | 흉내: 접촉력 150 N, 위치 오차 5°, 관절 속도 200 °/s | PLAN 3-7 |
| 리셋 | 사람이 정리 | `/sim/reset` 순간이동 | — |

---

## 5. 실행·검사 명령

```bash
cd ~/data_collection_ur5_gripper
source /opt/ros/jazzy/setup.bash            # ~/.bashrc 에 있으면 생략

# sim (GUI: 메인 뷰포트 하나 + 프로펠러 회전). [sim_ros2] t ... 상태 줄이 나오면 준비 완료 (약 30 s)
~/isaacsim/python.sh isaacsim/scripts/sim_ros2.py
#   --headless --duration 60, --flight geometric, --reset-offset 0, --seed N, --protective-stop on|measure, --init-pose q1..q6

# 다른 터미널
ros2 run rqt_image_view rqt_image_view /cam/wrist/color/image_raw              # 카메라 보기
ros2 topic hz --use-sim-time /joint_states                                     # sim 기준 주기
ros2 service call /sim/reset std_srvs/srv/Trigger                              # 에피소드 리셋 (약 24 s)
python3 isaacsim/scripts/replay_commands.py isaacsim/config/ros2_check/commands_success.csv   # 명령 재생 (텔레옵 대신)
python3 isaacsim/scripts/drone_cmd.py kill                                     # 드론 모터 정지 (잡은 뒤)
ros2 topic echo --once /protective_stop

# 자동 검사 (sim 을 직접 띄움, 약 8 분) → 9/9 PASS
python3 isaacsim/scripts/check_ros2.py                                         # --part A | B
```

---

## 6. 관련 문서

| 문서 | 내용 |
|------|------|
| `docs/PLAN.md` 3단계 | 단계별 결과 (3-0 ~ 3-8), 결정과 그 이유 |
| `docs/sim_performance.md` | RTF 측정·최적화, 카메라 QoS, DLSS 비교 |
| `docs/arm_command_interpolation.md` | 팔 명령 보간 (30 Hz 계단 문제, 다른 프로젝트 조사), PhysX 관절 속도, rclpy spin |
| `docs/drone_flight.md` | 드론 비행 (Pegasus 방식, PX4 SITL 연결·센서) |
| `isaacsim/config/ros2_iface.yaml` | 토픽 이름·주기·렌더·보호 정지·리셋 설정 |
