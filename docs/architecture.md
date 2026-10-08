# 데이터 수집 전체 구조

작성: 2026-10-06 · 4단계 완료 시점 (`docs/PLAN.md`), 브랜치 `isaacsim_v6.1.0` · 2026-10-08 5단계 장치 결정 반영 (팔 = SpaceMouse, 드론 = 실물 조종기)

명령 → 시뮬레이터 (또는 실물 로봇) → 녹화 → 변환 → 병합 → 검수까지, 지금 만들어져 있는 데이터 수집 흐름 전체를 한 장으로 보여 준다.
부분별 자세한 내용은 맨 아래 "관련 문서".

---

## 1. 전체 그림 (sim 기준, 위에서 아래로)

```text
+------------------------------------------------------------------------------------------------------------------------+
| [1] COMMAND SOURCE   (system python3 + ROS 2)                                                                          |
|------------------------------------------------------------------------------------------------------------------------|
| now    : isaacsim/scripts/replay_commands.py <commands.csv>   (replays a recorded grasp, paced by /clock)              |
| stage 5-B: arm teleop node (SpaceMouse Wireless -> TCP velocity -> IK)   <-- NOT BUILT YET                             |
| rule   : publishes only these two topics; /joint_command carries header.stamp = sim time at send                       |
+------------------------------------------------------------------------------------------------------------------------+
      |  /joint_command     sensor_msgs/JointState   ~60 Hz (teleop rate)   6 arm joint targets [rad]
      |  /gripper/command   std_msgs/Float64         on change              raw 0 (open) / 1150 (close)
      v
+------------------------------------------------------------------------------------------------------------------------+
| [2] SIMULATOR   ~/isaacsim/python.sh isaacsim/scripts/sim_ros2.py     (one process, ROS 2 node "isaac_sim")            |
|------------------------------------------------------------------------------------------------------------------------|
| app loop 30 Hz (render) | physics 120 Hz (4 steps per loop) | real-time paced | all stamps = sim time                  |
|                                                                                                                        |
|   ArmBridge        : latest /joint_command per loop -> linear interpolation 33 ms -> UR5 drive targets                 |
|   GripperBridge    : raw goal -> GripperProfile (0.516 rad/s) -> rh_r1_joint ; publishes present / goal                |
|   ProtectiveStop   : contact 150 N / tracking error 5 deg / joint speed 200 deg/s -> hold arm, ignore commands         |
|   EpisodeReset     : service /sim/reset       (teleport arm home + restart PX4 + re-takeoff, ~24 s)                    |
|                      service /sim/drone_kill  (drone motors off)                                                       |
|   Camera Helper x2 : OmniGraph, RTX render (DLSS Performance), 640x480                                                 |
|   PhysX scene      : table + UR5 + RH-P12-RN(A) gripper + wrist camera + third-view camera + drone (Iris 0.75x)        |
|                                                                                                                        |
|   DroneFlight + PX4Bridge  <== MAVLink TCP 4560, lockstep (HIL_SENSOR / HIL_ACTUATOR 120 Hz) ==>  PX4 SITL v1.16.0     |
|   PX4Commander (offboard)  <== MAVLink UDP 14540 (position setpoint 20 Hz)                  ==>  (child process)       |
|                                                                                                                        |
|   stage 5-A: drone RC input (Radiolink transmitter -> Pi Pico -> USB serial), option --drone-rc on                     |
|              -> PX4Commander -> MANUAL_CONTROL on UDP 14540 (after reset: Offboard takeoff, then hand-over to sticks)  |
|              not a ROS command topic, NOT recorded.  built, flown with the real transmitter (2026-10-08)               |
|                                                                                                                        |
|   publish queues: 3 s of messages per topic (ros2_iface.yaml publish_queue_sec)                                        |
+------------------------------------------------------------------------------------------------------------------------+
      |  /clock                            rosgraph_msgs/Clock      120 Hz   sim time
      |  /joint_states                     sensor_msgs/JointState   120 Hz   arm 6 joints: position, velocity, effort
      |  /gripper/joint_states             sensor_msgs/JointState    30 Hz   gripper present (raw 0..1150)
      |  /gripper/target                   sensor_msgs/JointState    30 Hz   gripper goal    (raw 0 / 1150)
      |  /protective_stop                  std_msgs/Bool             30 Hz   true while stopped
      |  /cam/wrist/color/image_raw        sensor_msgs/Image         30 Hz   640x480 rgb8   (+ camera_info)
      |  /cam/third_view/color/image_raw   sensor_msgs/Image         30 Hz   640x480 rgb8   (+ camera_info)
      v
+------------------------------------------------------------------------------------------------------------------------+
| [3] RECORDER   ./6_record_bag.sh <task> --sim   (= record_toggle.py, system python3 + ROS 2)                           |
|------------------------------------------------------------------------------------------------------------------------|
| keys:  r = call /sim/reset, then start recording     r again = stop                                                    |
|        k = call /sim/drone_kill (after the grasp)    d = move last episode to bags/_discarded/     q = quit            |
|                                                                                                                        |
|   1. before recording: every state topic has a publisher? camera publishers RELIABLE?  -> if not, refuse to record     |
|   2. spawn "ros2 bag record": 11 topics (the 9 above incl. camera_info, + /joint_command + /gripper/command)           |
|                               cameras received RELIABLE, receive queue = 3 s per topic (QoS override file)             |
|   3. on stop: write episode.json (mode, topics, camera roles, /sim/reset reply = seed, drone pose, git commit,         |
|                                   sim settings; drone kill time)                                                       |
|                                                                                                                        |
|   success / failure is decided by the operator: failed episodes are discarded with d                                   |
+------------------------------------------------------------------------------------------------------------------------+
      |  one r..r interval = one bag = one episode          (30 s episode ~ 1.5 GB, uncompressed images)
      v
      bags/<YYYYMMDD_HHMMSS>_<task>/   *.mcap   metadata.yaml   episode.json          bags/_discarded/  (not converted)
      |
      v
+------------------------------------------------------------------------------------------------------------------------+
| [4] STAGE 1   python3 lerobot_stage1_extract_bag.py <bag> --arm-action command --base-imu const   (python3 + ROS 2)    |
|------------------------------------------------------------------------------------------------------------------------|
|   - 25 Hz grid on header.stamp ; at each grid time take the latest message (zero-order hold)                           |
|   - error if a picked camera / arm / gripper message is older than 66 ms (--max-age)                                   |
|   - episode starts at the first /joint_command (earlier frames are dropped) ; error if a command stamp is 0            |
|   - state    = arm 6 joints [rad] + gripper present / 1150   (0..1)                                                    |
|   - action   = /joint_command 6 joints [rad] + gripper goal / 1150   ({0, 1})                                          |
|   - base_imu = constant [0, 0, 0, 0, 0, 9.81]   (fixed base)                                                           |
+------------------------------------------------------------------------------------------------------------------------+
      |
      v
      bag_lerobot_intermediate/<bag>/   states.npy (N,7)  actions.npy (N,7)  base_imu.npy (N,6)  timestamps.npy
                                        wrist/*.png  third_view/*.png  meta.json (options, cameras, message ages,
                                        /joint_command interval stats, protective stop, drone kill, episode.json)
      |
      v
+------------------------------------------------------------------------------------------------------------------------+
| [5] MERGE   python lerobot_merge_episodes_v21.py --manifest <txt> | --all --repo-id <id>   (conda lerobot_v2)          |
|------------------------------------------------------------------------------------------------------------------------|
|   - pre-check every episode (schema, value ranges) ; stop if arm-action kind / fps / image size differ                 |
|   - protective-stop episodes excluded by default ; idle trimming only with --trim-idle                                 |
|   - lerobot 0.3.3 : create() once, add_frame + save_episode per episode (AV1 mp4)                                      |
+------------------------------------------------------------------------------------------------------------------------+
      |
      v
      lerobot_dataset_v21/<repo_id>/    LeRobot v2.1, 25 fps
          observation.state (7)   action (7)   observation.base_imu (6)
          observation.images.wrist (480,640,3)   observation.images.third_view (480,640,3)
          meta/info.json   meta/merge_manifest.json (which bag -> which episode, exclusions)
      |
      v
+------------------------------------------------------------------------------------------------------------------------+
| [6] INSPECT   python inspect_dataset_v21.py <dataset> --load   (conda lerobot_v2)                                      |
|------------------------------------------------------------------------------------------------------------------------|
|   PASS / FAIL : format v2.1 | episode / frame / mp4 counts | value ranges | grasp signal | LeRobotDataset load         |
|   per episode : frames, gripper range, idle time at start / end, /joint_command interval (median / max / >40 ms)       |
+------------------------------------------------------------------------------------------------------------------------+
      |
      v
      openpi pi0.5 fine-tuning (stage 7)   <-- NOT STARTED
```

- 화살표 옆 글자 = 토픽 이름, 메시지 타입, 주기 (sim 시간 기준), 내용
- 드론 조종기 입력 (5-A) 은 [1] 을 거치지 않고 [2] 로 바로 들어간다 (USB 직렬). 자세한 구조는 `docs/drone_teleop.md`
- 그림은 고정폭 글꼴에서 볼 것 (영문·ASCII 만 써서 정렬이 깨지지 않게 했다)

---

## 2. 블록별 상태

| 블록 | 무엇인가 | 상태 |
|------|----------|------|
| [1] 명령 소스 | 팔·그리퍼 명령을 보내는 쪽 | **텔레옵 노드는 아직 없다 (5단계 5-B, SpaceMouse).** 지금은 기록해 둔 명령을 재생하는 스크립트가 대신한다 |
| 드론 조종 ([2] 안) | 실물 조종기로 sim 드론을 움직인다 (`sim_ros2.py --drone-rc on`) | **만듦 (5단계 5-A, 2026-10-08)**: `check_rc.py` 6/6, 실물 조종기로 sim 드론이 움직이는 것 확인. 옵션을 안 켜면 지금처럼 자동 (리셋 뒤 호버) 또는 `drone_cmd.py` |
| [2] 시뮬레이터 | 로봇·카메라·드론을 계산하고 실물과 같은 토픽을 낸다 | 완성 (3단계, `check_ros2.py` 9/9) |
| [3] 녹화 도구 | 키 입력으로 에피소드를 녹화한다 | 완성 (4단계) |
| [4] stage1 | bag → 25 Hz 중간 파일 | 완성 (4단계) |
| [5] 병합 | 에피소드 여러 개 → v2.1 데이터셋 하나 | 완성 (4단계) |
| [6] 검수 | 데이터셋이 스키마에 맞는지 PASS / FAIL | 완성 (4단계) |
| 학습 | openpi π0.5 파인튜닝 | 아직 (7단계) |

[3] ~ [6] 을 한 번에 확인: `python3 isaacsim/scripts/make_fake_episodes.py` (sim 을 띄워 가짜 에피소드 녹화 → 변환 → 병합 → 검수, 3/3 PASS, 약 6 분).

---

## 3. 그림에서 선 하나로 줄인 연결

세로 한 줄로 그려서 아래 두 가지가 그림에 직접 드러나지 않는다.

| 연결 | 내용 |
|------|------|
| 명령 토픽은 두 군데로 간다 | [1] 이 보낸 `/joint_command`·`/gripper/command` 는 [2] 가 받아 로봇을 움직이고, **동시에 [3] 도 같은 토픽을 받아 bag 에 넣는다**. 데이터셋의 팔 action 이 이 녹화된 명령에서 나온다 |
| [3] → [2] 서비스 호출 | `r` → `/sim/reset`, `k` → `/sim/drone_kill`. 응답 (시드, 드론 위치, sim 설정, kill 시각) 을 `episode.json` 에 저장 |
| 드론 조종기 → [2] (5-A) | 조종기 입력은 [2] 안의 `PX4Commander` 로만 간다. **[3] 은 받지 않는다: 드론 정보는 녹화하지 않는다** (2026-10-08 사용자 결정. 녹화는 로봇 팔·그리퍼·카메라만) |

---

## 4. 실물일 때 달라지는 곳

| 블록 | 실물 |
|------|------|
| [1] | 텔레옵 노드. 또는 freedrive (손으로 팔을 움직임): 명령 토픽이 없어 변환을 `--arm-action next_state` 로 |
| [2] | UR 드라이버 (`1_arm_driver.sh`), 그리퍼 노드 (`3_gripper_node.sh`), RealSense 2 대 (`5_cameras.sh` → `launch/cameras.launch.py`). 토픽 이름·타입은 같다. `/protective_stop`·`/clock`·서비스 두 개는 없다. **stamp 는 sim time 대신 PC 시각이고 프로그램마다 따로 찍는다** (`docs/timestamp_sync.md`) |
| [3] | `./6_record_bag.sh <task>` (`--sim` 없음: `r` 은 리셋 없이 바로 녹화, `k` 없음) |
| [4] ~ [6] | 같은 스크립트 |

실물에서 확인된 범위 (2026-10-06, 노트북): 카메라 2 대 + 그리퍼 + 가짜 관절값으로 [3] ~ [6] 통과 (검수 5/5). 실물 팔과 텔레옵 `/joint_command` 경로는 아직.

---

## 5. 5단계에서 만들 것

두 가지다 (2026-10-08 결정, `docs/PLAN.md` 5단계). [3] ~ [6] 은 어느 쪽에서도 바꾸지 않는다.

| 순서 | 무엇 | 그림에서 | 장치 |
|------|------|----------|------|
| 5-A (먼저) | 드론 텔레옵 | [2] 안: 조종기 입력 → `PX4Commander` → PX4 | 실물 Radiolink 조종기 → Raspberry Pi Pico → USB 직렬 |
| 5-B | 팔 텔레옵 노드 | [1] 블록 | 3Dconnexion SpaceMouse Wireless (GELLO 는 대안) |

**5-A 드론 텔레옵** (`docs/drone_teleop.md`)
- 새 포트 없이 지금 명령 링크 (UDP 14540) 로 스틱 값 (`MANUAL_CONTROL`) 을 보낸다. 리셋·이륙은 지금처럼 자동, 그 뒤 스틱이 가운데면 조종기에 넘긴다
- 녹화되지 않는다. 옵션을 안 켜면 지금과 같다 (`check_ros2.py` 9/9)
- 만들었고 실물 조종기로 확인했다 (2026-10-08, 축 방향 맞음). 남은 것: `make_fake_episodes.py` 회귀, 스틱 끝 속도는 임시값 그대로

**5-B 팔 텔레옵 노드**: 아래만 지키면 [2] ~ [6] 은 지금 그대로 동작한다.
- `/joint_command` (팔 6 관절 목표 [rad]) 와 `/gripper/command` (raw 0 / 1150) 만 발행
- `/joint_command` 의 `header.stamp` 를 넣는다 (sim = sim time). 50 Hz 이상으로 일정하게
- 리셋 뒤에는 로봇의 홈 자세에서 시작 (리셋 전 자세의 명령을 보내면 팔이 33 ms 만에 그 자세로 뛴다)
- SpaceMouse 는 카테시안 입력이라 IK 가 필요하다. sim 안의 `arm_ik.py` (PhysX Jacobian) 는 sim 밖 노드에서 쓸 수 없어 URDF 에서 Jacobian 을 계산한다

---

## 6. 관련 문서

| 문서 | 그림의 어느 부분 |
|------|------------------|
| `docs/sim_ros2_interface.md` | [2] 시뮬레이터: 토픽·주기·QoS, 리셋·보호 정지 |
| `docs/drone_flight.md` | [2] 안의 드론 (추력 모델, PX4 SITL) |
| `docs/drone_teleop.md` | [2] 안의 드론을 실물 조종기로 움직이는 경로 (5단계 5-A 계획. 녹화되지 않음) |
| `docs/arm_command_interpolation.md` | [2] ArmBridge 의 명령 보간 |
| `docs/data_recording.md` | [3] 녹화: 키, 녹화 토픽, `episode.json`, 녹화 순서 |
| `README.md`, `dataset_merge.md` | [4] ~ [6] 변환·병합·검수 옵션, 실물 수집 순서 |
| `docs/timestamp_sync.md` | 모든 블록에 걸친 시각 (stamp) 동기화: sim 과 실물 |
| `docs/PLAN.md` | 단계별 계획·결과·남은 문제 |
| `CLAUDE.md` | 데이터셋 스키마, 원칙 |
