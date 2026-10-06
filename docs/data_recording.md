# ROS 2 데이터 녹화 (토픽·주기·구조·명령)

작성: 2026-10-05 · 4단계 완료 시점 (`docs/PLAN.md` 4단계), 브랜치 `isaacsim_v6.1.0`

텔레옵 시연을 ros2 bag 으로 녹화하고 LeRobot v2.1 데이터셋으로 만들기까지를 정리한다.
녹화 도구와 변환 스크립트는 sim 과 실물이 같은 것을 쓴다 (CLAUDE.md 원칙 1). 이 문서의 주기·측정값은 sim 기준이다.
sim 이 토픽을 내는 쪽은 `docs/sim_ros2_interface.md`, 변환 옵션 전체는 `README.md`, 병합 절차는 `dataset_merge.md`.

---

## 1. 한눈에 보기

| 항목 | 내용 |
|------|------|
| 녹화 | `./6_record_bag.sh <작업명> [--sim]` (= `record_toggle.py`). 키 `r` 시작/종료, `k` 드론 모터 정지 (sim), `d` 버리기, `q` 종료 |
| 단위 | `r` ~ `r` 구간 1 개 = bag 1 개 = 에피소드 1 개. 저장 `bags/<YYYYMMDD_HHMMSS>_<작업명>/` (`.mcap` + `metadata.yaml` + `episode.json`) |
| 토픽 | 상태 7 개 (팔·그리퍼·카메라 2 대와 camera_info) + 명령 2 개, sim 은 2 개 더 (3 장) |
| 녹화 전 검사 | 상태 토픽마다 발행자가 있는지, 카메라 발행자가 RELIABLE 인지. 문제가 있으면 **녹화를 시작하지 않는다** |
| 카메라 수신 | RELIABLE (best effort 면 640x480 이미지가 통째로 버려진다, `docs/sim_performance.md` 5장) |
| 수신 큐 | 상태 토픽마다 3 s 분량 (QoS override). sim 발행 큐도 3 s 분량. 기본 깊이 10 이면 녹화 시작 직후 메시지가 버려진다 (7 장) |
| 시각 | 모든 `header.stamp` 가 같은 시계 (sim = sim time). 변환은 도착 시각이 아니라 stamp 로 프레임을 맞춘다 |
| 성공/실패 | 사람이 정한다. 실패한 에피소드는 `d` 로 `bags/_discarded/` 로 옮기고 변환하지 않는다 |
| 용량 | 30 s 에피소드 약 1.5 GB (압축 없는 640x480 이미지 2 대, 약 55 MB/s). 자동으로 지우지 않는다 |
| 전체 확인 | `python3 isaacsim/scripts/make_fake_episodes.py` → 3/3 PASS (녹화 → 변환 → 병합 → 검수, 약 6 분) |

---

## 2. 구조

```text
 PUBLISHERS                                    RECORDER                                    FILES
 sim : sim_ros2.py (node "isaac_sim")          6_record_bag.sh -> record_toggle.py
 real: UR driver, rh_gripper_node, RealSense   keys: r start/stop, k drone kill (sim),
                                                     d discard, q quit
 +---------------------------+                 +-----------------------------------+
 | arm (measured)            |  /joint_states                 JointState 120 Hz    |
 |                           |---------------->|                                   |
 | gripper (measured, goal)  |  /gripper/joint_states         JointState  30 Hz    |
 |                           |---------------->|  1. check publishers              |
 |                           |  /gripper/target               JointState  30 Hz    |
 |                           |---------------->|     (state topics, camera QoS)    |
 | cameras x2                |  /cam/wrist/color/image_raw    Image       30 Hz    |
 |  (wrist, third_view)      |---------------->|  2. ros2 bag record               |      bags/<time>_<task>/
 |                           |  /cam/third_view/color/image_raw  Image    30 Hz    |        *.mcap
 |                           |---------------->|     (cameras RELIABLE)            |----->  metadata.yaml
 |                           |  /cam/*/color/camera_info      CameraInfo  30 Hz    |        episode.json
 |                           |---------------->|  3. write episode.json            |
 | sim only                  |  /protective_stop              Bool        30 Hz    |
 |                           |---------------->|                                   |      bags/_discarded/
 |                           |  /clock                        Clock      120 Hz    |----->  (key d: failed
 +---------------------------+---------------->|                                   |         episodes)
                                               |                                   |
 +---------------------------+                 |                                   |
 | teleop device (stage 5)   |  /joint_command                JointState ~60 Hz    |
 |  or replay_commands.py    |---------------->|                                   |
 |                           |  /gripper/command              Float64  on change   |
 +---------------------------+---------------->|                                   |
            | (same commands also go to the robot / sim)                           |
            v                                  |                                   |
 +---------------------------+  /sim/reset        Trigger  key r  (~24 s)          |
 | sim_ros2.py services      |<----------------|  reply JSON -> episode.json       |
 |  (sim only)               |  /sim/drone_kill   Trigger  key k                   |
 |                           |<----------------|  time     -> episode.json         |
 +---------------------------+                 +-----------------------------------+

 CONVERSION (after recording, one bag at a time)
 bags/<name>/ --stage1 (25 Hz grid on header.stamp)--> bag_lerobot_intermediate/<name>/ --merge--> lerobot_dataset_v21/<repo_id>/
```

- 화살표 위 글자 = 토픽 (또는 서비스) 이름, 메시지 타입, 주기. 주기는 sim 시간 기준 (실제 시간으로는 RTF 만큼 느리다)
- 명령 토픽 (`/joint_command`, `/gripper/command`) 은 텔레옵 장치가 로봇 (sim) 에 보내는 것을 녹화기가 같이 받는다
- `--sim` 일 때만: `r` 을 누르면 `/sim/reset` 을 부르고 **성공 응답이 온 뒤** 녹화를 시작한다. 실물 모드에는 서비스 호출과 `k` 키가 없다

---

## 3. 녹화하는 토픽

| 토픽 | 타입 | 발행 | 주기 (sim) | 종류 | 데이터셋에서 |
|------|------|------|-----------|------|--------------|
| `/joint_states` | sensor_msgs/JointState | UR 드라이버 / sim | 120 Hz (실물 125 Hz) | 상태 | `observation.state[0:6]` (팔 6 관절 rad) |
| `/gripper/joint_states` | sensor_msgs/JointState | 그리퍼 노드 / sim | 30 Hz | 상태 | `observation.state[6]` (present raw / 1150) |
| `/gripper/target` | sensor_msgs/JointState | 그리퍼 노드 / sim | 30 Hz (present 와 같은 stamp) | 상태 | `action[6]` (goal raw / 1150 = 0 또는 1) |
| `/cam/wrist/color/image_raw` | sensor_msgs/Image | RealSense / sim | 30 Hz | 상태 | `observation.images.wrist` (640x480) |
| `/cam/third_view/color/image_raw` | sensor_msgs/Image | RealSense / sim | 30 Hz | 상태 | `observation.images.third_view` (640x480) |
| `/cam/wrist/color/camera_info` | sensor_msgs/CameraInfo | RealSense / sim | 30 Hz | 상태 | 안 씀 (기록만) |
| `/cam/third_view/color/camera_info` | sensor_msgs/CameraInfo | RealSense / sim | 30 Hz | 상태 | 안 씀 (기록만) |
| `/joint_command` | sensor_msgs/JointState | 텔레옵 장치 | 텔레옵 주기 (재생 스크립트 60 Hz) | 명령 | `action[0:6]` (`--arm-action command`) |
| `/gripper/command` | std_msgs/Float64 | 텔레옵 장치 | 값이 바뀔 때 | 명령 | 안 씀 (참고용. 그리퍼 action 은 `/gripper/target`) |
| `/protective_stop` | std_msgs/Bool | sim | 30 Hz | 상태 (`--sim`) | `meta.json` 에 발생 여부·시각, 병합에서 기본 제외 |
| `/clock` | rosgraph_msgs/Clock | sim | 120 Hz | 상태 (`--sim`) | header 없는 토픽 (`/protective_stop`) 의 시각을 sim time 으로 환산 |

- **상태 토픽**은 항상 발행되고 있어야 한다. `r` 을 누르면 발행자를 확인하고, 없는 토픽이 있으면 목록을 출력하고 녹화하지 않는다
- **명령 토픽**은 발행자 확인을 하지 않는다 (텔레옵이 녹화 뒤에 움직이기 시작해도 된다). 녹화 중에 나타나면 녹화기가 찾아서 받는다
- 카메라 토픽 이름과 역할 ↔ 실물 모델·시리얼은 `config/cameras.yaml` 한 곳 (지금 wrist = D435i, third_view = D456)
- `/joint_command` 의 `header.stamp` 는 **보내는 쪽이 넣는다** (sim = sim time). 변환이 이 stamp 로 팔 action 을 맞추고 0 이면 에러
- `/base/imu` 는 8단계부터 (고정 베이스에서는 발행하지 않고 변환에서 `--base-imu const`)

### 실제로 녹화된 양 (sim, 기하 제어기 드론, 26 s bag 1 개, 2026-10-05)

| 토픽 | 메시지 수 | stamp 기준 주기 | 최대 간격 |
|------|-----------|-----------------|-----------|
| `/joint_states`, `/clock` | 2694 / 2774 | 120.00 Hz | 8.3 ms |
| 카메라 `image_raw` 2 대 | 673 / 673 | 30.00 Hz | 33.3 ms |
| `/gripper/joint_states`, `/gripper/target`, `/protective_stop` | 673 | 30.00 Hz | 33.3 ms |
| `/joint_command` (재생 스크립트) | 992 | 60 Hz (중앙값 16.67 ms) | 41.7 ms (4~6 % 가 25 ms 늦음, 재생 스크립트의 한계) |
| `/gripper/command` | 2 | 이벤트 | — |

사용자가 GUI 에서 녹화한 bag 5 개 (25~42 s, 기하·PX4) 도 카메라 2 대 장수가 같고 끊김 0.

---

## 4. 녹화 순서와 명령

터미널마다 먼저:
```bash
cd ~/data_collection_ur5_gripper
source /opt/ros/jazzy/setup.bash
```

### 4.1 sim

| 순서 | 터미널 | 명령 | 하는 일 | 기다릴 것 |
|------|--------|------|---------|-----------|
| 1 | 1 | `~/isaacsim/python.sh isaacsim/scripts/sim_ros2.py` | 씬 + ROS 2 토픽 + `/sim/reset`·`/sim/drone_kill`. 기본 = PX4 드론, 리셋마다 드론 위치 ±5 cm | `[sim_ros2] t ...` 상태 줄 (약 30 s) |
| 2 | 2 | `./6_record_bag.sh <작업명> --sim` | 녹화 도구 대기 | `[IDLE] 대기 중...` |
| 3 | 2 | 키 `r` | `/sim/reset` (팔 홈, 그리퍼 열림, 드론 재이륙, 약 24 s) → 성공하면 녹화 시작 | `[RESET] 완료 ...` → 벨 소리 + `[REC ●] 녹화 시작` |
| 4 | 3 또는 텔레옵 | 시연 (텔레옵 장치. 장치가 없으면 `python3 isaacsim/scripts/replay_commands.py isaacsim/config/ros2_check/commands_success.csv`) | 팔·그리퍼 명령 | 드론을 잡음 |
| 5 | 2 | 키 `k` | 드론 모터 정지 (`/sim/drone_kill`). 드론 무게가 그리퍼에 실린다 | `[KILL] 드론 모터 정지 (sim t ...)` |
| 6 | 2 | 키 `r` | 녹화 종료, bag 마무리 | `[SAVED] .../bags/<이름> (..s)` |
| 7 | 2 | (실패한 시연이면) 키 `d` → `y` | 방금 에피소드를 `bags/_discarded/` 로 | `[DISCARDED] → ...` |
| 8 | 2 | 3 번부터 반복, 끝나면 `q` | | `[EXIT] 종료` |

sim 옵션: `--flight geometric` (기하 제어기 드론, 리셋 약 2 s), `--reset-offset 0` (드론을 항상 기준 위치에), `--seed N`, `--headless --duration <s>`.

### 4.2 실물

`0_setup_network.sh` → `1_arm_driver.sh` → (freedrive 면 `2_arm_freedrive.sh`) → `3_gripper_node.sh` → `4_gripper_teleop.sh` → `5_cameras.sh` 뒤:
```bash
./6_record_bag.sh <작업명>          # r 시작 / r 종료, d 버리기, q 종료 (리셋·k 없음)
```
`5_cameras.sh` (`launch/cameras.launch.py`) 가 `/cam/wrist/...`·`/cam/third_view/...` 로 발행한다 (RealSense 드라이버 기본 QoS = RELIABLE). 자세한 순서는 `README.md` 1 절.
실물 PC (노트북, 카메라 2 대 + 그리퍼 + 가짜 관절값) 에서 녹화 → 변환 → 검수 5/5 를 확인했다 (2026-10-06, 카메라 29.99 Hz. PLAN "실물 PC 확인"). 실물 팔·텔레옵 경로는 아직.

### 4.3 녹화 확인

```bash
ls bags bags/_discarded
cat bags/<이름>/episode.json                 # 모드, 토픽, 카메라 역할 → 장치, sim 리셋 응답, kill 시각
ros2 bag info bags/<이름> | head -30         # 토픽별 메시지 수: 카메라 두 대 개수가 같고 ≈ 녹화 초 × 30
```
이미지를 직접 보려면 **sim 을 끈 뒤** `ros2 bag play bags/<이름>` + `ros2 run rqt_image_view rqt_image_view`.
sim 이 떠 있는 채로 재생하면 bag 의 `/clock`·`/joint_command` 가 다시 발행되어 로봇이 옛 명령대로 움직인다.

---

## 5. `episode.json` (bag 폴더, 녹화 도구가 저장)

| 키 | 내용 |
|----|------|
| `episode`, `mode`, `task_name`, `recorded_at`, `wall_duration_s` | bag 이름, `sim` / `real`, 작업명, 녹화 시각·길이 |
| `topics`, `camera_reliability` | 녹화한 토픽 목록, 카메라 수신 QoS |
| `cameras` | 역할 → 토픽 + 장치 (실물 = 모델·시리얼, sim = 설정 파일) |
| `sim_reset` (sim) | `/sim/reset` 응답: `seed`, `drone_goal`·`drone_pos` (world) 와 `_base` (로봇 base 기준), `t_start`·`t_end` (sim time), `duration_s`, `px4_log`, `sim` |
| `sim_reset.sim` | 고정 정보: git (commit·수정 여부), 물리·루프 주기, 렌더 (DLSS), 로봇·그리퍼 (`max_force_nm`·마찰)·드론 (제어기·위치 정보 방식) 설정, 카메라 prim·intrinsics·`cam_tilt_deg`, 보호 정지 기준 |
| `drone_kill` (sim) | `k` 를 누른 sim 시각 목록 |

변환 (stage1) 은 `episode.json` 이 없는 bag 을 받지 않는다. 내용은 중간 파일 `meta.json` 의 `episode` 로 그대로 들어간다.

---

## 6. 녹화 뒤: 변환 → 병합 → 검수

```bash
# 1) bag 마다 stage1 (시스템 python3 + ROS). 텔레옵·sim = command, 고정 베이스 = const
python3 lerobot_stage1_extract_bag.py bags/<이름> --arm-action command --base-imu const

# 2) 병합·검수 (conda lerobot_v2)
source ~/miniconda3/etc/profile.d/conda.sh && conda activate lerobot_v2
env -u PYTHONPATH python lerobot_merge_episodes_v21.py --manifest merge_<데이터셋>.txt --repo-id foodbanana/<데이터셋>   # 또는 --all
env -u PYTHONPATH python inspect_dataset_v21.py lerobot_dataset_v21/foodbanana/<데이터셋> --load
```

| 단계 | 하는 일 | 녹화와 관련된 규칙 |
|------|---------|--------------------|
| stage1 | `header.stamp` 기준 25 Hz 격자, 격자 시각 직전의 최신 메시지 (zero-order hold) | 고른 메시지가 **66 ms 보다 오래됐으면 에러** (`--max-age`). `command` 면 **첫 `/joint_command` 이전 구간을 뺀다**. stamp 0 이면 에러 |
| 병합 | 중간 파일 여러 개 → v2.1 데이터셋 하나 | 보호 정지 에피소드 기본 제외, 팔 action 종류가 다르면 중단, 정지 구간 자르기는 옵션 (`--trim-idle`) |
| 검수 | 형식·값 범위·파지 신호·LeRobot 로드, PASS / FAIL | 에피소드별 앞·뒤 정지 [s], `/joint_command` 간격 (중앙값·최댓값·40 ms 넘은 횟수) |

옵션 전체와 에러로 멈추는 경우는 `README.md` 2·3 절, 병합은 `dataset_merge.md`.

---

## 7. 알아 둘 점 (2026-10-05 기준, `docs/PLAN.md` "4단계에서 남은 문제")

| 항목 | 내용 |
|------|------|
| 녹화 시작 직후 메시지 누락 (고침) | 녹화가 시작된 직후 새 ROS 노드가 뜨면 0.4~1.5 s 동안 전달이 멈추고, 큐 깊이 10 (카메라 0.33 s 분량) 이 넘쳐 메시지가 버려졌다 (30 번 중 10 번). **발행 큐 (sim `publish_queue_sec`) 와 수신 큐 (녹화 도구 `QUEUE_SEC`) 를 모두 3 s 분량으로 늘려 고침** (90 번 중 0 번, PLAN "A1 추적"). 실물 드라이버의 발행 큐는 여기서 못 바꾸므로 실물에서는 남을 수 있다 → stage1 나이 검사가 잡고 `--skip-start 0.5` 로 살린다 |
| 녹화 중 새 노드 | 녹화 중에 `ros2 topic echo`·rqt 같은 새 노드를 띄우면 같은 전달 멈춤이 생길 수 있다. 필요한 노드는 녹화 전에 띄울 것 |
| 텔레옵 명령 주기 | 50 Hz 이상으로 일정하게, stamp 를 넣어서. 재생 스크립트는 `/clock` 을 받아 보내는 방식이라 4~6 % 가 25 ms 늦는다 (학습에 안 쓰는 가짜 데이터) |
| 리셋 뒤 시작 자세 | 리셋은 팔을 홈으로 순간이동한다. 텔레옵 장치는 홈 자세에서 시작해야 한다 (리셋 전 자세의 명령을 보내면 팔이 33 ms 만에 그 자세로 뛴다) |
| 드론 모터 정지 시점 | 정책의 action 은 아니지만 그 뒤의 이미지·관절 상태가 달라진다. 에피소드마다 일관되게 |
| 잡은 채 RTF 0.69 | 드론을 잡은 뒤 sim 이 약 30 % 느려진다. stamp 는 sim time 이라 데이터 정합성은 문제없고 조작감만 영향 |
| 실물 카메라 주기 | 실물 카메라는 30 Hz 로 발행한다. `ros2 topic hz <image_raw>` 가 20 Hz 로 보이는 것은 그 도구가 best effort 로 받기 때문 → **주기는 `camera_info` 로 잰다**. RELIABLE 녹화는 29.99 Hz, 변환에서 고른 이미지 나이 최대 33.4 ms |
| 실물 그리퍼 발행 간격 | 30 Hz 인데 고른 메시지 나이가 최대 48.2 ms (간격이 고르지 않음). 나이 한계 66 ms 까지 여유 18 ms |
| **실물 stamp 동기화** | 변환은 모든 토픽을 `header.stamp` 로 맞춘다. sim 은 모든 stamp 가 같은 물리 스텝이지만, 실물은 카메라 (RealSense 드라이버)·팔 (UR 드라이버)·그리퍼 (그리퍼 노드)·명령 (텔레옵 노드) 이 각자 stamp 를 찍는다. 그 차이만큼 한 프레임 안에서 이미지와 관절값이 어긋난다 (30 ms × 30 °/s = 0.9°). **카메라 ↔ 그리퍼 1 차 측정: −50 ~ 0 ms 사이** (`measure_stamp_offset.py`, 정확한 값은 못 정함: 닫을 때 −51, 열 때 0 ms). 카메라 stamp 는 찍은 시각에 가깝고 (`global_time_enabled` True), 그리퍼 stamp 는 발행 순간. **카메라 ↔ 팔은 재지 않음** → 새 팔이 오면 측정 (PLAN 4단계 "실물 stamp 동기화 문제 (D1)") |
| sim bag 재생 | sim 을 띄운 채 `ros2 bag play` 하지 말 것 |

---

## 8. 관련 파일·문서

| 파일 | 내용 |
|------|------|
| `6_record_bag.sh`, `record_toggle.py` | 녹화 도구 (토픽 목록 `STATE_TOPICS`·`COMMAND_TOPICS`·`SIM_STATE_TOPICS`) |
| `config/cameras.yaml`, `camera_config.py` | 카메라 역할 ↔ 토픽·실물 장치 |
| `isaacsim/scripts/sim_ros2.py`, `sim_reset.py` | sim 토픽, `/sim/reset`·`/sim/drone_kill` |
| `isaacsim/scripts/replay_commands.py` | 명령 CSV 재생 (텔레옵 대신) |
| `isaacsim/scripts/make_fake_episodes.py` | 녹화 → 변환 → 병합 → 검수 자동 확인 |
| `docs/sim_ros2_interface.md` | sim 이 내는 토픽·주기·QoS, 리셋·보호 정지 |
| `docs/timestamp_sync.md` | 시각 (stamp) 동기화: sim 과 실물에서 stamp 를 누가 찍는지, 측정 결과, 모르는 것 |
| `README.md`, `dataset_merge.md` | 변환·병합·검수 실행 방법과 옵션 |
| `docs/PLAN.md` 4단계 | 4-0 ~ 4-5 결과, 결정과 이유, 남은 문제 |
