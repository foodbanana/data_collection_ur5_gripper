# data_collection_ur5_gripper

UR5 + RH-P12-RN(A) 그리퍼 + RealSense 2대로 드론 파지 시연 데이터를 수집하고,
ros2 bag → **LeRobot v2.1 데이터셋** 으로 변환하는 스크립트 모음. 실물과 Isaac Sim (`isaacsim/`) 이 녹화·변환을 같이 쓴다.

**목적**: openpi π0.5 를 LoRA 로 파인튜닝하기 위한 데이터 수집. openpi 는 LeRobot **v2.1** 포맷만 호환.
전체 흐름은 아래 [전체 구조](#전체-구조) 참고. 데이터셋 스키마와 원칙은 `CLAUDE.md`, 계획·단계별 결과는 `docs/PLAN.md`,
전체 구조 한 장은 [docs/architecture.md](docs/architecture.md), 녹화 토픽·주기·구조도와 sim 녹화 순서는 [docs/data_recording.md](docs/data_recording.md).

기존 워크스페이스(`~/ur_freedrive_ws`, `~/rh_gripper_ros2_ws`, `~/realsense_ws`)는
**source 만** 한다. 이 폴더는 실행·녹화·변환 관리 전용.

수집 컨셉:
- 로봇팔 = 텔레옵 (`/joint_command`, sim·앞으로의 실물) 또는 freedrive (손으로 직접 움직임, 명령 토픽 없음)
- 그리퍼 = 열림/닫힘 이진 명령 (`/gripper/command`)
- 1 녹화 = 1 에피소드 = bag 1개. 실패한 에피소드는 사람이 버리고, 남은 것을 하나의 데이터셋으로 병합

> **2026-10-05 변경 (PLAN 4단계)**: 카메라 토픽·키 (`/cam/<역할>/...`, `head` → `wrist`), 그리퍼 값 (raw → 0~1), `observation.base_imu` 추가,
> 변환 스크립트 필수 인자 (`--arm-action`, `--base-imu`), v3.0 변환 제거. 그 전에 만든 bag·중간 파일·데이터셋은 지금 스크립트로 읽지 못한다.

---

## 전체 구조

### 1. 데이터 수집 파이프라인 (ros2bag)

```text
 ARM                        GRIPPER                      CAMERAS (color 640x480, 30 Hz)
 real: ur_robot_driver      real: rh_gripper_node        real: RealSense x2 (config/cameras.yaml)
 sim : sim_ros2.py          sim : sim_ros2.py            sim : sim_ros2.py
   |                          |                            |
   | /joint_states            | /gripper/joint_states      | /cam/wrist/color/image_raw
   |   (arm 6, ~125 Hz)       |   (present, 30 Hz)         | /cam/third_view/color/image_raw
   | /joint_command           | /gripper/target            |   (+ camera_info)
   |   (teleop target)        |   (goal, 30 Hz)            |
   |                          | /gripper/command           |
   +--------------------------+----------------------------+
                              v
        6_record_bag.sh -> record_toggle.py [task] [--sim]
          r = start / stop    d = discard last    q = quit    (--sim: r = /sim/reset then record, k = drone motor off)
          checks publishers before recording, cameras received RELIABLE
                              v
        bags/<YYYYMMDD_HHMMSS>_<task>/   (.mcap + episode.json)
```

### 2. ros2bag → LeRobot v2.1 → 검수

```text
 bags/<name>/
     |  lerobot_stage1_extract_bag.py <bag> --arm-action command|next_state --base-imu const|topic     (ROS 2, system python)
     |    25 Hz grid on header.stamp (zero-order hold), error if a picked message is older than 66 ms
     v
 bag_lerobot_intermediate/<name>/
     states.npy (N,7)  actions.npy (N,7)  base_imu.npy (N,6)  timestamps.npy  wrist/*.png  third_view/*.png  meta.json
     |
     |  one episode : lerobot_stage2_build_dataset_v21.py <dir>            (conda lerobot_v2, lerobot 0.3.3)
     |  many        : lerobot_merge_episodes_v21.py --manifest merge_<name>.txt | --all
     v
 lerobot_dataset_v21/<repo_id>/      (codebase_version v2.1, openpi training)
     |
     |  inspect_dataset_v21.py <dataset> [--load]     PASS / FAIL
     v
 inspect_parquet.py <episode.parquet>                 one episode, row by row
```

`convert_ros2bag_lerobot.sh <bag> --arm-action … --base-imu …` 는 bag 하나에 stage1 → stage2 를 이어서 돌린다.

---

## 핵심 설계 (바꾸기 전에 꼭 읽을 것)

| 항목 | 값 | 이유 |
|------|----|------|
| `observation.state` (7) | 팔 6관절(rad) + 그리퍼 **present / 1150** (0~1 연속, 0 = 열림) | 실측 상태 |
| `action` 팔 (6) | `--arm-action command`: `/joint_command` 의 그 프레임 직전 최신 명령 (텔레옵·sim) / `next_state`: `states[i+1]` 의 팔 (freedrive) | 두 방식은 의미가 달라 한 데이터셋에 섞지 않는다 (병합이 막음) |
| `action` 그리퍼 (1) | `/gripper/target` 의 goal / 1150 (이진 {0.0, 1.0}) | 실제로 내려진 goal 명령. **present 의 다음 프레임이 아님** |
| `observation.base_imu` (6) | 각속도 xyz (rad/s) + 선가속도 xyz (m/s²) | 고정 베이스는 `--base-imu const` = [0, 0, 0, 0, 0, 9.81] |
| 그리퍼 값 | 토픽은 raw 0(열림) ~ 1150(닫힘) 그대로, **0~1 정규화는 stage1 에서만** | 규약·가독성 (π0.5 는 자체 norm stats 를 씀) |
| fps | 25 고정 | 각 토픽 `header.stamp` 기준 zero-order hold 리샘플 |
| 카메라 | `/cam/wrist/...` → `wrist`(손목), `/cam/third_view/...` → `third_view`(외부 고정) | 역할 ↔ 실물 시리얼은 `config/cameras.yaml` 한 곳 |

**왜 그리퍼 action 을 target 으로 쓰나**: 물체를 잡으면 present 는 물체에 막혀 중간(예: raw 742 = 0.65)에서 멈추지만
goal 은 1150(= 1.0) 이다. action 을 present 로 만들면 "0.65 유지"로 기록되어, 추론 때 파지력이 생기지 않는다
(전류기반 위치제어는 goal 과 present 의 차이로 힘을 낸다). target 을 쓰면 "꽉 닫아라(1.0)"가 그대로 학습된다.

**`/joint_command` 의 `header.stamp` 는 보내는 쪽(텔레옵 노드)이 넣는다** (sim 은 sim time, 실물은 PC 시각). stage1 이 이 stamp 로 팔 action 을 맞추고 0 이면 에러.

그리퍼 제어: Current-based Position Control(mode 5) + Goal Current 400mA(힘 상한).
그리퍼 구조·토픽 상세는 [1DOF_gripper_data_collection.md](1DOF_gripper_data_collection.md) 참고.

---

## 최초 1회

```bash
chmod +x *.sh
```

각 스크립트 상단의 사용자 환경 변수(인터페이스 이름, IP, 캘리브레이션 경로 등)를
자신의 환경에 맞게 확인/수정한다. 카메라 역할 ↔ 모델·시리얼은 `config/cameras.yaml`.

변환에는 conda 환경이 필요하다 (`~/miniconda3`, `convert_ros2bag_lerobot.sh` 상단에서 이름 변경 가능):

| conda 환경 | 구성 | 용도 |
|-----------|------|------|
| `lerobot_v2` | python 3.10 + `pip install "lerobot==0.3.3"` | v2.1 데이터셋 (stage2·병합·검수) |

conda base 자동 활성화는 꺼 둔다 (`conda config --set auto_activate_base false`). 켜져 있으면 ROS·Isaac Sim 의 python 과 섞인다.

**실물 카메라 런치**: `5_cameras.sh` 가 이 레포의 `launch/cameras.launch.py` 를 실행한다. `config/cameras.yaml` 을 읽어 역할마다
realsense2_camera 노드 하나 (`namespace cam`, `name wrist` / `third_view`) 를 띄우므로 토픽이 `/cam/wrist/color/image_raw`, `/cam/third_view/color/image_raw` 로 나온다
(지금 wrist = D435i, third_view = D456. 카메라를 바꾸면 yaml 의 model·serial 만 고친다). `~/realsense_ws` 는 realsense2_camera 패키지를 쓰려고 source 만 하고,
예전 `realsense_dual_camera` 런치 (토픽 `/d435i/d435i/...`) 는 쓰지 않는다.
주기는 `ros2 topic hz /cam/wrist/color/camera_info` 로 잰다 (`image_raw` 를 `ros2 topic hz` 로 재면 best effort 수신·파이썬 처리 때문에 실제보다 낮게 나온다).

---

## 실행 전 확인

```bash
# 카메라 2대가 모두 USB 3.x 에 연결됐는지
rs-enumerate-devices | grep -iE "serial number|usb type"

# 그리퍼 U2D2 포트 이름 (보통 /dev/ttyUSB0), 파워서플라이 24V ON
ls -l /dev/ttyUSB*
```

---

## 1. 수집 — 실행 순서 (터미널을 여러 개 연다)

수동 단계(★)가 사이사이에 끼어 있으니 순서를 지킬 것.

### [사전] 네트워크 (부팅 후 1회, sudo)
```bash
cd ~/data_collection_ur5_gripper
./0_setup_network.sh
```

### ★ 폴리스코프 준비
- 시작 → 프로그램 로봇 → 구조 탭 → URCaps → External Control 프로그램 준비
- (아직 재생 누르지 말 것)

### [터미널 1] UR 드라이버
```bash
./1_arm_driver.sh
```

### ★ 펜던트에서 External Control 재생(▶)

### [터미널 2] freedrive 활성화
```bash
./2_arm_freedrive.sh
```
- freedrive ON → 손으로 팔을 움직일 수 있음
- 종료: Enter (freedrive 해제 + 컨트롤러 원복)

### [터미널 3] 그리퍼 노드
```bash
./3_gripper_node.sh
```
- 발행(30Hz, 같은 tick·같은 stamp): `/gripper/joint_states`(present) + `/gripper/target`(goal)
- 구독: `/gripper/command` (raw 0~1150)
- 시작 시 그리퍼를 **열림(0)으로 초기화** (에피소드 시작 상태 일관성)
- 파지력 조절이 필요하면: `ros2 run rh_p12_rn_ros2 rh_gripper_node --ros-args -p goal_current:=<mA>` (기본 400)
- 이 터미널은 켜둔 채로 둔다

### [터미널 4] 그리퍼 teleop (키보드 제어)
```bash
./4_gripper_teleop.sh
```
- `0` = 완전 열림, `1` = 완전 닫힘, `q` = 종료 (이진 명령만. 부드러운 이동은 그리퍼의 Profile Velocity 가 담당)
- 터미널 3이 켜져 있어야 실제로 움직인다

### [터미널 5] 카메라 2대
```bash
./5_cameras.sh
```

### [확인] 토픽이 다 살아있는지 (선택, 새 터미널)
```bash
source /opt/ros/jazzy/setup.bash
source ~/realsense_ws/install/setup.bash
ros2 topic hz /joint_states
ros2 topic hz /gripper/joint_states
ros2 topic hz /gripper/target
ros2 topic hz /cam/wrist/color/camera_info        # 카메라 주기는 camera_info 로 (image_raw 는 실제보다 낮게 나온다)
ros2 topic hz /cam/third_view/color/camera_info
```

### [터미널 6] 에피소드 녹화 (r 토글)
```bash
./6_record_bag.sh                  # 이름: <날짜시간>
./6_record_bag.sh pick_drone       # 이름: <날짜시간>_pick_drone
./6_record_bag.sh pick_drone --sim # Isaac Sim (sim_ros2.py 실행 중): r 을 누르면 /sim/reset 뒤 녹화, k = 드론 모터 정지
```
실행하면 **대기 상태**(녹화 안 함). 한 번 켜두고 키로 에피소드를 반복 녹화한다.

| 키 | 동작 |
|----|------|
| `r` | 녹화 시작 → 다시 `r` : 녹화 종료 (r~r 구간 1개 = bag 1개 = 에피소드 1개) |
| `d` | 방금 저장한 에피소드 버리기(실패한 시연) → `y` 로 확인하면 `bags/_discarded/` 로 **이동**(삭제 아님) |
| `q` | 종료 (녹화 중이면 현재 bag 을 안전 종료한 뒤 종료) |
| `k` | (`--sim` 만) 드론 모터 정지. 잡은 뒤 누른다. 시각이 `episode.json` 에 남는다 |

- `r` 을 누르면 먼저 상태 토픽 (`/joint_states`, 그리퍼 2개, 카메라 2대) 에 발행자가 있는지, 카메라가 RELIABLE 인지 확인한다.
  문제가 있으면 무엇이 없는지 출력하고 **녹화를 시작하지 않는다**
- bag 폴더에 `episode.json` (모드, 토픽, 카메라 역할 → 장치, sim 은 리셋 응답) 이 같이 저장된다. 변환에 필요하다

- 한 에피소드 흐름: 시작 자세 세팅 → `r` → 시연(한 손은 팔, 터미널 4에서 `0`/`1`) → `r` → `[SAVED]` 확인 → (실패면 `d`,`y`)
- `r`/`d`/`q` 는 **터미널 6**, 그리퍼 `0`/`1` 은 **터미널 4** 에 포커스가 있어야 입력된다
- 이름의 날짜시간은 `r` 로 시작하는 순간에 찍힘 → 에피소드끼리 겹치지 않음
- 종료 후 `[SAVED]` 가 뜰 때까지 기다릴 것 (카메라 이미지 flush 에 몇 초 걸릴 수 있음)
- `d` 는 대기 중에만, 이번 세션에서 방금 저장한 1개만 대상. 되살리려면 폴더를 `bags/` 로 다시 옮기면 됨
- `_discarded/` 는 자동으로 비워지지 않음 → 용량이 차면 직접 삭제
- 한글 입력 상태여도 동작 (`ㄱ`=r, `ㅇ`=d, `ㅂ`=q, `ㅛ`=y)
- 저장 위치: `~/data_collection_ur5_gripper/bags/<이름>/`
- 실제 로직은 `record_toggle.py`. bag 종료는 SIGINT 로만 한다(SIGKILL 금지 → mcap 손상 방지).
  Ctrl+C 나 터미널 창을 닫아도 현재 bag 을 안전 종료한 뒤 끝난다.


---

## 녹화되는 토픽

| 토픽 | 타입 | 용도 |
|------|------|------|
| `/joint_states` | sensor_msgs/JointState | 팔 6관절 → `observation.state` 앞 6칸 |
| `/joint_command` | sensor_msgs/JointState | 텔레옵 팔 관절 목표 → **팔 action** (`--arm-action command`). freedrive 수집에서는 없음 |
| `/gripper/joint_states` | sensor_msgs/JointState | 그리퍼 present, raw 0~1150 → `observation.state` 7번째 (/1150) |
| `/gripper/target` | sensor_msgs/JointState | 그리퍼 goal, 30Hz 상시 발행 → **그리퍼 action** (/1150) |
| `/gripper/command` | std_msgs/Float64 | teleop 이산 명령 (참고용, 변환에는 안 씀) |
| `/cam/wrist/color/image_raw` (+ `camera_info`) | sensor_msgs/Image | 손목 카메라 → `observation.images.wrist` |
| `/cam/third_view/color/image_raw` (+ `camera_info`) | sensor_msgs/Image | 외부 고정 카메라 → `observation.images.third_view` |
| `/protective_stop`, `/clock` | std_msgs/Bool, Clock | `--sim` 만: 보호 정지 흉내, sim time |

depth 는 제외(color 만). 압축 없음 (30 s 에피소드 약 1.5 GB). 토픽 목록은 `record_toggle.py` 의 `STATE_TOPICS`·`COMMAND_TOPICS` + `config/cameras.yaml`.

---

## 2. 변환 — bag → LeRobot v2.1

### bag 하나 → 데이터셋 하나
```bash
./convert_ros2bag_lerobot.sh bags/<에피소드이름> --arm-action command --base-imu const --task "pick up the drone"
```

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--arm-action command\|next_state` | **필수** | 팔 action: `command` = `/joint_command` (텔레옵·sim), `next_state` = `states[i+1]` (freedrive) |
| `--base-imu const\|topic` | **필수** | `const` = 고정 베이스 상수, `topic` = `/base/imu` 구간 평균 (흔들리는 베이스, PLAN 8단계) |
| `--task "문장"` | `pick up the drone` | 언어 명령 |
| `--fps 숫자` | `25` | 리샘플 목표 fps (25 고정 사용) |
| `--namespace 이름` | `foodbanana` | repo_id 앞부분 |
| `--project 이름` | `ur5_gripper_drone` | repo_id 프로젝트명 |

| 단계 | 환경 | 스크립트 | 출력 |
|------|------|----------|------|
| 1단계 | ROS 2 (시스템 python) | `lerobot_stage1_extract_bag.py` | `bag_lerobot_intermediate/<bag이름>/` |
| 2단계 | conda `lerobot_v2` | `lerobot_stage2_build_dataset_v21.py` | `lerobot_dataset_v21/<repo_id>/` (`repo_id = <namespace>/<project>_<bag이름>`) |

2단계로 나눈 이유: `rosbag2_py` 는 ROS 2 시스템 파이썬에만, `lerobot` 은 conda 에만 있어 한 프로세스에서 같이 import 하면 충돌한다.

### 에피소드 여러 개 → 학습용 데이터셋 하나 (보통 이쪽)
```bash
source /opt/ros/jazzy/setup.bash
for b in bags/*_pick_drone; do                      # _discarded/ 는 변환하지 않는다
  python3 lerobot_stage1_extract_bag.py "$b" --arm-action command --base-imu const
done
source ~/miniconda3/etc/profile.d/conda.sh && conda activate lerobot_v2
env -u PYTHONPATH python lerobot_merge_episodes_v21.py --manifest merge_<이름>.txt --repo-id foodbanana/<이름>
env -u PYTHONPATH python inspect_dataset_v21.py lerobot_dataset_v21/foodbanana/<이름> --load
```
자세한 절차·옵션 (`--all`, `--include-protective-stop`, `--trim-idle`) 은 [dataset_merge.md](dataset_merge.md).

**1단계 출력(중간 파일)**
```
states.npy      (N, 7)  [팔6(rad) + 그리퍼 present/1150]              → observation.state
actions.npy     (N, 7)  [팔6 목표(rad) + 그리퍼 goal/1150 {0, 1}]     → action
base_imu.npy    (N, 6)  [각속도 xyz + 선가속도 xyz]                   → observation.base_imu
timestamps.npy  (N,)
wrist/000000.png ...       (/cam/wrist/color/image_raw)
third_view/000000.png ...  (/cam/third_view/color/image_raw)
meta.json       옵션, 카메라 역할 → 장치, 메시지 나이, /joint_command 간격 통계, 보호 정지, episode.json
```
- action 은 **1단계에서 완성**한다. 2단계·병합은 `actions.npy` 를 그대로 읽어 쓰기만 한다
- `--arm-action command`: 첫 `/joint_command` 이전 구간은 에피소드에서 뺀다 (버린 프레임 수는 `meta.json`). `next_state`: 마지막 프레임의 팔 action 은 `states[N-1]` 복제
- `/joint_states`·`/joint_command` 는 `name` 기준으로 관절 순서를 재정렬한다
  (shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3)
- 출력 폴더가 이미 있으면 에러 (`--overwrite` 로 지우고 다시)

**의도적으로 에러로 중단하는 경우 (조용한 fallback 없음)**
- bag 에 `episode.json` 이 없음 → `record_toggle.py` 로 녹화한 bag 만 변환한다
- 필요한 토픽 (`/joint_states`, 그리퍼 2개, 카메라 2대, `command` 면 `/joint_command`) 에 메시지가 없음
- `header.stamp` 가 0 인 메시지가 있음 (텔레옵 노드가 `/joint_command` 에 stamp 를 안 넣음)
- **격자 시각에 고른 메시지가 66 ms 보다 오래됨** (`--max-age`): 이미지·상태가 빠진 bag. 옛 이미지 + 새 관절값 프레임을 만들지 않는다.
  녹화 시작 직후에만 빠졌으면 `--skip-start 0.5` 로 그 구간을 빼고 변환할 수 있다
- 그리퍼 present 가 0~1150 밖이거나 target 이 0 / 1150 이 아님
- bag 에 `/base/imu` 가 있는데 `--base-imu const` 를 줌 (또는 반대)
- 병합: 에피소드끼리 팔 action 종류·fps·이미지 크기가 다름, 출력 데이터셋 폴더가 이미 있음

---

## 3. 검수

### bag 검사
```bash
source /opt/ros/jazzy/setup.bash
python3 inspect_ur5_gripper_ros2_bag.py bags/<에피소드이름>
```
- 토픽 목록·타입·메시지 개수, 토픽별 샘플 3개, 카메라 첫 프레임 png
- bag 전체를 읽지 않고 앞쪽 60초까지만 훑는다
- png 저장 위치: `inspect_out/<bag이름>_<검사시각>/` (`bags/` 안에는 실제 bag 만 남도록 분리)

빠른 확인: `ros2 bag info bags/<이름>` → 카메라 두 대의 `image_raw` 개수가 같고 ≈ 녹화 초 × 30

### 데이터셋 검사 (PASS / FAIL)
```bash
conda activate lerobot_v2
env -u PYTHONPATH python inspect_dataset_v21.py lerobot_dataset_v21/<repo_id> [--load]
```
1 형식 (`codebase_version` v2.1, fps 25, feature 키·크기) · 2 에피소드·프레임·mp4 수 · 3 값 범위 (유한값, 팔 ±2π, 그리퍼 state 0~1,
그리퍼 action {0.0, 1.0}) · 4 파지 신호 (닫기 명령 중 state 가 중간에서 멈춤, 최댓값 < 0.95) · 5 `--load`: LeRobotDataset 으로 열어 프레임 읽기.
에피소드별 표에 앞·뒤 정지 구간 [s] 과 `/joint_command` 간격 (중앙값·최댓값·40 ms 넘은 횟수 → 텔레옵 명령이 끊긴 에피소드) 이 나온다.

### 에피소드 하나(parquet) 들여다보기
```bash
env -u PYTHONPATH python inspect_parquet.py \
  lerobot_dataset_v21/<repo_id>/data/chunk-000/episode_000000.parquet [--rows 20] [--xlsx]
```
- 열 목록, `observation.state` / `action` 을 관절별 열(j1~j6, grip)로 펼친 미리보기
- 그리퍼: action 이 이진 {0, 1} 인지, state 범위, 닫기 명령(action=1) 구간에서 state 가 어디서 멈췄는지
  (예: `action=1 인데 state=0.26 에서 멈춤` = 물체를 잡음)
- `--xlsx` : 펼친 표를 엑셀로 저장

---

## 로봇팔 없이 그리퍼+카메라만 테스트할 때

- `0`, `1`, `2` (로봇팔 관련) 건너뛰기
- 대신 가짜 팔 관절을 발행:
  ```bash
  source /opt/ros/jazzy/setup.bash
  python3 fake_joint_states.py        # /joint_states 발행
  ```
- `3`, `4`, `5` 실행 후 `6` 으로 녹화 → 변환 (`--arm-action next_state --base-imu const`)·검수까지 그대로 가능
- `fake_joint_states.py` 없이는 `/joint_states` 발행자가 없어 녹화 도구가 녹화를 시작하지 않는다
- ※ 실제 UR 드라이버(터미널 1)와 **동시에 켜지 말 것** (`/joint_states` 충돌)

---

## 종료 순서

1. 터미널 6: `q` (녹화 중이면 안전 종료 후 종료)
2. 터미널 2: Enter (freedrive 해제 + 원복) — **반드시 원복하고 끌 것**
3. 터미널 4: `q` 또는 Ctrl+C (teleop 종료)
4. 터미널 3, 5: Ctrl+C (그리퍼 노드는 종료 시 토크 OFF)
5. 터미널 1: Ctrl+C

---

## 알아둘 점 / 문제 해결

- **그리퍼 present 가 42억(4294967295)으로 튀던 문제** — 열림(0) 근처 엔코더 노이즈 -1 을
  Dynamixel SDK 가 unsigned 로 읽어서 생김. `rh_gripper_node.py` 의 `_read_present_position` 에서
  signed 복원 + 0~1150 clamp 로 차단함(실물 검증 완료). 이 수정 **이전에** 녹화한 bag 에는 남아 있을 수 있으니
  학습에 쓰지 말 것. 지금 stage1 은 present 가 0~1150 밖이면 에러로 멈춘다.
- **그리퍼 노드 수정 후 재빌드** — `~/rh_gripper_ros2_ws` 는 `--symlink-install` 로 빌드되어 있어
  파이썬 소스 수정은 재빌드 없이 노드 재시작만 하면 반영된다.
- **`r` 을 눌렀는데 `[ERROR] 녹화를 시작하지 않음 — 토픽 문제`** — 나열된 토픽의 장치 노드(터미널 1/3/5)가 안 떠 있거나, 카메라가 RELIABLE 로 발행하지 않음.
- **`bags/` 안에는 bag 만** — `_discarded/` 는 예외 (변환하지 않는다).
- **디스크 용량** — 비압축 카메라 2대라 30 s 에피소드가 약 1.5 GB. 자동으로 지우지 않는다 → `bags/`, `bags/_discarded/`, `bag_lerobot_intermediate/`(png) 를 직접 정리.
- **sim 을 띄운 채 sim bag 을 `ros2 bag play` 하지 말 것** — bag 의 `/clock`·`/joint_command` 가 다시 발행되어 로봇이 옛 명령대로 움직인다.

---

## 파일 목록

**수집**

| 파일 | 터미널 | 역할 |
|------|--------|------|
| `0_setup_network.sh` | 사전(sudo) | 노트북↔UR5 IP 설정 |
| `1_arm_driver.sh` | 1 | UR 드라이버 (이후 펜던트 재생 ▶) |
| `2_arm_freedrive.sh` | 2 | freedrive ON (Enter 로 해제+원복) |
| `3_gripper_node.sh` | 3 | 그리퍼 present/target 발행 + 명령 수신 |
| `4_gripper_teleop.sh` | 4 | 그리퍼 키보드 제어 (0/1) |
| `5_cameras.sh`, `launch/cameras.launch.py` | 5 | 카메라 2대 (color). `config/cameras.yaml` 의 역할 이름으로 `/cam/<역할>/...` 발행 |
| `6_record_bag.sh` | 6 | 에피소드 녹화 (r 토글) — `record_toggle.py` 실행 래퍼 |
| `record_toggle.py` | 6 | 녹화 토글 본체 (토픽 목록, 녹화 전 검사, `--sim` 리셋·드론 kill 연동) |
| `config/cameras.yaml`, `camera_config.py` | - | 카메라 역할 (wrist / third_view) ↔ 실물 모델·시리얼, 토픽 이름. 녹화·변환 공용 |
| `fake_joint_states.py` | - | 로봇팔 없을 때 `/joint_states` 대체 발행 |
| `my_robot_calibration.yaml` | - | UR5 캘리브레이션 보관용 사본. `1_arm_driver.sh` 가 실제로 읽는 것은 `~/my_robot_calibration.yaml` (현재 내용 동일) |

**변환 / 검수**

| 파일 | 환경 | 역할 |
|------|------|------|
| `convert_ros2bag_lerobot.sh` | - | bag 1개 → 1단계 → v2.1 일괄 실행 |
| `lerobot_stage1_extract_bag.py` | ROS 2 | bag → 중간 파일 (`states.npy`, `actions.npy`, `base_imu.npy`, png, `meta.json`) |
| `lerobot_v21_common.py` | - | 중간 파일 읽기·스키마 검사·feature 정의 (2단계·병합·검수 공용) |
| `lerobot_stage2_build_dataset_v21.py` | conda `lerobot_v2` | 중간 파일 1개 → LeRobot v2.1 |
| `lerobot_merge_episodes_v21.py` | conda `lerobot_v2` | 중간 파일 여러 개 → LeRobot v2.1 하나 (**학습용**, [dataset_merge.md](dataset_merge.md)) |
| `inspect_dataset_v21.py` | conda `lerobot_v2` | 데이터셋 전체 검수 (PASS / FAIL) |
| `inspect_parquet.py` | conda | 에피소드 parquet 하나 들여다보기 |
| `inspect_ur5_gripper_ros2_bag.py` | ROS 2 | bag 내용 검사 → `inspect_out/` |
| `lerobot_stage2_build_dataset_v30.py` | conda `lerobot_v1` | (옛 형식 전용) LeRobot v3.0. 지금 중간 파일은 읽지 못한다. 학습에 쓰지 않음 |

**폴더**

| 폴더 | 내용 |
|------|------|
| `bags/` | 녹화된 에피소드 bag (`_discarded/` = `d` 로 버린 에피소드) |
| `bag_lerobot_intermediate/` | 1단계 중간 파일 |
| `lerobot_dataset_v21/` | 변환된 데이터셋 |
| `inspect_out/` | bag 검사 이미지 미리보기 |

**관련 문서 / 외부 코드**
- [1DOF_gripper_data_collection.md](1DOF_gripper_data_collection.md) — 그리퍼 노드·토픽·구조 메모
- `~/rh_gripper_ros2_ws/src/rh_p12_rn_ros2/` — `rh_gripper_node.py`, `rh_gripper_teleop.py`

---

## Isaac Sim 파이프라인 (브랜치 `isaacsim_v6.1.0`)

같은 데이터를 Isaac Sim 6.1.0 에서도 모은다. sim 은 실물과 같은 ROS 2 토픽을 내므로 녹화·변환은 위와 같은 코드를 쓴다.
sim 코드는 모두 `isaacsim/` 아래, 계획·결과는 `docs/PLAN.md`, 토픽·구조는 `docs/sim_ros2_interface.md`.

```bash
cd ~/data_collection_ur5_gripper
source /opt/ros/jazzy/setup.bash
~/isaacsim/python.sh isaacsim/scripts/sim_ros2.py          # 터미널 1: 씬 (UR5 + 그리퍼 + 카메라 2대 + PX4 드론) + ROS 2 토픽
./6_record_bag.sh pick_drone --sim                         # 터미널 2: r = 리셋 (약 24 s) 뒤 녹화 / 다시 r = 종료, k = 드론 모터 정지, d = 버리기
python3 isaacsim/scripts/check_ros2.py                     # sim ROS 2 자동 검사 → 9/9 PASS (약 8 분)
python3 isaacsim/scripts/make_fake_episodes.py             # 녹화 → 변환 → 병합 → 검수 자동 확인 → 3/3 PASS (약 6 분)
```
변환은 `--arm-action command --base-imu const`. 단계별 실행 명령은 `docs/PLAN.md` 부록 D.

---

## 다음 단계 (아직 안 만듦)

- 텔레옵 장치 (`/joint_command` + `/gripper/command`, PLAN 5단계). `/joint_command` 에 stamp 를 넣고 일정한 주기 (50 Hz 이상) 로 보낼 것
- 실물 PC 에서 새 토픽 이름 (`/cam/...`) 으로 녹화 → 변환 전 구간 검증 (지금까지 새 파이프라인은 sim 으로만 확인)
- openpi 쪽: data config (`wrist` → 손목 슬롯, `third_view` → base 슬롯), norm stats 계산, LoRA 학습 설정 (PLAN 7단계)
- 추론: 정책의 그리퍼 출력 (0~1) 을 0.5 기준으로 나눠 0 / 1150 → `/gripper/command`
