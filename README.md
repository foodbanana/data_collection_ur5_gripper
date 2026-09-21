# data_collection_ur5_gripper

UR5(freedrive) + RH-P12-RN(A) 그리퍼 + RealSense 2대로 시연 데이터를 수집하고,
ros2 bag → **LeRobot 데이터셋(v2.1 / v3.0)** 으로 변환하는 스크립트 모음.

**목적**: openpi π0 / π0.5 를 LoRA 로 파인튜닝하기 위한 데이터 수집.
openpi 는 LeRobot **v2.1** 포맷만 호환 → 학습에는 v2.1 사용 (v3.0 은 함께 만들어만 둠).
전체 흐름은 아래 [전체 구조](#전체-구조) 다이어그램 참고.

기존 워크스페이스(`~/ur_freedrive_ws`, `~/rh_gripper_ros2_ws`, `~/realsense_ws`)는
**source 만** 한다. 이 폴더는 실행·녹화·변환 관리 전용.

수집 컨셉:
- 로봇팔 = freedrive (손으로 직접 움직임, 명령 토픽 없음)
- 그리퍼 = 키보드 teleop (열림/닫힘 이진 명령)
- 1 녹화 = 1 에피소드 = bag 1개 = 데이터셋 1개 (병합은 나중에)

---

## 전체 구조

### 1. 데이터 수집 파이프라인 (ros2bag)

```text
┌─────────────────────┐   ┌─────────────────────┐   ┌─────────────────────────┐
│   UR5 로봇팔        │   │  RH-P12-RN(A)       │   │  RealSense 카메라 2대   │
│  (freedrive 시연)   │   │  그리퍼 (1DOF)      │   │  (color 640x480@30)     │
│                     │   │                     │   │                         │
│  ur_robot_driver    │   │  rh_gripper_node    │   │  dual_camera.launch.py  │
│  ※ 현재 fake 대체   │   │  + rh_gripper_teleop│   │  (depth/IR/IMU 꺼짐)    │
└──────────┬──────────┘   └──────────┬──────────┘   └────────────┬────────────┘
           │                         │                           │
           ▼                         ▼                           ▼
 /joint_states            /gripper/joint_states    /d435i/d435i/color/image_raw
 (관절 6축 present)       (present, obs)           (외부 카메라 → third_view)
 sensor_msgs/JointState   /gripper/target          /d456/d456/color/image_raw
 ~125Hz                   (goal, action)           (손목 카메라 → head)
                          /gripper/command         sensor_msgs/Image
                          (내부 통신, 변환 X)      ~30Hz (각)
                          2×JointState 30Hz
           │                         │                           │
           └─────────────────────────┼───────────────────────────┘
                                     ▼
                  ┌─────────────────────────────────────┐
                  │ 6_record_bag.sh → record_toggle.py  │
                  │ 'r' 시작/종료 · 'd' 버리기          │
                  │ 'q' 종료 · SIGINT 안전 마무리       │
                  │ 1 record = 1 에피소드               │
                  └──────────────────┬──────────────────┘
                                     ▼
             ┌───────────────────────────────────────────────┐
             │ ros2 bag record (mcap, 압축 없음)             │
             │ 녹화 토픽 6개:                                │
             │   /joint_states                               │
             │   /gripper/joint_states          (obs)        │
             │   /gripper/target                (action)     │
             │   /gripper/command               (참고용)     │
             │   /d435i/d435i/color/image_raw                │
             │   /d456/d456/color/image_raw                  │
             └───────────────────────┬───────────────────────┘
                                     ▼
             bags/<YYYYMMDD_HHMMSS>_<작업명>/  (.mcap)
```

### 2. ros2bag → LeRobot 변환 → 검증

```text
   bags/<이름>/ (.mcap)
         │
         │  convert_ros2bag_lerobot.sh <bag> --task "..." --fps 25
         ▼
┌──────────────────────────────────────────────────────────────────────────┐
│ [1/3] Stage 1  lerobot_stage1_extract_bag.py   (시스템 파이썬 / ROS2)    │
│                                                                          │
│   · header.stamp 기준 25Hz 격자로 리샘플 (zero-order hold)               │
│   · state  = [팔6 관절(rad) + 그리퍼 present(raw 0~1150)]   → states.npy │
│   · action = [팔6 = states[i+1] , 그리퍼 = target[i](goal)] → actions.npy│
│     └ 팔은 다음프레임 유도(freedrive), 그리퍼는 실제 goal 명령           │
│   · 이미지 → head/(d456) · third_view/(d435i) PNG                        │
│   · /gripper/target 없으면 에러 중단 (fallback 없음)                     │
└──────────────────────────────────┬───────────────────────────────────────┘
                                   ▼
              bag_lerobot_intermediate/<이름>/
                states.npy (N,7)   actions.npy (N,7)   timestamps.npy
                head/*.png   third_view/*.png   meta.json
                                   │
               ┌───────────────────┴────────────────────┐
               ▼                                        ▼
┌────────────────────────────────┐       ┌────────────────────────────────┐
│ [2/3] Stage 2 → v3.0           │       │ [3/3] Stage 2 → v2.1           │
│ build_dataset_v30.py           │       │ build_dataset_v21.py           │
│ conda: lerobot_v1 (py3.12)     │       │ conda: lerobot_v2 (py3.10)     │
│ lerobot 0.6.1                  │       │ lerobot 0.3.3 (v2.1)           │
│                                │       │                                │
│ action_i = actions[i]          │       │ action_i = actions[i]          │
│ (Stage1이 완성한 걸 읽기만)    │       │ (동일)                         │
│ 이미지 → mp4(AV1) 인코딩       │       │ 이미지 → mp4(AV1) 인코딩       │
└──────────────┬─────────────────┘       └──────────────┬─────────────────┘
               ▼                                        ▼
lerobot_dataset_v30/foodbanana/...       lerobot_dataset_v21/foodbanana/...
(보관용)                                 (★ openpi 학습에 사용)
               │                                        │
               └───────────────────┬────────────────────┘
                                   ▼
┌──────────────────────────────────────────────────────────────────────────┐
│ 검증                                                                     │
│                                                                          │
│ · inspect_ur5_gripper_ros2_bag.py <bag>   (bag 단계: 토픽 개수/샘플)     │
│     출력 → inspect_out/                                                  │
│                                                                          │
│ · inspect_parquet.py <episode.parquet>    (변환 후: state/action 검수)   │
│     정상 기준:                                                           │
│       - action 그리퍼 = {0, 1150} 이진                                   │
│       - state 그리퍼 = 연속 (파지 시 중간값, 예 742)                     │
│       - 42억 이상치 0개, 음수 0개                                        │
│       - 파지 신호: action=1150인데 state가 742에서 멈춤                  │
└──────────────────────────────────────────────────────────────────────────┘
```

※ 다이어그램은 한글을 2칸 폭으로 계산해 정렬했다. 고정폭(모노스페이스) 글꼴에서 볼 것.
   (VS Code 미리보기·GitHub 에서 한글 폭이 정확히 2칸이 아닌 글꼴이면 오른쪽 테두리가 조금 어긋나 보일 수 있음)

---

## 핵심 설계 (바꾸기 전에 꼭 읽을 것)

| 항목 | 값 | 이유 |
|------|----|------|
| `observation.state` (7) | 팔 6관절(rad) + 그리퍼 **present**(raw 0~1150, 연속) | 실측 상태 |
| `action` 팔 (6) | `states[i+1]` 의 팔 | freedrive 라 명령 신호가 없어 다음 프레임에서 유도 |
| `action` 그리퍼 (1) | `/gripper/target` 의 `target[i]` (raw, 이진 {0, 1150}) | 실제로 내려진 goal 명령. **present 의 다음 프레임이 아님** |
| 그리퍼 값 | raw 0(열림) ~ 1150(닫힘) 그대로 | 정규화는 openpi norm stats 단계에서 |
| fps | 25 고정 | 각 토픽 `header.stamp` 기준 zero-order hold 리샘플 |
| 카메라 | d456 → `head`(손목), d435i → `third_view`(외부 고정) | |

**왜 그리퍼 action 을 target 으로 쓰나**: 물체를 잡으면 present 는 물체에 막혀 중간(예: 742)에서 멈추지만
goal 은 1150 이다. action 을 present 로 만들면 "742 유지"로 기록되어, 추론 때 파지력이 생기지 않는다
(전류기반 위치제어는 goal 과 present 의 차이로 힘을 낸다). target 을 쓰면 "꽉 닫아라(1150)"가 그대로 학습된다.

그리퍼 제어: Current-based Position Control(mode 5) + Goal Current 400mA(힘 상한).
그리퍼 구조·토픽 상세는 [1DOF_gripper_data_collection.md](1DOF_gripper_data_collection.md) 참고.

---

## 최초 1회

```bash
chmod +x *.sh
```

각 스크립트 상단의 사용자 환경 변수(인터페이스 이름, IP, 캘리브레이션 경로, 카메라 시리얼 등)를
자신의 환경에 맞게 확인/수정한다.

변환에는 conda 환경 2개가 필요하다 (`convert_ros2bag_lerobot.sh` 상단에서 이름 변경 가능):

| conda 환경 | 구성 | 용도 |
|-----------|------|------|
| `lerobot_v2` | python 3.10 + lerobot 0.3.3 | v2.1 데이터셋 |
| `lerobot_v1` | python 3.12 + lerobot 0.6.1 | v3.0 데이터셋 |

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
ros2 topic hz /d435i/d435i/color/image_raw
ros2 topic hz /d456/d456/color/image_raw
```

### [터미널 6] 에피소드 녹화 (r 토글)
```bash
./6_record_bag.sh            # 이름: <날짜시간>
./6_record_bag.sh pick_drone # 이름: <날짜시간>_pick_drone
```
실행하면 **대기 상태**(녹화 안 함). 한 번 켜두고 키로 에피소드를 반복 녹화한다.

| 키 | 동작 |
|----|------|
| `r` | 녹화 시작 → 다시 `r` : 녹화 종료 (r~r 구간 1개 = bag 1개 = 에피소드 1개) |
| `d` | 방금 저장한 에피소드 버리기(실패한 시연) → `y` 로 확인하면 `bags/_discarded/` 로 **이동**(삭제 아님) |
| `q` | 종료 (녹화 중이면 현재 bag 을 안전 종료한 뒤 종료) |

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

첫 에피소드에서는 녹화 시작 로그에 `Subscribed to topic '...'` 이 **6줄** 다 나오는지 확인한다.

---

## 녹화되는 토픽

| 토픽 | 타입 | 용도 |
|------|------|------|
| `/joint_states` | sensor_msgs/JointState | 팔 6관절 → `observation.state` 앞 6칸, 팔 action 유도 |
| `/gripper/joint_states` | sensor_msgs/JointState | 그리퍼 present, raw 0~1150 → `observation.state` 7번째 |
| `/gripper/target` | sensor_msgs/JointState | 그리퍼 goal, 30Hz 상시 발행 → **그리퍼 action** |
| `/gripper/command` | std_msgs/Float64 | teleop 이산 명령 (참고용, 변환에는 안 씀) |
| `/d435i/d435i/color/image_raw` | sensor_msgs/Image | 외부 고정 카메라 → `observation.images.third_view` |
| `/d456/d456/color/image_raw` | sensor_msgs/Image | 손목 카메라 → `observation.images.head` |

depth 는 이번 수집에서 제외(color 만). 압축 없음. 토픽 목록은 `record_toggle.py` 의 `TOPICS`.

---

## 2. 변환 — bag → LeRobot 데이터셋

```bash
./convert_ros2bag_lerobot.sh bags/<에피소드이름> --task "pick up the drone"
```

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--task "문장"` | `pick up the drone` | 언어 명령 |
| `--fps 숫자` | `25` | 리샘플 목표 fps (25 고정 사용) |
| `--namespace 이름` | `foodbanana` | repo_id 앞부분 |
| `--project 이름` | `ur5_gripper_drone` | repo_id 프로젝트명 |

bag 하나를 받아 3단계를 순차 실행한다 (`repo_id = <namespace>/<project>_<bag이름>`):

| 단계 | 환경 | 스크립트 | 출력 |
|------|------|----------|------|
| 1단계 | ROS 2 (시스템 python) | `lerobot_stage1_extract_bag.py` | `bag_lerobot_intermediate/<bag이름>/` |
| 2단계 v3.0 | conda `lerobot_v1` | `lerobot_stage2_build_dataset_v30.py` | `lerobot_dataset_v30/<repo_id>/` |
| 2단계 v2.1 | conda `lerobot_v2` | `lerobot_stage2_build_dataset_v21.py` | `lerobot_dataset_v21/<repo_id>/` |

2단계로 나눈 이유: `rosbag2_py` 는 ROS 2 시스템 파이썬에만, `lerobot` 은 conda 에만 있어 한 프로세스에서 같이 import 하면 충돌한다.

**1단계 출력(중간 파일)**
```
states.npy      (N, 7)  [팔6(rad) + 그리퍼 present(raw)]           → observation.state
actions.npy     (N, 7)  [팔6 = states[i+1] + 그리퍼 = target[i]]   → action
timestamps.npy  (N,)
head/000000.png ...        (d456)
third_view/000000.png ...  (d435i)
meta.json
```
- action 은 **1단계에서 완성**한다. 2단계(v21/v30)는 `actions.npy` 를 그대로 읽어 쓰기만 한다
- 마지막 프레임의 팔 action 은 `states[N-1]` 복제
- `/joint_states` 는 `name` 기준으로 관절 순서를 재정렬한다
  (shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3)

**의도적으로 에러로 중단하는 경우 (조용한 fallback 없음)**
- bag 에 `/gripper/target` 이 없음 → 옛 그리퍼 노드로 수집한 bag. 새 노드로 다시 수집할 것
- 중간 파일에 `actions.npy` 가 없음 → 옛 1단계 출력. 1단계를 다시 실행할 것
- 출력 데이터셋 폴더가 이미 있음 → 기존 것을 지우거나 `--namespace`/`--project` 변경

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

빠른 확인: `ros2 bag info bags/<이름>` → `/gripper/target` 메시지 수 ≈ 녹화 초 × 30

### 데이터셋(parquet) 검사
```bash
conda activate lerobot_v2
python3 inspect_parquet.py \
  lerobot_dataset_v21/<repo_id>/data/chunk-000/episode_000000.parquet [--rows 20] [--xlsx]
```
- 열 목록, `observation.state` / `action` 을 관절별 열(j1~j6, grip)로 펼친 미리보기
- 그리퍼 검수: action 그리퍼가 이진 {0, 1150} 인지, state 그리퍼 범위, **42억 이상치·음수 개수**
- 파지 신호: 닫기 명령(action=1150) 구간에서 state 가 어디서 멈췄는지
  (예: `action=1150 인데 state=742 에서 멈춤` = 물체를 잡음)
- `--xlsx` : 펼친 표를 엑셀로 저장

정상 기준: action 그리퍼 값 종류 `{0, 1150}` / state 그리퍼 0~1150 이내 / 42억 이상·음수 0개 /
`meta/info.json` 의 `codebase_version` 이 각각 `v2.1`, `v3.0`.

---

## 로봇팔 없이 그리퍼+카메라만 테스트할 때

- `0`, `1`, `2` (로봇팔 관련) 건너뛰기
- 대신 가짜 팔 관절을 발행:
  ```bash
  source /opt/ros/jazzy/setup.bash
  python3 fake_joint_states.py        # /joint_states 발행
  ```
- `3`, `4`, `5` 실행 후 `6` 으로 녹화 → 변환·검수까지 그대로 가능
- `fake_joint_states.py` 없이 녹화하면 `/joint_states` 가 비어 1단계 변환이 중단된다
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
  학습에 쓰지 말 것. `inspect_parquet.py` 의 "42억이상" 항목으로 확인 가능.
- **그리퍼 노드 수정 후 재빌드** — `~/rh_gripper_ros2_ws` 는 `--symlink-install` 로 빌드되어 있어
  파이썬 소스 수정은 재빌드 없이 노드 재시작만 하면 반영된다.
- **`Subscribed to topic` 이 6줄이 안 나옴** — 해당 장치 노드(터미널 1/3/5)가 안 떠 있음.
- **`bags/` 안에는 bag 만** — `_discarded/` 는 예외. 나중에 `bags/*` 를 일괄 변환하는 스크립트를 만들 때는 `_discarded` 를 제외할 것.
- **디스크 용량** — 비압축 카메라 2대라 에피소드당 수백 MB~GB. `bags/_discarded/`, `bag_lerobot_intermediate/`(png) 가 빨리 찬다.

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
| `5_cameras.sh` | 5 | 카메라 2대 (color) |
| `6_record_bag.sh` | 6 | 에피소드 녹화 (r 토글) — `record_toggle.py` 실행 래퍼 |
| `record_toggle.py` | 6 | 녹화 토글 본체 (토픽 목록 `TOPICS` 포함) |
| `fake_joint_states.py` | - | 로봇팔 없을 때 `/joint_states` 대체 발행 |
| `my_robot_calibration.yaml` | - | UR5 캘리브레이션 보관용 사본. `1_arm_driver.sh` 가 실제로 읽는 것은 `~/my_robot_calibration.yaml` (현재 내용 동일) |

**변환 / 검수**

| 파일 | 환경 | 역할 |
|------|------|------|
| `convert_ros2bag_lerobot.sh` | - | bag 1개 → 1단계 → v3.0 → v2.1 일괄 실행 |
| `lerobot_stage1_extract_bag.py` | ROS 2 | bag → 중간 파일 (`states.npy`, `actions.npy`, png) |
| `lerobot_stage2_build_dataset_v21.py` | conda `lerobot_v2` | 중간 파일 → LeRobot v2.1 (**학습용**) |
| `lerobot_stage2_build_dataset_v30.py` | conda `lerobot_v1` | 중간 파일 → LeRobot v3.0 |
| `inspect_ur5_gripper_ros2_bag.py` | ROS 2 | bag 내용 검사 → `inspect_out/` |
| `inspect_parquet.py` | conda | 데이터셋 parquet 검사 (그리퍼 검수·파지 신호) |

**폴더**

| 폴더 | 내용 |
|------|------|
| `bags/` | 녹화된 에피소드 bag (`_discarded/` = `d` 로 버린 에피소드) |
| `bag_lerobot_intermediate/` | 1단계 중간 파일 |
| `lerobot_dataset_v21/`, `lerobot_dataset_v30/` | 변환된 데이터셋 |
| `inspect_out/` | bag 검사 이미지 미리보기 |

**관련 문서 / 외부 코드**
- [1DOF_gripper_data_collection.md](1DOF_gripper_data_collection.md) — 그리퍼 노드·토픽·구조 메모
- `~/rh_gripper_ros2_ws/src/rh_p12_rn_ros2/` — `rh_gripper_node.py`, `rh_gripper_teleop.py`

---

## 다음 단계 (아직 안 만듦)

- 실제 UR5 연결 상태에서 전 구간 검증 (지금까지는 `fake_joint_states.py` + 실물 그리퍼·카메라로 검증)
- 에피소드 병합: 개별 v2.1 데이터셋 → 학습용 데이터셋 1개
- 에피소드 앞뒤 트리밍 (정지 구간 제거)
- openpi 쪽: data config, norm stats 계산, LoRA 학습 설정
- 추론: 정책의 그리퍼 출력(raw 0~1150) → `/gripper/command` 로 전달
