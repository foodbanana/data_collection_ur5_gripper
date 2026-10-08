# 드론 텔레옵 구조 (실물 조종기 → sim 드론)

작성: 2026-10-08 · 5단계 5-A 계획 시점 (`docs/PLAN.md` 5단계), 브랜치 `isaacsim_v6.1.0`

실물 Radiolink 조종기로 sim 의 PX4 드론을 조종하는 경로를 정리한다. **2 장은 지금 있는 구조 (코드에서 확인), 3 장부터는 계획이다 (아직 만들지 않음).**
드론 비행 자체 (추력 모델, PX4 연결·센서, PX4 내부 제어 식) 는 `docs/drone_flight.md`, 전체 수집 흐름은 `docs/architecture.md`.

---

## 1. 한눈에 보기

| 항목 | 내용 |
|------|------|
| 목적 | 사람이 실물 조종기로 sim 드론을 움직인다 (이동 드론 시연. 2단계에서 미구현으로 둔 `trajectory` 모드를 대신) |
| 입력 장치 | Radiolink 조종기 → Raspberry Pi Pico → USB 직렬 `/dev/ttyACM0` (있음, 2026-10-08 확인) |
| PX4 로 가는 길 | **새 포트 없음.** 지금 쓰는 명령 링크 (UDP 14540) 에 MAVLink `MANUAL_CONTROL` 을 실어 보낸다 (계획) |
| 비행 모드 | 리셋·이륙은 지금처럼 Offboard (자동), 그 뒤 조종기에 넘기면 PX4 수동 모드 (Position 예정) |
| 넘겨받는 곳 | sim 프로세스 안 `PX4Commander` (4 장: 밖에서 따로 붙으면 지금 코드와 충돌) |
| 녹화 | **드론 정보는 녹화하지 않는다** (2026-10-08 사용자 결정). 녹화 도구·변환·데이터셋 그대로 |
| 상태 | 조종기 → Pico → 직렬 출력까지 있음. 그 뒤 (읽는 쪽, sim 연결) 는 아직 |

---

## 2. 지금 드론이 움직이는 구조 (있음)

```text
 other terminal                      sim process (sim_ros2.py)                      child process
 +---------------+                  +--------------------------------+             +----------------------+
 | drone_cmd.py  |  UDP 14600       | PX4Commander  (drone_cmd.py)   |  UDP 14540  | PX4 SITL v1.16.0     |
 |  goto / hold  |  JSON, localhost |   CommandServer (JSON)         |<----------->|  API/offboard link   |
 |  land / kill  |----------------->|   position target 20 Hz        |  MAVLink    |  (PX4 side: 14580)   |
 |  status       |<-- reply --------|   mode / arm / kill requests   |             |                      |
 +---------------+                  |                                |             |  EKF2 -> position -> |
                                    | PX4Bridge  (px4_bridge.py)     |  TCP 4560   |  attitude -> rate -> |
 ROS 2 services                     |   virtual sensors, mocap  ---->|------------>|  motor command u     |
 /sim/drone_kill ------------------>|   motor command u         <----|<------------|                      |
 /sim/reset      ------------------>|   (lockstep, 120 Hz)           |  MAVLink    |                      |
                                    |          |                     |             +----------------------+
                                    |          v                     |
                                    | DroneFlight (drone_flight.py)  |
                                    |   w = 1000 u + 100 -> motor lag|
                                    |   -> thrust k w^2 -> PhysX     |
                                    +--------------------------------+
```

### 포트

| 포트 | 프로토콜 | 누가 여는가 | 내용 |
|------|----------|-------------|------|
| TCP 4560 | MAVLink | sim 이 서버 (`PX4Bridge`), PX4 가 접속 | **시뮬레이터 링크.** sim → PX4: 가상 센서 (`HIL_SENSOR`), 모션캡처 위치. PX4 → sim: 모터 명령 (`HIL_ACTUATOR_CONTROLS`). 모터 명령이 와야 sim 이 다음 물리 스텝으로 간다 (lockstep) |
| UDP 14540 | MAVLink | sim 의 `PX4Commander` 가 듣는다 (`udpin`). PX4 는 14580 에서 14540 으로 보낸다 | **명령 링크.** → PX4: 위치 목표 (`SET_POSITION_TARGET_LOCAL_NED`, 20 Hz), 모드 전환, arm, kill. ← PX4: 모드·arm 상태, 추정 위치·자세 |
| UDP 14600 | JSON (MAVLink 아님) | sim 의 `PX4Commander` 안 `CommandServer` 가 localhost 에서 듣는다 | **사람용 CLI 통로.** `drone_cmd.py goto\|hold\|land\|kill\|status` 가 `PX4Commander` 의 목표를 바꾼다. PX4 에 직접 가지 않는다 |

- 값은 `isaacsim/config/px4_sitl.yaml` (`sim_port`, `offboard_port`, `command_port`). 실제 포트 = 값 + `instance` (지금 0)
- 드론에게 "어디로 가라" 를 전하는 것은 **14540 하나**다. 4560 은 물리와 PX4 사이의 배선, 14600 은 14540 으로 나가는 목표를 바꾸는 리모컨
- `/sim/drone_kill` → `DroneFlight.release()` → `PX4Commander.kill()` → 14540. `/sim/reset` 도 `PX4Commander` 를 거친다
- PX4 SITL 은 이 밖에 GCS 링크 (UDP 18570) 등을 열지만 (`ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink`) 우리 코드는 쓰지 않는다

### `PX4Commander` 가 하는 일 (넘겨받기와 관련된 것)

| 동작 | 코드 | 넘겨받을 때의 문제 |
|------|------|--------------------|
| 위치 목표를 20 Hz (sim 시간) 로 계속 보냄 | `tick` → `_send_setpoint` | Offboard 는 목표가 끊기면 유지되지 않는다 → 넘긴 뒤에는 멈춰야 함 |
| 모드가 Offboard 가 아니면 1 s 마다 Offboard 를 다시 요청 | `tick` (`want_offboard`) | **밖에서 Position 모드로 바꿔도 곧 되돌린다** |
| hover 모드면 매 루프 사인파 목표로 덮어씀 | `DroneFlight.check` (`external` 이 아니면) | 넘긴 뒤에는 멈춰야 함 (CLI `goto` 때와 같은 `external` 표시) |
| `tick` 은 앱 루프마다 불린다 (30 Hz) | `DroneFlight.check` | 스틱 값도 이 주기로 나간다 |

---

## 3. 드론 텔레옵을 넣은 구조 (계획)

```text
 PILOT
 +-----------------------+
 | Radiolink transmitter |
 +-----------------------+
        | radio
        v
 +-----------------------+
 | receiver -> Pi Pico   |   EXISTS
 +-----------------------+
        | USB serial /dev/ttyACM0   (a device, not a network port)
        | "RC,0,0,1006,1002,995,996,200,200,200,1800"   fields 3..6 = 4 stick axes
        v
 +-----------------------+
 | RC input reader       |   NOT BUILT (5-A A-1): parse, check format / range / timeout
 +-----------------------+
        | stick values (inside the sim process, or a ROS 2 topic: to be decided)
        v
 +--------------------------------------+             +----------------------------+
 | sim process                          |             | PX4 SITL                   |
 |  PX4Commander                        |  UDP 14540  |                            |
 |   before hand-over (as now):         |------------>|  Offboard: position target |
 |     position target + Offboard       |             |                            |
 |   after hand-over (NEW):             |             |                            |
 |     MANUAL_CONTROL (4 sticks)        |------------>|  manual mode (Position):   |
 |     mode request Offboard -> manual  |             |   sticks -> velocity ->    |
 |     no position target, no Offboard  |             |   attitude -> rate -> u    |
 |     re-request                       |             |                            |
 |                                      |  TCP 4560   |                            |
 |  PX4Bridge (unchanged)               |<----------->|  sensors in, motors out    |
 |  DroneFlight (unchanged)             |             |                            |
 +--------------------------------------+             +----------------------------+
```

| 구간 | 통로 | 상태 |
|------|------|------|
| 조종기 → Pico | 무선 + 수신기 | 있음 |
| Pico → PC | USB 직렬 `/dev/ttyACM0`, 한 줄에 값 10 개 | 있음 (값의 뜻·범위·주기는 5-A A-0 에서 확인) |
| PC 에서 읽기 | 직렬 읽기 + 검사 | 만들 것. sim 프로세스 안에서 직접 읽을지, 별도 ROS 2 노드로 둘지는 정할 것 (PLAN 5-A "정할 것" 2 번) |
| sim → PX4 | **UDP 14540** (지금 명령 링크), MAVLink `MANUAL_CONTROL` | 만들 것 |
| PX4 → 드론 | TCP 4560 (모터 명령) → 추력 | 그대로 |

### PX4 소스에서 확인한 것 (v1.16.0, 돌려 본 것은 아님)

| 항목 | 내용 | 어디 |
|------|------|------|
| `MANUAL_CONTROL` 을 받는다 | 받은 값을 스틱 입력 (`manual_control_setpoint`) 으로 넘긴다 | `src/modules/mavlink/mavlink_receiver.cpp` `handle_message_manual_control` |
| 값 범위 | x (pitch)·y (roll)·r (yaw) = −1000 ~ 1000, **z (스로틀) = 0 ~ 1000 (500 이 중앙)** | 같은 함수 |
| SITL 은 조이스틱 입력만 받는다 | `COM_RC_IN_MODE 1` (Joystick only, RC 입력 처리와 그 검사를 끔) → `RC_CHANNELS_OVERRIDE` 가 아니라 `MANUAL_CONTROL` 이 맞다 | `ROMFS/px4fmu_common/init.d-posix/rcS`, `commander_params.c` |
| 스틱 입력이 끊기면 | `COM_RC_LOSS_T` 0.5 s 동안 새 값이 없으면 끊긴 것으로 보고 failsafe (`NAV_RCL_ACT` 기본 2 = Return) | `commander_params.c` |

---

## 4. 왜 `PX4Commander` 안에서 넘겨받나

별도 프로그램이 PX4 에 따로 붙어 스틱을 보내는 방식은 지금 코드와 맞지 않는다.

| 이유 | 내용 |
|------|------|
| 포트 | `PX4Commander` 가 UDP 14540 을 이미 열고 있다 (같은 포트를 다른 프로그램이 열 수 없음) |
| 모드 | `PX4Commander` 는 모드가 Offboard 가 아니면 1 s 마다 Offboard 를 다시 요청한다 → 밖에서 바꾼 모드가 되돌려진다 |
| 목표 | hover 모드의 사인파 목표, 위치 목표 송신이 계속된다 |
| 리셋 | `/sim/reset` 은 Offboard 로 재이륙한다 → "Offboard (자동) ↔ 조종기" 를 한 곳에서 바꿔야 리셋과 맞물린다 |
| 시각 | lockstep 이라 PX4 시계 = sim 시간. `PX4Commander.tick` 은 sim 시간으로 돈다 |

그래서 `PX4Commander` 에 "조종기에 넘김" 상태를 둔다: 위치 목표 송신·Offboard 재요청·hover 목표 갱신을 멈추고, 수동 모드를 요청하고, 스틱 값을 보낸다.
kill (`/sim/drone_kill`, 녹화 도구 `k`) 과 리셋은 지금처럼 `PX4Commander` 를 거치므로 그대로 쓴다.

---

## 5. 녹화할 때의 구조

```text
 PILOT                                        ARM OPERATOR
 Radiolink transmitter                        SpaceMouse Wireless
        | radio                                      | USB
        v                                            v
 receiver -> Pi Pico                          arm teleop node (5-B, NOT BUILT)
        | USB serial /dev/ttyACM0                    |
        v                                            | /joint_command
 RC input reader (5-A, NOT BUILT)                    | /gripper/command
        |                                            |
        | (drone only, NOT recorded)        +--------+--------+
        v                                   v                 v
 +-------------------------------------------------+   +---------------------------+
 | SIMULATOR  sim_ros2.py                          |   | RECORDER record_toggle.py |
 |   RC input -> PX4Commander -> PX4 SITL -> drone |   |   (unchanged)             |
 |   arm, gripper, protective stop, cameras x2     |   |                           |
 +-------------------------------------------------+   |   arm, gripper, cameras,  |
        | /joint_states, /gripper/*, /cam/*,           |   arm / gripper commands  |
        | /clock, /protective_stop                     |                           |
        +--------------------------------------------->|   key r -> /sim/reset     |
                                                       |   key k -> /sim/drone_kill|
                                                       +---------------------------+
                                                                   |
                                                                   v
                                                bag -> stage1 -> merge -> inspect (unchanged)
```

- **녹화하는 것은 지금과 같다: 로봇 팔·그리퍼·카메라 (와 팔·그리퍼 명령).** 조종기 입력도, 에피소드 중 드론 위치도 bag 에 넣지 않는다 (2026-10-08 사용자 결정)
  → `record_toggle.py`·stage1·병합·검수·데이터셋 스키마 그대로. 에피소드 중 드론이 어떻게 움직였는지는 카메라 영상에만 남는다
- 드론 조종 입력은 정책의 action 이 아니다 (정책은 팔·그리퍼만 움직인다)
- 조종기는 두 손을 쓴다 → 드론 조종과 팔 조작 (SpaceMouse) 을 한 사람이 같이 하기 어렵다 (PLAN 5-A "정할 것" 5 번)

### 에피소드 한 번의 순서 (계획)

| 순서 | 누가 | 하는 일 |
|------|------|---------|
| 1 | 녹화 담당 | `r` → `/sim/reset` (약 24 s): 팔 홈, 드론은 Offboard 로 자동 이륙·호버 |
| 2 | sim | 스틱이 중앙 근처면 조종기에 넘김 (아니면 넘기지 않고 알림) → 녹화 시작 |
| 3 | 조종사 | 드론을 작업 영역 안에서 움직이거나 세움 |
| 4 | 팔 조작자 | 접근해 잡음 |
| 5 | 녹화 담당 또는 조종사 | 드론 모터 정지 (`k`, 또는 조종기 스위치로 옮길지는 정할 것) |
| 6 | 녹화 담당 | `r` → 저장. 실패했거나 조종기 신호가 끊긴 에피소드는 `d` |

---

## 6. 아직 모르는 것·정할 것

| 항목 | 상태 |
|------|------|
| 직렬 줄의 값 10 개의 뜻 (스틱 4 축의 순서·방향, 1·2 번째, 7~10 번째), 범위, 주기, 신호가 끊길 때 출력 | 모름 → PLAN 5-A A-0 |
| 조종기 값 → `MANUAL_CONTROL` 환산 (특히 스로틀 0 ~ 1000, 500 중앙), dead zone | A-0 뒤에 |
| Offboard → 수동 모드로 넘길 때 드론이 튀지 않는지 | 돌려 보지 않음 |
| 어느 수동 모드를 쓸지 | Position 예정 (모션캡처로 위치 추정이 있음). 스로틀이 추력을 직접 정하는 모드는 hover thrust 문제와 겹친다 (`docs/drone_flight.md` 14-10 B) |
| 스틱 입력이 끊겼을 때의 PX4 동작 | 기본은 Return (실내 씬에 맞지 않음) → 파라미터를 정할 것 (그 자리 호버 등), 끊김은 터미널에 알림 |
| RTF < 1 (드론을 잡은 채 0.69) 에서의 조종 느낌 | 재지 않음. 스틱 값은 sim 시간 기준 30 Hz (앱 루프) 로 나간다 |
| 기하 제어기 드론 (`--flight geometric`) | `MANUAL_CONTROL` 방식은 PX4 전용 |
| 직렬 권한 | 지금 사용자 계정이 `dialout` 그룹에 없어 `sudo` 로만 읽힌다 → 그룹 추가 또는 udev 규칙 |

---

## 7. 관련 파일·문서

| 파일 | 내용 |
|------|------|
| `isaacsim/scripts/drone_cmd.py` | `PX4Commander` (명령 링크, 넘겨받기를 넣을 곳), `CommandServer`, CLI |
| `isaacsim/scripts/px4_bridge.py` | `PX4Launcher`, `PX4Bridge` (시뮬레이터 링크, lockstep) |
| `isaacsim/scripts/drone_flight.py` | `DroneFlight` (`arm`, `release`, `teleport`, `restart_px4`, `check` 에서 `PX4Commander.tick`) |
| `isaacsim/scripts/sim_reset.py` | `/sim/reset`, `/sim/drone_kill` |
| `isaacsim/config/px4_sitl.yaml` | 포트, 위치 정보 방식, PX4 파라미터 |
| `docs/drone_flight.md` | 13 장 PX4 SITL 연결, 14 장 PX4 내부 제어 식 |
| `docs/PLAN.md` 5단계 5-A | 작업 순서 (A-0 ~ A-4), 정할 것, 완료 기준 |
| `docs/architecture.md`, `docs/data_recording.md` | 전체 수집 흐름, 녹화 |
