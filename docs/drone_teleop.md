# 드론 텔레옵 구조 (실물 조종기 → sim 드론)

작성: 2026-10-08 · 5단계 5-A 계획 시점 (`docs/PLAN.md` 5단계), 브랜치 `isaacsim_v6.1.0`

실물 Radiolink 조종기로 sim 의 PX4 드론을 조종하는 경로를 정리한다. 2 장은 조종기를 넣기 전부터 있던 구조, 3 장부터가 조종기 경로다.
**만들었고, 실물 조종기로 sim 드론이 움직이는 것까지 확인했다 (2026-10-08, 8 장).**
드론 비행 자체 (추력 모델, PX4 연결·센서, PX4 내부 제어 식) 는 `docs/drone_flight.md`, 전체 수집 흐름은 `docs/architecture.md`.

---

## 1. 한눈에 보기

| 항목 | 내용 |
|------|------|
| 목적 | 사람이 실물 조종기로 sim 드론을 움직인다 (이동 드론 시연. 2단계에서 미구현으로 둔 `trajectory` 모드를 대신) |
| 입력 장치 | Radiolink 조종기 → Raspberry Pi Pico → USB 직렬 `/dev/ttyACM0` (있음, 2026-10-08 확인) |
| PX4 로 가는 길 | **새 포트 없음.** 지금 쓰는 명령 링크 (UDP 14540) 에 MAVLink `MANUAL_CONTROL` 을 실어 보낸다 |
| 비행 모드 | 리셋·이륙은 지금처럼 Offboard (자동), 그 뒤 조종기에 넘기면 PX4 수동 모드 (Position = `POSCTL`) |
| 실행 | `sim_ros2.py --drone-rc on` (기본 off). 설정 `isaacsim/config/rc_input.yaml` |
| 넘겨받는 곳 | sim 프로세스 안 `PX4Commander` (4 장: 밖에서 따로 붙으면 지금 코드와 충돌) |
| 녹화 | **드론 정보는 녹화하지 않는다** (2026-10-08 사용자 결정). 녹화 도구·변환·데이터셋 그대로 |
| 상태 | sim 쪽까지 만듦: `check_rc.py` 6/6, 가짜 직렬 입력으로 `sim_ros2.py` 10/10. **실물 조종기로도 확인** (GUI, 축 방향 맞음) |

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

## 3. 드론 텔레옵을 넣은 구조

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
 | RC input reader       |   rc_input.py (inside the sim process): parse, check format / range / timeout
 +-----------------------+
        | stick values pitch / roll / throttle / yaw, -1..1   (drone_rc.py decides hand-over / take-back)
        v
 +--------------------------------------+             +----------------------------+
 | sim process                          |             | PX4 SITL                   |
 |  PX4Commander                        |  UDP 14540  |                            |
 |   before hand-over (as now):         |------------>|  Offboard: position target |
 |     position target + Offboard       |             |                            |
 |   after hand-over:                   |             |                            |
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
| Pico → PC | USB 직렬 `/dev/ttyACM0`, 한 줄에 값 10 개, 142.9 Hz | 있음 (값의 뜻은 아래 "직렬 줄") |
| PC 에서 읽기 | sim 프로세스가 직접 읽는다 (`rc_input.RcInput`, 표준 라이브러리 termios. ROS 노드를 따로 두지 않음) | 만듦 |
| sim → PX4 | **UDP 14540** (지금 명령 링크), MAVLink `MANUAL_CONTROL` (`PX4Commander.handover`) | 만듦 |
| PX4 → 드론 | TCP 4560 (모터 명령) → 추력 | 그대로 |

### 직렬 줄 (A-0 에서 잰 것, 2026-10-08. `isaacsim/scripts/rc_input.py`, PLAN 5-A "A-0 결과")

`RC,<값 1>,<값 2>,<값 3> … <값 10>` = 상태 표시 2 개 + 채널 8 개, 142.9 Hz.

| 값 번호 | 뜻 | 범위 (가운데) | 방향 | `MANUAL_CONTROL` 로 |
|---------|-----|---------------|------|----------------------|
| 1, 2 | 신호 상태 (정상 0, 0. 조종기를 끄면 1) | 0 / 1 | — | 0 이 아니면 신호 끊김 |
| 3 | 오른쪽 스틱 좌우 | 200 ~ 1800 (1003) | 오른쪽 = 커짐 | y (roll, 오른쪽 +) |
| 4 | 오른쪽 스틱 상하 | 204 ~ 1800 (999) | **위 = 작아짐** | x (pitch, 앞 +) → 부호를 뒤집는다 |
| 5 | 왼쪽 스틱 상하 | 217 ~ 1800 (993) | 위 = 커짐 | z (스로틀, 0 ~ 1000, 가운데 500) |
| 6 | 왼쪽 스틱 좌우 | 200 ~ 1800 (994) | 오른쪽 = 커짐 | r (yaw, 오른쪽 +) |
| 7 ~ 10 | 안 바뀜 (스위치 없음) | — | — | 안 씀 |

- 조종기를 꺼도 줄은 계속 온다 (값이 `1,1,1000,1000,0,1000,…` 으로 바뀜) → 끊김은 값 1·2 로 판정, 줄이 안 오는 것 (Pico·USB) 은 시간 제한으로 따로
- 축 방향은 기록을 시간 순으로 맞춰 정했고, 실물 조종기로 드론을 움직여 맞는 것을 확인했다 (8 장)

### PX4 소스에서 확인한 것 (v1.16.0. 돌려 본 결과는 8 장)

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

그래서 `PX4Commander` 에 "조종기에 넘김" 상태를 두었다 (`set_sticks`, `handover`, `takeback`): 위치 목표 송신·Offboard 재요청·hover 목표 갱신을 멈추고, 수동 모드를 요청하고, 스틱 값을 보낸다.
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
 RC input reader (5-A, inside sim)                   | /gripper/command
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

### 에피소드 한 번의 순서

| 순서 | 누가 | 하는 일 |
|------|------|---------|
| 1 | 녹화 담당 | `r` → `/sim/reset` (약 24 s): 팔 홈, 드론은 Offboard 로 자동 이륙·호버 |
| 2 | sim | 리셋이 끝나면 녹화 시작. 스틱이 가운데 근처면 조종기에 넘김 (아니면 넘기지 않고 알림. 가운데에 두면 넘어감) |
| 3 | 조종사 | 드론을 작업 영역 안에서 움직이거나 세움 |
| 4 | 팔 조작자 | 접근해 잡음 |
| 5 | 녹화 담당 | 드론 모터 정지 (`k`. 조종기에 스위치가 없다) |
| 6 | 녹화 담당 | `r` → 저장. 실패했거나 조종기 신호가 끊긴 에피소드는 `d` |

---

## 6. 아직 모르는 것·정할 것

| 항목 | 상태 |
|------|------|
| 직렬 줄의 값 10 개의 뜻, 범위, 주기, 신호가 끊길 때 출력 | 쟀다 (3 장 "직렬 줄"). 축 방향은 드론을 움직여 다시 확인 |
| 조종기 값 → `MANUAL_CONTROL` 환산, dead zone, 축별 최대 속도 | 설정 파일에 (A-2) |
| Offboard → 수동 모드로 넘길 때 드론이 튀지 않는지 | 튀지 않는다 (0.2 ~ 0.3 s 뒤 POSCTL, 그 뒤 10 s 최대 30 mm, 8 장) |
| 어느 수동 모드를 쓸지 | Position (`POSCTL`, 스틱 = 속도, 가운데 = 그 자리). 스로틀이 추력을 직접 정하는 모드는 hover thrust 문제와 겹친다 (`docs/drone_flight.md` 14-10 B) |
| 조종기 신호가 끊겼을 때 | sim 이 Offboard 로 되돌려 그 자리에 세우고 터미널에 알린다 (PX4 failsafe 에 맡기지 않음, 8 장) |
| 스틱 끝 속도, 수직 속도 한계 | 임시값 (`rc_input.yaml` `px4_params`). 실물 조종기로 날려 보고 정한다 |
| 축 방향 | 실물 조종기로 확인함 (8 장). `rc_input.yaml` `axes` 그대로 |
| GUI 에서 드론을 보는 시점 | 드론 뒤에서 보지 않으면 스틱의 앞·뒤·좌·우가 화면과 어긋난다 → 실물 조종기로 날려 보고 정한다 |
| RTF < 1 (드론을 잡은 채 0.69) 에서의 조종 느낌 | 재지 않음. 스틱 값은 sim 시간 기준 30 Hz (앱 루프) 로 나간다 |
| 기하 제어기 드론 (`--flight geometric`) | `MANUAL_CONTROL` 방식은 PX4 전용 |
| 직렬 권한 | 지금 사용자 계정이 `dialout` 그룹에 없어 `sudo` 로만 읽힌다 → 그룹 추가 또는 udev 규칙 |

---

## 7. 관련 파일·문서

| 파일 | 내용 |
|------|------|
| `isaacsim/scripts/rc_input.py`, `isaacsim/config/rc_input.yaml` | 직렬 읽기·검사·스틱 환산 (`RcReader`, `RcMap`, `RcInput`), A-0 입력 확인 도구. 축 배치·범위·PX4 파라미터 |
| `isaacsim/scripts/drone_rc.py` | `DroneRc`: 넘기기·되돌리기 판단 (루프마다) |
| `isaacsim/scripts/check_rc.py` | 넘기기 시험 (조종기 없이, 스틱은 스크립트가 만듦) → 6/6 |
| `isaacsim/scripts/drone_cmd.py` | `PX4Commander` (명령 링크, `set_sticks`·`handover`·`takeback`), `CommandServer`, CLI |
| `isaacsim/scripts/px4_bridge.py` | `PX4Launcher`, `PX4Bridge` (시뮬레이터 링크, lockstep) |
| `isaacsim/scripts/drone_flight.py` | `DroneFlight` (`arm`, `release`, `teleport`, `restart_px4`, `check` 에서 `PX4Commander.tick`) |
| `isaacsim/scripts/sim_reset.py` | `/sim/reset`, `/sim/drone_kill` |
| `isaacsim/config/px4_sitl.yaml` | 포트, 위치 정보 방식, PX4 파라미터 |
| `docs/drone_flight.md` | 13 장 PX4 SITL 연결, 14 장 PX4 내부 제어 식 |
| `docs/PLAN.md` 5단계 5-A | 작업 순서 (A-0 ~ A-4), 정할 것, 완료 기준 |
| `docs/architecture.md`, `docs/data_recording.md` | 전체 수집 흐름, 녹화 |

---

## 8. 돌려 본 결과 (2026-10-08, PLAN 5-A "A-1 ~ A-3 결과")

**넘기기·되돌리기 규칙 (`drone_rc.DroneRc`, 루프마다)**

| 상태 | 조건 | 동작 |
|------|------|------|
| 자동 (Offboard 호버) | 이륙·리셋이 끝남 + 조종기 신호 있음 + 네 축이 가운데 근처 (`handover_center` 0.1) | 조종기에 넘김 (`handover`) |
| 자동 | 스틱이 가운데가 아님 / 신호 없음 | 넘기지 않고 터미널에 알림 (5 s 마다) |
| 조종기 | 신호 끊김 (상태 값 ≠ 0), 값이 범위 밖, 줄이 0.2 s 안 옴 | 그 자리 Offboard 호버로 되돌림 (`takeback`) + 알림. 신호가 돌아오고 가운데면 다시 넘김 |
| 어느 쪽이든 | `/sim/reset` | PX4 를 새로 띄우므로 자동으로 돌아감 → 재이륙 뒤 위 조건으로 다시 넘김 |
| 어느 쪽이든 | `/sim/drone_kill` | 모터 정지. 다음 리셋까지 아무것도 안 함 |

**`check_rc.py` (PX4 드론만, 스틱은 스크립트가 만든 값) → 6/6**

| 시험 | 결과 |
|------|------|
| 넘기기 | 0.2 ~ 0.3 s 뒤 POSCTL, 그 뒤 10 s 동안 넘긴 위치에서 최대 30 mm |
| 스틱 +0.6 을 2 s | 앞 229 mm (최고 0.41 m/s, 기울기 11.7°), 오른쪽 278 mm (0.45 m/s), 위 362 mm (0.30 m/s), yaw 오른쪽 38.8°. 놓으면 멈춤 |
| 되돌리기 | 1.0 s 뒤 Offboard, 그 자리에서 16 mm |
| `MANUAL_CONTROL` 송신을 멈춤 | 1.0 s 뒤 PX4 failsafe → AUTO.LAND. 다시 보내고 되돌리면 Offboard 로 돌아옴 |
| 수동 모드에서 kill | disarm 33 ms |

**`sim_ros2.py --drone-rc on` (가짜 직렬 장치)** → 자동 넘김, 스틱으로 이동 (오른쪽 스틱 위 = world +x 로 0.20 m), 신호 끊김 → 0.2 s 안에 되돌림 → 복귀 뒤 다시 넘김,
스틱을 민 채 리셋 → 넘기지 않음 → 가운데에 두면 넘김, `/sim/drone_kill` (10/10)

- `MANUAL_CONTROL` 은 한 번 보내기 시작하면 끊지 않는다 (넘기지 않은 동안은 가운데 값). 끊으면 1 s 뒤 PX4 가 착륙한다
- 조종기를 쓸 때의 PX4 파라미터 (임시값): 스틱 끝 수평 0.5 m/s, yaw 60 °/s, 수직 0.5 m/s. **수직 한계는 Offboard 이륙에도 걸려 리셋이 약 24 → 29 s**
- `--drone-rc` 없이는 지금까지와 같다: `check_px4.py` 5/5, `check_ros2.py` 9/9, `make_fake_episodes.py` 3/3 (첫 실행 2/3 은 조종기와 무관한 재생 스크립트 문제, PLAN 5-A 회귀 항목)
- sim 이 직렬 장치를 읽으려면 계정이 `dialout` 그룹이어야 한다 (`sudo usermod -aG dialout <계정>` 뒤 다시 로그인, 또는 그 터미널에서 `newgrp dialout`. sim 을 sudo 로 띄우지 않는다)

**실물 조종기 (2026-10-08, 사용자, GUI)**

| 스틱 | 드론 | 확인 |
|------|------|------|
| 왼쪽 위 / 아래 | 상승 / 하강 (가운데 = 높이 유지) | 맞음 |
| 왼쪽 좌 / 우 | 왼쪽 / 오른쪽으로 회전 (yaw) | 맞음 |
| 오른쪽 위 / 아래 / 좌 / 우 | 드론 기준 앞 / 뒤 / 왼쪽 / 오른쪽 (yaw 를 돌리면 "앞" 도 같이 돈다) | 맞음 |

- 스틱 = 속도 (PX4 Position 모드): 밀고 있으면 그 속도로 가고 놓으면 멈춘다
- 오른쪽 스틱 (수평 이동) 을 쓰는 동안에는 드론을 제자리에 세워 두기가 조금 어렵다 (사용자 관찰). 그대로 둔다: 모으려는 것은 날고 있는 드론이다. 원인은 재지 않았다
- 조종기를 껐다 켰을 때, 드론을 잡은 채의 조종 느낌은 실물 조종기로 아직 보지 않았다 (가짜 입력으로만)
- **RTF** (잡지 않은 상태): 조종기 때문에 느려지지 않는다. 헤드리스 자동 호버 0.96, 헤드리스 수동 모드 0.97 (프로펠러 회전 끔) / 0.94 (켬),
  **GUI 에서 실물 조종기로 수동 비행 0.89 ~ 0.91** (프로펠러 회전 켬 = GUI 기본. 실제보다 약 10 % 느림). 올리려면 `--prop-spin off` (PLAN 5-A 표)
