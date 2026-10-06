# 시각 (stamp) 동기화: sim 과 실물

작성: 2026-10-06 · 4단계 뒤 (`docs/PLAN.md` 4단계 "실물 stamp 동기화 문제 (D1)"), 브랜치 `isaacsim_v6.1.0`

데이터셋의 한 프레임은 "같은 순간의 이미지 + 관절값 + 명령" 이어야 한다. 이 문서는 그 "같은 순간" 을 무엇으로 맞추는지,
sim 에서는 어떻게 되어 있고 실물에서는 무엇이 확인됐고 무엇을 모르는지를 정리한다.

---

## 1. 한눈에 보기

| | sim (Isaac Sim) | 실물 |
|---|---|---|
| stamp 의 시계 | sim time 하나 (`/clock`) | PC 시각. 토픽마다 **다른 프로그램**이 찍는다 |
| 이미지 ↔ 관절값 | **같은 물리 스텝** (차이 0) | 모름 (팔이 없어 재지 못함) |
| 이미지 ↔ 그리퍼 | 같은 물리 스텝 (차이 0) | **−50 ~ 0 ms 사이** (이미지 stamp 가 같거나 이르다, 1 차 측정) |
| 확인 방법 | `check_ros2.py` 2·5 번 (자동, 매번) | `measure_stamp_offset.py` (그리퍼로 1 번 잼) |
| 상태 | 확인됨 | **카메라 ↔ 팔은 새 팔이 오면 측정** |

---

## 2. 변환이 시각을 쓰는 방법 (sim·실물 공통)

`lerobot_stage1_extract_bag.py` 는 bag 에 도착한 시각이 아니라 **각 메시지의 `header.stamp`** 로 프레임을 만든다.

```text
 25 Hz grid (every 40 ms)        t0          t1          t2
                                  |           |           |
 /joint_states   (120~125 Hz)  ..x.x.x.x.x.x.x.x.x.x.x.x.x.x.x..     pick the latest message with stamp <= grid time
 camera images   (30 Hz)       ....X.......X.......X.......X.....     (zero-order hold)
 /gripper/*      (30 Hz)       ......G.......G.......G.......G...
 /joint_command  (teleop)      ...c..c..c..c..c..c..c..c..c..c...     latest command with stamp <= grid time
```

| 규칙 | 내용 |
|------|------|
| 격자 | 상태 토픽이 모두 있는 구간을 25 Hz 로 나눈다 |
| 고르기 | 격자 시각 직전 (같음 포함) 의 최신 메시지 |
| 나이 검사 | 고른 메시지 (카메라·팔·그리퍼) 가 격자 시각보다 **66 ms 넘게 오래됐으면 에러** (`--max-age`). 메시지가 빠진 bag 에서 옛 이미지 + 새 관절값 프레임을 만들지 않는다 |
| 팔 명령 | `--arm-action command`: `/joint_command` 의 직전 최신 명령. **stamp 가 0 이면 에러**, 첫 명령 이전 구간은 뺀다 |
| header 없는 토픽 | `/protective_stop` (Bool) 은 bag 도착 시각을 `/clock` 으로 sim time 환산 |

따라서 **stamp 가 실제 사건의 시각과 어긋나 있으면, 그만큼 한 프레임 안에서 값들이 어긋난다.** 도착이 늦는 것 (전송 지연·재전송) 은 상관없다.

어긋남의 크기 = 속도 × 시각 차이.

| 시각 차이 | 팔 30 °/s | 팔 90 °/s | 그리퍼 (닫는 중 약 30 °/s) |
|-----------|-----------|-----------|----------------------------|
| 10 ms | 0.3° | 0.9° | 0.3° |
| 30 ms | 0.9° | 2.7° | 0.9° |
| 50 ms | 1.5° | 4.5° | 1.5° |

---

## 3. sim

### 3.1 누가 stamp 를 찍는가

| 토픽 | 찍는 곳 | 시각 |
|------|---------|------|
| `/clock`, `/joint_states` | `ros2_iface.ArmBridge` (물리 스텝 뒤 콜백) | 그 물리 스텝의 sim time (120 Hz) |
| `/gripper/joint_states`, `/gripper/target` | `ros2_iface.GripperBridge` | sim time 이 1/30 s 배수인 물리 스텝. 두 토픽이 같은 stamp |
| 카메라 `image_raw`, `camera_info` | OmniGraph Camera Helper (`useSystemTime` False) | 렌더한 물리 스텝의 sim time (30 Hz) |
| `/protective_stop` | `protective_stop.py` | header 없음 (Bool). 1/30 s 배수 시각에 발행 |
| `/joint_command` | **보내는 쪽** (텔레옵 노드, `replay_commands.py`) | 보낸 순간의 sim time (`/clock` 을 받아서 넣음) |

모두 `SimulationManager.get_simulation_time()` 하나에서 나온다. sim 이 실제보다 느리게 돌아도 (RTF < 1, 드론을 잡은 채 0.69) stamp 는 sim time 이라 **정합성은 변하지 않는다** (영향은 조작감뿐).

### 3.2 확인된 것

| 확인 | 결과 | 어디서 |
|------|------|--------|
| 카메라·그리퍼 stamp 가 `/joint_states` stamp 중 하나와 정확히 같다 | 안 맞는 것 0 개 | `check_ros2.py` 2 번 (매 실행) |
| 이미지가 stamp 시각의 물리 상태를 보여 준다 | 팔이 움직이기 시작한 stamp = 이미지가 바뀐 첫 stamp (0 프레임. 가끔 검사 민감도 때문에 1 프레임으로 나옴, PLAN 3-8) | `check_ros2.py` 5 번 |
| 토픽 주기 (stamp 기준) | `/joint_states` 120.00 Hz, 카메라·그리퍼 30.00 Hz, 끊김 0 | `check_ros2.py` 1 번, 녹화한 bag |
| 변환에서 고른 메시지 나이 | 카메라·그리퍼 최대 31.7 ms, 팔 8.3 ms (한계 66 ms) | stage1 `meta.json` |
| 카메라 ↔ 그리퍼 stamp 차이 (실물과 같은 도구로) | 닫는 방향 +5.6 ms, 여는 방향 −10.7 ms, 평균 −2.6 ms → **정답 0 을 ±10 ms 로 재현** (= 도구의 정확도) | `measure_stamp_offset.py` |

### 3.3 sim 에서 지켜야 할 것

- **텔레옵 노드는 `/joint_command` 에 sim time stamp 를 넣는다** (`use_sim_time`). PC 시각을 넣으면 팔 action 이 통째로 어긋나거나 "공통 시간 구간이 없음" 에러가 난다.
  4-1 에서 재생 스크립트가 stamp 를 안 넣어 전부 0 으로 녹화된 것을 찾아 고쳤다
- 명령은 **일정한 주기로** (50 Hz 이상). 재생 스크립트는 `/clock` 을 받아 보내는 방식이라 4~6 % 가 25 ms 늦는다 (학습에 안 쓰는 가짜 데이터).
  stamp 는 실제로 보낸 순간이라 데이터셋 action = sim 이 실제로 받은 명령이다
- sim 안에서 시간을 잴 때는 sim time. `flight.t` (드론 내부 시계) 는 기하 제어기를 다시 켤 때 0 으로 돌아간다 (4-1 에서 리셋 시간이 음수로 기록된 원인)
- **명령 → 움직임 지연은 stamp 문제가 아니라 실제 응답이다**: sim 은 받은 명령을 33 ms 에 걸쳐 보간하고 drive 가 따라간다 (계단 응답 50 % 도달 50~67 ms).
  데이터셋에서 action 이 state 보다 약 1 프레임 앞서는 것이 정상 (`docs/arm_command_interpolation.md`)

---

## 4. 실물

### 4.1 누가 stamp 를 찍는가

| 토픽 | 찍는 곳 | stamp 가 뜻하는 것 | 근거 |
|------|---------|--------------------|------|
| 카메라 `image_raw` | RealSense 드라이버 (realsense-ros 4.58.4) | **찍은 시각에 가깝다**: 카메라 하드웨어 시각을 PC 시각으로 환산 (`rgb_camera.global_time_enabled` = True) | 받은 시각 − stamp = wrist +36.9 ms, third_view +18.5 ms (stamp 가 도착보다 이르다). 환산의 정확도는 재지 않음 |
| `/gripper/joint_states`, `/gripper/target` | 그리퍼 노드 (`rh_gripper_node`) | **발행하는 순간** (직렬 통신으로 위치를 읽은 뒤). 발행 간격 32 ms, 가끔 48 ms | 받은 시각 − stamp = +0.6 ms |
| `/joint_states` | UR 드라이버 | 로봇 컨트롤러에서 값을 받은 PC 시각 (추정) | **확인 못 함** (팔 고장. 지금까지는 `fake_joint_states.py`) |
| `/joint_command` | 텔레옵 노드 | 보낸 PC 시각 | 텔레옵 노드를 아직 안 만듦 (5단계) |

모두 같은 PC 시계를 쓰지만, **실제 사건 (빛이 센서에 닿은 순간, 관절이 그 각도였던 순간) 과 stamp 사이의 지연이 토픽마다 다르다.**

### 4.2 측정한 것 (2026-10-06, 노트북, 카메라 ↔ 그리퍼)

도구 `measure_stamp_offset.py`: 그리퍼를 조금씩 움직였다 멈추며, 그리퍼 위치 곡선과 이미지 변화 곡선이 가장 잘 겹치는 시간 이동을 찾는다.
값 = 카메라 stamp − 그리퍼 stamp (양수 = 이미지 stamp 가 늦다).

| | 닫는 방향 | 여는 방향 | 평균 |
|---|---|---|---|
| sim (정답 0 ms) | +5.6 ms (±1.7, n 6) | −10.7 ms (±3.3, n 6) | −2.6 ms |
| **실물 wrist (D435I)** | **−51.1 ms** (±3.5, n 6) | **−0.2 ms** (±4.4, n 6) | −25.6 ms |
| 실물 third_view (D456) | −87 ms (±63) | +87 ms (±24) | 쓸 수 없음 |

- **결론: wrist 카메라 stamp − 그리퍼 stamp 는 −50 ~ 0 ms 사이** (이미지 stamp 가 같거나 이르다). 정확한 값은 정하지 못했다
- 같은 방향 안에서는 6 번이 ±4 ms 로 일치하는데 **닫을 때와 열 때가 51 ms 다르다**. 시각 차이라면 방향과 무관해야 하므로 시각이 아닌 것이 섞였다
  (그리퍼 기구가 방향에 따라 다르게 따라오는 것으로 추정). **원인 확인 못 함**
- third_view 는 손가락이 화면의 0.9 % 뿐이고 정지 때 흔들림이 커서 결과를 쓸 수 없다
- 이 측정의 한계: 손가락 사이에 물체가 있어 740 raw 에서 막힘 (280~570 raw 구간만 분석), 한 번만 잼, 도구 자체의 치우침 ±10 ms

같이 확인된 것 (녹화·변환, PLAN "실물 PC 확인"):

| 항목 | 결과 |
|------|------|
| 카메라 주기 | 30 Hz 로 발행 (`camera_info` 29.96 Hz, RELIABLE 녹화 29.99 Hz). `ros2 topic hz <image_raw>` 의 20 Hz 는 그 도구가 best effort 로 받아서 생긴 측정 문제 |
| 변환에서 고른 메시지 나이 | 카메라 33.3 / 33.4 ms, **그리퍼 48.2 ms**, (가짜) 팔 8.1 ms → 한계 66 ms 안. 그리퍼는 여유 18 ms |
| 변환 | 카메라·그리퍼·(가짜) 팔 stamp 가 같은 PC 시각 기준으로 겹쳐 에러 없이 738 프레임, 검수 5/5 |

### 4.3 모르는 것

| 항목 | 왜 모르는가 | 언제 |
|------|-------------|------|
| **카메라 ↔ 팔 (`/joint_states`) 의 stamp 차이** | 팔이 고장이라 진짜 관절값이 없다 | 새 팔이 오면. 가장 중요 (팔이 빠르게 움직인다) |
| 카메라 ↔ 그리퍼의 정확한 값, 방향에 따라 51 ms 다른 이유 | 그리퍼는 느리고 (닫는 데 2 s) 기구가 방향에 따라 다르게 따라와 정밀 측정에 맞지 않는다 | 그리퍼는 여기서 멈춤 (2026-10-06 결정: 한 프레임 안팎이고 느린 장치라 영향이 작다) |
| 텔레옵 명령 ↔ 팔 응답 (실물 servoJ 지연) | 텔레옵 노드·실물 팔 없음 | 5·9단계 |
| D456 의 stamp | third_view 측정이 쓸 수 없는 품질 | 팔 측정 때 같이 |
| 카메라 stamp 환산 (`global_time_enabled`) 의 정확도 | 직접 재지 않음 | 팔 측정으로 간접 확인 |

### 4.4 팔이 오면 재는 방법

원리는 같다: 갑자기 변하는 움직임을 만들고, 관절값이 변하는 stamp 와 이미지가 변하는 stamp 를 비교한다.

1. 카메라에 팔 (또는 그리퍼) 이 크게 보이게 두고, 관절 하나에 작은 왕복 움직임을 준다 (속도가 변하는 구간이 많을수록 잘 잰다)
2. `/joint_states` (125 Hz) 의 그 관절값 곡선과 이미지 변화 곡선이 가장 잘 겹치는 시간 이동을 찾는다 (`measure_stamp_offset.py` 의 방식. 지금은 그리퍼용이라
   움직임을 `/joint_command`, 위치를 `/joint_states` 로 바꿔야 한다)
3. 정방향·역방향을 따로 보고, 여러 번 반복해 평균의 오차를 본다. 팔은 관절값이 조밀하고 기구 유격이 작아 그리퍼보다 정확할 것으로 기대 (확인 전)

### 4.5 차이가 나오면 (대응 후보, 측정 뒤 결정)

| 후보 | 내용 |
|------|------|
| 그대로 두고 기록 | 차이가 한 프레임 (40 ms) 의 일부면 |
| stage1 에 토픽별 시각 보정 | 명시 옵션으로만 (`meta.json` 에 기록, 조용히 적용하지 않음) |
| 드라이버 설정 | 카메라 stamp 방식 (`global_time_enabled`) 등 |
| sim 에 같은 지연을 넣음 | sim 데이터와 실물 데이터를 섞을 때 (9단계) |

---

## 5. sim 과 실물의 차이가 뜻하는 것

- sim 데이터는 이미지와 관절값이 정확히 같은 순간이고, 실물 데이터는 수십 ms 까지 어긋날 수 있다. **sim 으로만 학습한 정책을 실물에 쓰거나 두 데이터를 섞을 때의 차이**가 된다 (9단계 sim2real)
- 추론 때도 같은 장치·같은 지연이면, 학습 데이터의 어긋남과 추론 입력의 어긋남이 같아 상쇄된다. 문제는 **수집 때와 추론 때의 지연이 다르거나, sim 과 실물이 다를 때**
- 그리퍼는 값이 느리게 변하고 action 이 이진 (0 / 1) 이라 영향이 작다. 팔은 action 이 연속이고 빠르게 움직이므로 팔 ↔ 카메라가 핵심

---

## 6. 명령

```bash
# sim: stamp 정합성 자동 검사 (2 번 같은 시계, 5 번 이미지 지연) → 9/9 PASS
python3 isaacsim/scripts/check_ros2.py

# 카메라 ↔ 그리퍼 stamp 차이 (sim·실물 공통. 카메라와 그리퍼 노드가 떠 있어야 하고, 그리퍼 teleop 은 끌 것)
#   준비: 그리퍼를 고정하고 손가락 사이를 비운다. 카메라에 손가락이 크게 보이게 (화면의 3 % 이상). 배경은 가만히
python3 measure_stamp_offset.py                                   # 약 2 분 반, 결과는 inspect_out/stamp_offset_<시각>/
python3 measure_stamp_offset.py --analyze <data.npz> --range 280 820   # 저장된 측정을 다시 분석

# 실물: 카메라 stamp 방식, 실제 주기
ros2 param get /cam/wrist rgb_camera.global_time_enabled
ros2 topic hz /cam/wrist/color/camera_info                        # image_raw 로 재면 실제보다 낮게 나온다

# 변환한 에피소드의 메시지 나이 (stage1 이 고른 메시지가 격자 시각보다 얼마나 오래됐는지)
python3 -c "import json; print(json.load(open('bag_lerobot_intermediate/<bag>/meta.json'))['max_age_ms'])"
```

---

## 7. 관련 문서

| 문서 | 내용 |
|------|------|
| `docs/PLAN.md` 4단계 "실물 stamp 동기화 문제 (D1)", "실물 PC 확인" | 측정 기록, 결정과 이유 |
| `docs/PLAN.md` 3-8, `isaacsim/scripts/check_ros2.py` | sim stamp 검사 |
| `docs/data_recording.md` | 녹화 토픽·주기, 녹화 뒤 변환 |
| `docs/sim_ros2_interface.md` | sim 토픽의 stamp·QoS |
| `docs/arm_command_interpolation.md` | 명령 → 움직임 지연 (보간 33 ms, 실물 servoJ) |
| `measure_stamp_offset.py` | 카메라 ↔ 그리퍼 stamp 차이 측정 도구 |
