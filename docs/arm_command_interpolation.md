# 팔 명령 보간 (sim `/joint_command` 계단 문제)

작성: 2026-10-05 · 3단계 3-7 측정 중 발견 (`docs/PLAN.md` 3단계), 브랜치 `isaacsim_v6.1.0`

sim 이 텔레옵 팔 명령(`/joint_command`)을 받아 관절 목표로 넣는 방식에서 생긴 문제와, 다른 프로젝트들이 같은 문제를
어떻게 다루는지, 우리가 고른 방법을 정리한다.

---

## 1. 한눈에 보기

| 항목 | 내용 |
|------|------|
| 문제 | sim 이 `/joint_command` 를 **앱 루프(30 Hz)마다 한 번** 관절 목표로 바로 넣는다 → 목표가 33 ms 마다 계단처럼 바뀌고, 단단한 팔 drive 가 계단마다 **관절 최대 속도(180 °/s)로 뛰었다 멈춘다** |
| 영향 | 팔이 30 Hz 로 덜컥거림 → 손목 카메라 영상·`/joint_states` 가 실물과 다른 모양. 보호 정지 기준(관절 속도·위치 오차)이 정상 동작에서도 걸림 |
| 다른 곳 | 텔레옵 명령을 계단식으로 그대로 넣는 곳은 없음. 실물 UR 은 **servoJ** (lookahead 0.03~0.2 s) 가, Franka 계열은 **1 kHz 보간기**가 부드럽게 만든다 |
| 해결 | **선형 보간 (deoxys 방식)**: 루프마다 받은 최신 명령을 새 목표로, 현재 목표에서 새 목표까지 물리 스텝(120 Hz)마다 직선 이동. 보간 시간은 설정값 (기본 = 루프 한 주기 33 ms) |
| 대가 | 명령 → 움직임 지연 약 +33 ms. 실물 servoJ 기본값(lookahead 0.1 s)보다는 짧다 |

---

## 2. 무엇이 문제였나

### 2.1 sim 의 명령 처리 순서 (보간 전)

```text
앱 루프 1 회 (33 ms sim, 카메라 주기 30 Hz) = 렌더 1 번 + 물리 4 스텝 (120 Hz)
  ① ROS 구독 처리 (ros2_iface.spin): 쌓인 /joint_command 중 마지막 것을 drive 목표로 바로 넣음
  ② 물리 4 스텝 동안 목표는 그대로
```

텔레옵이 60 Hz 로 부드러운 궤적을 보내도, sim 안의 관절 목표는 33 ms 마다 한 번씩 계단처럼 바뀐다.
팔 drive 는 높은 stiffness 로 정했으므로 (1-5, ω ≥ 70 rad/s) 목표가 바뀌면 관절 최대 속도(drive `max_velocity` = π rad/s = 180 °/s)로
따라갔다가 멈추기를 반복한다.

### 2.2 측정 (2026-10-05, 보호 정지 `measure` 모드, 기하 제어기 드론, 헤드리스)

텔레옵 대신 명령 CSV 를 `replay_commands.py` 로 재생했다.

| 명령 | 명령 자체의 관절 최대 속도 | 측정 관절 속도 최대 | 목표 − 측정 최대 |
|------|---------------------------|---------------------|------------------|
| 팔 흔들기 1 배 (wrist_3 ±0.6 rad, 0.4 Hz) | 86 °/s | **180 °/s** (wrist_3) | 2.8° |
| 팔 흔들기 2 배 | 173 °/s | 180 °/s | **7.3°** |
| grasp_demo 6 케이스 재생 (60 Hz 명령) | 최대 180 °/s (첫 관절 보간이 wrist_3 을 약 3.3 rad 돌림) | 180 °/s | 4.1~4.2° |

86 °/s 명령에서도 관절 속도가 180 °/s 까지 튄 것은, 86 °/s × 33 ms = 2.9° 계단을 최대 속도로 뛰기 때문이다.

---

## 3. 다른 곳은 어떻게 하나

| 프로젝트 / 장치 | 텔레옵·정책 명령 주기 | 명령 → 관절 목표 처리 |
|---|---|---|
| **UR 공식 ROS 2 드라이버** (`forward_position_controller`) | 스트리밍 | URScript **servoJ** 로 넘김. `servoj_lookahead_time` (0.03~0.2 s, 기본 0.1) 으로 목표를 미리 보고 부드럽게 따라감, `servoj_gain` (100~3000, 기본 300). 문서: "lookahead 가 클수록 부드럽지만 명령과 움직임 사이 지연이 커짐" |
| **GELLO** (UR5 텔레옵, 데이터 수집) | 100 Hz | servoJ (`velocity 0.5, acceleration 0.5, dt 1/500, lookahead_time 0.2, gain 100`). 가장 부드러운 쪽 설정 |
| **deoxys** (Franka, UT Austin) | 정책 20 Hz | 1 kHz 제어 루프에서 **선형 보간** (`traj_interpolator_type: LINEAR_JOINT_POSITION`, `time_fraction: 0.3`): 새 목표가 오면 명령 주기 × 0.3 동안 현재 목표에서 새 목표로 직선 (`q = q_start + t·(q_goal − q_start)`, t 는 0~1 로 clamp) |
| **DROID** (Franka, 대규모 데이터셋) | 15 Hz | Polymetis 관절 임피던스 제어기가 웨이포인트를 추종 |
| **franka-stack** (GELLO/VR + π0.5) | 20 Hz 기록 | 1 kHz 실시간 루프가 따로 있음 |
| **Isaac Sim 공식 ROS 2 튜토리얼** | - | OmniGraph Articulation Controller 가 렌더 프레임마다 목표를 바로 넣음. **보간 없음 = 우리 보간 전과 같은 방식.** 튜토리얼은 물리·렌더가 같은 60 Hz 라 계단이 덜 드러남 |
| **Isaac Lab** | 정책 주기 (decimation) | 같은 목표를 decimation 스텝 동안 유지. 문서: "decimation 이 크면 부드럽지만 지연" |

정리: 실물 로봇은 명령 주기와 제어 주기 사이에 **반드시 부드럽게 만드는 층**이 있다 (UR = servoJ, Franka = 1 kHz 보간기).
보간 전 sim 은 실물 UR 보다 덜컥거리고 (계단), 반응은 더 빨랐다 (servoJ 지연 없음). 둘 다 실물과 다르다.

---

## 4. 우리가 고른 방법: 선형 보간

deoxys 의 선형 보간과 같은 방식을 물리 스텝(120 Hz)에서 한다.

```text
루프마다 (ros2_iface.spin): 받은 최신 /joint_command q_cmd 를 새 목표로
    q_start ← 지금 관절 목표,  q_goal ← q_cmd,  t_start ← 지금 sim 시각
물리 스텝마다 (PHYSICS_PRE_STEP):
    u = clamp((t − t_start) / T, 0, 1),  관절 목표 = q_start + u · (q_goal − q_start)
T = command_interp_time (ros2_iface.yaml arm, 기본 = 1 / loop_hz = 33 ms)
```

- 명령이 일정한 속도로 움직이면 관절 목표도 같은 속도로 움직인다 (계단 없음). 대가는 T 만큼의 지연
- 보간 시간 T 는 설정값이다. 실물 UR 의 servoJ lookahead 를 정하면 (5단계) 반응 지연이 비슷해지도록 다시 맞출 수 있다
- 보호 정지가 걸리면 보간을 멈추고 그 순간 관절각에 고정한다
- 데이터셋의 팔 action 은 녹화 쪽이 `/joint_command` 를 그대로 기록하므로 (CLAUDE.md 스키마) 보간과 무관하다.
  `observation.state` (측정 관절각) 만 부드러워진다

---

## 5. 결과 (보간 후, 2026-10-05)

보간을 넣으면서 같이 찾아 고친 문제가 두 가지 있다. 셋 다 고친 뒤의 결과가 5.3 표다.

### 5.1 sim 이 루프마다 명령을 1 개만 처리하던 문제 (고침)

`ros2_iface.spin` 은 rclpy `spin_once` 를 반복하다 처리된 콜백이 없으면 멈췄다. 그런데 rclpy `spin_once` 는 메시지가 쌓여 있어도
**콜백 1 번 → 빈 호출 1 번을 번갈아** 한다 (시스템 python3 로 확인: 4 개를 보내고 10 번 부르면 `1, 0, 1, 0, 1, 0, 1, 0, 0, 0`).
그래서 루프마다 1 개만 처리했고, 60 Hz 명령의 절반은 수신 큐(깊이 10)에서 밀려 버려졌다 (1200 개 중 609 개 적용).
→ **빈 호출이 두 번 연속일 때 멈추게** 고침. 이후 60 Hz 재생에서 루프마다 2 개씩 받음 (600 루프 모두).

3-3 에서 본 "992 개 중 603 개 적용" 도 같은 원인이었다 (그때는 시험 클라이언트 탓으로 봤으나 sim 쪽 문제).

### 5.2 PhysX 가 보고하는 관절 속도가 실제보다 작음 (고침)

보간 후 `/joint_states` 의 관절각으로 직접 계산한 속도는 명령과 같았지만 (wrist_3 86.4 °/s = 명령 86.4 °/s),
PhysX 가 보고하는 관절 속도(`get_dof_velocities`)는 모양은 같고 (상관 1.00) 크기가 작았다.

| 관절 | 실제 (관절각 미분) 최대 | PhysX 보고 최대 | PhysX / 실제 |
|------|--------------------------|-----------------|--------------|
| shoulder_pan | 43.2 °/s | 36.8 °/s | 0.85 |
| wrist_1 | 37.8 °/s | 25.4 °/s | 0.61 |
| wrist_3 | 86.4 °/s | 37.3 °/s | 0.42 |
| 멈춘 관절 (shoulder_lift·elbow·wrist_2) | 0.1~0.4 °/s | 1.3~3.1 °/s | — |

끝쪽(가벼운 링크)일수록 더 작다. 드론에서 이미 본 것(그리퍼 접촉 때 PhysX 각속도가 실제와 다름, CLAUDE.md)과 같은 종류다.
→ `/joint_states` 의 velocity 와 보호 정지의 관절 속도를 **관절각 차분 (q − q_이전) / dt** 로 바꿈. 실물 UR 이 보고하는 속도는 실제 속도다.

### 5.3 보간 후 측정 (보호 정지 `measure` 모드, 기하 제어기 드론, 헤드리스, `replay_commands.py` 재생)

| 명령 | 명령 최대 관절 속도 | 측정 관절 속도 최대 (보간 전 → 후) | 목표 − 측정 최대 (보간 전 → 후) |
|------|--------------------|------------------------------------|----------------------------------|
| 팔 흔들기 1 배 | 86 °/s | 180 → **86.4 °/s** (= 명령) | 2.8 → **0.7°** |
| 팔 흔들기 2 배 | 173 °/s | 180 → 180 °/s (drive 최대 속도에 걸림) | 7.3 → **2.9°** |
| grasp_demo 6 케이스 | 최대 180 °/s | 180 → 160~165 °/s | 4.1~4.2 → **0.3~0.4°** |

- 팔 흔들기 2 배는 명령 자체가 drive 최대 속도(180 °/s)에 가깝다. 이 재생에서는 시험 클라이언트가 명령을 고르게 보내지 못해
  (루프마다 1·2·3 개) 따라잡는 구간이 생겼다
- 드론 파지 신호는 그대로: present 282~294, target 1150 (success·offset·drop), 빈손 1133 (empty·miss)

### 5.4 지연

보간 시간 33 ms 만큼 명령 → 움직임 지연이 늘어난다 (보간 전 계단 명령 50% 도달 약 30 ms). 3-8 검사에서 계단 응답으로 다시 잰다.

---

## 6. 참고

- UR ROS 2 Driver — ROS interface (`servoj_gain`, `servoj_lookahead_time`): https://docs.ros.org/en/humble/p/ur_robot_driver/doc/ROS_INTERFACE.html
- UR ROS 2 Driver — Controllers (`forward_position_controller` → servoj): https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/usage/controllers.html
- GELLO software (`gello/robots/ur.py`, `gello/env.py`): https://github.com/wuphilipp/gello_software · 논문 https://arxiv.org/abs/2309.13037
- deoxys_control (`config/joint-impedance-controller.yml`, `linear_joint_position_traj_interpolator.h`): https://github.com/UT-Austin-RPL/deoxys_control
- DROID: https://arxiv.org/pdf/2403.12945
- franka-stack: https://github.com/Loule0-0/franka-stack
- Isaac Sim ROS 2 Joint Control 튜토리얼: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_manipulation.html
- Isaac Lab Teleoperation and Imitation Learning: https://isaac-sim.github.io/IsaacLab/main/source/overview/imitation-learning/teleop_imitation.html
