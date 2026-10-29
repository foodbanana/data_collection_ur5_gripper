# 움직이는 베이스 구조 (공통: Husky A200 / Stewart 플랫폼 위 UR5)

PLAN 8단계 "움직이는 베이스 수집 연결" R2 ~ R5 까지 만든 것 (2026-10-29). 베이스별 세부는 `husky_base/husky_architecture.md`, `stewart_platform/stewart_architecture.md`.
근거: `isaacsim/scripts/sim_ros2_moving_base.py`, `moving_base_ros2.py`, `moving_base_scene.py`, `moving_base.py`, `docs/moving_base.md`.

상자 안에는 한글, ASCII 만 쓴다 (한글 = 2 칸으로 맞춤). 한글 고정폭 글꼴 (예: D2Coding) 에서 테두리가 맞게 보인다.
(*) = 고정 베이스 (`../fixed_base/fixed_base_architecture.md`) 와 달라지거나 새로 생긴 곳.

---

## [1] VR -> PC

고정 베이스와 같다 (Pico 4 Ultra -> XRoboToolkit -> xr_teleop_node.py -> /joint_command [60 Hz], /gripper/command, /teleop/request).
텔레옵 노드는 베이스가 움직이는지 모른다 -> 로봇 base 가 흔들리면 사람이 보고 보정 (운동 보상은 결과를 본 뒤 결정).

---

## [2] teleop 노드 <-> sim

```text
┌─ teleop node ─────┐  <-- (ROS)  /clock [120 Hz], /joint_states [120 Hz], /protective_stop [30 Hz]
│ xr_teleop_node.py │  <-- (ROS)  (*) /base/imu [120 Hz], /base/odom [30 Hz], /base/joint_states [30 Hz]
└───────────────────┘  --> (ROS)  /joint_command [60 Hz], /gripper/command

┌─ (*) Simulation: sim_ros2_moving_base.py --base husky|stewart --base-motion ... (30 Hz / 120 Hz) ─┐          ┌─ PhysX scene [120 Hz] ───────────┐
│ (1) UR5 로봇팔 - ArmBridge                 (고정 베이스와 같음)                                        │          │ 바닥 (*) 테이블 없음                 │
│ (2) RH 그리퍼 - GripperBridge              (고정 베이스와 같음)                                        │          │ (*) 베이스 강체 (kinematic)         │
│ (3) IRIS 드론 - DroneFlight + PX4          (*) 이륙 지점 = 바닥 (1.2, 0.15, 0)                        │          │       :  root_joint              │
│                                                                                                   │          │       :  body0 = 베이스 강체        │
│ (*) (4) 베이스 - KinematicBase (Husky) / StewartBase (Stewart)                                     │          │       :  excludeFromArticulation  │
│       PHYSICS_PRE_STEP: 궤적 식 q(t) -> 베이스 위치, 자세 (kinematic target)                           │    <-->  │ UR5 (*) floating articulation    │
│       리셋이 끝난 뒤부터 움직임 (sim 을 띄운 직후에는 서 있음)                                              │          │   (root = robot_mount)           │
│ (*) (5) BaseBridge (PHYSICS_POST_STEP)                                                            │          │ 드론, third view (world 고정)      │
│       같은 식, 같은 시각 t 로 /base/imu, /base/odom, /base/joint_states 계산                           │          └──────────────────────────────────┘
│       (--imu-compare 일 때만 Isaac IMU 센서 -> /base/imu_isaac, 검사용)                               │
│ (*) (6) MovingProtectiveStop                                                                      │
│       접촉 view 를 sim view 로 직접 만듦 (floating 로봇은 RigidPrim 금지)                               │
│       손가락이 닿으면 안 되는 대상 = 바닥, third view 받침대, 베이스 충돌체                                  │
│ (*) (7) MovingEpisodeReset: /sim/reset, /sim/drone_kill                                           │
└───────────────────────────────────────────────────────────────────────────────────────────────────┘
```

### (5) BaseBridge 식 (자세한 것 `docs/moving_base.md` 9.3)

```text
┌─ BaseBridge - 같은 궤적 식, 같은 시각 t ───────────────────────────────────────────────┐
│ 궤적 식 -> 베이스 강체 운동 (world): p, v, a, R, w, alpha                               │
│ IMU 위치 r (베이스 강체 좌표, imu_link)  ->  r_w = R r                                 │
│ a_imu = a + alpha x r_w + w x (w x r_w)          (접선 + 구심 가속도)                 │
│                                                                                    │
│ /base/imu  : 비력 f = R^T (a_imu - g),  g = (0, 0, -9.81)   (정지 = (0, 0, +9.81))   │
│              자이로 = R^T w,  자세 = R (world 기준 쿼터니언),  잡음 없음                  │
│ /base/odom : 위치 = R0^T (p - p0),  자세 = R0^T R     (원점 = 베이스 시작 자세)          │
│              twist = R^T v, R^T w   (child 좌표계)                                  │
│ /base/joint_states : Husky 바퀴 4 개 회전각 / Stewart 다리 6 개 길이                    │
│                                                                                    │
│ 베이스가 서 있을 때 (start 전, stop 뒤): 그 자리 자세, 속도, 가속도 0                       │
│ 모두 정답값 (베이스가 궤적대로 kinematic 으로 움직여서).                                    │
│ 실물처럼 구동하는 버전에서는 측정값 (Isaac IMU 센서, 바퀴 odometry, 다리 FK) 으로 바꿈         │
└────────────────────────────────────────────────────────────────────────────────────┘
```

### (7) 리셋 /sim/reset (약 24 s)

```text
┌─ MovingEpisodeReset (sim_reset.EpisodeReset 상속, 고정 베이스 파일은 안 고침) ────────────────────────┐
│ Step1. 팔, 그리퍼 명령 무시, 보호 정지 해제                                                            │
│ Step2. 드론 kill -> (*) 바닥 이륙 지점으로 순간이동 -> PX4 재시작                                       │
│ (*)    베이스 멈춤 -> 궤적 시작 자세 (t = 0) 로 순간이동 -> 새 seed                                     │
│ (*)    로봇 root (robot_mount) 도 시작 자세로 순간이동 (속도 0) -> 마운트 거리 확인 (2 mm 넘으면 실패)       │
│ Step3. 팔 시작 자세 (under_drone, --init-pose), 그리퍼 열림                                          │
│ Step4. PX4 이륙 -> 드론 기준 위치 (*) = 베이스 움직임 가운데의 로봇 base + drone_base (0.60, 0, 0.738)    │
│ (*)    베이스 움직임 시작 (흔들림은 리셋 끝부터 = 녹화 구간에만)                                           │
│ 응답 JSON: 고정 베이스와 같은 키 + (*) base (seed, 궤적 값, 마운트 거리, 시작 로봇 base)                   │
│                                  + (*) sim.base (종류, 프리셋, 마운트, IMU 위치, 잡음 없음)            │
│ /sim/drone_kill: 고정 베이스와 같음                                                                 │
└─────────────────────────────────────────────────────────────────────────────────────────────────┘
```

---

## [3] teleop 노드 <-> sim <-> drone <-> recorder

```text
┌─ Recorder: record_toggle.py (*) --base moving ──────────────────────────────────────────┐
│ Step1. /teleop/request 구독 (버튼 A / B / X)                                              │
│ Step2. 토픽 구독: 고정 베이스와 같은 것 + (*) /base/imu, /base/odom, /base/joint_states       │
│        (발행자 확인, /base/imu 수신 큐 120 Hz x 3 s)                                        │
│ Step3. A -> 녹화 시작 / 끝,  B -> /sim/reset,  X -> /sim/drone_kill                        │
│ episode.json: (*) "base": "moving", sim_reset.base                                      │
│ 실행: 6_record_bag.sh <작업명> --sim (*) --base moving                                    │
└─────────────────────────────────────────────────────────────────────────────────────────┘
          |
          v
      bag 파일 생성
          |  lerobot_stage1_extract_bag.py --arm-action command (*) --base-imu topic (*) --base-pose odom
          v
┌─ intermediate 파일 ─────────────────────────────────────────────────────────────────────────────┐
│ (*) base_imu.npy (N, 6) = /base/imu 를 25 Hz 구간 (t - 40 ms, t] 평균  -> observation.base_imu    │
│ (*) base_pose.npy (N, 7) = /base/odom 자세 [x, y, z, qw, qx, qy, qz]  (중간 파일만, 데이터셋 X)      │
└─────────────────────────────────────────────────────────────────────────────────────────────────┘
          |  lerobot_merge_episodes_v21.py (같음: base_imu 방식이 다른 에피소드는 안 섞음 -> const 와 topic 분리)
          v
┌─ lerobot v2.1 데이터셋 ───────┐
│ lerobot_dataset_v21/<repo id> │
└───────────────────────────────┘
          |  검수: inspect_dataset_v21.py --load (같음)
          v
      VLA finetuning

전체 확인: (*) make_fake_episodes_moving.py
  A. static 베이스: 실제로 잡는 에피소드 -> 병합, 검수
  B. 흔들리는 베이스: base_imu, base_pose 가 실제로 변하는지 (stage1 만)
```

---

## [4] Policy inference - 아직 만들지 않음 (계획)

```text
┌─ [A] [B] [C] [D] - 움직이는 베이스 ────────────────────────────────────────────────────────────────────────┐
│ [A] sim_ros2_moving_base.py : 고정 베이스의 (1) ~ (6) 토픽 + (*) /base/imu                                 │
│ [B] policy client           : (*) /base/imu 를 받아 stage1 과 같은 방식 (직전 40 ms 평균) 으로 관측에 붙임      │
│ [C] policy server           : (*) 새 openpi config (state 7 + base_imu 6)                               │
│                               주의: discrete_state_input=False 면 모델이 state 를 안 봄                    │
│                               -> IMU 를 붙여도 못 봄 (7단계 "본 학습 전에 정할 것" 결과를 보고 정함)              │
│ [D] operator                : 같음 (drone_cmd.py kill)                                                  │
│                                                                                                        │
│ 관측 -> 실행 지연 (약 0.18 s) 동안 베이스가 움직인 만큼 손끝이 밀림 (0.05 m/s 면 약 9 mm)                          │
└────────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

---

## 파일 한 장

```text
sim_ros2_moving_base.py --+-- moving_base_scene.py -- moving_base.py (Husky / Stewart 궤적)
                          +-- moving_base_ros2.py  (BaseBridge, MovingProtectiveStop, MovingEpisodeReset)
                          +-- 고정 베이스 부품 import, 상속만: ros2_iface, protective_stop, sim_reset, drone_scene.px4_takeoff
설정: scene_husky.yaml / scene_stewart.yaml, base_husky.yaml / base_stewart.yaml,
      ros2_iface.yaml base 섹션, scene_drone.yaml (읽기만)
검사: check_ros2_moving_base.py 22/22, make_fake_episodes_moving.py 4/4, moving_base_scene.py --check 6/6
```
