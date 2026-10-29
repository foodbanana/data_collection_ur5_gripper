# 고정 베이스 구조 (손그림 -> ASCII)

손그림 10 장 (`Screenshot from 2026-10-29 19-34-26.png` ~ `20-26-20.png`) 을 옮긴 것 (2026-10-29). 그림의 [1] ~ [4] 순서.
글씨가 작아 읽기 어려운 곳은 코드·문서 값으로 채웠다 (`docs/architecture.md`, `docs/sim_ros2_interface.md`, `docs/policy_client.md`).
상자 안에는 한글·ASCII 만 쓴다 (한글 = 2 칸으로 맞춤. 원 숫자·삼각 화살표·그리스 문자는 글꼴마다 폭이 달라 상자가 깨짐).

---

## [1] VR -> PC

```text
┌─ Pico 4 Ultra ────────┐                               ┌─ PC (xrobotoolkit service) ─┐
│ XRoboToolkit app 실행 │    wifi, TCP 63901, 90 Hz     │ XRoboToolkit                │
└───────────────────────┘   ------------------------->  └─────────────────────────────┘
                             버튼 + 조이스틱 입력
                                            |  TCP 127.0.0.1
                                            v
┌─────┐                      ┌─ teleop node ─────────────────────────────────────────────────────────────────┐
│ Sim │                      │ xr_teleop_node.py  (XRoboToolkit SDK 활용)                                    │
└─────┘                      │   C++ SDK libPXREARobotSDK.so -> Python binding xrobotoolkit_sdk              │
                             │                                                                               │
ros2 --------------------->  │ 가져오는 값 (60 Hz 발행)                                                      │
/joint_states, /clock,       │   - 컨트롤러 pose (x, y, z, qx, qy, qz, qw) = 위치 3 + 쿼터니언 4             │
/protective_stop             │   - trigger / grip (0 ~ 1)                                                    │
                             │   - 버튼 A / B / X / Y                                                        │
                             │         |                                                                     │
                             │         v  One Euro filter (LPF)                                              │
                             │         v  URDF IK                                                            │
                             │         v  safety limit 기준                                                  │
                             │              (max joint speed, 관절 오차 한계, max joint rad,                 │
                             │               TCP < 0.09 m = 책상 충돌 등)                                    │
                             │         v                                                                     │
                             │   joint command          trigger (> 0.6 닫기 / < 0.4 열기) -> gripper command │
                             └───────────────────────────────────────────────────────────────────────────────┘
                              |
                              v
ros2 토픽 (출력)
  출력 1. /joint_command [60 Hz] (= q_cmd)   <- 로봇팔 각도 (TCP 위치)
  출력 2. /gripper/command (0 or 1150)       <- 그리퍼 열기/닫기 (trigger)
  출력 3. /teleop/request                    <- 녹화 시 입력 (button)
```

---

## [2] teleop 노드 <-> sim (로봇팔·그리퍼)

```text
┌─ teleop node ─────┐  <-- (ROS)  (1) /clock [120 Hz] (sim time)
│ xr_teleop_node.py │  <-- (ROS)  (2) /protective_stop [30 Hz]
└───────────────────┘  <-- (ROS)  (3) /joint_states [120 Hz]

                       --> (ROS)  (1) /joint_command [60 Hz]
                                      joint_state [rad], stamp = /clock (sim time)
                                      (teleop 노드가 joint cmd 보낼 때 시각 표시 = /clock)
                       --> (ROS)  (2) /gripper/command (raw 0 or 1150, float64)
                                      -> 0, 1150 binary 값, 에피소드 결과 기록

┌─ Simulation: sim_ros2.py - app loop 30 Hz (카메라, 렌더) / sim loop 120 Hz (PhysX)  [터미널 1] ─┐          ┌─ PhysX scene [120 Hz] (8.3 ms) - 물리 계산 ───────────────────┐
│ ┌─ (1) UR5 로봇팔 - ArmBridge ──────────────────────────────────────────────────────┐           │          │ PhysX scene 구성 (테이블, 드론, 그리퍼, 카메라 ...)           │
│ │ Step1. /joint_command 30 Hz 수신 (명령 2 개씩 수신, latest cmd 이용)              │           │          │   로봇 USD = floating base -> scene 에서 articulation root 를 │
│ │ Step2. 선형 보간 (command_interp_time): q_start / q_goal / t_start                │           │          │   최상위 prim 으로 옮겨 고정                                  │
│ │        120 Hz 물리 step 마다 u = (t - t_start) / T   (T = 1/30 s)                 │           │          │                                                               │
│ │        q_target = q_start + u * (q_goal - q_start)  -> PD drive 목표              │           │          │ (1) PD drive 토크  tau = Kp (q_k - q) - Kd q_dot              │
│ │        (보간 없으면 step input -> 팔이 덜컥거림. 실물 UR = servoJ 1 kHz 보간)     │           │          │     (최대 tau = 150 N*m 제한)                                 │
│ │        -> 지연 33 ms                                                              │           │          │ (2) contact force 계산                                        │
│ │ Step3. q_k (drive 목표) [120 Hz]  --> PhysX                                       │           │          │ (3) link pose 계산                                            │
│ │ Step4. /joint_states [120 Hz] (ROS) <-- PhysX : position, velocity, effort (토크) │           │    <-->  │ (4) self-collision 계산                                       │
│ │ Step5. 범위 넘으면 ProtectiveStop                                                 │           │          │ (5) app loop [30 Hz]: PhysX 4 step 마다                       │
│ │        (접촉 150 N, 관절 오차 5 deg, 관절 속도 200 deg/s)                         │           │          │     GUI 렌더, camera image [30 Hz], ROS 메시지                │
│ └───────────────────────────────────────────────────────────────────────────────────┘           │          │                                                               │
│                                                                                                 │          │ 그리퍼: tau = Kp (q_k - q) + Kd q_dot                         │
│ ┌─ (2) RH 그리퍼 - GripperBridge ────────────────────────────────────────────────────────┐      │          │   토크 제한 2.28 N*m, Kp = 45.6, Kd = 1.3758                  │
│ │ Step1. /gripper/command [30 Hz] 수신: 0 / 1150 raw -> 0 rad / 1.1351 rad (latch)       │      │          │   근거: 실물 peak current 3.33 A, max grip force 170 N        │
│ │ Step2. 매 120 Hz 마다 target 이 0.516 rad/s 씩 이동 (GripperProfile, drive_gains.yaml) │      │          │         실험 전류 400 mA -> 0.4 / 3.33 x 170 = 약 20 N        │
│ │        0.516 x 1/120 = 0.0043 rad 씩 목표 이동                                         │      │          │         (손가락 하나) -> 20 N x 2 x 0.057 m = 2.28 N*m        │
│ │ Step3. q_k (drive 목표)  --> PhysX                                                     │      │          └───────────────────────────────────────────────────────────────┘
│ │ Step4. /gripper/joint_states [30 Hz] (ROS) <-- PhysX : pos (raw 0 ~ 1150)              │      │
│ └────────────────────────────────────────────────────────────────────────────────────────┘      │
└─────────────────────────────────────────────────────────────────────────────────────────────────┘

(메모) 30 Hz 를 올릴 수 있나?  (1) app loop 60 Hz  (2) PhysX scene 120 Hz 에서 ROS 읽기 -> 별로
```

---

## [2] teleop 노드 <-> sim (드론·PX4)

```text
┌─ Simulation (sim_ros2.py) ─────────────────────────────────────────────────────────┐                               ┌─ PhysX scene ─────────────────────────┐
│ ┌─ (3) IRIS 드론 - 축소 버전 (Iris 0.75 배, 0.8 kg) ─────────────────────────────┐ │                               │ 드론 body 위치 / 자세 / 속도,         │
│ │ ┌─ 드론 원하는 위치 명령 - PX4Commander [30 Hz] ─────────────────────────┐     │ │                               │ 로터 4 개 위치 계산                   │
│ │ │ 목표 위치를 PX4 SITL 에 계속 보낸다                                    │     │ │                               │                                       │
│ │ │ option 1) drone target: --drone-targets                                │     │ │                               │ 프로펠러 회전 (보이기용, --prop-spin) │
│ │ │           (scene_drone.yaml drone_targets_base 25 점 shuffle)          │     │ │                               │ drone_iris.yaml 에서 enabled: false   │
│ │ │ option 2) 사인파 hover: goto (reference pos p0 + sin 파 (x, y, z))     │     │ │                               │ 로 끌 수 있음                         │
│ │ │           drone_cmd.py goto x y z --yaw-deg -> p0 위치를 옮김          │     │ │                               └───────────────────────────────────────┘
│ │ │ option 3) 기하 제어기 (--flight geometric): PX4 안 씀, position PID    │     │ │
│ │ │ option 4) 조종기 velocity command (drone_rc.py: 0.5 m/s, 60 deg/s yaw) │     │ │
│ │ └────────────────────────────────────────────────────────────────────────┘     │ │
│ │                                                                                │ │    추력, 토크 -->
│ │ ┌─ 모터 추력 계산 - DroneFlight [120 Hz] (매 PhysX step 전) ─────────────────┐ │ │
│ │ │ Step1. 모터 속도 w_cmd = 1000 x u + 100 [rad/s]                            │ │ │    <-- body 위치, 자세, 속도
│ │ │ Step2. 모터 명령 지연 (1st-order motor lag, Gazebo 모터 모델)              │ │ │
│ │ │        w_rotor <- w_rotor + (w_cmd - w_rotor)(1 - exp(-dt / tau))          │ │ │
│ │ │ Step3. 추력 F = k w^2   (k = 8.55e-6, Pegasus IRIS 기본값)                 │ │ │
│ │ │        yaw 토크 tau_z = sum( c w_i^2 d_i )  (c = 1e-6, d = [-1, -1, 1, 1]) │ │ │
│ │ │        공기저항 F_d = -R D R^T v  (Pegasus simulator IRIS 기본값)          │ │ │
│ │ │ Step4. 추력, 토크를 PhysX 에 넣음                                          │ │ │
│ │ └────────────────────────────────────────────────────────────────────────────┘ │ │
│ │                                                                                │ │
│ │ ┌─ 에피소드 리셋 (/sim/reset) ───────────┐                                     │ │
│ │ │ Step1. drone kill (PX4 kill)           │                                     │ │
│ │ │ Step2. teleport                        │                                     │ │
│ │ │ Step3. PX4 재시작                      │                                     │ │
│ │ │ Step4. 이륙 후 target 위치로 드론 이동 │                                     │ │
│ │ │ Step5. 흔들림 with sine wave           │                                     │ │
│ │ │ motor off (PX4 kill): /sim/drone_kill  │                                     │ │
│ │ └────────────────────────────────────────┘                                     │ │
│ └────────────────────────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────────────────────┘
      |  ^                                    |  ^
      |  |  TCP 4560                          |  |  UDP 14540
      |  |  sim -> PX4:                       |  |  sim -> PX4: 목표 위치 (x, y, z), yaw,
      |  |   HIL_SENSOR [120 Hz]              |  |    heartbeat, 모드 offboard [30 Hz]
      |  |   (IMU, 자이로, 기압계)            |  |  PX4 -> sim: 추정 위치 LOCAL_POSITION_NED
      |  |   VISION_POSITION_ESTIMATE         |  |    [30 Hz], 추정 yaw [50 Hz], 상태
      |  |   [100 Hz] (위치 + yaw)            |  |    (비행 중 / 이륙 / 착륙 등)
      |  |  PX4 -> sim:                       |  |
      |  |   HIL_ACTUATOR_CONTROLS            |  |
      |  |   u 4 개 [120 Hz], arm 상태        |  |
      v  |                                    v  |
┌─ PX4 SITL - 드론 비행 제어 (실제 firmware v1.16.0) ─┐
│ 위치: ~/PX4-Autopilot                               │
│ EKF2: pose 추정 from IMU, mocap                     │
│ 리셋: /sim/reset                                    │
└─────────────────────────────────────────────────────┘

센서: 지금은 거의 완벽한 yaw, 위치 가정 (motion capture 가정, 잡음 1 mm, 0.3 deg, 지연 20 ms)
      -> px4_sitl.yaml 에서 설정 가능

option 4 (조종기): 실물 radiolink 조종기 --조이스틱--> raspberry pico (간단한 신호 변환 python)
  --USB 직렬 /dev/ttyACM0, {0, 0, ...} 10 비트 정수--> sim (drone_rc.py: rc 명령으로 변환 -> PX4)
  이때 PX4 mode = 수동 (POSCTL, offboard 아님)
```

---

## [3] teleop 노드 <-> sim <-> drone <-> recorder

```text
┌─ teleop node ─────┐  <-- /clock, /protective_stop, /joint_states  ┌─ Simulation: sim_ros2.py (30 Hz / 120 Hz) ─┐          ┌─ PhysX [120 Hz] ─┐
│ xr_teleop_node.py │  --> /joint_command [60 Hz]                   │ (1) UR5 로봇팔                             │    <-->  │ 팔, 그리퍼, 드론 │
└───────────────────┘  --> /gripper/command                         │ (2) RH 그리퍼                              │          │ 물리 계산        │
                                                                    │ (3) IRIS 드론 (축소 버전)                  │          └──────────────────┘
                                                                    │     에피소드 리셋 Step1 ~ 5                │
                                                                    └────────────────────────────────────────────┘

   teleop node --/teleop/request, /joint_command, /gripper/command--> Recorder
   Simulation  --상태 토픽-->  Recorder,    Recorder --/sim/reset, /sim/drone_kill-->  Simulation

┌─ Recorder: record_toggle.py ──────────────────────────────────────┐
│ Step1. teleop node 의 /teleop/request 구독 (버튼 A / B / X)       │
│ Step2. 토픽 구독 (joint command, gripper command, joint_states,   │
│        카메라 2 대, protective_stop, clock)                       │
│ Step3. 버튼 A -> ros2 bag record 시작 / 끝                        │
│        버튼 B -> /sim/reset (service)                             │
│        버튼 X -> /sim/drone_kill (service) = motor off (PX4 kill) │
│ 실행: 6_record_bag.sh drone_grasp_<작업명> --sim                  │
└───────────────────────────────────────────────────────────────────┘
          |
          v
      bag 파일 생성
          | lerobot_stage1_extract_bag.py (25 Hz 격자로 맞춤)
          v
┌─ intermediate 파일 ─────────────────┐
│ bag_lerobot_intermediate/<bag 이름> │
└─────────────────────────────────────┘
          | lerobot_merge_episodes_v21.py (conda lerobot_v2, lerobot 0.3.3, 매니페스트)
          v
┌─ lerobot v2.1 데이터셋 ───────┐
│ lerobot_dataset_v21/<repo id> │
└───────────────────────────────┘
          | 검수: inspect_dataset_v21.py --load -> pass / fail
          v
      VLA finetuning

option 4: 실물 radiolink 조종기 -> raspberry pico -> sim (드론 조종, [2] 드론 그림)
```

---

## [4] Policy inference - [A, B, C, D]

그림 `20-26-20.png` ([B] client 확대), `20-26-04.png` ([C] server + [D] operator 확대) 반영. 값은 `docs/policy_client.md`, `docs/policy_inference_desktop.md` 와 맞춤.

```text
┌─ [B] Policy client - policy_client_node.py (sim 과 policy server 사이 통역사 / 실행자) ─┐  (sim_time)                             ┌─ [A] Simulation - sim_ros2.py ─┐
│ policy_client_node.py  (~/openpi, ~/openpi_client/venv)                                 │  <-- (1) /joint_states [120 Hz]         │ (1) UR5 로봇팔                 │
│   policy_client.yaml 에 서버 주소, prompt 등 설정                                       │  <-- (2) /gripper/joint_states [30 Hz]  │ (2) RH 그리퍼                  │
│                                                                                         │  <-- (3) /cam/wrist                     │ (3) IRIS 드론                  │
│ Step1. policy_client.yaml 에서 설정 읽기                                                │  <-- (4) /cam/third_view                │ PhysX scene [120 Hz]           │
│ Step2. policy server 와 websocket 연결                                                  │  <-- (5) /clock                         │ ([2], [3] 과 같음)             │
│ Step3. ROS node 생성 (구독 6, 발행 2) / spin, sim 에 관절 명령 보내기                   │  <-- (6) /protective_stop               └────────────────────────────────┘
│        thread 3 개                                                                      │
│          #1 spin      -> ROS 콜백 처리 (버퍼에 쌓기, clock 갱신)                        │  --> (1) /joint_command [60 Hz]
│          #2 main      -> run_episode [60 Hz]                                            │  --> (2) /gripper/command [on change]
│          #3 inference -> observe() -> policy infer (약 5 Hz, 0.185 s, websocket)        │          (raw 0 or 1150, float64
│                                                                                         │           -> 0, 1150 binary 값,
│ Step4. 관측 짝짓기 - state, obs 의 buffer 에서                                          │           에피소드 결과 기록)
│          js, grip 버퍼 / img (wrist), img (third_view) 버퍼                             │  --> (3) /sim/reset
│        이미지 2 개 + 관절 2 개 (4 개 신호) time-stamp 비교                              │
│          t_obs = min(이미지 1, 이미지 2 stamp)                                          │
│          각 버퍼에서 t_obs 이하 가장 최근 값                                            │
│          t_obs 와 66.6 ms (카메라 2 frame) 넘게 차이 -> 에러                            │
│        -> 짝지은 obs 를 server 로                                                       │
│                                                                                         │
│ Step5. episode 실행 - sim time 기준 [60 Hz] 로 명령 보냄                                │
│        (teleop 명령 60 Hz 와 맞춤)                                                      │
│   (1) reset -> 0.2 s 대기 -> 처음 q_cmd = 지금 자세                                     │
│   (2) 첫 추론 기다림 (t_obs -> policy server -> action chunk (10, 7))                   │
│   (3) 60 Hz loop 반복 (1 loop = 16.7 ms)                                                │
│       1) chunk 10 칸의 시각을 t_obs 기준으로 보고 지금 칸 위치                          │
│          x = (지금 - t_obs) / 40 ms 를 실수로 (예: x = 4.87)                            │
│       2) joint_command: 칸 floor(x) 와 그다음 칸 사이 선형 보간                         │
│          (25 Hz 칸 2 개를 60 Hz 로 나눔)                                                │
│          gripper: 보간 안 함, 0.5 기준 -> 바뀔 때만 /gripper/command                    │
│       3) x >= 4 (replan_steps, policy_client.yaml) 이면 다음 추론 요청                  │
│          4 칸 x 0.04 s = 0.16 s ~ 추론 0.185 s 가 끝나는 때와 비슷                      │
│          -> 새 chunk 는 약 4 칸 위치에 도착 (추론 동안 지나간 칸은 건너뜀)              │
└─────────────────────────────────────────────────────────────────────────────────────────┘
  |  ^
  |  |  websocket 0.0.0.0:8000 (지금은 localhost 사용)                         추론 = 약 4 ~ 5 Hz (측정 0.185 s)
  |  |                                                                         action chunk (10, 7) = 10 칸 x (팔 6 + 그리퍼 1)
  |  |  --> Obs (state 7 개, image 2 장, prompt 1 개)                            10 칸 = 25 Hz 에서 0.4 s 미래까지 (action_horizon 10, pi0_config)
  |  |      (1) observation/state (7,) = 팔 6 (rad) + 그리퍼 (present / 1150)  (1) actions (10, 7) chunk
  |  |      (2) observation/base_image  = third_view (480, 640, 3)             (2) policy timing - 모델 계산 시간 (flow matching)
  |  |      (3) observation/wrist_image = wrist (480, 640, 3)                  (3) server timing - 모델 계산 + 입출력 변환 + GPU, CPU 통신
  |  |      (4) prompt = "pick up the drone"
  |  |  <-- action chunk (10, 7)
  v  |
┌─ [C] Policy server - serve_policy.py ──────────────────────────────────────────────────┐    ┌─ [D] operator ──────────┐
│ ~/openpi, 위치: desktop, JAX 라이브러리 사용                                           │    │ drone_cmd.py kill       │
│ serve_policy.py 실행, checkpoint 폴더에서 params, norm_stats.json 읽음                 │    │ (사람이 드론 모터 정지) │
│                                                                                        │    └─────────────────────────┘
│ Step1. get_config("pi05_ur5_...") 로 model 구조 읽기                                   │
│ Step2. weight 를 GPU 에 올리기 + norm stats 읽기 (create_trained_policy)               │    --/sim/drone_kill (service)-->
│ Step3. policy 전체 만들고 실행                                                         │      motor off (PX4 kill)
│   (1) 입력: UR5Inputs -> Normalize -> Resize 224 -> Tokenize prompt                    │
│             -> Pad state and action (32,)                                              │
│       third_view = base, wrist 이미지 -> 224 x 224                                     │
│       state (7,) 를 norm_stats.json 의 관절별 분위수 (1 %, 99 %) 로                    │
│         x_norm = (x - q01) / (q99 - q01) x 2 - 1   (-1 ~ 1)                            │
│       -> (32,) 로 0 채움                                                               │
│   (2) 모델: pi0.5 sample_actions - flow matching (10 step)                             │
│       시각 224 x 224, 언어 = 토큰화, 액션 (32,)                                        │
│       (잡음 -> 데이터 쪽으로 여러 번 조금씩 이동,                                      │
│        자세한 것 docs/pi05_flow_matching.md)                                           │
│   (3) 출력: Unnormalize -> AbsoluteActions -> UR5Outputs (= action)                    │
│       model 이 낸 정규화된 action (10, 32)                                             │
│         -> x = (x_norm + 1) / 2 x (q99 - q01) + q01  (delta rad)                       │
│         -> t_obs 의 측정 state 에 delta 를 더해 절대 action (팔 6 만, 그리퍼는 절대값) │
│         -> 패딩 25 개 버리고 (10, 32) -> (10, 7)                                       │
│ Step4. websocket 8000 에서 대기                                                        │
│        action chunk (10, 7) 를 요청마다 응답 (약 0.185 ~ 0.195 s)                      │
└────────────────────────────────────────────────────────────────────────────────────────┘

추론 시간 = obs 1 개 보내고 action 받을 때까지 걸리는 시간
  = 0.12 ~ 0.13 s (정책 단독, Isaac Sim GUI 없이)
  = 0.18 ~ 0.19 s (Isaac Sim GUI 와 같이, 클라이언트에서 잰 왕복)
  25 Hz 에서 1 칸 = 40 ms = 0.04 s -> 0.15 s 는 약 4 칸

option 4: 실물 radiolink 조종기 -> raspberry pico -> sim (드론)
```
