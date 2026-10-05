# CLAUDE.md

UR5(CB3, UR5e 아님) + ROBOTIS RH-P12-RN(A) 1DOF 그리퍼 + 손목 D435i 로 드론 파지 시연을 수집해
LeRobot v2.1 데이터셋으로 만들고 openpi π0.5 를 파인튜닝하는 레포. 실물 파이프라인과 Isaac Sim 6.1.0 파이프라인이 녹화·변환을 공유한다.

전체 계획과 단계별 완료 기준: `docs/PLAN.md`
1-5 drive 튜닝 정리 (확정값·근거·실행 명령): `docs/drive_tuning.md`
그리퍼 파지력 (실물 전류 ↔ sim max_force·마찰, 전류를 바꿀 때): `docs/gripper_force.md`
드론 비행 (Pegasus 방식: 추력 모델·기하 제어기·게인 환산, PX4 SITL 연결·센서·위치 정보, 식과 출처): `docs/drone_flight.md`
sim 실행 속도 (RTF 측정·시간 내역·카메라 QoS·Python 콜백 최적화·남은 후보): `docs/sim_performance.md`
sim 팔 명령 보간 (`/joint_command` 30 Hz 계단 문제, 다른 프로젝트 조사, PhysX 관절 속도·rclpy spin 문제): `docs/arm_command_interpolation.md`
sim ROS 2 인터페이스 (토픽·주기·QoS 표, 구조도, 리셋·보호 정지, 실행·검사 명령): `docs/sim_ros2_interface.md`
ROS 2 데이터 녹화 (녹화 도구 명령·키, 녹화 토픽·주기 표, 구조도, `episode.json`, 녹화 뒤 변환·병합·검수): `docs/data_recording.md`
변환·병합·검수 옵션 전체 (sim·실물 공용): `README.md`, `dataset_merge.md`

## 경로 규칙

| 항목 | 경로 | 규칙 |
|------|------|------|
| Isaac Sim 설치 | `~/isaacsim` | 실행만. **여기에 파일을 만들지 않는다**. 스크립트는 `~/isaacsim/python.sh <script>` 로 실행 |
| 우리 sim 코드 | `~/data_collection_ur5_gripper/isaacsim/` | sim 전용 코드는 전부 여기 |
| 로봇 description | `isaacsim/ur5_rh_p12_description/` | xacro → `scripts/build_urdf.sh` → `out/` (git 제외) |
| 로봇 USD | `isaacsim/assets/robots/ur5_rh_p12_d435i/ur5_rh_p12_d435i.usda` | **원본 수정 금지**. 로봇 USD 는 URDF 재빌드 → 재import 로만 갱신한다. 손으로 편집하지 않는다. 설정값(drive, 물리 재질 등)은 스크립트나 씬 레이어에서 적용한다 |
| 작업 브랜치 | `isaacsim_v6.1.0` | 실물 파이프라인(레포 최상위 스크립트)을 깨지 않는다 |

## 원칙

1. sim 은 실물과 같은 ROS 2 토픽 인터페이스를 흉내낸다. 녹화(`record_toggle.py`)와 변환(stage1 → v2.1)은 sim/실물 공유.
2. 텔레옵 장치는 `/joint_command` + `/gripper/command` 만 발행한다 (장치 교체 가능).
3. 에셋(USD)과 씬을 분리한다.
4. **조용한 fallback 금지.** 입력이 기대와 다르면 에러로 중단한다. 대체값은 명시적 옵션일 때만.

## 데이터셋 스키마 (0단계 확정, LeRobot v2.1, 25 Hz)

| 키 | 정의 |
|----|------|
| `observation.state[0:6]` | 측정 관절각 (rad). 순서 shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3 |
| `observation.state[6]` | 그리퍼 present / 1150, 0~1 연속, 0=열림 |
| `action[0:6]` | 텔레옵 절대 관절 목표 (rad) = `/joint_command`, 프레임 t 직전 최신 명령(zero-order hold). delta 변환은 openpi transform 에서 |
| `action[6]` | 실행된 그리퍼 명령 {0.0, 1.0} = goal / 1150, 0=열림 |
| `observation.base_imu` | (6,) = [각속도 xyz (rad/s), 선가속도 xyz (m/s²)], 베이스 IMU 좌표계. 고정 베이스에서는 상수 |
| `observation.images.wrist` | `/cam/wrist/color/image_raw`, 640x480 |
| `observation.images.third_view` | `/cam/third_view/color/image_raw`, 640x480 |

- 그리퍼 이진화: 수집 시 드라이버, 추론 시 0.5 threshold
- 토픽 값은 raw(0~1150) 유지, 0~1 정규화는 stage1 에서만
- 카메라 시리얼 → 역할 연결은 `config/cameras.yaml` 한 곳에서만 (로더 `camera_config.py`. 지금 실물 wrist = D435i, third_view = D456)
- `/joint_command` 의 `header.stamp` 는 보내는 쪽(텔레옵 노드)이 넣는다 (sim = sim time). stage1 이 이 stamp 로 팔 action 을 맞추고 0 이면 에러
- 로그·메타데이터 위치값은 로봇 base 기준
- 실물 freedrive 데이터(`action = states[i+1]`)와 텔레옵 데이터는 팔 action 의미가 달라 섞지 않는다

## 토픽 (sim = 실물)

`/clock`, `/joint_states`, `/joint_command`, `/gripper/command`, `/gripper/joint_states`(present raw), `/gripper/target`(goal raw 0/1150),
sim 전용: `/protective_stop`(Bool), 서비스 `/sim/reset`·`/sim/drone_kill`. 주기·타입은 `docs/sim_ros2_interface.md`.
`/cam/wrist/color/image_raw`, `/cam/third_view/color/image_raw`, `/base/imu`(8단계부터. 고정 베이스에서는 sim·실물 모두 발행 안 하고 stage1 `--base-imu const`).
자세한 표는 `docs/PLAN.md` 부록 B.

## 녹화·변환 (4단계, sim·실물 공용)

| 단계 | 실행 | 환경 |
|------|------|------|
| 녹화 | `./6_record_bag.sh <작업명> [--sim]` (`record_toggle.py`): r 시작/종료, d 버리기, q 종료. `--sim`: r → `/sim/reset` → 녹화, k = 드론 모터 정지. bag 폴더에 `episode.json` | 시스템 python3 + ROS |
| stage1 | `python3 lerobot_stage1_extract_bag.py <bag> --arm-action command\|next_state --base-imu const\|topic` (둘 다 필수) → `bag_lerobot_intermediate/<bag>/` | 시스템 python3 + ROS |
| 병합 | `python lerobot_merge_episodes_v21.py --manifest <txt> \| --all --repo-id <id>` (에피소드 1 개는 `lerobot_stage2_build_dataset_v21.py`) | conda `lerobot_v2` |
| 검수 | `python inspect_dataset_v21.py <데이터셋> --load` → PASS/FAIL | conda `lerobot_v2` |
| 전체 확인 | `python3 isaacsim/scripts/make_fake_episodes.py` (sim 을 띄워 가짜 에피소드 녹화 → 변환 → 병합 → 검수) → 3/3 PASS, 약 6 분 | 시스템 python3 + ROS |

- conda: `~/miniconda3`, 환경 `lerobot_v2` (Python 3.10, `lerobot==0.3.3` = v2.1). base 자동 활성화 끔. ROS 를 source 한 터미널에서는 `env -u PYTHONPATH python …`
- stage1 은 격자 시각에 고른 메시지 (카메라·팔·그리퍼) 가 66 ms 보다 오래됐으면 에러 (`--max-age`). `command` 면 첫 `/joint_command` 이전 구간을 뺀다
- 병합은 팔 action 종류가 다른 에피소드를 섞으면 중단, 보호 정지 에피소드는 기본 제외, 정지 구간 자르기는 옵션 (`--trim-idle`, 기본 안 함)
- 성공/실패 판정은 사람이 한다 (실패한 에피소드는 `d` 로 `bags/_discarded/` → 변환하지 않음). bag 은 자동으로 지우지 않는다 (30 s 에 약 1.5 GB)
- `lerobot_stage2_build_dataset_v30.py` 는 옛 중간 파일 형식 전용 (지금 stage1 출력을 읽지 못함, 학습은 v2.1)
- **sim 을 띄운 채 sim bag 을 `ros2 bag play` 하지 말 것** (bag 의 `/clock`·`/joint_command` 가 다시 발행됨)

## Isaac Sim 6.1.0 에서 확인된 사실

- **단위**: USD angular drive target·joint state 속성은 **degree**. ROS 토픽과 데이터셋은 rad. USD 속성을 직접 다룰 때 변환할 것
  - drive gain 도 같다: USD `stiffness`/`damping` 은 degree 기준(Nm/deg), tensor API(`Articulation.get/set_dof_gains`)는 rad 기준(Nm/rad).
    USD 1000 / 100 → tensor 5.73e4 / 5730 (× 180/π). 한계·위치도 tensor API 는 rad
- **물리 엔진**: `isaacsim.physics.newton` 이 켜져 있으면 시작 시 Newton 으로 자동 전환될 수 있다. 스크립트는 `SimulationManager.switch_physics_engine("physx")` 로 명시하고 확인할 것
- **Physics variant**: 로봇 USD 진입 파일의 `Physics` variant(physx / physics / mujoco / none)에 기본 선택이 없다. reference 할 때 `physx` 를 명시할 것
- **fixed base override**: USD 원본은 `ArticulationRootAPI` 가 `robot_mount` 에 있어 PhysX 가 floating base 로 만든다 (root_joint 가 외부 구속).
  1~7단계 씬은 `ArticulationRootAPI` 를 로봇 최상위 prim 으로 옮겨 fixed base 로 만든다 (`test_scene.build_test_scene(base="fixed")`, 메모리 stage 에만).
  `PhysxArticulationAPI` 설정도 새 root 에 적는다. ROS Publish Joint State 의 targetPrim 도 최상위 prim
- **DOF**: 10개 (mimic 3개 포함). 순서는 팔 6개 → `rh_r1_joint, rh_l1, rh_r2, rh_l2` (USD 선언 순서와 다름, 이름으로 찾을 것)
- **mimic 조인트(`rh_r2`, `rh_l1`, `rh_l2`)도 DOF 로 잡히지만 목표값·gain 을 주지 않는다. 그리퍼 명령은 `rh_r1_joint` 에만 준다**
- **rh_r1_joint 에 관절 속도 제한을 걸지 않는다.** mimic 과 함께 점성 저항처럼 동작해 열리지 못한다. 속도는 목표값 이동으로 맞춘다
  (`robot_drive.GripperProfile`, `profile_velocity`). 관절 속도 한계는 USD 값(6.5 rad/s) 그대로
- **그리퍼 armature 0.01 (`rh_r1_joint` 에만, D 1.3758)**: armature 0 이면 손가락 묶음(관성 3.78e-4)이 너무 가벼워 물체를 조일 때
  solver 가 수렴하지 못해 maxForce 의 약 26% 만 전달된다. 파지력은 `grasp_tests.py` K3 의 가상일 확인(전달률)으로 본다. 근거는 PLAN 1-6
- **articulation 수면 끔 (`sleep_threshold: 0`)**: 팔이 멈춘 채 가벼운 손가락만 움직이면 운동에너지가 작아 PhysX 가 articulation 을 재워 그리퍼가 중간에 멈춘다.
  설정 파일 `articulation.sleep_threshold` → 새 root 의 `PhysxArticulationAPI` 에 적고 재생 후 확인
- **시험용 하중(드론 무게 등)을 로봇 링크에 FixedJoint 로 붙일 때는 `physics:excludeFromArticulation = true`**.
  없으면 PhysX 가 그 강체를 articulation 의 새 링크로 흡수해 로봇 중력 보상 계산에 하중 무게가 들어간다
- **self-collision 켬** (`articulation.self_collision: true`, 충돌 제외 쌍 없음). 관절로 직접 연결된 링크 쌍은 USD 에서 이미 충돌이 꺼져 있다.
  빈손으로 끝까지 닫으면 손가락이 64.233° 에서 서로 닿아 멈춘다 (관절 한계 65.04° 까지 가지 않음). 근거는 PLAN 1-6
- 팔 drive 는 높은 stiffness(중력 보상 feedforward 없음)로 확정 (`isaacsim/config/drive_gains.yaml`). ω ≥ 70 rad/s (텔레옵 추종 지연 < 40 ms)
- 테스트 씬에서 로봇은 바닥에서 띄워 배치한다 (관절 0 자세에서 팔이 바닥에 닿음). 높이는 씬 스크립트의 값이고 로봇 USD 에는 넣지 않는다
- **구조 회귀 검사**: 로봇 USD 재import 후 `~/isaacsim/python.sh isaacsim/scripts/check_articulation.py --headless` → 8/8 PASS 확인 (리포트는 `isaacsim/reports/`, git 제외)
- **그리퍼**: 명령은 `rh_r1_joint` 하나에만. mimic(`rh_r2`, `rh_l1`, `rh_l2`)은 `NewtonMimicAPI` 로 들어왔고 PhysX 에서 동작 확인.
  mimic 조인트 3개에는 drive stiffness 를 주지 않는다. 한계 1.1351 rad(65.04°), 손가락 접촉 ≈ 64.2°. raw 0~1150 ↔ 0~1.1351 rad 선형
- **root_joint**: body0 = 로봇 최상위 prim(강체 아님), body1 = `robot_mount`. 유지한다. 로봇 위치는 최상위 prim Transform 으로 지정하고,
  `robot_mount` 에 FixedJoint 를 추가하지 않는다
- 질량 없는 링크(`tool0`, `flange`, `rh_p12_rn_tcp`, 카메라 프레임)는 물리 없는 Xform
- **PX4 SITL** (`--flight px4`, `docs/drone_flight.md` 13장): `~/PX4-Autopilot` v1.16.0, pymavlink 은 `isaacsim/.pydeps/` (git 제외, `~/isaacsim` 에 설치 안 함).
  **가상 센서에 주는 속도는 자세·위치 차분**: 그리퍼 접촉이 걸리면 PhysX 가 보고하는 강체 각속도가 실제 자세 변화와 다름 (잡힌 드론: 보고 29 °/s, 실제 0.5 °/s)
- PX4 를 띄운 스크립트는 `PR_SET_PDEATHSIG` 로 같이 끝난다 (`simulation_app.close()` 는 atexit 을 건너뜀). 같은 instance PX4 가 남아 있으면 시작 전에 에러
- **sim ROS 2 실행은 `isaacsim/scripts/sim_ros2.py`** (씬 + 토픽 + `/sim/reset`). 검사 `python3 isaacsim/scripts/check_ros2.py` → 9/9 PASS (약 8 분).
  `/sim/reset` 은 항상 순간이동 + PX4 재시작 (약 24 s, 매 에피소드 같은 깨끗한 PX4). 리셋 실패면 sim 은 계속, 다음 성공까지 명령 무시.
  응답 JSON 에 에피소드 메타데이터 (시드, 드론 위치 (world·base), git, 렌더·그리퍼·드론·카메라 설정). `/sim/drone_kill` = 드론 모터 정지 (PX4·기하 공통).
  보호 정지 흉내 (`protective_stop.py`): 접촉력 150 N (손가락은 환경과의 접촉만), 위치 오차 5°, 관절 속도 200 °/s
- **sim 안에서 시간을 잴 때는 sim time (`SimulationManager.get_simulation_time()`)**. `flight.t` 는 기하 제어기를 다시 켤 때 0 으로 돌아간다
- **팔을 순간이동하면 속도 차분 기준(ArmBridge `set_now`, 보호 정지 `resync`)도 맞출 것**. 안 하면 순간이동 거리 / dt 가 관절 속도로 잡힘
- **sim ROS 2** (3단계, `docs/sim_performance.md`): python.sh 실행 전에 `source /opt/ros/jazzy/setup.bash` (rclpy = 시스템 Jazzy, 아니면 에러).
  카메라는 OmniGraph Camera Helper, 주기는 카메라 prim `omni:sensor:tickRate` (6.0 부터, `frameSkipCount` deprecated). 앱 루프 30 Hz (= 카메라), 물리 120 Hz.
  카메라 받는 쪽은 RELIABLE (best effort 면 640x480 이미지가 통째로 버려짐)
- **sim 팔 명령은 보간한다** (`ros2_iface.yaml` `arm.command_interp_time`, 물리 스텝마다 선형). 앱 루프(30 Hz)마다 바로 넣으면 계단이 되어
  단단한 drive 가 관절 최대 속도로 뛰었다 멈춘다
- **PhysX 가 보고하는 관절 속도(`get_dof_velocities`)는 실제보다 작다** (wrist_3 0.42 배, 멈춘 관절도 1~3 °/s). `/joint_states` velocity·보호 정지는 관절각 차분
- **rclpy `spin_once` 는 메시지가 쌓여 있어도 콜백·빈 호출을 번갈아 한다** → 빈 호출 한 번에 멈추면 메시지가 버려짐. 두 번 연속일 때 멈출 것 (`ros2_iface.spin`)
- **카메라 렌더 설정은 `ros2_iface.yaml` `render` 에 명시** (DLSS Performance). RTX 실시간 렌더러는 DLSS·DLAA 만 지원(TAA·끔 불가),
  DLSS 모드는 새 stage 마다 기본값으로 돌아가므로 stage 를 만든 뒤 적용하고 다시 읽어 확인
- URDF importer 에는 Merge fixed joints, Joint Drive Type 옵션이 없다. USD Output 폴더 안에 로봇 이름 폴더를 만든다
