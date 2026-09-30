# CLAUDE.md

UR5(CB3, UR5e 아님) + ROBOTIS RH-P12-RN(A) 1DOF 그리퍼 + 손목 D435i 로 드론 파지 시연을 수집해
LeRobot v2.1 데이터셋으로 만들고 openpi π0.5 를 파인튜닝하는 레포. 실물 파이프라인과 Isaac Sim 6.1.0 파이프라인이 녹화·변환을 공유한다.

전체 계획과 단계별 완료 기준: `docs/PLAN.md`

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
- 카메라 시리얼 → 역할 연결은 `config/cameras.yaml` 한 곳에서만
- 로그·메타데이터 위치값은 로봇 base 기준
- 실물 freedrive 데이터(`action = states[i+1]`)와 텔레옵 데이터는 팔 action 의미가 달라 섞지 않는다

## 토픽 (sim = 실물)

`/clock`, `/joint_states`, `/joint_command`, `/gripper/command`, `/gripper/joint_states`(present raw), `/gripper/target`(goal raw 0/1150),
`/cam/wrist/color/image_raw`, `/cam/third_view/color/image_raw`, `/base/imu`. 자세한 표는 `docs/PLAN.md` 부록 B.

## Isaac Sim 6.1.0 에서 확인된 사실

- **단위**: USD angular drive target·joint state 속성은 **degree**. ROS 토픽과 데이터셋은 rad. USD 속성을 직접 다룰 때 변환할 것
  - drive gain 도 같다: USD `stiffness`/`damping` 은 degree 기준(Nm/deg), tensor API(`Articulation.get/set_dof_gains`)는 rad 기준(Nm/rad).
    USD 1000 / 100 → tensor 5.73e4 / 5730 (× 180/π). 한계·위치도 tensor API 는 rad
- **물리 엔진**: `isaacsim.physics.newton` 이 켜져 있으면 시작 시 Newton 으로 자동 전환될 수 있다. 스크립트는 `SimulationManager.switch_physics_engine("physx")` 로 명시하고 확인할 것
- **Physics variant**: 로봇 USD 진입 파일의 `Physics` variant(physx / physics / mujoco / none)에 기본 선택이 없다. reference 할 때 `physx` 를 명시할 것
- **DOF**: 10개 (mimic 3개 포함). 순서는 팔 6개 → `rh_r1_joint, rh_l1, rh_r2, rh_l2` (USD 선언 순서와 다름, 이름으로 찾을 것)
- **mimic 조인트(`rh_r2`, `rh_l1`, `rh_l2`)도 DOF 로 잡히지만 목표값·gain 을 주지 않는다. 그리퍼 명령은 `rh_r1_joint` 에만 준다**
- 테스트 씬에서 로봇은 바닥에서 띄워 배치한다 (관절 0 자세에서 팔이 바닥에 닿음). 높이는 씬 스크립트의 값이고 로봇 USD 에는 넣지 않는다
- **구조 회귀 검사**: 로봇 USD 재import 후 `~/isaacsim/python.sh isaacsim/scripts/check_articulation.py --headless` → 8/8 PASS 확인 (리포트는 `isaacsim/reports/`, git 제외)
- **그리퍼**: 명령은 `rh_r1_joint` 하나에만. mimic(`rh_r2`, `rh_l1`, `rh_l2`)은 `NewtonMimicAPI` 로 들어왔고 PhysX 에서 동작 확인.
  mimic 조인트 3개에는 drive stiffness 를 주지 않는다. 한계 1.1351 rad(65.04°), 손가락 접촉 ≈ 64.2°. raw 0~1150 ↔ 0~1.1351 rad 선형
- **root_joint**: body0 = 로봇 최상위 prim(강체 아님), body1 = `robot_mount`. 유지한다. 로봇 위치는 최상위 prim Transform 으로 지정하고,
  `robot_mount` 에 FixedJoint 를 추가하지 않는다
- 질량 없는 링크(`tool0`, `flange`, `rh_p12_rn_tcp`, 카메라 프레임)는 물리 없는 Xform
- URDF importer 에는 Merge fixed joints, Joint Drive Type 옵션이 없다. USD Output 폴더 안에 로봇 이름 폴더를 만든다
