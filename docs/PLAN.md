# Isaac Sim 드론 파지 데이터 수집 파이프라인 계획

작성일: 2026-09-29 · 0단계 확정 반영 (2026-09-29) · 1-3 import 확인 결과 반영 (2026-09-30) · 5단계 장치 결정 (2026-10-08: 드론 조종기 먼저, 팔은 SpaceMouse → 2026-10-11: 팔은 PICO 4 Ultra) · 5-B 설치·B-0·B-1·B-2 결과 (2026-10-12) · 5-B B-3·B-4, 컨트롤러 버튼 (2026-10-14) · 컨트롤러 필터, 손목 카메라 마운트 t10 (2026-10-15) · third view D456 intrinsics·위치 후보 비교, 리셋 시간 단축 계획 (2026-10-16) · 실물 드론 Spry+ 조사 (Iris 0.75 배 0.8 kg 유지), 잡는 곳 빨간 테이프, 대량 수집 전 순서 변경 (2026-10-16) · 드론 사인파 hover 프리셋 (2026-10-16) · 드론 기준 위치 목록 (2026-10-16) · third view 위치 Q2·D455 모양·이륙 지점 (2026-10-22) · 첫 소량 수집 13 개, GUI 보조 옆·위 뷰포트, openpi LoRA smoke 통과 (2026-10-23)

## 목표

Isaac Sim 6.1.0(standalone zip) 에서 UR5(CB3) + RH-P12-RN(A) 그리퍼 + 손목 D435i 로 공중의 드론을 파지하는
시연 데이터를 텔레오퍼레이션으로 수집하고, LeRobot v2.1 데이터셋으로 변환해 openpi π0.5 를 파인튜닝한다.

- 1차 목표: **고정 베이스**에서 드론 파지 (1~7단계)
- 2차 목표: **흔들리는 6DOF 베이스**에서 드론 파지 (8단계, VLA 또는 RL)
- 학습은 서버에서 수행

경로·브랜치 규칙은 `CLAUDE.md` 를 따른다. 이 문서의 sim 코드 경로는 모두 `~/data_collection_ur5_gripper/isaacsim/` 기준이다.

## 현재까지 된 것

계획을 쓸 때 (2026-09-29) 이미 있던 것:
- 실물 파이프라인 (`data_collection_ur5_gripper`): ros2 bag 녹화 → stage1(intermediate) → LeRobot v2.1 / v3.0 변환 → 검수
  (v3.0 변환은 4단계에서 뺐다: 학습은 v2.1)
- openpi π0.5 파인튜닝 smoke test 완료
- Isaac Sim 6.1.0 ROS 2 브릿지 사전 시험: `/clock`, `/joint_states` 발행, `/joint_command` 구독으로 UR5 구동 확인
- 사전 시험 sim bag(진짜 sim `/joint_states` + 가짜 그리퍼·카메라 노드, 모두 sim time) 이 **기존 stage1 을 코드 수정 없이 통과**
  (가짜 노드는 4단계에서 지움: `sim_ros2.py` 가 진짜 토픽을 낸다)
- 사전 시험 씬(UR5 단독): Ground Plane, Dome Light, EastRural_Table(collider 추가, 상판 z=0.762), UR5 를 테이블에 FixedJoint 로 고정
  (지금 씬은 2-3 `drone_scene.py`: 로봇 위치는 최상위 prim Transform, FixedJoint 를 추가하지 않는다)

그 뒤에 된 것:
- 로봇 description: `isaacsim/ur5_rh_p12_description/` (UR5 + 샌드위치 마운트 + RH-P12-RN(A) + D435i 단일 URDF, xacro 빌드·기구학 검증 완료)
- 로봇 USD import 완료: `isaacsim/assets/robots/ur5_rh_p12_d435i/ur5_rh_p12_d435i.usda` (1-3 확인 결과 참고)
- 1·2단계 sim 구성 완료 (로봇 에셋·drive·파지·손목 카메라, 드론 씬·PX4 SITL 드론·스크립트 파지 데모). **단계별 실행 명령: 부록 D**
- 3단계 sim ROS 2 인터페이스 완료 (2026-10-05): `sim_ros2.py`, `check_ros2.py` 9/9
- 4단계 녹화·변환·병합·검수 완료 (2026-10-05): `make_fake_episodes.py` 3/3. 실물 PC 는 카메라·그리퍼까지 확인 (팔·텔레옵 경로는 아직)
- 5단계 5-A 드론 텔레옵 완료 (2026-10-08): 실물 조종기로 sim PX4 드론 조종 (`sim_ros2.py --drone-rc on`). **다음: 5-B 팔 텔레옵 (PICO 4 Ultra)**
- 5-B 진행 (2026-10-12): XRoboToolkit 설치, B-0 컨트롤러 입력 확인 (90 Hz), B-1 URDF 기구학 (`teleop/ik_model.py`, sim 과 0.03 mm 이내), B-2 텔레옵 노드 (`teleop/xr_teleop_node.py`, 만든 입력으로 13/13, 실제 컨트롤러로 GUI 에서 동작 확인), B-3 실제 컨트롤러로 PX4 드론 파지·모터 정지·리셋 (2026-10-14 사용자 확인), B-4 컨트롤러 버튼으로 녹화 → 변환 → 검수 5/5 (2026-10-14, 에피소드 2 개). **다음: 조종기로 움직이는 드론 파지 (5단계 마지막 완료 기준) 또는 6단계 수집**
- 대량 수집 전 준비 (2026-10-15 ~): 컨트롤러 One Euro 필터, 손목 카메라 마운트 t10 (CAD → 실물 출력 → sim 재import, 5-B "손목 카메라 마운트"),
  third view 카메라 D456 확정·실물 intrinsics·위치 후보 비교 (6단계 "수집 전 확인"). `/sim/reset` 시간 단축은 조종기 모드만 줄었다 (29 → 24 s, 6단계 "수집 전 확인"). 잡는 곳 빨간 테이프 (2026-10-16).
  드론 사인파 hover 프리셋 small·mid·big, 드론 기준 위치 목록 25 점 (2026-10-16). third view 위치 Q2 (1.10, −0.30, 0.20)·D455 모양 (2026-10-22).
  third view 받침대 CAD (2026-10-22). 첫 소량 수집 13 개 (2026-10-23).
- 7단계 시작 (2026-10-23): first13 으로 openpi π0.5 LoRA smoke 100 step 통과 (연구실 서버 GPU 2 장, loss 0.074 → 0.051, `docs/openpi_finetuning.md`).
  full 파인튜닝 smoke 도 GPU 2 장 FSDP·batch 16 으로 통과. **다음: 데이터를 더 모은 뒤 본 학습 (LoRA / full) → sim 폐루프 평가** (7단계)

---

## 전체 원칙

1. **sim 은 실물과 같은 ROS 2 토픽 인터페이스를 흉내낸다.**
   녹화(`record_toggle.py`)와 변환(stage1 → v2.1)은 sim/실물이 공유한다. sim 에서 새로 만드는 것은 "토픽을 내보내는 쪽"뿐이다.
2. **텔레오퍼레이션 장치는 교체 가능한 부품이다.**
   어떤 장치든 `/joint_command`(팔 관절 목표) + `/gripper/command`(그리퍼 명령)만 내보낸다. PICO → SpaceMouse / GELLO 로 바꿔도 뒤쪽은 그대로.
   (드론 조종기 입력은 팔 텔레옵과 다른 통로, 5단계 5-A)
3. **에셋(USD)과 씬을 분리한다.**
   로봇 USD, 드론 USD, 씬 스크립트를 따로 둔다. 흔들리는 베이스·Isaac Lab(RL) 로 그대로 재사용하기 위함.
   로봇 USD 원본은 수정하지 않고, 설정(drive, 물리 재질 등)은 스크립트나 씬 레이어에서 덮어쓴다.
4. **조용한 fallback 금지.** (기존 파이프라인 원칙 유지) 입력이 기대와 다르면 에러로 중단한다.
5. **sim 전용 코드는 별도 폴더/브랜치.** (`isaacsim/`, 브랜치 `isaacsim_v6.1.0`) 실물 파이프라인을 깨지 않는다.

---

## 0단계: 설계 결정 확정

**상태: 확정 (2026-09-29).** 확정값은 `CLAUDE.md` 에 옮겨 적었다.

### 0-1. 데이터셋 스키마

| 항목 | 정의 | 비고 |
|------|------|------|
| 포맷 | LeRobot **v2.1** | π0 / π0.5 공용 (openpi 호환) |
| fps | **25 Hz** | 학습 주기 = 추론 주기로 맞춤. openpi 문서의 UR5e 규약은 20 Hz 이지만 필수 아님 |
| `observation.state[0:6]` | 측정 관절각 (rad) | 순서: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3 |
| `observation.state[6]` | 그리퍼 present / 1150 → **0~1 연속**, 0=열림 | sim: 브리지가 `rh_r1_joint`(rad) × 1150 / 1.1351 로 raw present 발행 → stage1 에서 /1150. 파지 시 중간값에서 멈춤(파지 신호) |
| `action[0:6]` | 텔레옵 **절대 관절 목표** (rad) = `/joint_command`, 프레임 t 직전의 최신 명령(zero-order hold) | 데이터셋에는 절대값 저장. delta 변환은 openpi transform(`DeltaActions(make_bool_mask(6, -1))`) |
| `action[6]` | **실행된 그리퍼 명령 {0.0, 1.0}** = goal / 1150, 0=열림 | 이진 제어이므로 이진 기록 (원칙: 로봇이 실제로 받은 명령을 기록). 제어 방식이 연속으로 바뀌면 그때 변경 |
| `observation.base_imu` | (6,) = [각속도 x, y, z (rad/s), 선가속도 x, y, z (m/s²)], **베이스 IMU 좌표계** | 처음부터 모든 데이터셋에 포함. 고정 베이스에서는 상수(각속도 ≈ 0, 선가속도 ≈ (0, 0, +9.81)). 사용 여부는 openpi repack 매핑으로 결정(매핑 안 하면 자동 무시) |

- 그리퍼 0~1 변환은 성능 목적이 아니라 규약·가독성 목적 (π0.5 는 자체 norm stats 의 quantile 정규화를 쓰므로 스케일 무관)
- 그리퍼 이진화 위치: **수집 시** 드라이버가 텔레옵 입력 → goal(0/1150, 전류 한계) 변환 / **추론 시** 모델 출력을 0.5 기준 threshold
- 연속 트리거 장치(GELLO, PICO)를 쓰면 이진화 전 원시값도 stamp 있는 토픽으로 **bag 에만** 녹화 (나중에 재수집 없이 재변환 가능)
- 실물 freedrive 데이터(`action = states[i+1]`)와는 팔 action 의미가 다름 → 섞지 않는다

### 0-2. 카메라

| 역할 | 데이터셋 키 | ROS 2 토픽 | 실물 | sim |
|------|-------------|-----------|------|-----|
| 손목 | `observation.images.wrist` | `/cam/wrist/color/image_raw` | D435i (시리얼은 yaml) | 손목 Camera prim |
| 외부 | `observation.images.third_view` | `/cam/third_view/color/image_raw` | D456 (시리얼은 yaml) | 외부 Camera prim |

- color 640x480, **sim 에서는 처음부터 두 대 모두 녹화**
- 토픽 이름 = 역할. 시리얼 → 역할 연결은 **카메라 실행 시점 한 곳**(`config/cameras.yaml`)에서만 한다
  ```yaml
  # config/cameras.yaml
  wrist:
    serial: "123456789012"
  third_view:
    serial: "234567890123"
  ```
  realsense2_camera: `serial_no:=_<시리얼>`(앞에 `_` 로 문자열 강제), `camera_namespace:=cam`, `camera_name:=<역할>`
- stage1 의 토픽 → 키 매핑은 고정 표. 토픽이 없으면 에러로 중단
- 에피소드 `meta.json` 에 역할 → 시리얼 기록 (sim 은 `sim` + prim 경로)

### 0-3. 기타

| 항목 | 정의 |
|------|------|
| 좌표 기준 | 로그·메타데이터의 EEF/드론 위치는 **로봇 base 기준** |
| 베이스 상태 | `observation.base_imu` (6,) 를 처음부터 기록 (0-1 표). 자세(quaternion)는 데이터셋에 넣지 않고, sim ground-truth 베이스 pose 는 intermediate 에만 |
| 텔레옵 인터페이스 | 모든 장치는 `/joint_command` + `/gripper/command` 만 발행. 녹화 도구 요청 `/teleop/request` (컨트롤러 버튼 → 녹화 도구 키, 녹화하지 않음) 는 예외 (2026-10-14, 5-B "컨트롤러 버튼") |

### 0단계 체크리스트 (확정)

- [x] 포맷: LeRobot v2.1, 25 Hz
- [x] `state[0:6]`: 측정 관절각 (rad), 순서 shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
- [x] `state[6]`: 그리퍼 present / 1150, 0~1 연속, 0=열림
- [x] `action[0:6]`: 텔레옵 절대 관절 목표 (rad), 프레임 t 직전 최신 명령. delta 변환은 openpi transform 에서
- [x] `action[6]`: 실행된 그리퍼 명령 {0.0, 1.0} (= goal / 1150), 0=열림. 제어 방식이 연속으로 바뀌면 그때 변경
- [x] 그리퍼 이진화: 수집 시 드라이버, 추론 시 0.5 threshold
- [x] 카메라: `observation.images.wrist`, `observation.images.third_view`, 토픽 `/cam/<역할>/color/image_raw`, 시리얼은 `config/cameras.yaml`, 640x480, 처음부터 두 대 모두 녹화
- [x] 베이스: `observation.base_imu` (6,) = [각속도 xyz, 선가속도 xyz], 베이스 IMU 좌표계. 고정 베이스에서는 상수. 사용 여부는 openpi repack 에서 결정
- [x] 좌표 기준: 로그·메타데이터 위치값은 로봇 base 기준
- [x] 텔레옵 인터페이스: `/joint_command` + `/gripper/command`

---

## 1단계: 로봇 에셋

**목표**: UR5 + 마운트 + RH-P12-RN(A) + D435i 가 단일 articulation 으로 동작하는 로봇 USD

### 1-1. 실측값 반영
- `mount_thickness`(STL 5 mm), `adapter_thickness`(브래킷 단독, 0.010: **실측 확인 (약 10 mm)**, 재빌드 불필요), `cam_tilt`
- `gripper_yaw` = π/2: **실물 확인 완료** (손가락이 카메라 긴 변과 나란히 벌어짐, 카메라 시야를 가리지 않음)
- `cam_xyz`, `cam_rpy`: 실물은 카메라 윗면이 **바깥쪽(공구 축 반대)** → 기본값을 `0 -0.0125 0.01015` / `-1.5708 -1.5708 0` 으로 변경 (2026-09-30)
  - 로봇 USD 재import 완료 (2026-09-30, 커밋 3e25e81)
- 카메라 장착 위치: D435 메시 후면 M3 구멍(Ø2.5, 간격 45 mm, 본체 높이 중앙)이 마운트 구멍과 일치 → **확인 완료** (방향 변경 후 FK 로 (±22.5, 57.5, 5.0) mm 재확인)

### 1-2. URDF 빌드
- `scripts/build_urdf.sh kinematics_params:=$HOME/my_robot_calibration.yaml`
- 기구학 yaml 은 고장 난 옛 UR5 기준. 새 로봇이 오면 교체 후 재빌드

### 1-3. Isaac Sim import — **완료, 확인 결과**
- 6.1.0 importer 실제 옵션: Collision From Visuals, Allow Self-Collision, Robot Type, Base Type(Source/Fixed/Mobile),
  Add Reference to Stage, Merge Mesh, Debug Mode. **Merge fixed joints, Joint Drive Type 옵션은 없다.**
  USD Output 은 지정한 폴더 안에 로봇 이름 폴더를 만든다 (`assets/robots` 지정 → `assets/robots/ur5_rh_p12_d435i/`)
- 사용한 설정: Collision From Visuals 끔, Allow Self-Collision 끔, Merge Mesh 끔, Base Type Source, ROS Package List 비움
- 프레임: 질량 없는 링크(`tool0`, `flange`, `rh_p12_rn_tcp`, 카메라 프레임)는 물리 없는 Xform 으로 남음
- **mimic**: `NewtonMimicAPI` 로만 들어왔지만(`physx.usda` 에 mimic 없음) **PhysX 에서 동작 확인**.
  `rh_r1_joint` 에만 drive(stiffness 1000, damping 100)를 주고 target 을 올리자 네 손가락이 대칭으로 같이 닫힘.
  → 그리퍼 명령은 `rh_r1_joint` 하나에만 준다
- **단위**: USD angular drive target 과 joint state 속성은 **도(degree)**. ROS 토픽과 데이터셋은 rad
  - target 63° 에서 손가락 닫힘 (한계 1.1351 rad = 65.04°), 양쪽 손가락 안쪽 면 접촉 ≈ 64.2°
- **root_joint**: importer 가 `PhysicsFixedJoint "root_joint"` 를 만듦. body0 = 로봇 최상위 prim(`/ur5_rh_p12_d435i`, 강체 아님), body1 = `robot_mount`
  - 로봇은 최상위 prim 위치에 고정된다 (world 원점이 아님)

### 1-4. 로봇 배치와 구조 검증
- **root_joint 는 유지**한다. 로봇 위치는 최상위 prim 의 Transform 으로 지정한다
- `robot_mount` 에 FixedJoint 를 **추가하지 않는다** (같은 링크를 두 번 고정하게 됨)
- 8단계에서는 씬에서 root_joint 의 body0 을 움직이는 베이스 강체로 바꾸는 방식을 검토
- **fixed base override (씬, `--base fixed`, 1~7단계)**: USD 원본의 `ArticulationRootAPI` 는 링크 `robot_mount` 에 있고
  `root_joint` 는 `/Physics` 아래에 있어서, PhysX 가 root_joint 를 articulation 밖 구속으로 보고 **floating base** 로 만든다
  (질량행렬 16×16 = DOF 10 + 베이스 6, 중력 토크 API 도 베이스 성분이 섞여 쓸 수 없음. 2026-09-30 확인)
  - 씬에서 `ArticulationRootAPI` 를 `robot_mount` → 로봇 최상위 prim 으로 옮긴다 (메모리 stage 에만, USD 원본 그대로) → `fixed_base = True`, 질량행렬 10×10
  - articulation 설정(`PhysxArticulationAPI`: solver 반복 횟수, self-collision)은 새 root(최상위 prim)에 적용.
    값은 `isaacsim/config/drive_gains.yaml` 의 `articulation`, 재생 후 읽어서 확인
  - 구현: `isaacsim/scripts/test_scene.py` 의 `build_test_scene(base="fixed")`. `--base floating` 은 USD 원본 그대로 (8단계에서 다시 결정)
- **구조 검증 스크립트** `isaacsim/scripts/check_articulation.py` (`~/isaacsim/python.sh`, `--headless` 지원)
  - 테스트 씬: physics scene + ground plane + 로봇 USD reference (위 배치 규칙대로, 최상위 prim z = 0.762 m = 테이블 상판 높이.
    0 이면 관절 0 자세에서 TCP 가 z = −5 mm 라 팔이 바닥에 걸림). 1-5 drive 튜닝도 같은 씬을 쓴다
  - 로봇 USD 를 재import 할 때마다 실행하는 회귀 검사. 로봇 USD 원본은 수정하지 않음 (검사용 값은 실행 중 메모리에서만)
  - 검사 항목 (각각 PASS/FAIL, 리포트는 `isaacsim/reports/check_articulation_<날짜시간>.txt`)
    1. DOF 이름·순서·개수 (mimic 조인트의 DOF 포함 여부는 가정하지 않고 보고). 팔 6개 이름이 실물 드라이버와 동일
    2. 관절 한계 (rad): elbow ±π, 나머지 팔 ±2π, 그리퍼 4개 0~1.1351
    3. drive·mimic 설정 출력 (mimic 대상, 계수, 오프셋)
    4. root_joint 의 body0 = 최상위 prim, body1 = robot_mount. articulation root 가 기대 위치(fixed: 최상위 prim)에 하나,
       PhysX `fixed_base == True`, 설정 파일의 articulation 설정이 새 root 에 적용됨
    5. `rh_p12_rn_tcp`, `wrist_camera_color_optical_frame` 존재와 world 좌표
    6. 링크별 질량·합계, 질량 0 이하나 관성 비정상 경고
    7. 3초 시뮬레이션 후 NaN·폭주 없음 (팔은 stiffness 0 이라 처져도 정상)
    8. `rh_r1_joint` 에만 임시 drive(stiffness 1000, damping 100, target 30°) → 2초 후 네 그리퍼 조인트 각도 차 1° 이내

### 1-5. drive 튜닝
- **정리 문서: `docs/drive_tuning.md`** (설계 근거, 최종값, 시험 결과, 알려진 위험, 남은 일, 실행 명령). 아래는 진행 기록
- import 직후 값: 팔 stiffness/damping 0 (명령을 줘도 추종하지 않고 중력에 처짐), maxForce 팔 150/150/150/28/28/28, 그리퍼 1000.
  속도 한계: fixed base 에서 tensor API 로 읽으면 팔 π rad/s (180°/s), 그리퍼 6.5 rad/s (372°/s) → URDF 값이 적용돼 있음
- **방침 (1회차 시험 후 확정, 2026-09-30)**
  - 중력은 켠다. 팔은 **높은 stiffness, 중력 보상 feedforward 없음** 으로 실물 UR 처럼 무게를 버티며 추종 (8단계 흔들리는 베이스에서 관성력도 실물처럼 나오게)
    - feedforward(`gravity_ff: true`, `drive_gains_ff.yaml`, ω = 30)는 비교 기록으로 남김: 드론 무게는 보상하지 못해 손목 처짐 2~9°, 추종 지연 약 80 ms 로 불합격
    - 공식 UR5 에셋 값 비교: `drive_gains_official.yaml` (공식 `ur5.usd` 의 drive 값을 rad 기준으로 변환)
  - 설정값은 `isaacsim/config/drive_gains.yaml` 에 **rad 단위**(Nm/rad, Nm·s/rad, rad/s)로 두고 tensor API(`set_dof_gains`, `set_dof_max_velocities`,
    `set_dof_max_efforts`, `set_dof_armatures`)로만 적용. USD 원본은 수정하지 않음
  - 적용 모듈 `isaacsim/scripts/robot_drive.py` 를 2·3단계 씬 스크립트도 그대로 쓴다. 관절 이름이 없거나 값이 비었거나 mimic 에 gain 이 있으면 에러, 적용 후 읽어서 확인
  - 테스트 씬은 1-4 와 같음 (최상위 prim z = 0.762 m, fixed base, `sleep_threshold: 0`)
  - armature 는 관절별 항목, 기본 0 (UR 회전자 관성 비공개, 튜닝 파라미터, 9단계 실물 응답으로 보정)
- **gain 시작값** (`isaacsim/scripts/compute_gain_seed.py`): K = I·ω², D = 2·ζ·I·ω (ζ = 1, 임계감쇠)
  - I = 무작위 300 자세(드론 무게 포함 경우까지) 중 최대 질량행렬 대각값 → 어느 자세에서도 ζ ≥ 1
  - ω = max(정착시간 기준 19.4, **추종 지연 기준 100**, 중력 처짐 기준). 추종 지연 실측 ≈ 2.47/ω + 10.7 ms (2회차). 중력은 드론 1.5 kg 무게중심이 공구 축에서 **5 cm 벗어난 최악 방향**
    (중력 토크가 무게중심 위치에 선형이라 세 위치에서 재서 모든 방향의 최악값을 정확히 구함)
  - 관절별 최악 경우는 `isaacsim/config/gravity_worst_cases.yaml` 로 저장 → B 시험이 사용
- 팔 6관절 stiffness/damping 튜닝
  - 방법: 관절마다 목표 각도를 갑자기 바꿔보고(스텝 입력), 얼마나 빨리 도달하는지, 지나치지 않는지(overshoot), 중력에 처지지 않는지를 재서 값을 정한다
  - 목표: 실물 UR5 처럼 명령을 빠르고 정확하게, 흔들림 없이 따라가는 것
  - 팔 최대 속도 π rad/s (UR5 관절 180°/s)
  - 큰 스텝(0.2·1.0 rad)에서 shoulder_lift overshoot 는 토크 포화(150 Nm − 중력)로 감속이 모자라서 생김. 50 Hz 텔레옵 명령 사이 변화는 최대 0.063 rad 이므로
    판정은 0.05 rad 스텝으로 하고 큰 스텝은 참고로 남긴다 (0.05 rad 통과 시 ζ 는 그대로)
- `rh_r1_joint` (그리퍼)
  - 실물: 이진 명령 + 전류 기반 위치 제어(mode 5). Profile Velocity 로 일정 속도로 움직이다가 물체에 닿으면 전류 한계(400 mA) 힘으로 누름
  - sim: **stiffness 는 낮추지 않고, drive 목표값을 0.516 rad/s(= 1.1351 rad / 2.2 s)로 goal 까지 옮긴다 (목표값 이동)**.
    목표값이 물체 너머까지 계속 가므로 닿으면 위치 오차가 커져 maxForce 로 누른다 (실물 Profile Velocity 와 같은 원리)
    - 공용 함수 `robot_drive.step_toward` / `robot_drive.GripperProfile`. 3단계 sim 그리퍼 브리지도 같은 것을 쓴다
    - **goal 이 바뀌면(열기↔닫기) 목표값을 현재 실제 손가락 위치에서 새로 시작한다** (실물 Dynamixel 과 같음).
      이전 목표값에서 이어 가면, 물체를 잡고 있다가 열 때 목표값이 물체 위치까지 되돌아오는 동안 손가락이 멈춰 있다 (0.6 rad 에서 잡았을 때 약 1.04 s)
    - **관절 속도 제한(0.516)은 쓰지 않는다**: 1회차 시험에서 mimic 과 함께 점성 저항(약 3 Nm·s/rad)처럼 동작해 maxForce 2.28 Nm 로 열리지 못함.
      속도 한계는 USD 원래 값(6.5 rad/s)
    - **가속 구간은 지금은 넣지 않는다 (등속)**. 실물 재측정(Profile Acceleration 300)과 차이가 크면 사다리꼴 속도 프로파일을 검토
  - maxForce 는 400 mA 환산값으로 따로 정함. 임시 파지력 20 N (사양 선형 환산, 실물에서 0.8 kg 드론 파지 성공) → 1-6 당김 시험으로 보정
  - stiffness 기준: 목표에 떨림 없이 도달하고, 물체에 닿았을 때 `stiffness × 남은 오차 ≥ maxForce` (실물처럼 힘 한계로 누르는 상태)
  - 실물 노드 설정 (원시값, **단위 미확인** → RH-P12-RN(A) e-Manual 컨트롤 테이블로 확인): Profile Velocity 1000, Profile Acceleration 300, Goal Current 400 mA
- **mimic 조인트 3개(`rh_r2`, `rh_l1`, `rh_l2`)에는 목표값·gain 을 주지 않는다** (DOF 로 잡히지만 mimic 이 따라가게 둠)
- 설정은 USD 원본이 아니라 스크립트/씬 레이어에서 적용
- **시험 (`isaacsim/scripts/tune_drives.py`, 리포트는 `isaacsim/reports/`)**
  - A. 스텝 응답: 홈 자세에서 관절별 **+0.05 rad (판정)**, +0.2·+1.0 rad (참고). 상승시간, overshoot, 2% 정착시간, 정상상태 오차, 최대 속도, 다른 관절 흔들림
  - B. 중력 유지: `gravity_worst_cases.yaml` 의 관절별 최악 자세 + 드론 1.5 kg (무게중심 공구 축에서 3·5 cm, 최악 방향) → 정상상태 오차.
    드론은 충돌 없는 별도 강체를 그리퍼 base 에 FixedJoint(`excludeFromArticulation`)로 붙임. 바닥 없는 씬 (팔이 바닥에 닿지 않게)
  - C. 텔레옵식 추종: 50 Hz zero-order hold 명령으로 부드러운 궤적 → 추종 오차(RMS·최대)와 지연
  - D. 그리퍼 동작 시간: 목표 **열림·닫힘 각각 약 2.2 s (임시, 재측정 예정)**. + 잡은 상태에서 열기: 명령부터 움직이기 시작할 때까지 지연 < 0.1 s
    (collider 가 아직 없어 `rh_r1_joint` 상한을 0.6 rad 로 임시로 낮춰 물체 대신 막음, 메모리에서만)
    **"완전 열림/닫힘 시간" 정의 = 명령 순간부터 위치 변화가 멈출 때(최종값의 2% 이내)까지.** sim D 시험과 실물 재측정 모두 이 정의를 쓴다
  - 설정 비교: A~C 는 기본·ff·공식 세 설정, D 는 기본 설정. GUI 로 보기: `--headless` 없이 `--realtime`
- **합격 기준 (A~C)**: overshoot < 2%, 0.05 rad 스텝 정착시간 < 0.5 s, 드론 무게 포함 정상상태 오차 < 0.1°, C 지연 < 40 ms, 관절 속도 ≤ π rad/s, 떨림·NaN 없음
- **1회차 시험 결과 (2026-09-30, 기본 vs ff)**: 기본 설정이 B(12/12)·C(5/6, shoulder_pan 지연 113 ms) 우세, ff 는 B 0/12·C 0/6.
  그리퍼는 관절 속도 제한 방식에서 열림 실패. 시험 중 articulation 수면(그리퍼 멈춤)과 시험 하중이 articulation 에 흡수되는 문제를 찾아 고침
- **2회차 (기본 vs ff vs 공식, 그리퍼 목표값 이동)**: 기본 A 6/6, B 12/12, C 5/6 (shoulder_pan ω 70 → 지연 42 ms), D 통과
  - **공식 UR5 값(`drive_gains_official.yaml`)은 채택하지 않음**: B 12/12·C 6/6(지연 18 ms)은 통과하지만 damping 이 작아(shoulder_lift ζ ≈ 0.2)
    0.05 rad 스텝 overshoot 가 최대 58% (shoulder_pan 9.9%, elbow 6.1%) → A 3/6
  - **ff 방식(`drive_gains_ff.yaml`)은 탈락**: 드론 하중을 보상하지 못해 B 0/12 (손목 처짐 2~9°), ω = 30 이라 C 지연 약 80 ms
  - **그리퍼 속도 읽기 값은 판정에서 제외**: D 에서 `rh_r1_joint` 속도가 0.25·0.44 rad/s 로 읽히지만 실제 평균은 약 0.53 rad/s
    (2.15 s 에 1.135 rad). mimic 조인트 속도 읽기 문제로 추정. 판정은 위치·시간으로만 한다
- **3회차 (기본, shoulder_pan ω 100, 그리퍼 현재 위치에서 재시작)**: A 6/6 (0.05 rad, overshoot 최대 1.2%), C 6/6 (지연 19~34 ms),
  D 닫힘 2.15 s·열림 2.15 s, 잡은 상태에서 열기 지연 0.017 s → **전부 통과**. B 는 2회차 결과(12/12)와 같은 설정이라 다시 돌리지 않음
  (shoulder_pan 은 B 대상 토크가 0 이라 영향 없음)
- **알려진 위험: 높은 K** (stiffness 가 커서 생기는 문제와 대응)
  1. **충돌 시 팔이 밀리지 않고 최대 토크로 밀어붙임**: 드론·테이블·자기 몸에 닿아도 위치 오차 × K 가 곧바로 maxForce(150/28 Nm)까지 올라감.
     실물 UR 은 힘·토크가 한계를 넘으면 보호 정지(protective stop)하지만 sim 에는 없음
     → **2·3단계에서 보호 정지 흉내 구현**: 관절 토크(drive 힘)·접촉력이 기준을 넘으면 팔 목표를 현재 위치에 고정해 멈추고,
       에피소드에 표시(메타데이터 `protective_stop`, 발생 시각·관절·값). 이런 에피소드는 학습 데이터에서 거르거나 따로 표시
     → **UR5(CB3) 보호 정지의 실제 기준값(힘·토크 한계, 안전 설정 기본값)은 UR 문서(User Manual / Safety 설정)에서 확인 필요**. 확인 전까지는 임시값
  2. **큰 명령에 오버슈트** (A 시험 0.2 rad 스텝에서 shoulder_lift 13.6%): 명령 제한기로 막음 — 이미 계획됨
     (3단계 리셋 서비스의 부드러운 홈 복귀, 텔레옵 명령의 한 스텝 변화량 제한)
  3. **physics 주기·엔진이 바뀌면 불안정 가능** (ω·dt 가 1~3.5 인 관절이 있음, 1/120 s·PhysX 기준으로 맞춘 값):
     physics 주기(`PHYSICS_DT`), 엔진(PhysX ↔ Newton), solver 반복 횟수를 바꿀 때마다 **1-5 시험(A~F)과 check_articulation 을 다시 돌린다**
- **GUI 육안 확인 (2026-09-30, `--realtime`, A·B·C·D 각각)**: 0.2 rad 스텝에서 shoulder_lift 가 살짝 넘어갔다 돌아오는 것(알려진 참고 항목)만 보이고
  떨림·이상 움직임 없음 → **1-5 drive 설정 확정 (`drive_gains.yaml`)**

### 1-6. 손가락 collider·마찰·파지 테스트
- 손가락(r2, l2) collider: convex hull 이 부정확하면 convex decomposition
- 고마찰 physics material 적용
- 파지 테스트: 작은 박스 + **드론 무게급(≈ 1.5 kg) 박스**. 무게중심이 잡은 곳에서 벗어나면 손가락 사이에서 박스가 돌아갈 수 있음.
  특히 그리퍼 max force 를 실물 수준으로 낮춘 뒤 확인
- **당김 시험 (실물과 같은 기준)**: 잡은 물체를 당겨서 **미끄러지기 시작하는 힘**을 잰다
  - 실물: 러기지 스케일로 잡은 물체를 당겨 미끄러지는 순간의 값 (측정 예정)
  - sim: 잡은 물체에 당기는 힘을 천천히 늘려(램프) 물체가 손가락에 대해 미끄러지기 시작하는 힘을 기록. 당기는 방향·물체·잡는 위치는 실물 측정과 맞춘다
  - 미끄럼 힘은 파지력 × 마찰계수라서 측정 하나로는 둘을 분리할 수 없음 → ~~마찰계수는 재질 기준으로 먼저 정하고 maxForce 로 맞춘다~~
    **(2026-09-30 변경) maxForce 는 전류 환산값(400 mA → 2.28 Nm)으로 고정하고 마찰계수로 맞춘다.**
    이유: armature 0.01 이후 sim 손가락 수직력이 손가락당 약 22~25 N 으로 전류 환산 예상(약 20 N)과 비슷 → 누르는 힘은 실물 수준이고,
    부족한 것은 PhysX 유효 마찰(설정 0.4 의 약 65%). maxForce 를 부풀리면 손가락끼리 누르는 힘 등 다른 동작도 같이 세진다.
    마찰계수는 재질값이 아니라 "sim 유효 마찰 보정값"이 된다. 사양 170 N 이 손가락당인지 합계인지(2.28 이 2 배 달라짐)는 이 방식으로도 분리되지 않는 가정으로 남음

**1-6 진행 계획 (2026-09-30 확정)** — 시험 스크립트 `isaacsim/scripts/grasp_tests.py`, 설정 `drive_gains.yaml` 의 `contact`
- 점검 (설정 바꾸지 않음)
  - K1 손가락 충돌 형상: 파지면(안쪽 끝 0.5 mm 이내 면)을 원래 메시와 convex hull 로 비교. 평평한 박스 면에 처음 닿는 위치는
    convex hull 과 원래 메시가 수학적으로 같다(한 방향의 가장 바깥 점은 볼록 껍질과 같음) → 다른 것은 접촉 면의 넓이·모양뿐.
    넓이가 크게 다르거나(파지면이 오목) 차이가 1 mm 넘으면 손가락에도 convex decomposition
  - K2 접촉 거리: 손가락 contact offset·rest offset 의 USD 값과 PhysX 가 실제로 쓰는 값을 기록. 문제가 보일 때만 바꿈
- 재질 (임시, 실물 당김 시험 후 보정): 손가락 = 알루미늄, 박스 = PLA 로 가정.
  - 알루미늄–PLA 마찰 정지 0.4 / 운동 0.3 (금속–플라스틱 일반값 0.3~0.5 의 중간), 반발 0
    → **2026-09-30 0.5/0.4 → 0.6/0.5 로 올림 (임시, sim 유효 마찰 보정값)**. 아래 결과의 마찰 조합 비교 참고
  - 손가락·박스 재질에 **같은 쌍 값**을 넣고 합치는 방식은 **min**.
    이유: 알고 있는 것은 "알루미늄–PLA 쌍" 값 하나 → 두 재질에 같은 값을 넣고 min 이면 쌍 값이 그대로 나온다.
    PhysX 는 두 재질의 방식이 다르면 우선순위가 높은 쪽(average < min < multiply < max)을 쓰므로, 기본 재질(average, 0.5)인
    받침·바닥과 닿아도 min 이 적용돼 손가락 마찰이 설정값보다 커지지 않는다 (파지력을 과대평가하지 않는 쪽).
    multiply 는 0.4 × 0.4 = 0.16 이 되어 쌍 값이 아니고, max 는 과대평가
- 시험 물체
  - G1: 40 mm 정육면체, 0.2 kg
  - G2: 박스. 손가락이 잡는 방향 폭 60 mm (그리퍼 최대 열림 약 106 mm 안쪽), 긴 방향 160 mm, 높이 100 mm.
    무게 0.3·0.5·0.8 kg (0.8 은 참고, 2026-09-30 0.8·1.5 에서 변경). 무게중심을 박스 안에서 긴 방향(잡는 축에 수직, 수평)으로 0·3·5 cm 옮겨서,
    들어 올린 뒤 손목 ±30° 회전 시 박스가 손가락 사이에서 돌아가는지 본다 (회전 < 2°, 미끄러짐 < 1 mm)
  - G3 당김, G4 정지도 G2 박스로 한다. G3 박스 무게 = 실물 당김 시험 박스의 실제 무게 (재기 전 임시 0.3 kg, `grasp_tests.G3_MASS`),
    G4 = 0.5 kg
  - **무게를 낮춘 이유**: 실물은 24 V·400 mA 로 드론(0.8 kg)을 **잡기만 했고 들어 올리지는 않았다**. 들어 올렸을 때 버티는지는 확인된 적이 없으므로
    무거운 박스를 반드시 들어야 하는 기준은 두지 않는다. 실제로 버티는 힘은 실물 당김 시험으로 정한다
  - **봉·암처럼 원통을 잡게 되면 G2 회전 시험을 그 형상으로 다시 한다**
- 합격 기준: G1 미끄러짐 < 1 mm·떨림 < 0.1 mm / G2 미끄러짐 < 1 mm·잡는 축 회전 < 2° / G3 미끄럼 힘 (실물 당김 값과 비교) /
  G4 관절 떨림 < 0.01°·박스 떨림 < 0.1 mm. ~~0.8 kg 은 반드시 통과 (실물 400 mA 로 파지 성공)~~ → 삭제 (실물은 들어 올림 미확인).
  무게 기준은 실물 당김 시험 후 다시 정한다
- maxForce 는 2.28 Nm (400 mA 환산) 고정. 마찰계수는 조합 비교(`grasp_tests.py --frictions`) 후 임시값, 실물 당김 시험으로 확정.
  실물 전류를 바꿀 때 sim 값 바꾸는 법은 `docs/gripper_force.md`
- **실물 당김 시험 절차**: sim G2 와 같은 크기의 PLA 박스(60 × 160 × 100 mm)를 400 mA 로 잡고, 공구 축을 따라 손가락 밖으로
  러기지 스케일로 당겨 미끄러지는 순간의 최댓값을 5 회 재서 평균 낸다.
  - sim G3 와 같게 공구가 아래를 향한 자세에서 아래로 당긴다. 그러면 미끄럼 힘 = 스케일 값 × 9.81 + 박스 무게
  - 박스 무게·잡은 위치(손가락 끝에서 박스 윗면까지)를 함께 기록. 박스 무게는 `grasp_tests.G3_MASS` 에 넣는다
  - 결과로 maxForce 2.28 에서 실물과 같은 미끄럼 힘이 나오는 마찰계수를 골라 `contact.finger_material`·`object_material` 을 바꾼다
    (합치는 방식 min 이라 둘 다. 2026-09-30 전략 변경 전에는 maxForce 를 고르기로 했었음)

**1-6 결과 (2026-09-30, `grasp_tests.py`)**
- K1 손가락 파지면: 원래 메시는 파지면 아래쪽 절반(링크 z 2~19 mm, r2 관절 쪽)이 **3.9 mm 파여 있고**, 실제 파지면은 z 20~40 mm (26 × 20 mm).
  convex hull 은 이 홈을 메우지만 메운 면이 파지면 평면보다 평균 0.06~0.17 mm(최대 1.1 mm) 뒤에 있어 평평한 박스 면에는 닿지 않음.
  K3 PhysX 접촉점도 convex hull 그대로 z 20~40 mm 에만 생김 → **박스 파지는 convex hull 유지 (decomposition 안 함)**.
  **봉·암처럼 원통을 잡으면 이 홈에 원통이 들어갈 수 있으므로(Ø25 봉, 폭 17 mm 홈에 약 3.3 mm) 손가락 decomposition 을 다시 확인**
- K2: contact offset PhysX 자동값 1.363 mm (= 2·g·dt², dt 1/120), rest offset 0. 손가락·박스 재질 0.4/0.3 적용 확인. 바꾸지 않음
- 손가락 기구학(URDF 식) vs sim: 0.07 mm 이내. 박스 폭 40 mm 에서 닿는 각도 예상 43.80° / 실제 43.81°
- **그리퍼 힘 전달 문제**: armature 0 에서 박스를 조이면 maxForce 2.28 Nm 중 **약 0.58 Nm(26%)만 전달**
  (손가락 수직력 5.8 N × 2, 가상일 = 수직력 합 × 파지면 이동비 50.8 mm/rad, PhysX projected joint force 합과 일치).
  drive 설정·mimic(네 관절 각도 0.003° 이내)·관절 마찰은 정상 → **solver 수렴 문제**: 위치 반복 32 → 64 → 128 → 255 에서
  26 → 33 → 68 → 100%, 계산 8.6 → 12.6 → ? → 34 ms/step (120 Hz 실시간 8.3 ms 불가). 속도 반복은 효과 없음
  - 손가락별 구동(rh_l1 을 mimic 대신 drive, rh_l2 → rh_l1 mimic) 시험: 25% 그대로, G1 미끄러짐 20.8 mm → 되돌림
  - **armature (rh_r1_joint 에만) 로 해결**: 손가락 묶음 관성 3.78e-4 가 너무 작아 박스(0.8 kg)를 양쪽에서 조일 때 수렴이 느림.
    armature 는 관절 공간 관성에 더해지는 값 (모터 회전자 관성이 감속기를 거쳐 보이는 값), 계산 시간 변화 없음
  - 값 비교 (D = 2√(K·(3.78e-4 + armature)) 로 ζ = 1 유지):

    | armature | D | 전달률 (maxForce 2~6 Nm) | 당김 곡선 | D 시험 닫힘 / 열림 / 반응 |
    |---|---|---|---|---|
    | 0 | 0.2626 | 6 Nm 에서 22%, 4 Nm 이하는 들어 올리다 놓침 | – | 2.13 / 2.13 / 0.017 s |
    | 0.005 | 0.9904 | 101~117% | 들쭉날쭉 (3 Nm 11.9 N < 2.28 Nm 16.0 N), 좌우 불균형 | 2.14 / 2.14 / 0.033 s |
    | **0.01** | **1.3758** | 104~112% | 거의 단조 증가 (13.1·12.9·17.1·18.5·26.4 N) | 2.15 / 2.15 / 0.033 s |
    | 0.02 | 1.9279 | 104~112% | 단조 증가 | 2.15 / 2.16 / 0.042 s |

  - **결정: armature 0.01, D 1.3758** (수렴·안정 전달·D 약 2.13 s 유지를 만족하는 가장 작은 값). mimic 3 관절 armature 는 0:
    실물은 모터 하나가 링크로 네 관절을 움직이므로 회전자 관성은 구동 관절에 한 번만 붙음. mimic 에도 나눠 넣어도(0.0025 × 4) 전달률 97% 로 같음.
    armature 는 관성이라 mimic gain 0 규칙과 별개 (mimic 에 넣어도 K·D 는 0 그대로 확인)
  - 변경 후 tune_drives D·E·F 재확인 (모두 통과): D 닫힘·열림 2.15 s, 반응 0.033 s / E 떨림 0.00001°, 6.76 ms/step /
    F 빈손 닫기 64.215° (이전 64.233°), 손가락 떨림 0.00007°
  - 참고 (6.1.0 에셋·공개 설정): Isaac Sim 예제 Franka·UR10e+Robotiq 2F-140 에셋, Isaac Lab UR10e+Robotiq 2F-85/140 설정 모두 armature 0.
    Robotiq 예제는 maxForce 200 Nm 로 힘을 크게 줘서 잡음 (실물 파지력 보정에는 못 씀). Isaac Sim 매뉴얼은 mimic 이 많은 그리퍼에 위치/속도 반복 64/4 권장.
    특정 armature 값을 권장한 자료는 없음
- **마찰 — PhysX 유효 마찰이 설정값보다 낮음** (armature 0.01): 미끄럼 힘 / 수직력 합이 0.30(2 Nm) → 0.22(6 Nm) 로 설정 정지 마찰 0.4 보다 낮고
  파지력이 커질수록 떨어짐. 들어 올리는 동안 1~5 mm 미끄러짐 (6 Nm 은 마찰 여유가 무게의 6 배인데도). maxForce 2.28 에서 미끄럼 힘 약 13 N → 1.5 kg(14.7 N) 은 놓침 (G4).
  - 대응: maxForce 는 400 mA 환산 고정, 마찰계수를 "sim 유효 마찰 보정값"으로 올려 맞춘다 (위 보정 전략). PhysX 마찰 설정 조사는 보류
  - 마찰 조합 비교 (`grasp_tests.py --frictions`, maxForce 2.28, 당김 박스 0.8 kg, G4 1.5 kg, 2026-09-30):

    | 정지/운동 | 유효 마찰 | 유효 ÷ 설정 정지 | 들어올림 미끄러짐 | 미끄럼 힘 | G4 1.5 kg |
    |---|---|---|---|---|---|
    | 0.4/0.3 | 0.256 | 0.64 | 3.55 mm | 12.9 N | 놓침 |
    | 0.5/0.4 | 0.337 | 0.67 | 2.00 mm | 16.7 N | 놓침 (11 mm) |
    | **0.6/0.5** | 0.386 | 0.64 | 1.75 mm | 19.1 N | 놓침 (3.3 mm) |
    | 0.7/0.6 | 0.424 | 0.61 | 2.29 mm | 21.1 N | 놓침 (1.07 mm) |
    | 0.8/0.7 | 0.490 | 0.61 | 0.30 mm | 24.4 N | 들고 있음 (0.54 mm) |
    | 1.0/0.9 | 0.618 | 0.62 | 0.07 mm | 30.7 N | 들고 있음 (0.19 mm) |
    | 0.6/0.6 | 0.416 | 0.69 | 1.28 mm | 20.6 N | 놓침, 떨림 0.82 mm |
    | 0.8/0.8 | 0.579 | 0.72 | 0.07 mm | 28.8 N | 들고 있음 (0.66 mm) |
    | 1.0/1.0 | 0.675 | 0.68 | 0.08 mm | 33.6 N | 들고 있음 (0.52 mm) |

    - G1 은 9 조합 모두 통과. 유효 마찰 ≈ 설정 정지 마찰 × 0.6~0.7 로 일정, 미끄럼 힘은 마찰계수에 거의 비례
      → 실물 미끄럼 힘에서 역산: 설정 정지 마찰 ≈ 실물 유효 마찰(미끄럼 힘 / 수직력 합, 수직력 합 약 50 N) ÷ 0.63
    - 들어 올리는 동안 미끄러짐 < 1 mm 는 정지 0.8 부터
    - **결정 (임시): 0.6/0.5** — 실물 당김 값 전까지 조금 올려 둠 (처음 0.5/0.4 → G2~G4 를 GUI 로 본 뒤 조금 더 현실적인 쪽으로).
      실물 값이 나오면 위 비율로 다시 고른다
- **G2~G4 (마찰 0.5/0.4 일 때, maxForce 2.28, armature 0.01, 2026-09-30)**
  - G2: 9 경우 모두 불합격 — 손목 ±30° 에서 박스가 잡는 축 둘레로 돎 (0.3 kg·무게중심 0 에서도 약 10°, 0.5 kg·5 cm 약 40°, 0.8 kg·3~5 cm 는 떨어짐).
    GUI 로 본 결과 실물과 비슷해 보임 (400 mA 의 약한 파지력) → **회전 마찰(PhysxCollisionAPI torsionalPatchRadius / minTorsionalPatchRadius, 기본 0) 설정은 보류**.
    실물 당김 시험 때 손목을 기울여 박스가 도는지도 보고 필요하면 다시 검토. G2 합격 기준(회전 < 2°, 미끄러짐 < 1 mm)도 실물 시험 후 다시 정함
  - G3 (0.3 kg): 전달률 97% (손가락당 21~22 N, maxForce 로 누르고 있음), 들어 올리는 동안 0.81 mm, 미끄럼 힘 11.6 N (유효 마찰 0.267)
  - G4 (0.5 kg, 5 초): 통과 — 들고 있음, 미끄러짐 0.49 mm, 팔 떨림 0.0027°, 박스 떨림 0.013 mm, 전달률 108%
  - 참고: Robotiq 에셋(Isaac Sim 예제 2F-140: K 2149.7 Nm/rad, maxForce 200 Nm, armature 0, 반복 64/4)은 실물보다 훨씬 센 힘으로 잡는 모델이라
    armature 가 필요 없음. 우리는 실물 400 mA 수준의 약한 힘에 맞추므로 armature 와 마찰 보정이 필요하다

- **self-collision: 켬 (2026-09-30 결정)** — `drive_gains.yaml` 의 `articulation.self_collision: true`, `collision_filter_pairs: []`
  (씬 레이어에서 적용, 로봇 USD 원본은 그대로. ff·official 비교 설정은 끈 채로 둠)
  - 사전 점검 (`isaacsim/scripts/check_self_collision.py`, 7 자세 × 그리퍼 열림·닫힘, 충돌 형상 간 거리, 가까움 기준 2 mm):
    - 관절로 직접 연결된 쌍은 USD 에서 이미 충돌이 꺼져 있음 (joint collisionEnabled = false). 그 외에 항상 붙어 있는 쌍 없음 → **충돌 제외 쌍 0 개**
    - 그리퍼를 끝까지 닫으면 좌우 손가락 4 쌍(l1-r1, l1-r2, l2-r1, l2-r2)이 1.6~2.0 mm 겹침 → 켜면 실제로 닿음 (의도한 동작)
    - 공구가 팔 쪽을 향하는 자세(홈에서 wrist_1 = 0)에서 forearm - 카메라 1.1 mm 겹침 → 실물에서도 닿을 수 있는 자세
    - B 시험 최악 자세(무작위 추출) 일부가 자기 충돌 자세: wrist_3@3·5cm, wrist_2@5cm 는 수십 mm 파고듦, wrist_1@3·5cm·wrist_2@3cm 는 base-upper_arm 약 2.7 mm
  - 켠 상태 / 끈 상태 비교 (`tune_drives.py`, 설정 덮어쓰기 `drive_gains.yaml@articulation.self_collision=true`):
    - D·E·F·A·C 모두 켠 상태에서 통과, 결과는 끈 상태와 같거나 더 현실적
    - F (빈손으로 끝까지 닫기): 켠 상태에서 손가락이 **64.233° 에서 서로 닿아 멈춤, 떨림 없음** (끈 상태는 관절 한계 65.04° 까지 겹쳐 들어감)
    - `check_articulation.py`: 켠 상태 8/8 PASS
    - 계산 시간: 끈 상태 대비 **약 +1%**
    - B 는 비교하지 않음: 자기 충돌 자세가 섞여 있어, 가능한 자세만으로 최악 경우를 다시 찾은 뒤 따로 결정
- **가능한 자세 기준 B (2026-09-30)**: `compute_gain_seed.py --collision-free` 로 자기 충돌 자세를 버리고 가능한 자세 302 개만으로
  최악 중력 경우를 다시 찾아 `isaacsim/config/gravity_worst_cases.yaml` 을 교체 (823 번 뽑아 521 개 버림).
  K 는 기존 대비 −0.6 ~ +1.1% 로 거의 같아 `drive_gains.yaml` 은 그대로. 새 경우로 B 12/12 통과 (최대 오차 0.067°, self-collision 켬)
  - B 는 드론 무게를 별도 강체 대신 **그리퍼 base 링크에 매 스텝 가하는 외력**으로 바꿈 (씬을 설정마다 한 번만 만듦 → 설정 하나 약 33 s)
- **forearm convex decomposition (2026-09-30)**: `collision_shapes.convex_decomposition` (links: [forearm_link], 씬 레이어에서 적용,
  그 링크의 instance 만 풀어 approximation 을 convexDecomposition 으로. 로봇 USD 원본은 그대로)
  - 이유: forearm 의 convex hull 이 오목한 부분을 메워, 원래 메시로는 겹치지 않는 자세 62 개가 겹친다고 판정됨 (주로 forearm–wrist_2)
  - 확인 (`check_self_collision.py --skip-distances --physx-poses`, 순간이동 직후 PhysX 접촉 보고로 파고듦 > 0.5 mm 인 자세 수):
    - convex hull 때문에만 겹침 62 개: decomposition 끔 58 → 켬 **2** (남은 2 개는 upper_arm 쪽 쌍, forearm 과 무관)
    - 실제 겹침 274 개: 끔 270 → 켬 263 (**8 개는 켜면 PhysX 가 겹침을 보고하지 않음**)
    - **놓친 8 개 확인 (원래 메시 기준 깊이, `--physx-poses` 재실행 2026-09-30)**: 모두 forearm 과의 쌍,
      원래 메시 깊이 **최대 1.31 mm** (forearm–wrist_2 0.38·1.31·0.59 mm, 나머지 forearm–카메라·마운트·브래킷·그리퍼 쌍은 0.00 mm = 거의 맞닿음).
      convex hull 로는 1.8~29.8 mm 겹쳐 보였지만 실제로는 스치는 수준 → **판단 기준(모두 3 mm 이하) 충족, decomposition 켬 확정.
      해상도(볼록 조각 수·voxel) 올리기는 하지 않음**
    - 재실행 때 분류: 829 번 뽑아 가능 302 / convex hull 때문에만 63 / 실제 275 / 판정 불가 189 (앞선 실행과 경계 몇 개만 다름).
      PhysX 파고듦: hull 때문에만 60 → 2, 실제 270 → 263, 가능 0 → 0. 순간이동 시험 스텝당 14.54 → 15.22 ms (+4.7%)
      분류 결과는 `isaacsim/reports/pose_classes_*.npz` 로 저장, 다시 돌릴 때 `--reuse-classes latest`
    - 가능한 자세 302 개: 끔 1 → 켬 0
  - E·F: 켜기 전과 같음 (가만히 떨림 0.00001°, 빈손 닫기 64.233°·떨림 0.00003°). 계산 시간 약 +3~5% (E 6.79 → 7.00 ms/step, 전체 7.00 → 7.35 ms/step, 1 회 측정)
- **메모 — base–upper_arm 판정 불가 자세 (약 187 개)**: `base_link_inertia` 원래 메시가 닫혀 있지 않아(watertight 아님) convex hull 겹침이
  실제 겹침인지 판정하지 못함. upper_arm 이 base 쪽으로 크게 내려오는 자세라 공중 드론 파지에서는 거의 쓰지 않으므로 지금은 넘어감.
  필요하면 base 메시를 닫거나 decomposition 을 base 에도 적용해 다시 확인
  - 끄는 기준 (하나라도 해당하면 false 로 되돌리고 원인·겹친 쌍을 여기에 기록): 가만히 있을 때 관절 0.01° 이상 흔들림이나 링크 튐 /
    끈 상태에서 통과하던 A~D 항목 실패 / 실물에서는 닿지 않을 자세에서 막힘 / 계산 시간 크게 증가

### 1-7. 손목 카메라
- `wrist_camera_color_optical_frame` 아래 Camera prim, X 축 180° 회전 (optical: +z 전방/+y 아래 ↔ USD camera: -z 전방/+y 위)
- `horizontalAperture=20.955`, `verticalAperture=15.716`, `focalLength ≈ fx*20.955/640 ≈ 20.14` (fx≈615 px, D435 color 640x480 전형값. 실물 `camera_info` 로 확인)
- `cam_tilt` 결정: README 기구학 계산상(윗면 바깥쪽 기준) tilt 0 이면 열린 손가락이 화면에 0% (닫힘 6~7%), 15~20° 면 닫힘 시 94~100%.
  손가락은 영상 아래쪽에서 들어옴

**1-7 결과 (2026-09-30)** — 설정 `isaacsim/config/wrist_camera.yaml`, 씬 레이어 `test_scene.add_wrist_camera`, 확인 `isaacsim/scripts/check_wrist_camera.py`
- 실물 D435i(serial 843112074130, 이전 third view 카메라를 손목으로 옮김) color 640x480 camera_info: fx 618.55, fy 618.96, cx 332.28, cy 239.38, 왜곡 0
  (1280x720 값에서 세로 기준 축소·좌우 자름으로 추정한 값과 일치). 가로 시야각 54.7°, 세로 42.4°
- intrinsics 확인: 카메라 좌표계의 알려진 점 3 개 → 렌더링 픽셀 중심 vs OpenCV 투영 **최대 0.22 px**.
  USD 환산에서 가로 aperture offset 부호가 반대(−), 픽셀 중심 규약 0.5 px 보정 필요함을 표식으로 찾아 반영
- tilt 비교 (Camera prim 자세만 돌린 이상적 회전, 회전축 = 탭의 M3 구멍 두 개를 잇는 선, 양수 = 렌즈를 공구 축 쪽으로):

  | tilt | 박스 (파지 자세) | 왼쪽 손가락 열림 / 닫힘 | TCP (u, v) |
  |---|---|---|---|
  | 0° | 38% | 0 / 0.4% | (535, 598) 화면 밖 |
  | 10° | 44% | 5.8 / 7.3% | (519, 446) |
  | 15° | 37% | 10.7 / 8.5% | (515, 377) |
  | 20° | 29% | 15.0 / 10.0% | (512, 311) |

  TCP 좌표는 README 기구학 예측과 일치 (u 차이 13 px 는 README 가 cx 320 을 썼기 때문). 오른쪽 손가락은 모든 tilt 에서 거의 안 보임
- **손목 카메라 시점의 문제 (다운받은 마운트, tilt 0)**
  - 증상
    - 화면 가운데(u = 320)가 잡는 물체가 아니라 **물체 바로 옆**을 봄. 파지 자세에서 박스 왼쪽 모서리가 u ≈ 345, 박스는 화면 오른쪽 절반을 차지
    - 그리퍼 중심(TCP)이 화면 오른쪽 아래 **밖** (u ≈ 535, v ≈ 598). 손가락은 거의 안 보임 (열림 0%, 닫힘 0.4~0.7%), 오른쪽 손가락은 어떤 tilt 에서도 안 보임
    - 화면 회전(roll)은 아님: 정지 자세에서 바닥 격자선이 수직·수평에서 ±0.5~1.9° 이내 (원근 효과). G2 처럼 손목을 기울이는 동안에는 실제로 기울어 보임
  - 원인
    - 좌우 치우침: D435 color 렌즈가 카메라 몸체 중앙에서 가로 32.5 mm 옆. 마운트는 몸체를 그리퍼 중심에 맞추므로 렌즈 중심선이 그리퍼 중심에서
      32.5 mm 떨어져 박스 반폭(30 mm) 바깥을 지남. 손가락이 화면 좌우로 벌어지게 달려 있어(gripper_yaw 90°) 이 방향이 곧 화면 가로
    - 위아래 치우침: 카메라가 공구 축에서 57.5 mm 떨어진 탭에서 공구 축과 평행하게 봄 → 손가락과 TCP 가 화면 아래 밖
    - 카메라를 반대로(윗면 공구 축 쪽) 달면 좌우가 뒤집힘 → 실물 조립 방향 확인 필요 (아래 "실물 마운트가 생기면 확인")
  - 영향 (데이터 수집·학습)
    - 손목 영상만으로는 손가락 위치나 "잡았는지"를 보기 어렵다 (그리퍼 상태는 `observation.state[6]` 로 따로 들어가고, third view 카메라가 보완)
    - 잡을 물체가 화면 한쪽 가장자리에 치우쳐 보여, 물체와 손가락의 상대 위치(정렬) 정보가 영상에 적다
    - 드론 손잡이 봉(Ø25 mm)처럼 좁은 물체는 박스(60 mm)보다 더 화면 가장자리로 밀리거나 렌즈 중심선 밖에 놓일 수 있음 → 2단계 드론으로 다시 확인
  - 해결 후보 (마운트 CAD 수정 때 검토, sim 은 xacro 값만 바꿔 미리 확인)
    1. 카메라를 가로로 약 32.5 mm 옮겨 color 렌즈를 그리퍼 중심선에 맞춤 → 물체가 화면 가로 가운데
    2. tilt 10~15° 로 렌즈를 공구 축 쪽으로 숙임 → 손가락·TCP 가 화면 아래쪽에 들어옴 (10° 에서 박스가 가장 크게 보임, 15° 이상은 몸체가 많이 가림)
    3. 1 + 2 를 함께. 바꾸면 카메라 몸체 위치가 달라지므로 재import 후 check_self_collision 으로 forearm·손가락과의 간섭 확인
  - 지금은 실물 마운트가 없어 **다운받은 마운트(tilt 0) 그대로 두고 2단계로 진행**. 이 문제는 실물 마운트 출력 전에 다시 결정
- ~~**결정 (임시): cam_tilt 0 유지 (다운받은 마운트 그대로, 실물 시점과 일치)**. 기울인 마운트는 나중에 CAD 수정 때 다시 검토~~
  → **(2026-10-15) t10 마운트로 확정** (렌즈를 그리퍼 중심선으로 옆 −32.5 mm, 10° 기울임, 실물 출력 확인, 로봇 USD 재import). xacro `mount_variant` 로 고르며
  `cam_tilt`·`cam_xyz` 인자는 없앴다. 경과·회귀 결과는 5-B "손목 카메라 마운트"
- **실물 마운트가 생기면 확인**: 그리퍼를 끝까지 닫았을 때 실물 손목 영상에서 손가락 끝이 오른쪽 아래 가장자리에 보이는지
  (sim `tilt0_home_closed.png` 와 같은지). 다르면 카메라 방향(`cam_xyz`/`cam_rpy`)이나 `mount_yaw` 확인
- 보기: `check_wrist_camera.py --tilts 0 --hold` (GUI, 손목 카메라 창 추가), `grasp_tests.py --realtime --wrist-view` (파지 동작 중 손목 시점)
- **실물 파이프라인 메모**: d435i 를 손목으로 옮겨 실물 third view 카메라가 없어짐. `5_cameras.sh`, stage2 변환 스크립트는 아직 d435i = third view.
  카메라 역할 매핑(`config/cameras.yaml`) 정리 때 함께 바꾼다

### 1-8. 저장
- 로봇 USD 원본(`ur5_rh_p12_d435i.usda`)은 import 결과 그대로 둔다. 튜닝값은 스크립트/씬 레이어로

**완료 기준**
- [x] 구조 검증 스크립트(1-4) 8개 항목 PASS (1-6 재질·armature 적용 후 2026-09-30 재확인 8/8)
- [x] `/joint_command` 로 팔이 목표 자세에 안정적으로 도달 (떨림·폭주 없음) — 1-5 시험 A·C·E (ROS 토픽 연결은 3단계)
- [x] 그리퍼 0 → 1.1351 rad 열림/닫힘 동작, 좌우 손가락 대칭 (1-5·1-6 스크립트 설정으로 D·F, check_articulation 8 번)
- [x] 작은 박스(0.2 kg)를 잡아 들어 올려도 미끄러지거나 튀지 않음 (1-6 G1)
- [ ] (실물 당김 시험 대기) 무거운 물체의 파지 한계(미끄럼 힘)와 손목 회전 시 회전 기준은 실물 당김 시험 후 마찰계수와 함께 정한다 (PLAN 1-6).
  ~~≈ 1.5 kg 물체를 잡아 들어 올려도 돌아가지 않음~~ → 실물은 400 mA 로 잡기만 확인, 들어 올림 미확인이라 변경
- [x] 손목 카메라: tilt 0 으로 sim 손목 영상 확인, 0·10·15·20° 비교 (1-7). 실측 intrinsics 적용·확인 (0.22 px)
- [x] 손목 카메라 마운트 확정 — 2026-10-15 t10 (10°, 렌즈 옆 −32.5 mm). 실물 10° 출력으로 손목 영상의 좌우·위아래 방향이 sim 과 같음을 확인 (5-B "손목 카메라 마운트")

---

## 2단계: 씬과 드론

**목표**: 스크립트 한 번으로 재현되는 드론 파지 씬

**진행 순서 (2026-10-01 결정)** — 2단계는 **ROS 없이** 진행 (ROS 2 연동은 3단계 이후, Pegasus 방식 검토)
1. 데모 드론 (3DR Iris 외형 0.75 배, 아래) — 씬·공중 유지·파지 판정 파이프라인을 먼저 만든다
2. PX4 로 띄운 드론 (Pegasus Simulator 방식, PX4 SITL ↔ MAVLink 직접 연결. 6.0.1 포크 livealive7/PegasusSimulator 가 6.1.0 후보)
3. 다른 드론 모델 (STL → USD 에셋)
- 드론은 설정 yaml(`isaacsim/config/drone_*.yaml`) 하나로 교체되게 한다 (씬 스크립트 수정 없이)
- 처음에는 Isaac Sim 공식 `Quadcopter` USD 를 쓰려 했으나 폐기: 몸체가 지름 200 mm 원판이라 옆면을 잡을 수 없음,
  질량 단위 오류(density 1e6 배, 합계 251,537 kg)·drive 없는 수동 관절 ±30°·손잡이 없음. 에셋은 삭제 (출처: Isaac Sim 6.1 에셋 서버
  `Isaac/Robots_Multiphysics/IsaacSim/Quadcopter/quadcopter.usda`)

**실물 시나리오 (2026-10-01 확인)**: 비행 중인 드론을 잡는다 → 드론 모터 정지 → 로봇 팔이 드론을 내려놓는다.
- 실물 드론: 접이식 소형 쿼드콥터 (좁고 긴 몸체, 프로펠러 4개), **0.8 kg**, 치수 미측정
- **그리퍼가 아래에서 위로 다가가 몸체 옆면을 폭 방향으로 잡는다** → 무게중심이 두 손가락 가운데, 평평한 옆면이라 박스 파지(1-6)와 같은 조건
- 잡기 전에는 드론이 스스로 떠 있고, 잡은 뒤에는 드론 무게 전체가 그리퍼에 걸린다

**데모 드론 = 3DR Iris 외형 0.75 배 (2026-10-01 결정)** — 설정 `isaacsim/config/drone_iris.yaml` (기본값), 에셋 `isaacsim/assets/drones/iris/`
- 3DR Iris = 예전 PX4 Gazebo SITL 기본 기체 ("PX4 처럼 보이는" 외형, 2번 PX4 단계에서도 같은 외형). Pegasus Simulator 6.0.1 포크의 `iris.usd` 복사 (BSD-3, 출처는 README)
- 원본 허리 폭 약 105 mm 는 그리퍼 완전 열림 107 mm 로 잡을 수 없음 → **0.75 배** (허리 81 mm, 전체 폭 약 36 cm)
- 씬 레이어 덮어쓰기 (`drone.add_drone`, 원본 USD 그대로): 축소, 질량 0.8 kg (body 0.78 + 프로펠러 0.005 × 4, 원본 body 1.5 kg),
  프로펠러 회전 관절 4 개 → FixedJoint, 저장된 초기 속도(rotor0 9.4 °/s 등. USD 각속도 속성은 degree/s, 예전에 rad/s 로 적었음) 0, 물체 재질
- 잡는 곳: 몸체 허리(드론 x = 0) 옆면, 폭(y) 방향으로 닫음, 아래에서 위로 접근
- 대안: 기본 도형 간이 드론 `isaacsim/config/drone_simple.yaml` → `isaacsim/scripts/build_simple_drone.py` → `assets/drones/simple_drone/`
  (박스 몸체 180 × 80 × 70 mm + 팔·모터·로터·다리 각 4, 평평한 옆면, 치수 전부 yaml). 같은 `drone.py`·`check_drone.py` 로 쓴다
- third view 카메라: **로봇 base 근처에서 위를 올려다보며** 팔이 드론을 잡는 장면을 찍는다 (위치·방향은 `isaacsim/config/third_view_camera.yaml`).
  렌더링 보고 가림이 심하면 base 옆·뒤로 조정
- `trajectory` 모드는 나중에. 인자 자리만 두고 선택하면 미구현 에러로 중단

**2단계 1번 작업 순서 (데모 드론)**
- 2-1 드론 에셋 (Iris 덮어쓰기, 대안 간이 드론)과 `check_drone.py` 점검
- 2-2 공중 유지 (Pegasus 방식: 로터 4 개 추력 + 기하 제어기, `static`/`hover`, `release()` = 모터 정지, 프로펠러 회전 `--prop-spin`)
- 2-3 씬 스크립트 `drone_scene.py` (테이블·로봇·조명·손목/third view 카메라·드론, 인자 `--drone-config --drone-pos --mode --seed --base --headless --realtime`)
- 2-4 스크립트 파지 데모(차분 IK, 아래에서 위로 접근) + 자동 판정 `approach → grasped → held` (실물처럼 잡고 모터 정지 후 끝), 실패 케이스(헛잡기·빗나감·떨어뜨림)도 판정 확인
- 2-5 0.8 kg 잡고 모터 정지 뒤 내려앉음·미끄러짐 기록 (2-4 결과에 포함)

**2-1 결과 (2026-10-01)** — `check_drone.py` (`--drone-config`, GUI 보기 `--hold`) → Iris **4/4 PASS**, 간이 드론 4/4 PASS
- 잡는 폭은 실제 충돌 형상을 손가락 폭 slab(잡는 곳 ±13 mm)으로 잘라서 잼 (`drone.grasp_width`). 기준 = 그리퍼 완전 열림 107 mm − 여유 2 × 10 mm = 87 mm 이하
- Iris 0.75: 잡는 폭 **81.3 mm** (−41.9 ~ +39.3), 질량 PhysX 0.80000 kg, 무게중심이 잡는 곳에서 수평 0.15 mm, DOF 0,
  0.5 m 낙하 중 링크 상대 자세 변화 0.04°·0.0001 mm. 축소 후 프로펠러 위치 PhysX vs USD 0.5 mm 이내 (관절 프레임도 같이 축소됨)
  - 바닥에 놓이면 **3.4° 기울어짐**: 앞쪽 아래 안테나(드론 x 73, y 49 mm)가 다리 끝보다 10 mm 더 내려옴 (원본 형상) → 허용 5° (`rest_tilt_max_deg`)
  - **2-4 주의**: 안테나가 몸체 아래 앞쪽에 있음. 아래에서 접근할 때 손목 카메라(공구 축에서 +45~70 mm)가 앞쪽(+x)을 향하면 닿을 수 있음
- 간이 드론: 잡는 폭 80.0 mm, 0.80000 kg, 무게중심 수평 0.000 mm, 다리로 똑바로 섬 (몸체 바닥–지면 10 mm)
- 참고 (폐기한 Quadcopter 시험): 관절 한계 0/0 고정은 착지 충격에 0.79° 움직임 → 원래 관절 비활성화 + FixedJoint 로 고정 (Iris 도 같은 방식).
  TCP 좌표계 충돌 형상 범위: 손목 카메라 공구 축에서 +45~70 mm (z −112~−87 mm), 반대쪽 최대 wrist_3 43 mm, 손가락 폭 ±13 mm,
  완전 열림 손가락 안쪽 면 ±53.5 mm

**2-2 결과 (2026-10-01)** — 비행 `isaacsim/scripts/drone_flight.py` (`DroneFlight`), 설정 `drone_iris.yaml` 의 `flight`, 시험 `isaacsim/scripts/check_flight.py` → **4/4 PASS** (프로펠러 회전 끔·켬 둘 다)
- **Pegasus Simulator 방식** (6.0.1 포크 코드 확인): 제어기 → 로터 4 개 목표 ω → 이차 추력 Fᵢ = k·ωᵢ² 을 각 로터 강체 z 축으로,
  반토크 Σ c·ωᵢ²·dirᵢ·선형 공기저항을 body 에 (매 physics step, PHYSICS_PRE_STEP). 추력 모델 값은 Pegasus Iris 기본값
  (k 8.55e-6, c 1e-6, ω 최대 1100 rad/s, 공기저항 [0.5, 0.3, 0])
  - Pegasus PX4 예제는 PX4 SITL 이 MAVLink 로 보내는 모터 출력을 ω 로 씀. 우리는 지금 Python 예제(NonlinearController)와 같은
    **기하 제어기**를 쓰고, 2단계 2번에서 제어기만 PX4 로 바꾼다 (로터 추력 인터페이스 그대로)
  - **제어기는 Pegasus 와 같게 (2026-10-01 사용자 결정: 오차가 조금 크더라도 Pegasus 와 같은 동작이 실제에 가깝다)**:
    식 그대로, 매 step 지난 step 에 계산한 ω 를 먼저 적용하고 제어기를 갱신 (Pegasus 의 한 step 지연),
    게인 = Pegasus 값(Kp 10, Kd 8.5, Ki 1.5, Kr 3.5, Kw 0.5, 세 축 같음)을 Pegasus Iris(1.5 kg, I 0.029/0.029/0.055) 대비
    우리 기체 비율로 환산 (Kp·Kd·Ki × m/m_ref, Kr·Kw × I/I_ref 축별, m·I 는 PhysX 값) → Pegasus Iris 와 같은 응답
    - Pegasus 게인을 그대로 쓰면 관성이 6~9 배 작은 우리 드론에서 로터 명령이 매 step 크게 흔들림 (정지 오차 9.9 mm, 기울기 1.5°,
      로터 ω 평균 363 vs 호버 479 rad/s, Kw·dt/I 최대 1.29) → 환산 후 Kw·dt/I 0.14, 정지 오차 0.56 mm
  - 모드: `static`, `hover` (목표 + 축별 사인파 2 개 합, 진폭 [20, 20, 10] mm, 0.2~0.5 Hz, seed 기록), `trajectory` 는 미구현 에러
  - `release()` = 모터 정지 (ω 0). 공기저항은 계속
- **프로펠러 회전 (보여 주기용, `--prop-spin on|off`, 기본 off)**: Pegasus `handle_propeller_visual` 과 같음 — 실제 ω 와 무관하게
  로터 추력 ≥ 0.1 N 이면 100 rad/s, 0~0.1 N 이면 5 rad/s, 0 이면 정지 (방향 = rot_dir), 관절 속도를 매 step 덮어씀. 추력과는 분리.
  on 이면 프로펠러 관절을 FixedJoint 로 바꾸지 않음 (DOF 4)
- 시험 결과 (off / on, 환산 게인 + 한 step 지연):
  - F1 정지 호버 30 s (처음 20 s 제외): 오차 최대 0.56 / 0.50 mm, 로터 ω 평균이 계산 호버 값 479 rad/s 와 0.03%
  - F2 계단 +5 cm: overshoot x 9.5 / 10.9%, z 9.5%, 2 mm 안 정착 9~10 s, 12 s 뒤 오차 1.2~1.4 mm, x 이동 중 기울기 1.5° (실물처럼 기울어 이동)
  - F3 hover 20 s: 추적 오차 RMS 1.19 / 0.96 mm (최대 2.1 mm), 기울기 최대 0.54°
  - F4 모터 정지: 수직 가속도 −9.810 m/s² (= −g), ω 0, 프로펠러 관절 속도 0.004 rad/s 로 멈춤
  - on 일 때 비행 중 프로펠러 관절 속도 ±99.95 rad/s (명령 ±100)
- **알아 둘 특성 (Pegasus PID)**: 자세 적분항이 없어 무게중심이 0.14 mm 만 치우쳐도 몸체가 약 0.16° 기운 채 버티고 옆으로 약 4 mm 밀림 →
  위치 적분(Ki/Kp ≈ 0.15 /s)이 수십 초에 걸쳐 없앰. 계단 목표에서는 적분이 쌓여 약 10% overshoot 후 느리게 돌아옴.
  static·hover 에는 계단이 없어 영향 작음 (hover 추적 RMS < 1 mm). 시작 직후(재생 ~ 제어 시작 사이) 약 7 mm 처짐
- **시간 주의**: 6.1.0 에서 `simulation_app.update()` 한 번에 physics 가 2 step (1/60 s) 진행됨 → 시간은 콜백이 센 sim 시간으로 잴 것
  (grasp_tests·tune_drives 는 이미 그렇게 함)
- **Pegasus 예제 코드와 비교 (2026-10-01, `nonlinear_controller.py`·`multirotor.py`·`vehicle.py`·`linear_drag.py`·`quadratic_thrust_curve.py`)**
  - 같음: 추력 k·ω² 를 로터 강체 z 축으로(Pegasus 는 로터 local frame 에 걸고 우리는 같은 힘을 world 로 변환), 반토크 Σ c·ω²·dir,
    공기저항 −D·v_body, 할당 행렬(로터 위치를 body 좌표계로, pinv, 음수 0, 최대 ω 넘으면 비율 유지 축소), 제어 식
    (F_des, u₁ = F_des·Z_B, R_des, e_R = ½ vee(R_desᵀR − RᵀR_des), 목표 각속도 = 목표 jerk 투영, τ = −Kr·e_R − Kw·e_ω), 적분 ∫e_p dt,
    프로펠러 표시 (0.1 N 기준 100 / 5 / 0 rad/s, 관절 속도 덮어쓰기)
  - 맞춤 (비교 후): 한 step 지연, Pegasus 게인 값 (우리 기체 비율로 환산)
  - 남은 차이: 질량·관성은 우리 기체(PhysX) 값, physics 1/120 s (Pegasus 1/250 s, 로봇 drive 튜닝 기준이라 유지),
    yaw 목표 = 시작 yaw 고정 (Pegasus 는 궤적 파일), IMU·GPS 등 센서 없음
  - 목표 각속도(jerk 투영) 비교 hover RMS: 0 배 0.56 mm, Pegasus 식(+1) 1.20 mm, −1 배 1.73 mm → 부호는 Pegasus 가 맞고,
    차이가 1 mm 미만이고 Pegasus 와 같은 동작이 실제에 가까우므로 **Pegasus 식 그대로 유지**
- GUI 보기: `check_flight.py --view [--mode hover|static] [--prop-spin on] [--release-after 8]` (실제 시간 속도, 비행 → 모터 정지 → 낙하)

**2-3 결과 (2026-10-01)** — 씬 `isaacsim/scripts/drone_scene.py` (`build_scene()`, 2-4·3단계도 그대로 씀) → `--check` **3/3 PASS**
- **로봇 설정 파일화**: 로봇 이름(USD·prim·마운트·팔 관절·그리퍼 구동/mimic·TCP·하중 링크·drive/손목 카메라 설정 파일)을
  `isaacsim/config/robot_ur5_rh_p12.yaml` 한 곳으로. `test_scene.py` 가 import 때 읽어 모듈 상수(ROBOT_PATH, ARM_JOINTS …)를 채우고,
  다른 로봇은 `test_scene.use_robot(경로)` / `drone_scene.py --robot-config`. 기존 시험 스크립트는 그대로 동작
  (check_articulation 8/8, tune_drives D 닫힘·열림 2.15 s·반응 0.033 s, 리팩터 전과 같음)
  - 로봇을 바꿀 때 설정 파일로 안 되는 것: drive 게인 재튜닝(1-5), "구동 관절 1 + mimic" 이 아닌 그리퍼는 robot_drive 확장,
    6 축이 아닌 팔은 데이터셋 스키마(0단계) 변경
- 씬 (`isaacsim/config/scene_drone.yaml`): 바닥 + **테이블 2 × 1 m** (상판 0.762 m, 박스 collider, 로봇이 −x 끝에서 0.25 m 안쪽) +
  로봇 (최상위 prim (0, 0, 0.762), fixed base, 확정 drive, 홈 자세) + Dome Light 1000 (headless 에서도) + 손목 카메라 + third view 카메라 + 드론
  - 로봇은 +x 쪽으로 작업 (홈 자세 TCP (0.49, 0.11, 1.07) 아래 향함, wrist_1 (0.39, 0.11, 1.28))
  - 드론 기본 위치 **(0.60, 0, 1.50)**: 처음 (0.45, 0, 1.40) 은 홈 자세 wrist_1 에서 약 10 cm 라 옮김 (32 cm). 닿는지는 2-4 IK 로 확인
- third view 카메라 (`isaacsim/config/third_view_camera.yaml`): 렌즈 (0.05, −0.45, 0.90) 에서 파지 구역 (0.55, 0, 1.35) 을 올려다봄,
  640x480. **intrinsics 는 임시값** (D456 가로 시야각 약 90° → fx = fy = 320, 주점 중앙). 실물 D456 이 생기면 camera_info 로 교체
  - **카메라 모양이 씬에 보임**: 로봇 USD 의 손목 D435i 하위 트리(`wrist_camera_link`, realsense2_description 메시 + 충돌 박스)를
    그대로 reference (원본은 읽기만), 렌더링 카메라는 그 color optical frame 에 붙임 → 렌즈 위치 = 설정 position.
    테이블 상판 → 카메라 아랫면 받침대 (바닥판 Ø70 + 기둥 Ø16, 높이 123 mm, 정적 collider). 렌즈 높이 0.80 → 0.90 (받침대가 보이게)
  - 카메라 intrinsics → USD 변환은 손목 카메라와 같은 함수 (`test_scene.define_pinhole_camera`)
- `--check` (10 s): 로봇 홈 자세 편차 최대 0.033°, 드론 목표와 거리 2.7 mm (static) / 3.2 mm (hover, prop-spin on),
  카메라 두 대 영상 정상 (밝기 평균 188 / 211, 표준편차 11 / 50). 영상 PNG 는 `isaacsim/reports/drone_scene_<시각>/`
  - 홈 자세에서는 그리퍼가 아래를 향해 손목 카메라에 테이블만 보임 (정상)
- **팔 시작 자세 `--init-pose q1..q6` [rad]** (기본 drive 설정 `home_pose` [0, −π/2, π/2, −π/2, −π/2, 0], 그리퍼는 항상 완전 열림으로 시작).
  관절 한계 밖이면 에러. `--check` 의 자세 유지는 이 자세 기준 (팔이 테이블·드론·받침대에 닿으면 편차로 드러남)
  - 그리퍼가 +z 를 보고 기본 위치 드론 바로 아래 21 cm: `--init-pose -0.1888 -0.7854 0.9599 1.3963 -1.5708 0.1888`
    (FK 탐색, TCP (0.602, 0, 1.291), wrist_3 +0.1888 로 손가락 닫는 방향을 드론 폭(y) 방향에 맞춤) → `--check` 3/3,
    **손목 카메라에 드론 아랫면이 화면 가득 보임** (화면 오른쪽으로 치우침 = 1-7 렌즈 32.5 mm 옆 문제)
- 실행: GUI `drone_scene.py [--mode hover] [--prop-spin on] [--drone-pos x y z] [--init-pose …]` (손목·third view 카메라 창 같이), 점검 `--headless --check`.
  `--mode trajectory`, `--base kinematic` 은 미구현 에러

**2-4 결과 (2026-10-01)** — 스크립트 파지 데모 `isaacsim/scripts/grasp_demo.py` (설정 `isaacsim/config/grasp_demo.yaml`) → `--case all` **6/6 PASS**
- **실물 시나리오에 맞춰 성공 = held** (잡기 → 드론 모터 정지 → 버팀 → 끝, 실물은 여기서 사람이 떼어 감). 내려놓기는 하지 않음
  - 가운데 홈 있는 착륙 받침대에 내려놓기를 시도했으나 **제거** (2026-10-01 사용자 결정): 실물로 만들 계획이 없고,
    팔뚝(팔꿈치 최고 1.275 m)·기둥·손목이 받침대에 막혀 경로가 복잡해짐. 내려놓기·충돌 없는 경로는 **나중에 MoveIt 등 경로 계획**으로
- 시퀀스: 홈 → 관절 보간으로 seed 자세 (그리퍼 +z, 드론 아래) → 차분 IK 로 잡는 자세 15 cm 아래 → 3 cm/s 수직 상승 → 그리퍼 닫기 → 모터 정지 → 2.5 s 유지
  - 팔 관절 목표는 50 Hz zero-order hold, 한 주기 변화 ≤ 0.063 rad (텔레옵 흉내, 1-5 기준)
  - **seed 자세 wrist_1 은 −4.8869 (= 1.3963 − 2π)**: 홈(−π/2)에서 +쪽으로 돌리면 그리퍼가 팔에 걸려 0.21 rad 에서 막힘
- **잡는 높이 계산** (`gripper_geom.plan_grasp_from_below`, 1-6 손가락 기구학을 `gripper_geom.py` 로 옮겨 일반화):
  드론 body 충돌 형상(표면 점 20 만 개)을 손가락 폭 slab(±13 mm)으로 잘라, 파지면 높이 띠의 폭으로 닿는 각도, 그리퍼 base·r1·l1 이
  닫히는 동안 드론 아랫면(2 mm 칸 높이 지도)과의 여유 ≥ 2 mm 인 가장 높은 TCP 를 고름
  → Iris 0.75: TCP 드론 좌표계 (0, −1.3, −4.5) mm, 폭 81.1 mm, 닿는 각도 예측 20.3° (실제 16.4°), 여유 2.1~2.3 mm
  - **body 링크 자신의 좌표계로 계산** (재생 직후 드론이 이미 몇 mm 내려가 최상위 prim 기준이면 높이가 6 mm 어긋남)
- **차분 IK** (`isaacsim/scripts/arm_ik.py`. 5단계 SpaceMouse·PICO 는 같은 방법을 쓰되 Jacobian 은 URDF 에서 계산한다: PhysX Jacobian 은 sim 프로세스 안에서만 얻을 수 있다, 5-B): PhysX Jacobian + damped least squares
  - **PhysX Jacobian 선속도 행은 링크 무게중심 기준** (그리퍼 base 무게중심 z 31.9 mm): 원점 기준으로 쓰면 축이 수평인 관절에서 약 3 cm/rad 차이.
    수치 미분과 비교해 보정 후 차이 0.0006 이하
  - 관절 목표 = 이전 목표 + Δq (측정값 + Δq 면 drive 중력 처짐만큼 0.6 mm 못 감) + 와인드업 방지 (목표가 측정값보다 0.05 rad 이상 앞서지 않게)
  - 5 cm·10° 이동이 14 주기 만에 수렴 (0.13 mm, 0.012°)
- **자동 판정** (`isaacsim/scripts/grasp_judge.py`, 3단계 메타데이터·7단계 평가에서도 씀): `approach → grasped → held` / `failed` + 사유
  - grasped: 닫기 중 그리퍼 멈춤(위치 변화로 판단, 관절 속도 읽기는 안 씀) + 62° 보다 덜 닫힘 + 양쪽 손가락 접촉력 > 1 N (PhysX contact report)
  - held: 모터 정지 2.5 s 뒤 TCP 기준 내려앉음 ≤ 15 mm, 마지막 0.5 s 변화 ≤ 0.5 mm, 회전 ≤ 10°
  - **아래에서 잡으면 모터 정지 뒤 드론이 약 5~6 mm 내려앉은 뒤 멈춤** (1~1.6 s): 무게가 실리며 손가락 뿌리(r1·l1)에 얹힘 → 형상으로 받쳐짐.
    그래서 마찰을 0.05 로 낮춰도 떨어지지 않음
- 판정기 검증 (기대 판정과 같으면 PASS): success → held (내려앉음 4~6 mm, 손가락 접촉 양쪽 27 N), empty (드론을 다른 곳에) → no_grasp_empty (64.06° 까지 닫힘),
  miss (잡는 높이 −4 cm) → no_grasp_empty, drop (모터 정지 순간 그리퍼 힘 2.28 → 0.05 Nm) → slip (0.05 s 만에 11~17 mm)
- **어긋나게 잡기** (텔레옵은 중앙을 정확히 못 잡음): 중앙을 잡으면 한쪽 손가락만 미는 시간이 0.017 s 라 드론이 약 1.4 mm 만 움직임
  (드론 제어기가 강해서가 아님: 위치 게인 약 5.3 N/m, 1 N 으로 계속 밀면 약 19 cm 밀림. 양쪽 손가락 힘 약 27 N 이 서로 상쇄)
  - offset_y (닫는 방향 10 mm): 먼저 닿은 손가락이 **드론을 약 9.3 mm 민 뒤** 잡힘 → held (내려앉음 4.7~5.5 mm, 기울기 4.6~5.1°)
    (열린 손가락과 몸체 사이가 한쪽 13 mm 라 20 mm 면 올라가다 부딪힘)
  - offset_x (몸체 길이 방향 5 mm) → held. **10·20 mm 는 잡는 높이까지 못 올라감**: Iris 몸체는 땅콩 모양이라 허리(x = 0)가 가장 좁고,
    옆으로 가면 넓은 부분에 손가락 끝(l2, 4.2 N)이 걸림 → 텔레옵에서는 허리에 맞춰 잡아야 함
  - 기울기·내려앉음이 실행마다 다름 (회전 2~5°) → held 기준을 회전 10°, 내려앉음 15 mm 로 (떨어뜨림 판정 30 mm 는 그대로)
  - S2 수직 상승 목표는 시작 순간의 드론 위치로 고정 (드론을 따라가면 손가락이 드론을 밀 때 목표도 움직여 끝없이 쫓아감)
- GUI: 메인 뷰포트 + 'Wrist camera'·'Third view camera' 창 (`drone_scene.open_camera_windows`)
- 결과 파일: `isaacsim/reports/grasp_demo_<시각>/` (report.txt, result_<케이스>.json = 4단계 meta.json 에 쓸 형식, timeline_<케이스>.csv)
- 실행: GUI `grasp_demo.py [--case success|offset_y|offset_x|empty|miss|drop] [--tcp-offset DX DY DZ] [--prop-spin on] [--hold]`, 점검 `--headless --case all` (6 케이스).
  `--tcp-offset` [m, world] 은 케이스 설정의 어긋남 대신 (드론 yaw 0: y = 손가락 닫는 방향, x = 몸체 길이 방향). 예: y −5 mm → 드론 6.6 mm 밀린 뒤 held

**작업**
- 씬 구성을 GUI Action Graph 대신 **Python standalone 스크립트**로 작성 (씬 + ROS 2 OmniGraph 생성), `~/isaacsim/python.sh` 로 실행
- 테이블 + 로봇 배치(최상위 prim Transform, root_joint 유지), 조명, third view 카메라
  - **조명은 씬 스크립트가 반드시 만든다** (Dome Light 등). 없으면 GUI 뷰포트가 비어 보이고 카메라 영상(손목·third view)도 검게 나온다.
    1-5 테스트 씬은 GUI 일 때만 Dome Light 를 넣지만(`test_scene.build_test_scene(light=...)`), 2단계 씬은 카메라를 녹화하므로 headless 에서도 항상 넣는다
- 베이스 모드 인자 자리 확보: `--base fixed` 만 구현 (나중에 `--base kinematic` 추가)
- 드론: 위 "데모 드론" (Iris 외형 0.75 배, 0.8 kg, 몸체 옆면을 잡음. 대안 간이 드론)
- 드론 모드: `static`(공중 고정) → `hover`(중력 보상 + 약한 흔들림) → `trajectory`(이동)
- 파지 성공 판정: 그리퍼 닫힘 + 드론이 손가락 사이 + 로봇과 함께 이동

**결정 항목**
- [x] **파지 후 중력 보상 유지 여부 → 끈다 (2026-10-01)**: 실물은 잡은 뒤 드론 모터를 멈추므로, 파지 판정 후 드론의 공중 유지 힘을 끄고
  드론 무게(0.8 kg)가 그리퍼에 걸리게 한다. 0.8 kg 을 몸체 옆면으로 들고 내려놓을 때 미끄러지지 않는지 2단계에서 확인 (1-6 은 0.5 kg 까지 확인)

**완료 기준**
- [x] 스크립트 실행만으로 씬이 뜸 (2-3 `drone_scene.py`)
- [x] 드론 위치·모드를 인자로 변경 가능 (`--drone-pos`, `--mode static|hover`, `--seed`)
- [x] 파지 성공/실패가 자동 판정됨 (2-4 `grasp_judge.py`, 판정기 검증 6/6)

### 2단계 2번: PX4 SITL 드론 (2026-10-02 계획)

**목표**: 실제 PX4 펌웨어가 드론을 날리는 상태에서 **잡을 때의 움직임·떨림**을 재현한다.
지금 기하 제어기는 정답 상태·잡음 없음·지연 없음·약한 위치 게인(5.3 N/m)이라 잡을 때 너무 조용함 (중앙 파지 시 1.4 mm 움직이고 멈춤).
실물은 제어기가 그리퍼와 싸움 (속도·각속도 적분, 빠른 각속도 루프 + 딱딱한 구속, 추정 잡음, 모터 지연) → 떨림.
기하 제어기는 비교 기준·빠른 시험용으로 남긴다 (`flight.controller: geometric | px4`).

**결정 (2026-10-02 사용자)**: 실물 드론 제어기는 미정이지만 데모 → PX4. **시연은 실내** → GPS 없음,
위치 정보 기본 = **Optical flow + 하향 거리 센서** (소비자 드론 실내 모드와 같은 원리). 모션캡처(외부 위치)는 비교용 옵션, 마커 + 드론 카메라는 나중에 같은 외부 위치 통로로
- 이유: 그리퍼가 **아래에서** 올라오므로 하향 거리 센서가 바닥 대신 그리퍼를 재고(EKF 고도 급변 → 드론 상승), 흐름 카메라에 올라오는 팔이 보임 → 실물에서 잡을 때 드론이 움직이는 원인을 재현. 모션캡처로는 안 나옴

**Pegasus 방식 확인 (livealive7/PegasusSimulator b1256ca, `px4_mavlink_backend.py`·`px4_launch_tool.py`·`sensors/*`)**
- PX4-Autopilot **v1.16.0** (Ubuntu 24.04 시험), `make px4_sitl_default none`, `PX4_SIM_MODEL=gazebo-classic_iris`, 임시 rootfs 에서 `bin/px4 ROMFS/px4fmu_common -s rcS -i 0 -d` subprocess
- pymavlink, TCP 4560 (sim = 서버), **lockstep**: 매 physics step (250 Hz) 지난 모터 명령 적용 → `HIL_ACTUATOR_CONTROLS` 올 때까지 대기 → `HIL_SENSOR`·`HIL_GPS` 전송 (시각 = sim μs), heartbeat 1 Hz
- 모터 명령 ω = 1000·u + 100 rad/s (u 0~1, arm 아니면 0). 우리 호버 ω 479 → u ≈ 0.38. **모터 반응 지연 없음**
- 센서 250 Hz: IMU (가속도 = 속도 차분 − g → 그리퍼가 미는 힘도 잡힘), 기압, 지자기, GPS. 드론 명령 도구 없음 (QGroundControl 등 외부)
- Pegasus 코드 문제: 자이로 bias 시간상수에 가속도계 값 사용, 가속도계 bias 미적용(randn 대신 rand), 기압 잡음 Box-Muller 에 randn, GPS 250 Hz·속도 잡음 꺼짐, 센서 값 한 step 늦음

**작업 순서**
- P-1 PX4 v1.16.0 `~/PX4-Autopilot` clone·빌드 (빌드용 Python 패키지는 venv). pymavlink 는 `isaacsim/.pydeps/` (`pip install --target`, git 제외, `~/isaacsim` 에 만들지 않음)
- P-2 Pegasus 그대로 재현 (기준선, 250 Hz·GPS·모터 지연 없음·gazebo-classic_iris): `px4_bridge.py`·`px4_sensors.py` (BSD-3 출처 표기). 연결·lockstep·EKF2·이륙·호버
- P-3 우리 조건: physics 120 Hz 부터 (불안정하면 240 Hz + 로봇 drive 회귀 시험), PX4 파라미터 `isaacsim/config/px4/` (0.75 배 로터 위치 `CA_ROTORx_PX/PY`, 호버 추력, 게인,
  flow·거리 센서 융합 `EKF2_OF_CTRL`·`EKF2_RNG_CTRL`·`EKF2_HGT_REF`, GPS 끔), `drone_iris.yaml` 에 `flight.controller`·px4 절
- P-4 사실성: 모터 1차 지연 (올림 0.0125 s, 내림 0.025 s, PX4 Gazebo 기본값, 켬·끔), Pegasus 센서 버그 수정 (문서에 표시),
  **거리 센서 = PhysX raycast** (팔·그리퍼에 맞음), **flow = 몸체 속도 ÷ 맞은 곳 거리 − 각속도** (맞은 물체가 움직이면 그 속도 반영, 잡음·품질), 모션캡처 옵션 (주기·지연·잡음)
- P-5 드론 명령 `drone_cmd.py` (pymavlink): arm, takeoff, goto x y z [yaw], hold, land, kill (world 좌표 m). 씬 안 자동 시퀀스·다른 터미널 공용. 시작 = 테이블 옆 바닥 이륙 → 드론 위치로
- P-6 `check_px4.py`: X1 이륙 → 목표 5 cm 안 30 s 이내, X2 호버 30 s 흔들림, X3 계단 +10 cm, X4 공중 kill → −g, X5 반복 실행 차이, X6 60 s 표류 (flow)
- P-7 `--flight px4` 로 파지: 잡은 뒤 kill 까지 대기 0.5 / 2 / 5 s, 떨림 측정 (드론 위치·자세 진폭·주파수, 접촉력, 모터 명령, 거리 센서·EKF 고도),
  기하 제어기 / PX4 (flow, mocap) / 모터 지연 켬·끔 비교, 6 케이스 다시
- P-8 `docs/drone_flight.md` PX4 절, PLAN 결과, CLAUDE.md

**P-1 결과 (2026-10-02)** — PX4 v1.16.0 `~/PX4-Autopilot` (shallow clone + submodule), 빌드용 venv `~/PX4-Autopilot/.venv` (Tools/setup/requirements.txt), sudo 불필요
- shallow clone 이라 NuttX 태그가 없어 버전 헤더 생성이 멈춤 → `platforms/nuttx/NuttX/nuttx` 에 로컬 태그 `nuttx-11.0.0` (SITL 영향 없음)
- `make px4_sitl_default none` 은 빌드 후 PX4 셸까지 띄움 (빌드만 확인하면 끌 것). 단독 실행 시 `Waiting for simulator to accept connection on TCP port 4560`
- pymavlink 2.4.50 → `isaacsim/.pydeps/` (git 제외). airframe 에 `1010_gazebo-classic_iris_opt_flow` 있음 (P-3 참고)

**P-2 결과 (2026-10-02)** — `px4_bridge.py` (PX4Launcher·PX4Bridge), `px4_sensors.py` (pegasus_compat), `pegasus_geo_mag.py` (원본 복사), `drone_cmd.py` (PX4Commander),
`drone_flight.py` 에 `flight.backend: geometric | px4`, 설정 `px4_sitl.yaml`·`drone_iris_pegasus.yaml` (Pegasus Iris 원래 크기 1.52 kg), 시험 `check_px4.py` → **5/5 PASS** (250 Hz, GPS, compat)
- 연결: lockstep 시작 sim 0.6 s, Offboard + arm 요청 1.2 s 뒤 armed. 실시간 비율 1.06 (headless). PX4 로그(.ulg)·콘솔은 `isaacsim/reports/px4_<시각>/`
- **Pegasus Iris 는 그대로 쓰면 모터 명령이 17.4 Hz 로 0 ↔ 0.9 포화 진동** (ω 표준편차 355 rad/s, 그래도 날고 기울기 1.4°):
  ~~`iris.usd` body 관성이 비어 있어(diagonalInertia 0)~~ PhysX 가 충돌 형상으로 계산 → [0.0175, 0.0107, 0.0268] kg·m², PX4 Iris 파라미터가 맞춰진 Gazebo Iris
  [0.029, 0.029, 0.055] 보다 1.7~2.7 배 작음 → 각속도 루프가 사실상 너무 셈. **관성을 Gazebo 값으로 주면 ω 표준편차 1.3 rad/s, 기울기 0.19°**
  → P-3 (0.75 배, 관성 6~9 배 작음) 은 PX4 각속도 게인을 기체 관성에 맞춰야 함 (기하 제어기 게인 환산과 같은 문제)
  - **[정정 2026-10-07] `iris.usd` 의 관성은 비어 있지 않다**: 파일을 열어 보면 body 에 `diagonalInertia = (0.029125, 0.029125, 0.055225)` 가 저장돼 있고,
    이것이 위의 Gazebo Iris 값과 같다. 관성을 0 으로 만드는 것은 **우리 코드**: `drone.add_drone` 이 `link_masses` 가 있으면 저장된 관성·주축을 지워
    PhysX 가 형상에서 다시 계산하게 한다 (질량을 바꾸면 저장된 관성이 맞지 않으므로). 기준선 `drone_iris_pegasus.yaml` 도 `link_masses` 를 주므로 지워진다
    → 위 17.4 Hz 진동은 Pegasus 에셋 탓이 아니라 우리가 원본 관성을 지운 결과로 보인다 (파일·코드를 읽어 확인. **지우지 않고 다시 돌려 보지는 않음**)
    - 수집 설정 (0.75 배, 0.8 kg) 에는 영향이 없을 것으로 본다: 원본 관성은 축소·감량한 기체에 어차피 맞지 않아 다시 계산하는 것이 맞고,
      `rate_gain_scaling` 이 그 관성에 맞춰 게인을 환산하며 시험 (`check_px4` 5/5, 파지 6/6) 도 그 상태에서 통과
    - 남은 것: "Pegasus 기준선 재현" 이 Pegasus 와 같은 관성으로 돌았는지 (질량을 바꾸지 않는 기준선에서는 관성을 지우지 않는 것이 맞는지) 는 확인하지 않음
- GPS 호버: 실제 위치 − 목표 RMS 48~55 mm (대부분 z −40 mm = EKF 고도 추정 오차, 추정 − 실제 RMS 43~47 mm), 계단 +10 cm overshoot 14~39% (실행마다 다름)
- kill: disarm 까지 32 ms, 그 뒤 수직 가속도 −9.810 m/s², ω 0
- 로터 순서·위치: USD rotor0~3 = PX4 CA_ROTOR0~3 (앞-오른쪽, 뒤-왼쪽, 앞-왼쪽, 뒤-오른쪽), 위치 차이 1~4 cm

**P-3 결과 (2026-10-02)** — 0.75 배 Iris 0.8 kg (`drone_iris.yaml` 의 `px4` 절, `--flight`/backend 는 아직 geometric 기본), `check_px4.py --drone-config isaacsim/config/drone_iris.yaml --start-z 0.08 --physics-hz 120` → **5/5 PASS**
- **각속도 게인 환산** (`px4.rate_gain_scaling`, `drone_flight._px4_airframe_params`): `MC_{ROLL,PITCH,YAW}RATE_K = (I / I_ref) ÷ (팔 / 팔_ref)`,
  기준 = P-2 에서 안정 확인한 Gazebo Iris 관성 + Pegasus 추력·로터 위치 → roll 0.246, pitch 0.148, yaw 0.148. `CA_ROTORi_PX/PY` 는 sim 로터 위치 (0.75 배)
  → 모터 ω 표준편차 0.5 rad/s (진동 없음), 호버 기울기 최대 0.2°
- **physics 120 Hz 그대로 됨** (250 Hz 와 같은 안정성, 실시간 비율 1.5) → 로봇 drive 재튜닝 불필요
- **실내 위치 = optical flow + 하향 거리 센서** (`px4_sitl.yaml` `position_source: flow`, `--position-source gps|flow`):
  거리 = PhysX raycast (드론 자기 충돌 형상 제외), flow = 몸체 FRD pixel_flow 각속도 `ω_xy + (−v_y, v_x)/거리` 적분 (50 Hz, 잡음 0.02 rad/s 가정), GPS 끔
  - **부호 주의**: PX4 `EKF2.cpp` 가 pixel_flow·delta_angle 부호를 뒤집어 받음. 처음 `(v_y, −v_x)` 로 넣어 EKF 발산 → 고침 (추정/실제 속도 기울기 0.9~1.06)
  - **고도 기준 = 기압계 + 거리 센서 조건부** (`EKF2_HGT_REF 0`, `EKF2_RNG_CTRL 1`, PX4 opt_flow airframe 과 같음).
    고도 기준 = 거리 센서(`HGT_REF 2`)는 PX4 v1.16 에서 terrain 추정이 시작되지 않아 flow 융합 시작 조건을 못 넘음 → flow 미사용 → 수평 위치 무효 → failsafe (ulog 확인)
  - PX4 local 원점을 arm 직전에 world 에 맞춤 (실제 − 추정, 실물에서 이륙 전 위치 등록과 같음). 이륙 판정은 PX4 추정 위치 기준
  - 실물 flow 드론 같은 성질이 나옴: **이륙 중 11~23 cm 수평 표류** (바닥에서는 flow 최소 높이 0.08 m 아래라 무효 + PX4 지면 효과 보정 구간 → 그 사이 관성 추정),
    그 뒤 위치 유지 (추정은 목표 ±1 cm, 실제는 밀린 자리에서 30 s 에 수 cm). **실제 고도가 기압 잡음으로 30 s 에 약 15 cm 천천히 표류** (추정은 목표에 고정)
  - 파지는 실제 드론 위치(sim 정답 = 실물의 인식 결과)를 향해 가므로 표류 자체는 문제 아님. 접근 중 표류 속도가 중요 (P-7)

**P-4 결과 (2026-10-02)** — 같은 명령 → **5/5 PASS** (비교 옵션 `--motor-lag on|off`, `--sensor-compat on|off`, `--px4-param NAME=VALUE`)
- **모터 1차 지연** (`px4.motor_dynamics`, PX4 Gazebo motor_model τ 올림 0.0125 s / 내림 0.025 s, Pegasus 에는 없음): 호버에는 영향 거의 없음.
  kill 뒤 모터가 지수적으로 감속 (125 ms = 5τ 뒤 −9.810 m/s² 낙하) → X4 판정을 감속 뒤로
- **센서 버그 수정** (`pegasus_compat: false` 기본, 기준선 `drone_iris_pegasus.yaml` 만 true): 가속도계 bias 가 실물처럼 들어가자
  **실제 고도가 40 s 에 1 m 넘게 표류** — PX4 기본 `EKF2_TERR_NOISE 5.0` 에서는 거리 센서 정보를 terrain 상태가 다 흡수하고 기압(EKF2_BARO_NOISE 3.5 m)은 약해서
  수직 속도 오차(0.3 m/s)를 아무도 못 바로잡음 (ulog: 기압·거리 혁신 ≈ 0, 추정 vz = 목표 vz)
  → **실내 평평한 바닥 `EKF2_TERR_NOISE 0.1`** (0.5 는 실제 오차 RMS 124 mm): 이륙 표류 3 mm, 호버 실제 − 목표 RMS 67 mm (고도 +48 mm 치우침, 기압 기준),
  추정 − 실제 RMS 45 mm, 기울기 0.26°, 계단 +10 cm overshoot 16% / 2 cm 안 2.1 s
  - 이 값은 그리퍼가 아래로 들어올 때의 반응도 바꿈 (작으면 "드론이 내려갔다" 쪽) → P-7 에서 기본값 5.0 과 비교
- 기준선 재현은 `--drone-config isaacsim/config/drone_iris_pegasus.yaml --position-source gps --physics-hz 250` (기본 위치 방식이 flow 로 바뀜)

**P-7 진행 (2026-10-02)** — `drone_scene.build_scene(flight=, position_source=)` (px4: 테이블 위 이륙 지점 (1.2, 0) → 수직 상승 → 수평 이동 → 호버 5 s),
`grasp_demo.py --flight px4 [--position-source] [--kill-delay] [--px4-param]`, 잡은 뒤 kill 까지 `S3_hold` (설정 `kill_delay` 1.0 s) 떨림 지표 (`hold_metrics`)
- PX4 프로세스 정리: Isaac Sim `close()` 는 빠른 종료라 atexit 이 안 돌아 PX4 가 남음 → `PR_SET_PDEATHSIG` (부모가 죽으면 커널이 PX4 종료) + 시작 전 같은 instance 검사
- **flow + 하향 거리 센서에서는 팔이 드론 아래로 오면 드론이 위로 도망감** (success 케이스, 접근 전 단계에서 실패):
  seed 자세(그리퍼가 드론 21 cm 아래)로 가면 거리 센서가 테이블(0.76 m) 대신 팔(0.66 m)을 잼 → PX4 가 "내려갔다"로 보고 상승 →
  팔이 따라 올라가면 계속 팔을 잼 → 드론 3.2 m 까지 상승 (`EKF2_TERR_NOISE 0.1`), PX4 기본 5.0 에서도 2.2 m 까지 상승.
  `EKF2_RNG_CTRL 0` (거리 센서 끔) 은 flow 가 terrain 을 못 써서 수평 위치를 잃고 failsafe 착륙
  - 실물과 같은 현상: 하향 거리·비전 센서를 쓰는 드론(DJI 등) 아래에 손을 넣으면 드론이 올라감 (손으로 잡을 때 하향 센서를 끄라고 안내하는 이유)
  - **결정 (2026-10-02 사용자): A — 실내 위치는 모션캡처(외부 위치)로 얻을 수 있다고 가정**. flow 는 옵션(`--position-source flow`)으로 남기고 이 문제를 기록
  - **[기록] 실물 시연 위험**: 실물 드론이 하향 거리·비전 센서로 위치를 잡으면 팔이 아래로 오는 순간 같은 일이 생길 수 있음.
    대책 후보: 모션캡처 등 아래를 보지 않는 위치 정보, 접근 직전 하향 센서 고도 보조 끄기(기압 고도 표류 감수), 하향 센서를 피하는 접근 경로(옆에서 들어오기)
- **모션캡처 (`position_source: mocap`, 기본)**: world = 모션캡처 좌표, body 위치·자세 + 잡음(1 mm, 0.3°), 100 Hz, 지연 20 ms → `VISION_POSITION_ESTIMATE`.
  PX4: `EKF2_EV_CTRL 11`(수평·수직 위치 + yaw), `EKF2_HGT_REF 3`, `EKF2_EV_NOISE_MD 1`, `EKF2_EVP_NOISE 0.01`, `EKF2_EVA_NOISE 0.05`, `EKF2_EV_DELAY 20`,
  `EKF2_MAG_TYPE 5`, GPS·flow·거리 끔 → `check_px4` **5/5**: 호버 실제 − 목표 RMS 19 mm (축별 표준편차 8~10 mm, PX4 위치 유지의 느린 흔들림),
  추정 − 실제 RMS 6 mm, 계단 overshoot 24%
- **접근 중 드론 따라가기** (`grasp_demo.yaml` `approach_track_until` 0.03 m): PX4 호버는 ±1~2 cm 로 천천히 움직여 S2 목표를 시작 순간으로 고정하면
  5 s 접근 중 18~22 mm 어긋나 손가락 끝(r2)이 몸체에 30 N 으로 부딪힘 → 잡는 높이 3 cm 전까지는 드론을 따라가고 그 뒤 고정 (닿은 뒤 쫓아가지 않게)
- PX4 + 모션캡처 success → held (kill_delay 1 s). 처음 측정한 "모터 ω 흔들림 20~81 rad/s" 는 아래 가상 IMU 오류 때문이라 무효 (아래 고친 뒤 표)

**P-7 결과 (2026-10-02)** — PX4 + 모션캡처 `grasp_demo.py --headless --case all --flight px4` → **6/6 PASS** (success·offset_y·offset_x held, empty·miss no_grasp_empty, drop slip; 가상 IMU 고친 뒤 다시 6/6)
- **[고침] 가상 IMU 속도 = 자세·위치 차분** (`px4.sensor_kinematics: pose_difference`, 기준선은 `physx_velocity`):
  그리퍼에 잡혀 접촉이 걸리면 **PhysX 가 보고하는 드론 각속도가 실제 자세 변화와 다름** (잡힌 동안 보고 평균 29 °/s, 실제 자세 차분 0.5 °/s,
  접촉 없을 때는 둘 다 0) → 이 값을 자이로로 보내 PX4 자세 추정이 1.5 s 에 15° 틀어짐 → 모션캡처 위치가 오차 검사에서 거부 → 추정 발산 →
  모터 최대 포화 → 드론이 비틀려 그리퍼에서 빠짐 (kill 5 s, 처음에 PX4 적분 누적으로 잘못 해석했다가 ulog·sim 기록 비교로 확인).
  실물 IMU 는 실제 움직임만 재므로 차분이 맞음. 고치기 전 PX4 비교 결과(모터 흔들림 수십 rad/s, 5 s 에 빠짐)는 무효
- **잡은 채 모터를 켜 두는 시간(kill_delay)별 비교** (고친 뒤, success, `S3_hold` 구간, 2026-10-02):

  | 제어기 | kill 까지 | 결과 | TCP 기준 드론 위치 흔들림 (ptp, mm) | 자세 변화 최대 | 모터 ω 흔들림 (std) | 모터 ω 끝−처음 (최대) | 손가락 힘 흔들림 (std) | 모터 정지 뒤 내려앉음 |
  |---|---|---|---|---|---|---|---|---|
  | PX4 | 0.5 s | held | 0.03 | 0.02° | 0.4 rad/s | 1 rad/s | 0.1 N | 0.8 mm |
  | PX4 | 1 s | held | 0.4 | 0.2° | 1.9 | 7 | 0.8~1.4 N | 1.4 mm |
  | PX4 | 2 s | held | 0.4 | 0.5° | 1.0 | 2 | 0.7~1.0 N | 1.3 mm |
  | PX4 | 5 s | held | 0.6 | 0.5° | 2.7 | 17 | 0.5~1.0 N | 0.9 mm |
  | PX4 | 10 s | held | 0.5 | 0.6° | 2.9 | 21 | 0.3~1.1 N | 0.8 mm |
  | 기하 | 1 s | held | 0.03 | 0.02° | 0.7 | 3 | 0.3 N | 6.1 mm |
  | 기하 | 5 s | held | 0.03 | 0.01° | 0.3 | 4 | 0.01 N | 5.5 mm |

  (호버 중 모터 ω 흔들림 0.5 rad/s. 잡힌 동안 자세 차분 자이로 흔들림 0.4~3 °/s = 접촉 미세 진동)
  - PX4 는 잡힌 뒤 **천천히** 적분이 쌓임 (10 s 에 로터 간 ±20 rad/s 벌어짐), 손가락 힘이 기하 제어기보다 3~100 배 흔들림 → 그리퍼와 약하게 싸움.
    10 s 켜 둬도 놓치지 않음 (고치기 전 "1 s 안에 끄기" 결론은 철회)
  - 모터 정지 뒤 내려앉음이 PX4 0.8~1.4 mm vs 기하 5.5~6.1 mm → **원인: 잡는 높이 차이** (최종으로 얹히는 높이는 둘 다 TCP 기준 −0.2~−1.1 mm 로 같음).
    S2 목표를 고정(남은 높이 3 cm)한 뒤 닫기 시작까지 1.3 s 동안 기하 드론은 0.4 mm 움직였지만 PX4 드론은 호버 흔들림으로 z −7.9 mm·x −8.2 mm
    → 기하는 계획 높이(아랫면 여유 2 mm, 손가락 뿌리보다 약 5 mm 위)에서 잡아 끈 뒤 6 mm 내려앉고, PX4 는 이미 얹히는 높이 근처에서 잡아
    (닫으며 약 2 mm 들어 올림) 1 mm 만 내려앉음. PX4 에서는 잡는 높이·내려앉음이 실행마다 달라짐 (드론이 더 내려오면 닫기 전 손가락 뿌리가
    아랫면에 닿을 수 있음 → 그러면 고정 높이를 줄이거나 높이만 계속 따라가기)
  - sim 에 없는 실물 요소: 프로펠러 바람이 그리퍼·팔에 부딪힘, 기체·프로펠러 진동, 모션캡처 마커 가림, ESC 잡음 → 실물 떨림은 이보다 클 수 있음
  - `pegasus_compat` 기준선(drone_iris_pegasus)은 비행 시험 전용이라 영향 없음. `check_px4` 는 접촉이 없어 두 방식 결과 같음 (고친 뒤 5/5, 호버 RMS 19.5 mm 로 같음)
  - 지표는 GUI 업데이트 주기(약 60 Hz)로 기록 → 30 Hz 넘는 떨림은 안 보임 (모터 명령 전체는 PX4 ulog 에 있음)
- PX4 + 모션캡처 6 케이스 (고친 뒤) → **6/6 PASS**

**이륙 이탈과 안테나 충돌 (2026-10-02)**
- PX4 이륙 때 앞으로 최대 200 mm 밀림 → 원인: Iris 메시 앞쪽 아래 부품(안테나 추정)이 다리 끝보다 13.5 mm(0.75 배: 10 mm) 아래 → 바닥에서 3.4° 숙여져 섬,
  뜨는 순간 7° 까지 숙여지며 앞으로 가속. PX4 공식 Gazebo Iris 는 충돌이 상자(0.47 × 0.47 × 0.11 m)라 평평하게 섬 (`iris.stl` 분석: 다리 끝 −53.5 mm, 부품 −67 mm)
- 안테나 충돌만 끄는 것을 시도했다가 **되돌림 (2026-10-02 사용자 결정: 안테나 있는 원래 모델 유지, 파지에 영향 작음)**:
  몸체는 보이는 메시 겸 충돌 메시 하나(점 18,171)라 부품만 끌 수 없어, 충돌 전용 복사본에서 안테나 점만 다리 높이로 올림 →
  바닥 기울기 0°, 이륙 이탈 200 → 118 mm 로 줄었지만 **잡는 형상이 바뀜**: 원본 USD 에는 볼록 분해 결과가 미리 계산돼(cooked) 저장돼 있는데
  복사본은 PhysX 가 다시 분해해 허리 모양까지 달라짐 → 손가락 닫힘 16.4° → 13.7~14.3°, 모터 정지 뒤 내려앉음 5~6 → 8~9 mm, PX4 6 케이스 4/6
  - 다른 방법(몸체 충돌 그대로 + 다리 끝에 보이지 않는 받침)은 다리가 바닥에서 약 12 mm 떠 보여 하지 않음
  - 남은 영향: 이륙 때 앞으로 최대 약 200 mm 밀렸다가 3 s 안에 되돌아옴 (파지는 호버 안정 뒤 시작). 실물 드론(STL, 2단계 3번)으로 바꾸면 사라짐

**위험**: flow 표류·EKF 흔들림이 손가락 여유(한쪽 13 mm)를 넘으면 접근 중 부딪힘 → 접근 중 드론 따라가기 필요할 수 있음 (실물에도 필요할 가능성).
그리퍼가 다가올 때 EKF 가 위치를 잃으면 PX4 가 고도 유지 모드로 failsafe → 그대로 기록

**완료 기준**
- [x] PX4 v1.16.0 SITL 이 Pegasus 조건(P-2)과 우리 조건(P-3)에서 이륙·호버·kill
- [x] X1~X4 결과 기록 (모션캡처·flow·GPS). X5 반복 차이·X6 flow 표류는 남음 (flow 는 옵션이 돼 우선순위 낮음)
- [x] `--flight px4` 로 파지 6 케이스 판정 → 6/6 (모션캡처)
- [x] 잡을 때 떨림 비교표 (기하 vs PX4 모션캡처, kill 대기 0.5~10 s). flow 는 팔이 아래로 오면 드론이 도망가 파지 불가 (기록)
- [x] 문서 정리 (`docs/drone_flight.md` 13장, CLAUDE.md)
- [x] 기본 제어기 px4 (2026-10-02 사용자 결정, `drone_iris.yaml` `flight.backend: px4`, 기하는 `--flight geometric`. `check_flight` 는 기하 고정,
  `drone_scene --check` 드론 오차 기준 geometric 10 mm / px4 50 mm)
- [x] P-5 `--drone-waypoints x,y,z ...` (이륙 뒤 차례로, 마지막 드론 위치 호버), 다른 터미널 CLI `drone_cmd.py goto|hold|land|kill|status`
  (UDP localhost 14600 → 씬 안 PX4Commander 가 목표를 바꿔 계속 송신. 이륙 중 이동 명령 거부, kill 은 항상)
  - **추천 씬**: `goto` 는 `drone_scene.py` (드론과 팔이 같이 있지만 팔이 자동으로 잡지 않는 씬)에서 — 드론 위치를 바꿔 카메라에 보이는 모습·팔이 닿는 범위 확인,
    나중에 텔레옵 수집(5~6단계)에서 에피소드마다 드론 위치 바꾸기. `grasp_demo.py` 는 ready 직후 팔이 자동으로 드론을 쫓아가 잡으므로 `goto` 를 쓰면
    파지를 방해 → `--drone-waypoints`·`--drone-pos` 만 씀
  - 확인 (2026-10-02): 이륙 중 goto 거부, waypoint 2 개 뒤 sim 29 s 에 ready, `goto 0.65 0.05 1.45 --yaw-deg 20` → 4 s 뒤 추정 (0.654, 0.060, 1.465).
    `drone_scene --check` 드론 오차는 px4 면 지금 명령 목표와 비교 (goto 로 바뀔 수 있음)
  - 회귀 (기본 px4): `drone_scene --check` 3/3 (목표와 20.8 mm), `check_flight` (기하 고정) 4/4, `grasp_demo --case success` held

---

## 3단계: sim ROS 2 인터페이스

**목표**: 실물과 같은 토픽이 sim time 으로 나옴

> **완료 (2026-10-05)**. 완료 기준 5 개 모두 충족, 자동 검사 `check_ros2.py` 9/9 PASS. 정리 문서: **`docs/sim_ros2_interface.md`** (토픽·주기·구조도)
> - `sim_ros2.py` 하나로 씬 + ROS 2 토픽 (`/clock`·`/joint_states` 120 Hz, 카메라·그리퍼·`/protective_stop` 30 Hz, 모두 sim time, 같은 물리 스텝 stamp)
> - 팔 명령 선형 보간 (`docs/arm_command_interpolation.md`), sim 그리퍼 브리지 (실물 rh_gripper_node 형식), 보호 정지 흉내, `/sim/reset` (항상 순간이동 + PX4 재시작, 약 24 s)
> - RTF 대기 0.96, **드론을 잡은 채 0.69** (5단계에서 불편하면 원인 확인), 카메라 DLSS Performance 명시 (`docs/sim_performance.md`)
> - `/base/imu` 는 8단계부터 (고정 베이스 데이터는 stage1 `--base-imu const`)
> - 4단계로 넘길 것: 녹화 카메라 RELIABLE, stage1 이미지 나이 검사, `/sim/reset` 연동, `/protective_stop`·렌더 설정을 에피소드 메타데이터에, 카메라 토픽 `/cam/...` 통일

**공식 방식 조사 (2026-10-02, Isaac Sim 5.1~6.1 문서·예제)**
- ROS 2 연결은 OmniGraph 노드(Publish Clock, Publish/Subscribe Joint State → Articulation Controller, Camera Helper)와
  같은 프로세스의 rclpy 두 가지. 공식 Reference Architecture 는 **혼합**(센서 = OmniGraph, 로직 = rclpy)을 권한다
- **6.0 부터 카메라 주기는 카메라 prim 의 `omni:sensor:tickRate`(Hz, `OmniSensorAPI`)** 로 정한다 (multi-tick rendering, 기본 켬).
  Camera Helper 의 `frameSkipCount` 는 deprecated (렌더한 뒤 버림). tickRate 는 렌더 자체를 그 주기로만 한다
- 시계 3개(물리 dt, 루프 dt, 렌더러)는 `SimulationManager.setup_simulation(dt)` + `RenderingManager.set_dt(dt)` 로 맞춘다.
  stamp 는 `IsaacReadSimulationTime` / `SimulationManager.get_simulation_time()` (timeline 시간 금지)
- `ros2 topic hz` 는 기본이 wall time (RTF 0.5 면 120 Hz 가 60 Hz 로 보임). sim 기준은 `--use-sim-time`
- rclpy 로 이미지를 보내면 GPU→CPU 복사와 publish 가 sim 스레드를 프레임당 몇 ms 붙잡는다 (`camera_rclpy_async.py` 예제 설명)
- `isaacsim.ros2.sim_control`(표준 simulation_interfaces: Reset, SetEntityState 등)은 PX4 disarm·재이륙·부드러운 홈 복귀가 없어 쓰지 않는다.
  `isaacsim.ros2.control`(ros2_control 내장)은 텔레옵이 `/joint_command` 만 보내므로 필요 없다
- 이 PC: 시스템 ROS 2 Jazzy, Python 3.12 rclpy, Isaac Sim 내장 Jazzy 라이브러리도 있음

**결정 (2026-10-02)**
1. **카메라 = OmniGraph Camera Helper, 나머지 = rclpy** (공식 혼합 방식. 보호 정지를 위해 `/joint_command` 를 가로채야 함)
2. ~~`/base/imu` 는 상수로 발행~~ → **바꿈 (2026-10-05): 고정 베이스 단계에서는 sim 도 `/base/imu` 를 발행하지 않는다.**
   실물에 지금 IMU 가 없어 실물 데이터는 어차피 stage1 `--base-imu const`(명시 옵션)로 채워야 하므로, sim 도 같은 방식으로 맞춘다.
   8단계에서 sim IMU 센서와 실물 IMU 를 함께 붙이고 `/base/imu` 를 발행한다
3. 리셋 서비스는 직접 만든다 (`std_srvs/Trigger`)
4. 보호 정지 기준: UR 문서 값을 찾아 쓰고, 공개되지 않은 값은 임시값 (아래 "보호 정지")
5. `/joint_states` 120 Hz (물리와 같음, 실물 CB3 드라이버 125 Hz), 카메라 tickRate 30
6. RTF < 1 이면 먼저 렌더러 설정·tickRate 로 줄이고, 안 되면 물리 substep 구조 변경을 따로 보고 (PX4 lockstep·drive 가 물리 스텝마다 돌도록 바꿔야 해 큰 작업)

**작업**
- **먼저 real-time factor 측정**: 카메라 2대 렌더링(640x480) + 물리를 켠 상태에서 RTF 와 카메라 발행 주기 측정.
  25 Hz 이상 + 텔레옵 조작감이 실제 병목일 가능성이 크므로 3단계 초반에 확인. tickRate 0(매 스텝) / 30, PX4 켬/끔 비교
- `/clock`, `/joint_states` 발행, `/joint_command` 구독 (rclpy)
  - `/joint_command` 는 이름으로 팔 6관절을 찾아 drive 목표로 넣는다. 모르는 이름·길이 불일치는 에러 (조용한 fallback 금지)
- 그리퍼 명령 토픽은 팔과 분리 (`/gripper/command`), 대상은 `rh_r1_joint` 하나
- **sim 그리퍼 브리지 노드** (실물 `rh_gripper_node` 의 sim 버전)
  - 속도는 **`robot_drive.GripperProfile`(목표값 이동, 1-5)을 그대로 쓴다**. 관절 속도 제한은 쓰지 않음
  - 구독: `/gripper/command` (raw 0~1150, 열기/닫기)
  - 발행(30 Hz, 같은 tick·같은 stamp, sim time): `/gripper/joint_states`(present, raw), `/gripper/target`(실행된 goal, raw 0 또는 1150)
  - **단위 변환은 브리지에서**: present raw = `rh_r1_joint`(rad) × 1150 / 1.1351, 명령 rad = raw × 1.1351 / 1150
    (USD 속성을 직접 읽거나 쓰면 degree 이므로 추가 변환 필요)
  - 시작 시 열림(0) 초기화
  - 토픽 값은 실물과 같은 raw 스케일 유지. 0~1 정규화는 stage1 에서만 한다
- 카메라 color 발행 (Camera Helper): `/cam/wrist/color/image_raw`, `/cam/third_view/color/image_raw` (640x480, frame_id 포함)
  - 카메라 prim 에 `OmniSensorAPI` + `omni:sensor:tickRate` 30 (씬 레이어에서, 로봇 USD 원본 수정 없음)
  - **realsense-ros wrapper 출력 형식을 흉내낸다** (녹화·stage1 은 토픽 메시지만 본다): `sensor_msgs/Image` `rgb8`, `.../color/camera_info`(1-7 intrinsics, 왜곡 0),
    frame_id `<카메라>_color_optical_frame`, QoS 는 실물 wrapper 와 같게 (실물 PC 에서 `ros2 topic info -v` 로 확인 필요)
  - pyrealsense2 / realsense-ros 를 sim 에 쓰지 않는 이유: 실제 USB 장치에서 프레임을 받는 드라이버라 sim 에는 장치가 없고,
    노출·노이즈 같은 실물 특성은 카메라 센서·펌웨어에서 생기므로 통과시켜도 이미지가 그대로다. 이미지 차이는 9단계에서 다룬다
- ~~베이스 IMU: `/base/imu` 상수 발행~~ → 하지 않음 (결정 2). 고정 베이스 데이터의 `observation.base_imu` 는 sim·실물 모두 4단계 stage1 `--base-imu const`
- **보호 정지 흉내** (1-5 "알려진 위험: 높은 K" 1번): 기준 초과 시 팔 목표를 현재 위치에 고정 (UR 보호 정지 = Cat 2 정지, 감속 후 정지 유지),
  `/protective_stop` 발행, 에피소드에 `protective_stop` 표시. 해제는 리셋 서비스로
  - **UR 문서에서 찾은 것** (UR5/CB3 User Manual, UR "Understanding Protective Stops" 2023):
    - General Limits 의 Force 는 100~250 N, Power 는 80~1000 W 사이로 설정. 한계(공차 없이)를 넘으면 안전 시스템이 Stop Category 0.
      컨트롤러는 "한계 − 공차" 안에서 움직이도록 속도를 줄인다 (예: 속도 공차 −150 mm/s)
    - Recovery 모드 한계: 관절 30 °/s, TCP 250 mm/s, TCP 힘 100 N, 운동량 10 kg·m/s, 파워 80 W
    - 충돌로 인한 보호 정지 (C153/C159 경로 이탈, C157/C158 관절 충돌 감지): 실제 전류가 목표 전류에서 갑자기 벗어나고 위치 오차가 빠르게 커질 때.
      **토크 창(window)은 목표 전류를 중심으로 하고, 크기는 힘 한계 × 거리** (UR 예: 힘 한계 150 N, 베이스-팔꿈치 0.5 m → 베이스 토크 창 75 Nm)
  - **찾지 못한 것**: CB3 기본 preset(Default 등)의 숫자 (매뉴얼에 "GUI 에 표시된다"고만 있음), 경로 이탈 판정의 위치 오차 숫자, 토크 창의 정확한 계산식.
    실물 PolyScope Installation → Safety → General Limits 에서 확인해 바꾼다
  - **sim 임시값 (처음 계획. 확정값은 아래 3-7: 접촉력 150 N (손가락은 환경과의 접촉만), 위치 오차 5°, 관절 속도 200 °/s. 관절 외력 토크 기준은 쓰지 않음)**:
    | 항목 | 임시값 | 근거 |
    |------|--------|------|
    | 관절 외력 토크 \|τ_drive − τ_예상(중력·관성)\| | 관절별 150 N × (관절 축 ↔ TCP 거리) | UR 토크 창 예시 (힘 한계 150 N) |
    | 팔 링크(손가락 제외) 접촉력 | 150 N | 위와 같은 힘 한계 |
    | 목표 ↔ 실제 관절 위치 오차 | 3° (임의) | UR 숫자 미공개. 정상 추종 오차 측정 후 조정 |
    | 관절 속도 | 180 °/s | UR5 관절 최대 속도 (URDF `velocity` π rad/s 와 같음, 넘으면 Cat 0) |
- 에피소드 리셋 서비스: 로봇 홈 자세, 드론 재배치(랜덤 시드 기록) — 3-6 완료
  - **홈 자세 복귀는 목표를 한 번에 바꾸지 않고 부드러운 궤적으로 이동** (큰 스텝은 오버슈트와 손목 흔들림 유발, 1-5 A 시험)
- **이미지 stamp ↔ 실제 화면 상태 지연 측정 (3-8)**: stamp 는 sim time 이지만 렌더 지연으로 이미지가 1프레임 전 물리 상태일 수 있다.
  팔을 일정 속도로 돌리며 화면 속 위치와 joint 값을 비교해 지연을 잰다

- **sim 카메라 안티에일리어싱(DLSS) 결정 — 6단계(데이터 수집) 전에** (2026-10-02, 9단계에서 옮김. 다 모은 뒤 바꾸면 이미지 분포가 달라짐):
  Isaac Sim 기본값 = `SimulationApp` `anti_aliasing: 3` (DLSS) + `rtx.post.dlss.execMode = 0` (Performance) → 640x480 카메라를 320x240 으로 렌더해 키움
  (3-1 GUI 경고 `DLSS increasing input dimensions`). 가는 경계(드론 프레임·프로펠러·손가락 끝)가 뭉개지거나, 시간 누적 방식이라 빠른 움직임에 잔상 가능.
  실물 RealSense 는 원래 해상도. DLSS Performance / Quality / DLAA(RTXAA) / TAA 비교 이미지(정지·팔 움직임) + RTF 로 정한다 (우리 코드는 아직 설정 안 함)
  - **비교 결과 (2026-10-02, `docs/sim_performance.md` 7장)**: RTX 실시간 렌더러는 DLSS·DLAA 만 지원 (TAA·끔 불가). DLAA 대비 평균 차이
    Performance 0.48~1.21 / Balanced 0.42~1.06 / Quality 0.35~0.90 (0~255), 경계 선명도 98~101%, 눈에 띄는 뭉개짐·잔상 없음.
    RTF (기하 제어기) Performance 1.01 / Balanced 0.99 / Quality 0.96 / DLAA 0.84. DLSS 모드는 새 stage 마다 기본값으로 돌아감 → stage 만든 뒤 설정
  - **결정 (사용자): DLSS Performance, Isaac Sim 기본값에 맡기지 않고 `ros2_iface.yaml` `render` 에 명시** (적용 후 다시 읽어 확인, 다르면 에러).
    4단계 에피소드 메타데이터에 렌더 설정 기록

**3-0 결과 (2026-10-02)**: `python.sh` 안 rclpy = 시스템 `/opt/ros/jazzy` (`rmw_fastrtps_cpp`, domain 기본), 외부 터미널에서 토픽 보임.
python.sh 실행 전에 `source /opt/ros/jazzy/setup.bash` 필요 (`ros2_iface.enable_ros2` 가 확인, 아니면 에러)

**3-1 결과 (2026-10-02, `measure_rtf.py`, headless, 기하 제어기 드론, 물리 120 Hz, RTX 4070 Ti SUPER)**
| loop(렌더) | 카메라 tickRate | RTF | update 1회 | 물리 구간 | 나머지(렌더·앱) | 카메라 (sim 기준) |
|---|---|---|---|---|---|---|
| 120 Hz | 끔 | 0.71 | 11.7 ms | – | – | – |
| 120 Hz | 0 (매 프레임) | 0.40 | 20.7 ms | – | – | 106 Hz |
| 120 Hz | 30 | 0.52 | 15.9 ms | 6.4 ms (1 스텝) | 9.5 ms | 30.0 Hz |
| 60 Hz | 30 | 0.81 | 20.7 ms | – | – | – |
| **30 Hz** | **30** | **0.94** | 35.6 ms | 23.5 ms (4 스텝) | 12.0 ms | **30.0 Hz**, 간격 최대 33 ms |
| 30 Hz | 끔 | 1.25 | 26.8 ms | 20.1 ms (4 스텝) | 6.7 ms | – |
- loop 30 Hz (= 카메라 주기) 가 맞다: 렌더·앱 처리가 카메라 한 번에 묶이고 카메라 주기는 그대로
- **남은 병목은 물리 스텝당 약 5~6 ms** (sim 1 s 에 0.6~0.7 s). PhysX 와 우리 Python 콜백(드론·drive)을 나눠 볼 것
- **받는 쪽 QoS**: best effort 로 받으면 640x480 이미지(0.9 MB)가 UDP 조각 손실로 통째로 버려져 sim 25 Hz, 0.6~1.2 s 끊김.
  RELIABLE 로 받으면 600/600 장 (위 표). 보내는 쪽(Camera Helper, realsense-ros)과 받는 쪽 중 하나라도 best effort 면 best effort 로 동작
- **GUI** (PX4, loop 30): 메인 뷰포트만 0.89 (+0.7 ms), 카메라 뷰포트 창 2개를 더 띄우면 0.75 (+7 ms, 같은 카메라를 또 렌더).
  **결정: 텔레옵 GUI 는 메인 뷰포트 하나, 손목·third view 는 ROS 토픽으로 본다** (`rqt_image_view` 등, 녹화 이미지 그대로)
- **Python 콜백 최적화** (PX4, loop 30, update 당 Python 6.3 → 2.4 ms. 렌더·PhysX 설정은 바꾸지 않음, 사용자 결정):
  - 드론 매 스텝 읽기·쓰기를 RigidPrim 대신 physics tensor view 직접 (`drone_flight.FastRigid`, 같은 값. warp 배열 생성·복사가 스텝당 18 번이던 것)
  - PX4 offboard 링크의 안 쓰는 스트림 끄기 (`px4_sitl.yaml` `commander.disable_streams`: HIGHRES_IMU 50 Hz, ATTITUDE_QUATERNION 50 Hz, ODOMETRY 30 Hz.
    onboard 모드는 sim 1 s 에 약 320 개를 보냄. 실물 companion 이 스트림을 고르는 것과 같음, 거부되면 에러)
  - 지자기 표 보간 캐시 (위경도 1e-5° ≈ 1.1 m 격자, 1 m 안 차이는 센서 잡음보다 훨씬 작음)
  - **결과: RTF 헤드리스 0.91 → 0.99, GUI(메인 뷰포트) 0.89 → 0.97**, 카메라 sim 30.0 Hz 그대로. 남은 시간 PhysX 약 18 ms + 렌더·앱 약 13 ms
  - 프로펠러 회전(GUI 기본으로 사용)을 켜면 GUI 0.94 (Articulation API 오버헤드 +1 ms, 지금은 최적화 안 함)
  - RTF 0.97 = 실제보다 3% 느림. stamp 가 sim time 이라 데이터 정합성은 문제없음. 실제 텔레옵에서 느리게 느껴지면 렌더·PhysX 를 다시 본다
- CPU 경고 `CPU performance profile is set to powersave` 는 잘못된 경보: `intel_pstate` 의 governor 이름이 powersave 일 뿐,
  EPP·전원 프로필은 performance, 부하 중 P코어 5.3 GHz(최대). 바꾸지 않는다

**3-2 결과 (2026-10-02)** — 실행 `isaacsim/scripts/sim_ros2.py` (드론 파지 씬 + ROS 2, 실제 시간 속도, GUI 는 메인 뷰포트 하나·프로펠러 회전 켬),
팔 인터페이스 `ros2_iface.ArmBridge`, 설정 `ros2_iface.yaml` `arm`
- `/clock`, `/joint_states` 를 물리 스텝마다 (PHYSICS_POST_STEP) 같은 sim time stamp 로 발행: **sim 기준 120.0 Hz**, stamp 간격 8.3 ms, `/clock` 단조 증가
  - `/joint_states` = 실물 UR 드라이버와 같게 팔 6 관절 (이름 `shoulder_pan_joint` … `wrist_3_joint`), position·velocity·effort.
    effort 는 실물이 관절 전류 [A], sim 은 관절 토크 [Nm] (PhysX projected joint force) — stage1 은 effort 를 쓰지 않음
  - 관절 값은 articulation tensor view 에서 직접 읽음 (실험용 API 오버헤드, 3-1)
- **카메라 stamp 와 joint stamp 가 같은 시계**: 카메라 343 장 중 339 장의 stamp 가 `/joint_states` stamp 와 정확히 같음 (같은 물리 스텝).
  나머지 4 장은 받는 쪽이 구독을 시작한 처음 0.14 s 에 joint 메시지를 못 받은 구간
- `/joint_command` (팔 6 관절 [rad]) 를 매 루프 받아 바로 drive 목표로. 계단 명령 응답: 움직이기 시작 wall 16~26 ms, 50% 24~35 ms, 90% 약 70 ms (drive 응답)
  - 잘못된 명령 (관절 이름이 팔 6 관절과 다름, 길이 불일치, 유한하지 않음, 관절 한계 밖) → 에러로 중단 (시험: 그리퍼 이름을 섞어 보냄 → 종료 코드 1)
- RTF (헤드리스) 기하 0.99, PX4 0.97~0.98
- 시험 클라이언트 주의: 한 rclpy 노드로 `/joint_states`+`/clock` (초당 240 개) 을 받으며 명령도 보내면 처리가 밀려 도착 시각이 늦게 기록된다
  (가짜 지연 220 ms). 3-8 검사는 stamp 기준으로 잰다

**3-3 결과 (2026-10-02)** — `ros2_iface.GripperBridge` (실물 `rh_gripper_node` 의 sim 버전), 설정 `ros2_iface.yaml` `gripper`.
실물 형식은 README.md·1DOF_gripper_data_collection.md 기준 (실물 노드 소스는 이 PC 에 없음)
- `/gripper/command` (std_msgs/Float64, raw) → goal latch → `GripperProfile`. 0~1150 밖·유한하지 않은 값은 에러 (실물 노드는 clamp, sim 은 조용한 fallback 금지)
- `/gripper/joint_states` (present raw, 0~1150 clamp = 실물 노드와 같음)·`/gripper/target` (goal raw), `name ['rh_p12_rn']`, **sim 30.00 Hz, 두 토픽 stamp 모두 같음**,
  stamp 는 모두 1/30 s 배수 (카메라와 같은 물리 스텝). 시작 시 열림 (goal 0 확인, 아니면 에러)
- 빈손 닫기: present 1135.5 에서 멈춤 (손가락 접촉 ≈ 1136), 2.23 s (profile 속도 계산 2.2 s), target 1150. 열기: present 1.5 (0.08°), target 0
- **ROS 로 드론 잡기** (grasp_demo `commands_success.csv` 를 `/joint_command`·`/gripper/command` 로 재생, 기하 제어기 드론):
  **present 289 에서 멈춤 (≈ 16.3°, grasp_demo 파지 각도와 같음), target 1150** → 실물과 같은 파지 신호
- `grasp_demo.py` 가 `commands_<케이스>.csv` (제어 주기마다 팔 관절 목표 + 그리퍼 goal raw) 를 남긴다 (ROS 재생 시험, 4단계 가짜 에피소드)
- 확인할 것 (3-8): **잡은 뒤 RTF 0.85~0.88** (기하 제어기 드론이 잡힌 채 계속 날려 해 접촉 계산이 무거움). 실제 시나리오(PX4, 잡은 뒤 모터 정지)로 다시 잰다
- ~~참고: 시험 클라이언트가 밀려 명령이 버려짐 (992 개 중 603 개 적용)~~ → **원인은 sim 쪽이었다 (2026-10-05, 고침)**: rclpy `spin_once` 가
  콜백·빈 호출을 번갈아 해서 루프마다 1 개만 처리했다. 빈 호출 두 번 연속일 때 멈추게 고침 (`docs/arm_command_interpolation.md` 5.1)

**3-7 결과 (2026-10-05)** — `isaacsim/scripts/protective_stop.py`, 설정 `ros2_iface.yaml` `protective_stop`, `sim_ros2.py --protective-stop on|measure`
- 물리 스텝마다 검사 (UR 보호 정지 = Cat 2 흉내): 걸리면 팔 목표를 그 순간 관절각에 고정, 이후 `/joint_command` 무시 (그리퍼 명령은 받음),
  `/protective_stop` (std_msgs/Bool) 을 1/30 s 배수 시각에 계속 발행. 해제는 리셋 서비스 (3-6)
  - 접촉력: 손가락이 아닌 링크는 모든 물체와의 순 접촉력, **손가락은 환경(테이블·바닥·third view 받침대)과의 접촉만**
    (처음엔 손가락을 통째로 뺐다가 손가락 끝으로 테이블을 찍는 충돌을 놓침. 손가락 view 는 직접 만듦: `RigidPrim(contact_filter_paths)` 는
    필터 수가 prim 수와 같으면 prim 마다 필터 하나씩 짝지어 버림)
  - 관절 속도는 관절각 차분 (PhysX 보고 관절 속도가 실제보다 작음)
- **측정하다 찾은 팔 명령 문제 3 가지를 고침** (`docs/arm_command_interpolation.md`): ① 명령이 30 Hz 계단으로 들어가 관절이 최대 속도로 뛰었다 멈춤
  → 선형 보간 (`arm.command_interp_time` 33 ms, deoxys 방식, 다른 프로젝트 조사 포함), ② sim 이 루프마다 명령을 1 개만 처리 (rclpy spin_once),
  ③ PhysX 관절 속도가 실제보다 작음 → `/joint_states` velocity 도 관절각 차분
- 기준 (정상 최대 → 기준, 충돌): 접촉력 0 N → **150 N** (테이블 317 N), 위치 오차 0.7°·최대 속도 근처 2.9° → **5°** (47.7°),
  관절 속도 180 °/s (= drive 최대) → **200 °/s** (drive 가 낼 수 있는 속도 위, 충격으로만 넘음)
- `on` 모드 확인: 테이블 충돌 → 손가락 끝 접촉 316.9 N 에서 정지, 이후 명령 322 개 무시. 팔 흔들기 2 배 (최대 172 °/s)·grasp success 재생 → 정지 안 함,
  파지 신호 그대로 (present 289, target 1150)
- `replay_commands.py`: 명령 CSV (grasp_demo `commands_<케이스>.csv` 등) 를 sim time 기준으로 `/joint_command`·`/gripper/command` 재생 (텔레옵 대신)

**3-6 결과 (2026-10-05)** — `/sim/reset` (std_srvs/Trigger), `isaacsim/scripts/sim_reset.py`, 설정 `ros2_iface.yaml` `reset`
- 서비스 콜백 안에서 sim 을 실제 시간 속도로 돌리며 리셋하고 끝나면 응답 (message = JSON: 시드, 새 드론 위치, 걸린 sim 시간).
  리셋 결과 = **에피소드 시작 상태** (팔 홈, 그리퍼 열림, 드론은 팔 위 호버). 처음 테이블에서 이륙하는 것은 PX4 부팅 과정일 뿐
- **항상 같은 경로 (순간이동, 사용자 결정)**: 명령 무시 → 보호 정지 해제 → 드론 kill → 이륙 지점으로 순간이동 + **PX4 재시작**
  → **팔·그리퍼를 홈·열림으로 순간이동** (`RobotDrive.reset_pose`) → 재이륙 → 새 호버 위치. **약 24 s**, 드론 위치 오차 6~9 mm (새 PX4 라 호버가 더 정확)
  - 매 에피소드 PX4 가 같은 깨끗한 상태로 시작 (EKF·센서 편향·제어기 적분값). 리셋은 데이터에 안 들어가므로 실물처럼 움직일 필요 없음
  - 새 호버 위치 = 씬 drone_pos ± 5 cm (시드 = `--seed` + 리셋 횟수). `sim_ros2.py --reset-offset <m>` (0 = 항상 기준 위치.
    PX4 는 그래도 호버 흔들림, 위치까지 고정하려면 `--flight geometric`, 기하 제어기 드론은 새 위치로 순간이동 후 다시 켬)
  - 팔 순간이동 뒤 속도 차분 기준(ArmBridge·보호 정지)도 새 자세로 맞춤: 안 맞추면 순간이동 거리 / dt 가 관절 속도로 잡혀 (22871 °/s) 보호 정지가 걸림
- 리셋 중 팔·그리퍼 명령은 무시하고, 리셋 중 쌓인 명령도 버린다 (명령 무시를 콜백 뒤 spin 이 쌓인 것을 꺼낸 다음에 풂)
- **리셋이 실패하면** `success=False` + 이유 (PX4 메시지 포함) 를 응답하고 **sim 은 계속**. 다음 리셋이 성공할 때까지 팔·그리퍼 명령 무시, 다시 호출해 재시도
- 처음 방식 (같은 날 바꿈): 날고 있는 드론은 이동 명령만 (약 7 s), kill 상태만 순간이동, 팔은 최소 jerk 궤적. 갈래마다 예외가 생겨 한 경로로 바꿈:
  1. 순간이동 뒤 EKF 수렴만 기다림 → 드론이 **팔 위로 떨어지면** EKF 가 가속도계 편향을 크게 잘못 잡아 "Preflight Fail: High Accelerometer Bias" 로
     60 s 넘게 arm 거부 (GUI 에서 발견, 헤드리스 3 회 재현) → PX4 재시작 (실물의 재부팅)
  2. 보호 정지가 테이블을 누른 자세에서 걸리면 접촉이 남아 정지를 풀자마자 다시 걸림 → (그때) 리셋 동안 접촉 기준 끔 → 팔 순간이동으로 필요 없어짐
  3. kill 된 드론이 빈손으로 닫힌 손가락 사이에 끼면 그리퍼가 안 열림 (check_ros2 에서 발견) → 드론을 먼저 치움 → 한 경로로 필요 없어짐
- `DroneFlight.teleport()` (순간이동 + 속도 0 + 가상 센서 차분 기준 지움), `DroneFlight.restart_px4()` / `PX4Bridge.restart()` / `PX4Commander.reset_link()`
- 확인: check_ros2 8 번 (대기 / 잡은 뒤 kill / 보호 정지 뒤 3 회 × 2 번 실행, PX4 드론을 실제로 잡은 채 kill 한 경우 포함) 모두 성공
- 남은 주의: 공중에서 kill 한 드론이 팔 위로 떨어지면 접촉력이 보호 정지 기준(150 N) 가까이 감 (141 N)
- (2026-10-12) 리셋이 되돌리는 팔 자세는 `--init-pose` (안 주면 홈). 5-B "에피소드 시작 자세"
- 5단계 주의: 리셋 뒤 텔레옵 장치는 로봇의 지금 자세(홈 또는 `--init-pose`)에서 시작해야 한다. 리셋 전 자세의 명령을 그대로 보내면 리셋 직후 팔이 그 자세로 33 ms 만에 뛴다

**3-8 결과 (2026-10-05)** — `isaacsim/scripts/check_ros2.py` (시스템 python3, sim 을 직접 띄워 ROS 2 로 시험, stamp 기준 판정) → **9/9 PASS**
| # | 항목 | 결과 |
|---|---|---|
| 1 | 토픽 주기 (sim 기준) | `/joint_states`·`/clock` 120.00 Hz, 카메라 2 대·그리퍼 2 개 30.00 Hz, `/protective_stop` 30 Hz, 끊김 0 |
| 2 | 같은 시계 | 카메라·그리퍼 stamp 가 모두 `/joint_states` stamp 와 일치 (안 맞는 것 0) |
| 3 | stage1 방식 이미지 읽기 | `rgb8` 640x480, `imgmsg_to_cv2(bgr8)` = 채널 뒤집기 (색 안 바뀜), frame_id `third_view_camera_color_optical_frame` |
| 4 | 팔 명령 응답 (계단 0.03 rad) | 움직이기 시작 33 ms, 50 % 50~58 ms, 오버슈트 0 % (0.2 rad 을 한 번에 주면 위치 오차 5° 초과로 보호 정지 = 설계대로) |
| 5 | 이미지 지연 | 팔이 움직이기 시작한 stamp = 이미지가 바뀐 첫 stamp (**0 프레임**): 이미지는 stamp 시각의 물리 상태. 8 번 실행 중 6 번 0 프레임, 2 번 1 프레임 (기준 ≤ 1): 변화 판정 기준 = max(정지 때 연속 이미지 차이 최댓값 × 3, 0.5) 라, 정지 때 차이가 큰 실행 (0.22·0.25, 보통 0.09~0.12) 은 기준이 0.66·0.75 로 올라 막 움직이기 시작한 첫 프레임의 작은 변화를 놓치고 다음 프레임에서 잡는다 (검사 민감도, 2026-10-05) |
| 6 | 파지 신호 (기하 제어기, 명령 재생) | present 297 / target 1150 |
| 7 | 보호 정지 | 재생 중 false, 테이블 충돌 2.8 s 에 true, 리셋 뒤 false |
| 8 | 리셋 3 회 (PX4: 대기 / 잡은 뒤 kill / 보호 정지 뒤) | 모두 순간이동 + PX4 재시작 약 24 s, 드론 오차 6~9 mm, 팔 홈 0.033°·그리퍼 열림·보호 정지 false |
| 9 | RTF (헤드리스, PX4, 카메라 2 대, 대기) | 0.96 |
- 기준 명령: `isaacsim/config/ros2_check/commands_success.csv` (grasp_demo 기하 제어기 success 명령, 만든 방법은 같은 폴더 README). 계단·테이블 충돌 명령은 스크립트가 만든다
- **잡은 채 RTF 가 낮다** (따로 잼): PX4 드론을 잡은 채 모터 켬 0.75, **모터 끔 (실제 수집 조건) 0.69**, 기하 제어기 드론이 잡힌 채 버팀 0.67.
  손가락·드론 접촉 계산이 무거움. stamp 정합성은 문제없고 텔레옵 조작감만 영향 (잡은 뒤 들고 옮기면 약 30 % 느림).
  PhysX 설정은 지금 안 바꿈 (사용자 결정) → 5단계 텔레옵에서 불편하면 원인(드론·손가락 충돌 형상, solver)을 찾는다
- B 의 grasp 재생은 잡힐 때도 빗나갈 때도 있음 (리셋이 드론 위치를 ±5 cm 랜덤으로 바꾸므로 고정 명령 재생). 어느 쪽이든 리셋 확인에는 상관없음

**완료 기준**
- [x] 카메라 렌더링 포함 real-time factor 측정·기록 (텔레오퍼레이션 조작감 기준) — 3-1, `docs/sim_performance.md` (대기 0.96~0.97, 잡은 채 0.69)
- [x] 모든 토픽 hz 가 목표에 맞음 (카메라 ≥ 25 Hz) — 3-8 (카메라 30.00 Hz)
- [x] 모든 header.stamp 가 sim time (카메라와 joint 가 같은 시계) — 3-8
- [x] stage1 방식(`imgmsg_to_cv2(bgr8)`)으로 읽은 sim 카메라 영상의 색이 바뀌지 않음 — 3-8
- [x] 보호 정지: 정상 파지·텔레옵에서는 걸리지 않고, 테이블에 일부러 부딪히면 걸림 — 3-7, 3-8
- [x] 드론을 잡았을 때 present 가 중간에서 멈추고 target 은 1150 (실물과 같은 파지 신호) — 3-3, present 289 / target 1150

---

## 4단계: 녹화·변환 수정

**목표**: sim 에피소드 → 병합된 v2.1 데이터셋이 자동으로 나옴

> **완료 (2026-10-05)**. 완료 기준 3 개 모두 충족 (`make_fake_episodes.py` 3/3 PASS). 정리 문서: **`docs/data_recording.md`** (녹화 토픽·주기·구조도·명령),
> 변환·병합 옵션: `README.md`·`dataset_merge.md`, 명령 모음: 부록 D
> - 녹화 `record_toggle.py [--sim]` (새 토픽, 녹화 전 발행자 확인, 카메라 RELIABLE, r → `/sim/reset` → 녹화, k → `/sim/drone_kill`, `episode.json`)
> - stage1 (`--arm-action`·`--base-imu` 필수, 그리퍼 0~1, 메시지 나이 검사 66 ms, 첫 명령 이전 구간 제외) → 병합 (보호 정지 제외, 정지 구간 자르기는 옵션) → 검수 `inspect_dataset_v21.py`
> - 성공/실패 판정은 사람 (`d` 로 버림). conda `lerobot_v2` (lerobot 0.3.3). v3.0 변환은 뺌
> - **남은 문제** (아래 "4단계에서 남은 문제"): ~~녹화 시작 직후 메시지 누락~~ (2026-10-06 고침: 발행·수신 큐를 3 s 분량으로), 
>   ~~실물 PC 에서는 새 파이프라인을 아직 돌려 보지 못함~~ (2026-10-06 노트북에서 카메라·그리퍼로 녹화 → 변환 → 검수 5/5. 팔·텔레옵 경로는 아직),
>   **실물 stamp 동기화 (D1)**: 카메라 ↔ 그리퍼는 −50 ~ 0 ms 로 범위만 확인, 카메라 ↔ 팔은 새 팔이 오면 측정 (`docs/timestamp_sync.md`)

**확정 계획 (2026-10-05 사용자 결정)**
| 단계 | 내용 |
|------|------|
| 4-0 | `config/cameras.yaml` (역할 ↔ 실물 시리얼·sim 설정) + 공용 로더 `camera_config.py`, 옛 sim smoke bag 삭제, 이 PC 에 conda `lerobot_v2` (Python 3.10, lerobot 0.3.3) |
| 4-1 | `record_toggle.py`: 새 토픽 (`/joint_command`, `/cam/...`, sim 전용 `/protective_stop`·`/clock`), 카메라 RELIABLE, `--sim` (r → `/sim/reset` → 성공 응답 뒤 녹화 시작), bag 폴더에 `episode.json` |
| 4-2 | `/sim/reset` 응답 JSON 에 에피소드 정보 추가 (드론 제어기·위치 정보 방식·설정 파일, 렌더 설정, 그리퍼 max_force, 카메라 prim, `cam_tilt`, git commit, PX4 로그 경로) |
| 4-3 | stage1: `/cam/...` 고정 표, `--arm-action command\|next_state` (필수), 그리퍼 /1150, `--base-imu const`, 이미지 나이 검사 (66 ms), 메타데이터 (`episode.json`, 보호 정지) |
| 4-4 | stage2 v2.1 (`wrist`·`third_view`, `observation.base_imu`), 병합 (`_discarded`·보호 정지 에피소드 제외), `inspect_parquet.py` 기준, 에피소드별 `/joint_command` 간격 통계 (중앙값·최댓값·40 ms 넘은 횟수: 텔레옵 명령이 끊긴 에피소드 찾기) |
| 4-5 | 가짜 에피소드 자동 생성 (리셋 → 녹화 → 명령 재생 → kill → 녹화 끝) → stage1 → 병합 → 검수 |
| 4-6 | 문서 (PLAN·README·CLAUDE.md, 다른 문서의 낡은 부분, 실물 카메라 런치 토픽 변경 안내) |

- **성공/실패 판정은 사람이 한다** (자동 판정 없음): 녹화는 r 토글 (r ~ r = 에피소드 1 개), 실패한 에피소드는 `d` 로 `bags/_discarded/` 로 버린다. 남은 것 = 성공 (sim·실물 같음)
- **`/joint_command` 의 `header.stamp` 는 보내는 쪽이 넣는다** (sim 텔레옵 = sim time (`use_sim_time`), 실물 = PC 시각. 다른 토픽과 같은 규칙).
  stage1 은 이 stamp 로 팔 action 을 맞추고, stamp 가 0 이면 에러. 4-1 에서 재생 스크립트가 stamp 를 안 넣어 전부 0 으로 녹화된 것을 찾음
  → `replay_commands.py` 가 보낸 순간의 sim time 을 넣게 고침. **5단계 텔레옵 노드 요구사항**
- **드론 모터 정지는 녹화 도구의 `k` 키** (`--sim`, 서비스 `/sim/drone_kill`, 시각을 `episode.json` 에 기록). 텔레옵 중에는 다른 터미널에 명령을 칠 수 없어서.
  (2026-10-14: PICO 컨트롤러 X 버튼도 같은 동작. 녹화 도구를 거치므로 시각이 똑같이 기록된다. 리셋 = B, 녹화 = A, 5-B "컨트롤러 버튼")
  정책의 action 은 아니지만 언제 끄느냐에 따라 그 뒤 이미지·관절 상태가 달라지므로 수집할 때 일관되게 한다. 텔레옵 장치 버튼으로 옮길지는 5단계에서. 자동 kill 은 안 함 (판정은 사람)
- **보호 정지가 걸린 에피소드**는 `meta.json` 에 항상 기록하고 병합에서 기본 제외 (옵션을 줄 때만 포함)
- **앞뒤 정지 구간 자르기**는 병합의 명시 옵션 (기본 끔). 검수 스크립트가 에피소드별 앞뒤 정지 길이를 출력 → 실제 텔레옵 데이터를 보고 결정.
  r 을 누른 뒤 텔레옵 장치를 잡기까지 팔이 멈춘 프레임이 쌓이면 정책이 "시작하면 가만히 있기" 를 배울 수 있다
- **예전 실물 데이터 (옛 토픽 `/d435i`·`/d456`, freedrive, `head` 키) 는 다시 변환하지 않는다** (고장 난 팔의 가짜 데이터, 새 팔로 교체 예정).
  stage1 은 새 토픽만 받는다. v3.0 변환 스크립트는 손대지 않는다 (학습은 v2.1)
- **실물 카메라 (지금)**: wrist = D435i (843112074130), third_view = D456 (252122301126). 두 번째 D435i 가 오면 `cameras.yaml` 의 third_view 만 바꾼다.
  실물 런치(`realsense_dual_camera`, 실물 PC 에만 있음)가 `/cam/wrist/...`·`/cam/third_view/...` 로 발행하도록 실물 PC 에서 고친다
- **sim third view 카메라는 나중에**: 모양은 D435i 메시, 화각은 D456 가정값 (가로 약 90°, 임시). 실물 구성이 정해지면 모양·intrinsics·위치를 같이 바꾼다
  (6단계 수집 전에. `realsense2_description` 에는 D455 메시만 있고 D456 은 없음)
- lerobot 환경: conda `lerobot_v2` (실물 PC 기록과 같게 Python 3.10 + `pip install "lerobot==0.3.3"`). conda 는 `~/miniconda3`, base 자동 활성화 끔
  (켜져 있으면 `~/isaacsim/python.sh`·ROS 의 python 과 섞일 수 있음)

**4-0 결과 (2026-10-05)**: `config/cameras.yaml` + `camera_config.py`, 옛 sim smoke bag 삭제, conda `lerobot_v2` (Python 3.10.21, lerobot 0.3.3, `CODEBASE_VERSION` v2.1)

**4-1 결과 (2026-10-05)** — `record_toggle.py` (`./6_record_bag.sh <작업명> --sim`), sim 서비스 `/sim/drone_kill`
- 토픽: 상태 (`/joint_states`, `/gripper/joint_states`, `/gripper/target`, 카메라 `image_raw`·`camera_info`) + 명령 (`/joint_command`, `/gripper/command`),
  `--sim` 이면 `/protective_stop`·`/clock`. 카메라 토픽은 `cameras.yaml`
- 녹화 전 검사: 상태 토픽에 발행자가 없거나 카메라 발행자가 RELIABLE 이 아니면 녹화를 시작하지 않음. 카메라는 QoS override 로 RELIABLE 수신
- `--sim`: `r` → `/sim/reset` → 성공 응답 뒤 녹화 시작 (실패·서비스 없음이면 녹화 안 함), `k` → 드론 모터 정지. bag 폴더에 `episode.json`
  (모드, 토픽, 카메라 역할 → 장치, 리셋 응답 (시드·드론 위치), kill 시각, 녹화 길이)
- 확인 (GUI 에서 사용자가 r·d·q, 헤드리스에서 같은 함수로 리셋 → 녹화 → 명령 재생 → kill → 종료): bag 5 개 (25~42 s, 기하·PX4) 모두
  카메라 2 대 장수 같음·30.00 Hz·끊김 0, `/joint_states` 120.00 Hz, 그리퍼 30.00 Hz, stamp 0 인 메시지 없음. 빗나간 에피소드 (present 1133) 는 `d` 로 `_discarded/` 로
- **[고침] 기하 제어기 드론에서 리셋 응답 `duration_s` 가 음수** (−95 s 등): 기하 제어기를 다시 켜면 드론 내부 시계 (`flight.t`) 가 0 으로 돌아가는데 리셋이 그 시계로 쟀다
  → sim time 으로 (기하 2.03 s, PX4 23.87 s). `t_start` 도 sim time
- **[알아 둘 것] 재생 스크립트 (`replay_commands.py`) 는 명령을 고르게 보내지 못한다**: 60 Hz 명령 (간격 16.67 ms) 992 개 중 **간격이 40 ms 를 넘은 것이 38~64 번 (4~6 %)**,
  최대 41.7 ms (= `/clock` 5 칸, 정상 2 칸. 한 명령이 25 ms 늦게 나가고 다음 것이 바로 따라감). 빠진 명령은 없음 (992 개 모두 녹화)
  - 원인: 이 스크립트는 `/clock` 을 받아 "sim 시각이 다음 줄 시각을 넘으면 보냄" 으로 동작. sim 은 물리 4 스텝을 한꺼번에 계산해 `/clock` 4 개가 실제 시간 33 ms 마다 뭉쳐 오고,
    한 노드가 `/clock` (120 Hz)·그리퍼 토픽 2 개를 받으며 보내기도 해서 가끔 한 묶음 밀린다. sim 쪽 문제가 아님
  - 데이터 정합성: stamp 는 실제로 보낸 순간이라 늦은 명령은 늦은 stamp 로 기록된다 → 데이터셋 action (프레임 직전 최신 명령) = sim 이 실제로 받은 명령.
    파지 결과도 같음 (present 282~291, target 1150)
  - 학습 영향: 재생 에피소드는 파이프라인 확인용 가짜 데이터라 학습에 안 쓴다. 실제 텔레옵 노드 (5단계) 는 `/clock` 을 기다리지 않고 자기 타이머로 일정하게 (50 Hz 이상) 보낸다.
    텔레옵 데이터에서 같은 일이 생기면 오차 = 팔 속도 × 늦은 시간 (30 °/s 에서 25 ms = 0.75°). 성능을 실제로 떨어뜨리는 것은 명령 주기가 25 Hz 보다 느리거나 자주 끊길 때
    (같은 action 이 반복되는 계단) → 4-4 검수 스크립트가 에피소드별 `/joint_command` 간격 통계를 출력해 찾는다
- **bag 용량**: 30 s 에피소드 하나 약 1.5 GB (압축 없는 640x480 이미지 2 대, 약 55 MB/s). 디스크 여유 156 GB → 약 100 개.
  **그대로 둔다 (2026-10-05 사용자 결정): 압축·자동 삭제 없이 bag 은 사용자가 직접 지운다**

**4-2 결과 (2026-10-05)** — `/sim/reset` 응답 JSON 에 에피소드 정보 (`sim_ros2.episode_info` → `sim_reset.py`). record_toggle 이 `episode.json` 의 `sim_reset` 에 그대로 저장
- 리셋마다 바뀌는 것: `seed`, `drone_goal`·`drone_pos` (world) + **`drone_goal_base`·`drone_pos_base` (로봇 base 기준, 0-3)**, `t_start`·`t_end` (sim time), `duration_s`, `px4_log` (그 에피소드의 PX4 로그 폴더)
- 고정 정보 `sim`: git (commit·branch·수정 여부), 물리·루프 주기, 렌더 (DLSS Performance), 로봇 (설정 파일, base 위치, 홈 자세, 명령 보간 시간),
  그리퍼 (`max_force_nm` 2.28, 목표값 이동 속도, 손가락 마찰), 드론 (설정 파일, 제어기, 비행 모드, 위치 정보 방식, 기준 위치, 시드),
  카메라 (tickRate, 역할별 prim 경로·설정 파일·intrinsics, 손목 `cam_tilt_deg` 0, third view 위치·look_at), 보호 정지 (모드·기준 3 개)
- 로봇 base = 로봇 최상위 prim (world 와 축이 같고 z 만 테이블 상판 0.762 m 위)
- **[고침] 리셋 기준 위치가 `--drone-pos` 를 무시하던 것**: 리셋은 항상 씬 설정 `drone_pos` 를 기준으로 했다 → 시작할 때의 드론 위치 (`--drone-pos` 또는 씬 설정) 를 기준으로
- 회귀: `check_ros2` 9/9 PASS
  - 메모: 4-2 직후 첫 실행에서 6 번 (파지 신호) 이 한 번 FAIL 했다 (검사가 받은 `/clock` 이 뒤로 감: 잡은 채 RTF −0.277, 마지막 1 s present 변화 296. sim 로그는 정상).
    원인은 확인하지 못했고, 그 뒤 여러 번 다시 돌려 재발하지 않아 남은 문제에서 뺐다 (2026-10-06). 같은 증상이 다시 보이면 같은 토픽을 발행하는 다른 프로세스 (다른 sim, `ros2 bag play`) 부터 확인
  - **sim 을 띄운 채 sim bag 을 재생하지 말 것**: bag 에 `/clock`·그리퍼 토픽·`/joint_command` 가 들어 있어 옛 시각·옛 명령이 다시 발행된다 (로봇이 옛 명령대로 움직임)

**4-3 결과 (2026-10-05)** — `lerobot_stage1_extract_bag.py <bag> --arm-action command|next_state --base-imu const|topic` (둘 다 필수)
- 입력 = record_toggle 로 녹화한 bag (`episode.json` 필수, sim 은 4-2 이후 녹화분). 카메라 토픽은 `cameras.yaml` → `wrist/`·`third_view/`
- 팔 action `command` = `/joint_command` 의 프레임 직전 최신 명령 (stamp 0 이면 에러). **첫 명령 이전 구간은 에피소드에서 뺀다** (사용자 결정: 값을 지어내지 않고,
  시작 직후 가만히 있는 구간도 빠짐. 버린 프레임 수는 `meta.json` `dropped_frames_at_start`). `next_state` = 기존 방식
- 그리퍼 /1150 (present 가 0~1150 밖이거나 target 이 0·1150 이 아니면 에러), `base_imu.npy` (N, 6): `const` = [0, 0, 0, 0, 0, 9.81]
  (bag 에 `/base/imu` 가 있으면 에러), `topic` = 구간 평균 (**IMU 데이터가 없어 돌려 보지 못함**, 8단계에서 확인)
- **메시지 나이 검사** (`--max-age`, 기본 66 ms): 격자 시각 직전 최신 메시지가 이보다 오래됐으면 에러. 카메라뿐 아니라 팔·그리퍼 상태에도 적용
- `meta.json`: 옵션, 카메라 역할 → 장치 (sim = prim·intrinsics, 실물 = 모델·시리얼), 고른 메시지 나이 최댓값, `/joint_command` 간격 통계
  (중앙값·최댓값·40 ms 넘은 횟수), 보호 정지 (발생 여부·sim 시각·프레임. Bool 에 header 가 없어 bag 도착 시각을 `/clock` 으로 sim time 환산), 드론 kill 시각·프레임, `episode.json` 전체
- 출력 폴더가 이미 있으면 에러 (`--overwrite` 로 지우고 다시)
- 확인 (PX4 드론 bag 1 개: 리셋 → 녹화 → grasp 재생 → kill): 572 프레임 (첫 명령 이전 0.96 s = 23 프레임 뺌), float32, 그리퍼 state 0.000~0.262·action {0, 1},
  action 이 state 보다 1 프레임 앞섬 (명령 → 움직임), 고른 메시지 나이 최대 카메라·그리퍼 31.7 ms·팔 8.3 ms, kill 프레임 524, 보호 정지 없음.
  에러 확인: 필수 인자 없음, 4-2 이전 bag (sim 정보 없음), `--base-imu topic` 인데 토픽 없음, `--max-age 0.02`, 출력 폴더 있음
- **[찾은 것] 녹화 시작 직후 메시지가 빠질 수 있다**: 이 bag 은 녹화 시작 뒤 0.03~0.4 s 구간의 카메라 2 대 이미지 10 장 (367 ms) 과 `/joint_states` 8 개가 없다
  (녹화기가 뜨는 0.7 s 동안 받지 못하고 그 뒤 한꺼번에 받음). 같은 조건으로 다시 재 보니 18 번 중 0 번 (기하 12, PX4 6) → **드물다 (원인 확인 못 함, 지금까지 약 27 번 중 1 번)**
  - 이 bag 에서는 첫 명령 (녹화 0.96 s 뒤) 이전이라 에피소드 밖. 에피소드 안에 걸리면 나이 검사가 에러로 멈춘다 (조용히 옛 이미지를 쓰지 않음)
  - 그런 bag 을 살리는 명시 옵션 `--skip-start <s>` (녹화 시작 뒤 그만큼 뺌, `meta.json` 에 기록). 확인: `next_state` 로 변환하면 프레임 1 에서 에러 (이미지 73 ms 전),
    `--skip-start 0.5` 면 통과
- 남은 것: `convert_ros2bag_lerobot.sh` 는 stage1 필수 인자를 아직 안 넘긴다 (4-4 에서)

**4-4 결과 (2026-10-05)** — conda `lerobot_v2` 에서 실행 (`env -u PYTHONPATH python ...`)
- 공용 모듈 `lerobot_v21_common.py` (중간 파일 읽기·스키마 검사·feature 정의·프레임 추가). stage2 (에피소드 1 개) 와 병합이 같은 코드를 쓴다
- `lerobot_stage2_build_dataset_v21.py`·`lerobot_merge_episodes_v21.py`: feature = `observation.state` (7), `action` (7), **`observation.base_imu` (6)**,
  `observation.images.wrist`·`observation.images.third_view`. 중간 파일이 `schema_version 2` 가 아니면 에러
- 병합: 매니페스트 또는 `--all` (중간 파일 폴더 전체). 사전검사에서 fps·이미지 크기·**팔 action 종류**·base_imu 방식이 에피소드끼리 다르면 중단.
  **보호 정지 에피소드는 기본 제외** (`--include-protective-stop`), 제외한 것은 출력과 `meta/merge_manifest.json` 에 남김.
  `_discarded` 의 bag 은 stage1 을 돌리지 않으므로 중간 파일이 없어 들어가지 않는다
- **정지 구간 자르기** `--trim-idle start|end|both` (기본 안 함), `--trim-margin` 0.5 s, `--idle-eps-deg` 0.1.
  정지 = **action 과 state 가 모두** 첫 / 마지막 프레임 값에서 그대로인 구간. 처음에 action 만 봤더니 닫기 명령 뒤 그리퍼가 닫히는 2 s 가 정지로 잡혀
  뒤를 자르면 파지 장면이 잘렸다 (그리퍼 state 0.191 에서 끊김) → state 도 보게 고침 (뒤 정지 233 → 33 프레임, 자른 뒤에도 0.262)
- 검수 `inspect_dataset_v21.py <데이터셋> [--load]`: 1 형식 (v2.1, fps 25, feature 키·크기), 2 에피소드·프레임·mp4 수, 3 값 범위 (유한값, 팔 ±2π,
  그리퍼 state 0~1, action {0, 1}), 4 파지 신호 (닫기 명령 중 state 최댓값 < 0.95. sim 빈손 닫기는 0.985), 5 LeRobotDataset 로드 (비디오 디코딩 포함).
  에피소드별 표: 프레임 수, 그리퍼, 파지 신호, 앞·뒤 정지 [s], `/joint_command` 간격 (중앙값·최댓값·40 ms 넘은 횟수). FAIL 이 있으면 종료 코드 1
- `inspect_parquet.py`: 그리퍼 기준을 0~1 / {0, 1} 로
- `convert_ros2bag_lerobot.sh <bag> --arm-action … --base-imu …` (필수 인자 전달). **v3.0 단계는 뺐다**: `lerobot_stage2_build_dataset_v30.py` 는 손대지 않았고
  옛 중간 파일 형식 (head/, 그리퍼 raw) 용이라 지금 stage1 출력을 읽지 못한다 (학습은 v2.1)
- 확인 (PX4 bag 1 개): 변환 스크립트 한 번으로 bag → 중간 파일 → v2.1, 검수 4/4 (`--load` 포함 5/5): 572 프레임, 그리퍼 state 0.000~0.262, 파지 신호 있음,
  정지 앞 0.08 s / 뒤 1.32 s. 병합 옵션: 팔 action 이 다른 에피소드 → 중단, 보호 정지 표시한 복사본 → 제외, `--trim-idle both` → 551 프레임
- 남은 것: 에피소드 여러 개 병합과 진짜 보호 정지 bag 은 4-5 에서

**4-5 결과 (2026-10-05)** — `python3 isaacsim/scripts/make_fake_episodes.py` (시스템 python3, 약 6 분) → **3/3 PASS**
- sim 을 헤드리스로 직접 띄우고 (기하 제어기 드론, `--reset-offset 0`: 고정 명령 재생이 매번 잡히게), record_toggle 의 함수로 에피소드 5 개를 자동 녹화:
  성공 3 (리셋 → 녹화 → `commands_success.csv` 재생 → 드론 kill → 3 s → 종료), 보호 정지 1 (테이블 충돌 명령), 버림 1 (녹화 뒤 `_discarded/` 로)
  → 남은 bag 4 개 stage1 → 매니페스트 병합 (conda `lerobot_v2`) → `inspect_dataset_v21.py --load`
- 완료 기준:
  1. 여러 에피소드가 v2.1 데이터셋 하나로: 에피소드 3 개·1740 프레임 (581·579·580). 보호 정지 에피소드는 `meta.json` 에 기록 (녹화 2.68 s·프레임 67 에서 발생) 되고 병합에서 제외,
     버린 에피소드는 `_discarded/` 에 있고 변환하지 않아 들어가지 않음
  2. `meta/info.json`: `codebase_version` v2.1, 카메라 키 `observation.images.wrist`·`observation.images.third_view`, `observation.base_imu` [6]
  3. 검수 5/5: 그리퍼 action {0.0, 1.0}, state 0.000~0.257, 이상치 0, 세 에피소드 모두 파지 신호 (닫는 중 state 최대 0.252~0.257), LeRobotDataset 로드 (길이 1740, 이미지 (3, 480, 640))
- 에피소드별: 고른 메시지 나이 최대 카메라·그리퍼 31.7 ms, kill 프레임 524, `/joint_command` 간격 40 ms 넘은 것 44~64 번 (재생 스크립트, 4-1), 정지 앞 0.08~0.12 s / 뒤 약 9.1 s
  (잡은 뒤 가만히 들고 있는 구간. 기하 제어기 드론은 kill 뒤에도 그리퍼 값이 거의 안 변함)
- 중간 파일 이미지를 눈으로 확인: `wrist/` = 손목 시점 (드론 아랫면), `third_view/` = 외부 시점 (팔이 드론을 잡은 모습), 색 정상
- 리포트·로그 `isaacsim/reports/fake_episodes_<시각>/`, 데이터셋 `lerobot_dataset_v21/local/fake_<시각>/`. 옵션 `--success N --pstop N --discard N --use-running-sim`

**완료 기준 확인**: 아래 세 항목 모두 4-5 에서 충족 (2026-10-05)

**작업 (처음 계획, 결과는 위 4-0 ~ 4-5. `fake_gripper_cameras_sim.py` 는 고치지 않고 지움: sim_ros2.py 가 진짜 토픽을 냄)**
- `record_toggle.py`: `TOPICS` 에 `/joint_command` 추가, 녹화 시작 시 리셋 서비스 호출 연동
  - **카메라는 RELIABLE 로 받는다** (best effort 면 이미지가 통째로 버려짐, 3-1 결과). 실물 녹화 QoS 도 확인
    (`ros2 topic info -v /d435i/d435i/color/image_raw`, 실물 30 Hz 카메라가 20~25 Hz 로 떨어지는 원인일 수 있음)
- stage1
  - **이미지 나이 검사**: 25 Hz 격자 시각 t 와 고른 이미지 stamp 의 차이가 기준(예: 66 ms = 2 프레임)을 넘으면 에러.
    지금은 이미지가 빠지면 `latest_at` 이 옛날 이미지를 조용히 고른다 (옛 이미지 + 새 joint). 실물에도 적용
  - 카메라 매핑: 고정 표 `/cam/wrist/...` → `wrist`, `/cam/third_view/...` → `third_view` (토픽 없으면 에러)
  - `fake_gripper_cameras_sim.py` 의 카메라 토픽을 `/cam/wrist/color/image_raw`(D435i 역할), `/cam/third_view/color/image_raw`(D456 역할)로 변경.
    stage1 카메라 매핑 변경과 같은 커밋에서 할 것
  - 팔 action 소스 옵션: `--arm-action command|next_state` 필수 인자 (sim·텔레옵 = `command`, 기존 실물 freedrive = `next_state`)
  - 그리퍼 정규화: `state[6] = present / 1150` (0~1 연속), `action[6] = target / 1150` ({0.0, 1.0})
  - 베이스 IMU: `/base/imu` → `base_imu.npy` (N, 6). 25 Hz 로 내릴 때 zero-order hold 대신 **구간 평균**. 토픽이 없을 때 상수로 채우는 것은 명시적 옵션(`--base-imu const`)일 때만 (조용한 fallback 금지)
- stage2(v2.1): `observation.base_imu` (float32, (6,)) feature 추가
- 에피소드 메타데이터: 성공 여부, 드론 모드·파라미터, 랜덤 시드, 카메라 역할 → 시리얼(sim 은 prim 경로), 카메라 장착 기준(`cam_tilt` 포함)
- `inspect_parquet.py` 검수 기준 갱신
- 에피소드 병합 스크립트 (`_discarded` 제외), 앞뒤 정지 구간 트리밍

**완료 기준**
- [x] 스크립트로 움직인 가짜 에피소드 여러 개가 하나의 v2.1 데이터셋으로 병합됨 — 4-5
- [x] `meta/info.json` 의 `codebase_version` = `v2.1`, 카메라 키 2개, `observation.base_imu` (6,) 포함 — 4-5
- [x] 검수 통과: action 그리퍼 값 종류 = {0.0, 1.0}, state 그리퍼 0~1 이내, 이상치 0, 파지 신호(action=1 인데 state 가 중간에서 멈춤) — 4-5

**4단계에서 남은 문제 (2026-10-05)**
| # | 문제 | 빈도·상태 | 영향·대응 |
|---|------|-----------|-----------|
| A1 | ~~녹화 시작 직후 메시지 누락~~ → **고침 (2026-10-06, 아래 "A1 추적")**: 양쪽 큐 (sim 발행·녹화기 수신) 깊이 10 이 넘쳐서 버려짐 → 3 s 분량으로 | 고친 뒤 90 번 중 0 번 | 실물은 드라이버의 발행 큐를 바꿀 수 없어 남을 수 있다 (stage1 이 에러로 잡고 `--skip-start` 로 살림). 녹화 중에는 새 노드를 띄우지 말 것 |
| B1 | 재생 스크립트가 명령을 고르게 못 보냄 (4-1): 60 Hz 명령의 4~6 % 가 25 ms 늦음 | 원인 앎 (`/clock` 을 받아 보내는 방식) | 학습 데이터에 안 씀. 텔레옵 데이터는 검수 표의 `/joint_command` 간격으로 확인 |
| B2 | stage1 이 bag 전체를 메모리에 올림 (30 s 에피소드 약 1.5 GB) | 기존 방식 | 에피소드가 몇 분으로 길어지면 메모리 부족 가능 → 그때 순차 읽기로 |
| B4 | **재생 스크립트가 보낸 명령을 sim 이 한동안 못 받다가 받으면 팔 목표가 뛰어 보호 정지** (2026-10-08, `make_fake_episodes.py` 에피소드 1 개: 1.7 s 못 받음 → 36° 뜀) | 이번에 처음 봄 (그날 2 번 실행, 재생 10 번 중 1 번). `replay_commands.py` 가 sim 구독 연결을 기다린 뒤 시작하게 고침, 원인 (연결 전 / 전달 멈춤) 은 못 가림 | 가짜 데이터용 도구의 문제. 텔레옵 노드는 녹화 전에 띄워 두고 지금 자세에서 시작 (5단계). 다시 나오면 `make_fake_episodes.py` 가 1 번 기준 FAIL 로 잡는다 |
| B3 | 5 번 이미지 지연 검사 민감도: 8 번 중 2 번이 1 프레임 | 기준 안 (3-8 표) | 검사가 첫 프레임 변화 값을 리포트에 남기게 하면 확인 가능 |
| C1 | ~~실물 PC 에서 새 파이프라인을 돌려 보지 못함~~ → **확인함 (2026-10-06, 아래 "실물 PC 확인")**: 카메라 2 대 + 그리퍼 + 가짜 관절값으로 녹화 → stage1 (`next_state`) → v2.1 → 검수 5/5 | 팔·텔레옵 (`/joint_command`) 경로는 못 봄 | 새 팔·텔레옵 장치가 오면 `--arm-action command` 로 다시 확인 |
| C2 | ~~실물 카메라 프레임 드롭 (30 Hz 설정에서 20~25 Hz)~~ → **카메라 문제가 아니었다 (2026-10-06)**: 발행은 30 Hz, `ros2 topic hz` 가 `image_raw` 를 best effort 로 받아 낮게 보인 것. RELIABLE 녹화는 29.99 Hz | 해결 | 주기는 `camera_info` 로 잴 것. 나이 기준 66 ms 는 실물에서도 통과 (카메라 최대 33.4 ms, 그리퍼 48.2 ms) |
| C3 | `--base-imu topic` (구간 평균), PX4 드론 에피소드 여러 개의 병합 | 못 돌려 봄 (IMU 데이터 없음 / 고정 명령 재생이 PX4 드론을 매번 잡지 못함. PX4 bag 1 개는 변환·검수함) | 8단계 / 5단계 텔레옵 데이터로 |
| D1 | **실물 stamp 동기화**: 카메라·팔·그리퍼·텔레옵 명령의 `header.stamp` 를 서로 다른 프로그램이 찍는다 (아래 "실물 stamp 동기화 문제") | **카메라 ↔ 그리퍼는 1 차 측정함 (2026-10-06): −50 ~ 0 ms 사이** (이미지 stamp 가 같거나 이름, 정확한 값은 못 정함). 카메라 ↔ 팔은 재지 않음 (팔 없음) | 그리퍼는 한 프레임 (40 ms) 안팎이고 느린 장치 (닫는 데 2 s) 라 영향이 작다. **팔 ↔ 카메라가 중요 → 새 팔이 오면 측정** |

**A1 추적: 녹화 시작 직후 메시지 누락 (2026-10-06)**
- 디스크의 bag 12 개 중 누락은 1 개: 녹화기가 녹화 0.05~0.75 s 동안 카메라를 받지 못하다가 한꺼번에 받았고, 카메라 2 대가 정확히 10 장씩 없음 (`/joint_states` 7 개)
- **재현**: 리셋 → 녹화 시작 → **곧바로 다른 ROS 노드 (명령 재생) 를 띄움** → 종료. 새 노드를 안 띄운 앞선 18 번은 0 번이었다
- **원인**: 녹화가 시작된 직후 새 노드가 나타나면 0.4~1.5 s 동안 녹화기로 가는 전달이 멈추고 (왜 멈추는지는 확인 못 함. 추정: 녹화기·DDS 가 새 노드의 토픽을 찾아 연결하는 동안),
  그동안 메시지는 큐에 쌓이는데 **발행 큐·수신 큐 깊이가 둘 다 10** (카메라 0.33 s, `/joint_states` 0.08 s 분량) 이라 넘친 만큼 버려진다.
  RELIABLE 이어도 발행 큐 (keep_last) 에서 밀려난 메시지는 다시 보내지 않는다

  | sim 발행 큐 | 녹화기 수신 큐 | 누락 있는 bag |
  |---|---|---|
  | 10 (기존) | 카메라 10 (기존) | **10 / 30** |
  | 10 | 3 s 분량 | 6 / 30 |
  | 2 s 분량 | 카메라 10 | 2 / 30 (한 번은 `/joint_states` 135 개·그리퍼 36 개: 전달이 1.4 s 멈춤) |
  | 2 s 분량 | 3 s 분량 | **0 / 30** |
  | **3 s 분량 (확정)** | **3 s 분량 (확정)** | **0 / 60** |

- **고침**: `ros2_iface.yaml` `publish_queue_sec: 3.0` (발행 큐 깊이 = 이 시간 × 토픽 주기: `/joint_states`·`/clock` 360, 30 Hz 토픽·카메라 90. Camera Helper `queueSize` 포함),
  `record_toggle.py` `QUEUE_SEC = 3.0` (상태 토픽마다 QoS override 로 수신 큐를 늘림. 카메라는 RELIABLE, 다른 토픽은 발행자와 같은 reliability).
  **양쪽 다 늘려야 없어진다**. sim 메모리는 최악의 경우 약 170 MB 더 (카메라 90 장 × 2 대). `check_ros2` 9/9 PASS
- 남은 한계: ① 전달이 멈추는 이유 자체는 모른다 (멈춰도 잃지 않게 했을 뿐). 3 s 넘게 멈추면 다시 잃는다 (본 최대 1.5 s)
  ② 실물은 UR 드라이버·그리퍼 노드·RealSense 의 발행 큐를 여기서 바꿀 수 없다 → 같은 누락이 나오면 stage1 나이 검사가 잡는다
  ③ 녹화 중에 새 노드를 띄우면 (`ros2 topic echo`, rqt 등) 같은 일이 생길 수 있다 → 필요한 노드는 녹화 전에 띄울 것

**실물 PC 확인 (C1·C2, 2026-10-06)** — 노트북 (Galaxy Book6 Pro), RealSense ROS 4.58.4 (LibRealSense 2.58.4, 카메라 펌웨어 5.17.3.10), 브랜치 `isaacsim_v6.1.0` `f3b98f1`.
사용자가 노트북에서 실행하고 출력을 보내 줌. 팔은 고장이라 `fake_joint_states.py` (가짜 `/joint_states` 125 Hz), 그리퍼는 실물
- 카메라: wrist = D435I (843112074130, USB 3.2, usb2 허브 경유), third_view = D456 (252122301126, USB 3.2, usb4). Color RGB8 640x480 30 FPS.
  발행 QoS RELIABLE·TRANSIENT_LOCAL (큐 깊이는 `ros2 topic info` 에 UNKNOWN), `rgb_camera.auto_exposure_priority` False (D435I 에서 확인)
- **"실물 카메라가 20~25 Hz 로 떨어진다" 는 측정 방법 문제였다**: `ros2 topic hz <image_raw>` 약 20 Hz (구독자가 BEST_EFFORT + 파이썬으로 0.9 MB 이미지 처리),
  같은 카메라의 `camera_info` 는 29.96 Hz (이미지와 한 쌍으로 발행), RELIABLE 로 녹화한 bag 은 29.99 Hz. 3-1 의 sim 결과 (best effort 25 Hz → RELIABLE 30 Hz) 와 같은 현상.
  예전 실물 녹화 (기본 QoS) 가 실제로 이미지를 잃었는지는 확인 못 함 (옛 bag 을 보지 않음)
- **카메라 런치를 레포 안으로** (`launch/cameras.launch.py`, `5_cameras.sh`): `config/cameras.yaml` 을 읽어 역할마다 노드 (`namespace cam`, `name <역할>`)
  → `/cam/wrist/color/image_raw`·`/cam/third_view/color/image_raw` (+ `camera_info`) 가 나오는 것 확인. 예전 `realsense_dual_camera` 런치 (모델 이름 토픽) 는 안 씀
- **녹화** (`./6_record_bag.sh realtest`, 실물 모드, 29.54 s, 1.5 GB): 녹화 전 토픽 검사 통과 (발행자·카메라 RELIABLE), 카메라 `image_raw` 886 / 886 (29.99 Hz),
  그리퍼 2 토픽 887 (30.0 Hz), `/joint_states` 3693 (125 Hz), `/gripper/command` 4, `/joint_command` 없음 (텔레옵 장치 없음). `episode.json` `mode: real`, 카메라 역할 → 모델·시리얼
- **변환·검수**: stage1 `--arm-action next_state --base-imu const` → 738 프레임, 고른 메시지 나이 최대 카메라 33.3·33.4 ms, **그리퍼 48.2 ms** (30 Hz 인데 33 ms 를 넘음:
  그리퍼 노드의 발행 간격이 고르지 않음. 한계 66 ms 까지 여유 18 ms), `/joint_states` 8.1 ms → stage2 → `inspect_dataset_v21.py --load` **5/5 PASS**
  (그리퍼 state 0.000~0.646, action {0, 1}, 파지 신호 있음: 물체를 잡아 0.646 = raw 743 에서 멈춤)
- 카메라 stamp 와 그리퍼·관절 stamp 가 같은 시각 기준 (PC 시각) 으로 겹쳐 변환이 됐다. **두 시계 사이의 정확한 차이 (수 ms~수십 ms) 는 재지 않았다** (9단계 "실물 시계 확인")
- 그리퍼 노드가 한 번 시작 7.5 s 뒤 포트 에러로 끝남 (`Present Position 읽기 실패` → `device reports readiness to read but returned no data`), 다시 실행하니 됨. 원인 확인 안 함 (연결 문제로 추정)
- 못 본 것: 실물 팔 (`/joint_states` 진짜 값), 텔레옵 `/joint_command` 경로 (`--arm-action command`), 에피소드 여러 개 병합, 긴 시간 녹화

**실물 stamp 동기화 문제 (D1, 2026-10-06 기록. 카메라 ↔ 그리퍼만 1 차 측정, 팔은 아직)**
- **무엇이 문제인가**: stage1 은 모든 토픽을 `header.stamp` 로 맞춘다 (격자 시각 직전의 최신 메시지). sim 은 모든 stamp 가 같은 물리 스텝의 sim time 이라
  이미지와 관절값이 정확히 같은 순간이다 (3-8: 이미지 지연 0 프레임). **실물은 stamp 를 찍는 주체와 시점이 토픽마다 다르다**:

  | 토픽 | stamp 를 찍는 곳 | 실제 사건과의 차이 (추정, 확인 필요) |
  |------|------------------|--------------------------------------|
  | 카메라 `image_raw` | RealSense 드라이버 (realsense-ros). 카메라 하드웨어 시각을 PC 시각으로 환산하는지 (`global_time_enabled`), 도착 시각을 쓰는지에 따라 다름 | 노출 시점 ~ PC 도착 사이 (USB 전송·처리 지연 수 ms~수십 ms) |
  | `/joint_states` | UR 드라이버가 로봇 컨트롤러에서 값을 받은 PC 시각 | 측정 시점보다 통신 지연만큼 늦음 (수 ms) |
  | `/gripper/joint_states`·`/gripper/target` | 그리퍼 노드가 직렬 통신으로 위치를 읽은 뒤의 PC 시각 | 읽는 데 걸린 시간만큼 늦음. 발행 간격도 고르지 않음 (메시지 나이 최대 48.2 ms) |
  | `/joint_command` | 텔레옵 노드가 보낸 PC 시각 | 로봇이 실제로 받아 움직이기까지의 지연은 stamp 에 없음 |

- **왜 중요한가**: 토픽 사이에 시각 차이 Δ 가 있으면 데이터셋의 한 프레임 안에서 이미지와 state (그리고 action) 가 Δ 만큼 어긋난다.
  어긋남 크기 = 속도 × Δ (팔 30 °/s·Δ 30 ms → 0.9°, 그리퍼가 닫히는 중 (약 30 °/s) 도 같은 크기). 정책은 "이 이미지일 때 이 관절값" 으로 배우므로
  어긋남이 크면 시각 정보와 고유감각이 맞지 않는 데이터가 된다. 추론 때도 같은 지연이 있으면 상쇄되지만, **sim 데이터 (Δ = 0) 와 실물 데이터를 섞으면 차이가 난다** (9단계)
- **지금 아는 것**: 실물 bag 1 개가 변환됐다 = 카메라·그리퍼·(가짜) 관절 stamp 가 같은 PC 시각 기준으로 겹친다 (고른 메시지 나이 카메라 33.4 ms, 그리퍼 48.2 ms).
  이것은 "수십 ms 안" 이라는 것만 말해 준다. **Δ 자체는 모른다**. 가짜 관절값이라 팔 쪽은 아예 볼 수 없었다
- **재는 방법 (sim 의 check_ros2 5 번과 같은 원리)**: 갑자기 시작하는 움직임을 만들고, 관절값이 변하기 시작한 stamp 와 이미지가 변하기 시작한 stamp 를 비교한다
  1. 그리퍼: 카메라에 손가락이 보이게 두고 닫기 명령 → `/gripper/joint_states` 가 변하기 시작한 stamp vs 이미지가 변하기 시작한 stamp (팔 없이 지금 장비로 가능)
  2. 팔 (새 팔이 온 뒤): 작은 계단 명령 → `/joint_states` 변화 시작 vs 이미지 변화 시작
  3. 여러 번 반복해 평균과 흔들림을 본다. 30 Hz 카메라라 한 번의 분해능은 33 ms → 반복이 필요
  - 같이 확인: `ros2 param get /cam/wrist global_time_enabled` (카메라 stamp 가 어떤 시계인지), 카메라 `color/metadata` 토픽의 하드웨어 시각
- **대응 후보 (측정 뒤 결정)**: ① 차이가 작으면 (한 프레임 40 ms 의 일부) 그대로 두고 기록 ② stage1 에 토픽별 시각 보정을 명시 옵션으로 (`meta.json` 에 기록, 조용히 적용하지 않음)
  ③ 카메라 stamp 방식을 드라이버 설정으로 바꿈 ④ sim 쪽에 실물과 같은 지연을 넣어 맞춤 (9단계)
- **1 차 측정 (2026-10-06, 노트북, 카메라 ↔ 그리퍼)** — 도구 `measure_stamp_offset.py` (그리퍼를 조금씩 움직이며 그리퍼 위치 곡선과 이미지 변화 곡선이
  가장 잘 겹치는 시간 이동 τ 를 찾음. 양수 = 이미지 stamp 가 늦다)

  | | 닫는 방향 | 여는 방향 | 평균 |
  |---|---|---|---|
  | sim (정답 0 ms, 방법 검증) | +5.6 ms (±1.7, n 6) | −10.7 ms (±3.3, n 6) | −2.6 ms |
  | **실물 wrist (D435I)** | **−51.1 ms** (±3.5, n 6) | **−0.2 ms** (±4.4, n 6) | −25.6 ms |
  | 실물 third_view (D456) | −87 ms (±63) | +87 ms (±24) | 쓸 수 없음 (손가락이 화면의 0.9 %, 정지 때 흔들림 큼) |

  - **말할 수 있는 것: wrist 카메라 stamp − 그리퍼 stamp 는 −50 ~ 0 ms 사이** (이미지 stamp 가 같거나 이르다). 정확한 값은 못 정했다:
    같은 방향 안에서는 6 번이 ±4 ms 로 일치하는데 닫을 때와 열 때가 51 ms 다르다. 시각 차이라면 방향과 무관해야 하므로 시각이 아닌 것
    (기구가 방향에 따라 다르게 따라옴 등) 이 섞였다. **원인 확인 못 함**. 방법 자체의 치우침도 ±10 ms (sim)
  - **stamp 가 무엇을 뜻하는지** (받은 시각 − stamp 중앙값): `/gripper/joint_states` +0.6 ms = **발행하는 순간**에 찍음. wrist 카메라 +36.9 ms, third_view +18.5 ms =
    stamp 가 도착보다 이르다. `rgb_camera.global_time_enabled` 가 두 카메라 모두 True (카메라 하드웨어 시각을 PC 시각으로 환산) → **카메라 stamp 는 도착 시각이 아니라
    찍은 시각에 가깝다** (간접 근거. 환산의 정확도는 재지 않음)
  - 이 측정의 한계: ① 손가락 사이에 물체가 있어 그리퍼가 740 raw 에서 막힘 (48 번 중 절반은 못 씀, 분석은 280~570 raw 구간만) ② 실물 그리퍼는 천천히 가속·감속해
    250 raw 에 약 1.9 s (sim 은 등속 0.5 s) → 처음 만든 "시작·정지 순간 찾기" 분석은 실물에서 대부분 탈락 (wrist 14 개, 표준편차 70 ms) → 곡선 겹치기 방식으로 바꿈
    ③ 그리퍼 발행 간격이 32 ms / 가끔 48 ms ④ 한 번만 잼
  - **판단: 그리퍼는 여기서 멈춘다** (사용자 결정). 한 프레임 (40 ms) 안팎이고 그리퍼는 닫는 데 2 s 걸리는 느린 장치라 데이터에 주는 영향이 작다.
    **중요한 것은 팔 ↔ 카메라**: 새 팔이 오면 팔로 잰다 (`/joint_states` 125 Hz, 기구 유격이 훨씬 작아 더 정확). 도구는 그리퍼용이라 팔용으로 고쳐야 한다
    (움직이는 것을 `/joint_command` 계단으로, 위치를 `/joint_states` 로)
- 관련: **정리 문서 `docs/timestamp_sync.md`** (sim 과 실물의 stamp, 측정 결과, 모르는 것, 팔이 오면 재는 방법), 9단계 "실물 시계 확인", `docs/data_recording.md` 7 장

> 1~4단계는 텔레오퍼레이션 장치 없이 진행한다. 키보드나 스크립트로 `/joint_command` 를 보낸다.

---

## 5단계: 텔레오퍼레이션

**목표**: 사람이 드론 파지 시연을 연속으로 녹화할 수 있음 (드론은 실물 조종기로, 팔은 PICO 4 Ultra 컨트롤러로)

**결정 (2026-10-08 사용자)**
1. **드론 텔레옵을 먼저 한다 (5-A)**: 실물 Radiolink 조종기 → Raspberry Pi Pico → USB 직렬 → sim 드론. 사람이 실물 조종기로 sim 드론을 조종한다
2. ~~**팔 텔레옵은 GELLO 대신 SpaceMouse (5-B)**: 3Dconnexion SpaceMouse Wireless. GELLO 는 대안으로 내린다 (아래 "대안")~~
   → **(2026-10-11 사용자) 팔 텔레옵은 PICO 4 Ultra (5-B)**: 사용자가 PICO 4 Ultra 를 가져옴. SpaceMouse 는 쓰지 않고 대안으로 내린다 (GELLO 와 함께, 아래 "대안")
- 순서: 5-A → 5-B. 둘은 서로 다른 통로다: 팔 텔레옵은 `/joint_command` + `/gripper/command` 만 (원칙 2 그대로), 드론 조종 입력은 따로

### 5-A. 드론 텔레옵: Radiolink 조종기 → sim (먼저)

구조도 (지금 드론 명령 경로·포트, 조종기를 넣은 구조, 녹화할 때 구조): **`docs/drone_teleop.md`**

**지금 되어 있는 것 (2026-10-08 사용자 확인)**
- 실물 조종기의 입력을 Raspberry Pi Pico 가 받아 USB 직렬 `/dev/ttyACM0` 로 한 줄씩 보낸다 (`sudo cat /dev/ttyACM0` 로 확인)
  ```text
  RC,0,0,1006,1002,995,996,200,200,200,1800
  ```
- 쉼표로 나눈 값 10 개 중 **3~6 번째 (위 예: 1006, 1002, 995, 996) 가 스틱 4 축** (2 축 스틱 2 개). 스틱을 움직이면 이 네 값이 바뀐다

**A-0 결과 (2026-10-08)** — `isaacsim/scripts/rc_input.py`, 기록 `isaacsim/reports/rc_probe_20261008_122839/` (raw.csv 16,154 줄, 113 s)
- 줄 형식: `RC,<값 1>,<값 2>,<값 3> … <값 10>` = 상태 표시 2 개 + 채널 8 개. **142.9 Hz** (간격 중앙값 7.0 ms, 최대 11.8 ms), 형식이 다른 줄 0 개

  | 값 번호 | 뜻 | 범위 (가운데) | 방향 | 쓰임 (왼쪽 스틱 = 스로틀·yaw 배치) |
  |---------|-----|---------------|------|--------------------------------------|
  | 1, 2 | 신호 상태. 정상 0, 0 | 0 / 1 | 조종기를 끄면 값 2 가 먼저 1, 약 0.5 s 뒤 값 1 도 1 | **끊김 판정: 둘 중 하나라도 0 이 아니면** |
  | 3 | 오른쪽 스틱 좌우 | 200 ~ 1800 (1003) | 오른쪽으로 밀면 커짐 | roll |
  | 4 | 오른쪽 스틱 상하 | 204 ~ 1800 (999) | **위로 밀면 작아짐** (다른 축과 반대) | pitch |
  | 5 | 왼쪽 스틱 상하 | 217 ~ 1800 (993) | 위로 밀면 커짐 | 스로틀 |
  | 6 | 왼쪽 스틱 좌우 | 200 ~ 1800 (994) | 오른쪽으로 밀면 커짐 | yaw |
  | 7 ~ 10 | 안 바뀜 (200, 200, 200, 1800) | — | — | 조종기에 스틱 말고 다른 스위치가 없다 (사용자) → **드론 모터 정지는 녹화 도구 `k` 키 그대로** |

- 가운데에 둔 동안 흔들림: 최대 1 → dead zone 은 가운데 값의 차이 (993 ~ 1003) 를 덮는 정도면 된다
- **조종기를 끄면 줄은 계속 온다** (142.9 Hz 그대로). 값이 `1,1,1000,1000,0,1000,1000,1000,1000,1000` 으로 바뀐다 (채널 1000, 값 5 만 0)
  → "줄이 안 온다" 로는 끊김을 알 수 없고 값 1·2 를 봐야 한다. 줄이 안 오는 것은 Pico·USB 문제 (따로 시간 제한으로 잡는다)
- **재는 방법에서 주의**: 이 실행은 안내가 나오자마자 스틱을 움직였다 놓아서 측정 구간 (안내 5 s 뒤 2 s) 에는 스틱이 가운데였다 → 도구가 출력한 요약 표는 틀리고,
  위 표는 raw.csv 에서 값이 크게 벗어난 구간을 시간 순으로 뽑아 안내 순서 (왼쪽 위·아래·왼·오, 오른쪽 위·아래·왼·오) 와 맞춘 것이다
  (구간 8 개가 안내가 나온 시각 1 ~ 2 s 뒤에 차례로 시작해 순서가 맞는다). **축 방향은 A-2 에서 드론을 움직여 보며 다시 확인** → 확인함, 맞음 (아래 "실물 조종기 확인")
- 스틱 → 기능 (roll·pitch·스로틀·yaw) 배치는 우리가 정하는 것이다: 왼쪽 = 스로틀·yaw, 오른쪽 = pitch·roll (조종기를 끄면 값 5 만 0 이 되는 것도 값 5 = 스로틀과 맞는다)
- 직렬 장치 이름: Pico 는 MicroPython 보드로 잡힌다. 꽂는 순서와 무관한 이름 `/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6647c15678f4f38-if00`
  (`/dev/ttyACM0` 은 다른 USB 직렬 장치를 먼저 꽂으면 바뀐다)
- 남은 것: Pico 펌웨어 소스는 레포에 없다 (줄 형식이 바뀌면 `rc_input.parse_line` 이 에러로 잡는다). 권한: 계정이 `dialout` 그룹에 없어 `sudo` 로만 읽힌다
  → **sim 이 직접 읽으려면 그룹 추가가 필요** (`sudo usermod -aG dialout $USER` 뒤 다시 로그인. sim 을 sudo 로 띄우지 않는다)
  → 같은 날 그룹에 넣음 (아래 "실물 조종기 확인")

**A-1 ~ A-3 결과 (2026-10-08)** — 조종기 입력을 sim 의 PX4 드론에 연결. 아래 숫자는 가짜 직렬 장치 (pty) 와 스크립트가 만든 스틱 값으로 잰 것. 실물 조종기로 날려 본 결과는 그 아래 "실물 조종기 확인"
- 만든 것
  | 파일 | 내용 |
  |------|------|
  | `isaacsim/scripts/rc_input.py` | `RcReader` (직렬을 스레드로 읽음), `RcMap` (값 10 개 → 스틱 pitch·roll·throttle·yaw −1 ~ 1, 신호 끊김·범위 밖이면 None + 이유), `RcInput`. A-0 도구도 여기 |
  | `isaacsim/config/rc_input.yaml` | 직렬 장치, 축 배치·방향·범위 (A-0 값), dead zone, 넘길 때 가운데 기준, 조종기를 쓸 때의 PX4 파라미터 |
  | `isaacsim/scripts/drone_cmd.py` `PX4Commander` | `set_sticks` / `handover` (PX4 수동 모드 POSCTL 요청 + `MANUAL_CONTROL`, 위치 목표·Offboard 재요청 멈춤) / `takeback` (그 자리에서 Offboard) |
  | `isaacsim/scripts/drone_rc.py` `DroneRc` | 루프마다: 호버 중 + 신호 있음 + 스틱 가운데 → 넘김. 넘긴 뒤 신호가 끊기면 되돌리고 터미널에 알림 |
  | `isaacsim/scripts/sim_ros2.py` | `--drone-rc on` (기본 off: 지금까지와 똑같이 동작), `--rc-port`. 리셋 응답 `sim.drone.rc_teleop` 에 켰는지 표시 |
  | `isaacsim/scripts/check_rc.py` | 넘기기 시험 (드론만, 스틱은 스크립트가 만듦) → **6/6 PASS** |
- 정한 것: 직렬은 **sim 프로세스가 직접 읽는다** (정할 것 2 번 (나). 조종 입력을 녹화하지 않아 ROS 노드를 따로 둘 이유가 없음).
  Isaac Sim 의 python 에는 pyserial 이 없어 표준 라이브러리 (termios) 로 읽는다
- **`check_rc.py` (PX4 드론만, 120 Hz, 모션캡처)**
  | 시험 | 결과 |
  |------|------|
  | R0 가운데 스틱 값을 보내는 채로 Offboard 이륙 | 됨 (지금 흐름 그대로) |
  | R1 넘기기 | 0.2 ~ 0.3 s 뒤 POSCTL. 그 뒤 10 s 동안 넘긴 위치에서 최대 30 mm (PX4 호버 흔들림 수준), yaw 0.2°, 기울기 0.7° → **튀지 않는다** |
  | R2 스틱 +0.6 을 2 s | pitch: 앞으로 229 mm (최고 0.41 m/s, 기울기 11.7°) / roll: 오른쪽 278 mm (0.45 m/s) / throttle: 위로 362 mm (0.30 m/s) / yaw: 오른쪽 38.8°. 다른 축은 거의 안 움직임. 놓으면 멈춤 (마지막 1 s 이동 4 ~ 15 mm) |
  | R3 되돌리기 | 1.0 s 뒤 Offboard, 그 자리에서 16 mm |
  | R4 (기록) `MANUAL_CONTROL` 송신을 2.5 s 멈춤 | 1.0 s 뒤 PX4 `Failsafe activated` → **AUTO.LAND** (0.5 m/s 로 내려감). 다시 보내고 되돌리면 Offboard 로 돌아옴 |
  | R5 수동 모드에서 kill | disarm 33 ms, 낙하 |
- **`sim_ros2.py --drone-rc on` (헤드리스, 가짜 직렬 장치가 실물과 같은 줄을 143 Hz 로 보냄)** → 10/10
  시작 뒤 자동으로 넘김 → 오른쪽 스틱 위 (값 4 = 520) 2.5 s: +x 로 0.20 m → 왼쪽 스틱 위 (값 5 = 1500): +0.48 m →
  신호 끊김 줄 (`0,1,1000,…`): 0.2 s 안에 Offboard 로 되돌림, 3 s 동안 위치 변화 8 mm → 신호 복귀: 1.2 s 뒤 다시 넘김 →
  스틱을 민 채 `/sim/reset`: 성공, **가운데가 아니라 넘기지 않음** → 가운데에 두자 0.9 s 뒤 넘김 → `/sim/drone_kill`: 모터 정지
- PX4 에서 확인된 것 (돌려 봄)
  - Offboard 로 나는 동안 스틱 값 (가운데) 을 같이 보내도 영향 없다. PX4 가 수동 모드로 바꾸려면 스틱 값이 먼저 와 있어야 한다 → 넘기기 전부터 가운데 값을 보낸다
  - **스틱 값은 한 번 보내기 시작하면 끊으면 안 된다** (1 s 뒤 failsafe 착륙). 그래서 조종기 신호가 끊겨도 `PX4Commander` 는 가운데 값을 계속 보내고,
    드론은 Offboard 로 되돌려 그 자리에 세운다. PX4 의 조종 입력 상실 failsafe 는 sim 쪽이 멈췄을 때만 나온다
  - 넘긴 뒤에는 CLI `goto`·`hold` 가 거부된다 (Offboard 가 아님). `kill`·`status` 는 된다
- 조종기를 쓸 때만 넣는 PX4 파라미터 (`rc_input.yaml` `px4_params`): `MPC_VEL_MANUAL` 0.5 m/s (PX4 기본 10), `MPC_MAN_Y_MAX` 60 °/s (기본 150),
  `MPC_Z_VEL_MAX_UP`·`MPC_Z_VEL_MAX_DN` 0.5 m/s (기본 3 / 1.5), `MAN_ARM_GESTURE` 0. **값은 임시** (실물 조종기로 날려 보고 정한다)
  - **수직 속도 한계는 수동 모드 전용이 아니라 Offboard 이륙에도 걸린다 → `--drone-rc on` 이면 리셋이 약 24 → 29 s**
    → (2026-10-16) 조종기에 넘기기 직전에 넣게 바꿔 리셋 23.9 s (6단계 "리셋 시간 단축" ②)
- 회귀 (`--drone-rc` 없이): `check_px4.py` 5/5, `check_ros2.py` 9/9 (리셋 23.8 ~ 23.9 s 그대로), `make_fake_episodes.py` **3/3** (2 번째 실행. 아래)
  - **`make_fake_episodes.py` 첫 실행은 2/3** (성공 에피소드 3 개 중 1 개에 보호 정지 → 병합에서 빠짐). 조종기 변경과 무관 (이 검사는 기하 제어기 드론, `--drone-rc` 없음):
    그 에피소드의 bag 을 보면 재생 스크립트가 명령을 보내기 시작한 뒤 **1.7 s 동안 팔이 전혀 움직이지 않았고** (명령은 녹화됨 = 녹화기는 받음, sim 은 못 받음),
    그 뒤 받기 시작한 명령이 이미 36° (wrist_1) 앞서 있어 목표가 뛰어 보호 정지 (위치 오차 5° 초과 = 설계대로). 같은 실행의 다른 에피소드는 정상.
    그 에피소드는 재생 노드가 뜬 직후 `/clock` 수신도 0.57·0.35·0.34 s 씩 끊겼다 (4단계 A1 과 같은 종류: 새 노드가 뜰 때 전달이 멈춤)
  - 고침: `replay_commands.py` 가 **sim 노드 (`--sim-node isaac_sim`) 의 명령 구독이 연결된 뒤에 시작** (안 되면 에러). 그 뒤 다시 돌려 3/3.
    **한 번 통과한 것이라 고쳐졌다고 확인된 것은 아니다**: sim 이 못 받은 이유가 "연결 전" 인지 "연결됐는데 전달이 멈춤" 인지 가리지 못했다 (뒤쪽이면 이 고침으로 안 막힌다)
  - 5-B 텔레옵 노드에 주는 뜻: 명령이 한동안 안 가다가 갑자기 가면 목표가 뛴다 → 노드는 녹화 전에 띄워 두고, 움직이기 전 `/joint_states` 의 지금 자세에서 시작
- **남은 것** (실물 조종기로 돌려 보기·축 방향 확인은 아래에서 끝남)
  1. 스틱 끝 속도·수직 속도 한계는 임시값 그대로 (필요해지면 조작감으로 다시), GUI 에서 드론을 보는 시점 (드론 뒤에서 보지 않으면 앞·뒤·좌·우가 화면과 어긋난다)
  2. 드론을 잡은 채 (RTF 0.69) 의 조종 느낌, 조종기로 드론을 팔 아래·테이블에 부딪혔을 때, 조종기를 껐다 켰을 때 (가짜 입력으로만 확인)

**실물 조종기 확인 (2026-10-08, 사용자, GUI `sim_ros2.py --drone-rc on`)**
- 실물 Radiolink 조종기로 sim 의 PX4 드론이 움직인다. **축 방향이 맞다** (A-0 기록에서 정한 `rc_input.yaml` 그대로):
  왼쪽 스틱 위·아래 = 상승·하강, 왼쪽 스틱 좌·우 = 왼쪽·오른쪽 회전 (yaw), 오른쪽 스틱 상·하·좌·우 = 드론 기준 앞·뒤·왼쪽·오른쪽
- 직렬 권한: 계정을 `dialout` 그룹에 넣음 (`sudo usermod -aG dialout taeung`. 다시 로그인하기 전의 터미널은 `newgrp dialout`)
- 실물 장치를 `rc_input.yaml` 의 장치 이름으로 8 s 읽음: 형식이 다른 줄 0 개. 읽기 시작한 직후의 한 줄은 깨질 수 있다 (버리고 넘어감)
- **사용자 관찰: 오른쪽 스틱 (수평 이동) 을 쓰는 동안에는 드론을 제자리에 세워 두기가 조금 어렵다.** 그대로 둔다 (사용자 판단: 우리가 모으려는 것은 날고 있는 드론이라 괜찮다).
  원인은 재지 않았다. 나중에 줄이려면 볼 것: 스틱 끝 속도 `MPC_VEL_MANUAL`, PX4 스틱 dead zone `MPC_HOLD_DZ` 0.1, 수동 모드 가속·제동 한계
- 오른쪽 스틱은 드론이 보는 방향 기준이라 yaw 를 돌리면 "앞" 도 같이 돈다 (실물 조종과 같음)
- **RTF (잡지 않은 상태, 카메라 2 대)**: 조종기 때문에 느려지지 않는다

  | 조건 | RTF (10 s 구간) |
  |------|-----------------|
  | 헤드리스, 옵션 없음 (자동 호버) | 0.957 ~ 0.965 |
  | 헤드리스, `--drone-rc on`, 실물 Pico 를 읽는 중 (조종기 꺼짐 → 자동 호버) | 0.951 ~ 0.966 |
  | 헤드리스, `--drone-rc on`, 수동 모드 (가짜 직렬, 스틱을 흔듦), 프로펠러 회전 끔 | 0.94 ~ 0.99 (평균 0.97) |
  | 같은 조건, 프로펠러 회전 켬 | 0.93 ~ 0.96 (평균 0.94) |
  | **GUI, `--drone-rc on`, 실물 조종기로 수동 비행 (사용자 터미널 출력), 프로펠러 회전 켬 (GUI 기본)** | **0.89 ~ 0.91** |

  - GUI 0.90 = 실제보다 약 10 % 느림. 헤드리스 수동 모드 (프로펠러 회전 켬) 0.94 와의 차이는 GUI 화면 비용 (3-1 에서도 GUI 가 헤드리스보다 0.02 ~ 0.03 낮았다)
  - 3-1 (2026-10-02) 의 GUI + 프로펠러 회전 0.94 보다 낮다: 그 뒤에 팔·그리퍼 브리지·보호 정지가 들어갔다 (조종기 없이 GUI 로 다시 재지는 않음)
  - 올리려면: `--prop-spin off` (약 +0.03), 프로펠러 회전을 tensor view 로 (3-1 에서 미뤄 둔 최적화, 약 1 ms)

**정할 것**
| # | 항목 | 후보 | 메모 |
|---|------|------|------|
| 1 | 스틱 입력을 sim 드론에 넣는 방법 | **(가) PX4 에 조종기 입력으로 넣는다** (MAVLink `RC_CHANNELS_OVERRIDE` 또는 `MANUAL_CONTROL`) → PX4 의 수동 비행 모드 (Position 등) 가 스틱을 해석 / (나) 스틱 → 속도로 바꿔 Offboard 위치 목표를 조금씩 옮긴다 (`PX4Commander` 목표) | (가) 는 실물 조종과 같은 경로라 드론이 실물처럼 반응한다 (PX4 를 쓴 이유와 같음). 지금 sim 은 Offboard 모드로만 날리므로 모드 전환·PX4 파라미터 (조종기 입력 설정, 신호 끊김 failsafe) 를 확인해야 한다. **코드·PX4 소스 확인 (2026-10-08, `docs/drone_teleop.md`): 새 포트 없이 지금 명령 링크 (UDP 14540) 로 `MANUAL_CONTROL` 을 보낸다 (SITL 은 `COM_RC_IN_MODE 1` = 조이스틱 입력만). 넘겨받기는 `PX4Commander` 안에서 (밖에서 모드를 바꾸면 1 s 마다 Offboard 로 되돌린다).** (나) 는 지금 구조에 바로 붙지만 조종 느낌이 실물 PX4 와 다르다. ~~(가) 를 먼저 시도, 안 되면 (나)~~ → **(가) `MANUAL_CONTROL` + Position 모드로 만듦 (2026-10-08, 위 "A-1 ~ A-3 결과")** |
| 2 | 직렬을 읽는 곳 | ~~(가) 별도 ROS 2 노드~~ / **(나) sim 프로세스가 직렬을 직접 읽음 (2026-10-08 결정)** | (가) 면 sim 코드가 장치를 모르고 조종기를 다른 것으로 바꾸기 쉽다. (나) 는 노드가 하나 적다. 조종 입력은 녹화하지 않으므로 (아래 결정) 어느 쪽이든 된다. (나) 로 정해 조종 입력 토픽은 없다 |
| 3 | 기하 제어기 드론 (`--flight geometric`) 도 조종기로 움직이게 할지 | **PX4 만 (2026-10-08)** / ~~둘 다~~ | 1 번을 (가) 로 해서 PX4 전용 (`drone_rc.DroneRc` 는 드론 제어기가 PX4 가 아니면 에러를 낸다) |
| 4 | 드론 모터 정지 | ~~조종기 스위치로 옮길지~~ → **녹화 도구 `k` 키 그대로 (2026-10-08)** | 조종기에 스위치가 없다 (A-0) |
| 5 | 누가 조종하나 | 두 사람 (드론 조종기 + 팔 PICO 컨트롤러) / 한 사람 (드론을 세워 두고 팔만) | 조종기는 두 손을 쓴다 → 한 사람이 팔 텔레옵 장치 (PICO 컨트롤러) 와 동시에 다루기 어렵다. 6단계 수집 방식에 영향 |

**작업 순서** (처음 계획. 실제로 한 것은 위 "A-0 결과"·"A-1 ~ A-3 결과"·"실물 조종기 확인". 달라진 곳: A-1 의 ROS 2 노드는 만들지 않았다 —
sim 프로세스가 직렬을 직접 읽으므로 조종 입력 토픽도 stamp 도 없다. 검사 (머리말 `RC`, 값 10 개, 범위, 무수신) 는 `rc_input.py` 가 한다)
- A-0 입력 확인 (sim 없이, 도구 `isaacsim/scripts/rc_input.py`): 값 10 개의 뜻을 채운다 (→ 위 "A-0 결과" 표). 스틱을 하나씩 끝까지 움직이며 값 기록, 스위치마다 어느 값이 바뀌는지, 줄 주기, 조종기 끔·신호 끊김 때 출력
- A-1 직렬 → ROS 2 노드: 줄을 읽어 검사하고 (머리말 `RC`, 값 10 개, 범위) stamp 를 넣어 발행. **형식이 다른 줄·범위 밖 값·일정 시간 무수신은 에러 또는 "신호 없음" 으로 알린다** (조용한 fallback 금지). stamp = sim time (`use_sim_time`, `/joint_command` 와 같은 규칙)
- A-2 sim 연결: 정할 것 1 번 방식으로 `sim_ros2.py` 에. 설정 (축 배치·방향·범위·dead zone) 은 설정 파일 한 곳
- A-3 리셋·모터 정지: `/sim/reset` 은 지금처럼 자동으로 이륙·호버까지 하고 **그 뒤에 조종기가 넘겨받는다**. 녹화 도구·녹화 토픽은 바꾸지 않는다 (아래 결정)
- A-4 확인: 조종기로 드론을 팔 위 작업 영역 안에서 움직이고 세울 수 있는지, 스틱 → 드론 반응 지연, RTF, 회귀 (`check_ros2.py` 9/9, `check_px4.py` 5/5, `make_fake_episodes.py` 3/3: 조종기 없이도 지금처럼 돌아야 한다)

**안전 장치**
- 넘겨받을 때 스틱이 중앙 근처일 때만 (리셋 직후 드론이 튀지 않게. 팔 텔레옵의 "현재 자세에서 시작" 과 같은 규칙)
- 조종기 신호가 끊기면 sim 이 Offboard 로 되돌려 드론을 그 자리에 세우고 터미널에 알린다 (PX4 failsafe 에 맡기지 않는다). 그런 에피소드는 사람이 `d` 로 버린다

**알아 둘 것**
- **드론 정보는 녹화하지 않는다 (2026-10-08 사용자 결정)**: 녹화하는 것은 지금처럼 로봇 팔·그리퍼·카메라뿐이다. 조종기 입력도, 에피소드 중 드론 위치도 bag 에 넣지 않는다
  → `record_toggle.py`·stage1·병합·검수·데이터셋 스키마 모두 그대로. 드론 조종 입력은 정책의 action 이 아니다 (정책은 팔·그리퍼만 움직인다)
  - 따라서 에피소드 중 드론이 어떻게 움직였는지는 카메라 영상에만 남는다 (bag 으로 드론 움직임을 다시 만들 수 없다)
- 2단계에서 미구현으로 둔 드론 `trajectory` 모드 (이동 드론) 를 사람의 조종이 대신할 수 있다 (6단계 난이도 순서의 "이동 드론")
- PX4 는 Offboard 위치 목표를 다듬지 않는다 (`docs/drone_flight.md` 14-10 C: `goto` 는 계단 입력). (나) 방식이면 목표를 조금씩 옮기는 것은 보내는 쪽 몫
  ((가) 로 정해 해당 없음: 스틱 → 속도 목표는 PX4 의 Position 모드가 만든다)
- hover thrust 추정기가 모션캡처 설정에서 돌지 않아 $T_h$ = 0.5 다 (`docs/drone_flight.md` 14-10 B). 스로틀 스틱이 추력을 직접 정하는 수동 모드를 쓰면 스틱 중앙과 호버 추력이 어긋난다 → 그래서 Position 모드 (`POSCTL`: 스틱 = 속도, 가운데 = 그 자리) 를 골랐다
- 녹화 중에는 새 노드를 띄우지 않는다 (4단계 A1). 직렬은 sim 이 직접 읽어 따로 띄울 노드가 없다. 5-B 텔레옵 노드는 녹화 전에 띄워 둔다

**완료 기준 (5-A)**
- [x] A-0 표가 다 채워짐 (축 배치·방향·범위·주기·신호 끊김 때 출력) — 2026-10-08 (축 방향은 실물 조종기로 날려 확인함)
- [x] 실물 조종기로 sim 의 PX4 드론을 작업 영역 안에서 움직이고 세울 수 있음 — 2026-10-08 사용자 확인 (GUI. 축 방향 맞음. 수평 이동 중에는 제자리에 세우기가 조금 어렵다 → 그대로 둠)
- [x] 조종기 신호가 끊기면 조용히 넘어가지 않고 드러남 (터미널 출력) — 그 자리 호버로 되돌리고 알림 (가짜 입력으로 확인)
- [x] 조종기 없이 도는 기존 검사가 그대로 통과 (`check_ros2.py`, `make_fake_episodes.py`) — `check_ros2.py` 9/9·`check_px4.py` 5/5, `make_fake_episodes.py` 3/3 (첫 실행 2/3 은 재생 스크립트 문제, A-1 ~ A-3 결과의 회귀 항목)

### 5-B. 팔 텔레옵: PICO 4 Ultra (5-A 다음)

**장치를 바꿈 (2026-10-11 사용자)**: SpaceMouse 대신 **PICO 4 Ultra** (VR 헤드셋 + 손 컨트롤러. 사용자가 가져옴). SpaceMouse 는 대안으로 내린다 (아래 "대안").
**설치, B-0 (입력 확인), B-1 (기구학), 에피소드 시작 자세, B-2 (텔레옵 노드) 까지 했다 (2026-10-12, 아래 "설치 결과"·"B-0 결과"·"B-1 결과"·"B-2 결과"·"에피소드 시작 자세"). B-3 (실제 컨트롤러로 PX4 드론 파지) 와 B-4 (컨트롤러 버튼으로 녹화 → 변환 → 검수) 는 2026-10-14 에 했다 (아래 "B-3 결과"·"B-4 결과").**
이름 주의: 여기의 **PICO** = VR 헤드셋 PICO 4 Ultra. 5-A 의 **Pico** = Raspberry Pi Pico (조종기 값을 USB 직렬로 보내는 보드). 서로 다른 장치다.

- 장치: PICO 4 Ultra 의 오른손 컨트롤러 한 개 (자세 6 자유도 + 버튼·트리거)
- **PC 에서 읽는 방법 = XRoboToolkit (2026-10-12 설치·확인)**: 헤드셋 앱 (APK) → Wi-Fi → PC 서비스 → Python 바인딩 `xrobotoolkit_sdk`.
  Ubuntu 24.04 용 `.deb` 가 있어 이 PC 에 그대로 설치됐다 (아래 "설치 결과")
- **입력 → 명령**: SpaceMouse (속도 입력) 와 달리 컨트롤러는 **자세 (위치·방향) 를 준다** → 컨트롤러 자세의 변화량을 TCP 목표 자세의 변화량으로 옮긴다
  (기준 = 시작한 순간의 컨트롤러 자세와 TCP 자세 → 시작할 때 팔이 튀지 않는다)
  → 차분 IK (타깃 프레임 `rh_p12_rn_tcp`) → `/joint_command` (50 Hz 이상, stamp)
  - **시작 방법 (2026-10-12 사용자 결정)**: 조작자가 **매번 같은 처음 자세**를 잡고 컨트롤러 **A 버튼**을 누르면 텔레옵이 시작된다 (→ 2026-10-14 **Y 버튼**으로 옮김, A 는 녹화. 아래 "컨트롤러 버튼").
    멈추는 방법 (A 를 다시 누름 / 누르고 있는 동안만) 과 손 위치를 다시 잡는 기능 (클러치) 을 둘지는 B-2 에서 정한다
  - `/joint_command` 의 stamp 는 보내는 순간의 sim time 이다. **헤드셋 타임스탬프는 ROS stamp 로 쓰지 않는다** (2026-10-12 사용자 결정. 헤드셋 시계와 PC 시계의 차이를 모른다, B-0 결과)
- 그리퍼: 컨트롤러 버튼 또는 트리거 → `/gripper/command` 0 / 1150 (열기 / 닫기). 데이터셋 action 은 지금처럼 `/gripper/target`.
  트리거 (연속값) 를 쓰면 이진화에 히스테리시스가 필요하고 (예: 닫힘 > 0.6 / 열림 < 0.4), 0단계 결정대로 이진화 전 원시값을 stamp 있는 토픽으로 bag 에만 남긴다
  (녹화 토픽이 하나 늘어난다). 버튼 (이진) 을 쓰면 둘 다 필요 없다 → 정할 것
- **IK 를 어디서 계산하나 → URDF 에서 numpy 로 직접 (2026-10-12, 아래 "B-1 결과")**: 2-4 의 `arm_ik.py` 는 PhysX Jacobian 을 써서 sim 프로세스 안에서만 돈다. 텔레옵 노드는 sim 밖의 ROS 2 노드이고 실물에도 그대로 써야 하므로
  **Jacobian 을 빌드한 URDF (`ur5_rh_p12_description/out/`) 에서 계산**한다 (`teleop/ik_model.py`. pinocchio 는 쓰지 않는다).
  `arm_ik.py` 에서 가져온 것: damped least squares, 관절 목표 = 이전 목표 + Δq (측정값 + Δq 가 아님), 와인드업 방지 (목표가 측정값보다 0.05 rad 이상 앞서지 않게),
  한 주기 변화 한계 (`max_dq` 기본 0.05 rad, 기준 0.063 rad 아래)
- 특이점·관절 한계 근처에서의 동작: 멈춤 + 알림 (조용히 다른 자세로 가지 않게). `ik_step` 이 한계에 걸린 관절 이름 (`limited`) 과 특이점 지표 (`manip`) 를 돌려준다. 알리는 것은 B-2 노드가 한다

**설치 결과 (2026-10-12)** — 순서대로 한 것. 모두 레포 밖 (`/opt/apps/roboticsservice`, `~/xrobotoolkit/`) 에 설치했고 `~/isaacsim`·conda·시스템 pip 는 건드리지 않았다
| 항목 | 내용 |
|------|------|
| PC 서비스 | `XRoboToolkit_PC_Service_1.0.0_ubuntu_24.04_amd64.deb` (XR-Robotics/XRoboToolkit-PC-Service v1.0.0 릴리스, sha256 `bce661f0…2a5d19`) → `sudo dpkg -i` → 패키지 `roboticsservice` 1.0.0.0, `/opt/apps/roboticsservice` |
| 서비스 실행 | `env -u LD_LIBRARY_PATH bash /opt/apps/roboticsservice/runService.sh` (ROS 라이브러리 경로 없이. `runService.sh` 에 `#!/bin/bash` 가 없어 bash 를 명시). 백그라운드로 뜨고 한 번에 하나만. 종료 `pkill -f RoboticsServiceProcess`. 로그 `~/.local/share/PICOBusinessSuitData/log/` |
| 포트 | TCP 63901 (헤드셋 → PC), TCP 127.0.0.1:60061 (바인딩 → 서비스, PC 안에서만), UDP 임시 포트 (PC → 헤드셋 찾기 방송, 목적지 29888) |
| 네트워크 | PC = 유선 `192.168.0.5`, 헤드셋 = 같은 공유기 Wi-Fi `192.168.0.10`. 고정 IP 가 아닐 수 있어 쓸 때마다 확인. **방화벽은 껐다** (`sudo ufw disable`, 재부팅해도 꺼진 채) |
| 헤드셋 앱 | `XRoboToolkit-PICO-1.1.1.apk` 를 헤드셋 브라우저로 받아 설치 (adb 불필요). 앱에서 PC IP 입력 → WORKING → 보낼 항목 (컨트롤러) 선택 → Send. PC 에서 `ss -tn \| grep 63901` 이 ESTAB |
| Python 바인딩 | `bash teleop/build_xr_binding.sh` → `~/xrobotoolkit/venv` (시스템 python3 3.12, `--system-site-packages`) 에 `xrobotoolkit_sdk` 1.0.2. 소스 커밋 고정 (Pybind `c64ccf6`, json 헤더는 PC-Service `85bac4d`), deb 의 미리 빌드된 SDK (`libPXREARobotSDK.so`) 에 링크, RUNPATH 로 경로를 넣어 `LD_LIBRARY_PATH` 불필요. 기록 `~/xrobotoolkit/BUILD_INFO.txt` |
| 실행 방법 | `~/xrobotoolkit/venv/bin/python <스크립트>` (activate 불필요). ROS 를 source 한 터미널에서 rclpy 와 같은 프로세스로 import 되는 것 확인 (텔레옵 노드의 조건) |

- 바인딩이 주는 값: `get_right_controller_pose()` = 위치 x, y, z + 자세 qx, qy, qz, qw. `get_right_trigger()`·`get_right_grip()` 0 ~ 1, `get_A_button()`·`get_B_button()`, 조이스틱 축·클릭, `get_time_stamp_ns()` (헤드셋 타임스탬프). 왼손·헤드셋 자세도 같은 형식
- **바인딩은 "최신값" 만 준다** (소스 `bindings/py_bindings.cpp` 확인): 새 데이터가 안 와도 옛 값이 계속 나오고, 한 번도 안 왔으면 전부 0 이다. **서비스가 없어도 `init()` 은 에러 없이 끝난다** → 쓰는 쪽이 타임스탬프 증가와 쿼터니언 크기를 봐야 한다

**B-0 결과 (2026-10-12)** — `teleop/xr_probe.py` 1 회 (오른손 컨트롤러), 기록 `teleop/reports/xr_probe_20261012_144239/` (git 제외)
| 항목 | 결과 |
|------|------|
| 주기 | **90.0 Hz** (헤드셋 타임스탬프 간격 중앙값 11.12 ms, 5 ~ 95 % 10.62 ~ 11.62 ms, 최대 22.30 ms). PC 도착 간격 중앙값 10.65 ms, 최대 29.68 ms |
| PC 시각 − 헤드셋 타임스탬프 | 중앙값 −91.3 ms, 표준편차 1.4 ms. 두 기기의 시계 차이가 섞여 있어 **지연 자체는 모른다** (흔들림만 약 1.4 ms) |
| 쿼터니언 크기 | 1.0000 (15,916 개) |
| 좌표계 | 오른손 좌표계, **+y 가 위** (위로 옮김 → +y, 비중 1.00). 위치 단위는 m 로 가정 (자로 재지 않았다: "약 20 cm" 안내에 손을 마음대로 움직여 0.25·0.34·0.41 m) |
| 수평 방향 | 앞으로 옮김 → Δ (+0.124, +0.054, −0.212), 오른쪽 → Δ (+0.226, +0.036, +0.253). **앞·오른쪽이 x·z 축과 나란하지 않았다** (몸이 좌표계 축에서 약 30 ~ 40° 돌아간 채 잰 것으로 보인다. 좌표계의 수평 방향이 무엇으로 정해지는지는 확인하지 않음) |
| 컨트롤러 자기 좌표계 | 끝 들기 → local +x (0.99), 오른쪽 돌리기 → local −y (0.98), 시계 방향 비틀기 → local −z (0.94) = +x 오른쪽, +y 위, −z 앞 |
| 버튼 | 트리거·그립 0.0 ~ 1.0 연속, A·B·조이스틱 클릭 0 / 1, 조이스틱 축 −1 ~ 1 |
| 숨김 (등 뒤, 8 s) | 값이 **89.9 Hz 로 계속 온다** (100 ms 넘는 간격 0 번, 전부 0 인 샘플 0). 그동안 위치 범위 435·108·350 mm |
| 내려놓음 (15 s) | 89.7 Hz 로 계속 오고, 1,345 개 중 1,158 개는 위치가 직전 값과 완전히 같다 (원인 확인 안 함) |
| 헤드셋 벗음 (15 s) | 89.3 Hz 로 계속 온다 |

- **추적을 잃어도 타임스탬프는 계속 증가한다** → 타임스탬프로 알 수 있는 것은 연결 (Wi-Fi·서비스·앱) 끊김뿐이고, 추적 상실은 알 수 없다. 숨긴 동안의 위치가 실제 추적인지 추정값인지는 확인하지 않았다
- 이 실행의 "가만히 흔들림" 값 (위치 표준편차 139 mm, 자세 12°) 은 측정 구간에 컨트롤러가 움직여 **쓸 수 없다. 다시 재지 않는다** (2026-10-12 사용자 결정, 아래 "안전 장치")

**B-1 결과 (2026-10-12)** — `teleop/ik_model.py` (numpy 와 표준 라이브러리만), 검사 `isaacsim/scripts/check_ik_model.py` → **2/2 PASS**
- `ArmModel`: 빌드한 URDF 에서 `robot_mount` → `rh_p12_rn_tcp` 체인 (원점 변환 14 개, 가동 관절 = 팔 6 개) 을 읽어 FK 와 기하 Jacobian (6 × 6, base 좌표계, 점 = TCP 원점) 을 계산.
  체인에 모르는 가동 관절이 있거나 이어지지 않으면 에러
- `ik_step`: 차분 IK 한 스텝 (damped least squares, λ 0.02). **오차는 명령 관절각의 FK 기준** (`arm_ik.py` 는 측정 기준): 노드가 sim 밖이라 명령이 팔에 반영되기까지
  약 50 ~ 80 ms 걸리고, 그동안 측정 기준 오차를 계속 더하면 목표를 지나친다. 측정값은 와인드업 방지에만 쓴다. 관절 한계는 한계 − 0.05 rad 안으로 자르고 표시
  (sim 은 한계 밖 `/joint_command` 를 받으면 에러로 끝난다)
- **pinocchio 를 쓰지 않은 이유**: 최신판이 numpy 2 를 요구해 venv 에서 시스템 numpy 1.26 을 가린다 (`ik_model.py` 주석의 2026-10-12 결정). venv 에 pinocchio 는 없다
- 자기 점검 (`python3 teleop/ik_model.py`): Jacobian vs 수치 미분 (무작위 200 자세) 최대 차이 4.98e-07, 홈 근처 무작위 목표 50 개에 `ik_step` 수렴 50 / 50 (200 스텝 안에 0.01 mm·0.006°)
- sim 대조 (`check_ik_model.py`, 리포트 `isaacsim/reports/check_ik_model_20261012_150834.txt`)
  | 검사 | 결과 |
  |------|------|
  | 1 TCP 고정 위치 (`rh_p12_rn_base` → `rh_p12_rn_tcp`): USD vs URDF | 차이 0.000 mm, 0.0000° (z 0.11 m) |
  | 2 그리퍼 base 자세 200 개 (홈 근처 절반, 관절 범위 넓게 절반): sim 링크 vs URDF FK (같은 물리 스텝의 측정 관절각) | 위치 차이 중앙값 0.011 mm, **최대 0.028 mm** (기준 0.5), 자세 차이 최대 0.0029° (기준 0.05) |
  | 3 홈 자세 TCP (로봇 base 기준) | (0.4862, 0.1114, 0.3063) m = world (0.4862, 0.1114, 1.0683), 공구 축 아래 (2-3 의 (0.49, 0.11, 1.07) 과 같음) |
- TCP 는 물리 없는 Xform 이라 sim 에서 자세를 읽을 수 없다 → 강체인 그리퍼 base 로 비교하고, base → TCP 고정 위치는 USD 와 URDF 로 따로 비교했다
- **URDF 를 다시 빌드하거나 로봇 USD 를 재import 하면 `check_ik_model.py` 를 다시 돌린다** (둘이 같은 URDF 에서 나와야 한다)

**정할 것 (PICO 로 바꾸며 생긴 것 포함)**
| # | 항목 | 후보·메모 |
|---|------|-----------|
| 1 | 컨트롤러 좌표계 → 로봇 좌표계 | 위 = +y 는 확인됨 (B-0). B-0 에서 앞·오른쪽이 좌표계 x·z 축과 나란하지 않았다 → **수평 방향 = A (2026-10-14 부터 Y) 를 누른 순간의 컨트롤러 방향을 "앞" 으로 삼는다 (2026-10-12 사용자 결정)**: 그 순간 컨트롤러 앞쪽 (자기 좌표계 −z) 을 수평면에 내린 방향이 "앞", 위는 +y. 서는 위치·방향이 조금 달라져도 맞는다 (~~고정 회전을 설정 파일에 두는 방식~~ 은 쓰지 않음). **그 "앞" = 로봇 base +x (드론 쪽)**: 조작자가 로봇 뒤에 서서 로봇과 같은 방향을 본다. **A 를 누를 때 컨트롤러 끝이 거의 수직이면 "앞" 을 정할 수 없으므로 시작을 거부하고 알린다** (둘 다 2026-10-12 사용자 결정. 수직으로 보는 각도 기준은 B-2 에서) |
| 2 | 손 움직임 → TCP 움직임 배율, TCP 최대 속도 | **1 : 1 (2026-10-12) → 0.9 (2026-10-14, 위치만)**, 손목 회전 따라감, 관절 목표 속도 한계 1.5 rad/s (B-2). 처음 후보: 1 : 1 / 축소 (정밀 파지용). 회전도 따라가게 할지, 위치만 할지. 손을 빨리 움직여도 TCP 가 넘지 않을 속도 한계 (직선·회전) |
| 3 | 시작·그리퍼·드론 모터 정지를 어느 버튼에 | **시작 = A 버튼 (2026-10-12 사용자 결정)**. 쓸 수 있는 것 (B-0): 트리거·그립 (0 ~ 1 연속), B, 조이스틱 클릭·축. 그리퍼는 버튼 (이진) 이냐 트리거 (연속 + 히스테리시스 + 원시값 녹화) 냐. 드론 모터 정지는 지금 녹화 도구 `k` 키 → 컨트롤러 버튼으로 옮기면 팔 조작자가 직접 끌 수 있다. **→ (2026-10-12) 그리퍼 = 트리거 이진화. (2026-10-14 사용자) Y = 텔레옵 시작·멈춤, A = 녹화 시작·종료 (리셋 뒤에만 시작), B = 리셋 (녹화 중이면 실패로 버림), X = 드론 모터 정지** (아래 "컨트롤러 버튼") |
| 4 | 조작자가 무엇을 보나 | **컨트롤러는 조작자가 볼 수 있는 곳에 두고 쓴다 (2026-10-12 사용자 결정)**. 헤드셋을 쓰면 모니터가 안 보인다. 후보: 헤드셋의 바깥 보기 (passthrough) 로 모니터를 봄 / 카메라 영상을 헤드셋 안에 띄움 / 헤드셋을 쓰지 않고 컨트롤러만 듦 (B-0: 헤드셋을 벗어도 값은 계속 오지만 그 값이 맞는 자세인지는 확인하지 않음). **정책이 보는 것은 손목·third view 카메라뿐이므로 조작자도 가능하면 그 영상을 보고 조작한다** |
| 5 | 헤드셋 ↔ PC 연결 | Wi-Fi (헤드셋) ↔ 공유기 ↔ 유선 (PC). B-0: 90 Hz, 가만히 구간 간격 최대 22 ms (PC 도착 최대 30 ms). 실행 전체 178 s 동안 간격 최대 45 ms, 50 ms 넘는 끊김 0 번. 지연 자체는 재지 못함 (시계 차이). 긴 시간 (에피소드 수십 개) 동안의 끊김은 B-3·B-4 에서 본다 |

**작업 순서**
- [x] 설치: PC 서비스, 헤드셋 앱, 연결, Python 바인딩 — 2026-10-12 (위 "설치 결과")
- [x] B-0 입력 확인 (sim 없이): 컨트롤러 자세·버튼 값이 PC 에 오는지, 좌표계, 주기와 간격 흔들림, 숨김·내려놓음·헤드셋 벗음 때 나오는 값 — 2026-10-12 (위 "B-0 결과". 지연과 가만히 흔들림은 재지 못했고 다시 재지 않는다)
- [x] B-1 URDF 에서 FK·Jacobian 계산, sim 과 비교 — 2026-10-12 (위 "B-1 결과", 2/2 PASS)
- [x] 에피소드 시작 자세: 그리퍼가 위를 보고 드론 아래에 있는 자세, 리셋이 그 자세로 되돌림 — 2026-10-12 (아래 "에피소드 시작 자세")
- [x] B-2 텔레옵 노드: 같은 처음 자세에서 A 버튼 (2026-10-14 부터 Y) → 컨트롤러 자세 변화 → TCP 목표 → IK → `/joint_command` + `/gripper/command`. 자기 타이머로 50 Hz 이상 일정하게 (컨트롤러 값은 90 Hz 로 오고, 주기와 무관하게 최신 값을 씀) — 2026-10-12 (아래 "B-2 결과". **숫자는 만든 입력으로 잰 것. 실제 컨트롤러로는 GUI 에서 동작만 확인, 드론 파지는 B-3**)
- [x] B-3 sim 연동 시험 (녹화 없이): **실제 PICO 컨트롤러로** 동작·조작감, 안전 장치 (아래), 보호 정지 오탐 확인 — 2026-10-14 사용자 확인 (아래 "B-3 결과". 숫자로 잰 것은 아님)
- [x] B-4 녹화 → 변환 → 검수 (`/joint_command` 간격 통계), 잡은 채 RTF 0.69 에서의 조작감 — 2026-10-14 (아래 "B-4 결과". 조작감은 사용자 판단으로 문제없음, 숫자로 재지 않음)

**B-2 결과 (2026-10-12)** — 노드 `teleop/xr_teleop_node.py`, 설정 `teleop/xr_teleop.yaml`, 시험 `teleop/check_xr_teleop.py` → **11/11 PASS** (만든 컨트롤러 입력, 기하 제어기 드론, 헤드리스)
> 이 절과 "B-3 결과" 는 그때의 기록이다. 그 뒤 바뀐 것 (2026-10-14): 텔레옵 시작·멈춤 버튼 A → **Y**, A = 녹화, B = 리셋, X = 드론 정지 (아래 "컨트롤러 버튼"),
> 배율 1 → **0.9**, 시험에 버튼 항목을 더해 **12/12** (2026-10-14 필터 항목 7b 를 더해 지금 **13/13**). "녹화의 첫 명령 = A 를 누른 순간" 도 지금은 아니다: 텔레옵 (Y) 을 녹화 (A) 전에 켜므로 녹화 시작부터 명령이 들어간다
- 정한 것 (2026-10-12 사용자): **A 토글** (다시 누르면 멈추고, 또 누르면 그 자리에서 기준을 새로 잡아 시작), **그리퍼 = 트리거를 이진으로** (0.6 넘으면 닫기, 0.4 아래면 열기. 원시값은 녹화하지 않음),
  **배율 1 : 1** (→ 2026-10-14 사용자: **0.9**, 위치만. 회전은 1 : 1 그대로), **손목 회전도 따라감**. 손목·third view 영상은 `rqt_image_view` 로 본다 (따로 뷰어를 만들지 않음)
- 구조: 제어 루프는 자기 스레드에서 wall 60 Hz (`rate_hz`), ROS 구독 (`/joint_states`·`/clock`·`/protective_stop`, 최신 값만) 은 다른 스레드. 발행은 `/joint_command`·`/gripper/command` 뿐
  - 좌표: A 를 누른 순간 컨트롤러 앞쪽 (−z) 을 수평면에 내린 방향 → base +x, 그 왼쪽 → +y, 위 (+y) → +z 인 회전 M 을 잡는다.
    TCP 목표 위치 = 시작 TCP + 배율 × M (손 위치 − 시작 손 위치), 목표 자세 = M (손 자세 변화) Mᵀ × 시작 TCP 자세 → `ik_step` (관절 목표 속도 한계 1.5 rad/s = 한 주기 0.025 rad, `max_lead` 0.08 rad)
  - **명령은 텔레옵 중에만 보낸다** → 녹화의 첫 명령 = A 를 누른 순간 (stage1 이 그 앞을 뺀다). 멈춘 동안에는 보내지 않아 팔이 마지막 목표에 머문다
  - stamp = 가장 최근에 받은 `/clock` (sim time). 실물은 `--stamp wall`. sim 은 루프 (30 Hz) 마다 최신 명령 하나를 쓰므로 한 루프 안에 보낸 두 명령은 stamp 가 같을 수 있다
    (stamp 간격이 0 / 33.3 ms 로 나온다. 검수의 "40 ms 넘은 횟수" 는 그대로 끊김을 뜻한다)
- 시작을 거부 (이유 출력): 컨트롤러 끝이 수평에서 ±60° 넘게 들리거나 숙여짐, `/joint_states`·`/clock` 이 0.5 s 넘게 안 옴, 보호 정지 중
- 스스로 멈춤 (이유 출력): **측정 관절각이 관절 목표와 0.2 rad 넘게 다름 (`/sim/reset` 의 순간이동·팔이 멈춤)**, `/protective_stop` true, `/joint_states` 끊김.
  리셋 뒤에도 계속 보내면 팔이 옛 목표로 뛴다 (3-6 주의) → 리셋하면 노드가 알아서 멈추고, A 를 누르면 새 자세에서 시작한다
- 노드를 띄울 때 헤드셋 데이터가 5 s 안에 안 오면 (값 전부 0 또는 타임스탬프 정체) 에러로 끝난다. 텔레옵 중 타임스탬프가 0.5 s 넘게 멈추면 알림만 한다 (자동 판정 안 함)
- 알림만 하는 것: 관절 한계에 걸림, 특이점 지표 < 0.01, TCP 목표가 팔보다 5 cm 넘게 앞섬
- **테이블 보호 (`min_wrist_height` 0.09 m)**: 손목 관절 (wrist_1·2·3) 원점의 높이 (base 기준 = 테이블 상판 위) 가 9 cm 아래로 더 내려가는 관절 목표는 보내지 않고 알린다 (올라가는 쪽은 막지 않음).
  wrist_2 원점이 5.8 cm 까지 내려오면 테이블에 닿으므로 약 3 cm 전에 멈춘다. 확인 (만든 입력): 아래로 30 cm → 13.4 cm 에서 멈추고 보호 정지 없음, 다시 올리면 따라옴,
  끝을 60° 숙임·앞으로 15 cm + 아래로 20 cm 도 보호 정지 없음. 막는 동안에는 옆으로도 움직이지 않는다 (그 주기의 관절 목표 전체를 보류)
- **보호 정지가 걸리면 sim 터미널에 이유가 바로 나온다** (`protective_stop.py`: `[protective_stop] 보호 정지 t …: force 200.7 (wrist_2_link) > 150`. 전에는 10 s 마다의 상태 줄에만 나왔다)
- **시험 (`check_xr_teleop.py`)**: 진짜 노드 코드에 만든 컨트롤러 (좌표축에서 35° 돌아가고 끝이 20° 들린 자세) 를 넣고 sim 의 `/joint_states` FK 로 잼
  | # | 시험 | 결과 |
  |---|------|------|
  | 1 | 끝을 80° 든 채 A | 거부, 명령 0 개 |
  | 2 | A → 시작 | 명령이 나가고 TCP 이동 0.35 mm (손을 안 움직임) |
  | 3 | 손을 앞으로 10 cm | TCP (base) [+100.1, −0.0, −0.5] mm. 이동 중 명령 − 측정 최대 1.53° |
  | 4 | 위 8 cm·오른쪽 5 cm | [+100.2, −50.0, +79.5] mm |
  | 5 | 왼쪽으로 20° 돌림 | TCP 회전 [+0.02, +0.04, +20.00]°, 위치 변화 0.00 mm |
  | 6 | 트리거 1.0 → 0.5 → 0.0 | `/gripper/target` 1150 → 1150 → 0 |
  | 7 | 명령 간격 (904 개) | 받은 간격 중앙값 16.67 ms, 95 % 17.18, 최대 17.85. stamp 0 없음, stamp 간격 최대 33.3 ms, 뒤로 간 것 0. 한 주기 관절 변화 최대 0.0086 rad |
  | 8 | A → 멈춤, 손을 30 cm 옮긴 뒤 A | 멈춘 동안 명령 0 개, 다시 시작할 때 TCP 이동 0.57 mm (뛰지 않음) |
  | 9 | 텔레옵 중 `/sim/reset` | 노드가 스스로 멈춤 (28.5° 차이 감지), 리셋 뒤 팔이 시작 자세에서 0.000°, 보호 정지 없음 |
  | 10 | 보호 정지 | 전체 동안 0 번 |
- 실제 헤드셋 경로: 헤드셋이 연결만 되고 Send 를 누르지 않은 상태에서 노드를 띄우니 5 s 뒤 에러로 끝났다 (값 전부 0). **B-3 시도 (2026-10-12 사용자, 실제 컨트롤러)**: A 로 시작은 됐고 두 번 다 곧 보호 정지. ① 1.4 s 뒤: 시작 자세가 테이블에 너무 가까움 (wrist_2 ↔ 테이블) → 10 cm 올리고 테이블 보호 추가. ② 3.4 s 뒤: 손목 마운트 ↔ 팔뚝 자기 충돌 → 손목을 뒤집은 팔 자세로 (아래 "에피소드 시작 자세"). 둘 다 노드가 아니라 시작 자세의 문제였다. **고친 뒤 실제 컨트롤러로 다시 해 봄 (2026-10-12 사용자, GUI): 잘 동작하는 것으로 보인다** (숫자로 잰 것은 아님. 드론 파지·녹화·PX4 드론과 잡은 채의 조작감은 B-3 나머지·B-4)
- 못 본 것: 실제 손 움직임 (빠른 동작·떨림) 에서의 추종과 보호 정지 오탐, PX4 드론·드론을 잡은 채 (RTF 0.69), 관절 한계·특이점 알림이 실제로 뜨는 경우, 실물 (`--stamp wall`)
  (PX4 드론 파지는 B-3 에서 사용자가 확인)

**B-3 결과 (2026-10-14 사용자, GUI)** — `sim_ros2.py --init-pose under_drone` (PX4 드론) + `xr_teleop_node.py` + `rqt_image_view` (손목 영상), 녹화 없이
- 실제 PICO 컨트롤러로 A → 드론 아래로 이동 → 트리거로 잡기 → `drone_cmd.py kill` (모터 정지) → `/sim/reset` → 다시 A 까지 **잘 동작한다** (사용자 판단)
- 숫자로 잰 것은 없다: 잡은 채 (RTF 0.69) 의 조작감, 보호 정지가 걸렸는지, 노드 알림 (관절 한계·특이점·테이블 보호) 이 떴는지는 기록하지 않았다. B-4 녹화 데이터에서 `/joint_command` 간격·보호 정지·파지 신호로 본다

**조작자 안내 (VR 연결·명령·버튼·녹화 순서·주의·데이터 위치): `docs/pico_vr_data_collection.md`**

**B-4 결과 (2026-10-14 사용자, GUI, PX4 드론, `--init-pose under_drone`, 배율 0.9)** — 컨트롤러 버튼만으로 연속 녹화: B → Y → A → 잡기 → X → A 를 2 번 연속 성공,
3 번째는 파지 실패 → 녹화 중 B → `_discarded/` 로 옮기고 리셋 (설계대로). 리셋 4 번 모두 23.8 ~ 23.9 s (시드 0 ~ 3)
| 에피소드 | 녹화 | 프레임 (25 Hz) | 그리퍼 state 최대 (파지 신호) | `/joint_command` | 메시지 나이 최대 | kill 프레임 | 보호 정지 |
|----------|------|----------------|-------------------------------|------------------|------------------|-------------|-----------|
| `20261014_181704_xr_test` | 25.1 s | 472 | 0.260 | 1478 개, 60.0 Hz, 간격 최대 33.3 ms, 40 ms 넘은 것 0 | 카메라·그리퍼 26.7 ms, 팔 6.7 ms | 345 | 없음 |
| `20261014_181812_xr_test` | 28.5 s | 576 | 0.254 | 1681 개, 60.0 Hz, 간격 최대 33.3 ms, 40 ms 넘은 것 0 | 카메라·그리퍼 28.3 ms, 팔 8.3 ms | 505 | 없음 |

- stage1 (`--arm-action command --base-imu const`) → 병합 `lerobot_dataset_v21/local/xr_test_20261014` (2 에피소드, 1048 프레임) → `inspect_dataset_v21.py --load` **5/5 PASS**
- 앞뒤 정지 0.04 ~ 0.12 s / 0.12 ~ 0.16 s: 텔레옵을 녹화 전에 시작해 A 직후부터 명령이 들어가므로 앞쪽 정지가 짧다 → `--trim-idle` 은 지금 필요 없다
- 첫 프레임 action − state 는 0.001 rad 수준 (텔레옵 중 녹화 시작, 팔이 명령을 따라가는 중)
- 남은 것: 에피소드 수가 적다 (학습용 아님, 파이프라인 확인용). 실물 (`--stamp wall`), 조종기로 움직이는 드론 파지는 아직
- 녹화 도구가 시작할 때 `~/realsense_ws/install/setup.bash: No such file or directory` 가 나온다 (`6_record_bag.sh` 가 실물 PC 의 RealSense 작업 공간을 source). 이 PC 에는 없고 녹화에는 영향 없음 (메시지 타입은 ROS 기본)

**명령 ↔ 관절 지연·오차 (2026-10-14)** — `teleop/measure_cmd_tracking.py` (bag 의 `/joint_command`·`/joint_states` 를 header.stamp 로 맞춤, sim·실물 공용),
B-4 bag 2 개 (`20261014_181704`·`181812_xr_test`, 첫 명령 ~ 마지막 명령 42 s, 측정 5048 개 중 움직이는 중 80 %. 명령 관절 속도 최대 109 °/s). 리포트 `teleop/reports/measure_cmd_tracking_20261014_200029.txt`
| 항목 | 움직이는 중 | 멈춘 중 |
|------|-------------|---------|
| 1 오차 = 측정 − 최신 명령 (데이터셋 action 규칙, 지연 포함) | 관절 RMS 0.13 ~ 0.34°, 최대 2.4° (wrist_2) / TCP 위치 RMS 2.7 mm, 최대 6.9 mm, 자세 RMS 0.39° | 관절 RMS 0.04 ~ 0.06° / TCP 0.95 mm, 0.13° |
| 2 지연 (명령을 늦춰 측정과 가장 잘 겹치는 시간) | **47 ms** (관절별 44 ~ 56 ms, 어깨·팔꿈치가 조금 더 늦다) | — |
| 3 지연을 뺀 오차 (따라가는 정확도) | 관절 RMS 0.03 ~ 0.06°, 최대 0.67° / TCP 위치 RMS 0.84 mm, 최대 2.3 mm, 자세 RMS 0.11° | 관절 RMS 0.01 ~ 0.04° / TCP 0.68 mm, 0.08° |

- 지연 47 ms 의 내역 (추정): sim 은 앱 루프 (30 Hz) 마다 최신 명령 하나를 받아 (평균 약 17 ms 늦음) 33 ms 동안 선형 보간하고 (`arm.command_interp_time`), drive 가 따라간다.
  3-8 계단 응답 (움직이기 시작 33 ms, 50 % 50 ~ 58 ms) 과 맞는다. 25 Hz 데이터셋에서 약 1.2 프레임: action 이 state 보다 그만큼 앞선다 (명령 → 움직임 순서라 자연스러움)
- 멈춘 중 TCP 0.7 mm 는 대부분 잡은 드론 (0.8 kg) 무게로 처진 것으로 보인다 (shoulder_lift·elbow 0.03°, 1-5 B 시험의 드론 무게 오차 최대 0.067° 와 같은 크기. 따로 나눠 재지는 않음)
- **판단 (제안, 2026-10-14)**: 허용 범위를 미리 정하지 않았던 항목이다. 지연 < 100 ms (데이터셋 2.5 프레임), 지연을 뺀 TCP 위치 오차 RMS < 2 mm 를 기준으로 보면 sim 은 충족.
  실물 (UR servoJ, `--stamp wall`) 은 새 팔이 오면 같은 도구로 다시 잰다 (`docs/timestamp_sync.md` 의 카메라 ↔ 팔 측정과 함께)

**컨트롤러 필터 (2026-10-14)** — 대량 수집 전 준비 1 번 (순서: 필터 → 손목 카메라 후보 sim 비교 → 실물 CAD → third view 카메라 → 파지점 파란 표시 → 호버 범위·사인파 → 대량 수집.
2026-10-16 바꿈: third view 는 intrinsics 까지 한 뒤 잡는 곳 표시 (빨강) → 사인파·기준 위치 범위 → third view 위치·받침대 CAD → 대량 수집).
사용자: 손 떨림은 크지 않으니 One Euro 필터 (Casiez et al. CHI 2012) 를 가볍게 걸어 보고 판단. `xr_teleop_node.py` `OneEuroPose`, 설정 `xr_teleop.yaml` `filter`
- 차단 주파수 = min_cutoff + beta × (1 Hz 로 거른) 속도. 손이 멈춰 있으면 강하게 (떨림 제거), 빠를수록 약하게 (지연 줄임). 위치·자세 따로 (자세는 지난 출력에서 새 자세 쪽으로 회전 벡터로)
- 텔레옵 시작 (Y) 때 그 자리 값으로 초기화 → 지난 값에 끌려 팔이 튀지 않는다. 트리거·버튼은 거르지 않음
- 값 고르기 (만든 손 움직임, 60 Hz): 위치 떨림 표준편차 1 mm 를 얼마로 줄이나 / 일정 속도로 움직일 때 늘어나는 지연
  | 후보 (min_cutoff Hz, beta Hz/(m/s)) | 멈춘 손 떨림 1 mm → | 3 cm/s | 15 cm/s | 50 cm/s |
  |---|---|---|---|---|
  | 1.0, 10 | 0.33 mm | 65 ms (1.9 mm) | 30 ms | 15 ms |
  | 2.0, 20 | 0.41 mm | 40 ms (1.2 mm) | 19 ms | 9 ms |
  | 3.0, 30 | 0.48 mm | 29 ms (0.9 mm) | 14 ms | 7 ms |
  | **1.5, 40 (채택)** | **0.43 mm** | **32 ms (1.0 mm)** | **13 ms** | **6 ms** |
  - 자세는 같은 min_cutoff 1.5 Hz, beta 10 Hz/(rad/s). 떨림이 크지 않아 지연이 작은 쪽을 골랐다. 이 지연은 47 ms (sim 명령 → 관절) 위에 더해진다
- 확인 (기하 제어기 드론, 헤드리스): `check_xr_teleop.py` **13/13** (7b 추가: 손을 가만히 두고 위치 1 mm·자세 0.3° 떨림을 넣으면 명령 TCP 떨림 필터 켬 0.40 mm / 끔 0.93 mm = 0.44 배).
  다른 항목 (이동·회전·멈춤·리셋·버튼) 결과는 필터 전과 같다
- **실제 손 (2026-10-15 사용자, GUI, PX4)**: 필터를 켠 쪽이 **더 낫다** (사용자 판단). 이 값 (1.5 Hz, 40) 으로 간다. 조작감 값이라 B-4 데이터 (필터 없음) 와 이후 데이터를 섞지 않는다
  (지연이 느껴지면 beta 를 올리거나 min_cutoff 를 올리고, 떨림이 남으면 min_cutoff 를 내린다)
- 작업명 규칙 (2026-10-15): 텔레옵 설정 (필터·배율·시작 자세·드론 움직임) 은 `episode.json` 에 남지 않으므로, 설정이 같은 데이터끼리 같은 작업명으로 모으고 바뀌면 새 이름 (예: 필터 켬 = `drone_grasp_f1`).
  `xr_test` bag 에는 필터 끔 (10-14) 과 켬 (10-15) 이 섞여 있어 날짜로만 구분된다. 데이터셋 이름은 병합의 `--repo-id` 로 따로 정한다 (bag 은 그대로)
- 필터를 켜고 녹화한 bag `20261015_105123_xr_test` (34.5 s, 성공, kill 1 번) → stage1 694 프레임 → `local/xr_filter_20261015` 검수 **5/5** (정지 앞 0.04 / 뒤 0.08 s, 명령 간격 40 ms 초과 0)
  - `measure_cmd_tracking.py`: 명령 → 관절 지연 **46 ms** (필터 전 47 ms 와 같음. 필터는 손 → 명령 구간에 있어 이 값을 바꾸지 않는다). 지연을 뺀 TCP 위치 오차 RMS 1.49 mm, 오차 (지연 포함) 3.5 mm
  - 필터 전 (B-4) 보다 오차가 큰 것은 이번에 손을 훨씬 빠르게 움직였기 때문으로 보인다 (명령 관절 속도 최대 158 °/s vs 109, 25 Hz state 관절 속도 중앙 10 vs 5 ~ 7 °/s, 손목 비틀기가 많음)
  - 25 Hz action 의 고주파 성분 (2 차 차분 RMS ÷ 1 차 차분 RMS): 필터 전 0.78·0.63 → 켬 0.47 (state 는 0.56·0.42 → 0.37), action 과 state 의 차이가 1.4 ~ 1.5 배 → 1.3 배로 줄었다.
    **에피소드 1 개이고 움직임 속도가 달라 필터 효과라고 단정할 수는 없다** (빠른 움직임은 이 비를 낮춘다). 판단은 조작감 (사용자) 으로 했다

**손목 카메라 마운트 (2026-10-15)** — 대량 수집 전 준비 2 번. 지금 마운트 (내려받은 "Realsense Support UniversalRobots", tilt 0) 의 문제 (1-7):
렌즈가 그리퍼 중심축에서 손목 영상 기준 왼쪽 32.5 mm·위 57.5 mm (시작 자세 base 기준 −y 32·+x 58 mm) → 영상에서 그리퍼 중심이 오른쪽 아래로 치우치고, 잡는 순간 손가락·잡는 곳이 화면 밖
- 32.5 mm = D435i 삼각대 구멍 중심선 → 왼쪽 IR 17.5 mm + RGB 까지 15 mm (RealSense D400 Series Datasheet Mar 2026, Figure 4-7·Table 4-20). sim `realsense2_description` 과 같다.
  뒷면 M3 2 개 간격 45 mm, **최대 삽입 깊이 3 mm**, 토크 0.4 Nm (Figure 10-9)
- **1 단계 sim 영상 비교** (`isaacsim/scripts/compare_wrist_camera.py`: 로봇 USD 그대로, 렌더링 카메라만 후보 위치로. 원래 카메라 몸체·마운트 겉모습은 감춤.
  기하 제어기 드론, 장면 = 시작 자세 / 잡기 10 cm 전 / 잡는 순간 열림·닫힘. 리포트 `isaacsim/reports/compare_wrist_camera_20261015_111146`·`_111300`)
  후보 = mount 좌표계 s 옆, d 공구 축에서 거리, b 뒤로 물리기, t 공구 축 쪽 기울기 (xacro `cam_tilt` 와 같은 정의)
  | 후보 | s / d / b / t | 시작 자세 드론 중심 v | 잡는 곳 v (열림) | TCP v | 닫힌 손가락 | 판정 |
  |---|---|---|---|---|---|---|
  | C0 (지금) | 0 / 57.5 / 0 / 0° | 236 | 501 밖 (가로 u 480) | 598 밖 | 0.7 % | 치우침, 손가락 안 보임 |
  | C1 | −32.5 / 57.5 / 0 / 0° | 236 | 501 밖 | 598 밖 | 0.7 % | 좌우는 해결 |
  | T5 | −32.5 / 57.5 / 0 / 5° | 188 | 432 | 519 밖 | 3.9 % | |
  | T7 | −32.5 / 57.5 / 0 / 7° | 169 | 405 | 489 | 6.0 % | 후보 |
  | **C4** | −32.5 / 57.5 / 0 / **10°** | 148 | 366 | **446** | **7.4 %** | **채택** |
  | T12 / C6 | −32.5 / 57.5 / 0 / 12° / 15° | 140 / 131 | 340 / 302 | 418 / 377 | 6.8 / 6.0 % | 그리퍼 몸통이 화면 아래를 많이 차지 |
  | C2·C3·C5 | 뒤로 20·40 mm | — | — | — | — | 불가: 카메라가 손목 몸체 속 |
  | R1·R2·R3 | d 75, 뒤로 40·70 mm | | 435 ~ 503 | 밖 | 0 % | 불가: 그리퍼 몸통이 화면 아래 절반을 가림 |
  (영상 640 × 480, 가운데 v 239. 옆 이동 s = −32.5 mm 면 잡는 순간 드론 중심 u 321 ~ 326, 가운데 332)
  - 기울이지 않고 손가락을 보려면 렌즈가 TCP 에서 축 방향 약 150 mm 이상이어야 하는데 그 자리는 손목 속이거나 그리퍼 몸통 뒤 → 기울기가 유일한 방법
- **2 단계 부딪힘** (`isaacsim/scripts/wrist_camera_clearance.py`: 카메라 링크 충돌 형상에 후보 변환을 곱함. 시작 자세 + 텔레옵 범위 29 자세 × 그리퍼 열림·닫힘 = 58):
  C0 13.3 mm (손목 끝) / T7 11.6 mm (브래킷) / C4 10.7 mm (그리퍼 몸통), 겹침 모두 0. 가장 가까운 것이 카메라와 같이 움직이는 부품이라 자세와 무관.
  후보 URDF (레포 밖 임시 빌드, `cam_tilt`·`cam_xyz` 인자만) 로 계산한 TCP 영상 좌표가 1 단계 렌더링과 같음 (C4 (332, 446), T7 (332, 489))
- **결정 (사용자)**: 10° 가 보기 좋다. 흔들리는 베이스 (8단계) 를 대비해 **0° / 7° / 10° 세 벌을 CAD 로** (쐐기로 바꿔 끼우는 방식은 안정성 때문에 쓰지 않음, 한 덩어리).
  sim 은 10° 로 반영한다 (아래 순서). 기울기가 다른 데이터는 섞지 않는다 (작업명에 `t10` 등)
- **CAD** (`isaacsim/ur5_rh_p12_description/cad/wrist_mount_cad.py`, CadQuery 2.8 = 레포 밖 venv `~/cad/venv`. 결과 `cad/out/wrist_mount_t0·t7·t10` `.stl`·`.step`·미리보기):
  - `wrist_mount` 좌표계 그대로 (원점 = 플랜지 면 중심, +z 그리퍼, +y 탭) → URDF 에서 회전 없이 씀
  - 원래 마운트에서 잰 맞물림 그대로: 판 5.0 mm (플랜지 ~ 브래킷 두께 그대로, TCP 위치 그대로), 플랜지 쪽 테두리 Ø63.5 × 6.45, Ø6.5 × 4 (피치원 Ø50), 핀 구멍 (탭 반대쪽), 가운데 Ø34
  - 카메라 탭: 옆 −32.5 mm, 공구 축에서 57.5 mm, 기울기 0 / 7 / 10°. 판과 탭을 볼록 껍질로 이어 한 덩어리 (두께 11.45 mm, 부피 67 ~ 68 cm³)
  - M3 구멍: 나사 자리 6.0 mm (원래 마운트와 같게, STL 단면으로 잼) + Ø5.8 카운터보어 + 입구 모따기 1 mm × 45°. 구멍 축은 카메라 면에 수직 → 세 벌 모두 **M3 × 8 이 카메라에 2 mm** (최대 3 mm)
  - 7°·10° 는 탭 바깥 띠가 그리퍼 쪽 면보다 0.7 / 1.0 mm 올라오지만 공구 축에서 57.5 mm 넘게 떨어져 브래킷 (반지름 31.5 mm) 과 닿지 않음
  - 확인 못 한 것: 브래킷 돌출부가 가운데 Ø34 에 걸리는지, 브래킷 볼트 길이 (+5 mm), USB 커넥터 (RGB 반대쪽 끝 = 탭이 12 mm 더 나온 쪽, 커넥터는 몸체 끝면이라 걸리지 않을 것으로 봄)
- **실물 확인 (2026-10-15 사용자, 10° 출력, 그리퍼 닫음, 전원 없이 카메라만 USB)**: 실물 손목 RGB 를 sim 10° 영상과 나란히 봄 (D435i RGB = UVC `/dev/video4`, YUYV 640 × 480. 이 PC 에는 realsense-viewer·realsense2_camera 없음)
  - 닫힌 손가락이 화면 아래 가운데, 그리퍼 끝이 TCP 자리 (332, 446) 근처, 위아래·좌우 방향 sim 과 같다
  - 손가락 가운데 선이 주점보다 약 10 px 왼쪽 (손가락 거리 약 10 cm 에서 약 1.6 mm. 출력·조립 공차와 RGB 렌즈 위치의 공장 캘리브레이션 차이 수준, 그대로 둠)
- **sim 반영 (2026-10-15)** — 이 날부터 sim 의 손목 카메라는 **t10 마운트**. 이전 데이터 (`xr_test`, `xr_filter`) 와 섞지 않는다 (작업명에 `t10`)
  1. `meshes/mount/` 에 `wrist_mount_t0·t7·t10.stl` + 충돌 판 `wrist_mount_collision_plate.stl` (원래 `wrist_mount.stl` 그대로)
  2. xacro `mount_variant` (기본 `t10`, 그 밖 `t7`·`t0`·`original`) 하나로 마운트 메시·충돌·질량·관성·카메라 기울기·옆 위치를 같이 고름.
     따로 바꾸던 `cam_tilt`·`cam_xyz`·`mount_mass` 인자는 없앰 (사용자: 배포에 불필요, 마운트와 카메라가 어긋날 수 있음). 없는 이름이면 xacro 에러
     - 충돌 = 충돌 판 (외곽 그대로, z 0 ~ 5 mm) + 탭 상자 (z −6.45 ~ 5 mm). STL 전체를 쓰면 볼록 껍질이 플랜지 홈·가운데 구멍을 메워 손목과 겹친다 (`original` 도 단순 도형)
     - 질량 32.36 g (10° 출력물 실측, PLA 채움 15 %), 무게중심·관성은 STL 에서 (균일 밀도)
     - `mount_variant:=original` 로 빌드하면 예전 URDF 와 **완전히 같음** (주석 빼고) 확인
  3. URDF 빌드 (`build_urdf.sh`, 실물 보정값) → 예전과 다른 곳은 마운트·카메라뿐, URDF 로 계산한 카메라 위치 = C4 후보 (TCP 영상 (332, 446)), TCP 위치 변화 0
  4. **로봇 USD 재import = 사용자 GUI** (1-3 과 같은 옵션, USD Output 은 레포 밖 `~/usd_import_t10`) → 레포 USD 와 비교:
     텍스트 `.usda` 8 개 중 5 개 같음, 3 개 (`physics.usda` 질량·관성, `instances.usda` 메시 이름·충돌 판, `base.usda` 카메라 마운트 10° 쿼터니언·카메라 x −0.0325·충돌 형상) 는 예상한 차이뿐.
     `geometries.usd` 메시 48 개 중 47 개 같음 (팔·그리퍼·카메라), 마운트 메시만 교체 + 충돌 판 추가 → 교체 (빈 `Textures/` 빼고)
  5. 회귀 검사 (헤드리스): `check_articulation` **8/8** (카메라 기준값을 새 마운트 값 (0, 53.9, 25.4) mm 로 고침, 마운트 0.0324 kg),
     `check_self_collision` 제외 쌍 0 (가까운 쌍은 예전과 같은 손가락끼리), `wrist_camera_clearance` 10.7 mm·겹침 0/58, `check_wrist_camera` intrinsics 최대 0.10 px·TCP (332, 446)·닫힌 손가락 7.4 %,
     `check_ik_model` **2/2** (TCP 최대 0.027 mm), `drone_scene --check` **3/3**, `check_xr_teleop` **13/13** (새 시작 자세, 보호 정지 0), `check_ros2` **9/9** (대기 RTF 0.953, 리셋 23.8 s.
     6 번 잡은 채 RTF 가 처음 0.416 이었는데 검사 도중 같은 GPU 로 다른 시험을 겹쳐 돌린 탓: A 부분만 다시 돌려 **0.646** (예전 0.65 ~ 0.68), 파지 신호 present 297.6 (예전 약 297) → 새 카메라·마운트가 잡힌 드론에 닿지 않음)
  6. 시작 자세 `under_drone` 다시 구함 (아래 "에피소드 시작 자세"): 드론 기준 위치가 손목 영상 주점에
  7. `episode.json` 의 손목 카메라 기록: 전에는 `cam_tilt_deg` 0.0 을 코드에 박아 두었음 → 로봇 USD 에서 읽음 (`sim_ros2.wrist_camera_mount_info`: `cam_tilt_deg` 10.0,
     `optical_in_wrist_mount` (0, 0.0539, 0.02543), `mount_mesh` wrist_mount_t10)

**B-4 데이터셋의 VLA 학습 관점 점검 (2026-10-14)** — `lerobot_dataset_v21/local/xr_test_20261014` (25 Hz, 에피소드 2 개) parquet 와 bag 으로 확인. **결론: 학습을 막는 문제 없음, 정책 25 Hz 로 문제없음**
| 항목 | 결과 | 판단 |
|------|------|------|
| action ↔ state 시간 관계 | `action[t]` 와 `state[t+1]` 의 차이 관절 RMS **0.05°** (`state[t]` 와는 0.18 ~ 0.21°, `state[t+2]` 0.16 ~ 0.18°) | 좋음. 지연 47 ms ≈ 1.2 프레임이라 action = 다음 프레임 자세. 모든 에피소드에서 같은 관계 |
| delta (`action − state`, openpi DeltaActions) | 움직일 때 관절 최댓값 중앙 0.2 ~ 0.3°, 95 % 0.9°, 최대 1.6°. 멈출 때 0.05 ~ 0.07° | 문제없음 (작지만 openpi 가 데이터로 정규화하고 action chunk 로 묶어 예측) |
| 같은 action 이 반복되는 프레임 | 0 / 1046 | 문제없음 (명령 끊김 없음) |
| 앞뒤 정지 (state 2 °/s 기준) | 앞 0.04 ~ 0.12 s, 뒤 0.40 ~ 0.52 s | 문제없음 |
| 그리퍼 | action {0, 1}, 닫힘 프레임 22 ~ 39 %, 잡은 뒤 state 0.25 | 문제없음 |
| action 의 프레임별 들쭉날쭉함 | 프레임당 변화량 편차 / 평균: action 0.58, state 0.33 (가장 많이 움직인 관절) | **작은 문제, sim 만**: 노드가 stamp 에 "가장 최근에 받은 `/clock`" 을 넣는데 sim 이 `/clock` 을 33 ms 마다 4 개씩 묶어 보내므로 명령 stamp 가 33 ms 단위로 뭉치고, 25 Hz 격자에서 고른 명령이 0 ~ 33 ms 씩 다르게 오래된다. 이것까지 포함한 차이가 위 0.05° 라 작다. 실물은 PC 시각 60 Hz 라 없다. 고치려면 노드가 `/clock` 사이를 메워 stamp (미룸, 첫 학습 뒤) |
| 이미지 ↔ state 시각 | stamp: 이미지의 99.6 ~ 99.9 % 가 `/joint_states` stamp 와 정확히 같음 (같은 물리 스텝. 이미지 내용이 그 순간인지는 3-8 의 5 번에서 확인, 이번 bag 으로 다시 재지는 않음). **데이터셋 한 프레임 안에서는 이미지가 state 보다 오래됨: 0 / 8.3 / 16.7 / 25 ms, 중앙 16.7 ms** | 심각하지 않음: stage1 이 격자 시각 직전의 최신 이미지 (30 Hz) 와 최신 관절값 (120 Hz) 을 따로 고르기 때문. 관절로 0.1° 미만 (중앙 속도 5 °/s), 빠를 때 (40 °/s) 최대 1°. 실물도 같은 구조 (+ D1). 없애려면 stage1 이 관절값을 고른 이미지의 stamp 에서 고르는 명시 옵션 (미룸, sim·실물 공용이라 옵션으로만) |

- 처음 이 점검을 정리할 때 "이미지 ↔ state = 같은 물리 스텝" 이라고만 적었다가, 데이터셋 프레임 기준으로는 위처럼 이미지가 최대 25 ms 오래된 것을 bag 으로 재서 고쳤다
- 데이터보다 중요한 것: ① 양 (지금 2 개는 파이프라인 확인용. 첫 학습 루프는 최소 수십 개, 가능하면 50 ~ 100 개 이상) ② 다양성 (시작 자세 하나, 드론 ±5 cm)
  ③ 정책 25 Hz = 추론을 40 ms 마다 한다는 뜻이 아니다: π0.5 한 번 추론이 보통 그보다 길어 action chunk 를 예측하고 몇 스텝씩 25 Hz 로 실행 (7단계 sim 평가에서 정함)
  ④ 드론 모터 정지는 action 에 없다 → 평가 때 언제 끌지 따로 정한다 (수집 때 끄는 시점을 일정하게) ⑤ sim2real: 실물 지연이 47 ms 와 크게 다르면 delta 크기 분포가 달라진다
  (움직일 때 delta ≈ 관절 속도 × 지연: 30 °/s 에서 sim 1.4°, 실물이 100 ms 면 3°) → sim2real 비교의 주 지표는 지연, 지연을 뺀 오차는 보조. 새 팔이 오면 같은 도구로 재서 맞춘다 (`arm.command_interp_time`)

**컨트롤러 버튼으로 녹화·리셋·드론 정지 (2026-10-14 사용자 결정, B-4 준비)** — 오른손은 텔레옵, 왼손이 키보드 (`r`·`k`) 를 누르던 것을 컨트롤러로.
같은 날 두 번 정했다: 처음 (B = 리셋 → 녹화, A = 텔레옵) 을 만든 뒤, 시작을 한 버튼으로 하고 리셋과 녹화를 나누도록 바꿨다 (아래)
| 버튼 | 동작 | 언제 |
|------|------|------|
| Y (왼손) | **텔레옵 시작 / 멈춤** (누른 순간의 오른손 컨트롤러 자세가 기준. 전에 A 가 하던 일) | 언제든, 녹화와 무관. 녹화 중 멈췄다 다시 시작하면 손 위치를 다시 잡는 클러치가 된다 |
| A (오른손) | **녹화 시작 / 종료** (리셋하지 않음) | 시작은 리셋 (B 또는 `r`) 뒤 아직 녹화하지 않았을 때만. 아니면 거부하고 알림 |
| B (오른손) | **에피소드 리셋** (`/sim/reset`, 약 24 s, 녹화하지 않음). **녹화 중이면 실패로 보고** 녹화를 끝내 `bags/_discarded/` 로 옮긴 뒤 리셋 (A 보다 우선) | 언제든 |
| X (왼손) | **드론 모터 정지** (= 키 `k`. 녹화 중이면 시각이 `episode.json` 에 남는다) | 언제든 |

- **녹화 순서**: B 리셋 → Y 텔레옵 시작 → 자세 유지 → A 녹화 시작 → 잡기 → X 모터 정지 → A 녹화 끝 → B 다음 에피소드 (잡기에 실패하면 녹화 중 바로 B)
  - 텔레옵을 녹화 전에 시작한다: 녹화 전 명령은 bag 에 없고, 녹화 시작부터 명령이 들어간다. A 를 누르고 자세를 유지한 시간만큼 앞쪽에 팔이 멈춘 프레임이 생긴다 (길면 병합 `--trim-idle start`)
  - A 로 끝낸 뒤 B 를 누르면 리셋 순간이동으로 노드가 스스로 텔레옵을 멈춘다 (B-2). 다음 에피소드는 다시 Y 부터
- **A 가 리셋 뒤에만 녹화를 시작하는 이유**: `episode.json` 에 리셋 응답 (시드·드론 위치) 을 넣고 stage1 이 쓴다. 리셋 없이 녹화하면 지난 에피소드의 끝 상태에서 시작하는 데이터가 된다 (한 리셋 = 한 에피소드)
- 텔레옵이 스스로 멈춰도 (보호 정지·`/joint_states` 끊김) 녹화는 계속된다. 알리기만 하고, 조작자가 A 로 끝내고 버린다 (또는 B)
- 경로: 텔레옵 노드가 누르는 순간 한 번 `/teleop/request` (std_msgs/String `record`·`reset`·`drone_kill`) 를 보내고 녹화 도구 (`record_toggle.py`) 가 처리한다.
  노드가 `/sim/reset`·`/sim/drone_kill` 을 직접 부르지 않는 이유: 녹화 상태·kill 시각·리셋 응답은 녹화 도구가 `episode.json` 에 적는다. 버튼은 `xr_teleop.yaml` `teleop_button`·`buttons`
- `/teleop/request` 는 녹화하지 않는다 → 데이터셋·stage1·병합은 그대로. 원칙 2 (팔·그리퍼 명령만 발행) 의 예외로 적어 둠
- 녹화 도구는 리셋·녹화 종료·모터 정지를 처리하는 동안 온 요청을 버리고 알린다. 키보드 `r` (리셋 → 녹화)·`k`·`d`·`q` 는 그대로
- X·Y 는 왼손 컨트롤러라 헤드셋 앱에서 왼손도 골라 Send 해야 한다 (Y = 텔레옵이므로 필수). 노드가 시작할 때 왼손 자세가 안 오면 경고한다. 받는 녹화 도구가 없을 때 A·B·X 를 누르면 노드가 알린다
- 확인 (2026-10-14, 기하 제어기 드론, 헤드리스): `check_xr_teleop.py` **12/12** (텔레옵 버튼 = Y 로, 11 번: A·B·X 를 0.5 s 누름 → `record`·`reset`·`drone_kill` 한 번씩, 텔레옵 그대로).
  녹화 도구를 가상 터미널로 띄워 요청을 보냄: 리셋 전 `record` → 거부, `reset` → 리셋 (리셋 중 보낸 `record`·`drone_kill` 2 개 버림), `record` → 녹화 시작,
  녹화 중 `reset` → 저장 → `_discarded/` → 리셋, `record` → 녹화, `drone_kill` → `episode.json` 에 시각, `record` → 저장, 리셋 없이 `record` → 거부. 두 bag 의 리셋 시드가 서로 다름 (2, 3).
  시험 bag (`20261014_17430*`, `20261014_1812*_btn_test`) 은 `bags/_discarded/` 에 있다. **실제 헤드셋 버튼으로는 아직** (B-4 에서)
- 배율 `scale` 1.0 → 0.9 (2026-10-14 사용자, 위치만. 회전은 1 : 1)

**에피소드 시작 자세 (2026-10-12 사용자 결정)** — 홈 자세는 그리퍼가 아래를 향해 손목 카메라에 테이블만 보이고 드론 (위쪽) 이 안 보인다 (2-3).
홈에서 시작하면 매 에피소드가 "드론이 안 보이는 화면에서 그리퍼를 뒤집어 올리는 동작" 으로 시작한다 → **그리퍼가 수직 위 (+z) 를 보고 드론 아래에 있는 자세에서 시작한다**
- **시작 자세 `under_drone`**: `--init-pose under_drone` (= `-0.1889 -1.0776 1.7304 -2.2242 1.5700 -2.9566` rad, 2026-10-15 t10 마운트에 맞춰 다시 구함. 값은 `isaacsim/config/scene_drone.yaml` `init_poses.under_drone`)
  - **2026-10-15 다시 구함**: 그리퍼 수직 위·TCP 높이 (잡는 곳보다 0.305 m 아래)·손목을 뒤집은 팔 자세는 그대로, TCP 수평 위치만 옮겨 드론 기준 위치가 손목 영상 주점 (332, 239) 에 오게.
    TCP 는 드론 기준에서 수평 17 mm (광축이 기울어 드론 높이에서 그리퍼 축을 약 2 cm 지나감), 렌즈 → 드론 0.41 m, 손목 관절 원점 22.6 cm (같음), 관절 한계 여유 1.41 rad,
    텔레옵 범위 29 자세 × 그리퍼 2 자기 충돌 0 (새 마운트). 렌더링: 드론 중심 (329, 228), 드론 몸체·팔·모터 모두 보임, 잡는 순간 손가락 7.4 %
    (`isaacsim/reports/compare_wrist_camera_20261015_172322/grid.png`). 아래는 예전 마운트로 고른 내용 (예전 값 `-0.1553 -1.2195 1.9541 -2.3059 1.5703 -2.9904`, 새 카메라에서는 드론이 (281, 125))
  - 손목 카메라 **렌즈가 드론 기본 위치 (0.60, 0, 1.50) 바로 아래**, TCP 는 잡는 곳보다 30 cm 아래 (world z 1.195), 렌즈 → 드론 0.42 m. 손가락 닫는 방향 = 드론 폭 (y) 방향
  - 손목 영상에 **드론 몸체·팔 4 개·모터가 화면 가운데에** 보인다 (프로펠러 끝은 화면 밖, `isaacsim/reports/drone_scene_20261012_173344/wrist.png`). `drone_scene.py --headless --check` 3/3 (자세 유지 편차 0.027°)
  - 잡으려면 수직 30 cm + 옆으로 약 6.6 cm (렌즈가 공구 축에서 벗어나 있는 만큼. 리셋마다 드론이 ±5 cm 바뀌므로 옆 이동은 어차피 필요)
  - **손목을 뒤집은 팔 자세를 쓴다** (wrist_2 = +π/2. 같은 TCP 자세에 팔 자세가 여러 개 있다): 처음 쓴 팔 자세 (`-0.1523 -0.8510 1.3376 1.0843 -1.5697 0.1502`, wrist_2 = −π/2) 는
    손목 마운트·그리퍼 브래킷이 팔뚝 바로 옆에 있어 **그리퍼를 한쪽으로 10° 만 기울여도 팔뚝에 닿는다** (원래 메시 기준으로도 겹침 = 실제 충돌).
    사용자가 실제 컨트롤러로 텔레옵하다 보호 정지 (`force 221.0 (forearm_link) > 150`, 명령 200 개 뒤. 그때 TCP 는 base x 축으로 약 20° 기울어 있었다)
    - 같은 TCP 자세의 팔 자세 4 개 중 팔꿈치가 테이블 위인 2 개를 비교 (`collision_geom` 으로 자기 충돌 판정. 팔뚝은 원래 메시, 나머지는 convex hull):
      회전 x·y ±10·20·30°, z ±30·60°, 이동 (위 15·28 cm, 아래·앞·뒤·좌·우 10 cm), 잡는 높이에서 기울이기 = 28 가지
      | 팔 자세 | 자기 충돌 | 못 가는 곳 |
      |---------|-----------|------------|
      | 처음 (wrist_2 −π/2) | 4 가지 (x +10° 부터 팔뚝 ↔ 마운트·브래킷, z −60° 에서 팔뚝 ↔ 카메라) | 1 가지 (잡는 높이 + y −20°: 특이점) |
      | **뒤집음 (wrist_2 +π/2)** | **0 가지** | 0 가지 |
    - TCP 자세가 같으므로 손목 영상·테이블까지의 여유는 그대로다. 관절 한계 여유 최소 1.19 rad (elbow)
    - 만든 입력으로 확인: 비틀기 ±30°·끝 들기·숙이기 30°·좌우 45° (각 0.5 s), 위 28 cm + 비틀기 20°, 앞·옆·위 + 비틀기 + 들기 동시 → 보호 정지 없음. `check_xr_teleop.py` 11/11 (비틀기 ±25° 항목 추가)
  - **처음 고른 값은 10 cm 더 아래였다** (`-0.1524 -0.7192 1.3904 0.8997 -1.5677 0.1496`, TCP 40 cm 아래, 드론 전체가 보임): 손목 관절 원점이 테이블 위 12.6 cm 라
    **TCP 를 6.8 cm 만 내려도 wrist_2 링크가 테이블에 닿아 보호 정지** (접촉 166 ~ 201 N > 150). 사용자가 실제 컨트롤러로 처음 텔레옵했을 때 명령 85 개 (1.4 s) 만에 걸렸고,
    만든 입력으로 재현했다 (빠른 이동·회전 9 가지 중 "아래로 10 cm" 만 걸림. 관절 목표 − 측정은 최대 4.6° 로 위치 오차 기준 5° 안) → **10 cm 올림** (닿기까지 16.8 cm)
  - 텔레옵 노드에도 테이블 보호를 넣었다 (B-2 결과 "테이블 보호"): 올린 자세에서 아래로 13.4 cm 까지 가고 멈춘다
  - 비교한 자세 A (TCP 가 잡는 곳 바로 아래 35 cm, `-0.1919 -0.6654 1.1314 1.1048 -1.5687 0.1903`): 수직으로만 올라가면 되지만 드론이 화면 오른쪽 아래로 치우쳐 프로펠러가 잘린다.
    2-3 의 21 cm 아래 자세는 드론 아랫면만 화면에 가득 찬다. 바로 아래에서는 드론을 더 작게 볼 수 없다 (렌즈를 더 내리면 손목이 테이블에 닿음)
- **`/sim/reset` 이 `--init-pose` 자세로 되돌린다** (`sim_reset.py`. 전에는 항상 홈). `--init-pose` 를 안 주면 지금까지와 같다 (홈). 리셋 응답 `sim.robot.init_pose`·`init_pose_name` 에 기록 → `episode.json`
  - `--init-pose` 는 관절각 6 개 또는 `scene_drone.yaml` `init_poses` 의 이름 하나를 받는다 (`sim_ros2.py`·`drone_scene.py`). 없는 이름·개수가 틀리면 에러
  - 확인 (PX4, 헤드리스): 시작 자세 B 로 띄움 → 리셋 (23.8 s, 팔 편차 0.027°) → 팔을 움직여 보호 정지 → 리셋 (23.9 s, 편차 0.027°, 보호 정지 해제). 회귀 `check_ros2.py` 9/9 (`--init-pose` 없이)
- 텔레옵 노드는 어느 자세에서든 `/joint_states` 의 지금 자세에서 시작하므로 (아래) 시작 자세를 바꿔도 노드는 그대로다
- **시작 자세 다양화는 미룬다 (2026-10-12 사용자 결정)**: 지금은 `under_drone` 하나로 모으고, 첫 학습 루프 (7단계) 를 한 번 돌린 뒤에 넣는다 (6단계 "랜덤화").
  방법 후보: 리셋마다 기준 자세 주변으로 조금씩 흔들기 (관절 ± 몇 도 또는 TCP ± 몇 cm, 시드 기록. 드론 위치 ±5 cm 와 같은 방식) /
  `init_poses` 의 여러 자세 중 하나를 리셋마다 고르기 / 자세를 바꿀 때마다 sim 을 다시 띄우기. 조건: 손목 카메라에 드론이 보이고 손목이 테이블에 닿지 않을 것

**안전 장치**
- 시작할 때와 `/sim/reset` 뒤에는 `/joint_states` 의 지금 자세에서 TCP 목표를 다시 잡는다 (튀는 동작 방지). 텔레옵 버튼 (Y) 을 누르기 전에는 명령을 보내지 않음 = 팔이 멈춰 있음
- **추적 상실·연결 끊김은 자동으로 판정하지 않는다 (2026-10-12 사용자 결정)**: 조작자가 컨트롤러를 볼 수 있는 곳에 두고 쓰고, 신호가 이상하면 GUI 에서 팔이 이상하게 움직이는 것이 보이므로
  그 에피소드를 `d` 로 버린다 (성공/실패를 사람이 정하는 것과 같은 방식). 그래서 가만히 흔들림·끊김은 더 재지 않는다
  - 처음 계획은 "값이 안 오거나 추적을 잃거나 자세가 뛰면 목표를 그 자리에 두고 알림" 이었다. B-0 에서 추적 상실은 타임스탬프로 알 수 없음을 확인했다 (값이 계속 온다)
  - 알아 둘 것: 연결이 끊기면 바인딩은 마지막 값을 계속 준다 → 목표가 멈춰 팔이 그 자리에 선다. 서비스가 없는 채로 노드를 띄우면 값이 전부 0 이다 (쿼터니언 크기 0)
    → 노드는 시작할 때 이것만은 에러로 잡는다 (텔레옵 버튼을 누른 순간 기준 자세가 0 이 되면 안 된다)
- 한 주기 관절 변화 ≤ 0.063 rad (위). 넘으면 잘라서 따라가고, 목표가 계속 앞서 있으면 알린다 (sim 은 위치 오차 5° 에서 보호 정지). 자세가 갑자기 뛰어도 팔은 이 한계 속도로만 따라간다

**대안: SpaceMouse / GELLO**
- SpaceMouse (3Dconnexion SpaceMouse Wireless. 2026-10-08 에 정했다가 2026-10-11 PICO 로 바꿈): 6 축 입력 = TCP 속도 명령 (직선 3 + 회전 3) → TCP 목표 자세에 적분 → 같은 IK 경로.
  버튼이 이진이라 그리퍼 히스테리시스 불필요. Linux 에서 읽는 방법 (spacenavd / hidapi 계열) 은 확인하지 않았다
- GELLO (처음 1순위였으나 2026-10-08 SpaceMouse 로 바꿈): GELLO 레포의 UR5 구성으로 하드웨어 제작, Dynamixel 관절 오프셋·부호 캘리브레이션, 관절 공간 그대로라 IK 불필요.
  트리거 연속값 → 열기/닫기 이진화 (히스테리시스, 예: 닫힘 > 0.6 / 열림 < 0.4) 는 그리퍼 드라이버 쪽에서, 이진화 전 원시값은 stamp 있는 토픽으로 bag 에만.
  시작 시 GELLO 자세와 로봇 자세가 허용 오차 안일 때만 추종

**텔레옵 노드 요구사항 (4단계에서 정해짐)**
- `/joint_command` 의 `header.stamp` 를 넣는다 (sim = sim time (`use_sim_time`), 실물 = PC 시각). stage1 이 이 stamp 로 팔 action 을 맞추고 0 이면 에러
- 50 Hz 이상으로 **일정하게** 보낸다. 데이터셋 (25 Hz) 보다 느리거나 자주 끊기면 같은 action 이 반복되는 계단이 된다
  (검수 `inspect_dataset_v21.py` 의 `/joint_command` 간격 통계로 확인)
- 녹화는 `./6_record_bag.sh <작업명> --sim`: `r` → 리셋 (약 24 s) → 녹화. **첫 명령 이전 구간은 stage1 이 뺀다** → 장치를 잡고 움직이기 시작하면 에피소드가 시작됨.
  녹화 전부터 명령을 계속 보내는 장치라면 녹화 시작 직후 구간이 그대로 들어간다. 4단계 A1 (녹화 시작 직후 메시지 누락) 은 2026-10-06 에 큐를 3 s 분량으로 늘려 고쳤지만 전달이 3 s 넘게 멈추면 다시 생기고,
  명령이 한동안 안 가다가 가면 팔 목표가 뛴다 (4단계 B4)
- 드론 모터 정지는 녹화 도구의 `k` 키 (`/sim/drone_kill`). → (2026-10-14) 컨트롤러 X 버튼으로도 (아래 "컨트롤러 버튼"). 언제 끄는지를 에피소드마다 일관되게

**sim 에서 확인할 것 (3단계에서 넘어옴, `docs/sim_ros2_interface.md`)**
- **드론을 잡은 채 RTF 0.69** (3-8, PX4 모터 끔): 텔레옵으로 잡은 뒤 들고 옮길 때 조작감 확인. 불편하면 원인
  (드론·손가락 충돌 형상 convex 수, solver 반복)을 측정한 뒤 결정 (`docs/sim_performance.md` 6장 끝). PhysX 설정을 바꾸면 파지 회귀 시험 다시
- `/joint_command` 는 **일정한 주기**로 보낼 것 (sim 은 루프 30 Hz 마다 최신 명령을 33 ms 선형 보간, `docs/arm_command_interpolation.md`)
- **리셋(`/sim/reset`) 뒤 텔레옵은 로봇의 지금 자세(홈)에서 시작**: 리셋 전 자세의 명령을 보내면 팔이 33 ms 만에 그 자세로 뜀 (위 안전 장치와 같은 규칙)
- 보호 정지 기준 (접촉력 150 N, 위치 오차 5°, 관절 속도 200 °/s) 이 실제 텔레옵 동작에서 오탐 없는지 (3-7 은 명령 재생으로만 확인)
- 보간 시간 33 ms 를 실물 UR servoJ lookahead (실물 텔레옵 설정을 정할 때) 와 반응 지연이 비슷하도록 다시 맞출지

**완료 기준 (5-B, 5단계 전체)**
- [x] PICO 컨트롤러로 고정 드론 파지 에피소드를 연속으로 녹화 가능 — 2026-10-14 (B-4: 컨트롤러 버튼만으로 2 개 연속 성공 + 실패 1 개 버림, 검수 5/5)
- [x] 명령과 실제 관절 사이 지연·오차가 허용 범위 — 2026-10-14 sim (B-4 bag 2 개): 지연 47 ms, 지연을 뺀 TCP 위치 오차 RMS 0.84 mm (위 "명령 ↔ 관절 지연·오차". 기준은 제안값, 실물은 새 팔이 오면)
- [ ] ~~컨트롤러 연결이 끊기거나 추적을 잃으면 조용히 넘어가지 않고 드러남 (팔이 멈추고 터미널 출력)~~ → (2026-10-12 사용자 결정) 자동 판정 없이 조작자가 GUI 로 보고 그 에피소드를 버린다.
  남기는 것: 서비스·헤드셋 데이터 없이 (값 전부 0) 노드를 시작하면 에러
- [ ] 조종기로 움직이는 드론 (5-A) 을 PICO 컨트롤러로 잡는 에피소드를 녹화 가능 (누가 조종할지는 5-A 정할 것 5 번)

---

## 6단계: 데이터 수집 운영 (고정 베이스)

- 난이도 순서: 고정 드론(위치 랜덤) → 호버링 드론 → 이동 드론 (사람이 실물 조종기로 조종, 5-A)
- 랜덤화: 조명, 드론 초기 위치·자세, 텍스처, 팔 시작 자세 (지금은 `under_drone` 하나. 첫 학습 루프 뒤에, 5-B "에피소드 시작 자세")
- 소량(수십 에피소드)으로 7단계 학습 루프를 먼저 한 번 돌려 문제를 찾은 뒤 대량 수집

**수집 전 확인** (다 모은 뒤 바꾸면 이미지 분포가 달라진다)
- [ ] **sim third view 카메라를 실물 구성에 맞춘다** (`isaacsim/config/third_view_camera.yaml`, 4단계 확정 계획·2-3 에서 넘어옴).
  ~~지금은 임시: 모양 = D435i 메시, intrinsics = D456 가정값 (가로 약 90°, fx = fy = 320), `serial: null`~~ → 아래 "third view 진행 (2026-10-15 ~ 16)"
  - 먼저 실물 third view 카메라 모델 (지금 D456 252122301126, 두 번째 D435i 로 바꿀 수 있음) 과 놓을 위치를 정한다 → **D456 확정**
  - `intrinsics`: 실물 `ros2 topic echo --once /cam/third_view/color/camera_info` 값으로 (손목 카메라 1-7 과 같은 방식), `serial` 도 채운다 → **채움**
  - `position`·`look_at`: 실물 카메라를 놓은 위치·방향 (로봇 base 기준으로 재서) → 후보 Q1·Q2 (아래), 드론 사인파·기준 위치 범위를 정한 뒤 결정 (2026-10-16 순서 변경) → **Q2 (2026-10-22, 아래 "third view 위치 결정")**
  - 모양 (`model`): D435i 면 그대로. D456 이면 메시가 없다 (`realsense2_description` 에 D455 까지만) → 대신할 모양을 정한다.
    third view 영상에는 자기 몸체가 안 나오므로 손목 카메라에 보일 때만 영향 → D455 메시 (외형 124 × 29 × 26 mm 로 D456 과 같음) → **넣음 (2026-10-22)**. 받침대는 CAD 로 (다음)
  - 바꾼 뒤 `drone_scene.py --headless --check` (3/3), `check_ros2.py` (9/9)

**third view 진행 (2026-10-15 ~ 16)**
- **카메라 = D456 (사용자 확정)**. 두 번째 D435i 로 바꾸지 않는다 (`config/cameras.yaml` 주석)
- **intrinsics = 실물 D456 color 640x480 공장 보정값** (pyrealsense2 로 스트리밍 없이 읽음 = realsense-ros camera_info 와 같은 값, fw 5.17.3.10):
  fx 386.413, fy 386.054, cx 325.208, cy 242.755. **가로 79.3°, 세로 63.7°** (640x480 = 1280x800 을 0.6 배로 줄이고 좌우를 자름. 1280x800 은 가로 89.6°).
  임시값 (가로 90°) 보다 좁아 같은 위치에서 물체가 약 1.2 배 크게 보인다. `serial` 252122301126
  - pyrealsense2 는 레포 밖 임시 venv 에 설치해 읽고 지웠다 (이 PC 에는 realsense2_camera 가 없다)
- **렌즈 왜곡: 기록만, sim 에는 넣지 않음 (사용자 결정)**: inverse_brown_conrady k1 −0.0564, k2 0.0637, k3 −0.0203 → 화면 가장자리에서 약 4 ~ 5 px
  (224x224 로 줄이면 약 2 px, 가운데는 거의 0). realsense-ros `image_raw` 는 펴지 않은 원본이라 sim (핀홀) 과 가장자리에서 그만큼 다르다.
  설정 `real_distortion` (없으면 씬이 에러), `episode.json` `cameras.third_view.real_distortion` 에 기록. 실물 ↔ sim 영상 비교에서 문제가 되면 sim 에 왜곡을 넣는 것을 다시 본다
- RGB 렌즈 위치 (도면 + `realsense2_description` D455): 1/4-20 중심선에서 오른쪽 IR 쪽으로 약 11 mm (왼쪽 IR 47.9 mm, RGB 는 왼쪽 IR 에서 59 mm), 몸체 높이 가운데, 앞 유리가 1/4-20 중심보다 15.8 mm 앞.
  사용자가 내려받은 D456 STEP (SLDPRT 를 웹에서 변환) 은 전체 크기가 1 m 로 나오고 부피 계산이 실패해 그대로는 못 쓴다. 1/4-20 (아랫면 가운데, 앞에서 15.8 mm)·뒷면 M4 2 개 (간격 95 mm) 위치는 정상
- **위치 후보 비교** (`isaacsim/scripts/compare_third_view.py`: 카메라만 옮겨 렌더, 기하 제어기 드론, 장면 = 시작 자세 / 잡기 10 cm 전 / 잡는 순간 열림·닫힘.
  잰 것 = 드론·그리퍼·손가락 화면 비율, 테두리에 걸림, 로봇에 가려진 드론 비율 (로봇을 감추고 한 번 더 렌더해 비교). 리포트 `isaacsim/reports/compare_third_view_20261015_211002`·`_211201`)

  | 후보 (base 기준 m) | 거리 | 드론 | 잡을 때 가려짐 | 손가락 두 개가 따로 보이나 | 메모 |
  |---|---|---|---|---|---|
  | P0 지금 (0.05, −0.45, 0.14) | 0.81 | 1.3 ~ 1.7 % | 13 ~ 22 % | 아니오 | 팔뚝이 화면 왼쪽을 크게 가림 |
  | P1 옆 (0.6, −0.48, 0.15) | 0.64 | 2.6 ~ 3.0 % | 10 ~ 15 % | 아니오 | 손가락이 닫히는 방향 (y) 을 따라 봐서 한 손가락이 다른 손가락을 가림 |
  | P2 옆 높게 (0.6, −0.48, 0.45) | 0.49 | 4.7 ~ 5.8 % | 1 ~ 4 % | 아니오 | 드론이 위 테두리에 걸림, 기둥 45 cm |
  | P3 앞 대각선 (1.3, −0.4, 0.15) | 0.90 | 1.3 ~ 1.4 % | 6 % | 예 | |
  | P4 정면 (1.4, 0, 0.15) | 0.89 | 1.3 ~ 1.4 % | 2 ~ 5 % | 예 | 작음 |
  | P5 반대 옆 (0.3, 0.45, 0.15) | 0.69 | 2.2 ~ 2.6 % | 10 ~ 17 % | 아니오 | 팔뚝이 화면 오른쪽을 가림 |
  | **Q1 정면 (1.1, 0, 0.20)** | 0.61 | 2.7 ~ 3.0 % | 6 ~ 7 % | 예 | 시작 ~ 잡기 전부 화면 안 |
  | **Q2 대각선 (1.1, −0.3, 0.20)** | 0.68 | 2.3 ~ 2.6 % | 7 ~ 8 % | 예 | 팔 자세가 입체적으로 보임 (사용자) |
  | Q3 (1.0, −0.3, 0.35) | 0.54 | 3.7 ~ 4.1 % | 7 % | 예 | 프로펠러가 위 테두리에 걸림 |
  | Q4 정면 가까이 (0.95, 0, 0.30) | 0.43 | 4.9 ~ 5.7 % | 8 % | 예 | 위 테두리·시작 때 그리퍼가 아래 테두리에 걸림 |

  보는 점 = 시작 자세 TCP 와 잡는 곳의 가운데 (base (0.61, 0, 0.57)). "가려짐" 의 ±0.5 % 는 렌더 잡음
  - **앞쪽이 좋다**: 드론은 손가락이 y 방향으로 닫히며 잡으므로 옆 (y 축을 따라 봄) 에서는 손가락 두 개가 겹쳐 잡았는지가 잘 안 보인다. 앞에서는 손가락 사이에 드론이 들어가는 모습과 팔 전체가 보이고 가림도 가장 적다
  - Q3·Q4 는 지금도 테두리에 걸려 호버 범위를 넓히면 잘린다. Q1·Q2 는 0.6 m 거리에서 화면 반폭 약 0.5 m 로 여유가 있다
  - **사용자: Q1·Q2 둘 다 좋고 Q2 가 더 입체적.** 결정은 드론 사인파·기준 위치 범위 뒤에 (이륙 지점이 base (1.2, 0) 이라 Q1 과 10 cm, Q2 와 약 32 cm. 이륙 때 앞으로 최대 20 cm 밀림, 2단계 2번)
  - GUI 로 보기: 레포 밖 임시 설정 (third view 위치만 바꾼 씬 설정) 으로 `grasp_demo.py --case success --flight geometric --hold --scene-config <임시>` (실제 시간, 손목·third view 창).
    Stage 창에서 `/World/ThirdView/camera` (2026-10-22 전 이름 `d435i`) 를 옮기면 third view 창이 바로 바뀐다 (받침대는 따라오지 않음)

- [x] **`/sim/reset` 시간 단축 (2026-10-16 사용자 제기, 끝)** — 약 24 s 그대로 (조종기를 쓰면 약 29 s → ② 뒤 약 24 s). ③ 공중 이륙대는 효과가 작아 쓰지 않음. 수집할 때 에피소드마다 기다리는 시간이라 비효율적
  - 사용자 생각: 떠 있는 상태로, 목표를 향해 가는 중에 리셋을 끝낼 수 없나. 드론 제어기는 **PX4 그대로** (기하 제어기로 바꾸지 않음). 드론이 여러 시작 지점에서 목표로 가게 하는 것도 좋겠다
  - 코드로 본 지금 순서 (`sim_reset.py`, `drone_scene.px4_takeoff`): kill → 테이블 이륙 지점으로 순간이동 → PX4 재시작·연결 → **바닥에서 EKF 대기 고정 5 s** → arm →
    수직 상승 약 0.7 m → 수평 이동 약 0.6 m (`goto` = 계단 목표, `docs/drone_flight.md` 14-10 C) → **도착 뒤 대기 고정 5 s** (`settle_time`)
  - 검토:
    - 날아오는 중에 에피소드를 시작하면 데이터가 "떠 있는 드론" 에서 "날아와서 멈추는 드론" 으로 바뀐다 (계단 목표라 목표 근처에서 약 23 % 지나쳤다 돌아옴).
      6단계 "이동 드론" 에 해당하므로 작업명을 나눠 따로 모으고, 그때는 목표를 조금씩 옮기는 경로를 만든다
    - 여러 시작 지점도 날아오는 과정을 녹화에 넣을 때만 데이터가 달라진다 (녹화 전에 도착하면 지금 드론 위치 ±5 cm 와 비슷)
    - **공중 이륙대**: 목표 아래에 카메라에 안 보이는 충돌판을 두고 그 위에서 PX4 를 띄워 이륙 → 판을 끈다. PX4 에게는 보통의 바닥 이륙이라 arm·EKF·이륙 판정이 그대로이고,
      상승·수평 이동이 거의 없어진다. 판 위치를 바꾸면 시작 지점도 바꿀 수 있다. 판을 끈 것을 확인해야 한다 (그리퍼가 올라오는 경로)
  - **순서 (2026-10-16)**: ① 리셋 단계별 시간 기록 (동작 그대로) ② 고정 대기 2 개를 상태 기준으로 (EKF·arm 준비, 도착 뒤 속도), 조종기 모드의 수직 속도 한계를 이륙 뒤에만
    ③ 공중 이륙대 ④ third view 위치 결정, 문서. (①·②·③ 끝, 2026-10-16. ③ 은 쓰지 않음) 각 단계 뒤 `check_ros2` 9/9, `grasp_demo --case all` (PX4) 6/6, `check_rc` 6/6 (③)
  - **① 결과 (2026-10-16)**: 리셋 응답 JSON 에 `phases` (단계별 sim·wall 걸린 시간) 와 `wall_s` 를 넣고 sim 터미널에 한 줄 요약 (`sim_reset.py`, `drone_scene.px4_takeoff(phases=)`, 동작은 같음).
    헤드리스 `sim_ros2.py --init-pose under_drone` (PX4, 모션캡처) 에서 리셋 3 번, 조종기 없이 (Pico 가 연결되지 않아 `--drone-rc on` 은 재지 않음)

    | 단계 | sim s (3 번) | 내용 |
    |---|---|---|
    | kill · 순간이동 + PX4 재시작 요청 · 팔 순간이동 | 0.0 · 0.0 · 0.0 | 합쳐 wall 0.15 s |
    | PX4 연결 | 2.1 ~ 2.2 | 부팅 → lockstep + 명령 링크 |
    | **EKF 대기 (고정)** | **5.0** | 바닥에 놓인 채 |
    | arm | 1.9 | arm 요청 → PX4 armed |
    | **수직 상승 약 0.7 m** | **7.2 ~ 7.3** | 추정 위치가 목표 5 cm 안 (평균 약 0.1 m/s) |
    | 수평 이동 약 0.6 m | 2.5 ~ 2.6 | |
    | **도착 뒤 대기 (고정)** | **5.0** | `settle_time` |
    | **합계** | **23.8 ~ 23.9 (wall 24.5 ~ 25.0)** | 드론 오차 5.9 ~ 7.1 mm, 팔 0.025° |

    - 고정 대기 10 s + 상승 7.3 s = 17 s 가 대부분. 상승이 수평 이동보다 거리당 3 배 느리다: 원인 후보 = 이륙 직후 hover thrust 0.5 가정 (실제 0.38) 의 수직 적분이 쌓이는 시간 (`docs/drone_flight.md` 14-10 B), 수직 위치 P 1.0 s⁻¹ 의 끝 꼬리 (재지 않음)
    - 회귀 `check_ros2` **9/9** (리셋 23.8 ~ 23.9 s, RTF 0.995)
    - **조종기 모드 (`--drone-rc on`)**: 실물 Pico 없이 가상 직렬 장치 (pty, 실물과 같은 줄 `RC,0,0,1003,999,993,994,…` 스틱 가운데 143 Hz, 레포 밖 임시 스크립트) 로 같은 조건.
      리셋 동안은 Offboard 자동 이륙이라 스틱 값을 쓰지 않으므로 실물과 같다. 3 번: **28.6 ~ 28.7 s (wall 29.3 ~ 29.4 s)**.
      다른 단계는 같고 **상승만 7.3 → 11.8 ~ 12.0 s** (조종기용 PX4 파라미터 수직 속도 한계 0.5 m/s 가 Offboard 이륙에도 걸림, `docs/drone_teleop.md` 8 장).
      리셋 끝 드론 오차 21.8 ~ 22.4 mm (조종기 없을 때 6 ~ 7 mm. 원인은 보지 않음). 리셋 직후 바로 다음 리셋을 불러 그 사이 조종기 넘김 줄은 나오지 않았다 (시작 때는 넘김 확인)
  - **② 결과 (2026-10-16)**: 조종기 모드만 줄었다 (28.7 → **23.8 ~ 23.9 s**, 드론 오차 22 → 5 ~ 8 mm). 일반 모드는 그대로 (23.9 s)

    | 시도 | 결과 | 남김 |
    |---|---|---|
    | 조종기용 PX4 파라미터 (수평·수직 속도 한계 등 4 개) 를 PX4 시작 때가 아니라 **넘기기 직전에** 실행 중 PARAM_SET, PX4 가 `PARAM_VALUE` 로 같은 값을 돌려준 뒤 넘김 (`rc_input.yaml` `handover_params`, `PX4Commander.set_params`, `drone_rc`) | 상승 12 → 7.2 s. 리셋마다 넘김 정상 (확인 0.02 s). `check_rc` 6/6 (위로 최고 0.30 m/s = 스틱 0.6 × 0.5, 전과 같음) | **예** |
    | 바닥 EKF 대기 5 s → PX4 SYS_STATUS pre-arm 비트 | 부팅 때 모드 기준이라 Offboard 로 arm 되는 상태에서도 30 s 넘게 False ("heading estimate not stable") → 쓸 수 없음 | 아니오 |
    | 바닥 대기 → 위치 추정을 받자마자 arm (PX4 arm 검사에 맡김) | 리셋 15 s 까지 줄지만 **PX4 grasp_demo success 실패** (접근 마지막에 드론이 ±8 mm 흔들려 팔이 시간 초과, 3/6). 추정 위치 안정·yaw 일치까지 기다려도 실패, 고정 3 s 실패, 4 s 한 번 통과 (경계), 5 s 통과 → 바닥에 가만히 있는 시간이 필요 (PX4 가 정지 상태에서 추정하는 것, 확인 안 함) | 아니오 (5 s 를 설정 `ekf_wait` 로) |
    | 도착 뒤 대기 5 s → 실제 속도 기준 | 목표에서 30 ~ 40 mm 떨어져 천천히 다가오는 중에 끝남 | 아니오 |
    | 도착 뒤 대기 → 실제 위치 (15 mm 안) + 속도 기준 | 4.4 s (−0.6 s), 끝 오차 13 mm | 옵션만 (`settle: near`), 기본은 5 s 고정 (사용자 결정) |
    | Offboard·arm 요청을 재요청 주기 (1 s) 를 기다리지 않고 바로 | arm 1.9 → 1.1 s 지만 리셋 끝 드론 오차 6 ~ 7 → 12 mm (주로 높이) | 아니오 |

    - 회귀 (최종 코드): `check_ros2` **9/9** (리셋 23.8 ~ 23.9 s, 드론 오차 6 ~ 7 mm), `check_rc` **6/6**, `grasp_demo --case all` (PX4) **6/6**·`drone_scene --check` 3/3 (이 둘은 settle `near`·바로 요청 상태에서 돌림. 그 뒤 둘 다 예전 동작으로 되돌림)
    - 남은 큰 부분은 상승 7.2 s + 수평 이동 2.5 s → ③ 공중 이륙대. 바닥 대기 5 s 가 필요하다는 것도 이륙대 (판 위에 가만히 놓인 채 시작) 와 맞는다
  - **③ 결과 (2026-10-16): 공중 이륙대는 쓰지 않는다 (사용자 결정, 테이블 이륙 유지)**. 목표 아래 보이지 않는 kinematic 판 (0.4 × 0.4 × 0.01 m) 위에서 PX4 를 띄우고,
    실제로 떠오른 뒤 판을 바닥 아래로 치우는 것까지 만들어 재 보고 되돌렸다. 단계 시간에 "liftoff" (arm 뒤 실제로 1 cm 떠오름) 를 더해 PX4 이륙 절차 시간을 따로 본다 (남김)

    | 이륙 자리 | arm → 떠오름 | 상승 | 수평 이동 | 합계 | 끝 드론 오차 |
    |---|---|---|---|---|---|
    | **테이블 (유지)** | 2.7 s | 4.5 s | 2.5 s | **23.9 s** | 6.5 ~ 10 mm |
    | 판, 목표 15 cm 아래 | **6.1 s** | 2.4 s | 0 | 22.7 s | 13 ~ 14 mm |
    | 판, 목표 22 cm 아래 | 3.6 s | 3.6 s | 0 | 21.2 s | 13 ~ 14 mm |

    - 줄지 않은 이유 (PX4 v1.16 `MulticopterPositionControl.cpp`·`Takeoff.cpp`): 아직 땅이라고 보는 동안은 추력을 0 으로 누르고 적분을 리셋, 허용 상승 속도는 −g / K_p,vel = −2.45 m/s 에서
      `MPC_TKO_RAMP_T` 3 s 에 걸쳐 올라가고, 목표 상승 속도는 남은 높이에 비례 (0.11 m 면 약 0.1 m/s) → 목표가 가까울수록 땅에서 떨어지는 데 오래 걸린다
    - 판을 더 내릴 수 없다: 시작 자세에서 그리퍼가 드론보다 약 30 cm 아래 (22 cm 판과 그리퍼 끝 사이 약 8 cm)
    - 처음 8 cm 로 했을 때: 판 위 body 원점이 이미 도착 판정 (5 cm) 안이라 뜨기 전에 판을 치워 드론이 팔 위로 떨어짐 (보호 정지, 손목 카메라 434 N)
      → 실제로 5 cm 떠오른 뒤에만 치우게 고쳤었다 (되돌린 코드에 있던 것)
    - **결론**: 리셋 24 s 의 대부분은 PX4 가 정한 동작 (재시작 2 s, 바닥 대기 5 s, arm 2 s, 이륙 절차, 도착 뒤 5 s). 이륙 자리로 줄일 수 있는 것은 3 s 미만.
      더 줄이려면 PX4 이륙 파라미터 (`MPC_TKO_RAMP_T`, `MPC_THR_HOVER` 0.38 등) 시험 → 비행 동작이 바뀌어 회귀 검사 필요 (하지 않음)
  - 리셋 시간이 적힌 문서 (바꾸면 같이 고침): CLAUDE.md, `docs/architecture.md`, `docs/sim_ros2_interface.md`, `docs/data_recording.md`, `docs/pico_vr_data_collection.md`, `docs/drone_teleop.md`

- [x] **sim 드론 = Iris 0.75 배 0.8 kg 유지 (2026-10-16 확인)**
  - 원본 Iris (1.0 배, 1.52 kg) 는 그대로 못 쓴다: 허리 폭 약 108 mm > 그리퍼 완전 열림 107 mm (2-1). 1.5 kg 은 지금 마찰·파지력으로 G4 에서 놓침 (1-6)
  - 실물 드론 = **SwellPro Spry+** (사용자). 인터넷에 3D 모델 (STL·STEP·CAD) 없음, 단종. 공식 사양 (SwellPro 스토어): 233.5 × 249 × 90 mm, 축간 대각 270 mm,
    기체 538 g + 배터리 220 g = **약 758 g** (두 값을 더한 것) → sim 0.8 kg 과 약 5 % 차이. 잡는 몸체 옆면 폭은 자료에 없음
  - 바꾸려면 실물을 재서 간이 드론 (`drone_simple.yaml` 방식) 으로 만든다 (사용자가 나중에 정함)
- [x] **잡는 곳 빨간 테이프 (2026-10-16 사용자 결정: 빨강, 손가락이 몸체에 닿는 옆면 + 아랫면 가장자리로 조금 감쌈)** — VLA 가 잡을 곳을 보는 시각 표시.
  **실물 드론에도 같은 색·같은 자리에 붙인다** (실물: 폭 40 mm 빨간 무광 테이프 약 6 cm 를 허리 옆면 위쪽에서 아래로 붙이고 아랫면으로 감아 팽팽하게 붙임, 양쪽)
  - 설정 `drone_iris.yaml` `grasp_marker`: 색 진한 빨강 linear (0.65, 0.02, 0.02), 무광 roughness 0.8, 드론 좌표계 폭 ±20 mm, 옆면 위 끝 z 22 mm,
    아랫면 안쪽 끝 = 닫는 방향 |y| 20 mm (`bottom_inner`), 단면 간격 1 mm, 껍질에서 0.5 mm. 항목이 없으면 에러, 끄려면 `null` (간이 드론·기준선 Iris 는 null).
    `episode.json` 드론 `grasp_marker` 에 기록 (색·크기·쪽별 면적·길이) → 테이프 모양이 다른 데이터를 섞지 않는다
  - 만드는 법 (`drone.add_grasp_marker`, 씬에서만, 원본 USD 그대로): 손가락 폭 띠를 1 mm 간격 단면 (x 마다) 으로 잘라, **단면의 볼록 껍질**을 따라
    옆면 위 끝에서 바깥 모서리를 돌아 아랫면 안쪽 끝까지 한 장으로 잇는다 (실물 테이프를 팽팽하게 감은 것: 구멍·홈 위는 건너고 돌기는 덮음).
    x 방향은 단면마다 따로라 허리의 오목한 곡선은 따라간다. 쪽마다 메시 하나, body 링크 아래 (같이 움직임), 충돌·질량 없음 (충돌 형상 수가 바뀌면 에러)
    - 결과: 쪽마다 24.6 / 25.6 cm², 길이 60 ~ 61 / 63 ~ 64 mm (좌우 차이 = 원본 메시가 좌우 대칭이 아님), z −29 ~ +22 mm.
      껍질이 표면에서 떨어진 곳 (건넌 홈) 최대 2.6 / 2.8 mm = 아랫면 통풍구 (z ≈ −31 mm, 손가락이 닿지 않음). **손가락이 닿는 곳** (|x| ≤ 17, z −2 ~ 21 mm) 은 평균 0.05 mm,
      0.5 mm 넘는 곳 1.2 %, 최대 1.7 mm → 영상·학습 영향 없다고 봄 (물리는 원래 메시 그대로, 실물 테이프도 홈 위는 건넘). 처음 잰 4.4 mm 는 단면 선분 끝점까지 잰 과대값
    - 시도한 것 (모두 2026-10-16): 메시 면을 그대로 복사 → Iris body 메시는 면 방향이 약 절반씩 섞이고 닫혀 있지 않으며 껍질 두께가 있어 바깥에서 쏜 광선이 처음 맞는 면만 썼는데,
      구멍·나사 자리가 그대로 남아 **손목 영상에서 띠 안 검은 점 2 ~ 15 %** (실물 테이프는 평평하게 덮음 → sim 만 다름) →
      높이 지도 + grey closing 으로 메움 → 옆면·아랫면 이음매 줄, 격자 끝 들뜸 (위 끝 7 mm), 돌기 벽에서 끊김 → **단면 볼록 껍질** (검은 점 0 %, 이음매 없음)
  - 높이 근거: **실제 PX4 파지 (success) 에서 손가락 파지면 위치 (드론 좌표계)** x −9.9 ~ +17.3, z −1.1 ~ +20.3 mm (잡은 직후·모터 정지 뒤 같음) → 테이프 안 (위 여유 1.7 mm).
    옆면은 z −23 ~ +25 mm 에서 거의 수직, 아랫면 z ≈ −31 mm
  - 바뀐 과정 (모두 2026-10-16 사용자): ① 파랑, 옆면만 (높이 −8 ~ +22 mm, 쪽마다 40 × 30 mm) → ② **빨강** (sim 앞 프로펠러 2 개가 파랑 = Iris 원본 재질 `Linen_Blue`, 뒤 2 개는 재질 없음 = 밝은 회색.
    **프로펠러 색은 그대로 둔다**) → ③ **아랫면으로 감쌈**: 옆면만이면 손목 카메라 (아래에서 봄) 에 안 보였다 (GUI 확인) → ④ 구멍 위를 덮게 (위 "시도한 것").
    지금: 손목 카메라에 시작 자세부터 잡기 직전까지 양쪽 빨간 띠, third view 는 옆·아래 비스듬히 선명, Q1·Q2 (몸체 길이 방향) 는 작게
  - 회귀: ① 에서 `grasp_demo --case all` (PX4) 6/6, `check_ros2` 9/9 (리셋 23.8 ~ 23.9 s, RTF 0.978). ④ 뒤 `drone_scene --check` 3/3, `check_ros2` 9/9 (보이기만 하는 메시라 물리 무관)
- [x] **드론 사인파 hover (2026-10-16 사용자 제안·결정)**: 드론 조종자 대신 리셋마다 사인파를 새로 뽑아 자동 비행 (PX4 그대로). 팔 조작자 한 명으로 수집
  - **프리셋 3 가지 (사용자 결정 (가): 이름으로 고르고 작업명을 나눔)** — `sim_ros2.py --mode hover --hover small|mid|big` (드론 설정 `flight.hover_presets`,
    `drone_scene.py`·`grasp_demo.py` 도 같은 인자). GUI 로 세 가지를 보고 "다 좋다" (사용자)

    | 프리셋 | 진폭 합 x, y, z | 주파수 | 최대 속도 (대략) |
    |---|---|---|---|
    | small | 20, 20, 10 mm | 0.2 ~ 0.5 Hz | 6 cm/s, 빠르게 떨림 (예전 hover 값, `check_flight` F3) |
    | mid | 30, 30, 15 mm | 0.05 ~ 0.3 Hz | 6 cm/s, 느리고 크게 |
    | big | 50, 50, 20 mm | 0.05 ~ 0.3 Hz | 9 cm/s |

  - 리셋마다 주파수·위상을 seed (리셋 seed + 1 000 000, 위치 난수와 따로) 로 다시 뽑는다. 진폭은 프리셋 그대로. 사인파는 이륙·도착 뒤 대기가 끝난 뒤 시작 (시작 순간 목표 = 기준 위치)
  - **잡은 뒤에도 사인파는 계속 (2026-10-16 사용자 결정)**: 실물 드론 제어기는 잡혔는지 모르고 하던 대로 움직이려 하고, 멈추는 것은 조작자가 모터를 끌 때 (X) 뿐 →
    sim 도 같게. 사인파는 모터 정지 (`/sim/drone_kill`) 때 멈추고 다음 리셋에서 풂.
    처음에는 그리퍼 닫기 명령 (raw > 575) 순간 사인파 목표에 고정했으나 뺐다: 명령만 보므로 헛잡기에도 멈추고, 고정점이 "그때의 목표" 라 PX4 가 약 2 cm 늦게 따라가는 만큼
    잡힌 드론을 그 점으로 끌어당기며, sim 에만 있는 규칙이라서
  - 기록: 리셋 응답·`episode.json` `sim_reset.drone_ref` (모드·기준 위치·프리셋·진폭·주파수·위상·seed·축별 최대 속도), `sim.drone.hover_preset`,
    `drone_kill` 응답 `hover_freeze` (이유 `kill`, 그 전에 CLI hold 면 `cli_hold`·sim 시각·목표). 드론 위치 토픽은 그대로 녹화하지 않음
  - **같이 쓰면 안 되는 것**: `--mode hover` + `--drone-rc on` → 시작 전 에러 (예전에는 조종기에 넘기는 순간 사인파가 조용히 멈추고 `episode.json` 에는 hover 로 남았음).
    `drone_cmd.py goto` = 기준 위치만 옮기고 사인파 계속, `hold` = 사인파 멈춤. hover 가 아닐 때 CLI 가 목표를 넘겨받은 표시 (`external`) 는 리셋 때 풂 (예전에는 리셋 뒤에도 남았음)
  - **고친 문제**: PX4 + hover 는 원래 동작하지 않았다 (이륙 중 매 루프 "기준 위치 + 사인파" 가 이륙 구간 목표를 덮어써 상승 지점에서 못 벗어나 `PX4 이동 1 실패`).
    PX4 가 기본 제어기가 된 뒤 이 조합을 돌린 적이 없었음 → 이륙이 끝난 뒤에만 사인파 목표를 보냄 (`DroneFlight.hover_active`)
  - 검사 `check_drone_hover.py` (mid) **5/5**: 조종기와 같이 주면 에러 / 리셋 → 사인파 (목표 범위 56·25·20 mm, 속도 상한 안, PX4 추적 RMS 23 mm) / goto = 기준 위치만 옮김 /
    그리퍼 닫기 뒤에도 움직임, kill → 멈춤 (응답 `kill`) / 다음 리셋 새 주파수·멈춤 풀림 (H4 를 바꾼 뒤 5/5). PX4 는 움직이는 목표를 약 2 cm 늦게 따라간다
    (위치 목표만 보내고 속도는 안 보냄. 카메라·조작자가 보는 것은 실제 드론이고 목표는 기록·학습에 안 쓰이므로 수집에는 영향 없다고 봄. 프리셋 진폭·속도는 목표 기준)
  - **잡힌 채 사인파가 계속될 때 (확인 시험, 레포 밖 임시 스크립트)**: PX4 로 정지 드론을 `grasp_demo` 순서대로 잡은 직후 사인파를 켜고 모터 정지를 1·3 s 늦춤 (seed 7)

    | 프리셋·모터 정지 | 판정 | 잡은 채 드론 흔들림 (TCP 기준) | 미끄러짐·회전 | 손가락 힘 | 팔 관절 오차·속도 최대 |
    |---|---|---|---|---|---|
    | mid 1 s | held | 0.07 mm | 0.4 mm·2.1° | 26 / 27 N | 0.043°·0.9 °/s |
    | mid 3 s | held | 0.3 mm | 1.2 mm·1.4° | 27 / 27 N | 0.043°·1.8 °/s |
    | big 1 s | held | 0.19 mm | 1.5 mm·2.1° | 26 / 27 N | 0.043°·0.9 °/s |
    | big 3 s | held | 0.06 mm | 1.8 mm·2.0° | 27 / 27 N | 0.043°·2.4 °/s |

    → 잡힌 드론은 팔에 대해 거의 움직이지 않고 (팔 drive 가 단단함, 드론 추력 여유로 당겨도 팔이 버팀), 손가락 외 로봇–드론 접촉 없음, 보호 정지 기준 (오차 5°, 속도 200 °/s) 에서 멂.
    정지 드론 (모터 정지 1 s) 과 같은 수준. 사람이 3 s 늦게 X 를 눌러도 괜찮다
  - 스크립트 파지 데모 (`grasp_demo --mode hover --hover mid`) 는 잡지 못함 (접근 단계 시간 초과, 위치 오차 4.9 mm): 목표에 수렴해야 다음 단계로 넘어가는 정지 드론용 데모라서.
    hover 수집은 사람 텔레옵으로 한다
  - 회귀: `check_flight` 4/4 (F3 = small, 예전과 같은 값), `drone_scene --check` 3/3, `grasp_demo --case all` (PX4) 6/6, `check_ros2` 9/9, `check_rc` 6/6
  - "여러 시작 지점에서 날아오기" 는 하지 않음 (사용자, 녹화 전에 끝나 데이터에 안 남음)
- [x] **드론 기준 위치 목록 (2026-10-16 사용자 제안·결정)**: VLA 데이터의 드론 위치를 다양하게. 사인파 중심 (기준 위치) 을 25 점 목록에서 고른다
  - `sim_ros2.py --drone-targets` → 리셋마다 `scene_drone.yaml` `drone_targets_base` (**로봇 base 기준**) 를 섞어 한 바퀴씩 돌며 하나 (점마다 개수가 고름, 섞는 seed = [--seed, 바퀴]),
    그 둘레 ± `reset.drone_offset`. `--drone-pos` 와 같이 쓰면 에러. 리셋 응답·`episode.json` `drone_target` (이름·번호·바퀴·위치 world/base), `sim.drone.targets` (목록 이름)
  - **리셋 흔들림 ±5 → ±3 cm** (`ros2_iface.yaml` `reset.drone_offset` 0.03, 사용자 동의): 다양성은 목록이 맡고, 여유 상자를 줄여 팔이 닿는 점을 늘림. 목록을 안 써도 ±3 cm
  - 고른 방법 `compare_drone_targets.py` (새, 헤드리스): base 기준 격자를 점마다 — 시작 자세 (`under_drone`) 에서 잡는 자세·10 cm 아래 접근 자세 URDF IK, 관절 한계 여유 ≥ 5°,
    조작성 (특이점), 자기 충돌·테이블 (팔을 순간이동해 PhysX 링크 자세로), **여유 상자** (리셋 흔들림 + 사인파 진폭) 꼭짓점 8 개 IK, 이륙 경로 (수직 상승 → 수평 이동) 와 시작 자세 팔 거리 ≥ 0.25 m,
    시작 자세 손목 카메라·third view 후보 (Q1·Q2) 에 드론 외곽 상자 투영. 그림에 후보 점을 겹쳐 그림
    - ±5 cm + mid (여유 ±8, ±8, ±6.5 cm), x 0.45 ~ 0.75·y ±0.2·z 0.65 ~ 0.85 (10 cm) 60 점: 19 점만 통과. 실패 = 여유 상자 끝까지 팔이 못 닿음 (앞·위), z 0.65 는 시작 자세 그리퍼와 드론 중심 21 ~ 24 cm (드론 반폭 약 18 cm)
    - ±3 cm + mid (±6, ±6, ±4.5 cm), x 0.45 ~ 0.70·z 0.70 ~ 0.85 (5 cm) 120 점: **88 점 통과**. x 0.45 ~ 0.55 는 높이·좌우 모두, 앞쪽 (0.60 ~ 0.70) 은 낮을수록 멀리 닿음.
      관절 한계 여유 (62° 이상)·특이점·테이블·자기 충돌은 모든 점에서 문제없음
    - 목록 26 점을 big 여유 (±8, ±8, ±5 cm) 로 다시: (0.65, 0, 0.80) 만 IK 6/8 → 빼서 **25 점 (모든 프리셋 통과)**:
      z 0.70: x 0.45·0.55·0.65 × y −0.2·−0.1·0·+0.1·+0.2 (15), z 0.80: x 0.45·0.55 × y 다섯 (10). 지금 기본 위치 (0.60, 0, 0.738) 둘레를 넓게 덮음
  - 닿지 않는 곳 (사용자 걱정: 실물에서 너무 먼 드론으로 팔을 뻗음) → **"뻗었다 실패하고 돌아오기" 시연은 넣지 않는다** (모방 학습이 뻗기를 배움, 같은 장면에 정답이 둘).
    대신 ① 본 데이터는 닿는 영역 안 ② **안전은 코드로**: 닿는 영역 (위 통과 점 + 여유) 판정 함수 + 팔 명령 작업 공간 제한 → 7단계 sim 평가·9단계 실물에서 정책 명령 앞에 둠 (만들 것)
    ③ 필요하면 "닿지 않는 곳이면 기다렸다가 들어오면 잡기" 를 따로 소량 (작업명을 나눔, 첫 학습 루프 뒤 정함) (2026-10-16 사용자 동의)
  - 검사 `check_drone_hover.py` (`--drone-targets` 로 띄움) **6/6**: H6 = 리셋마다 다른 점, 기준 위치 = 점 ±3 cm, `drone_ref.p0` = `drone_goal`. 회귀 `check_ros2` 9/9 (리셋 23.9 s, 드론 오차 6 ~ 7 mm), `check_rc` 6/6
  - **seed** (사용자 질문 "왜 필요한가"): 같은 seed = 같은 점 순서·흔들림·사인파 → 재현 (이상한 에피소드를 그대로 다시 만듦)·7단계 평가 (정책끼리 같은 조건)·검사 (결과 고정) 용.
    수집할 때 같은 seed (예전 기본 0) 로 다시 띄우면 순서가 반복되므로 **`--seed` 를 안 주면 시작 시각 (초 % 1 000 000) 으로 정하고** 터미널·`episode.json` (`seed_base`, `seed_auto`) 에 남김.
    검사 스크립트 (`check_ros2.Sim` 을 쓰는 것) 는 `--seed 0` 고정. 버린 에피소드 (`d`) 도 차례를 쓴다 (다음 리셋은 다음 점)
  - **수집 경로 끝까지 확인 (PICO 없이, 2026-10-16)**: 조작자 문서의 수집 기본 명령 (`--drone-targets --mode hover --hover mid`, seed 없음 → 시각) + 실제 녹화 도구 (`6_record_bag.sh --sim`, 가상 터미널) +
    가짜 텔레옵 (`/teleop/request` B → A → 닫기 → X → A, 팔 명령 60 Hz sim time stamp) 로 에피소드 2 개 → 서로 다른 목록 점 (`x45_y+00_z80`, `x55_y-20_z70`) ±3 cm,
    `episode.json` 에 `drone_target`·`drone_ref` (mid, 리셋마다 seed)·`seed_base`/`seed_auto`·`hover_freeze: kill`·`grasp_marker`, stage1 변환 성공 (`meta.json` 에 그대로). 시험 bag 은 지움
  - ~~**남은 것**: third view 위치를 이 영역 전체 (x 0.45 ~ 0.65, y ±0.2, z 0.70 ~ 0.80 + 여유 + 드론 크기) 가 보이게 고른다 (지금 Q1·Q2 는 기본 위치 기준이라 88 점 중 34 점만 여유 상자까지 다 보임).~~ → 아래 "third view 위치 결정" (Q2).
    시작 자세는 하나 (`under_drone`, 기본 위치 아래): 점에 따라 시작 때 손목 카메라에 드론이 일부만 보임 (점마다 시작 자세는 PLAN 5-B "에피소드 시작 자세", 첫 학습 루프 뒤)
- [x] **third view 위치 결정 (2026-10-22 사용자 결정: Q2)** — base (1.10, −0.30, 0.20) = 로봇 앞 1.1 m, 오른쪽 0.3 m, 상판 위 0.20 m, 보는 점 base (0.539, 0.044, 0.953) (방향 +148.5°, 올려봄 +48.9°)
  - 고른 방법 `compare_third_view_targets.py` (새, 헤드리스): 기준 위치 25 점마다 드론 외곽 상자 (프로펠러 포함, 드론 좌표계 −191 ~ 201, −180 ~ 178, −50 ~ 35 mm) + 여유를 화면에 투영해
    후보 격자 (x 0.9 ~ 1.4, y 0 ~ −0.45, z 0.15 ~ 0.45) 를 거름. 보는 방향은 자동 (넣을 점들이 화면 가운데), 통과 = 테두리 여유 ≥ 8 px·이륙 경로와 카메라·기둥 ≥ 0.25 m.
    상위 후보는 대표 점 9 개 (영역 꼭짓점 + 가운데) 에서 시작 자세·잡는 자세 (닫힘) 를 렌더해 드론 %·로봇에 가려진 드론 %·테이프·손가락 두 쪽·테두리를 잼
  - 조건을 바꿔 가며 본 결과 (렌더 드론 % = 대표 점 9 개 최소 ~ 최대. 참고: 예전 Q1 1.7 ~ 3.5 %)

    | 화면에 넣을 것 | 통과 | 대표 후보 | 렌더 드론 % | 잡을 때 가려짐 최대 |
    |---|---|---|---|---|
    | 외곽 + big 여유 (±8, ±8, ±5 cm) + 시작 자세 그리퍼 | 13/82 | (1.40, −0.30, 0.25) | 0.7 ~ 1.8 % | 12.7 % |
    | 외곽 + big 여유, 그리퍼 뺌 | 17/83 | (1.20, −0.45, 0.25) | 0.9 ~ 2.6 % | 18.5 % |
    | **외곽 + 리셋 흔들림 (±3 cm) 만, 그리퍼 뺌 (채택)** | 14/82 | **Q2 (1.10, −0.30, 0.20)** 테두리 30 px | **1.2 ~ 3.6 %** | 19.8 % |
    |  |  | (1.10, −0.45, 0.25) 테두리 47 px | 1.0 ~ 3.3 % | 19.6 % |

    - 크기를 정하는 것은 **좌우 폭**: 기준 위치 y ±0.20 + 여유 + 프로펠러 반폭 0.18 → 약 ±0.4 ~ 0.46 m 를 가로 79.3° 에 넣어야 함. 정면 (y 0) 은 폭이 넘치고 이륙 지점과 겹쳐 모두 실패, 통과는 대각선 (y −0.3 ~ −0.45)
    - 사용자 결정: 시작 자세 그리퍼는 화면에 꼭 넣지 않음 (손목 카메라가 봄. Q2 에서는 화면 아래 끝에 일부), 사인파 진폭은 넣지 않음 (big 이면 프로펠러 끝이 잠깐 화면 밖으로 나갈 수 있음),
      Q2 = 전에 "입체적" 으로 고른 위치, 테이블 가장자리 (y −0.5) 에서 20 cm 안쪽이라 실물에 놓기 쉬움
    - 렌더한 모든 후보에서 손가락 두 쪽이 9 점 모두 따로 보임. 잡을 때 테이프는 30 ~ 70 % 가려짐 (손가락이 테이프 위에 있음)
    - 리포트 `isaacsim/reports/compare_third_view_targets_20261016_195655` (엄격), `_20261022_184604` (그리퍼 뺌), `_20261022_185418` (채택 조건)
  - **sim 이륙 지점 (1.2, 0) → (1.2, 0.15)** (`scene_drone.yaml` `px4.takeoff_xy`): Q2 와 수직 상승 경로 거리 0.25 → 0.39 m (드론 반폭 약 0.18 m). 실물 드론은 사람이 띄우므로 실물과 무관.
    `compare_drone_targets.py` (25 점, big 여유) 이륙 경로–팔 0.26 ~ 0.29 m (기준 ≥ 0.25, 바뀌기 전 값은 리포트 없음), 설정 위치 third view 에 big 여유 상자까지 다 보임 23/25 (x65_y±20_z70 이 75·88 %: 채택 조건은 리셋 흔들림만이라 예상대로), 25 점 모두 팔 통과
  - **카메라 모양 = D455 메시** (`scripts/build_d455_usd.py` → `assets/cameras/d455/d455.usdc` 0.41 MB): realsense2_description (ROS 2 Jazzy, Apache-2.0) 의 `d455.stl`·`_d455.urdf.xacro` 에서
    프레임 값을 읽어 만듦. 최상위 prim = 아랫면 1/4-20 나사 자리, 메시 외곽 26 × 124 × 29 mm (xacro 와 같은지 확인, 다르면 에러), RGB optical frame = 나사 자리에서 앞 11.2·오른쪽 11.5·위 14.5 mm,
    몸체 상자 충돌 (정적). third view prim 이름 `/World/ThirdView/d435i` → `/World/ThirdView/camera` (`drone_scene.THIRD_VIEW_LINK`). `episode.json` `cameras.third_view.model` 에 모양 USD 를 기록
  - 받침대 높이 (sim, 기둥) 184 mm. 실물 받침대는 CAD 로: 렌즈 위치·방향 (올려봄 49°) 에 맞춘 기울어진 받침 (아래)
- [x] **third view 받침대 CAD** (출력·설치 안내 `docs/third_view_mount.md`) **(2026-10-22, 사용자 결정: 카메라 = 뒷면 M4 × 2, 테이블 = 나사 (Ø5.5 × 4, M5 볼트·목재 나사), 고정 각도)** — `isaacsim/cad/third_view_mount_cad.py` (CadQuery, `~/cad/venv`)
  → `isaacsim/cad/out/third_view_mount.step`·`.stl`·`preview.png`. 카메라 자세는 `third_view_camera.yaml`·`scene_drone.yaml` 에서 읽음 (위치를 바꾸면 다시 실행)
  - D456 고정 구멍 (Datasheet 337029-017 Figure 10-14, `~/Downloads/Intel-RealSense-D456-Datasheet-en.pdf`): **뒷면 M4 × 2 간격 95 mm, 최대 체결 깊이 4 mm**, 아랫면 1/4-20 최대 6 mm.
    D456 은 USB-C 단자가 아랫면 (1/4-20 에서 37 mm 옆, 케이블 고정 M2 × 2) → 뒷면으로 고정하고 아랫면은 비움
  - 모양: 카메라 뒷면 판 110 × 29 × 5 mm (48.85° 기울임, Ø4.5 × 2) ← 기둥 20 × 40 mm (윗면이 판 뒷면을 따라 기울어짐, 뒤·옆 리브 높이 60) ← 바닥판 115 × 100 × 7 mm
    (Ø5.5 × 4, 카운터보어 Ø10.5 깊이 3.5). 높이 193.5 mm, 245 cm³. 카메라 나사 **M4 × 8** (판 5 mm → 카메라에 3 mm)
  - 스크립트가 확인 (어긋나면 에러): 카메라 몸체·아랫면 아래 60 mm (USB 자리) 와 겹치지 않음, 나사 체결 깊이 0 ~ 4 mm, 렌즈 높이 = 설정 (200.0 mm)
  - 설치: 바닥판 윗면 홈 = 원점 십자 (1/4-20 자리 바로 아래) + 화살표 (카메라가 보는 방향). 로봇 base 기준 **십자 (1090.9, −307.9) mm, 화살표 끝 (1065.4, −292.2) mm** → 두 점으로 위치·방향 (148.5°) 을 맞춤
  - 출력: 바닥판을 아래로. 기울어진 판 아랫면은 수직에서 41° (서포트 없이 될 것으로 봄). 기둥 앞뒤 20 mm 는 판이 바닥에 투영되는 폭 (22 mm) 에 맞춤 → 실물에서 흔들리면 뒤로 두껍게
  - 남은 것: 실물 출력·설치 뒤 실물 third view 영상과 sim 영상 비교, sim 받침대 (지금 원기둥) 를 CAD 모양으로 바꿀지
- [x] **GUI 메인 뷰포트 처음 시점 (2026-10-22 사용자 요청)**: 텔레옵할 때 방향이 헷갈림 → 로봇 바로 뒤 1.3 m·높이 2.2 m 에서 드론 쪽 (+x) 을 봄 (`drone_scene.GUI_EYE`·`GUI_TARGET`, `sim_ros2.py`·`drone_scene.py` 공용).
  텔레옵 노드는 컨트롤러 "앞" 을 base +x 에 맞추므로 화면 앞 = 컨트롤러 앞. 예전 (1.9, −1.6, 1.7) 은 앞 오른쪽에서 마주 봄. 로봇 뒤 0.9 m 는 팔꿈치가 그리퍼를 가려 더 뒤·위로. 녹화 카메라와 무관 (사용자 GUI 확인)
  - `check_ros2` 5 번 (이미지 지연) 을 고침: 화면 전체 평균 변화로 찾던 것을 **팔이 옮겨 간 픽셀만** 보게 (계단 전·1 s 뒤 평균 영상 차이 > 10 인 픽셀). Q2 에서는 팔이 화면 아래에 작게 보이고
    호버 드론 흔들림이 섞여 "이미지 변화를 못 찾음" 으로 실패했었다 (팔 픽셀 355 개, 지연 0 프레임)
  - 회귀: `drone_scene --check` 3/3, `grasp_demo --case all` (PX4) 6/6, `check_drone_hover` 6/6 (리셋 23.9 s), `check_ros2` 9/9 (5 번 고친 뒤, 리셋 23.8 ~ 23.9 s, 드론 오차 4 ~ 7 mm, RTF 1.005),
    `check_rc` 6/6·6/6 (첫 번째는 5/6: R2 roll "놓은 뒤 마지막 1 s 이동" 20.5 mm > 기준 20 mm. 이륙 지점을 바꾸기 전에도 18.4·3.9·18.6 mm 로 기준 가까이 오가던 값 → 기준은 그대로, 계속 보기)
- [x] **첫 소량 수집 (2026-10-23, PICO, `drone_grasp_t10_f1_tgt_hmid`)**: 25 번 시도 → 저장 13 개 (6337 프레임, 에피소드 12 ~ 30 s), 버림 12 개. 변환·병합·검수 5/5 PASS
  (`lerobot_dataset_v21/local/drone_grasp_tgt_hmid_first13`): 13 개 모두 파지 신호 (닫힌 state 0.18 ~ 0.28)·모터 정지 기록, 보호 정지 0, 앞뒤 정지 0.04 ~ 0.24 s, 명령 끊김 0
  - 성공률이 드론 위치에 따라 다름: z 0.70 11/15, **z 0.80 2/9**, **x 0.45 2/9** (x45 z80 은 5/5 실패), x 0.55·0.65 11/16. 사용자: "그냥은 잡기 어렵다" → 메인 화면만으로는 앞뒤·높이 거리감이 없음
  - 수집 중 RTF 0.79 → 0.66 으로 조금씩 낮아짐 (뒤 4 개는 `/joint_command` 간격 중앙값 8.3 ms). 데이터 문제는 아님, 긴 세션에서 계속 보기
- [x] **GUI 보조 뷰포트 옆·위 (2026-10-23 사용자 요청)**: 메인 오른쪽에 **Side view** (로봇 오른쪽 (−y) 에서 +y 를 봄, 화면 오른쪽 = 앞) 와 **Top view** (위에서 7° 기울여 내려봄, 화면 위 = 앞)
  (`drone_scene.GUI_AUX_VIEWS`·`open_aux_views`, 카메라 prim `/World/GuiViews/*`, ROS 발행 없음). `sim_ros2.py --aux-views on|off` (기본 GUI on, headless 와 on 을 같이 주면 에러).
  렌더 해상도 512x288 (`resolution_scale` 0.4). RTF 0.98 → 0.82 (드론 잡기 전, `docs/sim_performance.md` 4.2). 녹화와 무관 (사용자 GUI 확인)

---

## 7단계: 학습과 sim 평가

- openpi 설정: 7차원 state/action, 카메라 키 매핑(`wrist` → 손목 슬롯, `third_view` → base 슬롯), LoRA
- delta action: `DeltaActions(make_bool_mask(6, -1))` (관절 6개 delta, 그리퍼 절대값)
- norm stats: **자체 데이터로 계산** (π0.5 문서상 제공되는 사전학습 stats 없음). 계산 후 그리퍼 action 의 q01/q99 가 0 과 1 로 잡혔는지 확인 (닫힘 프레임이 1% 미만이면 q99=0 → 정규화 붕괴)
- 추론: 25 Hz, 그리퍼 출력 0.5 기준 threshold → 0/1150 goal
- 서버에서 학습
- **sim 폐루프 평가**: openpi 정책 서버 ↔ sim 클라이언트로 실제 파지 시도, 성공률 측정
- action chunk 실행 주기 등 추론 설정을 sim 에서 튜닝

**진행 (계획·결과 전체는 `docs/openpi_finetuning.md`)**
- [x] **LoRA smoke 100 step (2026-10-23)**: first13 (13 에피소드, 6337 프레임), openpi fork `~/openpi` 커밋 `9408e3b` (`pi05_ur5_lora`: 손목 키 `observation.images.wrist`, repo_id `foodbanana/drone_grasp_tgt_hmid_first13`).
  연구실 서버 1 GPU 0·1 data parallel, batch 32, 2.5 s/step (GPU 1 장 4.3 s), loss 0.0736 → 0.0514, `param_norm` 그대로, wandb 카메라 배치 확인 (base = third view, 왼쪽 손목 = 손목).
  데스크탑 확인: norm stats 그리퍼 action q01 0 / q99 1, action 이 state 를 1 프레임 앞섬 (action[t] ≈ state[t+1], 팔 RMS 0.063°)
- [x] **full 파인튜닝 2 GPU FSDP smoke (2026-10-23)**: `pi05_ur5` (LoRA 없음, EMA 끔, `fsdp_devices=2`). batch 32 는 OOM (activation 약 32 GB > 남은 약 23 GB), **batch 16 통과**:
  loss 0.082 → 0.037, 1.9 s/step (같은 샘플 수당 LoRA 의 약 1.5 배), 체크포인트 31 GB
- [x] 추론 경로 (LoRA smoke 체크포인트, 가짜 입력 → actions (10, 7))
- [x] **full 10k 학습 + sim GUI 폐루프 (2026-10-23 ~ 26)**: first13 그대로 10 000 step (약 25 에포크, loss ≈ 0.00 과적합). 정책 클라이언트 노드 `policy/policy_client_node.py` (데스크탑 GPU 에 sim GUI + 정책 11.8 GB / 16 GB).
  6 에피소드 (seed 0, 목록 점·hover mid) 중 **5 개 파지** (사용자 GUI 확인), 4 개는 학습에 없던 드론 위치. 약점: 그리퍼가 닫기·열기를 오감 (6 개 중 3 개), 1 개는 그래서 놓침.
  결정 (사용자): 데이터를 더 모으고 과적합을 줄여 정책으로 해결. 성공 자동 판정·자동 모터 정지 조건은 만들지 않음 (사람이 보고 모터 정지)
- [ ] 본 학습 (데이터를 더 모은 뒤. step 수에 맞춰 lr schedule 의 warmup·decay 도 조정, 서버 디스크에 맞춰 체크포인트 저장 간격)
- [ ] sim 폐루프 평가 (본 학습 정책, 같은 `--seed` 로 비교)

**완료 기준**
- [ ] 고정 베이스에서 sim 파지 성공률이 수치로 나옴

---

## 8단계: 흔들리는 베이스 (1~7단계 완료 후)

- 6DOF 베이스를 kinematic 바디로 모델링 (사인파 합성 또는 실제 IMU 로그 기반 궤적)
- 로봇 고정: 씬에서 **root_joint 의 body0 을 움직이는 베이스 강체로 바꾸는 방식**을 검토. 이때 fixed base override 를 쓸지
  (`--base floating` 으로 USD 원본 구조를 쓸지)도 다시 결정 (1-4 참고. 최상위 prim 은 강체가 아니므로 Transform 을 움직여도 물리적으로 따라오지 않음)
  → 1~4단계 구조 유지, 씬 스크립트에 `--base kinematic` 추가
  - **메모 (2026-10-07, 구조 정리와 방식 후보. 아직 해 보지 않음 → 구현할 때 확인)**
    - 지금 구조: `root_joint` = 고정 관절, body0 = 로봇 최상위 prim (강체가 아님 → 세상의 그 자리에 고정), body1 = `robot_mount`. 지우지 않고 유지해 왔다.
      `ArticulationRootAPI` 는 **USD 파일에는 `robot_mount`** (PhysX 가 `root_joint` 를 articulation 밖 구속으로 봄 → floating base, 질량행렬 16×16),
      **씬 (메모리) 에서는 최상위 prim** 으로 옮긴다 (`test_scene._make_fixed_base` → `root_joint` 가 articulation 의 첫 관절 → fixed base, 10×10). 파일은 그대로
    - 방식 후보

      | 방식 | 내용 | 팔이 관성력을 느끼나 | 고칠 양 | 걱정 |
      |------|------|----------------------|---------|------|
      | A (먼저 시도) | kinematic 강체 (플랫폼) 를 궤적대로 움직이고 `root_joint` 의 body0 을 그 강체로. `ArticulationRootAPI` 는 USD 원본 위치 (`robot_mount`, floating) | 예 | 적음 (`--base kinematic` 자리가 비어 있음) | 연결이 articulation 밖 구속이라 플랫폼과 `robot_mount` 가 미세하게 벌어지거나 떨릴 수 있음. 질량행렬 16×16 (`compute_gain_seed.py` 그대로는 안 됨) |
      | B (A 가 안 되면) | 고정된 뿌리와 `robot_mount` 사이에 직선 3 + 회전 3 관절을 넣고 drive 로 구동 (fixed base, DOF 16) | 예 | 많음 (DOF 10 → 16: `check_articulation` 기대값, drive 설정 구조, 질량행렬을 쓰는 코드) | 회전 3 관절의 순서 (큰 각도) |
      | C (쓰지 않음) | 최상위 prim 위치를 매 스텝 덮어씀 (순간이동) | **아니오** | 가장 적음 | PhysX 가 속도·가속도를 모름 → 관성력 없음, IMU 값 틀어짐 (아래 "베이스 구동 방식 주의") |

    - A 를 만들면 먼저 볼 것 (드론·ROS 없이 `test_scene` 수준에서, 사인파로 흔들며): 플랫폼 ↔ `robot_mount` 상대 움직임 (0 에 가까워야 함),
      `tune_drives.py` A~F 가 floating 에서도 통과하는지, 아래 완료 기준 (sim IMU = 수식 계산값)
    - `ArticulationRootAPI` 를 USD 파일에 최상위 prim 으로 넣지 않는 이유: 손으로 고치면 재import 때 사라짐, 이 단계에서 floating 구조가 필요할 수 있음,
      고정 방식은 로봇이 아니라 씬의 성질. **확인해 볼 것**: importer 의 Base Type 을 Fixed 로 하면 (1-3 에서는 Source) 처음부터 fixed 구조로 나오는지
      (손 편집이 아니므로 재import 문제가 없음). 베이스 방식이 정해진 뒤에 결정
    - 방식과 무관하게 같이 고칠 곳: 플랫폼을 보호 정지 `environment_paths` 에 추가하고 기준 (위치 오차·관절 속도) 오탐 다시 측정, `/sim/reset` 이 베이스 움직임도 되돌림,
      `check_articulation.py` 의 `fixed_base` 기대값을 모드별로, `record_toggle.py` 토픽에 `/base/imu`, 메타데이터의 "로봇 base 기준" 위치를 어느 시각의 base 로 볼지,
      테이블·third view 카메라 배치
- **베이스 상태 기록 (bag → intermediate 는 상위집합, 데이터셋에서 선택)**
  - bag: 베이스 IMU(`sensor_msgs/Imu`, 수백 Hz). sim 은 Isaac IMU 센서로 같은 토픽 + ground-truth 베이스 pose/twist 추가
  - D435i 내장 IMU 는 **손목**의 움직임이라 베이스 IMU 를 대체하지 못함 (필요하면 별도로 녹화만)
  - stage1: `base_imu.npy` 는 4단계에서 이미 구현됨. (sim) ground-truth `base_pose.npy` 를 intermediate 에 추가 (데이터셋에는 넣지 않음)
  - 데이터셋: `observation.base_imu` 는 0단계에서 확정된 그대로 (고정 베이스 데이터와 열 구성이 이미 같으므로 병합 시 후처리 불필요)
  - 정책에 쓰려면 openpi repack 에 `observation.base_imu` 매핑 + Inputs transform 에서 state 에 이어붙이기 + **norm stats 재계산**. 추론 때도 같은 방식(25 Hz 구간 평균)으로 IMU 를 넣어야 함
- **`/base/imu` 발행은 이 단계에서 시작** (2026-10-05 결정: 고정 베이스 1~7단계는 sim·실물 모두 발행 안 하고 stage1 `--base-imu const`).
  sim IMU 센서와 실물 IMU 를 함께 붙이고, 이 단계 데이터는 `--base-imu const` 없이 변환
  - **설정 위치 (2026-10-07)**: `/base/imu` 의 토픽 이름·주기·frame_id 는 `isaacsim/config/ros2_iface.yaml` 에 둔다
    (팔·그리퍼·카메라 토픽과 같은 곳. 발행 큐 `publish_queue_sec` 도 같이 적용된다: 100 Hz 이상이라 기본 깊이 10 이면 0.1 s 분량).
    센서의 위치·방향·잡음은 카메라처럼 따로 둔다 (어느 파일인지는 베이스 설정을 만들 때 정함)
- **sim 에서 베이스 IMU 얻기** (물리적 IMU 모델 불필요, 가상 센서)
  - 방법 1 (기본): 베이스 링크에 Isaac Sim IMU 센서 prim 을 자식으로 추가 → 몸체 좌표계 선가속도·각속도·자세 → `ROS2 Publish Imu` 노드로 `sensor_msgs/Imu` 발행
    - 6.0 부터 `isaacsim.sensors.physics` IMU 는 deprecated, `isaacsim.sensors.experimental.physics.IMUSensor` 권장 → 6.1.0 에서 어느 API 를 쓸지 확인
    - IMU prim 위치·방향 = **실물 IMU 장착 위치·방향**
  - 방법 2 (검증·대체): 베이스 궤적을 직접 만들므로 수식으로 계산
    - 각속도 = 몸체 좌표계로 변환한 각속도, 선가속도 = `R^T (a_world − g)` (정지 시 위쪽 +9.81, 실제 IMU 규약)
    - 실제 IMU 수준의 노이즈·바이어스를 더해 실물과 같은 토픽으로 발행
  - **베이스 구동 방식 주의**: 매 프레임 xform 을 덮어써 순간이동시키면 물리 엔진이 아는 속도·가속도가 0 이거나 튀어서 IMU 값이 틀어짐 → kinematic target 또는 관절 drive 로 구동
- 실물: 베이스에 실제 IMU 장착. 6DOF 베이스가 모션 플랫폼이면 플랫폼 컨트롤러의 자세·속도 피드백을 IMU 대신 쓰는 것도 검토 (정확도·지연 비교)
- 정책이 베이스 움직임을 관측할지 결정 → state 확장 시 재학습/추가 파인튜닝
- third view 카메라를 world 에 둘지 플랫폼 위에 둘지 결정
- RL 경로: 같은 USD 를 Isaac Lab 으로 (Isaac Sim 6.1.0 과의 버전 호환성 확인 필요)

**완료 기준 (베이스 IMU)**
- [ ] 방법 1 IMU 값과 방법 2 계산값이 노이즈 범위 안에서 일치 (= 베이스 구동 방식이 올바름)
- [ ] 정지 상태에서 선가속도 ≈ (0, 0, +9.81) (IMU 축 기준), 각속도 ≈ 0
- [ ] IMU 토픽 header.stamp 가 sim time, 발행 주기 ≥ 100 Hz

---

## 9단계: sim2real

- 실물 파이프라인을 같은 스키마(카메라 키, 그리퍼 스케일)로 통일
- 실물 카메라 장착 위치를 sim 과 일치 (1-7 에서 확정한 `cam_tilt` 로 실물 마운트 출력)
- sim/실물 데이터 혼합 비율 실험
- **실물 시계 확인** (자세한 것은 4단계 "실물 stamp 동기화 문제 (D1)": 토픽별 stamp 주체, 재는 방법, 대응 후보): RealSense stamp(카메라 하드웨어 시각 또는 PC 시각)와 UR 드라이버 stamp(PC 시각)가 같은 기준인지.
  stage1 은 header.stamp 로 맞추므로 기준이 다르면 이미지와 관절이 어긋난다 (sim 은 둘 다 sim time)
- sim camera_info 의 fy 는 fx 로 맞춰진다 (렌더러가 정사각 픽셀만, 실물 fy 618.956 vs fx 618.551, 0.07%). stage1 은 camera_info 를 쓰지 않음

---

## 부록 A: 로봇 description 요약

```
robot_mount ─ base_link ─ ... ─ wrist_3_link ─ flange ─ tool0
                                                        └─ wrist_mount (샌드위치)
                                                             ├─ gripper_bracket
                                                             │    └─ rh_p12_rn_base
                                                             │         ├─ rh_r1_joint (구동, 0~1.1351 rad) ─ rh_r2 (mimic x1)
                                                             │         ├─ rh_l1 (mimic x1) ─ rh_l2 (mimic x1)
                                                             │         └─ rh_p12_rn_tcp
                                                             └─ wrist_camera_mount
                                                                  └─ wrist_camera_bottom_screw_frame
                                                                       └─ … ─ wrist_camera_color_optical_frame
```

- 구동 조인트: `shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint, rh_r1_joint`
- 그리퍼 소스: ROBOTIS `open_manipulator` (main) `rh_p12_rn_a.urdf.xacro` (Apache-2.0). RH-P12-RN 과 (A) 는 펌웨어만 다르고 기구 동일
- 관성: STL 메시로 재계산 (원본 1e-6 placeholder)
- 실물의 수동(adaptive) 관절은 모델링되지 않음 (평행 그리퍼로 동작)
- 카메라: `realsense2_description` 의 D435i macro, `use_nominal_extrinsics=true`
- USD 에서: root_joint(최상위 prim ↔ `robot_mount`), mimic 은 `NewtonMimicAPI`(PhysX 동작 확인), 질량 없는 링크는 Xform

## 부록 B: 토픽 인터페이스 (sim = 실물)

자세한 표 (타입·주기·QoS·stamp·sim 과 실물 차이)와 구조도는 **`docs/sim_ros2_interface.md`**. 주기는 sim 3단계 기준 (sim time).

| 토픽 | 방향 | 주기 | 내용 |
|------|------|------|------|
| `/clock` | sim → ROS | 120 Hz | sim time |
| `/joint_states` | sim → ROS | 120 Hz (실물 125 Hz) | 팔 6관절 position·velocity·effort (실물 effort = 전류 A, sim = 토크 Nm) |
| `/joint_command` | teleop → sim | 텔레옵 주기 (일정하게) | 팔 관절 목표 (**팔 action**). `header.stamp` 는 보내는 쪽이 넣는다 (stage1 이 이 stamp 로 맞춤). sim 은 물리 스텝마다 선형 보간 (33 ms) |
| `/gripper/command` | teleop → 브리지 | 이벤트 | raw 0 또는 1150 (열기/닫기), std_msgs/Float64 |
| `/gripper/joint_states` | 브리지 → ROS | 30 Hz | 그리퍼 present, raw → stage1 에서 /1150 (**state[6]**) |
| `/gripper/target` | 브리지 → ROS | 30 Hz (present 와 같은 stamp) | 실행된 goal, raw 0/1150 → stage1 에서 /1150 (**action[6]**) |
| `/cam/wrist/color/image_raw` (+ `camera_info`) | 카메라 → ROS | 30 Hz | `observation.images.wrist` (640x480 rgb8). 받는 쪽은 RELIABLE |
| `/cam/third_view/color/image_raw` (+ `camera_info`) | 카메라 → ROS | 30 Hz | `observation.images.third_view` |
| `/protective_stop` | sim → ROS | 30 Hz | 보호 정지 흉내 (sim 전용, std_msgs/Bool). stage1 `meta.json` 에 발생 여부·시각, 병합에서 기본 제외 |
| `/base/imu` | IMU → ROS | **8단계부터** | 고정 베이스에서는 발행 안 함, stage1 `--base-imu const`. `sensor_msgs/Imu` → stage1 구간 평균 → `observation.base_imu` |

| 서비스 | 내용 |
|--------|------|
| `/sim/reset` (std_srvs/Trigger, sim 전용) | 에피소드 리셋: 드론 kill → 순간이동 + PX4 재시작 → 팔을 `--init-pose` 자세 (안 주면 홈) 로·그리퍼를 열림으로 순간이동 → 재이륙 → 새 호버 위치 (약 24 s), 응답 JSON (시드·드론 위치·에피소드 메타데이터) |
| `/sim/drone_kill` (std_srvs/Trigger, sim 전용) | 드론 모터 정지 (잡은 뒤, 녹화 도구의 `k` 키 또는 PICO 컨트롤러 X). 다시 켜는 것은 `/sim/reset` |

## 부록 C: Claude Code 사용 방법

- 이 파일은 레포의 `docs/PLAN.md`
- 0단계 확정값(스키마, 토픽 이름, 원칙)과 경로 규칙은 `CLAUDE.md`
- 작업은 단계 단위로 요청하고, 각 단계의 **완료 기준을 검증 항목으로 그대로 전달**한다
- sim 전용 코드는 `isaacsim/` 폴더, `isaacsim_v6.1.0` 브랜치에서 작업한다

---

## 부록 D: 단계별 실행 명령 (bash)

모두 레포 최상위(`~/data_collection_ur5_gripper`)에서 실행. Isaac Sim 스크립트는 `~/isaacsim/python.sh`, `--headless` 는 점검(결과 PASS/FAIL, 리포트는 `isaacsim/reports/`),
빼면 GUI (실제 시간 속도, 창을 닫으면 종료). 결과 해석은 각 단계 절 참고.

```bash
cd ~/data_collection_ur5_gripper
```

### 1단계: 로봇 에셋

```bash
# 1-2 URDF 빌드 (xacro → out/, git 제외). 기구학 yaml 은 실물 로봇 값으로
isaacsim/ur5_rh_p12_description/scripts/build_urdf.sh kinematics_params:=$HOME/my_robot_calibration.yaml
#   손목 카메라 마운트: mount_variant:=t10 (기본) | t7 | t0 | original  (마운트 메시·충돌·질량·카메라 위치를 같이, 5-B "손목 카메라 마운트")
# 1-3 import 는 Isaac Sim GUI (URDF importer, 1-3 의 옵션). USD Output 은 레포 밖 임시 폴더 → 레포 USD 와 비교 → isaacsim/assets/robots/ur5_rh_p12_d435i/ 교체
#     (순서: isaacsim/ur5_rh_p12_description/README.md "Isaac Sim 으로 가져오기")
# 손목 카메라 마운트 CAD (레포 밖 venv ~/cad/venv, CadQuery) → isaacsim/ur5_rh_p12_description/cad/out/
~/cad/venv/bin/python isaacsim/ur5_rh_p12_description/cad/wrist_mount_cad.py
# 손목 카메라 후보 비교 (렌더링 카메라만 옮김) / 카메라 몸체 여유
~/isaacsim/python.sh isaacsim/scripts/compare_wrist_camera.py --headless [--cand "이름:s,d,b,t" ...]
~/isaacsim/python.sh isaacsim/scripts/wrist_camera_clearance.py --headless [--cand ...]

# 1-4 구조 회귀 검사 (로봇 USD 재import 할 때마다) → 8/8 PASS
~/isaacsim/python.sh isaacsim/scripts/check_articulation.py --headless
~/isaacsim/python.sh isaacsim/scripts/check_self_collision.py --headless

# 1-5 drive 튜닝 (자세한 것: docs/drive_tuning.md)
~/isaacsim/python.sh isaacsim/scripts/compute_gain_seed.py --headless
~/isaacsim/python.sh isaacsim/scripts/tune_drives.py --headless
~/isaacsim/python.sh isaacsim/scripts/tune_drives.py --tests ACD --configs isaacsim/config/drive_gains.yaml --realtime   # GUI

# 1-6 손가락 collider·마찰·파지 시험 (파지력: docs/gripper_force.md)
~/isaacsim/python.sh isaacsim/scripts/grasp_tests.py --headless
~/isaacsim/python.sh isaacsim/scripts/grasp_tests.py --tests K,G1 --realtime --wrist-view                              # GUI, 일부만

# 1-7 손목 카메라
~/isaacsim/python.sh isaacsim/scripts/check_wrist_camera.py --headless
~/isaacsim/python.sh isaacsim/scripts/check_wrist_camera.py --hold --tilts 0                                           # GUI 손목 시점
```

### 2단계 1번: 드론 씬·기하 제어기 드론

```bash
# 2-1 드론 에셋 점검 (드론 설정·에셋을 바꿀 때마다) → 4/4. 간이 드론은 먼저 빌드
~/isaacsim/python.sh isaacsim/scripts/check_drone.py --headless
~/isaacsim/python.sh isaacsim/scripts/build_simple_drone.py
~/isaacsim/python.sh isaacsim/scripts/check_drone.py --headless --drone-config isaacsim/config/drone_simple.yaml

# 2-2 기하 제어기 비행 시험 (드론 설정과 무관하게 기하) → 4/4
~/isaacsim/python.sh isaacsim/scripts/check_flight.py --headless --prop-spin on
~/isaacsim/python.sh isaacsim/scripts/check_flight.py --view --mode hover --prop-spin on --release-after 8              # GUI

# 2-3 씬 (테이블·로봇·카메라·드론) — 기하 제어기로 보려면 --flight geometric
~/isaacsim/python.sh isaacsim/scripts/drone_scene.py --headless --check                                                # → 3/3
~/isaacsim/python.sh isaacsim/scripts/drone_scene.py --flight geometric --mode hover --prop-spin on                      # GUI
~/isaacsim/python.sh isaacsim/scripts/drone_scene.py --init-pose -0.1888 -0.7854 0.9599 1.3963 -1.5708 0.1888          # 팔 시작 자세

# 2-4 스크립트 파지 데모 + 자동 판정
~/isaacsim/python.sh isaacsim/scripts/grasp_demo.py --headless --case all --flight geometric                          # → 6/6
~/isaacsim/python.sh isaacsim/scripts/grasp_demo.py --case offset_y --flight geometric --prop-spin on --hold            # GUI
~/isaacsim/python.sh isaacsim/scripts/grasp_demo.py --case success --tcp-offset 0 0.005 0 --flight geometric            # 어긋나게 잡기
```

### 2단계 2번: PX4 SITL 드론 (기본 제어기)

```bash
# 설치 (한 번, 2단계 2번 P-1): PX4 v1.16.0, 빌드용 venv, pymavlink
cd ~ && git clone --branch v1.16.0 --depth 1 https://github.com/PX4/PX4-Autopilot.git && cd PX4-Autopilot
git submodule update --init --recursive --depth 1
git -C platforms/nuttx/NuttX/nuttx tag nuttx-11.0.0 HEAD          # shallow clone 이라 버전 헤더용 태그가 없음 (SITL 영향 없음)
python3 -m venv .venv && .venv/bin/pip install -r Tools/setup/requirements.txt
PATH=$HOME/PX4-Autopilot/.venv/bin:$PATH make px4_sitl_default none   # 빌드 뒤 PX4 셸이 뜨면 Ctrl+C
cd ~/data_collection_ur5_gripper
~/isaacsim/python.sh -m pip install --target isaacsim/.pydeps pymavlink

# PX4 비행 시험 (0.75 배 Iris, 모션캡처, 120 Hz) → 5/5
~/isaacsim/python.sh isaacsim/scripts/check_px4.py --headless --drone-config isaacsim/config/drone_iris.yaml --start-z 0.08 --physics-hz 120
~/isaacsim/python.sh isaacsim/scripts/check_px4.py --view --drone-config isaacsim/config/drone_iris.yaml --start-z 0.08 --physics-hz 120 --prop-spin on   # GUI
#   비교 옵션: --position-source mocap|flow|gps, --motor-lag on|off, --sensor-compat on|off, --px4-param NAME=VALUE
#   Pegasus 기준선: --drone-config isaacsim/config/drone_iris_pegasus.yaml --position-source gps --physics-hz 250

# 씬 (테이블 위 이륙 → 호버, 약 25 s), waypoint
~/isaacsim/python.sh isaacsim/scripts/drone_scene.py --prop-spin on
~/isaacsim/python.sh isaacsim/scripts/drone_scene.py --drone-waypoints 1.0,0.2,1.3 0.8,-0.1,1.6 --drone-pos 0.6 0 1.5 --prop-spin on

# 다른 터미널에서 드론 명령 (drone_scene.py 에서, status 의 ready 가 true 가 된 뒤. 시스템 python3)
python3 isaacsim/scripts/drone_cmd.py status
python3 isaacsim/scripts/drone_cmd.py goto 0.65 0.05 1.45 --yaw-deg 20
python3 isaacsim/scripts/drone_cmd.py hold          # land | kill (kill 은 언제나)

# 파지 데모 (PX4 드론) → 6/6. 잡은 뒤 모터 정지까지 --kill-delay [s] (기본 1)
~/isaacsim/python.sh isaacsim/scripts/grasp_demo.py --headless --case all
~/isaacsim/python.sh isaacsim/scripts/grasp_demo.py --case success --kill-delay 10 --prop-spin on --hold                # GUI
~/isaacsim/python.sh isaacsim/scripts/grasp_demo.py --case success --position-source flow --prop-spin on                # 참고: flow 는 팔이 아래로 오면 드론이 도망감

# PX4 비행 로그 분석 (isaacsim/reports/px4_<시각>/log/*/*.ulg): pyulog (레포 밖 venv 에 설치해서)
```

### 3단계: sim ROS 2 인터페이스
```bash
# 모든 sim ROS 2 실행 전에 (python.sh 안 rclpy 가 시스템 Jazzy 를 쓰게)
source /opt/ros/jazzy/setup.bash

# 3-1 RTF·카메라 주기 측정 (카메라 받는 쪽 = 별도 프로세스 topic_rate.py, RELIABLE). 리포트 isaacsim/reports/measure_rtf_<시각>/
~/isaacsim/python.sh isaacsim/scripts/measure_rtf.py --headless --loop-hz 30                  # 기본 드론(PX4), 카메라 tickRate 30
~/isaacsim/python.sh isaacsim/scripts/measure_rtf.py --loop-hz 30                             # GUI (메인 뷰포트)
#   옵션: --flight geometric|px4, --tick-rate 0|30, --cameras off, --camera-windows (GUI), --prop-spin on, --profile (cProfile → profile.txt),
#         --init-pose q1..q6, --arm-motion, --save-images, --dlss-mode 0|1|2, --anti-aliasing 3|4 (렌더 비교, 기본은 ros2_iface.yaml render)

# sim ROS 2 인터페이스 (실제 시간 속도). GUI: 메인 뷰포트 + 보조 옆·위 뷰포트 (--aux-views off) + 프로펠러 회전 (손목·third view 는 rqt_image_view 등으로)
~/isaacsim/python.sh isaacsim/scripts/sim_ros2.py
~/isaacsim/python.sh isaacsim/scripts/sim_ros2.py --headless --duration 60          # --flight geometric, --init-pose q1..q6, --prop-spin on|off,
                                                                                    # --protective-stop on|measure, --reset-offset 0, --seed N
ros2 service call /sim/reset std_srvs/srv/Trigger                                   # 에피소드 리셋 (순간이동, 약 24 s: 팔 --init-pose 자세 (기본 홈), 그리퍼 열림, 새 PX4 로 드론 팔 위 호버 ± offset)
python3 isaacsim/scripts/replay_commands.py <commands.csv>                          # 명령 CSV 재생 (grasp_demo commands_<케이스>.csv 등)
python3 isaacsim/scripts/drone_cmd.py kill                                          # 드론 모터 정지 (잡은 뒤)
ros2 run rqt_image_view rqt_image_view                                              # 다른 터미널: 카메라 보기
python3 isaacsim/scripts/check_ros2.py                                              # 3-8 자동 검사 → 9/9 PASS (약 8 분, --part A|B)
ros2 topic hz --use-sim-time /joint_states                                          # sim 기준 주기 (기본은 wall 기준)

# 토픽 도착 시각·stamp 기록 (시스템 python3)
python3 isaacsim/scripts/topic_rate.py --out rate.json --duration 30 /cam/wrist/color/image_raw:sensor_msgs/msg/Image
```

### 4단계: 녹화·변환·병합·검수
```bash
source /opt/ros/jazzy/setup.bash

# 녹화 (sim): 터미널 1 = sim, 터미널 2 = 녹화 도구
~/isaacsim/python.sh isaacsim/scripts/sim_ros2.py                                   # PX4 드론, 리셋마다 드론 위치 ±3 cm (--drone-targets: 목록 25 점, --mode hover --hover mid: 사인파)
./6_record_bag.sh <작업명> --sim                                                    # r = 리셋 (약 24 s) 뒤 녹화 / 다시 r = 종료, k = 드론 모터 정지,
                                                                                    # d = 방금 에피소드 버리기 (→ bags/_discarded/), q = 종료
python3 isaacsim/scripts/replay_commands.py isaacsim/config/ros2_check/commands_success.csv   # (텔레옵 대신) 터미널 3: [REC ●] 뒤에 명령 재생

# 변환: bag 마다 stage1 (시스템 python3 + ROS). sim·텔레옵 = command, 고정 베이스 = const
python3 lerobot_stage1_extract_bag.py bags/<bag이름> --arm-action command --base-imu const   # → bag_lerobot_intermediate/<bag이름>/
#   --overwrite (출력 폴더가 있으면 다시), --max-age 0.066 (메시지 나이 한계 [s]), --skip-start 0.5 (녹화 시작 직후 누락된 bag 살리기)

# 병합·검수 (conda lerobot_v2, ROS 를 source 한 터미널이면 env -u PYTHONPATH)
source ~/miniconda3/etc/profile.d/conda.sh && conda activate lerobot_v2
env -u PYTHONPATH python lerobot_merge_episodes_v21.py --manifest merge_<이름>.txt --repo-id foodbanana/<이름>   # 또는 --all
#   --include-protective-stop, --trim-idle start|end|both [--trim-margin 0.5] [--idle-eps-deg 0.1]
env -u PYTHONPATH python inspect_dataset_v21.py lerobot_dataset_v21/foodbanana/<이름> --load                     # → PASS / FAIL
conda deactivate

# bag 하나만 데이터셋 하나로 (stage1 → stage2 를 이어서)
./convert_ros2bag_lerobot.sh bags/<bag이름> --arm-action command --base-imu const

# 파이프라인 자동 확인 (sim 을 직접 띄움: 가짜 에피소드 5 개 녹화 → 변환 → 병합 → 검수) → 3/3 PASS, 약 6 분
python3 isaacsim/scripts/make_fake_episodes.py                                      # --success 3 --pstop 1 --discard 1 [--use-running-sim]
```

### 5단계: 텔레오퍼레이션
```bash
# 5-A A-0 조종기 입력 확인 (sim 없이, 약 2 분. 안내에 따라 스틱·스위치를 움직인다) → isaacsim/reports/rc_probe_<시각>/
#   권한: /dev/ttyACM* 는 root:dialout → sudo usermod -aG dialout $USER (다시 로그인) 또는 sudo 로 실행
python3 isaacsim/scripts/rc_input.py                 # --port /dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_<번호>-if00
python3 isaacsim/scripts/rc_input.py --watch         # 값만 계속 보기

# 5-A 넘기기 시험 (PX4 드론만, 스틱은 스크립트가 만듦. 조종기·직렬 없이 돈다) → 6/6
~/isaacsim/python.sh isaacsim/scripts/check_rc.py --headless

# 5-A 실물 조종기로 sim 드론 조종 (계정이 dialout 그룹이어야 한다. 조종기를 켜고 스틱을 가운데에 두면 이륙·리셋 뒤 자동으로 넘어간다)
source /opt/ros/jazzy/setup.bash
~/isaacsim/python.sh isaacsim/scripts/sim_ros2.py --drone-rc on      # 설정 isaacsim/config/rc_input.yaml, 장치는 --rc-port 로 바꿈
#   [drone_rc] 줄: 넘김 / 스틱이 가운데가 아님 / 조종기 입력 없음 → 되돌림. 모터 정지는 녹화 도구 k (또는 python3 isaacsim/scripts/drone_cmd.py kill)

# 5-B 설치 (한 번): XRoboToolkit PC 서비스 (Ubuntu 24.04 deb) + Python 바인딩. 헤드셋 앱 XRoboToolkit-PICO-1.1.1.apk 는 헤드셋 브라우저로 설치
mkdir -p ~/xrobotoolkit && cd ~/xrobotoolkit
wget https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_24.04_amd64.deb
sha256sum XRoboToolkit_PC_Service_1.0.0_ubuntu_24.04_amd64.deb    # bce661f0be0b8a246ceecb2e5f1675a81c26b834648dc7fdf23f8c0bfe2a5d19
sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_24.04_amd64.deb
sudo ufw disable                                                  # 헤드셋 → PC TCP 63901
cd ~/data_collection_ur5_gripper
bash teleop/build_xr_binding.sh                                   # → ~/xrobotoolkit/venv (sudo 없이, conda·venv 를 끈 터미널에서)

# 5-B 쓸 때마다: PC 서비스 실행 → 헤드셋 앱에서 PC IP 입력 (WORKING) → 컨트롤러 선택 → Send
env -u LD_LIBRARY_PATH bash /opt/apps/roboticsservice/runService.sh   # 백그라운드. 종료: pkill -f RoboticsServiceProcess
ip -4 -o addr show | awk '{print $2, $4}'                             # 헤드셋 앱에 넣을 PC 주소 (지금 192.168.0.5)
ss -tn | grep 63901                                                   # ESTAB 이면 헤드셋이 연결됨

# 5-B B-0 컨트롤러 입력 확인 (sim 없이, 안내에 따라 움직인다) → teleop/reports/xr_probe_<시각>/
~/xrobotoolkit/venv/bin/python teleop/xr_probe.py                 # --skip-loss: 숨기기·내려놓기·헤드셋 벗기를 건너뜀

# 5-B B-1 URDF 기구학: 자기 점검 (numpy 만) / sim 과 대조 → 2/2 PASS
python3 teleop/ik_model.py
~/isaacsim/python.sh isaacsim/scripts/check_ik_model.py --headless

# 5-B B-2 텔레옵 노드 시험 (헤드셋 없이, 만든 입력. sim 을 먼저 띄운다) → 13/13
~/isaacsim/python.sh isaacsim/scripts/sim_ros2.py --headless --flight geometric --init-pose under_drone   # 터미널 1
python3 teleop/check_xr_teleop.py                                                                         # 터미널 2

# 5-B 텔레옵 (PC 서비스 실행, 헤드셋 앱 WORKING·Send 뒤). 터미널 1 = sim (아래), 터미널 2 = 노드, 터미널 3 = 영상, 터미널 4 = 녹화 도구 (노드를 띄운 뒤에. docs/pico_vr_data_collection.md 4 장)
~/xrobotoolkit/venv/bin/python teleop/xr_teleop_node.py           # 같은 처음 자세로 오른손 컨트롤러를 앞으로 들고 Y (왼손) = 시작 / 다시 Y = 멈춤, 트리거 = 그리퍼
./6_record_bag.sh <작업명> --sim                                  # 터미널 3. 컨트롤러: B 리셋 → Y 텔레옵 → A 녹화 → X 모터 정지 → A 녹화 끝 (녹화 중 B = 실패로 버리고 리셋). d·q 는 키보드
ros2 run rqt_image_view rqt_image_view /cam/wrist/color/image_raw # 손목 영상 (third view 는 창에서 토픽을 바꿈)

# 5-B 명령 ↔ 관절 지연·오차 (녹화한 bag, 시스템 python3 + ROS. sim·실물 공용) → teleop/reports/measure_cmd_tracking_<시각>.txt
python3 teleop/measure_cmd_tracking.py bags/<bag> [bags/<bag> ...]          # --max-lag 0.3, --moving-dps 2

# 5-B 텔레옵용 sim: 그리퍼가 위를 보고 드론 몸체·팔이 손목 카메라에 보이는 시작 자세. /sim/reset 도 이 자세로 되돌린다
~/isaacsim/python.sh isaacsim/scripts/sim_ros2.py --init-pose under_drone          # 조종기도 쓰면 --drone-rc on. 값: scene_drone.yaml init_poses.under_drone
```

### 6단계 이후

```bash
# 수집 전 확인: third view 카메라 위치 후보 비교 (카메라만 옮겨 렌더, intrinsics 는 third_view_camera.yaml) → isaacsim/reports/compare_third_view_<시각>/grid.png
~/isaacsim/python.sh isaacsim/scripts/compare_third_view.py --headless [--cand "Q2:1.1,-0.3,0.20,m" ...]   # 이름:x,y,z,보는 점 (base 기준 m, m = 시작 TCP 와 잡는 곳 가운데)
# third view 위치를 드론 기준 위치 목록 전체로 고르기 (계산으로 거른 뒤 상위만 렌더) → isaacsim/reports/compare_third_view_targets_<시각>/ (cands.csv, grid_start/grasp.png, cover_<후보>.png)
~/isaacsim/python.sh isaacsim/scripts/compare_third_view_targets.py --headless --start off --preset none   # 채택 조건 (2026-10-22). 엄격: --start on --preset big
#   --x/--y/--z 격자 (base m), --cand "Q2:1.1,-0.3,0.20", --box hull|body, --render 6 (0 = 계산만), --border-px 8, --path-m 0.25
# third view 받침대 CAD (레포 밖 venv ~/cad/venv, CadQuery. 카메라 자세는 third_view_camera.yaml 에서) → isaacsim/cad/out/third_view_mount.step·.stl·preview.png
~/cad/venv/bin/python isaacsim/cad/third_view_mount_cad.py [--screw-len 8]
# third view 카메라 모양 (D455) 다시 만들기 (realsense2_description 이 바뀌었을 때)
source /opt/ros/jazzy/setup.bash && ~/isaacsim/python.sh isaacsim/scripts/build_d455_usd.py   # → isaacsim/assets/cameras/d455/d455.usdc
# 드론 기준 위치 거르기 (팔 IK·한계·충돌·이륙 경로, third view 기본 = 설정 위치)
~/isaacsim/python.sh isaacsim/scripts/compare_drone_targets.py --headless --x 0.45 0.55 0.65 --y -0.2 -0.1 0 0.1 0.2 --z 0.70 0.80 --margin 0.08 0.08 0.05
```

