# Isaac Sim 드론 파지 데이터 수집 파이프라인 계획

작성일: 2026-09-29 · 0단계 확정 반영 (2026-09-29) · 1-3 import 확인 결과 반영 (2026-09-30) · 5단계 장치 결정 (2026-10-08: 드론 조종기 먼저, 팔은 SpaceMouse)

## 목표

Isaac Sim 6.1.0(standalone zip) 에서 UR5(CB3) + RH-P12-RN(A) 그리퍼 + 손목 D435i 로 공중의 드론을 파지하는
시연 데이터를 텔레오퍼레이션으로 수집하고, LeRobot v2.1 데이터셋으로 변환해 openpi π0.5 를 파인튜닝한다.

- 1차 목표: **고정 베이스**에서 드론 파지 (1~7단계)
- 2차 목표: **흔들리는 6DOF 베이스**에서 드론 파지 (8단계, VLA 또는 RL)
- 학습은 서버에서 수행

경로·브랜치 규칙은 `CLAUDE.md` 를 따른다. 이 문서의 sim 코드 경로는 모두 `~/data_collection_ur5_gripper/isaacsim/` 기준이다.

## 현재까지 된 것

- 실물 파이프라인 (`data_collection_ur5_gripper`): ros2 bag 녹화 → stage1(intermediate) → LeRobot v2.1 / v3.0 변환 → 검수
- openpi π0.5 파인튜닝 smoke test 완료
- Isaac Sim 6.1.0 ROS 2 브릿지: `/clock`, `/joint_states` 발행, `/joint_command` 구독으로 UR5 구동 확인
- sim bag(진짜 sim `/joint_states` + 가짜 그리퍼·카메라 노드, 모두 sim time) 이 **기존 stage1 을 코드 수정 없이 통과**
- 씬(UR5 단독 시험용): Ground Plane, Dome Light, EastRural_Table(collider 추가, 상판 z=0.762), UR5 를 테이블에 FixedJoint 로 고정
- 로봇 description: `isaacsim/ur5_rh_p12_description/` (UR5 + 샌드위치 마운트 + RH-P12-RN(A) + D435i 단일 URDF, xacro 빌드·기구학 검증 완료)
- 로봇 USD import 완료: `isaacsim/assets/robots/ur5_rh_p12_d435i/ur5_rh_p12_d435i.usda` (1-3 확인 결과 참고)
- 1·2단계 sim 구성 완료 (로봇 에셋·drive·파지·손목 카메라, 드론 씬·PX4 SITL 드론·스크립트 파지 데모). **단계별 실행 명령: 부록 D**

---

## 전체 원칙

1. **sim 은 실물과 같은 ROS 2 토픽 인터페이스를 흉내낸다.**
   녹화(`record_toggle.py`)와 변환(stage1 → v2.1)은 sim/실물이 공유한다. sim 에서 새로 만드는 것은 "토픽을 내보내는 쪽"뿐이다.
2. **텔레오퍼레이션 장치는 교체 가능한 부품이다.**
   어떤 장치든 `/joint_command`(팔 관절 목표) + `/gripper/command`(그리퍼 명령)만 내보낸다. SpaceMouse → GELLO / PICO 로 바꿔도 뒤쪽은 그대로.
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
| 텔레옵 인터페이스 | 모든 장치는 `/joint_command` + `/gripper/command` 만 발행 |

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
- **결정 (임시): cam_tilt 0 유지 (다운받은 마운트 그대로, 실물 시점과 일치)**. 기울인 마운트는 나중에 CAD 수정 때 다시 검토
  (그때는 CAD 치수로 `cam_tilt`·`cam_xyz` 를 xacro 에 넣고 재import → check_articulation·check_self_collision·check_wrist_camera)
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
- [ ] (실물 마운트 대기) `cam_tilt` 확정 — 지금은 0 (다운받은 마운트). 기울인 마운트 CAD 를 만들면 다시 결정, 실물 영상으로 좌우 방향 확인

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
- **차분 IK** (`isaacsim/scripts/arm_ik.py`, 5단계 SpaceMouse·PICO 에서도 씀): PhysX Jacobian + damped least squares
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
- 5단계 주의: 리셋 뒤 텔레옵 장치는 로봇의 지금 자세(홈)에서 시작해야 한다. 리셋 전 자세의 명령을 그대로 보내면 리셋 직후 팔이 그 자세로 33 ms 만에 뛴다

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

**목표**: 사람이 드론 파지 시연을 연속으로 녹화할 수 있음 (드론은 실물 조종기로, 팔은 SpaceMouse 로)

**결정 (2026-10-08 사용자)**
1. **드론 텔레옵을 먼저 한다 (5-A)**: 실물 Radiolink 조종기 → Raspberry Pi Pico → USB 직렬 → sim 드론. 사람이 실물 조종기로 sim 드론을 조종한다
2. **팔 텔레옵은 GELLO 대신 SpaceMouse (5-B)**: 3Dconnexion SpaceMouse Wireless. GELLO 는 대안으로 내린다 (아래 "대안")
- 순서: 5-A → 5-B. 둘은 서로 다른 통로다: 팔 텔레옵은 `/joint_command` + `/gripper/command` 만 (원칙 2 그대로), 드론 조종 입력은 따로

### 5-A. 드론 텔레옵: Radiolink 조종기 → sim (먼저)

구조도 (지금 드론 명령 경로·포트, 조종기를 넣은 구조, 녹화할 때 구조): **`docs/drone_teleop.md`**

**지금 되어 있는 것 (2026-10-08 사용자 확인)**
- 실물 조종기의 입력을 Raspberry Pi Pico 가 받아 USB 직렬 `/dev/ttyACM0` 로 한 줄씩 보낸다 (`sudo cat /dev/ttyACM0` 로 확인)
  ```text
  RC,0,0,1006,1002,995,996,200,200,200,1800
  ```
- 쉼표로 나눈 값 10 개 중 **3~6 번째 (위 예: 1006, 1002, 995, 996) 가 스틱 4 축** (2 축 스틱 2 개). 스틱을 움직이면 이 네 값이 바뀐다

**아직 모르는 것 → A-0 에서 확인** (아래 "추정" 은 확인 전)
| 항목 | 지금 아는 것 / 추정 |
|------|---------------------|
| 스틱 4 축이 각각 어느 값인지, 방향 (roll·pitch·throttle·yaw, 올리면 커지는지) | 모름 |
| 값 범위 (최소·중앙·최대) | 추정: 약 200 ~ 1800, 중앙 약 1000 (위 예에서 스틱 값이 1000 근처, 나머지가 200·1800) |
| 1·2 번째 값 (`0,0`) | 모름. 추정: 수신 상태 표시 (신호 끊김 등) |
| 7~10 번째 값 (`200,200,200,1800`) | 모름. 추정: 스위치·보조 채널 (200 / 1800 = 양 끝) |
| 한 줄이 오는 주기 [Hz] | 모름 |
| 조종기를 끄거나 신호가 끊기면 줄이 어떻게 되는지 | 모름 (끊김을 알 수 있어야 한다, 아래 안전 장치) |
| Pico 펌웨어 소스 | 레포에 없음. 넣을지 정할 것 (줄 형식이 바뀌면 받는 쪽이 조용히 틀린 값을 읽지 않게) |
| 권한 | 지금 사용자 계정이 `dialout` 그룹에 없어 `sudo` 로만 읽힌다 → 그룹에 추가하거나 udev 규칙 (노드를 sudo 로 띄우지 않는다) |

**정할 것**
| # | 항목 | 후보 | 메모 |
|---|------|------|------|
| 1 | 스틱 입력을 sim 드론에 넣는 방법 | **(가) PX4 에 조종기 입력으로 넣는다** (MAVLink `RC_CHANNELS_OVERRIDE` 또는 `MANUAL_CONTROL`) → PX4 의 수동 비행 모드 (Position 등) 가 스틱을 해석 / (나) 스틱 → 속도로 바꿔 Offboard 위치 목표를 조금씩 옮긴다 (`PX4Commander` 목표) | (가) 는 실물 조종과 같은 경로라 드론이 실물처럼 반응한다 (PX4 를 쓴 이유와 같음). 지금 sim 은 Offboard 모드로만 날리므로 모드 전환·PX4 파라미터 (조종기 입력 설정, 신호 끊김 failsafe) 를 확인해야 한다. **코드·PX4 소스 확인 (2026-10-08, `docs/drone_teleop.md`): 새 포트 없이 지금 명령 링크 (UDP 14540) 로 `MANUAL_CONTROL` 을 보낸다 (SITL 은 `COM_RC_IN_MODE 1` = 조이스틱 입력만). 넘겨받기는 `PX4Commander` 안에서 (밖에서 모드를 바꾸면 1 s 마다 Offboard 로 되돌린다).** (나) 는 지금 구조에 바로 붙지만 조종 느낌이 실물 PX4 와 다르다. **(가) 를 먼저 시도, 안 되면 (나)** |
| 2 | 직렬을 읽는 곳 | **(가) 별도 ROS 2 노드 (시스템 python3) 가 읽어 stamp 를 넣은 토픽으로 발행 → sim 이 구독** / (나) sim 프로세스가 직렬을 직접 읽음 | (가) 면 sim 코드가 장치를 모르고 조종기를 다른 것으로 바꾸기 쉽다. (나) 는 노드가 하나 적다. 조종 입력은 녹화하지 않으므로 (아래 결정) 어느 쪽이든 된다. 토픽 이름·타입은 A-1 에서 정함 (안: `sensor_msgs/Joy`) |
| 3 | 기하 제어기 드론 (`--flight geometric`) 도 조종기로 움직이게 할지 | PX4 만 / 둘 다 | 1 번을 (가) 로 하면 PX4 전용 |
| 4 | 드론 모터 정지 | 지금처럼 녹화 도구 `k` 키 / 조종기 스위치 (7~10 번째 값 중 하나) → `/sim/drone_kill` | 4단계에서 넘어온 항목. 스위치로 옮기면 조종하는 사람이 바로 끌 수 있다 |
| 5 | 누가 조종하나 | 두 사람 (드론 조종기 + 팔 SpaceMouse) / 한 사람 (드론을 세워 두고 팔만) | 조종기는 두 손을 쓴다 → 한 사람이 SpaceMouse 와 동시에 다루기 어렵다. 6단계 수집 방식에 영향 |

**작업 순서**
- A-0 입력 확인 (sim 없이): 위 "아직 모르는 것" 표를 채운다. 스틱을 하나씩 끝까지 움직이며 값 기록, 스위치마다 어느 값이 바뀌는지, 줄 주기, 조종기 끔·신호 끊김 때 출력
- A-1 직렬 → ROS 2 노드: 줄을 읽어 검사하고 (머리말 `RC`, 값 10 개, 범위) stamp 를 넣어 발행. **형식이 다른 줄·범위 밖 값·일정 시간 무수신은 에러 또는 "신호 없음" 으로 알린다** (조용한 fallback 금지). stamp = sim time (`use_sim_time`, `/joint_command` 와 같은 규칙)
- A-2 sim 연결: 정할 것 1 번 방식으로 `sim_ros2.py` 에. 설정 (축 배치·방향·범위·dead zone) 은 설정 파일 한 곳
- A-3 리셋·모터 정지: `/sim/reset` 은 지금처럼 자동으로 이륙·호버까지 하고 **그 뒤에 조종기가 넘겨받는다**. 녹화 도구·녹화 토픽은 바꾸지 않는다 (아래 결정)
- A-4 확인: 조종기로 드론을 팔 위 작업 영역 안에서 움직이고 세울 수 있는지, 스틱 → 드론 반응 지연, RTF, 회귀 (`check_ros2.py` 9/9, `check_px4.py` 5/5, `make_fake_episodes.py` 3/3: 조종기 없이도 지금처럼 돌아야 한다)

**안전 장치**
- 넘겨받을 때 스틱이 중앙 근처일 때만 (리셋 직후 드론이 튀지 않게. 팔 텔레옵의 "현재 자세에서 시작" 과 같은 규칙)
- 조종기 신호가 끊기면 드론은 그 자리에서 호버 (또는 PX4 failsafe 그대로) 하고 터미널에 알린다. 그런 에피소드는 사람이 `d` 로 버린다

**알아 둘 것**
- **드론 정보는 녹화하지 않는다 (2026-10-08 사용자 결정)**: 녹화하는 것은 지금처럼 로봇 팔·그리퍼·카메라뿐이다. 조종기 입력도, 에피소드 중 드론 위치도 bag 에 넣지 않는다
  → `record_toggle.py`·stage1·병합·검수·데이터셋 스키마 모두 그대로. 드론 조종 입력은 정책의 action 이 아니다 (정책은 팔·그리퍼만 움직인다)
  - 따라서 에피소드 중 드론이 어떻게 움직였는지는 카메라 영상에만 남는다 (bag 으로 드론 움직임을 다시 만들 수 없다)
- 2단계에서 미구현으로 둔 드론 `trajectory` 모드 (이동 드론) 를 사람의 조종이 대신할 수 있다 (6단계 난이도 순서의 "이동 드론")
- PX4 는 Offboard 위치 목표를 다듬지 않는다 (`docs/drone_flight.md` 14-10 C: `goto` 는 계단 입력). (나) 방식이면 목표를 조금씩 옮기는 것은 보내는 쪽 몫
- hover thrust 추정기가 모션캡처 설정에서 돌지 않아 $T_h$ = 0.5 다 (`docs/drone_flight.md` 14-10 B). 스로틀 스틱이 추력을 직접 정하는 수동 모드를 쓰면 스틱 중앙과 호버 추력이 어긋난다 → 모드를 정할 때 같이 볼 것
- 녹화 중에는 새 노드를 띄우지 않는다 (4단계 A1) → 직렬 노드를 따로 두면 녹화 전에 띄워 둔다

**완료 기준 (5-A)**
- [ ] A-0 표가 다 채워짐 (축 배치·방향·범위·주기·신호 끊김 때 출력)
- [ ] 실물 조종기로 sim 의 PX4 드론을 작업 영역 안에서 움직이고 세울 수 있음
- [ ] 조종기 신호가 끊기면 조용히 넘어가지 않고 드러남 (터미널 출력)
- [ ] 조종기 없이 도는 기존 검사가 그대로 통과 (`check_ros2.py`, `make_fake_episodes.py`)

### 5-B. 팔 텔레옵: SpaceMouse (5-A 다음)

- 장치: 3Dconnexion SpaceMouse Wireless (6 축 + 버튼). Linux 에서 읽는 방법 (spacenavd / hidapi 계열 라이브러리) 과 이 PC 에서 인식되는지는 확인할 것
- 6 축 입력 = TCP 속도 명령 (직선 3 + 회전 3) → TCP 목표 자세에 적분 → 차분 IK (타깃 프레임 `rh_p12_rn_tcp`) → `/joint_command` (50 Hz 이상, stamp)
- 버튼 → `/gripper/command` 0 / 1150 (열기 / 닫기). 버튼은 이진이라 히스테리시스가 필요 없고 데이터셋 action 은 지금처럼 `/gripper/target`
- **IK 를 어디서 계산하나 (정할 것)**: 2-4 의 `arm_ik.py` 는 PhysX Jacobian 을 써서 sim 프로세스 안에서만 돈다. 텔레옵 노드는 sim 밖의 ROS 2 노드이고 실물에도 그대로 써야 하므로
  **Jacobian 을 빌드한 URDF (`ur5_rh_p12_description/out/`) 에서 계산**해야 한다 (기구학 라이브러리를 쓸지 직접 계산할지. 시스템 python3 에 pinocchio 는 지금 없음).
  `arm_ik.py` 에서 가져올 것: damped least squares, 관절 목표 = 이전 목표 + Δq (측정값 + Δq 가 아님), 와인드업 방지 (목표가 측정값보다 0.05 rad 이상 앞서지 않게),
  한 주기 변화 ≤ 0.063 rad. URDF Jacobian 은 sim 의 FK (TCP 위치) 와 비교해 확인
- 정할 것: 움직이는 기준 좌표계 (로봇 base 기준 / TCP 기준 / 손목 카메라 화면 기준), 축별 최대 속도·dead zone, 특이점·관절 한계 근처에서의 동작 (멈춤 + 알림. 조용히 다른 자세로 가지 않게)

**안전 장치**
- 시작할 때와 `/sim/reset` 뒤에는 `/joint_states` 의 지금 자세에서 TCP 목표를 다시 잡는다 (튀는 동작 방지). 입력이 0 이면 목표가 그대로 = 팔이 멈춰 있음

**대안: GELLO / PICO**
- GELLO (처음 1순위였으나 2026-10-08 SpaceMouse 로 바꿈): GELLO 레포의 UR5 구성으로 하드웨어 제작, Dynamixel 관절 오프셋·부호 캘리브레이션, 관절 공간 그대로라 IK 불필요.
  트리거 연속값 → 열기/닫기 이진화 (히스테리시스, 예: 닫힘 > 0.6 / 열림 < 0.4) 는 그리퍼 드라이버 쪽에서, 이진화 전 원시값은 stamp 있는 토픽으로 bag 에만.
  시작 시 GELLO 자세와 로봇 자세가 허용 오차 안일 때만 추종
- PICO: SpaceMouse 와 같은 카테시안 명령 → IK 경로. 인터페이스는 동일

**텔레옵 노드 요구사항 (4단계에서 정해짐)**
- `/joint_command` 의 `header.stamp` 를 넣는다 (sim = sim time (`use_sim_time`), 실물 = PC 시각). stage1 이 이 stamp 로 팔 action 을 맞추고 0 이면 에러
- 50 Hz 이상으로 **일정하게** 보낸다. 데이터셋 (25 Hz) 보다 느리거나 자주 끊기면 같은 action 이 반복되는 계단이 된다
  (검수 `inspect_dataset_v21.py` 의 `/joint_command` 간격 통계로 확인)
- 녹화는 `./6_record_bag.sh <작업명> --sim`: `r` → 리셋 (약 24 s) → 녹화. **첫 명령 이전 구간은 stage1 이 뺀다** → 장치를 잡고 움직이기 시작하면 에피소드가 시작됨.
  녹화 전부터 명령을 계속 보내는 장치라면 녹화 시작 직후 구간이 그대로 들어가므로 4단계 남은 문제 A1 (녹화 시작 직후 메시지 누락) 에 걸릴 수 있다
- 드론 모터 정지는 녹화 도구의 `k` 키 (`/sim/drone_kill`). 텔레옵 장치 버튼으로 옮길지 여기서 정한다. 언제 끄는지를 에피소드마다 일관되게

**sim 에서 확인할 것 (3단계에서 넘어옴, `docs/sim_ros2_interface.md`)**
- **드론을 잡은 채 RTF 0.69** (3-8, PX4 모터 끔): 텔레옵으로 잡은 뒤 들고 옮길 때 조작감 확인. 불편하면 원인
  (드론·손가락 충돌 형상 convex 수, solver 반복)을 측정한 뒤 결정 (`docs/sim_performance.md` 6장 끝). PhysX 설정을 바꾸면 파지 회귀 시험 다시
- `/joint_command` 는 **일정한 주기**로 보낼 것 (sim 은 루프 30 Hz 마다 최신 명령을 33 ms 선형 보간, `docs/arm_command_interpolation.md`)
- **리셋(`/sim/reset`) 뒤 텔레옵은 로봇의 지금 자세(홈)에서 시작**: 리셋 전 자세의 명령을 보내면 팔이 33 ms 만에 그 자세로 뜀 (위 안전 장치와 같은 규칙)
- 보호 정지 기준 (접촉력 150 N, 위치 오차 5°, 관절 속도 200 °/s) 이 실제 텔레옵 동작에서 오탐 없는지 (3-7 은 명령 재생으로만 확인)
- 보간 시간 33 ms 를 실물 UR servoJ lookahead (실물 텔레옵 설정을 정할 때) 와 반응 지연이 비슷하도록 다시 맞출지

**완료 기준 (5-B, 5단계 전체)**
- [ ] SpaceMouse 로 고정 드론 파지 에피소드를 연속으로 녹화 가능
- [ ] 명령과 실제 관절 사이 지연·오차가 허용 범위
- [ ] 조종기로 움직이는 드론 (5-A) 을 SpaceMouse 로 잡는 에피소드를 녹화 가능 (누가 조종할지는 5-A 정할 것 5 번)

---

## 6단계: 데이터 수집 운영 (고정 베이스)

- 난이도 순서: 고정 드론(위치 랜덤) → 호버링 드론 → 이동 드론 (사람이 실물 조종기로 조종, 5-A)
- 랜덤화: 조명, 드론 초기 위치·자세, 텍스처
- 소량(수십 에피소드)으로 7단계 학습 루프를 먼저 한 번 돌려 문제를 찾은 뒤 대량 수집

**수집 전 확인** (다 모은 뒤 바꾸면 이미지 분포가 달라진다)
- [ ] **sim third view 카메라를 실물 구성에 맞춘다** (`isaacsim/config/third_view_camera.yaml`, 4단계 확정 계획·2-3 에서 넘어옴).
  지금은 임시: 모양 = D435i 메시, intrinsics = D456 가정값 (가로 약 90°, fx = fy = 320), `serial: null`
  - 먼저 실물 third view 카메라 모델 (지금 D456 252122301126, 두 번째 D435i 로 바꿀 수 있음) 과 놓을 위치를 정한다
  - `intrinsics`: 실물 `ros2 topic echo --once /cam/third_view/color/camera_info` 값으로 (손목 카메라 1-7 과 같은 방식), `serial` 도 채운다
  - `position`·`look_at`: 실물 카메라를 놓은 위치·방향 (로봇 base 기준으로 재서)
  - 모양 (`model`): D435i 면 그대로. D456 이면 메시가 없다 (`realsense2_description` 에 D455 까지만) → 대신할 모양을 정한다.
    third view 영상에는 자기 몸체가 안 나오므로 손목 카메라에 보일 때만 영향
  - 바꾼 뒤 `drone_scene.py --headless --check` (3/3), `check_ros2.py` (9/9)

---

## 7단계: 학습과 sim 평가

- openpi 설정: 7차원 state/action, 카메라 키 매핑(`wrist` → 손목 슬롯, `third_view` → base 슬롯), LoRA
- delta action: `DeltaActions(make_bool_mask(6, -1))` (관절 6개 delta, 그리퍼 절대값)
- norm stats: **자체 데이터로 계산** (π0.5 문서상 제공되는 사전학습 stats 없음). 계산 후 그리퍼 action 의 q01/q99 가 0 과 1 로 잡혔는지 확인 (닫힘 프레임이 1% 미만이면 q99=0 → 정규화 붕괴)
- 추론: 25 Hz, 그리퍼 출력 0.5 기준 threshold → 0/1150 goal
- 서버에서 학습
- **sim 폐루프 평가**: openpi 정책 서버 ↔ sim 클라이언트로 실제 파지 시도, 성공률 측정
- action chunk 실행 주기 등 추론 설정을 sim 에서 튜닝

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
| `/sim/reset` (std_srvs/Trigger, sim 전용) | 에피소드 리셋: 드론 kill → 순간이동 + PX4 재시작 → 팔·그리퍼 홈·열림 순간이동 → 재이륙 → 새 호버 위치 (약 24 s), 응답 JSON (시드·드론 위치·에피소드 메타데이터) |
| `/sim/drone_kill` (std_srvs/Trigger, sim 전용) | 드론 모터 정지 (잡은 뒤, 녹화 도구의 `k` 키). 다시 켜는 것은 `/sim/reset` |

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
# 1-3 import 는 Isaac Sim GUI (URDF importer, 1-3 의 옵션) → isaacsim/assets/robots/ur5_rh_p12_d435i/

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

# sim ROS 2 인터페이스 (실제 시간 속도). GUI: 메인 뷰포트 하나 + 프로펠러 회전 (손목·third view 는 rqt_image_view 등으로)
~/isaacsim/python.sh isaacsim/scripts/sim_ros2.py
~/isaacsim/python.sh isaacsim/scripts/sim_ros2.py --headless --duration 60          # --flight geometric, --init-pose q1..q6, --prop-spin on|off,
                                                                                    # --protective-stop on|measure, --reset-offset 0, --seed N
ros2 service call /sim/reset std_srvs/srv/Trigger                                   # 에피소드 리셋 (순간이동, 약 24 s: 팔 홈, 그리퍼 열림, 새 PX4 로 드론 팔 위 호버 ± offset)
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
~/isaacsim/python.sh isaacsim/scripts/sim_ros2.py                                   # PX4 드론, 리셋마다 드론 위치 ±5 cm
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

### 5단계 이후

(각 단계 작업 후 여기에 추가)

