# Isaac Sim 드론 파지 데이터 수집 파이프라인 계획

작성일: 2026-09-29 · 0단계 확정 반영 (2026-09-29) · 1-3 import 확인 결과 반영 (2026-09-30)

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

---

## 전체 원칙

1. **sim 은 실물과 같은 ROS 2 토픽 인터페이스를 흉내낸다.**
   녹화(`record_toggle.py`)와 변환(stage1 → v2.1)은 sim/실물이 공유한다. sim 에서 새로 만드는 것은 "토픽을 내보내는 쪽"뿐이다.
2. **텔레오퍼레이션 장치는 교체 가능한 부품이다.**
   어떤 장치든 `/joint_command`(팔 관절 목표) + `/gripper/command`(그리퍼 명령)만 내보낸다. GELLO → SpaceMouse / PICO 로 바꿔도 뒤쪽은 그대로.
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
  프로펠러 회전 관절 4 개 → FixedJoint, 저장된 초기 속도(rotor0 9.4 rad/s 등) 0, 물체 재질
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
- 2-4 스크립트 파지 데모(차분 IK, 아래에서 위로 접근) + 자동 판정 `approach → grasped → held → placed`, 실패 케이스(헛잡기·빗나감·떨어뜨림)도 판정 확인
- 2-5 0.8 kg 들기·이동·내려놓기 미끄러짐 기록

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
- [ ] 스크립트 실행만으로 씬이 뜸
- [ ] 드론 위치·모드를 인자로 변경 가능
- [ ] 파지 성공/실패가 자동 판정됨

---

## 3단계: sim ROS 2 인터페이스

**목표**: 실물과 같은 토픽이 sim time 으로 나옴

**작업**
- **먼저 real-time factor 측정**: 카메라 2대 렌더링(640x480) + 물리를 켠 상태에서 RTF 와 카메라 발행 주기 측정.
  25 Hz 이상 + 텔레옵 조작감이 실제 병목일 가능성이 크므로 3단계 초반에 확인
- `/clock`, `/joint_states` 발행, `/joint_command` 구독 (스크립트에서 생성)
  - 새 로봇 prim 기준 경로: Publish Joint State `targetPrim` = articulation root prim (**씬 override 후에는 로봇 최상위 prim**, USD 원본의 robot_mount 아님), Articulation Controller `robotPath` = 로봇 최상위 prim
- 그리퍼 명령 토픽은 팔과 분리 (예: `/gripper_joint_command`), 대상은 `rh_r1_joint` 하나
- **sim 그리퍼 브리지 노드** (실물 `rh_gripper_node` 의 sim 버전)
  - 속도는 **`robot_drive.GripperProfile`(목표값 이동, 1-5)을 그대로 쓴다**. 관절 속도 제한은 쓰지 않음
  - 구독: `/gripper/command` (raw 0~1150, 열기/닫기)
  - 발행(30 Hz, 같은 tick·같은 stamp, sim time): `/gripper/joint_states`(present, raw), `/gripper/target`(실행된 goal, raw 0 또는 1150)
  - **단위 변환은 브리지에서**: present raw = `rh_r1_joint`(rad) × 1150 / 1.1351, 명령 rad = raw × 1.1351 / 1150
    (USD 속성을 직접 읽거나 쓰면 degree 이므로 추가 변환 필요)
  - 시작 시 열림(0) 초기화
  - 토픽 값은 실물과 같은 raw 스케일 유지. 0~1 정규화는 stage1 에서만 한다
- 카메라 color 발행: `/cam/wrist/color/image_raw`, `/cam/third_view/color/image_raw` (640x480, frame_id 포함)
- 베이스 IMU: 베이스(현재는 고정) 링크에 IMU prim 추가 → `/base/imu` (`sensor_msgs/Imu`, sim time) 발행.
  고정 베이스에서는 상수값이므로, IMU prim 이 번거로우면 stage1 에서 상수로 채우는 것으로 대체 가능 (둘 중 하나로 통일하고 meta 에 기록)
- **보호 정지 흉내** (1-5 "알려진 위험: 높은 K" 1번): 관절 토크·접촉력 기준 초과 시 팔 목표를 현재 위치에 고정, 에피소드에 `protective_stop` 표시.
  기준값은 UR5 문서 확인 후 확정
- 에피소드 리셋 서비스: 로봇 홈 자세, 드론 재배치(랜덤 시드 기록)
  - **홈 자세 복귀는 목표를 한 번에 바꾸지 않고 부드러운 궤적으로 이동** (큰 스텝은 오버슈트와 손목 흔들림 유발, 1-5 A 시험)

**완료 기준**
- [ ] 카메라 렌더링 포함 real-time factor 측정·기록 (텔레오퍼레이션 조작감 기준)
- [ ] 모든 토픽 hz 가 목표에 맞음 (카메라 ≥ 25 Hz)
- [ ] 모든 header.stamp 가 sim time
- [ ] 드론을 잡았을 때 present 가 중간에서 멈추고 target 은 1150 (실물과 같은 파지 신호)

---

## 4단계: 녹화·변환 수정

**목표**: sim 에피소드 → 병합된 v2.1 데이터셋이 자동으로 나옴

**작업**
- `record_toggle.py`: `TOPICS` 에 `/joint_command` 추가, 녹화 시작 시 리셋 서비스 호출 연동
- stage1
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
- [ ] 스크립트로 움직인 가짜 에피소드 여러 개가 하나의 v2.1 데이터셋으로 병합됨
- [ ] `meta/info.json` 의 `codebase_version` = `v2.1`, 카메라 키 2개, `observation.base_imu` (6,) 포함
- [ ] 검수 통과: action 그리퍼 값 종류 = {0.0, 1.0}, state 그리퍼 0~1 이내, 이상치 0, 파지 신호(action=1 인데 state 가 중간에서 멈춤)

> 1~4단계는 텔레오퍼레이션 장치 없이 진행한다. 키보드나 스크립트로 `/joint_command` 를 보낸다.

---

## 5단계: 텔레오퍼레이션

**목표**: 사람이 드론 파지 시연을 연속으로 녹화할 수 있음

**GELLO (1순위)**
- GELLO 레포의 UR5 구성으로 하드웨어 제작
- Dynamixel 관절 오프셋·부호 캘리브레이션
- `/joint_command` 발행 ROS 2 노드 (50~100 Hz), 트리거 → `/gripper/command`
- 트리거 연속값 → 열기/닫기 이진화(히스테리시스, 예: 닫힘 > 0.6 / 열림 < 0.4)는 **그리퍼 드라이버** 쪽에서. 데이터셋 action 은 이진화 후 goal
- 이진화 전 트리거 원시값은 stamp 있는 토픽으로 bag 에만 녹화
- 관절 공간 그대로 → IK 불필요

**안전 장치**
- 시작 시 GELLO 자세와 sim 로봇 자세가 허용 오차 안일 때만 추종 시작 (튀는 동작 방지)

**대안: SpaceMouse / PICO**
- 카테시안 명령 → IK(차분 IK 등, 타깃 프레임 `rh_p12_rn_tcp`) → `/joint_command` 변환 노드 추가. 인터페이스는 동일

**완료 기준**
- [ ] 고정 드론 파지 에피소드를 연속으로 녹화 가능
- [ ] 명령과 실제 관절 사이 지연·오차가 허용 범위

---

## 6단계: 데이터 수집 운영 (고정 베이스)

- 난이도 순서: 고정 드론(위치 랜덤) → 호버링 드론 → 이동 드론
- 랜덤화: 조명, 드론 초기 위치·자세, 텍스처
- 소량(수십 에피소드)으로 7단계 학습 루프를 먼저 한 번 돌려 문제를 찾은 뒤 대량 수집

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
- **베이스 상태 기록 (bag → intermediate 는 상위집합, 데이터셋에서 선택)**
  - bag: 베이스 IMU(`sensor_msgs/Imu`, 수백 Hz). sim 은 Isaac IMU 센서로 같은 토픽 + ground-truth 베이스 pose/twist 추가
  - D435i 내장 IMU 는 **손목**의 움직임이라 베이스 IMU 를 대체하지 못함 (필요하면 별도로 녹화만)
  - stage1: `base_imu.npy` 는 4단계에서 이미 구현됨. (sim) ground-truth `base_pose.npy` 를 intermediate 에 추가 (데이터셋에는 넣지 않음)
  - 데이터셋: `observation.base_imu` 는 0단계에서 확정된 그대로 (고정 베이스 데이터와 열 구성이 이미 같으므로 병합 시 후처리 불필요)
  - 정책에 쓰려면 openpi repack 에 `observation.base_imu` 매핑 + Inputs transform 에서 state 에 이어붙이기 + **norm stats 재계산**. 추론 때도 같은 방식(25 Hz 구간 평균)으로 IMU 를 넣어야 함
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

| 토픽 | 방향 | 내용 |
|------|------|------|
| `/clock` | sim → ROS | sim time |
| `/joint_states` | sim → ROS | 팔 6관절 (+ 그리퍼 조인트) |
| `/joint_command` | teleop → sim | 팔 관절 목표 (**팔 action**) |
| `/gripper/command` | teleop → 브리지 | raw 0 또는 1150 (열기/닫기) |
| `/gripper/joint_states` | 브리지 → ROS | 그리퍼 present, raw → stage1 에서 /1150 (**state[6]**) |
| `/gripper/target` | 브리지 → ROS | 실행된 goal, raw 0/1150 → stage1 에서 /1150 (**action[6]**) |
| `/cam/wrist/color/image_raw` | 카메라 → ROS | `observation.images.wrist` |
| `/cam/third_view/color/image_raw` | 카메라 → ROS | `observation.images.third_view` |
| `/base/imu` | IMU → ROS | `sensor_msgs/Imu` → stage1 구간 평균 → `observation.base_imu` |

## 부록 C: Claude Code 사용 방법

- 이 파일은 레포의 `docs/PLAN.md`
- 0단계 확정값(스키마, 토픽 이름, 원칙)과 경로 규칙은 `CLAUDE.md`
- 작업은 단계 단위로 요청하고, 각 단계의 **완료 기준을 검증 항목으로 그대로 전달**한다
- sim 전용 코드는 `isaacsim/` 폴더, `isaacsim_v6.1.0` 브랜치에서 작업한다
