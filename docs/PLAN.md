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
- **구조 검증 스크립트** `isaacsim/scripts/check_articulation.py` (`~/isaacsim/python.sh`, `--headless` 지원)
  - 테스트 씬: physics scene + ground plane + 로봇 USD reference (위 배치 규칙대로, 최상위 prim z = 0.762 m = 테이블 상판 높이.
    0 이면 관절 0 자세에서 TCP 가 z = −5 mm 라 팔이 바닥에 걸림). 1-5 drive 튜닝도 같은 씬을 쓴다
  - 로봇 USD 를 재import 할 때마다 실행하는 회귀 검사. 로봇 USD 원본은 수정하지 않음 (검사용 값은 실행 중 메모리에서만)
  - 검사 항목 (각각 PASS/FAIL, 리포트는 `isaacsim/reports/check_articulation_<날짜시간>.txt`)
    1. DOF 이름·순서·개수 (mimic 조인트의 DOF 포함 여부는 가정하지 않고 보고). 팔 6개 이름이 실물 드라이버와 동일
    2. 관절 한계 (rad): elbow ±π, 나머지 팔 ±2π, 그리퍼 4개 0~1.1351
    3. drive·mimic 설정 출력 (mimic 대상, 계수, 오프셋)
    4. root_joint 의 body0 = 최상위 prim, body1 = robot_mount
    5. `rh_p12_rn_tcp`, `wrist_camera_color_optical_frame` 존재와 world 좌표
    6. 링크별 질량·합계, 질량 0 이하나 관성 비정상 경고
    7. 3초 시뮬레이션 후 NaN·폭주 없음 (팔은 stiffness 0 이라 처져도 정상)
    8. `rh_r1_joint` 에만 임시 drive(stiffness 1000, damping 100, target 30°) → 2초 후 네 그리퍼 조인트 각도 차 1° 이내

### 1-5. drive 튜닝
- import 직후 값: 팔 stiffness/damping 0 (명령을 줘도 추종하지 않고 중력에 처짐), maxForce 팔 150/150/150/28/28/28, 그리퍼 1000
- 팔 6관절 stiffness/damping 튜닝 (USD 값은 degree 기준 단위임에 주의)
  - 방법: 관절마다 목표 각도를 갑자기 바꿔보고(스텝 입력), 얼마나 빨리 도달하는지, 지나치지 않는지(overshoot), 중력에 처지지 않는지를 재서 값을 정한다
  - 목표: 실물 UR5 처럼 명령을 빠르고 정확하게, 흔들림 없이 따라가는 것
- `rh_r1_joint`: stiffness/damping + max force 를 실물 전류 한계(400 mA)에 맞춰 낮춤 (현재 1000 은 실물보다 훨씬 셈)
- **mimic 조인트 3개(`rh_r2`, `rh_l1`, `rh_l2`)에는 drive stiffness 를 주지 않는다** (mimic 과 충돌). import 시 붙은 drive(maxForce 1000)는 stiffness 0 유지
- 설정은 USD 원본이 아니라 스크립트/씬 레이어에서 적용

### 1-6. 손가락 collider·마찰·파지 테스트
- 손가락(r2, l2) collider: convex hull 이 부정확하면 convex decomposition
- 고마찰 physics material 적용
- 파지 테스트: 작은 박스 + **드론 무게급(≈ 1.5 kg) 박스/봉**. 1.5 kg 을 지름 25 mm 봉 하나로 잡으면 무게중심이 봉에서 벗어날 때
  봉을 축으로 돌아갈 수 있음. 특히 그리퍼 max force 를 실물 수준으로 낮춘 뒤 확인

### 1-7. 손목 카메라
- `wrist_camera_color_optical_frame` 아래 Camera prim, X 축 180° 회전 (optical: +z 전방/+y 아래 ↔ USD camera: -z 전방/+y 위)
- `horizontalAperture=20.955`, `verticalAperture=15.716`, `focalLength ≈ fx*20.955/640 ≈ 20.14` (fx≈615 px, D435 color 640x480 전형값. 실물 `camera_info` 로 확인)
- `cam_tilt` 결정: README 기구학 계산상(윗면 바깥쪽 기준) tilt 0 이면 열린 손가락이 화면에 0% (닫힘 6~7%), 15~20° 면 닫힘 시 94~100%.
  손가락은 영상 아래쪽에서 들어옴

### 1-8. 저장
- 로봇 USD 원본(`ur5_rh_p12_d435i.usda`)은 import 결과 그대로 둔다. 튜닝값은 스크립트/씬 레이어로

**완료 기준**
- [ ] 구조 검증 스크립트(1-4) 8개 항목 PASS
- [ ] `/joint_command` 로 팔이 목표 자세에 안정적으로 도달 (떨림·폭주 없음)
- [x] 그리퍼 0 → 1.1351 rad 열림/닫힘 동작, 좌우 손가락 대칭 (GUI drive 로 확인. 스크립트 설정으로 재확인 필요)
- [ ] 테이블 위 작은 박스와 ≈ 1.5 kg 물체를 잡아 들어 올려도 미끄러지거나 튀거나 돌아가지 않음
- [ ] 손목 카메라: tilt 0 으로 sim 손목 영상 확인 → 부족하면 15~20° 비교 → **실물 마운트 출력 전에 `cam_tilt` 확정**

---

## 2단계: 씬과 드론

**목표**: 스크립트 한 번으로 재현되는 드론 파지 씬

**작업**
- 씬 구성을 GUI Action Graph 대신 **Python standalone 스크립트**로 작성 (씬 + ROS 2 OmniGraph 생성), `~/isaacsim/python.sh` 로 실행
- 테이블 + 로봇 배치(최상위 prim Transform, root_joint 유지), 조명, third view 카메라
- 베이스 모드 인자 자리 확보: `--base fixed` 만 구현 (나중에 `--base kinematic` 추가)
- 파라미터화된 간이 드론: 450급(1.2~1.5 kg), 본체 박스 + 암 4개 + 프롭 디스크 + **지름 약 25 mm 손잡이 봉**
  (그리퍼 최대 열림 ≈ 107 mm → 본체가 아니라 손잡이/암을 잡는다)
  - **손잡이는 드론 무게중심 바로 위**에 둔다 (봉을 축으로 드론이 돌아가는 것 방지, 1-6 참고)
- 드론 모드: `static`(공중 고정) → `hover`(중력 보상 + 약한 흔들림) → `trajectory`(이동)
- 파지 성공 판정: 그리퍼 닫힘 + 드론이 손가락 사이 + 로봇과 함께 이동

**결정 항목**
- [ ] **파지 후 중력 보상 유지 여부**: 유지하면 로봇이 드론 무게를 거의 느끼지 않고, 끄면 잡는 순간 1.5 kg 하중이 갑자기 걸림.
  실제 시나리오(프로펠러가 도는 드론인지, 정지한 드론인지)에 맞춰 결정

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
  - 새 로봇 prim 기준 경로: Publish Joint State `targetPrim` = articulation root prim, Articulation Controller `robotPath` = 로봇 최상위 prim
- 그리퍼 명령 토픽은 팔과 분리 (예: `/gripper_joint_command`), 대상은 `rh_r1_joint` 하나
- **sim 그리퍼 브리지 노드** (실물 `rh_gripper_node` 의 sim 버전)
  - 구독: `/gripper/command` (raw 0~1150, 열기/닫기)
  - 발행(30 Hz, 같은 tick·같은 stamp, sim time): `/gripper/joint_states`(present, raw), `/gripper/target`(실행된 goal, raw 0 또는 1150)
  - **단위 변환은 브리지에서**: present raw = `rh_r1_joint`(rad) × 1150 / 1.1351, 명령 rad = raw × 1.1351 / 1150
    (USD 속성을 직접 읽거나 쓰면 degree 이므로 추가 변환 필요)
  - 시작 시 열림(0) 초기화
  - 토픽 값은 실물과 같은 raw 스케일 유지. 0~1 정규화는 stage1 에서만 한다
- 카메라 color 발행: `/cam/wrist/color/image_raw`, `/cam/third_view/color/image_raw` (640x480, frame_id 포함)
- 베이스 IMU: 베이스(현재는 고정) 링크에 IMU prim 추가 → `/base/imu` (`sensor_msgs/Imu`, sim time) 발행.
  고정 베이스에서는 상수값이므로, IMU prim 이 번거로우면 stage1 에서 상수로 채우는 것으로 대체 가능 (둘 중 하나로 통일하고 meta 에 기록)
- 에피소드 리셋 서비스: 로봇 홈 자세, 드론 재배치(랜덤 시드 기록)

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
- 로봇 고정: 씬에서 **root_joint 의 body0 을 움직이는 베이스 강체로 바꾸는 방식**을 검토 (1-4 참고. 최상위 prim 은 강체가 아니므로 Transform 을 움직여도 물리적으로 따라오지 않음)
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
