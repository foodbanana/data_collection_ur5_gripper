# 움직이는 베이스 씬 (Husky A200 / Stewart 플랫폼 위 UR5)

작성: 2026-10-28 · Stewart 추가 2026-10-29 · 수집 연결 (sim ROS 2·베이스 토픽) 2026-10-29 · 리셋·녹화·변환·PICO 시험 2026-10-30 · `docs/PLAN.md` 8단계, 브랜치 `isaacsim_v6.1.0`

UR5 를 움직이는 베이스 위에 올린 sim 씬. **고정 베이스 씬 (`drone_scene.py`, `sim_ros2.py`) 과 파일을 완전히 나눴다** (그쪽 파일·설정은 그대로).
베이스는 두 가지: **Husky A200** (직선 왕복) 와 **Stewart 플랫폼** (윗판 6 축 사인파, sim 전용, 8 장). 수집 연결은 `sim_ros2_moving_base.py` (9 장: 토픽·베이스 IMU 식, 10 장: 리셋·녹화·변환·PICO 시험).
둘 다 베이스를 kinematic 으로 정해진 대로 움직인다 (실물처럼 바퀴·액추에이터로 구동하는 버전은 나중, PLAN 8단계).

---

## 1. 한눈에 보기

| 항목 | 내용 |
|---|---|
| 씬 | 바닥 + Husky (kinematic 직선 왕복) + UR5 (윗판 가운데) + 손목 카메라 + 드론 (world 고정 호버, 기하 제어기) + third view (world 고정). 테이블 없음 |
| 실행 | `~/isaacsim/python.sh isaacsim/scripts/moving_base_scene.py` (GUI) / `--headless --duration S` |
| 검사 | `moving_base_scene.py --headless --check` → C1 ~ C6 **6/6** (약 1 분), `check_moving_base.py --headless` (Husky 단독) → **4/4** |
| 설정 | `isaacsim/config/scene_husky.yaml` (마운트·드론·시작 자세), `isaacsim/config/base_husky.yaml` (Husky 에셋·궤적) |
| 에셋 | `isaacsim/assets/bases/husky/` (Clearpath 공식 description, README) |
| Stewart | 8 장 (`--scene-config isaacsim/config/scene_stewart.yaml --base-motion small\|mid\|big\|lab`) |
| ROS 2 | 9 장 `sim_ros2_moving_base.py`: 고정 베이스와 같은 토픽 + `/base/imu`·`/base/odom`·`/base/joint_states` |
| 리셋·녹화·변환 | 10 장: `/sim/reset` (베이스·로봇 함께 순간이동, PX4 바닥 이륙), `record_toggle.py --base moving`, stage1 `--base-imu topic --base-pose odom`, `--base-motion static` (검사 22/22, 가짜 에피소드 4/4, PICO 시험) |
| 다음 | 수집 (디스크 확보 뒤), 정책 추론 연결, 실물처럼 구동하는 버전·운동 보상 (PLAN 8단계) |

---

## 2. 구조

```text
/World
 ├─ husky  (Husky A200 USD, kinematic 강체 묶음. 매 물리 스텝 최상위 prim 을 궤적 위치로 → PhysX kinematic target)
 │   └─ Geometry/base_link (차체) ─ 바퀴 4 (메시만 이동 거리 / 반지름 만큼 돔)
 │            ┊
 │            ┊ root_joint (고정 관절, articulation 밖 구속: excludeFromArticulation = true)
 │            ┊   body0 = Husky 차체, 위치 (0.0812, 0, 0.2515), yaw −90° = 윗판 가운데
 │            ▼
 ├─ ur5_rh_p12_d435i  (floating articulation, root = robot_mount) ─ 손목 카메라
 ├─ Drone  (Iris 0.75 배, 기하 제어기, world 고정 호버)
 └─ ThirdView  (D456 모양, world 고정, 받침대는 바닥부터)
```

- world 원점 = Husky 궤적 가운데에 있을 때의 로봇 base (xy). 로봇 정면 (+x) = 드론 쪽, Husky 는 world +y 를 보고 선다
- 로봇 base 높이 바닥 위 0.3838 m. 드론 = base 기준 (0.60, 0, 0.738) (고정 씬 `drone_pos` 와 같은 상대 위치). third view = 고정 씬 Q2 와 같은 base 기준 위치
- 궤적 (임시값): Husky 앞뒤 0.30 m 왕복, 최고 0.05 m/s, 가속 0.10 m/s², 양 끝 1 s 멈춤 → 로봇 base 는 드론 기준 y −0.15 ~ +0.15 m (한 주기 15 s)

---

## 3. UR5 를 Husky 에 붙이는 방법 (메모리 stage 만, USD 파일 그대로)

| # | 바꾼 것 | 없으면 |
|---|---|---|
| 1 | articulation root 를 `robot_mount` 에 둔다 (`ts.build_test_scene(base="floating")` = USD 원본 구조) | 최상위 prim 으로 옮기면 (고정 씬 방식) `root_joint` 가 articulation 첫 관절이 되어 world 에 박힘 |
| 2 | `root_joint` body0 = Husky 차체, localPos0/Rot0 = 마운트 위치·방향 | world 에 붙은 채 |
| 3 | `root_joint` 에 `physics:excludeFromArticulation = true` | USD Physics 해석기가 관절을 따라 **Husky 차체를 UR5 articulation 링크로 흡수** → 차체가 kinematic 이동을 안 받고 바닥에 내려앉음 (실측: robot_mount 높이 0.384 → 0.2515, y 고정). 근거: OpenUSD `pxr/usd/usdPhysics/parseUtils.cpp` `_TraverseHierarchy` (상대 body 가 강체이고 관절이 켜져 있으면 따라 들어감, kinematic 여부는 안 봄) |
| 4 | UR5 `robot_mount`·`base_link_inertia` ↔ Husky 강체 충돌 끔 (Husky 쪽 prim 에 `FilteredPairsAPI`) | UR5 바닥이 윗판 위 6.5 mm 라 접촉 거리 안에서 서로 밂. 로봇 쪽에 적으면 `ts.verify_articulation` 이 "충돌 제외 쌍이 설정과 다름" 으로 막음 |

결과: robot_mount ↔ 마운트 위치 0.004 mm·0.0000°, 팔 자세 오차 0.048° (가속·감속 포함).

### 알아 둘 것 (해 보고 찾음, 2026-10-28)
- **재생 중에 로봇 링크로 `RigidPrim` 을 새로 만들지 않는다**: PhysX 가 로봇을 다시 읽어 (`prim ... was deleted while being used by a shape in a tensor view`) 모든 tensor view 가 무효가 된다.
  floating 로봇의 root 는 `robot._physics_articulation_view.get_root_transforms()`, 다른 강체는 `SimulationManager._physics_sim_view__warp.create_rigid_body_view(...)` 로 읽는다
- **kinematic 바퀴 강체의 회전을 USD 로 바꾸지 않는다**: PhysX 가 그 충돌 형상을 지우고 다시 만들어 tensor view 가 무효. 바퀴 아래 보이는 메시 (`wheels.visual`, 물리 없음) 만 돌린다
- kinematic 강체를 USD 변환으로 옮기면 PhysX 가 속도를 안다 (0.2 m/s 로 옮기면 보고 속도 0.200 m/s). 최상위 prim 을 옮겨도 그 아래 강체가 모두 따라온다

---

## 4. 검사 `moving_base_scene.py --headless --check`

드론이 5 s 자리를 잡은 뒤 (Husky 정지, 시작 처짐 약 59 mm 는 판정 안 함) Husky 한 주기 동안:

| # | 항목 | 2026-10-28 결과 | 기준 |
|---|---|---|---|
| C1 | Husky **실제** 위치 (PhysX) = 궤적 | 0.000 mm | < 1 mm |
| C2 | UR5 robot_mount = Husky 마운트 위치 | 0.004 mm, 0.0000° | < 1 mm, 0.1° |
| C3 | 팔 시작 자세 유지 | 0.048°, 목표−측정 0.048°, 관절 속도 0.42 °/s | < 0.5°, 보호 정지 기준 5°·200 °/s |
| C4 | 팔 ↔ Husky 접촉 | 0 N | < 1 N |
| C5 | 드론 호버 (world 고정) | 6.8 mm | < 10 mm (`drone_scene --check` geometric 과 같음) |
| C6 | 손목·third view 영상 | 밝기 정상, 양 끝 사이 차이 손목 67 · third view 5 | 밝기 > 10, 차이 > 2 |

리포트·PNG: `isaacsim/reports/moving_base_husky_<시각>/` (git 제외). Husky 단독 `check_moving_base.py`: M1 궤적 계산, M2 위치 0.000 mm, M3 PhysX 속도 (차이 0.42 mm/s = a·dt/2), M4 바퀴 회전.

---

## 5. 임시값 (실물 Husky 측정 전)

| 항목 | 지금 | 바꾸는 곳 |
|---|---|---|
| UR5 마운트 | 윗판 가운데 (0.0812, 0, 0.2515), yaw −90° (판 위 높이·방향 = Clearpath 공식 Husky+UR5. 공식 위치는 윗판 앞쪽 (0.387, 0, 0.2515)) | `scene_husky.yaml` `mount` |
| 궤적 | 0.30 m 왕복, 0.05 m/s, 0.10 m/s², 멈춤 1 s | `base_husky.yaml` `trajectory` |
| Husky 구성 | 공식 기본 (윗판·범퍼·레일, 레이저·arch 없음) | `build_husky_urdf.sh` xacro 인자 → 다시 import |
| third view | world 고정 (나중에 world 고정 / Husky 위 기둥 결정) | `scene_husky.yaml` |

---

## 6. 명령

```bash
cd ~/data_collection_ur5_gripper
~/isaacsim/python.sh isaacsim/scripts/moving_base_scene.py                          # GUI (1 초마다 위치·오차 출력, 창을 닫으면 끝)
~/isaacsim/python.sh isaacsim/scripts/moving_base_scene.py --headless --check       # 판정 6 개
~/isaacsim/python.sh isaacsim/scripts/check_moving_base.py --headless               # Husky 단독 4 개

# Husky URDF 다시 만들기 (레포 밖 ~/husky_ws, 커밋 고정) → GUI import (isaacsim/assets/bases/husky/README.md)
source /opt/ros/jazzy/setup.bash && isaacsim/scripts/build_husky_urdf.sh
```

## 7. 관련 문서

| 문서 | 내용 |
|---|---|
| `docs/PLAN.md` 8단계 | 진행 기록, 다음 계획 (Stewart, `sim_ros2_moving_base.py`, `/base/imu`, 잡기 난이도·운동 보상) |
| `isaacsim/assets/bases/husky/README.md` | Husky 에셋 출처·구조·다시 만드는 법 |
| `docs/architecture.md` | 전체 수집 구조 (고정 베이스) 와 이 씬의 관계 |

---

## 8. Stewart 플랫폼 (2026-10-29)

실물이 없는 sim 전용 플랫폼. 윗판 (kinematic 강체) 을 6 축 사인파로 움직이고 UR5 를 윗판 윗면 가운데에 붙인다 (Husky 와 같은 방식 A).

### 8.1 구조 (`assets/bases/stewart/stewart.usda`, `build_stewart_usd.py`)
```text
 z [m]
 1.74 ┤                         ✕ 드론 (world 고정, 로봇 base 기준 앞 0.60 · 위 0.738)
 1.00 ┤   ═══════╪★═╪═══════   윗판 r 0.7 (kinematic 강체)  ★ 회전 중심 = 윗판 윗면 가운데 = UR5 robot_mount
 0.95 ┤   ●  ●     ◆     ●  ●  ◆ imu_link (윗판 아랫면 가운데)  ● 윗판 부착점 r 0.6
      │    \  \       /  /     다리 6 개 = 실린더 + 로드 (보이기만), 길이 0.747 ~ 1.395 m (가운데 1.067)
 0.05 ┤   ■  ■         ■  ■    ■ 아랫판 부착점 r 0.8 (Ingram ik.cpp 각도, 짝 ±7.2°)
 0.00 ┼═══════════════════════ 아랫판 r 0.9 (정적 collider)
```
크기는 `design_stewart.py` (numpy, 몇 초) 로 정함: 수집 프리셋 big 이 다리 범위 안 (신축 비율 1.87), 조건수 ≤ 3.3, 다리 간격 ≥ 15 cm.

### 8.2 움직임 (`moving_base.SineTrajectory`·`StewartBase`, 설정 `config/base_stewart.yaml`)
```text
q_k(t) = e(t) · s · A_k · sin(ω_k t + φ_k)        k = x, y, z, roll, pitch, yaw
  e(t) = 10u³ − 15u⁴ + 6u⁵, u = t / 5 s            시작 램프 (위치·속도·가속도가 0 에서 이어짐)
  ω_k = Ω_k × U(0.7, 1.3),  φ_k ~ U(0, 2π)         seed 로 (같은 seed = 같은 움직임)
윗판 자세: 위치 (x, y, 1.0 + z), 회전 Rz(yaw)·Ry(pitch)·Rx(roll)
매 물리 스텝: 윗판 USD translate + orient (→ PhysX kinematic target, 속도·각속도를 앎), 다리 IK L_i = ‖p + R·P_i − B_i‖ 로 메시 갱신·범위 검사
```
| 프리셋 | 진폭 x, y, z [m] · roll, pitch, yaw [°] | 쓰임 |
|---|---|---|
| `small` | big × 0.25 | **수집 주력** |
| `mid` | big × 0.5 | 수집 일부 |
| `big` | 0.15, 0.10, 0.15 · 6, 6, 3 (연구실 값에서 x·z·pitch 절반) | 지금 수집 안 함 (시작 자세 충돌 위험) |
| `lab` | 0.30, 0.10, 0.30 · 6, 12, 3 (연구실 이전 연구 값, Ω 0.9·0.5·0.9·0.5·0.9·0.2) | 비교용, 다리 범위 넘으면 경고만 |

실물 Stewart (참고 레포) 는 자세 → IK → 다리 6 개 위치 제어 → 물리로 윗판이 움직이지만, 여기서는 윗판 자세를 직접 넣고 IK 는 다리 그리기·검사에만 쓴다 (윗판 오차 0, 액추에이터 지연 없음).

### 8.3 검사
| 명령 | 결과 (2026-10-29, seed 0) |
|---|---|
| `check_moving_base.py --headless --base-config isaacsim/config/base_stewart.yaml --base-motion <프리셋>` | small·mid·big·lab **4/4** (윗판 자세 0.000 mm, 선속도 ≤ 1.5 mm/s·각속도 ≤ 0.06 °/s, 다리 끝 = 부착점) |
| `moving_base_scene.py --headless --check --scene-config isaacsim/config/scene_stewart.yaml --base-motion <프리셋>` | small·mid·big **6/6** (big: 기울기 8.5°, robot_mount ↔ 마운트 0.005 mm·0.02°, 팔 0.047°). lab 5/6 (base +0.17 m 에서 시작 자세 팔이 드론을 밈) |
| `python3 isaacsim/scripts/reach_moving_base.py --scene-config isaacsim/config/scene_stewart.yaml --base-motion small mid big` | 아래 표 |

| 프리셋 | 닿는 시간 | 잡기 창 (0.6 s 동안 3 cm 안) | 시작 자세 충돌 여유 |
|---|---|---|---|
| small | 100 % | 94 % | 12.4 cm |
| mid | 100 % | 26 % | 7.1 cm |
| big | 94 % | 4.5 % | −3.3 cm |

`reach_moving_base.py` 는 기구학만 (자기 충돌·시야·드론 hover 는 안 봄). 충돌 여유는 sim 결과 (lab 부딪힘 5.0 ~ 5.3 s, big 기본 위치 안 닿음) 로 맞춤.

### 8.4 알아 둘 것 (해 보고 찾음)
- 윗판처럼 **부모가 강체가 아닌** kinematic 강체는 USD translate + orient 로 돌려도 tensor view 가 유지된다 (Husky 바퀴처럼 강체 안의 강체를 돌리면 무효)
- **PhysX 강체 선속도는 질량 중심 속도**: 윗판 COM 은 회전 중심 2.5 cm 아래라 회전 중심 속도와 ω × r_com 만큼 다르다 (검사는 그 점에서 비교)
- 램프가 3u² − 2u³ 이면 시작·끝에서 가속도가 뛰어 IMU 가 계단처럼 튄다 → 5 차 램프
- 고정 베이스에서 튜닝한 팔 게인은 big (기울기 8.5°) 에서도 다시 튜닝할 필요 없음 (팔 0.047°)

### 8.5 명령
```bash
python3 isaacsim/scripts/design_stewart.py --config isaacsim/config/base_stewart.yaml             # 크기 확인
~/isaacsim/python.sh isaacsim/scripts/build_stewart_usd.py --config isaacsim/config/base_stewart.yaml   # 에셋 다시 빌드
~/isaacsim/python.sh isaacsim/scripts/moving_base_scene.py \
    --scene-config isaacsim/config/scene_stewart.yaml \
    --base-motion small                                                                          # GUI
```


---

## 9. 수집 연결: sim ROS 2 (2026-10-29, PLAN 8단계 R2·R3)

고정 베이스 `sim_ros2.py` 와 **같은 토픽 + 베이스 토픽 3 개**를 내는 별도 실행 파일. 고정 베이스 파일 (`sim_ros2.py`·`drone_scene.py`·`sim_reset.py`) 은 고치지 않고
부품 (`ros2_iface` 브리지, `protective_stop`, `sim_reset`) 을 import·상속해서 쓴다. 리셋·녹화·변환은 10 장.

| 파일 | 내용 |
|---|---|
| `isaacsim/scripts/sim_ros2_moving_base.py` | 실행 (씬 + 브리지 + 루프). `--base husky\|stewart`, `--base-motion` (Stewart 필수), `--imu-compare` (검사용) |
| `isaacsim/scripts/moving_base_ros2.py` | 부품: `MovingProtectiveStop`, `BaseBridge`, `add_isaac_imu` |
| `isaacsim/scripts/moving_base.py` | `kinematics(t)`·`joint_states(t)` (두 베이스 공통 이름) |
| `isaacsim/config/ros2_iface.yaml` `base` | 베이스 토픽 이름·주기·frame (고정 베이스 sim 은 이 섹션을 읽지 않음) |
| `isaacsim/scripts/check_ros2_moving_base.py` | 자동 검사 (아래 9.5) |

### 9.1 베이스 토픽

| 토픽 | 타입 | 주기 | frame | 내용 |
|---|---|---|---|---|
| `/base/imu` | sensor_msgs/Imu | 120 Hz (물리 스텝마다) | `imu_link` | 각속도 [rad/s]·비력 [m/s²] (IMU 좌표계), 자세 (world 기준). 잡음 없음, 공분산 0 (= 모름) |
| `/base/odom` | nav_msgs/Odometry | 30 Hz | `odom` → `base_link` / `top_plate` | 시작 자세 기준 위치·자세, twist 는 child 좌표계 |
| `/base/joint_states` | sensor_msgs/JointState | 30 Hz | | Husky 바퀴 4 개 회전각 [rad] / Stewart 다리 6 개 길이 [m] |
| `/base/imu_isaac` | sensor_msgs/Imu | 120 Hz | `imu_link` | `--imu-compare` 일 때만. Isaac IMU 센서 값 (검사용, 녹화 안 함) |

- stamp = sim time. 30 Hz 토픽은 sim time 의 1/30 s 배수 시각 (카메라·그리퍼와 같은 물리 스텝)
- **모두 정답값**: 베이스가 궤적 식대로 kinematic 으로 움직이므로, BaseBridge 가 **같은 식·같은 시각**으로 계산한다.
  실물처럼 구동하는 버전 (Husky `/cmd_vel`·바퀴 물리, Stewart 모터) 에서는 궤적 식으로 계산할 수 없으므로 측정값 (Isaac IMU 센서·바퀴 odometry·다리 FK) 으로 바꾼다 (PLAN 8단계)
- **수집 때는 `/base/imu` (궤적 계산값) 를 녹화** → stage1 이 25 Hz 구간 평균으로 `observation.base_imu`. `/base/odom`·`/base/joint_states` 는 bag·중간 파일에만
- IMU 잡음: 처음엔 없음. 실물 Xsens (나중에 실물 Stewart 를 쓸 때 윗판 IMU) 로 잡음을 재면 옵션으로 추가. 실제 배 데이터는 배 IMU 만 받을 예정 (Xsens 아님)

### 9.2 IMU 위치 (메시 없음)

| 베이스 | prim | 위치 (베이스 강체 기준) |
|---|---|---|
| Husky | `/World/husky/Geometry/base_link/imu_link` | (0, 0, 0) = 차체 원점, 바닥 위 0.132 m. 공식 URDF 에서 들어온 것 (실물 위치는 Husky 를 고친 뒤 확인) |
| Stewart | `/World/stewart/top_plate/imu_link` | (0, 0, −0.05) = 윗판 아랫면 가운데 (배의 갑판 아래 IMU 와 같은 구조). `build_stewart_usd.py` 가 넣음 |

- 둘 다 빈 `Xform` (메시·충돌·질량 없음). 방향은 베이스 강체와 같아야 함 (다르면 에러, `imu_offset`)
- BaseBridge 는 시작할 때 이 위치 r 만 한 번 읽고, 값은 궤적 식으로 계산한다 (prim 을 매 스텝 읽지 않음)
- `--imu-compare` 일 때만 `imu_link/isaac_imu` (`IsaacImuSensor`, 필터 없음) 가 메모리 stage 에 생긴다

### 9.3 BaseBridge 식 (`moving_base_ros2.BaseBridge` + `moving_base.kinematics`·`joint_states`)

기호: world = 바닥 고정 좌표계 (z 위). 베이스 강체 = Husky `base_link` / Stewart 윗판 (원점 = 회전 중심 = 윗판 윗면 가운데).
t = 베이스가 움직이기 시작한 뒤의 sim 시각 (이번 물리 스텝이 끝나는 시각 = PhysX 에 넣은 kinematic 자세의 시각).

**① 궤적 → 베이스 강체 운동** (`kinematics(t)`, 모두 world): 위치 p, 선속도 v = ṗ, 선가속도 a = p̈, 회전 R, 각속도 ω, 각가속도 α = ω̇

Husky (직선 왕복, 회전 없음):
```text
s, ṡ, s̈ = 사다리꼴 궤적 (가속 → 등속 → 감속 → 멈춤 → 반대로, LinearTrajectory)
p = p0 + s·d      v = ṡ·d      a = s̈·d          (d = 진행 방향 단위벡터, world)
R = Rz(yaw0)      ω = 0        α = 0
```

Stewart (6 축 사인파, 8.2):
```text
q_k(t) = e(t) · A_k · sin(ω_k t + φ_k),   e(t) = 10u³ − 15u⁴ + 6u⁵,  u = clamp(t / 5 s, 0, 1)
q̇_k, q̈_k = 곱의 미분 (식으로 정확히)
p = p0 + R0 · (q_x, q_y, H + q_z)              (H = 윗판 높이 1.0 m, R0 = 플랫폼 놓인 방향)
v = R0 · (q̇_x, q̇_y, q̇_z)      a = R0 · (q̈_x, q̈_y, q̈_z)
R = R0 · Rz(yaw) · Ry(pitch) · Rx(roll)          (ZYX 오일러)

ω_body = [ ṙ − ẏ·sin p ,                         (r = roll, p = pitch, ẏ = yaw 변화율)
           ṗ·cos r + ẏ·sin r·cos p ,
          −ṗ·sin r + ẏ·cos r·cos p ]
ω = R · ω_body
α(t) ≈ ( ω(t + h) − ω(t − h) ) / 2h,  h = 1e-4 s  (궤적이 식이라 오차 ~1e-8)
```
오일러 각 변화율 (ṙ, ṗ, ẏ) 은 각속도가 아니다. 윗판 좌표 각속도로 바꾸는 위 식이 필요하다.

**② 강체 운동 → IMU 자리 운동** (IMU 는 강체 원점에서 r 만큼 떨어짐: Husky r = 0, Stewart r = (0, 0, −0.05)):
```text
r_w   = R · r
a_imu = a + α × r_w + ω × (ω × r_w)       (접선 가속도 + 구심 가속도)
```

**③ IMU 출력** (`/base/imu`):
```text
비력 (가속도계)  f     = Rᵀ · (a_imu − g),   g = (0, 0, −9.81)  (씬 PhysicsScene 에서 읽음)
각속도 (자이로)  ω_imu = Rᵀ · ω
자세             R → 쿼터니언 (w, x, y, z), world 기준
```
- 가속도계는 운동 가속도가 아니라 **비력**을 잰다: 정지 f = Rᵀ·(0, 0, +9.81) (수평이면 (0, 0, 9.81)), 자유낙하 f = 0. 기울면 중력이 x·y 축에도 나뉨 (실제 IMU 와 같음)
- 베이스가 서 있을 때 (start 전·stop 뒤): 그 자리 자세, v = a = ω = α = 0 → f = Rᵀ·(0, 0, 9.81), ω_imu = 0

**④ `/base/odom`** (원점 = 베이스가 처음 선 자세 (p₀, R₀), 실물 odometry 처럼 시작이 0):
```text
위치 = R₀ᵀ · (p − p₀)      자세 = R₀ᵀ · R
선속도 = Rᵀ · v            각속도 = Rᵀ · ω          (ROS 규칙: twist 는 child 좌표계)
```
Stewart 위치는 윗판 회전 중심 (= 로봇 base 자리).

**⑤ `/base/joint_states`**:
```text
Husky    θ = s / r_wheel,  θ̇ = ṡ / r_wheel          (r_wheel = 0.1651 m, 네 바퀴 같음, 미끄러짐 없음)
Stewart  L_i = | p_top + R_top · P_i − B_i |          (P_i 윗판 부착점, B_i 아랫판 부착점, 최상위 prim 좌표)
         L̇_i ≈ ( L_i(t + h) − L_i(t − h) ) / 2h
```

### 9.4 `--imu-compare`: 두 방법 비교

| | 방법 2 (평소 발행·녹화) | 방법 1 (`--imu-compare` 일 때만) |
|---|---|---|
| 토픽 | `/base/imu` | `/base/imu_isaac` |
| 만드는 곳 | `BaseBridge` (9.3 식) | Isaac Sim 내장 IMU 센서 (`isaacsim.sensors.experimental.physics.IMUSensor`) |
| 계산 | 궤적 식을 미분 | PhysX 가 실제로 움직인 베이스 강체의 속도 변화 (스텝 차분) |

- 방법 2 는 직접 짠 식이라 좌표계·중력 부호·r 항 (접선·구심) 실수 가능성이 있다. 방법 1 은 그 식과 독립이라, **같은 시각에 같은 값이면 식이 맞다**
- 평소에 방법 1 을 쓰지 않는 이유: 스텝 차분이라 가속도가 갑자기 바뀌는 순간 한 스텝 (8.3 ms) 늦음 (Husky 사다리꼴 계단에서 그 한 샘플만 0.1 m/s² 차이),
  접촉이 걸리면 PhysX 보고 속도가 실제와 다를 수 있음 (드론에서 확인, CLAUDE.md), 센서 계산이 더 듦
- Isaac 센서 읽은 값 (C++ `ImuSensorReading`) 의 자세는 `orientation_w/x/y/z`, 시각 `time` 은 float32 정밀도 (소수 6 자리가 가끔 다름 → 검사는 가장 가까운 stamp 0.1 ms 이내로 짝지음)

### 9.5 검사 `check_ros2_moving_base.py` (sim 을 `--imu-compare` 로 띄움, 베이스마다 약 2 분)

| 번호 | 내용 | 기준 |
|---|---|---|
| 1 | 토픽 주기 (베이스 움직이는 중): `/joint_states`·`/clock` 120, 카메라·그리퍼 30, `/protective_stop` 약 30 Hz | ±1 %, 끊김 0 |
| 2 | 같은 시계: 카메라·그리퍼 stamp ∈ `/joint_states` stamp | 안 맞음 0 |
| 4 | 팔 명령 응답: 시작 자세에서 shoulder_pan 계단 0.03 rad | 50 % ≤ 80 ms, 보호 정지 없음 |
| 5 | 베이스 토픽 주기 (imu 120, odom·joint_states 30 Hz) + stamp ∈ `/joint_states` stamp | ±1 %, 끊김·안 맞음 0 |
| 6 | IMU 두 방법 일치 (같은 stamp) | 각속도 차 ≤ 0.005 rad/s, 비력 차 RMS ≤ 0.02·최대 ≤ 0.15 m/s², 자세 차 ≤ 0.1°, 짝 ≥ 95 % (기준은 처음 잰 값 보고 정함) |
| 7 | IMU 물리 확인: 비력 z 평균 ≈ \|g\|. Husky 는 각속도 0, 등속·정지 구간 비력 = (0, 0, \|g\|) | ±0.05 / ±0.01 m/s² |
| 8 | odom 일관성: 위치 차분 ≈ twist (child → odom 으로 돌려서), frame 하나, joint_states 개수 (4 / 6) | ≤ 2 mm/s |

**결과 (2026-10-29) 14/14**:

| 항목 | Husky | Stewart small |
|---|---|---|
| 5 주기 | 120.00 / 30.00 / 30.00 Hz | 같음 |
| 6 두 방법 | 비력 차 RMS 0.0073·최대 0.100 (가속도 계단), 각속도 0 | 비력 차 RMS 0.0039·최대 0.0077, 각속도 차 0.00013 rad/s, 자세 차 0.014° |
| 7 물리 | 각속도 0, 등속·정지 1081/1201 샘플 (0, 0, 9.81) | 비력 z 평균 9.8094 |
| 8 odom | 0.21 mm/s | 0.011 mm/s |
| 4 팔 계단 | 50 ms | 50 ms |
| RTF (헤드리스, 참고) | 0.89 | 0.81 |

고정 베이스 회귀 `check_ros2.py` 9/9 (공용 `ros2_iface.yaml` 에 `base` 섹션만 추가).

### 9.6 알아 둘 것 (해 보고 찾음)
- **보호 정지 접촉 view**: floating 로봇은 재생 중에 로봇 링크로 `RigidPrim` 을 만들면 PhysX 가 로봇을 다시 읽어 view 가 무효 → `MovingProtectiveStop` 이 sim view `create_rigid_contact_view` 로 직접 만듦
  (`protective_stop.py` 는 `_body_contact_view` 함수로만 나눔, 고정 베이스 동작 그대로)
- 보호 정지 대상 (손가락이 닿으면 안 되는 환경) 은 이 씬에 맞게 메모리에서만 바꿈: 바닥·third view 받침대·베이스 충돌체 (테이블 없음)
- Stewart RTF 0.81 (headless) / 0.61 ~ 0.64 (GUI, 10.5) 은 물리 스텝마다 다리 12 개 USD op 를 갱신하는 영향으로 보임 (재지 않음. 필요하면 렌더 주기로 줄일 수 있음)

### 9.7 명령
```bash
source /opt/ros/jazzy/setup.bash
~/isaacsim/python.sh isaacsim/scripts/sim_ros2_moving_base.py --base husky                       # GUI
~/isaacsim/python.sh isaacsim/scripts/sim_ros2_moving_base.py --base stewart --base-motion small
ros2 topic echo /base/imu --once                                                                   # 다른 터미널
python3 isaacsim/scripts/check_ros2_moving_base.py                                                 # 검사 (Husky + Stewart small)
python3 isaacsim/scripts/check_ros2_moving_base.py --base stewart --base-motion mid
```

---

## 10. 리셋·녹화·변환 (2026-10-29 ~ 30, PLAN 8단계 R4·R5·R6)

### 10.1 리셋 `/sim/reset` (`moving_base_ros2.MovingEpisodeReset`, `sim_reset.EpisodeReset` 상속, 약 24 s)

| 순서 | 하는 일 | 고정 베이스와 다른 점 |
|---|---|---|
| 1 | 팔·그리퍼 명령 무시, 보호 정지 해제 | 같음 |
| 2 | 드론 kill → 이륙 지점으로 순간이동 → PX4 재시작 | 이륙 지점 = **바닥** (고정 씬 `px4.takeoff_xy` (1.2, 0.15), 높이 = 씬 `px4_surface_z` 0 + 0.06) |
| 3 | **베이스 멈춤 → 궤적 시작 자세 (t = 0) 로 순간이동 → 새 seed (리셋 seed + 2 000 000)** | 새로 |
| 4 | **로봇 root (`robot_mount`) 를 시작 자세로 순간이동 (속도 0)** → 마운트 거리 확인 (2 mm 넘으면 리셋 실패) | 새로 (articulation view `set_root_transforms`) |
| 5 | 팔 시작 자세 (`--init-pose`, 기본 `under_drone`), 그리퍼 열림 | 같음 |
| 6 | PX4 이륙 → 드론 기준 위치 | 기준 = **베이스 움직임 가운데의 로봇 base** + 씬 `drone_base` (`--drone-targets` 면 고정 씬 목록을 같은 기준으로) |
| 7 | **베이스 움직임 시작** | 새로. 흔들림은 리셋 끝부터 (= 녹화 구간에만, 사용자 결정). sim 을 띄운 직후에는 서 있음 |

- 응답 JSON: 고정 베이스와 같은 키 + `base` (이번 에피소드: seed, 궤적 값 (Stewart ω·위상), 마운트 거리, 시작 로봇 base) + `sim.base` (종류·프리셋·마운트·IMU 위치·잡음 없음)
- 위치 메타데이터 (`drone_goal_base` 등) 의 기준 = **에피소드 시작 (궤적 t = 0) 의 로봇 base** (`robot.base_world`). Husky 는 왕복 한쪽 끝, Stewart 는 가운데
- 리셋이 실패하면 베이스는 서 있고, 다음 리셋이 다시 시작 자세로 보낸다. `/sim/drone_kill` 은 고정 베이스와 같음

### 10.2 sim 옵션 (`sim_ros2_moving_base.py`)

| 인자 | 값 | 설명 |
|---|---|---|
| `--base` | `husky` \| `stewart` | 필수 |
| `--base-motion` | Stewart `static`·`small`·`mid`·`big`·`lab` (필수) / Husky 없음·`static` | **`static` = 베이스 정지** (Husky 왕복 가운데, Stewart 진폭 0 프리셋). 토픽은 그대로 (정지값). 난이도 static → small → mid, 파이프라인 검사 |
| `--flight` `--mode` `--hover` `--drone-targets` `--reset-offset` `--position-source` | `sim_ros2.py` 와 같은 이름·뜻 | 기본 PX4. `--flight geometric` = 리셋 약 2 s (연습용, 녹화 안 함) |
| `--init-pose` | 관절각 6 개 또는 고정 씬 `init_poses` 이름 | 기본 `under_drone` |
| `--seed` `--headless` `--duration` `--prop-spin` `--protective-stop` `--imu-compare` | | 9 장 |

GUI 보조 뷰포트 (`--aux-views`) 는 아직 없음.

### 10.3 녹화·변환

| 단계 | 명령 | 움직이는 베이스에서 더해진 것 |
|---|---|---|
| 녹화 | `./6_record_bag.sh <작업명> --sim --base moving` | `/base/imu`·`/base/odom`·`/base/joint_states` 를 상태 토픽으로 (발행자 확인, `/base/imu` 수신 큐 120 Hz × 3 s). `episode.json` `"base": "moving"`. 기본 `--base fixed` 는 전과 같음 |
| stage1 | `python3 lerobot_stage1_extract_bag.py <bag> --arm-action command --base-imu topic --base-pose odom` | `base_imu.npy` = `/base/imu` 를 격자 구간 (t − 40 ms, t] 평균 → `observation.base_imu`. `base_pose.npy` (N, 7) = `/base/odom` 자세 [x, y, z, qw, qx, qy, qz] (중간 파일만, 66 ms 나이 검사). 두 옵션을 주면 그 토픽도 에피소드 시작·끝 시각 계산에 들어감 |
| 병합·검수 | 고정 베이스와 같음 | base_imu 방식 (`const` / `topic`) 이 다른 에피소드는 섞지 않음 → 고정 베이스 데이터와 자동으로 나뉨 |

IMU 를 구간 평균하는 이유: 120 Hz → 25 Hz 로 한 개만 고르면 12.5 Hz 보다 빠른 흔들림이 섞여 들어오고 (엘리어싱) 실물 IMU 잡음이 그대로 남는다.
평균 (약 5 개) 은 간단한 저역 통과 + 잡음 √5 배 감소. 직전 구간만 쓰는 것은 추론 때 클라이언트가 과거 IMU 만 갖고 있어서 (같은 규칙으로 붙일 계획). 대가는 약 20 ms 지연 (학습·추론이 같음).

### 10.4 검사

| 검사 | 결과 (2026-10-29 ~ 30) |
|---|---|
| `check_ros2_moving_base.py` (sim 을 PX4 + `--imu-compare` 로, 베이스마다) | **22/22** (Husky 11 + Stewart small 11). 9장 항목에 더해 9 첫 리셋 전 베이스 정지, 10 리셋 1 (23.9 / 24.2 s, 마운트 0.002 / 0.003 mm, 리셋 중 베이스 정지, 끝난 뒤 움직임), 11 `/sim/drone_kill` 뒤 리셋 2 (움직이던 베이스 → 시작 자세, Husky 팔 0.26°·마운트 0.51 mm), 11b 새 seed |
| `make_fake_episodes_moving.py [--base husky] [--cleanup]` | **4/4** (Stewart·Husky). A static 베이스에서 고정 명령으로 실제로 잡는 에피소드 2 개 → 병합·검수, B 흔들리는 베이스 1 개 → stage1 만 (Stewart small 비력 범위 0.54 m/s², base_pose 75 mm / Husky 0.20 m/s², 300 mm) |
| 회귀 | 고정 `make_fake_episodes.py` 3/3, `check_ros2` 9/9, 씬 `--check` 6/6 |

### 10.5 실제 PICO 텔레옵 (R6, 2026-10-29, Stewart)

| 에피소드 | 결과 |
|---|---|
| static | 17.3 s, 잡음 (그리퍼 state 최대 0.25), 리셋 24.3 s, 보호 정지 없음 |
| mid | 61.6 s, 잡음 (0.50), base_imu 비력 x·y 범위 약 1.1 m/s²·자이로 0.05 ~ 0.09 rad/s, 보호 정지 없음 |

- 변환·병합·검수 5/5 (시험 데이터는 지움). 사용자 소감: static 은 쉬움, mid 는 조금 어렵지만 풀 문제
- **GUI RTF 0.61 ~ 0.64** (고정 베이스 GUI 약 0.98). 텔레옵 60 Hz 명령이 sim time 기준 약 95 개/s 로 들어옴. 사용자: 불편하면 그때 개선 (후보: 다리 12 op 갱신을 렌더 주기로)

### 10.6 알아 둘 것 (해 보고 찾음)
- **녹화 시작 직후 토픽마다 첫 메시지가 한 스텝씩 다를 수 있다** (R6: `/base/imu` 가 8.3 ms 늦음). stage1 이 시작 시각을 팔·그리퍼·카메라만 보고 정하면 첫 프레임 IMU 구간이 비어 에러 → `--base-imu topic`·`--base-pose odom` 이면 그 토픽도 구간 계산에 넣음 (2026-10-30). 가짜 에피소드는 명령 재생 전 대기가 있어 이 경계가 안 생겼음
- **베이스 프리셋이 달라도 병합이 막지 않는다** (병합 검사는 팔 action 종류·base_imu 방식만). 수집 때는 작업명으로 나눈다 (`stw_bstatic`, `stw_bsmall`, …). 필요하면 병합에 베이스 종류·프리셋 검사를 옵션으로 추가
- 작업명과 실제 설정이 다를 수 있다 (R6 에서 `bstatic` 이름으로 mid 녹화). 진짜 설정은 `episode.json` `sim_reset.base.preset`
- 움직이는 중에 리셋하면 (Husky) 순간이동 뒤 팔 오차·마운트 거리가 조금 커진다 (0.26°, 0.51 mm, 기준 1°·2 mm 안)

### 10.7 명령
```bash
source /opt/ros/jazzy/setup.bash
# 터미널 1: sim (static 으로 손 풀기 → small → mid)
~/isaacsim/python.sh isaacsim/scripts/sim_ros2_moving_base.py --base stewart --base-motion small
~/isaacsim/python.sh isaacsim/scripts/sim_ros2_moving_base.py --base stewart --base-motion small --drone-targets --mode hover --hover small
# 터미널 2·3: 텔레옵 노드·손목 영상 (docs/pico_vr_data_collection.md 와 같음)
# 터미널 4: 녹화
./6_record_bag.sh stw_bsmall --sim --base moving
# 변환
python3 lerobot_stage1_extract_bag.py bags/<이름> --arm-action command --base-imu topic --base-pose odom
# 검사
python3 isaacsim/scripts/check_ros2_moving_base.py                       # 약 6 분
python3 isaacsim/scripts/make_fake_episodes_moving.py --cleanup          # 약 6 분
```
