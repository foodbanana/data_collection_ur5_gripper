# 움직이는 베이스 씬 (Husky A200 / Stewart 플랫폼 위 UR5)

작성: 2026-10-28 · Stewart 추가 2026-10-29 · `docs/PLAN.md` 8단계, 브랜치 `isaacsim_v6.1.0`

UR5 를 움직이는 베이스 위에 올린 sim 씬. **고정 베이스 씬 (`drone_scene.py`, `sim_ros2.py`) 과 파일을 완전히 나눴다** (그쪽 파일·설정은 그대로).
베이스는 두 가지: **Husky A200** (직선 왕복) 와 **Stewart 플랫폼** (윗판 6 축 사인파, sim 전용, 8 장). 다음은 수집 연결 (`sim_ros2_moving_base.py`) (PLAN 8단계 "다음 계획").
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
| 범위 밖 (다음) | ROS 2 토픽·텔레옵·녹화·리셋, `/base/imu`, PX4 드론 |

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

