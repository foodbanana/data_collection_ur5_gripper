# 움직이는 베이스 씬 (Husky A200 위 UR5)

작성: 2026-10-28 · `docs/PLAN.md` 8단계, 브랜치 `isaacsim_v6.1.0`

UR5 를 움직이는 베이스 위에 올린 sim 씬. **고정 베이스 씬 (`drone_scene.py`, `sim_ros2.py`) 과 파일을 완전히 나눴다** (그쪽 파일·설정은 그대로).
지금은 Husky A200 (직선 왕복) 만 있고, 다음에 Stewart 플랫폼 (6 축 사인파) 과 수집 연결 (`sim_ros2_moving_base.py`) 을 만든다 (PLAN 8단계 "다음 계획").

---

## 1. 한눈에 보기

| 항목 | 내용 |
|---|---|
| 씬 | 바닥 + Husky (kinematic 직선 왕복) + UR5 (윗판 가운데) + 손목 카메라 + 드론 (world 고정 호버, 기하 제어기) + third view (world 고정). 테이블 없음 |
| 실행 | `~/isaacsim/python.sh isaacsim/scripts/moving_base_scene.py` (GUI) / `--headless --duration S` |
| 검사 | `moving_base_scene.py --headless --check` → C1 ~ C6 **6/6** (약 1 분), `check_moving_base.py --headless` (Husky 단독) → **4/4** |
| 설정 | `isaacsim/config/scene_husky.yaml` (마운트·드론·시작 자세), `isaacsim/config/base_husky.yaml` (Husky 에셋·궤적) |
| 에셋 | `isaacsim/assets/bases/husky/` (Clearpath 공식 description, README) |
| 범위 밖 (다음) | ROS 2 토픽·텔레옵·녹화·리셋, `/base/imu`, PX4 드론, Stewart |

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
