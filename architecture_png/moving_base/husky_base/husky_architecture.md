# Husky A200 위 UR5 (움직이는 베이스: 직선 왕복)

공통 구조는 `../moving_base_architecture.md`. 여기는 Husky 전용 (에셋, 움직임, 배치, IMU, 토픽 값). 근거: `isaacsim/config/base_husky.yaml`, `scene_husky.yaml`, `moving_base.KinematicBase`, `docs/moving_base.md` 1 ~ 7 장.

상자 안에는 한글, ASCII 만 쓴다 (한글 = 2 칸으로 맞춤). 한글 고정폭 글꼴 (예: D2Coding) 에서 테두리가 맞게 보인다.
(*) = 고정 베이스 (`../fixed_base/fixed_base_architecture.md`) 와 달라지거나 새로 생긴 곳.

---

## 에셋 -> 씬

```text
┌─ Clearpath 공식 description ────────────────────────────────────────────────┐
│ husky/husky (BSD-3), 브랜치 humble-devel, 커밋 729f8aa (레포 밖 ~/husky_ws) │
└─────────────────────────────────────────────────────────────────────────────┘
          |  xacro (build_husky_urdf.sh) -> URDF -> Isaac Sim GUI URDF importer
          v
┌─ Husky USD ─────────────────────────────────────────────────────────┐
│ isaacsim/assets/bases/husky/husky.usda  (손으로 고치지 않음)        │
│ 차체 46 kg, 바퀴 4 개 (반지름 0.1651 m), 윗판, 레일, 범퍼, imu_link │
└─────────────────────────────────────────────────────────────────────┘
          |  moving_base.KinematicBase.add() - 메모리 stage 에서만
          v
┌─ 씬의 Husky (kinematic) ─────────────────────────────────────────────────┐
│ articulation root 를 떼고 모든 강체를 kinematic, 관절 (바퀴 포함) 끔     │
│ 매 물리 스텝: 최상위 prim translate = 궤적 위치 (PhysX 가 속도도 앎)     │
│ 바퀴: 보이는 메시만 회전 (각 = 이동 거리 / 반지름)                       │
│   (바퀴 강체를 USD 로 돌리면 충돌 형상이 다시 만들어져 tensor view 무효) │
└──────────────────────────────────────────────────────────────────────────┘
```

## 움직임 - 사다리꼴 직선 왕복 (LinearTrajectory)

```text
┌─ base_husky.yaml trajectory ─────────────────────────────────────────────────────────────┐
│ 방향 = Husky base_link x (앞) = world +y (Husky 는 yaw +90 도로 섬)                      │
│ 거리 0.30 m, 최고 속도 0.05 m/s, 가감속 0.10 m/s^2, 양 끝에서 1.0 s 멈춤 -> 한 주기 15 s │
│                                                                                          │
│ s(t): 가속 (0.5 s) -> 등속 -> 감속 (0.5 s) -> 멈춤 1 s -> 반대로                         │
│ p = p0 + s d,   v = s_dot d,   a = s_ddot d,   R = Rz(yaw0) (회전 없음)                  │
│                                                                                          │
│ 리셋마다 바뀌는 값 없음 (궤적 하나, seed 는 기록만)                                      │
│ static (--base-motion static): 왕복 가운데에 서 있음                                     │
└──────────────────────────────────────────────────────────────────────────────────────────┘

  시간 ->   |가속| 등속 (약 5.5 s) |감속| 멈춤 1 s |가속| 등속 |감속| 멈춤 1 s | ...
  위치       0 ---------------------> 0.30 m ---------------------> 0
  IMU 비력 x, y  : 가속 구간 +-0.1 m/s^2 계단, 나머지 0
  IMU 비력 z     : 9.81 (계속),  자이로 : 0
```

## 배치 (world, 로봇 base 기준 축 = world 축)

```text
┌─ scene_husky.yaml ───────────────────────────────────────────────────────────────────────────┐
│ UR5 마운트: base_link 기준 (0.0812, 0, 0.2515), yaw -90 도 (윗판 가운데 위)                  │
│   -> 로봇 정면 +x = Husky 오른쪽, 로봇 base 높이 = 0.13228 + 0.2515 = 0.384 m                │
│ 에피소드 시작: 로봇 base = 왕복 가운데 - 0.15 m (한쪽 끝) -> 0.30 m 이동해 반대쪽 끝         │
│   (static 은 가운데)                                                                         │
│ 드론 기준 위치 = 왕복 가운데의 로봇 base + (0.60, 0, 0.738)                                  │
│ PX4 이륙 지점 = (1.2, 0.15) 바닥 (body 높이 0.06 m)                                          │
│ third view = world 고정 (고정 씬 Q2 와 같은 로봇 base 상대 위치, 높이만 이 씬 base 에 맞춤)  │
│ 충돌 끔: 로봇 robot_mount, base_link_inertia <-> Husky 강체 (붙은 곳)                        │
│ UR5 연결: root_joint body0 = Husky Geometry/base_link + excludeFromArticulation              │
│   (없으면 USD Physics 해석기가 Husky 를 로봇 articulation 링크로 흡수 -> Husky 가 안 움직임) │
└──────────────────────────────────────────────────────────────────────────────────────────────┘
```

## 베이스 토픽 값

```text
┌─ Husky ────────────────────────────────────────────────────────────────────────────┐
│ /base/imu         : imu_link = 차체 base_link 원점 (r = 0), 바닥 위 0.132 m        │
│                     자이로 0, 비력 (0, +-0.1 가감속, 9.81) (Husky 좌표계, x = 앞)  │
│ /base/odom        : frame odom -> base_link, 시작 = 원점, 위치 0 ~ 0.30 m (x)      │
│ /base/joint_states: front_left / front_right / rear_left / rear_right _wheel_joint │
│                     회전각 s / 0.1651 [rad], 각속도 v / 0.1651 (네 바퀴 같음)      │
└────────────────────────────────────────────────────────────────────────────────────┘
```

## 검사, 결과 (2026-10-29)

```text
┌─ 검사 ───────────────────────────────────────────────────────────────────────────────────────────────────┐
│ moving_base_scene.py --headless --check   6/6 (차체 궤적 오차, 마운트 간격 < 1 mm, 팔 자세, 드론, 영상)  │
│ check_ros2_moving_base.py (husky)          11/11 (토픽 주기, IMU 두 방법 일치, 리셋 23.9 s)              │
│ make_fake_episodes_moving.py --base husky  4/4 (B 흔들림: 비력 범위 0.20, base_pose 300 mm)              │
│ 임시값 (실물 Husky 를 고친 뒤 맞출 것): 마운트 위치, 높이, 방향, 궤적 거리, 속도, 실물 구성 (윗판, 범퍼) │
└──────────────────────────────────────────────────────────────────────────────────────────────────────────┘
```
