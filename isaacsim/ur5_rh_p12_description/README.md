# ur5_rh_p12_description

## URDF 요약: `ur5_rh_p12_d435i`

### 파일과 빌드

| 항목 | 값 |
|------|----|
| 파일 | `~/data_collection_ur5_gripper/isaacsim/ur5_rh_p12_description/out/ur5_rh_p12_d435i.urdf` |
| 원본 | `urdf/ur5_rh_p12_d435i.urdf.xacro` → `scripts/build_urdf.sh` 로 빌드 (`out/` 은 git 제외) |
| 로봇 이름 | `ur5_rh_p12_d435i` |
| 규모 | 링크 36개, 조인트 35개 (revolute 10, fixed 25) |
| 기구학 | `~/my_robot_calibration.yaml` (고장 난 옛 UR5 기준, 새 로봇이 오면 교체) |

### 구조

| 항목 | 값 |
|------|----|
| 루트 링크 | `robot_mount` (world 에 고정 안 됨. 테이블이나 6DOF 베이스에 FixedJoint 로 붙임) |
| 체인 | `robot_mount → base_link → … → wrist_3_link → flange → tool0 → wrist_mount → gripper_bracket → rh_p12_rn_base` |
| 카메라 가지 | `wrist_mount → wrist_camera_mount → wrist_camera_bottom_screw_frame → … → wrist_camera_color_optical_frame` |

### 구동 조인트 (7개)

| 조인트 | 범위 (rad) | 비고 |
|--------|-----------|------|
| `shoulder_pan_joint` | ±2π | 팔, state/action 순서 1 |
| `shoulder_lift_joint` | ±2π | 2 |
| `elbow_joint` | ±π | 3 |
| `wrist_1_joint` | ±2π | 4 |
| `wrist_2_joint` | ±2π | 5 |
| `wrist_3_joint` | ±2π | 6 |
| `rh_r1_joint` | 0(열림) ~ 1.1351(닫힘) | 그리퍼 구동. raw 0~1150 ↔ 0~1.1351 선형 |

**mimic 조인트 (3개)**: `rh_r2`, `rh_l1`, `rh_l2` — 모두 `rh_r1_joint` 를 배율 1 로 따라감

### 주요 프레임

| 프레임 | 의미 | tool0 기준 위치 |
|--------|------|------|
| `tool0` | UR 플랜지 면 (+z = 공구 방향) | 원점 |
| `wrist_mount` | 카메라 마운트 (tool0 와 같은 위치, `mount_yaw` 만큼 회전) | z = 0 |
| `gripper_bracket` | RH-P12-RN-UR 브래킷 바닥 | z = 5 mm |
| `rh_p12_rn_base` | 그리퍼 바닥 | z = 15 mm |
| `rh_p12_rn_tcp` | 손가락 사이 파지점 | z = 125 mm |
| `wrist_camera_color_optical_frame` | 손목 카메라 color (Isaac Camera prim 을 여기에, X 축 180°) | 탭 쪽 +y 약 58 mm |

### 현재 인자값과 확인 상태

| 인자 | 값 | 상태 |
|------|----|------|
| `mount_thickness` | 0.005 | STL 에서 측정 |
| `adapter_thickness` | 0.010 | 매뉴얼 그림에서 추정 → **캘리퍼스로 확인** |
| `gripper_yaw` | π/2 | 매뉴얼 기반 추정 → **실물 조립으로 확인** |
| `mount_yaw` | 0 | 결정값 (실물을 이대로 조립) |
| `cam_tilt` | 0 | 다운받은 마운트 그대로. 1-7 영상 보고 판단 |
| `mount_mass` / `adapter_mass` | 0.05 / 0.08 kg | 추정 |

### 알려진 이슈

- 그리퍼 조인트 effort 한계가 원본 그대로 1000 → 실물보다 훨씬 셈. 1-5 drive 튜닝에서 실물 전류 한계(400 mA)에 맞춰 낮출 것

### 출처

| 부품 | 출처 |
|------|------|
| 그리퍼 xacro·메시 | [ROBOTIS-GIT/open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator) `main` @ `d31000d90c679af9c982e73de8b12d777c5ff7dd` (2026-09-28): `open_manipulator_description/urdf/omy_f3m/rh_p12_rn_a.urdf.xacro`, `open_manipulator_description/meshes/rh_p12_rn_a/` (Apache-2.0). 메시는 [ROBOTIS-GIT/RH-P12-RN](https://github.com/ROBOTIS-GIT/RH-P12-RN) `master` @ `58d06e797fd3835527ff2488775003ebc65fa6d6` 의 것과 바이트 단위로 동일 |
| 그리퍼 수정 사항 | 관성값을 메시로 재계산(원본 1e-6), TCP 프레임 추가, 메시 경로를 인자로 |
| 카메라 마운트 | "Realsense Support UniversalRobots" (Niko Bonomi, 2022) STL → `meshes/mount/wrist_mount.stl` |
| 브래킷 | 모델 없음. 원판 + 사각 블록 단순 형상 (RH-P12-RN-UR 매뉴얼 기준 추정) |
| UR5 | ROS 패키지 `ur_description` (apt) |
| D435i | ROS 패키지 `realsense2_description` (apt) |

---

UR5(CB3) + 샌드위치 손목 마운트 + RH-P12-RN(A) + RealSense D435i 를 **하나의 URDF(=하나의 articulation)** 로 만들어
Isaac Sim URDF Importer 로 가져오기 위한 패키지.

```
robot_mount ─ base_link ─ ... ─ wrist_3_link ─ flange ─ tool0
                                                        └─ wrist_mount (샌드위치, 두께 mount_thickness)
                                                             ├─ gripper_bracket (z = mount_thickness, 두께 adapter_thickness)
                                                             │    └─ rh_p12_rn_base
                                                             │         ├─ rh_r1_joint (구동, 0 ~ 1.1351 rad) ─ rh_r2 (mimic x1)
                                                             │         ├─ rh_l1 (mimic x1) ─ rh_l2 (mimic x1)
                                                             │         └─ rh_p12_rn_tcp (고정 frame)
                                                             └─ wrist_camera_mount
                                                                  └─ wrist_camera_bottom_screw_frame (마운트 나사 구멍)
                                                                       └─ wrist_camera_link ─ … ─ wrist_camera_color_optical_frame
```

- 구동 조인트 7개: `shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint, rh_r1_joint`
  (팔 6개 이름은 실물 UR 드라이버와 동일 → stage1 의 이름 기반 재정렬 그대로 사용 가능)
- 그리퍼 매핑: raw `0~1150` ↔ `rh_r1_joint` `0~1.1351 rad` (선형). 열림 폭 ≈ 107 mm (스펙 스트로크와 일치)
- 그리퍼 형상/메시: ROBOTIS `open_manipulator` (Apache-2.0). 관성은 메시로 재계산 (원본은 1e-6 placeholder)

## 필요 패키지 (ROS 2 Jazzy)

```bash
sudo apt install ros-jazzy-xacro ros-jazzy-ur-description ros-jazzy-realsense2-description liburdfdom-tools
```

## 빌드

```bash
source /opt/ros/jazzy/setup.bash
cd ~/data_collection_ur5_gripper/isaacsim/ur5_rh_p12_description
./scripts/build_urdf.sh                       # placeholder 값으로 빌드
# 실물 캘리브레이션 반영 예시
./scripts/build_urdf.sh kinematics_params:=$HOME/my_robot_calibration.yaml
```
결과: `out/ur5_rh_p12_d435i.urdf` (모든 mesh 경로가 절대경로로 치환됨, 누락 mesh 있으면 에러)

### 마운트 (`meshes/mount/wrist_mount.stl`, "Realsense Support UniversalRobots")

STL 에서 측정해 기본값으로 반영함:

| 항목 | 값 |
|------|----|
| 베이스 판 두께 (플랜지 면 → 그리퍼 면) | 5.0 mm → `mount_thickness=0.005` |
| 림 | 내경 63.5 mm, 6.45 mm. UR 플랜지를 감싸는 부분이라 오프셋에 포함 안 됨 |
| 볼트 패턴 | Ø6.5 x4, PCD 50 (ISO 9409-1-50-4-M6), PCD 50 위 Ø6.5 핀 구멍 1개 |
| 카메라 탭 | M3 관통 x2, 간격 45 mm (D435 후면 M3 간격과 일치), 플랜지 축에서 57.5 mm |
| 부피 | 45.1 cm³ |

### 실물 확인 필요 (CHECK)

| arg | 기본값 | 의미 |
|-----|--------|------|
| `mount_yaw` | 0 | tool0 z 축 기준 마운트(+카메라+그리퍼) 회전. 0 이면 카메라 탭이 tool0 +y 방향 |
| `gripper_yaw` | π/2 | 마운트 기준 그리퍼 회전. π/2 = 손목 영상에서 두 손가락이 좌우로 보임 (0 이면 열린 손가락이 카메라와 간섭) |
| `adapter_thickness` | 0.010 | RH-P12-RN-UR 브래킷(FRP42-A120K): 마운트 면 → 그리퍼 바닥. 매뉴얼 측면도에서 추정(±1 mm), 캘리퍼스로 확인 |
| `cam_xyz`, `cam_rpy` | 카메라 윗면이 공구 축 쪽 | 뒤집어 달았으면 xacro 주석의 upside-down 값 사용 |
| `cam_tilt` | 0 (다운받은 마운트) | 카메라를 공구 축 쪽으로 기울이는 각도 [rad]. 탭 면의 M3 구멍 중심 기준 회전. 기울인 마운트 평가용 |
| `mount_mass` | 0.05 kg | 재질·채움률에 따라 (PLA 100% ≈ 0.056, 알루미늄 ≈ 0.122) |

카메라 위치: D435 메시 후면 M3 구멍(Ø2.5, 간격 45 mm, 본체 높이 중앙)이 마운트 구멍과 0.05 mm 이내로 일치함을 확인.

그리퍼 형상은 ROBOTIS 도면(RH-P12-RN, 2020/02/21)과 대조함: 전체 높이 116.3 mm(URDF 116.5), 최대 열림 외폭 151 mm(URDF 150.8), 본체 폭 54 mm 일치.
그리퍼 바닥은 M3 탭(12개)만 있고 ISO 9409 M6 패턴이 없으므로 RH-P12-RN-UR 브래킷(FRP42-A120K)으로 체결한다.
조립 순서: UR 플랜지 → 카메라 마운트(5 mm) → 브래킷(≈10 mm) → 그리퍼. 브래킷 M6 볼트는 기본 M6x08 보다 5 mm 긴 것 사용(플랜지 나사 깊이 확인).
브래킷 두께 추정: 매뉴얼 측면도에서 본체 폭 54 mm 로 축척(1.593 px/mm) → 플랜지~본체 하단 18.8 mm − 메시 보스 8.9 mm = 9.9 mm,
교차검증: 플랜지~손가락 끝 126.5 mm − 도면 116.3 mm = 10.2 mm.

### 손목 카메라 시야 (fx=615, 640x480, 브래킷 10 mm 포함, 기구학 계산)

| cam_tilt | 손가락 끝 링크가 화면 안 (열림: r2 / l2) | (닫힘: r2 / l2) | 닫힘 시 TCP 영상 좌표 (u, v) |
|----------|------|------|------|
| 0° | 0% / 0% | 7% / 7% | (521, -116) 화면 밖 |
| 10° | 25% / 0% | 63% / 59% | (506, 35) |
| 15° | 49% / 0% | 96% / 93% | (502, 103) |
| 20° | 62% / 0% | 100% / 97% | (499, 168) |
| 25° | 89% / 0% | 100% / 97% | (498, 232) |

TCP 가 오른쪽(u≈515)에 치우치는 이유: D435 color 센서가 본체 중앙에서 32.5 mm 옆에 있음.

## Isaac Sim 으로 가져오기

import 절차와 확인 결과는 `docs/PLAN.md` 1-3 참고.
