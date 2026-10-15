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
| `wrist_camera_color_optical_frame` | 손목 카메라 color (Isaac Camera prim 을 여기에, X 축 180°) | `t10` (기본): (0, 53.9, 25.4) mm = 렌즈가 그리퍼 중심선, 10° 기울임. `original`: (32.5, 57.5, 25.75) mm |

### 현재 인자값과 확인 상태

| 인자 | 값 | 상태 |
|------|----|------|
| `mount_variant` | `t10` | 손목 카메라 마운트 (2026-10-15): 메시·충돌·질량·카메라 위치를 한꺼번에 고름. `t10`·`t7`·`t0` = `cad/wrist_mount_cad.py` (옆 −32.5 mm + 기울기), `original` = 내려받은 마운트. **10° 는 출력해서 실물 손목 영상 확인함** |
| `mount_thickness` | 0.005 | STL 에서 측정 (새 마운트도 같음) |
| `adapter_thickness` | 0.010 | 실측 확인 (약 10 mm, 브래킷 단독) |
| `gripper_yaw` | π/2 | 실물 확인 완료 (손가락이 카메라 긴 변과 나란히 벌어짐, 시야를 가리지 않음) |
| `mount_yaw` | 0 | 결정값 (실물을 이대로 조립) |
| 카메라 기울기·옆 위치 | `mount_variant` 에서 (따로 바꾸는 인자 없음) | `t10`: 10°, −32.5 mm / `t7`: 7° / `t0`: 0° / `original`: 0°, 0 |
| 마운트 질량 / `adapter_mass` | `mount_variant` 에서 / 0.08 kg | `t*`: 32.36 g (10° 출력물을 저울로, PLA 채움 15 %) / `original`: 0.05 kg 추정. 브래킷은 추정 |

### 알려진 이슈

- 그리퍼 조인트 effort 한계가 원본 그대로 1000 → 실물보다 훨씬 셈. 1-5 drive 튜닝에서 실물 전류 한계(400 mA)에 맞춰 낮출 것

### 출처

| 부품 | 출처 |
|------|------|
| 그리퍼 xacro·메시 | [ROBOTIS-GIT/open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator) `main` @ `d31000d90c679af9c982e73de8b12d777c5ff7dd` (2026-09-28): `open_manipulator_description/urdf/omy_f3m/rh_p12_rn_a.urdf.xacro`, `open_manipulator_description/meshes/rh_p12_rn_a/` (Apache-2.0). 메시는 [ROBOTIS-GIT/RH-P12-RN](https://github.com/ROBOTIS-GIT/RH-P12-RN) `master` @ `58d06e797fd3835527ff2488775003ebc65fa6d6` 의 것과 바이트 단위로 동일 |
| 그리퍼 수정 사항 | 관성값을 메시로 재계산(원본 1e-6), TCP 프레임 추가, 메시 경로를 인자로 |
| 카메라 마운트 (`original`) | "Realsense Support UniversalRobots" (Niko Bonomi, 2022) STL → `meshes/mount/wrist_mount.stl` |
| 카메라 마운트 (`t0`·`t7`·`t10`) | 이 레포 `cad/wrist_mount_cad.py` (CadQuery). 플랜지·브래킷과 맞물리는 치수는 `original` STL 에서 잼, 카메라 탭만 새로. → `meshes/mount/wrist_mount_t*.stl`, 충돌 판 `wrist_mount_collision_plate.stl` |
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

### 손목 카메라 마운트 `t10` (기본, 2026-10-15, `docs/PLAN.md` 5-B "손목 카메라 마운트")

`original` 마운트는 D435 color 렌즈가 그리퍼 중심축에서 32.5 mm 옆 (삼각대 구멍 중심선 → 왼쪽 IR 17.5 + RGB 15 mm, D400 datasheet Fig. 4-7) 이고
카메라가 공구 축과 평행이라, 손목 영상에서 그리퍼 중심이 오른쪽 아래로 치우치고 잡는 순간 손가락이 화면 밖이었다.
sim 에서 후보를 비교 (`isaacsim/scripts/compare_wrist_camera.py`, `wrist_camera_clearance.py`) 해 탭을 옆으로 −32.5 mm 옮기고 공구 축 쪽으로 기울인 마운트를 만들었다.

| 항목 | 값 |
|------|----|
| CAD | `cad/wrist_mount_cad.py` (CadQuery, 레포 밖 venv `~/cad/venv`) → `cad/out/wrist_mount_t0·t7·t10` `.stl`·`.step`·미리보기. `wrist_mount` 좌표계 그대로라 URDF 에서 회전 없음 |
| 맞물림 (`original` 과 같음) | 판 5.0 mm, 플랜지 쪽 테두리 Ø63.5 × 6.45, Ø6.5 × 4 (PCD 50), 핀 구멍 (탭 반대쪽), 가운데 Ø34 → **TCP 위치 그대로** |
| 카메라 탭 | 옆 −32.5 mm (x), 공구 축에서 57.5 mm, 기울기 10° (`t7` 7°, `t0` 0°). M3 나사 자리 6.0 mm + Ø5.8 카운터보어 + 입구 모따기 1 mm → **M3 × 8 이 카메라에 2 mm** (D435 최대 3 mm, 0.4 Nm) |
| 충돌 형상 | 판 (`wrist_mount_collision_plate.stl`: 외곽 그대로, z 0 ~ 5 mm, 볼록) + 탭 상자 (z −6.45 ~ 5 mm). STL 전체는 쓰지 않음 (볼록 껍질이 플랜지 홈을 메워 손목과 겹침) |
| 질량·관성 | 32.36 g (실측), 무게중심 (−26.6, 24.9, −0.3) mm·관성은 STL 에서 (균일 밀도) |
| 실물 확인 | 10° 를 출력해 장착, 그리퍼를 닫고 손목 영상: 손가락이 화면 아래 가운데, 방향 sim 과 같음 (가로 약 10 px ≈ 1.6 mm 차이) |

마운트를 바꿀 때: `./scripts/build_urdf.sh kinematics_params:=$HOME/my_robot_calibration.yaml mount_variant:=t0` → 아래 "Isaac Sim 으로 가져오기" → 회귀 검사 → 시작 자세 (`isaacsim/config/scene_drone.yaml` `init_poses.under_drone`) 다시 구하기.
기울기가 다른 데이터는 섞지 않는다.

### 마운트 `original` (`meshes/mount/wrist_mount.stl`, "Realsense Support UniversalRobots")

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
| `mount_yaw` | 0 | tool0 z 축 기준 마운트(+카메라+그리퍼) 회전. 0 이면 카메라 탭이 tool0 +y 방향. 결정값 (실물을 이 방향으로 조립) |
| `gripper_yaw` | π/2 | 마운트 기준 그리퍼 회전. π/2 = 손목 영상에서 두 손가락이 좌우로 보임 (0 이면 열린 손가락이 카메라와 간섭). 실물 확인 완료 |
| `adapter_thickness` | 0.010 | RH-P12-RN-UR 브래킷(FRP42-A120K): 마운트 면 → 그리퍼 바닥. 실측 확인 (약 10 mm, 브래킷 단독). ~~캘리퍼스로 측정~~ 완료 |
| `cam_rpy` | `-1.5708 -1.5708 0` = 카메라 윗면이 바깥쪽(공구 축 반대) | 실물 장착 방향 확인 완료. 윗면을 공구 축 쪽으로 달 경우 xacro 주석의 flipped 값 사용 |
| 카메라 위치·기울기·마운트 질량 | `mount_variant` 에서 | (2026-10-15 `cam_tilt`·`cam_xyz`·`mount_mass` 인자는 없앰: 마운트와 카메라가 어긋나지 않게) |

카메라 위치: D435 메시 후면 M3 구멍(Ø2.5, 간격 45 mm, 본체 높이 중앙)이 마운트 구멍과 0.05 mm 이내로 일치함을 확인.

그리퍼 형상은 ROBOTIS 도면(RH-P12-RN, 2020/02/21)과 대조함: 전체 높이 116.3 mm(URDF 116.5), 최대 열림 외폭 151 mm(URDF 150.8), 본체 폭 54 mm 일치.
그리퍼 바닥은 M3 탭(12개)만 있고 ISO 9409 M6 패턴이 없으므로 RH-P12-RN-UR 브래킷(FRP42-A120K)으로 체결한다.
조립 순서: UR 플랜지 → 카메라 마운트(5 mm) → 브래킷(실측 약 10 mm) → 그리퍼. 브래킷 M6 볼트는 기본 M6x08 보다 5 mm 긴 것 사용(플랜지 나사 깊이 확인).
브래킷 두께 추정(실측 전 근거, 실측 약 10 mm 와 일치): 매뉴얼 측면도에서 본체 폭 54 mm 로 축척(1.593 px/mm) → 플랜지~본체 하단 18.8 mm − 메시 보스 8.9 mm = 9.9 mm,
교차검증: 플랜지~손가락 끝 126.5 mm − 도면 116.3 mm = 10.2 mm.

### 손목 카메라 시야 (fx=615, 640x480, 브래킷 10 mm 포함, 기구학 계산) — `original` 마운트 기준 (2026-09-30)

`t10` 마운트 결과는 위 표와 `docs/PLAN.md` 5-B (TCP (332, 446), 닫힌 손가락 7.4 %). 카메라 윗면이 바깥쪽 기준. 손가락 비율은 메시 꼭짓점 중 화면 안에 들어오는 비율.

| cam_tilt | 손가락 끝 링크가 화면 안 (열림: r2 / l2) | (닫힘: r2 / l2) | 닫힘 시 TCP 영상 좌표 (u, v) |
|----------|------|------|------|
| 0° | 0% / 0% | 6% / 7% | (521, 596) 화면 밖 |
| 10° | 0% / 24% | 59% / 62% | (506, 445) |
| 15° | 0% / 47% | 94% / 97% | (502, 377) |
| 20° | 0% / 60% | 97% / 100% | (499, 312) |
| 25° | 0% / 89% | 97% / 100% | (498, 248) |

TCP 가 오른쪽(u≈515)에 치우치는 이유: D435 color 센서가 본체 중앙에서 32.5 mm 옆에 있음.
손가락과 TCP 는 영상 아래쪽에서 들어온다. 윗면을 공구 축 쪽으로 달았을 때(이전 기본값)와 비교하면 v 가 480 − v 로 뒤집히고, 열린 상태에서 보이는 손가락이 r2 → l2 로 바뀐다.

## Isaac Sim 으로 가져오기

import 절차와 확인 결과는 `docs/PLAN.md` 1-3 참고. 2026-10-15 (`t10`) 재import 순서:

1. `./scripts/build_urdf.sh kinematics_params:=$HOME/my_robot_calibration.yaml` → `out/ur5_rh_p12_d435i.urdf`
2. `~/isaacsim/isaac-sim.sh` → File → Import → 위 URDF. **USD Output = 레포 밖 임시 폴더** (예 `~/usd_import_t10`),
   Collision From Visuals 끔, Allow Self-Collision 끔, Robot Type 기본, Base Type Source, Add Reference to Stage 끔, Merge Mesh 끔, Debug Mode 끔, ROS Package List 비움
3. 임시 폴더 결과를 `isaacsim/assets/robots/ur5_rh_p12_d435i/` 와 비교 (`.usda` diff, `geometries.usd` 는 USD Python 으로 메시 비교). 바뀐 것만 바뀌었으면 교체 (빈 `Textures/` 는 빼고)
4. 회귀 검사: `check_articulation` 8/8, `check_self_collision`, `wrist_camera_clearance`, `check_wrist_camera`, `check_ik_model` 2/2, `drone_scene --check` 3/3, `teleop/check_xr_teleop.py` 13/13, `check_ros2` 9/9
