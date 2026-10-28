# Husky A200 (움직이는 베이스)

`husky.usda` (+ `payloads/`) = Clearpath Husky A200 공식 description 을 Isaac Sim URDF importer 로 바꾼 것. 손으로 고치지 않는다 (다시 import).
UR5 를 위에 올려 직선으로 움직이는 베이스로 쓴다 (PLAN 8단계, 고정 베이스 씬과 별도).

## 출처
- `https://github.com/husky/husky` 브랜치 `humble-devel`, 커밋 **`729f8aa45ccd86fa33a05e07ef698c52c451cd9c`** (2023-10-30), 패키지 `husky_description`
- 라이선스 **BSD-3-Clause** (Clearpath Robotics). 원본 레포는 레포 밖 (`~/husky_ws/husky`) 에 두고 여기에는 import 결과만 둔다
- 구성 = xacro 기본값: 차체·윗판 (`top_plate`)·레일·범퍼·바퀴 4. 레이저·RealSense·sensor arch 없음. **실물 Husky 의 구성 (윗판 종류, 범퍼, arch) 은 확인 전** (실물 고장, 2026-10-28)

## 다시 만들기
```bash
source /opt/ros/jazzy/setup.bash
isaacsim/scripts/build_husky_urdf.sh          # clone (없으면) → 커밋 확인 → xacro → ~/husky_ws/husky_out/husky.urdf
```
Isaac Sim GUI (`~/isaacsim/isaac-sim.sh`) → File → Import → 위 URDF. 옵션은 로봇 USD 와 같다 (`isaacsim/ur5_rh_p12_description/README.md` "Isaac Sim 으로 가져오기"):
USD Output = **레포 밖 임시 폴더** (예 `~/husky_ws/usd_import`), Collision From Visuals 끔, Allow Self-Collision 끔, Robot Type 기본, Base Type Source,
Add Reference to Stage 끔, Merge Mesh 끔, Debug Mode 끔, ROS Package List 비움 → 결과를 이 폴더와 비교한 뒤 교체 (빈 `Textures/` 는 뺀다)

## 구조 (2026-10-28 import, URDF USD Converter v0.3.2)
- `Physics` variant: physx / physics / mujoco / none (기본 선택 없음 → reference 할 때 `physx` 명시, 로봇 USD 와 같음)
- articulation root = `/husky/Geometry/base_link` (강체). `root_joint` (고정) = 최상위 prim `/husky` → `base_link`
- 바퀴 4 (`front|rear_left|right_wheel_joint`): revolute, 축 Y, drive 게인 0 (import 그대로)

| 항목 | 값 (`base_link` 기준, m) |
|---|---|
| 질량 | 차체 46.034 kg, 바퀴 2.637 kg × 4 |
| 바퀴 | 반지름 0.1651, 폭 0.1143, 위치 (±0.256, ±0.2854, 0.03282) |
| 바닥 (`base_footprint`) | z −0.13228 → `base_link` 는 바닥 위 0.132 |
| 윗판 (`top_plate_link`) | (0.0812, 0, 0.245) → 바닥 위 약 0.377 |
| 충돌 | 바퀴 원기둥 4, 차체 상자 2, 윗판 |

- UR5 장착 (임시값, 실물 측정 전): Clearpath 공식 Husky+UR5 (`husky/husky` `indigo-devel` `husky.urdf.xacro` `ur5_arm_base`) = `top_plate_link` 위 (0.3058, 0, 0.0065), yaw −90°
