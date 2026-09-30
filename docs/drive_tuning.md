# 1-5. Drive 튜닝 정리

작성: 2026-09-30 · 대상 커밋: `5f22d0d` (브랜치 `isaacsim_v6.1.0`)

로봇 USD(`ur5_rh_p12_d435i.usda`)를 import 하면 모든 관절의 drive 값이 0 이라 명령을 줘도 움직이지 않는다.
1-5 는 **팔과 그리퍼가 명령에 실물처럼 반응하도록** drive 값을 정한 단계다.

---

## 1. 한눈에 보기

| 항목 | 결론 |
|------|------|
| 제어 방식 | 관절별 **PD 제어** (PhysX joint drive, implicit) |
| 팔 | **높은 stiffness** 로 드론 무게까지 버팀. 중력 보상(feedforward) 없음 |
| 그리퍼 | **목표값 이동**(0.516 rad/s)으로 동작 시간, **최대 힘**으로 파지력을 따로 맞춤 |
| 베이스 | 씬에서 **fixed base override** |
| 자기 충돌 | **켬**, forearm 만 **convex decomposition** |
| 시험 | A~F 전부 통과 |
| 로봇 USD 원본 | **수정 안 함.** 모든 값은 설정 파일에 있고 씬을 띄울 때 적용 |

---

## 2. 파일

| 파일 | 역할 |
|------|------|
| `isaacsim/config/drive_gains.yaml` | **기본 설정.** 관절별 K·D·최대 힘·최대 속도, 그리퍼 설정, physics·articulation 설정 |
| `isaacsim/config/drive_gains_ff.yaml` | 비교 기록: 중력 보상 + 낮은 K (채택 안 함) |
| `isaacsim/config/drive_gains_official.yaml` | 비교 기록: Isaac Sim 공식 UR5 값 (채택 안 함) |
| `isaacsim/config/gravity_worst_cases.yaml` | B 시험 최악 경우 (가능한 자세 기준) |
| `isaacsim/scripts/test_scene.py` | 테스트 씬 생성 (fixed base override, 조명, 충돌 설정). 2·3단계 씬도 사용 |
| `isaacsim/scripts/robot_drive.py` | 설정 적용 (`RobotDrive(robot, cfg)` → `.apply()` 적용·확인, `.start()` 스텝 콜백), 그리퍼 목표값 이동 (`GripperProfile`) |
| `isaacsim/scripts/compute_gain_seed.py` | K·D 시작값 계산 (`--collision-free`) |
| `isaacsim/scripts/tune_drives.py` | 시험 A~F 실행과 리포트 |
| `isaacsim/scripts/check_articulation.py` | 구조 회귀 검사 (8/8 PASS) |
| `isaacsim/scripts/check_self_collision.py`, `collision_geom.py` | 자기 충돌 사전 점검 |
| `isaacsim/reports/` | 실행 리포트 (git 제외) |

**값을 적용하는 방식**
- 로봇 USD 는 import 결과(drive 0) 그대로 둔다
- 씬을 띄울 때 `test_scene.py` → fixed base override, `robot_drive.py` 의 `RobotDrive.apply()` → 설정 파일의 값을 **tensor API 로 메모리 속 로봇에 적용**
- 그래서 GUI 에서 로봇 USD 를 그냥 열면 튜닝값이 없다(stiffness 0, floating base). 튜닝된 로봇을 보려면 스크립트로 씬을 띄운다
- tensor API 로 넣은 값은 USD 속성을 거치지 않아서, GUI Property 창에는 0 으로 보일 수 있다. 실제 적용값은 스크립트로 읽는다

---

## 3. 선행 문제: fixed base override

**문제**: import 된 로봇은 `ArticulationRootAPI` 가 `robot_mount` 에 있고, `root_joint`(최상위 prim ↔ `robot_mount`)가 articulation **바깥**에 있었다.
PhysX 가 로봇을 **floating base + 외부 구속**으로 처리 → 질량 행렬이 16×16(베이스 6 DOF 포함), 중력 토크 API 사용 불가.

**해결**: 씬에서 `ArticulationRootAPI` 를 **최상위 prim** 으로 옮김 → `fixed_base = True`, 질량 행렬 10×10, 루트 이동 0.000 mm.
- `test_scene.py` 의 `build_test_scene(base="fixed")` 가 적용. `--base floating` 이면 원본 구조 유지
- solver 반복 횟수 등 articulation 설정도 새 root(최상위 prim)에 적용
- 3단계 ROS 그래프: Publish Joint State `targetPrim` = 로봇 최상위 prim
- 8단계(흔들리는 베이스)에서는 다시 결정

---

## 4. 제어 방식: PD

```
토크 = K × (목표 각도 − 현재 각도) + D × (목표 각속도 − 현재 각속도)
     = K × (목표 − 현재) − D × 현재 각속도        ← 목표 각속도는 0 으로 사용
토크는 최대 힘에서 잘림
```

- **K (stiffness, Nm/rad)**: 스프링. 목표에서 벗어난 만큼 되돌리는 힘
- **D (damping, Nm·s/rad)**: 완충기. 움직이는 속도에 비례하는 브레이크
- PhysX drive 는 PD 를 물리 계산 안에서 함께 푼다(implicit) → K 가 수만이어도 안정
- **I 항 없음** → 지속 하중(중력)에는 "하중 토크 ÷ K" 만큼 처진 채 멈춤 → 그래서 K 를 크게 잡음
- 목표 각속도 0 → 움직이는 목표를 따라갈 때 damping 이 브레이크로 작용 → 추종 지연의 주요 원인
- 단위: 설정 파일·tensor API 는 **rad 기준**. USD 속성은 degree 기준 (**USD 값 × 180/π = tensor 값**)

**로봇 연구에서 PD 를 쓰는 이유**: 시뮬레이터가 PD 만 내장(implicit 로 안정), 접촉 시 I 항은 힘이 계속 커지다 풀리면 튐(windup),
정책이 매 스텝 상태를 보고 새 목표를 줘서 남은 오차를 보정, 중력은 I 항이 아니라 모델(보상)로 처리하는 게 정석.

---

## 5. 팔 (6 관절)

### 5-1. 사양 그대로 쓰는 값

`ur_description` 의 `config/ur5/joint_limits.yaml` (UR5 CB3 사양):

| 관절 | 최대 힘 | 최대 속도 |
|------|---------|-----------|
| shoulder_pan, shoulder_lift, elbow | 150 Nm | π rad/s (180°/s) |
| wrist_1, wrist_2, wrist_3 | 28 Nm | π rad/s (180°/s) |

armature 는 0 (UR 회전자 관성 비공개, 9단계 실물 응답으로 보정할 튜닝 파라미터).

### 5-2. 모델: 관절 하나 = 감쇠 진동

```
I × θ̈ + D × θ̇ + K × θ = K × 목표
표준형 θ̈ + 2ζω θ̇ + ω² θ = ω² 목표 와 비교:
  ω = √(K / I)             고유진동수 = 반응 속도
  ζ = D / (2 √(K × I))     감쇠비
```

| ζ | 이름 | 동작 |
|---|------|------|
| < 1 | 부족 감쇠 | 빠르지만 출렁이며 멈춤 (오버슈트) |
| **= 1** | **임계 감쇠** | **출렁임 없이 가장 빨리 멈춤** |
| > 1 | 과감쇠 | 출렁임 없지만 느림 |

임계 감쇠의 2% 정착 시간 ≈ 5.8 / ω. 부족 감쇠 오버슈트 = e^(−πζ/√(1−ζ²)).

### 5-3. I (유효 관성)

- 관절 공간 **질량 행렬 M(q) 의 대각 성분** = 다른 관절을 고정하고 그 관절만 돌릴 때의 관성
- 손끝 쪽 부품(마운트, 브래킷, 카메라, 그리퍼)은 로봇 USD 에 있어 **자동 포함**. **드론(1.5 kg, TCP)을 단 경우도 계산에 포함**
  (그리퍼 base 링크 질량·무게중심을 메모리에서 바꿔 질량 행렬을 다시 읽음)
- 자세에 따라 변함 → **무작위 자세 약 300 개**에서 `get_mass_matrices` 로 읽고, 드론 없음·있음 두 경우 중 관절별 **최대값 I_max** 사용
  (예: shoulder_lift I_max 5.73 은 드론 포함 값, 드론 없이는 4.16)
- 비대각 성분(관절끼리의 결합)은 식에 없음 → A 시험 "다른 관절 흔들림"으로 확인 (최대 약 0.1°)

### 5-4. K: 세 조건 중 가장 큰 값

| 조건 | 기준 | 필요한 K |
|------|------|----------|
| (a) 정착 시간 | ≤ 0.3 s → ω ≥ 19.4 rad/s | K ≥ I_max × ω² |
| (b) 추종 지연 | < 40 ms. 실측식 지연 ≈ 2.47/ω + 10.7 ms → **ω ≥ 100** | K ≥ I_max × ω² |
| (c) 중력 처짐 | ≤ 0.05° (= 0.000873 rad) | K ≥ 최악 중력 토크 ÷ 0.000873 |

(c) 의 최악 중력 토크: 가능한 자세 × **드론 1.5 kg** (TCP), 손목 관절은 드론 무게중심이 **공구 축에서 5 cm 벗어난 최악 방향**까지 포함.
현재 `drive_gains.yaml` 의 K 는 **불가능한(자기 충돌) 자세도 섞인 무작위 자세로 계산한 값**을 그대로 둔 것.
가능한 자세만으로 다시 계산해도 차이가 −0.6~+1.1% 라 바꾸지 않았다 (가능한 자세 기준으로 바꾼 것은 B 경우 파일뿐).

```
K = max(a, b, c)
D = 2 × ζ × √(K × I_max),  ζ = 1
```
I_max(드론 포함) 기준 → 드론을 잡은 가장 무거운 경우에 ζ = 1, 그 밖의 자세·빈손에서는 ζ > 1 (출렁임 없음).

### 5-5. 최종값

| 관절 | I_max (kg·m²) | 결정 조건 | K (Nm/rad) | D (Nm·s/rad) |
|------|------|------|------|------|
| shoulder_pan | 5.65 | (b) 추종 지연, ω = 100 | 56,502 | 1,130 |
| shoulder_lift | 5.73 | (c) 중력 처짐 | 85,488 | ≈ 1,400 |
| elbow | 1.44 | (c) 중력 처짐 | 33,469 | ≈ 440 |
| wrist_1 | 0.116 | (c) 중력 처짐 (5 cm 오프셋) | 6,782 | ≈ 56 |
| wrist_2 | 0.082 | (c) 중력 처짐 (5 cm 오프셋) | 4,849 | ≈ 40 |
| wrist_3 | 0.005 | (c) 중력 처짐 (5 cm 오프셋) | 903.6 | ≈ 4.3 |

- D 의 ≈ 값은 위 식으로 계산한 참고값. **정확한 값은 `drive_gains.yaml`**
- shoulder_pan 은 회전축이 수직이라 중력 부하 0 → 추종 지연 조건으로 정해짐 (처음 ω 70 에서 지연 42 ms → ω 100 으로 올려 34 ms)
- wrist_3 는 드론 무게중심이 축 위에 있다고 가정했을 때 K 60.5 로 너무 작았음 → 5 cm 오프셋 반영 후 903.6

### 5-6. 비교 후 버린 방식

| 방식 | 결과 | 버린 이유 |
|------|------|----------|
| 중력 보상 feedforward + ω 30 | B 0/12, C 0/6 | 로봇 자체 무게만 보상하고 **드론 무게는 보상 못 함** → 손목 최대 9° 처짐. 추종 지연 약 80 ms |
| Isaac Sim 공식 UR5 값 | B·C 통과, A 3/6 | K 는 충분하지만 **damping 부족**(shoulder_lift ζ ≈ 0.2) → 0.05 rad 스텝 오버슈트 최대 58% |

---

## 6. 그리퍼 (`rh_r1_joint`)

### 6-1. 흉내 내는 실물 동작

실물 RH-P12-RN: 전류 기반 위치 제어(Operating Mode 5), Goal Current 400 mA, Profile Velocity 1000 / Acceleration 300 (원시값, 단위 미확인).
- 24 V, 400 mA: 우리 전원 공급기 설정 전압과 전류 한계(그리퍼 노드 Goal Current 400 mA)
- 170 N, 3.33 A: RH-P12-RN 제조사 사양(최대 파지력 170 N, 피크 전류 3.33 A). ROBOTIS e-Manual 사양표와 판매처 사양표에 같은 값
1. 열기/닫기 명령 → **일정 속도**로 움직임 (측정: 완전 열림↔닫힘 약 2.2 s, 임시)
2. 물체에 막히면 **전류 한계**의 힘으로 계속 누름

### 6-2. 값

| 값 | 설정 | 근거 |
|----|------|------|
| 목표값 이동 속도 (`profile_velocity`) | **0.516 rad/s** | 1.1351 rad ÷ 2.2 s. 실물 측정 2.24 / 2.20 s (임시, 재측정 예정) |
| 최대 힘 | **2.28 Nm** (임시) | 400 mA → 제조사 사양(170 N / 3.33 A) 선형 환산 ≈ 20 N × 2 손가락 × 0.057 m. 170 N 이 손가락당인지 합계인지 미확인 → 1-6 당김 시험으로 보정 |
| K | **45.6 Nm/rad** | 막혔을 때 반드시 최대 힘으로 누르도록: K ≥ 최대 힘 ÷ 남은 최소 오차 = 2.28 ÷ 0.05 |
| D | **0.263 Nm·s/rad** | 임계 감쇠, I = mimic 으로 묶인 손가락 4 DOF 전체 관성 (3.78×10⁻⁴) |
| 최대 속도 | 6.5 rad/s | USD 원래 값 (속도 제한은 쓰지 않음, 아래 참고) |
| mimic 3 개 (`rh_r2`, `rh_l1`, `rh_l2`) | K = D = 0 | 목표값도 주지 않음 |

### 6-3. 목표값 이동 (`GripperProfile`, `robot_drive.py`)

- 명령이 오면 목표를 한 번에 바꾸지 않고 **매 스텝 0.516 rad/s 만큼 이동**. Dynamixel Profile Velocity 와 같은 원리
- 물체에 막혀도 목표는 끝(1.1351 rad)까지 계속 감 → 위치 오차 × K 가 최대 힘을 넘어 **최대 힘으로 계속 누름** (실물 Goal Current 와 같은 역할)
- **명령이 바뀌면(열기↔닫기) 현재 실제 손가락 위치에서 새로 출발.** 이전 목표에서 이어 가면, 잡은 상태에서 열 때 약 1 초 멈춰 있는 지연이 생김
- 같은 명령이 반복되면 재시작하지 않음 (텔레옵이 같은 명령을 계속 보내도 안전)
- 3단계 sim 그리퍼 브리지도 같은 함수를 사용
- 가속 구간은 지금은 없음. 실물 재측정과 차이가 크면 사다리꼴(가속-등속-감속) 검토

**관절 속도 제한을 쓰지 않는 이유**: `rh_r1_joint` 에 속도 제한(0.516 rad/s)을 걸면 mimic 과 함께 약 3 Nm·s/rad 의 점성 저항처럼 동작해서 2.28 Nm 로는 열리지 못했다.

---

## 7. 공통 설정

| 항목 | 값 |
|------|------|
| physics 스텝 | 1/120 s (관례적 시작값. 3단계에서 카메라 주기와 함께 재검토) |
| articulation solver 반복 (위치 / 속도) | 32 / 4 |
| `sleep_threshold` | **0** (팔이 멈춘 채 그리퍼만 움직이면 PhysX 가 articulation 을 재워 그리퍼가 멈추던 문제 방지) |
| self-collision | **켬** |
| `collision_filter_pairs` | `[]` (조인트로 직접 연결된 쌍은 PhysX 가 기본으로 충돌 제외) |
| `collision_shapes.convex_decomposition` | `[forearm]` |
| 베이스 | fixed base override |
| `gravity_ff` | false |
| 시험 시작 자세 (홈) | `[0, −π/2, π/2, −π/2, −π/2, 0]` |

---

## 8. 시험 A~F

테스트 씬: physics scene + ground plane + 로봇(최상위 prim z = 0.762 m, 바닥 접촉 방지) + GUI 모드일 때 Dome Light.

| 시험 | 내용 | 기준 | 최종 결과 |
|------|------|------|----------|
| A 스텝 응답 | 관절 하나씩 0.05 / 0.2 / 1.0 rad. **판정은 0.05 rad** (50 Hz 텔레옵의 한 스텝 최대 변화 ≈ 0.063 rad) | 오버슈트 < 2%, 정착 < 0.5 s, 정상상태 오차 < 0.1° | 6/6. 오버슈트 최대 1.2%, 정착 0.02~0.08 s, 다른 관절 흔들림 ≤ 0.13° |
| B 중력 유지 | 최악 자세(가능한 자세 기준), 드론 1.5 kg, 무게중심 3·5 cm 오프셋. **외력 방식**(무게중심 위치에 14.7 N) | 오차 < 0.1° | 12/12. 최대 0.067° (shoulder_lift, τ 75.4 Nm) |
| C 텔레옵식 추종 | 50 Hz 계단 명령, 0.5 Hz 사인파, 진폭 0.3 rad | 지연 < 40 ms | 6/6. 지연 19~34 ms, RMS 0.7~1.3° |
| D 그리퍼 | 닫기, 열기, 막힌 상태(상한 0.6 rad)에서 열기 | 1.98~2.42 s, 열기 반응 < 0.1 s | 닫힘 2.13 s, 열림 2.13 s, 반응 0.017 s |
| E 가만히 | 홈 자세 3 초 | 떨림 < 0.01°, NaN 없음 | 떨림 0.00001°, 최대 벗어남 0.033° (중력 처짐) |
| F 빈손 닫기 | 끝까지 닫고 2 초 | 떨림 < 0.01° | 64.233° 에서 손가락끼리 접촉해 멈춤, 떨림 0.00003° |

- **"완전 닫힘" 정의**: 명령 순간부터 위치 변화가 멈출 때(최종값의 2% 이내)까지. sim 과 실물 재측정 모두 같은 정의
- A 큰 스텝(참고): shoulder_lift 0.2 rad 에서 오버슈트 13.6%. **토크 포화**가 원인 (K 가 커서 최대 속도로 가속, 멈출 때는 150 Nm − 중력으로만 감속). 게인으로는 잘 안 고쳐짐 → 명령 제한기로 대응
- B 외력 방식: 가만히 버티는 상황에서는 매달린 물체와 물리적으로 같음(무게중심에 m·g). 움직일 때의 관성(m·a, 회전)은 흉내 못 함 → 1-6·2단계에서 실제 물체로 확인. 씬을 한 번만 만들어 몇 분 → 약 33 초

---

## 9. 자기 충돌 (self-collision)

**켠 이유**: 끈 상태에서는 팔끼리, 그리퍼와 팔끼리, 손가락끼리 서로 통과했다. 켜면 실물에서 부딪히는 동작을 sim 도 막는다.

**켠 결과**: D·E·F·A·C 결과가 끈 상태와 같거나 더 현실적(F: 손가락 64.233° 에서 접촉), 계산 시간 +1%.

**불가능한 자세 정리**: 무작위 관절 자세 중 상당수가 자기 몸을 파고드는 자세였다 → B 경우와 K 계산을 가능한 자세만으로 다시 함 (K 변화 1% 안팎, B 경우 파일만 교체).

**가짜 충돌 수정 (forearm)**
- PhysX 는 메시 충돌 형상을 **볼록 덩어리(convex hull)** 로 근사 → forearm 의 오목한 부분이 메워져, 실물에서는 wrist_2 가 들어갈 공간이 sim 에서 막힘
- forearm 한 링크만 **convex decomposition** (여러 볼록 조각)으로 씬에서 변경 (instance 를 풀어 적용, 원본 USD 그대로)

| | decomposition 끔 | 켬 |
|------|------|------|
| 실물에서 가능한데 막히는 자세 (PhysX 기준) | 60 | **2** (남은 2 개는 forearm 과 무관) |
| 놓치는 실제 겹침 | 거의 없음 | 8 (모두 forearm 쌍, 원래 메시 깊이 **최대 1.31 mm**, 대부분 0.00 mm = 스치는 수준) |
| 떨림 | 없음 | 없음 |
| 계산 시간 | 기준 | +3~5% |

→ 판단 기준(놓치는 겹침 모두 3 mm 이하) 충족, **켬 확정**. 해상도는 올리지 않음.
base–upper_arm 판정 불가(베이스 메시가 닫혀 있지 않음) 약 190 개는 공중 드론 파지에서 거의 안 쓰는 자세라 보류.

**부품별 충돌 형상 (현재)**

| 부품 | 충돌 형상 |
|------|------|
| UR5 forearm | convex decomposition (씬에서 변경) |
| UR5 나머지 링크 | `ur_description` 충돌 전용 메시 → convex hull |
| 그리퍼 본체·손가락 | ROBOTIS STL → convex hull |
| 카메라 마운트, 브래킷 | 원판 + 박스 (xacro 에서 직접 정의) |
| D435i | 박스 1 개 (realsense2_description). 충돌 박스가 본체 중심이 아니라 깊이 센서 원점(camera_link) 기준으로 놓여, 공구 방향 13.2~38.3 mm (실제 본체 5.0~30.0 mm) → **보는 방향으로 약 8.2 mm 밀림**. 사전 점검의 카메라 박스–마운트 간격 8.22 mm 와 일치 (빌드된 URDF 에서 확인: 박스 13.22~38.28 mm, 본체 5.00~30.05 mm, wrist_mount 기준) |

GUI 에서 충돌 형상 보기: 뷰포트 왼쪽 위 눈 아이콘 → Show By Type → Physics → Colliders → All.

---

## 10. 시험 중 발견하고 고친 문제

| 문제 | 원인 | 조치 |
|------|------|------|
| 중력 토크·질량 행렬이 이상함 | floating base + 외부 구속으로 처리됨 | fixed base override (3 절) |
| [7] 3 초 시험에서 팔이 거의 안 처짐 | 로봇이 z = 0 이라 팔이 바닥에 걸침 | 테스트 씬 로봇 z = 0.762 m |
| 그리퍼가 중간에 멈춤 | PhysX 가 articulation 을 재움 | `sleep_threshold: 0` |
| B 에서 중력 보상이 드론 무게까지 보상 | 시험용 드론 강체가 articulation 링크로 흡수됨 | `excludeFromArticulation` (이후 외력 방식으로 교체) |
| 그리퍼가 열리지 못함 | 관절 속도 제한 + mimic → 점성 저항 | 목표값 이동 방식 |
| 잡은 상태에서 열 때 약 1 초 지연 | 목표값이 이전 목표에서 이어 감 | 명령이 바뀌면 현재 위치에서 재출발 |
| 손목 K 가 너무 작음 | 드론 무게중심이 축 위라는 가정 | 5 cm 오프셋 최악 방향으로 재계산 |
| 같은 설정이 두 번 돌아 결과 덮어씀 | `--configs` 상대경로와 절대경로 불일치 | 경로를 절대경로로 통일 |
| FAIL 인데 종료 코드 0 | `simulation_app.close()` 가 먼저 종료 | `close(exit_code=...)` |
| 로봇 위치 지정 실패 | 최상위 prim 에 `xformOp:translate` 없음 | `XformPrim(..., reset_xform_op_properties=True)` |
| GUI 뷰포트가 비어 보임 | 테스트 씬에 조명 없음 | GUI 모드에서 Dome Light 자동 추가 |

---

## 11. 알려진 위험: 높은 K

1. **충돌 시 최대 토크로 밀어붙임**: 드론·테이블·자기 몸에 닿으면 위치 오차 × K 가 곧바로 최대 힘(150/28 Nm)까지 올라감. 실물 UR 의 보호 정지(protective stop)가 sim 에는 없음
   → 2·3단계에서 관절 토크·접촉력이 기준을 넘으면 팔 목표를 현재 위치에 고정해 멈추고, 에피소드 메타데이터에 `protective_stop` 기록. UR5(CB3) 보호 정지 기준값은 UR 문서에서 확인 필요
2. **큰 명령에 오버슈트**: 명령 제한기(한 스텝 변화량을 속도·가속도 한계로 제한), 부드러운 홈 복귀로 대응. 정책 평가(7단계)의 action chunk 경계에서 목표가 튀는 경우도 포함
3. **physics 주기·엔진 변경 시 불안정 가능**: ω·dt 가 1~3.5 인 관절이 있음 → physics 주기, 엔진(PhysX ↔ Newton), solver 반복 횟수를 바꾸면 A~F 와 `check_articulation` 재실행. 물리 엔진은 항상 PhysX 로 명시

---

## 12. 남은 일

| 항목 | 시점 |
|------|------|
| 그리퍼 동작 시간 실물 재측정 (같은 "2% 이내" 정의) → `profile_velocity` 보정 | 실물 가능할 때 |
| 그리퍼 최대 힘 보정: 당김 시험 (0.8 kg 은 400 mA 로 반드시 성공, 1.5 kg 은 필요 전류 확인) | 1-6 |
| 명령 제한기 구현과 A 큰 스텝 재시험 | 1-5 후속 |
| 보호 정지 흉내 | 2·3단계 |
| physics 주기 확정 (카메라 발행 주기와 정수배), RTF 측정, CPU 성능 모드 (`powerprofilesctl set performance`) | 3단계 |
| 드론 질량·무게중심 도메인 랜덤화 | 6단계 |
| 실물 UR5 에 같은 궤적을 보내 응답 비교 → K·D·armature 보정 (시스템 식별), drive gain ±20% 랜덤화 검토 | 9단계 |
| 마운트·브래킷 질량 실측 (현재 0.05 / 0.08 kg 추정) | 다음 재import 때 |

---

## 13. 실행 명령

`cd ~/data_collection_ur5_gripper` 후 실행. ROS 를 source 하지 않은 터미널을 **권장(예방)**.
source 한 상태에서 문제가 생기는지는 확인하지 않음. 3단계 ROS 브리지에서는 별도로 확인.

```bash
# 구조 회귀 검사 (로봇 USD 재import 후 항상)
~/isaacsim/python.sh isaacsim/scripts/check_articulation.py --headless

# 시험 (headless, 숫자만)
~/isaacsim/python.sh isaacsim/scripts/tune_drives.py --headless --tests ACDEF --configs isaacsim/config/drive_gains.yaml
~/isaacsim/python.sh isaacsim/scripts/tune_drives.py --headless --tests B     --configs isaacsim/config/drive_gains.yaml

# 시험 (GUI, 실제 시간 속도로 보기)
~/isaacsim/python.sh isaacsim/scripts/tune_drives.py --tests ACDEF --configs isaacsim/config/drive_gains.yaml --realtime
~/isaacsim/python.sh isaacsim/scripts/tune_drives.py --tests B     --configs isaacsim/config/drive_gains.yaml --realtime

# 설정 값 하나만 바꿔서 비교 (설정 파일은 안 바뀜)
~/isaacsim/python.sh isaacsim/scripts/tune_drives.py --headless --tests DEF \
  --configs isaacsim/config/drive_gains.yaml "isaacsim/config/drive_gains.yaml@articulation.self_collision=false" \
  --d-configs isaacsim/config/drive_gains.yaml "isaacsim/config/drive_gains.yaml@articulation.self_collision=false"

# 옵션 확인
~/isaacsim/python.sh isaacsim/scripts/tune_drives.py --help
```

- **D·E·F 는 `--d-configs`, A·B·C 는 `--configs`** 를 읽음. 비교할 때는 시험에 맞는 쪽을 지정
  (위 `--tests DEF` 예시에서 `--configs` 는 쓰이지 않지만 넣어도 문제없음)
- 리포트: `isaacsim/reports/tune_drives_<날짜시간>.txt`, 마지막 `[계산 시간]` 항목에 스텝당 ms

---

## 14. 용어

| 용어 | 뜻 |
|------|------|
| K (stiffness) | P 게인. 목표에서 1 rad 벗어났을 때 되돌리는 토크 |
| D (damping) | D 게인. 1 rad/s 로 움직일 때 거는 브레이크 토크 |
| I (유효 관성) | 그 관절만 돌릴 때 느껴지는 회전 관성. 질량 행렬 대각 성분 |
| ω (고유진동수) | √(K/I). 반응 속도. 명령이 아니라 K·D 를 정하는 설계 기준 |
| ζ (감쇠비) | D / (2√(K·I)). 1 이면 출렁임 없이 가장 빨리 멈춤 |
| 추종 지연 | 계속 움직이는 명령을 팔이 뒤처져 따라가는 시간 |
| 토크 포화 | K × 오차가 최대 힘을 넘어 토크가 잘리는 상태 |
| implicit drive | PD 를 물리 계산 안에서 함께 푸는 방식. 높은 게인에서도 안정 |
| mimic | 한 조인트(`rh_r1_joint`)를 다른 조인트들이 같은 각도로 따라가게 묶는 제약 |
| convex hull / decomposition | 충돌 형상을 볼록 덩어리 1 개 / 여러 볼록 조각으로 근사 |
| fixed base | 로봇 루트가 고정된 articulation. 실물 UR5(볼트 고정)와 같음 |
