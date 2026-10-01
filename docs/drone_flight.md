# 2-2. 드론 비행 정리 (Pegasus 방식)

작성: 2026-10-01 · 대상 커밋: `4c6932d` (브랜치 `isaacsim_v6.1.0`)

2단계 씬의 드론은 **로터 4 개의 추력으로 실제 쿼드콥터처럼 난다.** 몸체를 손으로 붙잡아 두는 가상의 힘이 아니라,
제어기가 정한 로터 회전속도로 추력을 만들고 그 추력으로 몸체를 기울여 움직인다.
구조·식·게인은 **Pegasus Simulator 의 Iris 예제를 그대로 옮겼고**, 우리 기체(Iris 0.75 배, 0.8 kg)에 맞게 게인만 환산했다.

---

## 1. 한눈에 보기

| 항목 | 내용 |
|------|------|
| 출처 | Pegasus Simulator (6.0.1 포크) — Iris 기체 모델 + Python 제어 예제 `NonlinearController` |
| 기체 | 3DR Iris (PX4 Gazebo 기본 기체) 외형 **0.75 배**, 질량 **0.8 kg** |
| 추력 | 로터마다 $F_i = k\,\omega_i^2$ 를 로터 강체의 z 축 방향으로 (PhysX 외력) |
| 제어기 | **기하 제어기** (Mellinger & Kumar 2011) — 위치 PID → 원하는 힘 → 몸체 자세 → 토크 |
| 할당 | (총추력, 토크 3) → 할당 행렬 역행렬 → 로터 4 개의 $\omega_i^2$ |
| 게인 | Pegasus 값을 **우리 기체 질량·관성 비율로 환산** (Pegasus Iris 와 같은 응답) |
| 지연 | Pegasus 처럼 **한 physics step 지연** (지난 step 에 계산한 ω 를 이번 step 에 적용) |
| 모드 | `static` (제자리), `hover` (약한 흔들림, seed), `trajectory` (미구현) |
| 모터 정지 | `release()` → ω = 0 → 추력 0 → 자유 낙하 (실물: 잡은 뒤 모터를 멈춤) |
| 프로펠러 회전 | 보여 주기용, 추력과 분리 (`--prop-spin on`) |
| 다음 단계 | 제어기 자리만 **PX4 SITL** 로 바꾼다 (로터 추력 부분은 그대로) |

---

## 2. 출처

### 2-1. 코드
- Pegasus Simulator: https://github.com/PegasusSimulator/PegasusSimulator (BSD-3, Marcelo Jacinto)
- 실제로 읽고 옮긴 버전: Isaac Sim 6.0.1 포크 https://github.com/livealive7/PegasusSimulator , commit `b1256ca`

| Pegasus 파일 | 옮긴 내용 |
|---|---|
| `logic/thrusters/quadratic_thrust_curve.py` | 이차 추력 모델, 로터 상수, 반토크 계수, 회전 방향, 최대 ω |
| `logic/dynamics/linear_drag.py` | 선형 공기저항 |
| `logic/vehicles/multirotor.py` | 매 step 순서 (ω → 힘 적용 → 제어기 갱신), 할당 행렬 `force_and_torques_to_velocities`, 프로펠러 표시 `handle_propeller_visual` |
| `logic/vehicles/vehicle.py` | 힘·토크를 로터·몸체 강체에 거는 방식 (`apply_force`, `apply_torque`) |
| `logic/vehicles/multirotors/iris.py` | Iris 설정 (추력 모델, 공기저항 계수) |
| `examples/utils/nonlinear_controller.py` | 기하 제어기 식과 게인 (Kp 10, Kd 8.5, Ki 1.5, Kr 3.5, Kw 0.5, m 1.5) |
| `assets/Robots/Iris/iris.usd` | 기체 외형·충돌 형상 (→ `isaacsim/assets/drones/iris/`) |

### 2-2. 논문 (Pegasus 제어기가 인용)
1. D. Mellinger and V. Kumar, "Minimum snap trajectory generation and control for quadrotors," *ICRA 2011*, pp. 2520-2525, doi:10.1109/ICRA.2011.5980409
   — 기하 제어기 식, 목표 각속도(식 (7))
2. J. Pinto, B. J. Guerreiro and R. Cunha, "Planning Parcel Relay Manoeuvres for Quadrotors," *ICUAS 2021*, pp. 137-145, doi:10.1109/ICUAS51884.2021.9476757

---

## 3. 기체

| 항목 | 값 | 근거 |
|------|----|------|
| 외형 | Pegasus `iris.usd` 를 **0.75 배** | 원본 허리 폭 105 mm 는 그리퍼 열림(107 mm)으로 못 잡음 → 81 mm |
| 질량 | **0.8 kg** (body 0.78 + 프로펠러 0.005 × 4) | 실물 드론 무게. 원본 body 1.5 kg 은 씬에서 덮어씀 |
| 관성 (전체 무게중심 둘레, PhysX) | $I = \mathrm{diag}(0.005365,\ 0.003242,\ 0.008194)$ kg·m² | 링크 관성 + 평행축 정리 |
| 로터 위치 (드론 좌표계, mm) | rotor0 (103.5, −155.0, 17.3), rotor1 (−93.8, 164.1, 17.3), rotor2 (103.5, 151.9, 17.3), rotor3 (−93.1, −166.8, 17.3) | PhysX 에서 읽음 (매 step) |
| 회전 방향 $d_i$ | [−1, −1, +1, +1] | Pegasus Iris (rotor0·1, rotor2·3 이 대각선 쌍) |

좌표계는 Pegasus 와 같은 **FLU** (x 앞, y 왼쪽, z 위)·world **ENU** 이다.
원본 USD 는 수정하지 않고 축소·질량·프로펠러 관절 고정·초기 속도 0 은 씬 레이어에서 덮어쓴다 (`isaacsim/config/drone_iris.yaml`, `scripts/drone.py`).

---

## 4. 매 physics step 에 하는 일

physics 주기 $\Delta t = 1/120$ s. `PHYSICS_PRE_STEP` 콜백 (`DroneFlight._step`):

```
1. 상태 읽기: body 위치 p, 자세 R, 속도 v, 각속도 ω_world, 로터 위치 (PhysX)
2. 지난 step 에 제어기가 낸 ω_cmd 를 이번 ω 로 적용          ← Pegasus 와 같은 한 step 지연
3. 추력 Fᵢ = k ωᵢ²  → 로터 i 강체에, 그 로터의 z 축 방향으로
   반토크 τ_z = Σ c ωᵢ² dᵢ  +  공기저항 F_d  → body 에
4. (프로펠러 표시가 켜져 있으면) 프로펠러 관절 속도 덮어쓰기
5. 모터가 켜져 있으면 제어기 갱신 → 다음 step 의 ω_cmd
```

---

## 5. 추력 모델 (Pegasus `QuadraticThrustCurve`, `LinearDrag`)

로터 $i$ 의 추력과 몸체 반토크:

$$
F_i = k\,\omega_i^2, \qquad
\tau_z = \sum_{i=1}^{4} c\,\omega_i^2\,d_i, \qquad
0 \le \omega_i \le \omega_{\max}
$$

선형 공기저항 (몸체 좌표계 속도에 비례):

$$
\mathbf{F}_d = -R\,D\,R^\top \mathbf{v}, \qquad D = \mathrm{diag}(0.50,\ 0.30,\ 0)\ \mathrm{N\cdot s/m}
$$

| 기호 | 값 | 의미 |
|------|----|------|
| $k$ | $8.54858 \times 10^{-6}$ N/(rad/s)² | 로터 상수 |
| $c$ | $1.0 \times 10^{-6}$ N·m/(rad/s)² | 반토크 계수 |
| $\omega_{\max}$ | 1100 rad/s | 로터당 최대 추력 $k\,\omega_{\max}^2 = 10.3$ N (0.8 kg 무게 7.85 N 의 5.3 배) |

정지 호버에 필요한 로터 각속도 (네 로터 균등):

$$
\omega_{\text{hover}} = \sqrt{\frac{m g}{4k}} = \sqrt{\frac{0.8 \times 9.81}{4 \times 8.54858\times10^{-6}}} \approx 479\ \mathrm{rad/s}
$$

PhysX 에는 추력을 world 좌표로 바꿔 로터 위치에서 건다 ($\mathbf{F}_i^{w} = F_i\,R_i\,\hat{z}$, $R_i$ = 로터 자세).
Pegasus 는 같은 힘을 로터 local 좌표계에 건다. 결과는 같다.

---

## 6. 기하 제어기 (Pegasus `NonlinearController.update`, Mellinger & Kumar 2011)

### 6-1. 목표 궤적 (`Reference`)
- `static`: $\mathbf{p}_r = \mathbf{p}_0$, $\mathbf{v}_r = \mathbf{a}_r = \mathbf{j}_r = 0$
- `hover`: 축마다 사인파 $n$ 개의 합 ($n = 2$, 진폭 합 = $[20, 20, 10]$ mm, 주파수 $f \sim U(0.2, 0.5)$ Hz, 위상 $\varphi \sim U(0, 2\pi)$, `--seed` 로 뽑아 기록).
  시작 순간 목표가 $\mathbf{p}_0$ 이 되도록 $\sin\varphi$ 를 뺀다:

$$
p_r(t) = p_0 + \sum_{j=1}^{n} A_j\left[\sin(2\pi f_j t + \varphi_j) - \sin\varphi_j\right]
$$

  속도·가속도·jerk 는 이것을 시간으로 1·2·3 번 미분한 해석식.

### 6-2. 위치 → 원하는 힘

$$
\mathbf{e}_p = \mathbf{p} - \mathbf{p}_r, \qquad \mathbf{e}_v = \mathbf{v} - \mathbf{v}_r, \qquad \mathbf{e}_i = \int \mathbf{e}_p\,dt
$$

$$
\mathbf{F}_{des} = -K_p\,\mathbf{e}_p - K_d\,\mathbf{e}_v - K_i\,\mathbf{e}_i + m g\,\hat{z} + m\,\mathbf{a}_r
$$

### 6-3. 총추력과 목표 자세
쿼드콥터는 몸체 z 축 방향으로만 추력을 낼 수 있으므로, 원하는 힘 방향으로 **몸체를 기울인다**:

$$
u_1 = \mathbf{F}_{des} \cdot \mathbf{z}_B \qquad (\mathbf{z}_B = R\,\hat{z}, \text{ 현재 몸체 z 축})
$$

$$
\mathbf{z}_{B,des} = \frac{\mathbf{F}_{des}}{\lVert \mathbf{F}_{des} \rVert}, \quad
\mathbf{x}_{C} = [\cos\psi_r,\ \sin\psi_r,\ 0]^\top, \quad
\mathbf{y}_{B,des} = \frac{\mathbf{z}_{B,des} \times \mathbf{x}_C}{\lVert \mathbf{z}_{B,des} \times \mathbf{x}_C \rVert}, \quad
\mathbf{x}_{B,des} = \mathbf{y}_{B,des} \times \mathbf{z}_{B,des}
$$

$$
R_{des} = [\,\mathbf{x}_{B,des}\ \ \mathbf{y}_{B,des}\ \ \mathbf{z}_{B,des}\,]
$$

목표 yaw $\psi_r$ 는 모터를 켤 때(`arm()`)의 yaw 로 고정.

### 6-4. 자세 오차와 토크

$$
\mathbf{e}_R = \tfrac{1}{2}\left(R_{des}^\top R - R^\top R_{des}\right)^{\vee}
$$

목표 각속도 (Mellinger & Kumar 식 (7), 목표 jerk 를 몸체 x–y 평면에 투영, yaw rate 목표 0):

$$
\mathbf{h}_\omega = \frac{m}{u_1}\left(\mathbf{j}_r - (\mathbf{z}_{B,des}\cdot\mathbf{j}_r)\,\mathbf{z}_{B,des}\right), \qquad
\boldsymbol{\omega}_{des} = \left[-\mathbf{h}_\omega\cdot\mathbf{y}_{B,des},\ \ \mathbf{h}_\omega\cdot\mathbf{x}_{B,des},\ \ 0\right]
$$

$$
\mathbf{e}_\omega = \boldsymbol{\omega}_B - \boldsymbol{\omega}_{des}, \qquad
\boldsymbol{\tau} = -K_R\,\mathbf{e}_R - K_\omega\,\mathbf{e}_\omega
$$

($\boldsymbol{\omega}_B = R^\top \boldsymbol{\omega}_{world}$, 몸체 좌표계 각속도. $(\cdot)^{\vee}$ 는 반대칭 행렬 → 벡터)

---

## 7. 할당 (Pegasus `force_and_torques_to_velocities`)

로터 $i$ 의 body 좌표계 위치를 $(x_i, y_i)$ 라 하면 (매 step PhysX 에서 읽음):

$$
\begin{bmatrix} u_1 \\ \tau_x \\ \tau_y \\ \tau_z \end{bmatrix}
=
\underbrace{\begin{bmatrix}
k & k & k & k \\
k\,y_1 & k\,y_2 & k\,y_3 & k\,y_4 \\
-k\,x_1 & -k\,x_2 & -k\,x_3 & -k\,x_4 \\
c\,d_1 & c\,d_2 & c\,d_3 & c\,d_4
\end{bmatrix}}_{A}
\begin{bmatrix} \omega_1^2 \\ \omega_2^2 \\ \omega_3^2 \\ \omega_4^2 \end{bmatrix}
$$

$$
\boldsymbol{\omega}^2 = A^{+}\,[u_1,\ \boldsymbol{\tau}]^\top
\;\rightarrow\; \omega_i^2 \leftarrow \max(\omega_i^2, 0)
\;\rightarrow\; \text{가장 큰 } \omega_i^2 > \omega_{\max}^2 \text{ 이면 전부 같은 비율로 줄임}
\;\rightarrow\; \omega_i = \sqrt{\omega_i^2}
$$

($A^{+}$ 는 의사역행렬. 포화 때 비율을 유지해 자세 제어 방향이 바뀌지 않게 한다)

---

## 8. 게인

### 8-1. Pegasus 원래 값 (Iris 1.5 kg, $I_{ref} = \mathrm{diag}(0.029125,\ 0.029125,\ 0.055225)$)

| | $K_p$ | $K_d$ | $K_i$ | $K_R$ | $K_\omega$ |
|---|---|---|---|---|---|
| 세 축 | 10 | 8.5 | 1.5 | 3.5 | 0.5 |

### 8-2. 우리 기체로 환산 (`scale_to_airframe: true`)

$$
K_{p,d,i} = K^{\text{Pegasus}}_{p,d,i}\cdot\frac{m}{m_{ref}}, \qquad
K_{R,\omega} = K^{\text{Pegasus}}_{R,\omega}\cdot\frac{I}{I_{ref}} \ (\text{축별})
$$

| | $K_p$ | $K_d$ | $K_i$ | $K_R$ (x, y, z) | $K_\omega$ (x, y, z) |
|---|---|---|---|---|---|
| 환산 후 | 5.333 | 4.533 | 0.800 | 0.645, 0.390, 0.519 | 0.0921, 0.0557, 0.0742 |

같은 숫자를 쓰면 가벼운 기체에서는 반응이 훨씬 빨라져 Pegasus Iris 와 다르게 움직인다.
비율로 환산하면 **고유진동수·감쇠비가 Pegasus Iris 와 같다** (위치 $\omega_n = \sqrt{K_p/m} \approx 2.58$ rad/s, $\zeta \approx 1.10$).

**Pegasus 숫자를 그대로 넣어 본 결과 (채택 안 함, 2026-10-01)**: 관성이 6~9 배 작아 자세 감쇠가 너무 세짐
($K_\omega \Delta t / I$ 최대 1.29, 환산 후 0.14) → 로터 명령이 매 step 크게 흔들림.

| | 정지 호버 오차 | 기울기 | 로터 ω 평균 (호버 479 rad/s) |
|---|---|---|---|
| Pegasus 숫자 그대로 | 9.9 mm | 1.5° | 363 (떨림) |
| **환산 (채택)** | **0.56 mm** | **0°** | **478.9** |

---

## 9. 모터 정지와 프로펠러 표시

- **`release()`**: 모터 켜짐 플래그를 끄면 다음 제어기 출력이 ω = 0. 한 step 지연 때문에 1/120 s 동안은 지난 ω 가 적용되고 그 뒤 추력 0
  (공기저항은 계속). 실물 시나리오 "잡은 뒤 모터를 멈춤" 에 해당 → 드론 무게 전체가 그리퍼에 걸린다
- **프로펠러 회전** (`--prop-spin on`, Pegasus `handle_propeller_visual` 과 같음): 실제 ω 와 **무관**하게

$$
\dot{\theta}_{prop,i} =
\begin{cases}
100\,d_i & F_i \ge 0.1\ \mathrm{N} \\
5\,d_i & 0 < F_i < 0.1\ \mathrm{N} \\
0 & F_i = 0
\end{cases}
\ \ [\mathrm{rad/s}]
$$

  프로펠러 관절 속도를 매 step 덮어쓴다 (보여 주기용, 추력과 분리). `--prop-spin off` 면 프로펠러 관절을 FixedJoint 로 고정

---

## 10. Pegasus 와 같은 점·다른 점

| 항목 | Pegasus | 우리 | 비고 |
|------|---------|------|------|
| 추력·반토크·공기저항 식 | 위 식 | 같음 | |
| 할당 행렬·포화 | 위 식 | 같음 | |
| 제어 식 (목표 각속도 포함) | 위 식 | 같음 | |
| 한 step 지연 | 있음 | 있음 | |
| 프로펠러 표시 | 0.1 N 기준 100 / 5 / 0 | 같음 | |
| 게인 | m 1.5 고정, 세 축 같은 값 | 기체 비율로 환산 | 같은 응답을 내기 위해 |
| 질량·관성 | 1.5 kg (Iris) | PhysX 값 0.8 kg | 실물 드론 무게 |
| physics 주기 | 1/250 s | 1/120 s | 로봇 drive 튜닝 기준 (`docs/drive_tuning.md`) |
| yaw 목표 | 궤적 파일 | 시작 yaw 고정 | |
| 센서 (IMU·GPS 등) | 있음 | 없음 | 필요할 때 추가 |
| 제어기 교체 | PX4 / ROS 2 / Python backend | 지금은 Python 기하 제어기 | 2단계 2번에서 PX4 |

**목표 각속도(jerk 투영) 비교** — hover 추적 RMS: 0 배 0.56 mm, Pegasus 식 1.20 mm, −1 배 1.73 mm.
부호는 Pegasus 가 맞고 차이가 1 mm 미만이라, **오차가 조금 크더라도 Pegasus 와 같은 동작이 실제에 가깝다**는 판단(2026-10-01)으로 Pegasus 식을 유지.

---

## 11. 시험 결과 (`isaacsim/scripts/check_flight.py`)

| 시험 | 내용 | 결과 (프로펠러 표시 끔 / 켬) |
|------|------|------|
| F1 정지 호버 | 30 s, 처음 20 s 제외 | 위치 오차 최대 0.56 / 0.50 mm, 로터 ω 평균이 계산 호버 값과 0.03% |
| F2 계단 | 목표 +5 cm (x, 그다음 z), 각 12 s | overshoot 약 9.5~10.9%, 12 s 뒤 오차 1.2~1.4 mm, x 이동 중 기울기 1.5° (기울어서 이동) |
| F3 hover | 20 s, seed 0 | 추적 오차 RMS 1.19 / 0.96 mm, 기울기 최대 0.54° |
| F4 모터 정지 | release 후 0.15 s | 수직 가속도 −9.810 m/s² (= −g), ω 0, 프로펠러 정지 |

전부 PASS (4/4).

### 알아 둘 특성
- **위치 적분이 느리다** ($K_p/K_i \approx 6.7$ s): 무게중심이 0.14 mm 만 치우쳐도 자세 적분항이 없어 몸체가 약 0.16° 기운 채 버티며 옆으로 약 4 mm 밀리고,
  위치 적분이 수십 초에 걸쳐 없앤다. 계단 목표에서는 적분이 쌓여 약 10% overshoot 후 느리게 돌아온다. static·hover 에는 계단이 없어 영향이 작다
- **외란에 약하다**: 위치 게인 약 5.3 N/m → 1 N 으로 계속 밀면 약 19 cm 밀린다. 그리퍼로 중앙을 잡으면 양쪽 손가락 힘(약 27 N)이 서로 상쇄돼
  드론이 약 1.4 mm 만 움직이지만, 닫는 방향으로 10 mm 어긋나게 잡으면 먼저 닿은 손가락이 드론을 약 9 mm 민다 (`grasp_demo.py --case offset_y`)
- **시작 직후 약 7 mm 처짐**: 재생 시작 ~ 모터 켜기 사이에 자유 낙하하는 시간 때문

---

## 12. 파일과 실행

| 파일 | 역할 |
|------|------|
| `isaacsim/scripts/drone_flight.py` | `DroneFlight` (추력 모델·할당·한 step 지연·release·프로펠러 표시), `GeometricController`, `Reference` |
| `isaacsim/config/drone_iris.yaml` | `flight` 항목: 추력 모델 값, 게인(Pegasus 값 + 기준 질량·관성), hover 진폭·주파수 |
| `isaacsim/scripts/drone.py` | 드론 USD 를 씬에 넣고 덮어쓰기 (축소·질량·관절 고정, `prop_spin`) |
| `isaacsim/scripts/check_flight.py` | 시험 F1~F4, GUI 보기 `--view` |
| `isaacsim/assets/drones/iris/` | Iris USD 원본 (Pegasus, BSD-3), 출처 README |

```bash
# 시험 (프로펠러 표시 끔 / 켬)
~/isaacsim/python.sh ~/data_collection_ur5_gripper/isaacsim/scripts/check_flight.py --headless --prop-spin off
~/isaacsim/python.sh ~/data_collection_ur5_gripper/isaacsim/scripts/check_flight.py --headless --prop-spin on

# GUI: hover 비행 → 8 s 뒤 모터 정지 → 낙하 (실제 시간 속도)
~/isaacsim/python.sh ~/data_collection_ur5_gripper/isaacsim/scripts/check_flight.py --view --mode hover --prop-spin on --release-after 8

# 로봇·카메라와 함께
~/isaacsim/python.sh ~/data_collection_ur5_gripper/isaacsim/scripts/drone_scene.py --mode hover --prop-spin on
```
