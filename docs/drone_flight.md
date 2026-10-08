# 2-2. 드론 비행 정리 (Pegasus 방식, PX4 SITL)

작성: 2026-10-01 · 대상 커밋: `4c6932d` (브랜치 `isaacsim_v6.1.0`)
추가: 2026-10-02 — **13장 PX4 SITL** (실제 PX4 펌웨어가 드론을 날림, 2단계 2번)
추가: 2026-10-08 — **14장 PX4 내부 제어 식** (위치 → 속도 → 자세 → 각속도 → 할당, v1.16.0 소스)

2단계 씬의 드론은 **로터 4 개의 추력으로 실제 쿼드콥터처럼 난다.** 몸체를 손으로 붙잡아 두는 가상의 힘이 아니라,
제어기가 정한 로터 회전속도로 추력을 만들고 그 추력으로 몸체를 기울여 움직인다.
구조·식·게인은 **Pegasus Simulator 의 Iris 예제를 그대로 옮겼고**, 우리 기체(Iris 0.75 배, 0.8 kg)에 맞게 게인만 환산했다.
제어기는 두 가지다: **PX4 SITL**(13장, `flight.backend: px4`, **기본**, 2026-10-02)과 **기하 제어기**(1~12장, `--flight geometric`). 추력·반토크·공기저항·프로펠러 표시는 같은 코드.

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
| PX4 SITL | 제어기 자리를 **실제 PX4 v1.16.0** 으로 (13장, `--flight px4`). 로터 추력 부분은 그대로 |
| PX4 내부 식 | 목표 위치 → 모터 명령까지 PX4 안의 제어 식과 기본 게인 (14장) |

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
| 제어기 교체 | PX4 / ROS 2 / Python backend | Python 기하 제어기 + PX4 SITL | PX4 는 13장 |

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

---

## 13. PX4 SITL (2단계 2번, `--flight px4`)

기하 제어기는 정답 상태·잡음 없음·지연 없음이라 **잡힐 때 거의 반응하지 않는다** (실물은 제어기가 그리퍼와 싸움).
그래서 제어기 자리에 **실제 PX4 펌웨어**를 넣었다. 연결 방식은 Pegasus 의 PX4 backend 를 옮겼고, Pegasus 에 없는 사실성 요소를 더했다.

### 13-1. 구조

```
Isaac Sim (physics 120 Hz, lockstep)                         PX4-Autopilot v1.16.0 SITL (별도 프로세스)
  드론 실제 상태 ─ 가상 센서 ─────────── TCP 4560 ──────▶  EKF2 (IMU + 외부 위치)
    IMU·기압·지자기 (매 step), 모션캡처 (100 Hz, 20 ms)        위치 → 속도 → 자세 → 각속도 PID → 모터 출력 u (0~1)
  ω = 1000·u + 100 → 모터 1차 지연 → 추력 (1~9장 그대로) ◀──
                                             UDP 14540 ─── drone_cmd.PX4Commander (Offboard 위치 목표, arm, kill)
```

| 파일 | 역할 |
|---|---|
| `isaacsim/scripts/px4_bridge.py` | `PX4Launcher` (PX4 실행·종료, `PR_SET_PDEATHSIG`), `PX4Bridge` (MAVLink lockstep, 센서 송신, 모터 명령 → ω) |
| `isaacsim/scripts/px4_sensors.py` | 가상 센서: IMU, 기압, 지자기, GPS, 거리 센서, optical flow, 모션캡처 |
| `isaacsim/scripts/pegasus_geo_mag.py` | Pegasus `geo_mag_utils.py` 원본 복사 (지자기 표, 위경도 변환) |
| `isaacsim/scripts/drone_cmd.py` | `PX4Commander`: world ENU ↔ PX4 NED, Offboard 위치 목표 20 Hz, 모드·arm 재요청, kill |
| `isaacsim/scripts/drone_flight.py` | `flight.backend` 선택, PX4 기체 파라미터 자동값, 모터 지연, 하향 raycast, 센서용 속도 = 자세 차분 |
| `isaacsim/config/px4_sitl.yaml` | PX4 경로·포트, 위치 정보 방식(`mocap` / `flow` / `gps`)별 센서·PX4 파라미터, 센서 잡음 |
| `isaacsim/config/drone_iris.yaml` `px4` 절 | 기체별: airframe, 모터 명령 변환, 게인 환산, 모터 지연, 센서 덮어쓰기 |
| `isaacsim/config/drone_iris_pegasus.yaml` | 기준선: Pegasus Iris 원래 크기 1.52 kg + PX4 Iris 값 그대로 (Pegasus 조건 재현용) |
| `isaacsim/scripts/check_px4.py` | 비행 시험 X0~X4, GUI `--view` |

출처: livealive7/PegasusSimulator `b1256ca` `logic/backends/px4_mavlink_backend.py`, `tools/px4_launch_tool.py`, `logic/sensors/*.py` (BSD-3).
PX4: https://github.com/PX4/PX4-Autopilot `v1.16.0` (Pegasus 포크가 시험한 버전), `make px4_sitl_default none`, airframe `10015_gazebo-classic_iris`.

### 13-2. 매 physics step (Pegasus `PX4MavlinkBackend.update` 순서)
1. 지난 step 에 받은 모터 명령 → ω (모터 지연) → 추력 적용 (1~9장과 같은 코드)
2. 첫 모터 명령을 받은 뒤부터는 PX4 의 `HIL_ACTUATOR_CONTROLS` 가 올 때까지 기다림 (**lockstep**: sim 이 느려도 PX4 가 같이 기다림, 결과가 실행 속도와 무관)
3. heartbeat (실제 시간 1 Hz), `HIL_SENSOR` (이번 step 새 값만 비트), 거리·flow·모션캡처·GPS 메시지. 시각 = sim 시간 μs
4. 이번 상태로 센서 갱신 (다음 step 에 보냄 → 센서 한 step 지연, Pegasus 와 같음)

모터 명령 → ω: armed 이면 $\omega_i = 1000\,u_i + 100$ rad/s 를 $[0, \omega_{max}]$ 로 자름 (Pegasus 기본값), 아니면 0.
우리 호버 ω 479 rad/s → $u \approx 0.38$.

### 13-3. 기체에 맞춘 PX4 파라미터 (`drone_flight._px4_airframe_params`)
- `CA_ROTORi_PX/PY` = sim 로터 위치 (body 원점 기준 FRD, 0.75 배)
- **각속도 게인 환산** (기하 제어기 8-2 와 같은 문제): `MC_{ROLL,PITCH,YAW}RATE_K` (P·I·D 전체 배율) $= \dfrac{I/I_{ref}}{\text{팔}/\text{팔}_{ref}}$
  - 각가속도 = 토크 / I, 최대 토크 ∝ 팔 길이 (roll: 로터 |y| 평균, pitch: |x| 평균, yaw: 반토크라 팔 무관)
  - 기준 = P-2 에서 안정 확인한 조합: Gazebo Iris 관성 (0.029125, 0.029125, 0.055225) + Pegasus 추력 + Pegasus Iris 로터 위치 (|x| 0.131, |y| 0.213 m)
  - 결과 roll 0.246, pitch 0.148, yaw 0.148 → 호버 모터 ω 흔들림 0.5 rad/s
  - **환산하지 않으면**: Pegasus Iris(원래 크기)조차 모터 명령이 17.4 Hz 로 0 ↔ 0.9 포화 진동 (ω 표준편차 355 rad/s). PhysX 가
    충돌 형상으로 계산한 값(0.0175, 0.0107, 0.0268)이 PX4 Iris 게인이 맞춰진 Gazebo Iris 보다 1.7~2.7 배 작기 때문. 0.75 배는 6~9 배 작음
    - **[정정 2026-10-07]** 여기에 "`iris.usd` 관성이 비어" 라고 적혀 있었으나 사실이 아니다. `iris.usd` body 에는 `diagonalInertia = (0.029125, 0.029125, 0.055225)`
      (= Gazebo Iris 값) 가 저장돼 있고, `drone.add_drone` 이 `link_masses` 가 있으면 이를 지워 PhysX 가 형상에서 다시 계산하게 한다.
      즉 위 진동은 우리가 원본 관성을 지운 결과로 보인다 (지우지 않고 다시 돌려 보지는 않음, PLAN P-2 정정 참고)

### 13-4. Pegasus 에 없는 것 (사실성)
| 항목 | 내용 | 이유 |
|---|---|---|
| 모터 1차 지연 | $\omega \leftarrow \omega + (\omega_{cmd} - \omega)(1 - e^{-\Delta t/\tau})$, 올림 τ 0.0125 s, 내림 0.025 s (PX4 Gazebo `motor_model`) | 실물 모터는 즉시 바뀌지 않음. kill 뒤 125 ms 에 걸쳐 감속 |
| 센서 버그 수정 | `pegasus_compat: false`: 자이로 bias 시간상수, 가속도계 bias 적용, 기압 잡음 분포, GPS bias 이산화 | Pegasus 코드 오류 (기준선 설정만 compat) |
| 센서용 속도 = 자세 차분 | IMU·flow 에 주는 속도·각속도 = $(p_k - p_{k-1})/\Delta t$, $\log(R_{k-1}^T R_k)/\Delta t$ (`px4.sensor_kinematics: pose_difference`) | **그리퍼에 잡혀 접촉이 걸리면 PhysX 가 보고하는 각속도가 실제 자세 변화와 다름** (보고 평균 29 °/s, 실제 0.5 °/s). 이 값을 자이로로 쓰면 PX4 자세 추정이 1.5 s 에 15° 틀어지고 추정 발산 → 모터 포화 → 드론이 비틀려 빠짐. 실물 IMU 는 실제 움직임만 잼 |
| 실내 위치 정보 | 모션캡처 (13-5) | 실내 시연, GPS 없음 |
| Pegasus HIL_SENSOR 온도 인자 | 기압계 온도 (Pegasus 는 GPS 고도 mm 를 넣음, 인자 순서 오류) | |
| PX4 무응답 | 시간 제한 넘으면 에러 (Pegasus 는 무한 대기) | 조용한 실패 금지 |

### 13-5. 위치 정보 방식 (`px4_sitl.yaml` `position_source`, `--position-source`)
| 방식 | 센서 | PX4 EKF2 | 결과 |
|---|---|---|---|
| **mocap (기본)** | 실제 body 위치·자세 + 잡음 1 mm·0.3°, 100 Hz, 지연 20 ms → `VISION_POSITION_ESTIMATE` (카메라·마커 장면 없음, 정답값 + 잡음) | `EV_CTRL 11` (수평·수직 위치 + yaw), `HGT_REF 3`, `EV_DELAY 20`, 지자기·GPS 끔 | 호버 실제 − 목표 RMS 19 mm (PX4 위치 유지의 느린 흔들림), 추정 − 실제 RMS 6 mm |
| flow | 하향 거리 센서 (PhysX raycast, 50 Hz, 1 cm) + optical flow (50 Hz) | `OF_CTRL 1`, `RNG_CTRL 1`, `HGT_REF 0` (기압), `TERR_NOISE 0.1` | 이륙 표류 수 cm, 기압 고도 표류. **팔이 아래로 오면 드론이 위로 도망감** (아래) |
| gps | GPS + 기압 + 지자기 (Pegasus 기본) | PX4 기본 | 실외용. 고도 추정 오차 수 cm |

실물 모션캡처(Vicon·OptiTrack)는 잡음 0.1~0.5 mm, 0.1°, 지연 5~10 ms, 100~360 Hz → 지금 값은 약간 보수적. 마커 가림은 없음.

**optical flow 부호**: PX4 `EKF2.cpp` 가 pixel_flow·delta_angle 부호를 뒤집어 받으므로, 센서는 몸체 FRD 에서 pixel_flow 각속도
$= \omega_{xy} + (-v_y,\ v_x)/d$ 를 보낸다 (회전 +ω_x 는 화면을 +Y 로, 오른쪽 이동 +v_y 는 −Y 로). 반대로 넣으면 EKF 가 발산한다.

**flow 의 한계 (실물 위험, PLAN 기록)**: 그리퍼가 아래에서 올라오면 하향 거리 센서가 바닥 대신 팔을 잼 → PX4 가 "내려갔다"로 보고 상승 →
팔이 따라가면 계속 상승 (3.2 m 까지). 하향 센서 드론 아래에 손을 넣으면 올라가는 것과 같은 현상. 거리 센서를 끄면 flow 가 terrain 을 못 써 failsafe.
→ 실내 위치는 모션캡처로 얻을 수 있다고 가정 (2026-10-02 사용자 결정)

### 13-6. 시험 결과
**비행 (`check_px4.py`, 0.75 배 Iris 0.8 kg, 120 Hz, 모션캡처)** → 5/5

| 시험 | 결과 |
|---|---|
| X0 연결 | lockstep 시작 sim 약 1 s, arm 요청 1.5 s 뒤 armed, 실시간 비율 약 1.5 (headless) |
| X1 이륙 | 바닥 → 1 m 위, PX4 추정 5 cm 안 7.3 s. **뜨는 순간 앞으로 최대 약 200 mm 밀렸다가 3 s 안에 되돌아옴** (아래) |
| X2 호버 30 s | 실제 − 목표 RMS 19.5 mm (축별 표준편차 8~10 mm), 기울기 최대 0.23°, 모터 ω 흔들림 0.5 rad/s |
| X3 계단 +10 cm | overshoot 약 23%, 2 cm 안 정착 약 7 s |
| X4 kill | disarm 33 ms, 모터 감속 125 ms 뒤 −9.810 m/s² 낙하 |

이륙 이탈: Iris 메시 앞쪽 아래 부품이 다리 끝보다 13.5 mm(0.75 배 10 mm) 아래라 바닥에서 3.4° 숙여져 서고, 뜨는 순간 7° 까지 숙여지며 앞으로 가속.
PX4 공식 Gazebo Iris 는 충돌이 상자라 평평하게 선다. 충돌 메시를 고쳐 보았지만 원본 USD 의 미리 계산된 볼록 분해가 바뀌어 잡는 형상까지 달라져서 되돌림 (원래 모델 유지)

**파지 (`grasp_demo.py --flight px4`, 모션캡처)** → 6 케이스 6/6 (판정이 기대와 같음)
- 드론: 테이블 위 (1.2, 0) 에서 이륙 → 수직 상승 → (0.60, 0, 1.50) 수평 이동 → 5 s 호버 → 팔 시작
- 접근 중 드론 따라가기 (`approach_track_until` 3 cm): PX4 호버는 ±1~2 cm 로 움직여 목표를 미리 고정하면 손가락 끝이 몸체에 부딪힘

잡은 채 모터를 켜 두는 시간(`--kill-delay`)별 (success, 잡은 뒤 ~ kill 구간):

| 제어기 | kill 까지 | 결과 | 드론 흔들림 (TCP 기준) | 자세 변화 | 모터 ω 흔들림 | 모터 ω 끝−처음 | 손가락 힘 흔들림 |
|---|---|---|---|---|---|---|---|
| PX4 | 0.5 s | held | 0.03 mm | 0.02° | 0.4 rad/s | 1 rad/s | 0.1 N |
| PX4 | 1 s | held | 0.4 mm | 0.2° | 1.9 | 7 | 0.8~1.4 N |
| PX4 | 2 s | held | 0.4 mm | 0.5° | 1.0 | 2 | 0.7~1.0 N |
| PX4 | 5 s | held | 0.6 mm | 0.5° | 2.7 | 17 | 0.5~1.0 N |
| PX4 | 10 s | held | 0.5 mm | 0.6° | 2.9 | 21 | 0.3~1.1 N |
| 기하 | 1 s | held | 0.03 mm | 0.02° | 0.7 | 3 | 0.3 N |
| 기하 | 5 s | held | 0.03 mm | 0.01° | 0.3 | 4 | 0.01 N |

- PX4 는 잡힌 뒤 **천천히 적분이 쌓이며** 그리퍼와 약하게 싸움 (10 s 에 로터 간 ±20 rad/s), 손가락 힘이 기하보다 3~100 배 흔들림. 10 s 켜 둬도 놓치지 않음
- 모터 정지 뒤 내려앉음: PX4 약 1 mm vs 기하 약 6 mm. 최종으로 얹히는 높이는 같고, PX4 는 접근 목표 고정 뒤 호버 흔들림으로 드론이 약 8 mm 내려와
  이미 얹히는 높이 근처에서 잡혔기 때문 → PX4 에서는 잡는 높이가 실행마다 달라짐
- sim 에 없는 실물 요소: 프로펠러 바람이 그리퍼·팔에 부딪힘, 기체·프로펠러 진동, 모션캡처 가림 → 실물 떨림은 이보다 클 수 있음

### 13-7. 실행
```bash
# PX4 설치 (한 번): ~/PX4-Autopilot, v1.16.0, 빌드용 venv ~/PX4-Autopilot/.venv, make px4_sitl_default none (빌드 뒤 PX4 셸이 뜨면 끔)
#   pymavlink: ~/isaacsim/python.sh -m pip install --target isaacsim/.pydeps pymavlink

# 비행 시험 (0.75 배 Iris, 모션캡처)
~/isaacsim/python.sh ~/data_collection_ur5_gripper/isaacsim/scripts/check_px4.py --headless --drone-config ~/data_collection_ur5_gripper/isaacsim/config/drone_iris.yaml --start-z 0.08 --physics-hz 120
#   비교 옵션: --position-source mocap|flow|gps, --motor-lag on|off, --sensor-compat on|off, --px4-param NAME=VALUE
#   Pegasus 기준선: --drone-config .../drone_iris_pegasus.yaml --position-source gps --physics-hz 250 (--start-z 0.10)
# GUI 비행 보기
~/isaacsim/python.sh ~/data_collection_ur5_gripper/isaacsim/scripts/check_px4.py --view --drone-config ~/data_collection_ur5_gripper/isaacsim/config/drone_iris.yaml --start-z 0.08 --physics-hz 120 --prop-spin on

# 로봇 씬 + PX4 드론 (기본), 파지. 기하 제어기는 --flight geometric
~/isaacsim/python.sh ~/data_collection_ur5_gripper/isaacsim/scripts/drone_scene.py --prop-spin on
~/isaacsim/python.sh ~/data_collection_ur5_gripper/isaacsim/scripts/grasp_demo.py --case success [--kill-delay 10] --prop-spin on --hold
~/isaacsim/python.sh ~/data_collection_ur5_gripper/isaacsim/scripts/grasp_demo.py --headless --case all

# 이륙 뒤 들를 위치 (world m): 첫 점 높이까지 수직 상승 → 각 점 → --drone-pos (기본 0.60, 0, 1.50) 에서 호버
~/isaacsim/python.sh ~/data_collection_ur5_gripper/isaacsim/scripts/drone_scene.py --drone-waypoints 1.0,0.2,1.3 0.8,-0.1,1.6 --prop-spin on

# 다른 터미널에서 명령 (씬이 이륙·waypoint 를 마친 뒤 = status 의 ready true). 시스템 python3 로 실행
python3 ~/data_collection_ur5_gripper/isaacsim/scripts/drone_cmd.py status
python3 ~/data_collection_ur5_gripper/isaacsim/scripts/drone_cmd.py goto 0.65 0.05 1.45 --yaw-deg 20
python3 ~/data_collection_ur5_gripper/isaacsim/scripts/drone_cmd.py hold      # | land | kill (kill 은 언제나)
```
PX4 비행 로그(`.ulg`)·콘솔은 `isaacsim/reports/px4_<시각>/` (분석: pyulog). 스크립트가 어떻게 끝나도 PX4 는 같이 종료된다 (`PR_SET_PDEATHSIG`).

**다른 터미널 명령 (`drone_cmd.py` CLI)**: PX4 Offboard 는 위치 목표를 끊김 없이(2 Hz 이상) 받아야 유지되므로, 목표는 씬 안 `PX4Commander` 가 20 Hz 로 계속 보내고
CLI 는 UDP localhost `command_port` (14600) 로 JSON 명령을 보내 그 목표를 바꾼다 (실물에서 보조 컴퓨터가 setpoint 를 보내고 사람이 높은 수준 명령을 주는 구성).
씬이 이륙·waypoint 를 마치기 전에는 이동 명령 거부. CLI 로 goto·hold 하면 hover 모드 사인파 목표는 멈춤.
확인 (2026-10-02): 이륙 중 goto → 거부, waypoint 2 개 뒤 ready, `goto 0.65 0.05 1.45 --yaw-deg 20` → 4 s 뒤 추정 (0.654, 0.060, 1.465)


---

## 14. PX4 내부 제어 식 (v1.16.0 소스에서 읽음, 2026-10-08)

13장은 PX4 를 **연결하는 방법**이다. 이 장은 PX4 **안에서** 목표 위치가 모터 명령 $u$ 가 되기까지의 식이다.
`~/PX4-Autopilot` (v1.16.0) 소스를 읽고 옮겼다. 게인은 PX4 기본값이고 (airframe `10015_gazebo-classic_iris` 는 로터 위치·`CA_ROTORi_KM` 만 정함),
우리가 바꾸는 것은 `CA_ROTORi_PX/PY` 와 `MC_*RATE_K` 뿐이다 (13-3).
**PX4 를 돌려 로그로 확인한 내용이 아니라 소스를 읽은 내용이다.** 좌표계는 PX4 의 것: world **NED** (z 아래), 몸체 **FRD**.

### 14-1. 전체 흐름

```
PX4Commander (UDP 14540, 20 Hz)
  SET_POSITION_TARGET_LOCAL_NED: 위치 p_sp + yaw ψ_sp          (속도·가속도 목표는 보내지 않음)
        │
        ▼
[1] 위치 P            v_sp   = Kp_pos (p_sp − p)                               mc_pos_control
[2] 속도 PID          a_sp   = Kp_vel (v_sp − v) + ∫ − Kd_vel v̇
[3] 가속도 → 추력     T_sp (정규화 추력 벡터, 호버 = hover thrust)
[4] 추력 → 목표 자세  q_sp (몸체 z 축 = −T_sp 방향, yaw = ψ_sp),  총추력 |T_sp|
        │
        ▼
[5] 자세 P            Ω_sp   = Kp_att · 2 · Im(q⁻¹ q_sp)                       mc_att_control
        │
        ▼
[6] 각속도 PID        τ_sp   = K [ P (Ω_sp − Ω) + ∫ − D Ω̇ ]   (정규화 토크)    mc_rate_control
        │
        ▼
[7] 할당              u      = M [τ_sp ; T]  → 0~1 로 자름                      control_allocator
        │
        ▼
HIL_ACTUATOR_CONTROLS (TCP 4560)  →  sim: ω = 1000 u + 100  →  모터 지연  →  F = k ω²   (13-2, 5장)
```

상태 $p, v, \dot v, q, \Omega, \dot\Omega$ 는 정답값이 아니라 **EKF2 추정값** (sim 가상 센서로부터, 13-5) 이다.

| 단계 | 소스 파일 (`~/PX4-Autopilot/src/`) |
|---|---|
| 1~3 | `modules/mc_pos_control/PositionControl/PositionControl.cpp` (`_positionControl`, `_velocityControl`, `_accelerationControl`) |
| 4 | `modules/mc_pos_control/PositionControl/ControlMath.cpp` (`thrustToAttitude`, `bodyzToAttitude`, `limitTilt`) |
| 5 | `modules/mc_att_control/AttitudeControl/AttitudeControl.cpp` (`update`) |
| 6 | `lib/rate_control/rate_control.cpp` (`update`, `updateIntegral`), 게인 = `modules/mc_rate_control/MulticopterRateControl.cpp` |
| 7 | `modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessRotors.cpp`, `lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp`, `ControlAllocationSequentialDesaturation.cpp` |

### 14-2. 위치 → 속도 목표 (P)

$$
\mathbf{v}_{sp} = K_p^{pos} \odot (\mathbf{p}_{sp} - \mathbf{p}), \qquad K_p^{pos} = (0.95,\ 0.95,\ 1.0)\ \mathrm{s^{-1}}
$$

수평 크기는 `MPC_XY_VEL_MAX` 12 m/s, 수직은 위 3 m/s·아래 1.5 m/s 로 자른다.
위치 오차 1 cm → 속도 목표 약 1 cm/s. 적분항이 없다 (정상 상태 오차는 다음 단계의 속도 적분이 없앤다).

### 14-3. 속도 → 가속도 목표 (PID)

$$
\mathbf{a}_{sp} = K_p^{vel} \odot (\mathbf{v}_{sp} - \mathbf{v}) + \mathbf{i}_v - K_d^{vel} \odot \dot{\mathbf{v}},
\qquad \dot{\mathbf{i}}_v = K_i^{vel} \odot (\mathbf{v}_{sp} - \mathbf{v})
$$

| | 수평 (x, y) | 수직 (z) | 파라미터 |
|---|---|---|---|
| $K_p^{vel}$ | 1.8 | 4.0 | `MPC_XY_VEL_P_ACC`, `MPC_Z_VEL_P_ACC` |
| $K_i^{vel}$ | 0.4 | 2.0 | `MPC_XY_VEL_I_ACC`, `MPC_Z_VEL_I_ACC` |
| $K_d^{vel}$ | 0.2 | 0.0 | `MPC_XY_VEL_D_ACC`, `MPC_Z_VEL_D_ACC` |

- 미분항은 속도 오차의 미분이 아니라 **추정 가속도 $\dot{\mathbf v}$** 에 건다 (목표가 갑자기 바뀌어도 튀지 않음)
- 수직 적분은 $\pm g$ 로 제한. 추력이 한계에 닿으면 적분을 멈춘다 (수직), 수평은 포화된 만큼 적분을 되돌린다 (tracking anti-windup, 이득 $2 / K_p^{vel}$)
- 출력 단위가 m/s² 이다. 기하 제어기(6-2)는 같은 자리에서 힘 [N] 을 내고 질량을 곱한다. PX4 는 질량을 모르고 14-4 의 hover thrust 로 환산한다

### 14-4. 가속도 → 추력 벡터

PX4 는 추력을 뉴턴이 아니라 **0~1 정규화 값**으로 다룬다. "호버에 필요한 정규화 추력" $T_h$ (hover thrust) 하나로 가속도와 추력을 잇는다.

원하는 몸체 z 축 (FRD 라 **아래**를 향하는 단위 벡터, NED 성분. 추력은 그 반대 방향. `MPC_ACC_DECOUPLE 1` 기본):

$$
\hat{\mathbf{b}}_z = \frac{(-a_{sp,x},\ -a_{sp,y},\ g)}{\lVert\cdot\rVert}
\quad \text{(기울기는 } \texttt{MPC\_TILTMAX\_AIR}\ 45^\circ \text{ 로 제한)}
$$

수직 추력과 총추력:

$$
T_z = a_{sp,z}\,\frac{T_h}{g} - T_h, \qquad
T = \min\!\left(\frac{T_z}{\hat{\mathbf{b}}_z \cdot \hat{\mathbf{z}}},\ -T_{min}\right), \qquad
\mathbf{T}_{sp} = \hat{\mathbf{b}}_z\, T
$$

- $a_{sp,z} = 0$ 이면 $T_z = -T_h$ (위로 호버 추력). 기울어지면 $1/\cos$ 만큼 총추력을 늘려 수직 성분을 유지
- $T_{min}$ = `MPC_THR_MIN` 0.12, 최대 = `MPC_THR_MAX` 1.0. 포화 때는 수직을 먼저 지키고 수평에 `MPC_THR_XY_MARG` 0.3 만큼 남긴다
- $T_h$: `MPC_USE_HTE 1` (기본) 이면 PX4 의 hover thrust 추정기가 비행 중에 추정한다. 추정 전 초기값은 `MPC_THR_HOVER` 0.5.
  $T_h$ 가 바뀔 때 수직 적분을 같이 고쳐 추력이 튀지 않게 한다 (`updateHoverThrust`).
  **우리 모션캡처 설정에서는 이 추정기가 돌지 않아 $T_h$ 가 0.5 에 머문다** (14-10 B)
- **우리 sim 과의 차이**: PX4 는 추력 ∝ $u$ 로 가정한다 (`THR_MDL_FAC` 0 기본). sim 은 $F = k(1000u + 100)^2$ 이라 $u$ 에 대해 이차식이다.
  호버는 $u \approx 0.38$ (13-2). 이 차이가 비행에 주는 영향은 따로 재지 않았다

### 14-5. 추력 벡터 → 목표 자세

몸체 z 축을 $-\mathbf{T}_{sp}$ 방향으로 두고, yaw 목표 $\psi_{sp}$ 로 나머지 축을 정한다 (`bodyzToAttitude`):

$$
\hat{\mathbf{b}}_z = \frac{-\mathbf{T}_{sp}}{\lVert \mathbf{T}_{sp} \rVert}, \quad
\mathbf{y}_C = (-\sin\psi_{sp},\ \cos\psi_{sp},\ 0), \quad
\hat{\mathbf{b}}_x = \frac{\mathbf{y}_C \times \hat{\mathbf{b}}_z}{\lVert\cdot\rVert}, \quad
\hat{\mathbf{b}}_y = \hat{\mathbf{b}}_z \times \hat{\mathbf{b}}_x, \quad
R_{sp} = [\hat{\mathbf{b}}_x\ \hat{\mathbf{b}}_y\ \hat{\mathbf{b}}_z]
$$

총추력 명령은 $-\lVert \mathbf{T}_{sp} \rVert$ (몸체 z, FRD 라 음수 = 위). 기하 제어기 6-3 과 같은 구성이다 (좌표계와 부호만 다름).

### 14-6. 자세 → 각속도 목표 (P, quaternion)

$$
q_e = q^{-1} \otimes q_{sp}, \qquad
\boldsymbol{\Omega}_{sp} = K_p^{att} \odot \big(2\,\mathrm{sgn}(q_{e,w})\,\mathrm{Im}(q_e)\big), \qquad
K_p^{att} = (6.5,\ 6.5,\ 2.8)\ \mathrm{s^{-1}}
$$

- 오차 $2\,\mathrm{Im}(q_e) = 2 \sin(\alpha/2)\,\hat{\mathbf n}$ (회전축 × 각도에 가까운 값, 작은 각에서는 각도 [rad])
- **기울기(roll·pitch)를 yaw 보다 먼저 맞춘다**: 목표 자세를 "몸체 z 축만 맞추는 회전" 과 "남은 yaw 회전" 으로 나누고 yaw 쪽 각도에 `MC_YAW_WEIGHT` 0.4 를 곱한다
  (yaw 게인은 그만큼 나눠서 보정). 로터 반토크로 만드는 yaw 는 약해서 위치 제어에 필요한 기울기를 방해하지 않게 하려는 것
- 출력은 roll·pitch 220 °/s, yaw 200 °/s 로 자른다 (`MC_ROLLRATE_MAX` 등)
- 기하 제어기 6-4 는 자세 오차와 각속도 오차에서 **토크를 한 번에** 계산한다. PX4 는 자세 P → 각속도 PID 로 **두 단계**다

### 14-7. 각속도 → 토크 (PID)

$$
\boldsymbol{\tau}_{sp} = K \odot \Big[ P \odot (\boldsymbol{\Omega}_{sp} - \boldsymbol{\Omega}) + \mathbf{i}_\Omega - D \odot \dot{\boldsymbol{\Omega}} \Big] + FF \odot \boldsymbol{\Omega}_{sp},
\qquad \dot{\mathbf{i}}_\Omega = K \odot I \odot (\boldsymbol{\Omega}_{sp} - \boldsymbol{\Omega})
$$

| | roll | pitch | yaw | 파라미터 |
|---|---|---|---|---|
| $P$ | 0.15 | 0.15 | 0.2 | `MC_*RATE_P` |
| $I$ | 0.2 | 0.2 | 0.1 | `MC_*RATE_I` |
| $D$ | 0.003 | 0.003 | 0 | `MC_*RATE_D` |
| $FF$ | 0 | 0 | 0 | `MC_*RATE_FF` |
| $K$ (PX4 기본) | 1 | 1 | 1 | `MC_*RATE_K` |
| **$K$ (우리 기체, 13-3)** | **0.246** | **0.148** | **0.148** | sim 이 PX4 시작 때 넣음 |

- $\boldsymbol\Omega$ = 자이로 각속도, $\dot{\boldsymbol\Omega}$ = 그 미분 (PX4 가 걸러서 계산). 미분항은 14-3 처럼 오차가 아니라 측정값에 건다
- 적분은 $\pm 0.3$ 으로 제한, 할당이 포화된 방향으로는 쌓지 않음, 각속도 오차가 크면 줄임 (400 °/s 기준 $1 - (e/400°)^2$), 착지 상태면 멈춤
- **$\boldsymbol\tau_{sp}$ 는 N·m 가 아니라 −1~1 정규화 토크**다. PX4 는 기체 관성을 모른다. 같은 정규화 토크가 만드는 각가속도는 (최대 토크 / 관성) 에 비례하므로,
  관성이 작은 기체에서는 같은 게인이 그만큼 세다. 13-3 의 $K = (I/I_{ref}) / (\text{팔}/\text{팔}_{ref})$ 는 이 식의 $K$ 를 줄여 기준 기체와 같은 각가속도 응답을 만드는 것이다

### 14-8. 할당: (토크, 추력) → 모터 명령

로터 $i$ (위치 $\mathbf{r}_i$, 축 $\hat{\mathbf{a}}_i$ = 몸체 위쪽 (FRD 에서 $(0, 0, -1)$ = `CA_ROTORi_AX/AY/AZ` 0, 0, −1, 로그의 파라미터로 확인), 추력 계수 $c_T$, 모멘트 비 $k_{M,i}$) 가 만드는 모멘트와 추력으로 **effectiveness 행렬** $B$ (6 × 4) 의 열을 채운다:

$$
\mathbf{m}_i = c_T\,(\mathbf{r}_i \times \hat{\mathbf{a}}_i) - c_T\,k_{M,i}\,\hat{\mathbf{a}}_i, \qquad
\mathbf{f}_i = c_T\,\hat{\mathbf{a}}_i, \qquad
B = \begin{bmatrix} \mathbf{m}_1 & \cdots & \mathbf{m}_4 \\ \mathbf{f}_1 & \cdots & \mathbf{f}_4 \end{bmatrix}
$$

- $\mathbf{r}_i$ = `CA_ROTORi_PX/PY` (우리는 sim 로터 위치를 넣음, 13-3), $k_{M,i}$ = `CA_ROTORi_KM` ±0.05 (부호 = 회전 방향, airframe 값)
- 모터 명령은 $B$ 의 유사역행렬 $M = B^{+}$ 로 구한다: $\mathbf{u} = M\,[\boldsymbol\tau_{sp};\ \mathbf{T}]$.
  $M$ 의 열은 축별로 **정규화**한다 (roll·pitch 는 같은 배율, yaw 따로, 추력은 로터 평균이 1 이 되게). 그래서 입력이 정규화 값이어도 된다
- 멀티콥터는 `SEQUENTIAL_DESATURATION` 방식 (`CA_METHOD` 자동). 어떤 모터가 0~1 을 벗어나면 차례로 줄인다 (`MC_AIRMODE` 0 기본, `mixAirmodeDisabled`):
  1. yaw 를 빼고 계산 → 벗어나면 **추력을 줄임** (늘리지는 않음) → 그래도 벗어나면 roll, pitch 를 줄임
  2. yaw 를 더함 → 벗어나면 **yaw 를 줄임** (roll·pitch 는 그대로) → 추력을 줄임

  즉 포화 때 지키는 순서는 roll·pitch > yaw > 추력
- 기하 제어기의 할당(7장)과 같은 원리다. 차이: 7장은 $\omega_i^2$ 를 뉴턴 단위로 직접 풀고, PX4 는 정규화된 $u_i$ 를 푼다

출력 $u_i \in [0, 1]$ 이 `HIL_ACTUATOR_CONTROLS` 로 sim 에 온다 (13-2).

### 14-9. 기하 제어기(6장)와 비교

| | 기하 제어기 (`--flight geometric`) | PX4 (`--flight px4`, 기본) |
|---|---|---|
| 상태 | PhysX 정답값 | EKF2 추정값 (센서 잡음·지연) |
| 위치 | PID 한 단계 → 힘 [N] | 위치 P → 속도 PID → 가속도 [m/s²] (두 단계) |
| 자세 | 자세·각속도 오차 → 토크 [N·m] (한 단계) | 자세 P → 각속도 PID → 정규화 토크 (두 단계) |
| 질량·관성 | 식에 직접 들어감 (8-2 에서 게인 환산) | 모름. hover thrust 추정 + `MC_*RATE_K` 로 대신 |
| 할당 | 뉴턴 단위, $\omega_i^2$ | 정규화, $u_i$, 포화 때 roll·pitch 우선 |
| 모터 | 즉시 | 1차 지연 (13-4) |

### 14-10. 로그로 확인한 것 (2026-10-08)

PX4 비행 로그 `isaacsim/reports/px4_20261005_160728/log/2026-10-05/07_07_29.ulg` (모션캡처, 387 s, 호버 구간 96%) 를 pyulog 로 읽었다
(`~/PX4-Autopilot/.venv/bin/python`). B 는 다른 로그 3 개에서도 같았다. 새로 sim 을 돌리지는 않았다.

**A. 제어기 주기 = 120 Hz (sim physics 와 같음)**

| 토픽 (로그에 전부 기록되는 것) | 주기 | 의미 |
|---|---|---|
| `sensor_combined` | 120 Hz | sim 이 보낸 `HIL_SENSOR` (physics step 마다 1 개) |
| `vehicle_angular_velocity` | 120 Hz | 각속도 제어기를 깨우는 토픽 |
| `vehicle_attitude` | 120 Hz | 자세 제어기를 깨우는 토픽 (EKF2 출력) |
| `vehicle_local_position` | 120 Hz | 위치 제어기를 깨우는 토픽 (EKF2 출력) |
| `esc_status` | 120 Hz | `HIL_ACTUATOR_CONTROLS` 를 보낼 때마다 1 개 |

- 세 제어기는 타이머가 아니라 **위 토픽이 올 때마다** 돈다 (소스: 각 모듈의 `registerCallback`). 입력이 전부 120 Hz 이므로 위치·자세·각속도·할당이 모두 120 Hz
- 실물 PX4 는 각속도 루프가 훨씬 빠르다 (`IMU_GYRO_RATEMAX` 800 Hz). sim 은 센서를 120 Hz 로만 주므로 PX4 내부 루프도 120 Hz 로 묶인다
- 제어기 출력 토픽 (`vehicle_rates_setpoint` 등) 은 로거가 솎아서 기록하므로 (10~60 Hz) 그것으로는 주기를 알 수 없다. 위 표는 솎지 않는 토픽만
- 위치 제어기가 실제로 120 Hz 로 계산했는지는 입력 토픽 주기와 소스 구조에서 추론한 것이다 (직접 잰 값은 모터 명령 120 Hz)

**B. hover thrust 추정기는 돌지 않는다 → $T_h$ = 0.5 (기본값) 그대로**

| 항목 | 값 |
|---|---|
| `hover_thrust_estimate` 토픽 | 모션캡처 로그 3 개에 **없음** (발행된 적 없음). GPS·flow 방식 로그 (`px4_20261002_131143`) 에는 있음 |
| 호버 중 총추력 명령 (`vehicle_thrust_setpoint` z) | −0.3788 (표준편차 0.0004) |
| 호버 중 모터 명령 $u$ 평균 | 0.3788 (로터별 0.365, 0.392, 0.369, 0.389) → 13-2 의 예상값 0.38 과 일치 |
| 호버 중 수직 가속도 목표 $a_{sp,z}$ | **+2.4 m/s²** (0 이 아님, 로그 3 개에서 2.38~2.73) |

- 원인 (소스 `mc_hover_thrust_estimator`): 추정기는 "공중" 판정이 나야 시작하는데 그 조건이 `dist_bottom > 1 m` 이다.
  `dist_bottom` 은 EKF2 의 지면까지 높이 추정값이고, 모션캡처 설정은 거리 센서를 끄므로 (`EKF2_RNG_CTRL 0`) 이 값이 유효하지 않고 (`dist_bottom_valid` 0) 최대 0.85 m 였다
- 결과: 14-4 의 $T_h$ 가 0.5 인 채로 비행한다. 실제 호버 추력은 0.379 이므로 14-4 식에서
  $a_{sp,z} = g\,(1 - 0.379 / 0.5) = 2.38$ m/s² 가 되어야 하고, 로그 값과 맞는다. **이 차이는 수직 속도 적분항이 메우고 있다** (14-3)
- 비행은 된다 (13-6 의 시험은 전부 이 상태에서 한 것). 식에서 달라지는 점: 수직 가속도 → 추력 환산 배율 $T_h / g$ 가 $0.5 / 0.379 = 1.32$ 배 크고
  (같은 수직 가속도 목표에 추력을 더 많이 바꿈), 이륙 뒤 적분이 2.4 m/s² 만큼 쌓여야 고도가 맞는다. 수평은 추력 방향(기울기)으로 정해지므로 $T_h$ 와 무관.
  이것이 비행에 실제로 얼마나 보이는지는 재지 않았다
- 맞추려면 `drone_iris.yaml` `px4.params` 에 `MPC_THR_HOVER: 0.38` 을 넣으면 된다 (PX4 문서가 권하는 방법). **넣지 않았다** — 13-6 의 비행·파지 시험 결과가 지금 값 기준이라, 바꾸면 다시 시험해야 한다

**C. Offboard 위치 목표는 다듬어지지 않고 그대로 들어간다**

- 로그의 `trajectory_setpoint`: 위치·yaw 만 값이 있고 속도·가속도·yaw 속도는 NaN (표본의 98%)
- 소스: `mavlink_receiver` 가 `SET_POSITION_TARGET_LOCAL_NED` 를 `trajectory_setpoint` 로 바로 발행하고, `mc_pos_control` 이 그것을 그대로 읽는다 (사이에 궤적 생성 없음)
- `PX4Commander.goto` 도 목표를 한 번에 바꾼다 (`drone_cmd.py`, 속도 제한 없음). 따라서 **`goto` 는 위치 목표의 계단 입력**이고, 14-2 에 따라 속도 목표가 $0.95 \times$ 거리 [m/s] 로 뛴다
  (0.1 m 이동 → 0.095 m/s, 1 m 이동 → 0.95 m/s). 부드럽게 움직이려면 보내는 쪽이 목표를 조금씩 옮겨야 한다 (조종기 연결 때도 같음)

**D. 로터 축**: `CA_ROTOR0_AX` 0, `CA_ROTOR0_AZ` −1 (몸체 위쪽), `CA_ROTOR0_CT` 6.5, `CA_ROTOR0_KM` 0.05, `CA_ROTOR0_PX/PY` 0.1032 / 0.1551 (sim 로터 위치),
`MC_ROLLRATE_K` 0.2456, `MC_PITCHRATE_K`·`MC_YAWRATE_K` 0.1484, `MC_AIRMODE` 0, `THR_MDL_FAC` 0 — 14-7·14-8 에 적은 값과 같다

### 14-11. EKF2 (상태 추정) 개요

제어기가 쓰는 $p, v, q$ 는 EKF2 가 IMU 와 모션캡처 위치를 합쳐 만든다. 여기는 **구조와 우리 설정**만 적는다.
공분산 전파·관측 갱신의 식은 PX4 가 SymForce 로 자동 생성한 코드 (`src/modules/ekf2/EKF/python/ekf_derivation/generated/`) 라 옮기지 않았다.

**상태** (`generated/state.h`, 오차 상태 24 개):

| 상태 | 크기 | 우리 설정에서 |
|---|---|---|
| 자세 quaternion | 3 (오차) | IMU 적분 + 모션캡처 yaw |
| 속도 (NED) | 3 | IMU 적분 |
| 위치 (NED) | 3 | IMU 적분 + 모션캡처 위치 |
| 자이로 bias | 3 | 추정 |
| 가속도계 bias | 3 | 추정 |
| 지자기 (지구·몸체) | 3 + 3 | 안 씀 (`EKF2_MAG_TYPE 5`) |
| 바람 | 2 | 안 씀 |
| 지면 높이 (terrain) | 1 | 거리 센서가 없어 유효하지 않음 (14-10 B 의 원인) |

**예측** (IMU 가 올 때마다 = 120 Hz, `ekf.cpp` `predictState`). IMU 는 각도 증분 $\Delta\boldsymbol\theta$ 와 속도 증분 $\Delta\mathbf{v}$ 로 들어온다:

$$
q \leftarrow q \otimes \exp\!\big(\Delta\boldsymbol\theta - \mathbf{b}_g \Delta t\big), \qquad
\mathbf{v} \leftarrow \mathbf{v} + R(q)\,\big(\Delta\mathbf{v} - \mathbf{b}_a \Delta t\big) + \mathbf{g}\,\Delta t, \qquad
\mathbf{p} \leftarrow \mathbf{p} + \tfrac{1}{2}(\mathbf{v}_{old} + \mathbf{v})\,\Delta t
$$

($\mathbf{g} = (0, 0, 9.80665)$ NED. 지구 자전 항은 생략해 적음.) 가속도계가 재는 것은 중력을 뺀 비력이라 중력을 더한다.

**갱신** (모션캡처가 올 때, 표준 칼만 필터 형태):

$$
\mathbf{y} = \mathbf{z} - \mathbf{h}(\hat{\mathbf{x}}), \qquad
K = P H^\top (H P H^\top + R)^{-1}, \qquad
\hat{\mathbf{x}} \leftarrow \hat{\mathbf{x}} + K \mathbf{y}, \qquad
P \leftarrow (I - K H) P
$$

| 관측 $\mathbf z$ | 무엇과 비교 | 잡음 $R$ (우리 설정, `px4_sitl.yaml`) |
|---|---|---|
| 모션캡처 위치 (수평·수직) | 위치 상태 | `EKF2_EVP_NOISE` 0.01 m |
| 모션캡처 yaw | 자세 상태의 yaw | `EKF2_EVA_NOISE` 0.05 rad |

- `EKF2_EV_CTRL 11` = 수평 위치 + 수직 위치 + yaw 를 쓴다 (속도는 안 씀). `EKF2_HGT_REF 3` = 고도 기준도 모션캡처
- **지연 처리**: 모션캡처는 20 ms 늦게 온다 (`EKF2_EV_DELAY 20`). EKF2 는 IMU 를 버퍼에 쌓아 두고 **과거 시각의 상태**에서 관측을 합친 뒤,
  그 뒤의 IMU 를 다시 적분해 현재 상태를 낸다 (output predictor). 그래서 제어기는 지연 없는 현재 추정값을 받는다
- 관측이 예측과 너무 다르면 버린다 (innovation gate, `EKF2_EVP_GATE` 기본 5σ)
- 기압계는 로그에서 17 Hz 로 들어오지만 고도 기준이 아니다

**파지와의 관계**: 그리퍼가 드론을 잡으면 실제 움직임이 거의 없는데 PhysX 가 보고하는 각속도는 크다. 그 값을 자이로로 주면 위 예측 식의 $\Delta\boldsymbol\theta$ 가 틀려
자세 추정이 틀어진다 → 13-4 의 "센서용 속도 = 자세 차분" 이 필요했던 이유

### 14-12. 아직 확인하지 않은 것
- `MPC_THR_HOVER` 를 0.38 로 맞췄을 때 비행·파지 거동이 어떻게 달라지는지 (14-10 B)
- 추력 모델 차이 (PX4 는 $u$ 에 비례 가정, sim 은 이차식, 14-4) 의 영향
- EKF2 의 공분산 전파·갱신 식 자체 (자동 생성 코드), 모션캡처 지연 20 ms 가 버퍼에서 몇 표본으로 처리되는지
- 위치 제어기 계산 주기의 직접 측정 (14-10 A 는 입력 토픽 주기에서 추론)
