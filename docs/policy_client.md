# 정책 클라이언트 노드 (`policy/policy_client_node.py`) 동작

정책 클라이언트가 무엇을 받고 내보내는지, 안에서 무슨 일이 일어나는지 (관측 짝짓기, 60 Hz 루프, 보간, 안전 한계) 를 정리한다.
세 프로그램 (sim · 정책 서버 · 클라이언트) 구조·GPU 실측·실행 명령은 `docs/policy_inference_desktop.md`,
서버 안의 모델 (flow matching) 은 `docs/pi05_flow_matching.md`.

코드 줄 번호는 2026-10-27 기준. **4·5장의 시각·값은 이해를 돕는 예시** (추론 sim 0.166 s, 이미지 stamp 약 1 프레임 늦음, `--duration 30` 가정).
실측은 추론 왕복 약 0.18 s, 새 chunk 도착 약 4 칸, 마지막 칸 유지 0 % (`docs/openpi_finetuning.md` 8.2).

## 1. 실행되는 파일

```bash
cd ~/data_collection_ur5_gripper && source /opt/ros/jazzy/setup.bash
~/openpi_client/venv/bin/python policy/policy_client_node.py --reset --episodes 5 --duration 30
```

```text
policy/policy_client_node.py              본체: 노드, 관측 짝짓기, chunk 실행, 안전 한계
 ├ policy/policy_client.yaml              설정 (서버 주소, prompt, 주기, 안전 값, 토픽). 항목이 빠지면 에러
 ├ teleop/ik_model.py                     관절 이름 순서 ARM_JOINTS, 관절 한계 ArmModel.limits
 │   └ isaacsim/ur5_rh_p12_description/out/ur5_rh_p12_d435i.urdf   (텔레옵과 같은 URDF)
 ├ openpi_client (~/openpi/packages/openpi-client, venv 에 설치)
 │   ├ websocket_client_policy.py         서버 연결, infer() = obs 보내고 actions 받기
 │   └ msgpack_numpy.py                   numpy ↔ 바이트
 └ rclpy 등 (시스템 ROS 2 Jazzy)
```

`~/openpi_client/venv` 는 `--system-site-packages` venv (시스템 rclpy 를 읽고, `openpi-client` 만 설치). 클라이언트에는 JAX·GPU 가 필요 없다.

## 2. 입력과 출력

**받는 것**

| 출처 | 무엇 | 쓰는 곳 |
|---|---|---|
| `/clock` | sim time | 모든 시각 계산 (칸 위치, 60 Hz 루프, 에피소드 길이) |
| `/joint_states` (120 Hz) | 팔 6 관절각 | 관측 state[0:6], desync 검사, 안전 한계 기준 |
| `/gripper/joint_states` (30 Hz) | present raw 0~1150 | 관측 state[6] = /1150, `--auto-kill` |
| `/cam/wrist`, `/cam/third_view` (30 Hz) | rgb8 640×480 (아니면 에러) | 관측 이미지, 기준 시각 t_obs |
| `/protective_stop` | Bool | 걸리면 에피소드 중단 |
| 정책 서버 | actions (10, 7): 팔 절대각 + 그리퍼 연속값 | 실행할 계획 |

**내보내는 것**

| 대상 | 무엇 | 언제 |
|---|---|---|
| `/joint_command` | 6 관절 목표, stamp = 지금 sim time | sim time 60 Hz |
| `/gripper/command` | 0 / 1150 | 에피소드 시작 때 열기, 이후 바뀔 때만 |
| `/sim/reset` | 서비스 호출 (최대 120 s) | `--reset` 이면 에피소드마다 |
| `/sim/drone_kill` | 서비스 호출 | `--auto-kill S`: 닫기 S 초 뒤 present < 0.9 일 때 한 번 |
| 정책 서버 | obs: state (7,), 이미지 2 장 (640×480 그대로), prompt | 약 5 Hz (추론이 끝나면 다음, 5.4절) |

텔레옵 노드와 같은 명령 토픽 두 개만 보내므로 sim 은 상대가 사람인지 정책인지 모른다 (CLAUDE.md 원칙 2).

## 3. 안쪽 구조

### 3.1 시작 순서 (`main`, L347-393)

1. 설정 읽기·검사 (항목 누락, `replan_steps ≥ 1`, `max_lead < desync`)
2. 정책 서버 연결 (안 떠 있으면 5 s 마다 재시도), metadata 받기 (`pi05_ur5` 는 `{}`)
3. ROS 노드: 구독 6, 발행 2, 서비스 클라이언트 2, spin 스레드 시작
4. `wait_ready`: 관측 5 가지 + sim 의 명령 구독 + 관측 짝짓기 성공까지 (최대 60 s. 연결 전에 보낸 명령은 사라져 목표가 뛸 수 있음)
5. `warmup`: 추론 한 번 (서버 JIT 컴파일 11~13 s, 로봇은 안 움직임)
6. 에피소드마다 `/sim/reset` → sim 0.2 s 대기 → `run_episode`
7. 여러 에피소드면 요약 (파지 신호 몇 개, 멈춘 이유)

### 3.2 스레드 3 개

```text
spin 스레드:  [spin_once][spin_once]...   메시지를 꺼내 버퍼에 쌓고 clock 갱신
                    │ self.lock
메인 스레드:  [wait_sim_until][검사·보간·안전·발행][wait ...]   60 Hz 루프 (5장)
추론 스레드:         [observe → infer (약 0.18 s)]             필요할 때만
```

| 스레드 | 하는 일 |
|---|---|
| spin (L135-140) | `SingleThreadedExecutor.spin_once(0.05)` 반복. 버퍼: `js` 120 개 (1 s), `grip` 30 개 (1 s), 카메라마다 4 개, `clock`, `pstop`. 콜백에서 찾은 문제는 `self.err` 에 적고 메인이 던짐 (spin 스레드에서 던지면 그 스레드만 죽음) |
| 메인 | `run_episode` 60 Hz 루프. 버퍼는 lock 걸고 읽기만 |
| 추론 | `observe()` → `policy.infer()`. 메인은 기다리지 않고 지금 chunk 를 계속 실행 |

spin 을 메인에서 하면 기다리는 동안 (`wait_sim_until`, 추론, 리셋) `/clock` 콜백이 멈춰 sim 시각을 기다리는 루프가 끝나지 않는다 → 전용 스레드.
sim 쪽 spin (`ros2_iface.spin`, 빈 호출 두 번 연속에서 멈춤) 과 달리 계속 돌기 때문에 빈 호출 문제가 없다.

## 4. 관측 짝짓기 `observe()` (L189-214)

```text
t_obs = min(wrist 최신 stamp, third_view 최신 stamp)    카메라가 관절보다 1~2 프레임 늦게 도착하므로 늦은 쪽에 맞춤
버퍼마다 t_obs 이하에서 가장 최근 메시지
각각 (t_obs − stamp) > 0.066 s 이면 에러, t_obs 이하 메시지가 아예 없어도 에러
state = [팔 6 (ARM_JOINTS 순서), present / 1150]
obs   = {state, base_image = third view, wrist_image = 손목, prompt}
돌려줌 (obs, t_obs, 지금 관절각 js[-1], 지금 present, pstop)
```

stage1 (격자 시각 이전 최신, `--max-age 0.066`) 과 같은 규칙 → 학습 데이터와 같은 방식으로 짝지은 관측.

### 4.1 예: 정상 (호출 순간 sim 12.100)

| 버퍼 | stamp (뒤가 최신) | 고른 것 | 나이 |
|---|---|---|---|
| js | …, 12.0333, 12.0417, …, 12.1000 | 12.0333 | 0 |
| grip | …, 12.0333, 12.0667, 12.1000 | 12.0333 | 0 |
| wrist | 11.9667, 12.0000, 12.0333, 12.0667 | 12.0333 | 0 |
| third_view | 11.9333, 11.9667, 12.0000, 12.0333 | 12.0333 (= t_obs) | 0 |

- sim 은 카메라·그리퍼·관절 stamp 가 같은 물리 스텝이라 (`docs/timestamp_sync.md`) 넷 다 나이 0. **"이전 최신" 과 "가장 가까운" 이 같은 메시지를 고른다**
- `max` 를 쓰면 t_obs 12.0667 에 third view 가 없어 두 카메라가 33 ms 다른 순간이 된다

### 4.2 예: 그리퍼 메시지가 빠짐 → 에러

grip 최신이 11.9667, t_obs 12.0333 → 나이 0.067 > 0.066 → `관측이 기준 시각보다 오래됨 [s]: {'gripper': 0.067}`. 옛 값과 새 값을 섞어 보내지 않는다.

### 4.3 예: 시작 직후 → 재시도

js 가 5.1000 부터, 이미지는 5.0667 → `joint_states: t_obs 5.067 이전 메시지가 없음`. `wait_ready` 가 0.2 s 뒤 다시 (10 s 넘게 실패할 때만 에러).

### 4.4 예전 방식 (기준 = 지금 시각) 이 실패한 이유

기준 12.1000 이면 third view 12.0333 이 나이 0.067 → 정상인데도 자주 에러. 기준을 이미지 stamp 로 옮겨 "늦게 도착" 은 문제 삼지 않고 "같은 순간끼리 짝지었나" 만 본다.

### 4.5 검사하지 않는 것

t_obs 가 **지금** 보다 얼마나 오래됐는지는 안 본다. 두 카메라가 함께 멈추면 새 chunk 의 칸 위치가 9 를 넘어 마지막 칸 유지 (로봇이 섬, `held_frac` 증가),
1 s 넘으면 `js` 버퍼에 t_obs 이전 관절값이 없어 에러로 멈춘다.

## 5. 에피소드 실행 `run_episode()` (L251-339)

### 5.1 루프 전 준비

```text
sim 100.000  /sim/reset 응답 (드론 새 위치, 팔 시작 자세로 순간이동)
sim 100.200  0.2 s 대기 (리셋 뒤 관측이 새로 들어오게)
 ① observe() → 지금 관절각 q_meas
 ② q_cmd = q_meas            첫 명령 기준 = 지금 자세 (안전 한계가 여기서부터)
 ③ /gripper/command 0        열기로 시작
 ④ t_start = 100.200
 ⑤ 첫 추론은 기다림: t_obs_A = 100.167 → chunk A 도착 (sim 100.333). 그동안 /joint_command 없음 → sim 은 리셋 자세 유지
 ⑥ t_next = 100.333, 60 Hz 루프 시작
```

### 5.2 바퀴 하나의 단계 (sim time 16.7 ms 마다)

| 단계 | 하는 일 | 코드 |
|---|---|---|
| ⓪ 기다리기 | `t_next += 1/60`, `/clock` 이 t_next 이상이 될 때까지 (2 ms 간격 확인). sim 기준 60 Hz (RTF < 1 이면 wall 로는 느려짐) | L278-279 |
| 1 멈춤 검사 | 경과 ≥ duration / 보호 정지 / max\|측정 − q_cmd\| > 0.2 rad (desync) → 종료 | L280-290 |
| 2 새 chunk? | 추론 스레드가 끝났으면 `act`, `t_obs` 교체, 도착 칸 위치 기록. 추론 에러는 여기서 던짐 | L291-298 |
| 3 칸 위치 | x = (지금 − t_obs) / 40 ms. 추론 동안 지나간 칸은 자동으로 건너뜀 | L299 |
| 4 다음 추론 | x ≥ 4 (`replan_steps`) 이고 추론 중이 아니면 추론 스레드 시작 | L300-302 |
| 5 보간 | 칸 ⌊x⌋ 와 ⌊x⌋+1 사이 직선. x ≥ 9 이면 칸 9 유지 (`held`) | L303-309 |
| 6 안전 한계 | ① 한 번에 1.5 rad/s × 1/60 = 0.025 rad ② 측정 ± 0.08 rad ③ 관절 한계 안쪽 0.05 rad (텔레옵과 같은 값). 잘리면 `limited` | L310-312, `safe_target` L216-222 |
| 7 보내기 | `/joint_command`, stamp = 지금 sim time | L313 |
| 8 그리퍼 | 칸 ⌊x⌋ 의 그리퍼 값 (보간 안 함) > 0.5 → 닫기. 바뀔 때만 `/gripper/command` | L315-320 |
| 9 auto-kill | (옵션) 닫은 지 S 초 + present < 0.9 → `/sim/drone_kill` 한 번 | L321-327 |

### 5.3 바퀴별 진행 (예시)

| 바퀴 | sim 시각 | 쓰는 chunk (t_obs) | 칸 위치 x | 일어나는 일 |
|---|---|---|---|---|
| 1 | 100.350 | A (100.167) | (100.350−100.167)/0.04 = 4.58 | x ≥ 4 → 추론 B 시작 (관측 t_obs_B = 100.333). 칸 4↔5 보간. 첫 명령이라 q_cmd (= 지금 자세) 에서 최대 0.025 rad 만 움직임 |
| 2 | 100.367 | A | 5.00 | 추론 중. 칸 5 |
| 3 | 100.383 | A | 5.42 | 칸 5↔6 |
| 4~9 | 100.400~100.483 | A | 5.83 ~ 7.92 | 계속 A 실행, 바퀴마다 `/joint_command` |
| 10 | 100.500 | B (100.333) | (100.500−100.333)/0.04 = 4.17 | 추론 B 끝 → A 를 B 로 교체 (도착 칸 4.17 기록). x ≥ 4 → 곧바로 추론 C 시작 |
| 11~19 | 100.517~100.650 | B | 4.58 ~ 7.92 | B 실행 |
| 20 | 100.667 | C (100.500) | 4.17 | C 로 교체, 추론 D 시작 |
| … | … | … | … | 약 0.17 s 마다 이 반복 |

- 각 chunk 는 칸 약 4.2 ~ 8 구간 (약 0.17 s) 만 실행되고 다음 chunk 로 바뀐다. 앞 절반 (칸 0~4) 은 추론 동안 지나간 시간이라 안 쓴다
- 추론은 끝나자마자 다음이 시작되어 쉬지 않고 이어서 돈다 (약 5~6 Hz)
- 추론이 늦어 x 가 9 를 넘으면 칸 9 를 유지하고 `held` 로 센다 (지금까지 0 %)

### 5.4 바퀴 하나 안에서 (예: 바퀴 3, sim 100.383)

| 단계 | 하는 일 | 예시 값 |
|---|---|---|
| ⓪ 기다리기 | t_next = 100.333 + 3/60 = 100.383 이 될 때까지 `/clock` 확인 | now = 100.383 |
| 1 멈춤 검사 | 경과 0.183 s < 30 s ✓, 보호 정지 없음 ✓, \|측정 − q_cmd\| = 0.004 rad < 0.2 ✓ | 계속 |
| 2 새 chunk? | 추론 B 아직 진행 중 | A 유지 |
| 3 칸 위치 | (100.383 − 100.167) / 0.04 | x = 5.42 |
| 4 다음 추론 | 이미 B 가 진행 중이라 시작 안 함 | — |
| 5 보간 | i0 = 5, i1 = 6, a = A[5] + 0.42 × (A[6] − A[5]) | shoulder_pan: −0.2660 + 0.42 × 0.0035 = −0.2645 |
| 6 안전 한계 | ① 지난 목표 −0.2650 에서 +0.0005 → 0.025 안 ✓ ② 측정 −0.2690 ± 0.08 안 ✓ ③ 관절 한계 안 ✓ | −0.2645 그대로 |
| 7 보내기 | `/joint_command`, stamp 100.383 | 6 관절 목표 |
| 8 그리퍼 | A[5] 의 그리퍼 0.03 ≤ 0.5 → 열림 유지, 바뀐 것 없음 | 안 보냄 |
| 9 auto-kill | 닫힌 상태가 아님 | — |

### 5.5 그리퍼가 닫히는 순간 (예시)

```text
sim 109.217  칸 i0 의 그리퍼 출력 0.71 > 0.5 → /gripper/command 1150, closed = True, t_closed = 109.217
             로그 "[policy] 그리퍼 닫기 (에피소드 t 9.02 s, 출력 0.71)"
그 뒤        sim GripperProfile 이 0.516 rad/s 로 닫음 → 드론에 막혀 present 약 0.25 에서 멈춤
--auto-kill 1.0 이면 sim 110.217 바퀴에서 present 0.25 < 0.9 → /sim/drone_kill (지금 평가는 사람이 모터 정지)
```

출력이 0.5 근처를 오가면 닫기·열기가 번갈아 나간다 (8.3 평가의 그리퍼 오감, `docs/openpi_finetuning.md`).

### 5.6 끝

```text
now − t_start ≥ 30 → 종료 (stop = "duration"). 보호 정지·desync 면 그 바퀴에서 끝나고 stop 에 이유
진행 중인 추론을 기다림 (최대 10 s) → observe() 로 끝날 때 그리퍼 present → 결과 JSON
```

| 값 | 뜻 |
|---|---|
| `stop` | duration / protective_stop / desync |
| `chunks`, `sent` | 받은 chunk 수, 보낸 명령 수 (30 s × 60 Hz ≈ 1800) |
| `infer_wall_median_s` / `max_s` | 추론 왕복 시간 (클라이언트가 잼) |
| `chunk_arrive_step_median` | 새 chunk 가 몇 칸째 도착했나 |
| `held_frac` | 마지막 칸 유지 비율 |
| `limited_frac` | 안전 한계에 잘린 비율 |
| `gripper_closed`, `gripper_present_end` | 끝날 때 그리퍼 (0.2~0.3 = 드론을 쥠, 약 0.98 = 빈손 닫힘, 0 = 열림) |
| `drone_killed` | auto-kill 실행 여부 |

클라이언트가 잠깐 멈췄다 다시 돌면 밀린 바퀴가 기다림 없이 연달아 돌아 거의 같은 stamp 명령이 몇 개 한꺼번에 나간다 (x 는 실제 `now` 로 계산하므로 값은 맞음).

## 6. 보간이 들어가는 곳

```text
정책 서버        chunk 10 칸 (25 Hz)
  ▼ ① 클라이언트 보간 (칸 사이 직선)                 25 → 60 Hz   policy_client_node.py L303-309
/joint_command  16.7 ms 마다
  ▼ sim 앱 루프 (30 Hz) 가 최신 명령 하나를 가져감
  ▼ ② ArmBridge 보간 (지난 목표 → 새 목표, 33 ms)    30 → 120 Hz  ros2_iface.py ArmBridge L181·L232
물리 스텝 관절 목표 → PhysX PD
```

| | ① 클라이언트 | ② ArmBridge |
|---|---|---|
| 식 | a = act[i0] + (x − i0)·(act[i1] − act[i0]) | u = clamp((t − t0)/T, 0, 1), q = q_start + u·(q_goal − q_start), T = 33 ms (`ros2_iface.yaml` `command_interp_time`) |
| 데이터 수집 때 | 없음 (텔레옵이 처음부터 60 Hz) | 같음 |

- 그리퍼는 보간 없음 (0 / 1150 두 값). sim `GripperProfile` 이 목표를 0.516 rad/s 로 옮긴다
- 관측·데이터셋 (stage1, `observe()`, action[t]) 은 보간 없이 "이전 최신" (zero-order hold)

**60 Hz 인 이유**: 수집 때 텔레옵과 같고 (`teleop/xr_teleop.yaml` `rate_hz`), sim 앱 루프 30 Hz 마다 새 명령이 약 2 개 와 있어 계단이 없으며
(30 Hz 로 맞추면 박자가 어긋날 때 새 명령 없는 루프가 생김), 더 빠르게 보내도 sim 이 33 ms 마다 하나만 써서 얻는 것이 없다.

## 7. 텔레옵 노드와 비교

| | 텔레옵 노드 (수집) | 정책 클라이언트 (평가) |
|---|---|---|
| 명령을 만드는 곳 | PICO 컨트롤러 자세 → IK | 정책 서버 action chunk |
| ROS 입력 | `/joint_states` | `/joint_states` + 그리퍼 + 카메라 2 대 + clock + 보호 정지 |
| 명령 출력 | `/joint_command` 60 Hz, `/gripper/command` | 같음 |
| 안전 한계 | 1.5 rad/s, ± 0.08 rad, 한계 − 0.05 | 같음 (같은 URDF·값) |
| 리셋 | 녹화 도구 `/sim/reset` (B) | 클라이언트가 직접 |
| 모터 정지 | 사람 (X) | 사람 (`drone_cmd.py kill`) 또는 `--auto-kill` |
| 녹화 | 함 | 안 함 |
