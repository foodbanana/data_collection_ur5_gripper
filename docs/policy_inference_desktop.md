# 데스크탑에서 정책 추론 (sim GUI 폐루프)

작성: 2026-10-26 · `docs/openpi_finetuning.md` 8.2 · 8.3 (학습·평가 결과), `docs/PLAN.md` 7단계, 브랜치 `isaacsim_v6.1.0`

연구실 서버에서 학습한 openpi π0.5 체크포인트 (`pi05_ur5` full, `pi05_ur5_lora` LoRA) 를 데스크탑 GPU 에서 추론해,
Isaac Sim GUI 의 로봇을 정책이 직접 움직이게 하는 구성. 데스크탑 GPU 하나에 sim 과 정책 서버가 같이 들어가는지 잰 결과,
세 프로그램 (sim · 정책 서버 · 클라이언트 노드) 의 구조, 실행 명령을 정리한다.

---

## 1. 한눈에 보기

| 항목 | 내용 |
|------|------|
| 데스크탑 | RTX 4070 Ti SUPER 16 GB, i5-14600KF, Ubuntu 24.04, ROS 2 Jazzy, Isaac Sim 6.1.0 |
| 결론 | **sim GUI (보조 뷰포트 끔) + 정책 서버 = 약 11.8 GB / 16 GB** → 같이 돌아간다 (full·LoRA 같음) |
| 정책 서버 | openpi `scripts/serve_policy.py` (`~/openpi` uv 환경), websocket 포트 8000. **`XLA_PYTHON_CLIENT_PREALLOCATE=false` 필수** |
| 클라이언트 | `policy/policy_client_node.py` + `policy/policy_client.yaml` (`~/openpi_client/venv`). 텔레옵 노드 자리 (CLAUDE.md 원칙 2: `/joint_command`·`/gripper/command` 만 보냄) |
| sim | `isaacsim/scripts/sim_ros2.py` (수집 때와 같은 것, `--aux-views off`) |
| 모터 정지 | 사람이 GUI 를 보고 잡혔으면 끈다 (수집 때 X 와 같게, 2026-10-26 결정) |
| 첫 결과 | full 10k (first13, 과적합) 6 에피소드 중 5 개 파지 (`docs/openpi_finetuning.md` 8.3) |

---

## 2. 검증: 데스크탑 GPU 에 들어가는가 (실측, 2026-10-23 · 26)

### 2.1 GPU 메모리

| 프로그램 | GPU 메모리 | 비고 |
|----------|-----------|------|
| 화면 (기본) | 약 0.4 ~ 0.8 GB | 측정 때마다 조금 다름 |
| Isaac Sim **헤드리스** (카메라 2 대) | 최대 약 3.6 GB (화면 포함) → sim 몫 약 2.8 GB | `--headless --flight geometric` |
| Isaac Sim **GUI, `--aux-views off`** (카메라 2 대) | 최대 약 3.2 GB (화면 포함) → sim 몫 약 2.8 GB | 메인 뷰포트는 거의 추가 비용 없음. 메모리는 대부분 카메라 2 대 렌더 |
| 정책 **LoRA** (`pi05_ur5_lora`, smoke 체크포인트) | 약 8.4 GB | |
| 정책 **full** (`pi05_ur5`, `full_first13_10k/9999`) | 약 8.5 GB | params 는 12 GB (float32) 지만 추론 때 bfloat16 으로 올려 LoRA 와 같음 |
| **sim GUI + 정책 full (실제 평가 구성)** | **약 11.8 GB / 16 GB** | 약 4 GB 여유 |

- 보조 옆·위 뷰포트 (`--aux-views on`, 수집 기본) 는 평가에 쓰지 않는다 (사용자). 켜면 창 2 개 렌더가 더 들고 RTF 가 약 0.98 → 0.82 로 떨어진다
- PX4 드론은 CPU 에서 돌아 GPU 메모리와 무관

### 2.2 추론 시간·로딩

| 항목 | LoRA | full |
|------|------|------|
| 체크포인트 로딩 | 4.5 s | 4.8 s |
| 첫 추론 (서버 JIT 컴파일 포함) | 13.1 s | 11.3 s |
| 추론 (정책 단독, 가짜 입력 10 번 중앙값) | 0.13 s | 0.12 s |
| 추론 (sim 과 같이, 클라이언트에서 잰 왕복) | 약 0.19 s | 약 0.18 s |
| 출력 | actions (10, 7) | actions (10, 7), 모두 유한값 |

- 추론 0.18 s ≈ chunk 4.5 칸 (25 Hz). chunk 는 10 칸 = 0.4 s 라 실행 중 다음 chunk 를 받을 시간이 충분하다 (4 장)

### 2.3 full 체크포인트를 GPU 1 장에서 열기

- 서버에서 GPU 2 장 FSDP 로 학습·저장 → `params/_sharding` 에 2 장 mesh 가 기록돼 있다
- openpi `restore_params` (`src/openpi/models/model.py`) 는 sharding 을 주지 않으면 지금 있는 GPU 에 맞춰 새로 정한다 → **데스크탑 1 장에서 에러 없이 열림** (2026-10-26 확인)
- 추론에 필요한 것은 `params/` 와 `assets/` (norm stats) 뿐. `train_state/` (optimizer 상태, full 은 19 GB) 는 이어서 학습할 때만

### 2.4 재는 방법

```bash
# 정책 단독: 로딩·메모리·추론 시간 (sim 없이). 측정 스크립트는 레포 밖 임시 파일이었다 → 아래 내용을 그대로 쓰면 된다
cd ~/openpi && XLA_PYTHON_CLIENT_PREALLOCATE=false uv run python - <<'EOF'
import pyarrow, subprocess, time, numpy as np
from openpi.policies import policy_config, ur5_policy
from openpi.training import config as _config
used = lambda: int(subprocess.run(["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"], capture_output=True, text=True).stdout.split()[0])
print("before", used(), "MiB")
p = policy_config.create_trained_policy(_config.get_config("pi05_ur5"), "checkpoints/pi05_ur5/full_first13_10k/9999")
p.infer(ur5_policy.make_ur5_example())                      # 첫 추론 = 컴파일
ts = []
for _ in range(10):
    t = time.time(); out = p.infer(ur5_policy.make_ur5_example()); ts.append(time.time() - t)
print("after", used(), "MiB, infer median %.3f s, actions" % np.median(ts), np.asarray(out["actions"]).shape)
EOF

# sim 이 띄워져 있을 때 GPU 메모리
nvidia-smi --query-gpu=memory.used,memory.total --format=csv
```

---

## 3. 구조

### 3.1 프로그램과 토픽

```text
+--------------------------------------------------------------------------------------------------------------+
| [A] SIMULATOR  ~/isaacsim/python.sh isaacsim/scripts/sim_ros2.py ... --aux-views off   (GPU ~2.8 GB)          |
|     same scene / topics / reset as data collection (docs/sim_ros2_interface.md), all stamps = sim time         |
+--------------------------------------------------------------------------------------------------------------+
   |  /cam/wrist/color/image_raw        rgb8 640x480  30 Hz           ^  /joint_command    JointState  60 Hz
   |  /cam/third_view/color/image_raw   rgb8 640x480  30 Hz           |                    (stamp = sim time)
   |  /joint_states                     arm 6 joints  120 Hz          |  /gripper/command  Float64 0 / 1150
   |  /gripper/joint_states             present raw    30 Hz          |                    (on change)
   |  /clock, /protective_stop                                        |
   |                                     services: /sim/reset, /sim/drone_kill <------------------+
   v                                                                  |                           |
+--------------------------------------------------------------------------------------------------------------+
| [B] POLICY CLIENT  ~/openpi_client/venv/bin/python policy/policy_client_node.py   (system ROS 2 + openpi-client) |
|     1. observe: pair images / joints / gripper at the image stamp (cameras arrive 1-2 frames late)            |
|        state (7) = arm 6 [rad] + present / 1150,  base = third view,  wrist = wrist,  prompt                   |
|     2. send obs -> get action chunk (10, 7)    (next request starts in the background at chunk step 4)        |
|     3. execute chunk on sim time: 60 Hz, linear interpolation between steps, skip steps already passed        |
|     4. safety (same as teleop): 1.5 rad/s, lead <= 0.08 rad vs measured, joint limits - 0.05 rad              |
|        stop on protective stop / desync > 0.2 rad / stale observation                                         |
|     5. gripper: output > 0.5 -> 1150 (close), else 0                                                         |
+--------------------------------------------------------------------------------------------------------------+
   |  websocket ws://localhost:8000  (msgpack)                        ^
   |  obs: state (7), 2 images (480x640x3 uint8), prompt              |  actions (10, 7): arm 6 absolute [rad] + gripper 0..1
   v                                                                  |
+--------------------------------------------------------------------------------------------------------------+
| [C] POLICY SERVER  ~/openpi: uv run scripts/serve_policy.py policy:checkpoint ...   (GPU ~8.5 GB, JAX)        |
|     UR5Inputs -> Normalize (norm stats from checkpoint assets/) -> resize 224 -> pi0.5 -> Unnormalize         |
|     -> AbsoluteActions (delta + state for the 6 arm joints) -> UR5Outputs (first 7 dims)                       |
|     checkpoint: ~/openpi/checkpoints/pi05_ur5/full_first13_10k/9999/{params, assets}                          |
+--------------------------------------------------------------------------------------------------------------+

 [D] OPERATOR: watches the sim GUI; when the drone is grasped -> python3 isaacsim/scripts/drone_cmd.py kill
```

- [B] 는 텔레옵 노드 (`teleop/xr_teleop_node.py`) 와 같은 자리: sim 은 누가 명령을 보내는지 모른다 (원칙 2). 그래서 sim 쪽 코드는 그대로
- [C] 의 입력·출력 변환은 학습 때와 같은 config (`pi05_ur5`, `~/openpi` `config.py`) 에서 온다. 정규화 통계는 체크포인트 안 `assets/` 의 것
- 녹화는 하지 않는다 (녹화 도구를 띄우면 정책 실행도 bag 으로 남길 수 있지만, 지금은 안 씀)

### 3.2 chunk 실행 시각 (sim time)

```text
 sim time --->      t_obs                                          t_obs + 0.4 s
 chunk k            | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |      (1 step = 40 ms = dataset 25 Hz)
                    ^ observation (image stamp)
                    |<-- inference ~0.18 s -->|                     steps already passed are skipped
                                              |== executed (60 Hz, interpolated) ==>
                                      ^ step 4: request chunk k+1 in background (current obs)
 chunk k+1                                    | 0 | 1 | 2 | 3 | 4 | ...   arrives ~4 steps after its own t_obs,
                                              switch as soon as it arrives (no stop between chunks)
 if chunk k+1 is late and step 9 is reached -> hold step 9 (counted as "held_frac", was 0 % in tests)
```

- 칸 i = sim `t_obs + i/25 s` 의 명령 (데이터셋 `action[t]` = 프레임 t 직전 최신 명령). 지나간 칸을 건너뛰어 추론 지연만큼 늦은 명령을 보내지 않는다
- 처음 만든 동기 방식 (추론 → 실행 → 추론) 은 추론 동안 로봇이 멈춰 움직임이 끊겼다 → 지금의 비동기 방식으로 바꿈 (2026-10-23)
- 처음 시험 때 "관측이 오래됨" 에러: 카메라는 `/joint_states` 보다 1 ~ 2 프레임 늦게 도착한다 → 이미지 시각에 맞는 관절·그리퍼를 골라 짝짓게 고침 (stage1 과 같은 방식)

---

## 4. 실행 명령

### 4.1 처음 한 번

```bash
# 클라이언트 venv (레포 밖, 시스템 ROS python 을 읽기만, pip 은 venv 안에만)
python3 -m venv --system-site-packages ~/openpi_client/venv
~/openpi_client/venv/bin/pip install -e ~/openpi/packages/openpi-client

# 체크포인트: 서버에서 9999/params, 9999/assets 를 USB 로 → 데스크탑
mkdir -p ~/openpi/checkpoints/pi05_ur5/full_first13_10k/9999
cp -r <USB>/params <USB>/assets ~/openpi/checkpoints/pi05_ur5/full_first13_10k/9999/
```

### 4.2 매번 (터미널 4 개)

```bash
# 터미널 1: sim GUI (보조 뷰포트 끔). "[sim_ros2] 준비: ..." 줄까지 약 30 s (PX4 이륙 포함)
cd ~/data_collection_ur5_gripper && source /opt/ros/jazzy/setup.bash
~/isaacsim/python.sh isaacsim/scripts/sim_ros2.py --init-pose under_drone --drone-targets --mode hover --hover mid --seed 0 --aux-views off

# 터미널 2: 정책 서버. "server listening on 0.0.0.0:8000" 까지 (로딩 약 5 s)
cd ~/openpi && XLA_PYTHON_CLIENT_PREALLOCATE=false uv run scripts/serve_policy.py policy:checkpoint \
  --policy.config=pi05_ur5 --policy.dir=checkpoints/pi05_ur5/full_first13_10k/9999
#   LoRA: --policy.config=pi05_ur5_lora --policy.dir=checkpoints/pi05_ur5_lora/smoke_first13/99

# 터미널 3: 클라이언트. 에피소드마다 /sim/reset (약 24 s) → 정책 30 s. 첫 추론 (서버 컴파일 약 11 s) 은 시작 전에 한 번
cd ~/data_collection_ur5_gripper && source /opt/ros/jazzy/setup.bash
~/openpi_client/venv/bin/python policy/policy_client_node.py --reset --episodes 5 --duration 30

# 터미널 4: 사람이 GUI 를 보고 잡혔으면 드론 모터 정지
cd ~/data_collection_ur5_gripper && source /opt/ros/jazzy/setup.bash
python3 isaacsim/scripts/drone_cmd.py kill            # 또는 ros2 service call /sim/drone_kill std_srvs/srv/Trigger
```

| 클라이언트 옵션 | 뜻 |
|----------------|-----|
| `--duration S` (필수) | 에피소드 길이 [sim s]. 수집 에피소드는 12 ~ 30 s |
| `--reset` | 에피소드마다 먼저 `/sim/reset` (PX4 약 24 s, `--flight geometric` sim 이면 약 2 s) |
| `--episodes N` | 리셋 → 실행을 N 번 (2 이상이면 `--reset` 필수) |
| `--auto-kill S` | (선택) 닫기 명령 S 초 뒤 그리퍼가 막혀 있으면 (present < 0.9) 자동으로 모터 정지. 평가 기본은 쓰지 않고 사람이 끈다 |
| `--host`, `--port` | 정책 서버가 다른 PC 일 때 |

- 같은 `--seed` 로 sim 을 띄우면 드론 기준 위치 순서가 같아 정책끼리 같은 조건에서 비교할 수 있다
- 끝낼 때: 클라이언트는 에피소드가 끝나면 스스로 끝남 (중간에는 Ctrl+C), 정책 서버 Ctrl+C, sim 창 닫기

### 4.3 터미널에서 볼 것

| 줄 | 뜻 |
|----|-----|
| `[policy] 첫 추론 (서버 컴파일 포함) 11.x s` | 정책 서버 준비 끝 |
| `[policy] 리셋 완료: 드론 기준 위치 (base) [...], 목록 점 x45_y+20_z70` | 이번 에피소드의 드론 위치 |
| `[policy] 그리퍼 닫기 / 열기 (에피소드 t ... s, 출력 ...)` | 그리퍼 명령이 바뀔 때. 짧은 시간에 여러 번 = 정책이 오감 (지금 정책의 약점) |
| `[policy] 에피소드 끝: {...}` | `stop` (멈춘 이유: duration / protective_stop / desync), `infer_wall_median_s`, `chunk_arrive_step_median`, `held_frac` (다음 chunk 가 늦어 마지막 칸을 유지한 비율), `limited_frac` (안전 한계에 잘린 비율), `gripper_present_end` (0.2 ~ 0.3 = 드론을 쥔 채, 약 0.98 = 빈손으로 닫힘, 0 = 열림) |

---

## 5. 시험 기록

| 날짜 | 정책 | 구성 | 결과 |
|------|------|------|------|
| 2026-10-23 | LoRA smoke (100 step) | sim 헤드리스, 기하 제어기 드론 | 15 s × 2 끝까지, 보호 정지 0, 명령 60 Hz, 다음 chunk 약 4 칸 뒤, 마지막 칸 유지 0 %, 안전 한계 28 % (출력이 거침), 그리퍼가 오감 |
| 2026-10-26 | full 10k (first13, 과적합) | sim GUI, PX4, 목록 점·hover mid, seed 0, `--auto-kill 1.0` | **6 개 중 5 개 파지** (사용자 GUI 확인), 4 개는 학습에 없던 드론 위치. 안전 한계 0.7 ~ 6.5 %. 그리퍼가 오감 (3/6), 1 개 놓침 → 데이터를 더 모아 정책으로 해결 (`docs/openpi_finetuning.md` 8.3) |

---

## 6. 주의·알려진 차이

- **`XLA_PYTHON_CLIENT_PREALLOCATE=false` 없이 정책 서버를 띄우면** JAX 가 GPU 의 75 % (약 12 GB) 를 미리 잡아 sim 과 같이 못 쓴다
- 학습 이미지는 AV1 mp4 를 풀어 쓴 것이고 실행 때는 압축 없는 sim 이미지 (작은 차이. 문제가 보이면 확인)
- 정책은 rgb8 이 아닌 이미지, 이름이 다른 `/joint_states`, 66 ms 넘게 어긋난 관측을 받으면 에러로 멈춘다 (조용히 넘어가지 않음)
- 실물 (9단계) 에서는 `/joint_command` stamp 를 PC 시각으로 바꿔야 하고 (지금은 sim time 만), 닿는 영역 판정·작업 공간 제한을 정책 명령 앞에 둔다 (PLAN 6단계 "드론 기준 위치 목록")

---

## 7. 관련 파일·문서

| 파일 | 내용 |
|------|------|
| `policy/policy_client_node.py`, `policy/policy_client.yaml` | 클라이언트 노드와 설정 (항목이 빠지면 에러) |
| `~/openpi/src/openpi/training/config.py` | `pi05_ur5` (full), `pi05_ur5_lora` (LoRA), `LeRobotUR5DataConfig` |
| `~/openpi/src/openpi/policies/ur5_policy.py` | `UR5Inputs`, `UR5Outputs`, `make_ur5_example` |
| `~/openpi/scripts/serve_policy.py`, `~/openpi/docs/remote_inference.md` | 정책 서버 |
| `docs/openpi_finetuning.md` | 학습 계획·결과, 평가 결과 (8.3) |
| `docs/sim_ros2_interface.md` | sim 토픽·서비스 |
| `teleop/xr_teleop_node.py` | 같은 자리의 텔레옵 노드 (안전 한계 값의 출처) |
