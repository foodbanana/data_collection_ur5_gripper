# openpi π0.5 파인튜닝 계획 (7단계 첫 학습 루프)

작성: 2026-10-23 · 대상 데이터 `lerobot_dataset_v21/local/drone_grasp_tgt_hmid_first13` (첫 소량 수집, `docs/PLAN.md` 6단계), 브랜치 `isaacsim_v6.1.0` · LoRA smoke 통과 (2026-10-23, 6.5절)

sim 에서 모은 데이터로 openpi π0.5 를 파인튜닝하는 순서를 정리한다. 먼저 LoRA 100 step smoke test 로 데이터 → 학습 경로가 끝까지 도는지 보고,
그 뒤 full 파인튜닝 (GPU 2 장, FSDP) 이 가능한지 잰다. 상태: **1 ~ 4단계 끝 (2026-10-23): LoRA smoke 통과, full smoke 는 GPU 2 장 FSDP·batch 16 으로 통과 (batch 32 는 OOM)**.

---

## 1. 한눈에 보기

| 항목 | 내용 |
|------|------|
| 모델 | π0.5 (`pi05_base` 체크포인트에서 시작) |
| 학습 방식 | ① LoRA (`pi05_ur5_lora`, 이미 리허설 데이터로 smoke 통과) → ② full (`pi05_ur5`, 새로 만듦, GPU 2 장 FSDP) |
| 데이터 | `foodbanana/drone_grasp_tgt_hmid_first13`: 13 에피소드, 6337 프레임, 25 fps, 35 MB, 검수 5/5 |
| 코드 | openpi fork `~/openpi` (github `foodbanana/openpi`). 데스크탑에서 고치고 push → 서버에서 pull |
| 데스크탑 | 코드 수정, norm stats, 데이터 로더 확인 (학습 안 함, GPU 16 GB 로는 LoRA 도 부족) |
| 학습 서버 | 연구실 서버 1 (RTX 6000 Ada 48 GB × 4, 드라이버 535 / CUDA 12.2). 작업 폴더 `~/taeunglee/openpi`, 데이터 `~/taeunglee/datasets`. 공용 계정이라 다른 사람 환경을 건드리지 않는다 |
| 서버 실행 순서 | 런북 `~/openpi/docs/server_training_runbook.md` (0 ~ 9절) |
| 기록 | wandb |

---

## 2. 데이터 → 모델 입력

| 데이터셋 키 | openpi 입력 | 내용 |
|-------------|-------------|------|
| `observation.state` (7) | `state` → 32 로 패딩 | 팔 6 관절 rad + 그리퍼 present 0~1 |
| `action` (7) | `actions` (10, 7) → 32 로 패딩 | `/joint_command` 절대 관절 목표 + 그리퍼 goal {0, 1}. 관절 6 개는 `DeltaActions(make_bool_mask(6, -1))` 로 delta, 그리퍼는 절대값 |
| `observation.images.third_view` | `base_0_rgb` (224×224) | 외부 카메라 |
| `observation.images.wrist` | `left_wrist_0_rgb` (224×224) | 손목 카메라 |
| (없음) | `right_wrist_0_rgb` | 0 이미지, mask False |
| `task` | `prompt` | `"pick up the drone"` |
| `observation.base_imu` | 쓰지 않음 | 고정 베이스라 상수 (repack 에 넣지 않으면 무시됨) |

- action horizon 10 = 0.4 s (25 Hz). 리허설 설정 그대로
- norm stats 는 우리 데이터로 계산 (quantile 정규화). 그리퍼 닫힘 프레임이 23.9 % 라 action q01 = 0, q99 = 1 로 잡힌다 (1 % 미만이면 q99 = 0 이 되어 정규화가 무너짐)
- 추론 때 그리퍼 출력은 0.5 기준으로 0 / 1150 으로 나눠 `/gripper/command` 로 보낸다

---

## 3. 리허설 (2026-09) 과 다른 점

리허설 데이터 `foodbanana/ur5_gripper_drone_rehearsal` (실물 freedrive) 로 서버 LoRA smoke 100 step 을 이미 통과했다 (loss 0.12 → 0.075, step 당 약 4.3 s, batch 32, GPU 1 장).
그때 고친 4 가지 (pyarrow 를 jax 보다 먼저 import, `action` 단수 키, 서버에 FFmpeg 없음 → `video_backend="pyav"`, repack 에서 prompt 유지) 는 `~/openpi` 에 커밋되어 있다.

| 항목 | 리허설 | 지금 | 할 일 |
|------|--------|------|-------|
| 손목 카메라 키 | `observation.images.head` | `observation.images.wrist` (2026-10-05 스키마) | **repack 수정 필수** (안 고치면 KeyError) |
| 그리퍼 | raw 0~1150 | state 0~1, action {0, 1} | norm stats 를 다시 계산하므로 코드 영향 없음. 주석·추론 이진화 기준만 |
| 팔 action | freedrive 다음 프레임 (`next_state`) | 텔레옵 `/joint_command` (`command`) | 둘 다 절대 관절 목표라 DeltaActions 그대로 |
| repo_id | `foodbanana/ur5_gripper_drone_rehearsal` | `foodbanana/drone_grasp_tgt_hmid_first13` | config 에 직접 적음 (`compute_norm_stats.py` 는 `--data.repo-id` 를 받지 않음) |

---

## 4. 1단계: `~/openpi` 코드 수정 (데스크탑)

| 파일 | 수정 |
|------|------|
| `src/openpi/training/config.py` `LeRobotUR5DataConfig` | repack `"observation/wrist_image": "observation.images.wrist"`, 주석의 `head` → `wrist`, "freedrive actions" → 절대 관절 목표 (텔레옵·freedrive 공통) |
| 같은 파일 `pi05_ur5_lora` | `repo_id="foodbanana/drone_grasp_tgt_hmid_first13"`. 나머지 (action_horizon 10, batch 32, pyav, LoRA freeze) 그대로 |
| `src/openpi/policies/ur5_policy.py` | state 그리퍼 주석 raw 0~1150 → 0~1 |
| `docs/server_training_runbook.md` | 카메라 확인 코드 `head` → `wrist`, 9절 그리퍼 이진화 = 출력 0.5 기준 → 0 / 1150 |

---

## 5. 2단계: 데스크탑 확인 (학습 없음)

```bash
# 데이터셋을 LeRobot 캐시에 복사 (원본은 그대로)
mkdir -p ~/.cache/huggingface/lerobot/foodbanana
cp -r ~/data_collection_ur5_gripper/lerobot_dataset_v21/local/drone_grasp_tgt_hmid_first13 ~/.cache/huggingface/lerobot/foodbanana/

cd ~/openpi
uv run python -c "from openpi.training import config; print(config.get_config('pi05_ur5_lora').data)"
uv run scripts/compute_norm_stats.py --config-name pi05_ur5_lora 2>&1 | tee norm_stats_first13.log
echo "exit: ${PIPESTATUS[0]}"
```

| 확인 | 기준 |
|------|------|
| norm stats | exit 0. `assets/pi05_ur5_lora/foodbanana/drone_grasp_tgt_hmid_first13/norm_stats.json` 에서 7 차원 모두 std·(q99 − q01) 가 0 이 아님, 그리퍼 action q01 0 / q99 1 |
| 데이터 로더 첫 배치 (batch 2) | 이미지 3 개 224×224, state (32), actions (10, 32), actions 그리퍼 열이 0 / 1, prompt 토큰 있음 |
| 카메라 | 첫 배치의 `base_0_rgb`·`left_wrist_0_rgb` 를 png 로 저장해 눈으로: base = third view, 왼쪽 손목 = 손목 카메라, 오른쪽 = 검정 |

통과하면 `~/openpi` 를 커밋하고 `git push origin main`.

---

## 6. 3단계: 서버 LoRA smoke test (100 step)

런북 0 ~ 5절 순서. 서버는 AnyDesk 로 사람이 직접.

```bash
nvidia-smi --query-gpu=index,memory.used,memory.total,utilization.gpu --format=csv   # 빈 GPU 확인
df -h ~

# USB 로 옮긴 데이터셋을 캐시에
mkdir -p ~/.cache/huggingface/lerobot/foodbanana
cp -r ~/taeunglee/datasets/drone_grasp_tgt_hmid_first13 ~/.cache/huggingface/lerobot/foodbanana/

cd ~/taeunglee/openpi && git pull origin main
uv run scripts/compute_norm_stats.py --config-name pi05_ur5_lora 2>&1 | tee norm_stats_first13.log   # 데스크탑 값과 같아야 함
echo "exit: ${PIPESTATUS[0]}"

CUDA_VISIBLE_DEVICES=<빈 GPU> XLA_PYTHON_CLIENT_MEM_FRACTION=0.9 uv run scripts/train.py pi05_ur5_lora \
  --exp-name=smoke_first13 --project-name=ur5_sim \
  --num-train-steps=100 --save-interval=50 --log-interval=10 \
  --overwrite 2>&1 | tee train_smoke_first13.log
echo "exit: ${PIPESTATUS[0]}"
```

**통과 기준** (런북 5절)

| 항목 | 기준 |
|------|------|
| 종료 | exit 0 |
| loss | NaN 없이 내려감 |
| `param_norm` | 거의 그대로 (LoRA freeze 가 됨) |
| 체크포인트 | `checkpoints/pi05_ur5_lora/smoke_first13/99` |
| wandb `camera_views` (step 0) | 왼쪽 = third view, 가운데 = 손목, 오른쪽 = 검정 |
| (선택) 추론 | `serve_policy.py` 로 99 step 체크포인트를 띄우고 `make_ur5_example()` 형식 입력 → actions (10, 7) |

### 6.5 결과 (2026-10-23)

**데스크탑 확인 (2단계)** — openpi 커밋 `9408e3b` (손목 키 `wrist`, repo_id)
- norm stats (198 배치, 약 1 분): 0 에 가까운 std 없음. 그리퍼 state q01 0 / q99 0.268, action q01 0 / q99 0.9998. 팔 action 은 delta 기준 (std 0.017 ~ 0.030 rad)
- 데이터 로더 첫 배치: 이미지 3 개 224×224 (오른쪽 손목 0, mask False), state (32), actions (10, 32), 8 번째 차원부터 0
- action 이 state 를 1 프레임 앞섬: action[t] 와 state[t+k] 의 팔 RMS 가 k = 0, 1, 2 에서 0.268°, **0.063°**, 0.231° → 열이 바뀌지 않음 (명령 → 움직임 지연 40 ms 와 같음)

**서버 smoke (3단계)** — 서버 1, GPU 0·1 (RTX 6000 Ada, data parallel, 장당 batch 16), 디스크 여유 52 GB 에서 체크포인트 1 개만 저장 (`--save-interval=100`)
- norm stats: 데스크탑 값과 소수 넷째 자리까지 같음 (pyav, 3 분 32 초)
- 명령: `CUDA_VISIBLE_DEVICES=0,1 XLA_PYTHON_CLIENT_MEM_FRACTION=0.9 uv run scripts/train.py pi05_ur5_lora --exp-name=smoke_first13 --project-name=ur5_sim --num-train-steps=100 --save-interval=100 --log-interval=10 --overwrite`

| 항목 | 결과 |
|------|------|
| 종료 | exit 0, 약 6 분 (컴파일·저장 포함) |
| loss (step 0 → 90) | 0.0736 → 0.0514 (리허설 0.122 → 0.075). warmup 1000 step 중 100 step 이라 lr 이 최고값의 1/10 까지만 |
| grad_norm | 0.45 → 0.32 |
| `param_norm` | 1803.7705 → 1803.7708 (LoRA freeze 됨) |
| 속도 | **2.5 s/step** (리허설 GPU 1 장 4.3 s/step → 약 1.7 배) |
| 체크포인트 | `checkpoints/pi05_ur5_lora/smoke_first13/99` (약 11 GB, 저장 25 s) |
| wandb `camera_views` | 왼쪽 third view, 가운데 손목, 오른쪽 검정 (샘플 5 개 모두) — run `ur5_sim/runs/hlgqsy6r` |

- 추론 경로 (서버 GPU 1 장, `policy_config.create_trained_policy` + `ur5_policy.make_ur5_example()` 가짜 입력): actions **(10, 7)**, 팔 값 = 랜덤 state + 작은 delta (step 간 0.001 ~ 0.002 rad, `AbsoluteActions` 동작), 그리퍼 ≈ 0. 에러 없음
- smoke 체크포인트 (9.0 GB) 는 USB 로 옮기고 서버에서 지움
- 컴파일 때 XLA `slow_operation_alarm` (`Constant folding … > 1s`, `pi0.py:41` cumsum, `[32, 978]`) 경고가 나오지만 에러가 아니다
- 서버 디스크 (2026-10-23): 98 % (여유 43 GB) → 리허설 smoke 체크포인트 (8.9 GB) 를 지워 52 GB. **본 학습 전에 저장 간격·보관 개수를 디스크에 맞춰 정한다** (체크포인트 하나 약 11 GB)

---

## 7. 4단계: full 파인튜닝 가능성 (GPU 2 장, LoRA smoke 통과 뒤)

openpi README 의 메모리 기준 (GPU 1 장): 추론 > 8 GB, LoRA > 22.5 GB, **full > 70 GB**. 서버 1 의 GPU 는 장당 48 GB 라 1 장으로는 full 이 안 된다.
2 장을 쓸 수 있게 되어 (2026-10-23) FSDP (`fsdp_devices`, `config.py`) 로 파라미터·gradient·optimizer 상태를 두 장에 나누면 합계 96 GB 로 70 GB 를 넘는다.
activation 은 깔끔하게 반으로 나뉘지 않으므로 **장당 48 GB 안에 들어가는지는 돌려 봐야 안다**.

- 새 config `pi05_ur5` (full, `~/openpi` `config.py` `pi05_ur5_lora` 바로 뒤): `pi05_ur5_lora` 에서 LoRA variant·`freeze_filter` 를 빼고 EMA 끔 (EMA 는 파라미터 사본을 하나 더 들고 있어 메모리를 씀),
  `fsdp_devices=2` 를 config 에 적음 (명령줄에서 빠뜨려도 됨). 데이터·batch 32·lr schedule 은 LoRA 와 같음.
  norm stats 는 config 이름별 폴더 (`assets/pi05_ur5/…`) 라 다시 계산 (데스크탑에서 LoRA 값과 같음 확인, 2026-10-23)
- smoke 100 step:
  ```bash
  CUDA_VISIBLE_DEVICES=<빈 GPU 2 개> XLA_PYTHON_CLIENT_MEM_FRACTION=0.9 uv run scripts/train.py pi05_ur5 \
    --exp-name=smoke_full_first13 --project-name=ur5_sim --fsdp-devices=2 \
    --num-train-steps=100 --save-interval=50 --log-interval=10 --overwrite 2>&1 | tee train_smoke_full_first13.log
  ```
- 메모리가 모자라면 (README 순서): `XLA_PYTHON_CLIENT_MEM_FRACTION` 확인 → EMA 끔 (이미) → `--batch-size=16`
- 잴 것: 장당 메모리 (`nvidia-smi`), step 당 시간 (LoRA 1 장 약 4.3 s 와 비교), loss
- **결과 (2026-10-23)**: GPU 0·1, FSDP 2, 디스크 여유 78 GB

  | 시도 | 결과 |
  |------|------|
  | batch 32 | **OOM** (첫 step). XLA: 한 step 에 약 32 GB 필요한데 남은 공간 약 22.8 GB (`Can't reduce memory use below 22.78GiB … only reduced to 32.17GiB`), 13.1 GB 버퍼 할당 실패 (`RESOURCE_EXHAUSTED`). 장당 44 GB 중 약 21 GB 는 나눈 파라미터·optimizer 상태 |
  | **batch 16** | **통과**, exit 0. 장당 8 |

  | 항목 | full (batch 16) | LoRA (batch 32, 6.5절) |
  |------|-----------------|------------------------|
  | loss (step 0 → 90) | 0.0818 → **0.0374** | 0.0736 → 0.0514 |
  | grad_norm | 1.34 → 0.57 | 0.45 → 0.32 |
  | `param_norm` | 1802.3865 → 1802.3860 (가중치가 바뀜 = full) | 그대로 (freeze) |
  | 속도 | **1.9 s/step** (샘플당 0.12 s) | 2.5 s/step (샘플당 0.08 s) |
  | 같은 샘플 수당 시간 | LoRA 의 약 1.5 배 | 1 |
  | 체크포인트 | **31 GB** (step 99) | 약 9 GB |

  - loss 가 더 내려간 것은 바꾸는 파라미터가 많아서. 13 에피소드에서는 과적합일 수 있어 어느 쪽이 나은지는 sim 평가로만 판단한다
  - 본 학습을 full 로 하면 체크포인트 하나 31 GB → 서버 디스크 (여유 수십 GB) 로는 하나만 둘 수 있다. 저장 간격·보관 개수, 끝난 뒤 바로 옮기기를 미리 정한다
  - `config.py` `pi05_ur5` 의 batch 를 16 으로 고침 (`~/openpi`)
- **지금 데이터 (13 에피소드) 로는 full 이 과적합하기 쉽다.** 바꾸는 파라미터가 LoRA 보다 훨씬 많다. full 본 학습은 데이터가 수십 ~ 수백 에피소드로 늘었을 때 LoRA 와 비교해 정한다

---

## 8. 그 뒤 (본 학습, 7단계)

- step 수: 1 에포크 = 프레임 수 ÷ batch. 지금 6337 ÷ 32 ≈ 198 step. 기본값 30 000 step 은 소량 데이터에서 과적합 (런북 6절 표)
- 본 학습은 `nohup … &` 로 (AnyDesk 가 끊겨도 계속), 중단되면 `--resume` (`--overwrite` 와 같이 쓰지 않음)
- norm stats 비교 실험 후보: openpi UR5 예제 (`~/openpi/examples/ur5/README.md`, π0 full) 는 사전학습 UR5e 통계를 재사용한다 (`AssetsConfig(assets_dir="gs://openpi-assets/checkpoints/pi05_base/assets", asset_id="ur5e")`, `~/openpi/docs/norm_stats.md` 는 둘 다 해 보라고 권함).
  지금은 우리 데이터로 계산한 것을 쓴다: 사전학습 `ur5e` 의 팔 delta q01 ~ q99 가 약 ±0.6 ~ 1.2 rad 로 우리 (±0.05 ~ 0.10 rad, horizon 10·25 Hz) 의 약 10 배라 재사용하면 팔 action 이 정규화 범위의 ±0.1 근처에 몰리고,
  state 도 관절 0·장착 자세가 달라 범위가 다르다 (shoulder_pan 평균 2.82 대 −0.23 rad). 그리퍼는 둘 다 0 ~ 1 (2026-10-23 GCS 에서 확인)
- 서버 학습이 느리면 (GPU 사용률 낮음): pyav 대신 conda FFmpeg + torchcodec (`~/openpi/docs/video_backend_ffmpeg.md`)
- sim 폐루프 평가: openpi 정책 서버 ↔ sim 클라이언트 (`/joint_command`·`/gripper/command` 를 내는 노드), 같은 `--seed` 로 성공률 측정 (PLAN 7단계). 정책 명령 앞에 닿는 영역 판정·작업 공간 제한 (PLAN 6단계 "드론 기준 위치 목록")
- 학습에 쓴 데이터셋·config·step·체크포인트를 `docs/PLAN.md` 7단계에 기록하고, `docs/architecture.md` [6] 아래 "openpi … NOT STARTED" 를 고친다

### 8.1 full 오래 학습 (2026-10-23 시작)

first13 그대로, 데이터를 더 모으기 전에 학습 → 추론 → sim GUI 전체 고리를 보려고. 과적합이어도 된다.

```bash
CUDA_VISIBLE_DEVICES=0,1 XLA_PYTHON_CLIENT_MEM_FRACTION=0.9 uv run scripts/train.py pi05_ur5 \
  --exp-name=full_first13_10k --project-name=ur5_sim --batch-size=16 \
  --num-train-steps=10000 --lr-schedule.warmup-steps=1000 --lr-schedule.decay-steps=10000 \
  --save-interval=10000 --keep-period=None --log-interval=100 --overwrite 2>&1 | tee train_full_first13_10k.log
```

- 10 000 step = 약 25 에포크, 1.9 s/step → 약 5 시간 15 분 (20:02 시작)
- lr: warmup 1000 → 2.5e-5 → cosine 으로 10 000 step 에 2.5e-6 (기본 decay 30 000 이면 거의 안 내려간 채 끝남)
- 체크포인트는 마지막 (step 9999) 한 번만: 중간에 저장하면 새것을 다 쓴 뒤 옛것을 지워 순간 2 개 (62 GB), `keep_period=None` (5000 배수를 남기지 않음). 서버 디스크는 예약함
- 끝나면 `9999/params`·`assets` (추론에 필요한 것, 약 7 GB) 만 USB → 데스크탑, 서버 체크포인트는 지운다

### 8.2 정책 클라이언트 노드 (sim GUI 에서 정책 돌리기)

`policy/policy_client_node.py` + `policy/policy_client.yaml` (2026-10-23). 텔레옵 노드 자리를 대신한다 (CLAUDE.md 원칙 2: `/joint_command`·`/gripper/command` 만 보냄)

| 항목 | 내용 |
|------|------|
| 관측 | 데이터셋과 같은 모양: state = 팔 6 + present / 1150, base = third view, wrist = 손목 (rgb8 640x480 그대로, resize 는 서버), prompt `"pick up the drone"` |
| 시각 맞춤 | 카메라는 `/joint_states` 보다 1 ~ 2 프레임 늦게 도착 → 기준 시각 = 두 카메라 중 늦은 쪽의 최신 이미지, 그 이전의 관절·그리퍼를 고름 (stage1 과 같게). 66 ms 넘게 어긋나면 에러 |
| 실행 | chunk 칸 i = sim t_obs + i/25 s 의 명령. 60 Hz (sim time) 로 칸 사이 선형 보간해 보냄. 4 칸째에 다음 추론을 뒤에서 시작 (그동안 계속 실행), 결과가 오면 바꿈. chunk 끝에 닿으면 마지막 칸 유지 |
| 그리퍼 | 출력 > 0.5 → 1150 (닫기), 아니면 0, 바뀔 때만 보냄 |
| 안전 | 텔레옵과 같은 값: 관절 목표 속도 1.5 rad/s, 측정 대비 앞섬 0.08 rad (보호 정지 5° 아래), 관절 한계 안쪽 0.05 rad. 보호 정지·측정 대비 0.2 rad 어긋남 (리셋)·관측 오래됨이면 멈춤 |
| 옵션 | `--reset` (에피소드마다 `/sim/reset`), `--episodes N`, `--duration S` [sim s, 필수], `--auto-kill S` (닫기 명령 S 초 뒤 그리퍼가 막혀 있으면 (present < 0.9) 드론 모터 정지) |
| 환경 | `~/openpi_client/venv` (레포 밖, `--system-site-packages` 로 시스템 ROS python 을 읽기만, `pip install -e ~/openpi/packages/openpi-client`) |

**데스크탑 GPU (RTX 4070 Ti SUPER 16 GB) 실측**: 정책 (π0.5) 약 8.4 GB·추론 0.13 s (단독) / 0.19 s (sim 과 같이), sim 헤드리스 최대 약 3.6 GB (화면 포함) → 같이 약 12 GB.
정책 서버는 `XLA_PYTHON_CLIENT_PREALLOCATE=false` 로 띄운다 (기본은 GPU 의 75 % 를 미리 잡아 sim 과 같이 못 씀). 첫 추론은 서버 컴파일로 약 15 s → 노드가 에피소드 전에 한 번 미리 추론

**시험 (LoRA smoke 체크포인트, 헤드리스, 기하 제어기 드론, 2026-10-23)**: 에피소드 2 개 × 15 s 끝까지, 보호 정지 0, RTF 1.03. 명령 893 개 / 15 s (60 Hz),
다음 chunk 도착 = 약 4 칸 (0.19 s) 뒤, 마지막 칸 유지 0 %, 안전 한계로 잘린 비율 28 % (100 step 정책이라 출력이 거칢), 그리퍼가 닫기·열기를 오감 (약한 정책)

```bash
# 터미널 1: sim (GUI)
source /opt/ros/jazzy/setup.bash
~/isaacsim/python.sh isaacsim/scripts/sim_ros2.py --init-pose under_drone --drone-targets --mode hover --hover mid --seed 0
# 터미널 2: 정책 서버 (데스크탑 ~/openpi)
cd ~/openpi && XLA_PYTHON_CLIENT_PREALLOCATE=false uv run scripts/serve_policy.py policy:checkpoint \
  --policy.config=pi05_ur5 --policy.dir=checkpoints/pi05_ur5/full_first13_10k/9999
# 터미널 3: 클라이언트 (sim 의 "준비" 줄과 서버의 "server listening" 뒤)
source /opt/ros/jazzy/setup.bash
~/openpi_client/venv/bin/python policy/policy_client_node.py --reset --episodes 5 --duration 30 --auto-kill 1.0
```

- 학습 영상은 AV1 mp4 를 풀어 쓴 것이고 실행 때는 압축 없는 sim 이미지다 (작은 차이. 문제가 보이면 확인)
- 아직 없는 것: 성공 자동 판정·성공률 집계 (지금은 그리퍼 막힘 = 파지 신호만), 닿는 영역 판정

---

## 9. 관련 문서

| 문서 | 내용 |
|------|------|
| `docs/PLAN.md` 6·7단계 | 첫 소량 수집 결과, 학습·sim 평가 계획 |
| `README.md`, `dataset_merge.md` | 변환·병합·검수 (데이터셋 만들기) |
| `CLAUDE.md` | 데이터셋 스키마 |
| `~/openpi/docs/server_training_runbook.md` | 서버 학습 실행 순서·문제 대응 |
| `~/openpi/docs/video_backend_ffmpeg.md` | 서버 영상 디코딩 (pyav ↔ torchcodec) |
| `~/openpi/docs/remote_inference.md` | 정책 서버·클라이언트 |
