# π0.5 action 생성 원리 (flow matching, `sample_actions`)

정책 서버 안에서 관측 (이미지 2 장 + prompt) 이 action chunk (10, 32) 가 되는 과정을 openpi 코드 기준으로 정리한다.
앞뒤 변환 (UR5Inputs → Normalize → Resize·Tokenize·Pad, Unnormalize → AbsoluteActions → UR5Outputs) 과 서버·클라이언트 구조는
`docs/policy_inference_desktop.md`. 여기서는 그 사이의 **모델** 만 다룬다.

코드: `~/openpi/src/openpi/models/pi0.py` (π0 와 π0.5 공용, `pi05=True` 로 갈림), `gemma.py` (transformer 층), `pi0_config.py` (크기·기본값).
줄 번호는 2026-10-27 기준 `~/openpi` (foodbanana fork).

## 1. 한 줄 요약

- **flow matching** = diffusion 계열 생성 방법. 잡음에서 시작해, 네트워크가 예측한 **속도 (방향)** 를 따라 10 걸음 걸어가 action 에 닿는다
- 이미지·글자는 한 번만 계산하고 (KV cache), action 쪽 작은 네트워크 (action expert) 만 10 번 돌린다 → 데스크탑 추론 0.12 s
- 시간 t 는 action expert 의 모든 층 정규화 (adaRMSNorm) 에 조건으로 들어간다 (π0.5 의 특징)

## 2. 모델 구성

```text
 이미지 3 자리 (224×224, −1~1)        prompt 토큰 (200)            잡음 섞인 action x_t (10, 32), 시간 t
        │                                 │                              │                        │
        ▼                                 ▼                              ▼                        ▼
 ┌─────────────────┐              ┌──────────────┐              ┌────────────────┐     ┌────────────────────┐
 │ SigLIP So400m/14│              │ Gemma 단어   │              │ action_in_proj │     │ sincos 임베딩       │
 │ 장당 16×16 =     │              │ 임베딩       │              │ Linear 32→1024 │     │ → time_mlp_in/out  │
 │ 256 토큰 → 2048  │              │ → 2048       │              └───────┬────────┘     │   (swish, 1024)    │
 └────────┬────────┘              └──────┬───────┘                      │              └─────────┬──────────┘
          └──────────── prefix 968 토큰 ──┘                     suffix 10 토큰                adaRMS 조건
                         │                                              │                        │
                         ▼                                              ▼                        │
          ┌────────────────────────────────────────────────────────────────────────────┐        │
          │ 18 층 transformer. 층마다 attention 은 prefix + suffix 를 한 줄로 이어 같이 계산,  │        │
          │ 나머지 가중치 (QKV·FFN·정규화) 는 토큰 종류마다 따로:                            │◄───────┘
          │   prefix → PaliGemma (Gemma 2B, 폭 2048)   suffix → action expert (Gemma 300M,  │
          │                                                       폭 1024, adaRMSNorm)   │
          └────────────────────────────────────────────────────────────┬───────────────┘
                                                                       ▼ suffix 출력 (10, 1024)
                                                              action_out_proj Linear 1024→32
                                                                       ▼
                                                              속도 v_t (10, 32)
```

| 부분 | 크기 | 코드 |
|---|---|---|
| SigLIP (이미지 인코더) | So400m/14 (약 400M), 224×224 → 패치 14×14 → 256 토큰 | `pi0.py` `__init__` L81-90 |
| PaliGemma 언어 모델 | `gemma_2b`: 폭 2048, 18 층, FFN 16384, head 8 | `gemma.py` `get_config` |
| action expert | `gemma_300m`: 폭 1024, 18 층, FFN 4096, head 8 (층 수가 같아야 층마다 attention 을 같이 함) | 〃 |
| 입출력 프로젝션 | `action_in_proj` 32→1024, `action_out_proj` 1024→32, `time_mlp_in/out` 1024→1024 | `pi0.py` L92-100 |

- 두 transformer 는 **층마다 attention 만 공유** 한다 (토큰을 한 줄로 이어 attention 계산). 그래서 action 토큰이 이미지·글자 토큰을 직접 볼 수 있다
- PaliGemma (SigLIP + Gemma 2B) 는 원래 이미지-글자 모델이고, action expert 는 action 을 만들려고 붙인 작은 transformer

## 3. 토큰 구성 (우리 입력 기준)

| 구간 | 토큰 | 개수 | 유효 여부 |
|---|---|---|---|
| prefix | `base_0_rgb` (third view) | 256 | 유효 |
| prefix | `left_wrist_0_rgb` (손목) | 256 | 유효 |
| prefix | `right_wrist_0_rgb` (검은 이미지) | 256 | **mask False** → 아무도 안 봄 (SigLIP 계산은 함) |
| prefix | prompt `"pick up the drone"` + `\n` | 200 (진짜 몇 개 + 채움) | 진짜 토큰만 유효 |
| suffix | action 칸 0~9 | 10 | 유효 |

- `positions = cumsum(input_mask) − 1`: 무효 토큰은 위치 번호를 늘리지 않는다
- **state 토큰이 없다**: π0 는 suffix 앞에 state 토큰 1 개를 넣고 (`pi0.py` L151-157), π0.5 는 state 를 prompt 글자로 넣는 방식 (`discrete_state_input=True`) 을 쓴다.
  우리 `pi05_ur5`·`pi05_ur5_lora` 는 `discrete_state_input=False` 라 **어느 쪽으로도 state 가 모델에 들어가지 않는다** (state 는 출력 delta → 절대각 변환에만 쓰임). openpi `pi05_libero` 도 같은 설정

## 4. 누가 누구를 보나 (attention mask)

`make_attn_mask` (`pi0.py` L19-44): 토큰마다 `ar_mask` 를 두고, 누적합이 자기보다 작거나 같은 토큰만 본다.
prefix 는 전부 False, suffix 는 `[True, False × 9]` (`embed_prefix` L125·L133, `embed_suffix` L182).

| 보는 쪽 \ 보이는 쪽 | 이미지·글자 (prefix) | action 10 칸 (suffix) |
|---|---|---|
| 이미지·글자 | 서로 다 봄 (양방향) | **못 봄** |
| action 10 칸 | 다 봄 | 서로 다 봄 (양방향) |

- prefix 가 suffix 를 못 보므로, prefix 계산 결과는 x_t·t 와 상관없이 같다 → **추론 때 prefix 를 한 번만 계산해 KV cache 로 재사용** 할 수 있는 이유
- action 10 칸은 서로 다 보므로 chunk 전체가 한 번에 (칸끼리 맞춰) 만들어진다

## 5. flow matching 원리

### 5.1 직선 경로와 속도

openpi 코드는 **t = 1 이 잡음, t = 0 이 정답 action** 이다 (π0 논문과 반대, 코드 주석 L226).

```text
 정답 action A ●──────────────────────────────● 잡음 ε ~ N(0, I)
 (t = 0)               x_t                     (t = 1)

 x_t = t·ε + (1 − t)·A          (A 와 ε 를 직선으로 섞은 점)
 u   = dx_t/dt = ε − A           (직선의 방향 = 정답 속도, t 와 무관)
```

네트워크 v_θ(x_t, t | 이미지, prompt) 가 u 를 맞히도록 학습한다. 학습이 끝나면 잡음에서 출발해 −v 방향으로 t = 1 → 0 까지 적분하면 action 이 나온다 (상미분방정식 dx/dt = v).

### 5.2 학습 (`compute_loss`, `pi0.py` L189-214)

```python
noise  = N(0, I)  모양 (B, 10, 32)
time   = Beta(1.5, 1) * 0.999 + 0.001          # 0.001 ~ 1. 밀도 ∝ t^0.5 → 잡음 쪽 (t 큰 쪽) 을 더 자주 뽑음, 평균 0.6
x_t    = time * noise + (1 - time) * actions   # actions = 정규화된 delta chunk (패딩 25 차원은 0)
u_t    = noise - actions
v_t    = action_out_proj( transformer(prefix, embed_suffix(x_t, time))[action 10 칸] )
loss   = mean((v_t - u_t)², 차원 32 평균)        # 칸마다 값 → 배치·칸 평균이 학습 loss
```

- 학습 때는 prefix + suffix 를 **한 번에** 통과 (KV cache 없음), 샘플마다 t 를 하나 뽑는다
- 이미지 augmentation (`preprocess_observation`, `train=True`): 손목이 아닌 카메라는 95 % 자르기·±5° 회전, 모든 카메라 색 흔들기. 추론 때는 안 함
- loss 는 패딩 25 차원까지 32 차원 평균이다 (패딩 차원은 action 이 0 이라 u = ε, 맞히기 쉬움)
- full 10k 학습의 loss 0.00 = 학습 데이터에서 속도를 거의 완벽히 맞힘 → 13 에피소드에 과적합으로 판단 (`docs/openpi_finetuning.md` 8장)

### 5.3 추론 (`sample_actions`, `pi0.py` L216-280)

```python
observation = preprocess_observation(train=False)     # 이미 224 라 그대로
noise = N(0, I) (1, 10, 32);  dt = -1/10

# ① prefix 한 번: 이미지 3 장 + prompt → 18 층의 key/value 저장
_, kv_cache = llm([prefix_tokens, None], ...)

# ② 오일러 적분 10 걸음
x, time = noise, 1.0
while time >= -dt/2:                                   # 1.0, 0.9, …, 0.1 → 10 번 (부동소수 오차 대비 0.05 기준)
    suffix = embed_suffix(x, time)                     # x → action 토큰, time → adaRMS 조건
    out = llm([None, suffix], kv_cache=kv_cache, ...)  # action expert 만 계산, prefix 는 cache 를 봄
    v = action_out_proj(out[:, -10:])
    x, time = x + dt * v, time + dt                    # x ← x − 0.1·v
return x                                               # (1, 10, 32) 정규화된 delta
```

| 걸음 | 시작 time | 계산 | 결과 |
|---|---|---|---|
| 0 (준비) | — | SigLIP 3 장 + Gemma 2B (prefix 968 토큰) | KV cache |
| 1 | 1.0 | action expert (suffix 10 토큰) | x 를 잡음에서 0.1 만큼 action 쪽으로 |
| 2~9 | 0.9 ~ 0.2 | 〃 | 〃 |
| 10 | 0.1 | 〃 | time 0 → x = action chunk |

- 무거운 부분 (SigLIP·2B) 은 한 번, 가벼운 부분 (300M, 토큰 10 개) 만 10 번 → 데스크탑 RTX 4070 Ti SUPER 에서 정책 단독 0.12 s, sim 과 같이 왕복 약 0.18 s (`docs/policy_inference_desktop.md` 2.2)
- 걸음 수 `num_steps` 기본 10 (`sample_actions` 인자). 서버는 바꾸지 않는다
- 반복은 `jax.lax.while_loop` 라 JIT 컴파일 한 번에 10 걸음이 같이 컴파일된다 (첫 추론 11~13 s)

## 6. 시간 t 를 넣는 방법: adaRMSNorm (π0.5)

`embed_suffix` (L159-169):

```text
t (스칼라) → sincos 위치 임베딩 (1024, 주기 0.004 ~ 4) → Linear → swish → Linear → swish → 조건 c (1024)
x_t (10, 32) → action_in_proj → action 토큰 (10, 1024)      ← t 는 토큰에 섞지 않음
```

action expert 의 층마다 정규화 두 군데 (attention 앞, FFN 앞) 가 c 로 바뀐다 (`gemma.py` `RMSNorm` L113-131, 블록 L293-338):

```text
scale, shift, gate = Dense(c) 를 셋으로 나눔          (Dense 는 0 으로 초기화 → 학습 시작 때는 보통 RMSNorm 과 같음)
h   = RMSNorm(x) · (1 + scale) + shift
x  ← x + gate · 층(h)                                 (gated residual)
```

- "지금 잡음이 얼마나 남았나 (t)" 가 모든 층의 크기·치우침·잔차 세기를 조절한다. 같은 x 라도 t 가 크면 큰 방향을, 작으면 미세 조정을 내도록
- PaliGemma 쪽 (prefix) 은 조건 없는 보통 RMSNorm (`use_adarms=[False, True]`, L80)

## 7. π0 와 π0.5 차이 (코드에서 갈리는 곳)

| | π0 (`pi05=False`) | π0.5 (`pi05=True`, 우리) |
|---|---|---|
| state | suffix 앞 연속 토큰 1 개 (`state_proj`) | prompt 글자로 (`discrete_state_input=True`). **우리는 False → 안 넣음** |
| 시간 t | action 토큰과 이어 붙여 MLP (`action_time_mlp`) | adaRMSNorm 조건 (`time_mlp`) |
| prompt 최대 길이 | 48 | 200 (state 글자가 들어갈 자리) |
| 나머지 | 같음 (SigLIP + Gemma 2B + 300M expert, flow matching 10 걸음) | |

## 8. diffusion (DDPM) 과 비교

| | DDPM 계열 | flow matching (openpi) |
|---|---|---|
| 섞는 방법 | 잡음 일정표 (β schedule) | 직선 x_t = t·ε + (1−t)·A |
| 맞히는 것 | 섞인 잡음 ε | 속도 u = ε − A |
| 생성 | 걸음마다 잡음 빼고 (DDPM 은) 새 잡음을 조금 더함 | 속도를 따라 오일러 적분, 중간에 잡음 없음 |
| 걸음 수 | 수십 ~ 수천 (빠른 sampler 로 줄임) | 10 |

## 9. 우리 작업에 주는 의미

- **출력이 매번 조금 다르다**: 시작 잡음을 요청마다 새로 뽑는다 (서버 `Policy` 가 난수 키를 요청마다 쪼갬). 같은 장면이어도 chunk 가 다를 수 있다
- **여러 동작을 표현할 수 있다**: 데이터에 서로 다른 방법이 섞여 있으면 평균을 내지 않고 잡음에 따라 하나를 고른다 (회귀 정책과 다른 점)
- **그리퍼도 연속값으로 생성된다**: 그리퍼 칸도 같은 적분으로 나오므로 0 ~ 1 사이 값이 나올 수 있고, 클라이언트가 0.5 로 자른다.
  평가 때 그리퍼가 닫기·열기를 오간 것은 출력이 0.5 근처를 오갔기 때문일 수 있다 (확인 안 함. 클라이언트 로그 `출력 0.xx` 로 볼 수 있음)
- **chunk 10 칸이 서로를 본다**: 칸끼리 양방향 attention 이라 chunk 안 궤적이 매끄럽게 나온다. chunk 와 chunk 사이 이어짐은 클라이언트 교체·보간·안전 한계가 맡는다
- **state 를 안 본다** (3장): 지금 자세는 이미지로만 안다. state 를 prompt 로 넣는 설정 (`discrete_state_input=True`, π0.5 기본) 과 비교할지는 학습 계획에서 고를 항목.
  넣는다면 state 정규화 범위도 볼 것 (그리퍼 state q99 0.268 → 빈손 닫힘 0.985 는 정규화 약 +6.3 으로 범위 밖)

## 10. 코드 위치

| 내용 | 위치 |
|---|---|
| 모델 구성 | `pi0.py` `Pi0.__init__` L66-103 |
| attention mask | `pi0.py` `make_attn_mask` L19-44 |
| prefix (이미지·글자) | `pi0.py` `embed_prefix` L105-137 |
| suffix (action·시간) | `pi0.py` `embed_suffix` L139-186 |
| 학습 loss | `pi0.py` `compute_loss` L188-214 |
| 추론 | `pi0.py` `sample_actions` L216-280 |
| adaRMSNorm, 층 | `gemma.py` `RMSNorm` L113-131, 블록 L293-338 |
| 크기·기본값 (`action_horizon` 50, `max_token_len` 200, `discrete_state_input`) | `pi0_config.py` `Pi0Config` |
| 이미지 augmentation | `model.py` `preprocess_observation` L144 |
| 우리 config (`action_horizon=10`, `discrete_state_input=False`) | `training/config.py` `pi05_ur5`, `pi05_ur5_lora` |
