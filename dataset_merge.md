# 데이터셋 병합 (dataset_merge.md)

개별 LeRobot v2.1 에피소드들을 하나의 학습용 데이터셋으로 병합하는 절차.
openpi는 에피소드 여러 개가 든 데이터셋 1개를 학습에 사용하므로, `bag 1개 = 데이터셋 1개`로
수집·변환한 개별 데이터셋들을 병합 단계에서 하나로 합친다.

- 스크립트: `lerobot_merge_episodes_v21.py`
- 환경: conda `lerobot_v2` (python 3.10 + lerobot 0.3.3, CODEBASE_VERSION v2.1)
- 실행 위치: **반드시** `~/data_collection_ur5_gripper/` (출력 경로가 상대경로로 계산됨)

## 방식

이미 만든 v2.1 데이터셋의 mp4/parquet을 이어붙이는 게 아니라,
1단계 중간 산출물(`bag_lerobot_intermediate/<bag>/`의 states.npy·actions.npy·png)에서
데이터셋을 새로 `create()` 하고, 에피소드마다 `add_frame → save_episode`를 반복한다.
`episode_index`, `index`, `task_index`, `episodes_stats.jsonl`, `info.json` 총계는 lerobot이 자동 계산한다.

- 원본(중간 산출물)은 읽기 전용 — 병합이 실패해도 원본은 안전
- 이미지 손실 없음 (png 원본에서 다시 인코딩)
- mp4 재인코딩 시간이 유일한 비용 (에피소드 수백 개로 늘면 부담 → 그때 증분 방식 검토)

## 명명 규칙

매니페스트 파일 이름과 결과 데이터셋 이름(repo_id)을 짝맞춘다.

| 매니페스트 | repo_id |
|---|---|
| `merge_<이름>.txt` | `foodbanana/<이름>` |

예: `merge_ur5_gripper_drone_rehearsal.txt` → `foodbanana/ur5_gripper_drone_rehearsal`

- `foodbanana` = HuggingFace 닉네임 (namespace)
- 배관 검증용은 `_rehearsal`, 정식 데이터는 접미사 없이 `ur5_gripper_drone` 등으로 구분

## 절차

### 0. 사전 준비 (매번)

```bash
cd ~/data_collection_ur5_gripper
conda activate lerobot_v2
```

- `cd` 안 하면: 출력 경로가 어긋나거나 스크립트를 못 찾음
- `conda activate` 안 하면: `python: command not found` (conda 밖엔 python3만 있음)

### 1. 매니페스트 작성

병합할 에피소드 목록을 텍스트 파일로. 한 줄 = `bag_lerobot_intermediate/` 아래 중간파일 폴더 이름 하나.
(완성된 v2.1 폴더 이름이 아니라 **중간 산출물 폴더** 이름)
merge_<이름>.txt
병합 대상 에피소드 목록 -> foodbanana/<이름> 생성
'#' 주석과 빈 줄은 무시됨

20260921_172851_test_clamp
20260921_173008_test_clamp
20260921_173043_test_clamp


- 넣을 것만 적고, 뺄 것은 삭제하거나 `#`으로 주석 처리
- 중간 산출물은 **지금 stage1 (schema_version 2: wrist/third_view, 그리퍼 0~1, base_imu)** 로 만든 것만. 옛 형식은 사전검사에서 중단
- `bags/_discarded/` 로 버린 에피소드는 stage1 을 돌리지 않으므로 중간 산출물이 없다 (병합에 들어가지 않음)
- 매니페스트 없이 `bag_lerobot_intermediate/` 아래 전부를 병합하려면 `--all`

### 2. 병합 실행

```bash
python lerobot_merge_episodes_v21.py \
  --manifest merge_<이름>.txt \
  --repo-id foodbanana/<이름>
```

기본값(`merge_ur5_gripper_drone.txt` / `foodbanana/ur5_gripper_drone`)을 쓸 때는 인자 생략 가능:

```bash
python lerobot_merge_episodes_v21.py
```

**인자**

| 인자 | 기본값 | 설명 |
|---|---|---|
| `--manifest` | `merge_ur5_gripper_drone.txt` | 병합 대상 목록 파일 |
| `--all` | - | 매니페스트 대신 중간 파일 폴더 아래 전부 |
| `--repo-id` | `foodbanana/ur5_gripper_drone` | 결과 데이터셋 이름 |
| `--intermediate-dir` | `bag_lerobot_intermediate` | 중간 파일 폴더 |
| `--task` | `pick up the drone` | 언어 명령 (모든 에피소드 공통) |
| `--root` | (자동) | 출력 폴더 직접 지정 시 |
| `--include-protective-stop` | 끔 | 보호 정지가 걸린 에피소드 (sim) 도 넣는다. 기본은 제외하고 출력·`merge_manifest.json` 에 남김 |
| `--trim-idle start\|end\|both` | 안 자름 | 앞 / 뒤 / 양쪽 정지 구간을 자른다. 정지 = action 과 state 가 모두 첫 / 마지막 프레임 값에서 그대로인 구간 |
| `--trim-margin` | `0.5` | [s] 자를 때 남길 정지 구간 |
| `--idle-eps-deg` | `0.1` | [deg] 팔이 이만큼 안 변하면 정지로 본다 |

정지 구간을 자를지는 검수 표의 "정지 앞/뒤 [s]" 를 보고 정한다 (녹화 시작 뒤 가만히 있는 프레임이 많이 쌓이면 정책이 "시작하면 가만히 있기" 를 배울 수 있다).
`--arm-action command` 로 변환한 에피소드는 stage1 이 이미 첫 명령 이전 구간을 뺐다.

**실행 흐름**

Phase 1 사전검사 : 프레임수·차원·해상도·fps·**팔 action 종류 (command / next_state)**·base_imu 방식 일치 + 유한값·팔 ±2π·그리퍼 state 0~1·action 이진({0, 1}) 검사
→ 하나라도 실패하면 즉시 중단 (디스크에 아무것도 안 씀). 보호 정지 에피소드는 [제외] 로 표시하고 계속
Phase 2 빌드 : create 1회 + 에피소드마다 add_frame → save_episode (mp4 av1 인코딩)
Phase 3 기록 : meta/merge_manifest.json 에 어떤 폴더가 어떤 episode_index로 갔는지 기록


출력: `lerobot_dataset_v21/foodbanana/<이름>/`

### 3. 검수 (병합 후 항상)

`<이름>` 을 실제 데이터셋 이름으로 바꿔서 실행.

```bash
env -u PYTHONPATH python inspect_dataset_v21.py lerobot_dataset_v21/foodbanana/<이름> --load     # PASS / FAIL, 종료 코드
du -sh lerobot_dataset_v21/foodbanana/<이름>/                                                    # 용량
env -u PYTHONPATH python inspect_parquet.py lerobot_dataset_v21/foodbanana/<이름>/data/chunk-000/episode_000000.parquet   # (선택) 에피소드 하나
```

**정상 기준** (`inspect_dataset_v21.py` 가 항목별로 판정)

- 1 형식: `codebase_version` v2.1, fps 25, `observation.state` [7], `action` [7], `observation.base_imu` [6], `observation.images.wrist`·`third_view` [480, 640, 3]
- 2 에피소드·프레임 수가 `info.json`·parquet·mp4 에서 일치
- 3 값 범위: 유한값, 팔 ±2π, 그리퍼 state 0~1, 그리퍼 action `{0.0, 1.0}`
- 4 파지 신호: 모든 에피소드에서 닫기 명령 중 그리퍼 state 최댓값 < 0.95 (물체에 막혀 멈춤)
- 5 `--load`: LeRobotDataset 으로 열림 (openpi 가 읽을 수 있는지 + mp4 디코딩), 이미지 `(3, 480, 640)` (CHW)
- 에피소드별 표: 앞·뒤 정지 [s], `/joint_command` 간격 (중앙값·최댓값·40 ms 넘은 횟수)

## 트러블슈팅

| 증상 | 원인 | 해결 |
|---|---|---|
| `python: command not found` | conda 안 켬 | `conda activate lerobot_v2` |
| `can't open file ...py` | 실행 위치 틀림 | `cd ~/data_collection_ur5_gripper` |
| `[중단] 매니페스트 파일 없음` | 파일명 오타 | `--manifest` 값 확인 |
| `[중단] 출력 폴더가 이미 있고...` | 같은 이름 데이터셋 존재 | 아래처럼 지우고 재실행 or `--repo-id` 변경 |
| `[중단] 중간 파일 형식이 schema_version …` | 옛 stage1 출력 | 지금 `lerobot_stage1_extract_bag.py` 로 다시 변환 |
| `[중단] arm_action 가 앞 에피소드와 다름` | freedrive (`next_state`) 와 텔레옵 (`command`) 에피소드를 섞음 | 매니페스트를 나눠 따로 병합 |
| `[중단] 그리퍼 action 이 이진 … 아님` | 그리퍼 데이터 이상 | 해당 bag 재수집 or 제외 |
| `ModuleNotFoundError` (rclpy 등이 섞임) | ROS 를 source 한 터미널 | `env -u PYTHONPATH python …` 로 실행 |

**출력 폴더가 이미 있을 때 (재병합)**

```bash
# 경로 끝이 정확히 지울 데이터셋 이름인지 눈으로 확인 후!
rm -rf ~/data_collection_ur5_gripper/lerobot_dataset_v21/foodbanana/<이름>
# 그다음 2단계 재실행
```

## 참고

- 병합본 데이터셋(`lerobot_dataset_v21/foodbanana/<이름>/`)은 학습용이며,
  서버로 옮겨 openpi 학습에 사용한다 (repo_id 로 참조).
- 데이터셋 폴더·중간 산출물은 용량이 크므로 git에 커밋하지 않는다 (`.gitignore` 처리).
  git에는 스크립트(`lerobot_merge_episodes_v21.py`)·매니페스트(`merge_*.txt`)·이 문서만 올린다.
- v3.0 은 만들지 않는다 (학습은 v2.1. `lerobot_stage2_build_dataset_v30.py` 는 옛 중간 파일 형식 전용).
