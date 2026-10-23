# PICO VR 컨트롤러로 sim 데이터 수집하기

PICO 4 Ultra 컨트롤러로 Isaac Sim 의 UR5 팔을 텔레옵해서 드론 파지 시연을 녹화하고, LeRobot v2.1 데이터셋으로 만드는 방법.
결정 근거와 시험 결과는 `docs/PLAN.md` 5-B, 녹화·변환 전체 구조는 `docs/data_recording.md`.

작성 2026-10-14 (브랜치 `isaacsim_v6.1.0`). 이 PC: sim PC (Ubuntu 24.04, 유선 `192.168.0.5`).

---

## 1. 한눈에 보기

```
PICO 헤드셋 앱 ──Wi-Fi──▶ PC 서비스 (RoboticsServiceProcess) ──▶ 텔레옵 노드 (xr_teleop_node.py)
                                                                    ├─ /joint_command, /gripper/command ──▶ sim (sim_ros2.py)
                                                                    └─ /teleop/request (A·B·X 버튼) ─────▶ 녹화 도구 (record_toggle.py) ──▶ bags/
```

- 오른손 컨트롤러의 위치·자세 → 그리퍼 끝 (TCP) 이 따라간다. 트리거 = 그리퍼
- 녹화·리셋·드론 모터 정지도 컨트롤러 버튼으로 한다 (키보드는 실패 에피소드를 나중에 버릴 때와 종료할 때만)
- 터미널 4 개: sim / 텔레옵 노드 / 손목 영상 / 녹화 도구

---

## 2. 처음 한 번만: 설치

이미 이 PC 에 되어 있다 (2026-10-12). 다른 PC 에 다시 깔 때만.

```bash
# PC 서비스 (Ubuntu 24.04 deb)
mkdir -p ~/xrobotoolkit && cd ~/xrobotoolkit
wget https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_24.04_amd64.deb
sha256sum XRoboToolkit_PC_Service_1.0.0_ubuntu_24.04_amd64.deb    # bce661f0be0b8a246ceecb2e5f1675a81c26b834648dc7fdf23f8c0bfe2a5d19
sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_24.04_amd64.deb
sudo ufw disable                                                  # 헤드셋 → PC TCP 63901 이 막히지 않게

# Python 바인딩 → ~/xrobotoolkit/venv (sudo 없이, conda·venv 를 끈 터미널에서)
cd ~/data_collection_ur5_gripper
bash teleop/build_xr_binding.sh
```

- 헤드셋 앱: `XRoboToolkit-PICO-1.1.1.apk` 를 헤드셋 브라우저로 받아 설치 (adb 필요 없음)
- 헤드셋과 PC 는 같은 공유기에 있어야 한다 (헤드셋 Wi-Fi, PC 유선)

---

## 3. VR 연결 (매번)

### 3.1 PC 서비스 켜기

```bash
pgrep -af RoboticsServiceProcess || env -u LD_LIBRARY_PATH bash /opt/apps/roboticsservice/runService.sh
```

- 이미 떠 있으면 다시 띄우지 않는다 (한 번에 하나만). 백그라운드로 뜬다
- 끄기: `pkill -f RoboticsServiceProcess`
- 로그 보기: `tail -f ~/.local/share/PICOBusinessSuitData/log/$(date +%Y%m%d).txt | grep -v '192.168.0.255'`

### 3.2 PC 주소 확인

```bash
ip -4 -o addr show | awk '{print $2, $4}'      # 유선 주소 (지금 192.168.0.5). 공유기가 바꿀 수 있으니 매번 확인
```

### 3.3 헤드셋 앱

1. 헤드셋에서 XRoboToolkit 앱 실행 → PC IP 입력 → 상태가 **WORKING**
2. 보낼 항목에서 **오른손과 왼손 컨트롤러 둘 다** 고른다 (Y·X 버튼이 왼손)
3. **Send**

### 3.4 연결 확인

```bash
ss -tn | grep 63901                           # ESTAB 이 보이면 헤드셋이 연결됨
```

---

## 4. 터미널 명령

모든 터미널은 레포 최상위에서. ROS 를 쓰는 터미널은 먼저 `source /opt/ros/jazzy/setup.bash`.

### 터미널 1: sim

**수집 기본 명령** (2026-10-16: 드론 기준 위치 목록 25 점 + 사인파 mid)

```bash
cd ~/data_collection_ur5_gripper
source /opt/ros/jazzy/setup.bash
~/isaacsim/python.sh isaacsim/scripts/sim_ros2.py --init-pose under_drone --drone-targets --mode hover --hover mid
```

뜰 때 확인할 줄:

```
[sim_ros2] seed 145809 (--seed 를 안 줘서 시작 시각으로 정함. 같은 조건을 다시 만들려면 --seed 145809)
[sim_ros2] 준비: seed 145809 (시작 시각), 드론 px4 (hover), ... 서비스 /sim/reset (드론 위치 목록 25 점 ±0.03 m), ...
```

- 드론 = PX4 (기본). 테이블에서 이륙해 호버할 때까지 약 30 s. 리셋 (B) 마다 약 24 s
- `--init-pose under_drone`: 그리퍼가 위를 보고 드론 기본 위치 아래 30 cm 에 있는 시작 자세. 리셋도 이 자세로 되돌린다
- `--drone-targets`: 리셋마다 드론 기준 위치를 목록 25 점 (로봇 앞 0.45 ~ 0.65 m, 좌우 ±0.2 m, 높이 0.70·0.80 m, `scene_drone.yaml` `drone_targets_base`) 에서
  섞어 한 바퀴씩 고르고 ±3 cm 흔든다. 점에 따라 시작할 때 드론이 손목 카메라에 일부만 보인다 (팔 시작 자세는 하나). 버린 에피소드 (`d`) 도 차례를 쓴다
- `--mode hover --hover mid`: 조종자 없이 드론이 기준 위치 둘레를 사인파로 움직인다 (리셋마다 주파수·위상 새로). **잡은 뒤에도 드론은 계속 움직이려 한다 (실물처럼)** → 잡으면 바로 X (모터 정지)
- `--seed` 는 주지 않는다: sim 을 띄운 시각으로 정해져 세션마다 점 순서·흔들림·사인파가 달라진다 (같은 seed 로 다시 띄우면 같은 순서가 반복됨). 값은 위 첫 줄과 `episode.json` 에 남는다
- 손목 카메라는 **t10 마운트** (2026-10-15: 렌즈가 그리퍼 중심선, 10° 기울임 → 잡는 순간 손가락이 화면 아래 가운데). 실물도 같은 마운트 (`isaacsim/ur5_rh_p12_description/cad/out/wrist_mount_t10.stl`)
- 옵션 철자가 틀리면 sim 이 시작하지 않고 에러로 끝난다 (줄여 쓰기도 안 됨, 2026-10-22)

**바꿔 쓰기** (위 명령의 `--drone-targets --mode hover --hover mid` 자리. 설정이 바뀌면 작업명도 바꾼다, 터미널 4)

| 드론 위치 | 드론 움직임 | 명령 뒤쪽 | 작업명 끝 |
|---|---|---|---|
| 목록 25 점 | 사인파 mid (기본) | `--drone-targets --mode hover --hover mid` | `_tgt_hmid` |
| 목록 25 점 | 사인파 small (빠르게 떨림 ±2 cm) | `--drone-targets --mode hover --hover small` | `_tgt_hsmall` |
| 목록 25 점 | 사인파 big (±5 cm) | `--drone-targets --mode hover --hover big` | `_tgt_hbig` |
| 목록 25 점 | 제자리 | `--drone-targets` | `_tgt_static` |
| 기본 위치 하나 (±3 cm) | 제자리 | (아무것도 안 붙임) | `_fix_static` |
| 목록 25 점 | 사람이 실물 조종기로 (두 사람) | `--drone-targets --drone-rc on` | `_tgt_rc` |

- 같이 쓰면 시작 전에 에러: `--mode hover` + `--drone-rc on` (드론을 움직이는 쪽이 둘), `--drone-targets` + `--drone-pos`
- 연습만 할 때: `--flight geometric` 을 붙이면 리셋이 약 2 s (기하 제어기. 수집은 PX4 로, 녹화하지 않는다)
- 같은 조건 다시 만들기 (재현): `--seed <값>` (터미널 첫 줄이나 `episode.json` `sim_reset.sim.drone.seed_base`)
- 수집 중 다른 터미널에서 드론 보기·옮기기 (PX4): `python3 isaacsim/scripts/drone_cmd.py status` / `goto x y z` (world, hover 면 기준 위치만 옮김) / `hold` / `kill`

### 터미널 2: 텔레옵 노드

```bash
cd ~/data_collection_ur5_gripper
source /opt/ros/jazzy/setup.bash
~/xrobotoolkit/venv/bin/python teleop/xr_teleop_node.py
```

뜰 때 확인할 줄:

```
[xr_teleop] 준비: 60 Hz, stamp sim, 배율 0.9, 회전 따라감. ... Y → 텔레옵 시작 / 다시 Y → 멈춤. 트리거 = 그리퍼. 녹화 도구 요청: A = record, B = reset, X = drone_kill
```

- `경고: 왼손 컨트롤러 데이터가 오지 않음` → 헤드셋 앱에서 왼손도 골라 Send, 노드를 다시 띄운다
- `5 s 안에 헤드셋 데이터가 오지 않음` 으로 끝나면 → PC 서비스·헤드셋 앱 (WORKING·Send) 확인

### 터미널 3: 손목 카메라 영상

```bash
source /opt/ros/jazzy/setup.bash
ros2 run rqt_image_view rqt_image_view /cam/wrist/color/image_raw
```

- third view 는 창 위쪽 목록에서 `/cam/third_view/color/image_raw` 로 바꾼다
- 영상이 조금 끊겨 보이는 것은 뷰어 문제이고 녹화에는 영향 없다
- **녹화를 시작한 뒤에는 새로 띄우지 않는다** (아래 "주의")

### 터미널 4: 녹화 도구 (텔레옵 노드를 띄운 뒤에)

```bash
cd ~/data_collection_ur5_gripper
source /opt/ros/jazzy/setup.bash
./6_record_bag.sh <작업명> --sim             # 예 (수집 기본): ./6_record_bag.sh drone_grasp_t10_f1_tgt_hmid --sim
```

- 첫 줄 `~/realsense_ws/install/setup.bash: No such file or directory` 는 무시해도 된다 (실물 PC 용 줄)
- 도움말에 `컨트롤러 버튼 (/teleop/request ...)` 줄이 나오면 준비 끝
- **작업명은 설정 (필터·배율·시작 자세·드론 움직임) 이 같은 데이터끼리 같게, 설정이 바뀌면 새 이름으로.** bag 이름은 `<날짜_시각>_<작업명>` 이 되고,
  나중에 병합할 때 작업명으로 골라 모은다 (10 장). 예: 필터를 켠 지금 설정 = `drone_grasp_f1`. **손목 카메라 마운트 기울기도 설정이다** (2026-10-15 부터 t10): 예 `drone_grasp_t10_f1`.
  **드론 위치·움직임도 설정이다** (2026-10-16): 작업명 = `drone_grasp_<손목 마운트>_<필터>_<드론 위치>_<드론 움직임>`, 끝 부분은 터미널 1 "바꿔 쓰기" 표.
  예 `drone_grasp_t10_f1_tgt_hmid` (수집 기본), `drone_grasp_t10_f1_fix_static` (예전 방식)
  (`xr_test` 로 녹화한 bag 에는 필터 끔 (2026-10-14) 과 켬 (2026-10-15) 이 섞여 있어 날짜로만 구분된다)

---

## 5. 버튼

| 버튼 | 손 | 하는 일 |
|------|----|---------|
| **Y** | 왼손 | 텔레옵 시작 / 멈춤. 누른 순간의 오른손 컨트롤러 자세가 기준이 된다 (녹화와 무관) |
| **A** | 오른손 | 녹화 시작 / 종료. **리셋 뒤에만 시작된다** (아니면 거부하고 "B 로 먼저 리셋") |
| **B** | 오른손 | 에피소드 리셋 (약 24 s, 녹화하지 않음). **녹화 중에 누르면 실패로 보고** 녹화를 끝내 `bags/_discarded/` 로 옮긴 뒤 리셋 |
| **X** | 왼손 | 드론 모터 정지 (언제든). 녹화 중이면 시각이 `episode.json` 에 기록된다 |
| 트리거 | 오른손 | 그리퍼. 0.6 넘게 당기면 닫기, 0.4 아래로 놓으면 열기 (그 사이는 그대로). 텔레옵 중에만 |

키보드 (터미널 4 에 포커스): `d` → `y` 방금 저장한 에피소드 버리기, `q` 종료.
`r` (리셋 → 바로 녹화), `k` (드론 모터 정지) 도 그대로 된다.

**손 → 팔 방향**: Y 를 누른 순간 컨트롤러가 가리키는 수평 방향 = 로봇 base +x (드론 쪽). 그 왼쪽 = +y, 위 = +z.
로봇 뒤에 서서 로봇과 같은 방향을 보고 컨트롤러 끝을 앞으로 향하게 든다. 손 이동 10 cm → 팔 9 cm (배율 0.9), 손목 회전은 1 : 1. 손의 작은 떨림은 필터가 줄인다 (8 장 `filter`).

---

## 6. 녹화 순서 (에피소드 하나)

```
B (리셋) → Y (텔레옵 시작) → 잠깐 자세 유지 → A (녹화 시작) → 드론 잡기 → X (모터 정지) → 잠깐 들고 있기 → A (녹화 끝) → 다음 에피소드는 B 부터
```

| 순서 | 조작 | 터미널에서 확인 |
|------|------|-----------------|
| 1 | **B** | 터미널 4: `[RESET] ...` 약 24 s → `[RESET] 완료: 시드 ..., 드론 [...]` |
| 2 | 시작 자세로 오른손 컨트롤러를 들고 **Y** | 터미널 2: `[xr_teleop] 시작: TCP (base) [...]` |
| 3 | 팔이 손을 따라오는지 잠깐 확인 | sim 화면 |
| 4 | **A** | 터미널 4: `[REC ●] 녹화 시작: .../bags/<이름>` |
| 5 | 드론 아래에서 위로 올라가 몸체 허리를 감싸고 트리거를 당긴다 | 손목 영상, 그리퍼가 중간에서 멈추면 잡힌 것 |
| 6 | **X** | 터미널 4: `[KILL] 드론 모터 정지` |
| 7 | 1 ~ 2 s 들고 있다가 **A** | 터미널 4: `[SAVED] .../bags/<이름> (..s)` |
| 8 | 다음 에피소드: **B** | 리셋하면 텔레옵은 스스로 멈춘다 → 다시 Y 부터 |

- **잡기에 실패하면** 녹화 중 바로 **B**. 그 에피소드는 `bags/_discarded/` 로 옮겨지고 리셋된다
- **이미 A 로 저장한 에피소드가 실패였으면**: 다음 B 를 누르기 전에 터미널 4 에서 `d` → `y`
- 끝낼 때: 터미널 4 `q` → 터미널 2 `Ctrl+C` → 터미널 1 sim 창 닫기 → (필요하면) `pkill -f RoboticsServiceProcess`

---

## 7. 주의

### 데이터 품질 (VLA 파인튜닝 관점)

- **A 를 누르기 전후로 가만히 있는 시간을 길게 두지 않는다.** 녹화 시작 직후 팔이 오래 멈춰 있으면, 학습한 정책이 "시작하면 가만히 있기" 를 배워
  실제로 돌렸을 때 로봇이 움직이지 않을 수 있다. Y 로 텔레옵을 켠 뒤 손이 안정되면 A 를 누르고 **바로 움직이기 시작한다**
- **끝도 같다.** X 뒤에 오래 들고 있지 말고 1 ~ 2 s 뒤 A 로 끝낸다 (뒤쪽 정지 구간이 길면 "잡은 뒤 멈춤" 이 과하게 학습된다)
- 앞뒤 정지가 길어진 에피소드는 병합 때 `--trim-idle start|end|both` 로 자를 수 있다. 검수 표의 "정지 앞/뒤 [s]" 로 확인
- **매 에피소드 같은 방식으로**: 같은 시작 자세, 비슷한 접근 경로 (아래에서 위로), 잡은 뒤 비슷한 시점에 X. 끄는 시점이 에피소드마다 다르면 그 뒤 영상·관절값이 달라진다
- 정책이 보는 것은 손목·third view 카메라뿐이다. 가능하면 sim 화면보다 손목 영상을 보고 조작한다
- sim 화면 (메인 뷰포트) 은 처음에 **로봇 바로 뒤에서 드론 쪽**을 본다 (2026-10-22). 화면 앞 = 로봇 앞 = 컨트롤러 앞 (Y 를 누를 때 향한 방향). 마우스로 시점을 바꿨으면 방향이 달라지니 주의
- 메인 오른쪽에 보조 화면 2 개 (2026-10-23, 메인만으로는 앞뒤·높이를 맞추기 어려움): **Side view** (로봇 오른쪽에서 봄, 화면 오른쪽 = 앞 → 그리퍼와 드론의 앞뒤 거리·높이 차),
  **Top view** (위에서 봄, 화면 위 = 앞, 오른쪽 = 로봇 오른쪽 → 드론이 그리퍼 바로 위인지). 녹화와 무관. 켜면 sim 이 조금 느려짐 (RTF 0.98 → 0.82, 드론 잡기 전). 끄기: sim 명령에 `--aux-views off`
- 드론 몸체는 땅콩 모양이라 **허리 (가운데) 가 가장 좁다.** 옆으로 치우쳐 잡으면 손가락 끝이 넓은 부분에 걸린다
- 실패·이상한 에피소드는 아까워하지 말고 버린다 (B 또는 `d`). 성공/실패 판정은 사람이 한다

### 조작

- **Y 를 누를 때 컨트롤러 끝을 거의 수평으로** (±60° 안). 끝이 위나 아래를 향하면 "앞" 을 정할 수 없어 시작을 거부한다
- 손을 너무 빨리 움직이면 팔이 못 따라와 알림이 뜬다 (`TCP 목표가 팔보다 .. cm 앞서 있음`). 천천히
- **손이 모자랄 때 (클러치)**: Y 로 텔레옵을 멈추고 손을 편한 곳으로 옮긴 뒤 다시 Y. 팔은 튀지 않는다. 녹화 중에도 된다 (그동안 팔이 멈춘 프레임이 생기므로 짧게)
- 손목이 테이블에 가까워지면 더 내려가지 않는다 (`손목이 테이블에 가까움`). 손을 올린다
- 드론을 잡고 있으면 sim 이 실제보다 약 30 % 느리다 (RTF 0.69). 팔이 느리게 따라오는 것은 정상
- 리셋 (약 24 s) 중에 누른 버튼은 버려진다. `[RESET] 완료` 를 보고 다음 버튼을 누른다

### 끊김·보호 정지

- **보호 정지** (sim 터미널에 `[protective_stop] 보호 정지 ...`): 팔이 멈추고 텔레옵도 스스로 멈춘다. 녹화는 계속되므로 **B** 로 버리고 리셋한다
  (보호 정지 에피소드는 병합에서 어차피 빠진다)
- 텔레옵이 스스로 멈추면 (터미널 2 `멈춤: ...`) 녹화는 계속된다. 이유를 보고 그 에피소드는 보통 B 로 버린다
- **추적 상실·연결 끊김은 자동으로 잡지 않는다.** 컨트롤러는 헤드셋 카메라가 볼 수 있는 곳에서 쓰고, 팔이 이상하게 움직이면 그 에피소드를 버린다
- 녹화 도구 없이 A·B·X 를 누르면 터미널 2 에 `받는 녹화 도구가 없음` 이 뜬다

### 시스템

- **녹화 중에는 새 ROS 노드를 띄우지 않는다** (`ros2 topic echo`, rqt 등). 새 노드가 뜰 때 전달이 잠깐 멈춰 메시지를 잃을 수 있다. 필요한 것은 녹화 전에 띄운다
- **sim 을 띄운 채 sim bag 을 `ros2 bag play` 하지 않는다.** bag 의 `/clock`·`/joint_command` 가 다시 발행돼 로봇이 옛 명령대로 움직인다
- bag 은 30 s 에 약 1.5 GB 이고 **자동으로 지우지 않는다.** 디스크 여유를 가끔 확인 (`df -h ~`). `_discarded/` 도 직접 지운다
- sim 은 `sudo` 로 띄우지 않는다

---

## 8. 설정 바꾸기 (`teleop/xr_teleop.yaml`)

텔레옵 노드의 동작은 [`teleop/xr_teleop.yaml`](../teleop/xr_teleop.yaml) 한 곳에서 정한다. 고친 뒤에는 **텔레옵 노드 (터미널 2) 를 `Ctrl+C` 로 끄고 다시 띄운다** (sim·녹화 도구는 그대로).
항목이 빠지면 노드가 시작할 때 에러로 멈춘다 (기본값으로 조용히 넘어가지 않는다). 노드가 값까지 검사하는 것은 `rate_hz` ≥ 50, `open_below` < `close_above`,
`filter` 의 cutoff > 0·beta ≥ 0, `stamp`·`hand`·버튼 이름과 버튼 중복뿐이다. **아래 표의 나머지 규칙 (굵게) 은 노드가 검사하지 않으므로 직접 지킨다.**

### 8.1 조작감 (자주 바꾸는 것)

| 항목 | 지금 값 | 뜻 | 바꿀 때 |
|------|---------|-----|---------|
| `scale` | `0.9` | 손 이동 : 팔 (TCP) 이동. 0.9 = 손 10 cm → 팔 9 cm. 위치에만 적용 | 작게 (예 0.7) = 정밀하지만 손을 더 많이 움직여야 함 (손이 모자라면 Y 클러치). 크게 = 빠르지만 흔들림도 커짐. **0 보다 큰 값** |
| `rotation` | `true` | 손목 회전도 그리퍼가 따라감 (1 : 1) | `false` = 위치만 따라가고 그리퍼 자세는 시작할 때 그대로 (회전 실수가 줄지만 기울여 잡을 수 없음) |
| `max_joint_speed` | `1.5` [rad/s] | 관절 목표가 움직일 수 있는 최대 속도. 손을 빨리 움직여도 팔은 이 속도까지만 | 낮추면 팔이 더 천천히·부드럽게 따라옴 (손보다 늦어져 `TCP 목표가 팔보다 .. 앞서 있음` 이 자주 뜰 수 있음). **`max_joint_speed` ≤ π (3.14)** (sim 관절 최대 속도 180 °/s. 보호 정지 기준 200 °/s) |
| `max_start_elevation_deg` | `60` [°] | Y 를 누를 때 컨트롤러 끝이 수평에서 이보다 더 들리거나 숙여져 있으면 시작 거부 | 시작 거부가 잦으면 조금 키움 (70 등). 90 에 가까우면 "앞" 방향이 불안정해진다 |
| `filter.enabled` | `true` | 컨트롤러 자세에 One Euro 필터. 손이 멈춰 있으면 떨림을 잡고, 빠를수록 약하게 걸려 지연을 줄인다 | `false` = 필터 없음 (손 그대로) |
| `filter.pos_min_cutoff_hz` | `1.5` [Hz] | 손이 멈췄을 때의 위치 필터 세기 (작을수록 강함) | 떨림이 남으면 낮춤 (1.0). 느린 접근이 굼뜨면 올림 (2~3). 0 보다 커야 한다 (노드가 검사) |
| `filter.pos_beta` | `40` [Hz/(m/s)] | 손이 빠를수록 필터를 얼마나 약하게 할지 | 빠르게 움직일 때 늦게 느껴지면 올림 |
| `filter.rot_min_cutoff_hz`, `rot_beta` | `1.5`, `10` | 손목 회전에 거는 같은 값 | 위치와 같은 방식 |
| `gripper.close_above` | `0.6` | 트리거를 이 값 넘게 당기면 닫기 | 살짝만 당겨도 닫히게 하려면 낮춤 |
| `gripper.open_below` | `0.4` | 트리거를 이 값 아래로 놓으면 열기 | `open_below < close_above` 여야 한다 (노드가 검사. 그 사이는 그대로 = 떨림 방지) |

### 8.2 버튼

| 항목 | 지금 값 | 뜻 |
|------|---------|-----|
| `teleop_button` | `Y` | 텔레옵 시작 / 멈춤 버튼 |
| `buttons.record` | `A` | 녹화 시작 / 종료 |
| `buttons.reset` | `B` | 리셋 (녹화 중이면 실패로 버리고 리셋) |
| `buttons.drone_kill` | `X` | 드론 모터 정지 |

- 쓸 수 있는 버튼은 `A`·`B` (오른손), `X`·`Y` (왼손). 같은 버튼을 두 번 쓸 수 없다 (노드가 검사. 예: `teleop_button: A` 로 바꾸면 `record` 를 다른 버튼으로)
- 요청 하나를 버튼에서 빼려면 그 줄을 지운다. 버튼 요청을 다 끄려면 `buttons: {}` (그러면 키보드 `r`·`k` 로)
- 버튼을 바꾸면 이 문서의 5·6 장과 녹화 도구 도움말의 안내 문구 (A·B·X·Y) 는 자동으로 바뀌지 않는다

### 8.3 안전 (바꾸기 전에 이유를 확인)

| 항목 | 지금 값 | 뜻 | 주의 |
|------|---------|-----|------|
| `min_wrist_height` | `0.09` [m] | 손목 관절 높이 (테이블 상판 기준) 가 이 아래로 더 내려가는 목표는 보내지 않음 | sim 측정: 0.058 m 에서 손목이 테이블에 닿아 보호 정지. 0.06 보다 낮추지 않는다. 시작 자세·테이블을 바꾸면 다시 잰다 |
| `max_lead` | `0.08` [rad] | 관절 목표가 실제 관절보다 앞설 수 있는 최대 (4.6°) | **0.087 (5°) 아래로.** 넘으면 sim 보호 정지 (위치 오차 5°) 에 걸린다 |
| `desync` | `0.2` [rad] | 실제 관절이 목표와 이만큼 다르면 텔레옵을 멈춤 (리셋 순간이동·보호 정지 감지) | **`max_lead` 보다 커야 한다.** 작으면 정상 조작 중에도 멈춘다 |
| `limit_margin` | `0.05` [rad] | 관절 한계 안쪽 여유 | 줄이지 않는다 (sim 은 한계 밖 명령을 받으면 에러로 끝난다) |
| `warn_pos_err` | `0.05` [m] | 팔이 손보다 이만큼 넘게 뒤처지면 알림 | 알림만. 너무 자주 뜨면 키움 |
| `warn_manip` | `0.01` | 특이점 근처 알림 기준 | 알림만 |

### 8.4 바꾸지 않는 것

| 항목 | 이유 |
|------|------|
| `rate_hz` (60) | 명령 주기. 50 아래면 노드가 에러 (데이터셋 25 Hz 의 두 배 이상이어야 계단이 생기지 않는다). 바꾸면 `max_joint_speed` 기준도 같이 본다 |
| `stamp` (`sim`) | sim 은 `sim`, 실물은 `wall` (명령 줄 `--stamp wall` 로도 바꿀 수 있다) |
| `topics.*` | sim·녹화 도구·stage1 과 이름이 맞아야 한다 (`request` 는 `record_toggle.py` `TELEOP_REQUEST_TOPIC` 과 같아야 함) |
| `hand` | 지금 `right` 만 된다 |
| `gripper.raw_open`·`raw_close` | 그리퍼 명령 값 (0 / 1150). 데이터셋 스키마와 같아야 한다 |
| `damping` | IK 계산 값. 바꾸면 시험을 다시 돌린다 |
| `filter.d_cutoff_hz` (1.0) | 필터가 손 속도를 추정할 때 쓰는 값. 보통 바꾸지 않는다 |
| `xr_connect_timeout`, `xr_stale_warn`, `state_stale`, `status_sec` | 연결 확인·알림 주기. 필요할 때만 |

### 8.5 바꾼 뒤

- **수집 도중에는 조작감 값 (`scale`·`rotation`·`max_joint_speed`·`filter`) 을 바꾸지 않는다.** 지금 필터는 켜져 있다 (`filter.enabled: true`, 2026-10-15 실제 손으로 써 보고 결정. 노드 시작 줄에 `필터 One Euro ...`). 한 데이터셋 안에서 조작 방식이 섞이면 동작 분포가 달라진다. 바꿀 거면 새 작업명 (`./6_record_bag.sh <새 작업명> --sim`) 으로 모은다
- 안전 항목이나 IK 항목을 바꿨으면 헤드셋 없이 시험한다 (13/13 PASS 여야 함, 11 장 끝의 명령)
- 바꾼 값과 이유를 `docs/PLAN.md` 5-B 에 적어 두면 나중에 데이터를 볼 때 헷갈리지 않는다 (bag 의 `episode.json` 에는 텔레옵 설정이 들어가지 않는다)

---

## 9. 데이터 위치

모두 `~/data_collection_ur5_gripper/` 아래 (git 에 올라가지 않음).

| 무엇 | 경로 | 내용 |
|------|------|------|
| 녹화한 에피소드 | `bags/<YYYYMMDD_HHMMSS>_<작업명>/` | ros2 bag (`*.mcap`, `metadata.yaml`) + `episode.json` (리셋 시드·드론 위치·kill 시각·sim 설정·git commit) |
| 버린 에피소드 | `bags/_discarded/<이름>/` | B (녹화 중) 나 `d` 로 버린 것. 되살리려면 폴더를 `bags/` 로 옮긴다 |
| stage1 중간 파일 | `bag_lerobot_intermediate/<bag 이름>/` | `states.npy`, `actions.npy`, `base_imu.npy`, `timestamps.npy`, `wrist/*.png`, `third_view/*.png`, `meta.json` |
| LeRobot v2.1 데이터셋 | `lerobot_dataset_v21/<repo-id>/` | 병합 결과 (parquet + mp4 + `meta/`). 이름은 병합할 때 `--repo-id` 로 정한다 (bag 은 그대로). 예: `local/drone_grasp_tgt_hmid` |
| PX4 비행 로그 | `isaacsim/reports/px4_<시각>/` | 리셋마다의 PX4 로그 (`episode.json` 의 `px4_log`) |
| 텔레옵 시험 리포트 | `teleop/reports/` | `check_xr_teleop.py` 결과 |
| PC 서비스 로그 | `~/.local/share/PICOBusinessSuitData/log/<날짜>.txt` | 헤드셋 연결 기록 |
| 설정 | `teleop/xr_teleop.yaml` | 배율·버튼·그리퍼 기준·안전 기준. 바꾸는 법은 8 장 |
| 시작 자세 | `isaacsim/config/scene_drone.yaml` `init_poses.under_drone` | 관절각 6 개 |

---

## 10. 녹화 뒤: 변환 → 병합 → 검수

```bash
cd ~/data_collection_ur5_gripper

# 1) stage1: bag 마다 (시스템 python3 + ROS)
source /opt/ros/jazzy/setup.bash
python3 lerobot_stage1_extract_bag.py bags/<bag 이름> --arm-action command --base-imu const
#   여러 개: for b in bags/*_<작업명>; do python3 lerobot_stage1_extract_bag.py "$b" --arm-action command --base-imu const; done

# 2) 병합 (conda lerobot_v2). 매니페스트 = bag_lerobot_intermediate/ 안의 폴더 이름을 한 줄에 하나 (경로 없이)
source ~/miniconda3/etc/profile.d/conda.sh && conda activate lerobot_v2
ls bag_lerobot_intermediate/ | grep <작업명> > merge_<이름>.txt    # 설정이 같은 작업명끼리만 (필터 켬·끔 등을 섞지 않는다)
env -u PYTHONPATH python lerobot_merge_episodes_v21.py --manifest merge_<이름>.txt --repo-id local/<이름>
#   중간 파일 전부: --all.  앞뒤 정지 자르기: --trim-idle both

# 3) 검수 → 5/5 PASS 여야 한다
env -u PYTHONPATH python inspect_dataset_v21.py lerobot_dataset_v21/local/<이름> --load
conda deactivate
```

검수 표에서 볼 것:
- **파지 신호 "있음"**: 닫기 명령 중 그리퍼 state 가 중간 (sim 드론 약 0.25) 에서 멈춤
- **정지 앞/뒤 [s]**: 길면 위 "주의" 를 다시 보고, 병합 때 `--trim-idle`
- **`/joint_command` 간격**: 중앙값 16.7 ms, 40 ms 넘은 횟수 0 이 정상 (늘면 끊김)
- 보호 정지 에피소드는 병합이 자동으로 빼고 출력에 알린다

---

## 11. 문제가 생기면

| 증상 | 확인 |
|------|------|
| 노드가 `헤드셋 데이터가 오지 않음` 으로 끝남 | PC 서비스 (`pgrep -af RoboticsServiceProcess`), 헤드셋 앱 WORKING·Send, `ss -tn \| grep 63901` 이 ESTAB, 같은 공유기 |
| Y·X 를 눌러도 반응 없음 | 헤드셋 앱에서 왼손 컨트롤러를 골랐는지 (노드 시작 때 경고) |
| Y 를 눌렀는데 `시작 거부` | 이유를 본다: 컨트롤러 끝이 수직 (수평으로 들기) / `/joint_states`·`/clock` 이 안 옴 (sim 확인) / 보호 정지 중 (B) |
| A 를 눌렀는데 `녹화 시작 거부` | 지난 녹화 뒤 리셋하지 않음 → B 먼저 |
| A·B·X 가 녹화 도구에 안 감 (`받는 녹화 도구가 없음`) | 터미널 4 녹화 도구가 떠 있는지 |
| 녹화 도구가 `토픽 문제` 로 녹화를 시작하지 않음 | sim 이 떠 있는지 (상태 토픽에 발행자가 있어야 한다) |
| 리셋이 실패 (`[ERROR] /sim/reset 실패`) | sim 터미널 로그 (PX4 메시지). 다시 B |
| 팔이 갑자기 멈추고 sim 에 `[protective_stop]` | 이유 줄 (force·error·speed 와 링크 이름) 을 보고 B 로 리셋 |
| stage1 이 `메시지 나이` 에러 | 녹화 중 새 노드를 띄웠는지. `--skip-start 0.5` 로 앞부분을 빼고 살릴 수 있다 |

설정을 바꾼 뒤에는 헤드셋 없이 시험할 수 있다 (13/13 PASS):

```bash
~/isaacsim/python.sh isaacsim/scripts/sim_ros2.py --headless --flight geometric --init-pose under_drone   # 터미널 1
python3 teleop/check_xr_teleop.py                                                                         # 터미널 2
```
