# sim 실행 속도 (RTF) 와 최적화

작성: 2026-10-02 · 3단계 3-1 (`docs/PLAN.md` 3단계), 브랜치 `isaacsim_v6.1.0`

텔레옵으로 데이터를 모으려면 sim 이 **실제 시간 속도**로 돌아야 한다. 이 문서는 카메라 2대를 ROS 2 로 발행하는 상태에서
sim 이 얼마나 빨리 도는지 재고(RTF), 어디에 시간이 드는지 나누고, 무엇을 줄였는지 정리한다.

---

## 1. 한눈에 보기

| 항목 | 결론 |
|------|------|
| RTF (텔레옵 구성: GUI 메인 뷰포트, PX4 드론, 카메라 2대 30 Hz) | **0.89 → 0.97** (헤드리스 0.91 → 0.99). 프로펠러 회전 켜면 GUI 0.94 |
| 앱 루프(렌더) 주기 | **30 Hz** (= 카메라 주기). 물리는 120 Hz 그대로 (update 한 번에 물리 4 스텝) |
| 카메라 주기 | 카메라 prim `omni:sensor:tickRate` **30 Hz**. sim 기준 정확히 30.0 Hz, 끊김 없음 |
| 카메라 안티에일리어싱 | **DLSS Performance, 설정 파일에 명시** (DLAA 대비 평균 차이 1/255 이하, 7장) |
| 카메라 받는 쪽 QoS | **RELIABLE** (best effort 면 이미지가 통째로 버려짐) |
| 텔레옵 화면 | Isaac Sim GUI 는 **메인 뷰포트 하나**, 손목·third view 는 **ROS 토픽으로** (`rqt_image_view` 등) |
| 최적화 | **우리 Python 콜백만** (update 당 6.3 → 2.4 ms). 렌더·PhysX 설정은 바꾸지 않음 (사용자 결정) |
| 남은 시간 | PhysX 약 18 ms + 렌더·앱 약 13 ms (update 한 번 = sim 33.3 ms 안에 들어가야 RTF 1) |

---

## 2. RTF 란

**RTF (Real-Time Factor) = sim 에서 흐른 시간 ÷ 실제로 걸린 시간(wall clock)**

| RTF | 의미 |
|-----|------|
| 1.0 | 실제 시간과 같은 속도 |
| 0.5 | 실제의 절반 속도 (sim 10 s 계산에 실제 20 s, 슬로모션) |
| 2.0 | 실제보다 2 배 빠름 |

- 텔레옵 명령은 실제 시간으로 들어온다. **RTF < 1** 이면 로봇이 슬로모션처럼 늦게 반응해 조작감이 실물과 달라진다
- **RTF ≥ 1** 이면 남는 시간만큼 기다려(`--realtime`) 정확히 실제 시간으로 맞출 수 있다
- 데이터의 시각(header.stamp)은 모두 sim time 이라, RTF 가 1 보다 조금 낮아도 **데이터셋의 시각 정합성에는 문제가 없다**.
  영향은 사람의 조작감뿐이다 (RTF 0.97 = 실제보다 3% 느림)

---

## 3. 재는 방법

### 3.1 도구

| 파일 | 역할 |
|------|------|
| `isaacsim/scripts/measure_rtf.py` | drone_scene 씬(테이블·로봇·카메라 2대·드론)을 만들고 **기다리지 않고 최대 속도로** 돌려 RTF 를 잰다. update 시간을 Python 콜백 / PhysX / 나머지로 나눈다 |
| `isaacsim/scripts/topic_rate.py` | **별도 프로세스**(시스템 python3)에서 카메라 토픽을 받아 도착 시각(wall)과 header.stamp(sim)를 기록. 받는 쪽이 실제로 받은 주기를 잰다 |
| `isaacsim/scripts/ros2_iface.py` | ROS 2 bridge 켜기 확인, 카메라 tickRate, OmniGraph Camera Helper 발행 (3단계 공용 모듈) |
| `isaacsim/config/ros2_iface.yaml` | 카메라 토픽·frame_id·tickRate |

### 3.2 시간 나누기

update 한 번 = 렌더 한 번 + 물리 N 스텝 (loop 30 Hz, 물리 120 Hz 이면 N = 4).

| 구간 | 재는 법 |
|------|---------|
| 우리 Python 콜백 | 물리 직전(PRE_STEP) 콜백 두 개 사이. 하나는 drive·드론 콜백보다 **먼저**, 하나는 **나중에** 등록 (같은 order 는 등록 순서대로 불림) |
| PhysX | 두 번째 PRE_STEP 콜백 → POST_STEP 콜백 |
| 나머지 (렌더·앱·우리 루프) | update 전체 − 위 둘 |

`--profile` 은 측정 구간을 cProfile 로 돌려 함수별 시간을 `profile.txt` 에 남긴다 (프로파일러 때문에 RTF 숫자는 의미 없음).

### 3.3 실행

```bash
source /opt/ros/jazzy/setup.bash        # python.sh 안 rclpy 가 시스템 Jazzy 를 쓰게 (없으면 에러로 중단)
~/isaacsim/python.sh isaacsim/scripts/measure_rtf.py --headless --loop-hz 30      # 헤드리스, 기본 드론(PX4), tickRate 30
~/isaacsim/python.sh isaacsim/scripts/measure_rtf.py --loop-hz 30                 # GUI (메인 뷰포트)
#   --flight geometric|px4, --loop-hz 120|60|30, --tick-rate 0|30, --cameras off,
#   --camera-windows (GUI 에 카메라 뷰포트 창 2개), --prop-spin on, --profile, --seconds 20
```

리포트: `isaacsim/reports/measure_rtf_<시각>/report.txt` (+ `topics.json`, `profile.txt`).

측정 PC: Intel i5-14600KF (P코어 5.3 GHz), RTX 4070 Ti SUPER 16 GB, Ubuntu 24.04, ROS 2 Jazzy, Isaac Sim 6.1.0.

---

## 4. 측정 결과

### 4.1 루프 주기와 카메라 tickRate (헤드리스, 기하 제어기 드론)

| loop(렌더) | 카메라 tickRate | RTF | update 한 번 | 물리 구간 | 나머지(렌더·앱) | 카메라 (sim 기준) |
|---|---|---|---|---|---|---|
| 120 Hz | 끔 | 0.71 | 11.7 ms | – | – | – |
| 120 Hz | 0 (매 프레임) | 0.40 | 20.7 ms | – | – | 106 Hz |
| 120 Hz | 30 | 0.52 | 15.9 ms | 6.4 ms (1 스텝) | 9.5 ms | 30.0 Hz |
| 60 Hz | 30 | 0.81 | 20.7 ms | – | – | – |
| **30 Hz** | **30** | **0.94** | 35.6 ms | 23.5 ms (4 스텝) | 12.0 ms | **30.0 Hz** |
| 30 Hz | 끔 | 1.25 | 26.8 ms | 20.1 ms (4 스텝) | 6.7 ms | – |

- **loop 30 Hz (= 카메라 주기) 가 맞다.** 렌더·앱 처리 비용은 update 마다 들기 때문에, update 를 카메라와 같은 30 Hz 로 묶으면 그 비용이 1/4 로 준다.
  물리는 120 Hz 그대로 (drive·드론·PX4 는 물리 스텝마다 도는 콜백이라 update 주기와 무관)
- tickRate 0 은 매 update 마다 카메라 2대를 렌더해 가장 느리다. Isaac Sim 6.0 부터 카메라 주기는 Camera Helper 의 `frameSkipCount`(deprecated, 렌더한 뒤 버림)가 아니라
  카메라 prim 의 `omni:sensor:tickRate` 로 정한다 (multi-tick rendering, 렌더 자체를 그 주기로만)

### 4.2 GUI (PX4 드론, loop 30 Hz, 최적화 전)

| 모드 | RTF | update 한 번 | 비고 |
|---|---|---|---|
| 헤드리스 | 0.91 | 36.6 ms | |
| GUI, 메인 뷰포트만 | 0.89 | 37.3 ms | GUI 비용 +0.7 ms (작음) |
| GUI + 카메라 뷰포트 창 2개 | 0.75 | 44.5 ms | +7 ms. ROS 로 발행하려고 렌더한 같은 카메라를 창마다 또 렌더 |

→ **텔레옵 GUI 는 메인 뷰포트 하나만 연다.** 손목·third view 는 ROS 토픽으로 본다 (추가 렌더 없음, 녹화되는 이미지 그대로, 실물 수집 때와 같은 화면).

### 4.3 시간 내역 (헤드리스, loop 30 Hz, 카메라 30 Hz, 최적화 전)

| 항목 | 기하 제어기 | PX4 |
|---|---|---|
| 우리 Python 콜백 (drive, 드론 계산, PX4 통신) | 5.5 ms | 6.3 ms |
| PhysX (물리 4 스텝) | 18.1 ms | 16.7 ms |
| 렌더(카메라 2대)·앱 | 12.0 ms | 13.5 ms |
| **합계 → RTF** | 35.5 ms → 0.94 | 36.6 ms → 0.91 |

---

## 5. 카메라 끊김: 받는 쪽 QoS

처음 측정에서 카메라가 sim 기준 25 Hz 로 떨어지고 0.6~1.2 s 씩 끊겼다. 원인은 sim 이 아니라 **받는 쪽 QoS** 였다.

| 받는 쪽 Reliability | 결과 (sim 20 s, tickRate 30) |
|---|---|
| BEST_EFFORT | 약 25 Hz, stamp 간격 최대 0.6~1.2 s |
| **RELIABLE** | **600/600 장, 30.0 Hz, 간격 최대 33 ms, 100 ms 넘는 간격 0 번** |

- 640x480 컬러 이미지 한 장은 약 0.9 MB 이고 UDP 로 수백 조각으로 나뉜다. BEST_EFFORT 는 조각 하나만 빠져도 이미지 한 장을 통째로 버린다
- 보내는 쪽과 받는 쪽 중 하나라도 BEST_EFFORT 면 BEST_EFFORT 로 동작한다
- **RELIABLE 이어도 시각 맞춤은 정확하다.** stage1 은 도착 시각이 아니라 `header.stamp` 로 프레임을 맞춘다 (`lerobot_stage1_extract_bag.py`).
  재전송으로 도착이 늦어져도 stamp 는 바뀌지 않는다
- 오히려 BEST_EFFORT 가 위험하다: 이미지가 빠지면 stage1 `latest_at` 이 옛날 이미지를 조용히 골라 "옛 이미지 + 새 joint" 프레임이 생긴다
- 후속 (PLAN 4단계): 녹화는 카메라를 RELIABLE 로 받기, stage1 에 이미지 나이 검사 추가. 실물 카메라가 30 Hz 설정에서 20~25 Hz 로 떨어지는 것도
  같은 원인일 수 있음 (실물 PC 에서 `ros2 topic info -v /d435i/d435i/color/image_raw` 로 확인)

---

## 6. 최적화 (우리 Python 콜백만)

cProfile 로 보니 시간 대부분이 우리 계산이 아니라 **API 를 부를 때마다 생기는 오버헤드**였다.

| 변경 | 파일 | 내용 | 물리 결과 영향 |
|------|------|------|---------------|
| 드론 tensor view 직접 사용 | `drone_flight.py` `FastRigid` | 드론 콜백이 물리 스텝마다 `RigidPrim.get_world_poses` / `get_velocities` / `apply_forces_and_torques_at_pos` 를 불렀는데, 부를 때마다 warp 배열을 새로 만들고 복사했다 (스텝당 18 번). 그 아래 physics tensor view 를 직접 읽고(`get_transforms`, `get_velocities`), 힘 버퍼는 미리 만들어 재사용 | 없음 (같은 view 데이터, float32) |
| PX4 안 쓰는 스트림 끄기 | `px4_sitl.yaml` `commander.disable_streams`, `drone_cmd.py` | PX4 offboard 링크(onboard 모드)는 sim 1 s 에 약 320 개 메시지를 보내고 pymavlink 가 전부 파싱한다. 명령 쪽이 쓰지 않는 HIGHRES_IMU(50 Hz), ATTITUDE_QUATERNION(50 Hz), ODOMETRY(30 Hz)를 `MAV_CMD_SET_MESSAGE_INTERVAL −1` 로 끈다. 실물 companion computer 가 스트림을 고르는 것과 같은 방식. PX4 가 거부하면 에러로 중단 | 없음 (명령 쪽은 HEARTBEAT·LOCAL_POSITION_NED·ATTITUDE·EXTENDED_SYS_STATE·STATUSTEXT·COMMAND_ACK 만 씀) |
| 지자기 표 보간 캐시 | `px4_sensors.py` `_mag_field_enu` | 지자기 센서가 매 갱신마다 세계 자기장 표를 보간했다. 위경도를 1e-5° (약 1.1 m) 격자로 반올림해 결과를 재사용 | 무시할 수준 (1 m 안 자기장 차이는 센서 잡음보다 훨씬 작음) |

PX4 offboard 링크 메시지 (최적화 전, sim 10 s): ATTITUDE 60 Hz, HIGHRES_IMU 50 Hz, ATTITUDE_QUATERNION 50 Hz, LOCAL_POSITION_NED 30 Hz, ODOMETRY 30 Hz,
그 밖에 10 Hz 이하 20 여 종.

### 결과 (PX4 드론, loop 30 Hz, 카메라 2대 30 Hz)

| | 최적화 전 | 최적화 후 |
|---|---|---|
| 우리 Python 콜백 (update 당) | 6.3 ms | **2.4 ms** |
| PhysX (update 당) | 16.7 ms | 17.9~18.5 ms (변경 없음, 측정 편차) |
| 렌더·앱 (update 당) | 13.5 ms | 12.9~14.0 ms |
| **RTF 헤드리스** | 0.91 | **0.99** |
| **RTF GUI (메인 뷰포트)** | 0.89 | **0.97** |
| 카메라 (sim 기준) | 30.0 Hz | 30.0 Hz, 끊김 0 |

### 회귀 시험 (최적화 후)

| 시험 | 결과 |
|------|------|
| `check_flight.py` (기하 제어기) | 4/4 PASS |
| `check_px4.py` (PX4) | 5/5 PASS |
| `drone_scene.py --check` | 3/3 PASS |
| `grasp_demo.py --case all` (PX4) | 6/6 PASS |
| `grasp_demo.py --case all --flight geometric` | 6/6 PASS |

**같은 결과인가**: 기하 제어기(잡음 없음) success 케이스를 최적화 전 코드(커밋 `48eef5f`)와 후 코드로 두 번씩 돌렸다.

| 실행 | 잡는 동안 밀린 거리 | 모터 정지 뒤 내려앉음 | 잡은 뒤 미끄러짐 |
|---|---|---|---|
| 전 1 | 1.48 mm | 6.19 mm | 6.19 mm |
| 전 2 | 1.49 mm | 5.82 mm | 5.91 mm |
| 후 1 | 1.53 mm | 6.30 mm | 6.37 mm |
| 후 2 | 1.56 mm | 5.60 mm | 5.65 mm |

최적화 전 코드도 실행마다 값이 다르다 (PhysX 접촉 계산이 실행마다 완전히 같지 않음). 후 코드는 그 편차 안이다 → 최적화가 파지 결과를 바꾸지 않았다.
(`--case all` 은 한 씬에서 케이스를 이어서 돌려 단독 실행과 숫자가 다르다.)

### 프로펠러 회전 (`--prop-spin on`, GUI 기본으로 사용)

| 조건 (PX4, loop 30 Hz, 카메라 2대 30 Hz) | 회전 끔 | 회전 켬 |
|---|---|---|
| 헤드리스 | 0.99 | **0.97** |
| GUI (메인 뷰포트) | 0.97 | **0.94** |

- 회전은 보여 주기용으로 물리 스텝마다 프로펠러 관절 속도 4 개를 덮어쓴다 (`drone_flight` `art.set_dof_velocities`).
  실험용 Articulation API 호출 오버헤드로 Python 이 update 당 약 +1 ms (PhysX 는 그대로)
- **지금은 최적화하지 않는다 (2026-10-02 사용자 결정).** 텔레옵이 느리게 느껴지면 드론 위치·힘처럼 articulation tensor view 를 직접 쓰도록 바꾼다
  (예상 약 1 ms 절약, 표시용이라 물리 결과 영향 없음)

---

## 7. 카메라 안티에일리어싱 (DLSS) 비교

Isaac Sim 기본값은 **DLSS Performance** 다 (`SimulationApp` `anti_aliasing: 3` + `apps/isaacsim.exp.base.kit` `rtx.post.dlss.execMode = 0`).
640x480 카메라를 가로세로 절반으로 렌더해 AI 로 키운다. 우리 코드가 켠 것이 아니다. 실물 RealSense 는 원래 해상도라, VLA 학습 이미지에
차이가 생기는지 확인했다 (다 모은 뒤 바꾸면 이미지 분포가 달라지므로 6단계 전에 결정).

- **RTX 실시간 렌더러는 DLSS(3)·DLAA(4)만 지원한다.** 0 끔·1 TAA·2 FXAA 를 주면 렌더러가 3 으로 되돌린다 (`omni.rtx.settings.core` UI 도 "Invalid AA Mode")
- DLSS 모드는 **새 stage 를 만들면 기본값(0)으로 돌아간다** → stage 를 만든 뒤 설정해야 한다. `measure_rtf.py` 는 요청한 값이 실제로 적용됐는지 확인하고 아니면 에러

조건: 헤드리스, 기하 제어기 드론(위치가 일정), 손목 카메라가 드론을 보는 자세(`--init-pose -0.1888 -0.7854 0.9599 1.3963 -1.5708 0.1888`),
측정 중 팔을 움직임(shoulder_pan ±0.4 rad 0.3 Hz, wrist_3 ±0.6 rad 0.4 Hz), 받는 쪽이 발행된 이미지를 그대로 저장 (정지 1 장 + 움직임 2 장).

| 설정 | 렌더 해상도 (640x480 출력) | RTF (기하 제어기) | DLAA 대비 평균 차이 (0~255) | 경계 선명도 (DLAA = 100%) |
|------|---------------------------|-------------------|-----------------------------|---------------------------|
| DLSS Performance (기본) | 약 50% | **1.01** | 0.48~1.21 | 98~101% |
| DLSS Balanced | 약 58% | 0.99 | 0.42~1.06 | 99~101% |
| DLSS Quality | 약 67% | 0.96 | 0.35~0.90 | 99~101% |
| DLAA (원래 해상도) | 100% | **0.84** | 0 | 100% |

- 차이는 **평균 1/255 이하** (0.5% 미만). 확대하면 Performance 의 경계가 조금 부드럽고 DLAA 가 가장 또렷하지만 눈에 띄는 뭉개짐·잔상은 없다
  (팔이 텔레옵 속도로 움직일 때 포함). Quality 가 DLAA 에 가장 가깝다
- DLAA 는 렌더 비용이 커서 RTF 가 0.84 로 떨어진다 (PX4·GUI 면 더 낮음)
- 이 차이는 sim 과 실물의 차이(조명·재질·센서 잡음·자동 노출)보다 훨씬 작다
- **결정 (2026-10-02): DLSS Performance 를 그대로 쓰되 Isaac Sim 기본값에 맡기지 않고 `isaacsim/config/ros2_iface.yaml` `render` 에 명시.**
  `ros2_iface.app_config` (SimulationApp `anti_aliasing`), `set_dlss_mode` (stage 를 만든 뒤), `check_render_settings` (재생 후 다시 읽어 다르면 에러,
  에피소드 메타데이터용 값을 돌려줌). Isaac Sim 업데이트로 기본값이 바뀌어도 데이터가 조용히 달라지지 않는다
- 비교 이미지: `isaacsim/reports/aa_compare_20261002/aa_{wrist,third_view}_{static,moving_a,moving_b}.png`
  (윗줄 = 설정별 원본, 아랫줄 = 빨간 상자 확대 5배, git 제외)

```bash
~/isaacsim/python.sh isaacsim/scripts/measure_rtf.py --headless --loop-hz 30 --flight geometric \
    --init-pose -0.1888 -0.7854 0.9599 1.3963 -1.5708 0.1888 --arm-motion --save-images [--dlss-mode 0|1|2] [--anti-aliasing 4]
```

---

## 8. 손대지 않은 것 (나중에 필요하면)

RTF 1 을 넘기려면 update 당 약 1~2 ms 가 더 필요하다. 남은 큰 항목은 사용자 결정으로 지금은 바꾸지 않는다.
실제 텔레옵에서 느리게 느껴지면 다시 본다.

| 후보 | 기대 효과 | 대가 |
|------|-----------|------|
| 렌더 설정 (RTX 반사·그림자 품질 등) | 렌더·앱 약 13 ms 중 일부 | **카메라 이미지 모양이 바뀜** (VLA 학습 이미지). 바꾸기 전 비교 이미지 필요 |
| PhysX 설정 (solver 반복 32/4, 로봇 링크 convex 분해 수, self-collision) | PhysX 약 18 ms 중 일부, 가장 큼 | **파지 결과가 달라짐** → grasp_demo, 파지력 시험 등 회귀 시험 다시 |
| 물리 GPU / CPU 장치 비교 | 미지수 | 결과가 미세하게 달라질 수 있음 |

---

## 9. 경고 메시지 정리 (GUI 실행 시)

| 경고 | 판단 |
|------|------|
| `CPU performance profile is set to powersave` | **잘못된 경보.** `intel_pstate` 드라이버의 governor 이름이 `powersave` 일 뿐이다(부하에 따라 클럭을 바꾼다는 뜻). EPP·전원 프로필(`powerprofilesctl`)은 performance 이고 부하 중 P코어 5.3 GHz(최대). 바꾸지 않는다 |
| `DLSS increasing input dimensions: Render resolution of (320, 240)` | **확인 필요 (6단계 전에 결정).** Isaac Sim 기본값(`SimulationApp` `anti_aliasing: 3` = DLSS, `rtx.post.dlss.execMode = 0` = Performance)이라 640x480 카메라를 절반 해상도로 렌더하고 DLSS 로 키운다. 우리 코드가 켠 것이 아님. 실물 RealSense 는 원래 해상도라 VLA 학습 이미지의 sim2real 차이일 수 있다. 다 모은 뒤 바꾸면 이미지 분포가 달라지므로 PLAN 3단계에서 비교 후 결정 |
| `Forcing fy to fx (618.5508495 != 618.5508683)` | 렌더러가 정사각 픽셀만 지원해 camera_info 의 fy 를 fx 로. 1-7 에서 이미 fy ≈ fx 로 근사 (실물 fy 618.956, 0.07% 차이). stage1 은 camera_info 를 쓰지 않음 |
| `material:binding not found for /World/Drone/rotor0` | 드론 로터에 재질 연결 없음. 동작 영향 없음 |
| `maxHistoryTransformCount` | 노출이 긴 카메라의 모션 블러용. 모션 블러를 쓰지 않아 무관 |
| asset_converter pxr 중복, `omni.hydra was already registered`, `pxr.Semantics deprecated`, `ITimeline deprecated`, replicator material 설정 | Isaac Sim 내부 메시지. 무시 |

---

## 10. 참고

- Isaac Sim Multi-Tick Rendering (카메라 `omni:sensor:tickRate`, 시계 3개 맞추기): https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_multitick_rendering.html
- Isaac Sim ROS 2 FAQ (`ros2 topic hz` 는 wall 기준, RTF): https://docs.isaacsim.omniverse.nvidia.com/6.1.0/ros2_tutorials/help/ros2_faq.html
- Isaac Sim ROS 2 Reference Architecture (OmniGraph + rclpy 혼합): https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/ros2_reference_architecture.html
- 예제 `~/isaacsim/standalone_examples/api/isaacsim.ros2.bridge/camera_rclpy_async.py` (rclpy 로 이미지를 보내면 sim 스레드가 막히는 이유)
