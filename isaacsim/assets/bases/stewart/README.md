# Stewart 플랫폼 (움직이는 베이스, sim 전용)

`stewart.usda` = `isaacsim/scripts/build_stewart_usd.py` 가 `isaacsim/config/base_stewart.yaml` 로 만든 파일. **손으로 고치지 않는다** (설정을 바꾸고 다시 빌드).
실물이 없는 sim 전용 플랫폼이고 기본 도형만 쓴다 (메시 = 원기둥·구). UR5 를 윗판에 올려 6 축 사인파로 흔드는 베이스 (PLAN 8단계, 고정 베이스 씬과 별도).

## 출처
- 부착 각도: `daniel-s-ingram/stewart` (MIT) 커밋 `f276d049` (2019-07-27) `src/ik.cpp` 의 b·p 좌표 (120° 마다 두 개씩 짝, ±7.2°). 원본은 레포 밖 `~/stewart_ws/stewart`
- 크기는 원본과 다르게 새로 정함 (`isaacsim/scripts/design_stewart.py`, 2026-10-28): 원본은 판 반지름 1.0 m·높이 2.0 m 로 너무 크고, 같은 배율로 줄이면 조건수 9.4·다리 간격 10 cm 라 아래 크기 (조건수 3.3, 간격 15 cm) 로 함
- 메시 후보 (나중에 수집 전 모양을 정할 때): `mlayek21/Stewart-Platform` (MIT) 커밋 `7d14aaf6`, 레포 밖 `~/stewart_ws/Stewart-Platform`

## 구조
```text
/stewart                       바닥 가운데 (z 0 = 바닥, x = 로봇 정면)
 ├─ base_plate/geom            아랫판 원기둥 r 0.9, 두께 0.05 (정적 collider)
 │  └─ joint_1 … 6             아랫판 부착점 관절 구 (r 0.8, 보이기만)
 ├─ top_plate                  윗판 = kinematic 강체. 원점 = 회전 중심 = 윗판 윗면 가운데 (= 로봇 base 자리), 가운데 자세 z 1.0
 │  ├─ geom                    원기둥 r 0.7, 두께 0.05 (충돌)
 │  ├─ imu_link                윗판 아랫면 가운데 (회전 중심 기준 (0, 0, −0.05), 물리 없음)
 │  └─ joint_1 … 6             윗판 부착점 관절 구 (r 0.6, 보이기만)
 └─ legs/leg_1 … 6             보이기만 (충돌·질량 없음), 가운데 자세로 놓임
    ├─ cylinder (+geom)        아랫판 부착점에서 다리 방향으로 0.7225 m (r 0.045)
    └─ rod (+geom)             윗판 부착점에서 아랫판 쪽으로 0.7225 m (r 0.025)
```

| 항목 | 값 |
|---|---|
| 윗판 회전 중심 높이 (가운데 자세) | 1.0 m |
| 부착 반지름 아랫판 / 윗판 | 0.8 / 0.6 m |
| 다리 길이 (부착점 사이) | 가운데 1.067 m, 범위 0.747 ~ 1.395 m (신축 0.648 m, 비율 1.87) |
| 실린더·로드 길이 | 각 0.7225 m = (1.395 + 겹침 0.05) / 2 → 가장 짧을 때 여유 24.5 mm |
| 강체 / 충돌 | 윗판 1 (kinematic) / 아랫판·윗판 2 |

- 다리 i 는 아랫판 `joint_i` → 윗판 `joint_i`. 움직일 때 다리 길이·방향은 `moving_base.py` 가 역기구학 `L_i = ‖p + R·P_i − B_i‖` 로 갱신 (S4)
- 움직임 식·프리셋 (`small`·`mid`·`big`·`lab`) 은 `base_stewart.yaml` 주석과 `docs/PLAN.md` 8단계

## 다시 만들기
```bash
python3 isaacsim/scripts/design_stewart.py --config isaacsim/config/base_stewart.yaml    # 크기 확인 (numpy, 몇 초)
~/isaacsim/python.sh isaacsim/scripts/build_stewart_usd.py --config isaacsim/config/base_stewart.yaml
```
