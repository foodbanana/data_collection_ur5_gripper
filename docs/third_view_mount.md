# third view 카메라 받침대 (D456)

작성: 2026-10-22 · 근거와 위치를 고른 과정은 `docs/PLAN.md` 6단계 "third view 위치 결정"·"third view 받침대 CAD"

실물 third view 카메라 (RealSense D456) 를 sim 과 같은 위치·방향에 고정하는 3D 출력 받침대.

---

## 1. 한눈에 보기

| 항목 | 내용 |
|------|------|
| 출력 파일 | **`isaacsim/cad/out/third_view_mount.stl`** (단위 mm), 편집용 `third_view_mount.step` |
| 만드는 스크립트 | `isaacsim/cad/third_view_mount_cad.py` (CadQuery). 카메라 자세는 `isaacsim/config/third_view_camera.yaml` 에서 읽음 |
| 카메라 위치 | 로봇 base 기준 렌즈 (1.10, −0.30, 0.20) m = 앞 1.1 m, 오른쪽 0.3 m, 상판 위 0.2 m. 위로 48.85°, 수평 방향 148.5° (base +x 에서 왼쪽으로) |
| 카메라 고정 | 카메라 뒷면 M4 × 2 (간격 95 mm) ← **M4 × 8 mm 나사 2 개** |
| 테이블 고정 | 바닥판 Ø5.5 구멍 4 개 (카운터보어 Ø10.5 깊이 3.5) ← M5 볼트 또는 목재 나사 4 개 |
| 크기 | 바닥판 115 × 100 × 7 mm, 높이 193.5 mm, 부피 245 cm³ |
| 각도 | 고정 (조절 안 됨). 위치를 바꾸면 스크립트를 다시 돌려 새로 출력 |

---

## 2. 모양

```text
            카메라 (D456)
           /
   뒷면 판 /  ← M4 × 8 나사 2 개 (뒤에서 끼움)
         /
        |  기둥 (20 × 40 mm, 윗면이 판을 따라 48.85° 기울어짐)
        |
     /  |  \   리브 (뒤·양옆, 높이 60 mm)
  ===========  바닥판 (Ø5.5 × 4)   윗면 홈: 십자 + 화살표
```

- 카메라 뒷면 판: 110 × 29 × 5 mm, M4 구멍 Ø4.5 × 2 (간격 95 mm)
- **카메라 아랫면 쪽은 비어 있다**: D456 은 USB-C 단자가 아랫면에 있다. 케이블은 아래로 뺀다
- 스크립트가 확인하는 것 (어긋나면 에러): 카메라 몸체·아랫면 아래 60 mm (USB 자리) 와 겹치지 않음, 나사가 카메라에 0 ~ 4 mm 들어감, 렌즈 높이 = 설정 (200.0 mm)

### D456 고정 구멍 (데이터시트)

RealSense D456 Datasheet (337029-017) Figure 10-14 (`~/Downloads/Intel-RealSense-D456-Datasheet-en.pdf`, 156 쪽):

| 위치 | 규격 | 최대 체결 깊이 |
|------|------|----------------|
| 뒷면 | M4 × 2, 간격 95 mm (몸체 가운데 기준 대칭, 세로 가운데) | **4 mm** |
| 아랫면 | 1/4-20 × 1 (앞면에서 15.8 mm) | 6 mm |

뒷면 판이 5 mm 라서 M4 × 8 이면 카메라에 3 mm 들어간다. **10 mm 이상 나사는 쓰지 않는다** (4 mm 를 넘어 카메라 안쪽을 누름).

---

## 3. 출력

- 바닥판을 아래로 세워서 출력한다
- 기울어진 판의 아랫면은 수직에서 41° → 서포트 없이 될 것으로 본다 (출력해 보고 처지면 서포트)
- 기둥 앞뒤 두께 20 mm 는 판이 바닥에 투영되는 폭 (22 mm) 에 맞춘 값. 실물에서 흔들리면 스크립트의 `POST_INSET`·리브 값을 바꿔 다시 만든다

---

## 4. 설치

1. 카메라 뒷면에 받침대 판을 대고 M4 × 8 나사 2 개로 고정 (나사 머리는 판 뒤, 기둥 양옆)
2. USB-C 케이블을 카메라 아랫면에 꽂고 아래로 뺀다
3. 바닥판 윗면 홈 두 곳을 로봇 base 기준 위치에 맞춘다 (줄자로, 로봇 base 중심에서)

   | 표시 | 뜻 | 로봇 base 기준 (mm): 앞 x, 왼쪽 y |
   |------|----|-------------------------------|
   | 십자 | 카메라 아랫면 1/4-20 자리 바로 아래 | **(1090.9, −307.9)** = 앞 1090.9, 오른쪽 307.9 |
   | 화살표 끝 (십자에서 30 mm) | 카메라가 보는 수평 방향 | **(1065.4, −292.2)** = 앞 1065.4, 오른쪽 292.2 |

   두 점을 맞추면 위치와 방향 (148.5°) 이 같이 맞는다. 화살표는 대략 로봇 쪽을 향한다
4. 바닥판을 M5 볼트 또는 목재 나사 4 개로 테이블에 고정
5. 영상 확인: `ros2 run rqt_image_view rqt_image_view /cam/third_view/color/image_raw` (실물 카메라 노드를 띄운 뒤) → sim third view 영상과 같은 장면인지 본다

- 실물 테이블은 sim 과 같다고 가정한 값이다 (sim: 상판 2.0 × 1.0 m, 로봇이 −x 끝에서 0.25 m 안쪽). 실물 테이블이 다르면 로봇 base 기준 위치만 맞추면 된다 (테이블 가장자리는 기준이 아님)

---

## 5. 다시 만들기

third view 위치 (`third_view_camera.yaml` `position`·`look_at`) 를 바꾸면 각도·높이·설치 위치가 바뀐다.

```bash
# 레포 밖 venv (CadQuery). 결과: isaacsim/cad/out/third_view_mount.step, .stl, preview.png
~/cad/venv/bin/python isaacsim/cad/third_view_mount_cad.py [--screw-len 8]
```

실행 출력에 각도, 높이, 나사 체결 깊이, 설치 기준점 (십자·화살표 끝의 로봇 base 기준 위치) 이 나온다.

- sim 의 받침대는 아직 원기둥 (`third_view_camera.yaml` `mount`). third view 영상에는 받침대가 안 보여 데이터에는 영향 없다

---

## 6. 관련 파일

| 파일 | 내용 |
|------|------|
| `isaacsim/cad/third_view_mount_cad.py` | 받침대 CAD |
| `isaacsim/cad/out/third_view_mount.stl`·`.step` | 출력 파일 |
| `isaacsim/config/third_view_camera.yaml` | sim third view 카메라 위치·intrinsics |
| `isaacsim/scripts/compare_third_view_targets.py` | 위치를 고른 비교 스크립트 |
| `isaacsim/ur5_rh_p12_description/cad/wrist_mount_cad.py` | 손목 카메라 마운트 CAD (같은 방식) |
