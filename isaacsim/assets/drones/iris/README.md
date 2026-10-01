# 3DR Iris (Pegasus Simulator 에셋, 원본 그대로)

- 출처: https://github.com/livealive7/PegasusSimulator (Pegasus Simulator 6.0.1 포크), commit `b1256ca725ce3dbd3253a889e3851d54b62da54f`
  `extensions/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd` (sha256 앞 12자 `09f8390b45a6`)
- 라이선스: BSD 3-Clause (`LICENSE.PegasusSimulator`, Copyright (c) 2023 Marcelo Fialho Jacinto)
- 받은 날짜: 2026-10-01. **원본 수정 금지** — 축소·질량·관절 고정은 씬 레이어(`scripts/drone.py`, 설정 `config/drone_iris.yaml`)에서 덮어쓴다
- 3DR Iris 는 예전 PX4 Gazebo SITL 의 기본 기체. 2단계 2번(PX4)에서도 같은 외형을 쓸 수 있다

## 원본 구조 (2026-10-01 확인)
- defaultPrim `/vehicle` (ArticulationRootAPI, self-collision 켬), Z up, 1 m 단위
- `/vehicle/body`: 강체, mass 1.5 kg, 충돌 = 몸체+팔 메시 하나 (18,171 점, convexDecomposition)
  - 전체 x −143~156, y −241~237, z −67~47 mm. 가운데 몸체 길이(y = 0 단면 x) −99~113 mm,
    **가장 좁은 허리 폭(x = 0 단면 y) −54~51 mm ≈ 105 mm** → 그리퍼 완전 열림 107 mm 로는 잡을 수 없어 **0.75 배로 축소해 씀**
- `/vehicle/rotor0~3`: 프로펠러 강체 (질량 미지정), `rotorN/jointN` revolute Z 축 (body ↔ rotor)
- 저장된 초기 속도가 있음 (rotor0 각속도 9.4 rad/s 등) → 씬 레이어에서 0 으로
