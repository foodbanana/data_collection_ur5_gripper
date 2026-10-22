# D455 카메라 모양 (third view)

`d455.usdc` = `isaacsim/scripts/build_d455_usd.py` 가 만든 파일. 손으로 고치지 않는다 (다시 빌드).

- 출처: ROS 2 Jazzy `realsense2_description` (Intel RealSense, **Apache-2.0**) 의 `meshes/d455.stl`, `urdf/_d455.urdf.xacro`
- 실물 third view 카메라는 D456 (메시 없음). 외형이 같은 D455 (124 × 29 × 26 mm) 로 대신한다
- 최상위 prim `/d455` = 아랫면 1/4-20 나사 자리 (x 앞, y 왼쪽, z 위)
  - `visual`: 메시 (보이기만)
  - `collision`: 몸체 상자 (정적 collider, purpose guide = 렌더 안 함)
  - `color_optical_frame`: RGB 렌즈 (+z 앞, +y 아래, +x 오른쪽). 나사 자리에서 앞 11.2 mm, 오른쪽 11.5 mm, 위 14.5 mm
- 씬 배치·받침대는 `drone_scene.add_third_view_camera` (설정 `config/third_view_camera.yaml`)
