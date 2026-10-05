# ros2_check — check_ros2.py 기준 명령 (docs/PLAN.md 3-8)

| 파일 | 내용 | 만든 방법 |
|------|------|-----------|
| `commands_success.csv` | 드론 잡기 명령 (60 Hz: t, 팔 6 관절 목표 [rad], gripper_goal_raw). 기본 씬 (드론 `scene_drone.yaml` drone_pos, 팔 home_pose) 에서 기하 제어기 드론을 잡는다 | `grasp_demo.py --headless --case success --flight geometric` 이 남긴 `commands_success.csv` (2026-10-05) |

씬·드론·그리퍼 설정이 바뀌어 재생으로 잡히지 않으면 위 명령으로 다시 만들어 덮어쓴다.
