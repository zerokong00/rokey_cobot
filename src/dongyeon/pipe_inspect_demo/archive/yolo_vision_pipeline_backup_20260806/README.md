# YOLO Seg 파이프라인 백업 (2026-08-06)

`pipe_inspect_demo/pipe_inspect_demo/`의 당시 코드를 그대로 복사한 스냅샷이다.
**원본은 그대로 살아 있고 삭제하지 않았다** — `setup.py`의 console_scripts
(`pipe_vision`, `pipe_report`, `pipe_coordinator`, `yolo_viewer`,
`repair_target_test`)가 여전히 이 경로들을 가리키므로 지우면 빌드가 깨진다.

## 왜 백업했나

메인 결함검출 방식을 YOLOv8-Seg에서 `integration_test/defect/`의 OpenCV
기하 검출(학습 불필요, `repair_demo.py`의 `find_wall_hole`/`find_weld_bead`
로직 기반)로 옮기기로 했다. 이 폴더는 그 전환 시점의 참고용 스냅샷이다.

## 그대로 재사용된 설계

- `defect_position_tracker.py`의 입구 기준 오도메트리·IMU 결합, 원주 각도
  계산 아이디어는 `integration_test/localization/deadreckon.py`의
  `DeadReckoner`(arc_mm + roll_deg)가 이미 같은 역할을 하고 있어 그쪽을
  그대로 썼다(새로 만들지 않았다).
- `repair_target_registry.py`의 "결함 ID별 상태 관리(pending/in_progress/
  completed/failed)" 개념은 아직 `integration_test`에 없다 — 수리로봇 이동
  단계를 실제로 붙일 때 참고할 것.

## 알아 둘 것 — 이 스냅샷 자체의 미해결 문제

전환 작업 중 실측한 것들이다. 나중에 이 백업을 다시 살릴 일이 있으면 먼저
고쳐야 한다.

1. `pipe_vision_node.py`가 구독하는 `/odom`(`nav_msgs/Odometry`)과
   `/imu`(`sensor_msgs/Imu`, 쿼터니언)를 실제로 발행하는 노드가
   `integration_test`에 없다. `robot/state_bridge.py`는 스칼라
   `imu_roll`/`imu_yaw_rate`만, `localization/node.py`는 `pose`를 JSON
   문자열로 낸다 — 메시지 타입이 안 맞는다.
2. `/rgb/compressed`, `/depth/compressed`(`CompressedImage`)를 구독하는데
   `camera/rig.py`는 `/front/rgb`, `/front/depth`를 raw `Image`(32FC1)로
   낸다 — 토픽명·인코딩이 둘 다 다르다.
3. `defect_position_tracker.py`가 만드는 `repair_target` 딕셔너리에는
   `position_xyz_m`만 있고 `repair_target_registry.py`/
   `pipe_coordinator_node.py`가 요구하는 `navigation_goal_xyz_m`/
   `orientation_xyzw` 키가 없다. `registration`도 실제로 `"aligned"`가
   되는 경로가 없다 — `RepairTargetRegistry.add_aligned_report()`가 성공하는
   경로가 이 스냅샷 안에서 한 번도 안 이어져 있었다.
