# pipe_inspect

배관 점검 로봇 — Isaac Sim 카메라 토픽(`/rgb`, `/depth`) 확인용 ROS2 패키지 (PC B 측).

시뮬레이션(PC A 측) 코드와 작업 기록은 `isaacpjt/dongmin/README.md` 참조.

## 빌드

```bash
ros_set                                  # source /opt/ros/humble/setup.bash
cd ~/cobot3_ws
colcon build --packages-select pipe_inspect
source install/setup.bash
```

## 사용 (2개 터미널)

**터미널 1 — Isaac Sim (반드시 GUI):**
```bash
cd ~/cobot3_ws/isaacpjt/dongmin/graphic_file/robot
PYTHONUNBUFFERED=1 isaac_python pipe_chain_ros_demo.py
```

**터미널 2 — 토픽 확인:**
```bash
ros_set && source ~/cobot3_ws/install/setup.bash
ros2 topic list                          # /rgb /depth
# /camera_info 는 발행하지 않는다 — CameraInfo writer 가 이미지 파이프라인을
# 깨는 문제(실측). 내부 파라미터(fx=763.5 등)는 시뮬 시작 로그에 출력된다.
ros2 topic hz /rgb
ros2 run pipe_inspect camera_check       # 1초마다 Hz + 밝기 + depth 통계
```

## 프레임이 안 들어올 때 (CLAUDE.md E1~E3)

1. **Isaac 이 headless 면 토픽은 보여도 프레임이 오지 않는다 (E2).** GUI 로 실행할 것
2. 시뮬레이션이 **Play 상태**여야 발행된다 (OnPlaybackTick)
3. **ROS_DOMAIN_ID** 가 양쪽 셸에서 같아야 한다 (`.bashrc` 기본 143).
   멀티 PC 면 `fastdds_whitelist.xml` 에 두 PC IP 가 모두 있어야 한다 (E3)

## 현재 상태 (2026-08-01)

- `camera_check` 노드 구현 완료. cv_bridge 없이 `sensor_msgs/Image` 를 numpy 로
  직접 해석하고, 1초마다 토픽별 Hz + 통계를 출력한다:
  - `/rgb`: 해상도, 평균 밝기 (0 에 가까우면 조명 문제 — C2)
  - `/depth`: 해상도, 유효 픽셀 비율, min/중앙값 (m)
- 발행 측(`pipe_chain_ros_demo.py`)은 replicator writer 방식으로 `/rgb` `/depth` 발행 확인.
  `/camera_info` 는 발행하지 않는다 (writer 가 이미지 파이프라인을 깨는 문제 실측).
  카메라 내부 파라미터가 필요하면 시뮬 시작 로그의 K 값(fx=763.5 등)을 쓸 것.

## 다음 작업 후보

- 결함 탐지 노드 추가: `/rgb` 구독 → 결함 판정 → 판정 토픽 발행
  (시뮬 내 탐지 로직은 `pipe_inspect_demo.py` 에 이미 있음 — ROS 노드로 이식)
- 판정 결과를 시뮬 쪽 OmniGraph `ROS2Subscriber` 로 되받는 왕복 연동
  (`isaacpjt/dongmin/M0609/6_pick_place_color.py` 의 OBSERVE 패턴 참조)
- 임계값은 실제 프레임 통계로 정할 것 — fallback 금지 (C8)
