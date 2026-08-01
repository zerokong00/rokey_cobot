# pipe_inspect

배관 점검 로봇 — Isaac Sim 카메라 토픽(`/rgb`, `/depth`) 확인용 ROS2 패키지 (PC B 측).

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
