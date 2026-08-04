"""[공용] 파이썬 버전 구분 — 어느 인터프리터에서 돌아야 하는 코드인지 못박는다.

## 왜 나뉘는가

**Isaac Sim 5.1 은 Python 3.11 전용**이다. 우리 ROS 2 Humble 은 시스템 파이썬
3.10 으로 빌드돼 있다. 둘은 확장 모듈 ABI 가 달라 서로의 라이브러리를 못 읽는다.

    Isaac Sim 안        Python 3.11    pxr / omni / isaacsim
    ROS 2 노드          Python 3.10    rclpy / pipe_msgs
    순수 로직           둘 다          numpy / cv2 만

**Isaac Sim 을 띄우는 터미널에서 `/opt/ros/humble/setup.bash` 를 source 하면
안 된다.** 3.10 라이브러리가 앞에 잡혀 심볼이 충돌한다.

## 그런데 통신은 왜 되는가

**DDS 가 파이썬 버전과 무관하게 데이터를 나르기 때문이다.** 3.11 쪽에서 발행한
`/scout/front/image_raw` 를 3.10 쪽 노드가 그대로 받는다. 토픽 이름과 메시지
타입만 맞으면 된다.

## 다만 조건이 있다 — rclpy 를 쓰는가

    표준 메시지만 + OmniGraph 로 발행   아무것도 source 안 함. 그냥 실행
    Python 에서 rclpy 를 쓴다          ROS 2 를 3.11 로 다시 빌드해야 함

우리 `camera/rig.py` 와 `robot/state_bridge.py` 는 **rclpy 로 직접 발행**한다.
그래서 Isaac Sim 쪽에 3.11 용 ROS 2 빌드가 필요하다. 한 번만 하면 된다:

    git clone https://github.com/isaac-sim/IsaacSim-ros_workspaces
    cd IsaacSim-ros_workspaces
    git checkout IsaacSim-5.1.0   # 🚨 main 은 Python 3.12 만 만든다
    ./build_ros.sh -d humble -v 22.04
    source build_ws/humble/humble_ws/install/local_setup.bash
    source build_ws/humble/isaac_sim_ros_ws/install/local_setup.bash
    ~/isaacsim/isaac-sim.sh          # 같은 터미널에서

**우리가 쓰는 커스텀 메시지 `pipe_msgs` 는 Isaac Sim 쪽에 안 들어간다.** 3.11
빌드가 필요한 건 rclpy 하나뿐이고, 발행하는 메시지는 전부 표준
(`sensor_msgs`, `std_msgs`) 이다. `pipe_msgs` 는 3.10 쪽 노드끼리만 주고받는다.
"""

import os
import sys

ISAAC_PY = (3, 11)
ROS_PY = (3, 10)

SKIP = os.environ.get("PIPE_SKIP_PYVER") == "1"

_BUILD_ROS = """
Isaac Sim 안에서 rclpy 를 쓰려면 ROS 2 를 3.11 로 빌드해야 한다.
  git clone https://github.com/isaac-sim/IsaacSim-ros_workspaces
  cd IsaacSim-ros_workspaces
  git checkout IsaacSim-5.1.0   # 🚨 main 은 Python 3.12 만 만든다
  ./build_ros.sh -d humble -v 22.04
  source build_ws/humble/humble_ws/install/local_setup.bash
  source build_ws/humble/isaac_sim_ros_ws/install/local_setup.bash
그 다음 같은 터미널에서 isaac-sim.sh 를 띄운다.
"""


def _fail(msg):
    if SKIP:
        print(f"[pyver] 무시함(PIPE_SKIP_PYVER=1): {msg}", file=sys.stderr)
        return
    raise RuntimeError(msg)


def _here():
    return "%d.%d" % sys.version_info[:2]


def require_isaac(who="", needs_rclpy=False):
    """Isaac Sim 인터프리터에서만 돌아야 하는 파일이 맨 위에서 부른다.

    3.10 으로 잘못 띄우면 `from isaacsim import ...` 가 먼저 터지는데, 그
    메시지로는 원인을 알 수 없다. 그래서 그 전에 막는다.
    """
    if sys.version_info[:2] != ISAAC_PY:
        _fail(f"{who or '이 파일'} 은 Isaac Sim 전용(Python "
              f"{ISAAC_PY[0]}.{ISAAC_PY[1]})인데 {_here()} 로 실행됐다.\n"
              f"  isaac_python 으로 실행할 것 "
              f"(~/isaacsim/python.sh 또는 그 별칭).\n"
              f"  ROS 노드를 띄우려던 것이면 그 파일은 3.10 쪽이다 — "
              f"경로를 확인할 것.")
    if needs_rclpy:
        try:
            import rclpy  # noqa: F401
        except ImportError as e:
            _fail(f"{who or '이 파일'} 은 rclpy 로 직접 발행하는데 Isaac Sim "
                  f"3.11 환경에 rclpy 가 없다 ({e}).{_BUILD_ROS}")


def require_ros(who=""):
    """ROS 2 노드로 도는 파일이 맨 위에서 부른다."""
    if sys.version_info[:2] != ROS_PY:
        _fail(f"{who or '이 파일'} 은 ROS 2 Humble 노드(Python "
              f"{ROS_PY[0]}.{ROS_PY[1]})인데 {_here()} 로 실행됐다.\n"
              f"  source /opt/ros/humble/setup.bash 후 python3 로 실행할 것.\n"
              f"  Isaac Sim 안에서 돌리려던 것이면 이 파일은 그쪽이 아니다 — "
              f"Isaac 쪽은 camera/ robot/ welder/articulate.py 다.")


def describe():
    """지금 인터프리터가 어느 쪽인지 한 줄로."""
    v = _here()
    side = ("Isaac Sim" if sys.version_info[:2] == ISAAC_PY else
            "ROS 2" if sys.version_info[:2] == ROS_PY else "알 수 없음")
    return f"Python {v} → {side} 쪽"


if __name__ == "__main__":
    print(describe())
    for mod, tag in (("pxr", "Isaac"), ("omni", "Isaac"), ("isaacsim", "Isaac"),
                     ("rclpy", "ROS"), ("numpy", "공용"), ("cv2", "공용")):
        try:
            __import__(mod)
            got = "있음"
        except ImportError:
            got = "없음"
        print(f"  {tag:>6}  {mod:<10} {got}")
