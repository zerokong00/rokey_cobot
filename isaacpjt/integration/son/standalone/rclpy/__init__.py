"""[단독판] rclpy 대역 — ROS 없이 Isaac Sim 안에서 기능만 확인한다.

## 이게 왜 있는가

Isaac Sim 5.1 의 Python 3.11 에는 rclpy 가 없다. 넣으려면 ROS 2 를 3.11 로
다시 빌드해야 하고 docker 가 필요하다. 그런데 **지금 확인하려는 것은 PC 간
통신이 아니라 "구현한 것이 돌아가는가"** 다. 그 확인을 하려고 ROS 빌드부터
해야 하는 것은 순서가 뒤집힌 것이다.

그래서 이 판에서는 rclpy 자리에 이 모듈이 들어간다. **발행하는 대신 값을
찍고 이미지를 저장한다.** 코드는 한 줄도 안 고쳤다 — import 되는 것만 다르다.

## 무엇을 보여주는가

토픽이 "나왔다" 는 것은 정상 동작 확인이 아니다. **숫자가 물리적으로 말이
되는지** 봐야 한다. 그래서 이 대역은 Depth 를 받으면 min/max/중앙값과 무효
비율을 찍는다.

    front depth  min 0.024  max 4.87  중앙값 0.061  무효 0.3%   ← 정상
    전부 0 또는 inf        annotator 미부착 또는 headless
    min 25.0               단위가 mm 다 (1000배 틀림)
    max 5.0 만 가득        아무것도 안 맞았다
    무효 30%               판정기가 못 쓴다 (허용 2%)

⚠ **이것은 통신 검증이 아니다.** PC1↔PC2 ROS 통신은 원본 판으로 따로
확인해야 한다.
"""

import os
import sys
import time
from pathlib import Path

_state = dict(inited=False, t0=None)

OUT_DIR = Path(os.environ.get("PIPE_STANDALONE_OUT",
                              Path(__file__).resolve().parents[1] / "out"))


def init(*a, **kw):
    _state["inited"] = True
    _state["t0"] = time.time()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    print("=" * 74)
    print("[단독판] ROS 발행 대신 값을 찍고 이미지를 저장한다")
    print(f"         저장 위치: {OUT_DIR}")
    print("         PC1↔PC2 통신 검증이 아니다 — 기능 확인용이다")
    print("=" * 74)


def shutdown(*a, **kw):
    from .node import summary
    summary()
    _state["inited"] = False


def ok(*a, **kw):
    return _state["inited"]


def spin_once(node=None, timeout_sec=None):
    if node is not None:
        node._run_timers()


def spin(node=None):
    print("[단독판] spin() 은 단독판에서 할 일이 없다. Ctrl-C 로 끝낼 것.")
    try:
        while True:
            spin_once(node)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass


def try_shutdown(*a, **kw):
    if _state["inited"]:
        shutdown()
