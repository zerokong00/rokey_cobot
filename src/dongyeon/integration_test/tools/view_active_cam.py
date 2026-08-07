#!/usr/bin/env python3
"""[ROS2 Humble, Python 3.10] real_map_demo.py 의 활성 카메라 스트림을 실시간으로 본다.

real_map_demo.py(Isaac Sim, Python 3.11)가 상태별로(전진→front_camera,
후진→back_camera, 정렬~검증→torch_camera) 활성 카메라 영상을 압축 JPEG
로 발행한다 — 이 스크립트는 그걸 구독해서 OpenCV 로 디코딩하고 화면에 띄운다.

실행 (Isaac Sim 을 띄운 터미널과는 **별개의** 새 터미널에서):
    source /opt/ros/humble/setup.bash
    python3 tools/view_active_cam.py

구독 토픽
    /repair_robot/active_cam/rgb/compressed   sensor_msgs/CompressedImage
    /repair_robot/active_cam/which             std_msgs/String  (현재 활성 카메라 이름)

m 키 — 원본→그레이스케일→엣지검출(Canny)→결함검출 하이라이트 순서로 순환.
q 또는 ESC 로 창을 닫아 종료한다.

결함검출 하이라이트는 real_map_demo.py::find_wall_hole() 과 같은 핵심 로직
(밝기 임계값 이하 연결영역 = 결함 후보)을 쓰지만, 이 뷰어는 압축 컬러
영상만 받으므로 원본이 쓰는 **Depth 테두리 필터**(관 저 끝과 진짜 결함을
가르는 근거)와 **결함이 있어야 할 자리(expect_px) 최근접 선택**은 못 한다
— 그래서 "그럴듯한 어두운 덩어리를 전부" 표시한다. 실제 결함 판정은 항상
real_map_demo.py 쪽(Depth 있음)이 한다 — 이 하이라이트는 참고용 시각화다.

OpenCV 처리를 더 추가하고 싶으면 `_process()` 함수에 모드를 하나 더
넣으면 된다 — 그 시점에 `frame` 은 BGR uint8 numpy 배열(cv2 가 기대하는
그대로의 순서)이다.
"""
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

# real_map_demo.py 의 find_wall_hole() 과 같은 값(HOLE_DARK_FRAC) — 결함
# 후보로 볼 "얼마나 어두워야 하는가"의 기준. 화면 전체 분포의 5~90 퍼센타일
# 사이에서 이 비율 지점을 임계값으로 쓴다.
HOLE_DARK_FRAC = 0.12
MODES = ("raw", "gray", "edge", "defect")


def _process(frame, mode):
    """frame(BGR) 을 mode 에 맞게 처리해 표시용 BGR 이미지를 돌려준다."""
    if mode == "gray":
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if mode == "edge":
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 80, 160)
        return cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    if mode == "defect":
        return _defect_highlight(frame)
    return frame.copy()


def _defect_highlight(frame):
    """어두운 연결영역(결함 후보)을 찾아 원본 위에 빨간 원 + 픽셀수로 표시."""
    disp = frame.copy()
    gray = cv2.medianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), 5)
    h, w = gray.shape
    lo, hi = np.percentile(gray, 5), np.percentile(gray, 90)
    thr = lo + HOLE_DARK_FRAC * (hi - lo)
    n, lab, st, ce = cv2.connectedComponentsWithStats(
        (gray < thr).astype(np.uint8), 8)
    min_px = max(60, int(0.0009 * h * w))    # HOLE_MIN_PX 와 같은 취지
    big_px = 0.60 * h * w                     # 관 저 끝처럼 화면 대부분을
    for i in range(1, n):                     # 덮는 덩어리는 결함이 아니다
        area = int(st[i, cv2.CC_STAT_AREA])
        if area < min_px or area > big_px:
            continue
        cx, cy = ce[i]
        r = max(6, int(round(np.sqrt(area / np.pi))))
        cv2.circle(disp, (int(cx), int(cy)), r, (0, 0, 255), 2)
        cv2.putText(disp, f"{area}px", (int(cx) - r, int(cy) - r - 6),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
    return disp


class ActiveCamViewer(Node):
    """압축 이미지·활성 카메라 이름을 받아 최신 프레임만 들고 있는다."""

    def __init__(self):
        super().__init__("active_cam_viewer")
        # 🚨 Isaac 쪽(ActiveCamBridge)과 같은 QoS 를 써야 한다 — RELIABLE 로
        #    구독하면 BEST_EFFORT 발행자와 안 맞아 아무것도 안 들어온다.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.which = "?"
        self.frame = None          # BGR uint8, cv2.imshow 가 바로 받는 순서
        self.n_frames = 0
        self.create_subscription(String, "/repair_robot/active_cam/which",
                                 self._on_which, qos)
        self.create_subscription(CompressedImage,
                                 "/repair_robot/active_cam/rgb/compressed",
                                 self._on_image, qos)

    def _on_which(self, msg):
        self.which = msg.data

    def _on_image(self, msg):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        # Isaac 쪽이 BGR 로 뒤집어서 imencode 했으므로(camera/rig.py 와 같은
        # 관례) imdecode 결과도 그대로 BGR — cv2.imshow 에 바로 넣으면 된다.
        bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if bgr is None:
            return
        self.frame = bgr
        self.n_frames += 1


def main():
    rclpy.init()
    node = ActiveCamViewer()
    print("구독 시작 — /repair_robot/active_cam/{rgb/compressed,which}")
    print("Isaac Sim 쪽(real_map_demo.py)이 GUI 로 돌고 있어야 프레임이 온다.")
    print(f"m 키 — 처리 모드 순환({' → '.join(MODES)}), q 또는 ESC — 종료")
    mode_idx = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.frame is not None:
                mode = MODES[mode_idx]
                disp = _process(node.frame, mode)
                cv2.putText(disp, f"{node.which}  #{node.n_frames}  [{mode}]",
                           (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                           (0, 255, 0), 2, cv2.LINE_AA)
                cv2.imshow("repair_robot active_cam", disp)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key == ord("m"):
                mode_idx = (mode_idx + 1) % len(MODES)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        print(f"종료 — 받은 프레임 {node.n_frames}개, 마지막 활성 카메라 "
              f"'{node.which}'")
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():          # timeout/신호로 이미 shutdown 됐으면 또 부르지 않는다
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
