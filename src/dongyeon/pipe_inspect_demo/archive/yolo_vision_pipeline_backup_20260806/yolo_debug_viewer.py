"""압축된 YOLO 결함 검출 영상을 OpenCV 창으로 실시간 표시한다."""

import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

DEFAULT_TOPIC = "/defect/debug_image/compressed"
DEFAULT_WINDOW_NAME = "Pipe YOLO Seg Detection"
FPS_SAMPLE_SIZE = 30


class YoloDebugViewer(Node):
    """YOLO 디버그 CompressedImage를 구독해 FPS 정보와 함께 표시한다."""

    def __init__(self):
        """영상 토픽과 창 이름 parameter 및 압축 영상 구독자를 초기화한다."""
        super().__init__("yolo_debug_viewer")
        self.declare_parameter("image_topic", DEFAULT_TOPIC)
        self.declare_parameter("window_name", DEFAULT_WINDOW_NAME)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.window_name = str(self.get_parameter("window_name").value)
        self.frame_times = deque(maxlen=FPS_SAMPLE_SIZE)
        self.frame_count = 0
        self.latest_image = np.zeros((360, 640, 3), dtype=np.uint8)
        cv2.putText(self.latest_image, "Waiting for camera...", (145, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2, cv2.LINE_AA)
        self.create_subscription(CompressedImage, self.image_topic, self._on_image, qos_profile_sensor_data)
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 640, 640)
        self.create_timer(1.0 / 30.0, self._refresh_window)
        self.get_logger().info(f"YOLO 실시간 뷰어 시작: topic={self.image_topic}, 종료=Q/Esc")

    def _on_image(self, msg):
        """JPEG 검출 영상을 디코딩하고 FPS·해상도를 표시한 뒤 키 입력을 처리한다."""
        encoded = np.frombuffer(msg.data, dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().warn("YOLO 디버그 압축 영상을 디코딩할 수 없어.", throttle_duration_sec=5.0)
            return
        now = time.perf_counter()
        self.frame_times.append(now)
        self.frame_count += 1
        fps = self._calculate_fps()
        height, width = image.shape[:2]
        label = f"FPS {fps:.1f} | {width}x{height} | frame {self.frame_count}"
        cv2.putText(image, label, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(image, label, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
        self.latest_image = image

    def _refresh_window(self):
        """새 영상 유무와 무관하게 OpenCV 창과 사용자 입력을 계속 처리한다."""
        cv2.imshow(self.window_name, self.latest_image)
        if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q"), 27):
            self.get_logger().info("사용자 입력으로 YOLO 실시간 뷰어를 종료해.")
            rclpy.shutdown()

    def _calculate_fps(self):
        """최근 수신 시각 표본을 이용해 실제 영상 표시 FPS를 계산한다."""
        if len(self.frame_times) < 2:
            return 0.0
        elapsed = self.frame_times[-1] - self.frame_times[0]
        return 0.0 if elapsed <= 0.0 else (len(self.frame_times) - 1) / elapsed

    def close_window(self):
        """노드가 사용한 OpenCV 표시 창을 닫는다."""
        cv2.destroyWindow(self.window_name)
        cv2.waitKey(1)


def main(args=None):
    """ROS 2를 초기화하고 YOLO 디버그 영상 뷰어를 종료할 때까지 실행한다."""
    rclpy.init(args=args)
    node = None
    try:
        node = YoloDebugViewer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close_window()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
