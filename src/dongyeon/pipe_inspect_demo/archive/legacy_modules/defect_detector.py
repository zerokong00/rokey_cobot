"""ROS 2 입출력을 ROS 비의존 DarkBlobDetector에 연결하는 노드."""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray

from pipe_inspect_demo.defect_detection import DarkBlobDetector


def image_to_rgb(msg: Image) -> np.ndarray:
    """패딩(msg.step)을 고려해 ROS Image를 RGB uint8 배열로 변환한다."""
    encoding = msg.encoding.lower()
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(
        encoding
    )
    if channels is None:
        raise ValueError(f"지원하지 않는 RGB encoding: {msg.encoding}")
    if msg.height <= 0 or msg.width <= 0 or msg.step < msg.width * channels:
        raise ValueError("잘못된 Image 크기 또는 step")
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    expected = msg.height * msg.step
    if raw.size < expected:
        raise ValueError(f"Image 데이터 부족: {raw.size} < {expected}")
    rows = raw[:expected].reshape(msg.height, msg.step)
    image = rows[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
    if encoding == "rgb8":
        return image.copy()
    if encoding == "bgr8":
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    if encoding == "rgba8":
        return cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)
    if encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2RGB)
    return cv2.cvtColor(image[:, :, 0], cv2.COLOR_GRAY2RGB)


class DefectDetectorNode(Node):
    def __init__(self):
        super().__init__("defect_detector")
        defaults = {
            "image_topic": "/rgb",
            "detected_topic": "/defect_detected",
            "info_topic": "/defect_info",
            "debug_topic": "/defect/debug_image",
            "calibration_frames": 12,
            "dark_ratio": 0.45,
            "annulus_inner": 0.45,
            "annulus_outer": 0.92,
            "min_area": 40,
            "noise_area_scale": 2.0,
            "confirm_frames": 3,
            "release_frames": 3,
            "publish_debug": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._detector = DarkBlobDetector(
            calibration_frames=self.get_parameter("calibration_frames").value,
            dark_ratio=self.get_parameter("dark_ratio").value,
            annulus_inner=self.get_parameter("annulus_inner").value,
            annulus_outer=self.get_parameter("annulus_outer").value,
            min_area=self.get_parameter("min_area").value,
            noise_area_scale=self.get_parameter("noise_area_scale").value,
            confirm_frames=self.get_parameter("confirm_frames").value,
            release_frames=self.get_parameter("release_frames").value,
        )
        self._publish_debug = bool(self.get_parameter("publish_debug").value)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Image, str(self.get_parameter("image_topic").value), self._on_image, qos
        )
        self._detected_pub = self.create_publisher(
            Bool, str(self.get_parameter("detected_topic").value), 10
        )
        self._info_pub = self.create_publisher(
            Float32MultiArray, str(self.get_parameter("info_topic").value), 10
        )
        self._debug_pub = self.create_publisher(
            Image, str(self.get_parameter("debug_topic").value), qos
        )
        self._last_calibrated = False
        self._last_state = None
        self._last_reset_reason = None
        self.get_logger().info("결함 검출 노드 시작: 초기 정상 배관 영상으로 보정합니다")

    def _publish_debug_image(self, source, rgb, result):
        if not self._publish_debug or self._detector.mask is None:
            return
        view = rgb.copy()
        overlay = np.zeros_like(view)
        overlay[self._detector.mask] = (0, 80, 0)
        view = cv2.addWeighted(view, 1.0, overlay, 0.25, 0.0)
        if result.candidate is not None:
            x, y, width, height = result.candidate.box
            color = (255, 0, 0) if result.stable_detected else (255, 180, 0)
            cv2.rectangle(view, (x, y), (x + width, y + height), color, 2)
        label = "DEFECT" if result.stable_detected else "CLEAR"
        cv2.putText(
            view,
            label,
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 0, 0) if result.stable_detected else (0, 255, 0),
            2,
        )
        debug = Image()
        debug.header = source.header
        debug.height, debug.width = view.shape[:2]
        debug.encoding = "rgb8"
        debug.is_bigendian = 0
        debug.step = debug.width * 3
        debug.data = view.tobytes()
        self._debug_pub.publish(debug)

    def _on_image(self, msg):
        try:
            rgb = image_to_rgb(msg)
            result = self._detector.process(rgb)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
            return

        self._detected_pub.publish(Bool(data=result.stable_detected))
        if result.calibrated and not self._last_calibrated:
            self.get_logger().info(
                f"캘리브레이션 완료: 임계값={result.threshold:.1f}, "
                f"최소면적={result.area_min}px"
            )
        self._last_calibrated = result.calibrated

        if self._detector.last_reset_reason != self._last_reset_reason:
            reason = self._detector.last_reset_reason
            if reason == "low_brightness":
                self.get_logger().error("영상이 너무 어두워 캘리브레이션을 다시 시작합니다")
            elif reason == "resolution_changed":
                self.get_logger().warn("해상도가 바뀌어 캘리브레이션을 다시 시작합니다")
            self._last_reset_reason = reason

        if not result.calibrated:
            return
        candidate = result.candidate
        info = (
            [0.0, -1.0, -1.0, 0.0, float(result.threshold)]
            if candidate is None
            else [
                candidate.confidence,
                candidate.center_x,
                candidate.center_y,
                float(candidate.area),
                float(result.threshold),
            ]
        )
        self._info_pub.publish(Float32MultiArray(data=info))
        if result.stable_detected != self._last_state:
            self.get_logger().info("결함 확정" if result.stable_detected else "결함 없음")
            self._last_state = result.stable_detected
        self._publish_debug_image(msg, rgb, result)


def main(args=None):
    rclpy.init(args=args)
    node = DefectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
