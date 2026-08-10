"""real_map_demo.py 의 active_cam(그때그때 활성 카메라)에 YOLO 2차검증만 얹는다.

pipe_vision_node.py(front_camera 고정, RGB/Depth/CameraInfo/Odom/IMU 5-토픽
동기화, 축 위치 추적)와 달리 이 노드는 Depth·Odom·IMU 가 전혀 필요 없다 —
정찰+수리를 한 로봇이 동시에 하는 구조에서는 축 위치가 이미 OpenCV 쪽
(real_map_demo.py 의 코스 s좌표)에서 확정되므로, YOLO 쪽에서 따로 위치를
추적할 이유가 없다(2026-08-07, 사용자 지시). active_cam 은 FSM 상태에 따라
front/back/torch 카메라를 오가며 발행하는데, 이 노드는 그 카메라가 뭐든
상관없이 들어오는 RGB 프레임에 그대로 YOLO Seg 를 돌린다.

절대 이걸로 OpenCV(find_wall_hole/find_weld_bead) 판정을 대체하지 않는다 —
real_map_demo.py 가 /defect/detected, /defect/report_json 을 "참고용 비동기
의견"으로만 구독한다(PipeVisionBridge.yolo_opinion() 참고).

실행:
    source /opt/ros/humble/setup.bash
    ros2 run pipe_inspect_demo active_cam_vision
"""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String

from pipe_inspect_demo.pipe_vision_node import resolve_model_path
from pipe_inspect_demo.ros_image_utils import compressed_to_rgb, rgb_to_compressed_image
from pipe_inspect_demo.yolo_seg_detector import YoloSegDetector


class ActiveCamVisionNode(Node):
    """active_cam RGB 하나만 구독해 YOLO Seg 결과를 발행한다(Depth/Odom/IMU 불필요)."""

    def __init__(self):
        """ROS parameter, 구독·발행기, YOLO Seg 모델을 초기화한다."""
        super().__init__("active_cam_vision")
        defaults = {"model_path": "", "inference_confidence": 0.05, "registration_confidence": 0.8, "device": "", "publish_debug": True, "rgb_topic": "/repair_robot/active_cam/rgb/compressed", "which_topic": "/repair_robot/active_cam/which", "detected_topic": "/defect/detected", "json_topic": "/defect/report_json", "debug_topic": "/defect/debug_image/compressed"}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        model_path = resolve_model_path(self.get_parameter("model_path").value)
        self.detector = YoloSegDetector(model_path, self.get_parameter("inference_confidence").value, self.get_parameter("registration_confidence").value, str(self.get_parameter("device").value))
        self.publish_debug = bool(self.get_parameter("publish_debug").value)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.active_cam_name = "?"
        self.last_debug_detections = []  # 결함이 화면 밖으로 나가도 디버그 표시는 유지
        self.detected_pub = self.create_publisher(Bool, str(self.get_parameter("detected_topic").value), 10)
        self.json_pub = self.create_publisher(String, str(self.get_parameter("json_topic").value), 10)
        self.debug_pub = self.create_publisher(CompressedImage, str(self.get_parameter("debug_topic").value), qos)
        self.create_subscription(String, str(self.get_parameter("which_topic").value), self._on_which, qos)
        self.create_subscription(CompressedImage, str(self.get_parameter("rgb_topic").value), self._on_image, qos)
        self.get_logger().info(f"active_cam YOLO 2차검증 시작: model={model_path}, classes={self.detector.class_names}")

    def _on_which(self, message):
        """지금 active_cam 이 내보내는 카메라 이름(front/back/torch)을 기록한다."""
        self.active_cam_name = message.data

    def _on_image(self, message):
        """active_cam RGB 프레임에 YOLO Seg 를 돌리고 결과를 발행한다."""
        try:
            rgb = compressed_to_rgb(message)
            # 🚨 임시 디버그 — raw_scores 로 등록 문턱(0.8) 미만이라 걸러진
            #    것과 아예 신호가 없는 것을 구분한다(용접 중 torch_camera
            #    미검출 원인 진단용, 2026-08-07). 원인 파악되면 지워도 됨.
            raw_scores = []
            detections = self.detector.infer_rgb_only(rgb, raw_scores_out=raw_scores)
            if not detections and raw_scores:
                best_raw = max(raw_scores, key=lambda item: item[1])
                self.get_logger().info(
                    f"[디버그] camera={self.active_cam_name} 미등록(문턱 미달) — "
                    f"최고 원시점수 {best_raw[0]}={best_raw[1]:.3f} "
                    f"(등록 문턱 {self.detector.registration_confidence:.2f}), "
                    f"후보 {len(raw_scores)}개",
                    throttle_duration_sec=1.0)
            elif not detections and not raw_scores:
                self.get_logger().info(
                    f"[디버그] camera={self.active_cam_name} 신호 없음 "
                    f"(추론 문턱 {self.detector.inference_confidence:.2f} 이상 후보 0개)",
                    throttle_duration_sec=2.0)
        except (ValueError, RuntimeError) as error:
            self.get_logger().warn(str(error), throttle_duration_sec=5.0)
            return
        if detections:
            self.last_debug_detections = detections
        if self.publish_debug:
            self.debug_pub.publish(rgb_to_compressed_image(self.detector.draw(rgb, self.last_debug_detections), message))
        self.detected_pub.publish(Bool(data=bool(detections)))
        if not detections:
            return
        best = max(detections, key=lambda detection: detection.confidence)
        payload = {
            "camera": self.active_cam_name,
            "stamp": {"sec": int(message.header.stamp.sec), "nanosec": int(message.header.stamp.nanosec)},
            "class": best.class_name,
            "confidence": round(float(best.confidence), 4),
            "area_px": int(best.area_px),
            "center_pixel": [round(float(value), 1) for value in best.center_pixel],
            "detections_count": len(detections),
        }
        self.json_pub.publish(String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":"))))
        self.get_logger().info(f"결함 검출: camera={self.active_cam_name}, class={best.class_name}, confidence={best.confidence:.3f}, n={len(detections)}")


def main(args=None):
    """ROS 2를 초기화하고 ActiveCamVisionNode를 종료할 때까지 실행한다."""
    rclpy.init(args=args)
    node = None
    try:
        node = ActiveCamVisionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
