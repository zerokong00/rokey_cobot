"""동기화된 RGB·Depth로 결함을 검출하고 입구 기준 위치 JSON을 발행한다."""

import json
from pathlib import Path

import message_filters
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, CompressedImage, Imu
from std_msgs.msg import Bool, String

from pipe_inspect_demo.defect_position_tracker import DefectPositionTracker
from pipe_inspect_demo.ros_image_utils import compressed_to_depth_m, compressed_to_rgb, rgb_to_compressed_image
from pipe_inspect_demo.yolo_seg_detector import YoloSegDetector

PACKAGE_NAME = "pipe_inspect_demo"
DEFAULT_MODEL_NAME = "yolov8n_seg_best.pt"


def resolve_model_path(parameter_value):
    """명시적 경로 또는 설치·소스 resource의 기본 YOLO Seg 모델을 찾는다."""
    explicit_path = Path(str(parameter_value)).expanduser()
    if str(parameter_value).strip():
        if not explicit_path.is_file():
            raise FileNotFoundError(f"지정한 YOLO Seg 모델을 찾을 수 없어: {explicit_path}")
        return explicit_path.resolve()
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(Path(get_package_share_directory(PACKAGE_NAME)) / "resource" / DEFAULT_MODEL_NAME)
    except (ImportError, LookupError):
        pass
    candidates.append(Path(__file__).resolve().parent.parent / "resource" / DEFAULT_MODEL_NAME)
    model_path = next((path for path in candidates if path.is_file()), None)
    if model_path is None:
        raise FileNotFoundError("기본 YOLO Seg 모델을 찾을 수 없어:\n" + "\n".join(f"- {path}" for path in candidates))
    return model_path.resolve()


class PipeVisionNode(Node):
    """RGB·Depth·CameraInfo·Odom·IMU를 결함 위치 JSON 발행으로 연결한다."""

    def __init__(self):
        """ROS parameter, 구독·발행기, YOLO Seg 모델과 위치 추적기를 초기화한다."""
        super().__init__("pipe_vision")
        defaults = {"rgb_topic": "/rgb/compressed", "depth_topic": "/depth/compressed", "camera_info_topic": "/camera_info", "odom_topic": "/odom", "imu_topic": "/imu", "reset_topic": "/inspection/reset", "alignment_complete_topic": "/inspection/alignment_complete", "json_topic": "/defect/report_json", "alignment_observation_topic": "/defect/alignment_observation_json", "detected_topic": "/defect/detected", "target_pixel_topic": "/defect/target_pixel", "debug_topic": "/defect/debug_image/compressed", "model_path": "", "inference_confidence": 0.05, "registration_confidence": 0.8, "device": "", "sync_queue_size": 5, "sync_slop_sec": 0.05, "publish_debug": True, "pipe_axial_min_m": 0.0, "pipe_axial_max_m": 1.3}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        model_path = resolve_model_path(self.get_parameter("model_path").value)
        self.detector = YoloSegDetector(model_path, self.get_parameter("inference_confidence").value, self.get_parameter("registration_confidence").value, str(self.get_parameter("device").value))
        self.tracker = DefectPositionTracker()
        self.intrinsics = None
        self.sensor_status = {"rgb": False, "depth": False, "camera_info": False, "odom": False, "imu": False, "capture_synchronized": False}
        self.publish_debug = bool(self.get_parameter("publish_debug").value)
        self.pipe_axial_min_m = float(self.get_parameter("pipe_axial_min_m").value)
        self.pipe_axial_max_m = float(self.get_parameter("pipe_axial_max_m").value)
        if self.pipe_axial_min_m >= self.pipe_axial_max_m:
            raise ValueError("파이프 유효 축 범위의 최댓값은 최솟값보다 커야 해.")
        self.json_pub = self.create_publisher(String, str(self.get_parameter("json_topic").value), 10)
        self.alignment_observation_pub = self.create_publisher(String, str(self.get_parameter("alignment_observation_topic").value), 10)
        self.detected_pub = self.create_publisher(Bool, str(self.get_parameter("detected_topic").value), 10)
        self.target_pixel_pub = self.create_publisher(PointStamped, str(self.get_parameter("target_pixel_topic").value), 10)
        self.debug_pub = self.create_publisher(CompressedImage, str(self.get_parameter("debug_topic").value), qos_profile_sensor_data)
        self.reset_sub = self.create_subscription(Bool, str(self.get_parameter("reset_topic").value), self._on_reset, 10)
        self.alignment_complete_sub = self.create_subscription(Bool, str(self.get_parameter("alignment_complete_topic").value), self._on_alignment_complete, 10)
        self.rgb_sub = message_filters.Subscriber(self, CompressedImage, str(self.get_parameter("rgb_topic").value), qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(self, CompressedImage, str(self.get_parameter("depth_topic").value), qos_profile=qos_profile_sensor_data)
        self.camera_info_sub = message_filters.Subscriber(self, CameraInfo, str(self.get_parameter("camera_info_topic").value), qos_profile=qos_profile_sensor_data)
        self.odom_sub = message_filters.Subscriber(self, Odometry, str(self.get_parameter("odom_topic").value), qos_profile=qos_profile_sensor_data)
        self.imu_sub = message_filters.Subscriber(self, Imu, str(self.get_parameter("imu_topic").value), qos_profile=qos_profile_sensor_data)
        self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.camera_info_sub, self.odom_sub, self.imu_sub], int(self.get_parameter("sync_queue_size").value), float(self.get_parameter("sync_slop_sec").value))
        self.synchronizer.registerCallback(self._on_sensor_bundle)
        self._missing_position_warned = False
        self.report_frozen = False
        self.get_logger().info(f"파이프 YOLO Seg 노드 시작: model={model_path}, classes={self.detector.class_names}")

    def _on_reset(self, message):
        """Isaac Sim의 새 검사 임무 신호로 위치·결함·측정 추적 상태를 초기화한다."""
        if not message.data:
            return
        self.tracker.reset()
        self.report_frozen = False
        self.sensor_status = {key: False for key in self.sensor_status}
        self.get_logger().info("검사 추적 상태 초기화 완료")

    def _on_alignment_complete(self, message):
        """정렬 완료 동안 일반 검출·JSON 발행을 잠그고 재출발 시 해제한다."""
        self.report_frozen = bool(message.data)
        self.get_logger().info(f"결함 보고 발행 {'정지' if self.report_frozen else '재개'}")

    def _on_sensor_bundle(self, rgb_msg, depth_msg, camera_info_msg, odom_msg, imu_msg):
        """같은 촬영 시점의 영상·카메라·자세를 추론하고 결함 JSON으로 발행한다."""
        try:
            self.intrinsics = (camera_info_msg.k[0], camera_info_msg.k[4], camera_info_msg.k[2], camera_info_msg.k[5])
            point = odom_msg.pose.pose.position
            self.tracker.update_odometry((point.x, point.y, point.z))
            orientation = imu_msg.orientation
            self.tracker.update_imu((orientation.x, orientation.y, orientation.z, orientation.w))
            rgb = compressed_to_rgb(rgb_msg)
            depth = compressed_to_depth_m(depth_msg)
            self.sensor_status.update({"rgb": True, "depth": True, "camera_info": True, "odom": True, "imu": True, "capture_synchronized": True})
            detections = self.detector.infer(rgb, depth, self.intrinsics)
        except (ValueError, RuntimeError) as error:
            self.get_logger().warn(str(error), throttle_duration_sec=5.0)
            return
        if self.publish_debug:
            self.debug_pub.publish(rgb_to_compressed_image(self.detector.draw(rgb, detections), rgb_msg))
        if self.report_frozen:
            return
        if not detections:
            self.detected_pub.publish(Bool(data=False))
            return
        stamp = {"sec": int(rgb_msg.header.stamp.sec), "nanosec": int(rgb_msg.header.stamp.nanosec)}
        valid_detection_seen = False
        best_target = None
        for detection in detections:
            record = self.tracker.build_record(detection, self.intrinsics, stamp, rgb_msg.header.frame_id)
            axial_position = float(record["defect"]["axial_position_from_entry_m"])
            if not self.pipe_axial_min_m <= axial_position <= self.pipe_axial_max_m:
                self.get_logger().warn(f"파이프 범위 밖 검출 제외: axial={axial_position:.3f}m", throttle_duration_sec=5.0)
                continue
            valid_detection_seen = True
            record["sensor_status"] = self.sensor_status.copy()
            registered, status = self.tracker.register(record)
            alignment_record = self.tracker.attach_registered_identity(record)
            if alignment_record is not None and (best_target is None or float(alignment_record["confidence"]) > float(best_target["confidence"])):
                best_target = alignment_record
            if registered is None:
                continue
            registered["registration"] = status
            payload = json.dumps(registered, ensure_ascii=False, separators=(",", ":"))
            self.json_pub.publish(String(data=payload))
            self.get_logger().info(f"결함 JSON 발행: status={status}, id={registered['defect_id']}, class={registered['class']}, confidence={registered['confidence']:.3f}, axial={registered['defect']['axial_position_from_entry_m']:.3f}m")
        self.detected_pub.publish(Bool(data=valid_detection_seen))
        if best_target is not None:
            center = best_target["segmentation"]["center_pixel"]
            best_target["registration"] = "alignment_observation"
            self.alignment_observation_pub.publish(String(data=json.dumps(best_target, ensure_ascii=False, separators=(",", ":"))))
            target_message = PointStamped()
            target_message.header = rgb_msg.header
            target_message.point.x, target_message.point.y, target_message.point.z = float(center[0]), float(center[1]), float(best_target["confidence"])
            self.target_pixel_pub.publish(target_message)


def main(args=None):
    """ROS 2를 초기화하고 PipeVisionNode를 종료할 때까지 실행한다."""
    rclpy.init(args=args)
    node = None
    try:
        node = PipeVisionNode()
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
