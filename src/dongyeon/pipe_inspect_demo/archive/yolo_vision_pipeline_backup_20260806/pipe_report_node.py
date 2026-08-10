"""ROS 결함 JSON을 실행별 원본 이력과 결함별 요약 파일로 저장한다."""

import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pipe_inspect_demo.defect_report_store import DefectReportStore

DEFAULT_OUTPUT_ROOT = Path.home() / "cobot3_ws/rokey_cobot/src/dongyeon/pipe_inspect_demo/output"


class PipeReportNode(Node):
    """결함 보고 토픽을 구독해 임무 단위 검사 기록을 디스크에 보존한다."""

    def __init__(self):
        """출력 경로와 토픽 파라미터를 읽고 저장소와 구독자를 생성한다."""
        super().__init__("pipe_report")
        self.declare_parameter("report_topic", "/defect/report_json")
        self.declare_parameter("aligned_report_topic", "/defect/aligned_report_json")
        self.declare_parameter("output_root", str(DEFAULT_OUTPUT_ROOT))
        report_topic = self.get_parameter("report_topic").value
        aligned_report_topic = self.get_parameter("aligned_report_topic").value
        output_root = self.get_parameter("output_root").value
        self.store = DefectReportStore(output_root)
        self.aligned_defect_ids = set()
        self.subscription = self.create_subscription(String, report_topic, self.on_report, 10)
        self.aligned_subscription = self.create_subscription(String, aligned_report_topic, self.on_report, 10)
        self.get_logger().info(f"결함 보고 저장 시작: topics=({report_topic}, {aligned_report_topic}), folder={self.store.mission_dir}")

    def on_report(self, message):
        """수신한 문자열을 JSON 객체로 검증한 뒤 저장하고 처리 결과를 알린다."""
        try:
            report = json.loads(message.data)
            if not isinstance(report, dict):
                raise ValueError("최상위 JSON 형식이 객체가 아니야.")
            defect_id = report["defect_id"]
            registration = report.get("registration", "unknown")
            if defect_id in self.aligned_defect_ids and registration != "aligned":
                return
            self.store.add_event(report)
            if registration == "aligned":
                self.aligned_defect_ids.add(defect_id)
            self.get_logger().info(
                f"결함 보고 저장: id={report['defect_id']}, registration={report.get('registration', 'unknown')}, events={self.store.event_count}"
            )
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as error:
            self.get_logger().warning(f"결함 보고를 저장하지 못했어: {error}")

    def close_report(self):
        """노드 종료 전에 최종 임무 요약을 완료 상태로 기록한다."""
        self.store.close("completed")


def main(args=None):
    """ROS를 초기화하고 결함 보고 저장 노드를 종료 요청까지 실행한다."""
    rclpy.init(args=args)
    node = PipeReportNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
