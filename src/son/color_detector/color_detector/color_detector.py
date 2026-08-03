#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

# color_id 정의
BLUE = 1
GREEN = 2
NONE = 0

# HSV 색상 범위
COLOR_RANGES = {
    BLUE: [(100, 50, 50), (130, 255, 255)],   # 파란색
    GREEN: [(40, 50, 50), (80, 255, 255)]     # 초록색
}

# 검출로 인정할 최소 픽셀 수 (노이즈 제거)
MIN_PIXELS = 500


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        # Subscriber 설정 - Isaac Sim(PC A) 의 Wrist Camera 영상
        self.rgb_sub = self.create_subscription(Image, '/rgb', self.rgb_callback, 10)

        # Publisher 설정 - 감지한 색상 ID 를 Isaac Sim(PC A) 으로 송신
        self.color_id_pub = self.create_publisher(Int32, '/color_id', 10)

        self.bridge = CvBridge()
        self.last_color_id = NONE

    def rgb_callback(self, msg):
        # RGB 이미지 수신
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # 색상 검출 처리
        color_id = self.detect_color(image)

        if color_id != NONE:
            # 결과 송신
            self.color_id_pub.publish(Int32(data=color_id))

        # 검출 결과가 바뀔 때만 로그 출력
        if color_id != self.last_color_id:
            name = {BLUE: '파랑', GREEN: '초록', NONE: '없음'}[color_id]
            self.get_logger().info(f'detected: {name} (color_id={color_id})')
            self.last_color_id = color_id

    def detect_color(self, image):
        # 파랑/초록 각각의 마스크 픽셀 수를 세어 더 많은 쪽을 선택
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        counts = {}
        for color_id, (lower, upper) in COLOR_RANGES.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            counts[color_id] = int(cv2.countNonZero(mask))

        best_id = max(counts, key=counts.get)

        # 최소 픽셀 수를 넘지 못하면 미검출
        if counts[best_id] < MIN_PIXELS:
            return NONE

        return best_id


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
