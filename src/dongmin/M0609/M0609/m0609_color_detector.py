#!/usr/bin/env python3
# ══════════════════════════════════════════════════════════════
#  m0609_color_detector  (PC B / 일반 노트북)
#
#   Isaac Sim(PC A)의 Wrist Camera 영상 /rgb 를 구독하고
#   HSV 색상 검출로 파랑/초록 큐브를 판별하여 /color_id 발행
#
#     /rgb       sensor_msgs/Image   (구독)
#     /color_id  std_msgs/Int32      (발행)  0=없음 / 1=파랑 / 2=초록
#
#   ※ PC A 와 ROS_DOMAIN_ID 가 같아야 통신됩니다 (예: 50)
# ══════════════════════════════════════════════════════════════

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

# ── 색상 ID ───────────────────────────────────────────────────
COLOR_NONE, COLOR_BLUE, COLOR_GREEN = 0, 1, 2
COLOR_NAME = {COLOR_NONE: "없음", COLOR_BLUE: "파랑", COLOR_GREEN: "초록"}

# ── HSV 임계값 (OpenCV: H 0~179, S 0~255, V 0~255) ────────────
HSV_RANGES = {
    COLOR_BLUE:  [(np.array([100,  90,  60]), np.array([130, 255, 255]))],
    COLOR_GREEN: [(np.array([ 40,  80,  50]), np.array([ 85, 255, 255]))],
}

# ── 판정 파라미터 ─────────────────────────────────────────────
#   손목 카메라에는 대상 큐브뿐 아니라 Place 마커(파랑/초록)까지 잡힌다.
#   → 화면 중앙 ROI 만 보고, 1·2위 픽셀 수 차이가 뚜렷할 때만 판정한다.
MIN_PIXELS   = 250      # 이 픽셀 수 미만이면 무시 (노이즈)
ROI_RATIO    = 0.4      # 화면 중앙 사용 비율 (1.0 = 전체 화면)
MIN_DOMINANCE = 1.6     # 1위가 2위의 몇 배 이상이어야 인정


class ColorDetector(Node):

    def __init__(self):
        super().__init__("m0609_color_detector")

        # ── 파라미터 ─────────────────────────────────────────
        self.declare_parameter("image_topic", "/rgb")
        self.declare_parameter("color_topic", "/color_id")
        self.declare_parameter("min_pixels", MIN_PIXELS)
        self.declare_parameter("roi_ratio", ROI_RATIO)
        self.declare_parameter("min_dominance", MIN_DOMINANCE)
        self.declare_parameter("show_window", False)

        image_topic = self.get_parameter("image_topic").value
        color_topic = self.get_parameter("color_topic").value
        self._min_pixels = int(self.get_parameter("min_pixels").value)
        self._roi_ratio = float(self.get_parameter("roi_ratio").value)
        self._min_dominance = float(self.get_parameter("min_dominance").value)
        self._show = bool(self.get_parameter("show_window").value)

        # ── 통신 (센서 영상은 BEST_EFFORT) ───────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, image_topic, self.on_image, sensor_qos)
        self._pub = self.create_publisher(Int32, color_topic, 10)

        self._last_id = None
        self._frames = 0

        self.get_logger().info("=" * 55)
        self.get_logger().info("  M0609 색상 감지 노드 시작")
        self.get_logger().info(f"   구독 : {image_topic}  (sensor_msgs/Image)")
        self.get_logger().info(f"   발행 : {color_topic}  (std_msgs/Int32)")
        self.get_logger().info("   판정 : 1=파랑 / 2=초록 / 0=없음")
        self.get_logger().info("=" * 55)

    # ── Image → numpy (cv_bridge 없이 직접 변환) ─────────────
    @staticmethod
    def to_bgr(msg: Image):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        enc = msg.encoding.lower()

        if enc in ("rgb8", "bgr8"):
            ch = 3
        elif enc in ("rgba8", "bgra8"):
            ch = 4
        elif enc == "mono8":
            ch = 1
        else:
            raise ValueError(f"지원하지 않는 encoding: {msg.encoding}")

        img = buf.reshape(msg.height, msg.step // ch, ch)[:, : msg.width, :]

        if enc == "rgb8":
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if enc == "rgba8":
            return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        if enc == "bgra8":
            return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        if enc == "mono8":
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        return img.copy()

    # ── ROI 자르기 (화면 중앙) ───────────────────────────────
    def crop_roi(self, bgr):
        if self._roi_ratio >= 0.999:
            return bgr
        h, w = bgr.shape[:2]
        rh, rw = int(h * self._roi_ratio), int(w * self._roi_ratio)
        y0, x0 = (h - rh) // 2, (w - rw) // 2
        return bgr[y0:y0 + rh, x0:x0 + rw]

    # ── HSV 마스크 픽셀 수 계산 ──────────────────────────────
    @staticmethod
    def mask_of(hsv, color_id):
        mask = None
        for lo, hi in HSV_RANGES[color_id]:
            m = cv2.inRange(hsv, lo, hi)
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    # ── 콜백 : 감지 후 발행 ──────────────────────────────────
    def on_image(self, msg: Image):
        try:
            bgr = self.to_bgr(msg)
        except ValueError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)
            return

        roi = self.crop_roi(bgr)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        counts = {}
        masks = {}
        for cid in (COLOR_BLUE, COLOR_GREEN):
            masks[cid] = self.mask_of(hsv, cid)
            counts[cid] = int(cv2.countNonZero(masks[cid]))

        # 픽셀이 가장 많은 색을 선택
        #   ① 최소 픽셀 수를 넘고
        #   ② 2위보다 min_dominance 배 이상 많아야 인정 (마커 오인 방지)
        best = max(counts, key=counts.get)
        other = min(counts, key=counts.get)
        color_id = COLOR_NONE
        if counts[best] >= self._min_pixels:
            if counts[other] == 0 or counts[best] >= counts[other] * self._min_dominance:
                color_id = best

        self._pub.publish(Int32(data=int(color_id)))

        # 값이 바뀔 때만 로그
        if color_id != self._last_id:
            self.get_logger().info(
                f"color_id = {color_id} ({COLOR_NAME[color_id]})  "
                f"blue={counts[COLOR_BLUE]}  green={counts[COLOR_GREEN]}"
            )
            self._last_id = color_id

        self._frames += 1
        if self._show:
            self.preview(roi, masks, color_id, counts)

    # ── 디버그 창 (show_window:=true 일 때만) ────────────────
    def preview(self, roi, masks, color_id, counts):
        view = roi.copy()
        cv2.putText(
            view,
            f"id={color_id} {COLOR_NAME[color_id]}  B:{counts[COLOR_BLUE]} G:{counts[COLOR_GREEN]}",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2,
        )
        merged = cv2.bitwise_or(masks[COLOR_BLUE], masks[COLOR_GREEN])
        cv2.imshow("rgb", view)
        cv2.imshow("mask", merged)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
