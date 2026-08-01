"""Isaac Sim 이 발행하는 /rgb, /depth, /camera_info 를 수신·검증하는 노드.

cv_bridge 없이 sensor_msgs/Image 를 numpy 로 직접 해석한다.
1초마다 토픽별 수신율(Hz)과 데이터 통계를 출력한다:
  /rgb    : 해상도, 평균 밝기 (0 에 가까우면 조명 문제 — CLAUDE.md C2)
  /depth  : 해상도, 유효 픽셀 비율, min/중앙값 (m)

프레임이 안 들어올 때 점검 순서:
  1. Isaac Sim 이 GUI 로 떠 있는가?  (E2: headless 는 토픽만 있고 프레임이 없다)
  2. Play 상태인가? (OnPlaybackTick 은 재생 중에만 돈다)
  3. 양쪽 ROS_DOMAIN_ID 가 같은가? (.bashrc 기본 143)

실행:
  ros2 run pipe_inspect camera_check
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


def _img_to_np(msg: Image):
    """sensor_msgs/Image -> (numpy 배열, 설명문자열). 미지원 인코딩은 (None, enc)."""
    h, w = msg.height, msg.width
    enc = msg.encoding
    if enc in ("rgb8", "bgr8"):
        return np.frombuffer(msg.data, np.uint8).reshape(h, w, 3), enc
    if enc == "rgba8":
        return np.frombuffer(msg.data, np.uint8).reshape(h, w, 4)[:, :, :3], enc
    if enc == "32FC1":
        return np.frombuffer(msg.data, np.float32).reshape(h, w), enc
    if enc == "16UC1":
        return np.frombuffer(msg.data, np.uint16).reshape(h, w), enc
    return None, enc


class CameraCheck(Node):
    def __init__(self):
        super().__init__("camera_check")
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.rgb_n = 0
        self.depth_n = 0
        self.rgb_last = None      # (배열, 인코딩)
        self.depth_last = None
        self.info_last = None
        self.create_subscription(Image, "rgb", self._on_rgb, qos)
        self.create_subscription(Image, "depth", self._on_depth, qos)
        self.create_subscription(CameraInfo, "camera_info", self._on_info, qos)
        self.create_timer(1.0, self._report)
        self.get_logger().info("구독 시작: /rgb /depth /camera_info (1초마다 통계 출력)")

    def _on_rgb(self, msg):
        self.rgb_n += 1
        self.rgb_last = _img_to_np(msg)

    def _on_depth(self, msg):
        self.depth_n += 1
        self.depth_last = _img_to_np(msg)

    def _on_info(self, msg):
        self.info_last = msg

    def _report(self):
        lines = [f"rgb {self.rgb_n:3d} Hz | depth {self.depth_n:3d} Hz"]

        if self.rgb_last is not None:
            arr, enc = self.rgb_last
            if arr is not None:
                mean = float(arr.mean())
                warn = "  [경고] 평균 밝기 ~0 — 조명(C2) 확인" if mean < 5.0 else ""
                lines.append(f"  rgb   {arr.shape[1]}x{arr.shape[0]} {enc} "
                             f"평균밝기 {mean:.1f}/255{warn}")
            else:
                lines.append(f"  rgb   미지원 인코딩: {enc}")

        if self.depth_last is not None:
            arr, enc = self.depth_last
            if arr is not None:
                d = arr.astype(np.float32)
                if enc == "16UC1":
                    d = d / 1000.0                     # mm -> m 관례
                finite = d[np.isfinite(d) & (d > 0)]
                ratio = 100.0 * finite.size / d.size if d.size else 0.0
                if finite.size:
                    lines.append(
                        f"  depth {arr.shape[1]}x{arr.shape[0]} {enc} "
                        f"유효 {ratio:.0f}%  min {finite.min():.4f} m  "
                        f"중앙 {np.median(finite):.4f} m")
                else:
                    lines.append(f"  depth {enc} — 유효 픽셀 없음")
            else:
                lines.append(f"  depth 미지원 인코딩: {enc}")

        if self.info_last is not None:
            k = self.info_last.k
            lines.append(f"  info  fx {k[0]:.1f} fy {k[4]:.1f} "
                         f"cx {k[2]:.1f} cy {k[5]:.1f}")

        if self.rgb_n == 0 and self.depth_n == 0:
            lines.append("  프레임 0 — Isaac 이 GUI+Play 인지, ROS_DOMAIN_ID 가 같은지 확인")
        self.get_logger().info(" / ".join(lines) if len(lines) == 1
                               else "\n".join(lines))
        self.rgb_n = 0
        self.depth_n = 0


def main(args=None):
    rclpy.init(args=args)
    node = CameraCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
