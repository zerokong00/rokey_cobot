"""카메라 수신 검증 — Isaac 이 보낸 RGB·Depth·CameraInfo 가 쓸 만한지 본다.

1초마다 토픽별 수신율(Hz)과 통계를 찍는다. 결함 검출을 붙이기 **전에** 이걸로
그림이 오는지부터 가린다 — YOLO 가 아무것도 못 찾을 때 "모델이 나쁜 건지
영상이 검은 건지" 를 여기서 먼저 갈라야 한다.

  rgb    해상도, 평균 밝기.  5/255 아래면 조명 문제다(관 내부는 로봇 조명이
         유일한 광원이라 intensity 를 안 올리면 통째로 검다)
  depth  유효 픽셀 비율, min/중앙값.  유효율이 낮으면 near clip 이 관벽을
         자르고 있거나 카메라가 본체 안에 파묻혀 있다
  info   fx/cx/cy.  어안이라 fx 는 등거리 모델의 f (r = f·theta) 다

프레임이 0 일 때 보는 순서:
  1. Isaac 이 **GUI** 로 떠 있는가 — headless 는 토픽만 있고 프레임이 0 이다
  2. **Play** 상태인가
  3. 양쪽 `ROS_DOMAIN_ID` 가 같은가 (우리 팀 143)
  4. 구독 QoS 가 BEST_EFFORT 인가 — 이 노드는 `contract.sensor_qos()` 를 쓴다

실행:
  ros2 run pipe_comm camera_monitor
  ros2 run pipe_comm camera_monitor --ros-args -p ns:=elbow_v
  ros2 run pipe_comm camera_monitor --ros-args -p ns:=tee -p save_dir:=/tmp/shots
"""

import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from pipe_comm import contract
from pipe_comm.contract import Topics
from pipe_comm import image_codec as codec

# 이 밝기 아래면 조명이 안 켜진 것으로 본다.
DARK_MEAN = 5.0


class CameraMonitor(Node):

    def __init__(self):
        super().__init__("camera_monitor")
        self.declare_parameter("ns", contract.DEFAULT_NS)
        self.declare_parameter("period_sec", 1.0)
        # 비워 두면 저장하지 않는다. 경로를 주면 주기마다 최신 프레임을 굽는다.
        self.declare_parameter("save_dir", "")

        ns = str(self.get_parameter("ns").value)
        self.t = Topics(ns)
        self.save_dir = str(self.get_parameter("save_dir").value).strip()
        if self.save_dir:
            Path(self.save_dir).mkdir(parents=True, exist_ok=True)

        qos = contract.sensor_qos()
        self.n = {"rgb": 0, "depth": 0, "depth_raw": 0, "rear_rgb": 0}
        self.last = {}
        self.info = None
        self.t0 = time.time()

        self.create_subscription(CompressedImage, self.t.rgb,
                                 lambda m: self._take("rgb", m), qos)
        self.create_subscription(CompressedImage, self.t.depth,
                                 lambda m: self._take("depth", m), qos)
        # 32FC1 원본도 같이 본다 — 압축이 깊이를 얼마나 깎았는지 대조용이다.
        self.create_subscription(Image, self.t.depth_raw,
                                 lambda m: self._take("depth_raw", m), qos)
        self.create_subscription(CompressedImage, self.t.rear_rgb,
                                 lambda m: self._take("rear_rgb", m), qos)
        self.create_subscription(CameraInfo, self.t.camera_info,
                                 self._on_info, qos)

        self.create_timer(float(self.get_parameter("period_sec").value),
                          self._report)
        self.get_logger().info(
            f"구독 시작 [{ns}] — {self.t.rgb} / {self.t.depth} / "
            f"{self.t.camera_info}  (BEST_EFFORT, domain "
            f"{contract.ROS_DOMAIN_ID})")

    def _take(self, key, msg):
        self.n[key] += 1
        self.last[key] = msg

    def _on_info(self, msg):
        self.info = msg

    # ── 통계 ────────────────────────────────────────────────────
    def _rgb_line(self, key, msg):
        try:
            a = codec.jpeg_to_rgb(msg)
        except ValueError as exc:
            return f"  {key:9s} 디코딩 실패 — {exc}"
        mean = float(a.mean())
        warn = "   ⚠ 조명 확인 (관 내부는 로봇 조명이 유일한 광원)" \
            if mean < DARK_MEAN else ""
        return (f"  {key:9s} {a.shape[1]}x{a.shape[0]} jpeg "
                f"{len(msg.data) / 1024:.0f}KB  평균밝기 {mean:5.1f}/255{warn}")

    def _depth_line(self, key, msg, decode):
        try:
            d = decode(msg)
        except ValueError as exc:
            return f"  {key:9s} 디코딩 실패 — {exc}"
        fin = d[np.isfinite(d)]
        ratio = 100.0 * fin.size / d.size if d.size else 0.0
        if not fin.size:
            return f"  {key:9s} {d.shape[1]}x{d.shape[0]}  유효 픽셀 없음  ⚠"
        warn = "   ⚠ near clip / 카메라 매몰 확인" if ratio < 20.0 else ""
        return (f"  {key:9s} {d.shape[1]}x{d.shape[0]}  유효 {ratio:3.0f}%  "
                f"min {fin.min():.4f}  중앙 {float(np.median(fin)):.4f}  "
                f"max {fin.max():.4f} m{warn}")

    def _report(self):
        lines = ["  " + "  ".join(f"{k} {v}Hz" for k, v in self.n.items())]

        if "rgb" in self.last:
            lines.append(self._rgb_line("rgb", self.last["rgb"]))
        if "rear_rgb" in self.last:
            lines.append(self._rgb_line("rear_rgb", self.last["rear_rgb"]))
        if "depth" in self.last:
            lines.append(self._depth_line("depth", self.last["depth"],
                                          codec.png16_to_depth_m))
        if "depth_raw" in self.last:
            lines.append(self._depth_line("depth32", self.last["depth_raw"],
                                          codec.image_to_depth_m))
        if self.info is not None:
            k = self.info.k
            lines.append(f"  info      {self.info.width}x{self.info.height}  "
                         f"fx {k[0]:.1f}  cx {k[2]:.1f}  cy {k[5]:.1f}  "
                         f"모델 {self.info.distortion_model or '(없음)'}")

        if not any(self.n.values()):
            el = time.time() - self.t0
            lines.append(f"  프레임 0 ({el:.0f}초째) — ① Isaac 이 GUI 인가 "
                         f"② Play 인가 ③ ROS_DOMAIN_ID 가 "
                         f"{contract.ROS_DOMAIN_ID} 인가 ④ ns 파라미터가 "
                         f"맞는가 (지금 {self.t.ns!r})")
        else:
            self._save()

        self.get_logger().info("\n".join(lines))
        for k in self.n:
            self.n[k] = 0

    def _save(self):
        """최신 프레임을 파일로 굽는다. 눈으로 봐야 할 때만 쓴다."""
        if not self.save_dir:
            return
        import cv2
        stamp = time.strftime("%H%M%S")
        if "rgb" in self.last:
            a = codec.jpeg_to_rgb(self.last["rgb"])
            cv2.imwrite(f"{self.save_dir}/rgb_{stamp}.png", a[:, :, ::-1])
        if "depth" in self.last:
            d = codec.png16_to_depth_m(self.last["depth"])
            fin = d[np.isfinite(d)]
            if fin.size:
                # 보이라고 정규화하는 것이다 — 이 png 로 거리를 재면 안 된다.
                v = np.nan_to_num((d - fin.min()) /
                                  max(fin.max() - fin.min(), 1e-6))
                cv2.imwrite(f"{self.save_dir}/depth_{stamp}.png",
                            (v * 255).astype(np.uint8))


def main(args=None):
    rclpy.init(args=args)
    node = CameraMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
