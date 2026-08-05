"""ROS Image 메시지를 RGB와 미터 단위 Depth 배열로 변환한다."""

import cv2
import numpy as np


def image_to_rgb(msg):
    """패딩을 포함한 ROS Image를 RGB uint8 배열로 변환한다."""
    encoding = msg.encoding.lower()
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(encoding)
    if channels is None:
        raise ValueError(f"지원하지 않는 RGB encoding이야: {msg.encoding}")
    row_bytes = int(msg.width) * channels
    if msg.height <= 0 or msg.width <= 0 or msg.step < row_bytes:
        raise ValueError("RGB Image 크기 또는 step이 잘못됐어.")
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    expected = int(msg.height) * int(msg.step)
    if raw.size < expected:
        raise ValueError(f"RGB Image 데이터가 부족해: {raw.size} < {expected}")
    image = raw[:expected].reshape(msg.height, msg.step)[:, :row_bytes].reshape(msg.height, msg.width, channels)
    if encoding == "rgb8":
        return image.copy()
    if encoding == "bgr8":
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    if encoding == "rgba8":
        return cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)
    if encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2RGB)
    return cv2.cvtColor(image[:, :, 0], cv2.COLOR_GRAY2RGB)


def image_to_depth_m(msg):
    """ROS Depth Image를 잘못된 값을 NaN으로 표시한 미터 배열로 변환한다."""
    encoding = msg.encoding.lower()
    dtype = {"32fc1": np.dtype(np.float32), "16uc1": np.dtype(np.uint16)}.get(encoding)
    if dtype is None:
        raise ValueError(f"지원하지 않는 Depth encoding이야: {msg.encoding}")
    row_bytes = int(msg.width) * dtype.itemsize
    if msg.height <= 0 or msg.width <= 0 or msg.step < row_bytes:
        raise ValueError("Depth Image 크기 또는 step이 잘못됐어.")
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    expected = int(msg.height) * int(msg.step)
    if raw.size < expected:
        raise ValueError(f"Depth Image 데이터가 부족해: {raw.size} < {expected}")
    packed = np.ascontiguousarray(raw[:expected].reshape(msg.height, msg.step)[:, :row_bytes])
    source_dtype = dtype.newbyteorder(">" if bool(msg.is_bigendian) else "<")
    depth = packed.view(source_dtype).reshape(msg.height, msg.width).astype(np.float32)
    if encoding == "16uc1":
        depth *= 0.001
    depth[~np.isfinite(depth) | (depth <= 0.0)] = np.nan
    return depth


def compressed_to_rgb(msg):
    """JPEG CompressedImage를 YOLO 입력용 RGB uint8 배열로 복원한다."""
    encoded = np.frombuffer(msg.data, dtype=np.uint8)
    bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError("압축 RGB 영상을 디코딩할 수 없어.")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def compressed_to_depth_m(msg):
    """16비트 PNG CompressedImage를 미터 단위 Depth 배열로 복원한다."""
    encoded = np.frombuffer(msg.data, dtype=np.uint8)
    depth_mm = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
    if depth_mm is None or depth_mm.dtype != np.uint16 or depth_mm.ndim != 2:
        raise ValueError("압축 Depth 영상은 16UC1 PNG 형식이어야 해.")
    depth = depth_mm.astype(np.float32) * 0.001
    depth[depth_mm == 0] = np.nan
    return depth


def rgb_to_image(rgb, source_msg):
    """RGB 배열을 원본 header를 유지한 sensor_msgs/Image로 변환한다."""
    from sensor_msgs.msg import Image
    image = np.ascontiguousarray(rgb, dtype=np.uint8)
    msg = Image()
    msg.header = source_msg.header
    msg.height, msg.width = image.shape[:2]
    msg.encoding = "rgb8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = image.tobytes()
    return msg


def rgb_to_compressed_image(rgb, source_msg, quality=85):
    """RGB 배열을 원본 header를 유지한 JPEG CompressedImage로 변환한다."""
    from sensor_msgs.msg import CompressedImage
    bgr = cv2.cvtColor(np.ascontiguousarray(rgb, dtype=np.uint8), cv2.COLOR_RGB2BGR)
    success, encoded = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if not success:
        raise RuntimeError("디버그 RGB 영상 압축에 실패했어.")
    msg = CompressedImage()
    msg.header = source_msg.header
    msg.format = "jpeg; bgr8"
    msg.data = encoded.tobytes()
    return msg
