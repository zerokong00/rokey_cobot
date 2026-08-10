"""YOLO Seg 모델로 결함 mask와 mask 내부 Depth를 계산한다."""

import heapq
from dataclasses import dataclass

import cv2
import numpy as np

MIN_MEASUREMENT_POINTS = 20
MIN_VALID_DEPTH_RATIO = 0.5


@dataclass(frozen=True)
class PhysicalMeasurement:
    """Seg 마스크의 3차원 길이·폭 측정 결과와 품질을 보관한다."""

    length_mm: float | None
    width_mm: float | None
    physical_size_valid: bool
    method: str
    valid_depth_points: int
    valid_depth_ratio: float
    skeleton_points: int
    skeleton_components: int
    longest_component_points: int
    touches_image_border: bool
    quality: str


@dataclass(frozen=True)
class SegDetection:
    """한 결함의 클래스·신뢰도·형상·Depth 정보를 보관한다."""

    class_id: int
    class_name: str
    confidence: float
    bbox_xyxy: tuple
    center_pixel: tuple
    area_px: int
    depth_m: float
    polygon: np.ndarray
    mask: np.ndarray
    measurement: PhysicalMeasurement


def mask_depth_median(mask, depth):
    """결함 mask 내부의 유효 Depth 중앙값을 미터 단위로 반환한다."""
    if mask.shape != depth.shape:
        raise ValueError(f"mask와 Depth 크기가 달라: {mask.shape} != {depth.shape}")
    values = depth[mask & np.isfinite(depth) & (depth > 0.0)]
    return None if values.size == 0 else float(np.median(values))


def invalid_measurement(valid_points, valid_ratio, quality, skeleton_points=0, skeleton_components=0, longest_component_points=0, touches_image_border=False):
    """치수 산출이 불가능한 마스크의 명시적 무효 측정 결과를 만든다."""
    return PhysicalMeasurement(None, None, False, "mask_depth_3d_skeleton", int(valid_points), round(float(valid_ratio), 6), int(skeleton_points), int(skeleton_components), int(longest_component_points), bool(touches_image_border), quality)


def skeletonize_mask(mask):
    """형태학적 침식으로 마스크를 한 픽셀 중심선에 가깝게 축소한다."""
    rows, columns = np.nonzero(mask)
    skeleton = np.zeros(mask.shape, dtype=bool)
    if rows.size == 0:
        return skeleton
    top, bottom = max(int(rows.min()) - 1, 0), min(int(rows.max()) + 2, mask.shape[0])
    left, right = max(int(columns.min()) - 1, 0), min(int(columns.max()) + 2, mask.shape[1])
    image = mask[top:bottom, left:right].astype(np.uint8) * 255
    local_skeleton = np.zeros_like(image)
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    while cv2.countNonZero(image) > 0:
        opened = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
        local_skeleton = cv2.bitwise_or(local_skeleton, cv2.subtract(image, opened))
        image = cv2.erode(image, kernel)
    skeleton[top:bottom, left:right] = local_skeleton > 0
    return skeleton


def point_cloud_from_pixels(rows, columns, depth, intrinsics):
    """픽셀 좌표와 Depth를 카메라 광학 좌표의 3차원 점군으로 변환한다."""
    fx, fy, cx, cy = intrinsics
    values = depth[rows, columns].astype(np.float64)
    return np.column_stack(((columns - cx) * values / fx, (rows - cy) * values / fy, values))


def skeleton_longest_path_m(rows, columns, points):
    """8방향 Skeleton 그래프의 3차원 가중 지름과 연결 성분 통계를 반환한다."""
    pixel_to_index = {(int(row), int(column)): index for index, (row, column) in enumerate(zip(rows, columns))}
    graph = [[] for _ in range(len(points))]
    for (row, column), index in pixel_to_index.items():
        for row_offset, column_offset in ((-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1)):
            neighbor = pixel_to_index.get((row + row_offset, column + column_offset))
            if neighbor is not None:
                graph[index].append((neighbor, float(np.linalg.norm(points[index] - points[neighbor]))))

    def dijkstra(start, allowed):
        """한 연결 성분에서 시작점으로부터 최장 최단거리 노드를 찾는다."""
        distances = {start: 0.0}
        queue = [(0.0, start)]
        while queue:
            distance, node = heapq.heappop(queue)
            if distance != distances[node]:
                continue
            for neighbor, weight in graph[node]:
                if neighbor not in allowed:
                    continue
                candidate = distance + weight
                if candidate < distances.get(neighbor, float("inf")):
                    distances[neighbor] = candidate
                    heapq.heappush(queue, (candidate, neighbor))
        farthest = max(distances, key=distances.get)
        return farthest, distances[farthest]

    remaining = set(range(len(points)))
    longest = 0.0
    component_count = 0
    longest_component_points = 0
    while remaining:
        component_count += 1
        start = next(iter(remaining))
        component, stack = {start}, [start]
        while stack:
            node = stack.pop()
            for neighbor, _ in graph[node]:
                if neighbor not in component:
                    component.add(neighbor)
                    stack.append(neighbor)
        remaining -= component
        longest_component_points = max(longest_component_points, len(component))
        endpoint, _ = dijkstra(start, component)
        _, length = dijkstra(endpoint, component)
        longest = max(longest, length)
    return longest, component_count, longest_component_points


def measure_mask_size_3d(mask, depth, intrinsics):
    """마스크 Skeleton의 3차원 최장 경로와 국부 반경으로 길이·폭을 mm로 측정한다."""
    if mask.shape != depth.shape:
        raise ValueError(f"mask와 Depth 크기가 달라: {mask.shape} != {depth.shape}")
    fx, fy, cx, cy = (float(value) for value in intrinsics)
    if not np.all(np.isfinite((fx, fy, cx, cy))) or fx <= 0.0 or fy <= 0.0:
        raise ValueError("치수 측정용 카메라 내부 파라미터가 잘못됐어.")
    mask_area = int(np.count_nonzero(mask))
    touches_image_border = bool(np.any(mask[0]) or np.any(mask[-1]) or np.any(mask[:, 0]) or np.any(mask[:, -1]))
    if mask_area < MIN_MEASUREMENT_POINTS:
        return invalid_measurement(0, 0.0, "insufficient_mask_area", touches_image_border=touches_image_border)
    valid_mask = mask & np.isfinite(depth) & (depth > 0.0)
    rows, columns = np.nonzero(valid_mask)
    valid_count = int(rows.size)
    valid_ratio = valid_count / float(mask_area)
    if valid_count < MIN_MEASUREMENT_POINTS or valid_ratio < MIN_VALID_DEPTH_RATIO:
        return invalid_measurement(valid_count, valid_ratio, "insufficient_valid_depth", touches_image_border=touches_image_border)
    skeleton = skeletonize_mask(mask)
    skeleton_valid = skeleton & valid_mask
    skeleton_rows, skeleton_columns = np.nonzero(skeleton_valid)
    skeleton_count = int(skeleton_rows.size)
    if skeleton_count < 2:
        return invalid_measurement(valid_count, valid_ratio, "insufficient_skeleton_depth", skeleton_count, touches_image_border=touches_image_border)
    points = point_cloud_from_pixels(skeleton_rows, skeleton_columns, depth, (fx, fy, cx, cy))
    length_m, component_count, longest_component_points = skeleton_longest_path_m(skeleton_rows, skeleton_columns, points)
    distance_pixels = cv2.distanceTransform(mask.astype(np.uint8), cv2.DIST_L2, 5)[skeleton_rows, skeleton_columns]
    skeleton_depth = depth[skeleton_rows, skeleton_columns].astype(np.float64)
    pixel_scale_m = skeleton_depth * 0.5 * (1.0 / fx + 1.0 / fy)
    local_widths_m = np.maximum(2.0 * distance_pixels - 1.0, 1.0) * pixel_scale_m
    width_m = float(np.median(local_widths_m))
    if not np.all(np.isfinite((length_m, width_m))) or length_m <= 0.0 or width_m <= 0.0:
        return invalid_measurement(valid_count, valid_ratio, "degenerate_geometry", skeleton_count, component_count, longest_component_points, touches_image_border)
    quality = "partial_image_border" if touches_image_border else "fragmented_skeleton" if component_count > 1 else "valid"
    return PhysicalMeasurement(round(length_m * 1000.0, 3), round(width_m * 1000.0, 3), not touches_image_border, "mask_depth_3d_skeleton", valid_count, round(valid_ratio, 6), skeleton_count, component_count, longest_component_points, touches_image_border, quality)


class YoloSegDetector:
    """Ultralytics YOLO Seg 모델을 로딩하고 RGB·Depth 결함 추론을 수행한다."""

    def __init__(self, model_path, inference_confidence=0.05, registration_confidence=0.8, device=""):
        """모델 경로와 추론·등록 신뢰도 및 실행 장치를 초기화한다."""
        from ultralytics import YOLO
        self.model = YOLO(str(model_path))
        self.inference_confidence = float(inference_confidence)
        self.registration_confidence = float(registration_confidence)
        self.device = device or None
        if getattr(self.model, "task", None) != "segment":
            raise RuntimeError(f"YOLO Seg 모델이 아니야: task={getattr(self.model, 'task', None)}")

    @property
    def class_names(self):
        """모델의 클래스 ID와 이름 사전을 반환한다."""
        return self.model.names

    def infer(self, rgb, depth, intrinsics):
        """동기화된 RGB와 Depth에서 등록 신뢰도 이상 결함 목록을 반환한다."""
        if rgb.shape[:2] != depth.shape:
            raise ValueError(f"RGB와 Depth 해상도가 달라: {rgb.shape[:2]} != {depth.shape}")
        result = self.model.predict(source=rgb, conf=self.inference_confidence, device=self.device, verbose=False, save=False)[0]
        if result.boxes is None or result.masks is None:
            return []
        detections = []
        height, width = depth.shape
        polygons = result.masks.xy
        for index, box in enumerate(result.boxes):
            confidence = float(box.conf[0])
            if confidence < self.registration_confidence or index >= len(polygons):
                continue
            polygon = np.asarray(polygons[index], dtype=np.float32)
            if polygon.shape[0] < 3:
                continue
            mask = np.zeros((height, width), dtype=np.uint8)
            cv2.fillPoly(mask, [np.round(polygon).astype(np.int32)], 1)
            mask = mask.astype(bool)
            depth_m = mask_depth_median(mask, depth)
            if depth_m is None:
                continue
            class_id = int(box.cls[0])
            xyxy = tuple(float(value) for value in box.xyxy[0].tolist())
            center = ((xyxy[0] + xyxy[2]) * 0.5, (xyxy[1] + xyxy[3]) * 0.5)
            measurement = measure_mask_size_3d(mask, depth, intrinsics)
            detections.append(SegDetection(class_id, str(self.model.names[class_id]), confidence, xyxy, center, int(mask.sum()), depth_m, polygon, mask, measurement))
        return detections

    def draw(self, rgb, detections):
        """RGB 영상에 결함 mask·박스·클래스·신뢰도·Depth를 표시한다."""
        image = rgb.copy()
        for detection in detections:
            overlay = np.zeros_like(image)
            overlay[detection.mask] = (255, 64, 64)
            image = cv2.addWeighted(image, 1.0, overlay, 0.35, 0.0)
            x1, y1, x2, y2 = (int(round(value)) for value in detection.bbox_xyxy)
            cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), 2)
            label = f"{detection.class_name} {detection.confidence:.2f} {detection.depth_m:.3f}m"
            cv2.putText(image, label, (x1, max(22, y1 - 7)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2, cv2.LINE_AA)
            if detection.measurement.physical_size_valid:
                size_label = f"L={detection.measurement.length_mm:.1f}mm W={detection.measurement.width_mm:.1f}mm"
                cv2.putText(image, size_label, (x1, min(image.shape[0] - 8, y2 + 22)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2, cv2.LINE_AA)
        return image
