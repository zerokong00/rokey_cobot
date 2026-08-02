"""ROS에 의존하지 않는 배관 dark-blob 결함 검출 코어."""

from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class DefectCandidate:
    """한 프레임에서 선택된 가장 큰 결함 후보."""

    box: Tuple[int, int, int, int]
    area: int
    center_x: float
    center_y: float
    confidence: float


@dataclass(frozen=True)
class DetectionResult:
    """한 프레임의 검출 결과."""

    calibrated: bool
    raw_detected: bool
    stable_detected: bool
    threshold: Optional[float]
    area_min: int
    candidate: Optional[DefectCandidate]


def annulus_mask(height: int, width: int, inner: float, outer: float):
    """영상 중심 기준 고리형 마스크와 정규화 반경 배열을 반환한다."""
    if height <= 0 or width <= 0:
        raise ValueError("영상 크기는 0보다 커야 합니다")
    if not 0.0 <= inner < outer <= 1.5:
        raise ValueError("inner < outer 범위가 잘못되었습니다")
    yy, xx = np.mgrid[0:height, 0:width]
    radius = np.hypot(yy - height / 2.0, xx - width / 2.0) / (
        min(height, width) / 2.0
    )
    return (radius > inner) & (radius < outer), radius


class DarkBlobDetector:
    """캘리브레이션, blob 검출과 시간 안정화를 수행한다."""

    def __init__(
        self,
        calibration_frames=12,
        dark_ratio=0.45,
        annulus_inner=0.45,
        annulus_outer=0.92,
        min_area=40,
        noise_area_scale=2.0,
        confirm_frames=3,
        release_frames=3,
        minimum_wall_brightness=5.0,
    ):
        self.calibration_frames = max(1, int(calibration_frames))
        self.dark_ratio = float(dark_ratio)
        self.annulus_inner = float(annulus_inner)
        self.annulus_outer = float(annulus_outer)
        self.base_min_area = max(1, int(min_area))
        self.noise_area_scale = max(1.0, float(noise_area_scale))
        self.confirm_frames = max(1, int(confirm_frames))
        self.release_frames = max(1, int(release_frames))
        self.minimum_wall_brightness = float(minimum_wall_brightness)

        if not 0.0 < self.dark_ratio < 1.0:
            raise ValueError("dark_ratio는 0과 1 사이여야 합니다")
        if not 0.0 <= self.annulus_inner < self.annulus_outer <= 1.5:
            raise ValueError("annulus 범위가 잘못되었습니다")

        self.mask = None
        self.radius = None
        self.shape = None
        self.threshold = None
        self.area_min = self.base_min_area
        self.calibration_medians = []
        self.calibration_blob_max = 0
        self.history = deque(maxlen=max(self.confirm_frames, self.release_frames))
        self.stable_detected = False
        self.last_reset_reason = None

    @property
    def calibrated(self):
        return self.threshold is not None

    @property
    def calibration_progress(self):
        return len(self.calibration_medians), self.calibration_frames

    def reset(self, reason=None):
        """현재 캘리브레이션과 시간 상태를 초기화한다."""
        self.calibration_medians.clear()
        self.calibration_blob_max = 0
        self.threshold = None
        self.area_min = self.base_min_area
        self.history.clear()
        self.stable_detected = False
        self.last_reset_reason = reason

    def _ensure_mask(self, gray):
        shape = gray.shape[:2]
        if shape == self.shape:
            return
        was_initialized = self.shape is not None
        self.mask, self.radius = annulus_mask(
            shape[0], shape[1], self.annulus_inner, self.annulus_outer
        )
        self.shape = shape
        if was_initialized:
            self.reset("resolution_changed")

    def _calibrate(self, gray):
        median = float(np.median(gray[self.mask]))
        self.calibration_medians.append(median)
        provisional = self.dark_ratio * max(median, 1.0)
        dark = (gray < provisional) & self.mask
        count, labels, stats, _ = cv2.connectedComponentsWithStats(
            dark.astype(np.uint8), connectivity=8
        )
        inner_guard = self.annulus_inner + 0.015
        for index in range(1, count):
            pixels = labels == index
            if not np.any(pixels) or float(self.radius[pixels].min()) < inner_guard:
                continue
            self.calibration_blob_max = max(
                self.calibration_blob_max, int(stats[index, cv2.CC_STAT_AREA])
            )

        if len(self.calibration_medians) < self.calibration_frames:
            return
        wall_median = float(np.median(self.calibration_medians))
        if wall_median < self.minimum_wall_brightness:
            self.reset("low_brightness")
            return
        self.threshold = self.dark_ratio * wall_median
        self.area_min = max(
            self.base_min_area,
            int(np.ceil(self.noise_area_scale * self.calibration_blob_max)),
        )

    def _find_candidate(self, gray):
        dark = (gray < self.threshold) & self.mask
        count, labels, stats, centers = cv2.connectedComponentsWithStats(
            dark.astype(np.uint8), connectivity=8
        )
        hits = []
        inner_guard = self.annulus_inner + 0.015
        for index in range(1, count):
            area = int(stats[index, cv2.CC_STAT_AREA])
            if area < self.area_min:
                continue
            pixels = labels == index
            if float(self.radius[pixels].min()) < inner_guard:
                continue
            hits.append(index)
        if not hits:
            return None

        index = max(hits, key=lambda item: stats[item, cv2.CC_STAT_AREA])
        x, y, width, height, area = (int(value) for value in stats[index])
        confidence = min(1.0, area / max(self.area_min * 3.0, 1.0))
        return DefectCandidate(
            box=(x, y, width, height),
            area=area,
            center_x=float(centers[index][0]),
            center_y=float(centers[index][1]),
            confidence=float(confidence),
        )

    def _update_stable_state(self, detected):
        self.history.append(bool(detected))
        if not self.stable_detected:
            recent = list(self.history)[-self.confirm_frames :]
            if len(recent) == self.confirm_frames and all(recent):
                self.stable_detected = True
        else:
            recent = list(self.history)[-self.release_frames :]
            if len(recent) == self.release_frames and not any(recent):
                self.stable_detected = False

    def process(self, rgb_image):
        """RGB uint8 영상 한 장을 처리하고 DetectionResult를 반환한다."""
        image = np.asarray(rgb_image)
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("입력은 HxWx3 RGB 영상이어야 합니다")
        if image.dtype != np.uint8:
            raise ValueError("입력 영상 dtype은 uint8이어야 합니다")

        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        self._ensure_mask(gray)
        if not self.calibrated:
            self._calibrate(gray)
            return DetectionResult(
                calibrated=self.calibrated,
                raw_detected=False,
                stable_detected=False,
                threshold=self.threshold,
                area_min=self.area_min,
                candidate=None,
            )

        candidate = self._find_candidate(gray)
        self._update_stable_state(candidate is not None)
        return DetectionResult(
            calibrated=True,
            raw_detected=candidate is not None,
            stable_detected=self.stable_detected,
            threshold=float(self.threshold),
            area_min=self.area_min,
            candidate=candidate,
        )
