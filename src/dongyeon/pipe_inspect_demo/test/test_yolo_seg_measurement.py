"""Seg 마스크와 Depth를 이용한 실제 치수 측정을 검증한다."""

import numpy as np
import pytest

from pipe_inspect_demo.yolo_seg_detector import measure_mask_size_3d


def test_constant_depth_rectangle_size():
    """일정 Depth 직사각형 마스크의 긴 변과 짧은 변을 mm로 복원한다."""
    mask = np.zeros((100, 100), dtype=bool)
    mask[40:60, 30:70] = True
    depth = np.ones((100, 100), dtype=np.float32)
    result = measure_mask_size_3d(mask, depth, (100.0, 100.0, 50.0, 50.0))
    assert result.physical_size_valid is True
    assert 300.0 <= result.length_mm <= 500.0
    assert result.width_mm == pytest.approx(190.0, abs=15.0)
    assert result.skeleton_points > 0
    assert result.skeleton_components >= 1
    assert result.quality == "valid"


def test_partial_missing_depth_remains_valid():
    """마스크 Depth 일부가 누락돼도 유효 비율이 충분하면 치수를 계산한다."""
    mask = np.zeros((40, 40), dtype=bool)
    mask[10:30, 10:30] = True
    depth = np.full((40, 40), 0.1, dtype=np.float32)
    depth[10:15, 10:30] = np.nan
    result = measure_mask_size_3d(mask, depth, (200.0, 200.0, 20.0, 20.0))
    assert result.physical_size_valid is True
    assert result.valid_depth_ratio == pytest.approx(0.75)


def test_insufficient_depth_is_invalid():
    """유효 Depth 비율이 부족하면 거짓 치수 대신 무효 상태를 반환한다."""
    mask = np.ones((10, 10), dtype=bool)
    depth = np.full((10, 10), np.nan, dtype=np.float32)
    depth[:2] = 0.05
    result = measure_mask_size_3d(mask, depth, (100.0, 100.0, 5.0, 5.0))
    assert result.physical_size_valid is False
    assert result.length_mm is None
    assert result.quality == "insufficient_valid_depth"


def test_border_mask_is_not_valid_for_fusion():
    """화면 가장자리에 잘린 마스크 치수를 대표값 융합에 사용하지 않도록 표시한다."""
    mask = np.zeros((40, 40), dtype=bool)
    mask[:20, 10:30] = True
    depth = np.full((40, 40), 0.1, dtype=np.float32)
    result = measure_mask_size_3d(mask, depth, (200.0, 200.0, 20.0, 20.0))
    assert result.touches_image_border is True
    assert result.physical_size_valid is False
    assert result.quality == "partial_image_border"
