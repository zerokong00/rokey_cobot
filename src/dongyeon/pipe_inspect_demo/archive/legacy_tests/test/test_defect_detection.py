"""ROS 없이 실행되는 dark-blob 검출 코어 단위 테스트."""

import cv2
import numpy as np

from pipe_inspect_demo.defect_detection import DarkBlobDetector, annulus_mask


def normal_frame(size=200, brightness=180):
    return np.full((size, size, 3), brightness, dtype=np.uint8)


def calibrated_detector(**overrides):
    options = dict(calibration_frames=3, min_area=30, confirm_frames=3, release_frames=3)
    options.update(overrides)
    detector = DarkBlobDetector(**options)
    frame = normal_frame()
    for _ in range(options["calibration_frames"]):
        detector.process(frame)
    assert detector.calibrated
    return detector


def test_annulus_excludes_center_and_edge():
    mask, _ = annulus_mask(100, 100, 0.45, 0.92)
    assert not mask[50, 50]
    assert mask[50, 80]
    assert not mask[0, 0]


def test_normal_wall_is_not_detected():
    result = calibrated_detector().process(normal_frame())
    assert not result.raw_detected
    assert not result.stable_detected


def test_dark_hole_in_annulus_is_detected():
    detector = calibrated_detector()
    frame = normal_frame()
    cv2.circle(frame, (160, 100), 9, (5, 5, 5), -1)
    result = detector.process(frame)
    assert result.raw_detected
    assert result.candidate.area >= 30


def test_dark_center_is_ignored():
    detector = calibrated_detector()
    frame = normal_frame()
    cv2.circle(frame, (100, 100), 25, (0, 0, 0), -1)
    assert not detector.process(frame).raw_detected


def test_small_noise_is_ignored():
    detector = calibrated_detector(min_area=50)
    frame = normal_frame()
    cv2.circle(frame, (160, 100), 2, (0, 0, 0), -1)
    assert not detector.process(frame).raw_detected


def test_three_frames_confirm_and_release_detection():
    detector = calibrated_detector()
    normal = normal_frame()
    defect = normal.copy()
    cv2.circle(defect, (160, 100), 9, (0, 0, 0), -1)

    assert not detector.process(defect).stable_detected
    assert not detector.process(defect).stable_detected
    assert detector.process(defect).stable_detected
    assert detector.process(normal).stable_detected
    assert detector.process(normal).stable_detected
    assert not detector.process(normal).stable_detected
