"""결함 위치 추적과 JSON 스키마 v1.1을 검증한다."""

import math
from types import SimpleNamespace

import pytest

from pipe_inspect_demo.defect_position_tracker import DefectPositionTracker, quaternion_pitch_deg, quaternion_roll_deg, quaternion_yaw_deg, rotate_vector_by_quaternion


def make_detection(measurement=None):
    """JSON 레코드 검증에 사용할 크랙 검출 객체를 만든다."""
    return SimpleNamespace(center_pixel=(320.0, 300.0), depth_m=0.05, class_name="crack", confidence=0.91, area_px=120, bbox_xyxy=(300.0, 280.0, 340.0, 320.0), measurement=measurement)


def make_measurement(length_mm, width_mm):
    """다중 관측 치수 융합 테스트용 유효 Skeleton 측정을 만든다."""
    return SimpleNamespace(length_mm=length_mm, width_mm=width_mm, physical_size_valid=True, method="mask_depth_3d_skeleton", valid_depth_points=100, valid_depth_ratio=1.0, skeleton_points=20, skeleton_components=1, longest_component_points=20, touches_image_border=False, quality="valid")


def test_quaternion_angles():
    """단위 quaternion과 90도 Yaw quaternion의 각도 변환을 검증한다."""
    assert quaternion_roll_deg(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)
    half = math.radians(90.0) * 0.5
    assert quaternion_yaw_deg(0.0, 0.0, math.sin(half), math.cos(half)) == pytest.approx(90.0)
    assert quaternion_pitch_deg(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)
    assert rotate_vector_by_quaternion((1.0, 0.0, 0.0), (0.0, 0.0, math.sin(half), math.cos(half))) == pytest.approx((0.0, 1.0, 0.0))


def test_schema_v11_record_with_imu():
    """IMU가 준비된 결함 레코드에 v1.1 자세·측정 필드가 포함되는지 검증한다."""
    tracker = DefectPositionTracker()
    tracker.update_odometry((0.0, 0.0, 0.0))
    tracker.update_odometry((0.0, 0.6, 0.0))
    half = math.radians(90.0) * 0.5
    tracker.update_imu((0.0, 0.0, math.sin(half), math.cos(half)))
    record = tracker.build_record(make_detection(), (500.0, 500.0, 320.0, 320.0), {"sec": 1, "nanosec": 2}, "camera")
    assert record["schema_version"] == "1.5"
    assert record["robot"]["orientation_xyzw"] == pytest.approx((0.0, 0.0, math.sin(half), math.cos(half)))
    assert record["robot"]["roll_deg"] == pytest.approx(0.0)
    assert record["robot"]["yaw_deg"] == pytest.approx(90.0)
    assert record["robot"]["pitch_deg"] == pytest.approx(0.0)
    assert record["observation_pose"]["camera_position_xyz_m"] == pytest.approx((0.0, 0.6, 0.0))
    assert record["defect"]["axial_position_from_entry_m"] == pytest.approx(0.65)
    assert record["repair_target"]["position_xyz_m"] == pytest.approx((0.0, 0.65, 0.002))
    assert record["repair_target"]["pose_transform_valid"] is False
    assert record["repair_target"]["pose_transform_reason"] == "radial_out_of_tolerance"
    assert record["measurement"] == {"length_mm": None, "width_mm": None, "physical_size_valid": False, "method": "not_implemented", "valid_depth_points": 0, "valid_depth_ratio": 0.0, "skeleton_points": 0, "skeleton_components": 0, "longest_component_points": 0, "touches_image_border": False, "observation_count": 0, "length_range_mm": None, "width_range_mm": None, "quality": "not_implemented"}


def test_nonzero_pipe_entry_camera_coordinate():
    """첫 카메라 위치를 다시 영점화하지 않고 파이프 입구 상대좌표로 유지하는지 검증한다."""
    tracker = DefectPositionTracker()
    tracker.update_odometry((0.0, 0.12, 0.01))
    half = math.radians(90.0) * 0.5
    tracker.update_imu((0.0, 0.0, math.sin(half), math.cos(half)))
    record = tracker.build_record(make_detection(), (500.0, 500.0, 320.0, 320.0), {"sec": 1, "nanosec": 2}, "camera")
    assert record["observation_pose"]["camera_position_xyz_m"] == pytest.approx((0.0, 0.12, 0.01))
    assert record["defect"]["axial_position_from_entry_m"] == pytest.approx(0.17)


def test_camera_world_axes_ray_reaches_pipe_wall():
    """카메라 +X 전방 광선과 우측 픽셀이 파이프 벽 좌표로 변환되는지 검증한다."""
    tracker = DefectPositionTracker()
    tracker.update_odometry((0.0, 0.1, 0.0))
    half = math.radians(90.0) * 0.5
    tracker.update_imu((0.0, 0.0, math.sin(half), math.cos(half)))
    detection = make_detection()
    detection.center_pixel = (820.0, 320.0)
    record = tracker.build_record(detection, (500.0, 500.0, 320.0, 320.0), {"sec": 1, "nanosec": 2}, "camera")
    assert record["repair_target"]["wall_radius_m"] == pytest.approx(0.05)
    assert record["repair_target"]["pose_transform_valid"] is True


def test_schema_marks_missing_imu():
    """IMU가 없으면 자세를 임의 생성하지 않고 unavailable로 기록하는지 검증한다."""
    tracker = DefectPositionTracker()
    tracker.update_odometry((0.0, 0.0, 0.0))
    record = tracker.build_record(make_detection(), (500.0, 500.0, 320.0, 320.0), {"sec": 1, "nanosec": 2}, "camera")
    assert record["robot"]["orientation_xyzw"] is None
    assert record["robot"]["roll_deg"] is None
    assert record["robot"]["yaw_deg"] is None
    assert record["robot"]["orientation_source"] == "unavailable"
    assert record["repair_target"]["pose_transform_valid"] is False


def test_reset_clears_mission_state():
    """새 검사 임무 초기화가 이동량과 결함 등록 및 측정 이력을 모두 제거하는지 검증한다."""
    tracker = DefectPositionTracker(confirmation_frames=1)
    tracker.update_odometry((0.0, 0.0, 0.0))
    tracker.update_odometry((0.0, 0.5, 0.0))
    tracker.update_imu((0.0, 0.0, 0.0, 1.0))
    record = tracker.build_record(make_detection(), (500.0, 500.0, 320.0, 320.0), {"sec": 1, "nanosec": 0}, "camera")
    tracker.register(record)
    tracker.reset()
    assert tracker.ready is False
    assert tracker.travel_distance_m == 0.0
    assert tracker.imu_quaternion_xyzw is None
    assert tracker.records == []
    assert tracker.pending_records == []
    assert tracker.measurement_histories == {}


def test_multi_view_measurement_fusion():
    """동일 결함의 길이는 상위값, 폭은 중앙값으로 융합하고 치수 갱신을 분리한다."""
    tracker = DefectPositionTracker()
    tracker.update_odometry((0.0, 0.0, 0.0))
    tracker.update_imu((0.0, 0.0, 0.0, 1.0))
    result = status = None
    for nanosec, length, width in ((0, 10.0, 1.0), (200_000_000, 30.0, 3.0), (400_000_000, 20.0, 2.0)):
        record = tracker.build_record(make_detection(make_measurement(length, width)), (500.0, 500.0, 320.0, 320.0), {"sec": 1, "nanosec": nanosec}, "camera")
        result, status = tracker.register(record)
    assert status == "new"
    assert result["measurement"]["length_mm"] == pytest.approx(30.0)
    assert result["measurement"]["width_mm"] == pytest.approx(2.0)
    assert result["measurement"]["observation_count"] == 3
    record = tracker.build_record(make_detection(make_measurement(40.0, 4.0)), (500.0, 500.0, 320.0, 320.0), {"sec": 1, "nanosec": 600_000_000}, "camera")
    result, status = tracker.register(record)
    assert status == "measurement_updated"
    assert result["measurement"]["length_mm"] == pytest.approx(40.0)
    assert result["measurement"]["width_mm"] == pytest.approx(2.5)


def test_alignment_observation_keeps_registered_identity():
    """중복 관측에도 확정 결함 ID와 최신 융합 치수가 연결되는지 검증한다."""
    tracker = DefectPositionTracker(confirmation_frames=3)
    tracker.update_odometry((0.0, 0.6, 0.0))
    tracker.update_imu((0.0, 0.0, 0.0, 1.0))
    for nanosec in (0, 100_000_000, 200_000_000):
        first = tracker.build_record(make_detection(make_measurement(20.0, 1.0)), (500.0, 500.0, 320.0, 320.0), {"sec": 1, "nanosec": nanosec}, "camera")
        registered, status = tracker.register(first)
    assert status == "new"
    duplicate = tracker.build_record(make_detection(make_measurement(30.0, 2.0)), (500.0, 500.0, 320.0, 320.0), {"sec": 1, "nanosec": 300_000_000}, "camera")
    tracker.register(duplicate)
    aligned = tracker.attach_registered_identity(duplicate)
    assert aligned["defect_id"] == registered["defect_id"]
    assert aligned["measurement"]["observation_count"] == 4
    assert aligned["segmentation"]["center_pixel"] == duplicate["segmentation"]["center_pixel"]
