"""오도메트리 입구 기준 이동량과 결함의 Depth·원주 위치를 계산한다."""

import copy
import math

import numpy as np


def quaternion_roll_deg(x, y, z, w):
    """ROS xyzw 쿼터니언에서 중력 기준 Roll 각도를 계산한다."""
    return math.degrees(math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))


def quaternion_yaw_deg(x, y, z, w):
    """ROS xyzw 쿼터니언에서 월드 기준 Yaw 각도를 계산한다."""
    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def quaternion_pitch_deg(x, y, z, w):
    """ROS xyzw 쿼터니언에서 입구 기준 Pitch 각도를 계산한다."""
    value = 2.0 * (w * y - z * x)
    return math.degrees(math.copysign(math.pi * 0.5, value) if abs(value) >= 1.0 else math.asin(value))


def rotate_vector_by_quaternion(vector_xyz, quaternion_xyzw):
    """xyzw quaternion으로 3차원 벡터를 회전한다."""
    vector = np.asarray(vector_xyz, dtype=np.float64)
    quaternion = np.asarray(quaternion_xyzw, dtype=np.float64)
    xyz, w = quaternion[:3], quaternion[3]
    return vector + 2.0 * np.cross(xyz, np.cross(xyz, vector) + w * vector)


def normalize_angle_deg(angle):
    """각도를 0 이상 360도 미만 범위로 정규화한다."""
    return float(angle % 360.0)


class DefectPositionTracker:
    """입구 기준 로봇 이동과 중복 없는 결함 레코드를 관리한다."""

    def __init__(self, axial_tolerance_m=0.10, clock_tolerance_deg=45.0, confirmation_frames=3, confirmation_max_gap_sec=0.5, confidence_update_delta=0.02, pipe_inner_radius_m=0.05, radial_tolerance_m=0.015):
        """결함 병합·연속 검증·신뢰도 갱신 기준을 초기화한다."""
        self.axial_tolerance_m = float(axial_tolerance_m)
        self.clock_tolerance_deg = float(clock_tolerance_deg)
        self.confirmation_frames = int(confirmation_frames)
        self.confirmation_max_gap_sec = float(confirmation_max_gap_sec)
        self.confidence_update_delta = float(confidence_update_delta)
        self.pipe_inner_radius_m, self.radial_tolerance_m = float(pipe_inner_radius_m), float(radial_tolerance_m)
        if self.confirmation_frames < 1:
            raise ValueError("결함 확정 프레임 수는 1 이상이어야 해.")
        if self.pipe_inner_radius_m <= 0.0 or self.radial_tolerance_m < 0.0:
            raise ValueError("파이프 반지름은 양수이고 반경 허용 오차는 0 이상이어야 해.")
        self.reset()

    def reset(self):
        """새 검사 임무를 위해 이동량·자세·결함 등록·측정 융합 상태를 초기화한다."""
        self.entry_position = None
        self.previous_position = None
        self.current_position = None
        self.travel_distance_m = 0.0
        self.imu_quaternion_xyzw = None
        self.imu_roll_deg = None
        self.imu_pitch_deg = None
        self.imu_yaw_deg = None
        self.records = []
        self.pending_records = []
        self.measurement_histories = {}

    @property
    def ready(self):
        """입구 기준 오도메트리가 준비되었는지 반환한다."""
        return self.entry_position is not None

    def update_odometry(self, position_xyz):
        """현재 odom 위치를 반영하고 입구부터 누적 이동 거리를 갱신한다."""
        position = np.asarray(position_xyz, dtype=np.float64)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("odom 위치는 유효한 xyz 세 값이어야 해.")
        if self.entry_position is None:
            self.entry_position = position.copy()
            self.previous_position = position.copy()
        else:
            self.travel_distance_m += float(np.linalg.norm(position - self.previous_position))
            self.previous_position = position.copy()
        self.current_position = position.copy()

    def update_imu(self, quaternion_xyzw):
        """IMU quaternion을 정규화하고 현재 Roll·Yaw를 갱신한다."""
        quaternion = np.asarray(quaternion_xyzw, dtype=np.float64)
        norm = float(np.linalg.norm(quaternion))
        if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)) or norm <= 0.0:
            raise ValueError("IMU 자세는 유효한 xyzw 네 값이어야 해.")
        quaternion /= norm
        self.imu_quaternion_xyzw = quaternion
        self.imu_roll_deg = quaternion_roll_deg(*quaternion)
        self.imu_pitch_deg = quaternion_pitch_deg(*quaternion)
        self.imu_yaw_deg = quaternion_yaw_deg(*quaternion)

    def build_record(self, detection, intrinsics, stamp, frame_id):
        """Seg 검출과 카메라 내부 파라미터로 입구 기준 결함 레코드를 생성한다."""
        if not self.ready:
            raise RuntimeError("입구 기준 odom이 아직 준비되지 않았어.")
        fx, fy, cx, cy = (float(value) for value in intrinsics)
        if fx <= 0.0 or fy <= 0.0:
            raise ValueError("CameraInfo의 fx와 fy는 0보다 커야 해.")
        u, v = detection.center_pixel
        depth = float(detection.depth_m)
        camera_x = (u - cx) * depth / fx
        camera_y = (v - cy) * depth / fy
        displacement = self.current_position - self.entry_position
        camera_ray = np.array([depth, -camera_x, -camera_y], dtype=np.float64)
        ray_in_pipe = camera_ray if self.imu_quaternion_xyzw is None else rotate_vector_by_quaternion(camera_ray, self.imu_quaternion_xyzw)
        defect_position = self.current_position + ray_in_pipe
        axial_position = float(defect_position[1])
        radial_x, radial_z = float(defect_position[0]), float(defect_position[2])
        wall_radius = math.hypot(radial_x, radial_z)
        radial_error = abs(wall_radius - self.pipe_inner_radius_m)
        pose_transform_valid = self.imu_quaternion_xyzw is not None and radial_error <= self.radial_tolerance_m
        clock_angle = normalize_angle_deg(math.degrees(math.atan2(radial_x, radial_z)))
        approach_direction = None if wall_radius <= 1e-9 else [round(radial_x / wall_radius, 8), 0.0, round(radial_z / wall_radius, 8)]
        orientation = None if self.imu_quaternion_xyzw is None else [round(float(value), 8) for value in self.imu_quaternion_xyzw]
        roll = None if self.imu_roll_deg is None else round(self.imu_roll_deg, 3)
        pitch = None if self.imu_pitch_deg is None else round(self.imu_pitch_deg, 3)
        yaw = None if self.imu_yaw_deg is None else round(self.imu_yaw_deg, 3)
        camera_position = [round(float(value), 6) for value in self.current_position]
        pipe_position = [round(float(value), 6) for value in defect_position]
        measured = getattr(detection, "measurement", None)
        measurement = {"length_mm": None, "width_mm": None, "physical_size_valid": False, "method": "not_implemented", "valid_depth_points": 0, "valid_depth_ratio": 0.0, "skeleton_points": 0, "skeleton_components": 0, "longest_component_points": 0, "touches_image_border": False, "observation_count": 0, "length_range_mm": None, "width_range_mm": None, "quality": "not_implemented"} if measured is None else {"length_mm": measured.length_mm, "width_mm": measured.width_mm, "physical_size_valid": measured.physical_size_valid, "method": measured.method, "valid_depth_points": measured.valid_depth_points, "valid_depth_ratio": measured.valid_depth_ratio, "skeleton_points": measured.skeleton_points, "skeleton_components": measured.skeleton_components, "longest_component_points": measured.longest_component_points, "touches_image_border": measured.touches_image_border, "observation_count": 1 if measured.physical_size_valid else 0, "length_range_mm": None if measured.length_mm is None else [measured.length_mm, measured.length_mm], "width_range_mm": None if measured.width_mm is None else [measured.width_mm, measured.width_mm], "quality": measured.quality}
        return {"schema_version": "1.5", "event": "defect_detected", "timestamp": stamp, "frame_id": frame_id, "defect_id": None, "class": detection.class_name, "confidence": round(detection.confidence, 6), "robot": {"travel_distance_from_entry_m": round(self.travel_distance_m, 6), "displacement_from_entry_m": round(float(np.linalg.norm(displacement)), 6), "position_relative_to_entry_m": camera_position, "orientation_xyzw": orientation, "roll_deg": roll, "pitch_deg": pitch, "yaw_deg": yaw, "orientation_source": "simulated_camera_pose_relative_to_entry" if orientation is not None else "unavailable"}, "observation_pose": {"frame_id": "pipe_entry", "camera_position_xyz_m": camera_position, "camera_orientation_xyzw": orientation, "roll_deg": roll, "pitch_deg": pitch, "yaw_deg": yaw}, "defect": {"camera_depth_m": round(depth, 6), "axial_position_from_entry_m": round(axial_position, 6), "camera_position_m": [round(camera_x, 6), round(camera_y, 6), round(depth, 6)], "pipe_position_from_entry_m": pipe_position, "clock_angle_deg": round(clock_angle, 3), "clock_hour": int(round(clock_angle / 30.0)) % 12 or 12}, "repair_target": {"frame_id": "pipe_entry", "position_xyz_m": pipe_position, "axial_position_m": round(axial_position, 6), "clock_angle_deg": round(clock_angle, 3), "clock_hour": int(round(clock_angle / 30.0)) % 12 or 12, "wall_radius_m": round(wall_radius, 6), "expected_wall_radius_m": round(self.pipe_inner_radius_m, 6), "radial_error_m": round(radial_error, 6), "approach_direction_xyz": approach_direction, "pose_transform_valid": pose_transform_valid, "pose_transform_reason": "valid" if pose_transform_valid else "missing_orientation" if orientation is None else "radial_out_of_tolerance"}, "measurement": measurement, "segmentation": {"center_pixel": [round(float(u), 2), round(float(v), 2)], "area_px": int(detection.area_px), "bbox_xyxy": [round(float(value), 2) for value in detection.bbox_xyxy]}}

    def register(self, record):
        """연속 검증된 결함을 등록하고 의미 있는 최고 신뢰도 갱신만 반환한다."""
        duplicate_index = next((index for index, saved in enumerate(self.records) if self._is_same_defect(record, saved)), None)
        if duplicate_index is None:
            return self._confirm_pending(record)
        saved = self.records[duplicate_index]
        defect_id = saved["defect_id"]
        history = self.measurement_histories.setdefault(defect_id, [])
        previous_measurement = copy.deepcopy(saved["measurement"])
        self._append_measurement(history, record)
        fused_measurement = self._aggregate_measurements(history, record["measurement"])
        measurement_changed = self._measurement_changed(previous_measurement, fused_measurement)
        confidence_improved = record["confidence"] >= saved["confidence"] + self.confidence_update_delta
        if not confidence_improved and not measurement_changed:
            saved["measurement"] = fused_measurement
            return None, "duplicate"
        if confidence_improved:
            record["defect_id"] = defect_id
            record["measurement"] = fused_measurement
            self.records[duplicate_index] = record
            return record, "updated"
        saved["measurement"] = fused_measurement
        result = copy.deepcopy(saved)
        result["timestamp"] = record["timestamp"]
        return result, "measurement_updated"

    def attach_registered_identity(self, record):
        """현재 관측에 같은 결함의 확정 ID와 융합 치수를 연결한다."""
        saved = next((saved for saved in self.records if self._is_same_defect(record, saved)), None)
        if saved is None:
            return None
        result = copy.deepcopy(record)
        result["defect_id"] = saved["defect_id"]
        result["measurement"] = copy.deepcopy(saved["measurement"])
        return result

    def _confirm_pending(self, record):
        """같은 후보가 서로 다른 연속 프레임에서 반복될 때 신규 결함으로 확정한다."""
        stamp_key = self._stamp_key(record)
        pending_index = next((index for index, pending in enumerate(self.pending_records) if self._is_same_defect(record, pending["record"])), None)
        if pending_index is None:
            measurements = []
            self._append_measurement(measurements, record)
            self.pending_records.append({"record": record, "hits": 1, "last_stamp": stamp_key, "measurements": measurements})
            return None, "pending"
        pending = self.pending_records[pending_index]
        if pending["last_stamp"] != stamp_key:
            gap_sec = self._stamp_seconds(stamp_key) - self._stamp_seconds(pending["last_stamp"])
            pending["hits"] = pending["hits"] + 1 if 0.0 < gap_sec <= self.confirmation_max_gap_sec else 1
            pending["last_stamp"] = stamp_key
            self._append_measurement(pending["measurements"], record)
        if record["confidence"] > pending["record"]["confidence"]:
            pending["record"] = record
        if pending["hits"] < self.confirmation_frames:
            return None, "pending"
        confirmed = pending["record"]
        confirmed["defect_id"] = f"{confirmed['class'].lower()}_{len(self.records) + 1:04d}"
        confirmed["measurement"] = self._aggregate_measurements(pending["measurements"], confirmed["measurement"])
        self.records.append(confirmed)
        self.measurement_histories[confirmed["defect_id"]] = pending["measurements"]
        self.pending_records.pop(pending_index)
        return confirmed, "new"

    def _append_measurement(self, history, record):
        """같은 촬영 프레임을 제외한 유효 Skeleton 치수를 결함 관측 이력에 추가한다."""
        measurement = record["measurement"]
        stamp_key = self._stamp_key(record)
        if not measurement["physical_size_valid"] or any(sample["stamp"] == stamp_key for sample in history):
            return False
        history.append({"stamp": stamp_key, "length_mm": float(measurement["length_mm"]), "width_mm": float(measurement["width_mm"]), "valid_depth_points": int(measurement["valid_depth_points"]), "valid_depth_ratio": float(measurement["valid_depth_ratio"]), "skeleton_points": int(measurement["skeleton_points"]), "skeleton_components": int(measurement["skeleton_components"]), "longest_component_points": int(measurement["longest_component_points"])})
        del history[:-30]
        return True

    @staticmethod
    def _aggregate_measurements(history, fallback):
        """다중 관측 길이 상위값과 폭 중앙값으로 대표 결함 치수를 융합한다."""
        if not history:
            return copy.deepcopy(fallback)
        lengths = np.asarray([sample["length_mm"] for sample in history], dtype=np.float64)
        widths = np.asarray([sample["width_mm"] for sample in history], dtype=np.float64)
        representative_length = float(lengths.max()) if len(lengths) < 5 else float(np.percentile(lengths, 90.0))
        return {"length_mm": round(representative_length, 3), "width_mm": round(float(np.median(widths)), 3), "physical_size_valid": True, "method": "multi_view_skeleton_fusion", "valid_depth_points": max(sample["valid_depth_points"] for sample in history), "valid_depth_ratio": round(float(np.median([sample["valid_depth_ratio"] for sample in history])), 6), "skeleton_points": max(sample["skeleton_points"] for sample in history), "skeleton_components": max(sample["skeleton_components"] for sample in history), "longest_component_points": max(sample["longest_component_points"] for sample in history), "touches_image_border": False, "observation_count": len(history), "length_range_mm": [round(float(lengths.min()), 3), round(float(lengths.max()), 3)], "width_range_mm": [round(float(widths.min()), 3), round(float(widths.max()), 3)], "quality": "valid" if len(history) == 1 else "fused"}

    @staticmethod
    def _measurement_changed(previous, current):
        """대표 치수가 처음 융합되거나 허용 변화량을 넘었는지 판단한다."""
        if not current["physical_size_valid"]:
            return False
        if not previous["physical_size_valid"] or previous.get("observation_count", 0) < 2 <= current["observation_count"]:
            return True
        length_threshold = max(1.0, abs(float(previous["length_mm"])) * 0.05)
        width_threshold = max(0.2, abs(float(previous["width_mm"])) * 0.10)
        return abs(float(current["length_mm"]) - float(previous["length_mm"])) >= length_threshold or abs(float(current["width_mm"]) - float(previous["width_mm"])) >= width_threshold

    @staticmethod
    def _stamp_key(record):
        """한 프레임의 복수 마스크를 중복 횟수로 세지 않도록 타임스탬프 키를 만든다."""
        stamp = record.get("timestamp", {})
        return int(stamp.get("sec", 0)), int(stamp.get("nanosec", 0))

    @staticmethod
    def _stamp_seconds(stamp_key):
        """초·나노초 타임스탬프 키를 간격 비교용 실수 초로 변환한다."""
        return stamp_key[0] + stamp_key[1] * 1e-9

    def _is_same_defect(self, left, right):
        """클래스·축 거리·원주 각도 기준으로 같은 결함인지 판단한다."""
        if left["class"] != right["class"]:
            return False
        axial_delta = abs(left["defect"]["axial_position_from_entry_m"] - right["defect"]["axial_position_from_entry_m"])
        angle_delta = abs(left["defect"]["clock_angle_deg"] - right["defect"]["clock_angle_deg"])
        return axial_delta <= self.axial_tolerance_m and min(angle_delta, 360.0 - angle_delta) <= self.clock_tolerance_deg
