"""[공용] 관 상태 판정 — ROS 비의존 순수 로직.

판정 순서 (사양서):
  1. Depth 무효 픽셀 비율 > 임계        → DISCONNECTED
  2. 전방 개구부 원형도 < 임계          → UNDETERMINED
  3. 원 피팅 중심 이탈량 = 축 오프셋(mm) → NORMAL / MISALIGNMENT 등급

곡관 예외: 관절 엔코더 각도가 임계 이상이면 위를 전부 무시하고 NORMAL.

Disconnected 와 Misalignment 의 구분은 Depth 반사 유무 하나로 한다.
어긋난 조인트는 먼 관 끝면이 초승달로 노출돼 전 영역에서 Depth 가 유효하지만,
관이 끊겼으면 그 너머가 빈 공간이라 반사가 없어 무효가 된다.
"""

from dataclasses import dataclass, asdict

import cv2
import numpy as np

NORMAL = "NORMAL"
MISALIGNMENT = "MISALIGNMENT"
DISCONNECTED = "DISCONNECTED"
UNDETERMINED = "UNDETERMINED"

SPEED_FULL = "full"
SPEED_SLOW = "slow"
SPEED_STOP = "stop"


DEFAULTS = dict(
    projection="fisheye_equidistant",
    invalid_ratio_max=0.02,
    invalid_mode="empty_is_zero",
    max_range_m=1.5,
    circularity_min=0.60,
    roughness_max=0.015,
    aperture_min_px=400,
    offset_normal_mm=1.0,
    offset_slow_mm=3.0,
    offset_max_mm=8.0,
    bend_angle_deg=8.0,
    center_patch_frac=0.10,
    blur_ksize=5,
    morph_ksize=5,
)


@dataclass
class Condition:
    state: str = UNDETERMINED
    circularity: float = 0.0
    roughness: float = 0.0
    forward_range_m: float = 0.0
    joint_range_m: float = 0.0
    incidence_deg: float = 0.0
    offset_mm: float = 0.0
    invalid_ratio: float = 0.0
    aperture_px: int = 0
    passable: bool = False
    speed: str = SPEED_STOP
    reason: str = ""

    def as_dict(self):
        return asdict(self)


class PipeConditionDetector:
    def __init__(self, intrinsics, params=None):
        self.k = dict(DEFAULTS)
        if params:
            self.k.update({a: b for a, b in params.items() if a in self.k})
        self.fx = float(intrinsics["fx"])
        self.fy = float(intrinsics["fy"])
        self.ppx = float(intrinsics["ppx"])
        self.ppy = float(intrinsics["ppy"])
        self.f_fish = float(intrinsics.get("f_fish", self.fx))

    def incidence_angle(self, cx, cy):
        """개구부 중심 픽셀 -> 광축에서 벌어진 각(rad).

        어안은 등거리 r = f*theta, 핀홀은 theta = atan(r/f) 로 서로 다르다.
        140도 어안에서 핀홀 식을 쓰면 가장자리로 갈수록 크게 틀어진다.
        """
        r = np.hypot((cx - self.ppx), (cy - self.ppy))
        if self.k["projection"] == "fisheye_equidistant":
            return float(r / self.f_fish)
        return float(np.arctan2(r, self.fx))

    def invalid_mask(self, depth):
        if self.k["invalid_mode"] == "empty_is_far":
            return ~np.isfinite(depth) | (depth <= 0) | (
                depth > self.k["max_range_m"])
        return ~np.isfinite(depth) | (depth <= 0)

    def aperture(self, depth, invalid):
        """전방 개구부 = 무효 픽셀 + 먼 쪽 유효 픽셀. Otsu 로 원근을 가른다."""
        valid = ~invalid
        far = np.zeros(depth.shape, np.uint8)
        if valid.sum() > 100:
            d = depth[valid]
            lo, hi = float(d.min()), float(d.max())
            if hi - lo > 1e-4:
                norm = np.zeros(depth.shape, np.uint8)
                norm[valid] = np.clip((depth[valid] - lo) / (hi - lo) * 255,
                                      0, 255).astype(np.uint8)
                t, _ = cv2.threshold(norm[valid], 0, 255,
                                     cv2.THRESH_BINARY | cv2.THRESH_OTSU)
                far = ((norm >= t) & valid).astype(np.uint8)
        mask = (far | invalid.astype(np.uint8)) * 255
        k = int(self.k["morph_ksize"])
        if k > 1:
            el = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, el)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, el)
        return mask

    def _rim_depth(self, depth, mask, invalid, band=9):
        """개구부 테두리 바깥 띠의 깊이 중앙값 = 조인트 평면까지 거리."""
        el = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (band, band))
        ring = (cv2.dilate(mask, el) > 0) & (mask == 0) & (~invalid)
        vals = depth[ring]
        vals = vals[np.isfinite(vals) & (vals > 0)]
        return float(np.median(vals)) if vals.size > 20 else 0.0

    def run(self, depth, joint_angle_deg=0.0, rgb=None):
        c = Condition()
        depth = np.asarray(depth, dtype=np.float64)
        if self.k["blur_ksize"] > 1:
            depth = cv2.medianBlur(depth.astype(np.float32),
                                   int(self.k["blur_ksize"])).astype(np.float64)

        invalid = self.invalid_mask(depth)
        c.invalid_ratio = float(invalid.mean())

        pf = self.k["center_patch_frac"]
        h, w = depth.shape
        y0, y1 = int(h * (0.5 - pf)), int(h * (0.5 + pf))
        x0, x1 = int(w * (0.5 - pf)), int(w * (0.5 + pf))
        patch = depth[y0:y1, x0:x1]
        pv = patch[np.isfinite(patch) & (patch > 0)]
        c.forward_range_m = float(np.median(pv)) if pv.size else 0.0

        if abs(joint_angle_deg) >= self.k["bend_angle_deg"]:
            c.state, c.passable, c.speed = NORMAL, True, SPEED_FULL
            c.reason = f"곡관 (관절 {joint_angle_deg:.1f}°) — 판정 생략"
            return c

        if c.invalid_ratio > self.k["invalid_ratio_max"]:
            c.state, c.passable, c.speed = DISCONNECTED, False, SPEED_STOP
            c.reason = (f"무효 픽셀 {c.invalid_ratio * 100:.1f}% > "
                        f"{self.k['invalid_ratio_max'] * 100:.1f}%")
            return c

        mask = self.aperture(depth, invalid)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_NONE)
        if not cnts:
            c.reason = "개구부 윤곽 없음"
            return c
        cnt = max(cnts, key=cv2.contourArea)
        area = float(cv2.contourArea(cnt))
        peri = float(cv2.arcLength(cnt, True))
        c.aperture_px = int(area)
        if area < self.k["aperture_min_px"] or peri < 1e-6:
            c.reason = f"개구부 과소 {area:.0f}px"
            return c

        c.circularity = float(4.0 * np.pi * area / (peri ** 2))
        # 원형도는 둘레에 좌우돼 완만한 요철에 둔감하다. 윤곽 반경의 상대
        # 표준편차가 같은 요철을 훨씬 민감하게 잡는다(합성 장면 실측).
        pts = cnt.reshape(-1, 2).astype(np.float64)
        rad = np.hypot(*(pts - pts.mean(0)).T)
        c.roughness = float(rad.std() / rad.mean()) if rad.mean() > 1e-9 else 1.0
        if c.circularity < self.k["circularity_min"]:
            c.state, c.passable, c.speed = UNDETERMINED, False, SPEED_STOP
            c.reason = (f"원형도 {c.circularity:.3f} < "
                        f"{self.k['circularity_min']:.2f}")
            return c
        if c.roughness > self.k["roughness_max"]:
            c.state, c.passable, c.speed = UNDETERMINED, False, SPEED_STOP
            c.reason = (f"윤곽 거칠기 {c.roughness:.4f} > "
                        f"{self.k['roughness_max']:.4f}")
            return c

        (cx, cy), _ = cv2.minEnclosingCircle(cnt)
        # 역투영 거리는 화면 중앙 깊이(먼 관 안쪽)가 아니라 개구부가 놓인
        # 조인트 평면까지의 거리다. 개구부 테두리 바깥 띠의 깊이가 그 값이다.
        c.joint_range_m = self._rim_depth(depth, mask, invalid)
        rng = c.joint_range_m if c.joint_range_m > 0 else c.forward_range_m
        # depth 는 distance_to_camera(실제 거리)이므로 횡방향 성분은 D*sin(theta).
        # 투영 방식은 theta 를 구하는 단계에만 들어간다.
        theta = self.incidence_angle(cx, cy)
        c.incidence_deg = float(np.degrees(theta))
        c.offset_mm = float(rng * np.sin(theta) * 1000.0)

        if c.offset_mm <= self.k["offset_normal_mm"]:
            c.state, c.passable, c.speed = NORMAL, True, SPEED_FULL
        elif c.offset_mm <= self.k["offset_slow_mm"]:
            c.state, c.passable, c.speed = MISALIGNMENT, True, SPEED_FULL
        elif c.offset_mm <= self.k["offset_max_mm"]:
            c.state, c.passable, c.speed = MISALIGNMENT, True, SPEED_SLOW
        else:
            c.state, c.passable, c.speed = MISALIGNMENT, False, SPEED_STOP
        c.reason = f"오프셋 {c.offset_mm:.2f} mm"
        return c
