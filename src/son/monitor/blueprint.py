"""[공용] 도면 매칭 — 관리자 쪽. ROS 비의존.

**도면은 관리자만 가지고 있다.** 로봇은 상대 경로만 보내온다.

    로봇:    "직진 1.24m → 좌 90° → 직진 0.81m"
    관리자:  도면 위에 얹어  →  절대 좌표 (x, y) 와 결함의 실제 위치

진입점을 알고 있으므로(관리자가 거기에 로봇을 넣었다) 매칭은 단순하다.
도면 폴리라인을 따라 걸으면서 로봇이 보고한 곡관마다 **도면 꼭짓점에 스냅**한다.
이때 그 구간까지의 누적 오차가 끊긴다 — 설계 5.3 의 위상 매칭이 여기서 일어난다.

로봇이 도면을 안 갖는 대신 관리자가 갖는 구조라, 오차 리셋 효과는 그대로다.
"""

from dataclasses import dataclass, asdict, field

import math


@dataclass
class Placed:
    """도면 위에 얹은 한 구간."""
    kind: str = ""
    x0: float = 0.0
    y0: float = 0.0
    x1: float = 0.0
    y1: float = 0.0
    arc_start_mm: float = 0.0
    arc_end_mm: float = 0.0
    snapped: bool = False
    drift_mm: float = 0.0

    def as_dict(self):
        return asdict(self)


@dataclass
class MatchResult:
    placed: list = field(default_factory=list)
    robot_xy: tuple = (0.0, 0.0)
    robot_heading_deg: float = 0.0
    defects_xy: list = field(default_factory=list)
    total_arc_mm: float = 0.0
    max_drift_mm: float = 0.0
    snaps: int = 0
    note: str = ""

    def as_dict(self):
        d = asdict(self)
        d["placed"] = [p.as_dict() if hasattr(p, "as_dict") else p
                       for p in self.placed]
        return d


class Blueprint:
    """배관 도면. 꼭짓점이 곡관, 변이 직관이다.

    vertices : [(x, y), ...] mm.  vertices[0] 가 진입점.
    """

    def __init__(self, vertices, elbow_arc_mm=157.08):
        self.v = [tuple(map(float, p)) for p in vertices]
        self.elbow_arc = float(elbow_arc_mm)

    # ── 도면 자체 정보 ────────────────────────────────────────────
    def edge_len(self, i):
        (x0, y0), (x1, y1) = self.v[i], self.v[i + 1]
        return math.hypot(x1 - x0, y1 - y0)

    def edge_dir(self, i):
        (x0, y0), (x1, y1) = self.v[i], self.v[i + 1]
        return math.degrees(math.atan2(y1 - y0, x1 - x0))

    def turn_at(self, i):
        """꼭짓점 i 에서의 회전각(좌 +). i 는 1..n-2."""
        a = self.edge_dir(i - 1)
        b = self.edge_dir(i)
        return (b - a + 180.0) % 360.0 - 180.0

    def total_len(self):
        return sum(self.edge_len(i) for i in range(len(self.v) - 1))

    # ── 매칭 ──────────────────────────────────────────────────────
    def match(self, segments, defects=None):
        """로봇의 상대 경로를 도면에 얹는다.

        곡관 구간을 만날 때마다 다음 꼭짓점에 스냅한다. 직진 구간의 길이가
        도면 변 길이와 달라도 꼭짓점에서 맞춰지므로 오차가 누적되지 않는다.
        """
        res = MatchResult()
        ei = 0                       # 지금 걷고 있는 변
        along = 0.0                  # 그 변에서 진행한 거리
        arc = 0.0

        for seg in segments:
            kind = getattr(seg, "kind", None) or seg.get("kind")
            length = float(getattr(seg, "length_mm", None)
                           if hasattr(seg, "length_mm") else seg["length_mm"])
            if ei >= len(self.v) - 1:
                res.note = "도면 끝을 지났다 — 로봇 경로가 도면보다 길다"
                break

            if kind == "STRAIGHT":
                p0 = self._point(ei, along)
                remain = self.edge_len(ei) - along
                use = min(length, remain)
                along += use
                p1 = self._point(ei, along)
                res.placed.append(Placed(
                    kind="STRAIGHT", x0=p0[0], y0=p0[1], x1=p1[0], y1=p1[1],
                    arc_start_mm=arc, arc_end_mm=arc + length))
                arc += length
            else:
                # 곡관 = 다음 꼭짓점. 남은 변 길이만큼의 오차를 여기서 끊는다.
                drift = self.edge_len(ei) - along
                p0 = self._point(ei, along)
                v = self.v[ei + 1]
                res.placed.append(Placed(
                    kind="CURVE", x0=p0[0], y0=p0[1], x1=v[0], y1=v[1],
                    arc_start_mm=arc, arc_end_mm=arc + length,
                    snapped=True, drift_mm=drift))
                res.snaps += 1
                res.max_drift_mm = max(res.max_drift_mm, abs(drift))
                arc += length
                ei += 1
                along = 0.0

        res.total_arc_mm = arc
        if res.placed:
            last = res.placed[-1]
            res.robot_xy = (last.x1, last.y1)
        if ei < len(self.v) - 1:
            res.robot_heading_deg = self.edge_dir(ei)

        # 결함을 절대 좌표로
        if defects:
            for d in defects:
                a = float(getattr(d, "arc_mm", None)
                          if hasattr(d, "arc_mm") else d["arc_mm"])
                xy = self._xy_at_arc(res.placed, a)
                did = (getattr(d, "defect_id", None)
                       if hasattr(d, "defect_id") else d.get("defect_id", ""))
                roll = float(getattr(d, "roll_deg", None)
                             if hasattr(d, "roll_deg") else d.get("roll_deg", 0))
                res.defects_xy.append(dict(defect_id=did, x=xy[0], y=xy[1],
                                           arc_mm=a, roll_deg=roll))
        return res

    def _point(self, ei, along):
        (x0, y0), (x1, y1) = self.v[ei], self.v[ei + 1]
        L = max(self.edge_len(ei), 1e-9)
        t = max(0.0, min(1.0, along / L))
        return (x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)

    @staticmethod
    def _xy_at_arc(placed, arc):
        for p in placed:
            if p.arc_start_mm <= arc <= p.arc_end_mm:
                span = max(p.arc_end_mm - p.arc_start_mm, 1e-9)
                t = (arc - p.arc_start_mm) / span
                return (p.x0 + (p.x1 - p.x0) * t, p.y0 + (p.y1 - p.y0) * t)
        if placed:
            return (placed[-1].x1, placed[-1].y1)
        return (0.0, 0.0)
