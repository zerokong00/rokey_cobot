"""[공용] 코스(배관 중심선) 기하 — 두 데모가 공유하는 단일 출처.

repair_demo(해석식 3구간)와 real_map_demo(웨이포인트 CenterLine)에 따로
구현돼 있던 것을 합쳤다 (2026-08-06). 물 채우기·유동장·유체력·스파크 가둠·
진행거리가 전부 이 기하를 쓰므로, 코스 계산 수정은 여기 한 곳에서 한다.
검증은 `test_code/pipe/test_course.py` — **두 구현을 같은 곡관에 놓고
서로 대조**한다(교차 검증).

왜 구현이 두 개인가 (하나로 안 합친 이유):
- `ElbowCourse` — 해석식. 수만 점 벡터 연산이 공짜라 **물 입자 채우기·
  재투입(15Hz×수천 입자)** 에 쓴다. 단, 직관–90°곡관–직관 전용.
- `CenterLine` — 임의 웨이포인트 + 곡관 반경. 실전 맵처럼 코너가 많은
  코스용. 투영은 표본표(tabulate)로 하므로 점 몇 개(로봇 링크)에는 싸지만
  수만 점을 매 스텝 돌리기엔 비싸다.
같은 곡관에서 두 구현의 s·접선이 일치하는 것을 시험이 보증한다.
"""

import math

import numpy as np


class ElbowCourse:
    """직관(축 X) → 90° 곡관 → 직관(축 Y) 해석식 코스. z=0 평면.

    입구 직관 y=in_y (x: −s_in→0), 곡관 반경 arc_r, 출구 직관 x=out_x
    (y: 0→s_out). repair_demo 의 pipe_elbow_lr150 코스가 이것이다.
    모든 메서드는 numpy 배열을 그대로 받는다(벡터화).
    """

    def __init__(self, s_in, in_y, arc_r, out_x, s_out, pipe_ir):
        self.s_in, self.in_y, self.arc_r = s_in, in_y, arc_r
        self.out_x, self.s_out, self.pipe_ir = out_x, s_out, pipe_ir
        self.s_arc = arc_r * math.pi / 2.0
        self.total = s_in + self.s_arc + s_out

    def dist_tangent(self, px, py):
        """xy 점 → (중심선까지 xy 거리, 접선x, 접선y). 접선 = 진행 방향."""
        d1 = np.hypot(px - np.clip(px, -self.s_in, 0.0), py - self.in_y)
        ang = np.clip(np.arctan2(py, px), -np.pi / 2, 0.0)
        d2 = np.hypot(px - self.arc_r * np.cos(ang),
                      py - self.arc_r * np.sin(ang))
        d3 = np.hypot(px - self.out_x, py - np.clip(py, 0.0, self.s_out))
        d = np.stack([d1, d2, d3])
        which = np.argmin(d, axis=0)
        tx = np.where(which == 0, 1.0, np.where(which == 1, -np.sin(ang), 0.0))
        ty = np.where(which == 0, 0.0, np.where(which == 1, np.cos(ang), 1.0))
        return d.min(axis=0), tx, ty

    def point(self, px, py):
        """xy 점 → 가장 가까운 중심선 점 (cx, cy). 중심선은 z=0 에 있다."""
        ang = np.clip(np.arctan2(py, px), -np.pi / 2, 0.0)
        cx = np.stack([np.clip(px, -self.s_in, 0.0),
                       self.arc_r * np.cos(ang),
                       np.full_like(px, self.out_x)])
        cy = np.stack([np.full_like(py, self.in_y),
                       self.arc_r * np.sin(ang),
                       np.clip(py, 0.0, self.s_out)])
        which = np.argmin(np.hypot(px - cx, py - cy), axis=0)
        i = np.arange(px.size) if px.ndim else 0
        return cx[which, i], cy[which, i]

    def s(self, px, py):
        """코스 진행거리(m). 입구 끝이 0. 곡관에서 x 변위는 주행량이 아니다."""
        px = np.atleast_1d(np.asarray(px, float))
        py = np.atleast_1d(np.asarray(py, float))
        ang = np.clip(np.arctan2(py, px), -np.pi / 2, 0.0)
        d1 = np.hypot(px - np.clip(px, -self.s_in, 0.0), py - self.in_y)
        d2 = np.hypot(px - self.arc_r * np.cos(ang),
                      py - self.arc_r * np.sin(ang))
        d3 = np.hypot(px - self.out_x, py - np.clip(py, 0.0, self.s_out))
        which = np.argmin(np.stack([d1, d2, d3]), axis=0)
        s = np.where(which == 0, np.clip(px, -self.s_in, 0.0) + self.s_in,
                     np.where(which == 1,
                              self.s_in + self.arc_r * (ang + np.pi / 2),
                              self.s_in + self.s_arc
                              + np.clip(py, 0.0, self.s_out)))
        return float(s[0]) if s.size == 1 else s

    def confine(self, pts):
        """(N,3) 점 → (중심선 거리, 바깥 단위방향, 관 내반경) — 스파크 가둠용.

        🔑 직선 원기둥 근사는 안 된다 — 결함이 곡관 입구 22mm 앞이라
        스패터가 곧장 곡관 구간으로 넘어간다. 관 끝에서는 중심선 점이 끝점에
        고정되어 거리가 커지므로 저절로 되튕긴다(관 밖으로 안 샌다).
        """
        cx, cy = self.point(pts[:, 0], pts[:, 1])
        v = np.stack([pts[:, 0] - cx, pts[:, 1] - cy, pts[:, 2]], axis=1)
        r = np.linalg.norm(v, axis=1)
        return r, v / np.maximum(r, 1e-12)[:, None], self.pipe_ir


class CenterLine:
    """웨이포인트 + 곡관 반경 → 임의 코스 중심선 (real_map_demo 에서 이동).

    코너마다 반경 R 로 필렛을 넣어 line/arc 구간열을 만든다. 투영은
    `tabulate()` 로 구운 표본표를 쓴다 — 표 없이 매 호출 파이썬 루프를
    돌리면 **투영이 물리보다 비싸진다**(메인 루프가 스텝마다 링·팁·본체를
    투영한다).
    """

    def __init__(self, corners_world, R):
        C = [np.asarray(c, float) for c in corners_world]
        self.segs, cur = [], C[0]
        for i in range(1, len(C) - 1):
            B = C[i]
            t1 = B - C[i - 1]; t1 /= np.linalg.norm(t1)
            t2 = C[i + 1] - B; t2 /= np.linalg.norm(t2)
            p_in, p_out = B - t1 * R, B + t2 * R
            ctr = p_in + (p_out - B)
            L = float(np.linalg.norm(p_in - cur))
            if L > 1e-9:
                self.segs.append(("line", cur, t1, L))
            ang = float(np.arccos(np.clip(np.dot(t1, t2), -1.0, 1.0)))
            self.segs.append(("arc", ctr, (p_in - ctr) / R, t1, R, R * ang))
            cur = p_out
        L = float(np.linalg.norm(C[-1] - cur))
        self.segs.append(("line", cur, (C[-1] - cur) / L, L))
        self.cum = np.cumsum([s[-1] for s in self.segs])
        self.total = float(self.cum[-1])

    def point_tangent(self, s):
        s = min(max(float(s), 0.0), self.total)
        i = min(int(np.searchsorted(self.cum, s, side="right")),
                len(self.segs) - 1)
        u = s - (self.cum[i - 1] if i else 0.0)
        sg = self.segs[i]
        if sg[0] == "line":
            return sg[1] + sg[2] * u, sg[2]
        _, ctr, e1, t1, R, _ = sg
        a = u / R
        p = ctr + R * (math.cos(a) * e1 + math.sin(a) * t1)
        t = -math.sin(a) * e1 + math.cos(a) * t1
        return p, t / np.linalg.norm(t)

    def frame(self, s):
        """(점, 접선, e_up, e_side). 시계각은 e_up→e_side, **180° = 바닥.**

        e_up 은 월드 +Z 의 접선 수직 성분이다. 수직관에서는 퇴화하므로 +X 로
        떨어뜨린다(그 구간에는 결함을 두지 않는다).
        """
        p, t = self.point_tangent(s)
        u = np.array([0.0, 0.0, 1.0]) - t * t[2]
        if np.linalg.norm(u) < 1e-6:
            u = np.array([1.0, 0.0, 0.0]) - t * t[0]
        u /= np.linalg.norm(u)
        return p, t, u, np.cross(u, t)

    def radial(self, s, clock_deg):
        _, _, u, w = self.frame(s)
        a = math.radians(clock_deg)
        return math.cos(a) * u + math.sin(a) * w

    def clock_of(self, s, d):
        """방향벡터 d 의 시계각(도). radial() 의 역이다."""
        _, _, u, w = self.frame(s)
        return math.degrees(math.atan2(float(np.dot(d, w)),
                                       float(np.dot(d, u))))

    def tabulate(self, ds=0.002):
        """조밀 표본표를 굽는다. project() 전에 반드시 부를 것."""
        self.tab_s = np.arange(0.0, self.total + 1e-9, ds)
        self.tab_p = np.array([self.point_tangent(x)[0] for x in self.tab_s])
        self.tab_ds = ds
        return self

    def project(self, p, hint=None, win=0.30):
        """가장 가까운 s 와 그때의 중심선 거리. hint 가 있으면 그 둘레만 본다."""
        if hint is None:
            lo_i, hi_i = 0, len(self.tab_s)
        else:
            lo_i = max(0, int((hint - win) / self.tab_ds))
            hi_i = min(len(self.tab_s), int((hint + win) / self.tab_ds) + 1)
            if hi_i - lo_i < 2:
                lo_i, hi_i = 0, len(self.tab_s)
        d2 = np.sum((self.tab_p[lo_i:hi_i] - p) ** 2, axis=1)
        k = lo_i + int(np.argmin(d2))
        # 표 간격 안쪽을 한 번 더 좁힌다(정렬 판정이 0.4mm 를 다툰다)
        lo = max(0.0, self.tab_s[k] - self.tab_ds)
        hi = min(self.total, self.tab_s[k] + self.tab_ds)
        ss = np.linspace(lo, hi, 9)
        P = np.array([self.point_tangent(x)[0] for x in ss])
        k2 = int(np.argmin(np.sum((P - p) ** 2, axis=1)))
        return float(ss[k2]), float(np.linalg.norm(P[k2] - p))
