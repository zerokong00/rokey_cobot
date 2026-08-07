"""[오프라인] 코스 기하(pipe/course.py) 검증 — 두 구현의 교차 대조.

repair_demo 의 해석식(ElbowCourse)과 real_map_demo 의 웨이포인트
(CenterLine)를 **같은 곡관 코스**에 놓고 s·접선·거리를 서로 대조한다.
서로 독립적으로 작성된 구현이라, 일치하면 양쪽 다 맞다고 볼 근거가 된다.

코스 = repair_demo 실물: 입구 직관(축 X, y=-150, x -343→0) → R150 90° 곡관
       → 출구 직관(축 Y, x=+150, y 0→343).

실행:  python3 test_course.py
"""

import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SON = HERE.parents[1]
sys.path.insert(0, str(SON))

from pipe import course as CM  # noqa: E402

S_IN, IN_Y, ARC_R, OUT_X, S_OUT, IR = 0.343, -0.150, 0.150, 0.150, 0.343, 0.050


def main():
    ok = []

    def check(name, cond, extra=""):
        ok.append(bool(cond))
        print(f"  [{len(ok):2d}] {name:<46} {'OK' if cond else 'FAIL'} {extra}")

    el = CM.ElbowCourse(s_in=S_IN, in_y=IN_Y, arc_r=ARC_R,
                        out_x=OUT_X, s_out=S_OUT, pipe_ir=IR)
    cl = CM.CenterLine([(-S_IN, IN_Y, 0.0), (OUT_X, IN_Y, 0.0),
                        (OUT_X, S_OUT, 0.0)], ARC_R).tabulate(ds=0.001)

    print("=" * 78)
    print("① 총 길이")
    check("해석식 총 길이 = 921.6mm",
          abs(el.total - 0.9216) < 1e-4, f"({el.total * 1000:.1f})")
    check("두 구현 총 길이 일치 (±0.1mm)",
          abs(el.total - cl.total) < 1e-4, f"(차 {abs(el.total - cl.total) * 1e6:.0f}µm)")

    print("② s(진행거리) 교차 대조 — 코스 위 24점")
    ss = np.linspace(0.005, el.total - 0.005, 24)
    worst = 0.0
    for s0 in ss:
        p, _ = cl.point_tangent(s0)
        s_el = el.s(p[0], p[1])
        worst = max(worst, abs(s_el - s0))
    check("전 구간 |s차| < 1mm", worst < 1e-3, f"(최대 {worst * 1000:.3f}mm)")

    print("③ 접선 교차 대조 — 같은 24점")
    worst = 0.0
    for s0 in ss:
        p, t_cl = cl.point_tangent(s0)
        _, tx, ty = el.dist_tangent(np.array(p[0]), np.array(p[1]))
        ang = math.degrees(math.acos(np.clip(
            float(tx) * t_cl[0] + float(ty) * t_cl[1], -1, 1)))
        worst = max(worst, ang)
    check("전 구간 접선각 차 < 0.5°", worst < 0.5, f"(최대 {worst:.3f}°)")

    print("④ 중심선 거리 — 코스에서 반경 방향으로 띄운 점")
    p0, t0 = cl.point_tangent(0.5)     # 곡관 안쪽 어딘가
    n0 = np.array([-t0[1], t0[0], 0.0])   # 평면 법선 방향
    for off in (0.010, 0.030, 0.049):
        q = p0 + n0 * off
        d_el, _, _ = el.dist_tangent(np.array(q[0]), np.array(q[1]))
        _, d_cl = cl.project(q)
        check(f"반경 {off * 1000:.0f}mm 띄움 → 두 구현 거리 일치",
              abs(float(d_el) - off) < 5e-4 and abs(d_cl - off) < 5e-4,
              f"(el {float(d_el) * 1000:.2f} / cl {d_cl * 1000:.2f})")

    print("⑤ confine(스파크 가둠) — 방향이 바깥을 향하는가")
    pts = np.array([[(-0.10), IN_Y + 0.020, 0.0],
                    [OUT_X - 0.02, 0.10, 0.030]])
    r, v, ir = el.confine(pts)
    check("내반경 반환 = 50mm", ir == IR)
    check("거리 = |오프셋|", abs(r[0] - 0.020) < 1e-6, f"({r[0] * 1000:.2f})")
    check("방향 단위벡터", np.allclose(np.linalg.norm(v, axis=1), 1.0))
    check("방향이 중심선에서 멀어지는 쪽",
          el.dist_tangent(np.array(pts[0, 0] + v[0, 0] * 1e-3),
                          np.array(pts[0, 1] + v[0, 1] * 1e-3))[0] > r[0] - 1e-9)

    print("⑥ CenterLine frame/radial — 시계각 규약 (180°=바닥)")
    _, _, u, w = cl.frame(0.1)
    check("직관에서 e_up = +Z", np.allclose(u, [0, 0, 1]))
    d = cl.radial(0.1, 180.0)
    check("180° = 바닥(−Z)", np.allclose(d, [0, 0, -1], atol=1e-9))
    check("clock_of 는 radial 의 역",
          abs(cl.clock_of(0.1, cl.radial(0.1, 135.0)) - 135.0) < 1e-6)

    print("=" * 78)
    print(f"전체 판정: {'통과' if all(ok) else '실패'}  ({sum(ok)}/{len(ok)})")
    return 0 if all(ok) else 1


if __name__ == "__main__":
    sys.exit(main())
