"""[오프라인] water_fx 기하 검증 — Isaac 없이 순수 numpy 로.

시각 물 층(--fluid)의 중심선·수체 메시·흐름 순환·낙수 궤적이 수식대로인지
본다. USD 조립(WaterFX 클래스)은 Isaac 실기 몫이고 여기서는 기하 함수만.

특히 **면 방향**을 검사한다 — 전 면이 바깥향이어야 관 안 카메라에서
backface 로 걸러져 검출을 안 가린다(양면이면 결함이 파랗게 물들어 실패).
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from pipe import water_fx                                    # noqa: E402

FAIL = 0


def check(name, ok, detail=""):
    global FAIL
    print(f"  [{'OK' if ok else 'FAIL'}] {name}"
          + (f" — {detail}" if detail else ""))
    if not ok:
        FAIL += 1


# repair_demo.py 의 코스 상수
S_IN = S_OUT = 0.343
IN_Y = -0.150
ARC_R = OUT_X = 0.150
PIPE_IR = 0.050
R = PIPE_IR - 0.002       # 데모가 넘기는 값(관벽 2mm 안쪽)
LEVEL = -0.010            # 현재 수위

# ── 1. 중심선 — 데모의 path_dist_tangent 와 같은 기하인가 ────────────
pts, tans = water_fx.elbow_centerline(S_IN, IN_Y, ARC_R, OUT_X, S_OUT)
check("표본이 충분", len(pts) > 80, f"{len(pts)}점")
check("시작 = 입구 직관 끝", np.allclose(pts[0], [-S_IN, IN_Y, 0.0]))
check("끝 = 출구 직관 끝", np.allclose(pts[-1], [OUT_X, S_OUT, 0.0], atol=1e-9))
check("전 구간 z=0", np.allclose(pts[:, 2], 0.0))
check("접선 단위벡터", np.allclose(np.linalg.norm(tans, axis=1), 1.0))
_gap = np.linalg.norm(np.diff(pts, axis=0), axis=1)
check("끊긴 데 없음 (표본 간격 ≤ 12mm)", _gap.max() <= 0.012,
      f"최대 {_gap.max() * 1000:.1f}mm")
_total = float(np.sum(_gap))
check("총 길이 ≈ 직관+호+직관", abs(_total - (S_IN + ARC_R * math.pi / 2
                                        + S_OUT)) < 0.005,
      f"{_total * 1000:.1f}mm")


def dist_to_axis(p):
    """repair_demo.path_dist_tangent 와 같은 3갈래 최소거리."""
    px, py = p[:, 0], p[:, 1]
    d1 = np.hypot(px - np.clip(px, -S_IN, 0.0), py - IN_Y)
    ang = np.clip(np.arctan2(py, px), -np.pi / 2, 0.0)
    d2 = np.hypot(px - ARC_R * np.cos(ang), py - ARC_R * np.sin(ang))
    d3 = np.hypot(px - OUT_X, py - np.clip(py, 0.0, S_OUT))
    return np.minimum(np.minimum(d1, d2), d3)


check("중심선이 관 축 위에 있다", dist_to_axis(pts).max() < 1e-9,
      f"최대 이탈 {dist_to_axis(pts).max() * 1e6:.3f}µm")

# ── 2. 수면 띠 ───────────────────────────────────────────────────────
P, C, I = water_fx.surface_ribbon(pts, tans, LEVEL, R)
check("면 인덱스 유효", max(I) < len(P) and min(I) >= 0)
check("면 크기 합치", sum(C) == len(I))
check("전부 사각형", set(C) == {4})
check("면 수 = 표본−1", len(C) == len(pts) - 1, f"{len(C)}면")
# 🔑 **부피가 없어야 한다** — 로봇(휠 반경 40mm)을 감싸지도, 관 안으로 들어온
#    카메라를 가두지도 못한다. 전 정점이 수위 평면 위에 정확히 놓인다.
check("두께 0 — 전 정점이 수위 평면", np.allclose(P[:, 2], LEVEL),
      f"z 범위 {P[:, 2].min() * 1000:+.2f}~{P[:, 2].max() * 1000:+.2f}mm")
_d = dist_to_axis(P)
check("관 밖으로 안 샌다", _d.max() <= R + 1e-9,
      f"최대 {_d.max() * 1000:.2f}mm ≤ {R * 1000:.0f}")
check("관벽(50mm)을 안 뚫는다", _d.max() < PIPE_IR,
      f"여유 {(PIPE_IR - _d.max()) * 1000:.1f}mm")
check("수면 반폭 = √(r²−level²)",
      abs(water_fx.half_width(LEVEL, R) - math.sqrt(R * R - LEVEL * LEVEL))
      < 1e-12, f"{water_fx.half_width(LEVEL, R) * 1000:.1f}mm")

# 로봇이 잠기지 않는지 — 휠 최외곽(반경 40mm)이 수면 아래로 안 파묻힌다
check("수면이 로봇을 감싸지 않는다 (두께 0)",
      P[:, 2].max() - P[:, 2].min() == 0.0)

# ── 3. 면 방향 — 전부 +z (위에서 수면으로 보인다) ───────────────────
o, bad = 0, 0
for cnt in C:
    v = [P[I[o + t]] for t in range(cnt)]
    o += cnt
    n = np.cross(v[1] - v[0], v[2] - v[0])
    nn = np.linalg.norm(n)
    if nn < 1e-15 or n[2] / nn < 0.99:
        bad += 1
check("전 면이 +z 향 (수면)", bad == 0, f"어긋난 면 {bad}개")

# ── 4. 수위를 바꾸면 물의 양이 따라간다 ─────────────────────────────
def area_frac(level, r=PIPE_IR):
    a = math.acos(max(-1.0, min(1.0, -level / r)))
    return (a - math.sin(a) * math.cos(a)) / math.pi


check("충수율 수식 검산 (0→50%, +r→100%)",
      abs(area_frac(0.0) - 0.5) < 1e-12 and abs(area_frac(PIPE_IR) - 1.0) < 1e-9)
check("현재 수위 −10mm = 37.4%", abs(area_frac(-0.010) - 0.3739) < 1e-3,
      f"{area_frac(-0.010):.1%}")
check("옛 수위 +20mm = 74.8% (사용자가 본 '3/4')",
      abs(area_frac(0.020) - 0.7477) < 1e-3, f"{area_frac(0.020):.1%}")
check("수위를 올리면 수면이 넓어진다",
      water_fx.half_width(0.0, R) > water_fx.half_width(LEVEL, R),
      f"{water_fx.half_width(LEVEL, R) * 2000:.1f} → "
      f"{water_fx.half_width(0.0, R) * 2000:.1f}mm")

# ── 5. 흐름 순환 ─────────────────────────────────────────────────────
s = water_fx.advect_s(np.array([0.010]), 0.1, -0.855, _total)
check("역류 전진 + 순환", abs(s[0] - ((0.010 - 0.0855) % _total)) < 1e-12,
      f"{s[0] * 1000:.1f}mm")
check("순방향도 순환",
      abs(water_fx.advect_s(np.array([_total - 0.01]), 0.1, +0.855,
                            _total)[0] - ((_total - 0.01 + 0.0855) % _total))
      < 1e-12)

# ── 6. 낙수 (토리첼리) ───────────────────────────────────────────────
V0 = water_fx.torricelli(LEVEL, -PIPE_IR)
check("토리첼리 √(2gh), h=40mm", abs(V0 - math.sqrt(2 * 9.81 * 0.040)) < 1e-12,
      f"{V0:.3f} m/s")
check("수위가 낮으면 느리게 샌다",
      water_fx.torricelli(-0.010, -PIPE_IR)
      < water_fx.torricelli(0.020, -PIPE_IR))
check("t=0 은 구멍 자리", water_fx.drop_z(0.0, -0.054, V0) == -0.054)
z1, z2 = water_fx.drop_z(0.1, -0.054, V0), water_fx.drop_z(0.2, -0.054, V0)
check("단조 낙하 + 가속", (-0.054 - z1) < (z1 - z2))
T = water_fx.fall_time(0.45, V0)
check("낙하 주기 역산 일치",
      abs((-0.054) - water_fx.drop_z(T, -0.054, V0) - 0.45) < 1e-9,
      f"T={T:.3f}s")

# ── 7. USD 조립 — 인스턴서 회전이 항등인가 (usd-core 로, Isaac 불필요) ──
# 🚨 회귀 방지. `Vt.QuathArray(n)` 은 **길이 0 인 (0,0,0,0)** 을 채운다.
#    그대로 두면 회전 행렬이 무너져 인스턴스가 화면 전체로 퍼지고 배경이
#    통째로 파랗게 뜬다(실기: 누수 물방울 Leak 만 그 상태였다).
try:
    from pxr import Gf, Usd, Vt                              # noqa: E402
except ImportError:
    print("  [건너뜀] usd-core 없음 — USD 조립 검사 생략")
else:
    check("기본 QuathArray 는 길이 0 (그래서 명시 초기화가 필요)",
          Gf.Quath(Vt.QuathArray(1)[0]).GetLength() == 0.0)


    st = Usd.Stage.CreateInMemory()
    fx = water_fx.WaterFX(
        st, pts, tans, level=LEVEL, radius=R, flow_v=-0.855,
        hole_xyz=(-0.022, IN_Y, -PIPE_IR), v_out=V0)
    for nm in ("Flow", "Leak"):
        q = st.GetPrimAtPath(f"/World/WaterFX/{nm}") \
              .GetAttribute("orientations").Get()
        bad = [i for i, v in enumerate(q)
               if abs(Gf.Quath(v).GetLength() - 1.0) > 1e-3]
        check(f"{nm} 회전이 전부 단위 쿼터니언", not bad,
              f"{len(q)}개 중 어긋난 것 {len(bad)}개")
        p = st.GetPrimAtPath(f"/World/WaterFX/{nm}") \
              .GetAttribute("positions").Get()
        fin = all(all(math.isfinite(c) for c in v) for v in p)
        check(f"{nm} 위치가 전부 유한", fin, f"{len(p)}개")
    # 물방울은 구멍 아래로만 떨어진다 (관 위로 솟지 않는다)
    lp = st.GetPrimAtPath("/World/WaterFX/Leak") \
           .GetAttribute("positions").Get()
    check("누수 물방울이 구멍 아래에만", max(v[2] for v in lp) <= -PIPE_IR,
          f"최고 z {max(v[2] for v in lp) * 1000:+.1f}mm")

print()
if FAIL:
    raise SystemExit(f"[결과] 실패 {FAIL}건")
print("[결과] water_fx 전 항목 통과")
