"""[오프라인] 맵의 내반경 프로파일을 잰다 — 수정 전후 대조용. Isaac 불필요.

    python3 tools/check_map.py [맵.usd] [floor2|floor1]

메시별로 따로 재는 것이 핵심이다. 합쳐서 재면 **더 좁은 사본이 가려진다**
(중앙값이 정상으로 나온다) — 실제로 그 때문에 곡관 끼임의 원인을 한참 못 찾았다.
"""
import math
import sys
from pathlib import Path as _P

import numpy as np
from pxr import Usd, UsdGeom

sys.path.insert(0, str(_P(__file__).resolve().parent))
from fix_map import FLOOR_CORNERS, Path, mesh_prims       # noqa: E402


def caster(T):
    V0, E1, E2 = T[:, 0], T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]

    def cast(o, d):
        d = np.asarray(d, float); d = d / np.linalg.norm(d)
        h = np.cross(d, E2); a = np.einsum("ij,ij->i", E1, h)
        ok = np.abs(a) > 1e-12
        f = np.zeros_like(a); f[ok] = 1.0 / a[ok]
        s = o - V0
        u = f * np.einsum("ij,ij->i", s, h); q = np.cross(s, E1)
        v = f * (q @ d); t = f * np.einsum("ij,ij->i", E2, q)
        g = ok & (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9) & (t > 1e-9)
        return float(t[g].min()) if g.any() else float("inf")
    return cast


def main(src, floor, step=60.0):
    stage = Usd.Stage.Open(src)
    P = Path(FLOOR_CORNERS[floor])
    M = mesh_prims(stage, floor)
    C = {}
    for name, (prim, off) in M.items():
        m = UsdGeom.Mesh(prim)
        pts = np.array(m.GetPointsAttr().Get(), dtype=np.float64) + off
        idx = np.array(m.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
        C[name] = caster(pts[idx])
    names = list(C)
    print(f"=== {_P(src).name}  {floor}  중심선 {P.total:.0f}mm ===")
    print("  s(mm)  " + "  ".join(f"{n[:11]:>11s}" for n in names) + "   최소")
    worst = (1e9, 0.0)
    for s in np.arange(0, P.total + 1e-6, step):
        p, t = P.pt(s)
        u = np.array([0., 0., 1.]) - t * t[2]
        if np.linalg.norm(u) < 1e-6:
            u = np.array([1., 0., 0.]) - t * t[0]
        u /= np.linalg.norm(u); w = np.cross(u, t)
        row, mins = [], []
        for n in names:
            h = np.array([C[n](p, math.cos(math.radians(30 * k)) * u
                               + math.sin(math.radians(30 * k)) * w)
                          for k in range(12)])
            fin = np.isfinite(h)
            if fin.sum() >= 6:
                row.append(f"{h[fin].min():5.1f}/{h[fin].max():5.1f}")
                mins.append(h[fin].min())
            else:
                row.append("-")
        mn = min(mins) if mins else float("nan")
        if mins and mn < worst[0]:
            worst = (mn, s)
        print(f"{s:7.0f}  " + "  ".join(f"{r:>11s}" for r in row)
              + f"   {mn:5.1f}" + ("  ★" if mins and mn < 47.0 else ""))
    print(f"--- 최악 내반경 {worst[0]:.1f}mm @ s={worst[1]:.0f}mm "
          f"(DN100 기준 50.0, 로봇 휠 접지 40.0) ---")


if __name__ == "__main__":
    _src = (sys.argv[1] if len(sys.argv) > 1
            else str(_P.home() / "Downloads"
                     / "restroom_pipe150_final_fixed.usd"))
    main(_src, sys.argv[2] if len(sys.argv) > 2 else "floor2")
