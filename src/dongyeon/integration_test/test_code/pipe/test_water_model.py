"""[오프라인] 물 모델(pipe/water_model.py) 검증 — 수위·수압·잠김율 수식.

repair_demo 인라인 수식을 모듈로 뽑으면서(2026-08-06) 값이 안 변했는지,
경계 조건이 맞는지 본다. DN100(내반경 50mm) 기준.

실행:  python3 test_water_model.py
"""

import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SON = HERE.parents[1]
sys.path.insert(0, str(SON))

from pipe import water_model as WM  # noqa: E402

IR = 0.050          # DN100 내반경
R_ENV = 0.045       # 로봇 휠 포락 반경


def main():
    ok = []

    def check(name, cond, extra=""):
        ok.append(bool(cond))
        print(f"  [{len(ok):2d}] {name:<44} {'OK' if cond else 'FAIL'} {extra}")

    print("=" * 78)
    print("① 수위 — 충수율 → 수면 z")
    for fill, z_mm in [(0.5, 0.0), (1 / 3, -16.67), (2 / 3, +16.67),
                       (0.70, +20.0), (0.0, -50.0), (1.0, +50.0)]:
        z = WM.level_z(fill, IR) * 1000
        check(f"fill {fill:.2f} → z {z_mm:+.2f}mm", abs(z - z_mm) < 0.01,
              f"({z:+.2f})")

    print("② 수압·토리첼리 — 바닥 구멍(z=−50mm)")
    lv = WM.level_z(0.5, IR)
    p = WM.hydrostatic_pa(lv, -IR)
    v = WM.torricelli_ms(lv, -IR)
    check("1/2 충수: 수두 50mm → 490.5 Pa", abs(p - 490.5) < 0.1,
          f"({p:.1f})")
    check("1/2 충수: 유출속도 0.99 m/s", abs(v - 0.990) < 0.005, f"({v:.3f})")
    check("수면 위 지점은 압력 0",
          WM.hydrostatic_pa(lv, +0.010) == 0.0
          and WM.torricelli_ms(lv, +0.010) == 0.0)
    p23 = WM.hydrostatic_pa(WM.level_z(2 / 3, IR), -IR)
    check("수위 올리면 수압 증가 (2/3 > 1/2)", p23 > p, f"({p23:.1f})")
    p_riser = WM.hydrostatic_pa(2.49, 0.0)
    check("실전 맵 입상관 2.49m → 24.4 kPa",
          abs(p_riser - 24427) < 10, f"({p_riser:.0f} Pa)")

    print("③ 잠김율 — 활꼴 면적비 (로봇 반경 45mm)")
    for lv_m, frac in [(0.0, 0.50), (+R_ENV, 1.0), (-R_ENV, 0.0),
                       (+IR, 1.0), (-IR, 0.0)]:
        f = WM.submerged_fraction(lv_m, R_ENV)
        check(f"수위 {lv_m * 1000:+.0f}mm → 잠김 {frac:.0%}",
              abs(f - frac) < 1e-9, f"({f:.3f})")
    f13 = WM.submerged_fraction(WM.level_z(1 / 3, IR), R_ENV)
    f23 = WM.submerged_fraction(WM.level_z(2 / 3, IR), R_ENV)
    # 기대값 검산: α=arccos(16.67/45)=1.1914rad → (α−sinαcosα)/π = 0.2697
    check("1/3 충수 → 잠김 27%", abs(f13 - 0.2697) < 0.001, f"({f13:.4f})")
    check("2/3 충수 → 잠김 73% (대칭)", abs(f13 + f23 - 1.0) < 1e-9,
          f"({f23:.4f})")

    print("④ 부력·항력 — 설계 §12.3 확정값 재현")
    fb = WM.buoyancy_n(1.8e-4, 1.0)
    check("만관 부력 1.77N", abs(fb - 1.766) < 0.01, f"({fb:.3f})")
    fd = WM.drag_n(0.855, 2.32, 2.7e-3)
    check("만관 항력 2.29N", abs(fd - 2.290) < 0.01, f"({fd:.3f})")
    check("1/2 충수면 항력 절반",
          abs(WM.drag_n(0.855, 2.32, 2.7e-3 * 0.5) - fd / 2) < 1e-9)
    check("항력 부호 = 상대속도 부호", WM.drag_n(-0.855, 2.32, 2.7e-3) < 0)

    print("=" * 78)
    print(f"전체 판정: {'통과' if all(ok) else '실패'}  ({sum(ok)}/{len(ok)})")
    return 0 if all(ok) else 1


if __name__ == "__main__":
    sys.exit(main())
