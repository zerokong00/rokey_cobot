"""[오프라인] 휠 크라운 검증 — 트레드가 관벽에 전폭으로 닿는가.

설계확정본 : 휠 Ø20 x 폭 15mm, **크라운 반경 50mm**

2026-08-04 까지 이 값이 `parts_meta.json` 으로 옮겨지지 않아 휠이 **원통**
으로 만들어졌다. Isaac Sim 담당자가 실측으로 찾았다.

    yongbin_drive_test.py:90
    # 휠은 크라운 없는 원통(트레드 폭 ±7.5mm, STL 실측)이라 곡면 내벽에
    # 트레드 양끝 에지 2점으로 닿는다. ... R_c=39.434mm

**이 결함은 Isaac Sim 이 필요 없다.** STL 을 읽어 트레드 프로파일을 재면
바로 나온다. 여기서 그것을 한다.

## 왜 크라운 반경이 하필 50mm 인가

관 내반경과 같은 값이다. 휠 축은 원주 방향이므로, 폭 위치 y 의 트레드 점은
관축에서 `sqrt(y^2 + (R_c + r(y))^2)` 만큼 떨어져 있다. 크라운 반경을 관
내반경과 같게 두면 이 값이 **폭 전체에서 정확히 50.0** 이 된다.

실행:  python3 test_wheel_crown.py
"""

import json
import math
import sys
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
SON = HERE.parents[1]

META = json.loads((SON / "spec" / "parts_meta.json").read_text())
BORE_R = META["pipe_id"] / 2.0
WHEEL_R = META["wheel_r"]
HALF_W = META["wheel_width"] / 2.0
CROWN_R = META.get("wheel_crown_r")


def tread_profile(mesh):
    """휠 축(Y) 을 따라가며 각 단면의 최대 반경을 잰다.

    ⚠ 임의 간격으로 표본하면 안 된다 — 메시 단면이 있는 y 에만 정점이 있어
    빈 구간이 nan 으로 나온다. **메시가 실제로 가진 y 값**을 쓴다.
    """
    v = mesh.vertices
    r = np.hypot(v[:, 0], v[:, 2])
    ys = np.unique(np.round(v[:, 1], 4))
    ys = ys[(ys >= -HALF_W - 1e-6) & (ys <= HALF_W + 1e-6)]
    out = []
    for y in ys:
        sel = np.abs(v[:, 1] - y) < 1e-4
        out.append(float(r[sel].max()))
    return ys, np.array(out)


def main():
    ok = []
    print("=" * 84)
    print("휠 크라운 — 트레드가 관벽에 전폭으로 닿는가")
    print("=" * 84)
    print(f"  관 내반경 {BORE_R}  휠 반경 {WHEEL_R}  트레드 폭 {HALF_W * 2}")
    print(f"  parts_meta 크라운 반경: "
          f"{CROWN_R if CROWN_R else '없음 ← 설계에는 50mm 이 있다'}")
    ok.append(CROWN_R is not None and abs(CROWN_R - BORE_R) < 1e-6)

    m = trimesh.load(SON / "legacy" / "meshes" / "wheel.stl")   # 보관 이동
    ys, prof = tread_profile(m)

    print(f"\n  STL 실측 트레드 프로파일")
    print(f"  {'y(mm)':>8} {'휠반경':>9} {'기대':>9} {'관축거리':>10} {'오차':>8}")
    want_drop = (CROWN_R - np.sqrt(np.maximum(CROWN_R ** 2 - ys ** 2, 0.0))
                 if CROWN_R else np.zeros_like(ys))
    want = WHEEL_R - want_drop
    r_centre = BORE_R - WHEEL_R              # 휠 중심의 관축 반경 = 40
    worst = 0.0
    for y, got, w in zip(ys, prof, want):
        axis_d = math.hypot(y, r_centre + got)
        err = abs(axis_d - BORE_R)
        worst = max(worst, err)
        print(f"  {y:8.2f} {got:9.3f} {w:9.3f} {axis_d:10.3f} {err:8.3f}")

    print(f"\n  관축거리가 폭 전체에서 {BORE_R:.1f} 이어야 전폭 밀착이다")
    good = worst < 0.05
    ok.append(good)
    print(f"  최대 오차 {worst:.3f}mm  {'OK — 전폭 밀착' if good else 'FAIL'}")

    print("\n" + "-" * 84)
    print("  [대조] 크라운 없는 원통이었다면")
    flat_axis = [math.hypot(y, 39.434 + WHEEL_R) for y in (0.0, HALF_W)]
    print(f"    휠 중심 반경 39.434mm 로 밀려 앉는다 (설계 {r_centre:.1f})")
    print(f"    중앙 y=0   관축거리 {flat_axis[0]:.3f}  → 벽에서 "
          f"{BORE_R - flat_axis[0]:.3f}mm 떠 있다")
    print(f"    끝  y=7.5  관축거리 {flat_axis[1]:.3f}  → 여기만 닿는다")
    print(f"    = 에지 2점 접촉. 하중이 모서리에 몰리고 예압 기하가 어긋난다")
    ok.append(abs(flat_axis[1] - BORE_R) < 0.01)

    print("\n" + "-" * 84)
    print("  파생값이 설계와 맞는가")
    rows = [
        ("휠 접촉 반경", META["pivot_r"] + META["arm_dr_nominal"] + WHEEL_R,
         BORE_R),
        ("암 기준 각도", META["arm_angle_nominal"], 44.4),
        ("암 각도 하한", META["arm_angle_compressed"], 33.4),
        ("암 각도 상한", META["arm_angle_extended"], 58.2),
    ]
    for name, got, want_v in rows:
        good = abs(got - want_v) < 0.05
        ok.append(good)
        print(f"    {name:<14} {got:9.3f}  설계 {want_v:6.1f}  "
              f"{'OK' if good else 'FAIL'}")

    print("\n  메시 건전성")
    print(f"    삼각형 {len(m.faces):,}  watertight={m.is_watertight}  "
          f"부피 {m.volume:.1f} mm³")
    ok.append(m.is_watertight and m.volume > 0)

    print("=" * 84)
    print(f"전체 판정: {'통과' if all(ok) else '실패'}  ({sum(ok)}/{len(ok)})")
    return 0 if all(ok) else 1


if __name__ == "__main__":
    sys.exit(main())
