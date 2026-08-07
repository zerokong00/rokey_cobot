"""[오프라인] 속도 유지 조절기(driver/control.SpeedGovernor) 검증.

가상 플랜트: 측정 각속도 = 지령 × 부하율. 부하율 < 1 이 유체 항력으로
휠이 주저앉는 상황이다(실측 최저 0.37). 조절기가 지령을 올려 측정을
목표로 되돌리는지, 부하가 걷히면 다시 내려오는지 본다.

실행:  python3 test_governor.py
"""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SON = HERE.parents[1]
sys.path.insert(0, str(SON))

from driver.control import SpeedGovernor  # noqa: E402

TARGET = 286.5      # deg/s (0.05 m/s / r10mm)
DT = 0.05           # 20Hz 판정


def run(gov, load, steps):
    meas = None
    for _ in range(steps):
        meas = gov.cmd * load
        gov.update(meas, DT)
    return meas


def main():
    ok = []

    def check(name, cond, extra=""):
        ok.append(bool(cond))
        print(f"  [{len(ok):2d}] {name:<46} {'OK' if cond else 'FAIL'} {extra}")

    print("=" * 78)
    print("① 무부하 — 지령이 목표에 머문다")
    g = SpeedGovernor(TARGET)
    m = run(g, 1.0, 100)
    check("측정 = 목표 (±1%)", abs(m - TARGET) < 0.01 * TARGET, f"({m:.1f})")
    check("부스트 = 1.00", abs(g.boost - 1.0) < 0.02, f"({g.boost:.3f})")

    print("② 부하 0.8 — 지령을 올려 목표 속도를 회복")
    g = SpeedGovernor(TARGET)
    m = run(g, 0.8, 200)     # 10초
    check("측정이 목표로 수렴 (±4%)", abs(m - TARGET) < 0.04 * TARGET,
          f"({m:.1f} / 목표 {TARGET:.1f})")
    check("지령 = 목표/0.8 = 1.25배", abs(g.boost - 1.25) < 0.05,
          f"({g.boost:.3f})")
    check("상한 미포화", not g.saturated)

    print("③ 부하 0.5 — 상한 1.5배에서 멈춘다 (contactOffset 안전)")
    g = SpeedGovernor(TARGET)
    m = run(g, 0.5, 400)
    check("지령이 1.5배에서 포화", g.saturated and abs(g.boost - 1.5) < 1e-6,
          f"({g.boost:.3f})")
    check("측정은 0.75×목표에 그침 — 정직하게 한계 보고",
          abs(m - 0.75 * TARGET) < 0.02 * TARGET, f"({m:.1f})")

    print("④ 부하 해제 — 지령이 도로 내려온다 (과속 방지)")
    g = SpeedGovernor(TARGET)
    run(g, 0.8, 200)          # 부스트 상태에서
    m = run(g, 1.0, 200)      # 부하 제거
    check("측정이 목표로 복귀 (±4%)", abs(m - TARGET) < 0.04 * TARGET,
          f"({m:.1f})")
    check("부스트 해소", abs(g.boost - 1.0) < 0.05, f"({g.boost:.3f})")

    print("⑤ 역부하 1.3 (배후 유속이 미는 상황) — 하한까지 낮춘다")
    g = SpeedGovernor(TARGET)
    m = run(g, 1.3, 300)
    check("측정이 목표로 수렴 (±5%)", abs(m - TARGET) < 0.05 * TARGET,
          f"({m:.1f})")
    check("지령 = 목표/1.3 = 0.77배", abs(g.boost - 1 / 1.3) < 0.05,
          f"({g.boost:.3f})")

    print("⑥ reset — 정지 뒤 재출발은 목표부터")
    g = SpeedGovernor(TARGET)
    run(g, 0.5, 400)
    g.reset()
    check("reset 후 지령 = 목표", g.cmd == TARGET and not g.saturated)

    print("=" * 78)
    print(f"전체 판정: {'통과' if all(ok) else '실패'}  ({sum(ok)}/{len(ok)})")
    return 0 if all(ok) else 1


if __name__ == "__main__":
    sys.exit(main())
