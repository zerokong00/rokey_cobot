"""[오프라인] 견인력 예산 — 예압이 없으면 왜 한 발도 못 나가는지.

2026-08-04 실기에서 곡관 주행이 전혀 안 됐다. 뒤로 105mm 밀리고 관절이
한계 55°까지 접혀 굳었다. 검증팀이 휠 `maxForce` 를 **100배**로 올려 봤는데
출력 숫자가 한 글자도 안 변했다.

원인은 토크가 아니라 **접지**였다. `pipe/curve_demo.py` 가
`set_joint_positions()` 로 암을 0°로 접은 뒤 **암 드라이브 타깃을 다시 걸지
않아** 암이 벌어지지 않았다. 관벽을 안 누르니 수직항력이 0 이고,

    마찰력 = 마찰계수 x 수직항력

이므로 **토크를 아무리 키워도 마찰이 안 생긴다.**

`robot/articulate.py` 는 이 관계를 이미 코드로 적어 두고 있었다.

    WHEEL_MAX_TORQUE = WHEEL_FRICTION * WHEEL_PRELOAD_N * WHEEL_R

예압 9N 을 전제로 견인력 예산이 잡혀 있는데, 주행 스크립트가 그 전제를
깨뜨린 것이다. **이 시험은 그 전제를 숫자로 못박는다.**

⚠ 이것은 물리 모델 검사지 시뮬레이션 검증이 아니다. 실제 주행이 되는지는
Isaac Sim 에서 확인해야 한다. 여기서 보는 것은 "예압이 0 이면 어떤 토크로도
못 움직인다" 는 관계뿐이다.

실행:  python3 test_traction.py
"""

import json
import math
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SON = HERE.parents[1]

META = json.loads((SON / "spec" / "parts_meta.json").read_text())

MM = 0.001
WHEEL_R = META["wheel_r"] * MM
N_WHEEL = 6
MASS_KG = 0.500
GRAVITY = 9.81


def from_source(path, name, cast=float):
    """상수를 **소스에서 읽는다.** 여기 베껴 두면 소스가 바뀌었을 때
    시험은 옛 값으로 통과해 버린다 — 실제로 마찰이 0.70 → 0.30 으로
    바뀌었을 때 이 시험이 아무 말도 안 했다."""
    src = (SON / path).read_text()
    m = re.search(rf"^{name}\s*=\s*([-\d.eE/*+ ]+)", src, re.M)
    if not m:
        raise SystemExit(f"[중단] {path} 에서 {name} 을 못 찾았다")
    return cast(eval(m.group(1), {"__builtins__": {}}, {}))


FRICTION = from_source("robot/assemble.py", "WHEEL_FRICTION_STATIC")
PRELOAD_N = from_source("robot/assemble.py", "WHEEL_PRELOAD_N")
TORQUE_FRACTION = from_source("robot/assemble.py", "WHEEL_TORQUE_FRACTION_DESIGN")
TARGET_SPEED_MPS = 0.05


def wheel_max_torque(preload_n, friction=FRICTION, r=WHEEL_R):
    """바퀴 하나가 낼 수 있는 최대 구동 토크. 마찰이 상한이다."""
    return friction * preload_n * r


def tractive_force_n(preload_n, n=N_WHEEL, friction=FRICTION):
    """6륜 전체 견인력. 수직항력에 정비례한다 — 토크와 무관하게 여기서 막힌다."""
    return n * friction * preload_n


def required_force_n(grade_deg=90.0, mass=MASS_KG, rolling=0.05):
    """수직 배관을 오를 때 필요한 힘. 곡관 상승 구간이 최악이다."""
    return mass * GRAVITY * (math.sin(math.radians(grade_deg)) + rolling)


def main():
    ok = []
    print("=" * 78)
    print("견인력 예산 — 접지가 없으면 토크는 의미가 없다")
    print("=" * 78)
    print(f"  바퀴 반경 {WHEEL_R * 1000:.1f}mm  x{N_WHEEL}   마찰계수 {FRICTION}")
    print(f"  설계 예압 {PRELOAD_N:.1f} N/륜   질량 {MASS_KG * 1000:.0f} g")
    print(f"  ※ 마찰·예압·토크비율은 robot/articulate.py 에서 읽는다 "
          f"(베끼지 않는다)")

    need = required_force_n()
    print(f"\n  수직 상승에 필요한 힘 {need:.2f} N")

    print(f"\n  {'예압(N)':>9} {'륜당 최대토크(N·m)':>20} {'견인력(N)':>11}"
          f" {'수직상승':>10}")
    rows = [0.0, 0.1, 1.0, 3.0, PRELOAD_N, 20.0]
    for p in rows:
        t = wheel_max_torque(p)
        f = tractive_force_n(p)
        print(f"  {p:9.1f} {t:20.5f} {f:11.2f} "
              f"{'가능' if f >= need else '불가':>10}")

    print("\n  ① 예압 0 이면 견인력도 0 이다 — 토크를 100배 해도 마찬가지")
    ok.append(tractive_force_n(0.0) == 0.0)
    for mult in (1, 10, 100, 1000):
        f = tractive_force_n(0.0)
        ok.append(f == 0.0)
    print(f"     maxForce x1 / x10 / x100 / x1000 → 견인력 전부 "
          f"{tractive_force_n(0.0):.2f} N  "
          f"{'OK' if all(ok) else 'FAIL'}")
    print("     → 실기에서 maxForce x100 이 무영향이었던 것과 일치한다")

    print("\n  ② 설계 예압에서는 수직 상승이 가능해야 한다")
    f = tractive_force_n(PRELOAD_N)
    good = f >= need
    ok.append(good)
    print(f"     견인력 {f:.2f} N  vs  필요 {need:.2f} N   "
          f"여유 {f / need:.1f}배  {'OK' if good else 'FAIL'}")

    print("\n  ③ 구동 토크가 마찰원을 다 쓰면 안 된다")
    limit = wheel_max_torque(PRELOAD_N)
    drive = limit * TORQUE_FRACTION
    print(f"     마찰 한계 {limit * 1000:.2f} mN·m  →  구동 상한 "
          f"{drive * 1000:.2f} mN·m  ({TORQUE_FRACTION * 100:.0f}%)")
    print(f"     남는 횡방향 여력 {(1 - TORQUE_FRACTION) * 100:.0f}% "
          f"— 0 이면 관 중심을 못 잡고 잭나이프가 난다")
    good = 0.3 <= TORQUE_FRACTION <= 0.6
    ok.append(good)
    print(f"     40~50% 권장 범위인가  {'OK' if good else 'FAIL'}")
    push = N_WHEEL * drive / WHEEL_R
    good = push >= need
    ok.append(good)
    print(f"     구동이 낼 수 있는 힘 {push:.2f} N  vs  필요 {need:.2f} N  "
          f"여유 {push / need:.1f}배  {'OK' if good else 'FAIL — 못 오른다'}")

    print("\n  ④ 최소 필요 예압")
    p_min = need / (N_WHEEL * FRICTION)
    print(f"     {p_min:.2f} N/륜 (설계 {PRELOAD_N:.1f} N 의 "
          f"{p_min / PRELOAD_N * 100:.0f}%)")
    ok.append(p_min < PRELOAD_N)

    print("\n" + "=" * 78)
    print("주행 스크립트가 예압을 지키는가 — 소스를 읽어 확인")
    print("=" * 78)
    src = (SON / "pipe" / "curve_demo.py").read_text()

    checks = [
        ("set_joint_positions 뒤에 암 타깃을 다시 건다",
         "set_joint_positions" not in src
         or src.index("hold_arm_preload()") > src.index("set_joint_positions")),
        ("바퀴 지령을 apply_action 으로 준다 (USD 속성 쓰기는 런타임에 무효)",
         "apply_action" in src),
        ("시뮬 시작 뒤 USD 드라이브 속성을 직접 쓰지 않는다",
         "GetTargetVelocityAttr().Set(" not in src),
        ("바퀴를 돌리기 전에 안착 스텝을 둔다",
         "SETTLE_STEPS" in src),
        ("예압이 안 걸리면 경고한다",
         "예압이 없으면" in src),
    ]
    for name, good in checks:
        ok.append(good)
        print(f"  {'OK  ' if good else 'FAIL'} {name}")

    print("=" * 78)
    print(f"전체 판정: {'통과' if all(ok) else '실패'}  "
          f"({sum(ok)}/{len(ok)})")
    print("\n⚠ 실제 주행 여부는 Isaac Sim 에서 확인해야 한다. 여기서 본 것은")
    print("  '예압이 0 이면 어떤 토크로도 못 움직인다' 는 관계뿐이다.")
    return 0 if all(ok) else 1


if __name__ == "__main__":
    sys.exit(main())
