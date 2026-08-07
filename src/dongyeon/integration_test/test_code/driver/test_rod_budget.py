"""[오프라인] 용접봉 예산(driver/rod_budget.RodBudget) 검증.

설계문서 v3 §8 시나리오를 그대로 재현한다: 결함을 차례로 수리하며 잔량을
깎다가, 예비량 아래로 내려가면 소진(=복귀 신호)이 뜨는지 본다.

실행:  python3 test_rod_budget.py
"""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SON = HERE.parents[1]
sys.path.insert(0, str(SON))

from driver.rod_budget import RodBudget, required_length_mm  # noqa: E402


def main():
    ok = []

    def check(name, cond, extra=""):
        ok.append(bool(cond))
        print(f"  [{len(ok):2d}] {name:<52} {'OK' if cond else 'FAIL'} {extra}")

    print("=" * 78)
    print("① 초기 상태 — 꽉 찬 코일(1880mm), 소진 아님")
    b = RodBudget()
    check("remaining_mm = 1880", abs(b.remaining_mm - 1880.0) < 1e-6)
    check("소진 아님", not b.s.exhausted)
    check("기본 크기 결함 수리 가능", b.can_repair())

    print("② 결함 3개(각 100mm) 수리 — 잔량이 정확히 깎인다")
    b = RodBudget()
    for i in range(3):
        b.consume(100.0)
    check("used_mm = 300", abs(b.s.used_mm - 300.0) < 1e-6, f"({b.s.used_mm})")
    check("remaining_mm = 1580", abs(b.remaining_mm - 1580.0) < 1e-6)
    check("n_repairs = 3", b.s.n_repairs == 3)
    check("아직 소진 아님", not b.s.exhausted)

    print("③ §8.1 공식 — 구멍 d8mm/t4mm ≈ 100mm (설계문서 예시값)")
    req = required_length_mm("hole", {"diameter_mm": 8.0, "pipe_wall_thickness_mm": 4.0})
    check("100mm 근처(예시 표와 일치)", req is not None and abs(req - 99.96) < 1.0,
          f"({req:.1f}mm)" if req else "(None)")

    print("④ §8.1 공식 — 크랙 40×1.5×2.0mm ≈ 46mm (설계문서 예시값)")
    req = required_length_mm("crack", {"length_mm": 40.0, "width_mm": 1.5,
                                       "defect_depth_mm": 2.0})
    check("46mm 근처(예시 표와 일치)", req is not None and abs(req - 45.86) < 1.0,
          f"({req:.1f}mm)" if req else "(None)")

    print("⑤ 치수 없는 클래스/미지원 클래스는 None — default_repair_mm 로 물러날 몫")
    check("치수 없음 → None", required_length_mm("hole", {}) is None)
    check("미지원 클래스 → None",
          required_length_mm("pipe_disconnection", {}) is None)

    print("⑥ 예비량 아래로 내려가면 소진 확정 (총량 120mm, 예비 10mm)")
    b = RodBudget(dict(total_mm=120.0, reserve_mm=10.0, default_repair_mm=50.0))
    check("1차 수리 전 가능", b.can_repair(50.0))
    b.consume(50.0)               # 잔량 70
    check("1차 후 아직 소진 아님", not b.s.exhausted, f"(잔량 {b.remaining_mm:.0f})")
    check("2차 수리 전 가능", b.can_repair(50.0))
    b.consume(50.0)               # 잔량 20 → 20-50=-30 < 10 이므로 다음은 불가
    check("2차 후 잔량 20mm", abs(b.remaining_mm - 20.0) < 1e-6)
    check("2차 후 소진 확정(다음 50mm 결함은 예비량 침범)", b.s.exhausted,
          f"({b.s.reason})")
    check("소진 사유 문자열에 잔량이 들어간다", "20" in b.s.reason, f"({b.s.reason})")

    print("⑦ 소진 뒤에는 더 작은 결함도 거절 — 예비량 자체가 없다")
    check("남은 20mm 로는 15mm 결함도 불가(예비 10 확보 못 함)",
          not b.can_repair(15.0))
    check("남은 20mm 로 5mm 결함은 가능(20-5=15 ≥ 예비 10)", b.can_repair(5.0))

    print("⑧ mark_exhausted_if_needed — 다음 결함 실측 후 재검사")
    b = RodBudget(dict(total_mm=120.0, reserve_mm=10.0, default_repair_mm=50.0))
    b.consume(60.0)                              # 잔량 60, 평균(50)으로는 통과
    check("평균 크기 기준으로는 아직 통과", b.can_repair())
    exhausted = b.mark_exhausted_if_needed(next_required_mm=55.0)  # 60-55=5 < 예비10
    check("실측(55mm)이 더 크면 그 자리에서 소진 확정", exhausted)
    check("사유에 실제 필요량(55)이 들어간다", "55" in b.s.reason, f"({b.s.reason})")

    print("⑨ 임무 규칙 8 재현 — 결함을 계속 수리하다 자연 소진되는 전체 시나리오")
    b = RodBudget(dict(total_mm=350.0, reserve_mm=10.0, default_repair_mm=100.0))
    repaired, returned_at = 0, None
    for i in range(10):
        if not b.can_repair():
            returned_at = i
            break
        b.consume(100.0)
        repaired += 1
    check("3건 수리 후 복귀(350mm 로는 3건×100=300, 남은 50<필요 100)",
          repaired == 3 and returned_at == 3,
          f"(수리 {repaired}건, {returned_at}번째 결함에서 복귀)")

    print("=" * 78)
    print(f"전체 판정: {'통과' if all(ok) else '실패'}  ({sum(ok)}/{len(ok)})")
    return 0 if all(ok) else 1


if __name__ == "__main__":
    sys.exit(main())
