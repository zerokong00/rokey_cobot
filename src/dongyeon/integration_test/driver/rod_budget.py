"""[공용] 용접봉 예산 — ROS/Isaac 비의존 순수 로직.

설계문서 v3 §8 그대로: 결함 하나를 메우는 데 드는 용접봉 길이를 체적에서
역산해 잔량을 깎고, 다음 수리 뒤에도 예비량이 못 남으면 **소진**으로
판정한다. 소진 사유는 관 단절과 이 소진 둘뿐이다(§5.3 절차 7, "임무 규칙
8") — real_map_demo.py/repair_demo.py 의 기존 RETURN 상태(관 단절 전용)를
그대로 재사용할 수 있도록 **recall 신호만** 낸다. 복귀 주행 자체는 FSM 몫.

  L_req = (V_defect × α) / A_rod                              (§8.1)

치수를 모르는 호출자(현재 real_map_demo.py 의 DEFECTS 는 위치·시계각만
갖고 길이·폭·깊이가 없다)를 위해 **근사 소모량**(`default_repair_mm`)도
같이 둔다 — `WeldSequencer.k["rod_per_mm3"]` 근사와 같은 층위다. 두 경로
모두 같은 잔량 하나를 깎으므로 부분적으로만 계측되는 임무에서도 예산
추적이 끊기지 않는다.

🚨 **예비량(reserve_mm)을 반드시 둘 것.** 잔량이 정확히 0 이 될 때까지
   쓰면 다음 결함이 조금이라도 크면 용접 도중 끊긴다 — 그 상태로는
   §8.3 "즉시 복귀"를 지킬 수 없다(이미 팔이 나가 있다). 예비량은 항상
   *쓰기 전에* 남을지 검사하는 하한이다.
"""

import math
from dataclasses import dataclass, asdict

ROD_DIAMETER_MM = 2.0
ROD_AREA_MM2 = math.pi * (ROD_DIAMETER_MM / 2.0) ** 2   # §8.1 A_rod
VOLUME_MARGIN_FACTOR = 1.2                                # §8.1 α (스패터 손실·덧살)
HOLE_VOLUME_FACTOR = 1.3                                   # §8.1 구멍 체적 보정
ROD_COIL_CAPACITY_MM = 1880.0                              # §8.2 코일 수용 한계

DEFAULTS = dict(
    total_mm=ROD_COIL_CAPACITY_MM,
    reserve_mm=10.0,           # 소진 판정 예비량 — repair_decision.py 와 동일값
    default_repair_mm=100.0,   # 치수 미계측 결함의 예상 소모(§8.1 예시: 구멍 d8/t4 ≈100mm)
)


def required_length_mm(defect_class, measurement, rod_area_mm2=ROD_AREA_MM2,
                       volume_margin_factor=VOLUME_MARGIN_FACTOR,
                       hole_volume_factor=HOLE_VOLUME_FACTOR):
    """§8.1 그대로 — 결함 체적에서 소모 길이(mm)를 역산한다.

    치수가 부족하면 None(호출자는 default_repair_mm 로 물러난다).
    pipe_inspect_demo/repair_decision.py 의 같은 함수와 값이 동일하다 —
    ROS 패키지에 의존하지 않으려고 이 파일 안에 다시 둔 것뿐이다.
    """
    m = measurement or {}
    cls = str(defect_class or "").strip().lower()
    if cls in ("crack", "크랙", "균열"):
        length_mm, width_mm, depth_mm = (m.get("length_mm"), m.get("width_mm"),
                                         m.get("defect_depth_mm"))
        if not all(isinstance(v, (int, float)) and v > 0
                  for v in (length_mm, width_mm, depth_mm)):
            return None
        volume_mm3 = length_mm * width_mm * depth_mm
    elif cls in ("hole", "구멍"):
        diameter_mm, wall_mm = m.get("diameter_mm"), m.get("pipe_wall_thickness_mm")
        if not all(isinstance(v, (int, float)) and v > 0
                  for v in (diameter_mm, wall_mm)):
            return None
        volume_mm3 = math.pi * (diameter_mm / 2.0) ** 2 * wall_mm * hole_volume_factor
    else:
        return None
    return volume_mm3 * volume_margin_factor / rod_area_mm2


@dataclass
class RodBudgetState:
    used_mm: float = 0.0
    n_repairs: int = 0
    exhausted: bool = False
    reason: str = ""

    def as_dict(self):
        return asdict(self)


class RodBudget:
    """용접봉 잔량 하나를 추적해 소진(=복귀) 신호를 낸다.

    사용 순서(호출자 FSM 쪽):
      1. 결함을 만날 때마다 `can_repair(required_mm)` 로 이번 걸 해도
         되는지 물어본다 — 거짓이면 수리하지 말고 §8.3 복귀로 보낸다.
      2. 실제로 용접했으면 `consume(required_mm)` 로 잔량을 깎는다.
      3. `s.exhausted` (또는 `can_repair()` 의 기본 예측)를 매 스텝
         주행 FSM 의 recall 조건에 물린다(driver.control.DriveController
         의 `recall=` 인자와 같은 자리).
    """

    def __init__(self, params=None):
        self.k = dict(DEFAULTS)
        if params:
            self.k.update({a: b for a, b in params.items() if a in self.k})
        self.s = RodBudgetState()

    @property
    def remaining_mm(self):
        return max(0.0, self.k["total_mm"] - self.s.used_mm)

    def can_repair(self, required_mm=None):
        """이 결함을 수리해도 예비량(reserve_mm)이 남는가.

        required_mm 을 안 주면(치수 미계측) default_repair_mm 로 가늠한다 —
        "다음 결함도 평균 크기려니" 하는 보수적 사전 판단이다.
        """
        need = self.k["default_repair_mm"] if required_mm is None else float(required_mm)
        return (self.remaining_mm - need) >= self.k["reserve_mm"]

    def consume(self, mm):
        """용접 하나가 실제로 쓴 길이를 반영하고 최신 상태를 반환한다."""
        self.s.used_mm += max(0.0, float(mm))
        self.s.n_repairs += 1
        self._refresh_exhausted()
        return self.s

    def mark_exhausted_if_needed(self, next_required_mm=None):
        """다음 결함을 만난 시점에 호출 — 그 결함을 감당 못 하면 소진 확정.

        `consume()` 이후의 사전 판정(§8.3 "소진되면 즉시 복귀")과 별개로,
        다음 결함의 **실측 소요량을 아는 순간**(치수가 나온 뒤) 다시 검사할
        수 있게 분리해 뒀다 — 평균값으로는 통과했는데 실제 결함이 더 커서
        예비량을 까먹는 경우를 여기서 잡는다.
        """
        if not self.can_repair(next_required_mm):
            self.s.exhausted = True
            self.s.reason = (f"용접봉 소진 — 잔량 {self.remaining_mm:.0f}mm, "
                             f"필요 {self.k['default_repair_mm'] if next_required_mm is None else next_required_mm:.0f}mm, "
                             f"예비 {self.k['reserve_mm']:.0f}mm 미만")
        return self.s.exhausted

    def _refresh_exhausted(self):
        if not self.can_repair() and not self.s.exhausted:
            self.s.exhausted = True
            self.s.reason = (f"용접봉 소진 — 잔량 {self.remaining_mm:.0f}mm "
                             f"(예비 {self.k['reserve_mm']:.0f}mm 미만)")
