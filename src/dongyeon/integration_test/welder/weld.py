"""[공용] 용접 시퀀스 + 3겹 검증 — ROS 비의존 순수 로직.

수리 성공을 "검출기가 못 찾음" 하나로 판정하면 자기충족이 된다. 검출기가
조명·각도 때문에 놓쳐도 성공으로 기록되기 때문이다. 세 겹으로 막는다.

  ① 정렬 오차 조건   토치가 결함 위치에 안 맞으면 결함 프림을 그대로 둔다
                     → 검증 촬영에서 여전히 검출 → 실패
                     (이게 없으면 성공률이 항상 100%가 된다. 설계 7.2)

  ② Depth 프로파일   홈이 실제로 메워졌는지 깊이로 확인한다
                     결함이면 파여서 반경이 크고, 메워졌으면 관벽과 같거나
                     살짝 돌출한다. 검출기 거짓 음성을 잡는 겹이다
                     (설계에 없던 항목. 검출기 하나에만 기대지 않으려고 추가)

  ③ 정답 대조        임무 종료 시 defect_spawner 의 ground truth 와 전수 감사
                     (설계 7.2 "정찰기 결함 목록과 수리기 처리 목록 전수 대조")

②가 필요한 이유가 하나 더 있다. 실패 시나리오는 "결함 유지 + 비드를 어긋난
위치에 표시"인데, 그 비드가 카메라 시선에서 결함 앞을 가리면 **실패인데
미검출**이 되어 우연히 성공으로 기록된다. Depth 프로파일은 그 경우도 잡는다.
"""

import math
import sys
from dataclasses import dataclass, asdict, field
from pathlib import Path

import numpy as np

# 🔑 결함 실측 크기 → 용접봉 소요량(§8.1 L_req) 판정에 driver/rod_budget.py
#    의 순수 함수를 재사용한다(같은 §8 공식을 두 곳에 따로 두지 않는다).
#    이 파일 자체는 sys.path 설정 없이도(예: test_code/welder/ 에서 단독
#    import 될 때도) 동작해야 하므로 여기서 직접 경로를 잡는다. driver/ 가
#    없는 배포본에서도 죽지 않도록 실패하면 조용히 None 으로 둔다 —
#    rod_budget 을 안 주면(step 의 기본값) 이 경로 자체를 안 타므로 무해하다.
try:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from driver.rod_budget import required_length_mm as _required_length_mm
except ImportError:
    _required_length_mm = None

APPROACH = "APPROACH"
RECHECK = "RECHECK"
ALIGN = "ALIGN"
ARC = "ARC"
COOL = "COOL"
REPOSITION = "REPOSITION"
VERIFY = "VERIFY"
RETRY = "RETRY"
SUCCESS = "SUCCESS"
FAILED = "FAILED"

DEFAULTS = dict(
    align_tol_mm=1.5,
    approach_tol_mm=5.0,
    arc_s=2.0,
    cool_base_s=5.0,
    # 용접 지점은 관벽에 있고 로봇 바로 옆이다. 화각 140° 카메라도 전방
    # 18.2mm 안쪽의 관벽은 못 본다(원리적 제약, 링 위치와 무관).
    # 그래서 냉각 후 로봇을 물려야 검증 촬영이 가능하다.
    verify_backoff_mm=120.0,
    backoff_speed_mps=0.05,
    cool_per_mm_s=0.8,
    max_retry=1,

    bore_r_mm=50.0,
    band_px=36,
    median_px=3,
    min_deep_px=6,
    depth_defect_mm=0.6,
    depth_repaired_mm=0.25,
    profile_min_px=60,

    rod_per_mm3=1.0,
    rod_total_mm=1880.0,

    # §8 용접봉 예산 — rod_budget 을 넘겼을 때만 쓰인다(밑 RECHECK 참고).
    pipe_wall_mm=4.0,          # 설계확정본 v3 §8 확정값
    default_repair_mm=100.0,  # 치수 계측 실패 시 근사(§8.1 예시: d8/t4 ≈100mm)
)


@dataclass
class WeldResult:
    state: str = APPROACH
    aligned: bool = False
    align_err_mm: float = 0.0
    profile_ok: bool = False
    profile_peak_mm: float = 0.0
    detector_clear: bool = False
    verdict: str = ""
    retry: int = 0
    backoff_mm: float = 0.0
    rod_used_mm: float = 0.0
    rod_required_mm: float = 0.0
    need_return: bool = False
    reason: str = ""
    layers: dict = field(default_factory=dict)

    def as_dict(self):
        return asdict(self)


def radial_profile(depth, intr, centre_px, band_px=40):
    """용접 지점 주변의 벽면 반경 지도(mm)를 구한다.

    카메라가 관 축 위에 있으므로 화소의 실제 거리 D 와 입사각 theta 로부터
    그 점의 축 거리(반경)는 D*sin(theta) 다. 판정기의 오프셋 계산과 같은 식이다.
    무효 화소는 NaN 으로 둔다.
    """
    d = np.asarray(depth, dtype=np.float64)
    h, w = d.shape
    cx, cy = centre_px
    x0, x1 = int(max(0, cx - band_px)), int(min(w, cx + band_px))
    y0, y1 = int(max(0, cy - band_px)), int(min(h, cy + band_px))
    sub = d[y0:y1, x0:x1]
    if sub.size == 0:
        return np.zeros((0, 0))

    yy, xx = np.mgrid[y0:y1, x0:x1]
    theta = np.hypot(xx - intr["ppx"], yy - intr["ppy"]) / intr["f_fish"]
    radius_mm = sub * np.sin(theta) * 1000.0
    return np.where(np.isfinite(radius_mm) & (sub > 0), radius_mm, np.nan)


class WeldSequencer:
    """결함 하나에 대한 용접 시퀀스와 검증."""

    def __init__(self, params=None):
        self.k = dict(DEFAULTS)
        if params:
            self.k.update({a: b for a, b in params.items() if a in self.k})
        self.r = WeldResult()
        self._t = 0.0
        self._backed = 0.0

    # ── ① 정렬 ──────────────────────────────────────────────────
    def align_error(self, torch_pos, defect_pos):
        """토치 끝과 결함 중심의 거리(mm)."""
        a = np.asarray(torch_pos, dtype=float)
        b = np.asarray(defect_pos, dtype=float)
        return float(np.linalg.norm(a - b))

    # ── ② Depth 프로파일 ────────────────────────────────────────
    def _smoothed_radius(self, depth, intr, centre_px):
        """profile_verdict 와 defect_diameter_mm 이 공유하는 반경지도.

        분위수를 쓰면 안 된다. 크랙은 폭 1.6mm 라 화면에서 몇 화소뿐이고
        창 전체의 99분위에 묻혀 사라진다(실측으로 확인). 중앙값 필터로
        단발 잡음만 걷어내고 최댓값을 본다.
        """
        from scipy.ndimage import median_filter

        rad = radial_profile(depth, intr, centre_px,
                             band_px=int(self.k["band_px"]))
        n_valid = int(np.isfinite(rad).sum())
        if n_valid < self.k["profile_min_px"]:
            return None, n_valid
        bore = self.k["bore_r_mm"]
        filled = np.where(np.isfinite(rad), rad, bore)
        sm = median_filter(filled, size=int(self.k["median_px"]))
        return sm, n_valid

    def profile_verdict(self, depth, intr, centre_px):
        """벽면이 파여 있는가, 메워졌는가.

        관벽보다 바깥이면 파인 것(결함), 같거나 안쪽이면 메워진 것(수리).
        검출기 결과와 무관하게 기하로 판정하므로 거짓 음성을 잡는다.
        """
        sm, n_valid = self._smoothed_radius(depth, intr, centre_px)
        if sm is None:
            return None, 0.0, f"프로파일 표본 부족 ({n_valid})"

        bore = self.k["bore_r_mm"]
        peak = float(sm.max() - bore)

        # 한 점이 아니라 이어진 면적이어야 결함이다.
        deep = int((sm - bore >= self.k["depth_defect_mm"] * 0.5).sum())
        if peak >= self.k["depth_defect_mm"] and deep >= self.k["min_deep_px"]:
            return False, peak, f"여전히 파임 {peak:+.2f} mm ({deep}px)"
        if peak <= self.k["depth_repaired_mm"]:
            return True, peak, f"메워짐 {peak:+.2f} mm"
        return None, peak, f"판정 애매 {peak:+.2f} mm ({deep}px)"

    # ── 결함 실측 크기 (§8.1 L_req 의 diameter_mm 입력) ───────────
    def defect_diameter_mm(self, depth, intr, centre_px):
        """파인 화소들의 실면적을 적분해 결함의 등가 직경(mm)을 잰다.

        🚨 **관통 구멍에서는 그 화소의 실측 거리(depth)를 화소당 실면적
           계산에 쓰면 안 된다.** 구멍 난 화소는 결함까지의 거리가 아니라
           **구멍 뒤 배경**까지의 거리를 반환한다 — 실측으로 확인: 같은
           ø38.1mm 관통 개구인데 카메라 각도 차이만으로 한쪽은 1151mm,
           다른 쪽은 36mm(32배)로 튀었다. 화소당 실면적은 depth 가 아니라
           **정상 벽(bore_r_mm) 을 기준으로 한 기하학적 기대 거리**로 잰다
           — "이 각도의 화소가 정상 벽 위치에 있었다면 커버했을 실공간
           크기"이므로 구멍 뒤 배경 거리에 오염되지 않는다. depth(sm) 는
           오직 "이 화소가 파였는가"(이진 판정)에만 쓰고, 파인 정도의
           크기 자체는 면적 계산에 넣지 않는다 — 관통 구멍은 "파인 깊이"
           라는 개념 자체가 없다(뒤가 뚫려 있다).
        치수를 못 재면(표본 부족·파인 화소 없음) (0.0, 0.0, 0) — 호출자는
        `default_repair_mm` 근사로 물러난다.
        """
        sm, n_valid = self._smoothed_radius(depth, intr, centre_px)
        if sm is None:
            return 0.0, 0.0, 0
        bore = self.k["bore_r_mm"]
        deep_mask = (sm - bore) >= (self.k["depth_defect_mm"] * 0.5)
        if not deep_mask.any():
            return 0.0, 0.0, 0

        h_win, w_win = sm.shape
        cx, cy = centre_px
        bp = int(self.k["band_px"])
        x0 = int(max(0, cx - bp))
        y0 = int(max(0, cy - bp))
        yy, xx = np.mgrid[y0:y0 + h_win, x0:x0 + w_win]
        theta = np.hypot(xx - intr["ppx"], yy - intr["ppy"]) / intr["f_fish"]
        theta = np.clip(theta, 1e-6, np.pi / 2 * 0.999)
        d_bore_mm = bore / np.sin(theta)      # 그 화소의 "정상 벽" 거리
        px_w_mm = d_bore_mm / intr["f_fish"]  # 화소 한 칸의 실공간 폭(mm)
        area_mm2 = float(np.sum((px_w_mm ** 2)[deep_mask]))
        diam_mm = math.sqrt(4.0 * area_mm2 / math.pi) if area_mm2 > 0 else 0.0
        return diam_mm, area_mm2, int(deep_mask.sum())

    def defect_diameter_from_area_px(self, area_px, centre_px, intr):
        """OpenCV 밝기 검출 면적(area_px, `find_wall_hole` 의 결과)을 mm² 로
        환산해 등가직경을 낸다 — **관통 구멍은 이 경로를 쓴다.**

        `defect_diameter_mm`(Depth 프로파일) 은 관통 구멍에서 못 믿는다
        (구멍 뒤 배경까지의 거리를 보게 되어 카메라 각도만으로 값이 수십
        배 널뛴다 — 실측: 같은 ø38.1mm 결함인데 결함1 딥스 기반 875mm vs
        결함2 5mm). 반면 OpenCV 화소 개수는 두 결함이 2,049px/2,105px로
        거의 일치했다 — **화소 개수는 카메라 각도에 안정적**이라는 뜻.
        화소 하나가 가리는 실공간 크기만 기하로(정상 벽 반경 기준) 환산해
        곱한다(`defect_diameter_mm` 과 같은 식, `bore/sin(theta)`).
        """
        if area_px is None or area_px <= 0:
            return 0.0, 0.0
        bore = self.k["bore_r_mm"]
        cx, cy = centre_px
        theta = math.hypot(cx - intr["ppx"], cy - intr["ppy"]) / intr["f_fish"]
        theta = max(min(theta, math.pi / 2 * 0.999), 1e-6)
        d_bore_mm = bore / math.sin(theta)
        px_w_mm = d_bore_mm / intr["f_fish"]
        area_mm2 = float(area_px) * (px_w_mm ** 2)
        diam_mm = math.sqrt(4.0 * area_mm2 / math.pi) if area_mm2 > 0 else 0.0
        return diam_mm, area_mm2

    # ── §8.3 용접봉 사전 판정 ──────────────────────────────────────
    def rod_precheck(self, rod_budget, hole_area_px=None, centre_px=None,
                     intr=None, depth=None):
        """이 결함을 감당할 잔량이 있는지, **용접을 시도하기 전에** 묻는다.

        RECHECK 이 이 메서드를 그대로 쓴다(상태 전이는 RECHECK 쪽 몫) —
        real_map_demo.py 처럼 `step()` 의 시간축 상태머신 전체를 돌리지
        않고 판정 로직만 재사용하고 싶은 호출자를 위해 독립 메서드로 뺐다.

        `hole_area_px`(OpenCV `find_wall_hole` 의 `area_px`) 가 있으면
        그걸로 잰다(관통 구멍류 — 위 `defect_diameter_from_area_px` 참고).
        없고 `depth` 만 있으면 Depth 프로파일로(비관통 크랙류에는 유효).
        둘 다 없으면 `default_repair_mm` 근사로 물러난다.

        returns (required_mm, ok) — `ok` 는 rod_budget 이 None 이면
        항상 True(판정 자체를 생략).
        """
        req = None
        if hole_area_px is not None and centre_px is not None and intr is not None:
            diam, area = self.defect_diameter_from_area_px(
                hole_area_px, centre_px, intr)
        elif depth is not None and intr is not None and centre_px is not None:
            diam, area, npix = self.defect_diameter_mm(depth, intr, centre_px)
        else:
            diam = 0.0
        if diam > 0 and _required_length_mm is not None:
            req = _required_length_mm("hole", {
                "diameter_mm": diam,
                "pipe_wall_thickness_mm": self.k["pipe_wall_mm"]})
        if req is None:
            req = self.k["default_repair_mm"]
        ok = rod_budget is None or rod_budget.can_repair(req)
        return req, ok

    # ── 시퀀스 ──────────────────────────────────────────────────
    def cool_time(self, defect_size_mm):
        return self.k["cool_base_s"] + self.k["cool_per_mm_s"] * defect_size_mm

    def step(self, dt, torch_pos=None, defect_pos=None, defect_size_mm=6.0,
             depth=None, intr=None, centre_px=None, detected=None,
             rod_budget=None, hole_area_px=None):
        """한 스텝. 상태에 따라 필요한 입력만 쓴다.

        `rod_budget` — driver.rod_budget.RodBudget 인스턴스(선택). 주면
        RECHECK 에서 이 결함을 감당할 수 있는지 **용접을 시도하기 전에**
        먼저 묻고(§5.3 절차 4~7, §8.3 "즉시 복귀"), 감당 못 하면 토치를
        뻗지 않고 바로 `need_return=True` 로 반환한다. 안 주면(기본값)
        이 판정 자체를 건너뛴다 — 기존 호출자(test_weld.py 등)는 그대로
        동작한다.
        """
        r = self.r
        k = self.k
        self._t += dt

        if r.state == APPROACH:
            if torch_pos is None or defect_pos is None:
                r.reason = "위치 미수신"
                return r
            e = self.align_error(torch_pos, defect_pos)
            r.align_err_mm = e
            if e <= k["approach_tol_mm"]:
                r.state, r.reason, self._t = RECHECK, "접근 완료", 0.0
            else:
                r.reason = f"접근 중 {e:.1f} mm"

        elif r.state == RECHECK:
            # 수리기가 목표 지점에서 자체 카메라로 결함을 재검출한다.
            # 여기서 안 보이면 애초에 수리 대상이 아니다(설계 7.1 의 "반전").
            if detected is None:
                r.reason = "재확인 대기"
            elif detected:
                # 🔑 여기서 실측 치수 → 소요량 → 잔량 판정까지 끝낸다.
                #    ALIGN/EXTEND 로 넘어가면 이미 팔이 나간 것이라, 그 뒤에
                #    부족을 알아도 §8.3 "즉시 복귀"를 못 지킨다(토치를 다시
                #    수납해야 하는 시간이 더 든다). 반드시 *쓰기 전에* 묻는다.
                req, ok = self.rod_precheck(rod_budget, hole_area_px=hole_area_px,
                                            centre_px=centre_px, intr=intr,
                                            depth=depth)
                r.rod_required_mm = req
                if not ok:
                    r.state = FAILED
                    r.need_return = True
                    r.verdict = "용접봉 부족"
                    r.reason = (f"필요 {req:.0f}mm, 잔량 "
                                f"{rod_budget.remaining_mm:.0f}mm — 시도 안 "
                                f"함, 복귀")
                else:
                    r.state, r.reason, self._t = ALIGN, "결함 재확인됨", 0.0
            else:
                r.state = FAILED
                r.verdict = "용접 전 미검출"
                r.reason = "재확인에서 결함이 안 보인다 — 대상 아님"

        elif r.state == ALIGN:
            e = self.align_error(torch_pos, defect_pos)
            r.align_err_mm = e
            r.aligned = e <= k["align_tol_mm"]
            r.state, self._t = ARC, 0.0
            r.reason = (f"정렬 {e:.2f} mm "
                        f"({'허용 이내' if r.aligned else '허용 초과'})")

        elif r.state == ARC:
            if self._t >= k["arc_s"]:
                used = (r.rod_required_mm if r.rod_required_mm > 0
                        else defect_size_mm * k["rod_per_mm3"])
                r.rod_used_mm += used
                if rod_budget is not None:
                    rod_budget.consume(used)
                r.state, r.reason, self._t = COOL, "아크 종료", 0.0

        elif r.state == COOL:
            if self._t >= self.cool_time(defect_size_mm):
                r.state, r.reason, self._t = REPOSITION, "냉각 완료", 0.0
                self._backed = 0.0

        elif r.state == REPOSITION:
            # 제자리에서는 용접 지점이 시야 밖이다. 뒤로 물러서야 보인다.
            self._backed += k["backoff_speed_mps"] * dt * 1000.0
            r.backoff_mm = self._backed
            if self._backed >= k["verify_backoff_mm"]:
                r.state, r.reason, self._t = VERIFY, "후진 완료 — 검증 촬영", 0.0
            else:
                r.reason = (f"검증 위치로 후진 {self._backed:.0f}/"
                            f"{k['verify_backoff_mm']:.0f} mm")

        elif r.state == VERIFY:
            layers = {}
            layers["align"] = bool(r.aligned)

            prof, peak, pmsg = (None, 0.0, "Depth 없음")
            if depth is not None and intr is not None and centre_px is not None:
                prof, peak, pmsg = self.profile_verdict(depth, intr, centre_px)
            r.profile_peak_mm = peak
            r.profile_ok = bool(prof) if prof is not None else False
            layers["profile"] = prof

            r.detector_clear = (detected is False)
            layers["detector"] = r.detector_clear
            r.layers = layers

            # 세 겹을 AND 로 묶는다. 어느 하나라도 아니라고 하면 성공이 아니다.
            # 특히 프로파일이 "여전히 파임"이면 검출기가 못 찾아도 실패다.
            if prof is False:
                ok, why = False, f"Depth: {pmsg}"
            elif not r.detector_clear:
                ok, why = False, "검증 촬영에서 여전히 검출"
            elif not r.aligned:
                ok, why = False, f"정렬 초과 {r.align_err_mm:.2f} mm"
            elif prof is None:
                ok, why = False, f"Depth 판정 불가 — {pmsg}"
            else:
                ok, why = True, f"3겹 통과 ({pmsg})"

            if ok:
                r.state, r.verdict, r.reason = SUCCESS, "수리 성공", why
            elif r.retry < k["max_retry"]:
                r.retry += 1
                r.state, r.reason, self._t = RETRY, f"재시도 {r.retry} — {why}", 0.0
            else:
                r.state, r.verdict, r.reason = FAILED, "수리 실패", why

        elif r.state == RETRY:
            r.state, r.reason, self._t = APPROACH, "재접근", 0.0

        return r
