"""[공용] 주행 제어 — ROS 비의존 순수 로직.

이 로봇에는 조향이 없다. 관절은 수동이고 cmd_vel 은 linear.x 만 쓴다.
6륜이 관벽에 압착되어 횡방향은 기계가 이미 잡아주므로, 제어기가 정하는 것은
**전진 속도 하나**다. 그래서 여기 있는 것은 조향 PID 가 아니라 속도 법칙이다.

  v = V_MAX x 전방개방도 x 곡관감속 x 상태게이트

여기에 FSM 과 끼임 판정을 얹는다. 끼임은 **바퀴는 도는데 실제로 안 나가는 것**
이므로 휠 오도메트리와 시각 오도메트리를 비교해야 알 수 있다.
"""

from dataclasses import dataclass, asdict, field

IDLE = "IDLE"
CRUISE = "CRUISE"
SLOW = "SLOW"
HOLD = "HOLD"
RECOVER = "RECOVER"
RETURN = "RETURN"
DONE = "DONE"

DEFAULTS = dict(
    v_max_mps=0.10,
    v_slow_frac=0.35,
    v_return_mps=0.12,
    accel_mps2=0.08,
    decel_mps2=0.40,

    aperture_ref_px=90000.0,
    aperture_min_frac=0.25,
    incidence_slow_deg=8.0,
    incidence_stop_deg=30.0,

    mission_range_m=30.0,

    stuck_wheel_min_mps=0.02,
    stuck_visual_frac=0.30,
    stuck_hold_s=1.5,
    recover_back_m=0.05,
    recover_max_try=3,

    condition_timeout_s=1.0,
    hold_release_s=2.0,
)


@dataclass
class DriveState:
    state: str = IDLE
    v_cmd: float = 0.0
    v_target: float = 0.0
    distance_m: float = 0.0
    slip_ratio: float = 1.0
    stuck_for_s: float = 0.0
    recover_try: int = 0
    reason: str = ""
    gates: dict = field(default_factory=dict)

    def as_dict(self):
        return asdict(self)


def _clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


class DriveController:
    """조건 스칼라를 받아 전진 속도와 FSM 상태를 낸다.

    이미지는 보지 않는다. 설계 원칙상 이미지는 인지 노드 밖으로 나가지 않으며,
    여기로는 검출 결과와 치수만 들어온다.
    """

    def __init__(self, params=None):
        self.k = dict(DEFAULTS)
        if params:
            self.k.update({a: b for a, b in params.items() if a in self.k})
        self.s = DriveState()
        self._hold_s = 0.0
        self._recover_from = 0.0
        self._no_cond_s = 0.0
        self._last_cond = None

    # ── 속도 법칙 ────────────────────────────────────────────────
    def openness(self, aperture_px):
        """전방이 얼마나 열려 있나. 개구부 면적이 작을수록 감속."""
        if aperture_px is None or aperture_px <= 0:
            return self.k["aperture_min_frac"]
        r = aperture_px / self.k["aperture_ref_px"]
        return _clamp(r, self.k["aperture_min_frac"], 1.0)

    def curve_factor(self, incidence_deg):
        """개구부가 화면 중앙에서 벗어난 각. 곡관 접근의 선행 지표다.

        관절 엔코더는 이미 꺾인 뒤에야 반응하지만 이 각은 곡관에 닿기 전에
        커진다. 그래서 선제 감속에 쓴다.
        """
        a, b = self.k["incidence_slow_deg"], self.k["incidence_stop_deg"]
        d = abs(incidence_deg or 0.0)
        if d <= a:
            return 1.0
        if d >= b:
            return self.k["v_slow_frac"]
        t = (d - a) / (b - a)
        return 1.0 - t * (1.0 - self.k["v_slow_frac"])

    def gate(self, cond):
        """관 상태 판정이 내리는 통행 허가."""
        if cond is None:
            return 0.0, "판정 없음"
        if not cond.get("passable", False):
            return 0.0, f"{cond.get('state', '?')} — {cond.get('reason', '')}"
        sp = cond.get("speed", "stop")
        if sp == "stop":
            return 0.0, "정지 지시"
        if sp == "slow":
            return self.k["v_slow_frac"], "저속 지시"
        return 1.0, ""

    # ── 끼임 판정 ────────────────────────────────────────────────
    def update_slip(self, wheel_mps, visual_mps, dt):
        """바퀴는 도는데 실제로 안 나가면 끼임이다.

        휠 엔코더만으로는 6륜이 동시에 미끄러지는 경우를 못 잡는다.
        시각 오도메트리와 비교해야 실제 이동을 안다.
        """
        w = abs(wheel_mps or 0.0)
        if w < self.k["stuck_wheel_min_mps"]:
            self.s.slip_ratio = 1.0
            self.s.stuck_for_s = 0.0
            return False
        v = abs(visual_mps if visual_mps is not None else w)
        self.s.slip_ratio = _clamp(v / w, 0.0, 2.0)
        if self.s.slip_ratio < self.k["stuck_visual_frac"]:
            self.s.stuck_for_s += dt
        else:
            self.s.stuck_for_s = 0.0
        return self.s.stuck_for_s >= self.k["stuck_hold_s"]

    # ── 본체 ─────────────────────────────────────────────────────
    def step(self, dt, cond=None, wheel_mps=None, visual_mps=None,
             start=False, recall=False):
        s = self.s
        k = self.k

        if wheel_mps is not None:
            s.distance_m += wheel_mps * dt

        # 판정 주기(10Hz)가 제어 주기(20Hz)보다 느리다. 새 메시지가 없는 틱을
        # 그대로 "판정 없음"으로 다루면 매 틱 걸러 정지해 주행이 끊긴다.
        # 마지막 판정을 타임아웃까지 유지하고, 그 뒤에야 두절로 본다.
        if cond is not None:
            self._last_cond = cond
            self._no_cond_s = 0.0
        else:
            self._no_cond_s += dt
        cond = (None if self._no_cond_s > k["condition_timeout_s"]
                else self._last_cond)

        stuck = self.update_slip(wheel_mps, visual_mps, dt)
        g, greason = self.gate(cond)

        if recall and s.state not in (RETURN, DONE):
            s.state, s.reason = RETURN, "복귀 지시"
        elif s.state == IDLE:
            if start:
                s.state, s.reason = CRUISE, "출발"
        elif s.state in (CRUISE, SLOW):
            if s.distance_m >= k["mission_range_m"]:
                s.state, s.reason = RETURN, f"{k['mission_range_m']:.0f}m 도달"
            elif g <= 0.0:
                s.state, s.reason = HOLD, greason
                self._hold_s = 0.0
            elif stuck:
                s.state = RECOVER
                s.reason = f"끼임 (슬립률 {s.slip_ratio:.2f})"
                s.recover_try += 1
                self._recover_from = s.distance_m
            else:
                s.state = SLOW if g < 1.0 else CRUISE
                s.reason = greason or "정상 주행"
        elif s.state == HOLD:
            self._hold_s += dt
            if g > 0.0 and self._hold_s >= k["hold_release_s"]:
                s.state, s.reason = CRUISE, "통행 재개"
            else:
                s.reason = greason or "정지 유지"
        elif s.state == RECOVER:
            if s.recover_try > k["recover_max_try"]:
                s.state, s.reason = RETURN, "끼임 반복 — 임무 중단"
            elif self._recover_from - s.distance_m >= k["recover_back_m"]:
                s.state, s.reason = CRUISE, "후진 완료 — 재시도"
                s.stuck_for_s = 0.0
            else:
                s.reason = f"후진 중 ({s.recover_try}/{k['recover_max_try']})"
        elif s.state == RETURN:
            if s.distance_m <= 0.0:
                s.state, s.reason = DONE, "복귀 완료"

        if s.state in (IDLE, HOLD, DONE):
            s.v_target = 0.0
        elif s.state == RECOVER:
            s.v_target = -k["v_return_mps"] * 0.5
        elif s.state == RETURN:
            s.v_target = -k["v_return_mps"]
        else:
            openness = self.openness((cond or {}).get("aperture_px"))
            curve = self.curve_factor((cond or {}).get("incidence_deg"))
            s.v_target = k["v_max_mps"] * openness * curve * g
            s.gates = dict(openness=round(openness, 3), curve=round(curve, 3),
                           condition=round(g, 3))

        # 가속은 완만하게, 감속은 빠르게. 정지는 늦으면 안 된다.
        rate = k["accel_mps2"] if abs(s.v_target) > abs(s.v_cmd) else k["decel_mps2"]
        step = rate * dt
        if s.v_cmd < s.v_target:
            s.v_cmd = min(s.v_cmd + step, s.v_target)
        else:
            s.v_cmd = max(s.v_cmd - step, s.v_target)
        if s.state in (IDLE, HOLD, DONE) and abs(s.v_cmd) < 1e-4:
            s.v_cmd = 0.0
        return s
