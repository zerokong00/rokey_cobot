"""[공용] 균열 주입 — 깨끗한 배관에 균열을 새긴다. 시각층과 물리층을 나눈다.

CAD 팀은 **문제 없는 배관만** 만든다. 균열은 우리가 넣는다.

## 왜 시각과 물리를 나누는가

메시에 홈을 파는 것만으로는 물이 새지 않는다. 팀원(dongmin)이 실측으로 규칙을
확인했다 (`isaacpjt/dongmin/graphic_file/robot/water_leak_demo.py:105`).

    실효 통과경 = 개구지름 - 2 x 접촉오프셋 - 메시절삭
    누수 조건   = 실효 통과경 >= 약 2 x 입자지름

그쪽 실측: PCO 6mm / 입자지름 7.2mm 에서
    ø14 개구 -> 실효 ø2  -> 유출 0
    ø30 개구 -> 실효 ø18 -> 샌다

우리 균열은 폭 1.6mm 다. 이걸 통과시키려면 입자를 **16.5배 줄여야** 하고
입자 수는 N^3 = **4,096배**가 된다. 어떤 해상도로도 성립하지 않는다.

**결론: 눈에 진짜 같은 균열은 물리적으로 샐 수 없다.** 그래서 나눈다.

    시각층  얇은 균열 홈 (폭 1.6mm)   렌더 전용, 콜라이더 없음
            카메라와 YOLO 가 보는 것
    물리층  통과 가능한 개구 (입자 기준 자동 산정)  충돌 전용, 렌더 안 함
            물이 새는 것

같은 자리에 겹쳐 둔다. 카메라에는 얇은 균열이 보이고, 물은 실제로 샌다.
시뮬레이션 충실도의 맞바꿈이며 숨길 것이 아니라 명시할 사항이다.

## 물리층이 원본 콜라이더를 대체해야 한다

깨끗한 배관에 보이지 않는 개구 메시를 얹는다고 원본 콜라이더에 구멍이 나지
않는다. 그래서 `inject()` 는

    1. 원본 렌더 메시에서 CollisionAPI 를 뗀다
    2. 같은 형상에 개구를 뚫은 사본을 만들어 숨기고 거기에 콜라이더를 건다

## 파이썬 버전

기하·배치 함수는 **numpy 만** 쓴다 (3.10/3.11 아무데서나 돈다). 자리를 뽑고
메시를 만드는 것까지는 노트북에서 확인할 수 있다.

`inject()` **한 함수만 Isaac Sim 3.11 전용**이다 — `pxr` 을 그 안에서 늦게
import 하는 이유다. 파일 맨 위에서 import 하면 3.10 쪽 시험이 통째로 못 돈다.
"""

import math

import numpy as np

# ⚠ 입자·물 설정은 **읽기 전용**이다. 연산량 한계에 맞춰 팀에서 이미 조율한
# 값이라 여기서 바꾸지 않는다. 이 모듈은 그 값을 입력으로 받아 **개구 크기를
# 거기에 맞출 뿐**이다 (반대 방향이 아니다).
REF_PCO_MM = 6.0                  # dongmin water_leak_demo.py 기본 모드
REF_PARTICLE_DIA_MM = 7.2         # 〃
LEAK_MARGIN = 2.0          # 실효 통과경 / 입자지름. 그쪽 실측에서 ~2 가 하한
MESH_CUT_MM = 1.0          # 테셀레이션 절삭 여유
# dongmin 의 "~2배"는 실측 어림값이다. 경계에 딱 맞추면 조건이 조금만 달라져도
# 누수가 사라진다. 개구를 산정할 때 10% 여유를 둔다.
PORT_SAFETY = 1.10


def port_diameter_mm(particle_dia_mm=REF_PARTICLE_DIA_MM, pco_mm=REF_PCO_MM,
                     margin=LEAK_MARGIN):
    """입자 조건에서 물이 실제로 새는 최소 개구 지름.

    실효 통과경 = D - 2*PCO - 절삭  >=  margin * 입자지름
    """
    return (2.0 * pco_mm + margin * particle_dia_mm * PORT_SAFETY
            + MESH_CUT_MM)


def effective_bore_mm(port_dia_mm, pco_mm=REF_PCO_MM):
    return port_dia_mm - 2.0 * pco_mm - MESH_CUT_MM


def leaks(port_dia_mm, particle_dia_mm=REF_PARTICLE_DIA_MM,
          pco_mm=REF_PCO_MM, margin=LEAK_MARGIN):
    return (effective_bore_mm(port_dia_mm, pco_mm)
            >= margin * particle_dia_mm - 1e-9)


def why_two_layers(crack_width_mm, pco_mm=REF_PCO_MM,
                   particle_dia_mm=REF_PARTICLE_DIA_MM, margin=LEAK_MARGIN):
    """층을 나눈 근거 — 그 폭을 그대로 뚫으려면 입자를 몇 배 줄여야 하나.

    **입자를 줄이자는 제안이 아니다.** 입자 수가 N^3 로 늘어 연산이 감당 안
    되므로 그 길이 막혀 있다는 것을 숫자로 남긴 것이다. 입자 설정은 팀에서
    연산량에 맞춰 이미 조율했으므로 건드리지 않는다.
    """
    return (2.0 * pco_mm + margin * particle_dia_mm) / max(crack_width_mm, 1e-9)


# ── 바퀴가 안 지나는 자리 찾기 ───────────────────────────────────
#
# 개구가 ø28.8mm 인데 휠 반경은 10mm 다. 개구 위를 지나면 바퀴가 통째로
# 빠진다. 슬롯으로 축방향을 줄여도 소용없다 — 2xPCO(12mm)가 어느 방향이든
# 테두리에서 마개 노릇을 해서 **좁은 쪽도 28.8mm 이상**이어야 물이 샌다.
# 바퀴가 안 빠지려면 축방향 8mm 이하가 필요하니 동시 만족이 불가능하다.
#
# 그래서 모양이 아니라 **위치**로 푼다. 휠 접지 궤도는 원주 6곳뿐이고
# 그 사이에 42.8° (호 37.4mm) 가 비어 있다. 개구를 거기 두면 바퀴가
# 영영 지나지 않는다.

WHEEL_TRACKS_DEG = (0.0, 60.0, 120.0, 180.0, 240.0, 300.0)
WHEEL_WIDTH_MM = 15.0


def wheel_track_half_deg(r_in_mm, wheel_width_mm=WHEEL_WIDTH_MM):
    """휠 접지 궤도의 반각. 휠 폭을 관 내반경의 호로 환산한다."""
    return math.degrees(wheel_width_mm / 2.0 / r_in_mm)


def safe_angles_deg(r_in_mm, port_dia_mm, tracks=WHEEL_TRACKS_DEG,
                    wheel_width_mm=WHEEL_WIDTH_MM, clearance_mm=1.0):
    """개구를 놓아도 바퀴가 안 지나는 각도 목록(궤도 사이 중앙).

    개구가 그 틈에 안 들어가면 빈 목록을 돌려준다 — 조용히 겹치게 두는 것보다
    낫다.
    """
    half = wheel_track_half_deg(r_in_mm, wheel_width_mm)
    need_half = math.degrees((port_dia_mm / 2.0 + clearance_mm) / r_in_mm)
    out = []
    ts = sorted(tracks)
    for a, b in zip(ts, ts[1:] + [ts[0] + 360.0]):
        lo, hi = a + half, b - half
        if hi - lo >= 2.0 * need_half:
            out.append(((lo + hi) / 2.0) % 360.0)
    return out


def is_on_wheel_track(angle_deg, r_in_mm, port_dia_mm,
                      tracks=WHEEL_TRACKS_DEG, wheel_width_mm=WHEEL_WIDTH_MM):
    """그 각도에 개구를 두면 휠 궤도와 겹치는가."""
    half = wheel_track_half_deg(r_in_mm, wheel_width_mm)
    port_half = math.degrees(port_dia_mm / 2.0 / r_in_mm)
    for t in tracks:
        d = abs((angle_deg - t + 180.0) % 360.0 - 180.0)
        if d < half + port_half:
            return True
    return False


def snap_to_safe(angle_deg, r_in_mm, port_dia_mm, **kw):
    """요청 각도를 가장 가까운 안전 각도로 옮긴다. 이미 안전하면 그대로."""
    if not is_on_wheel_track(angle_deg, r_in_mm, port_dia_mm, **kw):
        return angle_deg, False
    cand = safe_angles_deg(r_in_mm, port_dia_mm, **kw)
    if not cand:
        return angle_deg, False
    best = min(cand, key=lambda c: abs((c - angle_deg + 180) % 360 - 180))
    return best, True


# ── 무작위 배치 ──────────────────────────────────────────────────
# 시연은 시간이 한정된다. 균열을 많이 뿌리면 로봇이 첫 두 개를 처리하다 끝난다.
# 기본 2 개로 잡고, 다음 두 가지를 만족하는 자리만 뽑는다.
#
#   1. 각도는 안전 각도(휠 궤도 사이)에서만 고른다 -> 개구를 옮길 일이 없다
#   2. 축방향 간격은 용접 후퇴 거리(welder/weld.py verify_backoff_mm=120) 보다
#      넓게 -> 한 균열을 검증하려고 물러설 때 다른 균열에 걸리지 않는다

MIN_SITE_GAP_MM = 150.0

CRACK_WIDTH_MM = (1.2, 2.2)
CRACK_LEN_MM = (30.0, 50.0)
CRACK_DEPTH_MM = (0.9, 1.6)     # 하한은 용접 판정 문턱(depth_defect_mm=0.6) 위


def pipe_span_mm(kind, length_mm=600.0, bend_r_mm=100.0, sweep_deg=90.0, **_):
    """호 위치 s 가 움직일 수 있는 구간. 직관은 중앙 기준, 곡관은 0 부터다."""
    if kind == "straight":
        return -length_mm / 2.0, length_mm / 2.0
    return 0.0, math.radians(sweep_deg) * bend_r_mm


def random_sites(n=2, kind="straight", r_in_mm=50.0, seed=None,
                 particle_dia_mm=REF_PARTICLE_DIA_MM, pco_mm=REF_PCO_MM,
                 min_gap_mm=MIN_SITE_GAP_MM, end_margin_mm=None,
                 width_mm=CRACK_WIDTH_MM, len_mm=CRACK_LEN_MM,
                 depth_mm=CRACK_DEPTH_MM, tries=2000, **geom):
    """깨끗한 배관 어디에 균열을 낼지 무작위로 고른다.

    `seed` 를 주면 같은 배치가 나온다 — 시연 전에 한 번 보고 그대로 다시
    띄울 수 있어야 한다.

    자리를 못 찾으면 조용히 줄이지 않고 예외를 낸다. 시연 직전에 균열이 하나만
    나와 있는 것보다 지금 실패하는 편이 낫다.
    """
    rng = np.random.default_rng(seed)
    port_d = port_diameter_mm(particle_dia_mm, pco_mm)
    angles = safe_angles_deg(r_in_mm, port_d)
    if not angles:
        raise ValueError(
            f"개구 ø{port_d:.1f} 가 휠 궤도 사이에 안 들어간다 "
            f"(내반경 {r_in_mm}mm)")

    lo, hi = pipe_span_mm(kind, **geom)
    if end_margin_mm is None:
        end_margin_mm = port_d / 2.0 + max(len_mm) / 2.0 + 5.0
    lo, hi = lo + end_margin_mm, hi - end_margin_mm
    if hi - lo < (n - 1) * min_gap_mm:
        raise ValueError(
            f"{kind} 관에 균열 {n} 개가 안 들어간다 — 놓을 구간 "
            f"{hi - lo:.0f}mm, 필요 {(n - 1) * min_gap_mm:.0f}mm")

    picked = []
    for _ in range(tries):
        if len(picked) == n:
            break
        s = float(rng.uniform(lo, hi))
        if any(abs(s - p[0]) < min_gap_mm for p in picked):
            continue
        picked.append((
            s,
            float(angles[rng.integers(len(angles))]),
            float(rng.uniform(*width_mm)),
            float(rng.uniform(*len_mm)),
            float(rng.uniform(*depth_mm)),
        ))
    if len(picked) != n:
        raise ValueError(f"{tries} 번 뽑아도 {n} 자리를 못 찾았다 "
                         f"({len(picked)} 개까지) — 간격을 줄여라")
    return sorted(picked)


def random_sites_over(segments, n=2, seed=None, min_gap_mm=MIN_SITE_GAP_MM,
                      **kw):
    """맵 전체(관 여러 개)에 균열 n 개를 흩어 놓는다.

    segments : [dict(kind=..., r_in_mm=..., length_mm=... 또는 bend_r_mm=...),
                ...]  주행 순서대로. `inject()` 에 넘길 나머지 키를 그대로
                담아 두면 된다.

    반환 : [(구간 번호, sites), ...]

    **한 관에 하나씩, 서로 붙어 있지 않은 관에 놓는다.** 곡관은 호가 157mm 라
    균열 두 개가 애초에 안 들어가고, 이웃한 관에 하나씩 놓으면 로봇이 물러설 때
    겹친다. 관이 모자라면 한 관 안에 간격을 지켜 넣는 쪽으로 되돌아간다.
    """
    rng = np.random.default_rng(seed)
    idx = list(range(len(segments)))

    def fits(i):
        lo, hi = pipe_span_mm(**segments[i])
        return hi - lo > min_gap_mm

    usable = [i for i in idx if fits(i)]
    for _ in range(2000):
        if len(usable) < n:
            break
        pick = sorted(rng.choice(usable, n, replace=False).tolist())
        if all(b - a >= 2 for a, b in zip(pick, pick[1:])):
            out = []
            for i in pick:
                s = dict(segments[i])
                out.append((i, random_sites(
                    1, s.pop("kind"), s.pop("r_in_mm", 50.0),
                    seed=int(rng.integers(1 << 30)), min_gap_mm=min_gap_mm,
                    **{**s, **kw})))
            return out

    # 되돌아가기 — 붙어 있지 않은 관을 못 고르면 가장 긴 관 하나에 몰아 넣는다
    if not usable:
        raise ValueError("균열을 놓을 만큼 긴 관이 없다")
    i = max(usable, key=lambda j: (lambda t: t[1] - t[0])(
        pipe_span_mm(**segments[j])))
    s = dict(segments[i])
    return [(i, random_sites(n, s.pop("kind"), s.pop("r_in_mm", 50.0),
                             seed=seed, min_gap_mm=min_gap_mm,
                             **{**s, **kw}))]


# ── 기하 ─────────────────────────────────────────────────────────
def _ring(centers, us, vs, r, phi):
    return (centers[:, None, :]
            + r[:, :, None] * (np.cos(phi)[None, :, None] * us[:, None, :]
                               + np.sin(phi)[None, :, None] * vs[:, None, :]))


def tube_with_openings(centers, us, vs, r_in, r_out, sections, open_fn=None,
                       cap_ends=True):
    """중심선을 따라 관을 만들되 open_fn 이 True 인 셀은 뚫는다.

    안팎 같은 자리를 뚫으므로 관통이 된다. 삼각형을 실제로 빼기 때문에
    렌더에서도 충돌에서도 열려 있다.
    """
    n = len(centers)
    phi = np.linspace(0.0, 2.0 * np.pi, sections, endpoint=False)
    rin = np.full((n, sections), r_in)
    rout = np.full((n, sections), r_out)
    inner = _ring(centers, us, vs, rin, phi).reshape(-1, 3)
    outer = _ring(centers, us, vs, rout, phi).reshape(-1, 3)
    verts = np.concatenate([inner, outer])
    m = n * sections

    live = np.ones((n - 1, sections), dtype=bool)
    if open_fn is not None:
        for i in range(n - 1):
            for j in range(sections):
                if open_fn(i + 0.5, (j + 0.5) % sections):
                    live[i, j] = False

    faces = []
    for i in range(n - 1):
        for j in range(sections):
            if not live[i, j]:
                continue
            a = i * sections + j
            b = i * sections + (j + 1) % sections
            c = (i + 1) * sections + j
            d = (i + 1) * sections + (j + 1) % sections
            faces += [[a, c, b], [b, c, d]]
            faces += [[m + a, m + b, m + c], [m + b, m + d, m + c]]

    # 뚫린 자리의 옆벽 — 안팎을 이어 막는다. 없으면 벽 속이 열려 보인다.
    def wall(p, q):
        faces.append([p, q, m + p])
        faces.append([q, m + q, m + p])

    for i in range(n - 1):
        for j in range(sections):
            if live[i, j]:
                continue
            jn = (j + 1) % sections
            if i == 0 or live[i - 1, j]:
                wall(i * sections + jn, i * sections + j)
            if i == n - 2 or live[i + 1, j]:
                wall((i + 1) * sections + j, (i + 1) * sections + jn)
            if live[i, (j - 1) % sections]:
                wall(i * sections + j, (i + 1) * sections + j)
            if live[i, jn]:
                wall((i + 1) * sections + jn, i * sections + jn)

    if cap_ends:
        for k, i in enumerate((0, n - 1)):
            for j in range(sections):
                a = i * sections + j
                b = i * sections + (j + 1) % sections
                if k == 0:
                    faces += [[a, b, m + a], [b, m + b, m + a]]
                else:
                    faces += [[a, m + a, b], [b, m + a, m + b]]

    # 감김 방향을 뒤집어야 법선이 바깥을 본다(실측으로 확인).
    # 콜라이더는 법선 방향을 보므로 이걸 틀리면 물이 반대로 샌다.
    return verts, np.array(faces, dtype=np.int64)[:, ::-1]


def straight_frames(length_mm, n):
    xs = np.linspace(-length_mm / 2.0, length_mm / 2.0, n)
    centers = np.stack([xs, np.zeros(n), np.zeros(n)], axis=-1)
    us = np.tile(np.array([0.0, 0.0, 1.0]), (n, 1))
    vs = np.tile(np.array([0.0, 1.0, 0.0]), (n, 1))
    return xs, centers, us, vs


def elbow_frames(bend_r_mm, sweep_deg, n):
    """곡관 단면 기저.

    ⚠ 각도 규약 — us = (-sin t, 0, cos t) 는 곡률 중심을 **향한다**.
    즉 site_deg = 0 은 곡관의 **안쪽(조인 쪽, r=50mm)** 이고
    180 도가 바깥쪽(r=150mm)이다. 직관은 0 도가 +Z 다.
    균열을 곡관 바깥쪽에 두려면 site_deg=180 을 줘야 한다.
    """
    ts = np.radians(np.linspace(0.0, sweep_deg, n))
    centers = np.stack([bend_r_mm * np.sin(ts), np.zeros(n),
                        bend_r_mm * (1.0 - np.cos(ts))], axis=-1)
    us = np.stack([-np.sin(ts), np.zeros(n), np.cos(ts)], axis=-1)
    vs = np.tile(np.array([0.0, 1.0, 0.0]), (n, 1))
    return ts * bend_r_mm, centers, us, vs


def leak_port_mesh(kind, site_s_mm, site_deg, port_dia_mm, r_in, r_out,
                   length_mm=600.0, bend_r_mm=100.0, sweep_deg=90.0,
                   sections=96, step_mm=4.0):
    """물리층 — 통과 개구가 뚫린 관. 숨겨 놓고 콜라이더만 건다."""
    total = length_mm if kind == "straight" else math.radians(sweep_deg) * bend_r_mm
    n = max(8, int(total / step_mm) + 1)
    if kind == "straight":
        ss, centers, us, vs = straight_frames(length_mm, n)
    else:
        ss, centers, us, vs = elbow_frames(bend_r_mm, sweep_deg, n)

    site = math.radians(site_deg)
    rad = port_dia_mm / 2.0

    def open_fn(i, j):
        s = np.interp(i, np.arange(len(ss)), ss)
        ang = 2.0 * np.pi * j / sections
        da = (ang - site + np.pi) % (2 * np.pi) - np.pi
        return math.hypot(s - site_s_mm, da * r_in) < rad

    return tube_with_openings(centers, us, vs, r_in, r_out, sections, open_fn)


def crack_visual_mesh(kind, site_s_mm, site_deg, width_mm, len_mm, depth_mm,
                      r_in, length_mm=600.0, bend_r_mm=100.0, sweep_deg=90.0,
                      sections=192, fine_mm=0.4, pad_mm=4.0):
    """시각층 — 관 안쪽 면에 새긴 얇은 균열 홈. 콜라이더 없이 렌더만 한다.

    카메라와 YOLO 가 보는 것은 이쪽이다. 균열 주변만 조밀하게 표본한다.
    """
    total = length_mm if kind == "straight" else math.radians(sweep_deg) * bend_r_mm
    lo = max(0.0 if kind != "straight" else -length_mm / 2.0,
             site_s_mm - len_mm / 2.0 - pad_mm)
    hi = min(total if kind != "straight" else length_mm / 2.0,
             site_s_mm + len_mm / 2.0 + pad_mm)
    n = max(8, int((hi - lo) / fine_mm) + 1)

    if kind == "straight":
        ss = np.linspace(lo, hi, n)
        centers = np.stack([ss, np.zeros(n), np.zeros(n)], axis=-1)
        us = np.tile(np.array([0.0, 0.0, 1.0]), (n, 1))
        vs = np.tile(np.array([0.0, 1.0, 0.0]), (n, 1))
    else:
        ts = np.linspace(lo, hi, n) / bend_r_mm
        centers = np.stack([bend_r_mm * np.sin(ts), np.zeros(n),
                            bend_r_mm * (1.0 - np.cos(ts))], axis=-1)
        us = np.stack([-np.sin(ts), np.zeros(n), np.cos(ts)], axis=-1)
        vs = np.tile(np.array([0.0, 1.0, 0.0]), (n, 1))
        ss = np.linspace(lo, hi, n)

    site = math.radians(site_deg)
    half_arc = width_mm / 2.0 + 2.0
    dphi = half_arc / r_in
    ph = np.linspace(-dphi, dphi, 41) + site
    hw, hl = width_mm / 2.0, len_mm / 2.0

    da = (ph - site + np.pi) % (2 * np.pi) - np.pi
    arc = da * r_in
    prof = np.clip(1.0 - np.abs(arc) / hw, 0.0, 1.0)[None, :]
    along = np.clip(1.0 - (np.abs(ss - site_s_mm) / hl) ** 6, 0.0, 1.0)[:, None]
    r = r_in + depth_mm * prof * along

    pts = (centers[:, None, :]
           + r[:, :, None] * (np.cos(ph)[None, :, None] * us[:, None, :]
                              + np.sin(ph)[None, :, None] * vs[:, None, :]))
    nu, nv = pts.shape[0], pts.shape[1]
    verts = pts.reshape(-1, 3)
    faces = []
    for i in range(nu - 1):
        for j in range(nv - 1):
            a, b = i * nv + j, i * nv + j + 1
            c, d = (i + 1) * nv + j, (i + 1) * nv + j + 1
            faces += [[a, c, b], [b, c, d]]
    return verts, np.array(faces, dtype=np.int64)


# ── Isaac Sim 주입 ───────────────────────────────────────────────
def inject(stage, pipe_prim_path, sites, r_in_mm, r_out_mm, kind="straight",
           particle_dia_mm=REF_PARTICLE_DIA_MM, pco_mm=REF_PCO_MM,
           scale=0.001, **geom):
    """깨끗한 배관 프림에 균열을 주입한다.

    sites : [(호위치mm, 각도deg, 균열폭mm, 균열길이mm, 깊이mm), ...]

    원본 렌더 메시의 콜라이더를 떼고, 개구를 뚫은 사본을 숨겨 콜라이더로 쓴다.
    보이지 않는 개구 메시를 얹는 것만으로는 원본 콜라이더에 구멍이 안 난다.
    """
    # 여기서부터가 Isaac Sim 3.11 전용이다. 위쪽 기하 함수는 3.10 에서도 돈다.
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from pyver import require_isaac

    require_isaac("crack_inject.inject()")
    from pxr import Gf, UsdGeom, UsdPhysics, Vt

    port_d = port_diameter_mm(particle_dia_mm, pco_mm)
    made = dict(port_dia_mm=port_d,
                effective_mm=effective_bore_mm(port_d, pco_mm),
                visual=[], collision=[])

    # ① 원본 렌더 메시에서 콜라이더를 뗀다
    for p in stage.Traverse():
        if not str(p.GetPath()).startswith(pipe_prim_path):
            continue
        if p.IsA(UsdGeom.Mesh) and p.HasAPI(UsdPhysics.CollisionAPI):
            p.RemoveAPI(UsdPhysics.CollisionAPI)

    def add_mesh(path, verts, faces, visible, collider):
        mesh = UsdGeom.Mesh.Define(stage, path)
        v = (verts * scale).astype(float)
        mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*x) for x in v]))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(faces)))
        mesh.CreateFaceVertexIndicesAttr(
            Vt.IntArray(faces.reshape(-1).tolist()))
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateExtentAttr([Gf.Vec3f(*v.min(0)), Gf.Vec3f(*v.max(0))])
        if not visible:
            UsdGeom.Imageable(mesh).CreateVisibilityAttr("invisible")
        if collider:
            UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(
                mesh.GetPrim()).CreateApproximationAttr("none")
        return mesh

    # ② 물리층 — 모든 사이트를 한 메시에 뚫는다
    for k, (s_mm, ang, w, ln, dep) in enumerate(sites):
        # 개구가 휠 접지 궤도에 걸리면 바퀴가 빠진다. 궤도 사이로 옮긴다.
        port_ang, moved = snap_to_safe(ang, r_in_mm, port_d)
        if moved:
            made.setdefault("snapped", []).append(
                dict(site=k, requested=ang, used=port_ang))
        v, f = leak_port_mesh(kind, s_mm, port_ang, port_d, r_in_mm, r_out_mm,
                              **geom)
        add_mesh(f"{pipe_prim_path}/leak_port_{k}", v, f,
                 visible=False, collider=True)
        made["collision"].append(f"{pipe_prim_path}/leak_port_{k}")

        # ③ 시각층 — 얇은 균열. 콜라이더 없음
        v, f = crack_visual_mesh(kind, s_mm, ang, w, ln, dep, r_in_mm, **geom)
        add_mesh(f"{pipe_prim_path}/crack_visual_{k}", v, f,
                 visible=True, collider=False)
        made["visual"].append(f"{pipe_prim_path}/crack_visual_{k}")

    return made


# ── 확인용 ───────────────────────────────────────────────────────
DEMO_MAP = [
    dict(kind="straight", r_in_mm=50.0, length_mm=600.0, prim="/World/pipe_0"),
    dict(kind="elbow", r_in_mm=50.0, bend_r_mm=100.0, sweep_deg=90.0,
         prim="/World/elbow_0"),
    dict(kind="straight", r_in_mm=50.0, length_mm=600.0, prim="/World/pipe_1"),
    dict(kind="elbow", r_in_mm=50.0, bend_r_mm=100.0, sweep_deg=90.0,
         prim="/World/elbow_1"),
    dict(kind="straight", r_in_mm=50.0, length_mm=600.0, prim="/World/pipe_2"),
]


def _table(rows, r_in, port_d):
    print(f"{'관':>16} {'호위치mm':>9} {'각도':>6} {'폭mm':>6} {'길이mm':>7} "
          f"{'깊이mm':>7}  바퀴")
    for name, (s, ang, w, ln, dep) in rows:
        on = is_on_wheel_track(ang, r_in, port_d)
        print(f"{name:>16} {s:9.1f} {ang:6.1f} {w:6.2f} {ln:7.1f} {dep:7.2f}"
              f"  {'걸림!' if on else '안 걸림'}")


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="시연용 균열 자리 뽑기")
    ap.add_argument("--kind", default="straight", choices=("straight", "elbow"))
    ap.add_argument("--map", action="store_true", help="관 여러 개에 흩어 놓기")
    ap.add_argument("--n", type=int, default=2)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--r-in", type=float, default=50.0)
    a = ap.parse_args()

    port_d = port_diameter_mm()
    print(f"개구 ø{port_d:.1f} (실효 ø{effective_bore_mm(port_d):.1f}, "
          f"{'샌다' if leaks(port_d) else '막힘'}), "
          f"안전 각도 {[round(x) for x in safe_angles_deg(a.r_in, port_d)]}")

    if a.map:
        rows = []
        for i, sites in random_sites_over(DEMO_MAP, a.n, seed=a.seed):
            for st in sites:
                rows.append((DEMO_MAP[i]["prim"].split("/")[-1], st))
        _table(rows, a.r_in, port_d)
    else:
        _table([(a.kind, st)
                for st in random_sites(a.n, a.kind, a.r_in, seed=a.seed)],
               a.r_in, port_d)
