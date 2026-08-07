"""[공용] 물 모델 — 수위·정수압·토리첼리·잠김율·유체력의 수식 단일 출처.

repair_demo 인라인 구현을 뽑아낸 것 (2026-08-06). 압력장을 풀지 않고
해석식으로 낸다 — 부력이 곧 수압의 적분(∮p·dA = ρgV)이라 로봇 동역학에는
부력·항력으로 들어가고, 여기서는 그 수식과 정량 지표(구멍 수압, 유출속도)를
한 곳에 모은다. 검증은 `test_code/pipe/test_water_model.py` (오프라인).

수평관은 수두가 cm 급이라 수백 Pa 수준이 정상값이다. 큰 값은 실전 맵의
입상관 몫(고저차 2.49m → 24.4 kPa).
"""

import math

import numpy as np

RHO = 1000.0      # kg/m³ 물
G = 9.81          # m/s²


def level_z(fill, pipe_ir):
    """충수율(관 지름 대비 0~1) → 수면 z (관 중심 기준 m).

    1/3 → −IR/3,  1/2 → 0,  2/3 → +IR/3,  옛 고정값 z+20mm = fill 0.70.
    """
    return (2.0 * fill - 1.0) * pipe_ir


def head_m(level, z):
    """z 지점(관 중심 기준) 위의 물기둥 높이. 수면 위면 0."""
    return max(level - z, 0.0)


def hydrostatic_pa(level, z, rho=RHO, g=G):
    """z 지점의 게이지 정수압 p = ρ·g·h [Pa]."""
    return rho * g * head_m(level, z)


def torricelli_ms(level, z, g=G):
    """z 지점 개구의 이론 유출속도 v = √(2gh) [m/s]."""
    return math.sqrt(2.0 * g * head_m(level, z))


def submerged_fraction(level, r_env):
    """수면 z=level 아래 잠긴 원 단면(반경 r_env)의 면적 비율 0~1.

    원의 활꼴: α = arccos(−level/r) 일 때 (α − sinα·cosα)/π.
    검산: level 0(반 채움)→0.5, +r(만관)→1, −r(마른 관)→0.
    """
    a = math.acos(max(-1.0, min(1.0, -level / r_env)))
    return (a - math.sin(a) * math.cos(a)) / math.pi


def buoyancy_n(v_disp, sub_frac=1.0, rho=RHO, g=G):
    """부력 F = ρ·g·V_disp·잠김율 [N]."""
    return rho * g * v_disp * sub_frac


def drag_n(rel_v, cd_eff, area_eff, rho=RHO):
    """2차 항력 F = ½ρ·C_d,eff·A·v|v| [N]. rel_v = 유속 − 로봇속도(부호 유지).

    🚨 C_d,eff·A 는 설계 §12.3 확정값을 쓸 것 — bbox 유도는 이 로봇처럼
       관을 거의 채우는 고폐색(β>0.6)에서 보정식이 폭주한다(메모리 기록).
    """
    return 0.5 * rho * cd_eff * area_eff * rel_v * abs(rel_v)


def drag_buoy_forces(tangents, vels, v_flow, cd_eff, area_each, buoy_each,
                     rho=RHO):
    """링크별 항력(코스 접선 방향, 역류) + 부력(연직 위) → (N,3) 힘 [N].

    두 데모의 `apply_fluid` 물리부 공용화 (2026-08-06). 기하(각 링크
    위치의 코스 접선)는 호출 쪽이 course 로 구해 넘긴다 — 물리는 여기,
    기하는 course, USD 는 데모.

    tangents: (N,3) 단위 접선.  vels: (N,3) 링크 속도.
    v_flow: 유속 크기(양수, 흐름은 접선 **반대** = 역류 주행).
    area_each / buoy_each: 링크 하나 몫(정면적·부력 — 반씩 나눠 진다).
    반환: (forces (N,3), 항력 합 스칼라).
    """
    t = np.asarray(tangents, float)
    v = np.asarray(vels, float)
    v_along = np.sum(v * t, axis=1)
    rel = (-v_flow) - v_along
    f = 0.5 * rho * cd_eff * area_each * rel * np.abs(rel)
    out = f[:, None] * t
    out[:, 2] += buoy_each
    return out, float(np.sum(f))
