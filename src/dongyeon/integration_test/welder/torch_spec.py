"""[공용] 용접 토치 드라이브 상수 — 현역 플랫폼의 단일 출처.

치수(반경·스트로크·한계)는 `spec/parts_meta.json` 의 `torch` 가 정본이고,
여기에는 **드라이브(강성·감쇠·최대힘)와 링크 질량**만 둔다. 둘을 합치지 않은
이유는 parts_meta.json 이 `tools/build_parts.py` 가 다시 쓰는 파생물이기
때문이다 — 손으로 넣으면 재생성 때 날아간다.

🚨 이 값들을 눈대중으로 바꾸지 말 것 (2026-08-04 실측).
   J1 강성을 `2.0 N·m/**deg**` 로 넣었다가 로봇이 주행 불능이 됐다. 맞는 값은
   `2.0 N·m/**rad**` 이라 **57배**였고 maxForce 도 7배였다. 링(45g)에 걸린 과대
   토크의 반작용이 앞 세그먼트를 비틀어 벨로우즈가 -21° 접히고 휠 각속도가
   지령의 37% 로 주저앉았다(토치를 떼면 정상 주행하는 것으로 대조 확인).
   **USD 각도 드라이브의 강성·감쇠 단위는 N·m/deg 다** — 라디안 값을 그대로
   넣으면 57.3배가 된다. `usd_angular()` 를 쓸 것.

출처: son 1세대 조립(`legacy/assemble.py`)에서 검증된 값을 옮겨 왔다. 보관본은
동결이므로 앞으로 값이 바뀌면 **이 파일만** 고친다.
"""

import math

DEG = math.pi / 180.0

# ── 드라이브 ────────────────────────────────────────────────────────
J1_STIFF_NM_RAD = 2.0        # 링 회전(관 중심축)
J2_STIFF_N_M = 400.0         # 토치 직동(반경 방향)

# ── 링크 질량 ───────────────────────────────────────────────────────
MASS_RING_KG = 0.045
MASS_ROD_KG = 0.010
MASS_TIP_KG = 0.008
MASS_TORCH_KG = MASS_RING_KG + MASS_ROD_KG + MASS_TIP_KG


def usd_angular():
    """J1 각도 드라이브의 (stiffness, damping, maxForce) — 단위 **N·m/deg**."""
    k = J1_STIFF_NM_RAD * DEG
    return k, k / 10.0, 20.0 * k


def usd_linear(stroke_m):
    """J2 직동 드라이브의 (stiffness, damping, maxForce) — 단위 N/m, N."""
    return J2_STIFF_N_M, J2_STIFF_N_M / 10.0, 3.0 * J2_STIFF_N_M * stroke_m
