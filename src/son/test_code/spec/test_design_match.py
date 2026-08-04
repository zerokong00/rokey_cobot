"""[오프라인] 설계 문서 ↔ parts_meta 대조 — 치수 누락을 구조적으로 막는다.

## 왜 필요한가

`spec/parts_meta.json` 을 "치수 단일 출처" 라고 해 두고 정작 **설계 문서와
대조하는 절차가 없었다.** 그래서 2026-08-04 까지

    설계확정본 : 휠 Ø20 x 폭 15mm, **크라운 반경 50mm**
    parts_meta : wheel_r 10, wheel_width 15   (크라운 없음)

이 상태로 갔고, 휠이 원통으로 만들어져 관벽에 **트레드 양끝 2점으로만**
닿았다. Isaac Sim 담당자가 실측으로 찾아 줬는데, **이건 시뮬레이터가
필요 없는 실수다** — 문서만 대조했으면 잡혔다.

## 세 가지 상태로 가른다

    일치      문서값과 parts_meta 가 같다
    의도된 차이  다르지만 **이유가 적혀 있다** (링 재설계, 간섭 회피 등)
    누락      문서에 있는데 parts_meta 에 없다   ← 크라운이 여기 있었다

**"의도된 차이" 는 통과시키되 매번 출력한다.** 조용히 넘어가면 그게 다시
누락이 된다.

문서 원문은 `spec/design_v3_extract.txt` (tools/extract_design.py 로 갱신).

실행:  python3 test_design_match.py
"""

import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SON = HERE.parents[1]

DESIGN = SON / "spec" / "design_v3_extract.txt"
META = SON / "spec" / "parts_meta.json"

TOL = 0.05          # mm / 도. 유도값은 반올림 표기라 이 정도는 같은 값으로 본다


def near(a, b, tol=TOL):
    return abs(float(a) - float(b)) <= tol


# ── 대조표 ───────────────────────────────────────────────────────
# (항목, 문서에 있어야 할 문구, 검사 함수 or None, 의도된 차이면 이유)
#
# 검사 함수는 meta 를 받아 (실측값, 기대값) 을 돌려준다. None 이면 문구가
# 문서에 있는지만 본다.

def _torch(m, k):
    return m["torch"][k]


CHECKS = [
    ("암 배치 (전방)", "0° / 120° / 240°",
     lambda m: (m["front_arms"], [0.0, 120.0, 240.0]), None),
    ("암 배치 (후방)", "60° / 180° / 300°",
     lambda m: (m["rear_arms"], [60.0, 180.0, 300.0]), None),
    ("축 방향 스태거", "세트 내 10mm 간격",
     lambda m: (m["stagger"], 10.0),
     "7mm 로 줄임 — 토치 링(x 58~68mm)과 암이 1.0mm 밖에 안 떨어져 "
     "곡관 스트로크가 ±4.59→±4.02mm 로 개선됐다"),
    ("암 길이", "40mm / 12mm", lambda m: (m["arm_len"], 40.0), None),
    ("피벗 반경", "40mm / 12mm", lambda m: (m["pivot_r"], 12.0), None),
    ("암 각도 정상", "정상 44.4°",
     lambda m: (m["arm_angle_nominal"], 44.4), None),
    ("암 각도 하한", "33.4° ~ 58.2°",
     lambda m: (m["arm_angle_compressed"], 33.4), None),
    ("암 각도 상한", "33.4° ~ 58.2°",
     lambda m: (m["arm_angle_extended"], 58.2), None),
    ("휠 반경", "Ø20 × 폭 15mm", lambda m: (m["wheel_r"], 10.0), None),
    ("휠 폭", "Ø20 × 폭 15mm", lambda m: (m["wheel_width"], 15.0), None),
    ("휠 크라운 반경", "크라운 반경 50mm",
     lambda m: (m["wheel_crown_r"], 50.0), None),
    ("세그먼트 길이", "전방 62 + 관절 26 + 후방 62",
     lambda m: (m["seg_len"], 62.0), None),
    ("관절 길이", "전방 62 + 관절 26 + 후방 62",
     lambda m: (m["joint_len"], 26.0), None),
    ("배관 내경", "강관 / DN100 / 4mm",
     lambda m: (m["pipe_id"], 100.0), None),
    ("배관 외경", "강관 / DN100 / 4mm",
     lambda m: (m["pipe_od"], 114.0), None),
    ("곡관 반경", "SR (R = 1.0D = 100mm)",
     lambda m: (m["elbow_r"], 100.0), None),
    ("토치 도달 반경", "48mm — 용접 간극 2mm 확보",
     lambda m: (_torch(m, "reach_radius_mm"), 48.0), None),
    ("토치 수납 반경", "20mm — 본체 외곽 25mm 안쪽",
     lambda m: (_torch(m, "stow_radius_mm"), 20.0),
     "문서 v3 은 **본체에서 뻗는 직동 토치** 기준이다. 구현은 팀 결정으로 "
     "**본체를 감싸는 회전 링**으로 바꿨다 — 바퀴에 조향이 없어 로봇을 "
     "결함 방향으로 돌릴 수 없기 때문이다. 링 외경 기준 40mm"),
    ("토치 직동 스트로크", "스트로크 35mm",
     lambda m: (_torch(m, "j2_stroke_mm"), 35.0),
     "링 방식에서는 링 외경 40mm 에서 48mm 까지 8mm 만 뻗으면 된다. "
     "축에서 20mm 부터 뻗던 직동 방식의 35mm 와 출발점이 다르다"),
]


# ── 물리 설정 대조 ───────────────────────────────────────────────
# parts_meta 는 **형상만** 담는다. 물리 설정은 스크립트 안에 흩어져 있어서
# 위 대조표가 못 잡았고, 그래서 2026-08-04 까지 아래가 전부 어긋나 있었다.
#
#   물리 스텝 1/240   → World() 기본 1/60      (실측 전진 0mm → 42.9mm)
#   마찰 0.30/0.25    → 0.70 하나로 뭉뚱그림
#   contactOffset     → 맞았음
#
# 소스에서 정규식으로 뽑아 문서값과 맞춘다. 실행하지 않는다.

PHYS_CHECKS = [
    # 물리 스텝은 World 를 만드는 쪽이 정한다. assemble.py 는 조립만 하고
    # World 를 만들지 않으므로 대상에서 뺀다.
    ("물리 스텝", "물리 스텝            : 1/240 이상 (불안정 시 1/500)",
     ["repair_demo.py", "legacy/curve_demo.py", "legacy/articulate.py"],
     r"PHYSICS_(?:DT\s*=\s*1\.0\s*/|HZ_PRE\s*=.*?\"PHYSICS_HZ\",)\s*([\d.]+)",
     lambda v: float(v) >= 240.0, "1/240 이상",
     "1/240 은 연산량이 4배다. 근거였던 실측(전진 0.0mm → 42.9mm)이 "
     "크라운 누락·마찰 0.70·예압 소실 빌드에서 나온 값이라, 설계대로 고친 "
     "지금은 1/60 으로도 될 수 있다. **Isaac Sim 에서 재측정 후 결정할 것** — "
     "1/60 에서 0mm 면 설계값으로 복귀한다"),
    ("솔버 position", "솔버 반복 (position) : 32",
     ["legacy/assemble.py"],
     r"CreateSolverPositionIterationCountAttr\((\d+)\)",
     lambda v: float(v) >= 32, "32 이상", None),
    ("솔버 velocity", "솔버 반복 (velocity) : 4",
     ["legacy/assemble.py"],
     r"CreateSolverVelocityIterationCountAttr\((\d+)\)",
     lambda v: float(v) >= 4, "4 이상", None),
    ("contactOffset", "contactOffset       : 0.0005 m",
     ["legacy/curve_demo.py", "legacy/assemble.py"],
     r"CONTACT_OFFSET\s*=\s*([\d.]+)",
     lambda v: abs(float(v) - 0.0005) < 1e-9, "0.0005", None),
    # 🔑 마찰은 **로봇 종류가 아니라 관 상태**로 고른다 (2026-08-05 방침).
    #    정찰기/수리기를 나누지 않고 한 대가 점검·수리를 다 한다. 물속에서
    #    용접하므로 시연 내내 만관값이다. 현역 repair_demo.py 가 FLOODED 로
    #    갈라 쓰고, legacy 는 1세대 두 로봇 구분이 남아 있어 대상에서 뺀다.
    ("만관 정지마찰", "friction_static   : 0.30", ["repair_demo.py"],
     r"FRICTION_STATIC\s*=\s*([\d.]+)\s+if\s+FLOODED",
     lambda v: abs(float(v) - 0.30) < 1e-9, "0.30", None),
    ("만관 운동마찰", "friction_dynamic  : 0.25", ["repair_demo.py"],
     r"FRICTION_DYNAMIC\s*=\s*([\d.]+)\s+if\s+FLOODED",
     lambda v: abs(float(v) - 0.25) < 1e-9, "0.25", None),
    ("배수 정지마찰", "friction_static   : 0.40", ["repair_demo.py"],
     r"FRICTION_STATIC\s*=.*else\s+([\d.]+)",
     lambda v: abs(float(v) - 0.40) < 1e-9, "0.40", None),
    ("배수 운동마찰", "friction_dynamic  : 0.35", ["repair_demo.py"],
     r"FRICTION_DYNAMIC\s*=.*else\s+([\d.]+)",
     lambda v: abs(float(v) - 0.35) < 1e-9, "0.35", None),
    # 만관 유체력 (§12.3 physics_flooded) — 물속 용접이므로 시연 내내 유효하다
    ("유속", "v_flow            : 0.855   m/s", ["repair_demo.py"],
     r"V_FLOW_DESIGN\s*=\s*([\d.]+)",
     lambda v: abs(float(v) - 0.855) < 1e-9, "0.855", None),
    ("항력계수", "C_d_eff           : 2.32", ["repair_demo.py"],
     r"CD_EFF\s*=\s*([\d.]+)",
     lambda v: abs(float(v) - 2.32) < 1e-9, "2.32", None),
    ("정면적", "A_frontal         : 2.7e-3  m²", ["repair_demo.py"],
     r"A_FRONTAL\s*=\s*([\d.e-]+)",
     lambda v: abs(float(v) - 2.7e-3) < 1e-12, "2.7e-3", None),
    ("배수 체적", "V_disp            : 1.8e-4  m³", ["repair_demo.py"],
     r"V_DISP\s*=\s*([\d.e-]+)",
     lambda v: abs(float(v) - 1.8e-4) < 1e-12, "1.8e-4", None),
]


def check_physics(text):
    """소스에서 물리 설정을 뽑아 문서값과 맞춘다. 반환 (실패목록, 줄목록)."""
    fails, lines, devs = [], [], []
    for name, phrase, files, pat, ok_fn, want, reason in PHYS_CHECKS:
        if phrase not in text:
            fails.append(f"{name}: 문서에서 '{phrase}' 를 못 찾았다")
            lines.append((name, "문서 문구 없음", want, False))
            continue
        for f in files:
            src = (SON / f).read_text()
            m = re.search(pat, src)
            if not m:
                fails.append(f"{name} ({f}): 값을 못 찾았다 — 설정 누락")
                lines.append((f"{name}·{Path(f).stem}", "없음", want, False))
                continue
            good = ok_fn(m.group(1))
            if not good and reason:
                # 의도된 차이 — 통과시키되 반드시 보여준다
                if not any(d[0] == name for d in devs):
                    devs.append((name, want, m.group(1), reason))
                lines.append((f"{name}·{Path(f).stem}", m.group(1), want,
                              "의도된 차이"))
                continue
            if not good:
                fails.append(f"{name} ({f}): {m.group(1)} — 문서는 {want}")
            lines.append((f"{name}·{Path(f).stem}", m.group(1), want,
                          "일치" if good else "불일치"))
    return fails, lines, devs


def cmp_value(got, want):
    if isinstance(want, list):
        return (len(got) == len(want)
                and all(near(a, b) for a, b in zip(got, want)))
    return near(got, want)


def fmt(v):
    if isinstance(v, list):
        return "[" + ", ".join(f"{float(x):g}" for x in v) + "]"
    return f"{float(v):g}"


def main():
    if not DESIGN.is_file():
        print(f"[중단] {DESIGN} 이 없다. tools/extract_design.py 를 먼저 실행할 것")
        return 1
    text = DESIGN.read_text()
    meta = json.loads(META.read_text())

    hard_fail = []
    deviations = []
    print("=" * 92)
    print("설계 문서 ↔ parts_meta 대조")
    print("=" * 92)
    print(f"  문서 {DESIGN.name}   {len(text.splitlines())}줄")
    print(f"  {'항목':<16}{'문서':>12}{'parts_meta':>14}  판정")
    print("-" * 92)

    for name, phrase, getter, reason in CHECKS:
        if phrase not in text:
            hard_fail.append(f"{name}: 문서에서 '{phrase}' 를 못 찾았다 "
                             f"(문서가 개정됐으면 대조표를 갱신할 것)")
            print(f"  {name:<16}{'?':>12}{'?':>14}  문서 문구 없음")
            continue
        try:
            got, want = getter(meta)
        except KeyError as exc:
            hard_fail.append(f"{name}: parts_meta 에 {exc} 가 없다 — 누락")
            print(f"  {name:<16}{fmt_want(getter):>12}{'없음':>14}  누락")
            continue
        if cmp_value(got, want):
            print(f"  {name:<16}{fmt(want):>12}{fmt(got):>14}  일치")
        elif reason:
            deviations.append((name, want, got, reason))
            print(f"  {name:<16}{fmt(want):>12}{fmt(got):>14}  의도된 차이")
        else:
            hard_fail.append(f"{name}: 문서 {fmt(want)} vs meta {fmt(got)}")
            print(f"  {name:<16}{fmt(want):>12}{fmt(got):>14}  불일치")

    if deviations:
        print("\n" + "-" * 92)
        print("  의도된 차이 — 이유가 적혀 있어 통과시킨다. 하지만 매번 보여준다")
        for name, want, got, reason in deviations:
            print(f"\n    {name}  문서 {fmt(want)} → 구현 {fmt(got)}")
            for line in _wrap(reason, 84):
                print(f"      {line}")

    # ── 물리 설정 ────────────────────────────────────────────────
    print("\n" + "=" * 92)
    print("  물리 설정 — parts_meta 가 아니라 스크립트에 들어 있다")
    print("=" * 92)
    pf, plines, pdevs = check_physics(text)
    hard_fail += pf
    deviations += pdevs
    print(f"  {'항목':<28}{'소스':>12}{'문서':>12}  판정")
    print("-" * 92)
    for name, got, want, verdict in plines:
        print(f"  {name:<28}{got:>12}{want:>12}  {verdict}")
    for name, want, got, reason in pdevs:
        print(f"\n    {name}  문서 {want} → 구현 {got}")
        for line in _wrap(reason, 84):
            print(f"      {line}")

    # ── 대조표에 없는 문서 치수 ──────────────────────────────────
    print("\n" + "=" * 92)
    print("  대조표가 아직 안 다루는 문서 치수 (누락 후보)")
    print("=" * 92)
    covered = " ".join(p for _, p, _, _ in CHECKS)
    body = text.split("\n")
    cand = []
    for line in body:
        if line.startswith("#") or len(line) > 60:
            continue                       # 서술 문단은 치수표가 아니다
        for tok in re.findall(r"\d+(?:\.\d+)?\s*(?:mm|°|N/mm|N·m|N)\b", line):
            if tok not in covered and line not in covered:
                cand.append((tok.strip(), line.strip()))
    seen = set()
    shown = 0
    for tok, line in cand:
        if line in seen:
            continue
        seen.add(line)
        shown += 1
        if shown <= 20:
            print(f"    {tok:>12}   {line[:60]}")
    print(f"    ... 총 {len(seen)}줄. 여기 있는 값이 parts_meta 에 필요한지")
    print("    사람이 판단해 CHECKS 에 넣을 것. 넣기 전까지는 검사되지 않는다.")

    print("\n" + "=" * 92)
    if hard_fail:
        print(f"전체 판정: 실패 ({len(hard_fail)}건)")
        for f in hard_fail:
            print(f"  - {f}")
        return 1
    print(f"전체 판정: 통과  (대조 {len(CHECKS)}건, "
          f"의도된 차이 {len(deviations)}건)")
    return 0


def fmt_want(getter):
    return "?"


def _wrap(s, n):
    out, cur = [], ""
    for w in s.split():
        if len(cur) + len(w) + 1 > n:
            out.append(cur)
            cur = w
        else:
            cur = f"{cur} {w}".strip()
    if cur:
        out.append(cur)
    return out


if __name__ == "__main__":
    sys.exit(main())
