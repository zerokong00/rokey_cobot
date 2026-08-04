"""[오프라인] 판정기 시험

실행:  python3 make_scenes.py && python3 test_detector.py
"""

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "condition"))
from detector import DEFAULTS, PipeConditionDetector

HERE = Path(__file__).resolve().parent
SCENES = HERE / "scenes"
OFFSET_TOL_MM = 1.5


def expected_speed(truth, off):
    if truth in ("DISCONNECTED", "UNDETERMINED"):
        return "stop"
    if off <= DEFAULTS["offset_normal_mm"] or off <= DEFAULTS["offset_slow_mm"]:
        return "full"
    if off <= DEFAULTS["offset_max_mm"]:
        return "slow"
    return "stop"


def main():
    meta = json.loads((SCENES / "scenes_meta.json").read_text())
    det = PipeConditionDetector(meta)
    rows, ok = [], True

    print("=" * 104)
    print(f"{'장면':20s} {'정답':14s} {'판정':14s} {'원형도':>7s} {'조인트(m)':>10s} "
          f"{'오프셋':>10s} {'무효':>7s} {'속도':>6s}  결과")
    print("=" * 104)
    for name, info in meta["scenes"].items():
        truth, off = info["truth"], info["offset_mm"]
        depth = np.load(SCENES / f"{name}_depth.npy")
        c = det.run(depth, joint_angle_deg=0.0)
        good = c.state == truth
        if truth == "MISALIGNMENT":
            good = good and abs(c.offset_mm - off) <= OFFSET_TOL_MM
        good = good and c.speed == expected_speed(truth, off)
        ok = ok and good
        rows.append((truth, c))
        exp = f"  (기대 {off:.0f}mm)" if truth == "MISALIGNMENT" else ""
        print(f"{name:20s} {truth:14s} {c.state:14s} {c.circularity:7.3f} "
              f"{c.joint_range_m:10.3f} {c.offset_mm:8.2f}mm "
              f"{c.invalid_ratio * 100:6.2f}% {c.speed:>6s}  "
              f"{'OK' if good else 'FAIL'}{exp}")
        if not good:
            print(f"{'':20s} → {c.reason}")

    print("-" * 104)
    depth = np.load(SCENES / "disconnected_depth.npy")
    c = det.run(depth, joint_angle_deg=20.0)
    bend_ok = c.state == "NORMAL"
    ok = ok and bend_ok
    print(f"곡관 예외 (관절 20°, 단절 장면) → {c.state}  "
          f"{'OK' if bend_ok else 'FAIL'}   {c.reason}")

    print("-" * 104)
    print("임계값 분리도 — 두 무리 사이가 벌어져 있어야 임계값이 안정적이다")

    def spread(pred, field):
        v = [getattr(c, field) for t, c in rows if pred(t, c)]
        return (min(v), max(v)) if v else (float('nan'),) * 2

    inv_ok = spread(lambda t, c: t != "DISCONNECTED", "invalid_ratio")
    inv_bad = spread(lambda t, c: t == "DISCONNECTED", "invalid_ratio")
    print(f"  무효 픽셀 비율   정상군 {inv_ok[0] * 100:6.2f}~{inv_ok[1] * 100:6.2f}%"
          f"   단절군 {inv_bad[0] * 100:6.2f}~{inv_bad[1] * 100:6.2f}%"
          f"   임계 {DEFAULTS['invalid_ratio_max'] * 100:.1f}%")

    for field, key, lo_is_ok, fmt in (
            ("circularity", "circularity_min", True, "6.3f"),
            ("roughness", "roughness_max", False, "7.4f")):
        a = spread(lambda t, c: t in ("NORMAL", "MISALIGNMENT"), field)
        b = spread(lambda t, c: t == "UNDETERMINED", field)
        gap = (a[0] - b[1]) if lo_is_ok else (b[0] - a[1])
        print(f"  {field:12s}     정상군 {a[0]:{fmt}}~{a[1]:{fmt}}"
              f"   불규칙군 {b[0]:{fmt}}~{b[1]:{fmt}}"
              f"   임계 {DEFAULTS[key]}")
        print(f"  {'':12s}     → 간극 {gap:+.4f}  "
              f"({'분리됨' if gap > 0 else '겹침 — 이 지표로는 못 가른다'})")

    print("-" * 104)
    errs = [(abs(c.offset_mm - o), o, c.offset_mm)
            for (t, c), o in zip(rows, [i["offset_mm"] for i in
                                        meta["scenes"].values()])
            if t == "MISALIGNMENT"]
    if errs:
        print(f"  오프셋 실측 오차 최대 {max(e[0] for e in errs):.2f} mm  "
              f"(허용 {OFFSET_TOL_MM} mm)")

    print("=" * 104)
    print("전체 판정:", "통과" if ok else "실패")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
