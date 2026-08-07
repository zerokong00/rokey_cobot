"""[오프라인] 균열 주입 검증 — 시각층은 얇은가, 물리층은 실제로 뚫렸는가.

CAD 팀은 깨끗한 배관만 만든다. 균열은 우리가 넣는다.
눈에 진짜 같은 균열(1.6mm)은 입자 해상도상 물리적으로 샐 수 없어
시각층과 물리층을 나눴다. 그 두 층이 각각 제 역할을 하는지 본다.

실행:  python3 test_crack_inject.py
"""

import math
import sys
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
SON = HERE.parents[1]
sys.path.insert(0, str(SON / "pipe"))

import crack_inject as CI

R_IN, R_OUT = 50.0, 57.0
LEN = 600.0


def mesh(v, f):
    return trimesh.Trimesh(vertices=v, faces=f, process=False)


def main():
    ok = []
    print("=" * 78)
    print("① 사이징 규칙 — dongmin 실측 재현")
    print("=" * 78)
    for d, want in ((14.0, False), (30.0, True)):
        eff = CI.effective_bore_mm(d)
        got = CI.leaks(d)
        good = got == want
        ok.append(good)
        print(f"  ø{d:.0f} 개구 → 실효 ø{eff:.1f}  "
              f"{'샌다' if got else '안 샌다':>7} (실측 {'샌다' if want else '안 샌다'}) "
              f"{'OK' if good else 'FAIL'}")

    need = CI.port_diameter_mm()
    print(f"\n  기본 입자(지름 7.2mm, PCO 6mm)에서 필요한 최소 개구 ø{need:.1f}")
    ok.append(CI.leaks(need))

    print(f"\n  [층을 나눈 근거] 폭 1.6mm 를 그대로 뚫으려면 입자를 "
          f"{CI.why_two_layers(1.6):.1f}배 줄여야 하고 개수는 "
          f"{int(CI.why_two_layers(1.6))**3:,}배가 된다 — 연산이 안 된다.")
    print("  입자 설정은 팀에서 이미 조율한 값이므로 건드리지 않는다.")
    ok.append(CI.why_two_layers(1.6) > 10)

    print("\n" + "=" * 78)
    print("② 물리층 — 정말로 관통됐는가")
    print("=" * 78)
    v, f = CI.leak_port_mesh("straight", 0.0, 0.0, need, R_IN, R_OUT,
                             length_mm=LEN)
    m = mesh(v, f)
    plain_v, plain_f = CI.tube_with_openings(
        *CI.straight_frames(LEN, int(LEN / 4) + 1)[1:], R_IN, R_OUT, 96)
    plain = mesh(plain_v, plain_f)
    print(f"  삼각형 {len(f):,}  정점 {len(v):,}")
    print(f"  부피 {m.volume:.1f} vs 구멍 없는 관 {plain.volume:.1f} "
          f"→ 제거 {plain.volume - m.volume:.1f} mm^3")
    ok.append(m.volume < plain.volume)

    # 개구부가 실제로 열렸는지 — 안쪽/바깥쪽 면 모두에서 그 자리에 면이 없어야 한다
    # 개구는 (축, 호) 평면에서 반지름 need/2 인 원이다. 그 안쪽만 봐야 한다.
    tri = v[f]
    cen = tri.mean(axis=1)
    r = np.hypot(cen[:, 1], cen[:, 2])
    arc = np.arctan2(cen[:, 1], cen[:, 2]) * R_IN
    inside = np.hypot(cen[:, 0], arc) < need / 2 * 0.7
    inner_face = inside & (np.abs(r - R_IN) < 1.0)
    outer_face = inside & (np.abs(r - R_OUT) < 1.0)
    print(f"  개구 원(반경 {need/2*0.7:.1f}mm) 안에 남은 면: "
          f"안쪽 {int(inner_face.sum())}  바깥쪽 {int(outer_face.sum())}")
    print(f"  → 둘 다 0 이어야 관통이다")
    ok.append(inner_face.sum() == 0 and outer_face.sum() == 0)

    # 옆벽이 세워졌는지 (벽 두께 방향 면)
    side = (np.hypot(cen[:, 0], arc) < need / 2 * 1.3) \
        & (r > R_IN + 1.0) & (r < R_OUT - 1.0)
    print(f"  개구 옆벽 면 {int(side.sum())}개 (0 이면 벽 속이 뚫려 보인다)")
    ok.append(side.sum() > 0)

    print("\n" + "=" * 78)
    print("③ 시각층 — 얇고 얕은가")
    print("=" * 78)
    vv, vf = CI.crack_visual_mesh("straight", 0.0, 0.0, 1.6, 18.0, 2.2, R_IN,
                                  length_mm=LEN)
    rr = np.hypot(vv[:, 1], vv[:, 2])
    deep = vv[rr > R_IN + 0.3]
    arc = np.degrees(np.arctan2(deep[:, 1], deep[:, 2])) / 180 * math.pi * R_IN
    print(f"  삼각형 {len(vf):,}")
    print(f"  파임 최대 {rr.max() - R_IN:+.2f} mm  (설정 2.2)")
    print(f"  균열 길이 {deep[:, 0].max() - deep[:, 0].min():.1f} mm  (설정 18)")
    print(f"  균열 폭   {arc.max() - arc.min():.2f} mm  (설정 1.6)")
    ok += [abs(rr.max() - R_IN - 2.2) < 0.05,
           deep[:, 0].max() - deep[:, 0].min() <= 20.0,
           arc.max() - arc.min() <= 2.2]

    print("\n" + "=" * 78)
    print("④ 두 층의 크기 대비 — 이게 맞바꿈의 실체다")
    print("=" * 78)
    print(f"  {'':14s} {'폭/지름':>10} {'카메라에 보이는 것':>20}")
    print(f"  {'시각층 균열':14s} {1.6:9.1f}mm {'얇은 균열':>20}")
    print(f"  {'물리층 개구':14s} {need:9.1f}mm {'안 보임 (invisible)':>20}")
    print(f"\n  물리층이 시각층보다 {need/1.6:.0f}배 크다. 숨기지 않으면 티가 난다.")
    print(f"  → 물리층은 visibility=invisible + 콜라이더만, 렌더 안 함")
    print(f"  → 시각층은 콜라이더 없음, 렌더만")

    print("\n" + "=" * 78)
    print("⑤ 곡관에도 되는가")
    print("=" * 78)
    ev, ef = CI.leak_port_mesh("elbow", 78.5, 0.0, need, R_IN, R_OUT,
                               bend_r_mm=100.0, sweep_deg=90.0)
    em = mesh(ev, ef)
    print(f"  곡관 물리층 삼각형 {len(ef):,}  부피 {em.volume:.1f} mm^3")
    cen = ev[ef].mean(axis=1)
    rho = np.hypot(cen[:, 0], cen[:, 2] - 100.0)
    dist = np.hypot(rho - 100.0, cen[:, 1])
    t = np.arctan2(cen[:, 0], 100.0 - cen[:, 2]) * 100.0
    # 곡관은 φ=0 이 곡률 안쪽(us 가 중심을 향함)이다. 부호를 맞춘다.
    arc_e = np.arctan2(cen[:, 1], 100.0 - rho) * R_IN
    inside_e = np.hypot(t - 78.5, arc_e) < need / 2 * 0.7
    near = inside_e & (np.abs(dist - R_IN) < 1.0)
    print(f"  호 78.5mm 개구 원 안에 남은 안쪽 면 {int(near.sum())}개 (0 이어야 함)")
    ok.append(near.sum() == 0)

    print("\n" + "=" * 78)
    print("⑥ 바퀴가 개구에 빠지지 않는가 — 위치로 푼다")
    print("=" * 78)
    half = CI.wheel_track_half_deg(R_IN)
    print(f"  휠 접지 궤도 {list(CI.WHEEL_TRACKS_DEG)} (도), 각 ±{half:.1f}°")
    print(f"  개구 ø{need:.1f} → 필요한 반각 "
          f"{math.degrees(need/2/R_IN):.1f}°")
    safe = CI.safe_angles_deg(R_IN, need)
    print(f"  안전 각도 {len(safe)}곳: {[round(a,1) for a in safe]}")
    ok.append(len(safe) == 6)
    for a in safe:
        ok.append(not CI.is_on_wheel_track(a, R_IN, need))
    print(f"  → 전부 궤도와 안 겹침 "
          f"{'OK' if all(not CI.is_on_wheel_track(a, R_IN, need) for a in safe) else 'FAIL'}")

    print("\n  요청 각도를 안전한 곳으로 옮기는가")
    for req in (0.0, 5.0, 30.0, 62.0, 180.0):
        got, moved = CI.snap_to_safe(req, R_IN, need)
        on = CI.is_on_wheel_track(req, R_IN, need)
        print(f"    요청 {req:6.1f}° ({'궤도 위' if on else '안전':>5}) → "
              f"{got:6.1f}° {'이동' if moved else '유지'}")
        ok.append(not CI.is_on_wheel_track(got, R_IN, need))

    print("\n  왜 슬롯으로는 안 되는가")
    print(f"  {'축방향 길이':>12} {'바퀴 빠짐':>12} {'입자 통과':>10}")
    for L in (8.0, 20.0, need):
        drop = (R_IN * 0 + 10.0 - math.sqrt(max(100.0 - (L / 2) ** 2, 0.0))
                if L / 2 < 10.0 else 10.0)
        fell = "완전히" if L / 2 >= 10.0 else f"{drop:.2f}mm"
        print(f"  {L:11.1f}mm {fell:>12} "
              f"{'통과' if L >= need else '막힘':>10}")
    print("  → 2xPCO(12mm)가 어느 방향이든 마개라 좁은 쪽도 28.8 이상이어야 한다")

    print("\n" + "=" * 78)
    print("⑦ 무작위 배치 — 시연용 2개")
    print("=" * 78)
    a = CI.random_sites(2, "straight", R_IN, seed=0, length_mm=LEN)
    b = CI.random_sites(2, "straight", R_IN, seed=0, length_mm=LEN)
    c = CI.random_sites(2, "straight", R_IN, seed=1, length_mm=LEN)
    print(f"  같은 시드 = 같은 배치 {'OK' if a == b else 'FAIL'}")
    print(f"  다른 시드 = 다른 배치 {'OK' if a != c else 'FAIL'}")
    ok += [a == b, a != c]

    lo, hi = CI.pipe_span_mm("straight", length_mm=LEN)
    print(f"\n  {'#':>2} {'호위치mm':>9} {'각도':>6} {'폭mm':>6} {'깊이mm':>7}"
          f"  바퀴  관안")
    for i, (s_mm, ang, w, ln, dep) in enumerate(a):
        on = CI.is_on_wheel_track(ang, R_IN, need)
        inside = lo + need / 2 <= s_mm - ln / 2 and s_mm + ln / 2 <= hi - need / 2
        deep = dep > 0.6                     # welder depth_defect_mm
        thin = dep < R_OUT - R_IN
        print(f"  {i:>2} {s_mm:9.1f} {ang:6.1f} {w:6.2f} {dep:7.2f}"
              f"  {'걸림!' if on else '안 걸림'}  {'OK' if inside else 'FAIL'}")
        ok += [not on, inside, deep, thin]

    gap = abs(a[1][0] - a[0][0])
    print(f"\n  두 균열 간격 {gap:.0f}mm >= 용접 후퇴 120mm "
          f"{'OK' if gap >= 150 else 'FAIL'}")
    ok.append(gap >= CI.MIN_SITE_GAP_MM)

    print("\n  뽑은 자리가 실제로 메시가 되는가")
    for i, (s_mm, ang, w, ln, dep) in enumerate(a):
        pv, pf = CI.leak_port_mesh("straight", s_mm, ang, need, R_IN, R_OUT,
                                   length_mm=LEN)
        cv, cf = CI.crack_visual_mesh("straight", s_mm, ang, w, ln, dep, R_IN,
                                      length_mm=LEN)
        # ② 와 같은 판정 — 개구 원 안에 안쪽/바깥쪽 면이 남아 있으면 안 뚫린 것.
        # 옆벽이 세워져 있어 메시는 watertight 로 남는다. 터널이지 틈이 아니다.
        cen = pv[pf].mean(axis=1)
        rr = np.hypot(cen[:, 1], cen[:, 2])
        da = (np.arctan2(cen[:, 1], cen[:, 2]) - math.radians(ang)
              + np.pi) % (2 * np.pi) - np.pi
        near = np.hypot(cen[:, 0] - s_mm, da * R_IN) < need / 2 * 0.7
        left = int((near & ((np.abs(rr - R_IN) < 1.0)
                            | (np.abs(rr - R_OUT) < 1.0))).sum())
        holed = left == 0
        print(f"    {i}: 물리층 면 {len(pf):,} 개구 안 남은 면 {left} "
              f"{'뚫림' if holed else '막힘'} / 시각층 면 {len(cf):,}")
        ok += [holed, len(cf) > 0]

    print("\n  곡관 하나에 2개는 안 들어간다 — 조용히 줄이지 않고 실패하는가")
    try:
        CI.random_sites(2, "elbow", R_IN, seed=0, bend_r_mm=100.0)
        print("    FAIL — 예외가 안 났다")
        ok.append(False)
    except ValueError as e:
        print(f"    OK — {e}")
        ok.append(True)

    print("\n  맵 전체에 흩어 놓기 (직관3 + 곡관2)")
    segs = [dict(kind="straight", r_in_mm=R_IN, length_mm=LEN),
            dict(kind="elbow", r_in_mm=R_IN, bend_r_mm=100.0, sweep_deg=90.0),
            dict(kind="straight", r_in_mm=R_IN, length_mm=LEN),
            dict(kind="elbow", r_in_mm=R_IN, bend_r_mm=100.0, sweep_deg=90.0),
            dict(kind="straight", r_in_mm=R_IN, length_mm=LEN)]
    for seed in range(6):
        out = CI.random_sites_over(segs, 2, seed=seed)
        idxs = [i for i, _ in out]
        one_each = all(len(v) == 1 for _, v in out) and len(out) == 2
        apart = idxs[1] - idxs[0] >= 2
        safe_ang = all(not CI.is_on_wheel_track(st[1], R_IN, need)
                       for _, v in out for st in v)
        print(f"    시드 {seed}: 구간 {idxs}  "
              f"{'하나씩' if one_each else '몰림'}  "
              f"{'안 붙음' if apart else '붙음!'}  "
              f"{'각도OK' if safe_ang else '각도FAIL'}")
        ok += [one_each, apart, safe_ang]

    print("=" * 78)
    print(f"전체 판정: {'통과' if all(ok) else '실패'}  ({sum(ok)}/{len(ok)})")
    print("  실패 항목 번호:", [i for i, x in enumerate(ok) if not x])
    return 0 if all(ok) else 1


if __name__ == "__main__":
    sys.exit(main())
