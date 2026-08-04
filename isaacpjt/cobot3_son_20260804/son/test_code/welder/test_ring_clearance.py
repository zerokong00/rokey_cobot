"""[오프라인] 토치 링 간섭 검사 — 링이 돌 때 암·휠과 부딪히지 않는가.

바퀴는 전진·후진만 되고 조향이 없어 로봇을 결함 방향으로 돌릴 수 없다.
대신 본체를 감싸는 링이 돌아 토치를 겨눈다. 그 링이 서스펜션 암과
축 방향으로 겹치면 어느 각도에서 반드시 충돌한다.

같이 확인하는 것
  - 링·로드·토치 끝이 관벽(50mm)을 넘지 않는가
  - 토치가 전 원주(±180°)에 도달하는가
  - 카메라 시야를 가리지 않는가
  - 용접 지점을 카메라가 볼 수 있는가  ← 여기서 문제가 드러난다

실행:  python3 test_ring_clearance.py
"""

import json
import math
import sys
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
SON = HERE.parents[1]
META = json.loads((SON / "spec" / "parts_meta.json").read_text())
T = META["torch"]
BORE = META["pipe_id"] / 2.0
HFOV = 140.0


def rot_x(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    m = np.eye(4)
    m[1, 1], m[1, 2], m[2, 1], m[2, 2] = c, s, -s, c
    return m


def trans(x, y, z):
    m = np.eye(4)
    m[3, :3] = [x, y, z]
    return m


def load(cat, name):
    return trimesh.load(str(SON / cat / "meshes" / f"{name}.stl"))


def placed(mesh, M):
    v = np.asarray(mesh.vertices)
    return (np.column_stack([v, np.ones(len(v))]) @ M)[:, :3]


def arm_slots():
    dx, st = META["seg_center_offset"], META["stagger"]
    for tag, xc, angles in (("r", -dx, META["rear_arms"]),
                            ("f", +dx, META["front_arms"])):
        for i, phi in enumerate(angles):
            yield tag, xc, i, phi, (i - 1) * st


def main():
    ok = []
    ring_x, rw = T["ring_x"], T["ring_w"]
    ring_lo, ring_hi = ring_x - rw / 2, ring_x + rw / 2

    print("=" * 76)
    print("① 축 방향 간섭 — 링과 암이 겹치는가")
    print("=" * 76)
    print(f"  링          x {ring_lo:+7.1f} ~ {ring_hi:+7.1f}  (폭 {rw:.0f}mm)")

    arm = load("robot", "arm")
    wheel = load("robot", "wheel")
    pivot_r = META["pivot_r"]
    adx, adr = META["arm_dx_nominal"], META["arm_dr_nominal"]

    worst = None
    for tag, xc, i, phi, xo in arm_slots():
        M = trans(0, 0, pivot_r) @ rot_x(-phi) @ trans(xc + xo, 0, 0)
        av = placed(arm, M)
        Mw = trans(-adx, 0, pivot_r + adr) @ rot_x(-phi) @ trans(xc + xo, 0, 0)
        wv = placed(wheel, Mw)
        v = np.vstack([av, wv])
        lo, hi = v[:, 0].min(), v[:, 0].max()
        if worst is None or hi > worst[1]:
            worst = (f"arm_{tag}{i}", hi)
        gap = ring_lo - hi
        print(f"  arm_{tag}{i}      x {lo:+7.1f} ~ {hi:+7.1f}   "
              f"링까지 {gap:+7.1f} mm  {'OK' if gap > 0 else '충돌'}")
        ok.append(gap > 0)

    print(f"\n  가장 앞선 암·휠: {worst[0]} 의 x={worst[1]:+.1f}  "
          f"→ 링 시작 {ring_lo:+.1f} 까지 {ring_lo - worst[1]:+.1f} mm 여유")

    print("\n" + "=" * 76)
    print("② 반경 방향 — 360° 도는 동안 관벽을 넘지 않는가")
    print("=" * 76)
    ring = load("welder", "torch_ring")
    rod = load("welder", "torch_rod")
    tip = load("welder", "torch_tip")
    r0 = T["rod_origin_r"]
    ELBOW_CLEAR = 41.69         # 곡관에서 본체·암이 도달하는 최대 반경(실측)

    for ext_label, ext, limit in (
            ("수납 (신장 0)", 0.0, ELBOW_CLEAR),
            ("최대 신장", T["j2_stroke_mm"], BORE)):
        rmax = 0.0
        for j1 in range(-180, 181, 10):
            Mring = rot_x(j1) @ trans(ring_x, 0, 0)
            Mrod = trans(0, 0, r0 + ext) @ rot_x(j1) @ trans(ring_x, 0, 0)
            Mtip = trans(0, 0, r0 + T["rod_len"] + ext) @ rot_x(j1) \
                @ trans(ring_x, 0, 0)
            v = np.vstack([placed(ring, Mring), placed(rod, Mrod),
                           placed(tip, Mtip)])
            rmax = max(rmax, float(np.hypot(v[:, 1], v[:, 2]).max()))
        good = rmax <= limit + 0.01
        ok.append(good)
        note = "곡관 통과 한계" if ext == 0.0 else "관벽"
        print(f"  {ext_label:16s} 최대 반경 {rmax:6.2f} mm  "
              f"({note} {limit:.0f}) {'OK' if good else '초과'}")

    reach = T["reach_radius_mm"]
    print(f"\n  토치 끝 최대 도달 반경 {reach:.1f} mm  "
          f"→ 관벽까지 간극 {BORE - reach:+.1f} mm  (용접 간극)")
    ok.append(0.0 <= BORE - reach <= 5.0)
    print(f"  수납 시 반경 {T['stow_radius_mm']:.1f} mm  "
          f"→ 곡관 한계 {ELBOW_CLEAR:.0f} 까지 {ELBOW_CLEAR - T['stow_radius_mm']:+.1f} mm")

    print("\n" + "=" * 76)
    print("③ 원주 도달 — ±180° 로 전 방향이 되는가")
    print("=" * 76)
    print(f"  J1 한계 ±{T['j1_limit_deg']:.0f}°  →  "
          f"시계·반시계 중 가까운 쪽으로 돌면 전 방향 도달")
    print("  360° 연속 회전은 슬립링이 필요하고, 슬립링은 용접 전류에 부적합하다")
    ok.append(T["j1_limit_deg"] >= 180.0)

    print("\n" + "=" * 76)
    print("④ 카메라 — 링이 시야를 가리는가")
    print("=" * 76)
    import yaml
    cam_x = yaml.safe_load(
        (SON / "camera" / "config" / "camera.yaml").read_text()
    )["camera"]["front"]["mount_x_mm"]
    dx = ring_x - cam_x
    cam_lo, cam_hi = cam_x - 5.1, cam_x + 5.1
    overlap = not (cam_lo >= ring_hi or cam_hi <= ring_lo)
    ok.append(not overlap)
    print(f"  카메라 하우징 x {cam_lo:+.1f} ~ {cam_hi:+.1f}, "
          f"링 x {ring_lo:+.1f} ~ {ring_hi:+.1f}  →  "
          f"{'겹침' if overlap else '안 겹침'}")
    print(f"  카메라 x {cam_x:+.1f} (선단), 링 x {ring_x:+.1f}  →  "
          f"링은 카메라 {'뒤' if dx < 0 else '앞'}쪽 {abs(dx):.1f}mm")
    blocked = dx > 0 and math.degrees(math.atan2(T["ring_r_in"], dx)) < HFOV / 2
    ok.append(not blocked)
    print(f"  가림 여부: {'가림' if blocked else '안 가림'}")

    print("\n" + "=" * 76)
    print("⑤ 용접 지점을 카메라가 볼 수 있는가")
    print("=" * 76)
    zmin = BORE / math.tan(math.radians(HFOV / 2))
    print(f"  화각 {HFOV:.0f}° 에서 관벽이 시야에 들어오려면 전방 {zmin:.1f}mm 이상")
    weld_dx = ring_x - cam_x
    th = math.degrees(math.atan2(BORE, weld_dx)) if weld_dx > 0 else 180.0
    visible = 0 < weld_dx and th < HFOV / 2
    print(f"  용접 지점은 카메라에서 x {weld_dx:+.1f}mm, 입사각 {th:.1f}°")
    print(f"  →  {'보인다' if visible else '안 보인다'}")
    if not visible:
        print("\n  [중요] 링 위치를 바꿔도 해결되지 않는다.")
        print("         용접 지점은 관벽에 있고 로봇 바로 옆이라, 전방을 보는")
        print("         카메라는 원리적으로 '옆의 벽'을 볼 수 없다.")
        print("         검증 촬영은 로봇을 이동시킨 뒤 해야 한다.")
        print(f"         전진 후 후방 카메라로 보려면 최소 {zmin:.0f}mm 이상 이동")

    print("=" * 76)
    print(f"전체 판정: {'통과' if all(ok) else '실패'}  ({sum(ok)}/{len(ok)})")
    return 0 if all(ok) else 1


if __name__ == "__main__":
    sys.exit(main())
