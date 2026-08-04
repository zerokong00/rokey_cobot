"""[오프라인] 조립 결과 확인 — Isaac Sim 없이 형상·간섭만 검사하고 그림으로 낸다.

robot_articulated.py 와 같은 변환식을 써서 링크를 배치하므로, 이 스크립트가
통과하면 articulation 의 배치 좌표는 맞는 것이다. 물리(스프링·마찰·접촉)는
검증하지 않는다.

실행:  python3 preview_assembly.py [--bend 25]
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
SON = HERE.parents[1]
MESHES = SON / "robot" / "meshes"
META = json.loads((SON / "spec" / "parts_meta.json").read_text())

SEG_DX = META["seg_center_offset"]
PIVOT_R = META["pivot_r"]
ARM_DX = META["arm_dx_nominal"]
ARM_DR = META["arm_dr_nominal"]
WHEEL_R = META["wheel_r"]
STAGGER = META["stagger"]
PIPE_ID = META["pipe_id"]
ELBOW_R = META["elbow_r"]


def trans(x, y, z):
    m = np.eye(4)
    m[3, :3] = [x, y, z]
    return m


def rot(deg, axis):
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    m = np.eye(4)
    if axis == "x":
        m[:3, :3] = [[1, 0, 0], [0, c, s], [0, -s, c]]
    elif axis == "y":
        m[:3, :3] = [[c, 0, -s], [0, 1, 0], [s, 0, c]]
    return m


def apply(mesh, M):
    m = mesh.copy()
    v = np.asarray(m.vertices)
    m.vertices = (np.column_stack([v, np.ones(len(v))]) @ M)[:, :3]
    return m


def arm_slots():
    for tag, xc, angles in (("r", -SEG_DX, META["rear_arms"]),
                            ("f", +SEG_DX, META["front_arms"])):
        for i, phi in enumerate(angles):
            yield tag, xc, i, phi, (i - 1) * STAGGER


_PART_CACHE = {}


def part_meshes():
    if not _PART_CACHE:
        for n in ("body_rear", "body_front", "bellows", "arm", "wheel"):
            _PART_CACHE[n] = trimesh.load(str(MESHES / f"{n}.stl"))
    return _PART_CACHE


def link_frames(bend_deg=0.0, deltas=None):
    """링크별 4x4 배치 행렬. deltas 는 암 6개의 조인트 각도(도), 기본 0."""
    bend = rot(bend_deg, "y")
    out = {"body_rear": trans(-SEG_DX, 0, 0),
           "bellows": rot(bend_deg / 2.0, "y"),
           "body_front": trans(+SEG_DX, 0, 0) @ bend}
    for k, (tag, xc, i, phi, xo) in enumerate(arm_slots()):
        post = bend if tag == "f" else np.eye(4)
        seat = trans(0, 0, PIVOT_R) @ rot(-phi, "x") @ trans(xc + xo, 0, 0) @ post
        d = rot(deltas[k] if deltas is not None else 0.0, "y")
        out[f"arm_{tag}{i}"] = d @ seat
        out[f"wheel_{tag}{i}"] = trans(-ARM_DX, 0, ARM_DR) @ d @ seat
    return out


def wheel_center(frame):
    return (np.array([0.0, 0.0, 0.0, 1.0]) @ frame)[:3]


def build(bend_deg=0.0, deltas=None):
    part = part_meshes()
    src = {"body_rear": "body_rear", "body_front": "body_front",
           "bellows": "bellows"}
    frames = link_frames(bend_deg, deltas)
    links = {}
    for name, M in frames.items():
        key = src.get(name, "arm" if name.startswith("arm") else "wheel")
        links[name] = apply(part[key], M)
    return links


def radial_report(links):
    print("=" * 74)
    print(f"직관 내부 간섭 검사 (DN100 내반경 {PIPE_ID / 2:.1f} mm)")
    print("=" * 74)
    worst = 0.0
    for name, m in sorted(links.items()):
        v = np.asarray(m.vertices)
        r = np.hypot(v[:, 1], v[:, 2]).max()
        worst = max(worst, r)
        mark = "접촉" if name.startswith("wheel") else ("간섭" if r > PIPE_ID / 2 else "여유")
        print(f"  {name:14s} 최대반경 {r:7.2f} mm   {mark} "
              f"{abs(PIPE_ID / 2 - r):5.2f} mm")
    print(f"\n  전체 최대반경 {worst:.2f} mm  "
          f"→ {'OK' if worst <= PIPE_ID / 2 + 0.6 else 'FAIL'}")
    return worst


def wall_distance(verts):
    """곡관 중심선(반경 ELBOW_R, 중심 (0,0,ELBOW_R), XZ 평면)까지의 거리."""
    rho = np.hypot(verts[:, 0], verts[:, 2] - ELBOW_R)
    return np.hypot(rho - ELBOW_R, verts[:, 1])


def place(verts, theta, tx, tz):
    M = rot(theta, "y") @ trans(tx, 0.0, tz)
    return (np.column_stack([verts, np.ones(len(verts))]) @ M)[:, :3]


def elbow_fit(verbose=True):
    """곡관 안에 로봇이 들어가는 자세를 수치로 찾는다.

    굽힘각과 강체 배치(회전 + 평행이동)를 동시에 최적화해 모든 정점의
    관벽까지 최대 거리를 최소화한다. 그 최솟값이 내반경 50mm 이하이면
    통과 가능한 자세가 존재한다는 뜻이다.
    """
    from scipy.optimize import minimize

    cache = {}

    def verts_of(bend):
        key = round(bend, 4)
        if key not in cache:
            links = build(key)
            cache[key] = (links,
                          np.concatenate([np.asarray(m.vertices)
                                          for m in links.values()]))
        return cache[key]

    def body_verts(bend):
        links, _ = verts_of(bend)
        return np.concatenate([np.asarray(m.vertices)
                               for k, m in links.items()
                               if not k.startswith("wheel")])

    def wheel_centers(bend):
        links, _ = verts_of(bend)
        return np.array([np.asarray(m.vertices).mean(0)
                         for k, m in sorted(links.items())
                         if k.startswith("wheel")])

    def cost(p):
        bend, theta, tx, tz = p
        if abs(bend) > 55.0:
            return 1e3 + abs(bend)
        return wall_distance(place(body_verts(bend), theta, tx, tz)).max()

    best = None
    guess = math.degrees(2 * math.atan2(SEG_DX, ELBOW_R))
    for b0 in (guess, -guess, 0.5 * guess, -0.5 * guess):
        for th0 in (-b0 / 2.0, 0.0):
            r = minimize(cost, [b0, th0, 0.0, ELBOW_R * 0.5],
                         method="Nelder-Mead",
                         options=dict(maxiter=4000, xatol=1e-4, fatol=1e-5))
            if best is None or r.fun < best.fun:
                best = r

    bend, theta, tx, tz = best.x
    links, _ = verts_of(bend)
    placed = {k: apply(m, rot(theta, "y") @ trans(tx, 0.0, tz))
              for k, m in links.items()}
    dmax = best.fun

    wc = place(wheel_centers(bend), theta, tx, tz)
    wd = wall_distance(wc)
    seat = PIPE_ID / 2.0 - WHEEL_R
    stroke = wd - seat

    if verbose:
        print("=" * 74)
        print(f"곡관 내부 간섭 검사 (SR R={ELBOW_R:.0f}mm, 내반경 {PIPE_ID / 2:.1f}mm)")
        print("=" * 74)
        print(f"  최적 관절각    {bend:+.2f}°   (한계 ±55°, "
              f"{'여유 %.2f°' % (55 - abs(bend)) if abs(bend) < 55 else '초과'})")
        print(f"  본체·암 최대   {dmax:.2f} mm  → ", end="")
        if dmax <= PIPE_ID / 2:
            print(f"OK   관벽까지 여유 {PIPE_ID / 2 - dmax:.2f} mm")
        else:
            print(f"FAIL 간섭 {dmax - PIPE_ID / 2:.2f} mm")
        print(f"\n  휠 중심이 있어야 할 반경 {seat:.1f} mm 기준 편차")
        print(f"  (= 서스펜션이 흡수해야 할 스트로크. 설계 주장 ±2.5mm)")
        names = [k for k in sorted(placed) if k.startswith("wheel")]
        for n, s in zip(names, stroke):
            print(f"    {n:14s} {s:+7.2f} mm")
        print(f"    {'필요 스트로크':14s} ±{max(abs(stroke.min()), abs(stroke.max())):.2f} mm "
              f"(설계 여유 ±6mm → "
              f"{'OK' if max(abs(stroke.min()), abs(stroke.max())) <= 6.0 else 'FAIL'})")
    return dmax, bend, placed


def render(links, path, extra=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.font_manager as fm
    from matplotlib.collections import PolyCollection

    have = {f.name for f in fm.fontManager.ttflist}
    for cand in ("Noto Sans CJK KR", "NanumGothic", "Noto Sans CJK JP"):
        if cand in have:
            plt.rcParams["font.family"] = cand
            break
    plt.rcParams["axes.unicode_minus"] = False

    fig, axes = plt.subplots(1, 2, figsize=(17, 6.4))
    views = [((0, 2), "side  (X-Z)"), ((1, 2), "front  (Y-Z)")]
    for ax, ((a, b), title) in zip(axes, views):
        if extra is not None:
            for m in extra:
                t = m.triangles[:, :, [a, b]]
                ax.add_collection(PolyCollection(t, facecolors="#d8dde3",
                                                 edgecolors="none", alpha=0.5))
        for name, m in links.items():
            col = "#c94f4f" if name.startswith("wheel") else (
                "#4f7fc9" if name.startswith("arm") else "#7d8894")
            t = m.triangles[:, :, [a, b]]
            ax.add_collection(PolyCollection(t, facecolors=col,
                                             edgecolors="#2b3440",
                                             linewidths=0.15, alpha=0.9))
        th = np.linspace(0, 2 * np.pi, 200)
        if a == 1:
            ax.plot(PIPE_ID / 2 * np.cos(th), PIPE_ID / 2 * np.sin(th),
                    "k--", lw=1.1)
        else:
            for s in (-1, 1):
                ax.axhline(s * PIPE_ID / 2, ls="--", c="k", lw=1.1)
        ax.set_aspect("equal")
        ax.autoscale_view()
        ax.set_title(title)
        ax.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(path, dpi=100)
    print(f"  렌더 저장: {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bend", type=float, default=0.0)
    args = ap.parse_args()

    links = build(0.0)
    radial_report(links)
    render(links, str(HERE / "preview_straight.png"))

    print()
    dmax, bend, placed = elbow_fit()
    render(placed, str(HERE / "preview_bent.png"))

    if args.bend:
        print(f"\n지정 굽힘각 {args.bend:.2f}° 로 추가 렌더")
        render(build(args.bend), str(HERE / "preview_custom.png"))


if __name__ == "__main__":
    main()
