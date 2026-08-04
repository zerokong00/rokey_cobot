"""[오프라인] 곡관 통과 애니메이션 — Isaac Sim 없이 기하만으로 GIF 생성.

물리 시뮬이 아니다. 배관 중심선을 따라 로봇을 밀어 넣으면서 매 위치에서
  - 두 세그먼트 중심을 중심선 위에 두는 관절 각도
  - 각 바퀴가 관벽에 닿도록 하는 서스펜션 각도
를 역기하로 풀어 그린다. 접촉·마찰·구동은 포함하지 않는다.

실행:
  python3 animate_curve.py                 # curve_pass.gif
  python3 animate_curve.py --frames 90 --mp4
"""

import argparse
import math
from pathlib import Path

import numpy as np
from scipy.optimize import brentq

import preview_assembly as PA

HERE = Path(__file__).resolve().parent
R = PA.ELBOW_R
BORE = PA.PIPE_ID / 2.0
SEAT = BORE - PA.WHEEL_R
ARC = R * math.pi / 2.0
LEAD = 260.0
TAIL = 260.0


def path_point(s):
    """중심선 위 호길이 s 의 위치와 접선. s<0 입구 직관, 0~ARC 곡관, 이후 출구 직관."""
    if s <= 0.0:
        return np.array([s, 0.0, 0.0]), np.array([1.0, 0.0, 0.0])
    if s >= ARC:
        d = s - ARC
        return np.array([R, 0.0, R + d]), np.array([0.0, 0.0, 1.0])
    t = s / R
    return (np.array([R * math.sin(t), 0.0, R * (1 - math.cos(t))]),
            np.array([math.cos(t), 0.0, math.sin(t)]))


def pose_at(s_w):
    """관절 호길이 s_w 에서 로봇의 굽힘각·전역 회전·관절 위치를 푼다."""
    A, _ = path_point(s_w - PA.SEG_DX)
    B, _ = path_point(s_w + PA.SEG_DX)
    mid = 0.5 * (A + B)
    chord = B - A
    half = np.linalg.norm(chord) / 2.0
    h2 = PA.SEG_DX ** 2 - half ** 2
    n = np.array([-chord[2], 0.0, chord[0]])
    nn = np.linalg.norm(n)
    if h2 <= 0.0 or nn < 1e-12:
        W = mid
    else:
        n = n / nn
        centre = np.array([0.0, 0.0, R])
        if np.dot(mid - centre, n) < 0:
            n = -n
        W = mid + n * math.sqrt(h2)
    u = W - A
    u /= np.linalg.norm(u)
    v = B - W
    v /= np.linalg.norm(v)
    psi = math.degrees(math.atan2(-u[2], u[0]))
    bend = math.degrees(math.atan2(-v[2], v[0])) - psi
    bend = (bend + 180.0) % 360.0 - 180.0
    return bend, psi, W


def wall_dist(p):
    rho = math.hypot(p[0], p[2] - R)
    if p[0] >= 0.0 and p[2] <= R:
        return math.hypot(rho - R, p[1])
    if p[2] > R and p[0] > 0.0:
        return math.hypot(p[0] - R, p[1])
    return math.hypot(p[2], p[1])


LO = PA.META["arm_angle_compressed"] - PA.META["arm_angle_nominal"]
HI = PA.META["arm_angle_extended"] - PA.META["arm_angle_nominal"]
WHEEL_NAMES = [k for k in PA.link_frames(0.0) if k.startswith("wheel")]


def solve_deltas(bend, M):
    """각 바퀴 중심이 관벽에서 휠 반경만큼 떨어지도록 암 각도를 푼다."""
    out = []
    for k, name in enumerate(WHEEL_NAMES):
        def err(d, k=k, name=name):
            ds = [0.0] * 6
            ds[k] = d
            f = PA.link_frames(bend, ds)[name] @ M
            return wall_dist(PA.wheel_center(f)) - SEAT
        a, b = err(LO), err(HI)
        if a * b > 0:
            out.append(LO if abs(a) < abs(b) else HI)
        else:
            out.append(brentq(err, LO, HI, xtol=1e-4))
    return out


def relax(bend, psi, W, span=6.0, n=13):
    """관 단면 안에서 로봇을 옆으로 띄워 스트로크 최대치를 줄인다.

    중심선에 정확히 앉히면 한쪽 바퀴만 과도하게 눌린다. 실제로는 6개 스프링이
    균형점을 찾아 로봇이 살짝 편심하므로, 그 자유도를 여기서 흉내낸다.
    """
    d = np.array([math.cos(math.radians(-psi)), 0.0,
                  math.sin(math.radians(-psi))])
    nvec = np.array([-d[2], 0.0, d[0]])
    best = None
    for lam in np.linspace(-span, span, n):
        M = PA.rot(psi, "y") @ PA.trans(*(W + nvec * lam))
        ds = solve_deltas(bend, M)
        worst = max(abs(x) for x in ds)
        if best is None or worst < best[0]:
            best = (worst, lam, M, ds)
    return best[2], best[3], best[1]


def pipe_outline():
    seg = []
    for off in (-BORE, BORE):
        seg.append(np.array([[-LEAD, 0.0], [0.0, off]]) * 0 + np.array(
            [[-LEAD, off], [0.0, off]]))
    t = np.linspace(0.0, math.pi / 2, 120)
    for sgn in (-1, 1):
        rr = R + sgn * BORE
        seg.append(np.column_stack([rr * np.sin(t), R - rr * np.cos(t)]))
    for off in (-BORE, BORE):
        seg.append(np.array([[R + off, R], [R + off, R + TAIL]]))
    return seg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=72)
    ap.add_argument("--mp4", action="store_true")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.font_manager as fm

    have = {f.name for f in fm.fontManager.ttflist}
    for cand in ("Noto Sans CJK KR", "NanumGothic", "Noto Sans CJK JP",
                 "Malgun Gothic"):
        if cand in have:
            plt.rcParams["font.family"] = cand
            break
    plt.rcParams["axes.unicode_minus"] = False
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.collections import PolyCollection, LineCollection

    parts = PA.part_meshes()
    src = {"body_rear": "body_rear", "body_front": "body_front",
           "bellows": "bellows"}

    s_all = np.linspace(-LEAD + 120.0, ARC + TAIL - 120.0, args.frames)
    print(f"프레임 {args.frames}개 계산 중...")
    frames = []
    for n, s in enumerate(s_all):
        bend, psi, W = pose_at(s)
        M, ds, _ = relax(bend, psi, W)
        fr = PA.link_frames(bend, ds)
        tris, cols = [], []
        for name, F in fr.items():
            key = src.get(name, "arm" if name.startswith("arm") else "wheel")
            v = np.asarray(parts[key].vertices)
            w = (np.column_stack([v, np.ones(len(v))]) @ (F @ M))[:, :3]
            f = parts[key].faces
            tris.append(w[f][:, :, [0, 2]])
            c = ("#c94f4f" if name.startswith("wheel")
                 else "#4f7fc9" if name.startswith("arm")
                 else "#8b96a3" if name == "bellows" else "#6b7684")
            cols += [c] * len(f)
        frames.append((s, bend, ds, np.concatenate(tris), cols))
        if (n + 1) % 12 == 0:
            print(f"  {n + 1}/{args.frames}")

    fig = plt.figure(figsize=(13.5, 7.2))
    gs = fig.add_gridspec(2, 2, width_ratios=[2.1, 1.0], hspace=0.32, wspace=0.22)
    ax = fig.add_subplot(gs[:, 0])
    ax2 = fig.add_subplot(gs[0, 1])
    ax3 = fig.add_subplot(gs[1, 1])

    outline = pipe_outline()
    ax.add_collection(LineCollection(outline, colors="#2b3440", linewidths=1.6))
    t = np.linspace(0, math.pi / 2, 120)
    ax.plot(R * np.sin(t), R - R * np.cos(t), ls=":", c="#9aa4b0", lw=1.0)
    ax.plot([-LEAD, 0], [0, 0], ls=":", c="#9aa4b0", lw=1.0)
    ax.plot([R, R], [R, R + TAIL], ls=":", c="#9aa4b0", lw=1.0)
    ax.set_xlim(-LEAD - 20, R + BORE + 40)
    ax.set_ylim(-BORE - 40, R + TAIL + 20)
    ax.set_aspect("equal")
    ax.set_title("SR 곡관 통과 (기하 해석, 물리 시뮬 아님)", fontsize=12)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Z (mm)")
    ax.grid(alpha=0.25)

    robot = PolyCollection([], edgecolors="#243040", linewidths=0.2)
    ax.add_collection(robot)

    ss = [f[0] for f in frames]
    bends = [f[1] for f in frames]
    strokes = [np.array(f[2]) for f in frames]
    psi0 = PA.META["arm_angle_nominal"]
    arm_len = PA.META["arm_len"]

    def to_mm(d):
        return arm_len * (np.sin(np.radians(psi0 + np.array(d)))
                          - math.sin(math.radians(psi0)))

    stroke_mm = [to_mm(d) for d in strokes]

    ax2.plot(ss, bends, c="#4f7fc9", lw=1.8)
    ax2.axhline(55, ls="--", c="#e03131", lw=1.2)
    ax2.axhline(-55, ls="--", c="#e03131", lw=1.2)
    ax2.set_ylim(-70, 70)
    ax2.set_title("관절 각도 (°)   빨강 = 한계 ±55°", fontsize=10)
    ax2.grid(alpha=0.25)
    m2, = ax2.plot([], [], "o", c="#1b3350", ms=7)

    sm = np.array(stroke_mm)
    for j in range(6):
        ax3.plot(ss, sm[:, j], lw=1.0, alpha=0.8)
    ax3.axhline(6, ls="--", c="#e03131", lw=1.2)
    ax3.axhline(-6, ls="--", c="#e03131", lw=1.2)
    ax3.set_ylim(-8, 8)
    ax3.set_title("서스펜션 스트로크 (mm)   빨강 = 한계 ±6mm", fontsize=10)
    ax3.set_xlabel("관절 위치 (mm)")
    ax3.grid(alpha=0.25)
    m3, = ax3.plot([], [], "o", c="#1b3350", ms=6)

    for a in (ax2, ax3):
        a.axvspan(0, ARC, color="#ffd9a0", alpha=0.35, lw=0)

    txt = ax.text(0.02, 0.97, "", transform=ax.transAxes, va="top",
                  fontsize=11,
                  bbox=dict(fc="white", ec="#c8ced6", alpha=0.9))

    def update(i):
        s, bend, ds, tris, cols = frames[i]
        robot.set_verts(tris)
        robot.set_facecolors(cols)
        m2.set_data([s], [bend])
        m3.set_data([s] * 6, sm[i])
        zone = ("입구 직관" if s < 0 else "곡관 통과 중" if s < ARC else "출구 직관")
        txt.set_text(f"{zone}\n관절 {bend:+6.2f}°\n"
                     f"스트로크 {sm[i].min():+5.2f} ~ {sm[i].max():+5.2f} mm")
        return robot, m2, m3, txt

    print("GIF 인코딩 중...")
    out = args.out or str(HERE / ("curve_pass.mp4" if args.mp4 else "curve_pass.gif"))
    anim = FuncAnimation(fig, update, frames=len(frames), interval=90, blit=False)
    if args.mp4:
        anim.save(out, fps=12, dpi=95)
    else:
        anim.save(out, writer=PillowWriter(fps=11), dpi=85)
    plt.close(fig)

    print("=" * 66)
    print(f"저장: {out}")
    print(f"  관절 각도    {min(bends):+.2f}° ~ {max(bends):+.2f}°  (한계 ±55°)")
    print(f"  스트로크     {sm.min():+.2f} ~ {sm.max():+.2f} mm  (한계 ±6mm)")
    print(f"  판정         "
          f"{'통과' if max(abs(np.array(bends))) < 55 and abs(sm).max() < 6 else '한계 초과'}")
    print("=" * 66)


if __name__ == "__main__":
    main()
