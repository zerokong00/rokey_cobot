"""[오프라인] 수리기 시각화 — 링 토치·카메라·결함/비드를 눈으로 확인한다.

숫자만으로는 무엇이 어떻게 된 건지 판단하기 어렵다. 지금까지 검사로만
확인해 온 것들을 그림으로 낸다.

  welder_layout.png   측면·정면 배치, 링 회전 도달 범위, 토치 수납/신장
  welder_camera.png   카메라 시야와 사각지대, 결함/비드 단면
  welder_ring.gif     링이 ±180° 도는 애니메이션

실행:  python3 preview_welder.py
"""

import json
import math
import sys
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
SON = HERE.parents[1]
sys.path.insert(0, str(SON / "test_code" / "robot"))

import preview_assembly as PA

META = json.loads((SON / "spec" / "parts_meta.json").read_text())
T = META["torch"]
BORE = META["pipe_id"] / 2.0
HFOV = 140.0

RING_X = T["ring_x"]
RING_R = T["rod_origin_r"]
ROD_LEN = T["rod_len"]
STROKE = T["j2_stroke_mm"]

C_BODY = "#6b7684"
C_ARM = "#4f7fc9"
C_WHEEL = "#c94f4f"
C_RING = "#e08a2b"
C_ROD = "#b05ecb"
C_CAM = "#2fa36b"
C_PIPE = "#2b3440"


def load(cat, name):
    return trimesh.load(str(SON / cat / "meshes" / f"{name}.stl"))


def torch_frames(j1_deg, ext_mm):
    """링 / 로드 / 팁의 배치 행렬."""
    base = PA.rot(j1_deg, "x") @ PA.trans(RING_X, 0, 0)
    return {
        "torch_ring": base,
        "torch_rod": PA.trans(0, 0, RING_R + ext_mm) @ base,
        "torch_tip": PA.trans(0, 0, RING_R + ROD_LEN + ext_mm) @ base,
    }


def welder_parts(bend=0.0, j1=0.0, ext=0.0, deltas=None):
    """정찰기 14링크 + 토치 3링크 + 카메라 하우징 2개."""
    out = {}
    for name, m in PA.build(bend, deltas).items():
        out[name] = (m, C_WHEEL if name.startswith("wheel")
                     else C_ARM if name.startswith("arm") else C_BODY)
    tm = {"torch_ring": load("welder", "torch_ring"),
          "torch_rod": load("welder", "torch_rod"),
          "torch_tip": load("welder", "torch_tip")}
    for name, M in torch_frames(j1, ext).items():
        out[name] = (PA.apply(tm[name], M),
                     C_RING if name == "torch_ring" else C_ROD)
    cam = load("camera", "camera_housing")
    import yaml
    cfg = yaml.safe_load((SON / "camera" / "config" / "camera.yaml").read_text())
    for tag, spec in (("front", cfg["camera"]["front"]),
                      ("rear", cfg["camera"]["rear"])):
        x = spec["mount_x_mm"]
        M = PA.trans(x, 0, 0)
        m = cam.copy()
        if tag == "rear":
            v = np.asarray(m.vertices) * np.array([-1.0, 1.0, 1.0])
            m.vertices = v
        out[f"cam_{tag}"] = (PA.apply(m, M), C_CAM)
    return out


def draw(ax, parts, plane, alpha=0.92, lw=0.15):
    from matplotlib.collections import PolyCollection
    a, b = plane
    for name, (m, col) in parts.items():
        tri = m.triangles[:, :, [a, b]]
        ax.add_collection(PolyCollection(tri, facecolors=col,
                                         edgecolors="#243040",
                                         linewidths=lw, alpha=alpha))


def pipe_lines(ax, plane):
    a, b = plane
    if a == 1:
        th = np.linspace(0, 2 * np.pi, 240)
        ax.plot(BORE * np.cos(th), BORE * np.sin(th), "--", c=C_PIPE, lw=1.4)
        ax.plot(57 * np.cos(th), 57 * np.sin(th), "-", c=C_PIPE, lw=1.0,
                alpha=0.4)
    else:
        for s in (-1, 1):
            ax.axhline(s * BORE, ls="--", c=C_PIPE, lw=1.4)
            ax.axhline(s * 57, ls="-", c=C_PIPE, lw=1.0, alpha=0.4)


def fig_layout(path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.font_manager as fm
    have = {f.name for f in fm.fontManager.ttflist}
    for c in ("Noto Sans CJK KR", "NanumGothic", "Noto Sans CJK JP"):
        if c in have:
            plt.rcParams["font.family"] = c
            break
    plt.rcParams["axes.unicode_minus"] = False

    fig = plt.figure(figsize=(16, 11))
    gs = fig.add_gridspec(3, 2, height_ratios=[1.15, 1.0, 1.0],
                          hspace=0.28, wspace=0.18)

    # A 측면
    ax = fig.add_subplot(gs[0, :])
    draw(ax, welder_parts(ext=0.0), (0, 2))
    pipe_lines(ax, (0, 2))
    ax.set_xlim(-115, 105)
    ax.set_ylim(-62, 62)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    ax.set_title("측면 (X-Z) — 수리기 17링크, 토치 수납 상태", fontsize=13)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Z (mm)")
    ax.annotate(f"토치 링\nx {RING_X - T['ring_w'] / 2:.0f}~"
                f"{RING_X + T['ring_w'] / 2:.0f}",
                xy=(RING_X, 32), xytext=(RING_X - 4, 52),
                fontsize=9, ha="center", color=C_RING,
                arrowprops=dict(arrowstyle="->", color=C_RING))
    ax.annotate("전방 카메라", xy=(74, 8), xytext=(96, 42), fontsize=9,
                ha="center", color=C_CAM,
                arrowprops=dict(arrowstyle="->", color=C_CAM))
    ax.annotate("후방 카메라", xy=(-74, 8), xytext=(-96, 42), fontsize=9,
                ha="center", color=C_CAM,
                arrowprops=dict(arrowstyle="->", color=C_CAM))

    # B 정면 (토치 수납)
    ax = fig.add_subplot(gs[1, 0])
    draw(ax, welder_parts(ext=0.0), (1, 2))
    pipe_lines(ax, (1, 2))
    ax.set_xlim(-60, 60)
    ax.set_ylim(-60, 60)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    ax.set_title(f"정면 — 토치 수납 (반경 {T['stow_radius_mm']:.0f}mm)",
                 fontsize=12)

    # C 링 회전 도달 범위
    ax = fig.add_subplot(gs[1, 1])
    base = welder_parts(ext=0.0)
    draw(ax, {k: v for k, v in base.items()
              if not k.startswith("torch")}, (1, 2), alpha=0.28, lw=0.05)
    tm = {n: load("welder", n) for n in
          ("torch_ring", "torch_rod", "torch_tip")}
    cmap = __import__("matplotlib.cm", fromlist=["cm"]).get_cmap("plasma")
    for k, j1 in enumerate(range(-180, 181, 45)):
        col = cmap(0.15 + 0.7 * (j1 + 180) / 360)
        p = {n: (PA.apply(tm[n], M), col)
             for n, M in torch_frames(j1, STROKE).items() if n != "torch_ring"}
        draw(ax, p, (1, 2), alpha=0.85, lw=0.1)
    pipe_lines(ax, (1, 2))
    ax.set_xlim(-60, 60)
    ax.set_ylim(-60, 60)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    ax.set_title("J1 ±180° 회전 — 전 원주 도달 (45° 간격)", fontsize=12)

    # D 수납 vs 신장
    ax = fig.add_subplot(gs[2, 0])
    for ext, col, lab in ((0.0, "#8899aa", f"수납 {T['stow_radius_mm']:.0f}mm"),
                          (STROKE, C_ROD,
                           f"신장 {T['reach_radius_mm']:.0f}mm")):
        p = {n: (PA.apply(tm[n], M), col)
             for n, M in torch_frames(0.0, ext).items()}
        draw(ax, p, (0, 2), alpha=0.8)
        ax.plot([], [], "s", color=col, label=lab)
    ax.axhline(BORE, ls="--", c=C_PIPE, lw=1.4)
    ax.axhline(41.69, ls=":", c="#c94f4f", lw=1.4)
    ax.text(RING_X + 14, 41.69, "곡관 통과 한계 41.7", fontsize=8,
            color="#c94f4f", va="bottom")
    ax.text(RING_X + 14, BORE, "관벽 50", fontsize=8, color=C_PIPE,
            va="bottom")
    ax.set_xlim(RING_X - 18, RING_X + 18)
    ax.set_ylim(20, 58)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=9, loc="lower left")
    ax.set_title("J2 직동 — 수납 / 신장", fontsize=12)
    ax.set_xlabel("X (mm)")

    # E 곡관 자세
    ax = fig.add_subplot(gs[2, 1])
    dmax, bend, placed = PA.elbow_fit(verbose=False)
    p = {k: (v, C_WHEEL if k.startswith("wheel")
             else C_ARM if k.startswith("arm") else C_BODY)
         for k, v in placed.items()}
    draw(ax, p, (0, 2), alpha=0.9)
    t = np.linspace(0, math.pi / 2, 120)
    R = META["elbow_r"]
    for sgn in (-1, 1):
        rr = R + sgn * BORE
        ax.plot(rr * np.sin(t), R - rr * np.cos(t), "--", c=C_PIPE, lw=1.3)
        ax.plot([-220, 0], [sgn * BORE, sgn * BORE], "--", c=C_PIPE, lw=1.3)
        ax.plot([R + sgn * BORE] * 2, [R, R + 120], "--", c=C_PIPE, lw=1.3)
    ax.plot([-220, 0], [0, 0], ":", c="#9aa4b0", lw=1.0)
    ax.plot(R * np.sin(t), R - R * np.cos(t), ":", c="#9aa4b0", lw=1.0)
    ax.set_xlim(-215, 175)
    ax.set_ylim(-70, 200)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    ax.set_title(f"SR 곡관 자세 — 관절 {abs(bend):.1f}°, "
                 f"본체 여유 {BORE - dmax:.1f}mm", fontsize=12)
    ax.set_xlabel("X (mm)")

    fig.suptitle("수리기 — 링 토치 배치", fontsize=16, y=0.985)
    fig.savefig(path, dpi=95, bbox_inches="tight")
    plt.close(fig)
    print(f"  저장: {path}")


def fig_camera(path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.font_manager as fm
    have = {f.name for f in fm.fontManager.ttflist}
    for c in ("Noto Sans CJK KR", "NanumGothic", "Noto Sans CJK JP"):
        if c in have:
            plt.rcParams["font.family"] = c
            break
    plt.rcParams["axes.unicode_minus"] = False

    fig, axes = plt.subplots(1, 2, figsize=(15, 6.2))

    # 좌: 카메라 시야와 사각지대
    ax = axes[0]
    import yaml
    cfg = yaml.safe_load((SON / "camera" / "config" / "camera.yaml").read_text())
    cx = cfg["camera"]["front"]["mount_x_mm"]
    half = math.radians(HFOV / 2)
    zmin = BORE / math.tan(half)

    for s in (-1, 1):
        ax.axhline(s * BORE, ls="--", c=C_PIPE, lw=1.4)
    L = 300
    ax.fill([cx, cx + L, cx + L], [0, L * math.tan(half), -L * math.tan(half)],
            color="#8fd0a8", alpha=0.35, label=f"시야 {HFOV:.0f}°")
    ax.fill_between([cx, cx + zmin], -BORE, BORE, color="#e8a0a0", alpha=0.45,
                    label=f"사각지대 (전방 {zmin:.1f}mm)")
    ax.plot(cx, 0, "o", color=C_CAM, ms=10)
    ax.annotate("전방 카메라", xy=(cx, 0), xytext=(cx - 30, 30), fontsize=10,
                color=C_CAM, arrowprops=dict(arrowstyle="->", color=C_CAM))
    ax.plot(RING_X, BORE, "*", color=C_ROD, ms=18)
    ax.annotate("용접 지점\n(제자리에서 안 보임)", xy=(RING_X, BORE),
                xytext=(RING_X - 60, 78), fontsize=10, color=C_ROD, ha="center",
                arrowprops=dict(arrowstyle="->", color=C_ROD))
    for b, lab in ((0, "제자리"), (120, "후진 120mm")):
        wx = RING_X + b
        th = math.degrees(math.atan2(BORE, wx - cx)) if wx > cx else 180
        ax.plot([cx, wx], [0, BORE], ":", lw=1.2,
                c="#c94f4f" if th > HFOV / 2 else "#2fa36b")
        ax.text(wx, BORE + 5, f"{lab}\n{th:.0f}°", fontsize=9, ha="center",
                color="#c94f4f" if th > HFOV / 2 else "#2fa36b")
    ax.set_xlim(cx - 40, cx + 230)
    ax.set_ylim(-95, 95)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=9, loc="lower right")
    ax.set_title("카메라 시야 — 옆의 관벽은 원리적으로 못 본다", fontsize=13)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("반경 (mm)")

    # 우: 결함 / 비드 단면
    ax = axes[1]
    d = META["defect"]
    arc = np.linspace(-8, 8, 400)
    hw = d["crack_width_mm"] / 2
    groove = BORE + d["crack_depth_mm"] * np.clip(1 - np.abs(arc) / hw, 0, 1)
    w = hw + d["bead_margin_mm"]
    bead = BORE - d["bead_proud_mm"] * np.clip(1 - (np.abs(arc) / w) ** 2, 0, 1)
    ax.axhline(BORE, c=C_PIPE, lw=1.6, label="관벽 50.0")
    ax.plot(arc, groove, c="#c94f4f", lw=2.4,
            label=f"결함 (파임 {d['crack_depth_mm']:.1f}mm)")
    ax.plot(arc, bead, c="#2fa36b", lw=2.4,
            label=f"비드 (돌출 {d['bead_proud_mm']:.1f}mm)")
    ax.fill_between(arc, BORE, groove, color="#c94f4f", alpha=0.2)
    ax.fill_between(arc, BORE, bead, color="#2fa36b", alpha=0.2)
    ax.axhline(BORE + 0.6, ls=":", c="#888", lw=1.2)
    ax.text(7.6, BORE + 0.63, "미수리 판정 임계 +0.6", fontsize=8, ha="right")
    ax.axhline(BORE + 0.25, ls=":", c="#888", lw=1.2)
    ax.text(7.6, BORE + 0.28, "수리 판정 임계 +0.25", fontsize=8, ha="right")
    ax.invert_yaxis()
    ax.set_xlabel("원주 방향 거리 (mm)")
    ax.set_ylabel("관 축에서의 반경 (mm)   아래가 관 안쪽")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=9, loc="lower right")
    ax.set_title("결함 / 비드 단면 — 가시성 전환으로 형상이 바뀐다", fontsize=13)

    fig.tight_layout()
    fig.savefig(path, dpi=100, bbox_inches="tight")
    plt.close(fig)
    print(f"  저장: {path}")


def gif_ring(path, frames=48):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.font_manager as fm
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.collections import PolyCollection
    have = {f.name for f in fm.fontManager.ttflist}
    for c in ("Noto Sans CJK KR", "NanumGothic", "Noto Sans CJK JP"):
        if c in have:
            plt.rcParams["font.family"] = c
            break
    plt.rcParams["axes.unicode_minus"] = False

    tm = {n: load("welder", n) for n in
          ("torch_ring", "torch_rod", "torch_tip")}
    base = welder_parts(ext=0.0)
    static = {k: v for k, v in base.items() if not k.startswith("torch")}

    j1s = np.concatenate([np.linspace(0, 180, frames // 2),
                          np.linspace(180, -180, frames // 2)])
    seq = []
    for j1 in j1s:
        ext = STROKE if abs(abs(j1) % 90) < 12 else 0.0
        tris, cols = [], []
        for n, M in torch_frames(j1, ext).items():
            m = PA.apply(tm[n], M)
            t = m.triangles[:, :, [1, 2]]
            tris.append(t)
            c = C_RING if n == "torch_ring" else C_ROD
            cols += [c] * len(t)
        seq.append((j1, ext, np.concatenate(tris), cols))

    fig, ax = plt.subplots(figsize=(6.6, 6.8))
    for name, (m, col) in static.items():
        ax.add_collection(PolyCollection(m.triangles[:, :, [1, 2]],
                                         facecolors=col, edgecolors="#243040",
                                         linewidths=0.05, alpha=0.30))
    pipe_lines(ax, (1, 2))
    torch = PolyCollection([], edgecolors="#243040", linewidths=0.15)
    ax.add_collection(torch)
    ax.set_xlim(-60, 60)
    ax.set_ylim(-60, 60)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    ax.set_xlabel("Y (mm)")
    ax.set_ylabel("Z (mm)")
    txt = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", fontsize=11,
                  bbox=dict(fc="white", ec="#c8ced6", alpha=0.9))

    def upd(i):
        j1, ext, tris, cols = seq[i]
        torch.set_verts(tris)
        torch.set_facecolors(cols)
        txt.set_text(f"J1 {j1:+7.1f}°\nJ2 {ext:4.1f} mm"
                     + ("   ← 용접" if ext > 0 else ""))
        return torch, txt

    ax.set_title("토치 링 회전 — 바퀴로 못 도는 대신 링이 돈다", fontsize=12)
    anim = FuncAnimation(fig, upd, frames=len(seq), interval=90, blit=False)
    anim.save(path, writer=PillowWriter(fps=11), dpi=85)
    plt.close(fig)
    print(f"  저장: {path}")


def fig_overview(path):
    """전체 요약 — 무엇을 만들었고 무엇이 검증됐는가."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.font_manager as fm
    from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
    have = {f.name for f in fm.fontManager.ttflist}
    for c in ("Noto Sans CJK KR", "NanumGothic", "Noto Sans CJK JP"):
        if c in have:
            plt.rcParams["font.family"] = c
            break
    plt.rcParams["axes.unicode_minus"] = False

    fig, ax = plt.subplots(figsize=(15, 9))
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 62)
    ax.axis("off")

    def box(x, y, w, h, title, lines, col, done):
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.5",
                                    fc=col, ec="#2b3440", lw=1.4, alpha=0.9))
        ax.text(x + w / 2, y + h - 1.6, title, ha="center", va="top",
                fontsize=11, fontweight="bold")
        for i, t in enumerate(lines):
            ax.text(x + 1.2, y + h - 4.2 - i * 2.1, t, fontsize=8.6, va="top")
        mark = "검증됨" if done == 1 else ("미실행" if done == 0 else "일부")
        mc = "#2fa36b" if done == 1 else ("#c94f4f" if done == 0 else "#d99a2b")
        ax.text(x + w - 1.2, y + 1.0, mark, ha="right", fontsize=8.6,
                color=mc, fontweight="bold")

    def arrow(x1, y1, x2, y2, label="", col="#4f7fc9"):
        ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), color=col,
                                     arrowstyle="-|>", mutation_scale=15,
                                     lw=1.6, shrinkA=2, shrinkB=2))
        if label:
            ax.text((x1 + x2) / 2, (y1 + y2) / 2 + 0.8, label, fontsize=8,
                    ha="center", color=col)

    ax.text(50, 60, "배관 점검·수리 로봇 — 구현 현황", ha="center",
            fontsize=17, fontweight="bold")
    ax.text(50, 57.4, "초록 = 오프라인 검증 완료   빨강 = Isaac Sim 필요, 미실행",
            ha="center", fontsize=9.5, color="#555")

    box(2, 40, 26, 14, "Isaac Sim (PC1)",
        ["robot/articulate.py  14링크",
         "welder/articulate.py 17링크",
         "camera/rig.py  어안 2대",
         "pipe/  직관 + SR곡관 + 결함"], "#f3c9c9", 0)
    box(37, 40, 26, 14, "자산 생성 (오프라인)",
        ["tools/build_parts.py",
         "로봇 5 + 카메라 1 + 배관 6",
         "토치 링/로드/팁 3",
         "spec/parts_meta.json 단일출처"], "#c9e6d3", 1)
    box(72, 40, 26, 14, "형상 검증",
        ["곡관 관절 44.9° (한계 55)",
         "스트로크 ±4.02mm (한계 6)",
         "링·암 여유 4.0mm",
         "토치 수납 40 / 도달 48mm"], "#c9e6d3", 1)

    box(2, 22, 26, 14, "condition/  인지",
        ["Depth 무효율 → 단절",
         "윤곽 거칠기 → 판정불가",
         "원 피팅 → 어긋남 mm",
         "시각 오도메트리 ±0.25mm"], "#c9e6d3", 1)
    box(37, 22, 26, 14, "driver/  주행",
        ["속도 법칙 (조향 없음)",
         "FSM 7상태",
         "끼임 = 휠 vs 시각 비교",
         "cmd_vel linear.x"], "#c9e6d3", 1)
    box(72, 22, 26, 14, "welder/  용접",
        ["링 J1 ±180 / J2 8mm",
         "3겹 검증",
         "복귀 감사 (후방 카메라)",
         "결함·비드 가시성 전환"], "#c9e6d3", 1)

    box(20, 3, 60, 13, "오프라인 검증 총계",
        ["관 상태 판정  11장면 전수 · 오프셋 오차 0.14mm",
         "주행 FSM      19항목 전수",
         "용접 3겹      10항목 · 복귀 감사 12항목 · 링 간섭 12항목",
         "ROS 왕복      판정 → 주행 → cmd_vel 실제 확인",
         "미실행        Isaac Sim 스크립트 4종 (물리·카메라·주행·용접 조립)"],
        "#dfe6ee", 2)

    arrow(15, 40, 15, 36, "센서")
    arrow(15, 22, 37, 29, "condition")
    arrow(63, 29, 72, 29, "목표")
    arrow(50, 40, 50, 36, "")
    arrow(85, 40, 85, 36, "")
    arrow(28, 47, 37, 47, "메시", "#8a8f98")
    arrow(63, 47, 72, 47, "치수", "#8a8f98")

    fig.savefig(path, dpi=100, bbox_inches="tight",
                facecolor="white")
    plt.close(fig)
    print(f"  저장: {path}")


def main():
    print("=" * 66)
    print("수리기 시각화")
    print("=" * 66)
    fig_layout(str(HERE / "welder_layout.png"))
    fig_camera(str(HERE / "welder_camera.png"))
    gif_ring(str(HERE / "welder_ring.gif"))
    fig_overview(str(HERE / "overview.png"))
    print("=" * 66)
    return 0


if __name__ == "__main__":
    sys.exit(main())
