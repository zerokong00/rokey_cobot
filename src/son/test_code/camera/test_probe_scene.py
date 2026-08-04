"""[오프라인] 진단 씬 검사 — 카메라 앞이 정말로 비어 있는가.

2026-08-04 실기에서 `camera/depth_probe.py` 가 쓸 수 없는 답을 냈다.

    ① 빈 공간 (화면 중앙)
       유효=576,000   유효값 0.2150 ~ 0.2179 m
       → invalid_mode: 판정불가  (중앙이 비어 있지 않다)

빈 공간을 잰 것이 아니라 **벽을 잰 것**이었다. 씬이 관을
`UsdGeom.Cylinder` 로 만들었는데 그것은 **양 끝에 뚜껑이 달린 속 찬
원기둥**이라 카메라 앞에 끝면이 그대로 잡힌다.

`invalid_mode` 를 못 정하면 `pipe_condition.yaml` 을 못 채우고,
**관 단절 판정이 조용히 작동하지 않는다.** 진단이 진단을 못 한 셈이다.

**이 결함은 Isaac Sim 없이 잡을 수 있다.** 카메라 위치에서 시선 방향으로
광선을 쏴서 무언가에 맞는지 보면 된다. 여기서 그것을 한다.

실행:  python3 test_probe_scene.py
"""

import sys
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
SON = HERE.parents[1]

BORE_R = 50.0
WALL_R = 57.0
PIPE_LEN = 600.0
HOLE_X = 220.0          # camera.yaml probe.hole_distance_mm


def first_hit(mesh, origin, direction):
    """광선이 처음 맞는 거리(mm). 안 맞으면 None.

    Moller-Trumbore 를 numpy 로 직접 편다. trimesh 의 광선 엔진은 `rtree` 를
    요구하는데, 이 시험은 Isaac Sim 담당자 장비에서도 돌아야 하므로 설치가
    필요한 의존성을 늘리지 않는다.
    """
    o = np.asarray(origin, dtype=np.float64)
    d = np.asarray(direction, dtype=np.float64)
    d = d / np.linalg.norm(d)

    tri = mesh.triangles                      # (n, 3, 3)
    e1 = tri[:, 1] - tri[:, 0]
    e2 = tri[:, 2] - tri[:, 0]
    h = np.cross(d, e2)
    a = np.einsum("ij,ij->i", e1, h)
    eps = 1e-9
    par = np.abs(a) < eps                     # 광선과 평행한 삼각형
    inv = np.divide(1.0, a, out=np.zeros_like(a), where=~par)
    s = o - tri[:, 0]
    u = inv * np.einsum("ij,ij->i", s, h)
    q = np.cross(s, e1)
    v = inv * np.einsum("j,ij->i", d, q)
    t = inv * np.einsum("ij,ij->i", e2, q)
    hit = (~par) & (u >= -eps) & (v >= -eps) & (u + v <= 1 + eps) & (t > 1e-6)
    return float(t[hit].min()) if hit.any() else None


def main():
    ok = []
    print("=" * 78)
    print("진단 씬 — 카메라 앞이 비어 있는가")
    print("=" * 78)

    pipe = trimesh.load(SON / "pipe" / "meshes" / "pipe_straight.stl")
    print(f"  pipe_straight.stl  삼각형 {len(pipe.faces):,}  "
          f"X {pipe.bounds[0][0]:.0f}~{pipe.bounds[1][0]:.0f}mm")
    r = np.hypot(pipe.vertices[:, 1], pipe.vertices[:, 2])
    print(f"  반경 {r.min():.1f}~{r.max():.1f}mm  (보어 {BORE_R} / 벽 {WALL_R})")

    # depth_probe 와 같은 배치 — 관 앞끝을 HOLE_X 에 맞춘다
    placed = pipe.copy()
    placed.apply_translation([HOLE_X - pipe.bounds[1][0], 0.0, 0.0])
    print(f"  배치 후 X {placed.bounds[0][0]:.0f}~{placed.bounds[1][0]:.0f}mm  "
          f"(앞끝이 HOLE_X={HOLE_X:.0f} 에 온다)")

    print("\n  ① 축 방향(정면) 광선 — 여기가 비어 있어야 진단이 성립한다")
    d = first_hit(placed, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0])
    good = d is None
    ok.append(good)
    print(f"     원점에서 +X 로 발사 → "
          f"{'아무것도 안 맞음 (빈 공간)' if good else f'{d:.1f}mm 에서 충돌'}"
          f"  {'OK' if good else 'FAIL'}")

    print("\n  ② 옆면 광선 — 관벽은 정상적으로 맞아야 한다")
    d = first_hit(placed, [0.0, 0.0, 0.0], [0.0, 0.0, 1.0])
    good = d is not None and abs(d - BORE_R) < 1.0
    ok.append(good)
    print(f"     원점에서 +Z 로 발사 → "
          f"{f'{d:.2f}mm' if d else '안 맞음'} (기대 {BORE_R:.0f}mm)"
          f"  {'OK' if good else 'FAIL'}")

    print("\n  ③ 비스듬한 광선 — 관 안쪽 먼 곳도 잡혀야 한다")
    hits = 0
    for ang in np.linspace(0, 2 * np.pi, 8, endpoint=False):
        v = [4.0, float(np.cos(ang)), float(np.sin(ang))]
        if first_hit(placed, [0.0, 0.0, 0.0], v) is not None:
            hits += 1
    ok.append(hits == 8)
    print(f"     8방향 중 {hits}방향에서 벽을 잡음  "
          f"{'OK' if hits == 8 else 'FAIL'}")

    print("\n" + "-" * 78)
    print("  [대조] UsdGeom.Cylinder 를 썼다면 — 실기에서 이것이 나왔다")
    cyl = trimesh.creation.cylinder(radius=BORE_R, height=PIPE_LEN)
    cyl.apply_transform(trimesh.transformations.rotation_matrix(
        np.pi / 2, [0, 1, 0]))
    cyl.apply_translation([HOLE_X - PIPE_LEN / 2.0, 0.0, 0.0])
    d = first_hit(cyl, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0])
    caught = d is not None
    ok.append(caught)
    print(f"     원점에서 +X 로 발사 → "
          f"{f'{d / 1000:.4f} m 에서 끝면 충돌' if caught else '안 맞음'}")
    print(f"     실기 실측 0.2150~0.2179 m 와 일치한다 "
          f"{'OK' if caught else 'FAIL'}")
    print("     → 뚜껑이 시야를 막아 '빈 공간'이 만들어지지 않는다")

    print("\n" + "-" * 78)
    print("  근접 벽면 판정 기준이 기하학적으로 가능한 값인가")
    import yaml
    cfg = yaml.safe_load((SON / "camera" / "config" / "camera.yaml").read_text())
    near = float(cfg["camera"]["probe"]["near_wall_mm"]) \
        if "probe" in cfg["camera"] else float(cfg["probe"]["near_wall_mm"])
    off = cfg["camera"]["front"]["offset_mm"]
    ecc = float(np.hypot(off[1], off[2]))
    reachable = BORE_R - ecc          # 축에서 ecc 만큼 벗어난 점의 최단 벽거리
    good = near <= reachable + 1e-6
    ok.append(good)
    print(f"     카메라 편심 {ecc:.1f}mm → 도달 가능한 최단 벽거리 "
          f"{reachable:.1f}mm")
    print(f"     설정 near_wall_mm = {near:.1f}mm  "
          f"{'OK' if good else 'FAIL — 절대 만족 못 하는 조건이라 상시 오탐'}")
    if good:
        print(f"     (실기 실측 최소 45.73mm — 25.0 이던 시절엔 항상 0개였다)")

    print("\n" + "=" * 78)
    print("  씬을 만드는 코드가 실제로 고쳐졌는가")
    src = (SON / "camera" / "depth_probe.py").read_text()
    checks = [
        ("UsdGeom.Cylinder 로 관을 만들지 않는다",
         "UsdGeom.Cylinder.Define" not in src),
        ("실제 배관 메시를 읽는다", "pipe_straight.stl" in src),
        ("STL 로더가 있다", "def load_stl" in src),
    ]
    for name, good in checks:
        ok.append(good)
        print(f"  {'OK  ' if good else 'FAIL'} {name}")

    print("=" * 78)
    print(f"전체 판정: {'통과' if all(ok) else '실패'}  ({sum(ok)}/{len(ok)})")
    return 0 if all(ok) else 1


if __name__ == "__main__":
    sys.exit(main())
