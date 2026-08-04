"""[Isaac 3.11] 카메라 Depth 반환값 진단 — 다른 구현보다 우선 실행할 것.

두 가지를 확인한다. 이 확인 없이 판정 노드를 돌리면 관 단절 판정이
**전혀 작동하지 않는다.**

  ① 빈 공간의 distance_to_camera 반환값
     배관에 의도적으로 구멍을 뚫고 그 방향 픽셀 값을 본다.
     0 / inf / NaN / 최대거리 중 무엇인지에 따라 판정식이 달라진다.

  ② 근접 벽면(25mm) 측정 가능 여부
     near clip 을 5mm 로 두고도 관벽이 무효로 나오면 무효 픽셀 비율이
     상시 높아져 DISCONNECTED 오탐이 발생한다.

ROS 를 거치지 않고 annotator 를 직접 읽는다. 브리지 문제와 센서 문제를
섞지 않기 위해서다.

⚠ headless 에서는 카메라 프레임이 발행되지 않는다. 반드시 GUI 로 실행할 것.

실행:
  PYTHONUNBUFFERED=1 isaac_python camera_depth_probe.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pyver import require_isaac

require_isaac(__file__, needs_rclpy=False)

HEADLESS = "--headless" in sys.argv

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": HEADLESS})

import json
import math
import struct
from pathlib import Path

import numpy as np
import yaml
from isaacsim.core.api import World
from isaacsim.sensors.camera import Camera
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics

import omni.replicator.core as rep

HERE = Path(__file__).resolve().parent
SON = HERE.parent


def load_stl(path):
    """바이너리 STL → (정점 m, 삼각형 인덱스). 다른 스크립트와 같은 구현."""
    data = Path(path).read_bytes()
    n = struct.unpack("<I", data[80:84])[0]
    a = np.frombuffer(data[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    tri = a[:, 12:48].copy().view("<f4").reshape(n * 3, 3).astype(np.float64)
    pts, inv = np.unique(np.round(tri, 5), axis=0, return_inverse=True)
    return pts * 0.001, inv.reshape(n, 3)


CFG = yaml.safe_load((HERE / "config" / "camera.yaml").read_text())
CAM = CFG["camera"]
LIGHT = CFG["light"]
PROBE = CFG["probe"]

MM = 0.001
BORE_M = 0.050
HOLE_D = PROBE["hole_diameter_mm"] * MM
HOLE_X = PROBE["hole_distance_mm"] * MM
NEAR_WALL = PROBE["near_wall_mm"] * MM
FRAMES = int(PROBE["frames"])

W, H = int(CAM["width"]), int(CAM["height"])
NEAR, FAR = CAM["clipping_range_m"]
HFOV = float(CAM["hfov_deg"])
F_PX = (W / 2.0) / math.radians(HFOV / 2.0)
PPX, PPY = W / 2.0, H / 2.0
PIXEL_SIZE_UM = 3.0
HORIZ_APERTURE_M = PIXEL_SIZE_UM * W * 1e-6
FOCAL_LENGTH_M = PIXEL_SIZE_UM * F_PX * 1e-6

if HEADLESS:
    print("[경고] headless 에서는 카메라 프레임이 발행되지 않는다. "
          "결과가 전부 0 으로 나올 것이다.")


def build_scene(stage):
    """앞이 뚫린 관 — 구멍 대신 길이를 잘라 개구부를 만든다.

    불리언 컷은 Isaac Sim 에 없는 기능이라 관을 두 토막으로 두고 그 사이를
    비워 '끊긴 관'을 만든다. 진단 목적으로는 동일하다.
    """
    UsdGeom.Xform.Define(stage, "/World")

    # 🚨 UsdGeom.Cylinder 를 쓰면 안 된다. 그것은 **양 끝에 뚜껑이 달린 속 찬
    # 원기둥**이라 관 안에서 보면 화면 중앙에 끝면이 잡힌다. 빈 공간이
    # 안 만들어지므로 진단이 성립하지 않는다 — 2026-08-04 실기에서
    # "중앙이 비어 있지 않다 / 판정불가" 로 나왔던 원인이 이것이다.
    # 실제 배관 메시(양끝이 열린 600mm 직관)를 쓴다.
    def tube(path, front_x):
        """앞끝이 front_x 에 오도록 직관을 놓는다. 그 너머가 빈 공간이다."""
        pts, idx = load_stl(SON / "pipe" / "meshes" / "pipe_straight.stl")
        xf = UsdGeom.Xform.Define(stage, path)
        UsdGeom.Xformable(xf).AddTranslateOp().Set(
            Gf.Vec3d(front_x - float(pts[:, 0].max()), 0.0, 0.0))
        mesh = UsdGeom.Mesh.Define(stage, f"{path}/mesh")
        mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
        mesh.CreateFaceVertexCountsAttr([3] * len(idx))
        mesh.CreateFaceVertexIndicesAttr(idx.reshape(-1).tolist())
        mesh.CreateExtentAttr([Gf.Vec3f(*pts.min(0)), Gf.Vec3f(*pts.max(0))])
        mesh.CreateSubdivisionSchemeAttr("none")
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(
            mesh.GetPrim()).CreateApproximationAttr("none")
        return xf

    # 카메라 앞 HOLE_X 까지만 관이 있고 그 너머는 빈 공간
    tube("/World/Pipe/near", HOLE_X)

    light = UsdLux.SphereLight.Define(stage, "/World/ProbeLight")
    light.CreateIntensityAttr(float(LIGHT["intensity"]))
    light.CreateRadiusAttr(LIGHT["radius_mm"] * MM)
    UsdGeom.Xformable(light).AddTranslateOp().Set(Gf.Vec3d(0.01, 0.0, 0.0))


def classify(depth, max_range):
    n = depth.size
    zero = int((depth == 0).sum())
    nan = int(np.isnan(depth).sum())
    inf = int(np.isinf(depth).sum())
    finite = np.isfinite(depth) & (depth > 0)
    v = depth[finite]
    at_max = int((v > max_range * 0.99).sum()) if v.size else 0
    return dict(n=n, zero=zero, nan=nan, inf=inf, at_max=at_max,
                valid=int(v.size),
                vmin=float(v.min()) if v.size else float("nan"),
                vmax=float(v.max()) if v.size else float("nan"))


def main():
    world = World(stage_units_in_meters=1.0)
    stage = world.stage
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    build_scene(stage)

    cam = Camera(prim_path="/World/ProbeCamera",
                 position=np.array([0.0, 0.0, 0.0]),
                 frequency=int(CAM["publish_hz"]),
                 resolution=(W, H))
    cam.initialize()
    # Isaac Camera 클래스의 시선은 프림 로컬 +X 다 (USD 관례인 -Z 가 아님).
    # near 를 관벽 25mm 보다 작게 두지 않으면 근접 벽면이 통째로 잘린다.
    cam.set_clipping_range(float(NEAR), float(FAR))
    # 어안. fisheyePolynomial 은 Isaac Sim 5.0 부터 폐기됐다(OmniLensDistortion
    # 스키마가 대체). set_horizontal_fov 라는 세터는 없고 초점거리와 센서폭으로
    # 화각이 정해진다. 왜곡 계수 0 이면 순수 등거리 r = f*theta.
    cam.set_focal_length(FOCAL_LENGTH_M)
    cam.set_horizontal_aperture(HORIZ_APERTURE_M)
    try:
        cam.set_opencv_fisheye_properties(cx=PPX, cy=PPY, fx=F_PX, fy=F_PX,
                                          fisheye=[0.0, 0.0, 0.0, 0.0])
    except Exception as exc:
        print(f"[경고] set_opencv_fisheye_properties 실패 ({exc}) — 핀홀로 진행")
    cam.add_distance_to_camera_to_frame()

    world.reset()
    for _ in range(30):
        world.step(render=True)

    acc = []
    for _ in range(FRAMES):
        world.step(render=True)
        d = cam.get_current_frame().get("distance_to_camera")
        if d is not None:
            acc.append(np.asarray(d, dtype=np.float64))

    print("\n" + "=" * 74)
    print("카메라 Depth 진단")
    print("=" * 74)
    if not acc:
        print("[실패] distance_to_camera 프레임을 하나도 못 받았다.")
        print("  - GUI 로 띄웠는가 (headless 는 프레임 0)")
        print("  - add_distance_to_camera_to_frame() 이후 충분히 step 했는가")
        simulation_app.close()
        return 1

    a = np.stack(acc)
    print(f"수집 {len(acc)} 프레임  해상도 {a.shape[2]}x{a.shape[1]}  "
          f"clipping {NEAR}~{FAR} m  어안 f={F_PX:.1f}px HFOV={HFOV:.0f}°")

    # ── ① 빈 공간 ────────────────────────────────────────────────
    # 화면 중앙 영역 = 관이 끊긴 방향
    h, w = a.shape[1], a.shape[2]
    cy, cx = h // 2, w // 2
    r = min(h, w) // 12
    centre = a[:, cy - r:cy + r, cx - r:cx + r]
    s = classify(centre, FAR)

    print("-" * 74)
    print(f"① 빈 공간 (화면 중앙 {2 * r}x{2 * r}, 관이 끊긴 방향)")
    print(f"   픽셀 {s['n']:,}  0={s['zero']:,}  NaN={s['nan']:,}  "
          f"inf={s['inf']:,}  최대값부근={s['at_max']:,}  유효={s['valid']:,}")
    if s["valid"]:
        print(f"   유효값 {s['vmin']:.4f} ~ {s['vmax']:.4f} m")

    frac = lambda k: s[k] / s["n"]
    if frac("zero") > 0.5:
        mode, expr = "empty_is_zero", "invalid = (depth == 0)"
    elif frac("nan") + frac("inf") > 0.5:
        mode, expr = "empty_is_zero", "invalid = ~np.isfinite(depth)"
    elif frac("at_max") > 0.5:
        mode, expr = "empty_is_far", f"invalid = (depth > {FAR} * 0.99)"
    else:
        mode, expr = "판정불가", "중앙이 비어 있지 않다 — 씬 배치 확인"
    print(f"   → invalid_mode: {mode}")
    print(f"     판정식: {expr}")

    # ── ② 근접 벽면 ──────────────────────────────────────────────
    v = a[np.isfinite(a) & (a > 0)]
    near_cnt = int((v < NEAR_WALL * 1.4).sum())
    print("-" * 74)
    print(f"② 근접 벽면 ({PROBE['near_wall_mm']:.0f}mm)")
    if v.size:
        print(f"   전체 유효 {v.size:,}  최소 {v.min() * 1000:.2f} mm  "
              f"{NEAR_WALL * 1400:.0f}mm 미만 {near_cnt:,}")
    if near_cnt == 0:
        print("   [경고] 근접 벽면 Depth 가 없다. 무효 픽셀 비율이 상시 높아져")
        print("          DISCONNECTED 오탐이 발생한다. near clip 을 더 줄이거나")
        print("          결함 치수 실측을 RGB 기반으로 전환할지 검토할 것.")
    else:
        print("   [OK] 근접 벽면이 유효하게 측정된다")

    out = HERE / "camera_probe_result.json"
    out.write_text(json.dumps(
        dict(invalid_mode=mode, expression=expr, clipping=[NEAR, FAR],
             resolution=[W, H], f_px=F_PX, hfov_deg=HFOV,
             stats=s, near_wall_px=near_cnt), indent=2))
    print("-" * 74)
    print(f"저장: {out}")
    print(f"→ 이 값을 ../condition/config/pipe_condition.yaml 의")
    print(f"  invalid_mode 에 넣을 것: {mode}")
    print("=" * 74)

    if not HEADLESS:
        print("\nGUI 실행 중 — 창을 닫으면 종료됩니다.")
        while simulation_app.is_running():
            world.step(render=True)
    simulation_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
