"""[Isaac 3.11] pipe/meshes 결함 패치(defect_hole/crack)를 간격을 두고
나란히 배치해 형태를 눈으로 확인하는 뷰어 + 결함별 독립 USD 저장 스크립트.

    DISPLAY=:1 isaac_python pipe/inspect_defect_shapes.py            # GUI로 보기(뷰포트 유지, 둘을 나란히 비교)
    isaac_python pipe/inspect_defect_shapes.py --headless            # 저장만 하고 종료

저장 위치: training/<이름>_inspect.usd — 결함마다 완전히 별도의 스테이지
파일(defect_hole_inspect.usd, defect_crack_inspect.usd)로 각각 1개씩만
담아 저장한다(--outdir <경로> 로 저장 폴더 변경 가능). GUI 미리보기용
라이브 스테이지는 비교하기 좋게 둘을 한 화면에 띄워 두지만, 파일로
저장되는 결과물은 서로 완전히 독립적인 prim이다.
STL들이 원본 바운딩박스가 동일(약 30x33x9mm, 곡관 패치 형태)해서 공통 중심을
원점으로 옮긴 뒤 X축으로 SPACING 만큼 띄워 배치한다 — 곡률은 그대로
유지되므로 형태 차이를 있는 그대로 비교할 수 있다. 색상은 흰색(1,1,1)으로
통일해 형태(음영)에만 집중해서 볼 수 있게 했다.
"""
import sys
from pathlib import Path

HEADLESS = "--headless" in sys.argv
HOLD = "--hold" in sys.argv or not HEADLESS
SPACING = 0.09   # 90mm 간격 (패치 폭 30mm + 여유, GUI 비교용 라이브 씬에서만 사용)

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": HEADLESS})

import numpy as np
from isaacsim.core.api import World
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdShade, Sdf

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))
import usd_util

OUT_DIR = ROOT / "training"
if "--outdir" in sys.argv:
    OUT_DIR = Path(sys.argv[sys.argv.index("--outdir") + 1]).expanduser()
OUT_DIR.mkdir(parents=True, exist_ok=True)

DEFECTS = [
    ("defect_hole", "hole", (1.0, 1.0, 1.0)),
    ("defect_crack", "crack", (1.0, 1.0, 1.0)),
]

world = World(stage_units_in_meters=1.0)
stage = world.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage, "/World")

# 4개 STL이 전부 동일한 바운딩박스를 공유하므로 공통 중심 하나만 구하면 된다.
_pts0, _ = usd_util.load_stl(str(ROOT / "pipe" / "meshes" / f"{DEFECTS[0][0]}.stl"))
CENTER = (_pts0.min(0) + _pts0.max(0)) / 2.0

mat_cache = {}


def _material(name, color):
    if name in mat_cache:
        return mat_cache[name]
    mat = UsdShade.Material.Define(stage, f"/World/Mat_{name}")
    shader = UsdShade.Shader.Define(stage, f"/World/Mat_{name}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    mat_cache[name] = mat
    return mat


n = len(DEFECTS)
x0 = -SPACING * (n - 1) / 2.0
for i, (stl_stem, cls, color) in enumerate(DEFECTS):
    stl_path = ROOT / "pipe" / "meshes" / f"{stl_stem}.stl"
    mesh = usd_util.make_mesh(stage, f"/World/{stl_stem}", str(stl_path))
    UsdGeom.Mesh(mesh).CreateDoubleSidedAttr(True)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(_material(cls, color))

    x = x0 + i * SPACING
    # 자기 중심(원점)을 기준으로 Z만 뒤집어(위아래 반전) 슬롯 위치로 옮긴다.
    flip_ud = Gf.Matrix4d(1.0)
    flip_ud.SetScale(Gf.Vec3d(1.0, 1.0, -1.0))
    off = usd_util.trans(*(-CENTER)) * flip_ud * usd_util.trans(x, 0.0, 0.0)
    xf = UsdGeom.Xformable(mesh)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(off)

    # 이름표 대용 — 원점 바로 위에 아주 작은 구를 세워 배치 순서를 표시
    marker = UsdGeom.Sphere.Define(stage, f"/World/{stl_stem}/_label_marker")
    marker.CreateRadiusAttr(0.001)
    UsdGeom.Xformable(marker).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.02))

# 배경 참조용 바닥 평면
floor = UsdGeom.Plane.Define(stage, "/World/Floor")
floor.CreateAxisAttr("Z")
floor.CreateWidthAttr(SPACING * n + 0.1)
floor.CreateLengthAttr(0.15)
UsdGeom.Xformable(floor).AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.02))
floor_mat = UsdShade.Material.Define(stage, "/World/Mat_floor")
floor_shader = UsdShade.Shader.Define(stage, "/World/Mat_floor/Shader")
floor_shader.CreateIdAttr("UsdPreviewSurface")
floor_shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.25, 0.25, 0.27))
floor_shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
floor_mat.CreateSurfaceOutput().ConnectToSource(floor_shader.ConnectableAPI(), "surface")
UsdShade.MaterialBindingAPI.Apply(floor.GetPrim()).Bind(floor_mat)

light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
light.CreateIntensityAttr(1200.0)
key_light = UsdLux.SphereLight.Define(stage, "/World/KeyLight")
key_light.CreateRadiusAttr(0.02)
key_light.CreateIntensityAttr(15000.0)
UsdGeom.Xformable(key_light).AddTranslateOp().Set(Gf.Vec3d(0, -0.15, 0.2))

world.reset()


def export_single(stl_stem, cls, color):
    """결함 하나만 담은 완전히 독립된 스테이지 파일을 새로 만들어 저장한다."""
    out_path = OUT_DIR / f"{stl_stem}_inspect.usd"
    s = Usd.Stage.CreateNew(str(out_path))
    UsdGeom.SetStageUpAxis(s, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(s, "/World")

    stl_path = ROOT / "pipe" / "meshes" / f"{stl_stem}.stl"
    mesh = usd_util.make_mesh(s, f"/World/{stl_stem}", str(stl_path))
    UsdGeom.Mesh(mesh).CreateDoubleSidedAttr(True)

    mat = UsdShade.Material.Define(s, "/World/Mat")
    shader = UsdShade.Shader.Define(s, "/World/Mat/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)

    # 자기 중심을 원점으로 옮기고 Z만 뒤집는다(위아래 반전) — 혼자만 있는
    # 파일이라 슬롯 이동(X 오프셋)은 필요 없다.
    flip_ud = Gf.Matrix4d(1.0)
    flip_ud.SetScale(Gf.Vec3d(1.0, 1.0, -1.0))
    off = usd_util.trans(*(-CENTER)) * flip_ud
    xf = UsdGeom.Xformable(mesh)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(off)

    dome = UsdLux.DomeLight.Define(s, "/World/DomeLight")
    dome.CreateIntensityAttr(1200.0)
    key = UsdLux.SphereLight.Define(s, "/World/KeyLight")
    key.CreateRadiusAttr(0.02)
    key.CreateIntensityAttr(15000.0)
    UsdGeom.Xformable(key).AddTranslateOp().Set(Gf.Vec3d(0, -0.15, 0.2))

    s.GetRootLayer().Save()
    return out_path


print(f"[완료] 결함 {len(DEFECTS)}종을 독립 USD 파일로 저장")
for stl_stem, cls, color in DEFECTS:
    out_path = export_single(stl_stem, cls, color)
    print(f"  {stl_stem} ({cls})  -> {out_path}")

if HOLD:
    print("[대기] GUI 창에서 확인하세요. Ctrl+C 로 종료.")
    try:
        while simulation_app.is_running():
            world.step(render=True)
    except KeyboardInterrupt:
        pass

import os
import threading
threading.Thread(target=simulation_app.close, daemon=True).start()
threading.Event().wait(5.0)
os._exit(0)
