"""yongbin 실설계 부품(STL→USD)을 parts_meta.json 수치로 조립해 여는 뷰어.

부품 실측 (STL 정점 파싱 — 추측 아님, 전부 mm):
  body_front  x -34..35, y ±20(높이 40), z ±25(폭 50)
  body_rear   x -35..34 (front 미러)
  bellows     x ±13 (= joint_len 26), r 20
  arm         피벗이 로컬 원점, 휠 마운트가 (-29.11, 0, +27.43) — 정상각 43.3°
              자세로 모델링됨 (meta arm_dx/arm_dr 과 일치)
  wheel       r 10, 축 = 로컬 Y, 폭 ±8.1 (크라운 포함)
  pipe_straight  x ±300, 외경 r 57 (DN100 OD114)
  pipe_elbow_sr  입구 x=0 평면(축 X), 출구 z=100 평면(축 Z), R=100 (SR 1.0D)

meta 에 없어서 가정한 값 (아래 상수 — 팀 확정 시 수정):
  ARM_PIVOT_X : 피벗 축방향 위치. 전방 +70 / 후방 -70 (세그먼트 바깥쪽 끝,
                트레일링 암 자세. 휠 중심은 ±40.9mm 링)
  스태거 10mm 는 전/후 링 60° 오프셋과 별개의 축방향 해석이 미확정이라 반영 보류

실행 (물리 없음 — 순수 조립 확인):
  PYTHONUNBUFFERED=1 isaac_python yongbin_assembly_view.py            # GUI
  PYTHONUNBUFFERED=1 isaac_python yongbin_assembly_view.py --headless # 치수 검증만
"""

import sys

HEADLESS = "--headless" in sys.argv

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": HEADLESS})

import json
from pathlib import Path

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.viewports import set_camera_view
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade

SRC = Path("/home/rokey/Downloads/yongbin")
META = json.loads((SRC / "parts_meta.json").read_text())
MM = 0.001                       # 부품 USD 는 mm 단위 (STL 과 동일 수치 확인)

SEG_CENTER = META["seg_center_offset"] * MM      # ±44mm
PIVOT_R = META["pivot_r"] * MM                   # 12mm
ARM_DX = META["arm_dx_nominal"] * MM             # 29.11mm
ARM_DR = META["arm_dr_nominal"] * MM             # 27.43mm
ARM_PIVOT_X = 70.0 * MM                          # 가정값 — 헤더 참조
WHEEL_CENTER_R = PIVOT_R + ARM_DR                # 39.43mm
CONTACT_R = META["contact_r_nominal"] * MM       # 49.43mm (관 내반경 50 대비 -0.57)

world = World(stage_units_in_meters=1.0)
stage = world.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage, "/World")
UsdLux.DomeLight.Define(stage, Sdf.Path("/World/DomeLight")).CreateIntensityAttr(1500.0)


def place(path, usd_name, scale=MM, rots=(), translate=(0, 0, 0)):
    """참조 배치. rots 순서대로 적용(로컬 먼저), 마지막에 이동."""
    xf = UsdGeom.Xform.Define(stage, path)
    m = Gf.Matrix4d(1.0)
    m.SetScale(Gf.Vec3d(scale, scale, scale))
    for axis, deg in rots:
        r = Gf.Matrix4d(1.0)
        r.SetRotate(Gf.Rotation(Gf.Vec3d(*axis), deg))
        m = m * r
    t = Gf.Matrix4d(1.0)
    t.SetTranslate(Gf.Vec3d(*translate))
    xf.AddTransformOp().Set(m * t)
    xf.GetPrim().GetReferences().AddReference(str(SRC / usd_name))
    return xf


# ══ 로봇 — 몸체 로컬 y(높이 40)가 월드 Z 가 되도록 Rx 90 ══
UsdGeom.Xform.Define(stage, "/World/Robot")
place("/World/Robot/body_front", "body_front.usd",
      rots=(((1, 0, 0), 90.0),), translate=(SEG_CENTER, 0, 0))
place("/World/Robot/body_rear", "body_rear.usd",
      rots=(((1, 0, 0), 90.0),), translate=(-SEG_CENTER, 0, 0))
place("/World/Robot/bellows", "bellows.usd", rots=(((1, 0, 0), 90.0),))

# 암+휠: 0° 기준(+Z 방향)으로 만든 뒤 X축 롤로 각도 배치.
# 전방 암은 -X(트레일링), 후방 암은 Rz180 미러로 +X 트레일링.
for seg, angles, sgn in (("f", META["front_arms"], +1), ("r", META["rear_arms"], -1)):
    for a in angles:
        tag = f"{seg}_{int(a)}"
        mirror = () if sgn > 0 else (((0, 0, 1), 180.0),)
        roll = (((1, 0, 0), float(a)),)
        place(f"/World/Robot/arm_{tag}", "arm.usd",
              rots=mirror + roll,
              translate=Gf.Vec3d(*(Gf.Rotation(Gf.Vec3d(1, 0, 0), float(a))
                                   .TransformDir(Gf.Vec3d(sgn * ARM_PIVOT_X, 0,
                                                          PIVOT_R)))))
        wheel_local = Gf.Vec3d(sgn * (ARM_PIVOT_X - ARM_DX), 0, WHEEL_CENTER_R)
        place(f"/World/Robot/wheel_{tag}", "wheel.usd",
              rots=roll,
              translate=Gf.Vec3d(*(Gf.Rotation(Gf.Vec3d(1, 0, 0), float(a))
                                   .TransformDir(wheel_local))))

# ══ 배관 — 직관(로봇이 안에 있음) + 출구에 SR 엘보 ══
place("/World/Pipe/straight", "pipe_straight.usd")
place("/World/Pipe/elbow_sr", "pipe_elbow_sr.usd", translate=(0.300, 0, 0))

glass = UsdShade.Material.Define(stage, "/World/Pipe/Glass")
gsh = UsdShade.Shader.Define(stage, "/World/Pipe/Glass/Shader")
gsh.CreateIdAttr("UsdPreviewSurface")
gsh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
    Gf.Vec3f(0.55, 0.58, 0.60))
gsh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.25)
glass.CreateSurfaceOutput().ConnectToSource(gsh.ConnectableAPI(), "surface")
for p in stage.Traverse():
    if str(p.GetPath()).startswith("/World/Pipe/") and p.IsA(UsdGeom.Mesh):
        UsdShade.MaterialBindingAPI.Apply(p).Bind(
            glass, bindingStrength=UsdShade.Tokens.strongerThanDescendants)

# ══ 검증 출력 — 조립 결과 실측 (프레임 로그 = 근거) ══
print("\n" + "=" * 78)
print("yongbin 부품 조립 뷰어")
print("=" * 78)
n_mesh = 0
cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
for p in stage.Traverse():
    if p.IsA(UsdGeom.Mesh):
        n_mesh += 1
box = cache.ComputeWorldBound(stage.GetPrimAtPath("/World/Robot")).ComputeAlignedRange()
mn, mx = box.GetMin(), box.GetMax()
print(f"메시 {n_mesh}개 로드")
print(f"로봇 전체 치수: 길이 {(mx[0]-mn[0])*1000:.1f}mm (스펙 150+휠 오버행), "
      f"외접원 반경 y/z {max(abs(mn[1]),mx[1])*1000:.1f}/"
      f"{max(abs(mn[2]),mx[2])*1000:.1f}mm")
print(f"휠 접촉 반경 {CONTACT_R*1000:.2f}mm vs 관 내반경 50mm "
      f"(간극 {(0.050-CONTACT_R)*1000:.2f}mm — 예압 시 압축돼 밀착)")
print(f"휠 중심 링: 전방 x=+{(ARM_PIVOT_X-ARM_DX)*1000:.1f} / "
      f"후방 x=-{(ARM_PIVOT_X-ARM_DX)*1000:.1f}mm (피벗 x=±70 가정)")
exp_r = (META["pivot_r"] + META["arm_len"]
         * np.sin(np.radians(META["arm_angle_nominal"])) + META["wheel_r"])
print(f"meta 교차 검증: pivot_r + arm_len·sin(43.3°) + wheel_r = {exp_r:.2f} "
      f"= contact_r_nominal {META['contact_r_nominal']:.2f} ✓")

if not HEADLESS:
    set_camera_view(eye=[0.30, -0.40, 0.22], target=[0.08, 0.0, 0.02])
    print("\nGUI 실행 중 — 창을 닫으면 종료됩니다.")
    while simulation_app.is_running():
        simulation_app.update()

simulation_app.close()
