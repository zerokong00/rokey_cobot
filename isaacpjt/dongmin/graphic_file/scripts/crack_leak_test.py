"""균열 배관(inner_pipe_crack1)에서 현재 물 입자 크기로 누수가 나는지 검증.

배경: 팀원이 제작한 inner_pipe_crack1.stl/usd — 축방향 50mm(y 225~275),
원주 90° 위치, 개구 폭 ~1.3mm, 깊이 3.6mm(벽 8mm 중)인 **비관통 홈 크랙**.
STL은 워터타이트(경계 에지 0) — 즉 기하학적으로는 물이 샐 구멍이 없다.
따라서 여기서 "누수"가 관측되면 전부 수치 침투(터널링/오프셋 문제)다.

조건은 water_particle_demo.py 기본 모드와 동일:
  간격 9mm, PCO 8mm, fluid_rest 4.8mm, contact 4mm / rest 2.5mm, dt 1/60
크랙을 바닥(-Z)으로 돌려 수두(만관 ~100mm)가 크랙에 걸리게 한다.

실행:
  PYTHONUNBUFFERED=1 isaac_python crack_leak_test.py --headless   # 숫자만
  PYTHONUNBUFFERED=1 isaac_python crack_leak_test.py              # GUI 확인
"""

import sys

HEADLESS = "--headless" in sys.argv

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": HEADLESS})

from pathlib import Path

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.viewports import set_camera_view
from omni.physx.scripts import particleUtils
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

HERE = Path(__file__).resolve().parent
PIPE_USD = str(HERE.parent / "pipe" / "inner_pipe_crack1.usd")

# ── STL 실측값 (mm) — 사전 파싱으로 확인, 추측 아님 ──
PIPE_LEN_MM = 500.0
PIPE_BORE_R_MM = 50.0
PIPE_OUT_R_MM = 58.0
CRACK_X_MM = (225.0, 275.0)      # 회전 후 월드 x 구간
PIPE_SCALE = 0.001               # USD 는 pipe.usd 와 같은 파이프라인 = mm 단위

# ── 물 — water_particle_demo 기본 모드와 동일 ──
SPACING = 0.009
PCO = 0.008
FLUID_REST = 0.0048
MAX_VEL = 5.0
PHYS_DT = 1.0 / 60.0

CAP_X = (0.02, 0.48)
WATER_X = (0.04, 0.46)
WATER_R_MAX = 0.045
SETTLE_STEPS = 1800              # 30초 정지수 — 터널링은 시간에 따라 누적된다
REPORT_EVERY = 200

world = World(stage_units_in_meters=1.0, physics_dt=PHYS_DT)
stage = world.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage, "/World")
UsdLux.DomeLight.Define(stage, Sdf.Path("/World/DomeLight")).CreateIntensityAttr(1200.0)

pc = world.get_physics_context()
pc.enable_gpu_dynamics(True)
pc.set_broadphase_type("GPU")


def mat4(scale=1.0, rots=(), translate=(0.0, 0.0, 0.0)):
    m = Gf.Matrix4d(1.0)
    m.SetScale(Gf.Vec3d(scale, scale, scale))
    for axis, deg in rots:
        r = Gf.Matrix4d(1.0)
        r.SetRotate(Gf.Rotation(Gf.Vec3d(*axis), deg))
        m = m * r
    t = Gf.Matrix4d(1.0)
    t.SetTranslate(Gf.Vec3d(*translate))
    return m * t


# ══ 균열 배관 — 축 Y→X 회전(Rz -90°) 후 크랙을 바닥으로(Rx 180°) ══
UsdGeom.Xform.Define(stage, "/World/Pipe")
pipe_mat = UsdPhysics.MaterialAPI.Apply(
    UsdShade.Material.Define(stage, "/World/Pipe/Mat").GetPrim())
pipe_mat.CreateStaticFrictionAttr(0.30)
pipe_mat.CreateDynamicFrictionAttr(0.25)
pipe_mat.CreateRestitutionAttr(0.0)

xf = UsdGeom.Xform.Define(stage, "/World/Pipe/section_0")
xf.AddTransformOp().Set(
    mat4(PIPE_SCALE, rots=(((0, 0, 1), -90.0), ((1, 0, 0), 180.0))))
xf.GetPrim().GetReferences().AddReference(PIPE_USD)

# 로드된 메시 검증 — 단위·치수가 가정과 다르면 즉시 중단 (침묵 오류 방지)
n_mesh = 0
for p in stage.Traverse():
    if str(p.GetPath()).startswith("/World/Pipe/section_0") and p.IsA(UsdGeom.Mesh):
        pts = np.array(UsdGeom.Mesh(p).GetPointsAttr().Get())
        span = pts.max(axis=0) - pts.min(axis=0)
        long_mm = float(span.max())
        if not (PIPE_LEN_MM * 0.98 < long_mm < PIPE_LEN_MM * 1.02):
            raise RuntimeError(
                f"메시 최장 치수 {long_mm:.1f} — mm 단위 500 가정과 불일치. "
                f"PIPE_SCALE 재확인 필요")
        UsdPhysics.CollisionAPI.Apply(p)
        UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr("none")
        UsdShade.MaterialBindingAPI.Apply(p).Bind(
            UsdShade.Material.Get(stage, "/World/Pipe/Mat"),
            bindingStrength=UsdShade.Tokens.weakerThanDescendants,
            materialPurpose="physics")
        n_mesh += 1
if n_mesh == 0:
    raise RuntimeError("배관 Mesh 를 못 찾음")

# 밖에서 크랙·입자가 보이게 반투명
glass = UsdShade.Material.Define(stage, "/World/Pipe/Glass")
gsh = UsdShade.Shader.Define(stage, "/World/Pipe/Glass/Shader")
gsh.CreateIdAttr("UsdPreviewSurface")
gsh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
    Gf.Vec3f(0.55, 0.58, 0.60))
gsh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.30)
glass.CreateSurfaceOutput().ConnectToSource(gsh.ConnectableAPI(), "surface")
for p in stage.Traverse():
    if str(p.GetPath()).startswith("/World/Pipe/section_0") and p.IsA(UsdGeom.Mesh):
        UsdShade.MaterialBindingAPI.Apply(p).Bind(
            glass, bindingStrength=UsdShade.Tokens.strongerThanDescendants)

# 양끝 캡 — 밀폐 수조
for k, cx in enumerate(CAP_X):
    cap = UsdGeom.Cube.Define(stage, f"/World/Cap_{k}")
    UsdGeom.XformCommonAPI(cap).SetTranslate(Gf.Vec3d(cx, 0.0, 0.0))
    UsdGeom.XformCommonAPI(cap).SetScale(Gf.Vec3f(0.02, 0.06, 0.06))
    UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
    cap.CreateDisplayOpacityAttr([0.15])

# ══ 물 파티클 — 데모 기본 모드와 동일 파라미터 ══
psys_path = Sdf.Path("/World/ParticleSystem")
particleUtils.add_physx_particle_system(
    stage, psys_path,
    particle_contact_offset=PCO,
    fluid_rest_offset=FLUID_REST,
    contact_offset=0.004,
    rest_offset=0.0025,
    max_velocity=MAX_VEL,
)
water_pbd = UsdShade.Material.Define(stage, "/World/WaterPBD")
particleUtils.AddPBDMaterialWater(water_pbd.GetPrim())
UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(psys_path)).Bind(
    water_pbd, bindingStrength=UsdShade.Tokens.weakerThanDescendants,
    materialPurpose="physics")

positions = []
for x in np.arange(WATER_X[0], WATER_X[1], SPACING):
    for y in np.arange(-WATER_R_MAX, WATER_R_MAX + 1e-9, SPACING):
        for z in np.arange(-WATER_R_MAX, WATER_R_MAX + 1e-9, SPACING):
            if y * y + z * z <= WATER_R_MAX * WATER_R_MAX:
                positions.append(Gf.Vec3f(float(x), float(y), float(z)))
velocities = [Gf.Vec3f(0.0)] * len(positions)

instancer_prim = particleUtils.add_physx_particleset_pointinstancer(
    stage, Sdf.Path("/World/WaterParticles"),
    Vt.Vec3fArray(positions), Vt.Vec3fArray(velocities),
    particle_system_path=psys_path,
    self_collision=True, fluid=True, particle_group=0,
    particle_mass=0.0, density=1000.0,
)
proto = UsdGeom.Sphere(
    stage.GetPrimAtPath("/World/WaterParticles/particlePrototype0"))
proto.GetRadiusAttr().Set(FLUID_REST)
proto.CreateDisplayColorAttr([Gf.Vec3f(0.25, 0.55, 0.85)])
water_instancer = UsdGeom.PointInstancer(instancer_prim)

world.reset()

print("\n" + "=" * 78)
print("균열 배관 누수 테스트 (inner_pipe_crack1, 크랙 바닥 방향)")
print("=" * 78)
print(f"입자 {len(positions):,}개, 간격 {SPACING*1000:.0f}mm, PCO {PCO*1000:.0f}mm, "
      f"contact/rest {4.0}/{2.5}mm, dt 1/{round(1/PHYS_DT)}")
print(f"크랙: x {CRACK_X_MM[0]:.0f}~{CRACK_X_MM[1]:.0f}mm, 바닥(-Z), "
      f"폭 1.3mm 비관통 홈 (STL 워터타이트 — 새면 전부 수치 침투)")

if not HEADLESS:
    set_camera_view(eye=[0.25, -0.25, -0.15], target=[0.25, 0.0, -0.05])


def leak_stats():
    pts = np.array(water_instancer.GetPositionsAttr().Get())
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    r = np.sqrt(y * y + z * z)
    in_wall = (r > PIPE_BORE_R_MM * 1e-3 + 0.002) & (r < PIPE_OUT_R_MM * 1e-3)
    out_wall = r >= PIPE_OUT_R_MM * 1e-3
    out_end = (x < CAP_X[0] - 0.03) | (x > CAP_X[1] + 0.03)
    near_crack = ((CRACK_X_MM[0] * 1e-3 - 0.01 < x)
                  & (x < CRACK_X_MM[1] * 1e-3 + 0.01) & (z < 0)
                  & (in_wall | out_wall))
    return pts, in_wall, out_wall, out_end, near_crack


for step in range(1, SETTLE_STEPS + 1):
    world.step(render=not HEADLESS)
    if step % REPORT_EVERY == 0:
        _, in_wall, out_wall, out_end, near_crack = leak_stats()
        print(f"스텝 {step:>5}: 벽내부 {int(in_wall.sum()):>3} "
              f"벽관통 {int(out_wall.sum()):>3} 끝이탈 {int(out_end.sum()):>3} "
              f"(크랙 부근 {int(near_crack.sum()):>3})")

pts, in_wall, out_wall, out_end, near_crack = leak_stats()
total_leak = int((in_wall | out_wall | out_end).sum())
print("=" * 78)
print(f"최종: 총 이탈/침투 {total_leak}개 / {len(positions):,}개 "
      f"({100.0 * total_leak / len(positions):.2f}%)")
print(f"  벽 내부 침투(홈 포함) {int(in_wall.sum())}, 벽 관통 {int(out_wall.sum())}, "
      f"끝 이탈 {int(out_end.sum())}, 이 중 크랙 부근(x 215~285, 하단) "
      f"{int(near_crack.sum())}")
if int((in_wall | out_wall).sum()):
    lx = pts[in_wall | out_wall, 0] * 1000
    print(f"  침투 입자 x 위치(mm): {np.sort(np.round(lx, 0))[:20]}")
if total_leak == 0:
    print("[OK] 현재 입자 크기(9mm)로는 크랙 침투·누수 없음 — 크랙 개구 1.3mm "
          "<< 입자 유효 지름, rest_offset 2.5mm 로 진입 자체가 차단됨")
else:
    print("[LEAK] 수치 침투 발생 — 위 위치가 크랙 부근이면 크랙 메시 문제, "
          "아니면 일반 터널링")

if not HEADLESS:
    print("\nGUI 실행 중 — 창을 닫으면 종료됩니다.")
    while simulation_app.is_running():
        world.step(render=True)

simulation_app.close()
