"""결함(관통 크랙) 배관에서 물이 새는지 검증하는 데모.

pipe_inner_crack2.stl 실측 (STL 파싱):
  - 실물 스케일: 내경 ø98mm(반경 49), 벽 ~2mm, 길이 200mm, 축 Y
  - 메시 경계 모서리 146개 = 벽 관통 개구부 존재 (축방향 y 55~159mm 구간)

실험 설계:
  - 크랙 섹션 1개를 눕혀(축→X) 양끝을 캡으로 막고 만관 충수 (크랙이 어느
    각도에 있든 잠기도록). 로봇 없음 — 순수 누수 관찰
  - 판정: 관 밖(r>55mm)으로 나간 입자의 "위치"가 크랙 구간(x 55~159mm)에
    몰리면 진짜 누수, 전 구간에 퍼지면 얇은 벽(2mm) 터널링 아티팩트
  - 벽이 얇아 터널링 위험 → dt 1/240 + max_velocity 2 (fine 모드 검증 조합)
  - 입자 지름 7.2mm — 크랙 틈이 이보다 좁으면 물리적으로 새지 않는다
    (그 자체도 유효한 결과: "관통이지만 입자 해상도에서는 밀봉")

실행:
  PYTHONUNBUFFERED=1 isaac_python water_leak_demo.py --headless
  PYTHONUNBUFFERED=1 isaac_python water_leak_demo.py            # GUI 관찰
"""

import math
import sys

HEADLESS = "--headless" in sys.argv
# --hole: 크랙 STL 대신 구멍(ø14mm) 뚫린 배관을 코드로 생성해 사용.
# 크랙 실측 폭 0.3mm 는 입자(7.2mm)가 통과할 수 없어 물리 누수가 불가능하지만
# (해상도 한계), 구멍은 입자보다 크므로 시각 효과 없이 순수 물리로 샌다.
# 메시에서 구멍 자리 삼각형을 실제로 제거하므로 렌더·충돌 모두 뚫려 있다
HOLE = "--hole" in sys.argv
# --shrink N: 입자 크기를 1/N 로 축소 (개수는 N³ 배 증가 주의).
# N>=3 이면 터널링 방지를 위해 dt 1/480 + 최대속도 하향 자동 적용
SHRINK = 1.0
if "--shrink" in sys.argv:
    SHRINK = float(sys.argv[sys.argv.index("--shrink") + 1])
# --sections N: 구멍 배관 N개를 이어 붙이고 각 구간마다 구멍 1개 (HOLE 모드 전용)
SECTIONS = 1
if "--sections" in sys.argv:
    SECTIONS = int(sys.argv[sys.argv.index("--sections") + 1])
# --window W: 물을 배관 중앙 결함 주변 W(m) 구간에만 채움 (부분 채움 기법).
# 경계는 보이지 않는 캡으로 막는다 — 수평관 정지수는 어느 단면이나 정수압이
# 같아 국소 물리가 전체 채움과 동일. 작은 입자를 긴 배관에 쓰기 위한 필수 기법
WINDOW = 0.0
if "--window" in sys.argv:
    WINDOW = float(sys.argv[sys.argv.index("--window") + 1])

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": HEADLESS})

from pathlib import Path

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.viewports import set_camera_view
from omni.physx.scripts import particleUtils
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

HERE = Path(__file__).resolve().parent
PIPE_USD = str(HERE.parent / "pipe" / "pipe_inner_crack2.usd")

# 크랙 배관 실측 (실물 스케일 mm → m 변환만)
PIPE_SCALE = 0.001
BORE_R = 0.049                 # 내벽 반경
PIPE_LEN = 0.200
CRACK_X = (0.055, 0.159)       # 크랙(경계 모서리) 축방향 구간 — 회전 후 월드 X

# 물 입자 (얇은 벽 2mm → 터널링 방지 조합 필수)
SPACING = 0.007 / SHRINK
PCO = 0.006 / SHRINK
FLUID_REST = 0.0036 / SHRINK
MAX_VEL = 2.0
PHYS_DT = 1.0 / 240.0
WATER_R_MAX = 0.044            # 만관 (캡 밀폐라 유지됨)
if SHRINK >= 3.0:
    PHYS_DT = 1.0 / 480.0      # 작은 입자 + 얇은 벽 → 스텝당 이동을 더 조인다
    MAX_VEL = 0.8

SIM_STEPS = int(round(12.0 / PHYS_DT))   # 12초
REPORT_EVERY = SIM_STEPS // 10

world = World(stage_units_in_meters=1.0, physics_dt=PHYS_DT)
stage = world.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage, "/World")
UsdLux.DomeLight.Define(stage, Sdf.Path("/World/DomeLight")).CreateIntensityAttr(1500.0)

pc = world.get_physics_context()
pc.enable_gpu_dynamics(True)
pc.set_broadphase_type("GPU")


def mat4(scale=1.0, rot_axis=None, rot_deg=0.0, translate=(0.0, 0.0, 0.0)):
    m = Gf.Matrix4d(1.0)
    m.SetScale(Gf.Vec3d(scale, scale, scale))
    r = Gf.Matrix4d(1.0)
    if rot_axis is not None:
        r.SetRotate(Gf.Rotation(Gf.Vec3d(*rot_axis), rot_deg))
    t = Gf.Matrix4d(1.0)
    t.SetTranslate(Gf.Vec3d(*translate))
    return m * r * t


HOLE_X = 0.10                  # 구멍 중심 (축방향)
HOLE_ANG = -90.0               # 구멍 각도 (도) — -90 = 관 바닥
HOLE_R = 0.015                 # 구멍 반경 15mm (ø30). ø14 는 유출 0 이었다:
                               # 접촉 오프셋(6mm)이 테두리에서 마개 역할을 해
                               # 실효 통과경 = 구멍 - 2x오프셋 - 메시절삭 ≈ ø2.
                               # 실효 통과경이 입자 지름의 ~2배는 되어야 샌다
SECTION_LEN = 0.200
TOTAL_LEN = SECTION_LEN * SECTIONS if HOLE else PIPE_LEN
# 구간별 구멍: 위치는 각 구간 중앙, 각도는 바닥 부근에서 조금씩 다르게
_ANGS = (-90.0, -65.0, -115.0, -80.0, -100.0)
HOLES = [(SECTION_LEN * i + HOLE_X, _ANGS[i % len(_ANGS)])
         for i in range(SECTIONS)] if HOLE else []
if HOLE:
    CRACK_X = (HOLE_X - 0.02, HOLE_X + 0.02)   # (단일 구간용 잔재 — 다중은 HOLES 기준)


def build_tube_with_hole(stage, path, r_in=0.049, r_out=0.054, length=0.200,
                         nseg=64, naxial=40):
    """구멍 뚫린 이중벽 원통 메시를 직접 생성 (축 = 월드 X).

    - 원주 64분할 (v3 §13.5 테셀레이션 기준)
    - 내/외벽 같은 자리에 구멍 → 관통. 벽 틈 5mm < 입자 지름이라 벽 사이로
      입자가 흘러들 수 없어 테두리 마감 불필요
    - 삼각형을 실제로 제거하므로 시각적으로도 물리적으로도 뚫려 있다
    """
    pts, idx = [], {}

    def vid(surf, i, j, r):
        key = (surf, i, j % nseg)
        if key not in idx:
            ang = 2.0 * math.pi * (j % nseg) / nseg
            pts.append(Gf.Vec3f(length * i / naxial,
                                r * math.cos(ang), r * math.sin(ang)))
            idx[key] = len(pts) - 1
        return idx[key]

    def in_hole(i, j, r):
        x = length * i / naxial
        ang_deg = math.degrees(2.0 * math.pi * (j % nseg) / nseg)
        for hx, hang in HOLES:
            da = (ang_deg - hang + 180.0) % 360.0 - 180.0
            arc = math.radians(da) * r
            if math.hypot(x - hx, arc) < HOLE_R:
                return True
        return False

    counts, indices = [], []
    for surf, r in (("in", r_in), ("out", r_out)):
        for i in range(naxial):
            for j in range(nseg):
                if in_hole(i + 0.5, j + 0.5, r):
                    continue
                a = vid(surf, i, j, r)
                b = vid(surf, i + 1, j, r)
                c = vid(surf, i + 1, j + 1, r)
                d = vid(surf, i, j + 1, r)
                counts.append(4)
                indices.extend((a, b, c, d))
    for i in (0, naxial):                      # 양끝 고리 마감
        for j in range(nseg):
            a = vid("in", i, j, r_in)
            b = vid("out", i, j, r_out)
            c = vid("out", i, j + 1, r_out)
            d = vid("in", i, j + 1, r_in)
            counts.append(4)
            indices.extend((a, b, c, d))

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(indices))
    mesh.CreateDoubleSidedAttr(True)
    return mesh


# ══ 크랙 배관 (로컬 Y축 → 월드 X로 눕힘) ══════════════════════════
pipe_mat_path = "/World/Pipe/PipeMaterial"
UsdGeom.Xform.Define(stage, "/World/Pipe")
pm = UsdPhysics.MaterialAPI.Apply(
    UsdShade.Material.Define(stage, pipe_mat_path).GetPrim())
pm.CreateStaticFrictionAttr(0.30)
pm.CreateDynamicFrictionAttr(0.25)
pm.CreateRestitutionAttr(0.0)

if HOLE:
    build_tube_with_hole(stage, "/World/Pipe/section",
                         length=TOTAL_LEN, naxial=40 * SECTIONS)
else:
    xf = UsdGeom.Xform.Define(stage, "/World/Pipe/section")
    xf.AddTransformOp().Set(mat4(PIPE_SCALE, (0, 0, 1), -90.0, (0.0, 0.0, 0.0)))
    xf.GetPrim().GetReferences().AddReference(PIPE_USD)

n_mesh = 0
for p in stage.Traverse():
    if str(p.GetPath()).startswith("/World/Pipe/section") and p.IsA(UsdGeom.Mesh):
        UsdPhysics.CollisionAPI.Apply(p)
        UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr("none")
        UsdShade.MaterialBindingAPI.Apply(p).Bind(
            UsdShade.Material.Get(stage, pipe_mat_path),
            bindingStrength=UsdShade.Tokens.weakerThanDescendants,
            materialPurpose="physics")
        # 반투명 — 밖에서 누수가 보이게
        n_mesh += 1
if n_mesh == 0:
    raise RuntimeError("배관 Mesh 를 못 찾음")

glass = UsdShade.Material.Define(stage, "/World/Pipe/Glass")
gs = UsdShade.Shader.Define(stage, "/World/Pipe/Glass/Shader")
gs.CreateIdAttr("UsdPreviewSurface")
gs.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.6, 0.6, 0.62))
gs.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.35)
glass.CreateSurfaceOutput().ConnectToSource(gs.ConnectableAPI(), "surface")
for p in stage.Traverse():
    if str(p.GetPath()).startswith("/World/Pipe/section") and p.IsA(UsdGeom.Mesh):
        UsdShade.MaterialBindingAPI.Apply(p).Bind(
            glass, bindingStrength=UsdShade.Tokens.strongerThanDescendants)

# 양끝 캡 (만관 유지)
for k, cx in enumerate((0.004, TOTAL_LEN - 0.004)):
    cap = UsdGeom.Cube.Define(stage, f"/World/Cap_{k}")
    UsdGeom.XformCommonAPI(cap).SetTranslate(Gf.Vec3d(cx, 0.0, 0.0))
    UsdGeom.XformCommonAPI(cap).SetScale(Gf.Vec3f(0.004, 0.06, 0.06))
    UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
    cap.CreateDisplayOpacityAttr([0.15])

# 바닥 (샌 물이 떨어져 고이는 곳 — 날아가 버리면 관찰이 안 된다)
floor = UsdGeom.Cube.Define(stage, "/World/Floor")
UsdGeom.XformCommonAPI(floor).SetTranslate(Gf.Vec3d(TOTAL_LEN * 0.5, 0.0, -0.12))
UsdGeom.XformCommonAPI(floor).SetScale(
    Gf.Vec3f(TOTAL_LEN * 0.5 + 0.2, 0.3, 0.005))
UsdPhysics.CollisionAPI.Apply(floor.GetPrim())

# ══ 물 (만관) ═════════════════════════════════════════════════════
psys_path = Sdf.Path("/World/ParticleSystem")
particleUtils.add_physx_particle_system(
    stage, psys_path,
    particle_contact_offset=PCO,
    fluid_rest_offset=FLUID_REST,
    max_velocity=MAX_VEL,
)
wm = UsdShade.Material.Define(stage, "/World/WaterPBD")
particleUtils.AddPBDMaterialWater(wm.GetPrim())
UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(psys_path)).Bind(
    wm, bindingStrength=UsdShade.Tokens.weakerThanDescendants,
    materialPurpose="physics")

# 채움 (벡터화 — 축소 배율이 크면 입자가 수십만 개라 파이썬 루프는 못 쓴다)
if WINDOW > 0:
    _wc = TOTAL_LEN * 0.5                      # 창 중심 = 배관 중앙 (가운데 구멍)
    _w0, _w1 = _wc - WINDOW * 0.5, _wc + WINDOW * 0.5
    xs = np.arange(_w0 + 0.004, _w1 - 0.004, SPACING)
    # 창 경계 캡 — 충돌만 있고 렌더에서 제외 (시청자에게 안 보임)
    for k, cx in enumerate((_w0, _w1)):
        wcap = UsdGeom.Cube.Define(stage, f"/World/WindowCap_{k}")
        UsdGeom.XformCommonAPI(wcap).SetTranslate(Gf.Vec3d(cx, 0.0, 0.0))
        UsdGeom.XformCommonAPI(wcap).SetScale(Gf.Vec3f(0.003, 0.06, 0.06))
        UsdPhysics.CollisionAPI.Apply(wcap.GetPrim())
        UsdGeom.Imageable(wcap.GetPrim()).MakeInvisible()
else:
    xs = np.arange(0.012, TOTAL_LEN - 0.012, SPACING)
ys = np.arange(-WATER_R_MAX, WATER_R_MAX + 1e-9, SPACING)
X, Y, Z = np.meshgrid(xs, ys, ys, indexing="ij")
m = (Y ** 2 + Z ** 2) <= WATER_R_MAX ** 2
arr = np.stack([X[m], Y[m], Z[m]], axis=1).astype(np.float32)
positions = Vt.Vec3fArray.FromNumpy(arr)
velocities = Vt.Vec3fArray.FromNumpy(np.zeros_like(arr))

inst_prim = particleUtils.add_physx_particleset_pointinstancer(
    stage, Sdf.Path("/World/WaterParticles"),
    positions, velocities,
    particle_system_path=psys_path,
    self_collision=True, fluid=True, particle_group=0,
    particle_mass=0.0, density=1000.0)
proto = UsdGeom.Sphere(stage.GetPrimAtPath("/World/WaterParticles/particlePrototype0"))
proto.GetRadiusAttr().Set(FLUID_REST)
proto.CreateDisplayColorAttr([Gf.Vec3f(0.25, 0.55, 0.85)])
water = UsdGeom.PointInstancer(inst_prim)

# isosurface — 입자를 연속된 물 표면으로 렌더 (구슬처럼 보이는 문제 해결).
# KB 경고: 렌더 전용 + 메모리 누수 이슈 → 짧은 데모 세션에서만
particleUtils.add_physx_particle_isosurface(
    stage, psys_path,
    enabled=True,
    grid_spacing=FLUID_REST * 1.5,
    surface_distance=FLUID_REST * 1.6,
    num_mesh_smoothing_passes=6,
    num_mesh_normal_smoothing_passes=6,
)
wvis = UsdShade.Material.Define(stage, "/World/WaterVis")
ws = UsdShade.Shader.Define(stage, "/World/WaterVis/Shader")
ws.CreateIdAttr("UsdPreviewSurface")
ws.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
    Gf.Vec3f(0.30, 0.60, 0.90))
ws.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
    Gf.Vec3f(0.04, 0.10, 0.18))
ws.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.7)
ws.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.2)
wvis.CreateSurfaceOutput().ConnectToSource(ws.ConnectableAPI(), "surface")
UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(psys_path)).Bind(
    wvis, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
UsdGeom.Imageable(inst_prim).MakeInvisible()   # 구체 대신 isosurface 만 보이게

world.reset()

print("\n" + "=" * 78)
if HOLE:
    print(f"구멍 배관 누수 검증 — 코드 생성, 구간 {SECTIONS}개 x 200mm "
          f"(구멍 ø{HOLE_R * 2000:.0f}mm x {len(HOLES)}개)")
    for hi, (hx, hang) in enumerate(HOLES):
        print(f"  구멍 {hi + 1}: x={hx * 1000:.0f}mm, 각도 {hang:.0f}°")
else:
    print("크랙 배관 누수 검증 — pipe_inner_crack2 (실물 스케일 ø98)")
print("=" * 78)
print(f"입자 {len(positions):,}개 (지름 {FLUID_REST * 2000:.1f} mm), "
      f"{'물 창 ' + format(WINDOW, '.1f') + 'm (부분 채움)' if WINDOW > 0 else '만관 밀폐'}, "
      f"물리 스텝 {1.0 / PHYS_DT:.0f} Hz")


def near_holes_count(cxa):
    """통과 x 가 어느 구멍이든 ±50mm 이내인 개수."""
    if not len(cxa) or not HOLES:
        return 0
    hxs = np.array([h[0] for h in HOLES])
    d = np.min(np.abs(np.asarray(cxa)[:, None] - hxs[None, :]), axis=1)
    return int(np.sum(d < 0.05))

if not HEADLESS:
    set_camera_view(eye=[TOTAL_LEN * 0.5, -(0.30 + 0.14 * SECTIONS),
                         0.08 + 0.04 * SECTIONS],
                    target=[TOTAL_LEN * 0.5, 0.0, -0.03])


# 통과 "순간"의 x 를 기록해야 한다 — 유출수는 바닥에서 퍼지므로 현재 위치로
# 판정하면 결함 위치와 무관하게 분산된 것처럼 보인다 (3차 실행의 오판정)
_escaped = np.zeros(len(positions), dtype=bool)
_cross_x = []


def leak_scan():
    pts = np.array(water.GetPositionsAttr().Get())
    r = np.sqrt(pts[:, 1] ** 2 + pts[:, 2] ** 2)
    out = (r > 0.052) | (pts[:, 2] < -0.06)
    new = out & ~_escaped
    if new.any():
        _cross_x.extend(pts[new, 0].tolist())
        _escaped[new] = True
    return int(_escaped.sum())


SCAN_EVERY = 8 if SHRINK < 3.0 else 48   # 대량 입자는 판독 비용도 크다
# 렌더 분리: 물리는 매 스텝, 화면은 RENDER_EVERY 스텝에 1번만.
# 480Hz 물리에서 매 스텝 렌더하면 GPU 가 렌더링에 잠식된다 — 8스텝당 1렌더
# = 60fps 로 물리 정확도 손실 없이 연산 부담을 크게 줄인다
RENDER_EVERY = 8 if PHYS_DT <= 1.0 / 240.0 else 4
print(f"\n{'스텝':>6} {'경과':>6} {'유출':>7} {'결함구간통과':>10}")
for step in range(1, SIM_STEPS + 1):
    world.step(render=(not HEADLESS and step % RENDER_EVERY == 0))
    if step % SCAN_EVERY == 0:
        leak_scan()
    if step % REPORT_EVERY == 0:
        in_crack = near_holes_count(_cross_x)
        print(f"{step:>6} {step * PHYS_DT:>5.1f}s {int(_escaped.sum()):>7} "
              f"{in_crack:>10}")

n_out = leak_scan()
cx = np.array(_cross_x) if _cross_x else np.zeros(0)
in_crack = near_holes_count(cx)
if HOLE and len(cx):
    print("-" * 78)
    for hi, (hx, hang) in enumerate(HOLES):
        n_h = int(np.sum(np.abs(cx - hx) < 0.05))
        print(f"구멍 {hi + 1} (x={hx * 1000:.0f}mm, {hang:.0f}°): 통과 {n_h:,}개")
print("=" * 78)
if n_out == 0:
    print("[결과] 유출 0 — 크랙 틈이 입자 지름(7.2mm)보다 좁아 이 해상도에서는 밀봉. "
          "관통 여부와 별개로 입자 크기의 한계")
elif in_crack / max(n_out, 1) > 0.7:
    print(f"[결과] 유출 {n_out}개 중 {in_crack}개가 크랙 구간 — **크랙 누수 재현 성공**")
else:
    print(f"[결과] 유출 {n_out}개, 크랙 구간 비율 {in_crack}/{n_out} — "
          "전 구간 분산 = 얇은 벽(2mm) 터널링 아티팩트 우세. dt/입자 크기 재조정 필요")

if not HEADLESS:
    print("\nGUI 실행 중 — 창을 닫으면 종료됩니다.")
    while simulation_app.is_running():
        world.step(render=True)

simulation_app.close()
