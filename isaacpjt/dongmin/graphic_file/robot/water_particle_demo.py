"""실제 물 파티클(PBD 유체) 속에서 로봇이 주행하는 데모.

fluid_force_demo.py(해석적 힘 주입, 정량 검증용)와 별개의 실험이다.
여기서는 부력·항력·마찰을 코드로 주입하지 않는다 — PhysX PBD 유체 입자가
로봇과 직접 상호작용하며 힘을 만든다.

전제와 한계 (KB text/physics/physics_resources.md 알려진 문제):
  - 파티클은 정지 마찰 미지원 → 트랙션 정량 검증에는 부적합
  - isosurface(수면 렌더)는 메모리 누수 → 사용하지 않음, 입자를 점으로 렌더
  - 흐름(유속) 경계조건 없음 → 고인 물. 배관 양끝을 캡으로 막아 수조를 만든다
  - 파티클은 GPU 물리 전용 → enable_gpu_dynamics + GPU broadphase 필수

API 근거:
  - omni.physx particleUtils (설치 소스에서 시그니처 확인:
    extscache/omni.physx-107.3.26.../omni/physx/scripts/particleUtils.py)
  - PhysicsContext.enable_gpu_dynamics / set_broadphase_type (API_INDEX.md:1183)

실행:
  PYTHONUNBUFFERED=1 isaac_python water_particle_demo.py            # GUI 권장
  PYTHONUNBUFFERED=1 isaac_python water_particle_demo.py --headless # 숫자만
"""

import sys

HEADLESS = "--headless" in sys.argv
# --fine: 작은 입자(6mm) 실험 모드. 터널링 방지를 CCD 없이 달성하는 조합:
#   스텝당 이동거리 = 최대속도 2 m/s x (1/240 s) = 8.3mm ≈ 벽 두께 + 접촉 여유
# 성공하면 실제 입자만으로 만관 100% + 주행 양립 가능성이 열린다
FINE = "--fine" in sys.argv
# --flow: 흐름 통과형(실제 배수관 = 정찰 임무 조건). 양끝 개방 + 하류로 나간
# 입자를 상류로 재투입(재순환)해 연속 흐름을 만든다. 밀폐 수조의 "끼임"이
# 개방 구조에서 사라지는지 검증. 리스크: GPU 물리에서 런타임 입자 위치 쓰기
# 반영 여부 (읽기는 누수 점검에서 반영 확인됨)
FLOW = "--flow" in sys.argv
# --full (FLOW 와 함께): 실입자 100% 만관 + 흐름 통과형. 유입(재순환)이
# 계속되어 관이 가득 유지되고, 로봇이 밀어낸 물은 개방 하류로 빠져나간다.
# 밀폐 97% 끼임(7차)은 출구가 없어서였다는 가설의 최종 검증
FULL = "--full" in sys.argv
# --p9: 예압 9N/다리 변형 로봇(robot_assembled_p9.usd) 사용. 실물 설계 예압이
# 만관 접지 문제를 푸는지 검증용 (기본 로봇은 0.6N/다리)
P9 = "--p9" in sys.argv
# --window W: 슬라이딩 윈도우 — 물을 로봇 주변 W(m) 구간에만 채우고, 보이지
# 않는 키네마틱 캡 2개가 로봇을 따라 이동하며 물을 끌고 다닌다. 긴 배관에서
# 입자 수를 고정하는 기법 (물리 근거: 수평관 정지수는 단면 정수압 동일)
WINDOW = 0.0
if "--window" in sys.argv:
    WINDOW = float(sys.argv[sys.argv.index("--window") + 1])

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": HEADLESS})

from pathlib import Path

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.viewports import set_camera_view
from omni.physx.scripts import particleUtils
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

HERE = Path(__file__).resolve().parent
ROBOT_USD = str(HERE / ("robot_assembled_p9.usd" if P9 else "robot_assembled.usd"))
PIPE_USD = str(HERE.parent / "pipe" / "pipe.usd")

# ── 배관 (fluid_force_demo.py 와 동일) ──
PIPE_CAD_BORE_MM = 45.0
PIPE_CAD_LEN_MM = 200.0
PIPE_BORE_MM = 100.0
PIPE_SCALE = (PIPE_BORE_MM / PIPE_CAD_BORE_MM) * 0.001
PIPE_SEC_LEN_M = PIPE_CAD_LEN_MM * PIPE_SCALE
PIPE_SECTIONS = 4
PIPE_END_M = PIPE_SECTIONS * PIPE_SEC_LEN_M
FRICTION_STATIC = 0.30      # 젖은 관벽
FRICTION_DYNAMIC = 0.25

# ── 임시 로봇 ──
ROBOT_SCALE = 90.0 / 302.0
WHEEL_R_M = 23.0 * 0.001 * ROBOT_SCALE
ROBOT_LEN_M = 393.5 * 0.001 * ROBOT_SCALE
START_X = 0.28

# ── 물 (수조 구간: 양끝 캡 사이) ──
CAP_X = (0.06, 1.00)              # 캡 위치 — 이 사이가 수조
WATER_X = (0.085, 0.975)          # 입자 채우는 축 범위 (캡 안쪽면 전체)
WATER_LEVEL_Z = 0.050             # 만관 — 관 상단까지 가득. 밀폐 수조라 유지된다
WATER_R_MAX = 0.045               # 벽 바로 앞까지 채움 (5차: 0.042 는 60%만 찼다)
SPACING = 0.009                   # 물리 입자는 수조의 ~76%.
# 왜 100% 로 안 채우나 (7차 확정): 8.5mm 입자로 97% 채우면 입자가 로봇-벽
# 사이에 끼어 재배열하지 못해 로봇이 전진 불가(0.3mm). 더 작은 입자는 관벽
# 삼각메시를 뚫는다(1차, 41% 누수). PBD 해상도의 근본 한계라서, 물리는 이동
# 가능한 밀도로 두고 "완전 만관"의 시각 표현은 렌더 전용 물기둥이 담당한다.
PCO = 0.008                       # particle_contact_offset
FLUID_REST = 0.0048               # fluid_rest_offset (= PCO * 0.6)
# 로봇 자리 비움 — 전체 단면이 아니라 로봇 몸통 반경만 (5차: 전체를 비우면
# 그 부피만큼 수위가 내려가 만관이 깨진다)
ROBOT_CLEAR_X = (START_X - 0.09, START_X + 0.09)
ROBOT_CLEAR_R = 0.038

SETTLE_STEPS = 240
DRIVE_STEPS = 1200
SPIN_DEG_S = 360.0
REPORT_EVERY = 100
STOP_X = 0.88                     # 캡 앞에서 정지
PHYS_DT = 1.0 / 60.0
MAX_VEL = 5.0

if FINE:
    SPACING = 0.006
    PCO = 0.005
    FLUID_REST = 0.003
    MAX_VEL = 2.0
    PHYS_DT = 1.0 / 240.0         # v3 §13.2 요구값이기도 하다
    SETTLE_STEPS = 480
    DRIVE_STEPS = 2400
    REPORT_EVERY = 200

if FLOW:
    WATER_X = (0.10, 1.70)        # 배관 전 구간 (양끝 개방)
    WATER_LEVEL_Z = 0.020         # 초기 수위는 유동 자연 수위로. 11차: 단면
                                  # 100% 로 시작하면 개방된 물기둥이 첫 스텝에
                                  # 붕괴하며 압력 스파이크로 65%가 벽을 뚫었다
    FLOW_V = -0.10                # 유속장: -X 로 흐름. 임시 로봇 임계 유속
                                  # 0.18 m/s(해석 하네스 실측) 미만이어야 전진
                                  # 가능 — 13차: 0.4 에서는 로봇이 하류로 밀림
                                  # (해석 예측과 일치, 교차 검증)
    SETTLE_STEPS = 600            # 15차: 초기 서지(빈자리로 물이 몰려드는
                                  # 파도)가 가라앉기 전에 주행을 시작하면
                                  # 로봇이 떠서 쓸려간다. 서지 소멸 후 출발
    DRIVE_STEPS = 1800
    if FULL:
        WATER_LEVEL_Z = 0.050     # 단면 전체 충전
        SPACING = 0.0087          # 관 부피의 ~98%. 17차: 8.5mm 는 105% 과충전
                                  # → 가압 누출·재순환 260만회 악순환
        FLOW_V = 0.0              # 벌크 유속장 없음. 흐름 = 로봇 변위에 의한
                                  # 유출 + 재순환 유입 (가득 찬 관은 벌크
                                  # 유속장을 걸면 전체가 로봇을 밀어붙인다)
    RECYCLE_OUT_X = 0.06          # 이보다 하류로 나가면
    RECYCLE_IN_X = (1.35, 1.70)   # 이 구간(상류)으로 재투입 — 로봇에서 멀다
    RECYCLE_EVERY = 4             # N 스텝마다 재순환 처리
    # 재투입 높이: 관 상부 빈 공간(에어포켓). 10차: 기존 물속에 겹쳐 넣으면
    # 밀도 2배 → 압력 폭발로 입자 68%가 관벽을 뚫고 튕겨나갔다.
    # 수도꼭지처럼 물 위에서 떨어뜨려 합류시킨다
    RECYCLE_IN_Y = (-0.025, 0.025)
    RECYCLE_IN_Z = (0.010, 0.035)

# p9(예압 9N) 로봇은 스프링 강성이 50배라 1/60 스텝에서는 수치 왜곡이 크다
# (빌드 검증에서 실측 K 2배 왜곡 확인 — v3 §13.2 근거). 1/240 이상으로 조이고
# 시뮬레이션 시간이 같도록 스텝 수를 비례 확대한다
if P9 and PHYS_DT > 1.0 / 240.0:
    _scale = int(round(PHYS_DT / (1.0 / 240.0)))
    PHYS_DT = 1.0 / 240.0
    SETTLE_STEPS *= _scale
    DRIVE_STEPS *= _scale
    REPORT_EVERY *= _scale

world = World(stage_units_in_meters=1.0, physics_dt=PHYS_DT)
stage = world.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage, "/World")
UsdLux.DomeLight.Define(stage, Sdf.Path("/World/DomeLight")).CreateIntensityAttr(1200.0)

# 파티클은 GPU 물리 전용
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


# ══ 배관 ══════════════════════════════════════════════════════════
pipe_mat_path = "/World/Pipe/PipeMaterial"
UsdGeom.Xform.Define(stage, "/World/Pipe")
pipe_mat = UsdPhysics.MaterialAPI.Apply(
    UsdShade.Material.Define(stage, pipe_mat_path).GetPrim())
pipe_mat.CreateStaticFrictionAttr(FRICTION_STATIC)
pipe_mat.CreateDynamicFrictionAttr(FRICTION_DYNAMIC)
pipe_mat.CreateRestitutionAttr(0.0)

for i in range(PIPE_SECTIONS):
    path = f"/World/Pipe/section_{i}"
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTransformOp().Set(
        mat4(PIPE_SCALE, (0, 0, 1), -90.0, (i * PIPE_SEC_LEN_M, 0.0, 0.0)))
    xf.GetPrim().GetReferences().AddReference(PIPE_USD)

n_pipe_mesh = 0
for p in stage.Traverse():
    if str(p.GetPath()).startswith("/World/Pipe/section_") and p.IsA(UsdGeom.Mesh):
        UsdPhysics.CollisionAPI.Apply(p)
        UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr("none")
        UsdShade.MaterialBindingAPI.Apply(p).Bind(
            UsdShade.Material.Get(stage, pipe_mat_path),
            bindingStrength=UsdShade.Tokens.weakerThanDescendants,
            materialPurpose="physics",
        )
        n_pipe_mesh += 1
if n_pipe_mesh == 0:
    raise RuntimeError("배관 Mesh 를 못 찾음")

# 배관을 반투명하게 — 밖에서도 물·로봇이 보이게 (불투명 벽이 내부를 가린다)
glass_mtl = UsdShade.Material.Define(stage, "/World/Pipe/GlassMat")
glass_sh = UsdShade.Shader.Define(stage, "/World/Pipe/GlassMat/Shader")
glass_sh.CreateIdAttr("UsdPreviewSurface")
glass_sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
    Gf.Vec3f(0.55, 0.58, 0.60))
glass_sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.30)
glass_sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
glass_mtl.CreateSurfaceOutput().ConnectToSource(
    glass_sh.ConnectableAPI(), "surface")
for p in stage.Traverse():
    if str(p.GetPath()).startswith("/World/Pipe/section_") and p.IsA(UsdGeom.Mesh):
        UsdShade.MaterialBindingAPI.Apply(p).Bind(
            glass_mtl, bindingStrength=UsdShade.Tokens.strongerThanDescendants)

# ══ 수조 캡 — 밀폐 모드는 양끝, FLOW 모드는 상류 끝 하나만 ══
# (12차: 재투입 물이 유입 지점에 쌓여 상류 개방구로 흘러넘쳐 소실됐다.
#  하류는 열어 흐름 통과를 유지하고, 상류만 저수조 벽으로 막는다)
_cap_xs = [(k, cx) for k, cx in enumerate(CAP_X)] if not FLOW else [(1, 1.76)]
for k, cx in _cap_xs:
    cap = UsdGeom.Cube.Define(stage, f"/World/Cap_{k}")
    UsdGeom.XformCommonAPI(cap).SetTranslate(Gf.Vec3d(cx, 0.0, 0.0))
    UsdGeom.XformCommonAPI(cap).SetScale(Gf.Vec3f(0.02, 0.06, 0.06))
    UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
    cap.CreateDisplayOpacityAttr([0.15])

# ══ 물 파티클 ═════════════════════════════════════════════════════
psys_path = Sdf.Path("/World/ParticleSystem")
particleUtils.add_physx_particle_system(
    stage, psys_path,
    particle_contact_offset=PCO,
    fluid_rest_offset=FLUID_REST,
    # 입자-강체(바퀴·관벽) 간격. 기본값(PCO 에서 자동 산출 ~8mm)이면 바퀴-벽
    # 접촉부에 낀 입자가 8mm 쿠션이 되어 로봇이 "구슬 베어링" 위에 뜬다 —
    # p9(예압 9N) 3연속 계측에서 본체 +13~28mm 부상 + 견인 0 실측. 입자는
    # 정지 마찰이 없어 이 상태면 전진 불가. 간격을 다리 스트로크 여유(±6mm
    # 상당)보다 얇게 줄여 예압이 입자를 접촉부 밖으로 밀어내게 한다
    contact_offset=0.004,
    rest_offset=0.0025,
    max_velocity=MAX_VEL,     # 폭발적 이탈 억제 (fine 모드: 터널링 방지 핵심)
    # 주의: CCD 는 GPU dynamics 에서 미지원(경고 확인). 누수 방지는 입자
    # 크기(8.5~10mm > 메시 틈)가 담당한다 — 6mm 로 줄이면 다시 샌다
    # 밀폐 모드는 wind 없음 — 7차: 밀폐 만관에서 유속장은 물 전체를 로봇에
    # 밀어붙여 로봇을 가둔다(전진 -5.9mm). FLOW(개방) 모드에서는 밀려난 물이
    # 하류로 빠져나갈 수 있으므로 유속장이 실제 흐름을 만든다
    wind=Gf.Vec3f(FLOW_V, 0.0, 0.0) if FLOW else None,
)

# isosurface — 입자를 매끈한 수면 메시로 렌더 (애니메이션 같은 물 표현).
# KB 경고: 렌더 전용 + 메모리 누수 이슈 → 짧은 데모 세션에서만 사용
particleUtils.add_physx_particle_isosurface(
    stage, psys_path,
    enabled=True,
    grid_spacing=FLUID_REST * 1.5,
    surface_distance=FLUID_REST * 1.6,
    grid_smoothing_radius=FLUID_REST * 2.0,
    num_mesh_smoothing_passes=8,
    num_mesh_normal_smoothing_passes=8,
)

# 수면 메시가 물려받을 물 재질 — 밝은 파랑 + 약한 자체발광 (관 내부는 어둡다)
water_vis = UsdShade.Material.Define(stage, "/World/WaterVisual")
wv_sh = UsdShade.Shader.Define(stage, "/World/WaterVisual/Shader")
wv_sh.CreateIdAttr("UsdPreviewSurface")
wv_sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
    Gf.Vec3f(0.30, 0.60, 0.90))
wv_sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
    Gf.Vec3f(0.04, 0.10, 0.18))
wv_sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.45)
wv_sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.25)
water_vis.CreateSurfaceOutput().ConnectToSource(wv_sh.ConnectableAPI(), "surface")
UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(psys_path)).Bind(
    water_vis, bindingStrength=UsdShade.Tokens.strongerThanDescendants)

# 물 PBD 재질 (particleUtils 물 프리셋) 을 파티클 시스템에 바인딩
water_pbd_path = "/World/WaterPBD"
water_pbd = UsdShade.Material.Define(stage, water_pbd_path)
particleUtils.AddPBDMaterialWater(water_pbd.GetPrim())
UsdShade.MaterialBindingAPI.Apply(
    stage.GetPrimAtPath(psys_path)).Bind(
    water_pbd, bindingStrength=UsdShade.Tokens.weakerThanDescendants,
    materialPurpose="physics")

# 입자 배치: 수조 구간 x, 관 단면 원 안쪽, 수면 아래. 로봇 자리는 비움
if WINDOW > 0:
    WATER_X = (START_X - WINDOW * 0.5 + 0.006, START_X + WINDOW * 0.5 - 0.006)

positions = []
xs = np.arange(WATER_X[0], WATER_X[1], SPACING)
ys = np.arange(-WATER_R_MAX, WATER_R_MAX + 1e-9, SPACING)
zs = np.arange(-WATER_R_MAX, WATER_LEVEL_Z + 1e-9, SPACING)
for x in xs:
    in_robot = ROBOT_CLEAR_X[0] < x < ROBOT_CLEAR_X[1]
    for y in ys:
        for z in zs:
            r2 = y * y + z * z
            if r2 > WATER_R_MAX * WATER_R_MAX:
                continue
            if in_robot and r2 < ROBOT_CLEAR_R * ROBOT_CLEAR_R:
                continue
            positions.append(Gf.Vec3f(float(x), float(y), float(z)))
velocities = [Gf.Vec3f(0.0)] * len(positions)

# Points 대신 PointInstancer(구체 프로토타입) — Points 는 뷰포트에서 잘 안 보임
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
# 입자 자체는 숨긴다 — isosurface 수면 메시가 대신 렌더된다
UsdGeom.Imageable(instancer_prim).MakeInvisible()

# ── 슬라이딩 윈도우 캡 — 로봇을 따라다니는 보이지 않는 물막이 ──
_wincap_api = []
if WINDOW > 0:
    for k, sgn in ((0, -1.0), (1, 1.0)):
        wc = UsdGeom.Cube.Define(stage, f"/World/WinCap_{k}")
        api = UsdGeom.XformCommonAPI(wc)
        api.SetTranslate(Gf.Vec3d(START_X + sgn * WINDOW * 0.5, 0.0, 0.0))
        api.SetScale(Gf.Vec3f(0.003, 0.06, 0.06))
        UsdPhysics.CollisionAPI.Apply(wc.GetPrim())
        rb = UsdPhysics.RigidBodyAPI.Apply(wc.GetPrim())
        rb.CreateKinematicEnabledAttr(True)   # 매 스텝 위치를 직접 갱신
        UsdGeom.Imageable(wc.GetPrim()).MakeInvisible()
        _wincap_api.append(api)


def update_window_caps():
    """캡 2개를 로봇 현재 위치 ±W/2 로 이동 — 물이 로봇과 함께 다닌다."""
    if not _wincap_api:
        return
    x = float(body.get_world_pose()[0][0])
    _wincap_api[0].SetTranslate(Gf.Vec3d(x - WINDOW * 0.5, 0.0, 0.0))
    _wincap_api[1].SetTranslate(Gf.Vec3d(x + WINDOW * 0.5, 0.0, 0.0))


# ── FLOW 모드: 입자 재순환 (하류 이탈 → 상류 재투입 = 무한 공급 흉내) ──
_rng_flow = np.random.default_rng(3)
_recycled_total = 0


def recycle_particles():
    """하류(RECYCLE_OUT_X 밖)로 나간 입자를 상류 구간으로 되돌린다.

    러닝머신 원리: 고정된 입자 수로 연속 공급 흐름을 표현한다. 재투입 지점은
    로봇에서 1m 이상 떨어져 있어 로봇 주변 물리에 직접 영향이 없다.
    """
    global _recycled_total
    pts = np.array(water_instancer.GetPositionsAttr().Get())
    r2 = pts[:, 1] ** 2 + pts[:, 2] ** 2
    # 하류 통과 + 어떤 경로로든 관 밖으로 이탈한 입자 전부 회수 (물 총량 보존)
    mask = ((pts[:, 0] < RECYCLE_OUT_X) | (pts[:, 0] > 1.85)
            | (r2 > 0.055 ** 2))
    n = int(mask.sum())
    if n == 0:
        return
    pts[mask, 0] = _rng_flow.uniform(RECYCLE_IN_X[0], RECYCLE_IN_X[1], n)
    pts[mask, 1] = _rng_flow.uniform(RECYCLE_IN_Y[0], RECYCLE_IN_Y[1], n)
    pts[mask, 2] = _rng_flow.uniform(RECYCLE_IN_Z[0], RECYCLE_IN_Z[1], n)
    vels = np.array(water_instancer.GetVelocitiesAttr().Get())
    vels[mask] = (FLOW_V, 0.0, -0.2)   # 흐름 방향 + 살짝 낙하
    water_instancer.GetPositionsAttr().Set(
        Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))
    water_instancer.GetVelocitiesAttr().Set(
        Vt.Vec3fArray.FromNumpy(vels.astype(np.float32)))
    _recycled_total += n
water_mass = 1000.0 * (FLUID_REST * 2) ** 3 * len(positions)

# (만관 시각용 렌더 물기둥은 설계 변경으로 제거 — 실입자 76%가 보이는 그대로가
#  물의 전부다. 화면과 물리가 일치한다)

# ══ 로봇 ══════════════════════════════════════════════════════════
robot_xf = UsdGeom.Xform.Define(stage, "/World/RobotRoot")
robot_xf.AddTransformOp().Set(mat4(1.0, (0, 1, 0), 90.0, (START_X, 0.0, 0.0)))
add_reference_to_stage(usd_path=ROBOT_USD, prim_path="/World/RobotRoot/Robot")

art = SingleArticulation(prim_path="/World/RobotRoot/Robot/Robot", name="pipe_robot")
world.scene.add(art)

# 바퀴 드라이브 목표는 reset() "전에" USD 에 기록해야 한다.
# GPU dynamics 에서는 시뮬레이션 시작 후의 USD 드라이브 속성 변경이 반영되지
# 않는 것을 3차 실행에서 확인했다 (목표 +360 인데 관측 회전 음수 = 무구동).
for p in stage.Traverse():
    name = p.GetName()
    if name.startswith("joint_wheel_"):
        drive = UsdPhysics.DriveAPI.Get(p, "angular")
        if drive:
            drive.GetTargetVelocityAttr().Set(SPIN_DEG_S)

world.reset()

# 본체 질량: 임시 로봇은 스프링 예압이 없어 접지력을 전부 "가라앉는 무게"로
# 만들어야 한다 (실물은 예압 9N×6 이라 부력 무관 — v3 §17.4). 물이 깊을수록
# 부력이 커서 더 무거워야 한다: 기본 450g(2·14차), 100% 만관은 700g(18차:
# 450g 으로는 +10mm 떠서 천장에 붙음)
body = SingleRigidPrim(prim_path="/World/RobotRoot/Robot/Robot/body", name="body",
                       mass=0.70 if (FLOW and FULL) else 0.45 if FLOW else 0.35)
dof = list(art.dof_names)
wheel_idx = [k for k, n in enumerate(dof) if n.startswith("joint_wheel_")]
leg_idx = [k for k, n in enumerate(dof) if n.startswith("joint_leg_")]


def leg_state_mm():
    """다리 프리즘 관절 위치(mm) — 스트로크 한계 대비 어디에 있는지 진단용."""
    jp = art.get_joint_positions()
    return " ".join(f"{float(jp[k]) * 1000:+.2f}" for k in leg_idx)
dt = world.get_physics_dt()

print("\n" + "=" * 78)
print("실제 물 파티클 주행 데모")
print("=" * 78)
print(f"입자 {len(positions):,}개 (간격 {SPACING * 1000:.0f} mm, 물 질량 약 "
      f"{water_mass:.1f} kg 상당)")
_mode = ("FLOW+FULL(100% 만관+개방)" if FLOW and FULL
         else "FLOW(개방+재순환)" if FLOW
         else "FINE(6mm/dt 1/240/vmax 2)" if FINE else "기본(밀폐 9mm)")
if WINDOW > 0:
    _mode += f" + 슬라이딩윈도우 {WINDOW:.2f}m"
print(f"물 구간 x {WATER_X[0]}~{WATER_X[1]} m, GPU dynamics ON, 모드 {_mode}")
print(f"본체 실측 질량 {float(body.get_mass()) * 1000:.0f} g (목표 350g)")

if not HEADLESS:
    # 시작 카메라를 배관 내부(축 위)에 둔다 — 밖이면 관벽이 시야를 가린다
    set_camera_view(eye=[START_X + 0.40, 0.0, 0.008],
                    target=[START_X, 0.0, 0.0])

# ── 흐름 추적 입자 — 만관이라 수면이 없어 흐름이 안 보인다. 물속을 유속으로
# 떠가는 발광 구슬(시각 전용)로 방향·속도를 표시한다 ──
WIND_X = -0.4
_tracer_api = []
_tracer_pos = []
if not HEADLESS and WINDOW <= 0:   # 창 모드는 물이 이동하므로 고정 추적 구슬 제외
    tr_mtl = UsdShade.Material.Define(stage, "/World/Tracers/Mat")
    tr_sh = UsdShade.Shader.Define(stage, "/World/Tracers/Mat/Shader")
    tr_sh.CreateIdAttr("UsdPreviewSurface")
    tr_sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.9, 0.95, 1.0))
    tr_mtl.CreateSurfaceOutput().ConnectToSource(tr_sh.ConnectableAPI(), "surface")
    rng = np.random.default_rng(11)
    for i in range(12):
        sp = UsdGeom.Sphere.Define(stage, f"/World/Tracers/t{i}")
        sp.CreateRadiusAttr(0.003)
        sp.CreatePurposeAttr(UsdGeom.Tokens.render)
        UsdShade.MaterialBindingAPI.Apply(sp.GetPrim()).Bind(tr_mtl)
        r = rng.uniform(0.008, 0.038)
        th = rng.uniform(0.0, 2.0 * np.pi)
        pos = np.array([rng.uniform(WATER_X[0], WATER_X[1]),
                        r * np.cos(th), r * np.sin(th)])
        api = UsdGeom.XformCommonAPI(sp)
        api.SetTranslate(Gf.Vec3d(*pos))
        _tracer_api.append(api)
        _tracer_pos.append(pos)


def update_tracers():
    if HEADLESS:
        return
    for api, pos in zip(_tracer_api, _tracer_pos):
        pos[0] += WIND_X * dt
        if pos[0] < WATER_X[0]:
            pos[0] += WATER_X[1] - WATER_X[0]
        api.SetTranslate(Gf.Vec3d(*pos))

# ── 정착: 물이 로봇 주변으로 흘러들며 수면이 안정될 때까지 ──
x0 = float(body.get_world_pose()[0][0])
for _s in range(SETTLE_STEPS):
    update_tracers()
    update_window_caps()
    if FLOW and _s % RECYCLE_EVERY == 0:
        recycle_particles()
    world.step(render=not HEADLESS)
x_settle = float(body.get_world_pose()[0][0])
print(f"\n정착 완료: 로봇 X 이동 {(x_settle - x0) * 1000:+.1f} mm "
      f"(물이 밀려들며 생긴 변위)")
print(f"정착 후 다리 관절(mm): {leg_state_mm()}  "
      f"본체 높이 {float(body.get_world_pose()[0][2]) * 1000:+.1f} mm")

# ── 주행: 고인 물을 헤치고 전진 (드라이브는 reset 전에 이미 설정됨) ──
spin = 0.0
x_d0 = float(body.get_world_pose()[0][0])
print(f"\n{'스텝':>6} {'주행':>9} {'이론':>9} {'슬립률':>8} {'높이':>8}")
for step in range(1, DRIVE_STEPS + 1):
    update_tracers()
    update_window_caps()
    if FLOW and step % RECYCLE_EVERY == 0:
        recycle_particles()
    world.step(render=not HEADLESS)
    vel = art.get_joint_velocities()
    spin += sum(float(vel[k]) for k in wheel_idx) / len(wheel_idx) * dt
    p_now = body.get_world_pose()[0]
    if float(p_now[0]) > STOP_X:
        print(f"→ 수조 끝 캡 근접 — 정지 (스텝 {step})")
        break
    if step % REPORT_EVERY == 0:
        travel = (float(p_now[0]) - x_d0) * 1000
        ideal = WHEEL_R_M * spin * 1000
        slip = travel / ideal if abs(ideal) > 1e-6 else 0.0
        extra = ""
        if WINDOW > 0:
            wpts = np.array(water_instancer.GetPositionsAttr().Get())
            extra = (f" 물중심오프셋 "
                     f"{float(wpts[:, 0].mean() - float(p_now[0])) * 1000:+.0f}mm")
        print(f"{step:>6} {travel:>7.1f}mm {ideal:>7.1f}mm {slip:>8.3f} "
              f"{float(p_now[2]) * 1000:>+6.1f}mm{extra} "
              f"다리 {leg_state_mm()}")

p_end = body.get_world_pose()[0]
travel = (float(p_end[0]) - x_d0) * 1000
ideal = WHEEL_R_M * spin * 1000
slip = travel / ideal if abs(ideal) > 1e-6 else 0.0
print("=" * 78)
print(f"결과: 주행 {travel:.1f} mm / 이론 {ideal:.1f} mm → 슬립률 {slip:.3f}")

# 누수 점검 — 어디로 새는지 분해 (관벽 관통 vs 캡 통과 vs 섹션 이음매)
pts_now = water_instancer.GetPositionsAttr().Get()
if pts_now:
    arr = np.array([(p[0], p[1], p[2]) for p in pts_now])
    x_arr, z_arr = arr[:, 0], arr[:, 2]
    r_arr = np.sqrt(arr[:, 1] ** 2 + arr[:, 2] ** 2)
    out_wall = r_arr > 0.055                      # 관벽(내경 r=0.05) 관통
    if FLOW:
        out_cap = np.zeros_like(out_wall)         # 개방 구조 — 끝 이탈은 재순환 대상
    else:
        out_cap = ((x_arr < CAP_X[0] - 0.03)
                   | (x_arr > CAP_X[1] + 0.03)) & ~out_wall
    leaked = int(np.sum(out_wall | out_cap))
    print(f"누수 점검: 총 {leaked}개 / {len(pts_now):,}개 "
          f"(관벽 관통 {int(np.sum(out_wall))}, 캡 통과 {int(np.sum(out_cap))})")
    if WINDOW > 0:
        # 윈도우 캡을 빠져나가 뒤에 남겨진 입자 — 많으면 물이 로봇을 못 따라온 것
        x_robot = float(p_end[0])
        left_behind = int(np.sum((np.abs(x_arr - x_robot) > WINDOW * 0.5 + 0.03)
                                 & ~out_wall))
        print(f"윈도우 점검: 로봇 ±{WINDOW * 0.5 * 1000:.0f}mm(+30mm 여유) 밖 "
              f"잔류 {left_behind}개, 물중심 오프셋 "
              f"{float(x_arr[~out_wall].mean() - x_robot) * 1000:+.1f}mm")
    if FLOW:
        print(f"재순환 처리: 누적 {_recycled_total:,}개 — 0이면 위치 쓰기가 GPU에 "
              f"반영 안 된 것")
    if int(np.sum(out_wall)):
        lx = x_arr[out_wall]
        near_joint = int(np.sum((np.abs(lx - PIPE_SEC_LEN_M) < 0.04)
                                | (np.abs(lx - 2 * PIPE_SEC_LEN_M) < 0.04)))
        print(f"  관벽 관통 입자 중 섹션 이음매(x={PIPE_SEC_LEN_M:.2f}, "
              f"{2 * PIPE_SEC_LEN_M:.2f}) ±40mm 부근: {near_joint}개 — "
              f"많으면 이음매 틈, 아니면 메시 관통")
        print(f"  관통 입자 x 범위 {lx.min():.3f} ~ {lx.max():.3f} m")
else:
    print("누수 점검: USD 포인트 동기화 안 됨 — GUI 뷰포트로 확인")

if travel < 10.0:
    print("[FAIL] 물속에서 거의 전진 못함")
else:
    print(f"[OK] 실제 물 입자를 헤치고 {travel:.0f} mm 전진")

if not HEADLESS:
    print("\nGUI 실행 중 — 창을 닫으면 종료됩니다.")
    _k = 0
    while simulation_app.is_running():
        update_tracers()
        update_window_caps()
        _k += 1
        if FLOW and _k % RECYCLE_EVERY == 0:
            recycle_particles()
        world.step(render=True)

simulation_app.close()
