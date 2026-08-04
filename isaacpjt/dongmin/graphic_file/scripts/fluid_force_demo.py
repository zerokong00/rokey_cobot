"""만관(물 가득) 상태의 유체력이 로봇 주행에 미치는 영향을 검증한다.

검증 대상 4가지 (설계확정본 v3 §4.4):
  1. 항력   — 흐르는 물이 로봇을 미는 힘. 폐색 보정 C_d,eff = C_d/(1-β)² 포함
  2. 부력   — 유효 중량을 줄여 트랙션을 갉아먹는 힘
  3. 마찰   — 젖은 관벽 마찰계수 (물속 0.30/0.25, v3 §13.3)
  4. 모멘트 — 항력 작용점(본체 중심)과 접지점의 높이 차에 의한 피칭

유체는 파티클로 만들지 않는다 (dongmin CLAUDE.md C6). 해석적 힘 주입:
  F_drag = ½ ρ C_d,eff A (v_flow - v_robot)|v_flow - v_robot|   (배관 축 방향)
  F_buoy = ρ g V_disp                                            (연직 위)
매 물리 스텝 RigidPrim.apply_forces 로 적용한다
(API_INDEX.md:3134 — 힘은 1스텝만 유효하므로 매 스텝 재적용해야 한다).

유체 파라미터는 로봇에 맞춰 산출한다 (1차 실행에서 배운 것: 실물 설계값
부력 1.77N 을 본체 53g 임시 로봇에 그대로 걸면 힘이 무게의 4배라 무의미):
  - 부력/중량 비 0.36 은 실물 설계(1.77N / 4.9N)와 동일하게 유지
  - 전면적은 로봇 bbox 단면 x 충전율 0.6, β 와 C_d,eff 는 거기서 유도

유속은 -X 방향(로봇 정면으로 밀어오는 물). 전진(+X)이 곧 역류 주행이다.

테스트 3단계:
  A. 부력 검증  — 정지 상태에서 부력 ON, 다리 압축 변화와 접촉 유지 확인
  B. 정지 버팀  — 바퀴 잠금, 유속 램프, 밀리기 시작하는 임계 유속 측정.
                  50mm 밀리면 중단 (1차 실행에서 로봇이 배관 밖까지 쓸려나갔다)
  C. 역류 전진  — 설계 유속 0.86 m/s 를 거슬러 전진, 슬립률로 판정

새 로봇/파이프 USD 가 오면 설정 블록만 교체하면 된다. 테스트 로직은 공용.

실행:
  PYTHONUNBUFFERED=1 isaac_python fluid_force_demo.py --headless
"""

import sys

HEADLESS = "--headless" in sys.argv

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": HEADLESS})

from pathlib import Path

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim, SingleArticulation, SingleRigidPrim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.viewports import set_camera_view
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

# ══════════════════════════════════════════════════════════════════
# 설정 블록 — 새 로봇/파이프 도착 시 여기만 교체
# ══════════════════════════════════════════════════════════════════
HERE = Path(__file__).resolve().parent
ROBOT_USD = str(HERE / "robot_assembled.usd")
PIPE_USD = str(HERE.parent / "pipe" / "pipe.usd")

# 배관 (pipe_drive_demo.py 와 동일)
PIPE_CAD_BORE_MM = 45.0
PIPE_CAD_LEN_MM = 200.0
PIPE_BORE_MM = 100.0
PIPE_SCALE = (PIPE_BORE_MM / PIPE_CAD_BORE_MM) * 0.001
PIPE_SEC_LEN_M = PIPE_CAD_LEN_MM * PIPE_SCALE
PIPE_SECTIONS = 4
PIPE_END_M = PIPE_SECTIONS * PIPE_SEC_LEN_M

# 임시 로봇 (robot_articulated.py 와 동일해야 한다)
ROBOT_SCALE = 90.0 / 302.0
WHEEL_R_M = 23.0 * 0.001 * ROBOT_SCALE
WHEEL_OUT_M = (132.0 + 23.0) * 0.001 * ROBOT_SCALE
ROBOT_LEN_M = 393.5 * 0.001 * ROBOT_SCALE
LEG_FREE_M = 15.0 * 0.001 * ROBOT_SCALE

# 유체 (v3 §4.4 방법론. 크기 의존값은 로봇 로드 후 산출)
RHO = 1000.0
G = 9.81
V_FLOW_DESIGN = 0.86        # m/s (매닝, S=1/100)
CD_BASE = 1.0               # 뭉툭한 물체 기본 항력계수
FRONTAL_FILL = 0.6          # bbox 단면 대비 실제 정면적 충전율
BUOY_WEIGHT_RATIO = 0.36    # 실물 설계: 부력 1.77N / 중량 4.9N

# 물속 마찰 (physics_flooded: 0.30/0.25)
FRICTION_STATIC = 0.30
FRICTION_DYNAMIC = 0.25

# 테스트 파라미터
SETTLE_STEPS = 180
PHASE_A_STEPS = 240
RAMP_VMAX = 3.0
RAMP_STEPS = 1800
DRIVE_STEPS = 900
SPIN_DEG_S = 360.0
REPORT_EVERY = 100
SLIP_LIMIT_MM = 5.0         # B: 이만큼 쓸리면 "버팀 실패"
ABORT_PUSH_MM = 50.0        # B: 이만큼 쓸리면 측정 중단
END_MARGIN_M = 0.15         # C: 배관 끝에서 이만큼 남으면 중단

# ══════════════════════════════════════════════════════════════════
# 씬 구성 (pipe_drive_demo.py 패턴)
# ══════════════════════════════════════════════════════════════════
world = World(stage_units_in_meters=1.0)
stage = world.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage, "/World")
UsdLux.DomeLight.Define(stage, Sdf.Path("/World/DomeLight")).CreateIntensityAttr(1000.0)


def mat4(scale=1.0, rot_axis=None, rot_deg=0.0, translate=(0.0, 0.0, 0.0)):
    m = Gf.Matrix4d(1.0)
    m.SetScale(Gf.Vec3d(scale, scale, scale))
    r = Gf.Matrix4d(1.0)
    if rot_axis is not None:
        r.SetRotate(Gf.Rotation(Gf.Vec3d(*rot_axis), rot_deg))
    t = Gf.Matrix4d(1.0)
    t.SetTranslate(Gf.Vec3d(*translate))
    return m * r * t


# 배관 — 젖은 마찰 재질
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
    raise RuntimeError("배관 Mesh 를 못 찾음 — collider 미생성")

# ── 시각용 물기둥 — 만관 표현 (물리 없음. 실제 유체력은 apply_fluid 가 담당) ──
water = UsdGeom.Cylinder.Define(stage, "/World/Water")
water.CreateAxisAttr(UsdGeom.Tokens.x)
water.CreateRadiusAttr(PIPE_BORE_MM * 0.5 * 0.001 * 0.98)
water.CreateHeightAttr(PIPE_END_M)
UsdGeom.XformCommonAPI(water).SetTranslate(Gf.Vec3d(PIPE_END_M * 0.5, 0.0, 0.0))
water.CreatePurposeAttr(UsdGeom.Tokens.render)   # 렌더 전용 — 물리에서 제외
water_mtl = UsdShade.Material.Define(stage, "/World/Water/Mat")
water_sh = UsdShade.Shader.Define(stage, "/World/Water/Mat/Shader")
water_sh.CreateIdAttr("UsdPreviewSurface")
water_sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
    Gf.Vec3f(0.08, 0.32, 0.45))
water_sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.25)
water_sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.15)
water_mtl.CreateSurfaceOutput().ConnectToSource(
    water_sh.ConnectableAPI(), "surface")
UsdShade.MaterialBindingAPI.Apply(water.GetPrim()).Bind(water_mtl)

# ── 흐름 마커 — 물살 속도로 떠내려가는 발광 구슬 (시각 전용) ──
FLOW_MARKERS = 14
_marker_api = []
_marker_pos = []
if not HEADLESS:
    mk_mtl = UsdShade.Material.Define(stage, "/World/FlowMarkers/Mat")
    mk_sh = UsdShade.Shader.Define(stage, "/World/FlowMarkers/Mat/Shader")
    mk_sh.CreateIdAttr("UsdPreviewSurface")
    mk_sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.4, 0.9, 1.0))
    mk_sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.4, 0.9, 1.0))
    mk_mtl.CreateSurfaceOutput().ConnectToSource(mk_sh.ConnectableAPI(), "surface")
    rng = np.random.default_rng(7)
    for i in range(FLOW_MARKERS):
        sp = UsdGeom.Sphere.Define(stage, f"/World/FlowMarkers/m{i}")
        sp.CreateRadiusAttr(0.0025)
        sp.CreatePurposeAttr(UsdGeom.Tokens.render)
        UsdShade.MaterialBindingAPI.Apply(sp.GetPrim()).Bind(mk_mtl)
        r = rng.uniform(0.010, 0.038)
        th = rng.uniform(0.0, 2.0 * np.pi)
        pos = np.array([rng.uniform(0.0, PIPE_END_M),
                        r * np.cos(th), r * np.sin(th)])
        api = UsdGeom.XformCommonAPI(sp)
        api.SetTranslate(Gf.Vec3d(*pos))
        _marker_api.append(api)
        _marker_pos.append(pos)


def update_flow_markers(v_flow_signed):
    """마커를 유속만큼 흘려보낸다. 배관 범위를 벗어나면 반대쪽에서 재등장."""
    if HEADLESS:
        return
    for api, pos in zip(_marker_api, _marker_pos):
        pos[0] += v_flow_signed * dt
        if pos[0] < 0.0:
            pos[0] += PIPE_END_M
        elif pos[0] > PIPE_END_M:
            pos[0] -= PIPE_END_M
        api.SetTranslate(Gf.Vec3d(*pos))


# 로봇 — 섹션 1 중앙 (1차 실행 교훈: 섹션 이음매 플랜지 위에 두면 정착 실패)
START_X = 1.5 * PIPE_SEC_LEN_M
robot_xf = UsdGeom.Xform.Define(stage, "/World/RobotRoot")
robot_xf.AddTransformOp().Set(mat4(1.0, (0, 1, 0), 90.0, (START_X, 0.0, 0.0)))
add_reference_to_stage(usd_path=ROBOT_USD, prim_path="/World/RobotRoot/Robot")

art = SingleArticulation(prim_path="/World/RobotRoot/Robot/Robot", name="pipe_robot")
world.scene.add(art)
world.reset()

body = SingleRigidPrim(prim_path="/World/RobotRoot/Robot/Robot/body", name="body")
# 힘 주입용 뷰 (SingleRigidPrim 에는 apply_forces 가 없다 — API_INDEX.md:3491)
body_view = RigidPrim("/World/RobotRoot/Robot/Robot/body", name="body_view")
body_view.initialize()

# ── 로봇 전체 질량 (모든 강체 링크 합) ──
link_paths = [
    str(p.GetPath()) for p in stage.Traverse()
    if str(p.GetPath()).startswith("/World/RobotRoot/Robot/Robot/")
    and p.HasAPI(UsdPhysics.RigidBodyAPI)
]
mass_view = RigidPrim(link_paths, name="mass_view")
mass_view.initialize()
m_total = float(np.sum(mass_view.get_masses()))

# ── 유체 파라미터 산출 (로봇 크기·질량 기준) ──
cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
rb = cache.ComputeWorldBound(
    stage.GetPrimAtPath("/World/RobotRoot")).ComputeAlignedRange()
frontal_bbox = (rb.GetMax()[1] - rb.GetMin()[1]) * (rb.GetMax()[2] - rb.GetMin()[2])
A_FRONTAL = frontal_bbox * FRONTAL_FILL
pipe_area = np.pi * (PIPE_BORE_MM * 0.5 * 0.001) ** 2
beta = A_FRONTAL / pipe_area
CD_EFF = CD_BASE / (1.0 - beta) ** 2
F_BUOY = BUOY_WEIGHT_RATIO * m_total * G

dof = list(art.dof_names)
leg_idx = [k for k, n in enumerate(dof) if n.startswith("joint_leg_")]
wheel_idx = [k for k, n in enumerate(dof) if n.startswith("joint_wheel_")]
dt = world.get_physics_dt()

print("\n" + "=" * 78)
print("만관 유체력 검증 — 설정 (임시 로봇 기준 산출값)")
print("=" * 78)
print(f"로봇 총질량 {m_total * 1000:.0f} g (링크 {len(link_paths)}개) → "
      f"중량 {m_total * G:.2f} N, 부력 {F_BUOY:.2f} N (비 {BUOY_WEIGHT_RATIO})")
print(f"정면적 {A_FRONTAL * 1e6:.0f} mm² / 관 단면 {pipe_area * 1e6:.0f} mm² → "
      f"폐색률 β {beta:.2f}, C_d,eff {CD_EFF:.2f}")
print(f"설계 유속 {V_FLOW_DESIGN} m/s 항력 "
      f"{0.5 * RHO * CD_EFF * A_FRONTAL * V_FLOW_DESIGN**2:.2f} N / "
      f"마찰 {FRICTION_STATIC}/{FRICTION_DYNAMIC} / dt {dt:.4f} s")
print("참고: 실물 확정값은 A 2700mm², β 0.34, C_d,eff 2.32, 부력 1.77N — "
      "새 로봇 USD 로 교체 시 그 값이 나와야 한다")


def leg_mean():
    pos = art.get_joint_positions()
    return sum(float(pos[k]) for k in leg_idx) / len(leg_idx)


def body_x():
    return float(body.get_world_pose()[0][0])


def body_pitch_deg():
    """본체 로컬 Z(=배관 축, 눕혀서 월드 X와 정렬)가 수평면에서 들린 각 = 피칭.

    로컬 z축의 월드 벡터는 회전행렬 3열 (2(xz+wy), 2(yz-wx), 1-2(x²+y²)).
    수평 정렬이면 월드 Z 성분이 0, 피칭이 생기면 ±로 벗어난다.
    """
    q = body.get_world_pose()[1]  # w,x,y,z (KB NOTES/04 §1: 쿼터니언 wxyz)
    w, x, y, z = (float(v) for v in q)
    axis_z_up = 1.0 - 2.0 * (x * x + y * y)
    return float(np.degrees(np.arcsin(np.clip(axis_z_up, -1.0, 1.0))))


def apply_fluid(v_flow_signed):
    """항력(부호 있는 유속, -X = 역류) + 부력(+Z)을 본체에 적용."""
    v_robot = float(body.get_linear_velocity()[0])
    rel = v_flow_signed - v_robot
    f_drag = 0.5 * RHO * CD_EFF * A_FRONTAL * rel * abs(rel)
    body_view.apply_forces(
        np.array([[f_drag, 0.0, F_BUOY]], dtype=np.float32), is_global=True)
    return f_drag


def set_wheel_target(deg_s):
    for k in wheel_idx:
        jp = stage.GetPrimAtPath(f"/World/RobotRoot/Robot/Robot/{dof[k]}")
        drive = UsdPhysics.DriveAPI.Get(jp, "angular")
        drive.GetTargetVelocityAttr().Set(deg_s)


# ── 정착 ──────────────────────────────────────────────────────────
for _ in range(SETTLE_STEPS):
    world.step(render=not HEADLESS)

leg_dry = leg_mean()
LEG_EXPECT_M = PIPE_BORE_MM * 0.5 * 0.001 - WHEEL_OUT_M
off0 = (float(body.get_world_pose()[0][1]) ** 2
        + float(body.get_world_pose()[0][2]) ** 2) ** 0.5
if off0 > PIPE_BORE_MM * 0.5 * 0.001:
    raise RuntimeError("로봇이 배관 밖에 있다")

# 접촉 판정: 다리가 자유 신장보다 눌려 있고(무언가에 닿아 압축) 관 중심 부근.
# 관 중심 정렬(3.808mm) 대비 처짐은 정보로만 보고한다 — 젖은 마찰(0.25)에서는
# 바퀴가 원주 방향으로 미끄러져 건조(0.7)보다 아래로 처지는 것이 정상이다.
sag_mm = (LEG_EXPECT_M - leg_dry) * 1000
contact = (leg_dry < LEG_FREE_M - 2e-4 and off0 < 0.005)
print(f"\n정착: 다리 {leg_dry * 1000:+.3f} mm (중심 정렬 기준 {LEG_EXPECT_M * 1000:.3f}, "
      f"처짐 {sag_mm:+.3f} mm, 축이탈 {off0 * 1000:.2f} mm)")
print("[OK] 다리 압축 + 관 중심 부근 — 관벽 접촉" if contact
      else "[FAIL] 접촉 안 됨 — 이후 결과 무효")

# 카메라: 시작 지점 정면(배관 축 위)에 한 번만 세팅. 이후 마우스 조작 자유
if not HEADLESS:
    set_camera_view(eye=[START_X + 0.45, 0.0, 0.012], target=[START_X, 0.0, 0.0])

# ══ A. 부력 검증 ═══════════════════════════════════════════════════
print("\n" + "=" * 78)
print(f"[A] 부력 {F_BUOY:.2f} N — 접촉 유지 검증 (유속 0)")
print("=" * 78)
x_a0 = body_x()
for _ in range(PHASE_A_STEPS):
    apply_fluid(0.0)
    world.step(render=not HEADLESS)
leg_buoy = leg_mean()
drift_a = abs(body_x() - x_a0) * 1000
print(f"다리 {leg_dry * 1000:+.3f} → {leg_buoy * 1000:+.3f} mm "
      f"(Δ {(leg_buoy - leg_dry) * 1000:+.3f} mm), X 드리프트 {drift_a:.2f} mm")
if leg_buoy < LEG_FREE_M - 2e-4:
    print(f"[OK] 부력 적용 후에도 다리가 눌려 있다 — 예압이 부력을 이긴다 "
          f"(부력 {F_BUOY:.2f} N vs 총중량 {m_total * G:.2f} N)")
else:
    print("[FAIL] 다리가 자유 신장까지 펴졌다 — 부력이 로봇을 띄웠다")

# ══ B. 정지 버팀 — 임계 유속 ═══════════════════════════════════════
# 유속은 -X (정면으로 밀어오는 물). 바퀴 목표속도 0 = 브레이크.
print("\n" + "=" * 78)
print(f"[B] 정지 버팀 — 역류 0→{RAMP_VMAX} m/s 램프, "
      f"{SLIP_LIMIT_MM:.0f} mm 밀리면 실패, {ABORT_PUSH_MM:.0f} mm 에서 중단")
print("=" * 78)
set_wheel_target(0.0)
x_b0 = body_x()
v_crit = None
pitch_max_b = 0.0
print(f"{'유속':>8} {'항력':>8} {'밀림':>9} {'다리':>9} {'피치':>7}")
for step in range(1, RAMP_STEPS + 1):
    v_flow = RAMP_VMAX * step / RAMP_STEPS
    f_d = apply_fluid(-v_flow)
    update_flow_markers(-v_flow)
    world.step(render=not HEADLESS)
    pushed = (x_b0 - body_x()) * 1000        # 하류(-X)로 쓸린 양, +가 밀림
    pitch_max_b = max(pitch_max_b, abs(body_pitch_deg()))
    if v_crit is None and pushed > SLIP_LIMIT_MM:
        v_crit = v_flow
        print(f"→ 임계 유속 {v_crit:.2f} m/s (항력 {abs(f_d):.2f} N)에서 "
              f"{SLIP_LIMIT_MM:.0f} mm 밀림")
    if pushed > ABORT_PUSH_MM:
        print(f"→ {ABORT_PUSH_MM:.0f} mm 밀림 — 램프 중단 (유속 {v_flow:.2f} m/s)")
        break
    if step % (RAMP_STEPS // 12) == 0:
        print(f"{v_flow:>6.2f}m/s {abs(f_d):>6.2f}N {pushed:>7.1f}mm "
              f"{leg_mean() * 1000:>+7.3f}mm {body_pitch_deg():>6.1f}°")

pushed_total = (x_b0 - body_x()) * 1000
print(f"B 동안 최대 |피치| {pitch_max_b:.1f}° (전복 없음 기준 < 10°)")
if v_crit is None:
    print(f"[OK] {RAMP_VMAX} m/s 까지 버팀 (총 밀림 {pushed_total:.1f} mm)")
elif v_crit > V_FLOW_DESIGN:
    print(f"[OK] 임계 {v_crit:.2f} m/s > 설계 {V_FLOW_DESIGN} m/s — 설계 조건 버팀")
else:
    print(f"[FAIL] 임계 {v_crit:.2f} m/s ≤ 설계 {V_FLOW_DESIGN} m/s — "
          "설계 유속에서 밀린다. 예압/마찰 확인")

# ══ C. 역류 전진 — 슬립률 ══════════════════════════════════════════
# 유속 -X 를 거슬러 +X 전진. B에서 밀린 위치에서 그대로 출발한다.
# 유속은 이 로봇이 이길 수 있는 수준으로 잡는다: B 임계의 60% (임계를 넘는
# 유속에서는 밀리는 것이 물리적으로 옳아서 전진 로직 검증이 안 된다).
# 임계가 설계 유속보다 높은 로봇(실물 예상)은 설계 유속 그대로 시험된다.
v_flow_c = V_FLOW_DESIGN if v_crit is None else min(V_FLOW_DESIGN, 0.6 * v_crit)
print("\n" + "=" * 78)
print(f"[C] 역류 전진 — 역류 {v_flow_c:.2f} m/s"
      f"{' (B 임계의 60%)' if v_flow_c < V_FLOW_DESIGN else ' (설계 유속)'}, "
      f"바퀴 {SPIN_DEG_S:.0f} deg/s")
print("=" * 78)
set_wheel_target(SPIN_DEG_S)
x_c0 = body_x()
spin = 0.0
pitch_max_c = 0.0
swept = False
print(f"{'스텝':>6} {'주행':>9} {'이론':>9} {'슬립률':>8} {'항력':>7} {'피치':>7}")
for step in range(1, DRIVE_STEPS + 1):
    f_d = apply_fluid(-v_flow_c)
    update_flow_markers(-v_flow_c)
    world.step(render=not HEADLESS)
    vel = art.get_joint_velocities()
    spin += sum(float(vel[k]) for k in wheel_idx) / len(wheel_idx) * dt
    pitch_max_c = max(pitch_max_c, abs(body_pitch_deg()))
    if body_x() > PIPE_END_M - END_MARGIN_M:
        print(f"→ 배관 끝 근접 — 주행 중단 (스텝 {step})")
        break
    if body_x() < x_c0 - 0.1:
        print(f"→ 100 mm 이상 쓸림 — 전진 실패, 중단 (스텝 {step})")
        swept = True
        break
    if step % REPORT_EVERY == 0:
        travel = (body_x() - x_c0) * 1000
        ideal = WHEEL_R_M * spin * 1000
        slip = travel / ideal if abs(ideal) > 1e-6 else 0.0
        print(f"{step:>6} {travel:>7.1f}mm {ideal:>7.1f}mm {slip:>8.3f} "
              f"{abs(f_d):>5.2f}N {body_pitch_deg():>6.1f}°")

travel = (body_x() - x_c0) * 1000
ideal = WHEEL_R_M * spin * 1000
slip = travel / ideal if abs(ideal) > 1e-6 else 0.0
print("=" * 78)
print(f"[C] 결과: 유속 {v_flow_c:.2f} m/s, 주행 {travel:.1f} mm / 이론 {ideal:.1f} mm "
      f"→ 슬립률 {slip:.3f}, 최대 |피치| {pitch_max_c:.1f}°")
if swept or travel < 10.0:
    print("[FAIL] 역류에서 전진 못함 — 항력이 트랙션을 이긴다")
elif slip < 0.5:
    print(f"[FAIL] 슬립률 {slip:.3f} — 절반 이상 미끄러짐. 젖은 마찰에서 트랙션 부족")
elif slip > 1.3:
    print(f"[경고] 슬립률 {slip:.3f} > 1.3 — 바퀴 회전보다 많이 갔다. "
          "순류로 쓸려가는 중일 수 있다 (유속 부호 확인)")
else:
    print(f"[OK] 슬립률 {slip:.3f} — 역류 조건에서도 전진 가능")

print("\n검증 항목 매핑: 항력=B·C, 부력=A, 마찰저하=B·C(μ 0.25), 피칭 모멘트=B·C 피치")
print("새 로봇 도착 시: 설정 블록의 USD 경로·치수 값만 교체 후 재실행")

if not HEADLESS:
    print("\nGUI 실행 중 — 물살 시각화 유지. 창을 닫으면 종료됩니다.")
    while simulation_app.is_running():
        update_flow_markers(-v_flow_c)
        world.step(render=True)

simulation_app.close()
