"""배관 점검 로봇 — articulation(조인트) 구성 및 robot_assembled.usd 저장.

assemble_robot.py 가 형상 배치만 했다면 이 스크립트는 물리 구조를 만든다.
  링크 13개 : body 1 + leg 3 + wheel 9
  조인트 12개: prismatic 3 (leg 반경방향) + revolute 9 (바퀴)

구조상 주의 두 가지 (isaacsim_kb/NOTES/04 §5):
  - Joint 의 Body0 는 articulation 트리의 **부모**여야 한다.
  - USD 물리는 **rigid body 중첩을 허용하지 않는다.** assemble_robot.py 에서는
    바퀴가 leg 의 자식이었지만, 여기서는 전부 /World/Robot 바로 아래 형제로 평탄화하고
    상대 위치는 조인트의 local frame 으로 표현한다.

스케일은 루트가 아니라 **메시 리프**에만 건다. rigid body 변환 체인에 scale 이 끼면
PhysX 쪽에서 문제가 생기기 쉬우므로, 링크 변환은 전부 m 단위 실좌표로 둔다.

실행:
  PYTHONUNBUFFERED=1 isaac_python robot_articulated.py            # GUI
  PYTHONUNBUFFERED=1 isaac_python robot_articulated.py --headless # 검증만
"""

import sys

HEADLESS = "--headless" in sys.argv

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": HEADLESS})

from pathlib import Path
import time

from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

HERE = Path(__file__).resolve().parent
BODY_USD = str(HERE / "body.usd")
LEG_USD = str(HERE / "leg.usd")
WHEEL_USD = str(HERE / "wheel.usd")
OUT_USD = str(HERE / "robot_assembled.usd")

# ══ STL 실측값 (mm) — 추측 아님, 정점 파싱으로 확인 ═══════════════
LEG_ANGLES = [0.0, 120.0, 240.0]   # body 보스 3개 실측 각도
WHEEL_X_MM = 132.0                 # leg 바퀴 슬롯 중심 X
WHEEL_Z_MM = [-88.0, 0.0, 88.0]    # leg 바퀴 슬롯 중심 Z
WHEEL_R_MM = 23.0

# leg 로드(r=8, x=57~102) 가 body 보스 소켓(보어 r=8.3, x=53~87) 에 물리는 구조.
# 조인트 원점은 현재 CAD 자세의 물림 구간 중앙 x=(57+87)/2=72 로 둔다.
JOINT_X_MM = 72.0

# ══ 스트로크 — 소켓/로드 치수에서 유도 (CAD 원치수 mm) ═════════════
# 현재 자세 물림량 = 87 - 57 = 30 mm.
#   안쪽(-X): 로드 끝 57 이 보어 바닥 53 에 닿을 때까지 = 4 mm
#   바깥(+X): 물림량 15 mm 를 남기는 선까지 = 15 mm
LEG_LIMIT_LOWER_MM = -4.0
LEG_LIMIT_UPPER_MM = 15.0

# ══ 목표 크기 — 아직 확정 전이므로 여기 두 줄만 고치면 전체가 따라간다 ══
# CAD 원치수 기준 로봇 외경:
#   최대 수축 302 mm / 조인트 0 에서 310 mm / 최대 신장 340 mm
# "로봇 크기"를 **최대 수축 상태의 외경**으로 잡는다. 그래야 관에 여유를 두고 들어간 뒤
# 스프링이 다리를 벌려 관벽을 물 수 있다.
ROBOT_MIN_DIA_MM = 90.0            # 최대 수축 시 외경 (= 카탈로그상 "로봇 크기")
CAD_MIN_DIA_MM = 2.0 * (WHEEL_X_MM + WHEEL_R_MM + LEG_LIMIT_LOWER_MM)   # 302
ROBOT_SCALE = ROBOT_MIN_DIA_MM / CAD_MIN_DIA_MM

MM = 0.001 * ROBOT_SCALE           # CAD mm -> 월드 m (스케일 포함)

LEG_LIMIT_LOWER_M = LEG_LIMIT_LOWER_MM * MM
LEG_LIMIT_UPPER_M = LEG_LIMIT_UPPER_MM * MM
WHEEL_R_M = WHEEL_R_MM * MM


def dia_at_joint_m(x_m):
    """조인트 위치(m) -> 바퀴 외곽 지름(mm). 이 값이 곧 대응 가능한 관 내경."""
    return 2.0 * ((WHEEL_X_MM + WHEEL_R_MM) * MM + x_m) * 1000.0


# ══ 아래는 STL 에서 나오지 않는 값 — 전부 가정치. 실측/사양 확보 시 교체 ══
# 질량은 CAD 원치수(310mm) 기준 부피 개산값을 스케일 세제곱으로 줄인다.
_S3 = ROBOT_SCALE ** 3
MASS_BODY_KG = 2.00 * _S3
MASS_LEG_KG = 0.35 * _S3
MASS_WHEEL_KG = 0.05 * _S3
MASS_TOTAL_KG = MASS_BODY_KG + 3 * MASS_LEG_KG + 9 * MASS_WHEEL_KG

# leg 스프링. drive 가 곧 스프링-댐퍼다:  F = K*(target - x) - C*v
#
# 스프링 상수를 직접 주는 대신 **최대 신장에서 다리 하나가 관벽을 미는 힘(예압)**을
# 설계 입력으로 두고 K 를 역산한다. 스케일이 바뀌어도 접지 성능이 유지된다.
#   필요 조건: 3개 다리의 마찰이 자중을 이겨야 한다
#   3 * mu * 예압 > m*g   ->  mu=0.7, m*g 는 아래 출력에 찍힌다
# 1.5 로 두었더니 테이퍼 배관(pipe_levelup)에서 로봇이 진입하지 못했다:
#   바닥 주행 추진력(마찰 한계) = 0.7 * 자중 0.9N = 0.63N
#   테이퍼 진입 저항 = 3다리 * 스프링힘 * 기울기 0.1 = 0.5N + 등판 0.09N = 스톨
# 예압을 자중보다 낮추면 진입 저항이 추진력의 1/3 이 되고,
# 좁은 구간 파지 마찰(0.7*3*1.2N = 2.6N)은 여전히 자중의 3배라 충분하다.
LEG_PRELOAD_N = 0.6
LEG_DRIVE_OVERTRAVEL_M = 0.010 * ROBOT_SCALE   # 자유장을 상한보다 이만큼 바깥에
LEG_DRIVE_TARGET_M = LEG_LIMIT_UPPER_M + LEG_DRIVE_OVERTRAVEL_M
LEG_DRIVE_STIFFNESS = LEG_PRELOAD_N / LEG_DRIVE_OVERTRAVEL_M
LEG_DRIVE_DAMPING = LEG_DRIVE_STIFFNESS / 10.0   # KB NOTES/03 §6: 한 자릿수 아래

# 최대 압축까지 스프링이 선형이려면 그때 필요한 힘보다 커야 한다(포화 방지)
LEG_DRIVE_MAX_FORCE = 1.5 * LEG_DRIVE_STIFFNESS * (
    LEG_DRIVE_TARGET_M - LEG_LIMIT_LOWER_M)

# 바퀴 velocity drive. KB NOTES/03 §6: stiffness=0, damping 만.
# 최대 토크는 마찰 한계 근처로 잡는다. 너무 크면 미끄러지기만 하고 나아가지 않는다.
#   바퀴 1개 수직력 ~= 예압/3, 마찰계수 0.7 -> 견인력 한계 = 0.7*N, 토크 = 견인력*r
WHEEL_DRIVE_MAX_FORCE = 3.0 * (0.7 * LEG_PRELOAD_N / 3.0) * WHEEL_R_M
WHEEL_DRIVE_DAMPING = WHEEL_DRIVE_MAX_FORCE / 5.0   # 5 rad/s 오차에서 최대토크

# 마찰. 기본 물성(0.5)에 맡기면 "바퀴가 구르는가" 가 임의값에 좌우된다.
WHEEL_FRICTION = 0.7
PIPE_FRICTION = 0.7

ROBOT = "/World/Robot"

# ── World 를 먼저 만들어 physics scene 을 확보한 뒤 그 스테이지 위에 로봇을 세운다
world = World(stage_units_in_meters=1.0)
stage = world.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

UsdGeom.Xform.Define(stage, "/World")
root = UsdGeom.Xform.Define(stage, ROBOT)


def rz(deg):
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), deg))
    return m


def tr(x, y, z):
    m = Gf.Matrix4d(1.0)
    m.SetTranslate(Gf.Vec3d(x, y, z))
    return m


# 바퀴용 마찰 물성. 기본값에 맡기면 주행 성능이 임의값에 좌우된다.
WHEEL_MAT_PATH = f"{ROBOT}/PhysicsMaterials/WheelMaterial"
UsdGeom.Scope.Define(stage, f"{ROBOT}/PhysicsMaterials")
_wheel_mat = UsdPhysics.MaterialAPI.Apply(
    UsdShade.Material.Define(stage, WHEEL_MAT_PATH).GetPrim())
_wheel_mat.CreateStaticFrictionAttr(WHEEL_FRICTION)
_wheel_mat.CreateDynamicFrictionAttr(WHEEL_FRICTION)
_wheel_mat.CreateRestitutionAttr(0.0)


def make_link(path, matrix, usd_path, mass_kg, approximation, material=None):
    """링크 1개 = rigid body prim + 그 아래 스케일된 메시 자식."""
    link = UsdGeom.Xform.Define(stage, path)
    # 단일 transform op 로 준다. translate/rotate op 조합은 적용 순서 해석이 갈리므로
    # 4x4 를 직접 넣어 애매함을 없앤다.
    link.AddTransformOp().Set(matrix)
    prim = link.GetPrim()

    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass_kg)

    # reference 는 변환을 얹지 않은 별도 자식에. mm→m 스케일도 여기서만.
    vis = UsdGeom.Xform.Define(stage, path + "/visual")
    vis.AddScaleOp().Set(Gf.Vec3f(MM, MM, MM))
    vis.GetPrim().GetReferences().AddReference(usd_path)

    # 참조로 들어온 실제 Mesh gprim 들에 collider 를 건다.
    # 주의: 순회 중에 스키마를 Apply 하면 스테이지가 바뀌어 이터레이터가 프림을
    # 건너뛸 수 있다. 반드시 먼저 다 모은 뒤에 적용한다.
    meshes = [p for p in Usd.PrimRange(prim) if p.IsA(UsdGeom.Mesh)]
    if not meshes:
        raise RuntimeError(f"{path}: 참조 안에서 Mesh 를 못 찾음 — collider 미생성")
    for p in meshes:
        UsdPhysics.CollisionAPI.Apply(p)
        UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr(approximation)
        if material is not None:
            UsdShade.MaterialBindingAPI.Apply(p).Bind(
                UsdShade.Material.Get(stage, material),
                bindingStrength=UsdShade.Tokens.weakerThanDescendants,
                materialPurpose="physics",
            )
    print(f"  collider: {path:24s} mesh {len(meshes)}개 ({approximation})")
    return prim


def set_joint_bodies(joint, path0, path1):
    joint.CreateBody0Rel().SetTargets([path0])   # Body0 = 트리의 부모
    joint.CreateBody1Rel().SetTargets([path1])


# ══ 링크 생성 ═════════════════════════════════════════════════════
make_link(f"{ROBOT}/body", Gf.Matrix4d(1.0), BODY_USD, MASS_BODY_KG,
          "convexDecomposition")

for i, angle in enumerate(LEG_ANGLES):
    make_link(f"{ROBOT}/leg_{i}", rz(angle), LEG_USD, MASS_LEG_KG,
              "convexDecomposition")
    for j, z_mm in enumerate(WHEEL_Z_MM):
        # 바퀴 월드 변환 = T(132,0,z) * Rz(angle)  (행벡터 규약: p*T*R)
        m = tr(WHEEL_X_MM * MM, 0.0, z_mm * MM) * rz(angle)
        make_link(f"{ROBOT}/wheel_{i}_{j}", m, WHEEL_USD, MASS_WHEEL_KG,
                  "convexHull", material=WHEEL_MAT_PATH)

# ══ 조인트 ════════════════════════════════════════════════════════
# leg: body -> leg_i, 반경방향 prismatic (leg 로컬 X)
for i, angle in enumerate(LEG_ANGLES):
    jp = f"{ROBOT}/joint_leg_{i}"
    j = UsdPhysics.PrismaticJoint.Define(stage, jp)
    set_joint_bodies(j, f"{ROBOT}/body", f"{ROBOT}/leg_{i}")
    j.CreateAxisAttr("X")

    anchor = Gf.Vec3d(JOINT_X_MM * MM, 0.0, 0.0)
    # body 는 변환이 항등이므로 body 로컬 = 어셈블리 좌표. 앵커를 leg 방향으로 회전시킨다.
    j.CreateLocalPos0Attr(rz(angle).Transform(anchor))
    j.CreateLocalRot0Attr(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 0, 1), angle).GetQuat()))
    # leg 프레임은 이미 Rz(angle) 이 적용돼 있으므로 회전 없이 같은 점
    j.CreateLocalPos1Attr(anchor)
    j.CreateLocalRot1Attr(Gf.Quatf(1.0))

    j.CreateLowerLimitAttr(LEG_LIMIT_LOWER_M)
    j.CreateUpperLimitAttr(LEG_LIMIT_UPPER_M)

    d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(LEG_DRIVE_STIFFNESS)
    d.CreateDampingAttr(LEG_DRIVE_DAMPING)
    d.CreateTargetPositionAttr(LEG_DRIVE_TARGET_M)
    d.CreateMaxForceAttr(LEG_DRIVE_MAX_FORCE)

# wheel: leg_i -> wheel_i_j, 회전축 Y (leg 로컬). 한계 없음(연속 회전).
for i, angle in enumerate(LEG_ANGLES):
    for j_idx, z_mm in enumerate(WHEEL_Z_MM):
        jp = f"{ROBOT}/joint_wheel_{i}_{j_idx}"
        j = UsdPhysics.RevoluteJoint.Define(stage, jp)
        set_joint_bodies(j, f"{ROBOT}/leg_{i}", f"{ROBOT}/wheel_{i}_{j_idx}")
        j.CreateAxisAttr("Y")

        # leg 프레임에서 본 슬롯 위치. 바퀴 프레임에서는 자기 원점.
        j.CreateLocalPos0Attr(Gf.Vec3d(WHEEL_X_MM * MM, 0.0, z_mm * MM))
        j.CreateLocalRot0Attr(Gf.Quatf(1.0))
        j.CreateLocalPos1Attr(Gf.Vec3d(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0))

        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(0.0)                     # velocity drive
        d.CreateDampingAttr(WHEEL_DRIVE_DAMPING)
        d.CreateTargetVelocityAttr(0.0)                # deg/s
        d.CreateMaxForceAttr(WHEEL_DRIVE_MAX_FORCE)

# ══ articulation root ═════════════════════════════════════════════
UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
px = PhysxSchema.PhysxArticulationAPI.Apply(root.GetPrim())
px.CreateSolverPositionIterationCountAttr(64)
px.CreateSolverVelocityIterationCountAttr(4)
# leg 로드는 body 소켓 안에, 바퀴는 leg 슬롯 안에 들어가 있어 형상이 겹친다.
# self-collision 을 켜면 서로 밀어내며 발산한다.
px.CreateEnabledSelfCollisionsAttr(False)

# ══ 검증 ══════════════════════════════════════════════════════════
art = SingleArticulation(prim_path=ROBOT, name="pipe_robot")
world.scene.add(art)
world.reset()

for _ in range(60):
    world.step(render=not HEADLESS)

print("\n" + "=" * 78)
print("articulation 검증")
print("=" * 78)
print(f"스케일 {ROBOT_SCALE:.5f}  (CAD {CAD_MIN_DIA_MM:.0f}mm -> {ROBOT_MIN_DIA_MM:.0f}mm)")
print(f"  대응 관 내경 {dia_at_joint_m(LEG_LIMIT_LOWER_M):.1f} ~ "
      f"{dia_at_joint_m(LEG_LIMIT_UPPER_M):.1f} mm")
print(f"  총질량 {MASS_TOTAL_KG * 1000:.1f} g  (자중 {MASS_TOTAL_KG * 9.81:.3f} N)")
print(f"  바퀴 반경 {WHEEL_R_M * 1000:.2f} mm")
print(f"  스프링 K {LEG_DRIVE_STIFFNESS:.1f} N/m, 예압 {LEG_PRELOAD_N:.2f} N/다리")
print(f"  마찰 여유: 3*mu*예압 = {3 * 0.7 * LEG_PRELOAD_N:.2f} N "
      f"vs 자중 {MASS_TOTAL_KG * 9.81:.3f} N")
print(f"  바퀴 최대토크 {WHEEL_DRIVE_MAX_FORCE * 1000:.3f} mN·m")

dof_names = list(art.dof_names or [])
n_pris = sum(1 for n in dof_names if n.startswith("joint_leg_"))
n_rev = sum(1 for n in dof_names if n.startswith("joint_wheel_"))
print(f"DOF 개수 : {art.num_dof}   (기대 12 = prismatic 3 + revolute 9)")
print(f"  prismatic {n_pris} / revolute {n_rev}")
print(f"링크 개수 : {art.num_bodies}   (기대 13 = body 1 + leg 3 + wheel 9)")
print(f"DOF 이름 : {dof_names}")

pos = art.get_joint_positions()
print("\n60 스텝 후 조인트 위치:")
for name, v in zip(dof_names, pos):
    unit = "m" if name.startswith("joint_leg_") else "rad"
    print(f"  {name:22s} {v: .5f} {unit}")

leg_pos = [v for n, v in zip(dof_names, pos) if n.startswith("joint_leg_")]
ok = True

# 바퀴가 실제로 도는지 — DOF 존재만으로는 revolute 가 살아있는지 알 수 없다.
SPIN_DEG_S = 180.0
wheel_idx = [k for k, n in enumerate(dof_names) if n.startswith("joint_wheel_")]
for k in wheel_idx:
    UsdPhysics.DriveAPI.Get(
        stage.GetPrimAtPath(f"{ROBOT}/{dof_names[k]}"), "angular"
    ).GetTargetVelocityAttr().Set(SPIN_DEG_S)

before = art.get_joint_positions()
for _ in range(60):
    world.step(render=not HEADLESS)
after = art.get_joint_positions()

spun = [abs(after[k] - before[k]) for k in wheel_idx]
print(f"\n바퀴 구동 테스트 (target {SPIN_DEG_S:.0f} deg/s, 60 스텝):")
print(f"  회전량 최소 {min(spun):.4f} rad / 최대 {max(spun):.4f} rad")
if min(spun) < 0.05:
    print("  [FAIL] 돌지 않는 바퀴가 있다 — revolute 축/드라이브 확인")
    ok = False
else:
    print("  [OK] 9개 전부 회전")

# 원복 (저장되는 USD 는 정지 상태여야 한다)
for k in wheel_idx:
    UsdPhysics.DriveAPI.Get(
        stage.GetPrimAtPath(f"{ROBOT}/{dof_names[k]}"), "angular"
    ).GetTargetVelocityAttr().Set(0.0)

if art.num_dof != 12 or art.num_bodies != 13:
    print("\n[FAIL] DOF/링크 개수가 기대와 다름 — 조인트 연결을 확인할 것")
    ok = False
if any(abs(v) > 1.0 for v in pos):
    print("\n[FAIL] 조인트 값이 발산했다 (|v|>1). self-collision / 질량 / 게인 확인")
    ok = False
if leg_pos and all(v > LEG_LIMIT_UPPER_M - 1e-4 for v in leg_pos):
    print(f"\n[OK] leg 3개 모두 스트로크 상한({LEG_LIMIT_UPPER_M} m)까지 신장 — "
          "스프링 drive 동작 확인 (관벽 없는 자유 공간이므로 정상)")

# ── 스프링 실측 ───────────────────────────────────────────────────
# get_measured_joint_efforts() 는 이 구성에서 0 만 돌려준다(조인트가 limit 에 붙어 있어
# drive 힘이 limit 구속으로 받아지는 탓으로 보인다). 그래서 힘을 직접 읽는 대신
# **알려진 하중을 걸어 평형 위치를 재는 방식**으로 K 와 자유장을 역산한다.
#   평형: K*(target - x) + E = 0   ->   x = target + E/K
# 서로 다른 E 두 점이면 K 와 target 이 모두 나온다.
leg_idx = [k for k, n in enumerate(dof_names) if n.startswith("joint_leg_")]
phys = world.get_physics_context()
# get_gravity() -> (방향벡터, 크기),  set_gravity(float) 은 부호로 방향을 준다
g_dir, g_mag = phys.get_gravity()
saved_gravity = g_mag * (-1.0 if g_dir[2] < 0 else 1.0)
phys.set_gravity(0.0)   # 다리에 걸리는 중력이 측정을 오염시키므로 제거

# 평형이 스트로크 안에 들어오도록 하중을 스프링에서 역산한다 (스케일 바뀌어도 유효)
#   x = target + E/K  이므로  E = K*(x - target)
PROBE_N = [
    LEG_DRIVE_STIFFNESS * (
        LEG_LIMIT_UPPER_M + f * (LEG_LIMIT_LOWER_M - LEG_LIMIT_UPPER_M)
        - LEG_DRIVE_TARGET_M)
    for f in (0.2, 0.4, 0.6, 0.8)
]
PROBE_SETTLE = 400                        # 시정수 C/K=0.1s 대비 충분히 길게
PROBE_AVERAGE = 120                       # 정착 후 평균낼 스텝 수 (노이즈 억제)
measured = []
for load in PROBE_N:
    eff = [0.0] * len(dof_names)
    for k in leg_idx:
        eff[k] = load
    art.set_joint_efforts(eff)
    for _ in range(PROBE_SETTLE):
        world.step(render=False)
    # 순간값을 쓰면 솔버 노이즈가 그대로 실린다. 정착 후 여러 스텝을 평균한다.
    acc = []
    for _ in range(PROBE_AVERAGE):
        world.step(render=False)
        p = art.get_joint_positions()
        acc.append(sum(float(p[k]) for k in leg_idx) / len(leg_idx))
    measured.append(sum(acc) / len(acc))

art.set_joint_efforts([0.0] * len(dof_names))
phys.set_gravity(saved_gravity)

# x = target + E/K  를 최소제곱으로 적합 (E 를 x 의 1차식으로)
n = len(PROBE_N)
mx = sum(measured) / n
me = sum(PROBE_N) / n
sxx = sum((x - mx) ** 2 for x in measured)
sxe = sum((x - mx) * (e - me) for x, e in zip(measured, PROBE_N))
# 평형 K*(target - x) + E = 0  ->  E = K*x - K*target  이므로 E-vs-x 기울기 = +K
k_meas = sxe / sxx
target_meas = mx - me / k_meas
preload_meas = k_meas * (target_meas - LEG_LIMIT_UPPER_M)

print("\nleg 스프링 실측 (하중 → 평형위치, 최소제곱 적합):")
resid = []
for load, x in zip(PROBE_N, measured):
    fit = target_meas + load / k_meas
    resid.append(abs(x - fit))
    print(f"  하중 {load:+7.1f} N  →  평형 x = {x: .5f} m "
          f"(설정값 기준 이론 {LEG_DRIVE_TARGET_M + load / LEG_DRIVE_STIFFNESS: .5f}, "
          f"적합 잔차 {(x - fit) * 1000:+.3f} mm)")
print(f"  선형성: 최대 잔차 {max(resid) * 1000:.3f} mm")
print(f"  스프링 상수 K   = {k_meas:8.1f} N/m   (설정 {LEG_DRIVE_STIFFNESS:.0f})")
print(f"  자유장 target   = {target_meas: .5f} m  (설정 {LEG_DRIVE_TARGET_M:.5f})")
print(f"  최대 신장 예압  = {preload_meas:8.1f} N   "
      f"(= K x 상한 초과분 {LEG_DRIVE_OVERTRAVEL_M:.3f} m)")

# 이번 수정의 핵심은 자유장을 스트로크 상한 바깥으로 뺀 것이므로 거기에 기준을 건다.
# K 는 PhysX drive 가 설정값보다 5% 안팎 높게 나오는 것이 관측됐다(원인 미규명).
# 값 자체는 어차피 실접촉 데이터로 튜닝할 대상이라 게이트는 느슨하게 둔다(C8).
# 기준은 스트로크 비율로 둔다(스케일 무관). 값은 관측 데이터에서 정했다:
# 스케일 1.0 에서 잔차 0.000mm, 스케일 0.298 에서 0.086mm(=스트로크의 1.5%).
# 평균화를 해도 줄지 않아 노이즈가 아니라 계통 편차이며, 실측 K 가 두 스케일 모두
# 설정값 대비 +5.4% 로 나오는 것과 같은 뿌리로 보인다. 원인은 규명하지 못했다.
# 게이트는 관측치의 2배인 3% 로 둔다 — 힘 포화 같은 진짜 비선형(>10%)은 여전히 잡힌다.
_stroke = LEG_LIMIT_UPPER_M - LEG_LIMIT_LOWER_M
if max(resid) > 0.03 * _stroke:
    print("  [FAIL] 선형성 이탈 — 스프링이 아닌 다른 힘이 섞여 있다")
    ok = False
elif abs(target_meas - LEG_DRIVE_TARGET_M) > 0.10 * _stroke:
    print("  [FAIL] 자유장이 설정과 다르다 — target 반영 실패")
    ok = False
elif preload_meas < 0.9 * LEG_DRIVE_STIFFNESS * LEG_DRIVE_OVERTRAVEL_M:
    print("  [FAIL] 최대 신장에서 예압이 남지 않는다 — target/limit 확인")
    ok = False
else:
    print("  [OK] 자유장이 스트로크 상한 바깥에 있고 전 구간 선형. "
          f"최대 관경({dia_at_joint_m(LEG_LIMIT_UPPER_M):.0f}mm)에서도 예압 유지")
    if abs(k_meas - LEG_DRIVE_STIFFNESS) > 0.02 * LEG_DRIVE_STIFFNESS:
        print(f"  [참고] 실측 K 가 설정값보다 "
              f"{(k_meas / LEG_DRIVE_STIFFNESS - 1) * 100:+.1f}% — "
              "PhysX drive 특성. 원인 미규명이며 게인 튜닝 시 실측값 기준으로 볼 것")
print("=" * 78)

if ok:
    world.stop()
    # defaultPrim 이 없으면 다른 스테이지에서 이 USD 를 reference 할 때 경로가 안 풀린다
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
    stage.Export(OUT_USD)
    print(f"\n저장 완료: {OUT_USD}")
    print("  ※ reference 가 절대경로다. 폴더를 옮기면 이 스크립트를 다시 실행할 것.\n")

if not HEADLESS:
    print("GUI 실행 중 — 창을 닫으면 종료됩니다.")
    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.005)

simulation_app.close()
