"""[Isaac 3.11] 수리기 — 정찰기 14링크 + 토치 링 모듈 3링크 = 17링크 articulation.

build_robot_parts.py 가 출력한 링크별 STL 을 읽어 물리 구조를 만든다.
  링크 17개 : body 2 + arm 6 + wheel 6 + ring 1 + rod 1 + tip 1
  조인트 15개: revolute 1 (관절) + revolute 6 (서스펜션) + revolute 6 (바퀴)
               + revolute 1 (J1 링 회전) + prismatic 1 (J2 토치 직동)

바퀴는 전진·후진만 되고 조향이 없어 로봇을 결함 방향으로 돌릴 수 없다.
대신 본체를 감싸는 링이 돌아 토치를 원주 방향 어디로든 겨눈다. 회전축을
관 중심축과 일치시켜 어느 방향으로 돌려도 토치·관벽 간극이 일정하다.

J1 은 ±180° 다. 360° 연속 회전은 슬립링이 필요하고 슬립링은 용접 전류를
흘리기에 부적합하다. ±180° 로 전 방향에 도달하며 케이블도 안 꼬인다.

STL 을 USD 로 변환하지 않고 UsdGeom.Mesh 로 직접 저작한다. 변환기 의존이 없고
정점을 읽을 때 m 단위로 바꾸므로 변환 체인에 scale op 이 아예 생기지 않는다.

실행:
  PYTHONUNBUFFERED=1 isaac_python robot_articulated.py            # GUI
  PYTHONUNBUFFERED=1 isaac_python robot_articulated.py --headless # 검증만
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
import time
from pathlib import Path

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade

HERE = Path(__file__).resolve().parent
SON = HERE.parent
OUT_USD = str(HERE / "welder_2seg.usd")

META = json.loads((SON / "spec" / "parts_meta.json").read_text())

_CATEGORY = {
    "body_rear": "robot", "body_front": "robot", "bellows": "robot",
    "arm": "robot", "wheel": "robot",
    "camera_housing": "camera",
    "pipe_straight": "pipe", "pipe_elbow_sr": "pipe",
    # 토치 3종을 빠뜨려 2026-08-04 실기에서 KeyError 로 즉시 죽었다.
    # STL 은 welder/meshes/ 에 정상적으로 있었고 이 표에만 등록이 없었다.
    # test_code/welder/test_part_paths.py 가 이제 누락을 잡는다.
    "torch_ring": "welder", "torch_rod": "welder", "torch_tip": "welder",
}


def part_path(name):
    """parts/ 는 robot / camera / pipe 로 나뉘어 있다."""
    return SON / _CATEGORY[name] / "meshes" / f"{name}.stl"


MM = 0.001

SEG_DX = META["seg_center_offset"] * MM
PIVOT_R = META["pivot_r"] * MM
ARM_LEN = META["arm_len"] * MM
ARM_PSI = META["arm_angle_nominal"]
ARM_DX = META["arm_dx_nominal"] * MM
ARM_DR = META["arm_dr_nominal"] * MM
WHEEL_R = META["wheel_r"] * MM
STAGGER = META["stagger"] * MM
FRONT_ARMS = META["front_arms"]
REAR_ARMS = META["rear_arms"]

ARM_LIMIT_LOWER = META["arm_angle_compressed"] - ARM_PSI
ARM_LIMIT_UPPER = META["arm_angle_extended"] - ARM_PSI

TORCH = META["torch"]
RING_X = TORCH["ring_x"] * MM
RING_R = TORCH["rod_origin_r"] * MM
ROD_LEN = TORCH["rod_len"] * MM
J1_LIMIT = TORCH["j1_limit_deg"]
J2_STROKE = TORCH["j2_stroke_mm"] * MM

MASS_RING_KG = 0.045
MASS_ROD_KG = 0.010
MASS_TIP_KG = 0.008
J1_STIFF_NM_RAD = 2.0
J2_STIFF_N_M = 400.0

WAIST_LIMIT = 55.0
WAIST_SPRING_NM_RAD = 0.01

MASS_BODY_KG = 0.190
MASS_ARM_KG = 0.012
MASS_WHEEL_KG = 0.008
MASS_TOTAL_KG = (2 * MASS_BODY_KG + 6 * MASS_ARM_KG + 6 * MASS_WHEEL_KG
                 + MASS_RING_KG + MASS_ROD_KG + MASS_TIP_KG)

WHEEL_PRELOAD_N = 9.0
ARM_MOMENT_M = ARM_LEN * math.cos(math.radians(ARM_PSI))
ARM_TORQUE_NM = WHEEL_PRELOAD_N * ARM_MOMENT_M

ARM_OVERTRAVEL_DEG = 6.0
ARM_DRIVE_TARGET_DEG = ARM_LIMIT_UPPER + ARM_OVERTRAVEL_DEG
ARM_K_NM_RAD = ARM_TORQUE_NM / math.radians(ARM_DRIVE_TARGET_DEG)

DEG = math.pi / 180.0
ARM_K_USD = ARM_K_NM_RAD * DEG
ARM_C_USD = ARM_K_USD / 10.0
ARM_MAX_FORCE = 1.5 * ARM_K_NM_RAD * math.radians(
    ARM_DRIVE_TARGET_DEG - ARM_LIMIT_LOWER)

WAIST_K_USD = WAIST_SPRING_NM_RAD * DEG
WAIST_C_USD = WAIST_K_USD / 10.0

# 설계 v3 §12.3 은 마찰을 **두 벌**로 나눈다. 하나로 뭉뚱그리면 안 된다.
#   physics_flooded.yaml  정찰기  static 0.30 / dynamic 0.25  (관에 물이 찬 상태)
#   physics_drained.yaml  수리기  static 0.40 / dynamic 0.35  (배수 후)
# 0.70 을 쓰고 있었다 — 문서에서 옮겨오지 않은 값이다.
WHEEL_FRICTION_STATIC = 0.40
WHEEL_FRICTION_DYNAMIC = 0.35
WHEEL_FRICTION = WHEEL_FRICTION_STATIC        # 견인력 계산은 정지마찰 기준

# 마찰이 낼 수 있는 최대 접선력에 대응하는 토크. **상한이지 목표가 아니다.**
WHEEL_FRICTION_LIMIT_NM = WHEEL_FRICTION * WHEEL_PRELOAD_N * WHEEL_R

# 🚨 구동 토크를 마찰 한계까지 쓰면 안 된다. 마찰은 원(circle)이라 구동이
# 전부 소비하면 **횡방향 안내력이 0** 이 되어 로봇이 관 중심을 못 잡고
# 중앙 관절이 스토퍼까지 접힌다(잭나이프). 팀 실측 주석:
#   "0.034(94%)로 걸면 마찰 원을 구동이 다 써서 횡방향 안내력이 0 —
#    주행 시작 2초 만에 중앙 관절이 ±55° 스토퍼로 잭나이프"
# 40~50% 를 남긴다.
WHEEL_TORQUE_FRACTION = 0.45
WHEEL_MAX_TORQUE = WHEEL_FRICTION_LIMIT_NM * WHEEL_TORQUE_FRACTION

# 🚨 USD 각도 드라이브의 damping 단위는 **N·m / (deg/s)** 다. 여기서 쓰려던
# 뜻은 "속도 오차 5 rad/s 에서 최대 토크" 였는데 변환을 빼먹어 57.3배 컸다.
# stiffness 쪽은 ARM_K_USD = K * DEG 로 제대로 변환하고 있어 둘이 어긋나 있었다.
WHEEL_DAMPING_REF_RAD_S = 5.0
WHEEL_DAMPING = WHEEL_MAX_TORQUE / WHEEL_DAMPING_REF_RAD_S * DEG
WHEEL_DAMPING = WHEEL_MAX_TORQUE / 5.0

CONTACT_OFFSET = 0.0005
REST_OFFSET = 0.0

ROBOT = "/World/Robot"


def load_stl(path):
    data = Path(path).read_bytes()
    n = struct.unpack("<I", data[80:84])[0]
    a = np.frombuffer(data[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    tri = a[:, 12:48].copy().view("<f4").reshape(n * 3, 3).astype(np.float64)
    pts, inv = np.unique(np.round(tri, 5), axis=0, return_inverse=True)
    return pts * MM, inv.reshape(n, 3)


def make_mesh(stage, path, stl_name):
    pts, idx = load_stl(part_path(stl_name))
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    mesh.CreateFaceVertexCountsAttr([3] * len(idx))
    mesh.CreateFaceVertexIndicesAttr(idx.reshape(-1).tolist())
    lo, hi = pts.min(0), pts.max(0)
    mesh.CreateExtentAttr([Gf.Vec3f(*lo), Gf.Vec3f(*hi)])
    mesh.CreateSubdivisionSchemeAttr("none")
    return mesh


def rx(deg):
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), deg))
    return m


def tr(x, y, z):
    m = Gf.Matrix4d(1.0)
    m.SetTranslate(Gf.Vec3d(x, y, z))
    return m


def quat_x(deg):
    return Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0, 0), deg).GetQuat())


# 🚨 물리 스텝을 기본값(1/60)으로 두면 안 된다. 설계 v3 §12.2 가
# "1/240 이상 (불안정 시 1/500)" 을 지정한다. 휠 반경 10mm 에
# contactOffset 0.0005 규모의 접촉이라 60Hz 로는 접촉이 풀리지 않는다.
# 2026-08-04 실측: dt 만 1/240 으로 바꿔 전진 0.0mm → 42.9mm.
PHYSICS_DT = 1.0 / 240.0
RENDER_DT = 1.0 / 60.0
world = World(stage_units_in_meters=1.0,
              physics_dt=PHYSICS_DT, rendering_dt=RENDER_DT)
stage = world.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage, "/World")
root = UsdGeom.Xform.Define(stage, ROBOT)

UsdGeom.Scope.Define(stage, f"{ROBOT}/PhysicsMaterials")
WHEEL_MAT = f"{ROBOT}/PhysicsMaterials/WheelMaterial"
_m = UsdPhysics.MaterialAPI.Apply(
    UsdShade.Material.Define(stage, WHEEL_MAT).GetPrim())
_m.CreateStaticFrictionAttr(WHEEL_FRICTION_STATIC)
_m.CreateDynamicFrictionAttr(WHEEL_FRICTION_DYNAMIC)
_m.CreateRestitutionAttr(0.0)


def make_link(path, matrix, stl_name, mass_kg, approximation, material=None,
              extra_visual=None):
    link = UsdGeom.Xform.Define(stage, path)
    link.AddTransformOp().Set(matrix)
    prim = link.GetPrim()

    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass_kg)

    mesh = make_mesh(stage, f"{path}/collision", stl_name)
    mp = mesh.GetPrim()
    UsdPhysics.CollisionAPI.Apply(mp)
    UsdPhysics.MeshCollisionAPI.Apply(mp).CreateApproximationAttr(approximation)
    px = PhysxSchema.PhysxCollisionAPI.Apply(mp)
    px.CreateContactOffsetAttr(CONTACT_OFFSET)
    px.CreateRestOffsetAttr(REST_OFFSET)
    if material is not None:
        UsdShade.MaterialBindingAPI.Apply(mp).Bind(
            UsdShade.Material.Get(stage, material),
            bindingStrength=UsdShade.Tokens.weakerThanDescendants,
            materialPurpose="physics")

    if extra_visual is not None:
        vis = make_mesh(stage, f"{path}/{extra_visual[0]}", extra_visual[0])
        vis.AddTransformOp().Set(extra_visual[1])
        UsdGeom.Imageable(vis).CreatePurposeAttr(UsdGeom.Tokens.default_)

    print(f"  link {path:26s} mass {mass_kg * 1000:6.1f} g  ({approximation})")
    return prim


def arm_slots():
    for tag, xc, angles in (("r", -SEG_DX, REAR_ARMS), ("f", +SEG_DX, FRONT_ARMS)):
        for i, phi in enumerate(angles):
            yield tag, xc, i, phi, (i - 1) * STAGGER


print("=" * 78)
print("링크 생성")
print("=" * 78)

make_link(f"{ROBOT}/body_rear", tr(-SEG_DX, 0, 0), "body_rear",
          MASS_BODY_KG, "convexHull",
          extra_visual=("bellows", tr(SEG_DX, 0, 0)))
make_link(f"{ROBOT}/body_front", tr(+SEG_DX, 0, 0), "body_front",
          MASS_BODY_KG, "convexHull")

for tag, xc, i, phi, xo in arm_slots():
    m_arm = tr(0, 0, PIVOT_R) * rx(-phi) * tr(xc + xo, 0, 0)
    make_link(f"{ROBOT}/arm_{tag}{i}", m_arm, "arm", MASS_ARM_KG, "convexHull")
    m_wh = tr(-ARM_DX, 0, PIVOT_R + ARM_DR) * rx(-phi) * tr(xc + xo, 0, 0)
    make_link(f"{ROBOT}/wheel_{tag}{i}", m_wh, "wheel", MASS_WHEEL_KG,
              "convexHull", material=WHEEL_MAT)


# ── 토치 링 모듈 (링 안지름 26 > 본체 최대 반경 25) ───────────────
make_link(f"{ROBOT}/torch_ring", tr(RING_X, 0, 0), "torch_ring",
          MASS_RING_KG, "convexDecomposition")
make_link(f"{ROBOT}/torch_rod", tr(RING_X, 0, RING_R), "torch_rod",
          MASS_ROD_KG, "convexHull")
make_link(f"{ROBOT}/torch_tip", tr(RING_X, 0, RING_R + ROD_LEN), "torch_tip",
          MASS_TIP_KG, "convexHull")


def set_bodies(j, p0, p1):
    j.CreateBody0Rel().SetTargets([p0])
    j.CreateBody1Rel().SetTargets([p1])


jw = UsdPhysics.RevoluteJoint.Define(stage, f"{ROBOT}/joint_waist")
set_bodies(jw, f"{ROBOT}/body_rear", f"{ROBOT}/body_front")
jw.CreateAxisAttr("Y")
jw.CreateLocalPos0Attr(Gf.Vec3d(+SEG_DX, 0, 0))
jw.CreateLocalRot0Attr(Gf.Quatf(1.0))
jw.CreateLocalPos1Attr(Gf.Vec3d(-SEG_DX, 0, 0))
jw.CreateLocalRot1Attr(Gf.Quatf(1.0))
jw.CreateLowerLimitAttr(-WAIST_LIMIT)
jw.CreateUpperLimitAttr(+WAIST_LIMIT)
dw = UsdPhysics.DriveAPI.Apply(jw.GetPrim(), "angular")
dw.CreateTypeAttr("force")
dw.CreateStiffnessAttr(WAIST_K_USD)
dw.CreateDampingAttr(WAIST_C_USD)
dw.CreateTargetPositionAttr(0.0)
dw.CreateMaxForceAttr(10.0 * WAIST_K_USD * WAIST_LIMIT)

for tag, xc, i, phi, xo in arm_slots():
    seg = "body_rear" if tag == "r" else "body_front"
    jp = f"{ROBOT}/joint_arm_{tag}{i}"
    j = UsdPhysics.RevoluteJoint.Define(stage, jp)
    set_bodies(j, f"{ROBOT}/{seg}", f"{ROBOT}/arm_{tag}{i}")
    j.CreateAxisAttr("Y")
    j.CreateLocalPos0Attr(Gf.Vec3d(xo,
                                   PIVOT_R * math.sin(math.radians(phi)),
                                   PIVOT_R * math.cos(math.radians(phi))))
    j.CreateLocalRot0Attr(quat_x(-phi))
    j.CreateLocalPos1Attr(Gf.Vec3d(0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1.0))
    j.CreateLowerLimitAttr(ARM_LIMIT_LOWER)
    j.CreateUpperLimitAttr(ARM_LIMIT_UPPER)
    d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(ARM_K_USD)
    d.CreateDampingAttr(ARM_C_USD)
    d.CreateTargetPositionAttr(ARM_DRIVE_TARGET_DEG)
    d.CreateMaxForceAttr(ARM_MAX_FORCE)

    jw2 = f"{ROBOT}/joint_wheel_{tag}{i}"
    j2 = UsdPhysics.RevoluteJoint.Define(stage, jw2)
    set_bodies(j2, f"{ROBOT}/arm_{tag}{i}", f"{ROBOT}/wheel_{tag}{i}")
    j2.CreateAxisAttr("Y")
    j2.CreateLocalPos0Attr(Gf.Vec3d(-ARM_DX, 0, ARM_DR))
    j2.CreateLocalRot0Attr(Gf.Quatf(1.0))
    j2.CreateLocalPos1Attr(Gf.Vec3d(0, 0, 0))
    j2.CreateLocalRot1Attr(Gf.Quatf(1.0))
    d2 = UsdPhysics.DriveAPI.Apply(j2.GetPrim(), "angular")
    d2.CreateTypeAttr("force")
    d2.CreateStiffnessAttr(0.0)
    d2.CreateDampingAttr(WHEEL_DAMPING)
    d2.CreateTargetVelocityAttr(0.0)
    d2.CreateMaxForceAttr(WHEEL_MAX_TORQUE)

# ── J1: 링 회전 (body_front -> torch_ring), 축 X = 관 중심축 ──────
j1 = UsdPhysics.RevoluteJoint.Define(stage, f"{ROBOT}/joint_torch_j1")
set_bodies(j1, f"{ROBOT}/body_front", f"{ROBOT}/torch_ring")
j1.CreateAxisAttr("X")
j1.CreateLocalPos0Attr(Gf.Vec3d(RING_X - SEG_DX, 0, 0))
j1.CreateLocalRot0Attr(Gf.Quatf(1.0))
j1.CreateLocalPos1Attr(Gf.Vec3d(0, 0, 0))
j1.CreateLocalRot1Attr(Gf.Quatf(1.0))
j1.CreateLowerLimitAttr(-J1_LIMIT)
j1.CreateUpperLimitAttr(+J1_LIMIT)
d1 = UsdPhysics.DriveAPI.Apply(j1.GetPrim(), "angular")
d1.CreateTypeAttr("force")
d1.CreateStiffnessAttr(J1_STIFF_NM_RAD * DEG)
d1.CreateDampingAttr(J1_STIFF_NM_RAD * DEG / 10.0)
d1.CreateTargetPositionAttr(0.0)
d1.CreateMaxForceAttr(20.0 * J1_STIFF_NM_RAD * DEG)

# ── J2: 토치 직동 (torch_ring -> torch_rod), 반경 방향 = 링 로컬 Z ──
j2 = UsdPhysics.PrismaticJoint.Define(stage, f"{ROBOT}/joint_torch_j2")
set_bodies(j2, f"{ROBOT}/torch_ring", f"{ROBOT}/torch_rod")
j2.CreateAxisAttr("Z")
j2.CreateLocalPos0Attr(Gf.Vec3d(0, 0, RING_R))
j2.CreateLocalRot0Attr(Gf.Quatf(1.0))
j2.CreateLocalPos1Attr(Gf.Vec3d(0, 0, 0))
j2.CreateLocalRot1Attr(Gf.Quatf(1.0))
j2.CreateLowerLimitAttr(0.0)
j2.CreateUpperLimitAttr(J2_STROKE)
d2 = UsdPhysics.DriveAPI.Apply(j2.GetPrim(), "linear")
d2.CreateTypeAttr("force")
d2.CreateStiffnessAttr(J2_STIFF_N_M)
d2.CreateDampingAttr(J2_STIFF_N_M / 10.0)
d2.CreateTargetPositionAttr(0.0)
d2.CreateMaxForceAttr(3.0 * J2_STIFF_N_M * J2_STROKE)

# 토치 끝은 로드에 고정. 자유도를 늘리지 않는다.
jf = UsdPhysics.FixedJoint.Define(stage, f"{ROBOT}/joint_torch_tip")
set_bodies(jf, f"{ROBOT}/torch_rod", f"{ROBOT}/torch_tip")
jf.CreateLocalPos0Attr(Gf.Vec3d(0, 0, ROD_LEN))
jf.CreateLocalRot0Attr(Gf.Quatf(1.0))
jf.CreateLocalPos1Attr(Gf.Vec3d(0, 0, 0))
jf.CreateLocalRot1Attr(Gf.Quatf(1.0))

UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
pxa = PhysxSchema.PhysxArticulationAPI.Apply(root.GetPrim())
pxa.CreateSolverPositionIterationCountAttr(64)
pxa.CreateSolverVelocityIterationCountAttr(4)
pxa.CreateEnabledSelfCollisionsAttr(False)

art = SingleArticulation(prim_path=ROBOT, name="pipe_welder_2seg")
world.scene.add(art)
world.reset()
for _ in range(60):
    world.step(render=not HEADLESS)

print("=" * 78)
print("articulation 검증")
print("=" * 78)
print(f"  총질량        {MASS_TOTAL_KG * 1000:.0f} g   (설계 500 g)")
print(f"  휠 접촉 반경  {(PIVOT_R + ARM_DR + WHEEL_R) * 1000:.2f} mm  (DN100 내반경 50.0)")
print(f"  암 한계       {ARM_LIMIT_LOWER:+.2f}° ~ {ARM_LIMIT_UPPER:+.2f}°  "
      f"(자유장 {ARM_DRIVE_TARGET_DEG:+.2f}°)")
print(f"  암 스프링     K = {ARM_K_NM_RAD:.4f} N·m/rad  "
      f"→ 기준자세 토크 {ARM_TORQUE_NM:.4f} N·m = 예압 {WHEEL_PRELOAD_N:.1f} N")
print(f"  관절          ±{WAIST_LIMIT:.0f}°  센터링 {WAIST_SPRING_NM_RAD} N·m/rad (수동)")
print(f"  바퀴 최대토크 {WHEEL_MAX_TORQUE * 1000:.1f} mN·m  (마찰 한계)")

dof = list(art.dof_names or [])
n_waist = sum(1 for n in dof if n.startswith("joint_waist"))
n_torch = sum(1 for n in dof if n.startswith("joint_torch"))
n_arm = sum(1 for n in dof if n.startswith("joint_arm_"))
n_wheel = sum(1 for n in dof if n.startswith("joint_wheel_"))
print(f"\n  링크 {art.num_bodies} (기대 17)   DOF {art.num_dof} (기대 15)")
print(f"    관절 {n_waist} / 서스펜션 {n_arm} / 바퀴 {n_wheel} / 토치 {n_torch}")
print(f"  토치  링 x {TORCH['ring_x']:.0f}mm, 안지름 {TORCH['ring_r_in']:.0f}"
      f" (본체 최대 반경 25)")
print(f"        J1 ±{J1_LIMIT:.0f}°  J2 0~{TORCH['j2_stroke_mm']:.0f}mm")
print(f"        수납 {TORCH['stow_radius_mm']:.0f}mm / "
      f"도달 {TORCH['reach_radius_mm']:.0f}mm (관벽 50)")

ok = art.num_bodies == 17 and art.num_dof == 15
if not ok:
    print("  [FAIL] 링크/DOF 개수 불일치 — 조인트 연결 확인")

pos = art.get_joint_positions()
if any(abs(float(v)) > 5.0 for v in pos):
    print("  [FAIL] 조인트 발산 — self-collision / 질량 / 게인 확인")
    ok = False

arm_idx = [k for k, n in enumerate(dof) if n.startswith("joint_arm_")]
arm_deg = [math.degrees(float(pos[k])) for k in arm_idx]
print(f"\n  자유공간 60스텝 후 암 각도: "
      f"{min(arm_deg):+.2f}° ~ {max(arm_deg):+.2f}°  (상한 {ARM_LIMIT_UPPER:+.2f}°)")
if min(arm_deg) > ARM_LIMIT_UPPER - 1.0:
    print("  [OK] 6개 전부 상한까지 신장 — 스프링 drive 동작 (관벽 없으므로 정상)")
else:
    print("  [WARN] 상한까지 벌어지지 않았다 — 예압/한계 확인")

SPIN = 180.0
widx = [k for k, n in enumerate(dof) if n.startswith("joint_wheel_")]
for k in widx:
    UsdPhysics.DriveAPI.Get(
        stage.GetPrimAtPath(f"{ROBOT}/{dof[k]}"), "angular"
    ).GetTargetVelocityAttr().Set(SPIN)
before = art.get_joint_positions()
for _ in range(60):
    world.step(render=not HEADLESS)
after = art.get_joint_positions()
spun = [abs(float(after[k] - before[k])) for k in widx]
print(f"\n  바퀴 구동 ({SPIN:.0f} deg/s, 60스텝): "
      f"최소 {min(spun):.4f} rad / 최대 {max(spun):.4f} rad")
if min(spun) < 0.05:
    print("  [FAIL] 돌지 않는 바퀴가 있다 — revolute 축/드라이브 확인")
    ok = False
else:
    print("  [OK] 6개 전부 회전")
for k in widx:
    UsdPhysics.DriveAPI.Get(
        stage.GetPrimAtPath(f"{ROBOT}/{dof[k]}"), "angular"
    ).GetTargetVelocityAttr().Set(0.0)

widx_name = [n for n in dof if n.startswith("joint_waist")]
if widx_name:
    k = dof.index(widx_name[0])
    print(f"\n  관절 각도 {math.degrees(float(after[k])):+.3f}°  "
          f"(수동이므로 자유공간에서 0 부근이어야 정상)")

print("=" * 78)

tj1 = [k for k, n in enumerate(dof) if n == "joint_torch_j1"]
tj2 = [k for k, n in enumerate(dof) if n == "joint_torch_j2"]
print("\n  토치 동작 시험")
if tj1 and tj2:
    for tgt in (90.0, -90.0, 0.0):
        UsdPhysics.DriveAPI.Get(
            stage.GetPrimAtPath(f"{ROBOT}/joint_torch_j1"), "angular"
        ).GetTargetPositionAttr().Set(tgt)
        for _ in range(120):
            world.step(render=not HEADLESS)
        got = math.degrees(float(art.get_joint_positions()[tj1[0]]))
        err = abs(got - tgt)
        print(f"    J1 목표 {tgt:+7.1f}° → 실제 {got:+7.1f}°  "
              f"오차 {err:5.2f}°  {'OK' if err < 5.0 else 'FAIL'}")
        if err >= 5.0:
            ok = False
    UsdPhysics.DriveAPI.Get(
        stage.GetPrimAtPath(f"{ROBOT}/joint_torch_j2"), "linear"
    ).GetTargetPositionAttr().Set(J2_STROKE)
    for _ in range(120):
        world.step(render=not HEADLESS)
    got = float(art.get_joint_positions()[tj2[0]])
    err = abs(got - J2_STROKE)
    print(f"    J2 목표 {J2_STROKE * 1000:5.1f}mm → 실제 {got * 1000:5.1f}mm  "
          f"오차 {err * 1000:4.2f}mm  {'OK' if err < 0.002 else 'FAIL'}")
    if err >= 0.002:
        ok = False
    UsdPhysics.DriveAPI.Get(
        stage.GetPrimAtPath(f"{ROBOT}/joint_torch_j2"), "linear"
    ).GetTargetPositionAttr().Set(0.0)
    for _ in range(60):
        world.step(render=not HEADLESS)
else:
    print("    [FAIL] 토치 조인트를 못 찾음")
    ok = False

if ok:
    world.stop()
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
    stage.Export(OUT_USD)
    print(f"\n저장 완료: {OUT_USD}\n")
else:
    print("\n검증 실패 — USD 저장하지 않음\n")

if not HEADLESS:
    print("GUI 실행 중 — 창을 닫으면 종료됩니다.")
    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.005)

simulation_app.close()
