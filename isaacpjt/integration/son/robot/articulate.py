"""[Isaac 3.11] 배관 점검 로봇 — 2단 1관절 articulation 구성 및 robot_2seg.usd 저장.

build_robot_parts.py 가 출력한 링크별 STL 을 읽어 물리 구조를 만든다.
  링크 14개 : body 2 + arm 6 + wheel 6
  조인트 13개: revolute 1 (관절) + revolute 6 (서스펜션) + revolute 6 (바퀴)

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
OUT_USD = str(HERE / "robot_2seg.usd")

META = json.loads((SON / "spec" / "parts_meta.json").read_text())

_CATEGORY = {
    "body_rear": "robot", "body_front": "robot", "bellows": "robot",
    "arm": "robot", "wheel": "robot",
    "camera_housing": "camera",
    "pipe_straight": "pipe", "pipe_elbow_sr": "pipe",
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

WAIST_LIMIT = 55.0
WAIST_SPRING_NM_RAD = 0.01

MASS_BODY_KG = 0.190
MASS_ARM_KG = 0.012
MASS_WHEEL_KG = 0.008
MASS_TOTAL_KG = 2 * MASS_BODY_KG + 6 * MASS_ARM_KG + 6 * MASS_WHEEL_KG

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

WHEEL_FRICTION = 0.7
WHEEL_MAX_TORQUE = WHEEL_FRICTION * WHEEL_PRELOAD_N * WHEEL_R
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


world = World(stage_units_in_meters=1.0)
stage = world.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage, "/World")
root = UsdGeom.Xform.Define(stage, ROBOT)

UsdGeom.Scope.Define(stage, f"{ROBOT}/PhysicsMaterials")
WHEEL_MAT = f"{ROBOT}/PhysicsMaterials/WheelMaterial"
_m = UsdPhysics.MaterialAPI.Apply(
    UsdShade.Material.Define(stage, WHEEL_MAT).GetPrim())
_m.CreateStaticFrictionAttr(WHEEL_FRICTION)
_m.CreateDynamicFrictionAttr(WHEEL_FRICTION)
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

UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
pxa = PhysxSchema.PhysxArticulationAPI.Apply(root.GetPrim())
pxa.CreateSolverPositionIterationCountAttr(64)
pxa.CreateSolverVelocityIterationCountAttr(4)
pxa.CreateEnabledSelfCollisionsAttr(False)

art = SingleArticulation(prim_path=ROBOT, name="pipe_robot_2seg")
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
n_arm = sum(1 for n in dof if n.startswith("joint_arm_"))
n_wheel = sum(1 for n in dof if n.startswith("joint_wheel_"))
print(f"\n  링크 {art.num_bodies} (기대 14)   DOF {art.num_dof} (기대 13)")
print(f"    관절 {n_waist} / 서스펜션 {n_arm} / 바퀴 {n_wheel}")

ok = art.num_bodies == 14 and art.num_dof == 13
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
