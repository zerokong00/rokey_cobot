"""[Isaac 3.11] 배관 점검 로봇 — SR 곡관 통과 시험.

설계확정본이 본체를 2등분한 근거(일체형 150mm 는 SR 곡관에 ±16.9mm 스트로크가
필요해 통과 불가, 62mm 로 쪼개면 ±2.5mm)를 시뮬레이션으로 확인한다.

배관은 직관 600mm → SR 곡관 90도(R=100mm, XZ 평면) → 직관 600mm 이다.
곡관을 XZ 평면에 둔 이유는 관절 축이 Y(피치)이기 때문이다. XY 평면 곡관은
로봇이 롤로 자세를 맞춰야 통과하므로 기하 검증이 오염된다.

선행: robot_articulated.py 를 먼저 실행해 robot_2seg.usd 를 만들 것.

실행:
  PYTHONUNBUFFERED=1 isaac_python pipe_curve_demo.py
  PYTHONUNBUFFERED=1 isaac_python pipe_curve_demo.py --cameras
  PYTHONUNBUFFERED=1 isaac_python pipe_curve_demo.py --headless --steps 6000

--cameras 를 주면 robot_2seg_cam.usd 를 불러 주행하면서 RGB·Depth 도 함께
발행한다. camera/rig.py --save 를 먼저 실행해 둬야 한다.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pyver import require_isaac

# rclpy 는 --cameras 일 때만 import 한다(아래 CAMERAS 분기). 카메라 없이
# 주행만 볼 때까지 ROS 빌드를 요구할 이유가 없다 — 가드가 실제 import 조건과
# 어긋나 있어서 2026-08-04 실기에서 주행 시험 자체가 막혔다.
require_isaac(__file__, needs_rclpy="--cameras" in sys.argv)

HEADLESS = "--headless" in sys.argv
STEPS = 4000
if "--steps" in sys.argv:
    STEPS = int(sys.argv[sys.argv.index("--steps") + 1])

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
from pxr import Gf, PhysxSchema, UsdGeom, UsdLux, UsdPhysics, UsdShade

HERE = Path(__file__).resolve().parent
SON = HERE.parent
CAMERAS = "--cameras" in sys.argv

# 카메라 포함본이 있으면 그쪽을 우선한다. --cameras 를 주면 주행하면서
# RGB·Depth 도 함께 발행한다.
CAM_USD = SON / "robot" / "robot_2seg_cam.usd"
PLAIN_USD = SON / "robot" / "robot_2seg.usd"
ROBOT_USD = CAM_USD if CAM_USD.is_file() else PLAIN_USD

if not ROBOT_USD.is_file():
    print(f"[중단] {PLAIN_USD} 가 없다. robot_articulated.py 를 먼저 실행할 것.")
    simulation_app.close()
    sys.exit(1)
if CAMERAS and ROBOT_USD != CAM_USD:
    print(f"[중단] --cameras 를 주려면 {CAM_USD} 가 필요하다.")
    print("       camera/rig.py --save 를 먼저 실행할 것.")
    simulation_app.close()
    sys.exit(1)

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

PIPE_LEN = 600.0 * MM
ELBOW_R = META["elbow_r"] * MM
PIPE_FRICTION = 0.7
CONTACT_OFFSET = 0.0005
REST_OFFSET = 0.0

WHEEL_R = META["wheel_r"] * MM
TARGET_SPEED_MPS = 0.05
SPIN_DEG_S = math.degrees(TARGET_SPEED_MPS / WHEEL_R)
# 바퀴를 돌리기 전에 암이 벌어져 관벽을 물 시간을 준다.
SETTLE_STEPS = 60

START_X = -0.50
ROBOT = "/World/Robot"
PIPE = "/World/Pipe"


def load_stl(path):
    data = Path(path).read_bytes()
    n = struct.unpack("<I", data[80:84])[0]
    a = np.frombuffer(data[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    tri = a[:, 12:48].copy().view("<f4").reshape(n * 3, 3).astype(np.float64)
    pts, inv = np.unique(np.round(tri, 5), axis=0, return_inverse=True)
    return pts * MM, inv.reshape(n, 3)


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

light = UsdLux.SphereLight.Define(stage, "/World/Light")
light.CreateIntensityAttr(3e7)
light.CreateRadiusAttr(0.05)
UsdGeom.Xformable(light).AddTranslateOp().Set(Gf.Vec3d(0.0, -0.6, 0.6))

UsdGeom.Scope.Define(stage, "/World/PhysicsMaterials")
PIPE_MAT = "/World/PhysicsMaterials/PipeMaterial"
_m = UsdPhysics.MaterialAPI.Apply(
    UsdShade.Material.Define(stage, PIPE_MAT).GetPrim())
_m.CreateStaticFrictionAttr(PIPE_FRICTION)
_m.CreateDynamicFrictionAttr(PIPE_FRICTION)
_m.CreateRestitutionAttr(0.0)


def rot(deg, axis):
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(Gf.Vec3d(*axis), deg))
    return m


def trans(x, y, z):
    m = Gf.Matrix4d(1.0)
    m.SetTranslate(Gf.Vec3d(x, y, z))
    return m


def add_pipe(name, stl_name, matrix):
    pts, idx = load_stl(part_path(stl_name))
    xf = UsdGeom.Xform.Define(stage, f"{PIPE}/{name}")
    xf.AddTransformOp().Set(matrix)
    mesh = UsdGeom.Mesh.Define(stage, f"{PIPE}/{name}/mesh")
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    mesh.CreateFaceVertexCountsAttr([3] * len(idx))
    mesh.CreateFaceVertexIndicesAttr(idx.reshape(-1).tolist())
    mesh.CreateExtentAttr([Gf.Vec3f(*pts.min(0)), Gf.Vec3f(*pts.max(0))])
    mesh.CreateSubdivisionSchemeAttr("none")
    mp = mesh.GetPrim()
    UsdPhysics.CollisionAPI.Apply(mp)
    UsdPhysics.MeshCollisionAPI.Apply(mp).CreateApproximationAttr("none")
    px = PhysxSchema.PhysxCollisionAPI.Apply(mp)
    px.CreateContactOffsetAttr(CONTACT_OFFSET)
    px.CreateRestOffsetAttr(REST_OFFSET)
    UsdShade.MaterialBindingAPI.Apply(mp).Bind(
        UsdShade.Material.Get(stage, PIPE_MAT),
        bindingStrength=UsdShade.Tokens.weakerThanDescendants,
        materialPurpose="physics")
    return xf


UsdGeom.Xform.Define(stage, PIPE)
add_pipe("straight_in", "pipe_straight", trans(-PIPE_LEN / 2.0, 0, 0))
add_pipe("elbow", "pipe_elbow_sr", Gf.Matrix4d(1.0))
add_pipe("straight_out", "pipe_straight",
         rot(-90, (0, 1, 0)) * trans(ELBOW_R, 0, ELBOW_R + PIPE_LEN / 2.0))

robot_xf = UsdGeom.Xform.Define(stage, ROBOT)
robot_xf.AddTransformOp().Set(trans(START_X, 0, 0))
robot_xf.GetPrim().GetReferences().AddReference(str(ROBOT_USD), "/World/Robot")

art = SingleArticulation(prim_path=ROBOT, name="pipe_robot_2seg")
world.scene.add(art)
world.reset()

bridge = None
cam_rigs = []
if CAMERAS:
    sys.path.insert(0, str(SON / "camera"))
    import rig as camera_rig
    import rclpy
    cam_rigs = camera_rig.attach_existing(ROBOT)
    rclpy.init()
    bridge = camera_rig.CameraBridge(cam_rigs, camera_rig.topic_sets())
    for _ in range(20):
        world.step(render=not HEADLESS)
    print(f"카메라 {len(cam_rigs)}대 연결 — 주행하며 RGB·Depth 발행")

dof = list(art.dof_names or [])
arm_idx = [k for k, n in enumerate(dof) if n.startswith("joint_arm_")]
wheel_idx = [k for k, n in enumerate(dof) if n.startswith("joint_wheel_")]
waist_idx = [k for k, n in enumerate(dof) if n.startswith("joint_waist")]

# ── 초기 자세와 구동 지령 ────────────────────────────────────────
#
# 🚨 2026-08-04 실기에서 주행이 전혀 안 됐다. 원인이 두 겹이었다.
#
# (1) `set_joint_positions` 는 관절을 순간이동시키면서 **PhysX 런타임
#     드라이브 타깃을 지운다.** USD 속성값은 멀쩡히 남아 있어(19.402°)
#     파일을 읽는 것만으로는 안 보인다. 예전 코드는 바퀴만 다시 걸고
#     **암 타깃을 빠뜨렸다.** 그래서 암이 벌어지지 않아 관벽을 안 누르고,
#     수직항력 0 → 마찰 0 → 바퀴를 아무리 돌려도 안 나간다.
#     `maxForce` 를 100배 올려도 출력이 한 글자도 안 변한 이유가 이것이다.
#     견인력 예산 자체가 예압을 전제로 잡혀 있다:
#         WHEEL_MAX_TORQUE = WHEEL_FRICTION * WHEEL_PRELOAD_N * WHEEL_R
#
# (2) 시뮬레이션이 시작된 뒤의 **USD 드라이브 속성 쓰기는 PhysX 에 안 간다.**
#     런타임에는 `apply_action` 을 써야 한다.
#
# 그래서 여기서는 USD 속성을 건드리지 않고 전부 apply_action 으로 건다.
from isaacsim.core.utils.types import ArticulationAction   # noqa: E402

# 구동 부호. +TARGET_SPEED_MPS 를 주면 로봇이 -X 로 갔다(실측). 암 링크가
# rx(-phi) 로 배치되어 휠 회전축이 원주 접선의 음(-) 방향이기 때문이고,
# 6륜 전부 일관되므로 부호 하나로 맞춘다.
DRIVE_SIGN = -1.0
WHEEL_CMD_DEG_S = DRIVE_SIGN * SPIN_DEG_S


# 암 타깃은 상수를 다시 적지 않고 **USD 에 저장된 값을 읽는다.** articulate.py
# 가 유일한 출처이고, 지워지는 것은 PhysX 런타임 타깃이지 USD 속성이 아니다.
# 코어 API 는 라디안, USD 드라이브 속성은 도(degree) 다.
ARM_TARGET_DEG = [
    float(UsdPhysics.DriveAPI.Get(
        stage.GetPrimAtPath(f"{ROBOT}/{dof[k]}"), "angular"
    ).GetTargetPositionAttr().Get())
    for k in arm_idx]
ARM_TARGET_RAD = np.radians(ARM_TARGET_DEG)


def hold_arm_preload():
    """암 드라이브 타깃을 다시 건다. set_joint_positions 뒤에는 필수다."""
    art.apply_action(ArticulationAction(
        joint_positions=ARM_TARGET_RAD,
        joint_indices=np.array(arm_idx)))


def drive_wheels(deg_s):
    art.apply_action(ArticulationAction(
        joint_velocities=np.array([math.radians(deg_s)] * len(wheel_idx)),
        joint_indices=np.array(wheel_idx)))


start = art.get_joint_positions()
for k in arm_idx:
    start[k] = 0.0
art.set_joint_positions(start)
hold_arm_preload()          # ← 이 줄이 없어서 주행이 죽어 있었다

# 암이 벌어져 관벽을 물 때까지 기다린 다음 바퀴를 돌린다. 접지 전에 돌리면
# 헛돌면서 자세가 흐트러진다.
for _ in range(SETTLE_STEPS):
    world.step(render=not HEADLESS)
    hold_arm_preload()

arms_now = [math.degrees(float(art.get_joint_positions()[k])) for k in arm_idx]
tgt = max(ARM_TARGET_DEG)
print(f"  예압 안착  암 {min(arms_now):+.2f} ~ {max(arms_now):+.2f}°  "
      f"(목표 {tgt:+.2f}°)")
if max(arms_now) < tgt * 0.5:
    print("  [경고] 암이 안 벌어졌다 — 예압이 없으면 절대 전진하지 못한다")

drive_wheels(WHEEL_CMD_DEG_S)

print("=" * 78)
print("SR 곡관 통과 시험")
print("=" * 78)
print(f"  배관   직관 {PIPE_LEN * 1000:.0f}mm → 곡관 R{ELBOW_R * 1000:.0f}mm 90° "
      f"→ 직관 {PIPE_LEN * 1000:.0f}mm  (XZ 평면)")
print(f"  목표속도 {TARGET_SPEED_MPS * 1000:.0f} mm/s  (바퀴 {SPIN_DEG_S:.0f} deg/s)")
print(f"  시작 x = {START_X * 1000:.0f} mm,  스텝 {STEPS}")
print(f"  DOF {art.num_dof} / 링크 {art.num_bodies}")
print("-" * 78)
print(f"{'step':>6} {'x(mm)':>9} {'z(mm)':>9} {'호길이(mm)':>11} "
      f"{'관절(°)':>9} {'암 min~max(°)':>18}")
print("-" * 78)

LOG_EVERY = max(1, STEPS // 25)
entered = passed = False
max_waist = 0.0
arm_span = [+1e9, -1e9]
traj = []

CAM_EVERY = max(1, int(60.0 / max(camera_rig.HZ, 1.0))) if CAMERAS else 0

for step in range(STEPS):
    world.step(render=not HEADLESS)
    if bridge is not None and step % CAM_EVERY == 0:
        for rig in cam_rigs:
            bridge.publish(rig)
    if step % LOG_EVERY and step != STEPS - 1:
        continue
    p, _ = art.get_world_pose()
    x, z = float(p[0]), float(p[2])
    pos = art.get_joint_positions()
    waist = math.degrees(float(pos[waist_idx[0]])) if waist_idx else 0.0
    arms = [math.degrees(float(pos[k])) for k in arm_idx]
    arm_span = [min(arm_span[0], min(arms)), max(arm_span[1], max(arms))]
    max_waist = max(max_waist, abs(waist))
    if x > 0.0:
        entered = True
    if z > ELBOW_R:
        passed = True
    s = x if x <= 0 else (ELBOW_R * math.atan2(z, ELBOW_R - z + 1e-9)
                          if z < ELBOW_R else ELBOW_R * math.pi / 2 + (z - ELBOW_R))
    traj.append((step, x, z))
    print(f"{step:6d} {x * 1000:9.1f} {z * 1000:9.1f} {s * 1000:11.1f} "
          f"{waist:9.2f} {min(arms):8.2f}~{max(arms):7.2f}")

print("-" * 78)
p, _ = art.get_world_pose()
print(f"  최종 위치      x={float(p[0]) * 1000:.1f} mm  z={float(p[2]) * 1000:.1f} mm")
print(f"  최대 관절각    {max_waist:.2f}°  (한계 55°)")
print(f"  암 각도 범위   {arm_span[0]:+.2f}° ~ {arm_span[1]:+.2f}°")
print(f"  곡관 진입      {'예' if entered else '아니오'}")
print(f"  곡관 통과      {'예' if passed else '아니오'}")
if passed:
    print("\n  [OK] 2단 1관절 형상이 SR 곡관을 통과했다.")
elif entered:
    print("\n  [FAIL] 곡관 안에서 정지 — 스트로크/관절한계/마찰 확인")
else:
    print("\n  [FAIL] 직관에서 전진하지 못했다 — 예압/마찰/구동토크 확인")
print("=" * 78)

if bridge is not None:
    print(f"카메라 발행 {bridge.n} 프레임")

if not HEADLESS:
    print("GUI 실행 중 — 창을 닫으면 종료됩니다.")
    while simulation_app.is_running():
        world.step(render=True)
        if bridge is not None:
            for rig in cam_rigs:
                bridge.publish(rig)
        time.sleep(0.005)

if bridge is not None:
    import rclpy
    bridge.destroy_node()
    rclpy.shutdown()
simulation_app.close()
