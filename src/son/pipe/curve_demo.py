"""[Isaac 3.11] 배관 점검 로봇 — SR 곡관 통과 시험.

설계확정본이 본체를 2등분한 근거(일체형 150mm 는 SR 곡관에 ±16.9mm 스트로크가
필요해 통과 불가, 62mm 로 쪼개면 ±2.5mm)를 시뮬레이션으로 확인한다.

배관은 직관 600mm → SR 곡관 90도(R=100mm, XZ 평면) → 직관 600mm 이다.
곡관을 XZ 평면에 둔 이유는 관절 축이 Y(피치)이기 때문이다. XY 평면 곡관은
로봇이 롤로 자세를 맞춰야 통과하므로 기하 검증이 오염된다.

선행 준비 없음 — 로봇은 실행 중에 robot/assemble.py 로 씬에 직접 조립한다.

실행:
  PYTHONUNBUFFERED=1 isaac_python pipe_curve_demo.py
  PYTHONUNBUFFERED=1 isaac_python pipe_curve_demo.py --cameras
  PYTHONUNBUFFERED=1 isaac_python pipe_curve_demo.py --headless --steps 6000

--cameras 를 주면 조립 직후 카메라를 만들어(camera/rig.py build_cameras)
주행하면서 RGB·Depth 도 함께 발행한다. 사전 자산 생성 단계가 없다.
"""

import os
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

# 로봇은 **씬에 직접 조립한다.** USD 파일로 굽고 다시 불러오는 구조에서
# 관 안 주행이 0mm 였다(robot/assemble.py 머리말에 대조 실측 기록).
# 파일이 없으므로 세대 불일치·낡은 USD·덮어쓰기 우회가 전부 성립하지 않는다.
sys.path.insert(0, str(SON / "robot"))
import assemble   # noqa: E402
from pyver import hard_exit   # noqa: E402

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


# ⚠ 설계 v3 §12.2 는 "물리 스텝 1/240 이상 (불안정 시 1/500)" 을 지정하고
# "이 설정 누락이 가장 빈번한 실패 원인" 이라고 못박는다. 지금 여기는
# **기본값 1/60 이라 설계와 다르다.** 의도된 차이이며 근거는 이렇다.
#
#   1/240 을 넣으면 연산량이 4배가 된다. 그 근거였던 실측(전진 0.0mm →
#   42.9mm)은 **크라운 누락 · 마찰 0.70 · 예압 소실 상태의 빌드**에서 나온
#   값이라, 설계대로 고친 지금은 1/60 으로도 될 수 있다. 확인 전에는
#   연산량을 올리지 않는다.
#
# 🔬 확인 방법 — 아래를 1/240 로 바꿔 한 번 돌려 비교할 것.
#      1/60 에서도 주행한다  → 이대로 둔다
#      1/60 에서 0mm 다      → 1/240 으로 복귀 (설계값이기도 하다)
#
# 참고로 1/60 에서는 한 스텝에 0.833mm 전진하는데, 이는 접촉 감지폭
# contactOffset 0.5mm 의 1.67배다. 즉 감지 구간을 뛰어넘을 수 있다.
PHYSICS_DT = 1.0 / 60.0
RENDER_DT = 1.0 / 60.0

# 위 주석의 "🔬 확인 방법" 을 소스를 고치지 않고 돌리기 위한 진단 노브다.
# 기본값은 그대로 1/60 이고, env 로 준 경우에만 바뀌며 그 사실을 찍는다.
#   PHYSICS_HZ=240 isaac_python pipe/curve_demo.py --headless --steps 300
if os.environ.get("PHYSICS_HZ"):
    PHYSICS_DT = 1.0 / float(os.environ["PHYSICS_HZ"])
    print(f"[진단] PHYSICS_DT = 1/{os.environ['PHYSICS_HZ']} "
          f"(기본 1/60) — 연산량 비교용. 기본값은 바뀌지 않았다.")
world = World(stage_units_in_meters=1.0,
              physics_dt=PHYSICS_DT, rendering_dt=RENDER_DT)
stage = world.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

# 진단 노브: 곡관이 XZ 평면이라 이 코스는 **수직 상승**이다. 수직 상승에는
# 로봇 무게(4.91N)를 계속 들 견인력이 필요한데 가용은 7.26N — 여유 1.48배뿐이라
# 곡관에서 바퀴 2개만 떠도 못 올라간다. 중력을 꺼 보면 상승 부담과 기하 문제가 갈린다.
if os.environ.get("GRAVITY_OFF"):
    world.get_physics_context().set_gravity(0.0)
    print("[진단] 중력 OFF — 수직 상승 부담을 제거한 대조 실행")
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

# 시작 위치는 **각 링크 변환에 반영**된다. articulation root 프림에 변환 op 을
# 걸면 PhysX 가 어긋난다 — 그래서 origin 을 넘기고 root 는 건드리지 않는다.
assemble.build(stage, ROBOT, origin=(START_X, 0.0, 0.0))

# 카메라는 **조립 직후, world.reset() 전에** 만든다. reset 뒤에 만들면 센서가
# 갱신되지 않는다(IMU 와 같은 함정). 예전에는 rig.py --save 가 구운
# robot_2seg_cam.usd 를 불러 썼는데 그 파일 왕복을 없앴다.
camera_rig = None
if CAMERAS:
    sys.path.insert(0, str(SON / "camera"))
    import rig as camera_rig                              # noqa: E402
    camera_rig.build_cameras(stage, ROBOT)

art = SingleArticulation(prim_path=ROBOT, name="pipe_robot_2seg")
world.scene.add(art)
world.reset()

bridge = None
cam_rigs = []
if CAMERAS:
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

# 조립 결과가 기대와 맞는지 확인한다. 직접 조립이라 세대 불일치는 없지만,
# parts_meta 를 고쳤을 때 조립이 따라갔는지는 여전히 봐야 한다.
if (len(arm_idx), len(wheel_idx), len(waist_idx)) != (
        assemble.N_ARM, assemble.N_ARM, 1):
    print(f"[중단] 조립 결과가 기대와 다르다 — "
          f"암 {len(arm_idx)}/{assemble.N_ARM} "
          f"바퀴 {len(wheel_idx)}/{assemble.N_ARM} "
          f"중앙관절 {len(waist_idx)}/1")
    hard_exit(simulation_app)

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


# 위치는 **실제 강체 링크**에서 읽는다. `/World/Robot` 은 이 스크립트가 만든
# 컨테이너 Xform 이고 물리가 여기에 되쓰지 않는다 — art.get_world_pose() 만
# 보면 로봇이 굴러가도 값이 한 자리도 안 변할 수 있다(실제로 그렇게 보였다).
_BODY_PRIM = stage.GetPrimAtPath(f"{ROBOT}/body_front")
_XC = UsdGeom.XformCache()


def body_xz():
    _XC.Clear()
    t = _XC.GetLocalToWorldTransform(_BODY_PRIM).ExtractTranslation()
    return float(t[0]), float(t[2])


# 🚨 관절 위치는 ±180° 로 **감긴다**. 6개를 그대로 평균내면 값이 무의미해진다 —
# 실측에서 슬립률이 0.94 ~ 42.7 로 널뛰고 "바퀴 회전 -171.6°" 같은 거짓 수치가
# 나왔다(실제로는 수백 도 돌고 있었다). 스텝마다 증분을 ±180° 로 되감아 누적한다.
#
# ⚠ 그래서 `wheel_track()` 은 **매 스텝** 불러야 한다. 로그 간격(수십~수백 스텝)
#   마다 부르면 그 사이 회전이 180° 를 넘어 되감기가 틀린다
#   (286 deg/s · dt 1/60 이면 한 스텝 4.8°, 160스텝이면 763°).
_wheel_prev = None
_wheel_cum = None


def wheel_track():
    """바퀴 누적 회전각(도)을 갱신하고 돌려준다. 매 스텝 호출할 것."""
    global _wheel_prev, _wheel_cum
    cur = np.array([float(art.get_joint_positions()[k]) for k in wheel_idx])
    if _wheel_prev is None:
        _wheel_prev = cur
        _wheel_cum = np.zeros_like(cur)
        return 0.0
    d = cur - _wheel_prev
    d = (d + np.pi) % (2.0 * np.pi) - np.pi      # ±π 로 되감기
    _wheel_cum += d
    _wheel_prev = cur
    return float(np.degrees(_wheel_cum.mean()))


def wheel_spread_deg():
    """바퀴별 누적 회전의 편차(도). 크면 일부 바퀴가 헛돌거나 접지를 잃은 것이다."""
    if _wheel_cum is None:
        return 0.0
    return float(np.degrees(_wheel_cum.max() - _wheel_cum.min()))


def wheel_vel_deg_s():
    """바퀴 평균 각속도(deg/s). **지령이 PhysX 에 닿았는지**를 재는 값이다.

    각도만 보면 '안 돈다' 까지만 알 수 있고, 지령 자체가 안 간 것인지 걸려서
    못 도는 것인지 구분이 안 된다. 지령값 근처면 지령은 도달한 것이고,
    0 이면 도달하지 않았거나 즉시 제동되고 있다는 뜻이다.
    """
    vel = art.get_joint_velocities()
    if vel is None:
        return float("nan")
    return float(np.degrees(np.mean([float(vel[k]) for k in wheel_idx])))


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

# 예압은 **각도가 아니라 토크로** 판정한다. 관 안에서는 관벽이 암을 막아 각도가
# 0° 근처에 머무는 것이 정상이고, 목표각(관벽 바깥)에 도달하는 일은 원리적으로
# 없다. 각도로 재면 관 안에서 항상 "예압 없음" 오탐이 나서 진단을 망친다.
#   복원 토크 = stiffness × (목표각 − 실제각),  설계 예압 = maxForce
# 둘 다 articulate.py 가 USD 에 써 둔 값이라 여기서 상수를 다시 적지 않는다.
_arm_drive = [UsdPhysics.DriveAPI.Get(
    stage.GetPrimAtPath(f"{ROBOT}/{dof[k]}"), "angular") for k in arm_idx]
ARM_K_DEG = [float(d.GetStiffnessAttr().Get()) for d in _arm_drive]
ARM_MAXF = [float(d.GetMaxForceAttr().Get()) for d in _arm_drive]
arm_torque = [min(k_deg * (t - a), mf)          # 드라이브는 maxForce 로 잘린다
              for k_deg, t, a, mf
              in zip(ARM_K_DEG, ARM_TARGET_DEG, arms_now, ARM_MAXF)]
# 기준은 maxForce 가 아니라 **설계 예압 토크** = stiffness × (목표각 − 접촉각).
# 관벽 접촉각은 조인트 0°(= 암 44.427°, 휠 접촉반경 50.00mm = DN100 내반경)다.
# maxForce 를 기준으로 잡으면 예압이 정상이어도 40%대로 찍혀 오탐이 난다.
ARM_DESIGN_TORQUE = [k_deg * t for k_deg, t in zip(ARM_K_DEG, ARM_TARGET_DEG)]
load = [q / d if d else 0.0 for q, d in zip(arm_torque, ARM_DESIGN_TORQUE)]

print(f"  예압 안착  암 {min(arms_now):+.2f} ~ {max(arms_now):+.2f}°  "
      f"(무부하 목표 {tgt:+.2f}°, 관벽이 막으므로 0° 부근이 정상)")
print(f"             복원토크 {min(arm_torque):.4f}~{max(arm_torque):.4f} N·m  "
      f"= 설계 예압의 {min(load) * 100:.0f}~{max(load) * 100:.0f}%")
if max(load) < 0.5:
    print("  [경고] 예압이 안 걸렸다 — 예압이 없으면 수직항력이 0 이라 "
          "어떤 구동 토크로도 절대 전진하지 못한다")

drive_wheels(WHEEL_CMD_DEG_S)

# ── [진단] 각속도와 각도가 어긋나는지 원시값으로 확인 ──────────────
# 로그에서 각속도 -45 deg/s 가 유지되는데 순 회전이 -0.4° 였다. 1000배 어긋난다.
# 둘 중 하나는 물리값이 아니다. 연속 스텝의 raw 값을 그대로 찍어 가른다.
if "--probe" in sys.argv:
    print("\n[진단] 바퀴 0번 원시값 — 지령 "
          f"{WHEEL_CMD_DEG_S:.1f} deg/s, dt {PHYSICS_DT:.5f}s")
    print(f"  {'step':>5} {'pos(rad)':>12} {'Δpos(deg)':>11} {'vel(rad/s)':>12} "
          f"{'예상Δ(deg)':>11}")
    k = wheel_idx[0]
    prev = float(art.get_joint_positions()[k])
    for n in range(12):
        world.step(render=False)
        hold_arm_preload()
        drive_wheels(WHEEL_CMD_DEG_S)
        pos = float(art.get_joint_positions()[k])
        vel = float(art.get_joint_velocities()[k])
        print(f"  {n:5d} {pos:12.6f} {math.degrees(pos - prev):11.4f} "
              f"{vel:12.4f} {math.degrees(vel) * PHYSICS_DT:11.4f}")
        prev = pos
    # ── 어느 링크가 관벽에 닿는가 ────────────────────────────────
    # 바퀴가 물려 있다면 무엇이 잡는지 봐야 한다. 관 축은 X 이므로
    # 반경 = sqrt(y^2+z^2). 내반경 50mm 를 넘는 링크는 벽을 파고든 것이다.
    from pxr import UsdGeom as _UG
    print("\n[진단] 링크별 월드 반경 (관 내반경 50.0mm)")
    print(f"  {'링크':<16}{'반경 min':>10}{'반경 max':>10}   판정")
    bb = _UG.BBoxCache(0, ["default"], useExtentsHint=False)
    import numpy as _np
    for lp in sorted(assemble.build.__defaults__ and [] or []) or [
            f"{ROBOT}/{n}" for n in
            ["body_front", "body_rear"] +
            [f"arm_{t}{i}" for t in "fr" for i in range(3)] +
            [f"wheel_{t}{i}" for t in "fr" for i in range(3)]]:
        pr = stage.GetPrimAtPath(lp)
        if not pr.IsValid():
            continue
        rng = bb.ComputeWorldBound(pr).ComputeAlignedRange()
        lo, hi = rng.GetMin(), rng.GetMax()
        cor = _np.array([[y, z] for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        rr = _np.hypot(cor[:, 0], cor[:, 1]) * 1000.0
        mark = "  ← 관벽 침범" if rr.max() > 50.0 else ""
        print(f"  {lp.rsplit('/',1)[1]:<16}{rr.min():10.2f}{rr.max():10.2f}{mark}")
    hard_exit(simulation_app)


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
      f"{'관절(°)':>9} {'암 min~max(°)':>18} {'바퀴(°)':>9} "
      f"{'슬립':>7} {'편차(°)':>8} {'최대이탈':>16}")
print(f"{'':>6} {'':>9} {'':>9} {'':>11} {'':>9} {'':>18} "
      f"{'지령 ' + format(WHEEL_CMD_DEG_S, '.0f'):>9}")
print("-" * 78)

LOG_EVERY = max(1, STEPS // 25)
entered = passed = False
max_waist = 0.0
arm_span = [+1e9, -1e9]
traj = []

# 슬립률 기준점. 바퀴가 도는데 안 나가면(슬립≈0) 마찰·예압 문제이고,
# 바퀴 자체가 안 돌면(바퀴각 고정) 구동 지령이 PhysX 에 안 닿은 것이다.
X0, Z0 = body_xz()
wheel_track()                     # 누적 기준점 등록
S0 = X0 if X0 <= 0 else 0.0

CAM_EVERY = max(1, int(60.0 / max(camera_rig.HZ, 1.0))) if CAMERAS else 0

for step in range(STEPS):
    world.step(render=not HEADLESS)
    # 드라이브 타깃을 매 스텝 다시 건다. 한 번만 걸어도 유지되는 것이 정상이지만,
    # 유지되지 않는 경우(리셋·타깃 소실)를 조용히 통과시키지 않기 위해 매 스텝
    # 재적용한다 — 예압이 죽으면 마찰이 0 이 되어 어떤 토크로도 안 나간다.
    hold_arm_preload()
    drive_wheels(WHEEL_CMD_DEG_S)
    wdeg = wheel_track()          # 매 스텝 — 되감기 때문에 건너뛰면 안 된다
    if bridge is not None and step % CAM_EVERY == 0:
        for rig in cam_rigs:
            bridge.publish(rig)
    if step % LOG_EVERY and step != STEPS - 1:
        continue
    x, z = body_xz()
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
    theo = abs(math.radians(wdeg)) * WHEEL_R           # 이론 주행거리(m)
    # 바퀴가 거의 안 돌면 슬립률은 0/0 이라 의미가 없다. 숫자를 지어내지 않고
    # '--' 로 비워 "바퀴가 안 돈다" 는 사실 자체가 드러나게 한다.
    slip = f"{abs(s - S0) / theo:7.3f}" if theo > 1e-4 else "     --"
    # 바퀴별 이탈 — 어느 바퀴가 언제부터 접지를 잃는지 본다. 평균만 보면
    # "5개는 굴러가고 1개는 헛도는" 상태가 통째로 가려진다.
    _dev = np.degrees(_wheel_cum - _wheel_cum.mean())
    _w = int(np.argmax(np.abs(_dev)))
    print(f"{step:6d} {x * 1000:9.1f} {z * 1000:9.1f} {s * 1000:11.1f} "
          f"{waist:9.2f} {min(arms):8.2f}~{max(arms):7.2f} "
          f"{wdeg:9.1f} {slip} {wheel_spread_deg():8.1f} "
          f"{dof[wheel_idx[_w]].replace('joint_wheel_', ''):>6}{_dev[_w]:+9.1f}")

print("-" * 78)
fx, fz = body_xz()
rp, _ = art.get_world_pose()
print(f"  최종 위치      x={fx * 1000:.1f} mm  z={fz * 1000:.1f} mm  "
      f"(출발 x={X0 * 1000:.1f} → 이동 {(fx - X0) * 1000:+.1f} mm)")
print(f"  컨테이너 Xform x={float(rp[0]) * 1000:.1f} mm  "
      f"— 물리가 되쓰지 않으므로 여기서 판단하면 안 된다")
_wd = float(np.degrees(_wheel_cum.mean())) if _wheel_cum is not None else 0.0
print(f"  바퀴 회전      {_wd:+.1f}°  "
      f"(이론 주행 {abs(math.radians(_wd)) * WHEEL_R * 1000:.1f} mm)  "
      f"바퀴간 편차 {wheel_spread_deg():.1f}°")
print(f"  최대 관절각    {max_waist:.2f}°  (한계 55°)")

# ── 바퀴별 내역 ────────────────────────────────────────────────────
# 평균만 보면 "4개는 굴러가고 2개는 접지를 잃은" 상태가 안 보인다.
# 바퀴 누적 회전이 크게 갈리면 그 바퀴는 부하 없이 헛돌거나 구동을 못 전한다.
# 대응하는 암이 신장 상한에 붙어 있으면 벽에 못 닿고 있다는 뜻이다.
if _wheel_cum is not None:
    _pos = art.get_joint_positions()
    _arm_of = {k: None for k in wheel_idx}
    for wi in wheel_idx:                      # joint_wheel_f0 → joint_arm_f0
        tag = dof[wi].replace("joint_wheel_", "")
        for ai in arm_idx:
            if dof[ai] == f"joint_arm_{tag}":
                _arm_of[wi] = ai
    _mean = float(np.degrees(_wheel_cum.mean()))
    # 밀착 판정 — 휠 중심의 **월드 반경**으로 잰다. 조인트 각으로 재면 관 안에서
    # 항상 0° 부근이라 판정이 안 된다(yongbin 라인이 쓰는 방식).
    #   접촉 반경 = PIVOT_R + ARM_DR  (여기서 휠 반경만큼 더 가면 관벽 50mm)
    _xc = UsdGeom.XformCache()
    _contact_mm = (assemble.PIVOT_R + assemble.ARM_DR) * 1000.0
    print("\n  바퀴별 내역 — 밀착은 휠 중심 월드 반경으로 판정 "
          f"(접촉 {_contact_mm:.2f} mm)")
    print(f"    {'바퀴':<16}{'누적회전(°)':>12}{'평균대비':>10}"
          f"{'암각도(°)':>10}{'중심반경(mm)':>13}{'접촉편차':>9}   판정")
    _seated = 0
    for n, wi in enumerate(wheel_idx):
        cum = float(np.degrees(_wheel_cum[n]))
        ai = _arm_of[wi]
        ang = math.degrees(float(_pos[ai])) if ai is not None else float("nan")
        tag = dof[wi].replace("joint_wheel_", "")
        _xc.Clear()
        t = _xc.GetLocalToWorldTransform(
            stage.GetPrimAtPath(f"{ROBOT}/wheel_{tag}")).ExtractTranslation()
        # 관 축은 곡관 전까지 X — 반경은 YZ 성분. 곡관 안이면 근사값이다.
        r = float(np.hypot(t[1], t[2])) * 1000.0
        d = r - _contact_mm
        if abs(d) <= 0.5:
            _seated += 1
            mark = "밀착"
        else:
            mark = "← 떠 있음" if d < 0 else "← 파고듦"
        print(f"    {dof[wi]:<16}{cum:12.1f}{cum - _mean:+10.1f}{ang:10.2f}"
              f"{r:13.2f}{d:+9.2f}   {mark}")
    print(f"    밀착 {_seated}/{len(wheel_idx)}")

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
# Isaac 종료 지연을 잘라낸다 — pyver.hard_exit 참조.
# close() 는 "Shutting Down" 을 찍은 뒤 내부에서 매달리므로, 그 뒤에 무엇을
# 두어도 도달하지 못한다. 데몬 스레드에 맡기고 timeout 만 기다린다.
hard_exit(simulation_app)
