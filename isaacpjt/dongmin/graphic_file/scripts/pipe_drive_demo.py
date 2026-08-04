"""배관 안에서 로봇이 실제로 굴러가는지 확인한다.

핵심 측정은 **슬립률** 이다.
  이론 주행거리 = 바퀴반경 x 바퀴회전각
  슬립률 = 실제 주행거리 / 이론 주행거리
  1.0 에 가까우면 미끄러짐 없이 구르는 것, 0 에 가까우면 제자리에서 헛도는 것.
바퀴가 도는 것만 봐서는 "굴러간다"고 할 수 없다. 헛돌아도 각도는 올라간다.

C1(배관 내벽은 오목면) 대응: 배관은 정적 collider 이고 근사를 "none"(삼각메시)으로 둔다.
convex hull 로 두면 원통 안쪽이 꽉 찬 덩어리가 되어 로봇이 아예 들어가지 못한다.

실행:
  PYTHONUNBUFFERED=1 isaac_python pipe_drive_demo.py            # GUI
  PYTHONUNBUFFERED=1 isaac_python pipe_drive_demo.py --headless # 숫자만
"""

import sys

HEADLESS = "--headless" in sys.argv

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": HEADLESS})

from pathlib import Path

from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.viewports import set_camera_view
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

HERE = Path(__file__).resolve().parent
ROBOT_USD = str(HERE / "robot_assembled.usd")
PIPE_USD = str(HERE.parent / "pipe" / "pipe.usd")

# ── 배관 (pipe.stl 실측: 축 Y, 길이 200, 내경 ø45, 외경 ø50, 플랜지 ø60) ──
PIPE_CAD_BORE_MM = 45.0
PIPE_CAD_LEN_MM = 200.0
PIPE_BORE_MM = 100.0                       # 확정된 값
# 참조되는 pipe.usd 는 CAD 원좌표(mm)를 그대로 들고 온다. 따라서 스케일에
# mm->m 변환(0.001)이 함께 들어가야 한다. 빠뜨리면 배관이 1000배로 만들어진다.
PIPE_SCALE = (PIPE_BORE_MM / PIPE_CAD_BORE_MM) * 0.001
PIPE_SEC_LEN_M = PIPE_CAD_LEN_MM * PIPE_SCALE
PIPE_SECTIONS = 4                          # 이어 붙일 구간 수 (주행거리 확보용)
PIPE_FRICTION = 0.7

# ── 로봇 (robot_articulated.py 와 같은 값이어야 한다) ──
ROBOT_SCALE = 90.0 / 302.0
WHEEL_R_M = 23.0 * 0.001 * ROBOT_SCALE
WHEEL_OUT_M = (132.0 + 23.0) * 0.001 * ROBOT_SCALE   # 조인트 0 일 때 바퀴 외곽 반경
ROBOT_LEN_M = 393.5 * 0.001 * ROBOT_SCALE

# ── 주행 ──
SPIN_DEG_S = 360.0                         # 바퀴 목표 각속도
SETTLE_STEPS = 180                         # 다리가 관벽을 찾아 자리잡는 시간
DRIVE_STEPS = 900
REPORT_EVERY = 100

world = World(stage_units_in_meters=1.0)
stage = world.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage, "/World")

UsdLux.DomeLight.Define(stage, Sdf.Path("/World/DomeLight")).CreateIntensityAttr(1000.0)


def mat4(scale=1.0, rot_axis=None, rot_deg=0.0, translate=(0.0, 0.0, 0.0)):
    """행벡터 규약으로 S * R * T 를 만든다 (p_world = p_local * M)."""
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
pipe_mat.CreateStaticFrictionAttr(PIPE_FRICTION)
pipe_mat.CreateDynamicFrictionAttr(PIPE_FRICTION)
pipe_mat.CreateRestitutionAttr(0.0)

for i in range(PIPE_SECTIONS):
    path = f"/World/Pipe/section_{i}"
    xf = UsdGeom.Xform.Define(stage, path)
    # pipe 로컬 축은 Y. Z축 -90도 회전으로 Y -> 월드 X 로 눕힌다.
    xf.AddTransformOp().Set(
        mat4(PIPE_SCALE, (0, 0, 1), -90.0, (i * PIPE_SEC_LEN_M, 0.0, 0.0)))
    xf.GetPrim().GetReferences().AddReference(PIPE_USD)

n_pipe_mesh = 0
for p in stage.Traverse():
    if str(p.GetPath()).startswith("/World/Pipe/section_") and p.IsA(UsdGeom.Mesh):
        UsdPhysics.CollisionAPI.Apply(p)
        # C1: 오목 내벽이므로 삼각메시. rigid body 가 없는 정적 collider 라서 가능하다.
        UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr("none")
        UsdShade.MaterialBindingAPI.Apply(p).Bind(
            UsdShade.Material.Get(stage, pipe_mat_path),
            bindingStrength=UsdShade.Tokens.weakerThanDescendants,
            materialPurpose="physics",
        )
        n_pipe_mesh += 1
if n_pipe_mesh == 0:
    raise RuntimeError("배관 Mesh 를 못 찾음 — collider 미생성")

# ══ 로봇 ══════════════════════════════════════════════════════════
# 로봇 로컬 Z(본체 축) 를 월드 X(배관 축) 로 눕힌다.
START_X = 0.5 * ROBOT_LEN_M + 0.02
robot_xf = UsdGeom.Xform.Define(stage, "/World/RobotRoot")
robot_xf.AddTransformOp().Set(mat4(1.0, (0, 1, 0), 90.0, (START_X, 0.0, 0.0)))
add_reference_to_stage(usd_path=ROBOT_USD, prim_path="/World/RobotRoot/Robot")

art = SingleArticulation(prim_path="/World/RobotRoot/Robot/Robot", name="pipe_robot")
world.scene.add(art)
world.reset()

body = SingleRigidPrim(prim_path="/World/RobotRoot/Robot/Robot/body", name="body")
wheel0 = SingleRigidPrim(prim_path="/World/RobotRoot/Robot/Robot/wheel_0_1",
                         name="wheel0")

# ── 배치 검증: 배관과 로봇이 실제로 어디에 있는지 ──
cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
pb = cache.ComputeWorldBound(stage.GetPrimAtPath("/World/Pipe")).ComputeAlignedRange()
print("\n" + "=" * 78)
print("배치 확인")
print("=" * 78)
print(f"배관 bbox  X {pb.GetMin()[0]: .3f} ~ {pb.GetMax()[0]: .3f} m "
      f"(길이 {pb.GetMax()[0] - pb.GetMin()[0]:.3f})")
print(f"           Y {pb.GetMin()[1]: .3f} ~ {pb.GetMax()[1]: .3f} m  "
      f"Z {pb.GetMin()[2]: .3f} ~ {pb.GetMax()[2]: .3f} m")
print(f"배관 내경 {PIPE_BORE_MM:.0f} mm / 외경 "
      f"{(pb.GetMax()[1] - pb.GetMin()[1]) * 1000:.0f} mm (플랜지 포함)")
print(f"로봇 전장 {ROBOT_LEN_M * 1000:.0f} mm, 바퀴 반경 {WHEEL_R_M * 1000:.2f} mm")

if pb.GetMax()[0] - pb.GetMin()[0] < 2 * ROBOT_LEN_M:
    raise RuntimeError("배관이 로봇 길이의 2배도 안 된다 — PIPE_SECTIONS 를 늘릴 것")

dof = list(art.dof_names)
leg_idx = [k for k, n in enumerate(dof) if n.startswith("joint_leg_")]
wheel_idx = [k for k, n in enumerate(dof) if n.startswith("joint_wheel_")]

# ── 1단계: 다리가 관벽을 찾을 때까지 정착 ──
for _ in range(SETTLE_STEPS):
    world.step(render=not HEADLESS)

pos = art.get_joint_positions()
leg_settled = sum(float(pos[k]) for k in leg_idx) / 3.0
robot_dia = 2.0 * (WHEEL_OUT_M + leg_settled) * 1000.0

# 자유 신장 상태와 관벽에 눌린 상태를 반드시 구분해야 한다.
# 최대 신장 외경(101.3mm)이 관 내경(100mm)과 1.3mm 밖에 차이나지 않으므로
# "외경이 관 내경과 비슷하다" 는 조건은 허공에 떠 있어도 통과해 버린다.
LEG_FREE_M = 15.0 * 0.001 * ROBOT_SCALE            # 스트로크 상한 = 자유 신장
LEG_EXPECT_M = PIPE_BORE_MM * 0.5 * 0.001 - WHEEL_OUT_M   # 관벽에 닿았을 때
print(f"\n정착 후 다리 위치 {leg_settled * 1000:+.3f} mm "
      f"-> 로봇 외경 {robot_dia:.1f} mm  (배관 내경 {PIPE_BORE_MM:.0f} mm)")
print(f"  자유 신장 {LEG_FREE_M * 1000:.3f} mm / 관벽 접촉 예상 "
      f"{LEG_EXPECT_M * 1000:.3f} mm")
contact = (leg_settled < LEG_FREE_M - 2e-4
           and abs(leg_settled - LEG_EXPECT_M) < 3e-4)
if contact:
    print("  [OK] 다리가 자유 신장보다 눌려 있고 예상 접촉 위치와 일치 — 관벽에 닿았다")
else:
    print("  [FAIL] 다리가 눌리지 않았다 — 로봇이 관 안에 없거나 벽에 안 닿았다")

off0 = (float(body.get_world_pose()[0][1]) ** 2
        + float(body.get_world_pose()[0][2]) ** 2) ** 0.5
print(f"  배관 축 이탈 {off0 * 1000:.2f} mm")
if off0 > PIPE_BORE_MM * 0.5 * 0.001:
    raise RuntimeError("로봇이 배관 밖에 있다 — 배치/스케일 확인")

if not HEADLESS:
    set_camera_view(eye=[START_X - 0.25, -0.28, 0.16],
                    target=[START_X + 0.15, 0.0, 0.0])

# ── 2단계: 주행 ──
for k in wheel_idx:
    jp = stage.GetPrimAtPath(f"/World/RobotRoot/Robot/Robot/{dof[k]}")
    if not jp.IsValid():
        raise RuntimeError(f"바퀴 조인트 프림을 못 찾음: {dof[k]}")
    drive = UsdPhysics.DriveAPI.Get(jp, "angular")
    attr = drive.GetTargetVelocityAttr()
    if not attr:
        raise RuntimeError(f"{dof[k]} 에 angular drive 가 없다")
    attr.Set(SPIN_DEG_S)

x0 = float(body.get_world_pose()[0][0])

# 바퀴 누적 회전각은 **각속도 적분**으로 구한다.
#
# 조인트 "각도" 차분은 쓸 수 없다. revolute 각도가 ±pi 로 wrap 되기 때문인데,
# 한 스텝 변화가 pi 미만이라 언랩이 될 것 같지만 실제로는 회전수를 잃는다
# (get_joint_positions 가 매 스텝 갱신되지 않는 것으로 보인다).
# 실측으로 확인했다 — 목표 6.28 rad/s 에 대해
#   언랩 2.4 / 조인트 각속도 7.3 / 바퀴 강체 각속도 6.2 rad/s
# 서로 독립인 뒤의 두 측정이 일치하므로 언랩 쪽이 틀린 것이다.
dt = world.get_physics_dt()
spin = 0.0

print("\n" + "=" * 78)
print(f"주행 (바퀴 {SPIN_DEG_S:.0f} deg/s)")
print("=" * 78)
print(f"{'스텝':>6} {'주행거리':>10} {'바퀴회전':>10} {'이론거리':>10} "
      f"{'슬립률':>8} {'다리':>9} {'축이탈':>8} {'각속도적분|강체':>14}")

for step in range(1, DRIVE_STEPS + 1):
    world.step(render=not HEADLESS)
    vel = art.get_joint_velocities()
    spin += sum(float(vel[k]) for k in wheel_idx) / len(wheel_idx) * dt
    if step % REPORT_EVERY:
        continue
    p_now = body.get_world_pose()[0]
    pos = art.get_joint_positions()
    travel = float(p_now[0]) - x0
    ideal = WHEEL_R_M * spin
    slip = travel / ideal if abs(ideal) > 1e-9 else 0.0
    leg = sum(float(pos[k]) for k in leg_idx) / 3.0
    # 배관 축(X)에서 벗어난 거리 = 로봇이 관 중심을 유지하는지
    off = (float(p_now[1]) ** 2 + float(p_now[2]) ** 2) ** 0.5
    # 적분값이 맞는지 조인트를 거치지 않는 강체 각속도로 교차 확인한다.
    w_now = spin / (step * dt)
    w_rigid = max(abs(float(v)) for v in wheel0.get_angular_velocity())
    print(f"{step:>6} {travel * 1000:>8.1f}mm {spin:>8.2f}rad "
          f"{ideal * 1000:>8.1f}mm {slip:>8.3f} {leg * 1000:>+7.3f}mm "
          f"{off * 1000:>6.2f}mm {w_now:>6.2f}|{w_rigid:>5.2f}")

p_end = body.get_world_pose()[0]
travel = float(p_end[0]) - x0
ideal = WHEEL_R_M * spin
slip = travel / ideal if abs(ideal) > 1e-9 else 0.0

print("=" * 78)
print(f"총 주행거리 {travel * 1000:.1f} mm / 이론 {ideal * 1000:.1f} mm "
      f"-> 슬립률 {slip:.3f}")
if travel < 0.01:
    print("[FAIL] 거의 나아가지 않았다 — 바퀴가 헛돌거나 어딘가 걸려 있다")
elif slip < 0.5:
    print(f"[FAIL] 슬립률 {slip:.3f} — 바퀴가 절반 이상 미끄러진다. "
          "마찰/예압/토크 확인")
elif slip > 1.3:
    print(f"[경고] 슬립률 {slip:.3f} > 1 — 바퀴 회전보다 많이 갔다. 미끄러져 내려갔을 수 있다")
else:
    print(f"[OK] 슬립률 {slip:.3f} — 미끄러짐 없이 굴러간다")
print("=" * 78)

if not HEADLESS:
    print("\nGUI 실행 중 — 창을 닫으면 종료됩니다.")
    while simulation_app.is_running():
        world.step(render=True)

simulation_app.close()
