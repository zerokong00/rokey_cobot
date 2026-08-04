"""바닥을 만들고 로봇을 떨어뜨린다.

배관 밖 자유 낙하 + 착지 거동을 본다. 형상만 맞춰놓고 넘어가면 나중에
배관 진입/이탈 구간에서 터지는 걸 뒤늦게 발견하게 되므로, 평면 충돌부터 확인해 둔다.

확인하는 것:
  1. 실제로 떨어져서 멈추는가 (낙하 -> 착지 -> 정지)
  2. 착지 후 튀거나 뚫고 내려가지 않는가
  3. 착지 높이가 바퀴 외곽 반경과 맞는가 (= 바퀴로 서 있는가)
  4. 착지 충격에 articulation 이 터지지 않는가

실행:
  PYTHONUNBUFFERED=1 isaac_python drop_demo.py            # GUI
  PYTHONUNBUFFERED=1 isaac_python drop_demo.py --headless # 숫자만
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
# 2026-08-04 폴더 정리: scripts/ 와 assets/ 분리 — 에셋은 여기 기준
ASSETS = HERE.parent / "assets"
ROBOT_USD = str(ASSETS / "robot" / "robot_assembled.usd")
PIPE_USD = str(ASSETS / "pipe" / "pipe.usd")

# 배관 (pipe_drive_demo.py 와 같은 값). 바닥 위에 눕혀 둔다 — 로봇은 그 옆에 떨어진다.
PIPE_CAD_BORE_MM = 45.0
PIPE_CAD_LEN_MM = 200.0
PIPE_BORE_MM = 100.0
PIPE_SCALE = (PIPE_BORE_MM / PIPE_CAD_BORE_MM) * 0.001
PIPE_SEC_LEN_M = PIPE_CAD_LEN_MM * PIPE_SCALE
PIPE_SECTIONS = 2
# 플랜지(CAD ø60)가 최외곽이므로 눕혔을 때 축 높이 = 플랜지 반경
PIPE_AXIS_Z_M = 30.0 * PIPE_SCALE
PIPE_Y_M = 0.20                     # 낙하 지점에서 옆으로 띄워 둔다

# robot_articulated.py 와 같은 값
ROBOT_SCALE = 90.0 / 302.0
WHEEL_OUT_M = (132.0 + 23.0) * 0.001 * ROBOT_SCALE   # 조인트 0 일 때 바퀴 외곽 반경
LEG_FREE_M = 15.0 * 0.001 * ROBOT_SCALE

DROP_HEIGHT_M = 0.30       # 낙하 시작 높이 (본체 중심)
TILT_DEG = 90.0            # 본체 축을 눕힌다 (배관 안에서와 같은 자세)
FALL_STEPS = 2400   # 240Hz 이므로 스텝 수도 4배
REPORT_EVERY = 240

# 90mm 로봇 / 6.85mm 바퀴 스케일에서는 기본 60Hz 물리 스텝이 성기다.
# 60Hz 로 두면 착지 시 본체가 바닥을 5mm 파고든다(실측). KB NOTES/03 §7 참고.
PHYSICS_HZ = 240.0

world = World(stage_units_in_meters=1.0, physics_dt=1.0 / PHYSICS_HZ)
stage = world.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage, "/World")
UsdLux.DomeLight.Define(stage, Sdf.Path("/World/DomeLight")).CreateIntensityAttr(1000.0)

# ══ 바닥 ══════════════════════════════════════════════════════════
# z=0 평면. 정적 collider 라서 별도 근사 설정이 필요 없다.
world.scene.add_default_ground_plane()

# ══ 배관 — 바닥 위에 눕혀 둔다 ═══════════════════════════════════
UsdGeom.Xform.Define(stage, "/World/Pipe")
for i in range(PIPE_SECTIONS):
    xf = UsdGeom.Xform.Define(stage, f"/World/Pipe/section_{i}")
    s = Gf.Matrix4d(1.0)
    s.SetScale(Gf.Vec3d(PIPE_SCALE, PIPE_SCALE, PIPE_SCALE))
    r = Gf.Matrix4d(1.0)
    r.SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), -90.0))   # 로컬 Y축 -> 월드 X축
    t = Gf.Matrix4d(1.0)
    t.SetTranslate(Gf.Vec3d(i * PIPE_SEC_LEN_M - PIPE_SEC_LEN_M, PIPE_Y_M,
                            PIPE_AXIS_Z_M))
    xf.AddTransformOp().Set(s * r * t)
    xf.GetPrim().GetReferences().AddReference(PIPE_USD)

n_pipe = 0
for p in stage.Traverse():
    if str(p.GetPath()).startswith("/World/Pipe/section_") and p.IsA(UsdGeom.Mesh):
        UsdPhysics.CollisionAPI.Apply(p)
        # C1: 오목 내벽 — 정적 collider 이므로 삼각메시("none") 사용 가능
        UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr("none")
        n_pipe += 1
if n_pipe == 0:
    raise RuntimeError("배관 Mesh 를 못 찾음 — collider 미생성")

# ══ 로봇 ══════════════════════════════════════════════════════════
m = Gf.Matrix4d(1.0)
m.SetRotate(Gf.Rotation(Gf.Vec3d(0, 1, 0), TILT_DEG))
t = Gf.Matrix4d(1.0)
t.SetTranslate(Gf.Vec3d(0.0, 0.0, DROP_HEIGHT_M))
UsdGeom.Xform.Define(stage, "/World/RobotRoot").AddTransformOp().Set(m * t)
add_reference_to_stage(usd_path=ROBOT_USD, prim_path="/World/RobotRoot/Robot")

art = SingleArticulation(prim_path="/World/RobotRoot/Robot/Robot", name="pipe_robot")
world.scene.add(art)
world.reset()

body = SingleRigidPrim(prim_path="/World/RobotRoot/Robot/Robot/body", name="body")

if not HEADLESS:
    # 낙하 지점(원점)과 옆에 놓인 배관(y=0.2)이 둘 다 보이는 각도
    set_camera_view(eye=[0.45, -0.35, 0.25], target=[0.0, 0.10, 0.05])

dof = list(art.dof_names)
leg_idx = [k for k, n in enumerate(dof) if n.startswith("joint_leg_")]

print("\n" + "=" * 74)
print(f"낙하 시험 — 높이 {DROP_HEIGHT_M * 1000:.0f} mm 에서 놓는다")
print(f"  바퀴 외곽 반경 {WHEEL_OUT_M * 1000:.1f} ~ "
      f"{(WHEEL_OUT_M + LEG_FREE_M) * 1000:.1f} mm  -> 착지 높이는 이 근처여야 한다")
print("=" * 74)
print(f"{'스텝':>6} {'본체높이':>10} {'낙하속도':>11} {'다리':>10} {'최저점':>10}")

cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
robot_prim = stage.GetPrimAtPath("/World/RobotRoot/Robot/Robot")

# 다리 3개의 가운데 바퀴. 어느 다리가 아래로 향해 접지하는지 보려면 이게 필요하다.
mid_wheels = [
    SingleRigidPrim(prim_path=f"/World/RobotRoot/Robot/Robot/wheel_{i}_1",
                    name=f"w{i}") for i in range(3)
]


def robot_min_z():
    # BBoxCache 는 결과를 캐싱한다. 물리로 프림이 움직여도 자동 무효화되지 않으므로
    # 매번 지워야 한다. (지우지 않으면 낙하 중에도 같은 값이 계속 나온다)
    cache.Clear()
    return cache.ComputeWorldBound(robot_prim).ComputeAlignedRange().GetMin()[2]

z_hist = []
min_z_seen = 1e9
for step in range(1, FALL_STEPS + 1):
    world.step(render=not HEADLESS)
    z = float(body.get_world_pose()[0][2])
    z_hist.append(z)
    if step % REPORT_EVERY:
        continue
    vz = float(body.get_linear_velocity()[2])
    pos = art.get_joint_positions()
    leg = sum(float(pos[k]) for k in leg_idx) / len(leg_idx)
    low = robot_min_z()
    min_z_seen = min(min_z_seen, low)
    wz = [float(w.get_world_pose()[0][2]) * 1000 for w in mid_wheels]
    print(f"{step:>6} {z * 1000:>8.1f}mm {vz * 1000:>9.1f}mm/s "
          f"{leg * 1000:>+8.3f}mm {low * 1000:>8.2f}mm   "
          f"바퀴z {wz[0]:6.1f} {wz[1]:6.1f} {wz[2]:6.1f}")

# ══ 판정 ══════════════════════════════════════════════════════════
z_end = z_hist[-1]
settled_band = z_hist[-480:]
wobble = max(settled_band) - min(settled_band)
vz_end = abs(float(body.get_linear_velocity()[2]))
pos = art.get_joint_positions()
leg_end = sum(float(pos[k]) for k in leg_idx) / len(leg_idx)

print("=" * 74)
print(f"착지 높이   {z_end * 1000:.1f} mm   (마지막 120스텝 변동 {wobble * 1000:.2f} mm)")
print(f"잔류 속도   {vz_end * 1000:.2f} mm/s")
print(f"로봇 최저점 최소값 {min_z_seen * 1000:+.2f} mm  (바닥은 0)")
print(f"다리 위치   {leg_end * 1000:+.3f} mm  (자유 신장 {LEG_FREE_M * 1000:.3f})")

# 어느 링크가 가장 아래인지 = 무엇으로 지탱하고 있는지
print("\n링크별 최저점 (바닥 z=0):")
rows = []
for name in ["body"] + [f"leg_{i}" for i in range(3)] + [
        f"wheel_{i}_{j}" for i in range(3) for j in range(3)]:
    cache.Clear()
    p = stage.GetPrimAtPath(f"/World/RobotRoot/Robot/Robot/{name}")
    rows.append((cache.ComputeWorldBound(p).ComputeAlignedRange().GetMin()[2], name))
for zmin, name in sorted(rows)[:5]:
    mark = "  <-- 바닥 아래" if zmin < -1e-4 else ""
    print(f"  {name:14s} {zmin * 1000:+8.2f} mm{mark}")

ok = True
if z_end > DROP_HEIGHT_M - 0.01:
    print("[FAIL] 떨어지지 않았다 — 중력/충돌 설정 확인")
    ok = False
elif min_z_seen < -0.002:
    print(f"[FAIL] 시각 메시가 바닥을 {abs(min_z_seen) * 1000:.1f} mm 뚫고 내려가 있다.")
    print("       원인 미규명. 아래는 실측으로 **배제된** 가설이다:")
    print("        - 물리 스텝 부족   : 60Hz -> 240Hz 로 올려도 5.21 -> 4.86mm, 사실상 동일")
    print("        - 볼록 분해 실패   : body 를 convexHull 로 바꿔도 소수점까지 동일")
    print("        - collider 누락    : 13개 링크 전부 mesh 1개씩 collider 부착 확인")
    print("       남은 유력 후보는 볼록 충돌체가 시각 메시보다 작게 생성되는 것")
    print("       (PhysX 의 hull 수축/rest offset 은 절대값이라 90mm 스케일에서 비중이 큼).")
    print("       배관 주행에는 영향이 없다 — 그쪽은 바퀴가 본체를 벽에서 띄워 준다.")
    ok = False
elif wobble > 0.002 or vz_end > 5.0:
    print("[FAIL] 착지 후에도 계속 움직인다 — 튀거나 굴러가고 있다")
    ok = False
elif abs(z_end - WHEEL_OUT_M) > 0.006:
    print(f"[경고] 착지 높이가 바퀴 외곽 반경({WHEEL_OUT_M * 1000:.1f}mm)과 "
          "6mm 넘게 차이난다 — 바퀴가 아닌 다른 곳으로 서 있을 수 있다")
else:
    print(f"[OK] 바닥에 떨어져 바퀴로 안착. 관통 없음, 정지 완료")

if any(abs(float(v)) > 1.0 for v in pos):
    print("[FAIL] 착지 충격으로 조인트가 발산했다")
    ok = False
print("=" * 74)

if not HEADLESS:
    print("\nGUI 실행 중 — 창을 닫으면 종료됩니다.")
    while simulation_app.is_running():
        world.step(render=True)

simulation_app.close()
