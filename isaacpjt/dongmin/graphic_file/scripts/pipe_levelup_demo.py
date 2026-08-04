"""내경이 변하는 배관(pipe_levelup)에서 다리가 관경을 따라 늘고 주는지 확인한다.

pipe_levelup.stl 실측 (CAD mm, 축 Y):
  Y=-200        내경 r40 / 외경 r50    <- 넓은 입구
  Y=-200 -> 0   테이퍼 (선형 축소)
  Y=0 -> +200   내경 r20 / 외경 r30    <- 좁은 직관

내경비가 2:1 이라 로봇 스트로크(90~101.3mm)로는 전 구간 파지가 불가능하다.
좁은 구간을 스트로크 중앙(95mm)에 맞추고, **파지가 유지되는 구간을 왕복**한다:
  좁은 직관(95mm)에서 출발 -> 테이퍼를 따라 내려가며 다리 신장 (내경 ~100mm 까지)
  -> 역주행으로 복귀하며 다리 수축

넓은 구간(190mm)까지 내려가지 않는 이유 (실측으로 확인된 설계 한계):
  바닥에 얹혀 테이퍼를 오르면 몸이 경사각(5.7deg)만큼 들린 채 좁은 구멍에 도달한다.
  95mm 구멍은 정렬 상태에서도 여유가 0.3mm 라, 기운 채로는 기하학적으로 진입 불가
  (실측: 피치 +7.3deg 로 입구 모서리에 쐐기 고정, X=-24mm 에서 정지).
  이 로봇 구조(강체 레일)는 **파지 상태로만** 급격한 축관을 통과할 수 있다.

확인하는 것:
  1. 파지 구간 왕복이 정체 없이 완주되는가
  2. 다리가 국소 내경을 따라 늘었다 줄었다 하는가 (이 데모의 핵심)
  3. 다리 위치가 기하 예측값(내경/2 - 바퀴외곽반경)을 따라가는가

실행:
  PYTHONUNBUFFERED=1 isaac_python pipe_levelup_demo.py            # GUI
  PYTHONUNBUFFERED=1 isaac_python pipe_levelup_demo.py --headless # 숫자만
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
PIPE_USD = str(ASSETS / "pipe" / "pipe_levelup.usd")

# ── 로봇 (robot_articulated.py 와 같은 값) ──
ROBOT_SCALE = 90.0 / 302.0
WHEEL_R_M = 23.0 * 0.001 * ROBOT_SCALE
WHEEL_OUT_M = (132.0 + 23.0) * 0.001 * ROBOT_SCALE      # 조인트 0 에서 바퀴 외곽 반경
LEG_LO_M = -4.0 * 0.001 * ROBOT_SCALE
LEG_HI_M = 15.0 * 0.001 * ROBOT_SCALE
ROBOT_LEN_M = 393.5 * 0.001 * ROBOT_SCALE
GRIP_MAX_BORE_MM = 2.0 * (WHEEL_OUT_M + LEG_HI_M) * 1000.0   # 101.3

# ── 배관 스케일 ──
# 반경: 좁은 구간(CAD 내경 40)을 스트로크 중앙 95mm 에 맞춘다.
# 축: 3배로 늘려 테이퍼를 5.7deg -> 1.9deg 로 완화한다. 원본 기울기에서는
#   바퀴 열 간(26mm) 내경차 5.2mm ≈ 파지폭 6.3mm 라서 2열 동시 파지 구간이
#   사실상 없고, 1열 파지 상태는 피치가 자유 자유도가 되어 기울며 재진입이 막힌다
#   (실측: 중력 유무와 무관하게 5~10deg 기울어 정체). 축 3배면 열 간 내경차가
#   1.75mm 로 줄어 2열 파지가 유지된다.
NARROW_BORE_MM = 95.0
AXIAL_STRETCH = 3.0
PIPE_SCALE = (NARROW_BORE_MM / 40.0) * 0.001   # CAD mm -> m (반경 방향)
PIPE_AX = PIPE_SCALE * AXIAL_STRETCH           # CAD mm -> m (축 방향)
PIPE_X0_M = -200.0 * PIPE_AX
PIPE_X1_M = +200.0 * PIPE_AX
PIPE_FRICTION = 0.7


def bore_mm_at(x_m):
    """월드 X 위치의 배관 내경(mm). STL 실측 프로파일 그대로."""
    y_cad = x_m / PIPE_AX
    y_cad = max(-200.0, min(200.0, y_cad))
    r_cad = 20.0 + max(0.0, -y_cad) * 0.1   # Y<0: 20 + |Y|/10 (테이퍼), Y>=0: 20
    return 2.0 * r_cad * (PIPE_SCALE * 1000.0)


# ── 주행 ──
SPIN_DEG_S = 1080.0   # 3회전/s. 360 은 실속도 31mm/s 라 GUI 에서 움직임이 안 보인다
SETTLE_STEPS = 240
DRIVE_STEPS = 2600
REPORT_EVERY = 200

world = World(stage_units_in_meters=1.0)
stage = world.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage, "/World")
UsdLux.DomeLight.Define(stage, Sdf.Path("/World/DomeLight")).CreateIntensityAttr(1000.0)

# ══ 배관 ══════════════════════════════════════════════════════════
pipe_mat_path = "/World/Pipe/PipeMaterial"
UsdGeom.Xform.Define(stage, "/World/Pipe")
pm = UsdPhysics.MaterialAPI.Apply(
    UsdShade.Material.Define(stage, pipe_mat_path).GetPrim())
pm.CreateStaticFrictionAttr(PIPE_FRICTION)
pm.CreateDynamicFrictionAttr(PIPE_FRICTION)
pm.CreateRestitutionAttr(0.0)

xf = UsdGeom.Xform.Define(stage, "/World/Pipe/section_0")
s = Gf.Matrix4d(1.0)
s.SetScale(Gf.Vec3d(PIPE_SCALE, PIPE_AX, PIPE_SCALE))   # 로컬 Y(축)만 3배
# 로컬 Y축(배관 축) -> 월드 X축. -90 이 +Y -> +X (pipe_drive_demo 에서 bbox 로 실측된
# 매핑과 동일). +90 으로 하면 넓은 입구가 +X 로 가서 로봇이 좁은 구간에 스폰된다.
r = Gf.Matrix4d(1.0)
r.SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), -90.0))
xf.AddTransformOp().Set(s * r)
xf.GetPrim().GetReferences().AddReference(PIPE_USD)

n_pipe = 0
for p in stage.Traverse():
    if str(p.GetPath()).startswith("/World/Pipe/section_") and p.IsA(UsdGeom.Mesh):
        UsdPhysics.CollisionAPI.Apply(p)
        # C1: 오목 내벽 — 정적 collider 이므로 삼각메시("none")
        UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr("none")
        UsdShade.MaterialBindingAPI.Apply(p).Bind(
            UsdShade.Material.Get(stage, pipe_mat_path),
            bindingStrength=UsdShade.Tokens.weakerThanDescendants,
            materialPurpose="physics",
        )
        n_pipe += 1
if n_pipe == 0:
    raise RuntimeError("배관 Mesh 를 못 찾음 — collider 미생성")

# ══ 로봇 — 좁은 직관 안, 관 중심(파지 위치)에서 시작 ══════════════
START_X = 0.30
START_Z = 0.0

m = Gf.Matrix4d(1.0)
m.SetRotate(Gf.Rotation(Gf.Vec3d(0, 1, 0), 90.0))     # 본체 축 -> 월드 X
t = Gf.Matrix4d(1.0)
t.SetTranslate(Gf.Vec3d(START_X, 0.0, START_Z))
UsdGeom.Xform.Define(stage, "/World/RobotRoot").AddTransformOp().Set(m * t)
add_reference_to_stage(usd_path=ROBOT_USD, prim_path="/World/RobotRoot/Robot")

art = SingleArticulation(prim_path="/World/RobotRoot/Robot/Robot", name="pipe_robot")
world.scene.add(art)
world.reset()

# 좁은 구간(반경 여유 1.3mm)에서 다리가 0 -> 4.47mm 로 튀어나가는 과도 구간을 없앤다.
# 기본 상태로 두면 낙하 + 신장 + 3방향 벽 충돌이 겹치며 솔버가 발산했다
# (실측: 40스텝 내 36 m/s). 처음부터 파지 위치(-0.1mm 여유)로 세팅하고 시작한다.
_dof0 = list(art.dof_names)
_leg0 = [k for k, n in enumerate(_dof0) if n.startswith("joint_leg_")]
_init = art.get_joint_positions()
_grip_m = NARROW_BORE_MM * 0.5 * 0.001 - WHEEL_OUT_M - 0.0001
for k in _leg0:
    _init[k] = _grip_m
art.set_joint_positions(_init)

# set_joint_positions 는 drive target 도 현재 위치로 덮어쓴다 — 그대로 두면 스프링
# 자유장이 1.2mm 가 되어 예압이 사라진다 (실측: 다리가 0.1mm 에 눌러앉음).
# robot_articulated.py 의 자유장(상한 + overtravel)을 복원한다.
_free_len_m = LEG_HI_M + 0.010 * ROBOT_SCALE
for k in _leg0:
    _jp = stage.GetPrimAtPath(f"/World/RobotRoot/Robot/Robot/{_dof0[k]}")
    UsdPhysics.DriveAPI.Get(_jp, "linear").GetTargetPositionAttr().Set(_free_len_m)


body = SingleRigidPrim(prim_path="/World/RobotRoot/Robot/Robot/body", name="body")
wheel0 = SingleRigidPrim(prim_path="/World/RobotRoot/Robot/Robot/wheel_0_1", name="w0")

# ── 배치 검증 ──
cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
pb = cache.ComputeWorldBound(stage.GetPrimAtPath("/World/Pipe")).ComputeAlignedRange()
print("\n" + "=" * 84)
print("배치 확인")
print("=" * 84)
print(f"배관 bbox X {pb.GetMin()[0]: .3f} ~ {pb.GetMax()[0]: .3f} m  "
      f"(기대 {PIPE_X0_M:.3f} ~ {PIPE_X1_M:.3f})")
print(f"내경 프로파일: 입구 {bore_mm_at(PIPE_X0_M):.0f} -> 테이퍼 -> "
      f"X=0 부터 {bore_mm_at(PIPE_X1_M):.0f} mm 직관")
print(f"로봇 파지 가능 내경 {2 * (WHEEL_OUT_M + LEG_LO_M) * 1000:.1f} ~ "
      f"{GRIP_MAX_BORE_MM:.1f} mm  -> 파지 시작 지점 X = "
      f"{(20.0 - GRIP_MAX_BORE_MM / (2 * PIPE_SCALE * 1000)) / 0.1 * PIPE_SCALE:.3f} m")
print(f"시작 위치 X={START_X:.3f} m (내경 {bore_mm_at(START_X):.0f} mm, 바닥 얹힘)")
if abs(pb.GetMin()[0] - PIPE_X0_M) > 0.01 or abs(pb.GetMax()[0] - PIPE_X1_M) > 0.01:
    raise RuntimeError("배관 bbox 가 기대와 다르다 — 회전/스케일 확인")

dof = list(art.dof_names)
leg_idx = [k for k, n in enumerate(dof) if n.startswith("joint_leg_")]
wheel_idx = [k for k, n in enumerate(dof) if n.startswith("joint_wheel_")]

print("\n정착 단계:")
for i in range(SETTLE_STEPS):
    world.step(render=not HEADLESS)
    if i % 40 == 0 or i == SETTLE_STEPS - 1:
        pp = body.get_world_pose()[0]
        vv = body.get_linear_velocity()
        print(f"  step {i:>4}  X={float(pp[0]) * 1000:>8.1f}  "
              f"Z={float(pp[2]) * 1000:>7.1f} mm   "
              f"v=({float(vv[0]) * 1000:>7.1f},{float(vv[2]) * 1000:>7.1f}) mm/s")

if not HEADLESS:
    set_camera_view(eye=[START_X - 0.15, -0.45, 0.25], target=[0.1, 0.0, 0.0])

def set_wheel_speed(deg_s):
    for k in wheel_idx:
        jp = stage.GetPrimAtPath(f"/World/RobotRoot/Robot/Robot/{dof[k]}")
        drive = UsdPhysics.DriveAPI.Get(jp, "angular")
        if not drive or not drive.GetTargetVelocityAttr():
            raise RuntimeError(f"{dof[k]} 에 angular drive 가 없다")
        drive.GetTargetVelocityAttr().Set(deg_s)


ROW_OFF_M = 88.0 * 0.001 * ROBOT_SCALE   # 바퀴 열 간격 (CAD ±88mm)


def leg_expect_mm(x_m):
    """기하 예측: 다리는 강체 레일이라 3개 바퀴 열 중 **가장 좁은 내경**이 결정한다.
    테이퍼가 +X 로 갈수록 좁아지므로 최협소 열 = X + ROW_OFF (뒤쪽/+X 열)."""
    bore = bore_mm_at(x_m + ROW_OFF_M)
    v = bore * 0.5 - WHEEL_OUT_M * 1000.0
    return max(LEG_LO_M * 1000.0, min(LEG_HI_M * 1000.0, v))


dt = world.get_physics_dt()
leg_min, leg_max = 1e9, -1e9
max_err_grip = 0.0

# 왕복 목표: X=-0.093 에서 최협소 열(X+26mm) 내경 99.5mm — 다리 ~3.5mm 신장.
# 이보다 깊이 가면 두 번째 열이 파지 한계(101.3mm)를 넘어 1열 파지가 된다.
PHASES = [(-0.093, -SPIN_DEG_S, "신장 (95mm -> ~99.5mm 방향)"),
          (+0.280, +SPIN_DEG_S, "수축 (~99.5mm -> 95mm 복귀)")]
PHASE_STEP_CAP = 4000
REPORT = 250

print("\n" + "=" * 84)
print("파지 왕복 주행")
print("=" * 84)
print(f"{'구간':>4} {'스텝':>6} {'X위치':>8} {'국소내경':>9} {'다리':>9} "
      f"{'예측다리':>9} {'피치':>7}")

ok = True
for ph, (x_goal, speed, label) in enumerate(PHASES):
    set_wheel_speed(speed)
    print(f"-- Phase {ph + 1}: {label}")
    reached = False
    last_x = None
    stall = 0
    for step in range(1, PHASE_STEP_CAP + 1):
        world.step(render=not HEADLESS)
        p_now, q_now = body.get_world_pose()
        x = float(p_now[0])
        if (speed < 0 and x <= x_goal) or (speed > 0 and x >= x_goal):
            reached = True
        pos = art.get_joint_positions()
        leg = sum(float(pos[k]) for k in leg_idx) / 3.0
        leg_min, leg_max = min(leg_min, leg), max(leg_max, leg)
        bore = bore_mm_at(x)
        exp = leg_expect_mm(x)
        if bore_mm_at(x + ROW_OFF_M) <= GRIP_MAX_BORE_MM - 1.0:   # 파지 중일 때만 집계
            max_err_grip = max(max_err_grip, abs(leg * 1000 - exp))
        if step % REPORT == 0 or reached:
            import math as _m
            qw, qx, qy, qz = [float(v) for v in q_now]
            axv = (2 * (qx * qz + qw * qy), 1 - 2 * (qx * qx + qy * qy))
            pitch = _m.degrees(_m.atan2(axv[1], axv[0]))  # 본체축의 수평 이탈각 (0=정렬)
            print(f"P{ph + 1:>3} {step:>6} {x * 1000:>6.0f}mm {bore:>7.1f}mm "
                  f"{leg * 1000:>+7.3f}mm {exp:>+7.3f}mm {pitch:>+6.2f}deg")
        if reached:
            break
        # 정체 감지: 50스텝간 0.5mm 미만 전진
        if step % 50 == 0:
            if last_x is not None and abs(x - last_x) < 0.0005:
                stall += 1
                if stall >= 3:
                    break
            else:
                stall = 0
            last_x = x
    if not reached:
        print(f"[FAIL] Phase {ph + 1} 미완주 — X={x * 1000:.0f}mm 에서 정지")
        ok = False
        break

set_wheel_speed(0.0)

# ══ 판정 ══════════════════════════════════════════════════════════
print("=" * 84)
print(f"다리 가동폭 {leg_min * 1000:+.3f} ~ {leg_max * 1000:+.3f} mm "
      f"(스트로크 {LEG_LO_M * 1000:.2f} ~ {LEG_HI_M * 1000:.2f})")
print(f"파지 중 다리 위치 vs 기하 예측 최대 오차 {max_err_grip:.3f} mm")

if ok:
    print("[OK] 왕복 완주 — 정체 없음")
if leg_max - leg_min < 0.0020:
    print("[FAIL] 다리 가동폭이 2.0mm 미만 — 관경 적응이 안 보인다")
    ok = False
else:
    print(f"[OK] 다리가 관경을 따라 {(leg_max - leg_min) * 1000:.2f} mm 신축")
if max_err_grip > 0.8:
    print("[FAIL] 파지 중 다리 위치가 기하 예측에서 0.8mm 넘게 이탈")
    ok = False
else:
    print("[OK] 다리 위치가 기하 예측을 따라감 — 벽을 물고 이동")
print("=" * 84)

if not HEADLESS:
    print("\nGUI 실행 중 — 창을 닫으면 종료됩니다.")
    while simulation_app.is_running():
        world.step(render=True)

simulation_app.close()
