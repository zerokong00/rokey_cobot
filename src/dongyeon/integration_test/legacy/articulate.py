"""[Isaac 3.11] 2단 1관절 로봇 — **자유공간 자체검증**.

⚠ 이 파일은 더 이상 USD 를 만들지 않는다. 조립은 `robot/assemble.py` 한 벌뿐이고
주행 스크립트가 실행 중에 씬으로 직접 짓는다. 예전에는 여기서 `robot_2seg.usd` 를
구워 저장하고 주행 쪽이 그것을 `AddReference` 로 불러왔는데, 그 파일 왕복이
세대 불일치·낡은 USD·덮어쓰기 우회의 근원이었다(assemble.py 머리말 참조).

여기 남은 역할은 하나다 — **조립 결과가 설계값과 맞는지 자유공간에서 확인**한다.

🚨 **자유공간 통과는 관 내부 동작을 보장하지 않는다.** 여기서 "암 6개 상한까지
신장 / 바퀴 6개 회전" 이 전부 통과해도 관에 넣으면 안 움직일 수 있다 — 실제로
휠 콜라이더가 convexHull 이던 동안 이 검증은 계속 통과했고 관내 주행은 0mm 였다.
관내 주행은 `pipe/curve_demo.py` 로 확인할 것.

실행:
  PYTHONUNBUFFERED=1 isaac_python robot/articulate.py --headless   # 숫자만
  PYTHONUNBUFFERED=1 isaac_python robot/articulate.py              # GUI
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pyver import hard_exit, require_isaac

require_isaac(__file__, needs_rclpy=False)

HEADLESS = "--headless" in sys.argv

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": HEADLESS})

import math                                              # noqa: E402
import time                                              # noqa: E402

from isaacsim.core.api import World                      # noqa: E402
from isaacsim.core.prims import SingleArticulation       # noqa: E402
from pxr import UsdGeom, UsdPhysics                      # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import assemble                                          # noqa: E402

ROBOT = "/World/Robot"

# 설계 v3 §12.2 는 1/240 이상을 지정한다. 여기는 자유공간 자체검증이라 1/60 으로
# 둔다 — 관내 주행 판정은 pipe/curve_demo.py 가 하고, 그쪽에서 1/240 도 시험했다
# (1/60·1/240 둘 다 결과 동일. 주행을 가른 것은 휠 충돌체였다).
PHYSICS_DT = 1.0 / 60.0
RENDER_DT = 1.0 / 60.0

world = World(stage_units_in_meters=1.0,
              physics_dt=PHYSICS_DT, rendering_dt=RENDER_DT)
stage = world.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage, "/World")

assemble.build(stage, ROBOT)

art = SingleArticulation(prim_path=ROBOT, name="pipe_robot_2seg")
world.scene.add(art)
world.reset()
for _ in range(10):
    world.step(render=not HEADLESS)

print("=" * 78)
print("articulation 검증 (자유공간)")
print("=" * 78)
print(f"  총질량        {assemble.MASS_TOTAL_KG * 1000:.0f} g   (설계 500 g)")
print(f"  휠 접촉 반경  "
      f"{(assemble.PIVOT_R + assemble.ARM_DR + assemble.WHEEL_R) * 1000:.2f} mm"
      f"  (DN100 내반경 50.0)")
print(f"  암 한계       {assemble.ARM_LIMIT_LOWER:+.2f}° ~ "
      f"{assemble.ARM_LIMIT_UPPER:+.2f}°  "
      f"(무부하 목표 {assemble.ARM_DRIVE_TARGET_DEG:+.2f}°)")
print(f"  암 스프링     K = {assemble.ARM_K_NM_RAD:.4f} N·m/rad  → 기준자세 토크 "
      f"{assemble.ARM_TORQUE_NM:.4f} N·m = 예압 {assemble.WHEEL_PRELOAD_N:.1f} N")
print(f"  관절          ±{assemble.WAIST_LIMIT:.0f}°  센터링 "
      f"{assemble.WAIST_SPRING_NM_RAD} N·m/rad (수동)")
print(f"  바퀴 최대토크 {assemble.WHEEL_MAX_TORQUE * 1000:.1f} mN·m  "
      f"(마찰 한계의 {assemble.WHEEL_TORQUE_FRACTION * 100:.0f}%)")
print(f"  휠 충돌체     실린더 프리미티브 r={assemble.WHEEL_R * 1000:.1f} "
      f"h={assemble.WHEEL_WIDTH * 1000:.1f} mm  (convexHull 은 에지가 관벽을 문다)")

dof = list(art.dof_names or [])
n_arm = sum(n.startswith("joint_arm_") for n in dof)
n_wheel = sum(n.startswith("joint_wheel_") for n in dof)
n_waist = sum(n.startswith("joint_waist") for n in dof)
print(f"\n  링크 {art.num_bodies} (기대 {assemble.N_LINK})   "
      f"DOF {art.num_dof} (기대 {assemble.N_DOF})")
print(f"    관절 {n_waist} / 서스펜션 {n_arm} / 바퀴 {n_wheel}")

ok = (art.num_bodies == assemble.N_LINK and art.num_dof == assemble.N_DOF
      and (n_waist, n_arm, n_wheel) == (1, assemble.N_ARM, assemble.N_ARM))
if not ok:
    print("  [FAIL] 링크·DOF 구성이 기대와 다르다")

aidx = [k for k, n in enumerate(dof) if n.startswith("joint_arm_")]
widx = [k for k, n in enumerate(dof) if n.startswith("joint_wheel_")]

for _ in range(60):
    world.step(render=not HEADLESS)
arms = [math.degrees(float(art.get_joint_positions()[k])) for k in aidx]
print(f"\n  자유공간 60스텝 후 암 각도: {min(arms):+.2f}° ~ {max(arms):+.2f}°  "
      f"(상한 {assemble.ARM_LIMIT_UPPER:+.2f}°)")
if min(arms) > assemble.ARM_LIMIT_UPPER - 1.0:
    print("  [OK] 6개 전부 상한까지 신장 — 스프링 drive 동작 (관벽 없으므로 정상)")
else:
    print("  [WARN] 상한까지 벌어지지 않았다 — 예압/한계 확인")
    ok = False

SPIN = 180.0
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

if n_waist:
    wi = [k for k, n in enumerate(dof) if n.startswith("joint_waist")][0]
    print(f"  관절 각도 {math.degrees(float(art.get_joint_positions()[wi])):+.3f}°"
          f"  (수동이므로 자유공간에서 0 부근이어야 정상)")

print("=" * 78)
print(f"자체검증: {'통과' if ok else '실패'}")
print("⚠ 자유공간 통과는 관 내부 동작을 보장하지 않는다 — "
      "관내 주행은 pipe/curve_demo.py 로 확인할 것.")
print("=" * 78)

if not HEADLESS:
    for _ in range(600):
        world.step(render=True)
        time.sleep(0.005)

hard_exit(simulation_app)
