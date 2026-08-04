"""[Isaac 3.11] 수리기(정찰기 + 토치 링) — **자유공간 자체검증**.

⚠ 조립 코드는 여기 없다. `robot/assemble.py` 가 유일한 조립 구현이고
`build(..., welder=True)` 로 토치 모듈을 함께 짓는다. 수리기와 정찰기는 같은
섀시라 조립을 두 벌로 두면 반드시 어긋난다.

  링크 17개 : body 2 + arm 6 + wheel 6 + ring 1 + rod 1 + tip 1
  조인트 15개: 중앙 1 + 서스펜션 6 + 바퀴 6 + J1 회전 1 + J2 직동 1
              (토치 팁은 fixed 라 DOF 가 아니다)

USD 도 만들지 않는다. 예전에는 `welder_2seg.usd` 를 구웠지만 읽는 쪽이 없었다.

🔑 **토치는 본체를 감싸는 회전 링에 단다.** 바퀴는 전·후진만 되고 중앙 관절도
수동이라 로봇을 결함 방향으로 돌릴 수단이 없다. 대신 링이 돌아 토치를 겨눈다.
회전축을 관 중심축과 일치시켜 어느 방향으로 돌려도 토치·관벽 간극이 일정하다.
**J1 은 ±180° 다** — 360° 연속 회전은 슬립링이 필요하고 슬립링은 용접 전류에
부적합하다. ±180° 로도 전 방향에 도달하며 케이블도 안 꼬인다.

🚨 **자유공간 통과는 관 내부 동작을 보장하지 않는다.** 관내 주행은
`pipe/curve_demo.py` 로 확인할 것.

실행:
  PYTHONUNBUFFERED=1 isaac_python welder/articulate.py --headless
  PYTHONUNBUFFERED=1 isaac_python welder/articulate.py            # GUI
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

# 설계 v3 §12.2 는 1/240 이상을 지정한다. 여기는 자유공간 자체검증이라 1/60 이다.
PHYSICS_DT = 1.0 / 60.0
RENDER_DT = 1.0 / 60.0

world = World(stage_units_in_meters=1.0,
              physics_dt=PHYSICS_DT, rendering_dt=RENDER_DT)
stage = world.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage, "/World")

info = assemble.build(stage, ROBOT, welder=True)

art = SingleArticulation(prim_path=ROBOT, name="pipe_welder_2seg")
world.scene.add(art)
world.reset()
for _ in range(10):
    world.step(render=not HEADLESS)

MASS_TOTAL = assemble.MASS_TOTAL_KG + assemble.MASS_TORCH_KG

print("=" * 78)
print("수리기 articulation 검증 (자유공간)")
print("=" * 78)
print(f"  총질량        {MASS_TOTAL * 1000:.0f} g   "
      f"(정찰기 {assemble.MASS_TOTAL_KG * 1000:.0f} + 토치 "
      f"{assemble.MASS_TORCH_KG * 1000:.0f})")
print(f"  휠 접촉 반경  "
      f"{(assemble.PIVOT_R + assemble.ARM_DR + assemble.WHEEL_R) * 1000:.2f} mm"
      f"  (DN100 내반경 50.0)")
print(f"  바퀴 최대토크 {assemble.WHEEL_MAX_TORQUE * 1000:.1f} mN·m")
print("  휠 충돌체     실린더 프리미티브 (convexHull 은 에지가 관벽을 문다)")
print(f"  토치  링 x {assemble.RING_X * 1000:.0f}mm, "
      f"로드 원점 r {assemble.RING_R * 1000:.0f}mm")
print(f"        J1 ±{assemble.J1_LIMIT:.0f}°  "
      f"J2 0~{assemble.J2_STROKE * 1000:.0f}mm")

dof = list(art.dof_names or [])
n_arm = sum(n.startswith("joint_arm_") for n in dof)
n_wheel = sum(n.startswith("joint_wheel_") for n in dof)
n_waist = sum(n.startswith("joint_waist") for n in dof)
n_torch = sum(n.startswith("joint_torch") for n in dof)
print(f"\n  링크 {art.num_bodies} (기대 {info['n_link']})   "
      f"DOF {art.num_dof} (기대 {info['n_dof']})")
print(f"    관절 {n_waist} / 서스펜션 {n_arm} / 바퀴 {n_wheel} / 토치 {n_torch}")

ok = art.num_bodies == info["n_link"] and art.num_dof == info["n_dof"]
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
    print("  [OK] 6개 전부 상한까지 신장")
else:
    print("  [WARN] 상한까지 벌어지지 않았다")
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
    print("  [FAIL] 돌지 않는 바퀴가 있다")
    ok = False
else:
    print("  [OK] 6개 전부 회전")

# ── 토치 동작 시험 ────────────────────────────────────────────────
print("\n  토치 동작 시험")
tj1 = [k for k, n in enumerate(dof) if n == "joint_torch_j1"]
tj2 = [k for k, n in enumerate(dof) if n == "joint_torch_j2"]

if tj1:
    for target in (+90.0, -90.0, 0.0):
        UsdPhysics.DriveAPI.Get(
            stage.GetPrimAtPath(f"{ROBOT}/joint_torch_j1"), "angular"
        ).GetTargetPositionAttr().Set(target)
        for _ in range(120):
            world.step(render=not HEADLESS)
        got = math.degrees(float(art.get_joint_positions()[tj1[0]]))
        err = abs(got - target)
        print(f"    J1 목표 {target:+7.1f}° → 실제 {got:+7.1f}°  "
              f"오차 {err:5.2f}°  {'OK' if err < 2.0 else 'FAIL'}")
        if err >= 2.0:
            ok = False

if tj2:
    tgt = assemble.J2_STROKE
    UsdPhysics.DriveAPI.Get(
        stage.GetPrimAtPath(f"{ROBOT}/joint_torch_j2"), "linear"
    ).GetTargetPositionAttr().Set(tgt)
    for _ in range(120):
        world.step(render=not HEADLESS)
    got = float(art.get_joint_positions()[tj2[0]])
    err = abs(got - tgt) * 1000.0
    print(f"    J2 목표 {tgt * 1000:5.1f}mm → 실제 {got * 1000:5.1f}mm  "
          f"오차 {err:4.2f}mm  {'OK' if err < 0.5 else 'FAIL'}")
    if err >= 0.5:
        ok = False

print("=" * 78)
print(f"자체검증: {'통과' if ok else '실패'}")
print("(USD 저장 없음 — 조립은 robot/assemble.py 가 실행 중에 한다)")
print("⚠ 자유공간 통과는 관 내부 동작을 보장하지 않는다.")
print("=" * 78)

if not HEADLESS:
    for _ in range(600):
        world.step(render=True)
        time.sleep(0.005)

hard_exit(simulation_app)
