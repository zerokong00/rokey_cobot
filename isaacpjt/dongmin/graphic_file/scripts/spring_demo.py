"""leg 스프링 동작 확인용 데모 — 관경을 바꿔가며 다리가 열리고 닫히는 걸 눈으로 본다.

자유 공간에 두면 다리가 스트로크 상한에 붙어만 있어 아무 일도 일어나지 않는다.
그래서 여기서는 **관벽이 다리를 미는 힘을 재현**한다.

  내경 D 인 관에 들어가면 바퀴가 벽에 닿아 다리가 x = D/2 - 155mm 위치까지 눌린다.
  그 위치에서 평형이 되려면 관벽이 E = K*(x - target) 만큼 안쪽으로 밀어야 한다.
  이 E 를 조인트에 외력으로 걸어주면 실제 배관 안에 있는 것과 같은 상태가 된다.

관경을 300 -> 342 -> 300 mm 로 계속 왕복시키므로, 스프링이 살아 있으면
다리 3개가 같이 벌어졌다 오므라든다. 콘솔에는 실제 관경이 찍힌다.

중력은 끈다. 자유 낙하하면 로봇이 화면 밖으로 나가 다리 움직임을 볼 수 없다.

실행:
  PYTHONUNBUFFERED=1 isaac_python spring_demo.py            # GUI (이걸로 보세요)
  PYTHONUNBUFFERED=1 isaac_python spring_demo.py --headless # 숫자만 확인
"""

import math
import sys

HEADLESS = "--headless" in sys.argv

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": HEADLESS})

from pathlib import Path

from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.viewports import set_camera_view
from pxr import Sdf, UsdLux, UsdPhysics

HERE = Path(__file__).resolve().parent
ROBOT_USD = str(HERE / "robot_assembled.usd")

# robot_articulated.py 와 같은 값이어야 한다
WHEEL_OUTER_MM = 155.0          # 조인트 0 일 때 바퀴 외곽 반경 (132 + 23)
SPRING_K = 3000.0
SPRING_TARGET_M = 0.025
LIMIT_LO_M, LIMIT_HI_M = -0.004, 0.015

# 스트로크로 커버되는 관경은 302~340mm. 일부러 그 밖(300, 342)까지 훑어서
# 한계에 걸리는 지점도 눈에 보이게 한다.
BORE_MIN_MM, BORE_MAX_MM = 300.0, 342.0
PERIOD_FRAMES = 240             # 왕복 1주기

world = World(stage_units_in_meters=1.0)
add_reference_to_stage(usd_path=ROBOT_USD, prim_path="/Scene")

# GUI 에서 형상이 보이도록 조명
light = UsdLux.DomeLight.Define(world.stage, Sdf.Path("/Scene/DomeLight"))
light.CreateIntensityAttr(1000.0)

# 본체를 월드에 고정한다.
# 중력을 꺼도 free-floating articulation 은 리셋 직후 다리가 상한으로 튀어나가는
# 전이에서 미세한 알짜 힘을 받는데, 감쇠될 데가 없어 그대로 속도로 적분되고
# 로봇이 계속 표류해 시야를 벗어난다. 이 데모는 다리 움직임만 보면 되므로 고정한다.
# (articulation 이 인식하도록 root prim 하위에, world.reset() 전에 만들어야 한다)
fix = UsdPhysics.FixedJoint.Define(world.stage, "/Scene/Robot/root_fix")
fix.CreateBody1Rel().SetTargets(["/Scene/Robot/body"])   # body0 미지정 = 월드

art = SingleArticulation(prim_path="/Scene/Robot", name="pipe_robot")
world.scene.add(art)
world.reset()

world.get_physics_context().set_gravity(0.0)
set_camera_view(eye=[0.75, 0.75, 0.35], target=[0.0, 0.0, 0.0])

dof = list(art.dof_names)
leg_idx = [k for k, n in enumerate(dof) if n.startswith("joint_leg_")]
if len(leg_idx) != 3:
    raise RuntimeError(f"leg 조인트를 3개 못 찾음: {dof}")


def bore_to_joint_m(bore_mm):
    """관 내경(mm) -> 그 관에서 다리가 눌리는 조인트 위치(m)."""
    return (bore_mm / 2.0 - WHEEL_OUTER_MM) * 0.001


def joint_m_to_bore(x_m):
    """조인트 위치(m) -> 지금 대응 중인 관 내경(mm)."""
    return 2.0 * (x_m * 1000.0 + WHEEL_OUTER_MM)


print("\n" + "=" * 74)
print("leg 스프링 데모 — 관경을 300~342mm 로 왕복시킨다")
print(f"  스트로크 {LIMIT_LO_M:+.3f}~{LIMIT_HI_M:+.3f} m "
      f"= 관경 {joint_m_to_bore(LIMIT_LO_M):.0f}~{joint_m_to_bore(LIMIT_HI_M):.0f} mm")
print("  이 범위 밖에서는 다리가 한계에 걸려 더 움직이지 않는다 ([한계] 표시)")
print("=" * 74)
print(f"{'프레임':>6} {'목표관경':>9} {'실제관경':>9} {'다리위치':>10} {'벽면힘':>9} {'본체표류':>8}")

body = SingleRigidPrim(prim_path="/Scene/Robot/body", name="body")
body_home = body.get_world_pose()[0].copy()

frame = 0
MAX_FRAMES = PERIOD_FRAMES * 3 if HEADLESS else None

while simulation_app.is_running():
    phase = 2.0 * math.pi * frame / PERIOD_FRAMES
    bore = BORE_MIN_MM + (BORE_MAX_MM - BORE_MIN_MM) * 0.5 * (1.0 - math.cos(phase))

    # 그 관경에서 관벽이 다리를 미는 힘. 스프링과 평형을 이루는 값.
    x_want = bore_to_joint_m(bore)
    wall_force = SPRING_K * (x_want - SPRING_TARGET_M)

    eff = [0.0] * len(dof)
    for k in leg_idx:
        eff[k] = wall_force
    art.set_joint_efforts(eff)

    world.step(render=not HEADLESS)

    if frame % 20 == 0:
        pos = art.get_joint_positions()
        x_actual = sum(float(pos[k]) for k in leg_idx) / 3.0
        clamped = x_actual >= LIMIT_HI_M - 1e-4 or x_actual <= LIMIT_LO_M + 1e-4
        # 본체가 고정돼 있는지 같이 확인한다. 이 값이 변하면 표류하는 것.
        drift = max(abs(float(v)) for v in (body.get_world_pose()[0] - body_home))
        print(f"{frame:>6} {bore:>8.0f}mm {joint_m_to_bore(x_actual):>8.0f}mm "
              f"{x_actual:>+9.4f}m {wall_force:>+8.1f}N {drift * 1000:>7.2f}mm"
              f"{'  [한계]' if clamped else ''}")

    frame += 1
    if MAX_FRAMES is not None and frame >= MAX_FRAMES:
        break

print("=" * 74)
simulation_app.close()
