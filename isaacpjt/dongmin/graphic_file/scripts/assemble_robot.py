"""배관 점검 로봇 조립 — body / leg x3 / wheel x9 를 robot_img.png 형상으로 결합.

파츠 배치값은 추측이 아니라 STL 정점을 직접 파싱해 실측한 값이다 (단위 mm, CATIA 어셈블리 좌표계).
  body  : 튜브 r=48, Z가 축. Z~0 에 r=88 보스 3개 (0deg / 120deg / -120deg)
  leg   : x=57~142 로 이미 반경 방향에 배치됨. 바퀴 슬롯 x=132, Z=-88/0/+88
  wheel : r=23, 두께 20, 회전축 Y. 원점 중심

따라서 body 와 leg 는 변환 없이 그대로, leg 를 Z축 120deg 씩 회전 복사하고
바퀴를 슬롯 위치로 옮기면 조립이 끝난다.

실행:
  PYTHONUNBUFFERED=1 isaac_python assemble_robot.py            # GUI
  PYTHONUNBUFFERED=1 isaac_python assemble_robot.py --headless # 좌표 검증만
"""

import sys

HEADLESS = "--headless" in sys.argv

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": HEADLESS})

from pathlib import Path
import time

import omni.usd
from pxr import Usd, UsdGeom, Gf

HERE = Path(__file__).resolve().parent
BODY_USD = str(HERE / "body.usd")
LEG_USD = str(HERE / "leg.usd")
WHEEL_USD = str(HERE / "wheel.usd")

# ── STL 실측값 (mm) ──────────────────────────────────────────────
MM = 0.001                          # mm -> m
LEG_ANGLES = [0.0, 120.0, 240.0]    # body 보스 3개의 실측 각도
WHEEL_X = 132.0                     # leg 바퀴 슬롯 중심 X
WHEEL_Z = [-88.0, 0.0, 88.0]        # leg 바퀴 슬롯 중심 Z

ROBOT = "/World/Robot"

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

UsdGeom.Xform.Define(stage, "/World")

# 어셈블리 루트에서 한 번에 mm -> m 스케일. 하위는 전부 mm 단위로 다룬다.
robot = UsdGeom.Xform.Define(stage, ROBOT)
robot.AddScaleOp().Set(Gf.Vec3f(MM, MM, MM))


def add_ref(path, usd_path):
    """reference 는 항상 별도 자식 프림에 건다.

    변환을 얹은 프림에 직접 reference 를 걸면 참조 레이어의 xformOpOrder 가
    우리가 지정한 op 를 덮어쓸 수 있다.
    """
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    prim.GetReferences().AddReference(usd_path)
    return prim


# body 는 어셈블리 원점 그대로
add_ref(f"{ROBOT}/body", BODY_USD)

# leg 3개 — Z축 회전만 주면 leg 자체 좌표(x=57~142)가 그대로 맞는다
for i, angle in enumerate(LEG_ANGLES):
    leg_xf = UsdGeom.Xform.Define(stage, f"{ROBOT}/leg_{i}")
    leg_xf.AddRotateZOp().Set(angle)
    add_ref(f"{ROBOT}/leg_{i}/leg_mesh", LEG_USD)

    # 바퀴는 leg 의 자식이므로 회전은 부모가 처리, 여기서는 슬롯 위치만
    for j, z in enumerate(WHEEL_Z):
        w_xf = UsdGeom.Xform.Define(stage, f"{ROBOT}/leg_{i}/wheel_{j}")
        w_xf.AddTranslateOp().Set(Gf.Vec3d(WHEEL_X, 0.0, z))
        add_ref(f"{ROBOT}/leg_{i}/wheel_{j}/wheel_mesh", WHEEL_USD)

for _ in range(30):
    simulation_app.update()

# ── 검증: 실제 world bbox 가 STL 실측값 / 0.001 과 맞는지 ────────
EXPECTED = {
    f"{ROBOT}/body": ((-0.0548, -0.0818, -0.1980), (0.0870, 0.0818, 0.1955)),
    # leg_0 자체가 아니라 leg_mesh 를 본다. leg_0 의 bbox 에는 자식인 바퀴가 포함돼
    # x 최대가 0.155(바퀴 외곽)까지 늘어나기 때문.
    f"{ROBOT}/leg_0/leg_mesh": ((0.0570, -0.0170, -0.1175), (0.1420, 0.0170, 0.1175)),
    f"{ROBOT}/leg_0/wheel_2": ((0.1090, -0.0100, 0.0650), (0.1550, 0.0100, 0.1110)),
}

cache = UsdGeom.BBoxCache(
    Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
)

print("\n" + "=" * 78)
print("world bbox 검증 (단위 m)")
print("=" * 78)
ok = True
for path, (exp_min, exp_max) in EXPECTED.items():
    rng = cache.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
    got_min, got_max = rng.GetMin(), rng.GetMax()
    diff = max(
        max(abs(got_min[k] - exp_min[k]) for k in range(3)),
        max(abs(got_max[k] - exp_max[k]) for k in range(3)),
    )
    mark = "OK " if diff < 1e-3 else "FAIL"
    if diff >= 1e-3:
        ok = False
    print(f"[{mark}] {path}")
    print(f"       기대 min={tuple(round(v, 4) for v in exp_min)}  max={tuple(round(v, 4) for v in exp_max)}")
    print(f"       실제 min={tuple(round(v, 4) for v in got_min)}  max={tuple(round(v, 4) for v in got_max)}")
    print(f"       최대오차 {diff * 1000:.2f} mm")

full = cache.ComputeWorldBound(stage.GetPrimAtPath(ROBOT)).ComputeAlignedRange()
print(f"\n전체 로봇 bbox: min={tuple(round(v, 4) for v in full.GetMin())} "
      f"max={tuple(round(v, 4) for v in full.GetMax())}")
print(f"바퀴 최외곽 반경 = {WHEEL_X + 23.0:.0f} mm  ->  통과 가능 최소 배관 내경 "
      f"{2 * (WHEEL_X + 23.0):.0f} mm (레그 고정 기준)")

if not ok:
    print("\n[경고] bbox 불일치. STL->USD 변환 시 up-axis 보정 회전이 들어갔을 수 있다.")
    print("       변환된 USD 의 upAxis/metersPerUnit 을 확인할 것.")
print("=" * 78 + "\n")

if not HEADLESS:
    print("GUI 실행 중 — 창을 닫으면 종료됩니다.")
    while simulation_app.is_running():
        simulation_app.update()
        time.sleep(0.016)

simulation_app.close()
