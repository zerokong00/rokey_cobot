#!/usr/bin/env python3
"""맵·로봇 **주행 연습장** — 벨로우즈 12륜 + CAD 배관 3종.  [Isaac 3.11]

`repair_demo.py` 의 주행 부분만 떼어 온 **연습 파일**이다. 로봇은 **v2**
(`robot_v2/robot_v2_12wheel.usda`)를 쓴다 — 현역 시연은 아직 v1 이다.
아래 둘이 없다:

  ❌ **물이 없다** — `--water`(PBD 입자)도 `--fluid`(해석 항력)도, 시각 물 층도
     통째로 뺐다. 관 상태는 **배수(건식)** 고정이라 마찰은 0.40/0.35 다.
  ❌ **결함이 없다** — 관통 개구·비드·마개·용접 시퀀스·카메라 검출이 전부 없다.
     여기는 고치는 곳이 아니라 **굴려 보는 곳**이다.

배관 3개를 한 씬에 나란히 깐다. CAD 원본은 2개이고, 곡관을 두 자세로 쓴다:

  ┌ elbow_v  fixed_pipe.stp  — **수직 곡관**. 직관으로 가다 위로 꺾인다
  ├ elbow_h  fixed_pipe.stp  — **수평 곡관**. 같은 관을 진행축으로 90° 굴렸다
  └ tee      tshape_test.stp — T 분기. 직관 500mm + 위로 250mm 가지

**기본은 3대 동시**다 — 코스마다 v2 로봇이 한 대씩 들어가 각자 제 관을
**왕복 반복**한다(끝 → 복귀 → 다시). 끼이면 방향을 뒤집어 빠져나온다.
한 대만 보고 싶으면 `--course elbow_v` 처럼 고른다.

실행:
  isaac_python map_test_demo.py --headless                 # 로그로 확인
  DISPLAY=:1 isaac_python map_test_demo.py --hold          # GUI, 창 닫을 때까지
  DISPLAY=:1 isaac_python map_test_demo.py --course tee --hold
  isaac_python map_test_demo.py --course elbow_h --headless

옵션
  --course {all,elbow_v,elbow_h,tee}  로봇을 넣을 관 (기본 all = 3대)
  --glass        배관을 반투명으로 — 밖에서 로봇이 보인다 (물리 영향 0)
  --steps N      최대 물리 스텝 (기본 9000)
  SPEED_MPS=0.10 주행 속도. WHEEL_MAXFORCE=0.14 휠 토크. PHYSICS_HZ=240

🚨 배관 USD 는 `maps/*.usd` 다 — `tools/step_to_usd.py` 가 STEP 에서 구운 것이고
   `.gitignore` 의 `*.usd` 때문에 git 으로는 안 따라간다. 없으면 다시 구울 것:
     isaac_python tools/step_to_usd.py ~/Downloads/fixed_pipe.stp
     isaac_python tools/step_to_usd.py ~/Downloads/tshape_test.stp
"""

import json
import math
import os
import struct
import sys
from pathlib import Path

HEADLESS = "--headless" in sys.argv
HOLD = "--hold" in sys.argv
GLASS = "--glass" in sys.argv
# 기본은 **all** — 코스 3개에 로봇 3대를 동시에 굴린다.
COURSE = "all"
if "--course" in sys.argv:
    COURSE = sys.argv[sys.argv.index("--course") + 1]
STEPS = 9000
if "--steps" in sys.argv:
    STEPS = int(sys.argv[sys.argv.index("--steps") + 1])
# 🚨 이 파일은 argparse 를 안 쓴다(repair_demo 와 같은 방식). 그래서 오타난
#    플래그가 **조용히 무시된다** — `--know-defect` 를 줬는데 아무 일도 안
#    일어난 사고가 실제로 있었다. 모르는 `--` 인자는 여기서 잘라낸다.
_KNOWN = {"--headless", "--hold", "--glass", "--course",
          "--steps"}
_bad = [a for i, a in enumerate(sys.argv[1:], 1)
        if a.startswith("--") and a not in _KNOWN]
if _bad:
    raise SystemExit(f"[중단] 모르는 인자 {_bad} — 쓸 수 있는 것: "
                     f"{sorted(_KNOWN)}")

from isaacsim import SimulationApp                        # noqa: E402

simulation_app = SimulationApp({"headless": HEADLESS})

import numpy as np                                        # noqa: E402
from isaacsim.core.api import World                       # noqa: E402
from isaacsim.core.prims import SingleArticulation        # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from pxr import (Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux,  # noqa: E402
                 UsdPhysics, UsdShade)

MM = 0.001
PHYSICS_HZ = float(os.environ.get("PHYSICS_HZ", 240))
PHYSICS_DT = 1.0 / PHYSICS_HZ
SON = Path(__file__).resolve().parent
# 🔑 **이 연습장은 용접기 내장 v2 로봇을 쓴다** (2026-08-06 사용자 지시).
#    현역 시연(`repair_demo`)은 아직 `robot_bellows/robot_bellows.usda` 다.
# 🚨 이 usda 는 **씬 파일**이다 — `/World` 밑에 PhysicsScene·Looks 까지 들어
#    있다. 통째로 참조하면 물리 씬이 둘이 된다 → **`/World/Robot` 서브트리만**
#    참조한다(AddReference 의 두 번째 인자).
# 🚨 앞 세대(벨로우즈 12륜)와 **구조가 다르다.** 옮겨 쓸 때 걸린 것들:
#      관 축   로컬 **X** (벨로우즈는 Z) → 배치 회전이 필요 없다
#      휠      반경 **8mm** (벨로우즈 10mm), 중심 반경 40mm, 12개
#      링크    RearBody/FrontBody + <바디>_<A|B><0..2>_{Arm,Wheel}
#      용접기  **내장** (WeldRing·WeldTorch + RingRotate·TorchExtend 조인트)
#      질량    2.04kg (벨로우즈 0.91kg)
# 🔑 **현역은 벨로우즈 v2 다** (2026-08-06 실측 결론). 용접기 내장판
#    (`robot_from_bot_welder_art_v2.usda`)은 충돌체가 전부 **실린더
#    프리미티브**라 Isaac 에서 못 쓴다 — 자체 충돌에서 PhysX 가 세그폴트
#    (`contactConvexCoreConvex`)를 내고, 꺼도 관벽에 안 잡혀 떨어진다.
#    usda 쪽에서 충돌체를 **메시 convexHull** 로 바꿔 주면 그때 갈아끼운다:
#      ROBOT_USD=robot_v2/robot_from_bot_welder_art_v2.usda \
#      ROBOT_PRIM=/World/Robot AXIS_X=1 WHEEL_RAD=0.008
ROBOT_USDA = os.environ.get(
    "ROBOT_USD", str(SON / "robot_v2" / "robot_v2_12wheel.usda"))
ROBOT_PRIM_IN_USDA = os.environ.get("ROBOT_PRIM", "")
MAPS = SON / "maps"

# 🔑 용접기는 **로봇 usda 안에 이미 있다** — `welder/torch_spec.py` 로 따로
#    조립하지 않는다(2026-08-06 사용자 지시). 그래서 parts_meta 도 안 읽는다.

PIPE_IR = 0.050
WHEEL_R = float(os.environ.get("WHEEL_RAD", 0.010))   # 벨로우즈 v2
SEG_GAP = 0.076

# 주행 속도 — **0.10 m/s.** repair_demo 설계값(0.05)의 2배다.
# 🔑 여기는 검출을 안 하므로 "카메라 10Hz 로 결함을 놓치지 않는" 제약이 없다.
#    수직곡관은 0.05 로는 토크를 올려도 못 올라간다(아래 실측표).
TARGET_SPEED_MPS = float(os.environ.get("SPEED_MPS", 0.10))
SPIN_DEG_S = math.degrees(TARGET_SPEED_MPS / WHEEL_R)
# 감지 밴드 — 한 스텝 이동이 이보다 크면 바퀴가 관벽을 뛰어넘는다.
# 물이 없으므로 CPU 물리다 → GPU 하한(0.02) 을 깔 이유가 없다.
# 🚨 이 로봇은 2.04kg 으로 앞 세대(0.98kg)의 두 배다. 감지 밴드가 좁으면
#    관벽을 뛰어넘어 빠져나간다 — 필요하면 CONTACT_OFF 로 넓힌다.
CONTACT_OFFSET = float(os.environ.get(
    "CONTACT_OFF", max(0.0005, 1.2 * TARGET_SPEED_MPS / PHYSICS_HZ)))
REST_OFFSET = 0.0

# ── 휠 토크 한계 — **수직곡관을 오르려면 올려야 한다** (2026-08-06 실측) ──
# v2 usda 기본값은 0.014175 N·m/휠 → 12개 합산 견인력 17.0N, 로봇 중량 9.56N.
# 여유가 1.8배뿐이라 마찰·벨로우즈 저항을 빼면 수직 구간에서 남는 게 없다.
#
#   토크        속도        수직곡관 결과
#   0.014175    0.05        끼임 90mm
#   0.014175    0.10        끼임 90mm      ← 속도만 올리면 소용없다
#   0.0425      0.05        **발산**(로봇이 8m 밖으로) ← 중간값에 불안정 구간
#   0.14        0.05        끼임 135mm     ← 더 가지만 못 넘는다
#   0.14        0.10        **끝까지 296mm ✅**
#
# 🔑 **usda 는 안 고친다.** 로봇 설계값(v2 원본)을 그대로 두고 이 연습장에서만
#    덮어쓴다 — 되돌리기 쉽고, 다른 시연이 조용히 영향받지 않는다.
# 🚨 0.0425 에서 솔버가 터진 적이 있다. 이 값을 손으로 바꿀 때는 중간값에서
#    발산할 수 있다는 것을 알고 볼 것.  WHEEL_MAXFORCE 로 조절.
# 🔑 **0.05 N·m — 수직·수평 곡관을 둘 다 넘는 값**(2026-08-06 실측).
#    원본 0.014175 는 수직에서 90mm 에 멈추고, 0.14~0.28 은 수평에서 막힌다.
#      토크        속도   수직곡관   수평곡관
#      0.014175    0.05   끼임 90    ✅
#      0.014175    0.10   끼임 90    ✅
#      0.03        0.10   ✅         (미시험)
#      0.05        0.10   ✅         ✅      ← 채택
#      0.14        0.10   ✅         ✅
#      0.28        0.10   ✅         ❌ 190mm
WHEEL_MAXFORCE = float(os.environ.get("WHEEL_MAXFORCE", 0.05))

ROBOT = "/World/Robot"

world = World(stage_units_in_meters=1.0,
              physics_dt=PHYSICS_DT, rendering_dt=1.0 / 60.0)
stage = world.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage, "/World")

light = UsdLux.SphereLight.Define(stage, "/World/Light")
light.CreateIntensityAttr(3e6)
light.CreateRadiusAttr(0.05)
UsdGeom.Xformable(light).AddTranslateOp().Set(Gf.Vec3d(-0.2, -0.4, 0.6))


def rot(deg, axis):
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(Gf.Vec3d(*axis), deg))
    return m


def trans(x, y, z):
    m = Gf.Matrix4d(1.0)
    m.SetTranslate(Gf.Vec3d(x, y, z))
    return m


def scale(s):
    m = Gf.Matrix4d(1.0)
    m.SetScale(Gf.Vec3d(s, s, s))
    return m


# ── 코스 3종 ────────────────────────────────────────────────────────
# CAD 실측(STEP 곡면 + 변환 USD 정점, 단위 mm):
#   fixed_pipe  내반경 50 / 외 55, 굽힘 R150(LR).  중심선 =
#     (0,350,0) →(−Y 진행)→ (0,150,0) → R150 원호 → (0,0,150) →(+Z)→ (0,0,250)
#   tshape_test 내반경 50 / 외 55.  중심선 = (0,−250,0) ↔ (0,250,0) 직관,
#     y=0 에서 +Z 로 250 올라가는 가지
#
# 🔑 **세 코스 모두 입구 직관이 월드 +X 를 향하도록 눕힌다.** 그러면 로봇
#    배치(PLACE)와 진행 판정이 repair_demo 와 한 글자도 안 달라진다.
#    Rz(+90°) 이 원본의 −Y 진행을 +X 로 보낸다.
# 🔑 **수평 곡관은 같은 관을 진행축으로 굴린 것**이다 — Rx(90°) 은 월드 X 축
#    (=진행 방향)을 그대로 두고 위로 가던 출구만 −Y 로 눕힌다. 관을 새로
#    만들지 않는다("있는 것을 고칠 수 있는지 먼저 본다").
# 🚨 변환 USD 는 `metersPerUnit 0.001` 이다 — 미터 스테이지에 그냥 참조하면
#    1000배로 들어온다. 참조 Xform 에 scale 0.001 을 명시한다(real_map_demo
#    와 같은 처리).
_ELBOW = MAPS / "fixed_pipe.usd"
_TEE = MAPS / "tshape_test.usd"

# 코스별 중심선 — **원본 좌표(mm)로** 적고, 아래에서 코스 변환을 그대로 먹인다.
# 손으로 월드 좌표를 다시 쓰면 회전을 두 번 계산하게 되어 어긋난다.
_ARC_R, _ARC_N = 150.0, 48


def _elbow_centerline():
    pts = [(0.0, y, 0.0) for y in np.linspace(350.0, 150.0, 41)]
    # 원호 중심 (y, z) = (150, 150). 진행이 −Y 이고 +Z 로 꺾이므로
    #   P(t) = 중심 + R·(0, −sin t, −cos t)   t: 0 → π/2
    #   t=0  → (0, 150,   0)  직관 끝
    #   t=π/2→ (0,   0, 150)  수직관 시작
    cy, cz = 150.0, 150.0
    for t in np.linspace(0.0, math.pi / 2, _ARC_N)[1:]:
        pts.append((0.0, cy - _ARC_R * math.sin(t),
                    cz - _ARC_R * math.cos(t)))
    pts += [(0.0, 0.0, z) for z in np.linspace(150.0, 250.0, 21)[1:]]
    return np.array(pts)


def _tee_centerline():
    return np.array([(0.0, y, 0.0) for y in np.linspace(250.0, -250.0, 101)])


COURSES = {
    # 이름:      (USD, 코스 변환(원본 mm → 월드 m), 중심선(원본 mm), 설명)
    "elbow_v": (_ELBOW, scale(MM) * rot(90, (0, 0, 1)) * trans(0.0, 0.0, 0.0),
                _elbow_centerline(), "수직 곡관 — 직관 200mm 뒤 위로 꺾인다"),
    "elbow_h": (_ELBOW,
                scale(MM) * rot(90, (0, 0, 1)) * rot(90, (1, 0, 0))
                * trans(0.0, 0.9, 0.0),
                _elbow_centerline(), "수평 곡관 — 같은 관을 진행축으로 90° 굴렸다"),
    # 🔑 **T 분기는 눕혀 둔다** (2026-08-06 사용자 지시: "얘는 수직으로 있을 일이
    #    없어"). 원본은 가지가 +Z(위)로 서 있어서 곡관과 같은 Rx(90°) 을 먹여
    #    **가지까지 수평면**에 눕힌다 — 본관은 +X, 가지는 −Y 로 뻗는다.
    "tee": (_TEE,
            scale(MM) * rot(90, (0, 0, 1)) * rot(90, (1, 0, 0))
            * trans(0.0, -0.9, 0.0),
            _tee_centerline(), "T 분기 — 본관 500mm 수평, 가지도 수평(−Y)"),
}
if COURSE != "all" and COURSE not in COURSES:
    raise SystemExit(f"[중단] --course 는 all 또는 {list(COURSES)} 중 하나여야 "
                     f"한다 (받은 값: {COURSE!r})")
for _nm, (_usd, _x, _c, _d) in COURSES.items():
    if not _usd.is_file():
        raise SystemExit(
            f"[중단] 배관 USD 가 없다: {_usd}\n"
            f"        isaac_python tools/step_to_usd.py "
            f"~/Downloads/{_usd.stem}.stp 로 먼저 구울 것")


def xf_pts(mat, pts_mm):
    """원본 좌표(mm) 배열 → 월드(m). 코스 변환을 점에도 똑같이 먹인다."""
    return np.array([list(mat.Transform(Gf.Vec3d(*map(float, p))))
                     for p in pts_mm])


class Centerline:
    """코스 중심선 — 진행거리 s 와 중심선까지의 거리를 준다."""

    def __init__(self, pts):
        self.p = pts
        d = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        self.s = np.concatenate([[0.0], np.cumsum(d)])
        self.total = float(self.s[-1])

    def nearest(self, q):
        i = int(np.argmin(np.linalg.norm(self.p - q, axis=1)))
        return float(self.s[i]), float(np.linalg.norm(self.p[i] - q)), i

    def tangent(self, i):
        j = min(i + 1, len(self.p) - 1)
        k = max(j - 1, 0)
        v = self.p[j] - self.p[k]
        return v / max(np.linalg.norm(v), 1e-12)


# ── 물리 재질 — 물이 없으므로 **배수(건식)** 고정 ────────────────────
FRICTION_STATIC, FRICTION_DYNAMIC = 0.40, 0.35
_pm = UsdPhysics.MaterialAPI.Apply(
    UsdShade.Material.Define(stage, "/World/PipePhysMat").GetPrim())
_pm.CreateStaticFrictionAttr(FRICTION_STATIC)
_pm.CreateDynamicFrictionAttr(FRICTION_DYNAMIC)
_pm.CreateRestitutionAttr(0.0)

_gl = None
if GLASS:
    _gl = UsdShade.Material.Define(stage, "/World/Glass")
    _gs = UsdShade.Shader.Define(stage, "/World/Glass/Shader")
    _gs.CreateIdAttr("UsdPreviewSurface")
    _gs.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.55, 0.58, 0.60))
    _gs.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.25)
    _gs.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
    _gl.CreateSurfaceOutput().ConnectToSource(_gs.ConnectableAPI(), "surface")

# ── 배관 적재 ───────────────────────────────────────────────────────
print("=" * 78)
print(f"[코스] 배관 3개 적재 — 로봇 투입 {COURSE}")
paths = {}
for name, (usd, xform, cl_mm, desc) in COURSES.items():
    root_path = f"/World/Pipe_{name}"
    root = stage.DefinePrim(root_path, "Xform")
    root.GetReferences().AddReference(str(usd))
    UsdGeom.Xformable(root).AddTransformOp().Set(xform)
    # 🚨 인스턴스 프로토타입이면 콜라이더 부여가 안 먹는다 — 먼저 푼다.
    while True:      # 풀면 자식이 새로 드러나므로 더 나올 게 없을 때까지 돈다
        inst = [p for p in stage.Traverse()
                if p.IsInstance() and str(p.GetPath()).startswith(root_path)]
        if not inst:
            break
        for p in inst:
            p.SetInstanceable(False)
    meshes = [p for p in stage.Traverse()
              if p.IsA(UsdGeom.Mesh) and str(p.GetPath()).startswith(root_path)]
    n_col = n_skip = 0
    for p in meshes:
        # 🚨 `Sweep` 은 관 **속살(내면)만** 따로 들어 있는 면체(surface body)라
        #    PartBody 의 내면과 완전히 겹친다. 둘 다 콜라이더로 주면 같은 자리에
        #    접촉이 두 번 생기고, 렌더도 z-fighting 이 난다 → 표시·충돌 둘 다 뺀다.
        if "Sweep" in str(p.GetPath()):
            UsdGeom.Imageable(p).MakeInvisible()
            n_skip += 1
            continue
        UsdPhysics.CollisionAPI.Apply(p)
        # 🚨 배관은 반드시 approximation="none". convexHull 이면 관 속이 꽉 찬다.
        UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr("none")
        UsdGeom.Mesh(p).CreateDoubleSidedAttr(True)   # 단면 메시면 밖에서 투명
        UsdShade.MaterialBindingAPI.Apply(p).Bind(
            UsdShade.Material.Get(stage, "/World/PipePhysMat"),
            bindingStrength=UsdShade.Tokens.weakerThanDescendants,
            materialPurpose="physics")
        if GLASS:
            UsdShade.MaterialBindingAPI.Apply(p).Bind(
                _gl, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
        n_col += 1
    cl = Centerline(xf_pts(xform, cl_mm))
    paths[name] = cl
    print(f"  {name:8s} {usd.name:18s} 메시 {len(meshes)} "
          f"(콜라이더 {n_col} / 중복면 {n_skip} 제외)  중심선 "
          f"{cl.total * 1000:.0f}mm  입구 ({cl.p[0][0] * 1000:+.0f}, "
          f"{cl.p[0][1] * 1000:+.0f}, {cl.p[0][2] * 1000:+.0f})mm → 출구 "
          f"({cl.p[-1][0] * 1000:+.0f}, {cl.p[-1][1] * 1000:+.0f}, "
          f"{cl.p[-1][2] * 1000:+.0f})mm")
    print(f"           {desc}")
print(f"[준비] 관 상태 **배수(건식)** — 마찰 {FRICTION_STATIC}/"
      f"{FRICTION_DYNAMIC}. 이 연습장에는 물이 없다"
      + ("   배관 표시 반투명(유리)" if GLASS else ""))

# ── 로봇 3대 — **코스마다 한 대씩 동시에 굴린다** (2026-08-06 사용자 지시) ──
# 🔑 관을 하나씩 갈아 끼우며 세 번 돌리면 세 배 걸린다. 셋을 한 씬에 같이
#    올리고 각자 제 코스를 왕복하게 둔다. 물리는 어차피 한 스텝에 전부 푼다.
# 🔑 **왕복 반복**: 끝에 닿으면 되돌아오고, 되돌아오면 다시 간다. 끼이면
#    방향을 뒤집어 빠져나온다(임무 규칙 8 "끼임 → 후진 재시도"의 연습장판).
#    사람이 볼 때까지 계속 돈다.

def load_stl(path):
    data = Path(path).read_bytes()
    n = struct.unpack("<I", data[80:84])[0]
    a = np.frombuffer(data[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    tri = a[:, 12:48].copy().view("<f4").reshape(n * 3, 3).astype(np.float64)
    pts, inv = np.unique(np.round(tri, 5), axis=0, return_inverse=True)
    return pts * MM, inv.reshape(n, 3)


def make_mesh(path, stl, color=None, xform=None):
    pts, idx = load_stl(stl)
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    mesh.CreateFaceVertexCountsAttr([3] * len(idx))
    mesh.CreateFaceVertexIndicesAttr(idx.reshape(-1).tolist())
    mesh.CreateExtentAttr([Gf.Vec3f(*pts.min(0)), Gf.Vec3f(*pts.max(0))])
    mesh.CreateSubdivisionSchemeAttr("none")
    if color:
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if xform is not None:
        UsdGeom.Xformable(mesh).AddTransformOp().Set(xform)
    return mesh


CM = SON / "camera" / "meshes"
_res = os.environ.get("CAM_RES", "640x360").lower().split("x")
CAM_W, CAM_H, CAM_HFOV = int(_res[0]), int(_res[1]), 140.0
F_PX = (CAM_W / 2.0) / math.radians(CAM_HFOV / 2.0)
# 🔑 **이 로봇은 관 축이 로컬 X 다** — 카메라도 ±X 를 본다(앞 세대는 ∓Z 였다).
#    실측: FrontBody 원점 x=+62mm, 로봇 앞끝 x=+94mm, 뒤끝 −94mm.
# 🚨 카메라를 본체 안에 박아 두면 안 된다(기록된 사고 — 관벽 화소 0). 앞끝
#    보다 앞으로 내고, 하우징은 센서 뒤 5mm 에 둔다.
CAM_SPECS = [
    # 이름, 부모 링크, 링크 로컬 x, 전방 여부
    ("front_camera", "FrontBody", +0.035, True),
    ("back_camera", "RearBody", -0.035, False),
]

# 출발점 — 입구에서 안쪽으로 120mm. 🚨 안착 중에 로봇이 뒤로 밀린다(피스톤
# 6개가 동시에 벽을 밀며 자세를 잡는 과도 구간, 토치를 달면 74mm). 입구에
# 너무 붙여 세우면 밀린 뒤 후방 휠이 관 밖 자유공간에 떠서 못 나간다.
START_S = 0.120

RUN_NAMES = list(COURSES) if COURSE == "all" else [COURSE]
robots = []



def cyl_to_hull(cprim, n=16):
    """실린더 프리미티브 콜라이더 → **convexHull 메시 콜라이더**로 교체.

    🚨 이 로봇은 휠 12개와 본체 2개의 충돌체가 `UsdGeom.Cylinder` 다. PhysX 5.4
       는 실린더를 **convex-core** 로 다루는데, 관 안에서 narrow phase 가
       `contactConvexCoreConvex` 에서 세그폴트를 냈고(코어 덤프), 안 죽는
       조건에서도 로봇이 관을 뚫고 나갔다. 우리 벨로우즈 로봇이 멀쩡했던 이유가
       휠이 **메시 convexHull** 이었기 때문이다(기록된 대조).
    → 같은 치수의 n각기둥 메시를 만들어 콜라이더를 옮기고, 원래 실린더는
      충돌만 끈다(형상·재질은 그대로 두어 보이는 것은 안 바뀐다).
    """
    cy = UsdGeom.Cylinder(cprim)
    r = float(cy.GetRadiusAttr().Get())
    h = float(cy.GetHeightAttr().Get())
    ax = str(cy.GetAxisAttr().Get() or "Z")
    pts, idx = [], []
    for s in (-h / 2.0, +h / 2.0):
        for k in range(n):
            a = 2.0 * math.pi * k / n
            c, d = r * math.cos(a), r * math.sin(a)
            pts.append({"X": (s, c, d), "Y": (c, s, d)}.get(ax, (c, d, s)))
    for k in range(n):
        k2 = (k + 1) % n
        idx += [[k, k2, n + k2], [k, n + k2, n + k]]
    for k in range(1, n - 1):                      # 양 끝 뚜껑
        idx += [[0, k, k + 1], [n, n + k + 1, n + k]]
    m = UsdGeom.Mesh.Define(stage, str(cprim.GetPath()) + "_hull")
    m.CreatePointsAttr([Gf.Vec3f(*q) for q in pts])
    m.CreateFaceVertexCountsAttr([3] * len(idx))
    m.CreateFaceVertexIndicesAttr([i for f in idx for i in f])
    m.CreateSubdivisionSchemeAttr("none")
    mp = m.GetPrim()
    UsdPhysics.CollisionAPI.Apply(mp)
    UsdPhysics.MeshCollisionAPI.Apply(mp).CreateApproximationAttr("convexHull")
    UsdGeom.Imageable(mp).MakeInvisible()           # 충돌 전용
    UsdPhysics.CollisionAPI.Apply(cprim).CreateCollisionEnabledAttr(False)
    return mp


def build_robot(name, path_cl):
    """코스 하나에 로봇 한 대. 반환 = FSM 상태를 담을 dict."""
    rp = f"/World/Robot_{name}"
    prim = stage.DefinePrim(rp, "Xform")
    # 🚨 씬 파일이라 **로봇 서브트리만** 참조한다(PhysicsScene 중복 방지).
    if ROBOT_PRIM_IN_USDA:
        prim.GetReferences().AddReference(ROBOT_USDA,
                                         Sdf.Path(ROBOT_PRIM_IN_USDA))
    else:
        prim.GetReferences().AddReference(ROBOT_USDA)

    # 🚨 **자체 충돌을 끈다.** 이 로봇의 휠은 실린더 프리미티브라 PhysX 가
    #    convex-core 로 다루는데, 관 안에서 암·휠이 서로 닿는 순간
    #    `contactConvexCoreConvex` 에서 **세그폴트**가 났다(코어 덤프 2회).
    #    관벽과의 접촉은 그대로 살아 있다 — 꺼지는 것은 로봇 자기들끼리다.
    PhysxSchema.PhysxArticulationAPI.Apply(prim) \
        .CreateEnabledSelfCollisionsAttr(False)

    _nh = 0
    for _c in Usd.PrimRange(prim):
        if _c.GetTypeName() == "Cylinder" and _c.HasAPI(UsdPhysics.CollisionAPI):
            cyl_to_hull(_c)
            _nh += 1
    if _nh:
        print(f"  {name}: 실린더 콜라이더 {_nh}개 → convexHull 메시로 교체")

    i0 = int(np.argmin(np.abs(path_cl.s - START_S)))
    p0 = path_cl.p[i0]
    tan = path_cl.tangent(i0)
    if abs(tan[0]) < 0.99:
        raise SystemExit(f"[중단] {name} 출발 접선이 +X 가 아니다 "
                         f"{np.round(tan, 3)} — 코스 변환이 틀어졌다")
    # 배치는 각 링크 로컬 변환에 굽는다(부모 Xform·root 에 걸면 PhysX 가 발산).
    # 🔑 **회전이 없다.** 이 로봇은 관 축이 로컬 X 이고 앞쪽이 +X 라, 진행
    #    방향 +X 와 이미 맞는다(앞 세대는 축이 Z 라 Ry(−90) 이 필요했다).
    place = (trans(float(p0[0]), float(p0[1]), float(p0[2]))
             if os.environ.get("AXIS_X", "0") == "1"
             else rot(-90, (0, 1, 0)) * trans(float(p0[0]), float(p0[1]),
                                              float(p0[2])))
    for child in list(prim.GetChildren()):
        if not child.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        xf = UsdGeom.Xformable(child)
        local = xf.GetLocalTransformation()
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local * place)

    # ── 카메라 하우징·조명 (센서 프림은 reset 뒤에) ────────────────
    for nm, seg, x, fwd in CAM_SPECS:
        if not stage.GetPrimAtPath(f"{rp}/{seg}").IsValid():
            continue
        base = f"{rp}/{seg}/{nm}_rig"
        UsdGeom.Xform.Define(stage, base)
        hp, hi = load_stl(CM / "camera_housing.stl")
        hm = UsdGeom.Mesh.Define(stage, f"{base}/housing")
        hm.CreatePointsAttr([Gf.Vec3f(*q) for q in hp])
        hm.CreateFaceVertexCountsAttr([3] * len(hi))
        hm.CreateFaceVertexIndicesAttr(hi.reshape(-1).tolist())
        hm.CreateExtentAttr([Gf.Vec3f(*hp.min(0)), Gf.Vec3f(*hp.max(0))])
        hm.CreateSubdivisionSchemeAttr("none")
        # son 하우징 STL 은 광축이 +X — 전방은 그대로, 후방만 Rz(180) 으로 돌린다.
        # 🚨 센서와 같은 자리에 두면 렌즈·조명을 가둔다 → 광축 뒤 5mm.
        hm_x = x + (-0.005 if fwd else 0.005)
        UsdGeom.Xformable(hm).AddTransformOp().Set(
            (Gf.Matrix4d(1.0) if fwd else rot(180, (0, 0, 1)))
            * trans(hm_x, 0, 0))
        # 관 내부는 로봇 조명이 유일한 광원. 센서보다 4mm 앞, 광축 둘레 대칭.
        for k in range(2):
            lg = UsdLux.SphereLight.Define(stage, f"{base}/light_{k}")
            lg.CreateIntensityAttr(4.0e5)
            lg.CreateRadiusAttr(0.002)
            UsdGeom.Xformable(lg).AddTranslateOp().Set(Gf.Vec3d(
                x + (0.004 if fwd else -0.004),
                0.012 * (1 if k == 0 else -1), 0.0))

    return {"name": name, "path": rp, "cl": path_cl, "i0": i0,
            "state": "SETTLE", "t": 0, "dir": +1, "lap": 0, "stuck": 0,
            "s_last": 0.0, "mark": 0, "dead": False, "art": None,
            "wheel": [], "seg1": None, "best": 0.0, "wheel_rad": 0.0}


for _nm in RUN_NAMES:
    robots.append(build_robot(_nm, paths[_nm]))
print(f"[준비] 로봇 {len(robots)}대 = {', '.join(r['name'] for r in robots)} "
      f"— 코스마다 한 대씩 동시에 굴린다")

# ── 휠 토크 한계 덮어쓰기 ───────────────────────────────────────────
# 🚨 **world.reset() 앞이어야 한다.** 시뮬이 시작된 뒤 USD 드라이브 속성을 쓰면
#    PhysX 로 안 넘어간다(기록된 함정).
_n_wf = 0
# 🚨 구동 조인트는 **`DriveJoints` 스코프 안**에 있다. 이름만 보면 안 된다 —
#    같은 이름(`RearBody_A0` …)이 `SpringJoints`(서스펜션 직동)에도 있어서
#    이름으로 고르면 서스펜션까지 같이 덮어쓴다.
for _r in robots:
    _dj0 = stage.GetPrimAtPath(f"{_r['path']}/DriveJoints")
    for _p in (_dj0.GetChildren() if _dj0.IsValid() else
               [q for q in stage.Traverse()
                if str(q.GetPath()).startswith(_r['path'])
                and "wheel" in q.GetName().lower()
                and q.IsA(UsdPhysics.RevoluteJoint)]):
        _d = UsdPhysics.DriveAPI.Get(_p, "angular")
        if _d:
            _d.CreateMaxForceAttr(WHEEL_MAXFORCE)
            _n_wf += 1
print(f"[준비] 휠 토크 한계 {WHEEL_MAXFORCE:.4f} N·m × {_n_wf}개 "
      f"→ 로봇 1대당 합산 견인력 "
      f"{12 * WHEEL_MAXFORCE / WHEEL_R:.1f}N (v2 원본 0.014175 = 17.0N)")

# ── 감지 밴드 (contactOffset) ───────────────────────────────────────
# 🚨 **반드시 여기서 돌린다** — 배관·로봇·토치가 전부 스테이지에 올라온 뒤이고
#    world.reset() 의 cook 보다는 앞이다. 배관 직후에 돌리면 로봇 usda 가 아직
#    AddReference 전이라 휠 콜라이더에 안 닿는다(기록된 사고).
# 🚨 **로봇 콜라이더는 건드리지 않는다** (2026-08-06 실측). 이 로봇의 휠은
#    실린더 프리미티브(r8 h12)라 PhysX 가 convex-core 로 다룬다. 거기에
#    contactOffset 0.5mm 를 강제로 씌웠더니 narrow phase 에서
#    `contactConvexCoreConvex` 로 **세그폴트**가 났다(코어 덤프).
#    → 로봇은 usda 가 저작한 값 그대로 두고, **배관에만** 건다.
_n_off = 0
for _p in stage.Traverse():
    if not str(_p.GetPath()).startswith("/World/Pipe_"):
        continue
    if _p.HasAPI(UsdPhysics.CollisionAPI) or _p.IsA(UsdGeom.Mesh):
        _px = PhysxSchema.PhysxCollisionAPI.Apply(_p)
        _px.CreateContactOffsetAttr(CONTACT_OFFSET)
        _px.CreateRestOffsetAttr(REST_OFFSET)
        _n_off += 1
_n_wheel = sum(1 for _p in stage.Traverse()
               if _p.GetName().endswith("_Wheel")
               and _p.HasAPI(PhysxSchema.PhysxCollisionAPI))
print(f"[준비] 주행 {TARGET_SPEED_MPS * 1000:.0f} mm/s "
      f"(한 스텝 {TARGET_SPEED_MPS / PHYSICS_HZ * 1000:.3f}mm) → "
      f"contactOffset {CONTACT_OFFSET * 1000:.2f}mm, 프림 {_n_off}개")
print(f"         (배관 프림에만 건다 — 로봇 콜라이더는 usda 값 그대로. "
      f"휠 콜라이더 {_n_wheel}개는 손대지 않았다)")

for r in robots:
    r["art"] = SingleArticulation(prim_path=r["path"], name=f"bot_{r['name']}")
    world.scene.add(r["art"])
world.reset()

for r in robots:
    dof = list(r["art"].dof_names or [])
    # 🚨 **DOF 는 이름으로 고를 수 없다** — 구동(회전)과 서스펜션(직동)이 같은
    #    이름을 쓴다. 조인트 프림 경로로 골라 그 이름을 DOF 목록에서 찾되,
    #    이름이 겹치면 개수로 걸러 낸다. 12개가 아니면 즉시 멈춘다.
    # 🚨 구동 조인트와 서스펜션이 **이름이 같다**(`RearBody_A0` …). Isaac 이
    #    뒤에 오는 쪽에 `_0` 을 붙여 구분하는데, **어느 쪽이 붙는지는 보장이
    #    없다.** 그래서 이름으로만 고르면 서스펜션(직동)을 돌리게 된다.
    #    → 이름(접미사 무시) **AND 회전 DOF** 로 고른다.
    _dj = stage.GetPrimAtPath(f"{r['path']}/DriveJoints")
    if _dj.IsValid():
        _base = {q.GetName() for q in _dj.GetChildren()}
        _types = list(r["art"]._articulation_view.get_dof_types())
        r["wheel"] = [k for k, n in enumerate(dof)
                      if n.split(":")[0].removesuffix("_0") in _base
                      and "ROTATION" in str(_types[k]).upper()]
    else:
        r["wheel"] = [k for k, n in enumerate(dof) if "_wheel_" in n]
    if len(r["wheel"]) != 12:
        print(f"[경고] {r['name']} 휠 DOF {len(r['wheel'])}개 "
              f"(12 이어야 한다). DOF 이름: {dof}")
    _fb = stage.GetPrimAtPath(f"{r['path']}/FrontBody")
    r["seg1"] = _fb if _fb.IsValid() else stage.GetPrimAtPath(
        f"{r['path']}/seg1_body")

# 카메라 프림은 **reset 뒤에** 만든다.
try:
    from isaacsim.sensors.camera import Camera             # noqa: E402
    _SQ = 0.70710678
    _ncam = 0
    for r in robots:
        for nm, seg, x, fwd in CAM_SPECS:
            if not stage.GetPrimAtPath(f"{r['path']}/{seg}").IsValid():
                continue
            cam = Camera(prim_path=f"{r['path']}/{seg}/{nm}_rig/{nm}",
                         translation=np.array([x, 0.0, 0.0]),
                         frequency=10, resolution=(CAM_W, CAM_H))
            cam.initialize()
            # 🚨 USD 카메라는 **로컬 −Z 를 본다.** 이 로봇의 전방은 +X 이므로
            #    항등이면 옆을 본다. 원하는 카메라 축(로봇 로컬 기준):
            #      전방  −Z_cam=+X, +Y_cam=+Z(화면 위)  → X_cam=−Y
            #      후방  −Z_cam=−X, +Y_cam=+Z          → X_cam=+Y
            #    행벡터 규약이라 행이 곧 카메라 축이다.
            _M = Gf.Matrix3d(0, -1, 0, 0, 0, 1, -1, 0, 0) if fwd \
                else Gf.Matrix3d(0, 1, 0, 0, 0, 1, 1, 0, 0)
            _q = Gf.Rotation(_M.ExtractRotation()).GetQuat()
            cam.set_local_pose(
                orientation=np.array(
                    [_q.GetReal(), *[float(v) for v in _q.GetImaginary()]]),
                camera_axes="usd")
            cam.set_clipping_range(0.005, 5.0)
            cam.set_focal_length(3.0 * F_PX * 1e-6)
            cam.set_horizontal_aperture(3.0 * CAM_W * 1e-6)
            try:
                cam.set_opencv_fisheye_properties(
                    cx=CAM_W / 2, cy=CAM_H / 2, fx=F_PX, fy=F_PX,
                    fisheye=[0.0, 0.0, 0.0, 0.0])
            except Exception as exc:
                print(f"[경고] {nm} 어안 설정 실패({exc}) — 핀홀로 진행")
            _ncam += 1
    print(f"[준비] 카메라 {_ncam}대 (어안 {CAM_HFOV:.0f}°, {CAM_W}x{CAM_H}) "
          f"— GUI 목록 이름은 로봇마다 같으니 경로로 고를 것 "
          f"(/World/Robot_<코스>/{{Front,Rear}}Body/..._rig/...)")
except Exception as exc:
    print(f"[경고] 카메라 초기화 실패 ({exc}) — 하우징·조명만 남는다")

_XC = UsdGeom.XformCache()


def wpos(prim):
    _XC.Clear()
    t = _XC.GetLocalToWorldTransform(prim).ExtractTranslation()
    return np.array([float(t[0]), float(t[1]), float(t[2])])


def drive(r, deg_s):
    r["art"].apply_action(ArticulationAction(
        joint_velocities=np.array([math.radians(deg_s)] * len(r["wheel"])),
        joint_indices=np.array(r["wheel"])))


print("=" * 78)
_m = sum(float(UsdPhysics.MassAPI(q).GetMassAttr().Get() or 0.0)
         for q in stage.Traverse()
         if str(q.GetPath()).startswith(robots[0]["path"] + "/")
         and q.HasAPI(UsdPhysics.RigidBodyAPI)
         and UsdPhysics.MassAPI(q).GetMassAttr())
print(f"로봇  1대 질량 {_m * 1000:.0f} g  중량 {_m * 9.81:.2f} N "
      f"(용접기 내장, 카메라 2대 부착)")
print(f"      주행 {TARGET_SPEED_MPS * 1000:.0f} mm/s  왕복 반복  "
      f"휠 토크 {WHEEL_MAXFORCE:.3f} N·m")
print("-" * 78)

import time                                              # noqa: E402
_t_mark, _step_mark = time.time(), 0
step, was_playing = 0, True
STUCK_S, REPORT_S = 2.0, 3.0
_last_report = 0

while True:
    world.step(render=not HEADLESS)
    step += 1

    if not HEADLESS:
        _playing = world.is_playing()
        if _playing and not was_playing:
            world.reset()
            for r in robots:
                r.update(state="SETTLE", t=0, dir=+1, lap=0, stuck=0,
                         s_last=0.0, mark=step, dead=False, wheel_rad=0.0)
            print("[재시작] Stop → Play 감지 — 처음부터 다시 돈다")
        was_playing = _playing
        if not simulation_app.is_running():
            break
        if not _playing:
            continue

    if step > STEPS and STEPS > 0 and not (HOLD and not HEADLESS):
        break

    if step - _step_mark >= 5 * PHYSICS_HZ:
        _dt = time.time() - _t_mark
        _rate = (step - _step_mark) / max(_dt, 1e-9)
        print(f"  [속도] 물리 {_rate:6.1f} step/s "
              f"(실시간 대비 {_rate / PHYSICS_HZ:4.2f}x, 로봇 {len(robots)}대)")
        _t_mark, _step_mark = time.time(), step

    for r in robots:
        if r["dead"]:
            continue
        r["t"] += 1
        p1 = wpos(r["seg1"])
        s_now, off_now, i_now = r["cl"].nearest(p1)

        if r["state"] == "SETTLE":
            if r["t"] > 1.5 * PHYSICS_HZ:
                print(f"[{r['name']:8s}] 안착 s={s_now * 1000:.1f}mm "
                      f"이탈 {off_now * 1000:.1f}mm — 주행 시작")
                r.update(state="RUN", t=0, s_last=s_now, mark=step)
            continue

        # ── RUN : 끝까지 갔다가 되돌아오고, 다시 간다 ──────────────
        drive(r, SPIN_DEG_S * r["dir"])
        r["wheel_rad"] += math.radians(SPIN_DEG_S) * PHYSICS_DT

        # 🚨 관 밖으로 튄 것은 되살리지 않는다 — 조용히 계속 돌면 로그가
        #    거짓말이 된다. 그 로봇만 멈추고 나머지는 계속 간다.
        if off_now > PIPE_IR + 0.03 or not np.isfinite(p1).all():
            print(f"[{r['name']:8s}] ❌ 코스 이탈 — 중심선에서 "
                  f"{off_now * 1000:.0f}mm. 이 로봇은 여기서 멈춘다")
            drive(r, 0.0)
            r["dead"] = True
            continue

        # 진행 방향 기준으로 나아갔는가
        adv = (s_now - r["s_last"]) * r["dir"]
        if adv > 0.003:
            r["s_last"], r["mark"] = s_now, step
        elif step - r["mark"] > STUCK_S * PHYSICS_HZ:
            # 끼임 → **방향을 뒤집어 빠져나온다** (임무 규칙 8 의 연습장판)
            r["stuck"] += 1
            r["dir"] *= -1
            r["s_last"], r["mark"] = s_now, step
            print(f"[{r['name']:8s}] ⚠ 끼임 {r['stuck']}회 "
                  f"(s={s_now * 1000:.0f}mm, 이탈 {off_now * 1000:.1f}mm) "
                  f"→ 방향 전환 {'전진' if r['dir'] > 0 else '후진'}")
            continue

        # 끝 / 시작에 닿으면 방향 전환 — 이것이 '왕복 1회'
        if r["dir"] > 0 and i_now >= len(r["cl"].p) - 2:
            r["lap"] += 1
            r["dir"] = -1
            r["s_last"], r["mark"] = s_now, step
            print(f"[{r['name']:8s}] ✅ 코스 끝 도달 (왕복 {r['lap']}) "
                  f"— 되돌아간다")
        elif r["dir"] < 0 and i_now <= r["i0"]:
            r["lap"] += 1
            r["dir"] = +1
            r["s_last"], r["mark"] = s_now, step
            print(f"[{r['name']:8s}] 🔁 출발점 복귀 "
                  f"(왕복 {r['lap']}) — 다시 간다")

    if step - _last_report >= REPORT_S * PHYSICS_HZ:
        _last_report = step
        _line = []
        for r in robots:
            p1 = wpos(r["seg1"])
            s_now, off_now, _i = r["cl"].nearest(p1)
            _line.append(f"{r['name']} s={s_now * 1000:5.0f}"
                         f"{'→' if r['dir'] > 0 else '←'}"
                         f" 왕복{r['lap']} 끼임{r['stuck']}"
                         + ("(정지)" if r["dead"] else ""))
        print(f"  [{step / PHYSICS_HZ:6.1f}s] " + " | ".join(_line))

    if all(r["dead"] for r in robots):
        print("[중단] 세 대 모두 코스를 벗어났다")
        break

print("=" * 78)
for r in robots:
    p1 = wpos(r["seg1"])
    s_now, off_now, _i = r["cl"].nearest(p1)
    print(f"결과  {r['name']:8s} 왕복 {r['lap']}회  끼임 {r['stuck']}회  "
          f"s={s_now * 1000:.0f}mm  이탈 {off_now * 1000:.1f}mm"
          + ("  ❌ 이탈로 정지" if r["dead"] else ""))
sys.stdout.flush()
simulation_app.close()
