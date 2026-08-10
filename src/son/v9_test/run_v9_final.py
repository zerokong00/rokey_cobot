#!/usr/bin/env python3
"""pipe_robot_v9 + real_final_script(v8) 검증 러너 — 기존 자율주행 코드와 완전 분리.

CAD 개발자가 준 두 파일(pipe_robot_v9.usda / real_final_script.py)이 "되는지"를
본다. real_final_script 는 **한 글자도 안 고치고** 그대로 import 한다 — 이 러너가
해 주는 것은 씬 구성과 이 PC 한정 안전장치뿐이다:

  1. 각진 T(tshape_test.usd)를 real_final_script 가 기대하는 경로
     `/World/test_pipe_tee_ID100` 에 올린다. 본관을 월드 X 로 돌려
     (Rz-90) 접합부가 원점에 오게 한다 → JUNCTION=(0,0,0).
  2. 입구에 legacy 직관 600mm 를 맞대 붙인다 — 로봇(~회전축 간 228mm)이
     본관 반쪽(250mm)보다 길어서, 조향 스케줄이 잠든 채 달릴 조주로가 없다.
  3. 로봇을 `/World/pipe_robot_v9` 에 **파일 통째로** 참조하고(머티리얼
     바인딩 보호) 중복 PhysicsScene 만 끈다.
  4. 🚨 실린더 콜라이더 → convexHull 메시 교체 + 자체충돌 OFF.
     이 PC 의 PhysX 5.4 는 실린더를 convex-core 로 다뤄 관 안 접촉에서
     `contactConvexCoreConvex` 세그폴트를 낸다(map_test_demo 실측 기록).
     로봇 튜닝값(드라이브 강성·토크)은 CAD 개발자 저작값 그대로 둔다.
  5. 짧은 테스트 규율: 정체 STUCK_S(3s)에 5mm 미만이면 즉시 실패 종료,
     발산(좌표 3m 초과·NaN) 즉시 종료, 시뮬 상한 SIM_MAX_S(25s).

검증 2페이즈:
  A. selftest — BendF 40° 지령 → 기하학적 굽힘각 실측.
     "시뮬 중 USD 드라이브 쓰기가 PhysX 에 먹는가"를 직접 확인한다
     (이 스크립트의 성립 전제. 안 먹으면 그 자체가 결론).
  B. go() — 전체 기동. 헤드가 가지(+Z)로 0.12m 오르면 진입 성공,
     본관 끝(x>0.23m)으로 직진하면 진입 실패.

실행:
  PYTHONUNBUFFERED=1 isaac_python run_v9_final.py
노브(env): HEADLESS=0 GUI 확인 / BRANCH_ANGLE(기본 90=위) /
  V9_WHEEL_CMD(기본 600 deg/s) / SIM_MAX_S / STUCK_S / STUCK_MM
"""
import math
import os
import struct
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
SON = HERE.parent
HEADLESS = os.environ.get("HEADLESS", "1") == "1"
SIM_MAX_S = float(os.environ.get("SIM_MAX_S", 25.0))
STUCK_S = float(os.environ.get("STUCK_S", 3.0))
STUCK_MM = float(os.environ.get("STUCK_MM", 5.0))
BRANCH_ANGLE = float(os.environ.get("BRANCH_ANGLE", 90.0))
V9_WHEEL_CMD = float(os.environ.get("V9_WHEEL_CMD", 0.0))  # 0 = 저작값 그대로

_T0 = time.time()


def say(msg):
    print(f"[러너 {time.time() - _T0:6.1f}s] {msg}", flush=True)


from isaacsim import SimulationApp                        # noqa: E402

simulation_app = SimulationApp({"headless": HEADLESS})
say("SimulationApp 기동")

import numpy as np                                        # noqa: E402
from isaacsim.core.api import World                       # noqa: E402
from pxr import (Gf, PhysxSchema, Usd, UsdGeom, UsdLux,   # noqa: E402
                 UsdPhysics, UsdShade)

MM = 0.001
PHYSICS_HZ = 240
world = World(stage_units_in_meters=1.0,
              physics_dt=1.0 / PHYSICS_HZ, rendering_dt=1.0 / 60.0)
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


def load_stl(path):
    data = Path(path).read_bytes()
    n = struct.unpack("<I", data[80:84])[0]
    a = np.frombuffer(data[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    tri = a[:, 12:48].copy().view("<f4").reshape(n * 3, 3).astype(np.float64)
    pts, inv = np.unique(np.round(tri, 5), axis=0, return_inverse=True)
    return pts * MM, inv.reshape(n, 3)


def pipe_collider(prim):
    UsdPhysics.CollisionAPI.Apply(prim)
    # 🚨 배관은 approximation="none" — convexHull 이면 관 속이 꽉 찬다.
    UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("none")
    # 🎯 **접촉 감지폭** (2026-08-09 — 우리 실측값을 얹는다). 설계 문서가
    #    *"기본값 0.02m 그대로 두면 로봇이 뜬다. 최빈 실패 원인"* 이라고
    #    명시한 항목인데 이 러너에는 빠져 있었다. 배관에만 건다 —
    #    로봇 콜라이더에 걸면 convex-core narrow phase 에서 세그폴트(기록).
    _px = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    _co = float(os.environ.get("V9_CONTACT", 0.0))   # 0 = 자산값 그대로
    if _co > 0:
        _px.CreateContactOffsetAttr(_co)
        _px.CreateRestOffsetAttr(0.0)
    UsdGeom.Mesh(prim).CreateDoubleSidedAttr(True)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        UsdShade.Material.Get(stage, "/World/PipePhysMat"),
        bindingStrength=UsdShade.Tokens.weakerThanDescendants,
        materialPurpose="physics")


def cyl_to_hull(cprim, n=16):
    """실린더·원뿔 프리미티브 콜라이더 → convexHull 메시로 교체.

    map_test_demo.cyl_to_hull 과 같은 처리(이식). PhysX convex-core 세그폴트
    회피. 물리 재질·접촉 오프셋을 같이 옮긴다.
    """
    is_cone = cprim.IsA(UsdGeom.Cone)
    g = UsdGeom.Cone(cprim) if is_cone else UsdGeom.Cylinder(cprim)
    r = float(g.GetRadiusAttr().Get())
    h = float(g.GetHeightAttr().Get())
    ax = str(g.GetAxisAttr().Get() or "Z")

    def put(s, c, d):
        return {"X": (s, c, d), "Y": (c, s, d)}.get(ax, (c, d, s))

    pts, idx = [], []
    if is_cone:
        for k in range(n):
            a = 2.0 * math.pi * k / n
            pts.append(put(-h / 2.0, r * math.cos(a), r * math.sin(a)))
        pts.append(put(+h / 2.0, 0.0, 0.0))
        for k in range(n):
            idx.append([k, (k + 1) % n, n])
        for k in range(1, n - 1):
            idx.append([0, k + 1, k])
    else:
        for s in (-h / 2.0, +h / 2.0):
            for k in range(n):
                a = 2.0 * math.pi * k / n
                pts.append(put(s, r * math.cos(a), r * math.sin(a)))
        for k in range(n):
            k2 = (k + 1) % n
            idx += [[k, k2, n + k2], [k, n + k2, n + k]]
        for k in range(1, n - 1):
            idx += [[0, k, k + 1], [n, n + k + 1, n + k]]
    m = UsdGeom.Mesh.Define(stage, str(cprim.GetPath()) + "_hull")
    m.CreatePointsAttr([Gf.Vec3f(*q) for q in pts])
    m.CreateFaceVertexCountsAttr([3] * len(idx))
    m.CreateFaceVertexIndicesAttr([i for f in idx for i in f])
    m.CreateSubdivisionSchemeAttr("none")
    mp = m.GetPrim()
    UsdPhysics.CollisionAPI.Apply(mp)
    UsdPhysics.MeshCollisionAPI.Apply(mp).CreateApproximationAttr("convexHull")
    UsdGeom.Imageable(mp).MakeInvisible()
    _src = cprim.GetRelationship("material:binding:physics")
    if _src and _src.GetTargets():
        mp.CreateRelationship("material:binding:physics").SetTargets(
            _src.GetTargets())
    _po = PhysxSchema.PhysxCollisionAPI.Get(stage, cprim.GetPath())
    if _po:
        _pn = PhysxSchema.PhysxCollisionAPI.Apply(mp)
        for _get, _set in ((_po.GetContactOffsetAttr, "CreateContactOffsetAttr"),
                           (_po.GetRestOffsetAttr, "CreateRestOffsetAttr")):
            _v = _get().Get() if _get() else None
            if _v is not None:
                getattr(_pn, _set)(_v)
    UsdPhysics.CollisionAPI.Apply(cprim).CreateCollisionEnabledAttr(False)
    return mp


# ── 배관 물리 재질 (배수/건식: 0.40/0.35) ───────────────────────────
_pm = UsdShade.Material.Define(stage, "/World/PipePhysMat").GetPrim()
_pmat = UsdPhysics.MaterialAPI.Apply(_pm)
_pmat.CreateStaticFrictionAttr(0.40)
_pmat.CreateDynamicFrictionAttr(0.35)

# ── 각진 T — real_final_script 가 기대하는 경로에, 접합부가 원점에 오게 ──
# tshape_test.usd 원본(mm): 본관 Y −250~+250, 가지 y=0 에서 +Z 250.
# Rz(-90) 이 본관을 월드 X(−250~+250)로 돌리고 가지는 +Z 그대로 선다.
# 🚨 metersPerUnit 0.001 → scale(MM) 필수.
TEE = "/World/test_pipe_tee_ID100"
# TEE_ROLL_DEG: 본관축(X) 둘레로 관을 굴린다 — 0=가지 위, 180=아래, 90=옆.
#   가지를 돌리면 BRANCH_ANGLE 도 같이 맞출 것 (90=위, -90=아래, 0/180=옆).
# TEE_USD / TEE_YAW_DEG: 다른 T 판 시험용 (기본 = 각진 T, 본관 Y→X 로 -90).
#   tee_sweep_r185_dn100_oneside.usd 는 본관이 이미 X 라 yaw 180 으로 뒤집어
#   블렌드된 모서리를 진입측에 둔다.
TEE_ROLL = float(os.environ.get("TEE_ROLL_DEG", 0.0))
TEE_USD = os.environ.get("TEE_USD", "tshape_test.usd")
TEE_YAW = float(os.environ.get("TEE_YAW_DEG", -90.0))
tee = stage.DefinePrim(TEE, "Xform")
tee.GetReferences().AddReference(str(SON / "maps" / TEE_USD))
# 🎯 TEE_SHIFT_X (2026-08-09 신설) — 관을 진행축으로 밀어 준다. 곡관 시험용:
#    곡관(fixed_pipe)의 직선부가 200mm 뿐이라 로봇(축간 228mm)이 안 들어간다.
#    입구 연장 직관(x -0.85~-0.25)과 겹치지 않게 곡관을 +X 로 밀어 붙인다.
TEE_SHIFT_X = float(os.environ.get("TEE_SHIFT_X", 0.0))
UsdGeom.Xformable(tee).AddTransformOp().Set(
    scale(MM) * rot(TEE_YAW, (0, 0, 1)) * rot(TEE_ROLL, (1, 0, 0))
    * trans(TEE_SHIFT_X, 0.0, 0.0))
while True:  # 인스턴스 프로토타입이면 콜라이더가 안 먹는다 — 전부 푼다
    inst = [p for p in stage.Traverse()
            if p.IsInstance() and str(p.GetPath()).startswith(TEE)]
    if not inst:
        break
    for p in inst:
        p.SetInstanceable(False)
_ncol = 0
for p in stage.Traverse():
    if not (p.IsA(UsdGeom.Mesh) and str(p.GetPath()).startswith(TEE)):
        continue
    if "Sweep" in str(p.GetPath()):      # 내면 중복 면체 — 표시·충돌 제외
        UsdGeom.Imageable(p).MakeInvisible()
        continue
    pipe_collider(p.GetPrim() if hasattr(p, "GetPrim") else p)
    _ncol += 1
say(f"T관 적재 — 콜라이더 {_ncol} (접합부 = 원점, 가지 = +Z 위)")

# ── 입구 연장 직관 600mm (x −0.85 ~ −0.25) ──────────────────────────
# 🚨 본관이 긴 판(스위프 T ±400)에서는 EXT=0 — 관이 겹치면 발산한다(기록).
USE_EXT = os.environ.get("EXT", "1") == "1"
_ext_stl = SON / "legacy" / "meshes" / "pipe_straight.stl"
if USE_EXT:
    pts, idx = load_stl(_ext_stl)
    _ext = UsdGeom.Mesh.Define(stage, "/World/pipe_entrance_ext")
    _ext.CreatePointsAttr([Gf.Vec3f(*q) for q in pts])
    _ext.CreateFaceVertexCountsAttr([3] * len(idx))
    _ext.CreateFaceVertexIndicesAttr(idx.reshape(-1).tolist())
    _ext.CreateExtentAttr([Gf.Vec3f(*pts.min(0)), Gf.Vec3f(*pts.max(0))])
    _ext.CreateSubdivisionSchemeAttr("none")
    _ext.CreateDisplayColorAttr([Gf.Vec3f(0.55, 0.58, 0.60)])
    UsdGeom.Xformable(_ext).AddTransformOp().Set(trans(-0.55, 0.0, 0.0))
    pipe_collider(_ext.GetPrim())
    say("입구 연장관 600mm — 월드 x −0.85 ~ −0.25")

# ── 로봇 — 파일 통째 참조, 중복 씬 끔, 실린더→hull, 자체충돌 OFF ────
ROBOT = "/World/pipe_robot_v9"
rob = stage.DefinePrim(ROBOT, "Xform")
rob.GetReferences().AddReference(str(HERE / "pipe_robot_v9.usda"))
for c in [c for c in Usd.PrimRange(rob) if c.IsA(UsdPhysics.Scene)]:
    c.SetActive(False)

art = next((c for c in Usd.PrimRange(rob)
            if c.HasAPI(UsdPhysics.ArticulationRootAPI)), None)
if art is None:
    raise SystemExit("[중단] 아티큘레이션 루트를 못 찾았다 — usda 구조 확인")
PhysxSchema.PhysxArticulationAPI.Apply(art).CreateEnabledSelfCollisionsAttr(
    False)

_nh = 0
for c in Usd.PrimRange(rob):
    if c.GetTypeName() in ("Cylinder", "Cone") \
            and c.HasAPI(UsdPhysics.CollisionAPI):
        cyl_to_hull(c)
        _nh += 1
say(f"로봇 적재 — 실린더·원뿔 콜라이더 {_nh}개 → convexHull 교체, 자체충돌 OFF")

# ── 다리 한계 덮어쓰기 (2026-08-09 신설) ────────────────────────────
# 🎯 **개별 배관은 v6·v9 모두 통과했는데 통합 배관에서만 막힌다**(사용자
#    확인). 그렇다면 남는 것은 **세부 조절값**이고, 그중 근거가 가장 뚜렷한
#    것이 다리 수축 한계다.
# 🚨 자산 원본은 **수축 −4mm / 신장 +3.7mm** 다. 접합부(T 개구)에서 앞
#    세그먼트가 10mm 넘게 밀리면 가까운 쪽 다리는 그만큼 접어야 하는데
#    −4mm 스토퍼에 박혀 **쐐기**가 된다 — v6 검증에서 남은 두 원인 중
#    하나로 기록돼 있고, 우리 로봇(welder_126)에서는 이 한계를 **−15mm 로
#    늘려 T 를 정방향 통과**시킨 실측이 있다(2026-08-07).
# 🔑 **자산(usda)은 고치지 않는다.** 원본 무수정 원칙 그대로, 이 러너가
#    런타임에만 덮어쓴다. 0 이면 자산값을 그대로 쓴다(= 지금까지의 검증).
# 🚨 예압은 목표를 스트로크 **밖**에 두는 힘 드라이브다 — 신장 한계를 늘리면
#    목표도 같이 밀어야 같은 힘이 유지된다(목표 = 신장×2, 우리 데모 규약).
# 🚨 world.reset() **앞**이어야 한다. 시뮬 시작 뒤 USD 드라이브 쓰기는
#    PhysX 로 안 넘어간다(기록된 함정).
V9_STROKE = float(os.environ.get("V9_STROKE", 0.0))     # 신장 한계(m)
V9_RETRACT = float(os.environ.get("V9_RETRACT", 0.0))   # 수축 한계 크기(m)
V9_PRELOAD = float(os.environ.get("V9_PRELOAD", 0.0))   # 예압 maxForce(N)
if V9_STROKE > 0 or V9_RETRACT > 0 or V9_PRELOAD > 0:
    _nleg = 0
    for _j in [p for p in Usd.PrimRange(rob)
               if p.IsA(UsdPhysics.PrismaticJoint)]:
        _pj = UsdPhysics.PrismaticJoint(_j)
        if V9_STROKE > 0:
            _pj.GetUpperLimitAttr().Set(V9_STROKE)
        if V9_RETRACT > 0:
            _pj.GetLowerLimitAttr().Set(-V9_RETRACT)
        _d = UsdPhysics.DriveAPI.Get(_j, "linear")
        if _d:
            if V9_STROKE > 0:
                _d.GetTargetPositionAttr().Set(V9_STROKE * 2.0)
            if V9_PRELOAD > 0:
                _d.GetMaxForceAttr().Set(V9_PRELOAD)
        _nleg += 1
    say(f"다리 {_nleg}개 한계 덮어씀 — 신장 "
        f"{'자산값' if V9_STROKE <= 0 else f'{V9_STROKE * 1000:.0f}mm'} / 수축 "
        f"{'자산값' if V9_RETRACT <= 0 else f'-{V9_RETRACT * 1000:.0f}mm'} / 예압 "
        f"{'자산값' if V9_PRELOAD <= 0 else f'{V9_PRELOAD:.0f}N'}  (usda 는 무수정)")

# ── 배치 — 관축(z=0) 위 x=−0.30, 전방 = 로컬 X = 월드 X (회전 불필요) ──
# 🚨 루트 Xform 에 걸면 PhysX 가 발산한다(기록) → 강체 링크 로컬에 굽는다.
#    루트를 항등으로 두므로 real_final_script 의 authored-transform 기반
#    travel/ax0 계산도 월드축 그대로 나와 맞는다.
START_X = float(os.environ.get("START_X", -0.30))
place = trans(START_X, 0.0, 0.0)
_nl = 0
for child in Usd.PrimRange(rob):
    if not child.HasAPI(UsdPhysics.RigidBodyAPI):
        continue
    par = child.GetParent()
    while par.IsValid() and par != rob:
        _pm2 = UsdGeom.Xformable(par).GetLocalTransformation()
        if float(np.abs(np.array(_pm2) - np.eye(4)).max()) > 1e-9:
            say(f"[경고] 중간 Xform {par.GetName()} 이 항등이 아니다")
        par = par.GetParent()
    xf = UsdGeom.Xformable(child)
    local = xf.GetLocalTransformation()
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(local * place)
    _nl += 1
say(f"로봇 배치 — 링크 {_nl}개, 헤드 x≈{START_X + 0.114:+.3f}m (접합부=0)")

world.reset()
say("world.reset() — 콜라이더 cook 완료")

# ── 위치 리더 (physics view — USD 쓰기 지연과 무관) ─────────────────
from isaacsim.core.prims import RigidPrim                 # noqa: E402

_views = {}
for nm in ("DiscF", "Body", "DiscR"):
    v = RigidPrim(prim_paths_expr=f"{ROBOT}/Robot/{nm}", name=f"rd_{nm}")
    try:
        v.initialize()
    except Exception:
        pass
    _views[nm] = v


def pos_of(nm):
    p, _ = _views[nm].get_world_poses()
    return np.array([float(p[0][0]), float(p[0][1]), float(p[0][2])])


def bend_deg():
    """기하학적 굽힘각 — DiscF·Body·DiscR 세 점이 만드는 꺾임."""
    pf, pb, pr = pos_of("DiscF"), pos_of("Body"), pos_of("DiscR")
    u = pf - pb
    v = pb - pr
    u /= max(np.linalg.norm(u), 1e-12)
    v /= max(np.linalg.norm(v), 1e-12)
    return math.degrees(math.acos(float(np.clip(np.dot(u, v), -1.0, 1.0))))


def sane(p):
    return np.all(np.isfinite(p)) and np.max(np.abs(p)) < 3.0


def run_sim(sec, label="", watch_stuck=False):
    """sec 초 시뮬. 반환 False = 발산/정체로 중단."""
    steps = int(sec * PHYSICS_HZ)
    mark_p, mark_t = pos_of("DiscF"), 0.0
    for i in range(steps):
        world.step(render=not HEADLESS)
        t = (i + 1) / PHYSICS_HZ
        if (i + 1) % PHYSICS_HZ == 0 or i == steps - 1:
            p = pos_of("DiscF")
            if not sane(p):
                say(f"🚨 발산 — 헤드 {p} ({label})")
                return False
            if watch_stuck:
                if np.linalg.norm(p - mark_p) * 1000 >= STUCK_MM:
                    mark_p, mark_t = p, t
                elif t - mark_t >= STUCK_S:
                    say(f"🚨 정체 — {STUCK_S:.0f}s 에 {STUCK_MM:.0f}mm 미만 "
                        f"이동 ({label}) → 즉시 중단(테스트 규율)")
                    return False
    return True


# ── 안착 1.5s (휠 0 인 채로) ────────────────────────────────────────
if not run_sim(1.5, "안착"):
    raise SystemExit("[결론] ❌ 안착 단계에서 발산 — 물리 셋업 문제")
p0 = pos_of("DiscF")
say(f"안착 완료 — 헤드 ({p0[0] * 1000:+.0f}, {p0[1] * 1000:+.0f}, "
    f"{p0[2] * 1000:+.0f})mm, 굽힘 {bend_deg():.1f}°")

# ── real_final_script 를 그대로 import (자동 go 는 일단 멈춘다) ───────────
sys.path.insert(0, str(HERE))
import real_final_script as fs                                 # noqa: E402

fs.stop()                     # import 시 자동 go() 취소 — 페이즈 나눠 검증
fs.set_branch_angle(BRANCH_ANGLE)
if V9_WHEEL_CMD > 0:          # 테스트 회전율 노브 — 저작값은 600 deg/s
    fs.WHEEL_CMD = V9_WHEEL_CMD
    fs._S["v"] = math.radians(V9_WHEEL_CMD) * fs.WHEEL_R * fs.SPEED_SCALE
    say(f"휠 지령 {V9_WHEEL_CMD:.0f} deg/s 로 조절")
# 🔬 1차 실측: 스케줄 s(헤드 |Δp| 적산)가 위로 스윙하는 동안 실제 전진보다
#    빨리 가산 → s=+96 에서 조기 "통과 완료" → 굽힘 해제 → 접합부 정체
#    (z+77mm). 굽힘 유지 구간을 늘려 파라미터 문제인지 가르는 노브.
for _nm in ("TURN_FRONT", "TURN_BODY", "TURN_REAR", "LAG_MM", "TOTAL_BEND"):
    _v = os.environ.get(f"V9_{_nm}")
    if _v is not None:
        setattr(fs, _nm, float(_v))
        say(f"{_nm} = {float(_v):.0f} 로 덮어씀")

# ── 🎯 **우리 실측값을 얹는다** (2026-08-09 사용자 지적) ─────────────
# 🚨 그동안 이 러너는 "원본 무수정" 을 자산 파일뿐 아니라 **물리 설정 전체**로
#    확대 적용하고 있었다. 그 원칙의 취지는 *조원 코드를 안 고친다* 이지
#    *우리가 이틀 걸려 찾은 값을 버린다* 가 아니다 — 그 값들은 대부분
#    데모가 런타임에 덮어쓰는 것이라 원본을 건드리지도 않는다.
# 🔑 특히 **조향력**이 걸린다. 실측된 조향 저항은 **약 3 N·m** (다리 6개×9N 을
#    벽을 따라 미끄러뜨리는 힘 + 스프링 압축)인데, 조원 스크립트의 기본
#    `BEND_MF` 는 **2.5** 로 그보다 작다. 우리 로봇에서는 상한을 **15** 까지
#    올리고서야 관절이 지령을 따라왔다(강성 스윕 실측):
#        강성 2 / 상한 24  → 관절합 -16° (지령 -54°)  ❌ 이탈 80mm
#        강성 20 / 상한 12 → -58°   이탈 32.8mm
#        강성 60 / 상한 15 → -58°   이탈 21.6mm  ← 채택
#    그래서 방금 실측(굽힘 49.7°→22.2° 로 스스로 풀림)이 "스크립트가 통과로
#    판단해 놓은 것"인지 "힘이 모자라 벽에 밀려 풀린 것"인지 아직 안 갈렸다.
#    힘을 우리 값으로 올려 보면 그 둘이 갈린다.
# 🚨 스크립트가 **매 스텝 다시 쓰므로** 자산을 고쳐 봐야 소용없다 —
#    모듈 상수를 덮어써야 한다(`_drive(r, bend_k, bend_d, BEND_MF)`).
# 🚨 **기본 0 = 안 건드림** (2026-08-09 사용자 지시). 한때 우리 실측값을
#    기본값으로 박아 두었는데, 그러면 "조원 원본이 되는가" 라는 기준선이
#    사라진다. 한 번에 하나씩 바꿔야 무엇이 효과를 냈는지 가릴 수 있다.
for _nm, _dflt in (("BEND_MF", 0.0), ("ROLL_MF", 0.0)):
    _v = float(os.environ.get(f"V9_{_nm}", _dflt))
    if _v > 0:
        setattr(fs, _nm, _v)
        say(f"{_nm} = {_v:.1f} N·m 로 덮어씀 (자산·스크립트 기본보다 큼 — "
            f"실측 조향 저항 약 3 N·m)")
_bk = float(os.environ.get("V9_BEND_K", 0.0))   # 0 = 스크립트값 그대로
if _bk > 0:
    fs._S["bend_k"] = _bk
    say(f"조향 강성 {_bk:.0f} 로 덮어씀 (우리 실측 채택값)")

# ── 페이즈 A: selftest — USD 드라이브 쓰기가 PhysX 에 먹는가 ─────────
# 🚨 관 안이라 40° 지령이 그대로 나올 수 없다(벽·바퀴가 저항, maxF 2.5).
#    합격 기준은 절대각이 아니라 **안착 대비 증분** — 쓰기가 안 먹으면 0 이다.
_bend_base = bend_deg()
say(f"── 페이즈 A: selftest (BendF 40° 지령, 기준 {_bend_base:.1f}°) ──")
fs.selftest()
run_sim(2.0, "selftest")
_bend_a = bend_deg()
_A_OK = _bend_a - _bend_base > 3.0
say(f"selftest 굽힘 실측 {_bend_a:.1f}° (Δ{_bend_a - _bend_base:+.1f}°) → "
    + ("✅ 드라이브 쓰기가 물리에 먹는다" if _A_OK else
       "❌ 관절이 안 꺾인다 — 시뮬 중 USD 쓰기가 PhysX 에 안 넘어간다"))
fs.reset()
run_sim(1.5, "몸펴기")

# ── 페이즈 B: go() — 전체 분기 기동 ─────────────────────────────────
# 🎯 **곡관 시험 모드** (2026-08-09 신설) — 분기 기동(go)을 쓰지 않고 바퀴만
#    돌려 밀고 지나간다. 곡관은 벽이 몸을 꺾어 주므로 조향 스케줄이 필요 없다는
#    설계 전제(사용자)를 v9 로봇으로 확인하는 자리다.
CURVE_MODE = os.environ.get("CURVE_MODE", "0") == "1"
if CURVE_MODE:
    say("── 페이즈 B: **곡관 모드** — 분기 기동 없이 바퀴만 돌린다 ──")
    fs.stop()
    # 🚨 `_set_wheels` 는 **도/초를 그대로** 받는다(go() 도 WHEEL_CMD 를 날것으로
    #    넘긴다). 라디안으로 변환해 넘겼다가 600 대신 10.5 를 줘서 3초에 4mm 만
    #    가고 "정체" 로 오판된 사고(2026-08-09).
    _wv = float(os.environ.get("CURVE_WHEEL", fs.WHEEL_CMD))
    fs._set_wheels(_wv)
    say(f"휠 지령 {_wv:.0f} deg/s — 조향 스케줄 없음")
else:
    say(f"── 페이즈 B: go() — 가지 방향 {BRANCH_ANGLE:.0f}° "
        f"(90=위), 시뮬 상한 {SIM_MAX_S:.0f}s ──")
    fs.go()
# 🔬 V9_FORCE_ODO=1: live arc 적산(|Δp| — 스윙·요동까지 가산해 스케줄이
#    폭주하는 실측 결함) 대신 시간 기반 오도메트리로 강제하는 대조군.
if os.environ.get("V9_FORCE_ODO", "0") == "1":
    fs._S["reader"] = None
    say("오도메트리 강제 — live arc 대신 시간 기반 s")
# 진행도 = 접합부(원점) 기준 **가지 방향 변위** — 위/아래/옆 어느 배치든 같다.
_ba = math.radians(BRANCH_ANGLE)
_bdir = np.array([0.0, -math.cos(_ba), 0.0]) + np.array([0.0, 0.0, math.sin(_ba)])
_best_z, _verdict = 0.0, None
mark_p, mark_t = pos_of("DiscF"), 0.0
t = 0.0
# 🎯 **곡관 조향** (2026-08-09 신설) — 굽힘 목표를 0°(똑바로)로 두면 관절이
#    벽과 싸운다. 실측: 힘을 2.5→15 N·m 로 올리자 오히려 **덜 꺾였다**
#    (14.1°→12.3°) — 목표 0° 를 더 세게 붙잡은 것. 힘이 아니라 **지시**가
#    없는 것이 문제였다.
# 🔑 v9 중앙 강체가 **96mm**(welder 25.5mm)라 R150 곡관에서 필요한 굽힘이
#    96/150 ≈ **37°** 다(welder 는 10°). 굽힘 한계 ±95° 라 여유는 충분하다.
# 🚨 처음부터 꺾으면 직선 구간에서 벽에 박는다 → 머리가 원호 시작을 지난 뒤에만
#    준다. 실기에서는 이 자리를 카메라 센터링 조향이 맡는다(우리 자율 스택).
# 🚨 **부호 주의** (2026-08-09 실측): 굽힘축이 Y 라 **양(+)이 아래**다
#    (오른손 법칙 — +X 가 −Z 로 간다). 위로 휘는 곡관에는 **음수**를 준다.
#    +37 을 줬다가 머리를 바닥에 처박아 도달이 40→10mm 로 나빠졌다.
CURVE_BEND = float(os.environ.get("CURVE_BEND", 0.0))       # 목표 굽힘(도, 음수=위)
CURVE_BEND_AT = float(os.environ.get("CURVE_BEND_AT", -0.05))  # 이 x 부터 꺾는다
CURVE_REAR = os.environ.get("CURVE_REAR", "1") == "1"       # 뒤 굽힘도 같이
CURVE_REAR_SIGN = float(os.environ.get("CURVE_REAR_SIGN", -1.0))  # 뒤는 반대부호
_bend_on = False
while t < SIM_MAX_S:
    world.step(render=not HEADLESS)
    t += 1.0 / PHYSICS_HZ
    k = int(t * PHYSICS_HZ + 0.5)
    if CURVE_MODE and CURVE_BEND != 0.0 and k % 12 == 0:
        _hx = float(pos_of("DiscF")[0])
        _want = CURVE_BEND if _hx > CURVE_BEND_AT else 0.0
        fs._set_bend(fs._S["bends"][0], _want,
                     mf=float(os.environ.get("V9_BEND_MF", 0) or fs.BEND_MF))
        # 🚨 **뒤 굽힘은 부호가 반대다** (2026-08-09 실측). 두 굽힘축이 중앙
        #    몸통을 사이에 두고 마주 보므로, 같은 부호를 주면 **S 자**가 되어
        #    가운데서 상쇄된다 — 곡관을 따르려면 **C 자**여야 한다.
        #    실측: 앞뒤 같은 부호로 주자 전체 꺾임이 조향 없을 때(14.1°)보다
        #    **작아졌다**(6.9°). 방향을 뒤집어도 6.9° 로 같았던 것이 단서였다.
        if CURVE_REAR and len(fs._S["bends"]) > 1:
            fs._set_bend(fs._S["bends"][1], _want * CURVE_REAR_SIGN,
                         mf=float(os.environ.get("V9_BEND_MF", 0) or fs.BEND_MF))
        if _want != 0.0 and not _bend_on:
            _bend_on = True
            say(f"곡관 조향 시작 — 굽힘 목표 {CURVE_BEND:.0f}° (헤드 x={_hx * 1000:+.0f}mm)")
    if k % (PHYSICS_HZ // 2):
        continue
    p = pos_of("DiscF")
    _prog = float(np.dot(p, _bdir))
    _best_z = max(_best_z, _prog)
    if not sane(p):
        _verdict = "❌ 발산 — 좌표가 날아갔다"
        break
    if k % PHYSICS_HZ == 0:
        say(f"  t={t:4.1f}s 헤드 ({p[0] * 1000:+5.0f}, {p[1] * 1000:+5.0f}, "
            f"{p[2] * 1000:+5.0f})mm 가지진행 {_prog * 1000:+5.0f}mm "
            f"굽힘 {bend_deg():5.1f}° mode={fs._S.get('mode')}")
    if _prog > 0.12:
        _verdict = f"✅ 분기 진입 성공 — 가지 진행 {_prog * 1000:+.0f}mm"
        break
    if p[0] > 0.23:
        _verdict = "❌ 분기 진입 실패 — 본관을 직진 통과했다"
        break
    if np.linalg.norm(p - mark_p) * 1000 >= STUCK_MM:
        mark_p, mark_t = p, t
    elif t - mark_t >= STUCK_S:
        _verdict = (f"❌ 정체 — {STUCK_S:.0f}s 에 {STUCK_MM:.0f}mm 미만 "
                    f"(위치 x{p[0] * 1000:+.0f} z{p[2] * 1000:+.0f}mm)")
        break
if _verdict is None:
    _verdict = f"⏱ 시간 상한 {SIM_MAX_S:.0f}s — 최고 z {_best_z * 1000:+.0f}mm"

fs.stop()
print("=" * 70, flush=True)
say(f"페이즈 A(드라이브 쓰기): {'✅' if _A_OK else '❌'} (굽힘 {_bend_a:.1f}°)")
say(f"페이즈 B(분기 기동)  : {_verdict}")
say(f"최고 도달 z {_best_z * 1000:+.0f}mm / 목표 +120mm")
print("=" * 70, flush=True)

simulation_app.close()
