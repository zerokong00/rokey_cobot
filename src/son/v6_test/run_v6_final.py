#!/usr/bin/env python3
"""pipe_robot_v6 + final_script(v8) 검증 러너 — 기존 자율주행 코드와 완전 분리.

CAD 개발자가 준 두 파일(pipe_robot_v6.usda / final_script.py)이 "되는지"를
본다. final_script 는 **한 글자도 안 고치고** 그대로 import 한다 — 이 러너가
해 주는 것은 씬 구성과 이 PC 한정 안전장치뿐이다:

  1. 각진 T(tshape_test.usd)를 final_script 가 기대하는 경로
     `/World/test_pipe_tee_ID100` 에 올린다. 본관을 월드 X 로 돌려
     (Rz-90) 접합부가 원점에 오게 한다 → JUNCTION=(0,0,0).
  2. 입구에 legacy 직관 600mm 를 맞대 붙인다 — 로봇(~회전축 간 228mm)이
     본관 반쪽(250mm)보다 길어서, 조향 스케줄이 잠든 채 달릴 조주로가 없다.
  3. 로봇을 `/World/pipe_robot_v6` 에 **파일 통째로** 참조하고(머티리얼
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
  PYTHONUNBUFFERED=1 isaac_python run_v6_final.py
노브(env): HEADLESS=0 GUI 확인 / BRANCH_ANGLE(기본 90=위) /
  V6_WHEEL_CMD(기본 600 deg/s) / SIM_MAX_S / STUCK_S / STUCK_MM
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
V6_WHEEL_CMD = float(os.environ.get("V6_WHEEL_CMD", 0.0))  # 0 = 저작값 그대로

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

# ── 각진 T — final_script 가 기대하는 경로에, 접합부가 원점에 오게 ──
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
UsdGeom.Xformable(tee).AddTransformOp().Set(
    scale(MM) * rot(TEE_YAW, (0, 0, 1)) * rot(TEE_ROLL, (1, 0, 0)))
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
ROBOT = "/World/pipe_robot_v6"
rob = stage.DefinePrim(ROBOT, "Xform")
rob.GetReferences().AddReference(str(HERE / "pipe_robot_v6.usda"))
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

# ── 배치 — 관축(z=0) 위 x=−0.30, 전방 = 로컬 X = 월드 X (회전 불필요) ──
# 🚨 루트 Xform 에 걸면 PhysX 가 발산한다(기록) → 강체 링크 로컬에 굽는다.
#    루트를 항등으로 두므로 final_script 의 authored-transform 기반
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

# ── final_script 를 그대로 import (자동 go 는 일단 멈춘다) ───────────
sys.path.insert(0, str(HERE))
import final_script as fs                                 # noqa: E402

fs.stop()                     # import 시 자동 go() 취소 — 페이즈 나눠 검증
fs.set_branch_angle(BRANCH_ANGLE)
if V6_WHEEL_CMD > 0:          # 테스트 회전율 노브 — 저작값은 600 deg/s
    fs.WHEEL_CMD = V6_WHEEL_CMD
    fs._S["v"] = math.radians(V6_WHEEL_CMD) * fs.WHEEL_R * fs.SPEED_SCALE
    say(f"휠 지령 {V6_WHEEL_CMD:.0f} deg/s 로 조절")
# 🔬 1차 실측: 스케줄 s(헤드 |Δp| 적산)가 위로 스윙하는 동안 실제 전진보다
#    빨리 가산 → s=+96 에서 조기 "통과 완료" → 굽힘 해제 → 접합부 정체
#    (z+77mm). 굽힘 유지 구간을 늘려 파라미터 문제인지 가르는 노브.
for _nm in ("TURN_FRONT", "TURN_BODY", "TURN_REAR", "LAG_MM", "TOTAL_BEND"):
    _v = os.environ.get(f"V6_{_nm}")
    if _v is not None:
        setattr(fs, _nm, float(_v))
        say(f"{_nm} = {float(_v):.0f} 로 덮어씀")

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
say(f"── 페이즈 B: go() — 가지 방향 {BRANCH_ANGLE:.0f}° "
    f"(90=위), 시뮬 상한 {SIM_MAX_S:.0f}s ──")
fs.go()
# 🔬 V6_FORCE_ODO=1: live arc 적산(|Δp| — 스윙·요동까지 가산해 스케줄이
#    폭주하는 실측 결함) 대신 시간 기반 오도메트리로 강제하는 대조군.
if os.environ.get("V6_FORCE_ODO", "0") == "1":
    fs._S["reader"] = None
    say("오도메트리 강제 — live arc 대신 시간 기반 s")
# 진행도 = 접합부(원점) 기준 **가지 방향 변위** — 위/아래/옆 어느 배치든 같다.
_ba = math.radians(BRANCH_ANGLE)
_bdir = np.array([0.0, -math.cos(_ba), 0.0]) + np.array([0.0, 0.0, math.sin(_ba)])
_best_z, _verdict = 0.0, None
mark_p, mark_t = pos_of("DiscF"), 0.0
t = 0.0
while t < SIM_MAX_S:
    world.step(render=not HEADLESS)
    t += 1.0 / PHYSICS_HZ
    k = int(t * PHYSICS_HZ + 0.5)
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
