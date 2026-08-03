"""yongbin 실설계 부품을 articulation 으로 묶어 배관 내 밀착·주행을 검증.

yongbin_assembly_view.py(정적 조립)의 물리 버전. 답해야 하는 질문 두 가지:
  1. 밀착 — 스윙암 예압 스프링이 휠 6개를 관벽에 눌러 붙이는가
     (정적 조립은 접촉 반경 49.43mm 로 관벽에서 0.57mm 떠 있다)
  2. 이동 — 그 상태로 바퀴 구동 시 미끄럼 없이 전진하는가

스윙암 파라미터는 v3 §13 확정값을 그대로 쓴다:
  limits 33.4~58.2°, targetPosition 60°(상한 밖), maxForce 0.257 N·m(예압 9N),
  stiffness 0.74 N·m/rad, damping 0.009 N·m·s/rad
단, 암 STL 이 43.3° 자세로 모델링돼 있어 조인트 0° = 43.3° — 한계·목표를
그만큼 이동해 적용한다 (아래 ARM_MODELED_DEG).

접촉 기하 예측(검증 게이트): 휠이 내벽 r50 에 닿는 각도는
  12 + 40·sin(θ) + 10 = 50  →  θ = 44.43°  →  조인트 +1.13°
정착 후 조인트가 이 근처(한계 안)면 "밀착", 상한 +14.9° 에 붙어 있으면 "떠 있음".

실행:
  PYTHONUNBUFFERED=1 isaac_python yongbin_drive_test.py --headless
  PYTHONUNBUFFERED=1 isaac_python yongbin_drive_test.py            # GUI
"""

import sys

HEADLESS = "--headless" in sys.argv
# --front-drive: 전방 3휠만 구동 (견인 배치). 잭나이프가 "후방이 자유 힌지를
# 통해 전방을 미는" 불안정인지 확정하는 대조 실험 — 견인이면 힌지가 인장이라
# 자기 안정. 7차: 6륜 전구동은 토크를 40%로 줄여도 2초 내 ±55° 스토퍼로 접힘
FRONT_DRIVE = "--front-drive" in sys.argv
# --stiff-joint: 중앙 관절 스프링 1.0 N·m/rad — 관절을 사실상 강체화한 대조군.
# 이때 주행이 깨끗하면 잭나이프가 유일한 잔여 문제임이 증명되고, 도달 가능한
# 주행 성능의 상한을 얻는다
STIFF_JOINT = "--stiff-joint" in sys.argv
# --elbow: 직관 → SR 90° 엘보(R=100) → 출구 직관 통과 시험. 엘보를 수평
# 선회(x-y 평면)로 눕혀 중앙 관절(요 Z축)과 굽힘 평면을 일치시킨다.
# 핵심 질문: 강성화한 센터링 스프링(--stiff-joint 1.0 N·m/rad)이 곡관 굽힘
# 36.1°(v3 기하)를 허용하는가 — 직진 안정 vs 곡관 통과의 절충 검증
ELBOW = "--elbow" in sys.argv
# --water: 정찰 임무 조건 — 흐르는 물(PBD 파티클, 유속 -X + 재순환) 속을
# 거슬러 주행. 검증된 파티클 세팅(water_particle_demo: PCO 8mm,
# contact/rest 4/2.5mm — rest 미지정 시 로봇이 입자 위에 부상하는 함정 수정
# 포함)을 그대로 쓴다. 파티클은 GPU 물리 전용이라 휠 드라이브 목표를
# reset "전에" USD 에 기록한다 (GPU 는 시작 후 USD 드라이브 변경 무시)
WATER = "--water" in sys.argv
# 엘보+물: 유속장(wind)은 전역 단방향이라 굽은 경로에는 못 쓴다 — 엘보
# 코스는 고인 물(유속 0)로 채우고, 로봇이 헤치고 지나가는 물결로 표현한다.
# 굽힘이 수평(x-y)이라 수위 z=일정 이 코스 전체에서 성립한다
# --no-pipe: 배관 없이 예압만 — 암 조인트 부호 검증. 부호가 맞으면 정착 시
# 암이 신장 한계 +14.9°, 휠 중심 반경 46mm(자유 최대)로 가야 한다
NO_PIPE = "--no-pipe" in sys.argv

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": HEADLESS})

import json
from pathlib import Path

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
from isaacsim.core.utils.viewports import set_camera_view
from omni.physx.scripts import particleUtils
from pxr import (Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics,
                 UsdShade, Vt)

SRC = Path("/home/rokey/Downloads/yongbin")
META = json.loads((SRC / "parts_meta.json").read_text())
MM = 0.001

# ── 조립 치수 (meta + 실측) ──
SEG_CENTER = META["seg_center_offset"] * MM
PIVOT_R = META["pivot_r"] * MM
ARM_DX = META["arm_dx_nominal"] * MM
ARM_DR = META["arm_dr_nominal"] * MM
ARM_PIVOT_X = 0.070                    # 뷰어와 동일한 가정값
WHEEL_R = META["wheel_r"] * MM
WHEEL_CENTER_R = PIVOT_R + ARM_DR
ARM_MODELED_DEG = META["arm_angle_nominal"]     # 43.30 — STL 모델링 자세

# ── v3 §13 스윙암 확정 파라미터 (조인트 좌표로 이동) ──
ARM_LIMIT_LO = 33.4 - ARM_MODELED_DEG           # -9.9°
ARM_LIMIT_HI = 58.2 - ARM_MODELED_DEG           # +14.9°
ARM_TARGET = 60.0 - ARM_MODELED_DEG             # +16.7° (상한 밖 = 예압)
ARM_STIFF = 0.74 * np.pi / 180.0                # N·m/rad → N·m/deg
ARM_DAMP = 0.009 * np.pi / 180.0
ARM_MAX_TORQUE = 0.257                          # N·m — 예압 9N 상당
# 접촉각: 휠은 크라운 없는 원통(트레드 폭 ±7.5mm, STL 실측)이라 곡면 내벽에
# 트레드 양끝 에지 2점으로 닿는다. 에지 접촉 조건 √(7.5²+(R_c+10)²)=50 에서
# R_c=39.434mm = meta contact 기하 → 조인트 각 0.0° (모델링 자세 그대로)
HALF_TREAD = 7.5 * MM
CONTACT_JOINT_DEG = np.degrees(np.arcsin(
    (np.sqrt(0.050 ** 2 - HALF_TREAD ** 2) - WHEEL_R - PIVOT_R) / 0.040)) \
    - ARM_MODELED_DEG                           # ≈ 0.00°

# ── 중앙 관절 (수동, v3): ±55°, 센터링 스프링 0.01 N·m/rad ──
CENTER_LIMIT = 55.0
CENTER_STIFF = 0.01 * np.pi / 180.0
# 요 댐퍼: 8차까지 전 실행에서 자유 힌지가 주행 시작 수 초 내 ±55° 스토퍼로
# 잭나이프(두 자주식 세그먼트 + 자유 힌지 = 트레일러 밀기 불안정, 스프링
# 0.0096 N·m 로는 못 잡음). 댐퍼는 곡관 통과(준정적 굽힘)는 방해하지 않으면서
# 접힘 속도에만 저항 — 실물 반영 검토 대상 설계 제안
CENTER_DAMP = 0.05 * np.pi / 180.0     # 0.05 N·m·s/rad
if STIFF_JOINT:
    CENTER_STIFF = 1.0 * np.pi / 180.0
# --spring K: 센터링 스프링(N·m/rad) 직접 지정 — 직진 안정 vs 곡관 통과 절충
# 스윕용. 10차: 1.0 은 직진 완벽 / 엘보에서 쐐기+발산, 0.01(v3)은 엘보 통과 /
# 직진 잭나이프. 중간값을 찾는다
if "--spring" in sys.argv:
    CENTER_STIFF = float(sys.argv[sys.argv.index("--spring") + 1]) * np.pi / 180.0

# ── 질량 (합 ~500g, v3 건조 질량) ──
MASS_BODY = 0.18
MASS_ARM = 0.015
MASS_WHEEL = 0.008

# ── 구동: 50mm/s → ω = 5 rad/s. 토크 한계 = μ·예압·r 근처 ──
# 부호: 관내 휠은 조인트 축(로컬 Y)과 반경 방향이 롤과 함께 돌아서, 모든 휠이
# 같은 부호로 돌면 같은 방향으로 민다. 1차 실측: +회전 = -X 주행(후진해 배관
# 끝에 걸림) → +X 전진은 음의 목표 속도
WHEEL_TARGET_DEG_S = -np.degrees(0.050 / WHEEL_R)         # -286 deg/s = +X 전진
# 토크 상한: 정마찰 원(0.4·9N·r=0.036)의 ~40%. 6차 실측: 0.034(94%)로 걸면
# 마찰 원을 구동이 다 써서 횡방향 안내력이 0 — 주행 시작 2초 만에 중앙 관절이
# ±55° 스토퍼로 잭나이프(트레일러 후진 불안정, 센터링 스프링 0.0096 N·m 로는
# 못 잡음). 여유를 남기면 같은 마찰이 조향 강성으로 작동한다. 직관 평지 구름
# 저항은 mN·m 수준이라 15 mN·m 로도 스키드 없이 충분
WHEEL_MAX_TORQUE = 0.015
# --torque T: 휠 토크 상한(N·m) 지정. 11차: 엘보에서는 스프링 복원 토크를
# 이기며 전진해야 해서 15 mN·m 로는 정체 — 스프링으로 직진이 안정화된 뒤에는
# 토크를 올려도 잭나이프가 재발하지 않는지가 관건
if "--torque" in sys.argv:
    WHEEL_MAX_TORQUE = float(sys.argv[sys.argv.index("--torque") + 1])
WHEEL_DAMP = WHEEL_MAX_TORQUE / 60.0                      # 60 deg/s 오차서 최대

FRICTION = (0.30, 0.25) if WATER else (0.40, 0.35)   # 만관(정찰)/배수(수리)
PHYS_DT = 1.0 / 240.0                 # v3 §13.2
SETTLE_STEPS = 480
DRIVE_STEPS = 4800                    # 20 s
REPORT_EVERY = 480
START_X = -0.20
if ELBOW:
    # 전체 코스: 직관 시작점부터 550mm 직진 → 엘보 90° → 출구 직관 450mm.
    # 실제 임무 조건대로 직선·곡선이 이어진 경로를 한 번에 통과해야 한다
    START_X = -0.25
    DRIVE_STEPS = 16800               # 70 s (코스 ~1.15m)
    REPORT_EVERY = 1200
if WATER:
    SETTLE_STEPS = 1440               # 6 s — 물 유입 서지가 가라앉을 시간
    DRIVE_STEPS = 12000 if not ELBOW else 24000   # 물 저항 감속 감안
    REPORT_EVERY = 1200
    FLOW_V = -0.10 if not ELBOW else 0.0   # 엘보 코스는 고인 물 (전역 유속 불가)
    W_SPACING = 0.009
    W_PCO = 0.008
    W_FLUID_REST = 0.0048
    W_LEVEL_Z = 0.020                 # 유동 자연 수위 (부분 충전 흐름)
    W_R_MAX = 0.045
    W_X = (-0.28, 0.28)
    RECYCLE_OUT_X = -0.28             # 하류 이탈 → 상류 재투입 (러닝머신)
    RECYCLE_IN_X = (0.22, 0.29)
    RECYCLE_EVERY = 4
    STOP_X_WATER = 0.15               # 재투입 지점 앞에서 정지

world = World(stage_units_in_meters=1.0, physics_dt=PHYS_DT)
stage = world.stage
if WATER:
    _pc = world.get_physics_context()
    _pc.enable_gpu_dynamics(True)     # 파티클은 GPU 전용
    _pc.set_broadphase_type("GPU")

import omni.timeline

_timeline = omni.timeline.get_timeline_interface() if not HEADLESS else None


def sim_step(render=True):
    """world.step 래퍼 — GUI 툴바에서 사용자가 정지(Stop)해도 앱이 죽지 않게.

    스탠드얼론 스크립트에서 사용자가 정지를 누르면 물리 핸들이 무효화돼
    다음 world.step 이 예외로 죽고 앱이 통째로 종료된다 (실사용 보고).
    정지 감지 시 재생될 때까지 대기하고, 재생되면 world.reset() 으로 데모를
    처음부터 재시작한다 (정지 후 이어가기는 핸들 구조상 불가).
    """
    if _timeline is not None and not _timeline.is_playing():
        while simulation_app.is_running() and not _timeline.is_playing():
            simulation_app.update()
        if not simulation_app.is_running():
            simulation_app.close()
            sys.exit(0)
        try:
            world.reset()
        except Exception:
            pass
    try:
        world.step(render=render)
    except Exception:
        simulation_app.update()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage, "/World")
UsdLux.DomeLight.Define(stage, Sdf.Path("/World/DomeLight")).CreateIntensityAttr(1500.0)

# ── 관내 촬영용 조명: 코스 축을 따라 구형 라이트 배열 ──
# 주의: 임무 사양은 "로봇 조명이 유일한 광원"이다. 이 조명은 주행 데모
# 촬영용 연출일 뿐 — 결함 검출·비전 임계값 튜닝에는 절대 이 씬을 쓰지 말 것
# 위치: 상단에서 30° 비킨 벽 근처(r≈40mm) — 축 위에 두면 로봇이 지나가며
# 광구가 화면을 가리고, 휠 궤적(0/60/...° ±11°)·몸체와도 겹치지 않는 각도
_light_pts = [(x, 0.020, 0.0346) for x in np.arange(-0.25, 0.31, 0.10)]
if ELBOW:
    _light_pts += [(0.37, 0.03, 0.0346), (0.40, 0.10, 0.0346)]  # 엘보 구간
    _light_pts += [(0.42, y, 0.0346) for y in np.arange(0.20, 0.61, 0.10)]
for _i, _p in enumerate(_light_pts):
    _sl = UsdLux.SphereLight.Define(stage, f"/World/CourseLight_{_i}")
    _sl.CreateRadiusAttr(0.006)
    _sl.CreateIntensityAttr(80000.0)
    _sl.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.9))
    UsdGeom.XformCommonAPI(_sl).SetTranslate(Gf.Vec3d(*_p))

ROBOT = "/World/Robot"
root = UsdGeom.Xform.Define(stage, ROBOT)
root.AddTransformOp().Set(Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(START_X, 0, 0)))


def rx(deg):
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), deg))
    return m


def rz(deg):
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), deg))
    return m


def tr(x, y, z):
    m = Gf.Matrix4d(1.0)
    m.SetTranslate(Gf.Vec3d(x, y, z))
    return m


def rot_only(m):
    r = Gf.Matrix4d(1.0)
    r.SetRotate(m.ExtractRotation())
    return r


# 물리 재질
MAT = f"{ROBOT}/PhysicsMaterials/Wheel"
UsdGeom.Scope.Define(stage, f"{ROBOT}/PhysicsMaterials")
_m = UsdPhysics.MaterialAPI.Apply(UsdShade.Material.Define(stage, MAT).GetPrim())
_m.CreateStaticFrictionAttr(FRICTION[0])
_m.CreateDynamicFrictionAttr(FRICTION[1])
_m.CreateRestitutionAttr(0.0)


def set_offsets(prim):
    """v3 §13: contactOffset 0.0005 / restOffset 0 — 기본 0.02 는 로봇이 뜬다.

    예외: GPU 물리(--water)에서는 0.0005 가 GPU 접촉 생성 허용치보다 작아
    접촉이 전부 소실된다 (13차: 로봇이 배관을 관통해 자유 낙하 — 반면 오프셋
    미설정이던 임시 로봇은 GPU 에서 정상 주행). 물 모드는 엔진 기본값 사용.
    """
    if WATER:
        return
    api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    api.CreateContactOffsetAttr(0.0005)
    api.CreateRestOffsetAttr(0.0)


def make_link(path, matrix, usd_name, mass, approx, material=None,
              cylinder=None):
    """cylinder=(radius, height): v3 §13 "휠은 실린더 프리미티브" — 메시 hull 의
    날카로운 에지 접촉(원통 트레드 양끝 2점) 대신 해석적 원통 충돌체를 쓴다.
    3차 실측: hull 에지 접촉 + 예압 9N + 8g 휠 = 스틱슬립 채터로 슬립률 0.73."""
    link = UsdGeom.Xform.Define(stage, path)
    link.AddTransformOp().Set(matrix)
    prim = link.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    vis = UsdGeom.Xform.Define(stage, path + "/visual")
    vis.AddScaleOp().Set(Gf.Vec3f(MM, MM, MM))
    vis.GetPrim().GetReferences().AddReference(str(SRC / usd_name))
    if cylinder is not None:
        # 시각 메시는 그대로, 충돌체만 원통 프리미티브 (축 = 로컬 Y)
        cyl = UsdGeom.Cylinder.Define(stage, path + "/collider")
        cyl.CreateAxisAttr("Y")
        cyl.CreateRadiusAttr(cylinder[0])
        cyl.CreateHeightAttr(cylinder[1])
        cyl.CreatePurposeAttr(UsdGeom.Tokens.guide)   # 렌더 제외
        UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
        set_offsets(cyl.GetPrim())
        if material:
            UsdShade.MaterialBindingAPI.Apply(cyl.GetPrim()).Bind(
                UsdShade.Material.Get(stage, material),
                bindingStrength=UsdShade.Tokens.weakerThanDescendants,
                materialPurpose="physics")
        return link
    meshes = [p for p in Usd.PrimRange(prim) if p.IsA(UsdGeom.Mesh)]
    if not meshes:
        raise RuntimeError(f"{path}: Mesh 없음")
    for p in meshes:
        UsdPhysics.CollisionAPI.Apply(p)
        UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr(approx)
        set_offsets(p)
        if material:
            UsdShade.MaterialBindingAPI.Apply(p).Bind(
                UsdShade.Material.Get(stage, material),
                bindingStrength=UsdShade.Tokens.weakerThanDescendants,
                materialPurpose="physics")
    return link


# ══ 링크 ══
M_BF = rx(90.0) * tr(SEG_CENTER, 0, 0)
M_BR = rx(90.0) * tr(-SEG_CENTER, 0, 0)
make_link(f"{ROBOT}/body_front", M_BF, "body_front.usd", MASS_BODY, "convexHull")
make_link(f"{ROBOT}/body_rear", M_BR, "body_rear.usd", MASS_BODY, "convexHull")

# 벨로우즈 — 시각 전용, 충돌체 금지 (v3). body_front 에 붙인다
bel = UsdGeom.Xform.Define(stage, f"{ROBOT}/body_front/bellows_visual")
bel.AddTransformOp().Set(
    Gf.Matrix4d(1.0).SetScale(Gf.Vec3d(MM, MM, MM)) * rx(90.0) * M_BF.GetInverse())
bel.GetPrim().GetReferences().AddReference(str(SRC / "bellows.usd"))

arm_specs = []          # (tag, body_path, M_body, M_arm, M_wheel)
wheel_drives = []       # 주행 시작 시 목표 속도를 넣을 드라이브들
for seg, angles, sgn in (("f", META["front_arms"], +1), ("r", META["rear_arms"], -1)):
    body_path = f"{ROBOT}/body_front" if sgn > 0 else f"{ROBOT}/body_rear"
    M_body = M_BF if sgn > 0 else M_BR
    for a in angles:
        tag = f"{seg}_{int(a)}"
        roll = Gf.Rotation(Gf.Vec3d(1, 0, 0), float(a))
        p_pivot = Gf.Vec3d(*roll.TransformDir(
            Gf.Vec3d(sgn * ARM_PIVOT_X, 0, PIVOT_R)))
        p_wheel = Gf.Vec3d(*roll.TransformDir(
            Gf.Vec3d(sgn * (ARM_PIVOT_X - ARM_DX), 0, WHEEL_CENTER_R)))
        mirror = rz(180.0) if sgn < 0 else Gf.Matrix4d(1.0)
        M_arm = mirror * rx(float(a)) * tr(*p_pivot)
        M_wheel = rx(float(a)) * tr(*p_wheel)
        make_link(f"{ROBOT}/arm_{tag}", M_arm, "arm.usd", MASS_ARM, "convexHull")
        # 실린더 프리미티브는 커스텀 지오메트리라 GPU 물리(--water)에서 미지원
        # — 12차: GPU 에서 초기 관통 → 로봇 사출(+316m) 실측. 물 모드는
        # convexHull 로 대체 (에지 채터로 슬립 다소 증가 — 데모 용도 수용)
        make_link(f"{ROBOT}/wheel_{tag}", M_wheel, "wheel.usd", MASS_WHEEL,
                  "convexHull", material=MAT,
                  cylinder=None if WATER else (WHEEL_R, META["wheel_width"] * MM))
        arm_specs.append((tag, body_path, M_body, M_arm, M_wheel))


def define_revolute(path, path0, M0, path1, M1, anchor_world, axis="Y"):
    """월드 앵커/조인트 프레임(= body1 회전)을 두 링크의 로컬로 변환해 조인트 생성."""
    j = UsdPhysics.RevoluteJoint.Define(stage, path)
    j.CreateBody0Rel().SetTargets([path0])
    j.CreateBody1Rel().SetTargets([path1])
    j.CreateAxisAttr(axis)
    RJ = rot_only(M1)                   # 조인트 프레임 = body1 로컬 프레임
    for k, M in ((0, M0), (1, M1)):
        lp = M.GetInverse().Transform(anchor_world)
        L = RJ * rot_only(M).GetInverse()
        q = Gf.Quatf(L.ExtractRotation().GetQuat())
        getattr(j, f"CreateLocalPos{k}Attr")(Gf.Vec3d(lp))
        getattr(j, f"CreateLocalRot{k}Attr")(q)
    return j


# 중앙 관절: rear(부모) → front, 요(Z) 1축 근사. 수동 + 센터링 스프링
jc = define_revolute(f"{ROBOT}/joint_center", f"{ROBOT}/body_rear", M_BR,
                     f"{ROBOT}/body_front", M_BF, Gf.Vec3d(0, 0, 0), axis="Z")
jc.CreateLowerLimitAttr(-CENTER_LIMIT)
jc.CreateUpperLimitAttr(CENTER_LIMIT)
dc = UsdPhysics.DriveAPI.Apply(jc.GetPrim(), "angular")
dc.CreateTypeAttr("force")
dc.CreateStiffnessAttr(CENTER_STIFF)
dc.CreateDampingAttr(CENTER_DAMP)
dc.CreateTargetPositionAttr(0.0)

# 스윙암 + 휠 조인트
for tag, body_path, M_body, M_arm, M_wheel in arm_specs:
    ja = define_revolute(f"{ROBOT}/joint_arm_{tag}", body_path, M_body,
                         f"{ROBOT}/arm_{tag}", M_arm,
                         M_arm.Transform(Gf.Vec3d(0, 0, 0)), axis="Y")
    ja.CreateLowerLimitAttr(ARM_LIMIT_LO)
    ja.CreateUpperLimitAttr(ARM_LIMIT_HI)
    da = UsdPhysics.DriveAPI.Apply(ja.GetPrim(), "angular")
    da.CreateTypeAttr("force")
    da.CreateStiffnessAttr(ARM_STIFF)
    da.CreateDampingAttr(ARM_DAMP)
    da.CreateTargetPositionAttr(ARM_TARGET)
    da.CreateMaxForceAttr(ARM_MAX_TORQUE)

    jw = define_revolute(f"{ROBOT}/joint_wheel_{tag}", f"{ROBOT}/arm_{tag}", M_arm,
                         f"{ROBOT}/wheel_{tag}", M_wheel,
                         M_wheel.Transform(Gf.Vec3d(0, 0, 0)), axis="Y")
    dw = UsdPhysics.DriveAPI.Apply(jw.GetPrim(), "angular")
    dw.CreateTypeAttr("force")
    dw.CreateStiffnessAttr(0.0)
    dw.CreateDampingAttr(WHEEL_DAMP)
    # 정착은 무구동으로 — 5차: 정착 중에도 바퀴가 돌아 로봇이 접힌 채(요 55°
    # 스토퍼, 롤 99°) 출발했다. CPU 물리라 런타임 목표 변경이 반영된다.
    # 예외: --water 는 GPU dynamics 라 reset 후 USD 드라이브 변경이 무시됨 —
    # 목표를 지금 기록한다 (정착 중에도 바퀴가 도는 건 감수)
    dw.CreateTargetVelocityAttr(WHEEL_TARGET_DEG_S if WATER else 0.0)
    dw.CreateMaxForceAttr(WHEEL_MAX_TORQUE)
    if not (FRONT_DRIVE and tag.startswith("r")):
        wheel_drives.append(dw)          # front-drive 모드: 후방은 무구동 유지

UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
px = PhysxSchema.PhysxArticulationAPI.Apply(root.GetPrim())
px.CreateSolverPositionIterationCountAttr(64)
px.CreateSolverVelocityIterationCountAttr(4)
px.CreateEnabledSelfCollisionsAttr(False)

# ══ 배관 (직관 600mm, 삼각 메시) ══
if NO_PIPE:
    # 부호 검증 모드: 배관 없음 + 중력 끔 — 예압 방향만 본다
    world.get_physics_context().set_gravity(0.0)
UsdGeom.Xform.Define(stage, "/World/Pipe")
pm = UsdPhysics.MaterialAPI.Apply(
    UsdShade.Material.Define(stage, "/World/Pipe/Mat").GetPrim())
pm.CreateStaticFrictionAttr(FRICTION[0])
pm.CreateDynamicFrictionAttr(FRICTION[1])
pm.CreateRestitutionAttr(0.0)
def _scale_mat():
    m = Gf.Matrix4d(1.0)
    m.SetScale(Gf.Vec3d(MM, MM, MM))
    return m


_pipe_parts = []
if not NO_PIPE:
    _pipe_parts.append(("straight", "pipe_straight.usd", _scale_mat()))
if ELBOW:
    # 엘보 STL: 입구 x=0 평면(축 X), 출구 로컬 (100,0,100) 축 Z, R=100.
    # Rx(-90) 으로 굽힘을 수평(x-y)으로 눕히면 출구가 (100,100,0) 축 +Y.
    # 직관 끝 x=0.30 에 입구를 붙인다 → 출구 (0.40, 0.10) 축 +Y
    _pipe_parts.append(("elbow", "pipe_elbow_sr.usd",
                        _scale_mat() * rx(-90.0) * tr(0.300, 0.0, 0.0)))
    # 출구 직관: +Y 방향, 출구 (0.40, 0.10) 에서 시작 → 중심 (0.40, 0.40)
    _pipe_parts.append(("exit", "pipe_straight.usd",
                        _scale_mat() * rz(90.0) * tr(0.400, 0.400, 0.0)))
for name, usd, m in _pipe_parts:
    xf = UsdGeom.Xform.Define(stage, f"/World/Pipe/{name}")
    xf.AddTransformOp().Set(m)
    xf.GetPrim().GetReferences().AddReference(str(SRC / usd))

# 유리 재질 — 밖에서 로봇·물이 보이게. displayOpacity 프림바 방식은 RTX
# 뷰포트에서 안 먹히는 경우가 있어 water_particle_demo 에서 검증된
# UsdPreviewSurface 반투명 바인딩(strongerThanDescendants)을 쓴다
glass = UsdShade.Material.Define(stage, "/World/Pipe/Glass")
gsh = UsdShade.Shader.Define(stage, "/World/Pipe/Glass/Shader")
gsh.CreateIdAttr("UsdPreviewSurface")
gsh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
    Gf.Vec3f(0.55, 0.58, 0.60))
gsh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.25)
gsh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
glass.CreateSurfaceOutput().ConnectToSource(gsh.ConnectableAPI(), "surface")
for p in stage.Traverse():
    if str(p.GetPath()).startswith("/World/Pipe/") and p.IsA(UsdGeom.Mesh):
        UsdPhysics.CollisionAPI.Apply(p)
        UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr("none")
        set_offsets(p)
        UsdShade.MaterialBindingAPI.Apply(p).Bind(
            UsdShade.Material.Get(stage, "/World/Pipe/Mat"),
            bindingStrength=UsdShade.Tokens.weakerThanDescendants,
            materialPurpose="physics")
        UsdShade.MaterialBindingAPI.Apply(p).Bind(
            glass, bindingStrength=UsdShade.Tokens.strongerThanDescendants)

# ══ 물 (--water): 흐르는 PBD 파티클 + 재순환 ══
water_instancer = None
if WATER:
    psys_path = Sdf.Path("/World/ParticleSystem")
    particleUtils.add_physx_particle_system(
        stage, psys_path,
        particle_contact_offset=W_PCO,
        fluid_rest_offset=W_FLUID_REST,
        contact_offset=0.004,          # 입자-강체 간격 — 미지정 시 로봇 부상 함정
        rest_offset=0.0025,
        max_velocity=5.0,
        wind=Gf.Vec3f(FLOW_V, 0.0, 0.0),
    )
    # isosurface 수면 렌더 (KB: 메모리 누수 이슈 — 짧은 데모 세션 전용)
    particleUtils.add_physx_particle_isosurface(
        stage, psys_path, enabled=True,
        grid_spacing=W_FLUID_REST * 1.5,
        surface_distance=W_FLUID_REST * 1.6,
        grid_smoothing_radius=W_FLUID_REST * 2.0,
        num_mesh_smoothing_passes=8, num_mesh_normal_smoothing_passes=8)
    wv_mat = UsdShade.Material.Define(stage, "/World/WaterVisual")
    wv_sh = UsdShade.Shader.Define(stage, "/World/WaterVisual/Shader")
    wv_sh.CreateIdAttr("UsdPreviewSurface")
    # 유리 배관 너머로도 물이 또렷이 보여야 한다 — 실사용 보고: opacity 0.45
    # 는 유리(0.25)와 겹치면 거의 안 보임. 진한 파랑 + 강한 자체발광 + 불투명
    wv_sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.15, 0.45, 0.85))
    wv_sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.10, 0.28, 0.55))
    wv_sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.85)
    wv_sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.15)
    wv_mat.CreateSurfaceOutput().ConnectToSource(wv_sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(psys_path)).Bind(
        wv_mat, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
    w_pbd = UsdShade.Material.Define(stage, "/World/WaterPBD")
    particleUtils.AddPBDMaterialWater(w_pbd.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(psys_path)).Bind(
        w_pbd, bindingStrength=UsdShade.Tokens.weakerThanDescendants,
        materialPurpose="physics")

    _wpos = []
    if ELBOW:
        # 코스 전체(직관→엘보→출구 직관)를 경로 중심선 거리로 채운다.
        # 경로: 직선 (-0.28,0)~(0.30,0) + 원호 중심(0.30,0.10) r0.10 + 직선
        # (0.40,0.10)~(0.40,0.62). 굽힘이 수평이라 수위 z 는 전 구간 동일
        gx = np.arange(-0.28, 0.47, W_SPACING)
        gy = np.arange(-0.05, 0.63, W_SPACING)
        gz = np.arange(-W_R_MAX, W_LEVEL_Z + 1e-9, W_SPACING)
        X, Y, Z = np.meshgrid(gx, gy, gz, indexing="ij")
        d1 = np.hypot(X - np.clip(X, -0.28, 0.30), Y)
        ang = np.clip(np.arctan2(Y - 0.10, X - 0.30), -np.pi / 2, 0.0)
        d2 = np.hypot(X - (0.30 + 0.10 * np.cos(ang)),
                      Y - (0.10 + 0.10 * np.sin(ang)))
        d3 = np.hypot(X - 0.40, Y - np.clip(Y, 0.10, 0.62))
        dxy = np.minimum(np.minimum(d1, d2), d3)
        inside = (dxy ** 2 + Z ** 2 <= W_R_MAX ** 2)
        inside &= ~((np.abs(X - START_X) < 0.10) & (np.abs(Y) < 0.06))
        for x, y, z in zip(X[inside], Y[inside], Z[inside]):
            _wpos.append(Gf.Vec3f(float(x), float(y), float(z)))
    else:
        for x in np.arange(W_X[0], W_X[1], W_SPACING):
            if abs(x - START_X) < 0.10:   # 로봇 자리 비움 — 정착 중 흘러들어온다
                continue
            for y in np.arange(-W_R_MAX, W_R_MAX + 1e-9, W_SPACING):
                for z in np.arange(-W_R_MAX, W_LEVEL_Z + 1e-9, W_SPACING):
                    if y * y + z * z <= W_R_MAX * W_R_MAX:
                        _wpos.append(Gf.Vec3f(float(x), float(y), float(z)))
    _wvel = [Gf.Vec3f(FLOW_V, 0.0, 0.0)] * len(_wpos)
    _winst_prim = particleUtils.add_physx_particleset_pointinstancer(
        stage, Sdf.Path("/World/WaterParticles"),
        Vt.Vec3fArray(_wpos), Vt.Vec3fArray(_wvel),
        particle_system_path=psys_path,
        self_collision=True, fluid=True, particle_group=0,
        particle_mass=0.0, density=1000.0)
    UsdGeom.Sphere(stage.GetPrimAtPath(
        "/World/WaterParticles/particlePrototype0")).GetRadiusAttr().Set(
        W_FLUID_REST)
    water_instancer = UsdGeom.PointInstancer(_winst_prim)
    UsdGeom.Imageable(_winst_prim).MakeInvisible()   # isosurface 가 대신 렌더

_rng_flow = np.random.default_rng(3)
_recycled = 0


def recycle_particles():
    """하류로 나간 입자를 상류로 재투입 — 고정 입자 수로 연속 흐름 표현."""
    global _recycled
    pts = np.array(water_instancer.GetPositionsAttr().Get())
    r2 = pts[:, 1] ** 2 + pts[:, 2] ** 2
    mask = ((pts[:, 0] < RECYCLE_OUT_X) | (pts[:, 0] > 0.31) | (r2 > 0.055 ** 2))
    n = int(mask.sum())
    if n == 0:
        return
    pts[mask, 0] = _rng_flow.uniform(RECYCLE_IN_X[0], RECYCLE_IN_X[1], n)
    pts[mask, 1] = _rng_flow.uniform(-0.02, 0.02, n)
    pts[mask, 2] = _rng_flow.uniform(W_LEVEL_Z, W_LEVEL_Z + 0.015, n)
    vels = np.array(water_instancer.GetVelocitiesAttr().Get())
    vels[mask] = (FLOW_V, 0.0, -0.2)   # 물 위에서 떨어뜨려 합류 (압력 폭발 방지)
    water_instancer.GetPositionsAttr().Set(
        Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))
    water_instancer.GetVelocitiesAttr().Set(
        Vt.Vec3fArray.FromNumpy(vels.astype(np.float32)))
    _recycled += n

art = SingleArticulation(prim_path=ROBOT, name="yongbin_robot")
world.scene.add(art)
world.reset()
body = SingleRigidPrim(prim_path=f"{ROBOT}/body_front", name="bf")

dof = list(art.dof_names)
arm_idx = [k for k, n in enumerate(dof) if n.startswith("joint_arm_")]
wheel_idx = [k for k, n in enumerate(dof) if n.startswith("joint_wheel_")]
center_idx = dof.index("joint_center")


def body_roll_deg():
    """본체 X축 롤(조립 기준 Rx90° 제거 후) — 커지면 나선 주행 = 횡 스크럽."""
    q = body.get_world_pose()[1]          # w, x, y, z
    w, x, y, z = (float(v) for v in q)
    raw = np.degrees(np.arctan2(2 * (w * x + y * z),
                                1 - 2 * (x * x + y * y)))
    return raw - 90.0                     # 링크 프레임에 Rx90 조립 회전 포함
dt = world.get_physics_dt()

print("\n" + "=" * 78)
print("yongbin 실설계 주행 검증 (v3 §13 스윙암 예압 9N)")
print("=" * 78)
print(f"DOF {len(dof)}: 중앙 1 + 암 {len(arm_idx)} + 휠 {len(wheel_idx)}")
print(f"휠 순서: {[dof[k] for k in wheel_idx]}")
print(f"암 조인트 한계 {ARM_LIMIT_LO:+.1f}~{ARM_LIMIT_HI:+.1f}° "
      f"(절대 33.4~58.2°), 목표 {ARM_TARGET:+.1f}°, 예상 접촉각 "
      f"{CONTACT_JOINT_DEG:+.2f}°")
print(f"휠 목표 {WHEEL_TARGET_DEG_S:.0f} deg/s (= 50 mm/s), 최대토크 "
      f"{WHEEL_MAX_TORQUE*1000:.1f} mN·m")
if WATER:
    print(f"물: 입자 {len(_wpos):,}개, 유속 {FLOW_V} m/s(-X, 재순환), "
          f"마찰 {FRICTION[0]}/{FRICTION[1]} (만관), GPU dynamics ON")

if not HEADLESS:
    if ELBOW:
        # 코스 전체가 보이는 외부 부감 (배관은 유리 — 내부가 비쳐 보인다)
        set_camera_view(eye=[0.15, -0.55, 0.45], target=[0.20, 0.15, 0.0])
    else:
        # 관내 촬영 기본 앵글: 로봇 전방 관내에서 로봇을 마주 본다
        set_camera_view(eye=[START_X + 0.30, 0.0, 0.006],
                        target=[START_X, 0.0, 0.0])


def arm_deg():
    """암 조인트 각(°). 주의: API 반환은 라디안 — 3~4차에서 도로 착각해
    0.26 rad(=상한 14.9°)를 0.26°로 읽고 밀착 오판했다."""
    jp = art.get_joint_positions()
    return np.degrees(np.array([float(jp[k]) for k in arm_idx]))


_wheel_prims = [SingleRigidPrim(prim_path=f"{ROBOT}/wheel_{t}", name=f"w_{t}")
                for t, _, _, _, _ in arm_specs]


def wheel_center_r_mm():
    """휠 중심의 월드 반경(mm). 에지 접촉 시 39.43 — 이보다 작으면 미접촉."""
    out = []
    for w in _wheel_prims:
        p = w.get_world_pose()[0]
        out.append(np.sqrt(float(p[1]) ** 2 + float(p[2]) ** 2) * 1000)
    return np.array(out)


# ── 정착: 예압이 휠을 관벽에 눌러 붙일 때까지 (물 모드: 유입 서지 소멸도) ──
# 물 모드는 GPU 제약으로 바퀴가 정착 중에도 돈다 — 주행 거리는 여기서부터 잰다
x_pre_settle = float(body.get_world_pose()[0][0])
settle_spin = 0.0
for _s in range(SETTLE_STEPS):
    # 재순환은 흐름 모드 전용 — 엘보(고인 물)는 이탈 판정(r² 축 기준)이
    # 출구 직관 구간을 오판해 물을 통째로 되돌려 버린다
    if WATER and FLOW_V != 0 and _s % RECYCLE_EVERY == 0:
        recycle_particles()
    sim_step(render=not HEADLESS)
    if WATER:
        _v = art.get_joint_velocities()
        settle_spin += (sum(float(_v[k]) for k in wheel_idx)
                        / len(wheel_idx) * world.get_physics_dt())
a0 = arm_deg()
p0 = body.get_world_pose()[0]
wr0 = wheel_center_r_mm()
print(f"\n정착 후 암 조인트(°): {np.array2string(a0, precision=2)}")
print(f"휠 중심 반경(mm): {np.array2string(wr0, precision=2)} (접촉 = 39.43)")
print(f"본체 높이 {float(p0[2])*1000:+.2f} mm (0 = 중심)")
on_wall = np.abs(wr0 - 39.43) < 0.5
print(f"밀착 판정: {int(on_wall.sum())}/6 휠 중심이 39.43±0.5mm → "
      f"{'밀착 OK' if on_wall.all() else '일부/전부 미접촉'}")
print(f"정착 자세: 롤 {body_roll_deg():+.1f}°, 요 "
      f"{np.degrees(float(art.get_joint_positions()[center_idx])):+.1f}°")

if NO_PIPE:
    print("\n[부호 검증] 기대값: 암 전부 +14.9°(신장 한계), 휠 중심 반경 "
          "~46mm (자유 최대 신장). 반대 부호면 -9.9° / ~31mm 로 간다")
    simulation_app.close()
    sys.exit(0)

# 주행 개시 — 이제야 바퀴에 목표 속도 부여 (물 모드는 reset 전에 이미 설정
# — GPU 에서 지금 바꿔봐야 무시된다)
if not WATER:
    for dw in wheel_drives:
        dw.GetTargetVelocityAttr().Set(WHEEL_TARGET_DEG_S)

# ── 주행 ──
x0 = x_pre_settle if WATER else float(body.get_world_pose()[0][0])
spin = settle_spin if WATER else 0.0
wheel_spin = np.zeros(len(wheel_idx))    # 휠별 구간 누적(rad) — 슬립 분리용
print(f"\n{'스텝':>6} {'주행':>9} {'이론':>9} {'슬립률':>7} {'높이':>8} "
      f"암범위(°)  휠별 평균 rad/s")
max_bend = 0.0                           # 곡관 통과 중 관절 최대 굽힘(°)
elbow_done = False
for step in range(1, DRIVE_STEPS + 1):
    if WATER and FLOW_V != 0 and step % RECYCLE_EVERY == 0:
        recycle_particles()
    sim_step(render=not HEADLESS)
    vel = art.get_joint_velocities()
    wv_now = np.array([float(vel[k]) for k in wheel_idx])
    wheel_spin += wv_now * dt
    spin += wv_now.mean() * dt
    p = body.get_world_pose()[0]
    if step % 48 == 0:
        yaw_now = np.degrees(float(art.get_joint_positions()[center_idx]))
        max_bend = max(max_bend, abs(yaw_now))
    if ELBOW:
        if float(p[1]) > 0.55:
            elbow_done = True
            print(f"→ 전체 코스 주파: 직관 550mm + 엘보 90° + 출구 직관 "
                  f"450mm (스텝 {step}, {step * dt:.1f} s)")
            break
    elif float(p[0]) > (STOP_X_WATER if WATER else 0.24):
        print(f"→ 배관 끝 근접 — 정지 (스텝 {step})")
        break
    if step % REPORT_EVERY == 0:
        travel = (float(p[0]) - x0) * 1000
        ideal = -WHEEL_R * spin * 1000     # 휠 +X 전진 = 음의 회전
        slip = travel / ideal if abs(ideal) > 1e-6 else 0.0
        ar = arm_deg()
        wavg = wheel_spin / (REPORT_EVERY * dt)
        wheel_spin[:] = 0.0
        wv = " ".join(f"{w:+5.2f}" for w in wavg)
        jp = art.get_joint_positions()
        pos_txt = (f"xy {float(p[0])*1000:>4.0f},{float(p[1])*1000:>4.0f}mm"
                   if ELBOW else f"{travel:>7.1f}mm {ideal:>7.1f}mm "
                                 f"{slip:>7.3f}")
        print(f"{step:>6} {pos_txt} "
              f"{float(p[2])*1000:>+6.1f}mm  {ar.min():+.1f}~{ar.max():+.1f}  "
              f"{wv}  롤 {body_roll_deg():+6.1f}° "
              f"요 {np.degrees(float(jp[center_idx])):+5.1f}°")

p_end = body.get_world_pose()[0]
travel = (float(p_end[0]) - x0) * 1000
ideal = -WHEEL_R * spin * 1000
slip = travel / ideal if abs(ideal) > 1e-6 else 0.0
a1 = arm_deg()
wr1 = wheel_center_r_mm()
print("=" * 78)
print(f"결과: 주행 {travel:.1f} mm / 이론 {ideal:.1f} mm → 슬립률 {slip:.3f}")
print(f"주행 후 암 조인트(°): {np.array2string(a1, precision=2)}")
print(f"주행 후 휠 반경(mm): {np.array2string(wr1, precision=2)}")
if ELBOW:
    print(f"관절 최대 굽힘 {max_bend:.1f}° (v3 SR 기하 예측 36.1°, 한계 55°)")
    print(f"최종 위치 x {float(p_end[0])*1000:.0f}, y {float(p_end[1])*1000:.0f} mm")
    print(f"[{'OK' if elbow_done else 'FAIL'}] "
          f"{'직관+SR엘보+출구직관 전체 코스 주파' if elbow_done else '코스 미완주 — 위 위치에서 정체'}")
elif WATER:
    wpts = np.array(water_instancer.GetPositionsAttr().Get())
    wr_all = np.sqrt(wpts[:, 1] ** 2 + wpts[:, 2] ** 2)
    leaked = int(np.sum(wr_all > 0.055))
    print(f"물: 입자 {len(wpts):,}개, 재순환 누적 {_recycled:,}개 "
          f"(0이면 GPU 위치 쓰기 미반영), 관벽 관통 {leaked}개")
    ok = travel > 100.0
    print(f"[{'OK' if ok else 'FAIL'}] 흐르는 물 거슬러 {travel:.0f}mm 전진, "
          f"슬립률 {slip:.3f} (합격: 100mm+)")
else:
    ok_contact = np.abs(wr1 - 39.43) < 0.5
    ok = ok_contact.all() and travel > 100.0 and 0.85 < slip < 1.15
    print(f"[{'OK' if ok else 'FAIL'}] 밀착 {int(ok_contact.sum())}/6, "
          f"주행 {travel:.0f}mm, 슬립률 {slip:.3f} "
          f"(합격: 6/6 밀착 + 100mm+ + 슬립 0.85~1.15)")

if not HEADLESS:
    print("\nGUI 실행 중 — 창을 닫으면 종료됩니다.")
    _k = 0
    while simulation_app.is_running():
        _k += 1
        if WATER and FLOW_V != 0 and _k % RECYCLE_EVERY == 0:
            recycle_particles()
        sim_step(render=True)

simulation_app.close()
