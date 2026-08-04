"""[Isaac 3.11] 2단 1관절 로봇 조립 — **씬에 직접 짓는다. USD 파일을 거치지 않는다.**

## 왜 이렇게 바꿨나 (2026-08-04)

원래는 `articulate.py` 가 `robot_2seg.usd` 를 구워 저장하고, 주행 스크립트가
그것을 `AddReference` 로 불러왔다. 그 구조에서 **관 안 주행이 0mm 였다.**
같은 PC · 같은 Isaac 에서 씬에 직접 조립하는 방식(yongbin_drive_test.py)은
1,112.7mm 를 주행했다(슬립 0.996, 밀착 6/6). 실측 대조다.

son 라인에서 아래를 전부 바꿔 봤지만 0mm 는 그대로였다 — 즉 값의 문제가 아니다.

  예압(설계의 97~100% 확인) · 휠 최대토크(12.1 → 40.5 mN·m)
  물리 스텝(1/60 → 1/240)   · 크라운(제거해 contact_r 49.434 로 복귀)
  구동 지령 방식(apply_action → USD TargetVelocityAttr)
  로딩 방식(변환 op 을 articulation root → 상위 Xform 으로 이동)

남은 차이가 **조립 방식** 하나였다. 그래서 파일 왕복을 없앤다.

## 부수 효과 — 사고의 근원이 통째로 사라진다

- `.usd` 는 `.gitignore` 대상이라 레포에 없다 → 읽는 쪽이 그 파일이 지금 소스에서
  나온 것인지 알 수 없었다. STL 세대가 다른 로봇을 조용히 읽는 사고가 실제로 났다
- 형상·상수를 고쳐도 낡은 USD 를 계속 읽어 "수정이 반영 안 된다" 오진이 났고,
  그 우회로 "다른 디렉터리에서 만들어 덮어쓰기" 가 생겼다
- Isaac 을 2회 기동해야 했다(조립 + 주행). 이제 1회다

## 값의 출처

치수는 `spec/parts_meta.json`, 물리 상수는 이 파일이 유일한 출처다.
부르는 쪽에서 상수를 다시 적지 말고 이 모듈에서 읽을 것.
"""

import json
import math
import os
import struct
from pathlib import Path

import numpy as np
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade

HERE = Path(__file__).resolve().parent
SON = HERE.parent
META = json.loads((SON / "spec" / "parts_meta.json").read_text())

_CATEGORY = {
    "body_rear": "robot", "body_front": "robot", "bellows": "robot",
    "arm": "robot", "wheel": "robot",
    "torch_ring": "welder", "torch_rod": "welder", "torch_tip": "welder",
    "camera_housing": "camera",
    "pipe_straight": "pipe", "pipe_elbow_sr": "pipe",
}


def part_path(name):
    return SON / _CATEGORY[name] / "meshes" / f"{name}.stl"


MM = 0.001

SEG_DX = META["seg_center_offset"] * MM
PIVOT_R = META["pivot_r"] * MM
ARM_LEN = META["arm_len"] * MM
ARM_PSI = META["arm_angle_nominal"]
ARM_DX = META["arm_dx_nominal"] * MM
# 진단 노브: 휠 중심 반경 = PIVOT_R + ARM_DR. 접촉 반경이 관 내반경과
# 정확히 같으면(여유 0) 곡관에서 벽까지 거리가 변할 때 즉시 물린다.
# yongbin 은 여유 0.566mm 를 갖는다. 기본값은 parts_meta 그대로.
ARM_DR = float(os.environ.get("ARM_DR_MM", META["arm_dr_nominal"])) * MM
WHEEL_R = META["wheel_r"] * MM
WHEEL_WIDTH = META.get("wheel_width", 15.0) * MM
# 진단 노브: 축방향 스태거. son 은 토치 링 여유 때문에 10→7mm 로 줄였고
# yongbin(곡관 통과 성공)은 10mm 다. 곡관 필요 스트로크에 직접 영향.
STAGGER = float(os.environ.get("STAGGER_MM", META["stagger"])) * MM
FRONT_ARMS = META["front_arms"]
REAR_ARMS = META["rear_arms"]

ARM_LIMIT_LOWER = META["arm_angle_compressed"] - ARM_PSI
# ⚠ 신장 한계를 env 로 흔드는 노브를 뒀다가 제거했다. ARM_DRIVE_TARGET_DEG 가
# 이 값에서 파생되고 ARM_K = 토크/target 이라 **스프링 강성이 같이 바뀐다** —
# 단일 변수 시험이 성립하지 않는다(실측: 한계를 올리자 강성 1.6배 약화, 주행 악화).
# 스트로크를 시험하려면 강성을 고정한 채 한계만 바꾸도록 따로 설계할 것.
ARM_LIMIT_UPPER = META["arm_angle_extended"] - ARM_PSI

WAIST_LIMIT = 55.0
# 중앙 관절 센터링 스프링. 0.01 로는 잭나이프를 못 막는다 —
# 팀 10~14차 스윕에서 0.3 + 댐퍼 0.05 로 확정된 값이다.
WAIST_SPRING_NM_RAD = 0.3
WAIST_DAMP_NM_S_RAD = 0.05

MASS_BODY_KG = 0.190
MASS_ARM_KG = 0.012
MASS_WHEEL_KG = 0.008
MASS_TOTAL_KG = 2 * MASS_BODY_KG + 6 * MASS_ARM_KG + 6 * MASS_WHEEL_KG

WHEEL_PRELOAD_N = 9.0
ARM_MOMENT_M = ARM_LEN * math.cos(math.radians(ARM_PSI))
ARM_TORQUE_NM = WHEEL_PRELOAD_N * ARM_MOMENT_M

ARM_OVERTRAVEL_DEG = 6.0
ARM_DRIVE_TARGET_DEG = ARM_LIMIT_UPPER + ARM_OVERTRAVEL_DEG
ARM_K_NM_RAD = ARM_TORQUE_NM / math.radians(ARM_DRIVE_TARGET_DEG)

DEG = math.pi / 180.0
ARM_K_USD = ARM_K_NM_RAD * DEG
ARM_C_USD = ARM_K_USD / 10.0
ARM_MAX_FORCE = 1.5 * ARM_K_NM_RAD * math.radians(
    ARM_DRIVE_TARGET_DEG - ARM_LIMIT_LOWER)

WAIST_K_USD = WAIST_SPRING_NM_RAD * DEG
WAIST_C_USD = WAIST_DAMP_NM_S_RAD * DEG   # 스윕 확정값. K/10 이 아니다

# 설계 v3 §12.3 은 마찰을 **두 벌**로 나눈다. 하나로 뭉뚱그리면 안 된다.
#   physics_flooded.yaml  정찰기  static 0.30 / dynamic 0.25  (관에 물이 찬 상태)
#   physics_drained.yaml  수리기  static 0.40 / dynamic 0.35  (배수 후)
WHEEL_FRICTION_STATIC = 0.30            # 정찰기(만관)
WHEEL_FRICTION_DYNAMIC = 0.25
WHEEL_FRICTION_DRAINED_STATIC = 0.40    # 수리기(배수 후) — 수중 아크 용접 불가
WHEEL_FRICTION_DRAINED_DYNAMIC = 0.35
WHEEL_FRICTION = WHEEL_FRICTION_STATIC        # 견인력 계산은 정지마찰 기준

# 마찰이 낼 수 있는 최대 접선력에 대응하는 토크. **상한이지 목표가 아니다.**
WHEEL_FRICTION_LIMIT_NM = WHEEL_FRICTION * WHEEL_PRELOAD_N * WHEEL_R

# 🚨 구동 토크를 마찰 한계까지 쓰면 안 된다. 마찰은 원(circle)이라 구동이
# 전부 소비하면 횡방향 안내력이 0 이 되어 중앙 관절이 스토퍼까지 접힌다(잭나이프).
# 설계값 0.45. 진단용으로만 env 로 덮는다 — 벗어나면 경고한다.
WHEEL_TORQUE_FRACTION_DESIGN = 0.45     # 소스를 읽는 시험이 있으므로 리터럴 유지
WHEEL_TORQUE_FRACTION = float(
    os.environ.get("WHEEL_TORQUE_FRACTION", WHEEL_TORQUE_FRACTION_DESIGN))
WHEEL_MAX_TORQUE = WHEEL_FRICTION_LIMIT_NM * WHEEL_TORQUE_FRACTION
if abs(WHEEL_TORQUE_FRACTION - WHEEL_TORQUE_FRACTION_DESIGN) > 1e-9:
    print(f"[진단] WHEEL_TORQUE_FRACTION={WHEEL_TORQUE_FRACTION} "
          f"(설계 0.45) — 설계값이 아니다. 시험용으로만 쓸 것.")

# 🚨 USD 각도 드라이브의 damping 단위는 N·m/(deg/s) 다. 변환을 빼면 57.3배 커진다.
WHEEL_DAMPING_REF_RAD_S = 5.0
WHEEL_DAMPING = WHEEL_MAX_TORQUE / WHEEL_DAMPING_REF_RAD_S * DEG

CONTACT_OFFSET = 0.0005
REST_OFFSET = 0.0

# ── 토치 모듈 (수리기 전용) ───────────────────────────────────────────
# 🚨 J1 은 ±180° 다. 360° 연속 회전은 슬립링이 필요하고 슬립링은 용접 전류에
# 부적합하다. ±180° 로도 전 방향에 도달하며 케이블도 안 꼬인다.
TORCH = META["torch"]
RING_X = TORCH["ring_x"] * MM
RING_R = TORCH["rod_origin_r"] * MM
ROD_LEN = TORCH["rod_len"] * MM
J1_LIMIT = TORCH["j1_limit_deg"]
J2_STROKE = TORCH["j2_stroke_mm"] * MM

MASS_RING_KG = 0.045
MASS_ROD_KG = 0.010
MASS_TIP_KG = 0.008
MASS_TORCH_KG = MASS_RING_KG + MASS_ROD_KG + MASS_TIP_KG

J1_STIFF_NM_RAD = 2.0
J2_STIFF_N_M = 400.0

N_ARM = len(FRONT_ARMS) + len(REAR_ARMS)
N_LINK = 2 + 2 * N_ARM          # body 2 + arm 6 + wheel 6
N_DOF = 1 + 2 * N_ARM           # 관절 1 + 서스펜션 6 + 바퀴 6
N_LINK_WELDER = N_LINK + 3      # + torch ring / rod / tip
N_DOF_WELDER = N_DOF + 2        # + J1 회전 / J2 직동 (tip 은 fixed)


def load_stl(path):
    data = Path(path).read_bytes()
    n = struct.unpack("<I", data[80:84])[0]
    a = np.frombuffer(data[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    tri = a[:, 12:48].copy().view("<f4").reshape(n * 3, 3).astype(np.float64)
    pts, inv = np.unique(np.round(tri, 5), axis=0, return_inverse=True)
    return pts * MM, inv.reshape(n, 3)


def make_mesh(stage, path, stl_name):
    """STL 을 UsdGeom.Mesh 로 직접 저작한다.

    STL→USD 변환기를 안 쓰므로 변환 체인에 scale op 이 아예 생기지 않는다
    (정점을 읽을 때 m 로 바꾼다). scale 이 rigid body 체인에 끼면 PhysX 가 어긋난다.
    """
    pts, idx = load_stl(part_path(stl_name))
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    mesh.CreateFaceVertexCountsAttr([3] * len(idx))
    mesh.CreateFaceVertexIndicesAttr(idx.reshape(-1).tolist())
    lo, hi = pts.min(0), pts.max(0)
    mesh.CreateExtentAttr([Gf.Vec3f(*lo), Gf.Vec3f(*hi)])
    mesh.CreateSubdivisionSchemeAttr("none")
    return mesh


def rx(deg):
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), deg))
    return m


def tr(x, y, z):
    m = Gf.Matrix4d(1.0)
    m.SetTranslate(Gf.Vec3d(x, y, z))
    return m


def rz(deg):
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), deg))
    return m


def quat_z(deg):
    return Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 0, 1), deg).GetQuat())


def quat_x(deg):
    return Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0, 0), deg).GetQuat())


def arm_slots():
    """(tag, 세그먼트 중심 x, 인덱스, 원주각, 스태거, 부호) 를 낸다.

    🚨 sgn 은 **후방 세그먼트의 암을 반대로 뻗게** 하는 부호다(yongbin 방식).
    예전에는 전·후방 암이 모두 -X 로 뻗어 로봇이 앞뒤로 비대칭이었고, 그 결과
    직진 중 중앙 관절이 -8~-14° 로 꺾인 채 **전방 3륜이 관벽에서 들렸다**
    (실측 밀착 3/6, f2 는 2.49mm 부상 → 혼자 632° 헛돎, 슬립 0.94).
    앞뒤를 대칭으로 두면 전방이 끌려 들리지 않는다.
    """
    for tag, xc, angles, sgn in (("r", -SEG_DX, REAR_ARMS, -1.0),
                                 ("f", +SEG_DX, FRONT_ARMS, +1.0)):
        for i, phi in enumerate(angles):
            yield tag, xc, i, phi, (i - 1) * STAGGER, sgn


def _set_bodies(j, p0, p1):
    j.CreateBody0Rel().SetTargets([p0])
    j.CreateBody1Rel().SetTargets([p1])


def build(stage, root="/World/Robot", origin=(0.0, 0.0, 0.0),
          welder=False, drained=None, verbose=True):
    """`root` 아래에 로봇을 짓는다. 파일을 저장하지 않는다.

    welder : True 면 토치 모듈(링·로드·팁 3링크, J1·J2 2DOF)을 함께 짓는다.
             수리기와 정찰기는 같은 섀시라 조립 코드를 나누지 않는다.
    drained: 배수 상태 마찰(0.40/0.35)을 쓸지. 기본은 `welder` 를 따른다 —
             설계 v3 §12.3 은 정찰을 만관(0.30/0.25), 수리를 배수 후로
             나눈다(수중 아크 용접이 성립하지 않기 때문). 하나로
             뭉뚱그리면 안 된다.
    origin : 로봇을 놓을 월드 좌표(m). **articulation root 프림에 변환 op 을
             걸지 않고 각 링크 변환에 반영한다** — root 에 변환을 걸면 PhysX 가
             어긋난다(yongbin 방식과 동일).

    반환: {"links": [...], "joints": {...}} — 부르는 쪽이 이름을 다시 적지 않게.
    """
    ox, oy, oz = origin
    world_off = tr(ox, oy, oz)
    if drained is None:
        drained = welder
    mu_s = WHEEL_FRICTION_DRAINED_STATIC if drained else WHEEL_FRICTION_STATIC
    mu_d = WHEEL_FRICTION_DRAINED_DYNAMIC if drained else WHEEL_FRICTION_DYNAMIC

    UsdGeom.Xform.Define(stage, root)

    UsdGeom.Scope.Define(stage, f"{root}/PhysicsMaterials")
    wheel_mat = f"{root}/PhysicsMaterials/WheelMaterial"
    _m = UsdPhysics.MaterialAPI.Apply(
        UsdShade.Material.Define(stage, wheel_mat).GetPrim())
    _m.CreateStaticFrictionAttr(mu_s)
    _m.CreateDynamicFrictionAttr(mu_d)
    _m.CreateRestitutionAttr(0.0)

    links = []

    def make_link(path, matrix, stl_name, mass_kg, approximation,
                  material=None, extra_visual=None, cylinder=None):
        """cylinder=(radius, height) 면 충돌체를 **해석적 원통 프리미티브**로 만든다.

        🚨 휠에 convexHull 을 쓰면 안 된다. 메시 hull 은 트레드 둘레가 다각형이라
        **날카로운 에지가 관벽을 문다.** 실측 증상: 각속도는 나오는데 관절 위치가
        안 쌓이고 부호가 매 스텝 뒤집힌다(스틱슬립 채터) → 주행 0mm.
        yongbin 라인 주석의 3차 실측도 같다: "hull 에지 접촉 + 예압 9N + 8g 휠 =
        스틱슬립 채터로 슬립률 0.73". v3 §13.4 도 "휠은 실린더 프리미티브" 다.
        시각 메시(크라운 포함)는 그대로 두고 충돌체만 원통으로 바꾼다.
        """
        link = UsdGeom.Xform.Define(stage, path)
        link.AddTransformOp().Set(matrix * world_off)
        prim = link.GetPrim()

        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass_kg)

        if cylinder is not None:
            vis = make_mesh(stage, f"{path}/visual", stl_name)
            UsdGeom.Imageable(vis).CreatePurposeAttr(UsdGeom.Tokens.default_)
            cyl = UsdGeom.Cylinder.Define(stage, f"{path}/collider")
            cyl.CreateAxisAttr("Y")
            cyl.CreateRadiusAttr(cylinder[0])
            cyl.CreateHeightAttr(cylinder[1])
            cyl.CreatePurposeAttr(UsdGeom.Tokens.guide)      # 렌더 제외
            mp = cyl.GetPrim()
        else:
            mesh = make_mesh(stage, f"{path}/collision", stl_name)
            mp = mesh.GetPrim()
            UsdPhysics.MeshCollisionAPI.Apply(mp).CreateApproximationAttr(
                approximation)
        UsdPhysics.CollisionAPI.Apply(mp)
        px = PhysxSchema.PhysxCollisionAPI.Apply(mp)
        px.CreateContactOffsetAttr(CONTACT_OFFSET)
        px.CreateRestOffsetAttr(REST_OFFSET)
        if material is not None:
            UsdShade.MaterialBindingAPI.Apply(mp).Bind(
                UsdShade.Material.Get(stage, material),
                bindingStrength=UsdShade.Tokens.weakerThanDescendants,
                materialPurpose="physics")
        if extra_visual is not None:
            vname, vmat = extra_visual
            vm = make_mesh(stage, f"{path}/{vname}", vname)
            UsdGeom.Xformable(vm).AddTransformOp().Set(vmat)
        links.append(path)
        if verbose:
            print(f"  link {path:34s} mass {mass_kg * 1000:6.1f} g  "
                  f"({approximation})")

    if verbose:
        print("=" * 78)
        print("링크 생성 — 씬에 직접 (USD 파일 없음)")
        print("=" * 78)

    make_link(f"{root}/body_rear", tr(-SEG_DX, 0, 0), "body_rear",
              MASS_BODY_KG, "convexHull",
              extra_visual=("bellows", tr(SEG_DX, 0, 0)))
    make_link(f"{root}/body_front", tr(+SEG_DX, 0, 0), "body_front",
              MASS_BODY_KG, "convexHull")

    for tag, xc, i, phi, xo, sgn in arm_slots():
        # ⚠ 후방 암 미러링(yongbin 방식)을 두 번 시도했다가 모두 되돌렸다.
        #   1차: 조인트 암쪽 프레임에 Rz(180) → φ=60·300 인 암만 프레임이 1.73
        #        어긋나 그 두 바퀴가 반경 27mm 로 파묻힘. 주행 517.8 → 0.1mm
        #   2차: 프레임을 Rx(φ)·Rz(180)·Rx(-φ) 로 켤레 보정(수치 검증 0 오차).
        #        반경은 33.98·37.55 로 개선됐으나 여전히 밀착 실패,
        #        전방 3륜 회전 0° — 구동이 후방에만 들어감. 주행 0.0mm
        # 링크 변환·조인트 프레임만으로는 부족하고 암 STL 의 마운트 방향까지
        # 같이 봐야 한다. 지금은 비대칭(전·후 암 모두 -X) 을 유지한다.
        m_arm = tr(0, 0, PIVOT_R) * rx(-phi) * tr(xc + xo, 0, 0)
        make_link(f"{root}/arm_{tag}{i}", m_arm, "arm",
                  MASS_ARM_KG, "convexHull")
        m_wh = tr(-ARM_DX, 0, PIVOT_R + ARM_DR) * rx(-phi) * tr(xc + xo, 0, 0)
        make_link(f"{root}/wheel_{tag}{i}", m_wh, "wheel", MASS_WHEEL_KG,
                  "convexHull", material=wheel_mat,
                  cylinder=(WHEEL_R, WHEEL_WIDTH))

    # ── 중앙 관절 (수동 + 센터링 스프링) ──────────────────────────────
    jw = UsdPhysics.RevoluteJoint.Define(stage, f"{root}/joint_waist")
    _set_bodies(jw, f"{root}/body_rear", f"{root}/body_front")
    jw.CreateAxisAttr("Y")
    jw.CreateLocalPos0Attr(Gf.Vec3d(+SEG_DX, 0, 0))
    jw.CreateLocalRot0Attr(Gf.Quatf(1.0))
    jw.CreateLocalPos1Attr(Gf.Vec3d(-SEG_DX, 0, 0))
    jw.CreateLocalRot1Attr(Gf.Quatf(1.0))
    jw.CreateLowerLimitAttr(-WAIST_LIMIT)
    jw.CreateUpperLimitAttr(+WAIST_LIMIT)
    dw = UsdPhysics.DriveAPI.Apply(jw.GetPrim(), "angular")
    dw.CreateTypeAttr("force")
    dw.CreateStiffnessAttr(WAIST_K_USD)
    dw.CreateDampingAttr(WAIST_C_USD)
    dw.CreateTargetPositionAttr(0.0)
    dw.CreateMaxForceAttr(10.0 * WAIST_K_USD * WAIST_LIMIT)

    # ── 서스펜션(암) + 바퀴 ───────────────────────────────────────────
    for tag, xc, i, phi, xo, sgn in arm_slots():
        seg = "body_rear" if tag == "r" else "body_front"
        j = UsdPhysics.RevoluteJoint.Define(
            stage, f"{root}/joint_arm_{tag}{i}")
        _set_bodies(j, f"{root}/{seg}", f"{root}/arm_{tag}{i}")
        j.CreateAxisAttr("Y")
        j.CreateLocalPos0Attr(Gf.Vec3d(xo,
                                       PIVOT_R * math.sin(math.radians(phi)),
                                       PIVOT_R * math.cos(math.radians(phi))))
        j.CreateLocalRot0Attr(quat_x(-phi))
        j.CreateLocalPos1Attr(Gf.Vec3d(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0))
        j.CreateLowerLimitAttr(ARM_LIMIT_LOWER)
        j.CreateUpperLimitAttr(ARM_LIMIT_UPPER)
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(ARM_K_USD)
        d.CreateDampingAttr(ARM_C_USD)
        d.CreateTargetPositionAttr(ARM_DRIVE_TARGET_DEG)
        d.CreateMaxForceAttr(ARM_MAX_FORCE)

        j2 = UsdPhysics.RevoluteJoint.Define(
            stage, f"{root}/joint_wheel_{tag}{i}")
        _set_bodies(j2, f"{root}/arm_{tag}{i}", f"{root}/wheel_{tag}{i}")
        j2.CreateAxisAttr("Y")
        j2.CreateLocalPos0Attr(Gf.Vec3d(-ARM_DX, 0, ARM_DR))
        j2.CreateLocalRot0Attr(Gf.Quatf(1.0))
        j2.CreateLocalPos1Attr(Gf.Vec3d(0, 0, 0))
        j2.CreateLocalRot1Attr(Gf.Quatf(1.0))
        d2 = UsdPhysics.DriveAPI.Apply(j2.GetPrim(), "angular")
        d2.CreateTypeAttr("force")
        d2.CreateStiffnessAttr(0.0)
        d2.CreateDampingAttr(WHEEL_DAMPING)
        d2.CreateTargetVelocityAttr(0.0)
        d2.CreateMaxForceAttr(WHEEL_MAX_TORQUE)

    # ── 토치 모듈 (수리기) ────────────────────────────────────────────
    if welder:
        make_link(f"{root}/torch_ring", tr(RING_X, 0, 0), "torch_ring",
                  MASS_RING_KG, "convexDecomposition")
        make_link(f"{root}/torch_rod", tr(RING_X, 0, RING_R), "torch_rod",
                  MASS_ROD_KG, "convexHull")
        make_link(f"{root}/torch_tip", tr(RING_X, 0, RING_R + ROD_LEN),
                  "torch_tip", MASS_TIP_KG, "convexHull")

        # J1: 링 회전 (body_front → torch_ring), 축 X = 관 중심축.
        # 바퀴는 전·후진만 되고 관절도 수동이라 로봇을 결함 방향으로 돌릴
        # 수단이 없다. 대신 본체를 감싸는 이 링이 돌아 토치를 겨눈다.
        j1 = UsdPhysics.RevoluteJoint.Define(stage, f"{root}/joint_torch_j1")
        _set_bodies(j1, f"{root}/body_front", f"{root}/torch_ring")
        j1.CreateAxisAttr("X")
        j1.CreateLocalPos0Attr(Gf.Vec3d(RING_X - SEG_DX, 0, 0))
        j1.CreateLocalRot0Attr(Gf.Quatf(1.0))
        j1.CreateLocalPos1Attr(Gf.Vec3d(0, 0, 0))
        j1.CreateLocalRot1Attr(Gf.Quatf(1.0))
        j1.CreateLowerLimitAttr(-J1_LIMIT)
        j1.CreateUpperLimitAttr(+J1_LIMIT)
        d1 = UsdPhysics.DriveAPI.Apply(j1.GetPrim(), "angular")
        d1.CreateTypeAttr("force")
        d1.CreateStiffnessAttr(J1_STIFF_NM_RAD * DEG)
        d1.CreateDampingAttr(J1_STIFF_NM_RAD * DEG / 10.0)
        d1.CreateTargetPositionAttr(0.0)
        d1.CreateMaxForceAttr(20.0 * J1_STIFF_NM_RAD * DEG)

        # J2: 토치 직동 (torch_ring → torch_rod), 반경 방향 = 링 로컬 Z
        j2 = UsdPhysics.PrismaticJoint.Define(stage, f"{root}/joint_torch_j2")
        _set_bodies(j2, f"{root}/torch_ring", f"{root}/torch_rod")
        j2.CreateAxisAttr("Z")
        j2.CreateLocalPos0Attr(Gf.Vec3d(0, 0, RING_R))
        j2.CreateLocalRot0Attr(Gf.Quatf(1.0))
        j2.CreateLocalPos1Attr(Gf.Vec3d(0, 0, 0))
        j2.CreateLocalRot1Attr(Gf.Quatf(1.0))
        j2.CreateLowerLimitAttr(0.0)
        j2.CreateUpperLimitAttr(J2_STROKE)
        d2t = UsdPhysics.DriveAPI.Apply(j2.GetPrim(), "linear")
        d2t.CreateTypeAttr("force")
        d2t.CreateStiffnessAttr(J2_STIFF_N_M)
        d2t.CreateDampingAttr(J2_STIFF_N_M / 10.0)
        d2t.CreateTargetPositionAttr(0.0)
        d2t.CreateMaxForceAttr(3.0 * J2_STIFF_N_M * J2_STROKE)

        # 토치 끝은 로드에 고정. 자유도를 늘리지 않는다.
        jf = UsdPhysics.FixedJoint.Define(stage, f"{root}/joint_torch_tip")
        _set_bodies(jf, f"{root}/torch_rod", f"{root}/torch_tip")
        jf.CreateLocalPos0Attr(Gf.Vec3d(0, 0, ROD_LEN))
        jf.CreateLocalRot0Attr(Gf.Quatf(1.0))
        jf.CreateLocalPos1Attr(Gf.Vec3d(0, 0, 0))
        jf.CreateLocalRot1Attr(Gf.Quatf(1.0))

    root_prim = stage.GetPrimAtPath(root)
    UsdPhysics.ArticulationRootAPI.Apply(root_prim)
    pxa = PhysxSchema.PhysxArticulationAPI.Apply(root_prim)
    pxa.CreateSolverPositionIterationCountAttr(64)
    pxa.CreateSolverVelocityIterationCountAttr(4)
    pxa.CreateEnabledSelfCollisionsAttr(False)

    if verbose:
        print(f"  마찰  정지 {mu_s} / 운동 {mu_d}  "
              f"({'배수(수리기)' if drained else '만관(정찰기)'})")
    return {"links": links, "root": root, "welder": welder,
            "drained": drained, "mu_static": mu_s, "mu_dynamic": mu_d,
            "n_link": N_LINK_WELDER if welder else N_LINK,
            "n_dof": N_DOF_WELDER if welder else N_DOF}
