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
# 엘보+물(--elbow --water): 임무 조건 그대로 — 직관+곡관이 이어진 코스 전체가
# 물로 차 있고, 그 물이 코스를 따라 흐르며, 로봇은 그 흐름을 거슬러 주파한다.
# 유속장(wind)은 전역 단방향이라 굽은 경로에 못 쓰므로, recycle_particles()
# 에서 입자마다 경로 접선(path_dist_tangent)을 구해 상류→하류로 속도를 끌어
# 당긴다. 굽힘이 수평(x-y)이라 수위 z=일정 이 코스 전체에서 성립한다
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
if ELBOW:
    # 10~12차 스윕 결론: 0.01(v3)은 직진에서 잭나이프, 1.0 은 엘보에서 쐐기.
    # 0.3 N·m/rad + 댐퍼 0.05 + 휠 토크 30 mN·m 가 직진·곡관 겸용 확정값 —
    # 코스 주행이 기본 목적인 --elbow 에서는 이 조합을 기본값으로 쓴다
    CENTER_STIFF = 0.3 * np.pi / 180.0
if STIFF_JOINT:
    CENTER_STIFF = 1.0 * np.pi / 180.0
# --spring K: 센터링 스프링(N·m/rad) 직접 지정 — 직진 안정 vs 곡관 통과 절충
# 스윕용. 10차: 1.0 은 직진 완벽 / 엘보에서 쐐기+발산, 0.01(v3)은 엘보 통과 /
# 직진 잭나이프. 중간값을 찾는다
if "--spring" in sys.argv:
    CENTER_STIFF = float(sys.argv[sys.argv.index("--spring") + 1]) * np.pi / 180.0
# --damp D: 요 댐퍼(N·m·s/rad). 14차 착안: 잭나이프는 빠른 발산, 곡관 굽힘은
# 준정적 — 스프링은 각도에만 반응해 둘을 못 가르지만 댐퍼는 속도에만 반응하므로
# "빠르게 접히는 건 막고 천천히 꺾이는 건 허용"이 원리적으로 가능하다
if "--damp" in sys.argv:
    CENTER_DAMP = float(sys.argv[sys.argv.index("--damp") + 1]) * np.pi / 180.0

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
if ELBOW:
    WHEEL_MAX_TORQUE = 0.030   # 곡관에선 스프링 복원 토크를 이겨야 한다 (12차)
# --torque T: 휠 토크 상한(N·m) 지정. 11차: 엘보에서는 스프링 복원 토크를
# 이기며 전진해야 해서 15 mN·m 로는 정체 — 스프링으로 직진이 안정화된 뒤에는
# 토크를 올려도 잭나이프가 재발하지 않는지가 관건
if "--torque" in sys.argv:
    WHEEL_MAX_TORQUE = float(sys.argv[sys.argv.index("--torque") + 1])
WHEEL_DAMP = WHEEL_MAX_TORQUE / 60.0                      # 60 deg/s 오차서 최대

FRICTION = (0.30, 0.25) if WATER else (0.40, 0.35)   # 만관(정찰)/배수(수리)
PHYS_DT = 1.0 / 240.0                 # v3 §13.2
RENDER_DT = 1.0 / 60.0                # World 기본값

# ── 루프 1회의 시뮬 경과시간 (GUI/headless 가 다르다) ──
# isaacsim.core.api SimulationContext: set_simulation_dt 가
#   substeps = max(int(rendering_dt / physics_dt), 1) = 4
# 로 잡고, step(render=True) 는 _app.update() 로 렌더 1프레임 = 4 물리 서브스텝을
# 돌린다. step(render=False) 는 물리 1스텝. 즉 같은 "스텝 수"라도 GUI 는
# headless 의 4배 시간이 흐른다. 14차: 이 차이를 dt 로 반영 안 해서 GUI 슬립률이
# 4배(0.58 → 2.33)로 찍혔다. 시간 기준 값은 전부 STEP_DT 로 환산할 것
STEP_DT = PHYS_DT if HEADLESS else RENDER_DT


def _steps(sec):
    return max(1, int(round(sec / STEP_DT)))


SETTLE_SEC = 2.0
DRIVE_SEC = 20.0
REPORT_SEC = 2.0
START_X = -0.20
STOP_Y_ELBOW = 0.55                   # 출구 직관에서 주파 판정하는 y
if ELBOW:
    # 전체 코스: 직관 시작점부터 550mm 직진 → 엘보 90° → 출구 직관 450mm.
    # 실제 임무 조건대로 직선·곡선이 이어진 경로를 한 번에 통과해야 한다
    START_X = -0.25
    DRIVE_SEC = 70.0                  # 코스 ~1.15m
    REPORT_SEC = 5.0
if WATER:
    SETTLE_SEC = 6.0                  # 물 유입 서지가 가라앉을 시간
    DRIVE_SEC = 50.0 if not ELBOW else 100.0   # 물 저항 감속 감안
    REPORT_SEC = 5.0
    # 14차 실측: 물 모드는 set_offsets 를 건너뛰어(GPU 접촉 소실 회피) 엔진
    # 기본 contactOffset 0.02m 가 쓰인다. START_X=-0.25 면 후방 3휠이 직관
    # 입구(x=-0.30) 밖 -335mm 에 놓여 관 끝 링(r50~57)과 20mm 범위에서 겹치고,
    # 그 관통 해소 임펄스로 로봇이 -X 로 1.66 m/s 사출 → 이후 자유낙하
    # (본체 -178m). 후방 휠 뒷면이 관 안에 들어오는 -0.20 으로 당긴다
    START_X = max(START_X, -0.20)
    FLOW_V = -0.10                    # 경로 접선 기준 유속(상류 방향, 로봇 진행 반대)
    FLOW_BLEND = 0.20                 # 입자 속도를 유동 목표로 끌어당기는 계수
    W_SPACING = 0.009
    W_PCO = 0.008
    W_FLUID_REST = 0.0048
    W_LEVEL_Z = 0.020                 # 유동 자연 수위 (부분 충전 흐름)
    W_R_MAX = 0.045
    W_X = (-0.28, 0.28)
    RECYCLE_OUT_X = -0.28             # 하류 이탈 → 상류 재투입 (러닝머신)
    RECYCLE_IN_X = (0.22, 0.29)
    # 유동 강제 주기는 "시간" 기준 1/60 s — GUI 에서 스텝 수로 고정하면 4배
    # 뜸해져 유속이 55 → 44 mm/s 로 떨어진다 (14차 GUI 실측)
    RECYCLE_EVERY = _steps(1.0 / 60.0)
    STOP_X_WATER = 0.15               # 재투입 지점 앞에서 정지
    if ELBOW:
        # 엘보 코스의 "하류"는 직관 입구(x=-0.30), "상류"는 출구 직관 끝.
        # 물은 출구 직관 위쪽에서 투입돼 굽은 경로를 타고 입구로 빠져나가고,
        # 로봇은 그 흐름을 거슬러 올라간다 (정찰 임무 조건)
        RECYCLE_OUT_X = -0.30
        RECYCLE_IN_Y = (0.60, 0.65)   # 출구 직관 상단(수면 위)에서 낙하 합류
        # 로봇 주파 판정선(0.55)은 재투입 지점보다 하류 — 낙하 합류 물살이
        # 결승선의 로봇을 때리지 않게 한다

SETTLE_STEPS = _steps(SETTLE_SEC)
DRIVE_STEPS = _steps(DRIVE_SEC)
REPORT_EVERY = _steps(REPORT_SEC)
BEND_EVERY = _steps(0.2)              # 관절 굽힘 최대값 샘플링 주기

# ── 엘보 코스 중심선: 직선(-0.30~0.30, y=0) → 원호(중심 0.30,0.10 / r0.10)
#    → 직선(x=0.40, y=0.10~0.70). 물 채우기와 유동장이 같은 기하를 쓴다 ──
ARC_C = (0.300, 0.100)
ARC_R = 0.100


def path_dist_tangent(px, py):
    """xy 점 배열 → (중심선까지 xy 거리, 접선 x, 접선 y). 접선은 로봇 진행 방향."""
    d1 = np.hypot(px - np.clip(px, -0.30, 0.30), py)
    ang = np.clip(np.arctan2(py - ARC_C[1], px - ARC_C[0]), -np.pi / 2, 0.0)
    d2 = np.hypot(px - (ARC_C[0] + ARC_R * np.cos(ang)),
                  py - (ARC_C[1] + ARC_R * np.sin(ang)))
    d3 = np.hypot(px - 0.40, py - np.clip(py, 0.10, 0.70))
    d = np.stack([d1, d2, d3])
    which = np.argmin(d, axis=0)
    tx = np.where(which == 0, 1.0, np.where(which == 1, -np.sin(ang), 0.0))
    ty = np.where(which == 0, 0.0, np.where(which == 1, np.cos(ang), 1.0))
    return d.min(axis=0), tx, ty


def path_s(px, py):
    """코스 중심선 진행거리(m). 직관 입구 x=-0.30 이 0, 굽힘·출구 직관까지 이어짐.

    엘보 코스에서 x 변위는 주행량이 아니다(굽고 나면 x 가 멎는다) — 슬립률은
    이 곡선거리로 재야 의미가 있다."""
    px = np.atleast_1d(np.asarray(px, dtype=float))
    py = np.atleast_1d(np.asarray(py, dtype=float))
    ang = np.clip(np.arctan2(py - ARC_C[1], px - ARC_C[0]), -np.pi / 2, 0.0)
    d1 = np.hypot(px - np.clip(px, -0.30, 0.30), py)
    d2 = np.hypot(px - (ARC_C[0] + ARC_R * np.cos(ang)),
                  py - (ARC_C[1] + ARC_R * np.sin(ang)))
    d3 = np.hypot(px - 0.40, py - np.clip(py, 0.10, 0.70))
    which = np.argmin(np.stack([d1, d2, d3]), axis=0)
    s_arc0 = 0.60                                   # 직관 구간 길이
    s = np.where(which == 0, np.clip(px, -0.30, 0.30) + 0.30,
                 np.where(which == 1, s_arc0 + ARC_R * (ang + np.pi / 2),
                          s_arc0 + ARC_R * np.pi / 2
                          + (np.clip(py, 0.10, 0.70) - 0.10)))
    return float(s[0]) if s.size == 1 else s


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
        # 전역 wind 는 단방향이라 굽은 경로에 못 쓴다 — 엘보 코스는 0 으로 두고
        # recycle_particles() 에서 입자별 경로 접선 방향으로 속도를 끌어당긴다
        wind=Gf.Vec3f(0.0, 0.0, 0.0) if ELBOW else Gf.Vec3f(FLOW_V, 0.0, 0.0),
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
    _wvel_elbow = []
    if ELBOW:
        # 코스 전체(직관→엘보→출구 직관)를 경로 중심선 거리로 채운다.
        # 경로: 직선 (-0.30,0)~(0.30,0) + 원호 중심(0.30,0.10) r0.10 + 직선
        # (0.40,0.10)~(0.40,0.70). 굽힘이 수평이라 수위 z 는 전 구간 동일
        gx = np.arange(-0.28, 0.47, W_SPACING)
        gy = np.arange(-0.05, 0.66, W_SPACING)
        gz = np.arange(-W_R_MAX, W_LEVEL_Z + 1e-9, W_SPACING)
        X, Y, Z = np.meshgrid(gx, gy, gz, indexing="ij")
        dxy, TX, TY = path_dist_tangent(X, Y)
        inside = (dxy ** 2 + Z ** 2 <= W_R_MAX ** 2)
        inside &= ~((np.abs(X - START_X) < 0.10) & (np.abs(Y) < 0.06))
        for x, y, z, tx, ty in zip(X[inside], Y[inside], Z[inside],
                                   TX[inside], TY[inside]):
            _wpos.append(Gf.Vec3f(float(x), float(y), float(z)))
            # 초기 속도도 경로 접선의 반대(상류→하류 = 로봇 진행 반대)로
            _wvel_elbow.append(Gf.Vec3f(float(-tx * abs(FLOW_V)),
                                        float(-ty * abs(FLOW_V)), 0.0))
    else:
        for x in np.arange(W_X[0], W_X[1], W_SPACING):
            if abs(x - START_X) < 0.10:   # 로봇 자리 비움 — 정착 중 흘러들어온다
                continue
            for y in np.arange(-W_R_MAX, W_R_MAX + 1e-9, W_SPACING):
                for z in np.arange(-W_R_MAX, W_LEVEL_Z + 1e-9, W_SPACING):
                    if y * y + z * z <= W_R_MAX * W_R_MAX:
                        _wpos.append(Gf.Vec3f(float(x), float(y), float(z)))
    _wvel = _wvel_elbow if ELBOW else [Gf.Vec3f(FLOW_V, 0.0, 0.0)] * len(_wpos)
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
_flow_speed_now = 0.0        # 최근 유동 실측(경로 접선 성분 평균, m/s)


def recycle_particles():
    """하류로 나간 입자를 상류로 재투입 — 고정 입자 수로 연속 흐름 표현.

    엘보 코스는 여기서 유동장까지 만든다: 전역 wind 는 단방향이라 굽은 경로에
    쓸 수 없으므로 입자마다 경로 접선을 구해 상류→하류 방향으로 속도를
    끌어당긴다(계수 FLOW_BLEND). 직관 코스는 기존대로 wind 가 흐름을 맡고
    여기서는 이탈 입자 재투입만 한다.
    """
    global _recycled, _flow_speed_now
    pts = np.array(water_instancer.GetPositionsAttr().Get())
    vels = np.array(water_instancer.GetVelocitiesAttr().Get())
    if ELBOW:
        d, tx, ty = path_dist_tangent(pts[:, 0], pts[:, 1])
        spd = abs(FLOW_V)
        # 로봇 진행 방향(접선)의 반대 = 상류에서 하류로 흐른다
        vels[:, 0] += FLOW_BLEND * (-tx * spd - vels[:, 0])
        vels[:, 1] += FLOW_BLEND * (-ty * spd - vels[:, 1])
        _flow_speed_now = float(np.mean(-(vels[:, 0] * tx + vels[:, 1] * ty)))
        # 이탈: 하류 끝(직관 입구) 통과 · 관벽 관통 · 상류 끝 초과
        off = np.hypot(d, pts[:, 2])
        mask = ((pts[:, 0] < RECYCLE_OUT_X) | (off > 0.055)
                | (pts[:, 1] > RECYCLE_IN_Y[1] + 0.02))
        n = int(mask.sum())
        if n:
            pts[mask, 0] = 0.40 + _rng_flow.uniform(-0.02, 0.02, n)
            pts[mask, 1] = _rng_flow.uniform(*RECYCLE_IN_Y, n)
            pts[mask, 2] = _rng_flow.uniform(W_LEVEL_Z, W_LEVEL_Z + 0.015, n)
            vels[mask] = (0.0, -spd, -0.2)   # 수면 위에서 낙하 합류
    else:
        r2 = pts[:, 1] ** 2 + pts[:, 2] ** 2
        mask = ((pts[:, 0] < RECYCLE_OUT_X) | (pts[:, 0] > 0.31)
                | (r2 > 0.055 ** 2))
        n = int(mask.sum())
        if n == 0:
            return
        pts[mask, 0] = _rng_flow.uniform(RECYCLE_IN_X[0], RECYCLE_IN_X[1], n)
        pts[mask, 1] = _rng_flow.uniform(-0.02, 0.02, n)
        pts[mask, 2] = _rng_flow.uniform(W_LEVEL_Z, W_LEVEL_Z + 0.015, n)
        vels[mask] = (FLOW_V, 0.0, -0.2)   # 물 위에서 떨어뜨려 합류
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


# 루프 1회의 시뮬 경과시간 — GUI 는 rendering_dt(4 물리 서브스텝), headless 는
# physics_dt. 실제 값과 어긋나면 슬립률·경과시간이 통째로 틀리므로 검증한다
dt = STEP_DT
_rdt = world.get_rendering_dt()
if not HEADLESS and abs(_rdt - STEP_DT) > 1e-9:
    print(f"[경고] rendering_dt {_rdt:.5f} ≠ 가정 {STEP_DT:.5f} — 보정")
    dt = _rdt

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
print(f"중앙 관절: 스프링 {CENTER_STIFF*180/np.pi:.3f} N·m/rad, 댐퍼 "
      f"{CENTER_DAMP*180/np.pi:.3f} N·m·s/rad, 시작 x {START_X*1000:.0f}mm")
if WATER:
    print(f"물: 입자 {len(_wpos):,}개, 유속 {abs(FLOW_V)} m/s "
          f"({'경로 접선 상류→하류' if ELBOW else '-X'}, 재순환), "
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
    """휠 중심의 관 중심선까지 거리(mm). 에지 접촉 시 39.43 — 작으면 미접촉.

    엘보 코스는 중심선이 굽으므로 축 반경(√(y²+z²))이 아니라 경로 중심선까지의
    거리로 잰다 (직관 구간에서는 둘이 같다)."""
    out = []
    for w in _wheel_prims:
        p = w.get_world_pose()[0]
        x, y, z = float(p[0]), float(p[1]), float(p[2])
        if ELBOW:
            d, _, _ = path_dist_tangent(np.array([x]), np.array([y]))
            out.append(float(np.hypot(d[0], z)) * 1000)
        else:
            out.append(np.sqrt(y * y + z * z) * 1000)
    return np.array(out)


# ── 정착: 예압이 휠을 관벽에 눌러 붙일 때까지 (물 모드: 유입 서지 소멸도) ──
# 물 모드는 GPU 제약으로 바퀴가 정착 중에도 돈다 — 주행 거리는 여기서부터 잰다
x_pre_settle = float(body.get_world_pose()[0][0])
settle_spin = 0.0
for _s in range(SETTLE_STEPS):
    # 유동 갱신 + 이탈 입자 재투입 (엘보는 경로 접선 유동장도 여기서 만든다)
    if WATER and FLOW_V != 0 and _s % RECYCLE_EVERY == 0:
        recycle_particles()
    sim_step(render=not HEADLESS)
    if WATER:
        _v = art.get_joint_velocities()
        settle_spin += (sum(float(_v[k]) for k in wheel_idx)
                        / len(wheel_idx) * dt)
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
# 엘보 코스는 곡선거리 기준 (x 변위는 굽고 나면 멎는다)
s0 = path_s(x0, 0.0) if ELBOW else 0.0
spin = settle_spin if WATER else 0.0
wheel_spin = np.zeros(len(wheel_idx))    # 휠별 구간 누적(rad) — 슬립 분리용
print(f"\n{'스텝':>6} {'주행':>9} {'이론':>9} {'슬립률':>7} {'높이':>8} "
      f"암범위(°)  휠별 평균 rad/s")
max_bend = 0.0                           # 관절 |요| 최대(°) — 구간 무관
# 구간별 요 이력: (코스 곡선거리 mm, 요°). 14차: |요| 최대만 보면 직진 크랩과
# 곡관 굽힘이 섞여 "곡관에서 26° 꺾였다"고 오독한다 — 실제로는 곡관 안에서
# 가장 곧았다. 굽힘은 **구간별 각도와 변화량**으로 봐야 한다
yaw_trace = []
S_ARC = (600.0, 757.1)                   # 곡관(원호) 구간의 코스 곡선거리 mm
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
    if step % BEND_EVERY == 0:
        yaw_now = np.degrees(float(art.get_joint_positions()[center_idx]))
        max_bend = max(max_bend, abs(yaw_now))
        if ELBOW:
            # (코스 위치, 요, 바퀴가 말하는 누적거리) — 구간별 슬립 산출용.
            # 오도메트리가 주 측위 수단이라 "어느 구간에서 몇 mm 틀리는가"가
            # 전체 평균 슬립보다 훨씬 중요한 값이다
            yaw_trace.append((path_s(float(p[0]), float(p[1])) * 1000, yaw_now,
                              -WHEEL_R * spin * 1000))
    if ELBOW:
        if float(p[1]) > STOP_Y_ELBOW:
            elbow_done = True
            print(f"→ 전체 코스 주파: 직관 + 엘보 90° + 출구 직관 "
                  f"(스텝 {step}, {step * dt:.1f} s)")
            break
    elif float(p[0]) > (STOP_X_WATER if WATER else 0.24):
        print(f"→ 배관 끝 근접 — 정지 (스텝 {step})")
        break
    if step % REPORT_EVERY == 0:
        travel = ((path_s(float(p[0]), float(p[1])) - s0) * 1000 if ELBOW
                  else (float(p[0]) - x0) * 1000)
        ideal = -WHEEL_R * spin * 1000     # 휠 전진 = 음의 회전
        slip = travel / ideal if abs(ideal) > 1e-6 else 0.0
        ar = arm_deg()
        wavg = wheel_spin / (REPORT_EVERY * dt)
        wheel_spin[:] = 0.0
        wv = " ".join(f"{w:+5.2f}" for w in wavg)
        jp = art.get_joint_positions()
        pos_txt = (f"s{travel:>6.0f}mm xy{float(p[0])*1000:>4.0f},"
                   f"{float(p[1])*1000:>4.0f} 슬립{slip:>6.3f}" if ELBOW
                   else f"{travel:>7.1f}mm {ideal:>7.1f}mm {slip:>7.3f}")
        flow_txt = f" 유속 {_flow_speed_now*1000:+4.0f}mm/s" if (WATER and ELBOW) else ""
        print(f"{step:>6} {pos_txt} "
              f"{float(p[2])*1000:>+6.1f}mm  {ar.min():+.1f}~{ar.max():+.1f}  "
              f"{wv}  롤 {body_roll_deg():+6.1f}° "
              f"요 {np.degrees(float(jp[center_idx])):+5.1f}°{flow_txt}")

p_end = body.get_world_pose()[0]
travel = ((path_s(float(p_end[0]), float(p_end[1])) - s0) * 1000 if ELBOW
          else (float(p_end[0]) - x0) * 1000)
ideal = -WHEEL_R * spin * 1000
slip = travel / ideal if abs(ideal) > 1e-6 else 0.0
a1 = arm_deg()
wr1 = wheel_center_r_mm()
print("=" * 78)
print(f"결과: 주행 {travel:.1f} mm{' (코스 곡선거리)' if ELBOW else ''} / "
      f"이론 {ideal:.1f} mm → 슬립률 {slip:.3f}")
print(f"주행 후 암 조인트(°): {np.array2string(a1, precision=2)}")
print(f"주행 후 휠 반경(mm): {np.array2string(wr1, precision=2)}")


def water_stats():
    """물 상태 요약 — 재순환 0 이면 GPU 위치 쓰기가 안 먹은 것."""
    wpts = np.array(water_instancer.GetPositionsAttr().Get())
    if ELBOW:
        d, _, _ = path_dist_tangent(wpts[:, 0], wpts[:, 1])
        off = np.hypot(d, wpts[:, 2])
    else:
        off = np.sqrt(wpts[:, 1] ** 2 + wpts[:, 2] ** 2)
    print(f"물: 입자 {len(wpts):,}개, 재순환 누적 {_recycled:,}개 "
          f"(0이면 GPU 위치 쓰기 미반영), 관벽 관통 {int(np.sum(off > 0.055))}개"
          + (f", 최종 유동 {_flow_speed_now*1000:+.0f}mm/s "
             f"(목표 {abs(FLOW_V)*1000:.0f})" if ELBOW else ""))


if ELBOW:
    print(f"관절 |요| 최대 {max_bend:.1f}° (한계 55°) — 구간 무관 최대값")
    if yaw_trace:
        tr_s = np.array([t[0] for t in yaw_trace])
        tr_y = np.array([t[1] for t in yaw_trace])
        tr_i = np.array([t[2] for t in yaw_trace])
        print("구간별 요(°) / 슬립 — 오도메트리가 주 측위 수단이라 구간별이 핵심:")
        print(f"  {'구간':<16}{'요 평균':>8}{'요 범위':>16}{'실제':>8}{'바퀴':>8}{'슬립':>8}")
        for tag, m in (("직관 (s<600)", tr_s < S_ARC[0]),
                       ("곡관 (600~757)", (tr_s >= S_ARC[0]) & (tr_s <= S_ARC[1])),
                       ("출구직관 (s>757)", tr_s > S_ARC[1])):
            if m.sum() < 2:
                continue
            ds = tr_s[m][-1] - tr_s[m][0]
            di = tr_i[m][-1] - tr_i[m][0]
            sl = ds / di if abs(di) > 1e-6 else 0.0
            print(f"  {tag:<16}{tr_y[m].mean():>+8.1f}"
                  f"{f'{tr_y[m].min():+.1f}~{tr_y[m].max():+.1f}':>16}"
                  f"{ds:>7.0f}mm{di:>7.0f}mm{sl:>8.3f}")
        # ── 오도메트리 위치 오차 ──
        # 슬립률 자체는 오차가 아니다. 어디서나 일정하면 보정계수 하나로 지워지고,
        # 결함 위치 기록에 아무 문제가 없다. 실제 오차는 "보정 후에도 남는 편차"
        # — 구간마다 슬립이 다른 정도다. 결함 병합 규칙이 거리 50mm 기준이므로
        # 잔차가 그 절반(25mm)을 넘으면 같은 결함을 다른 결함으로 세기 시작한다
        k = ((tr_s[-1] - tr_s[0]) / (tr_i[-1] - tr_i[0])
             if abs(tr_i[-1] - tr_i[0]) > 1e-6 else 1.0)
        res_raw = tr_s - (tr_s[0] + (tr_i - tr_i[0]))        # 보정 없음
        res_cal = tr_s - (tr_s[0] + k * (tr_i - tr_i[0]))    # 단일 계수 보정
        print(f"  오도메트리 위치 오차 — 무보정 최대 "
              f"{np.abs(res_raw).max():.0f}mm / 보정계수 k={k:.3f} 적용 시 최대 "
              f"{np.abs(res_cal).max():.0f}mm, RMS {np.sqrt((res_cal**2).mean()):.0f}mm")
        print(f"     (결함 병합 규칙 50mm 기준 → 잔차 25mm 이하가 목표)")
        pre = tr_y[tr_s < S_ARC[0]]
        arc = tr_y[(tr_s >= S_ARC[0]) & (tr_s <= S_ARC[1])]
        if pre.size and arc.size:
            # 곡관이 실제로 요구한 굽힘 = 직진 크랩 기준선 대비 변화량
            base = float(np.median(pre[-10:] if pre.size >= 10 else pre))
            d = float(arc[np.argmax(np.abs(arc - base))] - base)
            print(f"  → 직진 크랩 기준선 {base:+.1f}° 대비 곡관 최대 변화 "
                  f"{d:+.1f}° (v3 SR 기하 요구 +36.1°, 달성률 {abs(d)/36.1*100:.0f}%)")
            print("     달성률이 낮으면 관절이 안 꺾이고 휠 스크럽으로 통과한 것 "
                  "— 곡관 구간 슬립률 하락과 같이 볼 것")
    print(f"최종 위치 x {float(p_end[0])*1000:.0f}, y {float(p_end[1])*1000:.0f} mm")
    if WATER:
        water_stats()
    print(f"[{'OK' if elbow_done else 'FAIL'}] "
          + ("직관+SR엘보+출구직관 전체 코스 주파"
             + (" (흐르는 물 거슬러)" if WATER else "")
             if elbow_done else "코스 미완주 — 위 위치에서 정체"))
elif WATER:
    water_stats()
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
