"""[Isaac 3.11] 배관 점검·수리 주 시연 — 주행 → 결함 접근 → 용접 → 검증.

**현역 기본 실행 코드다.** 2026-08-04 플랫폼 교체로 son 1세대(6륜·중앙관절
1개) 대신 벨로우즈 12륜을 쓴다. 1세대는 `legacy/` 에 동작 상태로 보관돼 있다.

구성
  로봇  robot_bellows/robot_bellows.usda
        12륜(레그당 2륜) · 프리즈매틱 피스톤 서스펜션(0~10mm, 예압 9N) ·
        벨로우즈 4관절(X-Y-X-Y 교대, 각 ±27.5°). DOF 22, 질량 0.81kg.
        장축 = 로컬 -Z (seg1 이 앞). 실측 제원은 robot_bellows/README.md.
  배관  pipe/pipe_elbow_lr150.usda
        LR 곡관 R=150mm 90°, 굽힘 평면 XY(수평), 내반경 50mm(DN100), 벽 6mm.
        입구 직관 y=-150 (x=-343→0), 출구 직관 x=+150 (y=0→343).
  토치  welder/meshes/{torch_ring,torch_rod,torch_tip}.stl
        + J1 회전(관 중심축) / J2 직동(반경 방향)
        치수는 spec/parts_meta.json, 드라이브는 welder/torch_spec.py
  카메라 camera/meshes/camera_housing.stl + camera/config/camera.yaml
        (어안 140°, 1280x720). 전방·후방 2대.
  결함  pipe/meshes/{defect_hole,bead_hole}.stl

🚨 수리 구현 방식 — CLAUDE.md 가 채택한 **프림 가시성 전환**이다.
   결함 프림(처음 보임)과 비드 프림(처음 숨김)을 같은 자리에 미리 두고
   용접 성공 시 결함을 숨기고 비드를 띄운다. **배관 충돌 메시에 실제로
   구멍이 뚫리는 것은 아니다** — 시각 시연이다. 진짜 관통(2층 분리:
   시각 홈 + 물리 개구)은 son 의 `pipe/crack_inject.py` 담당이고 그쪽은
   파라메트릭 관 재생성 방식이라 이 usda 배관에는 바로 못 얹는다.

시퀀스 (son 설계 7.x 를 따른다)
   APPROACH → ALIGN(J1 회전) → EXTEND(J2) → ARC(아크) → SWAP(결함→비드)
   → RETRACT(J2) → REPOSITION(후진 120mm) → VERIFY(전방 카메라 촬영) → DONE
   ※ REPOSITION 이 있는 이유: 용접 지점은 제자리에서 어느 카메라로도 안 보인다
     (화각 140° 라도 관벽 r=50 이 시야에 들어오려면 전방 18.2mm 이상 필요).

🚨 물리 스텝은 **1/240 이 필수다**(기본값). 1/60 이면 피스톤 드라이브가 한
   스텝에 50mm 를 움직여(target 0.02·강성 3000·감쇠 20 → 종단속도 3 m/s)
   휠에서 관벽까지 남은 6.75mm 를 통째로 뛰어넘고 파고들어 NaN 으로 발산한다.
   PHYSICS_HZ 로 바꿀 수 있으나 내리지 말 것.

🚨 수리 표현은 CLAUDE.md 채택 방식인 **프림 가시성 전환**이다. 결함 프림과
   비드 프림을 같은 자리에 두고 용접 성공 시 뒤집는다. **배관 충돌 메시에
   실제로 구멍이 뚫리지는 않는다.** 진짜 관통(시각 홈 + 물리 개구 2층 분리)은
   `pipe/crack_inject.py` 몫이고 파라메트릭 관 재생성 방식이라 이 usda 배관에는
   아직 못 얹었다.

물 모드 두 가지 — 목적이 다르다
  --water   PBD 입자. **보이는 물**. 누수가 눈에 보이고 무겁다
  --fluid   해석적 유체력. **느껴지는 물**. 항력·부력만 넣고 연산은 공짜다
            설계 v3 §4.4 모델 (F_drag = ½ρ·C_d,eff·A·Δv|Δv|, F_buoy = 0.36×중량)
            🚨 둘을 같이 주면 항력이 이중 계상되므로 --fluid 를 끈다

주행 속도
  SPEED_MPS=0.15  기본 0.15 m/s (설계 0.05 의 3배)
                  6배(0.30)는 카메라 10Hz 에서 프레임당 30mm 라 결함 38mm 를
                  1.3 프레임에만 잡아 검출을 놓친다. 3배가 상한이다.
  🚨 속도를 올리면 contactOffset 이 자동으로 따라 넓어진다
     (한 스텝 이동 > 감지폭이면 바퀴가 관벽을 뛰어넘어 박힌다)
     0.15 m/s @240Hz → 0.625mm/step → contactOffset 0.75mm

연산량 조절 (물 모드가 무거울 때 — 물리·입자 설정은 안 건드린다)
  RECYCLE_HZ=15  재투입 호출 주기. 기본 15. 60 이 옛 동작
                 (입자 위치를 GPU↔CPU 로 왕복시키는 비용이라 호출 횟수가 곧 비용)
  ISO_PASSES=2   물 표면 다듬기 횟수. 기본 2. 8 이 옛 동작, 0 이면 isosurface 끔
  예) RECYCLE_HZ=60 ISO_PASSES=8 isaac_python repair_demo.py --water   # 옛 설정
  실행 중 5초마다 `물리 xx step/s` 가 찍히므로 조합을 바꿔 가며 비교할 것.

실행 (Isaac 내장 rclpy 환경 — HANDOFF.md 참조):
  isaac_python repair_demo.py --headless            # 로그로 검증
  DISPLAY=:1 isaac_python repair_demo.py --shots    # GUI + 사진 저장 (out/)
  DISPLAY=:1 isaac_python repair_demo.py --hold     # GUI, 창 닫을 때까지 유지
"""

import json
import math
import os
import struct
import sys
from pathlib import Path

HEADLESS = "--headless" in sys.argv
SHOTS = "--shots" in sys.argv          # 카메라 프레임 저장 (GUI 필요)
HOLD = "--hold" in sys.argv            # 끝나고 창을 열어 둔다
# 🔑 **토치는 항상 붙어 있다** (2026-08-05 방침). 정찰기/수리기를 나누지 않고
# 한 대가 점검·수리를 다 하므로 "토치 없는 로봇" 이라는 운용 형태가 없다.
# --no-torch 는 **대조군 실험용**이다 — 주행 이상이 토치 탓인지 가리는 용도.
# 기준 질량은 토치 포함 876g (본체 813 + 토치 63) 이고, 안착 후퇴량도
# 토치 있는 74mm 가 기준이다(없으면 44mm).
NO_TORCH = "--no-torch" in sys.argv
# 토치 형상·질량은 두고 **접촉만** 끈다(콜라이더를 지우면 관성이 0 이 되어 NaN).
TORCH_NOCOLLIDE = "--torch-nocollide" in sys.argv
# --water: 만관 조건 — 투명 배관 + 흐르는 PBD 물. 파티클은 GPU 물리 전용이라
# contactOffset·시작 위치·마찰이 건식과 달라진다(아래 각 지점 주석 참조).
WATER = "--water" in sys.argv
# --fluid: 만관 유체력을 **해석적으로** 건다(입자 없이). 설계 v3 §4.4 모델을
# dongmin `fluid_force_demo.py` 에서 옮겨 왔다.
#   F_drag = ½ρ·C_d,eff·A·(v_flow − v_robot)|v_flow − v_robot|   (관 축 방향)
#   F_buoy = 부력비 × 중량                                        (연직 위)
# 🚨 --water 와 같이 쓰면 **항력이 두 번 걸린다** — 입자 충돌로 이미 밀리는데
#    해석 힘을 또 더하는 셈이다. 같이 주면 해석 쪽을 끈다.
FLUID = "--fluid" in sys.argv
# 🔑 **만관 상태에서 용접한다** (2026-08-05 방침). 정찰기/수리기를 나누지 않고
# 한 대가 점검하다 결함을 만나면 그 자리에서 고치고 계속 간다. 물을 빼는
# 단계가 없으므로 마찰은 **끝까지 만관값(0.30/0.25)** 이다.
#   FLOODED 는 "관에 물이 찼는가" 다 — 입자(--water)든 해석적 힘(--fluid)이든
#   물이 있으면 참이다. 예전에는 --water 만 봐서 --fluid 만 줬을 때 항력은
#   만관인데 마찰은 배수(0.40) 로 잡히는 불일치가 있었다.
FLOODED = WATER or FLUID
if FLUID and WATER:
    print("[경고] --fluid 와 --water 를 같이 줬다. 입자 충돌이 이미 항력을 "
          "만들므로 해석적 유체력은 끈다(이중 계상 방지).")
    FLUID = False
STEPS = 9000
if "--steps" in sys.argv:
    STEPS = int(sys.argv[sys.argv.index("--steps") + 1])

from isaacsim import SimulationApp                        # noqa: E402

simulation_app = SimulationApp({"headless": HEADLESS})

import numpy as np                                        # noqa: E402
from isaacsim.core.api import World                       # noqa: E402
from isaacsim.core.prims import SingleArticulation        # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from pxr import (Gf, PhysxSchema, Sdf, UsdGeom, UsdLux,   # noqa: E402
                 UsdPhysics, UsdShade)

MM = 0.001
# 물리 스텝은 속도·접촉 계산에 먼저 필요하므로 여기서 읽는다.
PHYSICS_HZ_PRE = float(os.environ.get("PHYSICS_HZ", 240))
SON = Path(__file__).resolve().parent
ROBOT_USDA = str(SON / "robot_bellows" / "robot_bellows.usda")
PIPE_USDA = str(SON / "pipe" / "pipe_elbow_lr150.usda")
OUT = SON / "out" / "repair_demo"      # out/ 은 gitignore 대상

META = json.loads((SON / "spec" / "parts_meta.json").read_text())
TORCH = META["torch"]

# 토치 드라이브·질량은 welder/torch_spec.py 가 단일 출처다(왜 거기인지와
# "N·m/deg vs N·m/rad" 함정은 그 파일 머리말에 적혀 있다).
sys.path.insert(0, str(SON / "welder"))
import torch_spec                                         # noqa: E402

# ── 코스 (test_pipe_elbow.usda 실측) ────────────────────────────────
IN_Y = -0.150          # 입구 직관 중심선 y (관 축은 X, z=0)
PIPE_IR = 0.050
ARC_R = 0.150          # LR 곡관 굽힘 반경
OUT_X = 0.150          # 출구 직관 중심선 x
S_IN, S_OUT = 0.343, 0.343                # 직관 길이
S_ARC = ARC_R * math.pi / 2               # 0.2356
S_TOTAL = S_IN + S_ARC + S_OUT            # 코스 중심선 총 길이 0.921 m
# 🚨 안착 중에 로봇이 **뒤로 밀린다**(피스톤 6개가 동시에 9N 으로 벽을 밀며
# 자세를 잡는 과도 구간). 토치 없이 44mm, 토치 63g 를 앞에 달면 74mm.
# 관 입구가 x=-343 이라 START_X=-0.30 이면 밀린 뒤 seg0 가 x=-374 로 **관 밖에
# 나가고**, 후방 휠 6개가 자유공간에 떠서(반경 54.6mm) 로봇이 관 끝면에 걸린다
# — 휠은 193 deg/s 로 돌지만 전진 0. 여유를 두고 관 안쪽에서 시작한다.
# 물 모드는 GPU 기본 contactOffset(0.02) 을 쓰므로 관 끝 링과 겹치면 관통 해소
# 임펄스로 사출된다(yongbin 12차 실측) → 더 안쪽에서 시작한다.
START_X = -0.18 if WATER else -0.22   # 입자 모드만 더 안쪽
DEFECT_X = -0.05       # 결함 축방향 위치 (직관은 x=0 에서 곡관으로 넘어간다)
# 결함 시계각 — +Z(위)에서 +Y 쪽으로 잰다. **180° = 바닥.**
# 바닥에 둔 이유: 물이 실제로 새는 것을 눈으로 보려면 중력 방향이어야 한다.
# 🚨 물리 개구는 **휠 접지 궤도 사이**에 놓아야 한다(설계: "슬롯으로는 못 푼다,
#    위치로 푼다"). 이 로봇 휠 궤도는 월드 시계각 90 / 210 / 330° 이고 폭 4mm
#    @r40 → ±2.9°. 개구 ø38.1 은 바닥 기준 ±22.4° 라 157.6~202.4° 를 차지하고
#    210° 궤도(207.1~212.9°)와 **4.7° 여유**로 비껴간다.
DEFECT_CLOCK_DEG = 180.0

# 🚨 누수 개구 치수는 **입자 설정에서 유도한다**(pipe/crack_inject.py 공식).
#      실효 통과경 = 개구지름 − 2×PCO − 메시절삭,  누수 조건 = 실효 ≥ 2×입자지름
#    우리 물 설정(PCO 8.0mm, 입자지름 2×fluid_rest = 9.6mm)에서 ø38.1mm 다.
#    메모리에 남은 ø28.8 은 dongmin 의 PCO 6mm 조건 값이라 **그대로 쓰면 안 샌다**
#    (실효 11.8mm < 필요 19.2mm). 입자를 바꾸면 이 값도 다시 구할 것.
LEAK_PORT_D = 0.0381

# ── 만관 유체력 (--fluid) : 설계 v3 §4.4, dongmin fluid_force_demo 모델 ──
# 파티클을 안 쓰고 힘만 넣으므로 **연산이 사실상 공짜다.**
# 🔑 설계 v3 §12.3 physics_flooded.yaml 의 **확정값을 그대로 쓴다.**
#    dongmin 은 bbox 에서 유도했는데(임시 로봇이라 크기를 몰라서), 이 로봇에
#    그 방식을 쓰면 bbox 가 관을 거의 채워 폐색률 β 가 0.6~0.8 로 튀고
#    C_d,eff = C_d/(1−β)² 가 7~17 로 폭주해 항력이 40N 까지 나온다.
#    β 보정식은 그런 고폐색 영역에서 성립하지 않는다. 설계 확정값이 옳다.
RHO = 1000.0
G = 9.81
V_FLOW_DESIGN = 0.855       # m/s (매닝, S=1/100)
A_FRONTAL = 2.7e-3          # m² — 설계 확정
CD_EFF = 2.32               # 설계 확정 (β 0.34 에서 유도된 값)
V_DISP = 1.8e-4             # m³ 배수 체적
ADDED_MASS = 0.09           # kg 부가질량 (지금은 기록만, 미적용)
# 유속 부호: 물은 출구→입구로 흐른다(로봇 진행 반대) = 역류 주행.
# 로봇 진행이 월드 +X(입구 직관)이므로 유속은 −X 다.
V_FLOW_SIGNED = -V_FLOW_DESIGN

# ── 로봇 (usda 실측) ────────────────────────────────────────────────
WHEEL_R = 0.010
SEG_GAP = 0.076        # seg0 z=0 → seg1 z=-0.076 (seg1 이 앞)

# 주행 속도. 설계 기준은 0.05 m/s 였으나 시연이 너무 느려 **3배**로 올렸다.
# 처음에 6배(0.30)로 올렸다가 되돌렸다 — 검출과 충돌해서다:
#   카메라 10Hz 에서 0.30 m/s 면 프레임당 30mm 전진하는데 결함이 38mm 라
#   화면에 잡히는 것이 1.3 프레임뿐이다. 한 장 놓치면 그냥 지나간다.
#   0.15 m/s 면 프레임당 15mm → 2.5 프레임에 걸린다.
# SPEED_MPS 로 바꿀 수 있다.
TARGET_SPEED_MPS = float(os.environ.get("SPEED_MPS", 0.15))
SPIN_DEG_S = math.degrees(TARGET_SPEED_MPS / WHEEL_R)

# 🚨 속도를 올리면 **접촉 감지폭도 같이 넓혀야 한다.**
# 엔진은 뚝뚝 끊어 계산하므로 한 스텝 이동이 contactOffset 보다 크면
# 바퀴가 감지 밴드를 통째로 뛰어넘어 관벽에 박히거나 뚫고 나간다.
#
#     0.05 m/s @240Hz → 0.208mm/step   (0.5mm 의 0.42배)  안전
#     0.15 m/s @240Hz → 0.625mm/step   (0.5mm 의 1.25배)  ← 이미 뛰어넘는다
#     0.30 m/s @240Hz → 1.250mm/step   (0.5mm 의 2.50배)
#
# 물리 Hz 를 올리면 연산량이 그만큼 늘어 시연이 더 버벅인다. 대신
# **감지 밴드를 넓힌다** — restOffset 은 0 이라 바퀴가 멈추는 자리는
# 그대로고, "미리 보는 거리"만 늘어나므로 값이 싸다.
# 설계 §12.2 가 경고한 Isaac 기본값 0.02(20mm)의 1/13 수준이다.
CONTACT_OFFSET = max(0.0005, 1.2 * TARGET_SPEED_MPS / PHYSICS_HZ_PRE)
REST_OFFSET = 0.0

# ── 토치 (son parts_meta) ───────────────────────────────────────────
RING_R_OUT = TORCH["ring_r_out"] * MM        # 0.030
ROD_ORIGIN_R = TORCH["rod_origin_r"] * MM    # 0.030
ROD_LEN = TORCH["rod_len"] * MM              # 0.005
J2_STROKE = TORCH["j2_stroke_mm"] * MM       # 0.008
STOW_R = TORCH["stow_radius_mm"] * MM        # 0.040
REACH_R = TORCH["reach_radius_mm"] * MM      # 0.048

PHYSICS_HZ = PHYSICS_HZ_PRE
PHYSICS_DT = 1.0 / PHYSICS_HZ

ROBOT = "/World/Robot"
PIPE = "/World/Pipe"
DEFECT = "/World/Defect"

world = World(stage_units_in_meters=1.0,
              physics_dt=PHYSICS_DT, rendering_dt=1.0 / 60.0)
stage = world.stage
if WATER:
    _pc = world.get_physics_context()
    _pc.enable_gpu_dynamics(True)      # 파티클은 GPU 전용
    _pc.set_broadphase_type("GPU")
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage, "/World")

light = UsdLux.SphereLight.Define(stage, "/World/Light")
light.CreateIntensityAttr(3e6)
light.CreateRadiusAttr(0.05)
UsdGeom.Xformable(light).AddTranslateOp().Set(Gf.Vec3d(-0.2, -0.4, 0.4))


def rot(deg, axis):
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(Gf.Vec3d(*axis), deg))
    return m


def trans(x, y, z):
    m = Gf.Matrix4d(1.0)
    m.SetTranslate(Gf.Vec3d(x, y, z))
    return m


def path_dist_tangent(px, py):
    """xy 점 → (중심선까지 xy 거리, 접선x, 접선y). 접선 = 로봇 진행 방향.

    물 채우기·유동장·진행거리가 전부 이 한 기하를 쓴다."""
    d1 = np.hypot(px - np.clip(px, -S_IN, 0.0), py - IN_Y)
    ang = np.clip(np.arctan2(py, px), -np.pi / 2, 0.0)
    d2 = np.hypot(px - ARC_R * np.cos(ang), py - ARC_R * np.sin(ang))
    d3 = np.hypot(px - OUT_X, py - np.clip(py, 0.0, S_OUT))
    d = np.stack([d1, d2, d3])
    which = np.argmin(d, axis=0)
    tx = np.where(which == 0, 1.0, np.where(which == 1, -np.sin(ang), 0.0))
    ty = np.where(which == 0, 0.0, np.where(which == 1, np.cos(ang), 1.0))
    return d.min(axis=0), tx, ty


def path_s(px, py):
    """코스 중심선 진행거리(m). 입구 x=-0.343 이 0.

    곡관에서 x 변위는 주행량이 아니다 — 굽고 나면 x 가 멎는다."""
    px = np.atleast_1d(np.asarray(px, float))
    py = np.atleast_1d(np.asarray(py, float))
    ang = np.clip(np.arctan2(py, px), -np.pi / 2, 0.0)
    d1 = np.hypot(px - np.clip(px, -S_IN, 0.0), py - IN_Y)
    d2 = np.hypot(px - ARC_R * np.cos(ang), py - ARC_R * np.sin(ang))
    d3 = np.hypot(px - OUT_X, py - np.clip(py, 0.0, S_OUT))
    which = np.argmin(np.stack([d1, d2, d3]), axis=0)
    s = np.where(which == 0, np.clip(px, -S_IN, 0.0) + S_IN,
                 np.where(which == 1, S_IN + ARC_R * (ang + np.pi / 2),
                          S_IN + S_ARC + np.clip(py, 0.0, S_OUT)))
    return float(s[0]) if s.size == 1 else s


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


# ── 배관 ────────────────────────────────────────────────────────────
stage.DefinePrim(PIPE, "Xform").GetReferences().AddReference(PIPE_USDA)

# 🚨 **충돌 메시에 진짜 구멍을 낸다.** 결함/비드 프림은 시각 전용이라 그것만으로는
#    물이 새지 않는다(1차 시연의 한계였다). 배관은 approximation="none" 삼각
#    메시이므로 개구 자리의 삼각형을 지우면 그대로 관통 구멍이 된다.
#    개구 축은 반경 방향(바닥이면 -Z) 이므로 축에서의 거리는 (x, y) 평면에서 잰다.
_pipe_mesh = UsdGeom.Mesh(stage.GetPrimAtPath(f"{PIPE}/geom"))
_pts = np.array(_pipe_mesh.GetPointsAttr().Get())
_idx = np.array(_pipe_mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
_cent = _pts[_idx].mean(axis=1)
_hole = np.hypot(_cent[:, 0] - DEFECT_X, _cent[:, 1] - IN_Y) < LEAK_PORT_D / 2
_kept = _idx[~_hole]
_pipe_mesh.GetFaceVertexIndicesAttr().Set(_kept.reshape(-1).tolist())
_pipe_mesh.GetFaceVertexCountsAttr().Set([3] * len(_kept))
print(f"[준비] 배관 바닥에 관통 개구 ø{LEAK_PORT_D * 1000:.1f}mm — "
      f"삼각형 {int(_hole.sum())}개 제거 ({len(_idx)} → {len(_kept)})")

# 속도에 맞춰 감지 밴드를 넓힌다 — usda 에 박힌 0.0005 를 덮어쓴다.
_n_off = 0
for _p in stage.Traverse():
    if _p.HasAPI(UsdPhysics.CollisionAPI) or _p.IsA(UsdGeom.Mesh):
        _px = PhysxSchema.PhysxCollisionAPI.Apply(_p)
        _px.CreateContactOffsetAttr(CONTACT_OFFSET)
        _px.CreateRestOffsetAttr(REST_OFFSET)
        _n_off += 1
print(f"[준비] 주행 {TARGET_SPEED_MPS * 1000:.0f} mm/s "
      f"(한 스텝 {TARGET_SPEED_MPS / PHYSICS_HZ * 1000:.3f}mm) → "
      f"contactOffset {CONTACT_OFFSET * 1000:.2f}mm 로 확대, 프림 {_n_off}개")

# 물리 재질 — **관 상태**로 고른다(로봇 종류가 아니다. v3 §12.3).
#   만관 physics_flooded  0.30 / 0.25   ← 물속 용접이므로 시연은 이쪽
#   배수 physics_drained  0.40 / 0.35   ← 물 없이 돌릴 때(건식 대조군)
FRICTION_STATIC = 0.30 if FLOODED else 0.40
FRICTION_DYNAMIC = 0.25 if FLOODED else 0.35
_pm = UsdPhysics.MaterialAPI.Apply(
    UsdShade.Material.Define(stage, "/World/PipePhysMat").GetPrim())
_pm.CreateStaticFrictionAttr(FRICTION_STATIC)
_pm.CreateDynamicFrictionAttr(FRICTION_DYNAMIC)
print(f"[준비] 관 상태 {'만관' if FLOODED else '배수'} — 마찰 "
      f"{FRICTION_STATIC}/{FRICTION_DYNAMIC} "
      f"({'물속에서 용접한다' if FLOODED else '건식 대조군'})")
_pm.CreateRestitutionAttr(0.0)

if WATER:
    # 유리 — displayOpacity 프림바는 RTX 뷰포트에서 안 먹히는 경우가 있어
    # water_particle_demo 에서 검증된 UsdPreviewSurface 반투명 바인딩을 쓴다
    _gl = UsdShade.Material.Define(stage, "/World/Glass")
    _gs = UsdShade.Shader.Define(stage, "/World/Glass/Shader")
    _gs.CreateIdAttr("UsdPreviewSurface")
    _gs.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.55, 0.58, 0.60))
    _gs.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.25)
    _gs.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
    _gl.CreateSurfaceOutput().ConnectToSource(_gs.ConnectableAPI(), "surface")
for _p in stage.Traverse():
    if str(_p.GetPath()).startswith(PIPE + "/") and _p.IsA(UsdGeom.Mesh):
        UsdShade.MaterialBindingAPI.Apply(_p).Bind(
            UsdShade.Material.Get(stage, "/World/PipePhysMat"),
            bindingStrength=UsdShade.Tokens.weakerThanDescendants,
            materialPurpose="physics")
        if WATER:
            UsdShade.MaterialBindingAPI.Apply(_p).Bind(
                _gl, bindingStrength=UsdShade.Tokens.strongerThanDescendants)

# ── 결함 / 비드 ─────────────────────────────────────────────────────
# STL 은 "관 축 X, 패치가 +Z" 로 만들어져 있고 이 배관 입구 직관도 축이 X 라
# 회전은 시계각만 주면 된다.  Rx(-θ): +Z → (0, sinθ, cosθ)
DEF_XF = rot(-DEFECT_CLOCK_DEG, (1, 0, 0)) * trans(DEFECT_X, IN_Y, 0.0)
UsdGeom.Xform.Define(stage, DEFECT)
defect_mesh = make_mesh(f"{DEFECT}/hole", SON / "pipe/meshes/defect_hole.stl",
                        color=(0.05, 0.05, 0.06), xform=DEF_XF)
bead_mesh = make_mesh(f"{DEFECT}/bead", SON / "pipe/meshes/bead_hole.stl",
                      color=(0.85, 0.55, 0.20), xform=DEF_XF)
UsdGeom.Imageable(bead_mesh).MakeInvisible()      # 처음에는 숨긴다
print(f"[준비] 결함(구멍) x={DEFECT_X * 1000:.0f}mm, 시계각 "
      f"{DEFECT_CLOCK_DEG:.0f}°(바닥) — 비드는 숨김. "
      f"이 두 프림은 시각 전용이고, 물이 새는 것은 위의 관통 개구가 만든다")

# ── 물 (--water) : yongbin_drive_test.py 검증값 그대로 ──────────────
# 🚨 입자·물 값은 새로 만들지 않는다(사용자 지시). PCO 8 / fluid_rest 4.8 /
#    입자-강체 contact 0.004·rest 0.0025(미지정 시 로봇이 입자 위에 부상하는
#    함정) / 간격 9 / 수위 z+20 / 유속 0.10 / blend 0.20.
FLOW_V, FLOW_BLEND = -0.10, 0.20
W_SPACING, W_PCO, W_FLUID_REST = 0.009, 0.008, 0.0048
W_LEVEL_Z, W_R_MAX = 0.020, 0.045
# 재투입 호출 주기 — **연산량의 주범 후보 1**.
# recycle_particles() 는 매번 입자 위치·속도를 GPU 에서 읽어 파이썬으로 계산한
# 뒤 다시 써 넣는다. 그 왕복이 파이프라인을 세우므로 **입자 수보다 호출 횟수가
# 비싸다.** 기본을 초당 60회에서 15회로 낮췄다.
#   물살은 FLOW_BLEND 0.2 로 서서히 끌어당기므로 목표 속도는 그대로 도달하고
#   (시정수 약 0.33초), 관 밖으로 나간 입자의 재투입이 최대 67ms 늦을 뿐이다
#   (그 사이 이동 6.7mm). 눈에 띄는 차이는 없다.
# 되돌리려면 RECYCLE_HZ=60 으로 실행할 것.
RECYCLE_HZ = float(os.environ.get("RECYCLE_HZ", 15))
RECYCLE_EVERY = max(1, int(round(PHYSICS_HZ / RECYCLE_HZ)))
ISO_PASSES = int(os.environ.get("ISO_PASSES", 2))
INJECT_Y = (0.300, 0.340)   # 재투입은 결승선보다 상류 — 물살이 로봇을 때리지 않게
water_instancer = None
n_particles = 0
if WATER:
    from omni.physx.scripts import particleUtils            # noqa: E402
    from pxr import Vt                                      # noqa: E402
    _psys = Sdf.Path("/World/ParticleSystem")
    particleUtils.add_physx_particle_system(
        stage, _psys, particle_contact_offset=W_PCO,
        fluid_rest_offset=W_FLUID_REST, contact_offset=0.004,
        rest_offset=0.0025, max_velocity=5.0,
        wind=Gf.Vec3f(0.0, 0.0, 0.0))     # 굽은 경로는 전역 wind 못 쓴다
    particleUtils.add_physx_particle_isosurface(
        stage, _psys, enabled=ISO_PASSES > 0, grid_spacing=W_FLUID_REST * 1.5,
        surface_distance=W_FLUID_REST * 1.6,
        grid_smoothing_radius=W_FLUID_REST * 2.0,
        # **연산량의 주범 후보 2** — isosurface 는 매 프레임 입자에서 표면
        # 메시를 뽑고 이 횟수만큼 다듬는다. 8+8 은 매끈하지만 비싸다.
        # 2+2 로도 물처럼 보인다(입자가 구슬로 보이는 것은 isosurface 를
        # **끌** 때고, 이건 켜 둔 채 다듬기만 줄이는 것이다).
        # 되돌리려면 ISO_PASSES=8 로 실행할 것. 0 이면 isosurface 자체를 끈다.
        num_mesh_smoothing_passes=ISO_PASSES,
        num_mesh_normal_smoothing_passes=ISO_PASSES)
    _wv = UsdShade.Material.Define(stage, "/World/WaterVisual")
    _ws = UsdShade.Shader.Define(stage, "/World/WaterVisual/Shader")
    _ws.CreateIdAttr("UsdPreviewSurface")
    _ws.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.15, 0.45, 0.85))
    _ws.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.10, 0.28, 0.55))
    _ws.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.85)
    _ws.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.15)
    _wv.CreateSurfaceOutput().ConnectToSource(_ws.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(_psys)).Bind(
        _wv, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
    _wp = UsdShade.Material.Define(stage, "/World/WaterPBD")
    particleUtils.AddPBDMaterialWater(_wp.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(_psys)).Bind(
        _wp, bindingStrength=UsdShade.Tokens.weakerThanDescendants,
        materialPurpose="physics")

    _gx = np.arange(-S_IN + 0.01, OUT_X + 0.06, W_SPACING)
    _gy = np.arange(IN_Y - 0.05, S_OUT + 0.01, W_SPACING)
    _gz = np.arange(-W_R_MAX, W_LEVEL_Z + 1e-9, W_SPACING)
    _X, _Y, _Z = np.meshgrid(_gx, _gy, _gz, indexing="ij")
    _d, _tx, _ty = path_dist_tangent(_X, _Y)
    _in = (_d ** 2 + _Z ** 2 <= W_R_MAX ** 2)
    _in &= ~((np.abs(_X - START_X) < 0.10) & (np.abs(_Y - IN_Y) < 0.06))
    _pos = [Gf.Vec3f(float(a), float(b), float(c))
            for a, b, c in zip(_X[_in], _Y[_in], _Z[_in])]
    _vel = [Gf.Vec3f(float(-a * abs(FLOW_V)), float(-b * abs(FLOW_V)), 0.0)
            for a, b in zip(_tx[_in], _ty[_in])]
    n_particles = len(_pos)
    _inst = particleUtils.add_physx_particleset_pointinstancer(
        stage, Sdf.Path("/World/WaterParticles"),
        Vt.Vec3fArray(_pos), Vt.Vec3fArray(_vel),
        particle_system_path=_psys, self_collision=True, fluid=True,
        particle_group=0, particle_mass=0.0, density=1000.0)
    UsdGeom.Sphere(stage.GetPrimAtPath(
        "/World/WaterParticles/particlePrototype0")).GetRadiusAttr().Set(
        W_FLUID_REST)
    water_instancer = UsdGeom.PointInstancer(_inst)
    UsdGeom.Imageable(_inst).MakeInvisible()   # isosurface 가 대신 렌더
    print(f"[준비] 물 입자 {n_particles:,}개 — 유속 {abs(FLOW_V):.2f} m/s "
          f"(출구→입구, 로봇 진행 반대), 수위 z+{W_LEVEL_Z * 1000:.0f}mm")
    print(f"[준비] 연산량 손잡이  RECYCLE_HZ={RECYCLE_HZ:.0f} "
          f"({RECYCLE_EVERY} 스텝마다)  ISO_PASSES={ISO_PASSES}"
          f"{' (isosurface 꺼짐)' if ISO_PASSES == 0 else ''}")

# ── 로봇 ────────────────────────────────────────────────────────────
robot_prim = stage.DefinePrim(ROBOT, "Xform")
robot_prim.GetReferences().AddReference(ROBOT_USDA)

# 배치는 각 링크 로컬 변환에 굽는다(부모 Xform·root 에 걸면 PhysX 가 발산한다).
# Ry(-90): 로봇 로컬 -Z → 월드 +X,  로컬 X → 월드 +Z,  로컬 Y → 월드 +Y
PLACE = rot(-90, (0, 1, 0)) * trans(START_X, IN_Y, 0.0)
for child in list(robot_prim.GetChildren()):
    if not child.HasAPI(UsdPhysics.RigidBodyAPI):
        continue
    xf = UsdGeom.Xformable(child)
    local = xf.GetLocalTransformation()
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(local * PLACE)

# ── 토치 (son STL) ──────────────────────────────────────────────────
# son 토치 프레임은 "관 축 X, 로드가 +Z" 다. 이 로봇은 관 축이 로컬 Z 이므로
# 메시에만 Ry(90) 을 걸어 눕힌다(링크 프레임 축은 로봇 축 그대로 둔다).
#   son X → 로봇 -Z (관 축) ,  son +Z(반경) → 로봇 +X
# 그래서 J1 축 = 로봇 Z, J2 축 = 로봇 X 가 된다.
# 🚨 son 링은 r_in 26mm 인데 이 본체는 반경 29.2mm 라 **감싸지 못한다.**
#    치수를 바꾸지 않고 앞면 앞 동축에 놓는다(전장 +19mm).
FRONT_FACE_Z = -(SEG_GAP + 0.025)
RING_Z = FRONT_FACE_Z - 0.007
MESH_LAY = rot(90, (0, 1, 0))


def torch_link(name, local, stl, mass, color):
    xf = UsdGeom.Xform.Define(stage, f"{ROBOT}/{name}")
    xf.AddTransformOp().Set(local * PLACE)
    p = xf.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(p)
    UsdPhysics.MassAPI.Apply(p).CreateMassAttr(mass)
    m = make_mesh(f"{ROBOT}/{name}/geom", stl, color=color, xform=MESH_LAY)
    mp = m.GetPrim()
    ca = UsdPhysics.CollisionAPI.Apply(mp)
    UsdPhysics.MeshCollisionAPI.Apply(mp).CreateApproximationAttr("convexHull")
    if TORCH_NOCOLLIDE:
        ca.CreateCollisionEnabledAttr(False)
    px = PhysxSchema.PhysxCollisionAPI.Apply(mp)
    px.CreateContactOffsetAttr(CONTACT_OFFSET)
    px.CreateRestOffsetAttr(REST_OFFSET)
    return p


WM = SON / "welder" / "meshes"
if NO_TORCH:
    print("[진단] 토치 없음 — 주행 대조군")
else:
  torch_link("torch_ring", trans(0, 0, RING_Z), WM / "torch_ring.stl",
             torch_spec.MASS_RING_KG, (0.35, 0.35, 0.40))
  torch_link("torch_rod", trans(ROD_ORIGIN_R, 0, RING_Z), WM / "torch_rod.stl",
             torch_spec.MASS_ROD_KG, (0.60, 0.60, 0.65))
  torch_link("torch_tip", trans(ROD_ORIGIN_R + ROD_LEN, 0, RING_Z),
             WM / "torch_tip.stl", torch_spec.MASS_TIP_KG,
             (0.90, 0.75, 0.30))

  # J1 — 링 회전. 축 = 로봇 로컬 Z = 관 중심축. ±180°(슬립링 회피).
  j1 = UsdPhysics.RevoluteJoint.Define(stage, f"{ROBOT}/joints/torch_j1")
  j1.CreateBody0Rel().SetTargets([Sdf.Path(f"{ROBOT}/seg1_body")])
  j1.CreateBody1Rel().SetTargets([Sdf.Path(f"{ROBOT}/torch_ring")])
  j1.CreateAxisAttr("Z")
  j1.CreateLocalPos0Attr(Gf.Vec3f(0, 0, RING_Z + SEG_GAP))
  j1.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
  j1.CreateLowerLimitAttr(-TORCH["j1_limit_deg"])
  j1.CreateUpperLimitAttr(+TORCH["j1_limit_deg"])
  j1.CreateCollisionEnabledAttr(False)
  d1 = UsdPhysics.DriveAPI.Apply(j1.GetPrim(), "angular")
  d1.CreateTypeAttr("force")
  _k1, _c1, _f1 = torch_spec.usd_angular()      # 단위 N·m/deg
  d1.CreateStiffnessAttr(_k1)
  d1.CreateDampingAttr(_c1)
  d1.CreateMaxForceAttr(_f1)
  d1.CreateTargetPositionAttr(0.0)

  # J2 — 토치 직동. 축 = 로봇 로컬 X = 반경 방향. 0~8mm.
  j2 = UsdPhysics.PrismaticJoint.Define(stage, f"{ROBOT}/joints/torch_j2")
  j2.CreateBody0Rel().SetTargets([Sdf.Path(f"{ROBOT}/torch_ring")])
  j2.CreateBody1Rel().SetTargets([Sdf.Path(f"{ROBOT}/torch_rod")])
  j2.CreateAxisAttr("X")
  j2.CreateLocalPos0Attr(Gf.Vec3f(ROD_ORIGIN_R, 0, 0))
  j2.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
  j2.CreateLowerLimitAttr(0.0)
  j2.CreateUpperLimitAttr(J2_STROKE)
  j2.CreateCollisionEnabledAttr(False)
  d2 = UsdPhysics.DriveAPI.Apply(j2.GetPrim(), "linear")
  d2.CreateTypeAttr("force")
  _k2, _c2, _f2 = torch_spec.usd_linear(J2_STROKE)
  d2.CreateStiffnessAttr(_k2)
  d2.CreateDampingAttr(_c2)
  d2.CreateMaxForceAttr(_f2)
  d2.CreateTargetPositionAttr(0.0)

  jt = UsdPhysics.FixedJoint.Define(stage, f"{ROBOT}/joints/torch_tip_fix")
  jt.CreateBody0Rel().SetTargets([Sdf.Path(f"{ROBOT}/torch_rod")])
  jt.CreateBody1Rel().SetTargets([Sdf.Path(f"{ROBOT}/torch_tip")])
  jt.CreateLocalPos0Attr(Gf.Vec3f(ROD_LEN, 0, 0))
  jt.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
  jt.CreateCollisionEnabledAttr(False)

  # 아크 불빛 — ARC 동안만 켠다
  arc_light = UsdLux.SphereLight.Define(stage, f"{ROBOT}/torch_tip/arc")
  arc_light.CreateRadiusAttr(0.002)
  arc_light.CreateIntensityAttr(0.0)
  arc_light.CreateColorAttr(Gf.Vec3f(0.75, 0.85, 1.0))
  UsdGeom.Xformable(arc_light).AddTranslateOp().Set(Gf.Vec3d(0.005, 0, 0))
  print(f"[준비] 토치 드라이브(torch_spec) — J1 {_k1:.4f} N·m/deg "
        f"(maxF {_f1:.3f}) / J2 {_k2:.0f} N/m (maxF {_f2:.1f}N), "
        f"질량 {torch_spec.MASS_TORCH_KG * 1000:.0f}g")
  print(f"[준비] 토치 — 링 r_out {RING_R_OUT * 1000:.0f} / 수납 "
        f"{STOW_R * 1000:.0f} / 신장 {REACH_R * 1000:.0f}mm "
        f"(관벽 {PIPE_IR * 1000:.0f} → 용접 간극 "
        f"{(PIPE_IR - REACH_R) * 1000:.0f}mm), J1 ±{TORCH['j1_limit_deg']:.0f}°"
        f" / J2 0~{J2_STROKE * 1000:.0f}mm")

# ── 카메라 (son camera.yaml 값) ─────────────────────────────────────
CAM_W, CAM_H, CAM_HFOV = 1280, 720, 140.0
F_PX = (CAM_W / 2.0) / math.radians(CAM_HFOV / 2.0)
CM = SON / "camera" / "meshes"

# 🚨 카메라 프림은 **링크의 자식**이므로 좌표가 링크 로컬이다. 처음에 로봇 루트
#    기준 오프셋(SEG_GAP=76mm)을 한 번 더 더해 카메라가 본체 안에 파묻혔고,
#    영상이 과노출된 벽면만 찍혔다(실측). 링크 로컬로만 계산한다.
#      seg1_body 원점 = 로봇 로컬 z -0.076, 앞면 = 링크 로컬 z -0.025
#      토치 링(링크 로컬 z -0.032 ± 0.005) 보다 앞에 둔다
#      seg0_body 원점 = 로봇 로컬 z 0, 뒷면 = 링크 로컬 z +0.025
# 🚨 시선 방향: **USD 카메라는 자기 로컬 -Z 를 본다.** 이 로봇의 전방도
#    로컬 -Z 라서 전방 카메라는 회전이 필요 없다(항등). 후방만 Y축 180°.
#    (son rig.py 의 "시선은 +X" 는 son 로봇 축 규약에서 나온 말이다.)
CAM_SPECS = [
    # 이름, 부모 링크, 링크 로컬 z, 전방 여부
    ("front_camera", "seg1_body", -0.045, True),
    ("rear_camera", "seg0_body", +0.037, False),
]
for nm, seg, z, fwd in CAM_SPECS:
    base = f"{ROBOT}/{seg}/{nm}"
    UsdGeom.Xform.Define(stage, base)
    pts, idx = load_stl(CM / "camera_housing.stl")
    hm = UsdGeom.Mesh.Define(stage, f"{base}/housing")
    hm.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    hm.CreateFaceVertexCountsAttr([3] * len(idx))
    hm.CreateFaceVertexIndicesAttr(idx.reshape(-1).tolist())
    hm.CreateExtentAttr([Gf.Vec3f(*pts.min(0)), Gf.Vec3f(*pts.max(0))])
    hm.CreateSubdivisionSchemeAttr("none")
    # son 하우징 STL 은 광축이 +X. Ry(90) 이면 광축이 로봇 -Z(전방),
    # Ry(-90) 이면 +Z(후방)로 간다.
    # 하우징을 센서와 같은 자리에 두면 렌즈·조명을 가둔다 → 광축 뒤 5mm.
    UsdGeom.Xformable(hm).AddTransformOp().Set(
        rot(90 if fwd else -90, (0, 1, 0))
        * trans(0, 0, z + (0.005 if fwd else -0.005)))
    # 관 내부는 로봇 조명이 유일한 광원. 센서보다 4mm 앞, 광축 둘레 대칭.
    for k in range(2):
        lg = UsdLux.SphereLight.Define(stage, f"{base}/light_{k}")
        lg.CreateIntensityAttr(4.0e5)
        lg.CreateRadiusAttr(0.002)
        UsdGeom.Xformable(lg).AddTranslateOp().Set(Gf.Vec3d(
            0.012 * (1 if k == 0 else -1), 0.0,
            z + (-0.004 if fwd else 0.004)))
print(f"[준비] 카메라 2대 (어안 {CAM_HFOV:.0f}°, {CAM_W}x{CAM_H}, "
      f"f={F_PX:.1f}px) + 조명 4  — 전방 seg1 링크로컬 z={-45}mm, "
      f"후방 seg0 z={+37}mm")

# ── 누수 마개 ───────────────────────────────────────────────────────
# 결함/비드 프림은 시각 전용이라 가시성만 바꿔서는 물이 계속 샌다. 수리 성공을
# **물리로도** 표현하려면 개구를 막아야 하는데, 배관 삼각 메시는 reset 때 cook
# 되므로 런타임에 구멍을 되메울 수 없다. → 키네마틱 강체 마개를 관 밖에
# 세워 두었다가 용접 성공 시 개구 자리로 옮긴다.
PLUG_PARK = Gf.Vec3d(DEFECT_X, IN_Y, -1.0)
PLUG_SEAT = Gf.Vec3d(DEFECT_X, IN_Y, -(PIPE_IR + 0.008))
_plug = UsdGeom.Cylinder.Define(stage, "/World/LeakPlug")
_plug.CreateAxisAttr("Z")
_plug.CreateRadiusAttr(LEAK_PORT_D / 2 + 0.006)
_plug.CreateHeightAttr(0.010)
_plug.CreateExtentAttr([Gf.Vec3f(-0.03, -0.03, -0.005),
                        Gf.Vec3f(0.03, 0.03, 0.005)])
_plug.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.55, 0.20)])
UsdGeom.Xformable(_plug).AddTranslateOp().Set(PLUG_PARK)
_plug_prim = _plug.GetPrim()
_rb = UsdPhysics.RigidBodyAPI.Apply(_plug_prim)
_rb.CreateKinematicEnabledAttr(True)
UsdPhysics.MassAPI.Apply(_plug_prim).CreateMassAttr(0.05)
UsdPhysics.CollisionAPI.Apply(_plug_prim)
UsdGeom.Imageable(_plug_prim).MakeInvisible()   # 비드 프림이 대신 보인다
print(f"[준비] 누수 마개 대기 (ø{(LEAK_PORT_D + 0.012) * 1000:.0f}mm 키네마틱) "
      "— 용접 성공 시 개구로 이동")

art = SingleArticulation(prim_path=ROBOT, name="bellows_welder")
world.scene.add(art)
from isaacsim.core.prims import SingleRigidPrim            # noqa: E402

_plug_rb = SingleRigidPrim(prim_path="/World/LeakPlug", name="leak_plug")
world.scene.add(_plug_rb)
world.reset()

dof = list(art.dof_names or [])
wheel_idx = [k for k, n in enumerate(dof) if "_wheel_" in n]
piston_idx = [k for k, n in enumerate(dof) if "_piston_" in n]
bel_idx = [k for k, n in enumerate(dof) if n.startswith("bellows_")]
j1_idx = dof.index("torch_j1") if "torch_j1" in dof else None
j2_idx = dof.index("torch_j2") if "torch_j2" in dof else None
print("=" * 78)
print(f"조립  DOF {len(dof)} = 휠 {len(wheel_idx)} / 피스톤 {len(piston_idx)}"
      f" / 벨로우즈 {len(bel_idx)} / 토치 J1,J2   물리 1/{PHYSICS_HZ:.0f}")

# 질량 — 토치 포함이 기준이다(방침상 항상 붙어 있다)
_m_links = [q for q in stage.Traverse()
            if str(q.GetPath()).startswith(ROBOT + "/")
            and q.HasAPI(UsdPhysics.RigidBodyAPI)]
_m_total = 0.0
for q in _m_links:
    _ma = UsdPhysics.MassAPI(q).GetMassAttr()
    if _ma:
        _m_total += float(_ma.Get() or 0.0)
print(f"질량  {_m_total * 1000:.0f} g (링크 {len(_m_links)}개"
      + ("" if NO_TORCH else f", 토치 {torch_spec.MASS_TORCH_KG * 1000:.0f}g 포함")
      + f")  중량 {_m_total * 9.81:.2f} N"
      + ("   ⚠ --no-torch 대조군" if NO_TORCH else ""))

# ── 유체력 준비 (--fluid) ───────────────────────────────────────────
_fluid_view = None
if FLUID:
    from isaacsim.core.prims import RigidPrim               # noqa: E402
    # 힘은 **본체 링크**에 넣는다. 항력 작용점(본체 중심)과 접지점의 높이 차가
    # 피칭 모멘트를 만드는 것도 자연히 재현된다(설계 §4.4 4번 항목).
    _fluid_view = RigidPrim([f"{ROBOT}/seg0_body", f"{ROBOT}/seg1_body"],
                            name="fluid_body")
    _fluid_view.initialize()

    _links = [str(q.GetPath()) for q in stage.Traverse()
              if str(q.GetPath()).startswith(ROBOT + "/")
              and q.HasAPI(UsdPhysics.RigidBodyAPI)]
    _mass_view = RigidPrim(_links, name="fluid_mass")
    _mass_view.initialize()
    M_TOTAL = float(np.sum(_mass_view.get_masses()))

    # 부력은 배수 체적에서 바로 나온다(설계 V_disp). 로봇 질량과 무관하다.
    F_BUOY = RHO * G * V_DISP
    F_DRAG_DESIGN = 0.5 * RHO * CD_EFF * A_FRONTAL * V_FLOW_DESIGN ** 2
    # 견인 한계 = 피스톤 6개가 각각 9N 으로 벽을 누르므로 수직항력 54N.
    F_TRACTION = 6 * 9.0 * FRICTION_STATIC
    print(f"[준비] 만관 유체력 (해석적, 입자 없음) — 설계 v3 §12.3 확정값")
    print(f"       질량 {M_TOTAL * 1000:.0f}g  중량 {M_TOTAL * G:.2f}N  "
          f"부력 {F_BUOY:.2f}N (배수 {V_DISP * 1e6:.0f}cm³)")
    print(f"       정면적 {A_FRONTAL * 1e6:.0f}mm²  C_d,eff {CD_EFF:.2f}  "
          f"유속 {V_FLOW_DESIGN}m/s")
    _verdict = (f"버틴다 (여유 {F_TRACTION / F_DRAG_DESIGN:.1f}배)"
                if F_DRAG_DESIGN < F_TRACTION else "⚠ 떠밀린다")
    print(f"       항력 {F_DRAG_DESIGN:.2f}N  vs 견인 {F_TRACTION:.2f}N  "
          f"→ {_verdict}")
    if F_BUOY > M_TOTAL * G:
        print(f"       ⚠ 부력이 중량을 넘는다 — 로봇이 뜬다")


def apply_fluid():
    """항력(관 축 방향) + 부력(연직 위)을 매 스텝 넣는다.

    🚨 apply_forces 는 **1 스텝만 유효하다** — 매 스텝 다시 걸어야 한다.
    항력 방향은 로봇이 어디 있든 **코스 접선**을 따라야 한다(곡관에서 X 고정이면
    엉뚱한 방향으로 민다). path_dist_tangent 가 그 접선을 준다.
    """
    if _fluid_view is None:
        return 0.0
    pos = _fluid_view.get_world_poses()[0]
    vel = _fluid_view.get_linear_velocities()
    _, tx, ty = path_dist_tangent(pos[:, 0], pos[:, 1])
    # 로봇 진행 방향(+접선) 성분
    v_robot = vel[:, 0] * tx + vel[:, 1] * ty
    rel = (-V_FLOW_DESIGN) - v_robot          # 유속은 진행 반대(역류)
    # 정면적·부력을 두 세그먼트가 반씩 나눠 진다.
    f = 0.5 * RHO * CD_EFF * (A_FRONTAL / 2.0) * rel * np.abs(rel)
    forces = np.stack([f * tx, f * ty,
                       np.full_like(f, F_BUOY / 2.0)], axis=-1)
    _fluid_view.apply_forces(forces.astype(np.float32), is_global=True)
    return float(np.sum(f))


_XC = UsdGeom.XformCache()
_seg1 = stage.GetPrimAtPath(f"{ROBOT}/seg1_body")
_ring = stage.GetPrimAtPath(f"{ROBOT}/torch_ring") if not NO_TORCH else _seg1
_tip = stage.GetPrimAtPath(f"{ROBOT}/torch_tip") if not NO_TORCH else _seg1


def wpos(prim):
    _XC.Clear()
    t = _XC.GetLocalToWorldTransform(prim).ExtractTranslation()
    return np.array([float(t[0]), float(t[1]), float(t[2])])


# 🚨 팁 반경은 **링크 원점이 아니라 메시 끝**으로 재야 한다. 원점으로 재면
# 팁 길이 5mm 를 통째로 빼먹어 "간극 10mm" 같은 거짓 수치가 나온다.
TIP_END_LOCAL = Gf.Vec3d(0.00497, 0.0, 0.0)   # torch_tip.stl 의 +Z 끝 (눕힌 뒤 +X)


def tip_end_r():
    """토치 팁 끝의 **관 중심선** 기준 반경(mm)."""
    _XC.Clear()
    m = _XC.GetLocalToWorldTransform(_tip)
    w = m.Transform(TIP_END_LOCAL)
    return math.hypot(w[1] - IN_Y, w[2]) * 1000


def drive(deg_s):
    art.apply_action(ArticulationAction(
        joint_velocities=np.array([math.radians(deg_s)] * len(wheel_idx)),
        joint_indices=np.array(wheel_idx)))


def set_torch(j1_deg=None, j2_m=None):
    if j1_idx is None:
        return
    idx, val = [], []
    if j1_deg is not None:
        idx.append(j1_idx)
        val.append(math.radians(j1_deg))
    if j2_m is not None:
        idx.append(j2_idx)
        val.append(j2_m)
    art.apply_action(ArticulationAction(joint_positions=np.array(val),
                                        joint_indices=np.array(idx)))


# ── 카메라 프레임 저장 (GUI 에서만 나온다) ──────────────────────────
rigs = []
if SHOTS:
    try:
        import omni.replicator.core as rep
        from isaacsim.sensors.camera import Camera
        OUT.mkdir(exist_ok=True)
        for nm, seg, z, fwd in CAM_SPECS:
            # 좌표는 **링크 로컬**. 루트 기준 오프셋을 더하면 안 된다.
            cam = Camera(prim_path=f"{ROBOT}/{seg}/{nm}/sensor",
                         translation=np.array([0.0, 0.0, z]),
                         frequency=10, resolution=(CAM_W, CAM_H))
            cam.initialize()
            # USD 카메라는 로컬 -Z 를 본다. 로봇 전방도 로컬 -Z 이므로
            # 전방 카메라는 항등, 후방만 Y축 180°.
            cam.set_local_pose(
                orientation=np.array([1.0, 0.0, 0.0, 0.0] if fwd
                                     else [0.0, 0.0, 1.0, 0.0]),
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
            rp = cam.get_render_product_path()
            ann = rep.AnnotatorRegistry.get_annotator("rgb")
            ann.attach(rp)
            # 🔑 Depth 도 같이 받는다. **distance_to_image_plane 이 아니라
            #    distance_to_camera** — 관 내부는 방사형이라 광축 투영 거리가
            #    아니라 실제 광선 거리가 필요하다(설계 §카메라, 실기 확정).
            #    결함 검출은 이쪽이 근거다. 구멍 너머는 inf 로 나온다.
            dep = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
            dep.attach(rp)
            rigs.append((nm, ann, dep))
        print(f"[준비] 카메라 프레임 저장 → {OUT}")
    except Exception as exc:
        print(f"[경고] 카메라 초기화 실패 — 사진 없이 진행 ({exc})")
        rigs = []


def _save_png(img, path):
    try:
        from PIL import Image
        Image.fromarray(img).save(path)
        return path
    except Exception:
        path = path.with_suffix(".ppm")
        with open(path, "wb") as f:
            f.write(b"P6\n%d %d\n255\n" % (img.shape[1], img.shape[0]))
            f.write(img.tobytes())
        return path


def snap(tag):
    """RGB 는 png, Depth 는 **npy + png** 로 저장한다.

    🚨 Depth 를 png 로만 남기면 못 쓴다. 8비트 256단계에 0.005~5.0m 를 담으면
    한 단계가 19.5mm 인데 우리가 재려는 홈은 0.6mm 다. 게다가 정규화 과정에서
    **inf 가 사라진다** — 구멍 너머가 inf 라는 것이 검출의 핵심 신호인데
    max 가 inf 가 되어 전체가 0 으로 뭉개진다.
    → 원본 float32 를 .npy 로 남기고, png 는 눈으로 보는 용도로만 따로 만든다.
    """
    if not rigs:
        return
    for nm, ann, dep in rigs:
        d = ann.get_data()
        if d is None or getattr(d, "size", 0) == 0:
            print(f"  [사진] {nm} {tag}: 프레임 없음 (headless 면 정상)")
        else:
            img = np.asarray(d)[:, :, :3].astype(np.uint8)
            q = _save_png(img, OUT / f"{tag}_{nm}_rgb.png")
            print(f"  [사진] {q.name}  평균밝기 {img.mean():.1f}/255")

        z = dep.get_data()
        if z is None or getattr(z, "size", 0) == 0:
            print(f"  [Depth] {nm} {tag}: 프레임 없음")
            continue
        a = np.asarray(z, dtype=np.float32)
        np.save(OUT / f"{tag}_{nm}_depth.npy", a)

        # 통계 — 숫자가 물리적으로 말이 되는지 여기서 바로 본다
        fin = np.isfinite(a) & (a > 0)
        n_inf = int(np.isinf(a).sum())
        n_nan = int(np.isnan(a).sum())
        n_zero = int((a == 0).sum())
        if fin.any():
            v = a[fin]
            note = "  ← 단위가 mm 인 듯" if v.min() > 10.0 else ""
            print(f"  [Depth] {tag}_{nm}_depth.npy  min {v.min():.4f} "
                  f"max {v.max():.4f} 중앙값 {np.median(v):.4f} m  "
                  f"무효 {(a.size - fin.sum()) / a.size * 100:.1f}% "
                  f"(inf {n_inf:,} / nan {n_nan:,} / 0 {n_zero:,}){note}")
        else:
            print(f"  [Depth] {tag}_{nm}: 유효 픽셀 0 — "
                  f"inf {n_inf:,} / nan {n_nan:,} / 0 {n_zero:,}")

        # 눈으로 볼 png — 유효 픽셀만으로 정규화하고 무효는 빨강으로 표시
        g = np.zeros(a.shape, np.uint8)
        if fin.any():
            v = a[fin]
            lo, hi = float(v.min()), float(v.max())
            if hi > lo:
                g[fin] = ((a[fin] - lo) / (hi - lo) * 255).astype(np.uint8)
        rgb = np.dstack([g, g, g])
        rgb[~fin] = (255, 0, 0)          # 무효(=구멍 너머) 는 빨강
        q = _save_png(rgb, OUT / f"{tag}_{nm}_depth.png")
        print(f"          {q.name} (회색=거리, 빨강=무효)")


_rng = np.random.default_rng(3)
_recycled = 0
_flow_now = 0.0
_leaked = 0


def recycle_particles():
    """하류로 나간 입자를 상류로 재투입 — 고정 입자 수로 연속 흐름 표현.

    전역 wind 는 단방향이라 굽은 경로에 못 쓴다 → 입자마다 경로 접선을 구해
    상류→하류로 속도를 끌어당긴다. **바닥 개구로 빠져나간 입자도 여기서
    잡아 재투입**하므로 누수가 끊기지 않고 계속 보인다."""
    global _recycled, _flow_now, _leaked
    pts = np.array(water_instancer.GetPositionsAttr().Get())
    vels = np.array(water_instancer.GetVelocitiesAttr().Get())
    d, tx, ty = path_dist_tangent(pts[:, 0], pts[:, 1])
    spd = abs(FLOW_V)
    vels[:, 0] += FLOW_BLEND * (-tx * spd - vels[:, 0])
    vels[:, 1] += FLOW_BLEND * (-ty * spd - vels[:, 1])
    _flow_now = float(np.mean(-(vels[:, 0] * tx + vels[:, 1] * ty)))
    off = np.hypot(d, pts[:, 2])
    below = pts[:, 2] < -(PIPE_IR + 0.010)      # 개구로 빠져 관 아래로 낙하
    _leaked = int(below.sum())
    mask = ((pts[:, 0] < -S_IN) | (off > 0.055)
            | (pts[:, 1] > S_OUT + 0.02) | below)
    n = int(mask.sum())
    if n:
        pts[mask, 0] = OUT_X + _rng.uniform(-0.02, 0.02, n)
        pts[mask, 1] = _rng.uniform(*INJECT_Y, n)
        pts[mask, 2] = _rng.uniform(W_LEVEL_Z, W_LEVEL_Z + 0.015, n)
        vels[mask] = (0.0, -spd, -0.2)
    water_instancer.GetPositionsAttr().Set(
        Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))
    water_instancer.GetVelocitiesAttr().Set(
        Vt.Vec3fArray.FromNumpy(vels.astype(np.float32)))
    _recycled += n


def leak_count():
    """지금 개구 아래로 빠져 있는 입자 수 — 누수의 정량 지표."""
    if water_instancer is None:
        return 0
    z = np.array(water_instancer.GetPositionsAttr().Get())[:, 2]
    return int((z < -(PIPE_IR + 0.010)).sum())


# ── 시퀀스 ──────────────────────────────────────────────────────────
STOP_TOL = 0.003
INSPECT_BACK_MM = 120.0     # 결함 몇 mm 앞에서 세워 촬영할지
ARC_STEPS = int(2.0 * PHYSICS_HZ)        # 아크 2초
COOL_STEPS = int(1.0 * PHYSICS_HZ)
REPOS_MM = 120.0

state = "SETTLE"
t_state = 0
inspected = False
repos_from = None
log = []

# 속도 계측 — 설정을 바꿔 가며 비교하려면 숫자가 있어야 한다.
import time                                              # noqa: E402
_t_mark, _step_mark = time.time(), 0
_f_drag = 0.0

print("-" * 78)
for step in range(STEPS):
    world.step(render=not HEADLESS)
    if step - _step_mark >= 5 * PHYSICS_HZ:
        _dt = time.time() - _t_mark
        _rate = (step - _step_mark) / max(_dt, 1e-9)
        print(f"  [속도] 물리 {_rate:6.1f} step/s  "
              f"(실시간 대비 {_rate / PHYSICS_HZ:4.2f}x)"
              + ("" if not WATER else
                 f"  입자 {n_particles:,}  RECYCLE_HZ={RECYCLE_HZ:.0f}"
                 f"  ISO_PASSES={ISO_PASSES}")
              + ("" if not FLUID else f"  항력 {_f_drag:+.2f}N"))
        _t_mark, _step_mark = time.time(), step
    if WATER and step % RECYCLE_EVERY == 0:
        recycle_particles()
    if FLUID:
        _f_drag = apply_fluid()
    t_state += 1
    ring = wpos(_ring)
    tip = wpos(_tip)
    tip_r = tip_end_r() if not NO_TORCH else 0.0

    if state == "SETTLE":
        if t_state > (6.0 if WATER else 1.5) * PHYSICS_HZ:
            p = wpos(_seg1)
            print(f"[SETTLE] 안착 seg1 ({p[0] * 1000:.1f}, {p[1] * 1000:.1f},"
                  f" {p[2] * 1000:.1f})mm — 주행 시작")
            wr = []
            for sg in (0, 1):
                for i in range(3):
                    for jj in (0, 1):
                        w = wpos(stage.GetPrimAtPath(
                            f"{ROBOT}/seg{sg}_wheel_{i}_{jj}"))
                        wr.append(math.hypot(w[1] - IN_Y, w[2]) * 1000)
            print(f"  휠 중심 반경 {min(wr):.1f}~{max(wr):.1f}mm (밀착 40.0) "
                  f"밀착 {sum(1 for r in wr if r > 39.0)}/12"
                  + ("" if NO_TORCH else
                     f"   토치 링 x{wpos(_ring)[0] * 1000:.1f} r"
                     f"{math.hypot(wpos(_ring)[1] - IN_Y, wpos(_ring)[2]) * 1000:.1f}"
                     f"  팁 x{wpos(_tip)[0] * 1000:.1f} r{tip_r:.1f}"))
            for sg in (0, 1):
                seg = wpos(stage.GetPrimAtPath(f"{ROBOT}/seg{sg}_body"))
                print(f"  seg{sg}_body x{seg[0] * 1000:8.1f} "
                      f"r{math.hypot(seg[1] - IN_Y, seg[2]) * 1000:5.1f}mm")
            state, t_state = "APPROACH", 0

    elif state == "INSPECT":
        # 🚨 전방 카메라는 토치 링보다 13mm **앞**에 있다. 링이 결함에 닿은
        #    시점에는 결함이 이미 카메라 뒤라 안 찍힌다. 그래서 도달 전에
        #    한 번 세워 촬영한다(정찰 → 판정 순서와도 맞는다).
        drive(0.0)
        if t_state > 0.7 * PHYSICS_HZ:      # 렌더·밝기 안정화
            print(f"[INSPECT] 결함 {INSPECT_BACK_MM:.0f}mm 앞에서 정지 — 촬영")
            snap("1_before")
            state, t_state = "APPROACH", 0

    elif state == "APPROACH":
        drive(SPIN_DEG_S)
        set_torch(j1_deg=0.0, j2_m=0.0)
        if not inspected and ring[0] >= DEFECT_X - INSPECT_BACK_MM * MM:
            inspected = True
            drive(0.0)
            state, t_state = "INSPECT", 0
        elif ring[0] >= DEFECT_X - STOP_TOL:
            drive(0.0)
            print(f"[APPROACH] 결함 도달 — 링 x={ring[0] * 1000:.1f}mm "
                  f"(목표 {DEFECT_X * 1000:.0f}), {t_state} 스텝")
            state, t_state = "ALIGN", 0
        elif t_state % 400 == 0:
            q = np.asarray(art.get_joint_positions(), dtype=float)
            v = np.asarray(art.get_joint_velocities(), dtype=float)
            wv = np.degrees(np.mean([v[k] for k in wheel_idx]))
            wq = np.degrees([q[k] for k in wheel_idx])
            pis = [q[k] * 1000 for k in piston_idx]
            bel = [math.degrees(q[k]) for k in bel_idx]
            tj = ("" if j1_idx is None else
                  f"  J1 {math.degrees(q[j1_idx]):+6.1f}°"
                  f"  J2 {q[j2_idx] * 1000:5.2f}mm  팁r {tip_r:5.1f}")
            print(f"  주행 중 링x{ring[0] * 1000:8.1f}  휠각속도 {wv:8.1f}deg/s"
                  f"  (지령 {SPIN_DEG_S:.0f})  휠각 {wq.min():7.1f}~{wq.max():7.1f}"
                  f"  피스톤 {min(pis):4.1f}~{max(pis):4.1f}"
                  f"  벨로우즈 {min(bel):+5.1f}~{max(bel):+5.1f}{tj}")

    elif state == "ALIGN":
        drive(0.0)
        set_torch(j1_deg=DEFECT_CLOCK_DEG, j2_m=0.0)
        j1_now = math.degrees(float(art.get_joint_positions()[j1_idx]))
        if abs(j1_now - DEFECT_CLOCK_DEG) < 1.0 and t_state > 0.5 * PHYSICS_HZ:
            print(f"[ALIGN] J1 = {j1_now:+.2f}° (목표 "
                  f"{DEFECT_CLOCK_DEG:+.0f}°) — 토치가 결함을 향한다")
            state, t_state = "EXTEND", 0
        elif t_state > 4 * PHYSICS_HZ:
            print(f"[ALIGN] 시간 초과, J1 = {j1_now:+.2f}° — 그대로 진행")
            state, t_state = "EXTEND", 0

    elif state == "EXTEND":
        drive(0.0)
        set_torch(j1_deg=DEFECT_CLOCK_DEG, j2_m=J2_STROKE)
        j2_now = float(art.get_joint_positions()[j2_idx])
        if j2_now > J2_STROKE * 0.9 or t_state > 3 * PHYSICS_HZ:
            print(f"[EXTEND] J2 = {j2_now * 1000:.2f}mm  팁 끝 반경 "
                  f"{tip_r:.2f}mm (관벽 50.0 → 용접 간극 {50 - tip_r:.2f}mm, "
                  f"설계 2mm)")
            state, t_state = "ARC", 0

    elif state == "ARC":
        drive(0.0)
        set_torch(j1_deg=DEFECT_CLOCK_DEG, j2_m=J2_STROKE)
        arc_light.GetIntensityAttr().Set(3.0e5)
        if t_state == 1:
            print(f"[ARC] 아크 {ARC_STEPS / PHYSICS_HZ:.1f}초")
        if t_state > ARC_STEPS:
            arc_light.GetIntensityAttr().Set(0.0)
            # 🔑 수리 표현 = 프림 가시성 전환 (CLAUDE.md 채택 방식)
            UsdGeom.Imageable(defect_mesh).MakeInvisible()
            UsdGeom.Imageable(bead_mesh).MakeVisible()
            # 물리로도 막는다 — 키네마틱 마개를 개구 자리로 이동
            _before = leak_count()
            _plug_rb.set_world_pose(position=np.array(
                [PLUG_SEAT[0], PLUG_SEAT[1], PLUG_SEAT[2]]))
            print(f"[SWAP] 결함 → 비드 전환 + 개구 마개 착좌 "
                  f"(직전 누수 입자 {_before}개)")
            state, t_state = "COOL", 0

    elif state == "COOL":
        drive(0.0)
        set_torch(j1_deg=DEFECT_CLOCK_DEG, j2_m=0.0)
        if t_state > COOL_STEPS:
            j2_now = float(art.get_joint_positions()[j2_idx])
            print(f"[COOL/RETRACT] J2 = {j2_now * 1000:.2f}mm 수납")
            repos_from = ring[0]
            state, t_state = "REPOSITION", 0

    elif state == "REPOSITION":
        # 용접 지점은 제자리에서 안 보인다 → 후진해서 전방 카메라로 확인
        drive(-SPIN_DEG_S)
        set_torch(j1_deg=0.0, j2_m=0.0)
        if repos_from - ring[0] >= REPOS_MM * MM:
            drive(0.0)
            print(f"[REPOSITION] 후진 {(repos_from - ring[0]) * 1000:.1f}mm "
                  "— 전방 카메라로 비드 확인 자세")
            state, t_state = "VERIFY", 0

    elif state == "VERIFY":
        drive(0.0)
        if t_state > 1.0 * PHYSICS_HZ:
            snap("2_after")
            print(f"[VERIFY] 촬영 완료 — 누수 입자 {leak_count()}개"
                  + ("" if not WATER else " (수리 전과 비교할 것)"))
            state, t_state = "RESUME", 0

    elif state == "RESUME":
        # 🚨 **수리했다고 복귀하지 않는다.** 임무 규칙상 복귀 사유는 관 단절과
        #    용접봉 소진뿐이고(v3 §임무 규칙 8), 결함 수리는 점검 중 한 건을
        #    처리한 것일 뿐이다. 전진 점검을 계속해 코스 끝까지 간다.
        #    (단절 판정은 condition/ 이 담당하며 이 시연에는 아직 안 붙였다.)
        drive(SPIN_DEG_S)
        set_torch(j1_deg=0.0, j2_m=0.0)
        _s = path_s(wpos(_seg1)[0], wpos(_seg1)[1])
        if t_state == 1:
            print(f"[RESUME] 점검 계속 — 복귀 아님. 현재 진행 {_s * 1000:.1f}mm "
                  f"/ 코스 {S_TOTAL * 1000:.0f}mm")
        if _s > S_IN + S_ARC + 0.25:
            print(f"[END_REACHED] 코스 끝 도달 — 진행 {_s * 1000:.1f}mm. "
                  "여기서부터가 복귀 구간이다(이 시연은 여기서 끝낸다)")
            state, t_state = "DONE", 0
        elif t_state % 1200 == 0:
            print(f"  점검 주행 중  진행 {_s * 1000:7.1f}mm"
                  + (f"  누수 {leak_count()}" if WATER else ""))

    elif state == "DONE":
        drive(0.0)
        break

print("=" * 78)
p = wpos(_seg1)
vis_d = UsdGeom.Imageable(defect_mesh).ComputeVisibility()
vis_b = UsdGeom.Imageable(bead_mesh).ComputeVisibility()
print(f"결과  최종 상태 {state}  seg1 ({p[0] * 1000:.1f}, {p[1] * 1000:.1f}, "
      f"{p[2] * 1000:.1f})mm")
print(f"      결함 프림 {vis_d} / 비드 프림 {vis_b}")
if WATER:
    print(f"      물 입자 {n_particles:,}개, 유동 {_flow_now:.3f} m/s, "
          f"재투입 누계 {_recycled:,}, 현재 누수 {leak_count()}개")
print("판정  " + ("✅ 결함 → 비드 전환 완료 (수리 시연 성공)"
                if state == "DONE" and vis_b == "inherited"
                else f"△ 시퀀스가 {state} 에서 끝났다"))
print("[참고] 배관 충돌 메시에 실제 구멍은 뚫려 있지 않다 — "
      "CLAUDE.md 채택 방식(프림 가시성 전환)에 따른 시각 시연이다")

sys.stdout.flush()
if HOLD and not HEADLESS:
    print("GUI 실행 중 — 창을 닫으면 종료됩니다")
    while simulation_app.is_running():
        world.step(render=True)

import threading                                          # noqa: E402

threading.Thread(target=simulation_app.close, daemon=True).start()
threading.Event().wait(8.0)
sys.stdout.flush()
os._exit(0)
