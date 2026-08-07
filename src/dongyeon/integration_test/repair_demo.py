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
  (없음)    배수 조건. 마찰 0.40/0.35, 유체력 없음

  --glass   배관을 반투명으로 렌더한다. **물과 무관한 표시 옵션**이고 물리에는
            영향이 없다. --water 는 자동으로 켠다.
            예) 배수관을 투명하게 들여다보기:
                DISPLAY=:1 isaac_python repair_demo.py --glass --hold

주행 속도
  SPEED_MPS=0.05  기본은 설계값 0.05 m/s. **카메라 10Hz 로 결함을 찾으며
                  가므로** 속도가 곧 검출 성능이다. 프레임당 전진이
                  0.05→5mm / 0.15→15mm / 0.30→30mm 이고, 결함 38mm 가
                  화면에 걸리는 프레임 수가 각각 7.6 / 2.5 / 1.3 장이다.
                  올렸다가(6배→3배) 검출을 놓쳐 설계값으로 되돌렸다.
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
# 🚨 **--known-defect : 결함 위치를 미리 알고 간다 (시연 전용).**
#    평소 이 시연의 원칙은 "제어는 카메라, 판정은 정답" 이고, 감지 실패 시
#    정답으로 대신하는 것은 금지돼 있다(CLAUDE.md — 실패가 드러나야 한다).
#    이 플래그는 그 금지를 어기는 것이 아니라 **선언하고 끄는 것**이다:
#    켜면 로그가 매 단계 "정답 사용" 이라고 찍고, 검증(VERIFY)이 무의미해진다.
#    결함이 물에 잠겨 있어 카메라 검출이 불안정한 동안 시연을 먼저 돌리기
#    위한 임시 모드다(2026-08-06 사용자 지시). 성능 평가에는 쓰지 말 것.
KNOWN_DEFECT = "--known-defect" in sys.argv
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
# --glass: 배관을 반투명 유리로 렌더한다. **물과는 무관한 순수 표시 옵션이다.**
# 전에는 투명 처리가 --water 안에 묻혀 있어서 "배수관인데 안이 보이게" 가
# 불가능했다(물을 빼면 관이 불투명해진다). 물리에는 아무 영향이 없다 —
# 물리 재질(마찰)은 `materialPurpose="physics"` 로 따로 바인딩되고,
# 유리는 `strongerThanDescendants` 로 **표시 재질만** 덮는다.
# --water 는 물을 봐야 하므로 자동으로 켠다.
GLASS = WATER or "--glass" in sys.argv
if FLUID and WATER:
    print("[경고] --fluid 와 --water 를 같이 줬다. 입자 충돌이 이미 항력을 "
          "만들므로 해석적 유체력은 끈다(이중 계상 방지).")
    FLUID = False
STEPS = 9000
if "--steps" in sys.argv:
    STEPS = int(sys.argv[sys.argv.index("--steps") + 1])

from isaacsim import SimulationApp                        # noqa: E402

simulation_app = SimulationApp({"headless": HEADLESS})

import cv2                                                # noqa: E402
import numpy as np                                        # noqa: E402
from isaacsim.core.api import World                       # noqa: E402
from isaacsim.core.prims import SingleArticulation        # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from pxr import (Gf, PhysxSchema, Sdf, UsdGeom, UsdLux,   # noqa: E402
                 UsdPhysics, UsdShade, Vt)

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
# 결함 축방향 위치 (직관은 x=0 에서 곡관으로 넘어간다).
# 🔑 **-0.05 에서 -0.022 로 옮겼다** (2026-08-05). -0.05 면 안착 직후 전방 카메라가
#    결함에서 **45.8mm** 밖에 안 떨어져 θ 47.5° → r 434px 인데, 이 카메라의
#    **세로 반화각은 39.4°(360px)** 뿐이라 결함이 화면 밖으로 나간다
#    (가로는 70°/640px 라 여유가 있어서, 카메라 롤을 바로잡기 전에는 왼쪽에
#     찍히며 우연히 들어와 있었다). 그러면 수리 전 검출이 표본 0 으로 죽는다.
# 창 반폭 band_px=36 까지 들어오려면  r + 36 ≤ 360 → 카메라까지 ≥ 70.3mm,
# 개구 ø38.1 이 곡관으로 넘어가지 않으려면  DEFECT_X ≤ -19mm.
#   → 성립 구간 -25.5 ~ -19mm. 가운데인 -22mm 를 쓴다(카메라 73.8mm, r 312px).
# ⚠ 로봇 시작 위치(START_X)가 뒤로 못 가서 생긴 제약이다. 실전 맵에서는
#   결함을 코스 하류에 두고 제대로 주행해서 접근하는 것이 맞다.
DEFECT_X = -0.022
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

# 주행 속도 — **설계값 0.05 m/s.**
# 🚨 이력: 시연이 느려 6배(0.30)로 올렸다가 3배(0.15)로 내렸고, 검출 기반
#    주행으로 바꾼 뒤 **다시 설계값으로 되돌렸다**(2026-08-05 사용자 지시:
#    GUI 로 직접 보고 *"속도가 너무 빨라서 감지를 못한다"*).
#    근거는 처음부터 같다 — 이 로봇은 **카메라 10Hz 로 결함을 찾으며 간다.**
#      0.30 m/s → 프레임당 30mm. 결함 38mm 가 1.3 프레임에만 걸린다
#      0.15 m/s → 프레임당 15mm. 2.5 프레임
#      0.05 m/s → 프레임당  5mm. 7.6 프레임 — 여기서야 여유가 생긴다
#    속도를 올리면 **정지 위치도 그만큼 늦어져** 결함을 지나친 자리에서
#    목표를 잡는다(실측: 0.15 에서 목표가 정답보다 13.7mm 앞).
# SPEED_MPS 로 바꿀 수 있다.
TARGET_SPEED_MPS = float(os.environ.get("SPEED_MPS", 0.05))
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
# 🚨 **GPU 물리에서는 이 하한이 통하지 않는다.** --water 는 파티클 때문에 GPU
# dynamics 를 켜는데(아래 enable_gpu_dynamics), GPU 접촉 생성 허용치보다 작은
# contactOffset 을 주면 접촉이 아예 생기지 않아 **로봇이 배관을 뚫고 낙하한다**
# (실측 기록: "0.0005 는 CPU 전용"). 물속 주파 성공(밀착 12/12)도 배관이 엔진
# 기본값 0.02 였을 때 나온 결과다. 그러니 물 모드에서는 0.02 를 바닥으로 깐다.
# — 건식·--fluid 는 CPU 라 해당 없고 계산값을 그대로 쓴다.
if WATER:
    CONTACT_OFFSET = max(0.02, CONTACT_OFFSET)
REST_OFFSET = 0.0

# ── 토치 (son parts_meta) ───────────────────────────────────────────
RING_R_OUT = TORCH["ring_r_out"] * MM        # 0.030
ROD_ORIGIN_R = TORCH["rod_origin_r"] * MM    # 0.030
ROD_LEN = TORCH["rod_len"] * MM              # 0.005
J2_STROKE = TORCH["j2_stroke_mm"] * MM       # 0.008
STOW_R = TORCH["stow_radius_mm"] * MM        # 0.040
REACH_R = TORCH["reach_radius_mm"] * MM      # 0.048

# ── 용접 간극 — J2 는 **폐루프로** 닫는다 ───────────────────────────
# 🚨 고정 스트로크로 뻗으면 간극이 원리적으로 안 맞는다. 설계 신장 48mm 는
#    **토치 링 중심이 관 중심선과 일치할 때** 팁을 관벽 2mm 앞에 두는 값인데,
#    로봇은 관 중심에 있지 않다(피스톤 6개가 자기들끼리 균형 잡은 자리에
#    선다). 실측 편심은 링 r6.0mm 였고, 그 방향이 토치와 같으면 팁 반경이
#    48+6 = 54mm 가 되어 관벽을 지나친다.
#      2026-08-05 실측  팁 54.57 / 53.89mm → 간극 **-4.57 / -3.89mm**
#      2026-08-04 실측  팁 44.91mm        → 간극 **+5.09mm** (부호 반대)
#    같은 J2 값이 실행마다 반대로 틀어진다 — 열린 루프로는 못 맞춘다.
# 🚨 게다가 지금은 팁이 **누수 개구(ø38.1) 자리로 뻗는다.** 그 자리는 삼각형을
#    지워 뚫어 놨으므로 막아 줄 벽이 없고, 팁이 관 밖으로 튀어나간다.
#    "벽을 파고든다" 가 아니라 "구멍으로 새어 나간다" 가 정확한 서술이다.
# → 측정한 **팁 끝 반경**으로 닫는다. 실기에서 이 되먹임은 영상 또는
#   서스펜션 엔코더(편심 추정)가 준다. 여기서는 시뮬레이터 기하를 쓴다.
WELD_GAP = 0.002                              # 설계 용접 간극 (v3 §2.6)
TIP_TARGET_R = (PIPE_IR - WELD_GAP) * 1000    # 48.0 mm — 팁 끝 목표 반경
J2_KP = 0.6                   # 팁 반경 오차(mm) → J2 보정(mm). 1.0 은 진동한다
J2_TOL = 0.15                 # 이 안이면 도달로 본다 (mm)
# J1 도 같은 이유로 폐루프다 (real_map 검증값 이식, 2026-08-06) — 스프링
# 드라이브라 지령과 실제가 벌어지고, 편심 때문에 같은 J1 이어도 팁이 가리키는
# 시계각이 다르다. 측정한 **팁 끝 시계각**으로 닫는다.
J1_KP = 0.5                   # 시계각 오차(deg) → J1 지령 보정(deg)
J1_TOL_DEG = 0.6              # 이 안이면 정렬로 본다 (r50 에서 0.52mm)

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


def path_point(px, py):
    """xy 점 → 가장 가까운 **중심선 점** (cx, cy). 중심선은 z=0 평면에 있다.

    `path_dist_tangent` 는 거리만 주는데, 스파크를 관에 가두려면 어느 쪽으로
    밀어내야 하는지(방향)가 필요해서 점 자체를 돌려준다. 세 갈래(입구 직관 /
    곡관 / 출구 직관) 판정은 위 두 함수와 같다.
    """
    ang = np.clip(np.arctan2(py, px), -np.pi / 2, 0.0)
    cx = np.stack([np.clip(px, -S_IN, 0.0), ARC_R * np.cos(ang),
                   np.full_like(px, OUT_X)])
    cy = np.stack([np.full_like(py, IN_Y), ARC_R * np.sin(ang),
                   np.clip(py, 0.0, S_OUT)])
    which = np.argmin(np.hypot(px - cx, py - cy), axis=0)
    i = np.arange(px.size) if px.ndim else 0
    return cx[which, i], cy[which, i]


def spark_confine(pts):
    """스파크 가둠 — 코스 중심선까지의 거리와 바깥 방향.

    🔑 직선 원기둥으로 근사하면 안 된다. 이 시연의 결함은 곡관 입구에서
       **22mm 앞**이라 스패터가 곧바로 곡관 구간으로 넘어간다.
    관 끝(입구 x=-343 / 출구 y=+343)에서는 중심선 점이 끝점에 고정되므로
    거리가 커져 저절로 되튕긴다 — 관 밖으로 새지 않는다.
    """
    cx, cy = path_point(pts[:, 0], pts[:, 1])
    v = np.stack([pts[:, 0] - cx, pts[:, 1] - cy, pts[:, 2]], axis=1)
    r = np.linalg.norm(v, axis=1)
    return r, v / np.maximum(r, 1e-12)[:, None], PIPE_IR


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
#    ⚠ 이 (x, y) 거리는 개구 축이 ±Z(천장/바닥)일 때만 맞다. 옆면(시계각 90·270)
#      으로 옮기면 축이 ±Y 가 되므로 (x, z) 로 재야 한다.
if DEFECT_CLOCK_DEG % 180.0 != 0.0:
    raise SystemExit(f"[중단] 개구 축 거리 계산이 천장/바닥(0·180°) 전용인데 "
                     f"시계각이 {DEFECT_CLOCK_DEG}° 다. (x, z) 로 재도록 고칠 것")
_pipe_mesh = UsdGeom.Mesh(stage.GetPrimAtPath(f"{PIPE}/geom"))
_pts = np.array(_pipe_mesh.GetPointsAttr().Get())
_idx = np.array(_pipe_mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
_cent = _pts[_idx].mean(axis=1)
_axis_d = np.hypot(_cent[:, 0] - DEFECT_X, _cent[:, 1] - IN_Y)
# 🚨 **여기서 반쪽을 안 자르면 천장에도 구멍이 뚫린다.** 위 거리는 (x, y) 만
#    보므로 사실상 **무한한 수직선**이고, 그 선은 관을 바닥과 천장 **두 번**
#    지난다. 실측(2026-08-05): 제거 98개 = 바닥 49 + 천장 49 — 구멍 하나면
#    49개다. 천장 구멍은 중력 반대편이라 물이 안 새서 눈에 덜 띄지만, GUI 에
#    구멍이 2개 보이고 마개는 바닥 하나만 막는다.
#    → 결함 시계각이 가리키는 **반쪽만** 남긴다. 규약은 아래 DEF_XF 와 같다
#      (+Z 에서 +Y 로 잰 각, Rx(-θ) 가 +Z 를 (0, sinθ, cosθ) 로 보낸다).
_th = math.radians(DEFECT_CLOCK_DEG)
_side = ((_cent[:, 1] - IN_Y) * math.sin(_th) + _cent[:, 2] * math.cos(_th)) > 0
_hole = _axis_d < LEAK_PORT_D / 2
_n_both = int(_hole.sum())
_hole &= _side
_kept = _idx[~_hole]
_pipe_mesh.GetFaceVertexIndicesAttr().Set(_kept.reshape(-1).tolist())
_pipe_mesh.GetFaceVertexCountsAttr().Set([3] * len(_kept))
print(f"[준비] 배관 {'바닥' if DEFECT_CLOCK_DEG == 180 else '천장'}에 관통 개구 "
      f"ø{LEAK_PORT_D * 1000:.1f}mm — 삼각형 {int(_hole.sum())}개 제거 "
      f"({len(_idx)} → {len(_kept)})   반대쪽 {_n_both - int(_hole.sum())}개는 "
      f"남겼다(안 자르면 관통 구멍이 2개가 된다)")

# 감지 밴드(contactOffset) 덮어쓰기는 **로봇·토치·마개가 다 생긴 뒤**로 옮겼다.
# 여기(배관 직후)에서 돌리면 로봇이 아직 스테이지에 없어서 정작 넓혀야 할
# 휠에 안 닿는다 — 아래 「감지 밴드」 절 참조.

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
      f"({'물속에서 용접한다' if FLOODED else '건식 대조군'})"
      f"   배관 표시 {'반투명(유리)' if GLASS else '불투명'}")
if GLASS:
    print("[경고] --glass 는 **사람이 밖에서 보기 위한 표시 옵션**이다. 관이 "
          "반투명해지면 로봇 조명이 벽에 반사돼 돌아오지 않아 "
          "**카메라 검출·판정을 믿을 수 없다**")
    print("       판정을 보려면 --glass 를 빼고 돌릴 것. 촬영용으로만 쓸 것.")
_pm.CreateRestitutionAttr(0.0)

if GLASS:
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
        # 🚨 **배관을 양면 렌더로 켠다** (2026-08-06 실기로 원인 확정).
        #    `pipe_elbow_lr150.usda` 에는 doubleSided 속성이 아예 없어 USD
        #    기본값 false = **단면 메시**이고, 법선이 관 안쪽을 향한다. 그래서
        #    RTX 가 밖에서 본 면을 전부 걸러내 **관이 통째로 투명**했다 —
        #    안의 로봇도, 시각 물 층의 수면(922mm)도 밖에서 그대로 비쳐
        #    "물이 관 밖에서도 보인다 / 바닥이 물이 됐다" 로 나타났다.
        #    안쪽 면은 원래도 정면이라 **로봇 카메라·검출에는 변화가 없고**,
        #    표시 속성이라 물리에도 영향이 없다.
        UsdGeom.Mesh(_p).CreateDoubleSidedAttr(True)
        UsdShade.MaterialBindingAPI.Apply(_p).Bind(
            UsdShade.Material.Get(stage, "/World/PipePhysMat"),
            bindingStrength=UsdShade.Tokens.weakerThanDescendants,
            materialPurpose="physics")
        if GLASS:
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
# 🔑 **비드는 결함 자리가 아니라 토치가 실제로 있던 자리에 생긴다.**
#    설계 7.2: 정렬이 틀리면 비드가 어긋난 곳에 남고 결함은 그대로다.
#    예전에는 성공·실패 상관없이 DEF_XF 에 고정으로 띄워서, 정렬이 실패해도
#    비드가 결함 자리에 정확히 찍혀 **검증이 뚫렸다**(실측: 정렬 1.60mm 실패인데
#    비드가 예상자리와 5px 로 잡혀 ✅ 가 나왔다).
_bead_op = UsdGeom.Xformable(bead_mesh).GetOrderedXformOps()[0]
UsdGeom.Imageable(bead_mesh).MakeInvisible()      # 처음에는 숨긴다
print(f"[준비] 결함(구멍) x={DEFECT_X * 1000:.0f}mm, 시계각 "
      f"{DEFECT_CLOCK_DEG:.0f}°(바닥) — 비드는 숨김. "
      f"이 두 프림은 시각 전용이고, 물이 새는 것은 위의 관통 개구가 만든다")

# ── 물 (--water) : yongbin_drive_test.py 검증값 그대로 ──────────────
# 🚨 입자·물 값은 새로 만들지 않는다(사용자 지시). PCO 8 / fluid_rest 4.8 /
#    입자-강체 contact 0.004·rest 0.0025(미지정 시 로봇이 입자 위에 부상하는
#    함정) / 간격 9 / 유속 0.10 / blend 0.20.  **수위만 아래에서 바꾼다.**
FLOW_V, FLOW_BLEND = -0.10, 0.20
W_SPACING, W_PCO, W_FLUID_REST = 0.009, 0.008, 0.0048
# 🔑 **수위 z+20 → z−10** (2026-08-06 사용자 지시: "물 양을 지금의 절반으로").
#    z+20 은 관 단면적의 **74.8%**(= 사용자가 본 "3/4")를 채운다. 그 절반인
#    37.4% 가 되는 수위가 z−10mm 다 — 원의 활꼴 (α−sinα·cosα)/π 로 역산.
#    입자 4,987 → 1,857개(37%)라 --water 의 0.07배속도 같이 나아진다.
#    ⚠ 바닥 개구까지 수두가 70 → 40mm 로 줄어 토리첼리 유출속도가
#      1.17 → 0.89 m/s 가 된다. 누수는 그대로 나지만 조금 느려진다.
#    되돌리려면 WATER_LEVEL_MM=20 으로 실행할 것.
W_LEVEL_Z = float(os.environ.get("WATER_LEVEL_MM", -10.0)) / 1000.0
W_R_MAX = 0.045
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

    # 충수율(원의 활꼴 면적비) — 로그로 찍어 "얼마나 찼는지" 를 눈이 아니라
    # 숫자로 확인한다. level=+r 이면 1.0(만관), 0 이면 0.5, -r 이면 0.
    _a_fill = math.acos(max(-1.0, min(1.0, -W_LEVEL_Z / PIPE_IR)))
    _fill_frac = (_a_fill - math.sin(_a_fill) * math.cos(_a_fill)) / math.pi
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
          f"(출구→입구, 로봇 진행 반대), 수위 z{W_LEVEL_Z * 1000:+.0f}mm "
          f"— 단면적 {_fill_frac:.1%} 충수, 바닥 개구 수두 "
          f"{(W_LEVEL_Z + PIPE_IR) * 1000:.0f}mm")
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
# 🔑 해상도는 **GUI 연산량의 큰 몫**이다(카메라 2대 × 렌더 프로덕트).
#    설계 기준 1280x720 의 절반이면 화소 수가 **1/4** 이라 렌더가 크게 가벼워진다.
#    기본을 640x360 으로 두고, 설계값으로 되돌리려면 `CAM_RES=1280x720`.
# 🚨 해상도를 바꾸면 **화소 개수로 잡은 임계가 조용히 깨진다**(기록된 함정).
#    - 초점거리 F_PX 는 화각에서 유도하므로 자동으로 따라간다(어안 140° 유지).
#    - **면적 임계는 손으로 같이 줄여야 한다** → CAM_AREA_SCALE 로 곱한다.
#      `condition` 의 `aperture_min_px`(400) 과 우리 `HOLE_MIN_PX`(300) 둘 다.
_res = os.environ.get("CAM_RES", "640x360").lower().split("x")
CAM_W, CAM_H, CAM_HFOV = int(_res[0]), int(_res[1]), 140.0
CAM_AREA_SCALE = (CAM_W * CAM_H) / (1280.0 * 720.0)   # 설계 기준 대비 면적비
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
# 🔑 카메라 프림 이름이 곧 **GUI 카메라 목록에 뜨는 이름**이다. 예전에는 둘 다
#    `.../sensor` 라서 뷰포트 드롭다운에 `sensor` 가 두 개 나와 구분이 안 됐다.
#    → 하우징 Xform 을 `<이름>_rig` 로 물리고 카메라 프림 자체를 `<이름>` 으로 둔다.
#      `/World/Robot/seg1_body/front_camera_rig/front_camera`  ← 용접기가 달린 쪽
#      `/World/Robot/seg0_body/back_camera_rig/back_camera`    ← 그 뒤쪽
#    토치(`torch_ring`)는 seg1_body 에 붙으므로 **front 가 용접기 쪽**이 맞다.
CAM_SPECS = [
    # 이름, 부모 링크, 링크 로컬 z, 전방 여부
    ("front_camera", "seg1_body", -0.045, True),
    ("back_camera", "seg0_body", +0.037, False),
]


def cam_prim_path(nm, seg):
    return f"{ROBOT}/{seg}/{nm}_rig/{nm}"


FRONT_CAM = cam_prim_path(*[(a, b) for a, b, _c, _d in CAM_SPECS
                            if a == "front_camera"][0])
for nm, seg, z, fwd in CAM_SPECS:
    base = f"{ROBOT}/{seg}/{nm}_rig"
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
print(f"[준비] 카메라 2대 (어안 {CAM_HFOV:.0f}°, {CAM_W}x{CAM_H} "
      f"= 설계 대비 화소 {CAM_AREA_SCALE * 100:.0f}%, "
      f"f={F_PX:.1f}px) + 조명 4  — 전방 seg1 링크로컬 z={-45}mm, "
      f"후방 seg0 z={+37}mm  — GUI 카메라 목록: front_camera / back_camera")

# ── 누수 마개 ───────────────────────────────────────────────────────
# 결함/비드 프림은 시각 전용이라 가시성만 바꿔서는 물이 계속 샌다. 수리 성공을
# **물리로도** 표현하려면 개구를 막아야 하는데, 배관 삼각 메시는 reset 때 cook
# 되므로 런타임에 구멍을 되메울 수 없다. → 키네마틱 강체 마개를 관 밖에
# 세워 두었다가 용접 성공 시 개구 자리로 옮긴다.
# 🔑 **메꿈은 잘라낸 벽 조각 그대로 만든다** (2026-08-05).
#    예전에는 ø50 원통을 관 밖(반경 53mm)에 갖다 붙였는데, 두 가지가 틀렸다:
#      ① **보이지 않았다** (`MakeInvisible`) → 카메라가 못 본다. 수리 후에도
#         구멍이 그대로 뚫려 보이고 Depth 가 "여전히 파임" 을 낸다.
#         실제 촬영으로 확인했다(out/repair_demo/2_after_front_camera_rgb.png —
#         아래쪽에 톱니 모양 구멍이 그대로 남아 있다).
#      ② **평면이라 원통 벽과 맞을 수 없다.** 납작한 뚜껑을 중앙에서 맞춰도
#         가장자리는 관축에서 더 멀다: 구멍 테두리(ρ=19mm)에서 √(19²+50²)=53.5mm
#         라 +3.5mm 파임으로 읽힌다. 곡률이 다른 물체로는 원리적으로 못 맞춘다.
#    → 관에서 **지운 삼각형 49개를 그대로 되살려** 메꿈으로 쓴다. 원본 벽면과
#      완전히 같은 곡률·반경이라 되돌려 놓으면 Depth 가 정확히 50.00mm 를 본다.
#      정점이 이미 월드 좌표라 자리(seat)는 원점, 대기(park)는 아래로 1m.
_hole_tris = _idx[_hole]
_used, _remap = np.unique(_hole_tris, return_inverse=True)
_plug = UsdGeom.Mesh.Define(stage, "/World/LeakPlug")
# 배관 usda 의 정점은 float32 다 — Gf.Vec3f 는 numpy 스칼라를 안 받는다.
_plug_pts = np.asarray(_pts[_used], dtype=float)
_plug.CreatePointsAttr([Gf.Vec3f(float(a), float(b), float(c))
                        for a, b, c in _plug_pts])
_plug.CreateFaceVertexCountsAttr([3] * len(_hole_tris))
_plug.CreateFaceVertexIndicesAttr(_remap.reshape(-1).tolist())
_plug.CreateExtentAttr([Gf.Vec3f(*[float(v) for v in _plug_pts.min(0)]),
                        Gf.Vec3f(*[float(v) for v in _plug_pts.max(0)])])
_plug.CreateSubdivisionSchemeAttr("none")
_plug.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.55, 0.20)])   # 용접 비드 색
_plug_prim = _plug.GetPrim()
# 🚨 **강체로 만들지 않는다.** 강체의 렌더·물리 자세는 PhysX 가 쥐고 있어서
#    시뮬 시작 뒤 USD 변환을 써도 안 먹고, 키네마틱이면 Isaac 의 rigid-prim
#    경로가 reset 때 `setLinearVelocity: Body must be non-kinematic!` 로 실패한다.
#    실측으로 세 번(원통/메시 + set_world_pose/USD 변환) 다 마개가 제자리로
#    돌아오지 않았다 — 판정값이 비트 단위로 같았다.
#    → **옮기지 않는다.** 잘라낸 자리에 그대로 두고 **보이기·충돌만 켠다.**
#      결함/비드 전환이 이미 이 방식으로 확실히 동작한다(CLAUDE.md 채택 방식).
_plug_coll = UsdPhysics.CollisionAPI.Apply(_plug_prim)
_plug_coll.CreateCollisionEnabledAttr(False)
UsdPhysics.MeshCollisionAPI.Apply(_plug_prim).CreateApproximationAttr("none")
_px_plug = PhysxSchema.PhysxCollisionAPI.Apply(_plug_prim)
_px_plug.CreateContactOffsetAttr(CONTACT_OFFSET)
_px_plug.CreateRestOffsetAttr(REST_OFFSET)
UsdGeom.Imageable(_plug_prim).MakeInvisible()   # 용접 성공 시 켠다
print(f"[준비] 메꿈 조각 대기 — 잘라낸 벽 삼각형 {len(_hole_tris)}개 "
      f"(정점 {len(_used)}개), 지금은 숨김+충돌끔. "
      "— 용접 성공 시 개구로 이동")

# ── 감지 밴드 (contactOffset) ───────────────────────────────────────
# 속도에 맞춰 넓힌다. **반드시 여기서 돌린다** — 배관·로봇·토치·마개가 전부
# 스테이지에 올라온 뒤이고 world.reset() 의 cook 보다는 앞이다.
#
# 🚨 배관 직후에서 돌리면 안 된다(한 번 그렇게 했다가 되돌렸다). 그 시점엔
#    로봇 usda 가 아직 AddReference 되기 전이라 **휠 콜라이더 20개가 0.0005 인
#    채로 남고**, 정작 contactOffset 이 없어 엔진 기본 0.02 를 쓰던 배관만
#    좁아진다 — 넓히려던 쪽은 그대로 두고 반대쪽만 조이는 꼴이었다.
_n_off = 0
for _p in stage.Traverse():
    if _p.HasAPI(UsdPhysics.CollisionAPI) or _p.IsA(UsdGeom.Mesh):
        _px = PhysxSchema.PhysxCollisionAPI.Apply(_p)
        _px.CreateContactOffsetAttr(CONTACT_OFFSET)
        _px.CreateRestOffsetAttr(REST_OFFSET)
        _n_off += 1
print(f"[준비] 주행 {TARGET_SPEED_MPS * 1000:.0f} mm/s "
      f"(한 스텝 {TARGET_SPEED_MPS / PHYSICS_HZ * 1000:.3f}mm) → "
      f"contactOffset {CONTACT_OFFSET * 1000:.2f}mm, 프림 {_n_off}개"
      + ("  ※ 물 모드 GPU 하한 0.02 적용" if WATER else ""))
_n_wheel = sum(1 for _p in stage.Traverse()
               if "_wheel_" in str(_p.GetPath())
               and _p.HasAPI(PhysxSchema.PhysxCollisionAPI))
print(f"         그중 휠 콜라이더 {_n_wheel}/12 개"
      + ("" if _n_wheel == 12 else "  ⚠ 12 가 아니다 — 순서가 또 틀어졌다"))

art = SingleArticulation(prim_path=ROBOT, name="bellows_welder")
world.scene.add(art)
# 🚨 마개를 `world.scene.add()` 하지 않는다. Isaac 이 reset 때 등록 물체의
#    속도를 0 으로 쓰는데 키네마틱 바디는 그걸 거부해
#    `PxRigidDynamic::setLinearVelocity: Body must be non-kinematic!` 가 2건씩
#    뜬다(재시작마다 반복). 게다가 그 경로로 준 `set_world_pose` 가 실제로
#    먹지 않아 메꿈이 제자리로 돌아오지 않았다(촬영으로 확인).
#    → USD 변환(`_plug_op`)으로만 옮긴다. 메모리에 적어 둔 방침 그대로다.
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


def tip_end_world():
    """토치 팁 끝의 월드 좌표(m). 아크가 실제로 일어나는 점이다."""
    _XC.Clear()
    w = _XC.GetLocalToWorldTransform(_tip).Transform(TIP_END_LOCAL)
    return np.array([float(w[0]), float(w[1]), float(w[2])])


def tip_end_r():
    """토치 팁 끝의 **관 중심선** 기준 반경(mm)."""
    w = tip_end_world()
    return math.hypot(w[1] - IN_Y, w[2]) * 1000


def wall_inward(p):
    """점 p(월드, m)에서 **관 안쪽을 향하는** 단위벡터.

    스패터가 튀어 나가는 쪽이다. 이 시연의 결함은 입구 직관(관 축 = X,
    중심선 y=IN_Y, z=0)에 있으므로 반경 성분은 (y−IN_Y, z) 로 잰다.
    `wall_uv` 와 같은 규약이다.
    """
    dy, dz = float(p[1]) - IN_Y, float(p[2])
    r = math.hypot(dy, dz)
    if r < 1e-9:
        return np.array([0.0, 0.0, 1.0])
    return np.array([0.0, -dy / r, -dz / r])


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


# ── 카메라 ──────────────────────────────────────────────────────────
# 🔑 **카메라는 항상 켠다.** 예전에는 이 블록 전체가 `if SHOTS:` 안에 있어서
#    `--shots` 없이 돌리면 카메라 프림 자체가 안 생겼다. 그러면 뷰포트
#    Perspective 목록에도 안 뜨고, 무엇보다 **검출·판정이 돌 수가 없다.**
#    카메라는 이 로봇의 주 센서다 — 옵션이 아니다.
#    `--shots` 는 이제 **파일로 저장할지**만 정한다(아래 snap 참조).
rigs = []
if True:
    try:
        import omni.replicator.core as rep
        from isaacsim.sensors.camera import Camera
        OUT.mkdir(parents=True, exist_ok=True)
        for nm, seg, z, fwd in CAM_SPECS:
            # 좌표는 **링크 로컬**. 루트 기준 오프셋을 더하면 안 된다.
            cam = Camera(prim_path=cam_prim_path(nm, seg),
                         translation=np.array([0.0, 0.0, z]),
                         frequency=10, resolution=(CAM_W, CAM_H))
            cam.initialize()
            # USD 카메라는 로컬 -Z 를 보고 **로컬 +Y 가 화면 위, +X 가 오른쪽**이다.
            # 로봇 전방이 로컬 -Z 라 시선 방향은 항등으로 맞지만, 그것만으로는
            # 🚨 **화면이 왼쪽으로 90° 돌아간다.**
            #   배치가 PLACE = Ry(-90) 이라 로봇 로컬 X → 월드 +Z(위) 다.
            #   항등이면 화면 오른쪽(+X_cam)이 월드 위가 되고, 관 바닥(월드 -Z)이
            #   화면 **왼쪽**에 찍힌다. 실측으로 확인 — 바닥 결함이 (229,381),
            #   즉 중앙에서 왼쪽으로 411px 떨어진 자리에 나왔다.
            # → 광축을 유지한 채 롤만 준다. 원하는 카메라 축(로봇 로컬 기준):
            #     전방  X_cam = -Y,  Y_cam = +X,  Z_cam = +Z   → Rz(-90°)
            #     후방  X_cam = +Y,  Y_cam = +X,  Z_cam = -Z   → (1,1,0) 축 180°
            #   이러면 둘 다 **화면 위 = 월드 위**가 되어 바닥 결함이 화면 아래로 온다.
            # ※ 판정 수치는 이 롤과 무관하다 — wall_radius_map 이 화소를 카메라
            #   실제 월드 변환으로 역투영하므로 어떤 롤이어도 같은 반경이 나온다.
            _SQ = 0.70710678
            cam.set_local_pose(
                orientation=np.array([_SQ, 0.0, 0.0, -_SQ] if fwd
                                     else [0.0, _SQ, _SQ, 0.0]),
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
        print(f"[준비] 카메라 2대 활성 (RGB + Depth 어노테이터)"
              + (f" — 프레임 저장 → {OUT}" if SHOTS
                 else " — 저장은 --shots 일 때만"))
    except Exception as exc:
        print(f"[경고] 카메라 초기화 실패 — 검출·판정 불가 ({exc})")
        rigs = []


# ── 검출 · 판정 (son 모듈을 그대로 쓴다) ─────────────────────────────
# 🚨 예전 이 시연은 **아크 2초가 지나면 무조건** 결함을 숨기고 비드를 띄웠다.
#    그건 CLAUDE.md 가 경고한 **자기충족 검증**이다 — 성공률이 항상 100% 다.
#    이제 두 군데를 실제 측정으로 막는다:
#      ① SWAP  : 토치 끝과 결함의 정렬 오차가 `align_tol_mm` 이내일 때만
#                결함을 없앤다 (임무 규칙 4, 설계 7.2)
#      ② VERIFY: 전방 카메라 **Depth 로 벽면 반경을 재서** 메워졌는지 본다
#                (`welder/weld.py` 의 ② 겹. 검출기 거짓 음성을 잡는 층)
#    ①이 없으면 용접이 빗나가도 성공이고, ②가 없으면 프림만 바꾼 것을
#    수리라고 우기게 된다.
sys.path.insert(0, str(SON))
from condition.detector import PipeConditionDetector      # noqa: E402
from welder.weld import WeldSequencer                     # noqa: E402
from welder.spark_fx import SparkFX                       # noqa: E402

# ── 아크 스파크 (시각 전용) ────────────────────────────────────────
# 🚨 **헤드리스에서는 만들지 않는다.** 렌더가 없어 보이지도 않는데 계산만 들고,
#    무엇보다 판정 경로(INSPECT/VERIFY 촬영)를 건드릴 이유가 없다.
#    물리에는 어느 모드에서도 관여하지 않는다 — 콜라이더 없는 PointInstancer 다.
# 🔑 매질을 넘긴다. 만관(수중 용접)이면 항력 45/s · 수명 0.20s 라 스패터가
#    몇 cm 못 가고 꺼진다. 공기 중이면 2.6/s · 0.50s 로 포물선을 그린다.
# 🚨 **매질 판정은 FLOODED 가 아니라 "입자 물이 실제로 있는가"(WATER) 다**
#    (2026-08-06 사용자 지적: "--fluid 와 --glass 의 스파크 정도가 다르다").
#    --fluid 는 해석적 힘 두 개뿐이고 **물 프림이 하나도 없다** — 화면에 물이
#    없는데 스패터만 수중 거동(발생률 340/s · 초속 1.1 · 비산 24mm)이라
#    --glass(기중 700/s · 3.2 · 1.2m)와 나란히 놓으면 꺼진 것처럼 보였다.
#    보이는 것과 거동을 일치시킨다. 물리·판정에는 어느 쪽도 영향이 없다.
#    SPARK_WET=1 로 수중 거동을 강제할 수 있다(설계 §4.4 수중 용접 검토용).
SPARK_WET = os.environ.get("SPARK_WET", "1" if WATER else "0") == "1"
sparks = None if (HEADLESS or NO_TORCH) else SparkFX(
    stage, "/World/weld_sparks", flooded=SPARK_WET)
if sparks is not None:
    print(f"[준비] 아크 스파크 — {'수중(급냉·단거리)' if SPARK_WET else '기중'} "
          f"조건, 시각 전용(물리·판정 무관)"
          + ("" if SPARK_WET == FLOODED else
             "   ※ 관은 만관이지만 보이는 물이 없어 기중 거동으로 그린다 "
             "(SPARK_WET=1 로 되돌림)"))

# ── 시각 물 층 (--fluid, 2026-08-06 사용자 요청) ─────────────────────
# 🔑 **유체압을 거는 이상 물은 있는 것이다 — 파티클이 없을 뿐이다**(사용자).
#    --fluid 는 항력·부력만 계산하고 물 프림이 0개라 화면에 아무것도 없었다.
#    수체·흐름 줄무늬·누수 물줄기를 **렌더 전용**으로 그린다. 콜라이더도
#    강체도 없어 물리·판정에 영향 0 이고, 유체력은 apply_fluid() 그대로다.
# 🔑 재질은 갈아끼울 수 있다 — WATER_MDL 로 Material 프림 경로를 주면
#    (vMaterials LIQUIDS 등) 그것을 쓰고, 없으면 로컬 반투명으로 간다.
#    ⚠ 이 PC 에 vMaterials 는 없다(*.mdl 0개, 실측). 원격에서 받아 스테이지에
#      올린 뒤 그 경로를 주는 방식이다.
# 🔑 **시연은 `--fluid` 로 한다** (2026-08-06 사용자 확정). 관은 일반관(불투명)
#    이고 시점을 관 안으로 넣어 본다. 그래서 물은 **수면 한 장**으로 그린다 —
#    속이 찬 수체로 만들면 로봇이 물에 잠겨 잘려 보이고, 관 안으로 들어간
#    카메라가 덩어리에 갇혀 화면이 통째로 파래진다(둘 다 실기 확인).
#    WATER_FX=0 으로 물 층만 끌 수 있다 — 화면에 이상한 것이 보일 때
#    "그게 물 층인가 아닌가" 를 한 번에 가르는 스위치다(단일 변수 대조).
waterfx = None
if FLUID and not HEADLESS and os.environ.get("WATER_FX") != "0":
    from pipe import water_fx                               # noqa: E402
    _wf_pts, _wf_tans = water_fx.elbow_centerline(
        S_IN, IN_Y, ARC_R, OUT_X, S_OUT)
    _v_out = water_fx.torricelli(W_LEVEL_Z, -PIPE_IR, G)
    waterfx = water_fx.WaterFX(
        stage, _wf_pts, _wf_tans,
        level=W_LEVEL_Z, radius=PIPE_IR - 0.002,   # 관벽 2mm 안쪽(z-파이팅 방지)
        flow_v=-V_FLOW_DESIGN,                     # 출구→입구 (역류 주행)
        hole_xyz=(DEFECT_X, IN_Y, -PIPE_IR), v_out=_v_out,
        material_path=os.environ.get("WATER_MDL"))
    _wf_fill = (lambda a: (a - math.sin(a) * math.cos(a)) / math.pi)(
        math.acos(max(-1.0, min(1.0, -W_LEVEL_Z / PIPE_IR))))
    print(f"[준비] 시각 물 층 — 중심선 {waterfx.total * 1000:.0f}mm, "
          f"수위 z{W_LEVEL_Z * 1000:+.0f}mm (단면적 {_wf_fill:.1%} 충수), "
          f"흐름 {V_FLOW_DESIGN} m/s, 낙수 {_v_out:.2f} m/s")
    _wf_mat = (waterfx.material_used or
               f"로컬 반투명 (opacity {water_fx.WATER_OPACITY})")
    print(f"       재질 {_wf_mat} — 수면 {waterfx.n_body_faces:,}면, "
          f"렌더 전용(콜라이더·강체 없음). 물리·유체력에는 영향 0")

# 🚨 **결함 프림의 원점은 관 축 위다.** `DEF_XF = trans(DEFECT_X, IN_Y, 0)` 이고
#    벽면 패치는 STL 안에 +Z 로 들어 있다(위 「결함 / 비드」 절). 그래서
#    `wpos(defect_mesh)` 를 결함 위치로 쓰면 **축 위 점**이 나와서 투영 화소도
#    프로파일도 통째로 틀린다(실측: 화면 중앙 근처 (686,377) 로 찍혀 벽 대신
#    로봇 몸통을 재고 "메워짐 −15mm" 라는 거짓 판정이 나왔다).
#    → 벽면 점을 기하로 직접 구한다. 시계각 규약은 DEF_XF 와 같다.
_TH_DEF = math.radians(DEFECT_CLOCK_DEG)
DEFECT_WORLD = np.array([DEFECT_X,
                         IN_Y + PIPE_IR * math.sin(_TH_DEF),
                         PIPE_IR * math.cos(_TH_DEF)])

INTR = {"fx": F_PX, "fy": F_PX, "ppx": CAM_W / 2.0, "ppy": CAM_H / 2.0,
        "f_fish": F_PX}
detector = PipeConditionDetector(
    INTR, params={"aperture_min_px": 400 * CAM_AREA_SCALE})
sequencer = WeldSequencer()
ALIGN_TOL_MM = float(sequencer.k["align_tol_mm"])
print(f"[준비] 검출·판정 활성 — 관상태 판정기 + 용접 3겹 검증 "
      f"(정렬 허용 {ALIGN_TOL_MM}mm, 파임 임계 "
      f"{sequencer.k['depth_defect_mm']}mm / 메움 "
      f"{sequencer.k['depth_repaired_mm']}mm, bore {sequencer.k['bore_r_mm']}mm)")


defect_geom = {}        # defect_pixel 이 채우는 진단 기하


def wall_radius_map(depth, centre_px, band_px):
    """창 안 화소를 3차원 점으로 되돌려 **관 중심선 기준 반경(mm)** 을 만든다.

    🚨 `welder/weld.py` 의 `radial_profile` 을 그대로 쓰면 안 된다. 그 식
    `D·sin θ` 는 **카메라가 관 축 위에 있다는 전제**의 값이라, 카메라 광축에서의
    거리를 준다. 이 로봇은 피스톤 6개가 균형 잡은 자리에 서므로 관 중심에서
    **4.8~5.0mm 편심**해 있고(검출기도 `offset_mm 4.99` 로 같은 값을 잰다),
    그 편심이 그대로 반경 오차가 된다. 우리가 재려는 결함 깊이는 0.9~1.6mm 라
    **오차가 신호의 3~5배**다 — 원리적으로 판정이 안 된다.
    실측(2026-08-05): 결함 위를 겨눴는데 창 최대반경이 49.0mm 로 관벽 50 보다
    작게 나왔다. 파임이 음수로 뒤집힌 것이다.

    → 어안 역투영으로 광선 방향을 구하고, 깊이를 곱해 카메라 로컬 점을 만든
      뒤 월드로 옮겨 **관 중심선까지의 거리**를 잰다. 편심이 상쇄된다.
      (실기에서 카메라 자세는 서스펜션 엔코더가 준다. 여기서는 시뮬 기하를 쓴다.)
    """
    cam_prim = stage.GetPrimAtPath(FRONT_CAM)
    if not cam_prim.IsValid():
        return np.zeros((0, 0))
    d = np.asarray(depth, dtype=np.float64)
    h, w = d.shape
    cx, cy = centre_px
    x0, x1 = int(max(0, cx - band_px)), int(min(w, cx + band_px))
    y0, y1 = int(max(0, cy - band_px)), int(min(h, cy + band_px))
    sub = d[y0:y1, x0:x1]
    if sub.size == 0:
        return np.zeros((0, 0))

    yy, xx = np.mgrid[y0:y1, x0:x1]
    ux = xx - INTR["ppx"]
    uy = -(yy - INTR["ppy"])            # 화소 y 는 아래로, 카메라 +Y 는 위로
    r = np.hypot(ux, uy)
    theta = r / INTR["f_fish"]          # 등거리 어안 r = f·θ
    with np.errstate(invalid="ignore", divide="ignore"):
        sx = np.where(r > 1e-9, ux / r, 0.0) * np.sin(theta)
        sy = np.where(r > 1e-9, uy / r, 0.0) * np.sin(theta)
    sz = -np.cos(theta)                 # USD 카메라는 로컬 -Z 를 본다
    pts = np.stack([sx * sub, sy * sub, sz * sub], axis=-1)

    _XC.Clear()
    m = _XC.GetLocalToWorldTransform(cam_prim)
    M = np.array([[m[i][j] for j in range(4)] for i in range(4)], dtype=float)
    flat = pts.reshape(-1, 3)
    ones = np.ones((flat.shape[0], 1))
    world = (np.hstack([flat, ones]) @ M)[:, :3].reshape(pts.shape)

    # 입구 직관 축은 (y=IN_Y, z=0) 을 지나는 X 축이다.
    radius_mm = np.hypot(world[..., 1] - IN_Y, world[..., 2]) * 1000.0
    return np.where(np.isfinite(radius_mm) & (sub > 0), radius_mm, np.nan)


def wall_verdict(depth, centre_px):
    """관 중심선 기준 반경으로 파임/메움을 판정한다.

    임계·창 크기는 `welder/config/weld.yaml`(= `sequencer.k`)을 그대로 쓴다.
    분위수를 쓰지 않고 중앙값 필터 + 최댓값을 보는 것도 그쪽 규칙 그대로다.
    """
    from scipy.ndimage import median_filter
    k = sequencer.k
    b = int(k["band_px"])
    rad = wall_radius_map(depth, centre_px, b)
    n_valid = int(np.isfinite(rad).sum())
    if n_valid < k["profile_min_px"]:
        return None, 0.0, f"프로파일 표본 부족 ({n_valid})"
    bore = float(k["bore_r_mm"])
    filled = np.where(np.isfinite(rad), rad, bore)
    sm = median_filter(filled, size=int(k["median_px"]))
    peak = float(sm.max() - bore)
    deep = int((sm - bore >= k["depth_defect_mm"] * 0.5).sum())
    if peak >= k["depth_defect_mm"] and deep >= k["min_deep_px"]:
        return False, peak, f"여전히 파임 {peak:+.2f} mm ({deep}px)"
    if peak <= k["depth_repaired_mm"]:
        return True, peak, f"메워짐 {peak:+.2f} mm"
    return None, peak, f"판정 애매 {peak:+.2f} mm ({deep}px)"


def wall_uv(p):
    """관벽을 펼친 좌표 (축방향 mm, 원주 호길이 mm) — 정렬 오차 전용.

    반경 성분을 일부러 버린다. 토치는 설계상 관벽에서 2mm 떨어져 있으므로
    반경까지 넣으면 그 간극이 오차로 둔갑해 허용치 1.5mm 를 늘 초과한다.
    기준 시계각을 결함에 맞춰 감아서 ±180° 경계에서 튀지 않게 한다.
    """
    x, y, z = float(p[0]), float(p[1]), float(p[2])
    clock = math.atan2(y - IN_Y, z)
    d = (clock - _TH_DEF + math.pi) % (2.0 * math.pi) - math.pi
    return np.array([x * 1000.0, PIPE_IR * 1000.0 * d, 0.0])


# ── 결함(구멍) 검출 — OpenCV 기하, 학습 모델 불필요 ────────────────────
# 🔑 **관 상태 판정(condition/, Depth)과 결함 검출은 다른 것이다.**
#    Depth 쪽은 단절·비틀어짐(DISCONNECTED/MISALIGNMENT)을 본다.
#    구멍·크랙은 영상에서 **원주 형태가 깨지는 것**으로 잡는 게 맞다.
# 🚨 YOLO 검출기는 이 PC 에서 못 돌린다 — 학습 가중치(`*.pt`)가 `.gitignore`
#    대상이라 레포에 없고 `ultralytics` 도 미설치다(실측). 그래서 학습이
#    필요 없는 OpenCV 기하 검사로 간다. cv2 는 Isaac 3.11 에 이미 있다(4.11.0).
# 원리: 관 벽은 로봇 조명이 정면으로 때려 밝다. 벽에 뚫린 구멍은 빛이 되돌아
#    오지 않아 **주변 밝은 벽에 둘러싸인 어두운 덩어리**가 된다. 관 저 끝
#    (전방 개구부)도 어둡지만 그건 화면 중앙에 걸리므로 **중앙을 포함하는
#    덩어리만 빼면** 남는 것이 벽면 결함이다.
# 🚨 밝기 임계를 고정값으로 박지 말 것 — 조명은 카메라 위치 종속이고 노출도
#    변한다(기록된 함정). 그 프레임의 분포에서 유도한다. 실측(저장 프레임)으로
#    5~90분위 사이 0.08~0.20 어디를 잡아도 같은 덩어리가 나왔다 → 0.12 채택.
HOLE_DARK_FRAC = 0.12
HOLE_MIN_PX = max(60, 300 * CAM_AREA_SCALE)   # 해상도 따라 같이 줄인다
# 이 비율을 넘는 어두운 덩어리는 벽면 구멍이 아니라 **관 저 끝**으로 본다.
HOLE_MAX_FRAC = float(os.environ.get("HOLE_MAX_FRAC", 0.10))
# 비드 색 검출 — 관 벽(회색)·관 저 끝(검정)은 채도 0 이라 안 걸린다.
BEAD_SAT_MIN = 60      # HSV 채도 하한 (0~255)
BEAD_VAL_MIN = 40      # 너무 어두우면 색이 무의미
BEAD_MIN_PX = max(20, 100 * CAM_AREA_SCALE)


def find_wall_hole(rgb, expect_px=None):
    """근접 벽면의 구멍을 찾는다 → dict 또는 None.

    🚨 **전방 개구부(관 저 끝)를 반드시 걸러야 한다.** 예전에는 "화면 중앙
    화소가 속한 덩어리" 하나만 뺐는데, 밝기 임계가 낮게 잡히면 중앙 화소가
    '어둡다' 에 안 걸려 라벨이 0(배경)이 되고 **아무것도 안 걸러졌다.**
    그러면 관 저 끝이 통째로 "구멍" 으로 잡힌다 — GUI 실측에서 화면의 56%
    (130,162px, 중심이 정확히 화면 한가운데)가 결함으로 보고됐다.
    → ① 중앙 원판에 걸치는 라벨은 **격자로 훑어** 전부 제외
      ② 화면의 25% 를 넘는 덩어리는 벽면 구멍일 수 없으므로 제외
    """
    # 🚨 **파란 화소를 물로 보고 빼면 안 된다** (2026-08-06 실기로 확인).
    #    결함은 관 **바닥(180°)** 이고 수위는 그 위다 — 즉 **결함이 물에 잠겨
    #    있어** 카메라는 수면 너머로 본다. 그래서 결함 화소도 파랗게 물들고,
    #    "파란 건 물" 로 지우면 **찾아야 할 결함이 같이 지워진다.** 실측:
    #    로봇이 결함을 61mm 지나칠 때까지 못 보고 `결함이 화각 밖` 으로 끝났다.
    #    물을 지우려던 필터가 결함을 지운 것이다. → 회색조 판정 그대로 간다.
    g = cv2.cvtColor(np.asarray(rgb)[:, :, :3].astype(np.uint8),
                     cv2.COLOR_RGB2GRAY)
    g = cv2.medianBlur(g, 5)
    h, w = g.shape
    lo, hi = np.percentile(g, 5), np.percentile(g, 90)
    thr = lo + HOLE_DARK_FRAC * (hi - lo)
    dark = (g < thr).astype(np.uint8)
    n, lab, st, ce = cv2.connectedComponentsWithStats(dark, 8)

    bore = set()
    rr = max(4, int(0.10 * min(h, w)))
    ys = range(max(0, h // 2 - rr), min(h, h // 2 + rr), 3)
    xs = range(max(0, w // 2 - rr), min(w, w // 2 + rr), 3)
    for yy in ys:
        for xx in xs:
            if lab[yy, xx]:
                bore.add(int(lab[yy, xx]))
    # 🚨 **중앙 제외만으로는 관 저 끝을 못 거른다** (2026-08-06, 저장된 프레임
    #    으로 확인). 로봇이 곡관에 가까워지면 관 저 끝이 화면 **왼쪽으로 밀려**
    #    중앙 원판 검사를 비껴간다. 그러면 저 끝이 통째로 "구멍" 으로 잡힌다 —
    #    실측 48,827px(화면의 **21.2%**)이 상한 25% 를 통과해 목표가 66mm
    #    틀어졌다. 결함은 촬영 거리에서 4,264px(**1.9%**)라 크기로 확실히
    #    갈린다. 상한을 10% 로 조인다.
    #    ⚠ 상한을 더 낮추면 가까이서 크게 보이는 결함까지 걸러진다 —
    #      APPROACH 재측정이 결함을 계속 봐야 하므로 여유를 남긴 값이다.
    big = HOLE_MAX_FRAC * h * w

    best = None
    for i in range(1, n):
        if i in bore:
            continue
        area = int(st[i, cv2.CC_STAT_AREA])
        if area < HOLE_MIN_PX or area > big:
            continue
        if best is None or area > best["area_px"]:
            best = {"area_px": area, "cx": float(ce[i][0]),
                    "cy": float(ce[i][1]), "thr": float(thr),
                    "r_eq_px": math.sqrt(area / math.pi)}
    if best and expect_px is not None:
        d = math.hypot(best["cx"] - expect_px[0], best["cy"] - expect_px[1])
        best["dist_px"] = d
        best["matched"] = d <= max(2.0 * best["r_eq_px"], 0.06 * w)
    return best


def find_weld_bead(rgb, expect_px=None):
    """용접 비드를 **색**으로 찾는다 → dict 또는 None.

    🔑 어둠으로 어둠을 가르지 않는다. 관 벽은 회색, 관 저 끝은 검정 —
    **둘 다 채도가 0** 이다. 비드만 주황(채도 높음)이므로 HSV 의 S 만 보면
    전방 개구부와 절대 헷갈리지 않는다. 밝기 임계에 흔들리지도 않는다.
    (실물 용접 비드도 산화막 때문에 모재와 색이 다르다 — 억지 표현이 아니다.)
    """
    hsv = cv2.cvtColor(np.asarray(rgb)[:, :, :3].astype(np.uint8),
                       cv2.COLOR_RGB2HSV)
    sat, val = hsv[:, :, 1], hsv[:, :, 2]
    mask = ((sat > BEAD_SAT_MIN) & (val > BEAD_VAL_MIN)).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, st, ce = cv2.connectedComponentsWithStats(mask, 8)
    best = None
    for i in range(1, n):
        area = int(st[i, cv2.CC_STAT_AREA])
        if area < BEAD_MIN_PX:
            continue
        if best is None or area > best["area_px"]:
            best = {"area_px": area, "cx": float(ce[i][0]),
                    "cy": float(ce[i][1]),
                    "r_eq_px": math.sqrt(area / math.pi)}
    if best and expect_px is not None:
        d = math.hypot(best["cx"] - expect_px[0], best["cy"] - expect_px[1])
        best["dist_px"] = d
        # 작은 덩어리는 등가반지름이 작아 반경 배수만으로는 너무 빡빡하다
        best["matched"] = d <= max(3.0 * best["r_eq_px"],
                                   0.08 * hsv.shape[1])
    return best


def front_frames(warm=200):
    """전방 카메라 Depth(m) 배열. 없으면 None.

    🚨 헤드리스는 메인 루프가 `world.step(render=False)` 로 도므로 렌더가
    아예 안 일어나고 **어노테이터가 빈다**. 검출·판정은 카메라가 근거이므로
    판정하는 순간에만 렌더를 몇 프레임 강제로 굽는다. 상시 렌더가 아니라
    INSPECT·VERIFY 두 지점뿐이라 연산 부담은 무시할 만하다.
    (여러 프레임을 도는 이유: 첫 프레임은 파이프라인이 안 차서 비어 나온다.)
    """
    for nm, ann, dep in rigs:
        if nm != "front_camera":
            continue
        # 🚨 **어노테이터는 마지막으로 렌더된 프레임을 그대로 들고 있다.**
        #    "데이터가 있으면 그만" 으로 짜면 두 번째 호출부터는 렌더를 한 번도
        #    안 굽고 **직전 판정 때의 낡은 영상**을 다시 읽는다.
        #    실측으로 걸렸다: 로봇이 120mm 후진했는데도 INSPECT 와 VERIFY 가
        #    화소 단위로 똑같은 영상을 봤고(구멍 면적 21,002px → 21,002px),
        #    그 바람에 메꿈을 4가지로 바꿔도 판정이 하나도 안 변했다.
        #    → 조건 없이 **먼저 렌더를 굽고 나서** 읽는다.
        # 🔑 **판정 프레임에서는 시각 물 층을 뺀다** (2026-08-06 실측).
        #    결함은 관 바닥이고 수위는 그 위라 결함이 물에 잠겨 있다 —
        #    수면 너머로 보면 구멍이 덜 어둡게 잡혀 검출 거리가 무너진다
        #    (정지 지점 결함 81mm 앞 → 33mm 앞, 그 거리에선 관벽이 원리적으로
        #    시야 밖이라 "벽면이 이어져 있다" 가 나온다). 색으로 물만 골라낼
        #    수는 없다 — 잠긴 결함도 같이 파랗기 때문이다(실기로 확인).
        #    사람이 보는 화면에는 그대로 두고, **카메라가 굽는 프레임에서만**
        #    감춘다. 여기가 렌더를 직접 굽는 유일한 지점이라 이것으로 충분하다.
        # 🚨 **읽기까지 감춘 채로 끝내야 한다.** 렌더만 감추고 `get_data()` 를
        #    다시 보이게 한 뒤에 부르면, 그 사이 뷰포트가 한 번 더 그리면서
        #    **물이 든 프레임**을 어노테이터에 얹는다. 실측으로 걸렸다 —
        #    같은 자리에서 밝기임계가 50(물 없음) 대 **27**(감췄다고 믿은
        #    쪽)로 반토막 났고, 검출이 화면의 22%(51,708px)짜리 어두운
        #    덩어리를 물었다. 읽기를 try 안으로 넣는다.
        #    --known-defect 시연에서는 검출을 안 쓰므로 감출 이유가 없다 →
        #    물이 깜빡이지 않는다.
        _wfx = None if KNOWN_DEFECT else globals().get("waterfx")
        if _wfx is not None:
            _wfx.set_visible(False)
        try:
            for _ in range(warm):
                world.step(render=True)
            c, z = ann.get_data(), dep.get_data()
        finally:
            if _wfx is not None:
                _wfx.set_visible(True)
        rgb = (np.asarray(c)[:, :, :3].astype(np.uint8)
               if c is not None and getattr(c, "size", 0) else None)
        depth = (np.asarray(z, dtype=np.float32)
                 if z is not None and getattr(z, "size", 0) else None)
        return rgb, depth
    return None, None


def defect_pixel():
    """결함 중심을 **전방 카메라 어안 화소**로 투영한다.

    등거리 어안이므로 r = f·θ 다(핀홀 tan 식을 쓰면 가장자리가 크게 틀어진다).
    USD 카메라는 자기 로컬 **-Z** 를 보고, 이 로봇 전방도 로컬 -Z 라
    전방 카메라는 회전이 항등이다(위 CAM_SPECS 주석과 같은 규약).
    """
    cam_prim = stage.GetPrimAtPath(FRONT_CAM)
    if not cam_prim.IsValid():
        return None
    _XC.Clear()
    m = _XC.GetLocalToWorldTransform(cam_prim)
    p_cam = m.GetInverse().Transform(Gf.Vec3d(*DEFECT_WORLD))
    x, y, z = float(p_cam[0]), float(p_cam[1]), float(p_cam[2])
    if -z <= 1e-9:                      # 카메라 뒤 → 안 보인다
        return None
    theta = math.atan2(math.hypot(x, y), -z)
    r_px = INTR["f_fish"] * theta
    rho = math.hypot(x, y)
    if rho < 1e-9:
        return INTR["ppx"], INTR["ppy"]
    # USD 카메라 화면: +X 오른쪽, +Y 위. 화소 y 는 아래로 증가한다.
    px = (INTR["ppx"] + r_px * x / rho, INTR["ppy"] - r_px * y / rho)
    # 진단용 기하 — 투영이 맞는지는 이 숫자들로만 가릴 수 있다.
    defect_geom.update(fwd_mm=-z * 1000.0, lat_mm=rho * 1000.0,
                       theta_deg=math.degrees(theta), r_px=r_px,
                       local=(x * 1000.0, y * 1000.0, z * 1000.0))
    return px


def hole_target(hole):
    """검출된 구멍 화소 → **결함의 월드 위치** → (축방향 x[m], 시계각[deg]).

    🔑 **이것이 "카메라로 찾아간다" 의 실체다.** 여기서 나온 값만 주행·정렬에
       쓰고, 정답 좌표(`DEFECT_X`·`DEFECT_CLOCK_DEG`)는 **판정에만** 남긴다.

    🚨 **깊이를 쓰지 않는다.** 두 가지 이유다.
       ① 구멍 자리는 실제로 관통이라 그 화소의 깊이는 관 너머로 빠진다(inf).
          구멍 한가운데를 깊이로 재면 결함이 아니라 그 뒤 허공을 잰다.
       ② 기록된 대로 이 조건의 Depth 역투영은 **±5mm 흩어진다** — 반경 50mm
          에서 5mm 면 시계각 **5.7°** 오차이고, 용접 정렬 허용치 1.5mm
          (원주 1.7°)를 통째로 넘는다.
    → 대신 어안 역투영으로 **광선**만 만들고 **관 내벽 원기둥과 해석적으로
      교차**시킨다. 관 내반경은 설계값(DN100)이고 카메라 자세는 실기에서
      서스펜션 엔코더가 준다. 깊이 잡음이 원리적으로 안 들어온다.
    """
    cam_prim = stage.GetPrimAtPath(FRONT_CAM)
    if hole is None or not cam_prim.IsValid():
        return None
    _XC.Clear()
    m = _XC.GetLocalToWorldTransform(cam_prim)
    M = np.array([[m[i][j] for j in range(4)] for i in range(4)], dtype=float)
    C = M[3, :3]
    ux = hole["cx"] - INTR["ppx"]
    uy = -(hole["cy"] - INTR["ppy"])    # 화소 y 는 아래로, 카메라 +Y 는 위로
    rr = math.hypot(ux, uy)
    th = rr / INTR["f_fish"]            # 등거리 어안 r = f·θ
    if rr < 1e-9:
        d_cam = np.array([0.0, 0.0, -1.0])
    else:
        d_cam = np.array([ux / rr * math.sin(th), uy / rr * math.sin(th),
                          -math.cos(th)])
    d = d_cam @ M[:3, :3]
    d /= max(float(np.linalg.norm(d)), 1e-12)
    # 관 내벽 원기둥(축 = X, 중심선 y=IN_Y·z=0) 과의 교차. 나가는 쪽 근이 벽면.
    a = d[1] ** 2 + d[2] ** 2
    b = 2.0 * ((C[1] - IN_Y) * d[1] + C[2] * d[2])
    c = (C[1] - IN_Y) ** 2 + C[2] ** 2 - PIPE_IR ** 2
    disc = b * b - 4.0 * a * c
    if a < 1e-12 or disc < 0.0:
        return None
    t = (-b + math.sqrt(disc)) / (2.0 * a)
    if t <= 0.0:
        return None
    P = C + t * d
    return (float(P[0]), math.degrees(math.atan2(P[1] - IN_Y, P[2])),
            float(np.linalg.norm(P - C)) * 1000.0)


def scan_hole():
    """주행 중 훑기 — 정답을 **전혀** 안 쓰는 구멍 검출. 설계 카메라 10Hz.

    `inspect_defect` 와 달리 관상태·비드·기대화소를 안 본다. 순수하게
    "지금 화면에 벽면 구멍이 보이는가" 만 보고 정지 시점을 정한다.

    🚨 `front_frames()` 의 기본 워밍업 200프레임을 그대로 쓰면 안 된다 —
       주행 중 10Hz 로 부르는 자리라 렌더가 200배로 든다. GUI 는 메인 루프가
       이미 매 스텝 렌더하므로 2프레임이면 최신 영상이고, 헤드리스는 렌더가
       아예 없으므로 파이프라인을 채울 만큼만(8) 굽는다.
    """
    rgb, _ = front_frames(warm=SCAN_WARM)
    return None if rgb is None else find_wall_hole(rgb)


def inspect_defect(tag):
    """카메라로 실제로 본다 → (관상태, 구멍검출결과).

    **주 판정은 OpenCV 구멍 검출**이다(`find_wall_hole`). Depth 프로파일은
    설계 3겹의 ② 보조 겹이라 참고로만 찍는다 — 이 로봇 조건에서는 역투영이
    ±5mm 흩어져 0.9~1.6mm 깊이를 가릴 만한 정밀도가 안 나온다(실측).
    """
    rgb, z = front_frames()
    if rgb is None:
        print(f"  [검출] {tag}: 카메라 프레임 없음 — 판정 불가")
        return None, None, None
    # 관 상태(단절·비틀어짐)는 Depth 담당. 결함 검출과는 별개다.
    if z is not None:
        joint = math.degrees(float(np.max(np.abs(
            art.get_joint_positions()[bel_idx])))) if bel_idx else 0.0
        cond = detector.run(z, joint_angle_deg=joint)
        print(f"  [관상태] {tag} {cond.state} ({cond.speed})  "
              f"원형도 {cond.circularity:.3f}  거칠기 {cond.roughness:.4f}"
              f"  오프셋 {cond.offset_mm:.2f}mm")
    else:
        cond = None

    px = defect_pixel()
    if px is None:
        print(f"  [검출] {tag}: 결함이 전방 카메라 화각 밖")
        return cond, None, None
    g = defect_geom
    hole = find_wall_hole(rgb, expect_px=px)
    bead = find_weld_bead(rgb, expect_px=px)
    # 🔑 **정답과 대조하기 전의 원 검출**을 따로 남긴다. `matched` 는 판정용
    #    교차확인이고, 주행·정렬 목표는 정답을 안 본 이 값에서 나와야 한다.
    globals()["last_raw_hole"] = hole
    if hole is None:
        print(f"  [검출] {tag} 구멍 없음 — 벽면이 이어져 있다 "
              f"(예상 자리 {px[0]:.0f},{px[1]:.0f} / 카메라 {g['fwd_mm']:.0f}mm 앞)")
    else:
        print(f"  [검출] {tag} 어두운 덩어리 면적 {hole['area_px']:,}px "
              f"중심({hole['cx']:.0f},{hole['cy']:.0f}) 등가반지름 "
              f"{hole['r_eq_px']:.0f}px  예상자리와 {hole['dist_px']:.0f}px  "
              f"→ {'구멍(결함)' if hole['matched'] else '위치 안 맞음 — 무시'}"
              f"   [밝기임계 {hole['thr']:.0f}]")
        if not hole["matched"]:
            hole = None
    if bead is None:
        print(f"  [검출] {tag} 비드(주황) 없음")
    else:
        print(f"  [검출] {tag} 비드(주황) 면적 {bead['area_px']:,}px "
              f"중심({bead['cx']:.0f},{bead['cy']:.0f}) 예상자리와 "
              f"{bead['dist_px']:.0f}px → "
              f"{'결함 자리' if bead['matched'] else '엉뚱한 자리'}")
        if not bead["matched"]:
            bead = None
    return cond, hole, bead



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

    카메라는 항상 켜져 있고(검출·판정이 쓴다) **파일 저장만** `--shots` 로 가른다.
    """
    if not rigs or not SHOTS:
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
# ALIGN 축방향 미세 정렬 허용치(mm). 정렬 판정 임계(weld.yaml align_tol_mm 1.5)
# 보다 넉넉히 작아야 SWAP 이 통과한다. 아크 2초 동안의 흘러감까지 감안한 값.
ALIGN_AXIAL_TOL = 0.4
# ── ALIGN 미세 되물림 + 정지마찰 돌파 램프 (2026-08-06 실측 근거) ──────
# 설계 12%(ALIGN_DRIVE_FRAC)로 시작한다. 그 값으로 **실제로 움직이면 그대로**
# 두고, 0.125초 동안 팁이 ALIGN_MOVE_EPS 도 안 움직이면 배율을 한 단 올린다.
# 근거 — 건식 실측에서 12% 지령에 바퀴는 13~15deg/s 로 도는데 로봇은 1μm 도
# 안 나갔다(1,920스텝 = 8초 타임아웃 전체). 0.6 으로 올리면 즉시 수렴한다.
# 조건마다 필요한 값이 다르다(만관 μ0.30 은 12% 로도 가고, 건식 μ0.40·
# --fluid 항력 2.3N 은 못 간다) → 고정값이 아니라 **필요한 만큼만** 올린다.
ALIGN_DRIVE_FRAC = float(os.environ.get("ALIGN_DRIVE_FRAC", 0.12))  # 시작 배율
ALIGN_DRIVE_MAX = float(os.environ.get("ALIGN_DRIVE_MAX", 0.8))     # 상한
ALIGN_RAMP_STEP = 0.06        # 한 번에 올리는 양
ALIGN_RAMP_EVERY = 30         # 판정 주기(스텝) = 0.125초 @240Hz
ALIGN_MOVE_EPS = 0.02         # 이 주기에 이만큼(mm) 못 가면 "정체" 로 본다
INSPECT_BACK_MM = 120.0     # (참고용) 설계상 촬영 정지 거리
# 🔑 주행 중 카메라 훑기 — **설계 카메라 10Hz** 그대로. 주행 0.15 m/s 를 고른
#    근거가 원래 이것이다(0.30 m/s 면 프레임당 30mm 라 결함 38mm 를 1.3
#    프레임에만 잡아 놓친다). 정지 시점을 이 훑기가 정한다.
SCAN_EVERY = max(1, int(PHYSICS_HZ / 10))
SCAN_WARM = 8 if HEADLESS else 2         # scan_hole 워밍업 프레임 (위 주석 참조)
# 🚨 시각 물 층이 켜져 있으면 **스캔도 워밍업을 늘려야 한다.** front_frames 가
#    물을 감추고 렌더를 굽는데, 2프레임으로는 감춘 것이 화면에 반영되기 전에
#    읽어 버린다 — 그래서 INSPECT(200프레임)는 깨끗한데 스캔만 물이 낀 화면을
#    보고, 결함이 충분히 어둡게 안 잡혀 **정지가 늦어졌다**(결함 81mm 앞 →
#    33mm 앞). 대가는 물이 잠깐씩 깜빡이는 것이다.
if not HEADLESS and globals().get("waterfx") is not None:
    SCAN_WARM = int(os.environ.get("SCAN_WARM", 6))
ARC_STEPS = int(2.0 * PHYSICS_HZ)        # 아크 2초
COOL_STEPS = int(1.0 * PHYSICS_HZ)
REPOS_MM = 120.0

state = "SETTLE"
j2_cmd = 0.0        # J2 지령(m). EXTEND 가 팁 반경 되먹임으로 정하고 ARC 가 문다
j1_cmd = 0.0        # J1 지령(deg). ALIGN 이 팁 끝 시계각 되먹임으로 정한다
align_frac = ALIGN_DRIVE_FRAC   # ALIGN 되물림 배율(정체하면 램프가 올린다)
_align_ref_x = 0.0              # 램프 판정용 직전 팁 끝 x (m)
t_state = 0
inspected = False
repos_from = None
inspect_fwd_mm = 0.0
log = []
weld_ok = None      # VERIFY 가 카메라로 정한다. None = 아직 판정 전
pre_hole = None     # INSPECT 가 본 구멍. VERIFY 가 이것과 비교한다
last_raw_hole = None    # 정답 대조 **전**의 원 검출 (주행·정렬 목표의 출처)
# 🔑 카메라가 정한 결함 위치. **주행·정렬은 이 둘만 쓴다.**
#    정답(`DEFECT_X`/`DEFECT_CLOCK_DEG`)은 판정과 오차 보고에만 남는다.
det_x = None        # 축방향 위치 (m, 월드)
det_clock = None    # 시계각 (deg, +Z 에서 +Y 로. DEF_XF 와 같은 규약)
# APPROACH 가 더 정면인 관측으로 목표를 갱신할 때 쓰는 기준값
best_hole_px = 0    # 지금까지 본 가장 큰 구멍 면적 (px)
first_hole_px = 0   # 최초 검출 면적 — 갱신 효과를 로그로 보이려고 남긴다
first_det_x = None  # 최초 검출 축 위치


# ── Stop → Play 로 다시 돌리기 ──────────────────────────────────────
# 🔑 GUI 의 Stop 은 **Isaac 타임라인만** 세운다. 이 시연을 굴리는 것은 파이썬
#    FSM 변수(state·t_state·j2_cmd…)이고 그것들은 Stop 으로 안 돌아간다.
#    그래서 다시 Play 해도 아무도 지령을 안 줘서 로봇이 선 채로 있었다.
#    M0609 `4_pick_place.py:318-328` 이 `is_playing and not was_playing` 로
#    전이를 잡아 reset 을 부르는 것과 같은 패턴을 여기에 넣는다.
# 🚨 **world.reset() 만으로는 부족하다.** 이 시연에는 USD 를 직접 고쳐서 만든
#    상태가 있어서 손으로 되돌려야 한다 — 결함/비드 가시성, 아크 조명,
#    그리고 개구에 앉힌 키네마틱 마개. 안 되돌리면 2회차는 "이미 수리된 관"
#    에서 시작해 물이 안 새고 비드가 처음부터 보인다.
# ❓ 미검증: `world.reset()` 이 PBD 입자(--water) 위치까지 되돌리는지는 안
#    돌려봤다. 안 되돌린다면 2회차 물은 흩어진 상태에서 시작한다.
def restart_demo():
    global state, t_state, j2_cmd, j1_cmd, align_frac, _align_ref_x
    global inspected, repos_from, log, weld_ok
    global pre_hole, det_x, det_clock, last_raw_hole
    weld_ok, pre_hole = None, None
    det_x, det_clock, last_raw_hole = None, None, None
    globals().update(best_hole_px=0, first_hole_px=0, first_det_x=None)
    UsdGeom.Imageable(defect_mesh).MakeVisible()
    UsdGeom.Imageable(bead_mesh).MakeInvisible()
    _bead_op.Set(DEF_XF)
    UsdGeom.Imageable(_plug_prim).MakeInvisible()
    if not NO_TORCH:
        arc_light.GetIntensityAttr().Set(0.0)
    if sparks is not None:
        sparks.clear()
    _plug_coll.CreateCollisionEnabledAttr(False)
    world.reset()
    state, t_state, j2_cmd, j1_cmd = "SETTLE", 0, 0.0, 0.0
    align_frac, _align_ref_x = ALIGN_DRIVE_FRAC, 0.0
    inspected, repos_from, log = False, None, []
    globals()['inspect_fwd_mm'] = 0.0
    print("=" * 78)
    print("[재시작] Stop → Play 감지 — 원위치 복귀 후 처음부터 다시 돈다")
    print("-" * 78)


def report():
    """시퀀스 결과 요약. 재실행 모드에서는 회차마다 한 번씩 찍는다."""
    print("=" * 78)
    p = wpos(_seg1)
    vis_d = UsdGeom.Imageable(defect_mesh).ComputeVisibility()
    vis_b = UsdGeom.Imageable(bead_mesh).ComputeVisibility()
    print(f"결과  최종 상태 {state}  seg1 ({p[0] * 1000:.1f}, "
          f"{p[1] * 1000:.1f}, {p[2] * 1000:.1f})mm")
    print(f"      결함 프림 {vis_d} / 비드 프림 {vis_b}")
    if WATER:
        print(f"      물 입자 {n_particles:,}개, 유동 {_flow_now:.3f} m/s, "
              f"재투입 누계 {_recycled:,}, 현재 누수 {leak_count()}개")
    # 🚨 판정 근거는 **프림 가시성이 아니라 카메라 Depth 측정**이다.
    #    가시성으로 판정하면 우리가 바꾼 것을 우리가 확인하는 자기충족이 된다.
    if state != "DONE":
        print(f"판정  △ 시퀀스가 {state} 에서 끝났다")
    elif weld_ok is True:
        print("판정  ✅ 수리 성공 — 전방 카메라 영상에서 구멍이 사라졌다")
    elif weld_ok is False:
        print("판정  ❌ 수리 실패 — 영상에 구멍이 남아 있다")
    else:
        print("판정  △ 용접 검증을 못 했다 (프레임 없음/화각 밖)")
    print("[참고] 배관 충돌 메시에 실제 구멍은 뚫려 있지 않다 — "
          "CLAUDE.md 채택 방식(프림 가시성 전환)에 따른 시각 시연이다")
    sys.stdout.flush()

# 속도 계측 — 설정을 바꿔 가며 비교하려면 숫자가 있어야 한다.
import time                                              # noqa: E402
_t_mark, _step_mark = time.time(), 0
_f_drag = 0.0

print("-" * 78)
# GUI + --hold 면 시퀀스가 끝나도 창을 열어 두고 **Stop → Play 로 다시 돈다.**
# 헤드리스는 타임라인 조작이 없으므로 예전처럼 STEPS 만큼 돌고 끝난다.
REPLAY = HOLD and not HEADLESS
step, was_playing, reported = 0, True, False
while True:
    world.step(render=not HEADLESS)
    step += 1

    if not HEADLESS:
        # M0609 4_pick_place.py:318-328 과 같은 전이 감지
        _playing = world.is_playing()
        if _playing and not was_playing:
            restart_demo()
            step, reported = 0, False
            _t_mark, _step_mark = time.time(), 0
        was_playing = _playing
        if not simulation_app.is_running():
            break
        if not _playing:
            continue          # 정지 중에는 FSM 을 진행하지 않는다

    if state == "DONE":
        if not reported:
            drive(0.0)
            report()
            reported = True
            if REPLAY:
                print("GUI 실행 중 — Stop 눌렀다 Play 하면 처음부터 다시, "
                      "창을 닫으면 종료됩니다")
        if not REPLAY:
            break
        continue
    if step > STEPS and not REPLAY:
        break

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
            # 로그가 거짓말하지 않게 **실제** 거리를 찍는다. 로봇이 결함보다
            # INSPECT_BACK_MM 만큼 뒤에서 시작하지 못하면 이 단계는 제자리에서
            # 바로 발동한다(그때 명목값을 찍으면 없는 주행을 한 것처럼 보인다).
            _ring_gap = (DEFECT_X - wpos(_ring)[0]) * 1000.0
            print(f"[INSPECT] 촬영 정지 — 결함까지 링 기준 {_ring_gap:.1f}mm "
                  f"(목표 {INSPECT_BACK_MM:.0f}mm)")
            # 🚨 `snap` 을 검출보다 **먼저** 부르면 안 된다. 헤드리스는 판정
            #    순간에만 렌더를 굽는데(front_frames), 그 전에는 어노테이터가
            #    비어 있어 1_before 가 아예 저장되지 않는다(실측).
            _, pre_hole, _ = inspect_defect("수리 전")
            snap("1_before")
            # 🔑 **여기서 주행·정렬 목표가 정해진다 — 카메라로.**
            if KNOWN_DEFECT:
                # 정답 좌표를 그대로 목표로 쓴다. 카메라 앞 거리만 실제
                # 기하로 재서 REPOSITION 이 같은 거리로 되돌아갈 수 있게 한다.
                _XC.Clear()
                _cm = _XC.GetLocalToWorldTransform(
                    stage.GetPrimAtPath(FRONT_CAM))
                _cw = np.array([float(_cm[3][0]), float(_cm[3][1]),
                                float(_cm[3][2])])
                _tg = (DEFECT_X, DEFECT_CLOCK_DEG,
                       float(np.linalg.norm(DEFECT_WORLD - _cw)) * 1000.0)
                print("  ⚠ **정답 위치 사용(--known-defect)** — 카메라 검출을 "
                      "건너뛴다. VERIFY 의 '수리 확인' 은 이 모드에서 "
                      "자기충족이므로 성능 근거로 쓸 수 없다")
            else:
                _tg = hole_target(last_raw_hole)
            if _tg is None:
                # 🚨 정답으로 대신하지 않는다(CLAUDE.md: 감지 실패 시 정답
                #    fallback 금지). 못 찾았으면 못 고치는 것이고, 그대로 보고한다.
                print("  ⚠ 구멍을 못 찾았다 — 수리 목표를 세울 수 없다. "
                      "정답 좌표로 대신하지 않는다(임무 규칙 7: 인지·보고가 성공)")
                state, t_state = "RESUME", 0
            else:
                det_x, det_clock, inspect_fwd_mm = _tg
                # APPROACH 가 더 정면인 관측으로 갱신할 수 있게 기준을 남긴다.
                first_det_x, first_hole_px = det_x, (pre_hole or {}).get(
                    "area_px", 0)
                best_hole_px = first_hole_px
                _ex = (det_x - DEFECT_X) * 1000.0
                _ec = ((det_clock - DEFECT_CLOCK_DEG + 180.0) % 360.0) - 180.0
                print(f"  [목표] {'⚠ **정답** 결함 위치' if KNOWN_DEFECT else '**카메라가 정한** 결함 위치'} "
                      f"x={det_x * 1000:+.1f}mm 시계각 {det_clock:+.1f}° "
                      f"(카메라에서 {inspect_fwd_mm:.1f}mm)")
                print(f"         정답 대비 축 {_ex:+.2f}mm / 원주 {_ec:+.2f}° "
                      f"= {PIPE_IR * 1000 * math.radians(abs(_ec)):.2f}mm "
                      f"— 이 정답은 **판정에만** 쓴다")
                print(f"  [판정] 결함 확인 — 수리 대상 "
                      f"(면적 {pre_hole['area_px']:,}px)" if pre_hole else
                      "  ⚠ 정답 대조에서 걸러졌다 — 검출 조건을 의심할 것")
                state, t_state = "APPROACH", 0

    elif state == "APPROACH":
        drive(SPIN_DEG_S)
        set_torch(j1_deg=0.0, j2_m=0.0)
        if not inspected:
            # 🔑 **정답을 안 쓰는 정지 조건.** 카메라를 설계 10Hz 로 훑다가
            #    벽면 구멍이 보이면 선다. 예전에는 `ring[0] >= DEFECT_X -
            #    INSPECT_BACK_MM` 이었는데 그건 결함 위치를 미리 아는 것이다.
            if KNOWN_DEFECT:
                # 시연 모드 — 정답 위치를 알고 설계 촬영 거리에서 선다.
                if ring[0] >= DEFECT_X - INSPECT_BACK_MM * MM:
                    inspected = True
                    drive(0.0)
                    print(f"[탐지] ⚠ **정답 위치 사용(--known-defect)** — 링 "
                          f"x={ring[0] * 1000:.1f}mm 에서 정지 "
                          f"(결함 {INSPECT_BACK_MM:.0f}mm 앞). 카메라 검출 아님")
                    state, t_state = "INSPECT", 0
            elif t_state % SCAN_EVERY == 0 and scan_hole() is not None:
                inspected = True
                drive(0.0)
                print(f"[탐지] 벽면 구멍 발견 — 링 x={ring[0] * 1000:.1f}mm "
                      f"에서 정지. 위치는 카메라가 정한다")
                state, t_state = "INSPECT", 0
            elif path_s(*wpos(_seg1)[:2]) > S_IN + S_ARC + 0.25:
                print("[END_REACHED] 코스 끝까지 결함을 못 봤다 — 보고하고 끝낸다")
                state, t_state = "DONE", 0
        elif ring[0] >= det_x - STOP_TOL:
            drive(0.0)
            print(f"[APPROACH] 결함 도달 — 링 x={ring[0] * 1000:.1f}mm "
                  f"(카메라가 정한 목표 {det_x * 1000:+.1f}), {t_state} 스텝")
            if best_hole_px > 0:
                print(f"           목표는 **가장 크게 보였을 때**({best_hole_px:,}px) "
                      f"값이다 — 최초 검출({first_hole_px:,}px) 대비 축 "
                      f"{(det_x - first_det_x) * 1000:+.2f}mm 옮겼다")
            state, t_state = "ALIGN", 0
        # 🔑 **접근하면서 목표를 다시 잰다** (2026-08-06). 최초 검출은 결함을
        #    **비스듬히 스쳐 본** 것이라 덩어리 중심이 구멍 중심이 아니다 —
        #    실측 축방향 오차 4.07mm 였고, ALIGN 이 그 틀린 값을 0.35mm 까지
        #    충실히 따라가 SWAP 이 4.73mm 로 실패했다(제어가 아니라 입력 문제).
        #    가까워질수록 정면이 되고 같은 구멍이 크게 보이므로, **가장 크게
        #    보일 때**의 값을 쓴다. 정답은 여전히 안 본다 — 카메라로 잰 것들
        #    중에서 가장 정면인 하나를 고르는 것뿐이다.
        elif t_state % SCAN_EVERY == 0 and not KNOWN_DEFECT:
            _h = scan_hole()
            if _h is not None and _h["area_px"] > best_hole_px:
                _tg2 = hole_target(_h)
                # 관 축 방향으로 앞쪽(아직 안 지나친 것)만 받는다 — 지나친 뒤의
                # 검출은 뒤통수를 보는 것이라 기하가 성립하지 않는다.
                if _tg2 is not None and _tg2[0] > ring[0]:
                    best_hole_px = _h["area_px"]
                    det_x, det_clock, inspect_fwd_mm = _tg2
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
        # 🚨 J1(원주 회전)만 맞추면 **축방향이 안 맞는다.** APPROACH 는
        #    `STOP_TOL 3mm` 로 멈추는데 정렬 허용치는 1.5mm 다 — 게다가 지령을
        #    0 으로 줘도 관성으로 흘러가서 실측 오차가 1.8~2.1mm 로 남았다.
        #    설계는 거칠게 접근(`approach_tol_mm 5.0`) 한 뒤 **ALIGN 에서 미세
        #    조정**하는 구조다. 여기서 바퀴를 저속으로 되물려 축을 맞춘다.
        # 🔑 목표는 **카메라가 정한 값**(det_x·det_clock)이다. 정답 좌표는
        #    아래 SWAP 의 판정에만 쓴다 — 제어와 판정을 같은 출처로 두면
        #    자기충족이 된다(CLAUDE.md).
        # 🚨 **축방향도 판정과 같은 점(팁 끝)으로 잰다** (real_map 검증 이식).
        #    아크는 팁 **끝**에서 일어난다. ALIGN 은 링크 원점으로 닫고 SWAP 만
        #    다른 점으로 재면 판정이 구조적으로 어긋난다.
        _tw = tip_end_world()
        ax_mm = (det_x - _tw[0]) * 1000.0             # + 면 결함이 아직 앞
        # 🔑 **정지마찰 돌파 램프** (2026-08-06 실측). 설계 12% 지령으로는
        #    바퀴만 돌고 로봇이 **1μm도 안 나간다** — 8초 타임아웃을 통째로
        #    쓰고 축 오차 2.68mm 를 남긴 채 ARC 로 가서 용접이 빗나갔다.
        #    안 움직이는 것이 확인될 때만 지령을 올린다. 움직이기 시작하면
        #    그 값에서 멈추므로, 잘 움직이는 조건(--water)에서는 12% 그대로다.
        if t_state % ALIGN_RAMP_EVERY == 0:
            if abs(ax_mm) > ALIGN_AXIAL_TOL and \
                    abs(_tw[0] - _align_ref_x) * 1000.0 < ALIGN_MOVE_EPS:
                align_frac = min(ALIGN_DRIVE_MAX,
                                 align_frac + ALIGN_RAMP_STEP)
            _align_ref_x = _tw[0]
        if abs(ax_mm) > ALIGN_AXIAL_TOL:
            drive(math.copysign(SPIN_DEG_S * align_frac, ax_mm))
        else:
            drive(0.0)
        # 🚨 **J1 관절값으로 닫으면 안 된다** (real_map 검증 이식). 스프링
        #    드라이브라 지령과 실제가 벌어지고, 로봇이 관 중심에서 편심해
        #    있어 같은 J1 이어도 팁이 가리키는 **시계각**이 달라진다.
        #    실측: J1 오차 1.14° 인데 정렬 오차 2.45mm. → 측정한 팁 끝
        #    시계각으로 닫는다(P 제어). J2 와 같은 이유·같은 방식.
        clock_tip = math.degrees(math.atan2(_tw[1] - IN_Y, _tw[2]))
        err_deg = ((det_clock - clock_tip + 180.0) % 360.0) - 180.0
        if t_state == 1:
            j1_cmd = det_clock                        # 되먹임의 출발점
        j1_cmd = max(-TORCH["j1_limit_deg"],
                     min(TORCH["j1_limit_deg"], j1_cmd + J1_KP * err_deg))
        set_torch(j1_deg=j1_cmd, j2_m=0.0)
        _ok = abs(err_deg) <= J1_TOL_DEG and abs(ax_mm) <= ALIGN_AXIAL_TOL
        # 🚨 **정체를 로그로 잡는다.** "바퀴가 도는가 / 도는데 로봇이 나가는가"
        #    를 이 한 줄로 가른다. 램프 배율도 같이 찍어 돌파 여부를 본다.
        if t_state % 480 == 0:
            _v = np.asarray(art.get_joint_velocities(), dtype=float)
            _wv = np.degrees(np.mean([_v[k] for k in wheel_idx]))
            print(f"  [ALIGN] {t_state:5d}스텝  축 {ax_mm:+6.2f}mm  "
                  f"시계각 오차 {err_deg:+5.2f}°  휠각속도 {_wv:+7.1f}"
                  f"(지령 {SPIN_DEG_S * align_frac:.0f}, 배율 {align_frac:.2f})  "
                  f"팁끝x {_tw[0] * 1000:+7.2f}mm")
        if _ok and t_state > 0.5 * PHYSICS_HZ:
            print(f"[ALIGN] 팁 시계각 {clock_tip:+.2f}° (카메라 목표 "
                  f"{det_clock:+.1f}°, 오차 {err_deg:+.2f}° = "
                  f"{PIPE_IR * 1000 * math.radians(abs(err_deg)):.2f}mm), "
                  f"축방향 {ax_mm:+.2f}mm, J1 지령 {j1_cmd:+.2f}°, "
                  f"되물림 배율 {align_frac:.2f}")
            state, t_state = "EXTEND", 0
        elif t_state > 8 * PHYSICS_HZ:
            print(f"[ALIGN] ⚠ 시간 초과 — 시계각 오차 {err_deg:+.2f}°, 축방향 "
                  f"{ax_mm:+.2f}mm 남은 채로 진행")
            state, t_state = "EXTEND", 0

    elif state == "EXTEND":
        # 🚨 **여기서 축이 밀린다.** drive(0.0) 으로 두면 J2 가 벽을 밀며
        #    뻗는 반작용에 로봇이 뒤로 흘러, ALIGN 이 0.33mm 로 닫아 둔 축이
        #    ARC 끝에 1.71mm 로 벌어졌다(실측). ALIGN 과 같은 되물림을 유지한다.
        _ax = (det_x - tip_end_world()[0]) * 1000.0
        if abs(_ax) > ALIGN_AXIAL_TOL:
            drive(math.copysign(SPIN_DEG_S * align_frac, _ax))
        else:
            drive(0.0)
        # 측정한 팁 끝 반경으로 J2 를 닫는다(위 「용접 간극」 절 참조).
        # err > 0 = 너무 뻗었다 → J2 를 줄인다.
        _err = tip_r - TIP_TARGET_R
        j2_cmd = min(max(j2_cmd - J2_KP * _err * MM, 0.0), J2_STROKE)
        set_torch(j1_deg=j1_cmd, j2_m=j2_cmd)   # J1 은 ALIGN 이 닫아 둔 지령
        j2_now = float(art.get_joint_positions()[j2_idx])
        _hit_end = j2_cmd <= 1e-9 or j2_cmd >= J2_STROKE - 1e-9
        if (abs(_err) <= J2_TOL and t_state > 0.3 * PHYSICS_HZ) \
                or t_state > 5 * PHYSICS_HZ:
            _gap = 50 - tip_r
            print(f"[EXTEND] J2 = {j2_now * 1000:.2f}mm (지령 "
                  f"{j2_cmd * 1000:.2f})  팁 끝 반경 {tip_r:.2f}mm "
                  f"(관벽 50.0 → 용접 간극 {_gap:.2f}mm, 설계 2mm)")
            print(f"         링 편심 r{math.hypot(ring[1] - IN_Y, ring[2]) * 1000:.1f}mm"
                  f" — 고정 {J2_STROKE * 1000:.0f}mm 로 뻗었으면 간극이 어긋난다")
            if abs(_err) > J2_TOL:
                print(f"         ⚠ 목표 {TIP_TARGET_R:.1f}mm 에 {_err:+.2f}mm "
                      f"못 맞췄다"
                      + ("  (J2 스트로크 끝 — 편심이 스트로크보다 크다)"
                         if _hit_end else "  (시간 초과)"))
            state, t_state = "ARC", 0

    elif state == "ARC":
        # 🚨 **아크 동안 축을 계속 잡아 준다** (real_map 검증 이식). 지령을
        #    0 으로 줘도 2초 사이에 흘러가 정렬이 문턱(1.5mm)을 오르내렸다
        #    (실측 1.13 / 1.45 / 1.60mm). ALIGN 과 같은 되물림을 유지한다 —
        #    램프로 올려 둔 배율을 그대로 쓴다(여기서 새로 올리지는 않는다).
        _ax = (det_x - tip_end_world()[0]) * 1000.0
        if abs(_ax) > ALIGN_AXIAL_TOL:
            drive(math.copysign(SPIN_DEG_S * align_frac, _ax))
        else:
            drive(0.0)
        set_torch(j1_deg=j1_cmd, j2_m=j2_cmd)
        if sparks is None:
            arc_light.GetIntensityAttr().Set(3.0e5)   # 깜빡임은 SparkFX 담당
        if t_state == 1:
            print(f"[ARC] 아크 {ARC_STEPS / PHYSICS_HZ:.1f}초"
                  + ("" if sparks is None else " — 스패터 발생"))
        if t_state > ARC_STEPS:
            arc_light.GetIntensityAttr().Set(0.0)
            if sparks is not None:
                sparks.clear()
            # 🚨 **정렬 오차가 허용치 이내일 때만** 결함을 없앤다.
            #    무조건 없애면 용접이 빗나가도 성공이 되어 검증이 죽는다
            #    (임무 규칙 4 / 설계 7.2 / weld.yaml align_tol_mm).
            # 🚨 3차원 직선거리를 쓰면 안 된다 — 설계 용접 간극 2mm 가 그대로
            #    오차로 잡혀 허용치 1.5mm 를 항상 넘는다. 정렬 오차는 **토치가
            #    결함을 겨누고 있는가**, 즉 관벽을 펼친 평면에서의 어긋남이다.
            # 🚨 **링크 원점이 아니라 팁 끝으로 잰다** (real_map 검증 이식) —
            #    아크는 팁 끝에서 일어나고, 로봇이 편심해 있어 같은 로드 위의
            #    두 점이 관 축 기준 **다른 시계각**을 갖는다(실측: 원주 오차가
            #    링크 원점 1.43mm vs 팁 끝 0.44mm). ALIGN 은 팁 끝으로 닫는데
            #    SWAP 만 원점으로 재서 판정이 구조적으로 어긋나 있었다.
            _tipw = tip_end_world()
            _uv_tip, _uv_def = wall_uv(_tipw), wall_uv(DEFECT_WORLD)
            align_mm = sequencer.align_error(_uv_tip, _uv_def)
            # 성분을 남긴다 — 합만 보면 축방향인지 원주인지 못 가른다.
            print(f"  [정렬] 축방향 {_uv_tip[0] - _uv_def[0]:+.2f}mm  "
                  f"원주 {_uv_tip[1] - _uv_def[1]:+.2f}mm  합 {align_mm:.2f}mm "
                  f"(허용 {ALIGN_TOL_MM}mm)")
            aligned = align_mm <= ALIGN_TOL_MM
            # 비드는 **토치가 실제로 있던 축방향 자리**에 놓는다.
            _bead_dx = _tipw[0] - DEFECT_X
            _bead_op.Set(DEF_XF * trans(_bead_dx, 0.0, 0.0))
            # 🔑 수리 표현 = 프림 가시성 전환 (CLAUDE.md 채택 방식)
            UsdGeom.Imageable(bead_mesh).MakeVisible()
            if aligned:
                UsdGeom.Imageable(defect_mesh).MakeInvisible()
                _before = leak_count()
                # 물리로도 막는다 — 키네마틱 마개를 개구 자리로 이동
                _plug_coll.CreateCollisionEnabledAttr(True)
                # 메꿈이 **보여야** 카메라가 수리를 확인할 수 있다.
                UsdGeom.Imageable(_plug_prim).MakeVisible()
                print(f"  [메꿈] 삼각형 {len(_hole_tris)}개 켬 — 가시성 "
                      f"{UsdGeom.Imageable(_plug_prim).ComputeVisibility()}, "
                      f"프림 유효 {_plug_prim.IsValid()}")
                print(f"[SWAP] 정렬 오차 {align_mm:.2f}mm ≤ {ALIGN_TOL_MM}mm "
                      f"— 결함 → 비드 전환 + 마개 착좌 "
                      f"(직전 누수 입자 {_before}개)")
            else:
                print(f"[SWAP] ⚠ 정렬 오차 {align_mm:.2f}mm > {ALIGN_TOL_MM}mm "
                      f"— 결함을 남긴다. 비드만 어긋난 자리에 표시(설계 7.2)")
            state, t_state = "COOL", 0

    elif state == "COOL":
        drive(0.0)
        set_torch(j1_deg=j1_cmd, j2_m=0.0)
        if t_state > COOL_STEPS:
            j2_now = float(art.get_joint_positions()[j2_idx])
            print(f"[COOL/RETRACT] J2 = {j2_now * 1000:.2f}mm 수납")
            repos_from = ring[0]
            state, t_state = "REPOSITION", 0

    elif state == "REPOSITION":
        # 용접 지점은 제자리에서 안 보인다 → 후진해서 전방 카메라로 확인
        # 🔑 **촬영했던 거리까지 물러난다.** 고정 120mm 로 물러나면 카메라가
        #    결함에서 108mm 로 멀어지는데, 그 거리에서는 구멍의 어두운 영역이
        #    관 저 끝과 이어져 중앙 덩어리로 흡수돼 **구멍을 놓친다**
        #    (실측: 정렬 실패로 결함을 안 지웠는데도 "구멍 없음" 이 나왔다).
        #    수리 전·후를 같은 거리에서 봐야 면적 비교가 성립한다.
        drive(-SPIN_DEG_S)
        set_torch(j1_deg=0.0, j2_m=0.0)
        # 후진 거리도 **카메라가 정한 위치** 기준이다(정답 좌표 아님).
        _cam_fwd = (det_x - wpos(stage.GetPrimAtPath(FRONT_CAM))[0]) * 1000.0
        _target = inspect_fwd_mm if inspect_fwd_mm > 1.0 else REPOS_MM
        if _cam_fwd >= _target or repos_from - ring[0] >= 0.30:
            drive(0.0)
            print(f"[REPOSITION] 후진 {(repos_from - ring[0]) * 1000:.1f}mm "
                  f"— 카메라가 결함 {_cam_fwd:.1f}mm 앞 "
                  f"(촬영 때 {inspect_fwd_mm:.1f}mm 와 맞춤)")
            state, t_state = "VERIFY", 0

    elif state == "VERIFY":
        drive(0.0)
        if t_state > 1.0 * PHYSICS_HZ:
            # 🔑 **여기가 진짜 판정이다.** 아크가 끝났다고 성공이 아니라,
            #    후진해서 다시 본 Depth 로 벽면이 메워졌는지 measure 한다.
            #    (REPOSITION 이 앞에 있는 이유 — 제자리에서는 안 보인다.)
            _, post_hole, post_bead = inspect_defect("수리 후")
            snap("2_after")
            # 🔑 판정 = **수리 전에 있던 구멍이 사라졌는가.** 아크 시간이 아니라
            #    카메라가 본 것으로 정한다.
            if pre_hole is None:
                weld_ok = None
                _msg = "△ 수리 전 검출을 못 해 비교 불가"
            elif post_hole is None:
                # 구멍이 사라졌고 **비드가 그 자리에 보이면** 수리다.
                # 비드가 없으면 "구멍을 놓친 것" 일 수도 있으므로 단정 안 한다.
                weld_ok = True if post_bead else None
                _msg = ((f"✅ 수리 확인 — 구멍 사라짐 + 비드 확인 "
                         f"({post_bead['area_px']:,}px)") if post_bead else
                        "△ 구멍은 안 보이는데 비드도 안 보인다 — 놓쳤을 수 있다")
            else:
                _ratio = post_hole["area_px"] / max(pre_hole["area_px"], 1)
                weld_ok = _ratio < 0.15
                _msg = (("✅ 수리 확인 — 구멍이 " if weld_ok
                         else "❌ 미수리 — 구멍이 ")
                        + f"{pre_hole['area_px']:,}px → "
                          f"{post_hole['area_px']:,}px ({_ratio * 100:.0f}%)")
            print(f"[VERIFY] 판정 {_msg}"
                  + (f"  누수 입자 {leak_count()}개" if WATER else ""))
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

    # ── 아크 스패터 (시각 전용) ────────────────────────────────────
    # 🔑 **FSM 뒤에 둔다.** 아크 조명 깜빡임을 여기서 걸기 때문이다 — 앞에
    #    두면 ARC 분기의 `Set(3.0e5)` 가 매 스텝 덮어써서 깜빡임이 안 보인다.
    #    ARC 를 빠져나온 스텝에는 `state` 가 이미 COOL 이라 발생도 즉시 멎는다.
    # 🚨 물리·판정과 무관하다. 콜라이더 없는 PointInstancer 이고, VERIFY 는
    #    COOL(5초 이상) 뒤라 수명 0.42s 짜리 스패터가 화면에 남을 수 없다.
    if sparks is not None:
        _arc_on = state == "ARC"
        _spo = tip_end_world() if _arc_on else None
        sparks.step(1.0 / PHYSICS_HZ, emitting=_arc_on, origin=_spo,
                    normal=wall_inward(_spo) if _arc_on else None,
                    light=arc_light if _arc_on else None,
                    # 관 안에 가둔다 — 코스 중심선을 그대로 쓰므로 곡관에서도
                    # 정확하다(결함이 곡관 입구 22mm 앞이라 직선 근사는 못 쓴다).
                    confine=spark_confine if _arc_on else None)

    # 시각 물 층 — 흐름 줄무늬를 흘리고, **마개가 착좌하면 누수를 멈춘다**.
    # 마개는 SWAP 에서 정렬이 허용치 안일 때만 켜지므로, 물이 멎는 것 자체가
    # "제대로 막았다" 의 표시가 된다(용접 이벤트만으로 멎지 않는다).
    if waterfx is not None:
        waterfx.step(1.0 / PHYSICS_HZ,
                     leaking=not _plug_coll.GetCollisionEnabledAttr().Get())

    # state == "DONE" 은 루프 맨 위에서 처리한다(재실행 대기 때문에 여기서
    # break 하면 안 된다). 그래서 이 자리에 DONE 분기가 없다.

if not reported:
    report()

import threading                                          # noqa: E402

threading.Thread(target=simulation_app.close, daemon=True).start()
threading.Event().wait(8.0)
sys.stdout.flush()
os._exit(0)
