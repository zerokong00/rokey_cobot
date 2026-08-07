#!/usr/bin/env python3
"""맵·로봇 **주행 연습장** — 벨로우즈 12륜 + CAD 배관 3종.  [Isaac 3.11]

`repair_demo.py` 의 주행 부분만 떼어 온 **연습 파일**이다.

🎯 **기준 로봇 = 용접기 내장 v2** (`robot_v2/robot_from_bot_welder_art_v2.usda`,
   2026-08-07 사용자 지시). 벨로우즈 12륜(`robot_v2_12wheel.usda`)으로도 돌지만
   `ROBOT_USD=...` 로 명시해야 한다. **로직은 하나이고 설정값만 자산별**이다
   (`TUNING`) — 조인트는 이름이 아니라 **구조**로 찾으므로(`discover()`)
   자산을 갈아끼워도 같은 코드가 그대로 돈다.

아래 둘이 없다:

  ❌ **물이 없다** — `--water`(PBD 입자)도 `--fluid`(해석 항력)도, 시각 물 층도
     통째로 뺐다. 관 상태는 **배수(건식)** 고정이라 마찰은 0.40/0.35 다.
  ❌ **결함이 없다** — 관통 개구·비드·마개·용접 시퀀스·카메라 검출이 전부 없다.
     여기는 고치는 곳이 아니라 **굴려 보는 곳**이다.

배관 4종을 **관 5개 · 로봇 5기**로 깐다(2026-08-07 사용자 지시) — T 만
정방향·역방향을 따로 굴린다. 곡관은 CAD 원본 하나를 두 자세로 쓴다:

  ┌ elbow_v     fixed_pipe.stp                   — **수직 곡관**. 가다 위로 꺾인다
  ├ elbow_h     fixed_pipe.stp                   — **수평 곡관**. 같은 관을 굴렸다
  ├ tee_s1      tee_sweep_..._oneside.stp        — **T 분기 · 본관 → 가지**
  ├ tee_s1_rev  (같은 관, 중심선 반전)           — **T 분기 · 가지 → 본관**
  └ reducer     reducer_dn100_dn150_dn100.stp    — **관경 변화** DN100→150→100

**기본은 5대 동시**다 — 코스마다 로봇이 한 대씩 들어가 각자 제 관을
**왕복 반복**한다(끝 → 복귀 → 다시). 끼이면 방향을 뒤집어 빠져나온다.
한 대만 보고 싶으면 `--course elbow_v` 처럼 고른다. 이때 **그 관만 올라온다.**
비교용 판은 `--course tee_s`(양방향 블렌드) / `tee_s2`(`_narrow`) 로 부른다.

실행:
  isaac_python map_test_demo.py --headless                 # 로그로 확인
  DISPLAY=:1 isaac_python map_test_demo.py --hold          # GUI, 창 닫을 때까지
  DISPLAY=:1 isaac_python map_test_demo.py --course tee --hold
  isaac_python map_test_demo.py --course elbow_h --headless

옵션
  --course {all,elbow_v,elbow_h,tee_s1,tee_s1_rev,reducer,tee_s,tee_s2}  기본 all = **5대**
  --cam          카메라 6대를 만든다 (기본 꺼짐 — GUI 가 크게 느려진다)
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
import time
from pathlib import Path

# 기동 시간 계측 — "실행하고 나서 왜 한참 기다리나" 를 구간별로 가른다.
_T0 = time.time()


def tick(what):
    print(f"[기동 {time.time() - _T0:6.1f}s] {what}")
    sys.stdout.flush()

HEADLESS = "--headless" in sys.argv
HOLD = "--hold" in sys.argv
GLASS = "--glass" in sys.argv
# 🔁 코스를 **반대 방향으로** 통과시킨다 (분기 복귀 시험).
REVERSE = "--reverse" in sys.argv
# 🚨 **카메라는 기본으로 안 만든다** (2026-08-07). 이 연습장은 영상을 한 번도
#    읽지 않는다 — `Camera` 를 만들고 설정만 하고 `get_rgba` 조차 안 부른다.
#    그런데 로봇당 2대 × 640×360 렌더 프로덕트 + 리그 조명 2개가 GUI 에서
#    매 프레임 값을 치른다. 🔑 어제 벨로우즈로 돌릴 때 가벼웠던 이유가
#    이것이다 — 그 자산엔 `FrontBody`/`RearBody` 가 없어 **카메라가 0대로
#    조용히 건너뛰어졌다.** 자산을 바꾸니 6대가 생겼다.
#    카메라 시야를 봐야 할 때만 `--cam` 으로 켠다.
CAMERAS = ("--cam" in sys.argv
            or os.environ.get("NAV", "onboard") == "vision")
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
          "--steps", "--reverse", "--cam"}
_bad = [a for i, a in enumerate(sys.argv[1:], 1)
        if a.startswith("--") and a not in _KNOWN]
if _bad:
    raise SystemExit(f"[중단] 모르는 인자 {_bad} — 쓸 수 있는 것: "
                     f"{sorted(_KNOWN)}")

from isaacsim import SimulationApp                        # noqa: E402

simulation_app = SimulationApp({"headless": HEADLESS})
tick("SimulationApp 기동 완료")

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
# 🎯 **2026-08-07 사용자 지시 — 기준 로봇은 용접기 내장 v2 다.**
#    벨로우즈 12륜(`robot_v2_12wheel.usda`)으로 돌던 것을 갈아탄다. 로직은
#    똑같이 가고 **설정값만 자산별로** 다르다(아래 TUNING).
ROBOT_USDA = os.environ.get(
    "ROBOT_USD", str(SON / "robot_v2" / "robot_from_bot_welder_art_v2.usda"))
ROBOT_PRIM_IN_USDA = os.environ.get("ROBOT_PRIM", "")
MAPS = SON / "maps"
ASSET = Path(ROBOT_USDA).stem

# 🔑 **자산별 설정값** — 로직은 하나, 숫자만 다르다.
#    두 로봇의 다리·휠 규격이 달라 같은 숫자를 쓰면 뜻이 달라진다:
#      벨로우즈  휠 r10 · 다리 예압 9N  · 휠 중심 33.25+스트로크 · 질량 0.91kg
#      용접기v2  휠 r8  · 다리 예압 18N · 휠 중심 40.0+스트로크  · 질량 2.04kg
#    🚨 **다리 개수가 다르다** (실측): 벨로우즈는 다리 6 · 다리당 휠 2,
#       용접기 v2 는 **다리 12** · 다리당 휠 1 이다. 그래서 예압을 다리 하나
#       기준으로 같게 두면 총 수직항력이 두 배가 된다.
#    🔑 **견인/마찰 비를 맞춘 값**이라 주행 거동이 대등하다:
#      벨로우즈 견인 12×0.05/0.010 = 60N  / 마찰 0.4× 6×9 =  21.6N → 2.8배
#      용접기v2 견인 12×0.08/0.008 = 120N / 마찰 0.4×12×9 =  43.2N → 2.8배
#    🚨 자산 기본 maxForce 60N 을 그대로 쓰면 안 된다. 자산이 60 인 것은
#       한계(6mm)에서 3000×0.006 = 18N 으로 **어차피 잘리기** 때문인데,
#       스트로크를 35mm 로 늘리는 순간 그 상한이 진짜로 걸린다 →
#       12다리×18N 이면 마찰 86.4N 대 견인 120N 으로 여유가 1.39배뿐이다.
TUNING = {
    "robot_from_bot_welder_art_v2": dict(
        wheel_r=0.008, wheel_maxf=0.08,
        piston_stroke=0.035, piston_init=0.002, piston_maxf=9.0,
        piston_retract=0.015,        # 자산 −4mm → −15mm (T 통과에 필요)
        center_delta=0.003,          # Δ×3000 = 예압 9N
        bel_stiff=60.0, bel_maxf=15.0,     # 실측 스윕 (위 표)
        wheel_center_mm=40.0,
        # 🔑 D6 관절이라 **조향이 된다** — 축이 rotY(피치)·rotZ(요) 둘 다
        #    자유이고 각 ±20°. DOF 이름은 `J0:1`·`J0:2` 로 오는데, PhysX 가
        #    D6 를 twist(rotX)·swing1(rotY)·swing2(rotZ) 순으로 펼치고
        #    rotX 는 잠겨 있으므로 **`:1`=피치 / `:2`=요** 다.
        steer=("1", "2")),
    "robot_v2_12wheel": dict(
        wheel_r=0.010, wheel_maxf=0.05,
        piston_stroke=0.035, piston_init=0.00675, piston_maxf=9.0,
        center_delta=0.003,          # Δ×3000 = 예압 9N
        bel_stiff=1.0, bel_maxf=12.0,
        wheel_center_mm=33.25,
        # 벨로우즈는 관절이 revolute 이고 축이 X·Y 로 **교대**한다 — 한 관절이
        # 한 축뿐이라 아래 조향식(피치/요를 관절마다 같이 준다)이 안 맞는다.
        # 넣으려면 축별로 나눠 배분해야 한다. 지금은 끈다.
        steer=None),
}
if ASSET not in TUNING:
    raise SystemExit(
        f"[중단] 모르는 로봇 자산 {ASSET!r} — TUNING 에 설정값을 먼저 적을 것 "
        f"(있는 것: {list(TUNING)})")
TUNE = TUNING[ASSET]

# 🔑 용접기는 **로봇 usda 안에 이미 있다** — `welder/torch_spec.py` 로 따로
#    조립하지 않는다(2026-08-06 사용자 지시). 그래서 parts_meta 도 안 읽는다.

PIPE_IR = 0.050
WHEEL_R = float(os.environ.get("WHEEL_RAD", TUNE["wheel_r"]))
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
#      용접기 v2 는 자산 기본값 0.08 N·m 를 그대로 쓴다 — 휠이 r8 이라
#      12개 합산 견인력 120N 으로 벨로우즈(60N)와 견인/마찰 비가 같다.
WHEEL_MAXFORCE = float(os.environ.get("WHEEL_MAXFORCE", TUNE["wheel_maxf"]))

# ── 휠 드라이브 댐핑 — **수직관에서 굴러 내려가지 않으려면 필요하다** ──────
# 🚨 자산값 0.012 N·m·s/rad 로는 제동이 사실상 없다(2026-08-07 실측). 속도
#    드라이브의 토크는 `댐핑 × (지령 − 실제)` 라, 자중 20N 을 버티는 데 필요한
#    휠당 0.0133 N·m 를 내려면 **ω = 1.1 rad/s(64°/s)** 까지 굴러야 하고,
#    상한 0.08 N·m 에 닿으려면 380°/s(= 3 m/s) 가 필요하다.
#    → `--reverse` 로 수직 가지에서 출발시켰더니 안착 1.5초 만에 **635mm 굴러
#      내려갔다**(실속도/지령 212%). 0 을 주면 그대로 자산값을 쓴다.
WHEEL_DAMP = float(os.environ.get("WHEEL_DAMP", 0.0))

# 🎯 **중앙 관절 강성 1.0 N·m/도** (usda 원본 0.0698 은 너무 무르다).
#    분기 구멍 위에서 앞 세그먼트가 관절째 11~19° 꺾여 내려앉는 것을 막는다.
#    2.0 은 오히려 나쁘고 4.0 은 발산한다(실측).
# 🚨 용접기 v2 의 중앙 관절은 **D6(`PhysicsJoint`)** 라 revolute 가 아니다 —
#    `rotY`·`rotZ` 두 축에 각각 드라이브가 붙어 있고(자산 강성 0.2 / maxForce
#    1 / 한계 ±20°), 4개(J0·J1·J3·J_front)가 직렬이다.
# 🚨 **이 값의 실효 단위는 도(度)당이 아니라 라디안당이다** (2026-08-07 실측).
#    강성 2.0 에 오차 13.5° 면 "N·m/도" 라면 27 N·m 라 상한 24 에 걸려야 하는데
#    관절이 지령의 **27% 밖에** 안 따라왔다. 라디안당으로 보면 2.0×0.236 =
#    0.47 N·m 뿐이라 앞뒤가 맞는다. 조향에 필요한 실측 저항은 **약 3 N·m**
#    (다리 6개×9N 을 벽을 따라 미끄러뜨리는 힘 + 스프링 압축).
#      강성 2 / 상한 24  → 관절합 −16° (지령 −54°)  ❌ 이탈 80mm
#      강성 20 / 상한 12 → 관절합 −58°              이탈 최대 32.8mm
#      강성 40 / 상한 12 → 관절합 −61°              이탈 최대 27.6mm
#      강성 **60** / 상한 **15** → 관절합 −58°      이탈 최대 21.6mm ← 채택
BELLOWS_STIFF = float(os.environ.get("BELLOWS_STIFF", TUNE["bel_stiff"]))
BELLOWS_MAXF = float(os.environ.get("BELLOWS_MAXF", TUNE["bel_maxf"]))

# 🔑 **다리는 반경 방향 직동 슬라이더**다. 도달 반경을 늘리는 데 기구를 바꿀
#    필요가 없다 — 스트로크 상한만 올리면 된다.
#      10mm → 휠 중심 40.0mm (DN100 r50 에 딱)
#      35mm → 휠 중심 68.2mm (DN150 r75 에 필요한 65mm 확보)
# 🚨 예압은 목표를 스트로크 밖에 두는 힘 드라이브라, 스트로크를 늘리면 목표도
#    같이 밀어야 9N 이 유지된다(목표 = 스트로크 × 2).
PISTON_STROKE = float(os.environ.get("PISTON_STROKE", TUNE["piston_stroke"]))
# 🚨 **다리를 벽에 닿은 자세로 시작한다.** 정지에서 9N 으로 튀어나가면 한 스텝
#    이동이 감지폭(0.5mm)을 넘어 벽을 뚫는다. 스트로크 10mm 면 뚫어도 관 안에
#    남지만 35mm 면 관 밖까지 나가 통째로 발산했다(실측).
#    용접기 v2 는 다리가 다 들어간 자세에서 휠 중심 반경이 이미 40.0mm 라
#    관벽(50 − 휠반경 8 = 42mm)까지 2mm 뿐이다 → 초기 신장 2mm.
PISTON_INIT = float(os.environ.get("PISTON_INIT", TUNE["piston_init"]))
# 🚨 **예압은 maxForce 로 잘린다.** 목표(Δ)를 키워도 상한이 9N 이면 힘은 9N 그대로다.
#    설계 예압이 9N(피스톤 6개 → 수직항력 54N)이라 기본은 9. 더 밀어야 하면
#    이 값을 같이 올려야 한다.
PISTON_MAXF = float(os.environ.get("PISTON_MAXF", TUNE["piston_maxf"]))
# 🚨 **수축 쪽 한계는 따로다.** `PISTON_STROKE` 는 뻗는 쪽(상한)만 바꾸고
#    수축 한계는 자산값(용접기 v2 = −4mm)이 그대로였다. 접합부에서 앞
#    세그먼트가 13mm 밀리면 가까운 쪽은 13mm 를 접어야 하는데 −4mm 에서
#    스토퍼에 박혀 물린다(실측: 앞 다리 3개가 −4.0 에 바닥). 0 이면 자산값.
# 🎯 **−15mm 로 늘리니 T 분기를 정방향으로 통과한다**(끼임 0회, 2026-08-07).
#    −25mm 도 결과가 소수점까지 같으니 −15 면 충분하다. **자산 설계값으로
#    넘길 것** — usda 는 안 고치고 여기서만 덮어쓴다.
PISTON_RETRACT = float(os.environ.get(
    "PISTON_RETRACT", TUNE.get("piston_retract", 0.0)))

# ── 중심 유지 — 다리 신장을 서로 같게 맞춘다 ────────────────────────
# 🚨 9N **상수력**만으로는 어느 쪽으로 치우쳐도 힘이 같아 되돌리는 힘이 없다
#    (실측: 관경 변화 구간에서 평균 이탈 16.9mm). 이 로봇은 실린더가 밖으로
#    팽창하며 벽을 밀어 가는 구조라, **세 다리 신장이 같아야 중앙에 선다.**
#    → 세그먼트마다 평균 신장 + Δ 를 모두의 목표로 준다. 덜 뻗은 다리가 더
#      세게 밀어 몸체가 중앙으로 돌아온다. Δ×강성(3000) = 예압 9N.
CENTER_ON = os.environ.get("CENTER", "1") == "1"
CENTER_DELTA = float(os.environ.get("CENTER_DELTA", TUNE["center_delta"]))

# ── 다리별 접촉 판정 — **분기 개구 위의 다리를 따로 다룬다** ──────────
# 🔑 예압 목표를 스트로크 **밖**에 두고 있으므로 검출이 공짜다:
#      벽에 닿은 다리 → 스트로크 안에서 멈춘다
#      허공을 짚는 다리 → **상한까지 뻗는다**
#    상한 근처면 "벽 없음"이다.
# 🚨 벽 없는 다리를 평균에 넣으면 **그 값이 평균을 끌어올려 나머지 다리가
#    몸을 구멍 쪽으로 민다**(기록된 진단). → 평균에서 빼고, 그 다리만 접는다.
# 🚨 **위치로는 못 가른다** (2026-08-07 실측). `CENTER` 가 켜지면 다리 목표가
#    스트로크 밖의 예압값이 아니라 **평균+Δ** 로 바뀌므로, 벽 없는 다리도
#    상한(35mm)까지 안 가고 평균 근처에 멈춘다(실측 최대 신장 11mm). 위치
#    기준 판정기는 **한 번도 안 걸렸다**(뜬다리 0개).
#    → **관절이 실제로 내는 힘**으로 가른다. 벽을 밀면 힘이 실리고, 허공을
#      짚으면 0 에 가깝다. 임계는 예압 9N 의 20% 로 둔다.
# 🚨 **기본은 꺼 둔다 — 이 판정기는 아직 틀린다.** `CENTER` 가 위치 목표를
#    주는 구조에서는 힘도 접촉 지표가 못 된다: 목표에 **도달한** 다리는 벽에
#    닿았든 허공이든 힘이 0 에 가깝다. 실측에서 12개 중 **10개를 '벽 없음'
#    으로 오판**했다(실제로는 전부 −9.6~+0.2mm 로 눌려 있었다).
#    제대로 하려면 예압을 **다리별 힘 지령**으로 주고(위치 목표를 버리고)
#    거기서 위치로 접촉을 읽어야 한다 — 구조 변경이라 별도 작업.
# 🔑 **위치로 판정한다 — 스트로크 한계에 붙은 다리는 확실히 벽이 없다.**
#    `center_legs` 가 **평균+Δ** 를 목표로 주므로, 벽이 없으면
#    뻗음 → 평균 상승 → 목표 상승 → 더 뻗음 으로 **한계까지 래칫**된다
#    (실측: `tee_in` 접합부에서 35.0mm = 도달 75mm = **관 밖 20mm**).
#    한계에 붙었다는 것 자체가 "아무것도 안 막았다"는 뜻이다.
# 🚨 힘으로 판정하는 것은 **틀렸다**(실측 12개 중 10개 오판) — 목표에 도달한
#    다리는 벽에 닿았든 허공이든 힘이 0 에 가깝다. 위치가 맞는 지표다.
LEG_FREE_ON = os.environ.get("LEG_FREE", "1") == "1"
LEG_FREE_MARGIN = float(os.environ.get("LEG_FREE_MARGIN", 0.004))
LEG_FREE_CLEAR = float(os.environ.get("LEG_FREE_CLEAR", 0.010))
# 🚨 한계에 **닿아야** 판정하면 이미 늦다(최대신장 35.0mm = 관 밖 20mm 가
#    그대로 찍혔다). 같은 세그먼트 다리들의 **중앙값**을 관벽 추정치로 삼아,
#    거기서 이만큼 넘게 튀어나간 다리는 그 전에 벽이 없다고 본다.
#    중앙값은 6개 중 2개까지 튀어도 안 흔들린다.
LEG_FREE_GAP = float(os.environ.get("LEG_FREE_GAP", 0.008))
LEG_FREE_GAP_CLEAR = float(os.environ.get("LEG_FREE_GAP_CLEAR", 0.004))
# 벽 없는 다리의 목표 신장 — 접어서 개구 턱에 안 걸리게 한다
LEG_FOLD = float(os.environ.get("LEG_FOLD", 0.0))

# ── 🎯 다리를 **위치가 아니라 힘으로** 몬다 (2026-08-07 신설) ──────────
# 🚨 위치 목표(`center_legs` 의 평균+Δ)는 **자기 참조**라 벽이 없으면
#    뻗음 → 평균 상승 → 목표 상승 → 더 뻗음 으로 **한계까지 래칫**된다
#    (실측: 각진 T 양방향 모두 35.0mm = 도달 75mm = 관 밖 20mm).
# 🔑 힘으로 몰면 그 결합이 사라진다 — 허공을 짚는 다리가 뻗어도 **다른 다리의
#    목표를 끌어올리지 않는다.** 그리고 위치가 곧 접촉 정보가 된다(같은 힘을
#    줬는데 끝까지 나가면 벽이 없는 것).
#    · 기본 예압  F0 = 9N 바깥쪽 (설계값)
#    · 중심 유지  덜 뻗은 다리(=벽이 가까운 쪽)를 더 세게 민다: +Kc·(평균−q)
#    · 벽 없는 다리는 **안쪽으로** 당겨 접는다(개구 턱에 안 걸리게)
# 🚨 드라이브는 **순수 댐퍼**로 바꾼다(강성 0) — 안 그러면 위치 드라이브와
#    힘 지령이 서로 싸운다.
LEG_FORCE = os.environ.get("LEG_FORCE", "1") == "1"
LEG_KC = float(os.environ.get("LEG_KC", 300.0))     # N/m, 중심 복원
LEG_FOLD_N = float(os.environ.get("LEG_FOLD_N", 6.0))   # N, 뜬 다리 접는 힘
LEG_DAMP = float(os.environ.get("LEG_DAMP", 100.0))     # N·s/m
# 🔑 **힘 모드에서만 쓸 수 있는 판정 — 속도.** 같은 힘을 주면 벽에 닿은
#    다리는 **서 있고**, 허공을 짚는 다리는 **계속 나간다**(종단속도
#    F0/damping = 9/100 = 90mm/s). 위치로는 DN150 확장과 구별이 안 되지만
#    (둘 다 많이 뻗는다) 속도로는 갈린다 — 확장 구간에서는 벽에 닿는 순간 선다.
#    기준 반경은 **서 있는 다리들**에서 잡는다(그것이 곧 관벽이다).
LEG_FREE_V = float(os.environ.get("LEG_FREE_V", 0.02))    # m/s, 계속 나가면 뜬 것
LEG_STOP_V = float(os.environ.get("LEG_STOP_V", 0.005))   # m/s, 이하면 서 있다
LEG_REF_GAP = float(os.environ.get("LEG_REF_GAP", 0.006))  # m, 기준보다 이만큼 밖

# ── 속도 유지 조절기 (휠 실측 각속도 되먹임) ────────────────────────
GOV_ON = os.environ.get("GOV", "1") == "1"
GOV_KI = float(os.environ.get("GOV_KI", 2.0))
GOV_MIN, GOV_MAX = 0.5, 1.5

# ── 곡관 감속 — 관절 엔코더로 스스로 정한다 ─────────────────────────
AUTO_SPEED = os.environ.get("AUTO_SPEED", "1") == "1"
CURVE_MIN = float(os.environ.get("CURVE_MIN", 0.45))
BEND_REF = float(os.environ.get("BEND_REF", 14.0))
ACC_MPS2, DEC_MPS2 = 0.08, 0.40

# ── 다리 실린더를 **판단층에 올린다** (2026-08-07 신설) ─────────────
# 🚨 지금까지 속도를 정하는 데 다리 값이 **한 군데도** 안 들어갔다. 판단은
#    중앙 관절 각도, 되먹임은 휠 각속도뿐이었다. 그런데 **분기에서는 관절이
#    거의 안 꺾인다**(R150 에서 2~3°) — 곡관 감속이 사실상 안 걸린다.
#    다리가 그 구멍을 메운다. 설계에도 "서스펜션 스트로크는 관경 변화 감지
#    보조"로 적혀 있던 항목이다.
#      · 다리 신장 **편차**가 커진다 = 벽이 사라졌다(분기) / 단면이 안 둥글다
#      · 다리 평균이 **기준선에서 벗어난다** = 관경이 변한다(리듀서)
# 🔑 곡관 감속과 **같은 층**이므로 곱하지 않고 **더 느린 쪽을 택한다**(min).
#    곱하면 둘 다 걸릴 때 0.45×0.45=0.20 으로 과도하게 죽는다.
LEG_SLOW_ON = os.environ.get("LEG_SLOW", "1") == "1"
LEG_SPREAD_REF = float(os.environ.get("LEG_SPREAD_REF", 0.008))   # 8mm
LEG_DEV_REF = float(os.environ.get("LEG_DEV_REF", 0.005))         # 5mm (실측 근거)
LEG_MIN = float(os.environ.get("LEG_MIN", 0.45))

# ── 조향 — 중앙 관절로 **중심선을 따라간다** (2026-08-07 신설) ────────
# 🔑 T 분기가 실패한 원인은 CAD 가 아니라 **아무도 꺾으라고 안 했기 때문**이다.
#    곡관은 벽이 로봇을 꺾어 주지만(최대 꺾임 46~61° 실측) 분기는 직진로가
#    열려 있어 안 꺾인다(6.9°). 로봇은 본관을 그대로 지나가고 중심선만 가지로
#    올라가니 이탈이 벌어진다.
# 🚨 **이것은 도면을 아는 주행이다.** 연습장이 중심선을 이미 갖고 있어서 쓰는
#    것이고, 자율 인지(카메라로 분기를 찾아 오른손 법칙으로 고르는 것)가
#    아니다. 실기에서는 `condition/` 의 개구 검출이 이 목표각을 줘야 한다.
STEER_ON = os.environ.get("STEER", "1") == "1"
STEER_KP = float(os.environ.get("STEER_KP", 0.9))
# 🚨 목표각을 한 번에 던지면 발산한다(벨로우즈에서 실측: 링크가 6.7g 인데
#    45° 를 즉시 주면 3.1 N·m 가 걸려 NaN). 초당 각도로 제한한다.
STEER_RATE = float(os.environ.get("STEER_RATE", 45.0))
# 관절 하나당 목표각 상한 — 자산 한계 ±20° 안쪽으로 둔다.
STEER_MAX = float(os.environ.get("STEER_MAX", 17.0))
# 직진 장면에서 잔류 굽힘을 펴는 속도(°/s) — steer_vision 의 완화 경로 전용
RELAX_RATE = float(os.environ.get("RELAX_RATE", 20.0))
# 앞 세그먼트 위치에서 **추가로** 얼마나 더 내다볼 것인가.
# 🎯 **0.05 → 0.12** (2026-08-07 실측). 50mm 만 보면 원호에 들어가고 나서야
#    꺾기 시작해 뒤처지고, 만회하려면 **80°** 가 필요해진다 — 그게 관절 한계
#    (D6 4개 × ±20° = 80°)와 같아서 못 따라잡는다.
#    ⚠ 한계를 넓히는 쪽(`STEER_MAX` 17→19.5°)은 **효과가 없었다**(끼임 2회).
#    늦게 꺾는 것이 원인이지 못 꺾는 것이 원인이 아니다.
#      LOOK 0.05  → 끼임 3회 (s=340~350)
#      LOOK 0.12  → **코스 끝 도달 · 끼임 0회** · 이탈 4.8/10.8mm  ← 채택
STEER_LOOK = float(os.environ.get("STEER_LOOK", 0.12))

# 출발 롤(°) — 다리 클럭 위치를 개구에 대해 돌려 본다. 다리가 120° 간격이라
# 0~120° 가 전부다. 🚨 실기에서는 로봇이 롤을 제어하지 못한다 — 이 값에
# 통과가 좌우되면 그 자체가 **설계가 불안정하다는 뜻**이다.
# 🎯 **항법 모드** (2026-08-07 사용자 지시: *"연습장에서 도면 의존부터 걷어내"*)
#   onboard   : **로봇 신호만** 쓴다 — 관절 엔코더 · 다리 실린더 · 휠 각속도.
#               중심선(도면)은 **채점에만** 쓰고 판단에는 안 쓴다.
#   blueprint : 예전 방식 — 중심선을 따라간다. **도면을 아는 주행**이므로
#               성능 수치를 자율주행 근거로 쓰면 안 된다. 비교용으로만 남긴다.
# 🚨 남아 있는 도면 의존(둘 다 **심판** 용도): 끼임·이탈 판정(`cl.nearest`),
#    출발/종료 지점(START_S·END_S). 실기로 옮길 때 이 둘도 걷어내야 한다.
#   vision    : **설계대로** — 카메라 → `condition/detector` + `condition/
#               odometry` → `driver/control.DriveController` 가 속도를 정한다.
#               연습장은 심판만 본다. 🚨 카메라를 켜므로 GUI 가 느려진다.
NAV = os.environ.get("NAV", "onboard")
# 개구 쪽으로 몰아 주는 조향 세기(총 꺾임, 도) — 로봇 신호만으로 정한다
STEER_ONBOARD_DEG = float(os.environ.get("STEER_ONBOARD_DEG", 40.0))

ROLL_DEG = float(os.environ.get("ROLL_DEG", 0.0))

# 안착 궤적을 0.25초마다 찍는다 (수직관에서 흘러내리는지 진단).
SETTLE_TRACE = os.environ.get("SETTLE_TRACE", "0") == "1"

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
# 🚨 **배관은 STEP 으로 받아 `tools/step_to_usd.py` 로 굽는다.** 직접 저작한
#    `.usda` 판은 면·법선·중복이 다 정상인데도 PhysX 가 못 받아 로봇이 닿는
#    즉시 발산했다(실측). 변환기로 구운 판은 다른 CAD 배관과 같은 경로라 정상.
_RED = MAPS / "reducer_r.usd"
# 🎯 **양방향 스위프 T** (R185, 2026-08-06 STEP 수령). 가지는 본관에 수직이지만
#    접합부 목이 본관 **양쪽 모두**와 R185 로 블렌드된다 — 각도가 아니라
#    곡률로 푸는 형상. 로봇 요축 한계 55° 로는 각진 90° 를 못 돌지만 R150
#    곡관은 통과한다는 실측에서 나온 요청이다.
_TEES = MAPS / "tee_sweep.usd"
# 🎯 **한쪽만 스위프한 T** (R185, 2026-08-07 STEP 수령). 양방향 판이 개구를
#    원주 ±93mm 로 벌려 다리가 밀 벽을 잃은 것에 대한 재요청분이다.
#    실측(`scratchpad/measure_tee.py`) — 내 50 / 외 55, 삼각형 1,625,
#    본관 X −400~+400, 가지 x=0 에서 +Z 로 800.
#    🔑 **개구 원주폭 109.1mm (±54.5mm)** — 양방향 판 ±93mm 에서 줄었다.
#      개구 축방향 구간 x −50~+145 로 **+X 쪽으로 치우쳐** 있다 = 블렌드가
#      본관 **+X 다리와 가지** 사이에만 있다. 그래서 매끄러운 경로는
#      `본관 +X → R185 원호 → +Z 가지` 이고, 원호 중심은 (+185, 0, +185) 다
#      (양방향 판의 (−185,0,185) 와 좌우가 뒤집힌다).
_TEES1 = MAPS / "tee_sweep_r185_dn100_oneside.usd"
# 🚨 **`_narrow` 는 이름과 달리 더 넓다** (2026-08-07 수령·실측). 개구 원주폭을
#    ≤100mm 로 좁혀 달라고 요청했는데 **109.1 → 126.5mm(145°)** 로 벌어졌고
#    축방향도 195 → 230mm 로 길어졌다. 형상 자체는 같은 계열(내 50/외 55,
#    본관 X ±400, 가지 x=0 에서 +Z 800, 블렌드는 +X 쪽 한 곳).
_TEES2 = MAPS / "tee_sweep_r185_dn100_oneside_narrow.usd"
# 🔑 **각진 T** — 접합부에 곡률이 전혀 없는 원래 형상 (2026-08-07 사용자 지시:
#    *"이번에는 각진 T로 해보자"*). 실측: 내 50 / 외 55, **본관은 Y 축**
#    (−250~+250), 가지는 +Z 로 250, 삼각형 2,676.
#    ⚠ 스위프 판의 "목"은 내가 만든 것이 아니라 `CAD_요청_분기관.md`(08-06)
#      에서 요청해 받은 것이다 — 각진 T 로는 분기 진입이 안 돼 곡률을 요청했다.
_TEEA = MAPS / "tshape_test.usd"
_ANG_R = float(os.environ.get("ANG_R", 90.0))


# 🚨 **2026-08-07 거울상 수정** — 사용자가 GUI 로 세 번 지적한 "턴이 반대"의
#    실체. 이전 정의는 두 코스 모두 월드 기준 **왼손 턴**이었다(수치 검증:
#    진행방향×상방(+Z) 과 탈출방향의 내적 −1.0). 코스 배치(Rz−90·Rx90)에서
#    CAD +Y 팔 = 월드 +X, CAD −Y 팔 = 월드 −X, CAD stem(+Z) = 월드 −Y 이므로
#    - tee_go  : stem(월드 −Y 쪽)에서 +Y 로 접근 → 오른쪽 = 월드 +X = **CAD +Y 팔**
#    - tee_back: 왼팔 = 월드 −X = **CAD −Y 팔**에서 +X 로 접근 → 오른쪽 턴 = stem
#    이전 정의는 정확히 이 반대(CAD y 부호가 뒤집힘)였다. 각진 T 는 좌우
#    대칭이라 y 만 뒤집으면 되고, 기존 진입·후진 기동 검증은 그대로 유효하다.
# 🚨 (2026-08-07 원복) 다리에 600mm 직관을 연장하는 안은 **폐기** — 사용자
#    요청은 "기존 관 안에서 양끝을 더 쓰라"였고, 연장은 개구 광선이 가지 관
#    안으로 달아나 곡관 판별자를 깨는 부작용까지 냈다(덤프 실측). 양끝 확대는
#    START_S·END_S 여유 축소로 한다.
def _tee_ang_out(R=None):
    """각진 T — **왼팔(CAD −Y) → 본관 stem(+Z)**. 월드 기준 오른손 턴."""
    R = _ANG_R if R is None else R
    pts = [(0.0, y, 0.0) for y in np.linspace(-245.0, -R, 40)]
    pts += [(0.0, -R + R * math.sin(a), R - R * math.cos(a))
            for a in np.linspace(0.0, math.pi / 2, 26)[1:]]
    pts += [(0.0, 0.0, z) for z in np.linspace(R, 245.0, 26)[1:]]
    return np.array(pts)


def _tee_ang_in(R=None):
    """각진 T — **본관 stem(+Z) → 오른팔(CAD +Y)**. 월드 기준 오른손 턴."""
    R = _ANG_R if R is None else R
    pts = [(0.0, 0.0, z) for z in np.linspace(245.0, R, 26)]
    pts += [(0.0, R - R * math.cos(a), R - R * math.sin(a))
            for a in np.linspace(0.0, math.pi / 2, 26)[1:]]
    pts += [(0.0, y, 0.0) for y in np.linspace(R, 245.0, 40)[1:]]
    return np.array(pts)

# 코스별 중심선 — **원본 좌표(mm)로** 적고, 아래에서 코스 변환을 그대로 먹인다.
# 손으로 월드 좌표를 다시 쓰면 회전을 두 번 계산하게 되어 어긋난다.
_ARC_R, _ARC_N = 150.0, 48


def _tee_centerline():
    """스위프 T 중심선 — 본관 +X 에서 와서 R185 원호로 +Z 가지로 오른다.

    블렌드가 +X 쪽 한 곳뿐이라(개구 x −50~+145) 매끄러운 통로는 이 하나다.
    원호 중심 (+185, 0, +185), 직관 끝 (185,0,0) → 가지 (0,0,185).
    """
    return np.vstack([
        np.array([(x, 0.0, 0.0) for x in np.linspace(395, 185, 43)]),
        np.array([(185 - 185 * math.sin(a), 0.0, 185 - 185 * math.cos(a))
                  for a in np.linspace(0, math.pi / 2, 30)[1:]]),
        np.array([(0.0, 0.0, z) for z in np.linspace(185, 790, 61)[1:]])])


def _tee_cl_out(R=185.0):
    """**본관 +X → 가지**. 원호 중심 (+R, 0, +R), (R,0,0) → (0,0,R)."""
    pts = [(x, 0.0, 0.0) for x in np.linspace(395.0, R, 43)]
    pts += [(R - R * math.sin(a), 0.0, R - R * math.cos(a))
            for a in np.linspace(0.0, math.pi / 2, 30)[1:]]
    pts += [(0.0, 0.0, z) for z in np.linspace(R, 790.0, 61)[1:]]
    return np.array(pts)


def _tee_cl_in(R=185.0):
    """**가지 → 본관 −X**. 원호 중심 (−R, 0, +R), (0,0,R) → (−R,0,0).

    🔑 `_tee_cl_out` 과 **반대쪽 모서리**다. 사용자 맵의 오른손 법칙 루프가
       한 분기에서 쓰는 두 모서리가 바로 이 둘이다 — 나갈 때 한쪽, 한 바퀴
       돌아 돌아올 때 반대쪽. **그래서 양쪽 다 R185 블렌드가 있어야 한다.**
       한쪽만 블렌드한 판(`_oneside`)은 이 경로가 직각이 되어 성립하지 않는다.
    """
    pts = [(0.0, 0.0, z) for z in np.linspace(790.0, R, 61)]
    pts += [(-R + R * math.cos(a), 0.0, R - R * math.sin(a))
            for a in np.linspace(0.0, math.pi / 2, 30)[1:]]
    pts += [(x, 0.0, 0.0) for x in np.linspace(-R, -395.0, 30)[1:]]
    return np.array(pts)


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


COURSES = {
    # 이름:      (USD, 코스 변환(원본 mm → 월드 m), 중심선(원본 mm), 설명)
    "elbow_v": (_ELBOW, scale(MM) * rot(90, (0, 0, 1)) * trans(0.0, 0.0, 0.0),
                _elbow_centerline(), "수직 곡관 — 직관 200mm 뒤 위로 꺾인다"),
    # 🚨 y=0.9 였을 때 **출구 다리(월드 x≈0, y 0.65~0.75)가 reducer 관축
    #    (y=0.7, x −1.25~+1.3)을 관통**해 reducer 로봇이 x=+70mm(s=670)에서
    #    막혔다(2026-08-07 GUI 실측, 사용자 발견). ALL 4대 공존을 위해 y=2.0.
    "elbow_h": (_ELBOW,
                scale(MM) * rot(90, (0, 0, 1)) * rot(90, (1, 0, 0))
                * trans(0.0, 2.0, 0.0),
                _elbow_centerline(), "수평 곡관 — 같은 관을 진행축으로 90° 굴렸다"),
    # 🔑 **관경 변화 시험편** DN100→150→100 (STEP 수령 → 변환).
    #    관 축이 이미 +X 라 회전은 필요 없다. 다른 코스와 안 겹치게 y=+3.2.
    # 가지 진입 경로: 본관 −X 에서 와서 R185 원호로 꺾여 +Z 로 오른다.
    #   원호 중심 (−185, 0, 185) → 직관 끝 (−185,0,0) 에서 (0,0,185) 로 잇는다
    "tee_s": (_TEES, scale(MM) * trans(0.0, -1.5, 0.0),
              np.vstack([
                  np.array([(x, 0.0, 0.0) for x in np.linspace(-400, -185, 44)]),
                  np.array([(-185 + 185 * math.sin(a), 0.0,
                             185 - 185 * math.cos(a))
                            for a in np.linspace(0, math.pi / 2, 30)[1:]]),
                  np.array([(0.0, 0.0, z) for z in np.linspace(185, 790, 61)[1:]])]),
              "스위프 T R185 — 본관에서 가지로 90° 진입"),
    # 한쪽 스위프 T — 블렌드가 +X 쪽이라 **+X 에서 와서 −X 로 가다** 가지로
    # 올라간다. 원호 중심 (+185,0,+185) → 직관 끝 (+185,0,0) 에서 (0,0,185) 로.
    # 🚨 다른 관과 겹치면 그 겹침 때문에 발산한다(기록된 사고) → y=−3.0 에 홀로.
    # 🚨 **T 는 눕힌다** (2026-08-07 사용자 지시 재확인 — *"얘는 수직으로 있을
    #    일이 없어"*). CAD 원본은 가지가 +Z 로 서 있는데, `Rx(90°)` 이 본관
    #    진행축(월드 X)은 그대로 두고 가지만 −Y 로 눕힌다. 곡관 `elbow_h` 와
    #    같은 수법이고, 중심선에도 같은 변환이 그대로 먹는다(`xf_pts`).
    # 🎯 **사용자 맵의 루프가 실제로 쓰는 두 모서리** (2026-08-07 정정).
    #    오른손 법칙 루프는 한 분기에서 **나갈 때 한쪽 / 돌아올 때 반대쪽**
    #    모서리를 쓴다. 그래서 **양쪽 다 R185 블렌드가 있어야** 하고,
    #    양방향 판(`tee_sweep.usd`)이 그 물건이다.
    #    🚨 내가 CAD 에 낸 "블렌드는 한쪽만" 요청은 **틀렸다** — 한쪽만 남기면
    #       복귀 모서리가 직각(곡률 0)이 되어 성립하지 않는다.
    #    🚨 **T 는 눕힌다**(*"얘는 수직으로 있을 일이 없어"*). `Rx(90°)` 이 본관
    #       진행축(월드 X)은 그대로 두고 가지만 −Y 로 눕힌다.
    "tee_out": (_TEES, scale(MM) * rot(90, (1, 0, 0)) * trans(0.0, 0.0, 0.0),
                _tee_cl_out(),
                "양방향 스위프 T R185(수평) — **본관 +X → 가지** (나가는 쪽)"),
    "tee_in": (_TEES, scale(MM) * rot(90, (1, 0, 0)) * trans(0.0, -1.1, 0.0),
               _tee_cl_in(),
               "양방향 스위프 T R185(수평) — **가지 → 본관 −X** (돌아오는 쪽)"),
    # 🎯 **각진 T** — 본관을 X 로 돌리고(Rz −90) 가지를 눕힌다(Rx 90).
    #    원본은 본관이 Y, 가지가 +Z 라 스위프 판과 축 규약이 다르다.
    # 🎯 **사용자 규약 (2026-08-07 확정): T 의 세로 획(stem)이 본관이다.**
    #    임무 루프 = 본관→오른팔로 나가고, 밖에서 한 바퀴 돈 뒤 **왼팔에서
    #    오른쪽으로 꺾어 본관으로 전진 복귀**한다. 둘 다 오른손 법칙 하나로
    #    풀린다(오른쪽 개구 → 진입, 왼쪽 개구 → 직진). 후진 recall 은 임무
    #    위상이 아니다 — 연습장 반복용일 뿐.
    #    🚨 이전 이름(tee_a_out/in)은 CAD 가로대를 본관으로 놓아 **의미가
    #    뒤집혀 있었다**(사용자가 세 번 지적). 기하·검증은 그대로 유효하다.
    "tee_go": (_TEEA,
               scale(MM) * rot(-90, (0, 0, 1)) * rot(90, (1, 0, 0))
               * trans(0.0, 0.0, 0.0),
               _tee_ang_in(),
               "각진 T(수평) — **본관(stem) → 오른팔** (나감, 오른손 법칙)"),
    "tee_back": (_TEEA,
                 scale(MM) * rot(-90, (0, 0, 1)) * rot(90, (1, 0, 0))
                 * trans(0.0, -1.1, 0.0),
                 _tee_ang_out(),
                 "각진 T(수평) — **왼팔 → 본관(stem)** (복귀, 오른손 법칙)"),
    # ── 아래는 비교용 (스위프 판). `all` 에는 안 들어간다 ──────────
    "tee_s1": (_TEES1, scale(MM) * rot(90, (1, 0, 0)) * trans(0.0, -3.0, 0.0),
               _tee_centerline(),
               "한쪽 스위프 T R185(수평) — 본관 → 가지 (블렌드된 모서리)"),
    "tee_s1_rev": (_TEES1,
                   scale(MM) * rot(90, (1, 0, 0)) * trans(0.0, -6.0, 0.0),
                   _tee_centerline()[::-1],
                   "한쪽 스위프 T — 같은 모서리를 거꾸로 (루프와는 다른 경로)"),
    "tee_s2": (_TEES2, scale(MM) * rot(90, (1, 0, 0)) * trans(0.0, -9.0, 0.0),
               _tee_centerline(),
               "한쪽 스위프 T '_narrow' — 개구 145°"),
    "reducer": (_RED, scale(MM) * trans(-0.65, 0.7, 0.0),
                np.array([(x, 0.0, 0.0)
                          for x in np.linspace(-600.0, 1300.0, 191)]),
                "관경 변화 DN100→150→100 — 테이퍼 반각 4.76°"),
}
# 🎯 **`all` = 관 5개 · 로봇 5기** (2026-08-07 사용자 지시). 배관은 4종인데
#    **T 만 정방향·역방향 두 대**로 나눠 각자 왕복시킨다 — 관을 따로 깔고
#    중심선만 뒤집는다.
#    `tee_s`(양방향 블렌드 판)는 제외 — 개구 213° 라 본관→가지에서 지금도
#    끼임 4회로 실패한다. 비교용 정의는 남겨 `--course tee_s` 로 부를 수 있다.
#    `tee_s2`(`_narrow` 판, 145°)도 통과하지만 대표는 `tee_s1` 하나로 둔다.
# 🎯 2026-08-07 사용자 지시 — 곡관 2종은 확인이 끝났으니 빼고 **3개만**,
#    그리고 **서로 붙여 놓는다**(전에는 y 로 3~6m 씩 떨어져 있어 한 화면에
#    안 들어왔고, 가지→본관 코스를 아예 못 찾았다).
# 🎯 2026-08-07 사용자 지시: "GUI 로 4개 다" — elbow_h 를 다시 넣는다
#    (센터링 조향·곡관 판별자 검증 이후). 배치 y=+0.9 라 기존 셋과 안 겹친다.
ALL_NAMES = ["tee_go", "tee_back", "reducer", "elbow_h"]
# 🎯 쉼표 목록 지원 (2026-08-07 사용자: 렌더 부하 때문에 단계별로 확인 —
#    "T 2개 먼저, 곡관 2개, 마지막 reducer"). `--course tee_go,tee_back`.
RUN_NAMES = list(ALL_NAMES) if COURSE == "all" else COURSE.split(",")
for _nm in ([] if COURSE == "all" else RUN_NAMES):
    if _nm not in COURSES:
        raise SystemExit(f"[중단] --course 는 all 또는 {list(COURSES)} 의 "
                         f"쉼표 목록이어야 한다 (받은 값: {_nm!r})")
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
paths = {}
# 🔑 **로봇을 넣을 관만 올린다.** 예전에는 정의된 관을 전부 올려서, 한 대만
#    돌릴 때도 빈 관이 같이 떠 GUI 가 지저분하고 cook 시간만 늘었다.
print(f"[코스] 배관 {len(RUN_NAMES)}개 적재 — 로봇 투입 {COURSE}")
for name in RUN_NAMES:
    usd, xform, cl_mm, desc = COURSES[name]
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
    if name == "reducer":
        _rt = xform.ExtractTranslation()
        # 🚨 입구 DN100 구간이 200mm 뿐인데 로봇은 170mm 다. 안착 중 74mm 뒤로
        #    밀리면 뒤가 관 밖으로 나가 발산한다 → **legacy 직관 600mm 를
        #    입구에 맞대 붙인다.** 🚨 코스 루트 아래에 넣으면 안 된다(루트에
        #    scale 0.001 이 걸려 있는데 make_mesh 는 이미 미터다) → 월드에 직접.
        _ep = make_mesh("/World/Pipe_reducer_ext",
                        SON / "legacy" / "meshes" / "pipe_straight.stl",
                        color=(0.55, 0.58, 0.60),
                        # 🚨 y 를 하드코딩하지 말 것 — 코스를 옮기면 연장관만
                        #    제자리에 남아 입구가 끊긴다. 코스 변환에서 끌어온다.
                        xform=trans(_rt[0] - 0.30, _rt[1],
                                    _rt[2])).GetPrim()
        UsdPhysics.CollisionAPI.Apply(_ep)
        UsdPhysics.MeshCollisionAPI.Apply(_ep).CreateApproximationAttr("none")
        UsdGeom.Mesh(_ep).CreateDoubleSidedAttr(True)
        UsdShade.MaterialBindingAPI.Apply(_ep).Bind(
            UsdShade.Material.Get(stage, "/World/PipePhysMat"),
            bindingStrength=UsdShade.Tokens.weakerThanDescendants,
            materialPurpose="physics")
        if GLASS:
            UsdShade.MaterialBindingAPI.Apply(_ep).Bind(
                _gl, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
        print("  reducer 입구 연장 — legacy 직관 600mm (월드 x −1.25~−0.65m)")

    if REVERSE and name == COURSE:
        cl_mm = cl_mm[::-1]          # 반대 방향 통과 시험
    cl = Centerline(xf_pts(xform, cl_mm))
    paths[name] = cl
    print(f"  {name:8s} {usd.name:18s} 메시 {len(meshes)} "
          f"(콜라이더 {n_col} / 중복면 {n_skip} 제외)  중심선 "
          f"{cl.total * 1000:.0f}mm  입구 ({cl.p[0][0] * 1000:+.0f}, "
          f"{cl.p[0][1] * 1000:+.0f}, {cl.p[0][2] * 1000:+.0f})mm → 출구 "
          f"({cl.p[-1][0] * 1000:+.0f}, {cl.p[-1][1] * 1000:+.0f}, "
          f"{cl.p[-1][2] * 1000:+.0f})mm")
    print(f"           {desc}")
tick("배관 적재 완료")
print(f"[항법] {NAV} — "
      + ("조향이 **중심선(도면)** 을 따라간다. 자율 근거로 쓰지 말 것"
         if NAV == "blueprint" else
         "조향·감속이 **로봇 신호만** 쓴다(관절·다리·휠). "
         "중심선은 채점 전용"))
print(f"[준비] 관 상태 **배수(건식)** — 마찰 {FRICTION_STATIC}/"
      f"{FRICTION_DYNAMIC}. 이 연습장에는 물이 없다"
      + ("   배관 표시 반투명(유리)" if GLASS else ""))

# ── 로봇 3대 — **코스마다 한 대씩 동시에 굴린다** (2026-08-06 사용자 지시) ──
# 🔑 관을 하나씩 갈아 끼우며 세 번 돌리면 세 배 걸린다. 셋을 한 씬에 같이
#    올리고 각자 제 코스를 왕복하게 둔다. 물리는 어차피 한 스텝에 전부 푼다.
# 🔑 **왕복 반복**: 끝에 닿으면 되돌아오고, 되돌아오면 다시 간다. 끼이면
#    방향을 뒤집어 빠져나온다(임무 규칙 8 "끼임 → 후진 재시도"의 연습장판).
#    사람이 볼 때까지 계속 돈다.



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
START_S = float(os.environ.get("START_S", 0.120))
# 🔑 리듀서만 다르다 — 입구 DN100 이 200mm 뿐이라 **연장관 안쪽**에서 출발한다.
START_S_RED = float(os.environ.get("START_S_RED", 0.30))
# 코스 **끝**에서도 여유를 둔다 — 안 그러면 로봇이 관 끝 밖으로 나간다.
# 🎯 0.16 → 0.10 (2026-08-07 사용자: *"양끝을 좀 더 이동하게"*). 관을 늘리지
#    않고 기존 관의 반환점을 끝쪽으로 60mm 당겼다. s 는 전방 세그먼트 기준
#    이라 0.10 이면 전방 휠이 관 끝 ~55mm 안쪽에서 돌아선다(사출 여유 유지).
END_S = float(os.environ.get("END_S", 0.10))

robots = []



def discover(root):
    """로봇 서브트리에서 **역할별 조인트**를 찾는다 — 이름이 아니라 구조로.

    🚨 두 자산의 이름 규약이 완전히 다르다(`seg0_piston_0` ↔ `RearBody_A0`).
       이름으로 고르면 자산을 갈아끼우는 순간 **조용히 0개**가 되어 아무것도
       안 하고, 로그만 정상으로 보인다(실제로 그렇게 돌고 있었다). 구조로 고른다:

         세그먼트 바디 = prismatic 조인트를 3개 이상 매단 body0
         다리(서스펜션) = body0 가 세그먼트 바디인 prismatic
         휠 구동        = body1 이름에 'wheel' 이 든 revolute
         중앙 관절      = 위 둘도 용접기도 아니면서 **회전 드라이브**가 있는 것

    🔑 용접기 조인트(`RingRotate`·`TorchExtend`)는 body 이름으로 걸러 낸다 —
       벨로우즈 자산에는 아예 없고, 용접기 v2 에서 중앙 관절로 오인되면
       주행 중에 토치가 돌아간다.
    """
    def b(j, k):
        t = j.GetRelationship(f"physics:body{k}").GetTargets()
        return t[0] if t else None

    joints = [p for p in Usd.PrimRange(root) if p.IsA(UsdPhysics.Joint)]
    pris = [j for j in joints if j.IsA(UsdPhysics.PrismaticJoint)]
    cnt = {}
    for j in pris:
        p0 = b(j, 0)
        if p0:
            cnt[p0] = cnt.get(p0, 0) + 1
    seg_bodies = [p for p, c in cnt.items() if c >= 3]

    legs, wheels, bend, weld = [], [], [], []
    for j in joints:
        p0, p1 = b(j, 0), b(j, 1)
        n0 = p0.name if p0 else ""
        n1 = p1.name if p1 else ""
        if any(k in (n0 + n1).lower() for k in ("weld", "ring", "torch")):
            weld.append(j)
            continue
        if j.IsA(UsdPhysics.PrismaticJoint) and p0 in seg_bodies:
            legs.append(j)
        elif "wheel" in n1.lower() and j.IsA(UsdPhysics.RevoluteJoint):
            wheels.append(j)
        elif any(a.startswith("drive:rot") or a.startswith("drive:angular")
                 for a in [x.GetName() for x in j.GetAttributes()]):
            bend.append(j)
    # 다리 → 세그먼트 묶음 (조인트 이름 → 세그먼트 번호)
    order = sorted({str(b(j, 0)) for j in legs})
    leg_seg = {j.GetName(): order.index(str(b(j, 0))) for j in legs}
    return dict(seg_bodies=seg_bodies, legs=legs, wheels=wheels,
                bend=bend, weld=weld, leg_seg=leg_seg)


def wlocal(prim):
    """프림 자기 로컬 변환의 이동 성분 (로봇 루트 기준 — 중간 Xform 은 항등)."""
    t = UsdGeom.Xformable(prim).GetLocalTransformation().ExtractTranslation()
    return np.array([float(t[0]), float(t[1]), float(t[2])])


def seg_body_prims(root, jd):
    """세그먼트 바디 두 개를 (뒤, 앞) 순서로 준다.

    이름 규약이 자산마다 다르므로(`seg0_body`/`seg1_body` ↔ `RearBody`/
    `FrontBody`) 'front' 또는 '1' 이 든 쪽을 앞으로 본다.
    """
    prims = [stage.GetPrimAtPath(p) for p in jd["seg_bodies"]]
    prims.sort(key=lambda p: (2 if "front" in p.GetName().lower() else 0)
               + (1 if "1" in p.GetName() else 0))
    return prims[0], prims[-1]


def cyl_to_hull(cprim, n=16):
    """실린더 프리미티브 콜라이더 → **convexHull 메시 콜라이더**로 교체.

    🚨 이 로봇은 휠 12개와 본체 2개의 충돌체가 `UsdGeom.Cylinder` 다. PhysX 5.4
       는 실린더를 **convex-core** 로 다루는데, 관 안에서 narrow phase 가
       `contactConvexCoreConvex` 에서 세그폴트를 냈고(코어 덤프), 안 죽는
       조건에서도 로봇이 관을 뚫고 나갔다. 우리 벨로우즈 로봇이 멀쩡했던 이유가
       휠이 **메시 convexHull** 이었기 때문이다(기록된 대조).
    → 같은 치수의 n각기둥 메시를 만들어 콜라이더를 옮기고, 원래 실린더는
      충돌만 끈다(형상·재질은 그대로 두어 보이는 것은 안 바뀐다).
    🚨 **`Cone` 도 같은 부류다** — PhysX 는 실린더·원뿔·캡슐을 전부 convex-core
       로 다룬다. 용접기 v2 는 토치 팁이 `Cone` 콜라이더라 이것 하나만 남겨도
       같은 세그폴트가 난다. 두 타입을 같이 받는다.
    🚨 물리 재질(`material:binding:physics`)을 **같이 옮겨야 한다** — 안 옮기면
       타이어 마찰이 조용히 씬 기본값으로 떨어진다.
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
        for k in range(n):                          # 밑면
            a = 2.0 * math.pi * k / n
            pts.append(put(-h / 2.0, r * math.cos(a), r * math.sin(a)))
        pts.append(put(+h / 2.0, 0.0, 0.0))         # 꼭짓점
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
        for k in range(1, n - 1):                  # 양 끝 뚜껑
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
    # 물리 재질을 그대로 옮긴다(안 옮기면 타이어 마찰이 씬 기본값이 된다)
    _src = cprim.GetRelationship("material:binding:physics")
    if _src and _src.GetTargets():
        mp.CreateRelationship("material:binding:physics").SetTargets(
            _src.GetTargets())
    # 원래 프리미티브에서 접촉 오프셋도 물려받는다(자산이 저작한 값)
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


def build_robot(name, path_cl):
    """코스 하나에 로봇 한 대. 반환 = FSM 상태를 담을 dict."""
    ref = f"/World/Ref_{name}"
    prim = stage.DefinePrim(ref, "Xform")
    # 🚨 용접기 v2 는 **씬 파일**이다 — `/World` 밑에 PhysicsScene 과 Looks 가
    #    같이 들어 있다. `/World/Robot` 서브트리만 참조하면 머티리얼 바인딩이
    #    스코프 밖이라 전부 끊긴다(**타이어 물리 재질까지** 끊겨 마찰이 바뀐다).
    #    → **파일을 통째로** 참조하고 중복 PhysicsScene 만 꺼 버린다.
    if ROBOT_PRIM_IN_USDA:
        prim.GetReferences().AddReference(ROBOT_USDA,
                                         Sdf.Path(ROBOT_PRIM_IN_USDA))
    else:
        prim.GetReferences().AddReference(ROBOT_USDA)
    _dead = [c for c in Usd.PrimRange(prim) if c.IsA(UsdPhysics.Scene)]
    for c in _dead:
        c.SetActive(False)

    # 아티큘레이션 루트를 **찾는다** — 참조 프림 자신일 수도, 그 밑일 수도 있다
    root = next((c for c in Usd.PrimRange(prim)
                 if c.HasAPI(UsdPhysics.ArticulationRootAPI)), prim)
    rp = str(root.GetPath())

    # 🚨 **자체 충돌을 끈다.** 이 로봇의 휠은 실린더 프리미티브라 PhysX 가
    #    convex-core 로 다루는데, 관 안에서 암·휠이 서로 닿는 순간
    #    `contactConvexCoreConvex` 에서 **세그폴트**가 났다(코어 덤프 2회).
    #    관벽과의 접촉은 그대로 살아 있다 — 꺼지는 것은 로봇 자기들끼리다.
    PhysxSchema.PhysxArticulationAPI.Apply(root) \
        .CreateEnabledSelfCollisionsAttr(False)

    jd = discover(root)
    if not (jd["legs"] and jd["wheels"] and jd["bend"]):
        raise SystemExit(
            f"[중단] {name}: 조인트를 못 찾았다 — 다리 {len(jd['legs'])} / "
            f"휠 {len(jd['wheels'])} / 중앙 {len(jd['bend'])}. "
            f"자산 구조가 바뀌었는지 볼 것 ({ASSET})")
    for _j in jd["legs"]:
        UsdPhysics.PrismaticJoint(_j).GetUpperLimitAttr().Set(PISTON_STROKE)
        if PISTON_RETRACT > 0:
            UsdPhysics.PrismaticJoint(_j).GetLowerLimitAttr().Set(
                -PISTON_RETRACT)
        _d = UsdPhysics.DriveAPI.Get(_j, "linear")
        if _d and LEG_FORCE:
            # 순수 댐퍼 — 예압·중심 유지는 매 스텝 **힘 지령**으로 준다
            _d.GetStiffnessAttr().Set(0.0)
            _d.GetDampingAttr().Set(LEG_DAMP)
            _d.GetTargetPositionAttr().Set(0.0)
            _d.GetMaxForceAttr().Set(max(PISTON_MAXF * 8.0, 60.0))
        elif _d:
            # 🚨 예압은 **목표를 스트로크 밖에** 두는 힘 드라이브라, 스트로크를
            #    늘리면 목표도 같이 밀어야 같은 힘이 유지된다.
            _d.GetTargetPositionAttr().Set(PISTON_STROKE * 2.0)
            _d.GetMaxForceAttr().Set(PISTON_MAXF)
    _nb = 0
    for _j in jd["bend"]:
        # 🚨 중앙 관절이 revolute(`angular`)일 수도 D6(`rotY`·`rotZ`)일 수도
        #    있다 — 붙어 있는 드라이브를 **전부** 덮는다.
        for _tok in ("angular", "rotX", "rotY", "rotZ"):
            _d = UsdPhysics.DriveAPI.Get(_j, _tok)
            if _d and _d.GetStiffnessAttr():
                _d.GetStiffnessAttr().Set(BELLOWS_STIFF)
                _d.GetMaxForceAttr().Set(BELLOWS_MAXF)
                _nb += 1
    print(f"  {name}: 다리 {len(jd['legs'])} · 휠 {len(jd['wheels'])} · "
          f"중앙 관절 {len(jd['bend'])}(드라이브 {_nb}축) · "
          f"용접기 {len(jd['weld'])} — 구조로 찾음")
    print(f"  {name}: 관절 강성 {BELLOWS_STIFF} N·m/도 (상한 {BELLOWS_MAXF}), "
          f"다리 스트로크 {PISTON_STROKE * 1000:.0f}mm 예압 {PISTON_MAXF:.0f}N "
          f"→ 휠 중심 도달 "
          f"{TUNE['wheel_center_mm'] + PISTON_STROKE * 1000:.1f}mm")

    _nh = 0
    for _c in Usd.PrimRange(prim):
        if _c.GetTypeName() in ("Cylinder", "Cone") \
                and _c.HasAPI(UsdPhysics.CollisionAPI):
            cyl_to_hull(_c)
            _nh += 1
    if _nh:
        print(f"  {name}: 실린더·원뿔 콜라이더 {_nh}개 → convexHull 메시로 교체")

    _s0 = START_S_RED if name == "reducer" else START_S
    i0 = int(np.argmin(np.abs(path_cl.s - _s0)))
    p0 = path_cl.p[i0]
    tan = path_cl.tangent(i0)
    # 배치는 각 링크 로컬 변환에 굽는다(부모 Xform·root 에 걸면 PhysX 가 발산).
    # 🔑 **출발 자세를 접선에서 만든다 — 수직 구간에서도 출발할 수 있다.**
    # 🚨 **전방 축을 자산에서 잰다.** 벨로우즈는 로컬 −Z 가 앞이고 용접기 v2 는
    #    +X 다. `rot(-90,(0,1,0))` 을 상수로 박아 두면 자산을 갈아끼우는 순간
    #    로봇이 관을 가로질러 놓인다 → 두 세그먼트 바디의 로컬 위치 차이를
    #    그대로 전방 벡터로 쓴다(이름 규약과 무관).
    _rear, _front = seg_body_prims(root, jd)
    _fw = np.array(
        wlocal(_front) - wlocal(_rear), dtype=np.float64)
    _fw = _fw / max(np.linalg.norm(_fw), 1e-12)
    _to_x = Gf.Matrix4d(1.0)
    _to_x.SetRotate(Gf.Rotation(Gf.Vec3d(*[float(v) for v in _fw]),
                                Gf.Vec3d(1, 0, 0)))
    _into = Gf.Matrix4d(1.0)
    _into.SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0),
                                Gf.Vec3d(*[float(v) for v in tan])))
    # 🚨 **출발 롤을 명시한다.** 접선이 +X 와 정반대면 `Gf.Rotation(from,to)` 가
    #    **임의의 수직축**을 고르므로 다리 클럭 위치가 실행마다 우연히 정해진다.
    #    다리는 120° 간격이라 개구와의 정렬이 통과 여부를 가를 수 있다 →
    #    `ROLL_DEG` 로 못박고 진단 때 쓸어 본다.
    place = _to_x * rot(ROLL_DEG, (1, 0, 0)) * _into \
        * trans(float(p0[0]), float(p0[1]), float(p0[2]))
    # 🚨 **직계 자식만 돌면 안 된다** — 용접기 v2 는 중앙 링크가
    #    `Robot/Bellows/Link0..3` 로 한 단 더 들어가 있다. 직계만 옮기면 중앙
    #    4링크가 원점에 남아 아티큘레이션이 통째로 찢어진다.
    #    중간 Xform(`Bellows`)이 항등이라 손자에도 같은 식이 그대로 성립한다 —
    #    항등이 아니면 경고를 찍는다(추측으로 넘어가지 않는다).
    _nl = 0
    for child in Usd.PrimRange(root):
        if not child.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        par = child.GetParent()
        while par.IsValid() and par != root:
            _pm2 = UsdGeom.Xformable(par).GetLocalTransformation()
            if float(np.abs(np.array(_pm2) - np.eye(4)).max()) > 1e-9:
                print(f"[경고] {name}: 중간 Xform {par.GetName()} 이 항등이 "
                      f"아니다 — 배치가 틀어질 수 있다")
            par = par.GetParent()
        xf = UsdGeom.Xformable(child)
        local = xf.GetLocalTransformation()
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local * place)
        _nl += 1
    print(f"  {name}: 링크 {_nl}개 배치 — 전방 축 로컬 "
          f"({_fw[0]:+.2f}, {_fw[1]:+.2f}, {_fw[2]:+.2f})")

    # ── 카메라 하우징·조명 (센서 프림은 reset 뒤에) ────────────────
    for nm, seg, x, fwd in (CAM_SPECS if CAMERAS else []):
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

    return {"name": name, "path": rp, "jd": jd, "fw": _fw,
            "cl": path_cl, "i0": i0,
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
# 🚨 구동 조인트와 서스펜션은 **이름이 같다**(`RearBody_A0` 이 양쪽에 있다).
#    그래서 `discover()` 가 구조로 골라 둔 것을 그대로 쓴다.
for _r in robots:
    for _p in _r["jd"]["wheels"]:
        _d = UsdPhysics.DriveAPI.Get(_p, "angular")
        if _d:
            _d.CreateMaxForceAttr(WHEEL_MAXFORCE)
            if WHEEL_DAMP > 0:
                _d.CreateDampingAttr(WHEEL_DAMP)
            _n_wf += 1
print(f"[준비] 휠 토크 한계 {WHEEL_MAXFORCE:.4f} N·m × {_n_wf}개 "
      f"(휠 반경 {WHEEL_R * 1000:.0f}mm) → 로봇 1대당 합산 견인력 "
      f"{12 * WHEEL_MAXFORCE / WHEEL_R:.1f}N, 다리 "
      f"{len(robots[0]['jd']['legs'])}개 마찰 "
      f"{FRICTION_STATIC * len(robots[0]['jd']['legs']) * PISTON_MAXF:.1f}N "
      f"(여유 "
      f"{12 * WHEEL_MAXFORCE / WHEEL_R / (FRICTION_STATIC * len(robots[0]['jd']['legs']) * PISTON_MAXF):.1f}배)")

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
               if _p.GetName().endswith("_hull")
               and _p.HasAPI(UsdPhysics.CollisionAPI))
print(f"[준비] 주행 {TARGET_SPEED_MPS * 1000:.0f} mm/s "
      f"(한 스텝 {TARGET_SPEED_MPS / PHYSICS_HZ * 1000:.3f}mm) → "
      f"contactOffset {CONTACT_OFFSET * 1000:.2f}mm, 프림 {_n_off}개")
print(f"         (배관 프림에만 건다 — 로봇 콜라이더는 usda 값 그대로. "
      f"교체한 convexHull 콜라이더 {_n_wheel}개는 손대지 않았다)")

tick("로봇 조립 완료 (콜라이더 교체·배치 포함)")
for r in robots:
    r["art"] = SingleArticulation(prim_path=r["path"], name=f"bot_{r['name']}")
    world.scene.add(r["art"])
world.reset()
tick("world.reset() 완료 — 메시 콜라이더 cook")

for r in robots:
    dof = list(r["art"].dof_names or [])
    types = [str(t).upper()
             for t in r["art"]._articulation_view.get_dof_types()]
    jd = r["jd"]
    # 🚨 **DOF 는 이름만으로 고를 수 없다** — 구동(회전)과 서스펜션(직동)이
    #    같은 이름을 쓴다(`RearBody_A0` 이 `DriveJoints` 와 `SpringJoints`
    #    양쪽에 있다). Isaac 이 뒤에 오는 쪽에 `_0` 을 붙이는데 **어느 쪽이
    #    붙는지는 보장이 없다.** → 이름(접미사·D6 축 접미사 무시) **AND
    #    DOF 종류**로 고른다. D6 조인트는 `J0:rotY` 처럼 축이 붙어 온다.
    # 🚨 **`_0` 을 무조건 떼면 안 된다.** 용접기 v2 는 이름이 겹쳐서 Isaac 이
    #    뒤쪽에 `_0` 을 붙이므로 떼야 하지만, 벨로우즈는 `seg0_wheel_0_0` 처럼
    #    **원래 `_0` 으로 끝나는 이름**이라 떼면 매칭이 깨진다(휠 6/12, 다리
    #    4/6, 중앙 3/4 로 조용히 절반만 잡혔다 — 검증이 잡아냈다).
    #    → **원래 이름을 먼저 보고, 그때만 `_0` 을 떼어 다시 본다.**
    def _base(n, names):
        b = n.split(":")[0]
        return b if b in names else b.removesuffix("_0")

    _wn = {j.GetName() for j in jd["wheels"]}
    _ln = {j.GetName() for j in jd["legs"]}
    _bn = {j.GetName() for j in jd["bend"]}
    r["wheel"] = [k for k, n in enumerate(dof)
                  if _base(n, _wn) in _wn and "ROTATION" in types[k]]
    r["piston"] = {}
    _seen = {}
    for k, n in enumerate(dof):
        if _base(n, _ln) not in _ln or "TRANSLATION" not in types[k]:
            continue
        sg = jd["leg_seg"].get(_base(n, _ln), 0)
        _seen[sg] = _seen.get(sg, 0) + 1
        r["piston"][(sg, _seen[sg] - 1)] = k
    r["bel"] = [k for k, n in enumerate(dof) if _base(n, _bn) in _bn]
    # 🚨 다리 개수는 자산마다 다르다 (벨로우즈 6 / 용접기 v2 **12**) —
    #    상수 6 으로 검사했다가 멀쩡한 매핑을 실패로 판정했다. 휠만 12 고정.
    if len(r["wheel"]) != 12 or len(r["piston"]) != len(jd["legs"]) \
            or not r["bel"]:
        raise SystemExit(
            f"[중단] {r['name']} DOF 매핑 실패 — 휠 {len(r['wheel'])}(12) / "
            f"다리 {len(r['piston'])}({len(jd['legs'])}) / "
            f"중앙 {len(r['bel'])}. DOF: {dof}")
    # 🔑 다리별 **시계각** — 자산의 조인트 부착 위치에서 읽는다(실측 0/120/240°,
    #    축방향 두 줄 ±24mm, 반경 40mm). 어느 방향 벽을 잃었는지 알아야
    #    그쪽으로 몰 수 있다. **도면이 아니라 로봇 제원이다.**
    r["leg_clock"] = {}
    for _j in jd["legs"]:
        _p0 = _j.GetAttribute("physics:localPos0").Get()
        for (s2, _i), k in r["piston"].items():
            pass
        _nm = _j.GetName()
        for k2, n2 in enumerate(dof):
            if _base(n2, _ln) == _nm and "TRANSLATION" in types[k2]:
                r["leg_clock"][k2] = math.degrees(
                    math.atan2(float(_p0[2]), float(_p0[1]))) % 360.0
    # 조향용 — 중앙 관절의 피치/요 DOF 를 갈라 둔다 (D6 는 `J0:1`·`J0:2`)
    _ax = TUNE.get("steer")
    if _ax and STEER_ON:
        r["bel_pitch"] = [k for k, n in enumerate(dof)
                          if _base(n, _bn) in _bn and n.endswith(":" + _ax[0])]
        r["bel_yaw"] = [k for k, n in enumerate(dof)
                        if _base(n, _bn) in _bn and n.endswith(":" + _ax[1])]
        if not (r["bel_pitch"] and r["bel_yaw"]):
            raise SystemExit(
                f"[중단] {r['name']} 조향 축 매핑 실패 — 피치 "
                f"{len(r.get('bel_pitch', []))} / 요 {len(r.get('bel_yaw', []))}"
                f". 중앙 DOF 이름: {[dof[k] for k in r['bel']]}")
    else:
        r["bel_pitch"] = r["bel_yaw"] = []
    print(f"  {r['name']}: DOF {len(dof)}개 중 휠 12 · 다리 "
          f"{len(r['piston'])} · 중앙 {len(r['bel'])}"
          + (f" (조향 피치 {len(r['bel_pitch'])} / 요 {len(r['bel_yaw'])})"
             if r["bel_pitch"] else " (조향 없음)") + " 매핑 완료")
    _rear, _front = seg_body_prims(stage.GetPrimAtPath(r["path"]), jd)
    r["seg0"], r["seg1"] = _rear, _front
    # 끼임 진단용 — 휠 링크 프림을 조인트에서 거꾸로 찾는다(이름 규약 무관)
    r["wheel_prims"] = []
    for _j in jd["wheels"]:
        _t = _j.GetRelationship("physics:body1").GetTargets()
        if _t:
            r["wheel_prims"].append(stage.GetPrimAtPath(_t[0]))
    r["leg_prims"] = []
    for _j in jd["legs"]:
        _t0 = _j.GetRelationship("physics:body0").GetTargets()
        r["leg_prims"].append(_t0[0].name if _t0 else "?")
    # 로봇 로컬 정규직교 프레임 — 전방을 x̂ 로 놓는다. 자산마다 전방 축이
    # 다르므로(−Z / +X) 여기서 한 번 만들어 두고 조향은 이 프레임에서만 잰다.
    _x = np.asarray(r["fw"], dtype=np.float64)
    _t = np.array([0.0, 0.0, 1.0]) if abs(_x[2]) < 0.9 \
        else np.array([1.0, 0.0, 0.0])
    _y = np.cross(_t, _x)
    _y /= max(np.linalg.norm(_y), 1e-12)
    r["fr"] = np.vstack([_x, _y, np.cross(_x, _y)])   # 행 = x̂, ŷ, ẑ

for r in robots:          # 다리를 벽에 닿은 자세로 놓는다 (첫 스텝 전)
    if r["piston"] and PISTON_INIT > 0:
        _q = np.array(r["art"].get_joint_positions())
        _q[list(r["piston"].values())] = PISTON_INIT
        r["art"].set_joint_positions(_q)
print(f"[준비] 다리 초기 신장 {PISTON_INIT * 1000:.2f}mm — 벽에 닿은 자세로 시작 "
      f"(휠 중심 {TUNE['wheel_center_mm'] + PISTON_INIT * 1000:.1f}mm, "
      f"관벽 {(PIPE_IR - WHEEL_R) * 1000:.1f}mm)")

# 카메라 프림은 **reset 뒤에** 만든다.
if not CAMERAS:
    print("[준비] 카메라 없음 — 이 연습장은 영상을 안 쓴다. "
          "필요하면 `--cam` (GUI 렌더 비용이 크게 는다)")
try:
    if not CAMERAS:
        raise ImportError("카메라 비활성")
    from isaacsim.sensors.camera import Camera             # noqa: E402
    _SQ = 0.70710678
    _ncam = 0
    for r in robots:
        for nm, seg, x, fwd in CAM_SPECS:
            # 🚨 vision 모드는 **전방 카메라만** 만든다 (2026-08-07 사용자
            #    지적 "카메라 다 켜서 렌더링 터진다"). 후방은 아무도 안 읽는데
            #    render product 가 생기는 순간 매 렌더마다 RTX 를 태운다 —
            #    4대면 카메라 8대 중 4대가 순수 낭비다.
            if NAV == "vision" and not fwd:
                continue
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
            if fwd:
                # 🚨 깊이는 어노테이터로 받는다(기록된 함정: writer 를 같은
                #    render product 에 붙이면 rgb/depth 가 깨진다).
                try:
                    cam.add_distance_to_camera_to_frame()
                except Exception as exc:
                    print(f"[경고] {r['name']} depth 어노테이터 실패({exc})")
                r["cam_front"] = cam
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
except ImportError:
    pass
except Exception as exc:
    print(f"[경고] 카메라 초기화 실패 ({exc}) — 하우징·조명만 남는다")

tick("카메라·센서 준비 완료")
# ── 자율주행 모듈 배선 (NAV=vision) ─────────────────────────────────
# 🔑 **연습장이 판단을 만들지 않는다.** `src/son` 의 정본 모듈이 판단한다:
#      condition/detector.py   깊이 → 관 상태(개방도·입사각·오프셋)
#      condition/odometry.py   영상 → **시각 속도** (휠이 헛돌아도 진짜 전진량)
#      driver/control.py       위 둘 + 휠속도 → 속도 지령 · 끼임 판정 · FSM
if NAV == "vision":
    sys.path.insert(0, str(SON))
    from condition.detector import PipeConditionDetector      # noqa: E402
    from condition.odometry import VisualOdometry             # noqa: E402
    from driver.control import DriveController                # noqa: E402
    _INTR = dict(fx=F_PX, fy=F_PX, ppx=CAM_W / 2.0, ppy=CAM_H / 2.0,
                 f_fish=F_PX)
    for r in robots:
        r["det"] = PipeConditionDetector(_INTR)
        r["vo"] = VisualOdometry(f_px=F_PX)
        # BRANCH_RULE 노브 — 오른손 법칙 진단용 (right/all/none)
        # CENTER_GAIN / CENTER_GAIN_CURVE — 센터링 조향 진단용 (0 = 끔)
        _ctl_kn = dict(branch_rule=os.environ.get("BRANCH_RULE", "right"))
        for _env, _key in (("CENTER_GAIN", "center_gain"),
                           ("CENTER_GAIN_CURVE", "center_gain_curve")):
            if os.environ.get(_env) is not None:
                _ctl_kn[_key] = float(os.environ[_env])
        r["ctl"] = DriveController(_ctl_kn)
        r["cond"] = None
        r["vis_mps"] = None
        r["vis_t"] = 0.0
        r["started"] = False
    print(f"[항법] vision — 판단을 `condition/` + `driver/` 정본 모듈에 맡긴다 "
          f"(카메라 {len(robots)}대 사용)")

_XC = UsdGeom.XformCache()


def wpos(prim):
    _XC.Clear()
    t = _XC.GetLocalToWorldTransform(prim).ExtractTranslation()
    return np.array([float(t[0]), float(t[1]), float(t[2])])


def _set_pos(r, values, indices):
    """관절 위치 목표를 **바로** 쓴다.

    🚨 `apply_action` 을 한 스텝에 여러 번 부르면 **마지막 것만 남는다** —
       다리·휠을 각각 부르면 휠만 먹고 나머지는 조용히 사라진다(실측).
    """
    a = r["art"]
    v = np.array(values, dtype=np.float32)
    i = np.array(indices)
    try:
        a._articulation_view.set_joint_position_targets(v.reshape(1, -1),
                                                        joint_indices=i)
    except Exception:
        a.apply_action(ArticulationAction(joint_positions=v, joint_indices=i))


def wrot(prim):
    """프림의 월드 자세 — **열이 기저**인 3×3 (world = C @ v_local).

    🚨 USD 는 행벡터 규약이라 `Gf.Matrix4d` 의 위 3×3 은 **행**이 기저다.
       그대로 쓰면 회전이 전치돼 조향 부호가 뒤집힌다.
    """
    _XC.Clear()
    a = np.array(_XC.GetLocalToWorldTransform(prim),
                 dtype=np.float64).reshape(4, 4)[:3, :3]
    a = a / np.maximum(np.linalg.norm(a, axis=1, keepdims=True), 1e-12)
    return a.T


def _set_eff(r, values, indices):
    """관절 힘 지령. 🚨 위치/속도 지령과 **다른 채널**이라 서로 안 지운다."""
    a = r["art"]
    v = np.array(values, dtype=np.float32)
    i = np.array(indices)
    try:
        a._articulation_view.set_joint_efforts(v.reshape(1, -1),
                                               joint_indices=i)
        return True
    except Exception as exc:
        if not r.get("eff_warned"):
            r["eff_warned"] = True
            print(f"[경고] {r['name']}: 힘 지령 실패({exc}) — 위치 모드로 돈다")
        return False


def force_legs(r):
    """다리 힘 지령 — 예압 + 중심 복원 + 뜬 다리 접기. 위치 목표를 안 쓴다."""
    if not r.get("piston"):
        return [], []
    q = np.asarray(r["art"].get_joint_positions())
    vel = np.asarray(r["art"].get_joint_velocities())
    free = r.setdefault("leg_free", {})
    idx, val, nfree = [], [], 0
    for sg in sorted({s2 for s2, _i in r["piston"]}):
        ks = [k for (s2, _i), k in sorted(r["piston"].items()) if s2 == sg]
        # 서 있는 다리 = 벽에 닿은 다리 → 그것들의 평균이 관벽 추정치
        stop = [k for k in ks if abs(float(vel[k])) < LEG_STOP_V]
        ref = float(np.mean(q[stop if stop else ks]))
        for k in ks:
            out = float(vel[k]) > LEG_FREE_V and q[k] > ref + LEG_REF_GAP
            if out or q[k] >= PISTON_STROKE - LEG_FREE_MARGIN:
                free[k] = True
            elif q[k] < ref + 0.5 * LEG_REF_GAP:
                free[k] = False
        cont = [k for k in ks if not free.get(k)]
        if not cont:                      # 전부 허공이면 접지 말고 벽을 찾는다
            cont = ks
            for k in ks:
                free[k] = False
        m = float(np.mean(q[cont]))
        for k in ks:
            idx.append(k)
            if free.get(k):
                val.append(-LEG_FOLD_N)
                nfree += 1
            else:
                val.append(PISTON_MAXF + LEG_KC * (m - float(q[k])))
    r["leg_free_n"] = nfree
    r["leg_free_max"] = max(r.get("leg_free_max", 0), nfree)
    r["leg_spread"] = float(np.ptp(q[list(r["piston"].values())]))
    return idx, val


def preload_legs(r):
    """다리 목표를 **스트로크 밖**에 둔다 = 상수력 예압 (안착 구간용).

    🚨 **`set_joint_positions()` 는 PhysX 런타임 드라이브 타깃을 지운다**
       (기록된 함정). 다리 초기 신장을 넣는 그 한 줄 때문에 예압이 0 이 되어
       **로봇이 벽을 아예 안 짚었다.** 수평 코스는 중력이 아래 벽에 눌러 줘서
       증상이 가려졌고, **수직관에서만** 드러났다 — 안착 중 자유낙하의 86%
       속도로 떨어지고 휠 각속도가 26°/s(굴렀다면 7,160°/s).
    """
    if not r.get("piston"):
        return [], []
    ks = list(r["piston"].values())
    return ks, [PISTON_STROKE * 2.0] * len(ks)


def leg_efforts(r):
    """관절이 실제로 내고 있는 힘(N) — 다리별 벽 접촉 판정용.

    🚨 Isaac 버전마다 이름이 달라 **있는 것을 찾아 쓰고, 없으면 판정을 끈다**
       (조용히 틀린 값을 쓰느니 안 쓰는 게 낫다). 처음 한 번 어느 API 를
       썼는지 로그로 남긴다.
    """
    if r.get("eff_api") == "none":
        return None
    av = r["art"]._articulation_view
    names = ([r["eff_api"]] if r.get("eff_api")
             else ["get_measured_joint_efforts", "get_applied_joint_efforts",
                   "get_measured_joint_forces"])
    for nm in names:
        fn = getattr(av, nm, None)
        if fn is None:
            continue
        try:
            e = np.asarray(fn())
        except Exception:
            continue
        if e is None or e.size == 0:
            continue
        e = e.reshape(-1) if e.ndim == 1 else e[0].reshape(-1)
        if e.shape[0] < max(r["piston"].values()) + 1:
            continue
        if not r.get("eff_api"):
            r["eff_api"] = nm
            _s = ", ".join(f"{float(e[k]):+.2f}"
                           for k in sorted(r["piston"].values()))
            print(f"  {r['name']}: 다리 힘 판정에 `{nm}` 사용 — 초기값(N) "
                  f"[{_s}]")
        return e
    r["eff_api"] = "none"
    print(f"[경고] {r['name']}: 관절 힘을 읽을 API 가 없다 — "
          f"다리별 접촉 판정을 끈다")
    return None


def center_legs(r):
    """세그먼트별로 다리 신장을 맞춘다 = 몸체를 관 중앙으로.

    🔑 **벽이 없는 다리는 따로 다룬다** — 평균에서 빼고 접는다. 안 그러면
       구멍 위에서 끝까지 뻗은 값이 평균을 끌어올려 **나머지 다리가 몸을
       구멍 쪽으로 민다**(사용자 표현: 왼발차기로 벽을 밀어야 하는데,
       지금은 허공을 짚는 발까지 똑같이 밀라고 하고 있었다).
    🚨 한 세그먼트의 다리가 **전부** 벽이 없으면 접으면 안 된다 — 주저앉는다.
       그때는 전부 밀어 벽을 찾는다.
    """
    if not r.get("piston"):
        return [], []
    q = np.asarray(r["art"].get_joint_positions())
    eff = leg_efforts(r)
    free = r.setdefault("leg_free", {})
    idx, val, nfree = [], [], 0
    for sg in sorted({s for s, _i in r["piston"]}):
        ks = [k for (s2, _i), k in sorted(r["piston"].items()) if s2 == sg]
        # 🚨 **중앙값 기준으로 미리 잡으려던 것은 되돌렸다** (실측 악화:
        #    `tee_in` 끼임 2→3회). 접합부에서는 한 세그먼트의 다리 **대부분**이
        #    벽을 잃어 중앙값 자체가 뜬 다리 쪽으로 끌려가므로 기준이 못 된다.
        #    다수결이 성립하지 않는 곳이라 중앙값은 원리적으로 안 맞는다.
        if LEG_FREE_ON:
            for k in ks:
                if q[k] >= PISTON_STROKE - LEG_FREE_MARGIN:
                    free[k] = True
                elif q[k] < PISTON_STROKE - LEG_FREE_CLEAR:
                    free[k] = False
        cont = [k for k in ks if not free.get(k)]
        if not cont:                      # 짚을 벽이 하나도 없다 → 전부 민다
            cont = ks
            for k in ks:
                free[k] = False
        m = float(np.mean(q[cont])) + CENTER_DELTA
        for k in ks:
            idx.append(k)
            if free.get(k):
                val.append(LEG_FOLD)
                nfree += 1
            else:
                val.append(m)
    r["leg_free_n"] = nfree
    r["leg_free_max"] = max(r.get("leg_free_max", 0), nfree)
    r["leg_spread"] = float(np.ptp(q[list(r["piston"].values())]))
    return idx, val


def diag_stuck(r):
    """끼임 순간의 몸통·다리·휠 상태를 찍는다.

    끼임의 원인을 **가른다**: 다리가 끝까지 뻗어 있으면 밀 벽이 없는 것
    (개구로 빠짐)이고, 다리가 눌려 있으면 벽에 물린 것이다. 휠 반경은
    관 중심선에서 얼마나 떨어졌는가 — 관벽(50 − 휠 8 = 42mm)이 기준이다.
    """
    cl = r["cl"]
    q = np.asarray(r["art"].get_joint_positions())
    p0, p1 = wpos(r["seg0"]), wpos(r["seg1"])
    s0, o0, _ = cl.nearest(p0)
    s1, o1, _ = cl.nearest(p1)
    print(f"           [진단] 뒤 s={s0 * 1000:.0f} 이탈{o0 * 1000:5.1f} "
          f"월드({p0[0] * 1000:+.0f},{p0[1] * 1000:+.0f},{p0[2] * 1000:+.0f}) | "
          f"앞 s={s1 * 1000:.0f} 이탈{o1 * 1000:5.1f} "
          f"월드({p1[0] * 1000:+.0f},{p1[1] * 1000:+.0f},{p1[2] * 1000:+.0f})")
    for sg in sorted({s for s, _i in r["piston"]}):
        ks = [k for (s2, _i), k in sorted(r["piston"].items()) if s2 == sg]
        ext = ", ".join(f"{q[k] * 1000:5.1f}" for k in ks)
        _fr = "".join("F" if r.get("leg_free", {}).get(k) else "."
                      for k in ks)
        print(f"           [진단] seg{sg} 접촉 [{_fr}] (F=벽없음)")
        print(f"           [진단] seg{sg} 다리 신장(mm, 상한 "
              f"{PISTON_STROKE * 1000:.0f}) [{ext}]")
    rad = []
    for wp in r["wheel_prims"]:
        _s, _o, _ = cl.nearest(wpos(wp))
        rad.append(_o * 1000)
    print(f"           [진단] 휠 중심선 거리(mm, 관벽 "
          f"{(PIPE_IR - WHEEL_R) * 1000:.0f}) "
          f"[{', '.join(f'{v:4.1f}' for v in rad)}]")
    print(f"           [진단] 관절합 P{r.get('q_pitch', 0):+.0f}"
          f"/Y{r.get('q_yaw', 0):+.0f}  지령 "
          f"{r.get('cmd_pitch', 0):+.1f}/{r.get('cmd_yaw', 0):+.1f}×4  "
          f"목표 P{r.get('want', (0, 0))[0]:+.0f}/Y{r.get('want', (0, 0))[1]:+.0f}"
          f"  휠 각속도 실측/지령 {r.get('gov_meas', 0) * 100:.0f}%")


# 분기 방위를 시야에서 잃은 뒤에도 이만큼은 그 방향을 유지한다(진입 중에는
# 개구가 화면 밖으로 벗어난다).
BRANCH_HOLD_S = float(os.environ.get("BRANCH_HOLD_S", 3.0))


def steer_vision(r):
    """**검출기의 분기 방위(`branch_deg`)로 조향한다** (2026-08-07 배선).

    🔑 화소 방위 → 로봇 몸 프레임 변환이 전부다. 전방 카메라 축은
       X_cam = −Y_robot, Y_cam = +Z_robot 이고 **화소 y 는 아래로 증가**하므로
       화면 방위 φ 에 보이는 개구는 로봇 (y,z) 평면에서
           y_r = −cos φ,  z_r = −sin φ    →   θ_robot = φ + 180°
    🔑 진입을 시작하면 개구가 화각 밖으로 벗어나 BRANCH 가 끊긴다 —
       마지막 방위를 `BRANCH_HOLD_S` 동안 유지해 마저 꺾는다.
    🚨 개구를 본 적이 없으면 지령 0 — 목표를 현재 각도로 두어 관절 스프링이
       벽이 만든 꺾임과 싸우지 않게 한다(`steer_onboard` 와 같은 규약).
    """
    if not r.get("bel_pitch"):
        return [], []
    q = np.asarray(r["art"].get_joint_positions())
    # 🎯 2026-08-07 — 조향은 **`DriveController` 의 분기 진입 기동 지시**를
    #    실행할 뿐이다. 여기서 판단하지 않는다(비례 조향은 실측 기각 —
    #    구멍으로 통째로 빠졌다). 컨트롤러가 방위를 래치하고 호 길이로
    #    기동을 끝내므로, 실행기는 화소 방위 → 몸 프레임 변환만 한다.
    mag, clock = r.get("ctl_steer", (0.0, 0.0))
    # 🚨 **직진 장면에서는 관절을 서서히 편다** (2026-08-07 사용자 스크린샷
    #    보고: *"일자관인데 계속 웅크려 있다"*). 지령 0 = "현재 각 유지"
    #    규약은 곡관에서 벽과 안 싸우려는 것인데, T 진입이 만든 40° 굽힘이
    #    직관에서도 영구 잔류하는 부작용이 있었다. 판정이 곧은 관(NORMAL·
    #    분기 미미·입사각 작음)일 때만 초당 RELAX_RATE 로 0° 를 향해 편다 —
    #    곡관·분기·기동 중에는 기존 규약 그대로.
    _c = r.get("cond")
    # MISALIGNMENT 도 편다 — 정렬 이탈이야말로 몸을 곧게 펴야 고쳐지는
    # 장면이다(실측: 안착 요동 굽힘 10° 가 유지되며 시야가 비뚤어져
    # MISALIGNMENT 가 영영 안 풀리던 reducer 출발 정체).
    # 재출발 유예(straighten_n) 중에는 판정과 무관하게 편다 — 접합부 앞이라
    # 판정이 BRANCH 라서 안 펴지는 것을 뚫는 것이 유예의 목적이다.
    # RECOVER 중에도 편다 — 굽힘을 유지한 채 흔들면 모서리에서 양방향 쐐기
    # (실측 s=178 동결). 회복 = 몸을 펴고 살살 빠져나오는 동작.
    if (r.get("straighten_n", 0) > 0
            or r.get("ctl_state") == "RECOVER"
            or (mag <= 0.0 and _c is not None
                and _c.state in ("NORMAL", "MISALIGNMENT")
                and _c.branch_ratio < 0.08
                and abs(_c.incidence_deg) < 12.0)):
        _lim = math.radians(RELAX_RATE) * PHYSICS_DT
        # 누적 조향 지령도 같이 0 으로 되감는다 — 안 그러면 완화가 끝난 뒤
        # 첫 조향 프레임에 묵은 지령(target = q + cmd)이 튀어 다시 굽힌다.
        _dl = RELAX_RATE * PHYSICS_DT
        for _k in ("cmd_pitch", "cmd_yaw"):
            _v = r.get(_k, 0.0)
            r[_k] = _v - max(-_dl, min(_dl, _v))
        idx = r["bel_pitch"] + r["bel_yaw"]
        return (idx,
                [float(q[k]) - max(-_lim, min(_lim, float(q[k])))
                 for k in idx])
    th = math.radians(clock + 180.0)      # 전방 카메라 장착 변환 (φ+180°)
    np_, ny = len(r["bel_pitch"]), len(r["bel_yaw"])
    tp = float(np.clip(-mag * math.sin(th) / np_, -STEER_MAX, STEER_MAX))
    ty = float(np.clip(+mag * math.cos(th) / ny, -STEER_MAX, STEER_MAX))
    lim = STEER_RATE * PHYSICS_DT
    cp, cy = r.get("cmd_pitch", 0.0), r.get("cmd_yaw", 0.0)
    cp += max(-lim, min(lim, tp - cp))
    cy += max(-lim, min(lim, ty - cy))
    r["cmd_pitch"], r["cmd_yaw"] = cp, cy
    r["steer_err"] = mag
    r["steer_max"] = max(r.get("steer_max", 0.0), mag)
    return (r["bel_pitch"] + r["bel_yaw"],
            [float(q[k]) + math.radians(cp) for k in r["bel_pitch"]]
            + [float(q[k]) + math.radians(cy) for k in r["bel_yaw"]])


def steer_onboard(r):
    """**도면 없이** 조향한다 — 앞 세그먼트가 벽을 잃은 방향으로 몬다.

    🔑 쓰는 신호는 전부 로봇 것이다: 다리별 접촉(힘/속도로 판정) + 다리
       시계각(로봇 제원) + 중앙 관절 엔코더. 중심선을 안 본다.
    🔑 분기에서 개구는 **벽이 사라진 방향**으로 나타난다. 그쪽으로 몸을 몰면
       가지로 들어간다(사용자 표현: 왼발차기).
    🚨 벽을 잃은 다리가 없으면 **목표를 현재 각도로 둔다** — 관절 스프링이
       벽이 만든 꺾임과 싸우지 않게 한다(강성 60 은 조향용이라 세다).
    """
    if not r.get("bel_pitch"):
        return [], []
    q = np.asarray(r["art"].get_joint_positions())
    free = r.get("leg_free", {})
    fs = max(sg for sg, _i in r["piston"])          # 앞 세그먼트
    ths = [r["leg_clock"].get(k, 0.0)
           for (sg, _i), k in r["piston"].items() if sg == fs and free.get(k)]
    if ths:
        _c = float(np.mean(np.cos(np.radians(ths))))
        _s = float(np.mean(np.sin(np.radians(ths))))
        th = math.atan2(_s, _c)
        mag = STEER_ONBOARD_DEG * min(len(ths) / 2.0, 1.0)
    else:
        th, mag = 0.0, 0.0
    np_, ny = len(r["bel_pitch"]), len(r["bel_yaw"])
    # 개구 방향 θ 로 코를 돌린다: yaw ∝ +cosθ, pitch ∝ −sinθ
    tp = float(np.clip(-mag * math.sin(th) / np_, -STEER_MAX, STEER_MAX))
    ty = float(np.clip(+mag * math.cos(th) / ny, -STEER_MAX, STEER_MAX))
    lim = STEER_RATE * PHYSICS_DT
    cp, cy = r.get("cmd_pitch", 0.0), r.get("cmd_yaw", 0.0)
    cp += max(-lim, min(lim, tp - cp))
    cy += max(-lim, min(lim, ty - cy))
    r["cmd_pitch"], r["cmd_yaw"] = cp, cy
    r["steer_err"] = mag
    r["steer_max"] = max(r.get("steer_max", 0.0), mag)
    r["want"] = (math.degrees(-math.sin(th)) * 0, mag)
    # 목표 = **현재 각도 + 지령** → 지령이 0 이면 벽이 만든 꺾임을 안 거스른다
    return (r["bel_pitch"] + r["bel_yaw"],
            [float(q[k]) + math.radians(cp) for k in r["bel_pitch"]]
            + [float(q[k]) + math.radians(cy) for k in r["bel_yaw"]])


def steer(r, i_now):
    """중앙 관절로 **중심선을 따라간다** — 앞을 내다보고 헤딩 오차를 닫는다.

    🔑 곡관은 벽이 꺾어 주지만 **분기는 직진로가 열려 있어 안 꺾인다.**
       가지로 들어가려면 관절에 목표각을 직접 줘야 한다.
    🚨 목표각은 **초당 각도로 제한**한다 — 한 번에 던지면 발산한다(기록된 사고).
    🚨 관절 위치 목표는 `center_legs` 것과 **한 번에 같이** 써야 한다.
       나눠 쓰면 뒤에 쓴 것만 남는다(기록된 `apply_action` 함정).
    """
    if not r.get("bel_pitch"):
        return [], []
    cl = r["cl"]
    # 🚨 **뒤 세그먼트의 헤딩 오차로 닫으면 안 된다** — 관절을 꺾어도 뒤
    #    세그먼트는 안 돌아가므로 오차가 영영 안 닫히고 목표각이 포화된다
    #    (실측: 직관에서 조향오차 79°, 관절 33° 꺾임). 관절이 실제로 만드는
    #    각은 **두 세그먼트 사이에서 경로가 꺾이는 각**이다 → 그것을 준다.
    # 🔑 진행 방향과 무관하다(전진·후진 같은 식) — 몸통 사슬을 관 모양에
    #    맞추는 것이지 진행 방향을 쫓는 것이 아니다.
    p0, p1 = wpos(r["seg0"]), wpos(r["seg1"])
    span = p1 - p0
    s1, _o1, i1 = cl.nearest(p1)
    j = int(np.argmin(np.abs(cl.s - (s1 + STEER_LOOK * r["dir"]))))
    d = cl.tangent(j)
    if float(np.dot(d, span)) < 0.0:          # 접선 부호를 몸통 방향에 맞춘다
        d = -d
    e = r["fr"] @ (wrot(r["seg0"]).T @ d)     # 로봇 전방 정렬 프레임에서 본 목표
    e = e / max(np.linalg.norm(e), 1e-12)
    yaw = math.degrees(math.atan2(e[1], e[0]))
    pitch = math.degrees(math.atan2(-e[2], math.hypot(e[0], e[1])))
    np_, ny = len(r["bel_pitch"]), len(r["bel_yaw"])
    tp = float(np.clip(STEER_KP * pitch / np_, -STEER_MAX, STEER_MAX))
    ty = float(np.clip(STEER_KP * yaw / ny, -STEER_MAX, STEER_MAX))
    lim = STEER_RATE * PHYSICS_DT
    cp, cy = r.get("cmd_pitch", 0.0), r.get("cmd_yaw", 0.0)
    cp += max(-lim, min(lim, tp - cp))
    cy += max(-lim, min(lim, ty - cy))
    r["cmd_pitch"], r["cmd_yaw"] = cp, cy
    # [진단] 실제 몸통 사슬이 어느 쪽으로 꺾였는가 — 목표(e)와 같은 프레임에서
    # 재서 나란히 본다. 축 매핑(`:1`=피치 가정)이 틀리면 여기서 갈린다.
    u = r["fr"] @ (wrot(r["seg0"]).T @ (span / max(np.linalg.norm(span), 1e-12)))
    r["act_yaw"] = math.degrees(math.atan2(u[1], u[0]))
    r["act_pitch"] = math.degrees(math.atan2(-u[2], math.hypot(u[0], u[1])))
    r["want"] = (pitch, yaw)
    # 관절이 지령을 실제로 따라갔는가 (드라이브 힘 부족 ↔ 벽에 눌림을 가른다)
    _q = np.asarray(r["art"].get_joint_positions())
    r["q_pitch"] = math.degrees(float(np.sum(_q[r["bel_pitch"]])))
    r["q_yaw"] = math.degrees(float(np.sum(_q[r["bel_yaw"]])))
    r["steer_err"] = math.hypot(pitch, yaw)
    r["steer_max"] = max(r.get("steer_max", 0.0), r["steer_err"])
    return (r["bel_pitch"] + r["bel_yaw"],
            [math.radians(cp)] * np_ + [math.radians(cy)] * ny)


def curve_speed(r):
    """관절 엔코더 → 목표 속도(m/s). **카메라를 쓰지 않는다.**

    중앙 관절 4개(X·Y·X·Y)를 합성해 잰다 — 수직곡관은 피치축이, 수평곡관은
    요축이 꺾이므로 한 축만 보면 나머지 방향을 못 알아본다.
    """
    if not r.get("bel"):
        return TARGET_SPEED_MPS
    ang = np.degrees(np.abs(
        np.asarray(r["art"].get_joint_positions())[r["bel"]]))
    bend = float(np.linalg.norm(ang))
    f = 1.0 - (1.0 - CURVE_MIN) * min(bend / max(BEND_REF, 1e-6), 1.0)
    r["bend_deg"], r["curve_f"] = bend, f
    r["bend_max"] = max(r.get("bend_max", 0.0), bend)
    return TARGET_SPEED_MPS * f


def leg_speed(r):
    """다리 실린더 → 속도 배율. **관절이 못 보는 분기·관경 변화를 본다.**

    기준선(느린 EMA)에서 평균이 벗어난 양과 다리 편차 중 큰 쪽으로 감속한다.
    둘 다 **로봇이 자기 관절에서 직접 읽는 값**이라 도면이 필요 없다.
    """
    if not LEG_SLOW_ON or not r.get("piston"):
        return 1.0
    q = np.asarray(r["art"].get_joint_positions())
    ks = list(r["piston"].values())
    spread = float(np.ptp(q[ks]))
    mean = float(np.mean(q[ks]))
    ema = r.get("leg_ema", mean)
    r["leg_ema"] = ema + 0.002 * (mean - ema)      # 느린 기준선
    dev = abs(mean - r["leg_ema"])
    # 🚨 **편차(spread)는 쓰지 않는다** (2026-08-07 실측). 정상 직관·곡관에서도
    #    8~18mm 라 임계 8mm 가 상시 발동해 속도가 내내 0.45 로 눌렸고, 리듀서는
    #    왕복 2→1회로 오히려 느려졌다. 분기에서도 정상 대비 분리가 없다.
    # 🔑 **평균의 기준선 이탈은 관경 변화를 40배 차이로 가른다** —
    #    리듀서 테이퍼 4.2~5.8mm vs 그 외 0.1~1.4mm. 이것만 쓴다.
    #    ⚠ 분기는 이 신호로 **안 잡힌다** — 다리가 개구로 뻗지 않기 때문이다.
    u = dev / LEG_DEV_REF
    f = 1.0 - (1.0 - LEG_MIN) * min(u, 1.0)
    r["leg_spread_now"], r["leg_dev"], r["leg_f"] = spread, dev, f
    r["leg_spread_max"] = max(r.get("leg_spread_max", 0.0), spread)
    r["leg_ext_max"] = max(r.get("leg_ext_max", 0.0), float(np.max(q[ks])))
    return f


def ramp(r, want):
    """비대칭 가감속 (설계 0.08 / 0.40 m/s²)."""
    cur = r.get("v_cmd", 0.0)
    lim = (ACC_MPS2 if want > cur else DEC_MPS2) * PHYSICS_DT
    cur += max(-lim, min(lim, want - cur))
    r["v_cmd"] = cur
    return cur


def drive(r, deg_s):
    """휠 각속도 지령 — **실측 각속도 되먹임으로 처짐을 보정한다.**

    🔑 속도 드라이브의 힘은 `댐핑 × (지령 − 실제)` 라 부하가 걸리면 실제가
       지령보다 낮은 자리에서 평형을 잡는다(열린 루프 실측 93%). 배율로 민다.
    🚨 maxForce 에 걸리면 배율을 올려도 소용없다 → 상한 1.5.
    🚨 정지·방향 전환에서 리셋한다(적분 폭주 방지).
    """
    a = r["art"]
    if GOV_ON and abs(deg_s) > 1e-6:
        want = math.radians(abs(deg_s))
        meas = float(np.mean(np.abs(
            np.asarray(a.get_joint_velocities())[r["wheel"]])))
        if r.get("gov_dir") != (deg_s > 0):
            r["gov"], r["gov_dir"] = 1.0, (deg_s > 0)
        r["gov"] = float(np.clip(
            r.get("gov", 1.0) + GOV_KI * ((want - meas) / want) * PHYSICS_DT,
            GOV_MIN, GOV_MAX))
        r["gov_meas"] = meas / want
        r.setdefault("gov_hist", []).append(r["gov_meas"])
        deg_s *= r["gov"]
    elif GOV_ON:
        r["gov"], r["gov_dir"] = 1.0, None
    v = np.array([math.radians(deg_s)] * len(r["wheel"]), dtype=np.float32)
    i = np.array(r["wheel"])
    try:
        a._articulation_view.set_joint_velocity_targets(v.reshape(1, -1),
                                                        joint_indices=i)
    except Exception:
        a.apply_action(ArticulationAction(joint_velocities=v,
                                          joint_indices=i))


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

_t_mark, _step_mark = time.time(), 0
step, was_playing = 0, True
# 🎯 FAIL_S 10→5s (2026-08-07 사용자 지시: *"끼였으면 그 자체로 이미 문제.
#    기다리지 말고 짧게 끊어야 다음 테스트로 개선한다"*). 하한은 의도된
#    정지 구간(출발 HOLD ~3s, 진입 확인 1.5s)보다는 길어야 오탐이 없다.
STUCK_S, REPORT_S = 2.0, 3.0
FAIL_S = float(os.environ.get("FAIL_S", 5.0))
_last_report = 0

tick("첫 물리 스텝 진입 — 여기서부터 로봇이 움직인다")
# 🚨 **NAV=vision 은 헤드리스여도 렌더해야 한다.** 안 그러면 카메라가
#    프레임을 못 만들어 `get_rgba()` 가 빈 배열이고, 판정이 없어 컨트롤러가
#    HOLD 로 멈춘다(실측: v=0 으로 제자리).
# 🎯 **단, 매 스텝 렌더하지 않는다** (2026-08-07 사용자 질문 "왜 3대에
#    0.10배인가"의 답) — 판정·카메라는 10Hz 만 쓰는데 물리 240Hz 마다
#    렌더하면 **필요한 것보다 24배** RTX 를 태운다. 그것이 GUI 0.10×,
#    헤드리스 vision 0.33× 의 정체였다. 렌더는 카메라 주기에만:
#    RENDER_EVERY 스텝마다 한 장. 아래 판정 게이트(PHYSICS_HZ/10)와 같은
#    주기라 렌더 직후 스텝에서 프레임을 읽는다 (신선도 1스텝).
#    GUI 뷰포트도 같은 렌더를 나눠 쓴다 — GUI_EVERY(기본 8 = 30fps)로
#    따로 조절. 뷰포트가 어느 쪽이든 렌더는 합집합으로 돈다.
RENDER_EVERY = max(1, int(round(PHYSICS_HZ / 10.0)))
GUI_EVERY = max(1, int(os.environ.get("GUI_EVERY", 8)))
while True:
    _render = (NAV == "vision" and step % RENDER_EVERY == 0) \
        or (not HEADLESS and step % GUI_EVERY == 0)
    world.step(render=_render)
    step += 1
    if step == 1:
        tick("첫 스텝 완료(GUI 는 여기서 RTX 셰이더를 굽는다)")

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
            # 🚨 안착 중에도 예압을 **매 스텝 다시 써야** 한다 — reset 뒤의
            #    `set_joint_positions()` 가 드라이브 타깃을 지웠기 때문이다.
            if LEG_FORCE:
                _li, _lv = force_legs(r)
                if _li:
                    _set_eff(r, _lv, _li)
            else:
                _li, _lv = preload_legs(r)
                if _li:
                    _set_pos(r, _lv, _li)
            # [진단] 안착 궤적 — 수직관에서 흘러내리는지 가른다.
            #   s 가 **가속**하면 접촉 없이 자유낙하, **등속**이면 굴러 내려가는
            #   것(휠 드라이브가 못 잡는 것), 안 변하면 정상.
            if SETTLE_TRACE and r["t"] % int(0.25 * PHYSICS_HZ) == 0:
                _q = np.asarray(r["art"].get_joint_positions())
                _v = np.asarray(r["art"].get_joint_velocities())
                print(f"           [안착 {r['t'] / PHYSICS_HZ:4.2f}s] "
                      f"{r['name']} s={s_now * 1000:6.1f} "
                      f"이탈{off_now * 1000:5.1f}  z={p1[2] * 1000:+7.1f}  "
                      f"다리평균 {np.mean(_q[list(r['piston'].values())]) * 1000:5.2f}mm  "
                      f"휠각속도 {np.degrees(np.mean(np.abs(_v[r['wheel']]))):7.1f}°/s")
            if r["t"] > 1.5 * PHYSICS_HZ:
                # [진단] 조향 프레임이 맞는지 — 안착 직후 로봇은 관과 나란하니
                # 헤딩과 접선이 거의 같아야 하고 로컬 목표는 (+1,0,0) 이어야 한다.
                _hd = wrot(r["seg0"]) @ np.asarray(r["fw"])
                _tg = r["cl"].tangent(i_now)
                _el = r["fr"] @ (wrot(r["seg0"]).T @ _tg)
                print(f"[{r['name']:8s}] 안착 s={s_now * 1000:.1f}mm "
                      f"이탈 {off_now * 1000:.1f}mm — 주행 시작")
                print(f"           [진단] 헤딩({_hd[0]:+.2f},{_hd[1]:+.2f},"
                      f"{_hd[2]:+.2f}) 접선({_tg[0]:+.2f},{_tg[1]:+.2f},"
                      f"{_tg[2]:+.2f}) 로컬목표({_el[0]:+.2f},{_el[1]:+.2f},"
                      f"{_el[2]:+.2f}) — 로컬목표가 (+1,0,0) 이어야 한다")
                # 안착이 이렇게 어긋났으면 주행 결과를 보기 전에 원인을 봐야
                # 한다(다리가 벽에 안 닿은 채 시작한 것인지 가른다).
                if off_now > 0.010:
                    print(f"           [경고] 안착 이탈 {off_now * 1000:.1f}mm "
                          f"— 출발부터 어긋났다. 아래 상태를 먼저 볼 것")
                    diag_stuck(r)
                r.update(state="RUN", t=0, s_last=s_now, mark=step)
            continue

        # ── RUN : 끝까지 갔다가 되돌아오고, 다시 간다 ──────────────
        # 🚨 다리(중심 유지)와 중앙 관절(조향)의 위치 목표를 **한 번에** 쓴다.
        #    나눠 부르면 뒤에 쓴 것만 남는다(기록된 함정).
        # 🚨 `CENTER=0` 이어도 **예압은 매 스텝 써야** 한다. 안 쓰면 reset 뒤의
        #    `set_joint_positions()` 가 지운 드라이브 타깃이 그대로 0 이라
        #    다리가 벽을 안 민다(수평에서는 중력이 가려 준다).
        # ── NAV=vision : 판단을 정본 모듈에 맡긴다 ─────────────────
        if NAV == "vision" and r.get("ctl") is not None:
            # 🚨 **부호 있는** 휠 속도를 먹인다. |ω| 로 주면 복귀 중에도
            #    distance_m 이 계속 늘어 RETURN 이 영영 안 끝난다(실측 —
            #    복귀 재트리거 끼임 반복의 절반이 이것).
            _wmps = float(np.mean(np.asarray(
                r["art"].get_joint_velocities())[r["wheel"]])) * WHEEL_R
            # 카메라·판정은 10Hz (설계값). 매 스텝 굽지 않는다.
            if step - r.get("vis_step", -999) >= PHYSICS_HZ / 10.0:
                r["vis_step"] = step
                _cam = r.get("cam_front")
                if _cam is not None:
                    try:
                        _fr = _cam.get_current_frame()
                        _dep = _fr.get("distance_to_camera")
                        _rgba = _cam.get_rgba()
                        # 첫 프레임 전에는 빈 1차원 배열이 온다 — 건너뛴다
                        _rgb = (_rgba[:, :, :3]
                                if _rgba is not None
                                and getattr(_rgba, "ndim", 0) == 3 else None)
                        if _dep is not None and _rgb is not None \
                                and getattr(_dep, "size", 0):
                            _ja = math.degrees(float(np.max(np.abs(
                                np.asarray(r["art"].get_joint_positions())
                                [r["bel"]]))))
                            r["cond"] = r["det"].run(np.asarray(_dep),
                                                     joint_angle_deg=_ja,
                                                     rgb=np.asarray(_rgb))
                            # 판별자 설계용 depth 덤프 (DUMP_DEPTH=디렉터리,
                            # DUMP_MIN=분기비율 문턱 — 0 이면 전 프레임)
                            if os.environ.get("DUMP_DEPTH") and \
                                    r["cond"].branch_ratio >= float(
                                        os.environ.get("DUMP_MIN", 0.04)) and \
                                    r.get("dump_n", 0) < 40:
                                r["dump_n"] = r.get("dump_n", 0) + 1
                                np.save(f"{os.environ['DUMP_DEPTH']}/"
                                        f"{r['name']}_s{int(s_now * 1000):04d}"
                                        f"_{step:06d}.npy", np.asarray(_dep))
                            _fl = r["vo"].step(np.asarray(_rgb),
                                               np.asarray(_dep), 0.1)
                            r["vis_mps"] = (float(_fl.speed_mps)
                                            if _fl.valid else None)
                            r["vis_valid"] = bool(_fl.valid)
                    except Exception as exc:
                        if not r.get("vis_warned"):
                            r["vis_warned"] = True
                            print(f"[경고] {r['name']} 영상 판단 실패({exc}) "
                                  f"— 이 로봇은 지령 속도로만 돈다")
            # 🚨 `DriveController` 는 **dict** 를 받는다(ROS 메시지 경유 규약).
            #    `Condition` 객체를 그대로 넘기면 `cond.get` 에서 죽는다.
            _cd = r.get("cond")
            # 재출발 유예 중에는 start 를 주지 않는다 — 컨트롤러가 IDLE(v=0)
            # 로 서 있는 동안 steer_vision 이 관절을 편다.
            _hold_start = r.get("straighten_n", 0) > 0
            if _hold_start:
                r["straighten_n"] -= 1
            _ds = r["ctl"].step(PHYSICS_DT, recall=r.get("recall", False),
                                suppress_branch=r.get("leg_dev", 0.0) > 0.0018,
                                cond=None if _cd is None else _cd.as_dict(),
                                wheel_mps=_wmps, visual_mps=r.get("vis_mps"),
                                start=not r["started"] and not _hold_start)
            if not _hold_start:
                r["started"] = True
            # 🎯 [턴검증] 몸 clock 논쟁을 끝내는 정본 판정 (2026-08-07) —
            #    진입 기동의 실제 턴 방향을 **월드 기준**(오른쪽 = 진행방향 ×
            #    상방 +Z)으로 잰다. 몸 프레임 clock 만으로는 거울상 코스를 못
            #    가른다는 것이 세 번 반복된 교훈이라, 화면(사용자)과 같은
            #    기준으로 로그에 박는다.
            _prev_st = r.get("ctl_state")
            if _ds.state == "BRANCH_ENTRY" and _prev_st != "BRANCH_ENTRY":
                r["turn_h0"] = wrot(r["seg0"]) @ np.asarray(r["fw"])
            elif _prev_st == "BRANCH_ENTRY" and _ds.state != "BRANCH_ENTRY" \
                    and r.get("turn_h0") is not None:
                _h1 = wrot(r["seg0"]) @ np.asarray(r["fw"])
                _rgt = np.cross(r["turn_h0"], np.array([0.0, 0.0, 1.0]))
                _dp = float(np.dot(_h1, _rgt))
                print(f"[{r['name']:8s}] [턴검증] 진입 기동 종료 — 월드 기준 "
                      f"{'✅ 오른손 턴' if _dp > 0.2 else '🚨 왼손 턴' if _dp < -0.2 else '? 직진(꺾임 미미)'}"
                      f" (h0→h1·right = {_dp:+.2f})")
                r["turn_h0"] = None
            r["ctl_state"], r["slip"] = _ds.state, _ds.slip_ratio
            # 🚨 부호를 살린다 — RETURN 은 음수 지령으로 뒤로 나온다.
            #    연습장 dir 는 vision 에서 항상 +1 로 두고 부호는 여기서 진다.
            r["v_cmd"] = float(_ds.v_cmd)
            r["dir"] = +1
            r["ctl_steer"] = (float(_ds.steer_deg),
                              float(_ds.steer_clock_deg))

        if LEG_FORCE:
            _fi, _fv = force_legs(r)
            if _fi:
                _set_eff(r, _fv, _fi)
            _pi, _pv = [], []
        else:
            _pi, _pv = center_legs(r) if CENTER_ON else preload_legs(r)
        _si, _sv = (steer(r, i_now) if NAV == "blueprint"
                    else steer_vision(r) if NAV == "vision"
                    else steer_onboard(r))
        if _pi or _si:
            _set_pos(r, _pv + _sv, _pi + _si)
        # 🔑 **속도를 스스로 정한다** — 관절이 꺾이면 줄이고 펴지면 올린다.
        # 🔑 판단층 둘(관절 꺾임 / 다리 실린더) 중 **더 느린 쪽**을 택한다.
        if NAV == "vision" and r.get("ctl") is not None:
            leg_speed(r)                         # 계측만 (판단은 컨트롤러가)
            # 🚨 bend_max 는 curve_speed() 안에서만 갱신되는데 vision 경로는
            #    그걸 안 부른다 — "최대 꺾임 0.0°" 로 찍히던 지표 구멍(실측).
            if r.get("bel"):
                _ang = np.degrees(np.abs(
                    np.asarray(r["art"].get_joint_positions())[r["bel"]]))
                r["bend_max"] = max(r.get("bend_max", 0.0),
                                    float(np.linalg.norm(_ang)))
            _v = r["v_cmd"]                      # ← `driver/control` 이 정한다
        elif AUTO_SPEED:
            curve_speed(r)                       # r["curve_f"] 갱신
            _f = min(r.get("curve_f", 1.0), leg_speed(r))
            r["want_f"] = _f
            _v = ramp(r, TARGET_SPEED_MPS * _f)
        else:
            _v = TARGET_SPEED_MPS
        drive(r, math.degrees(_v / WHEEL_R) * r["dir"])
        r["wheel_rad"] += (_v / WHEEL_R) * PHYSICS_DT
        r["off_hist"] = r.get("off_hist", []) + [off_now]

        # 🚨 관 밖으로 튄 것은 되살리지 않는다 — 조용히 계속 돌면 로그가
        #    거짓말이 된다. 그 로봇만 멈추고 나머지는 계속 간다.
        if off_now > PIPE_IR + 0.03 or not np.isfinite(p1).all():
            print(f"[{r['name']:8s}] ❌ 코스 이탈 — 중심선에서 "
                  f"{off_now * 1000:.0f}mm. 이 로봇은 여기서 멈춘다")
            drive(r, 0.0)
            r["dead"] = True
            continue

        # ── NAV=vision : 왕복을 컨트롤러의 RETURN/DONE 으로 돈다 ────
        # 🚨 연습장의 방향 뒤집기(dir=−1)와 컨트롤러가 **싸운다**(실측:
        #    복귀 중 BRANCH 재트리거 → s=155 끼임 반복). vision 에서는
        #    연습장이 개입하지 않는다 — 코스 끝에서 `recall=True` 를 주고,
        #    복귀·끼임 회복은 전부 컨트롤러(RETURN/RECOVER)가 한다.
        if NAV == "vision" and r.get("ctl") is not None:
            if not r.get("recall") and s_now >= r["cl"].total - END_S:
                r["recall"] = True
                print(f"[{r['name']:8s}] ✅ 코스 끝 도달 (왕복 {r['lap'] + 1})"
                      f" — recall 지시, 컨트롤러가 후진 복귀한다")
            # 🎯 조기 실패 판정 (2026-08-07 사용자 지적) — 끼인 채 스텝 예산을
            #    다 태우며 기다리지 않는다. 진행이 FAIL_S 동안 3mm 미만이면
            #    그 시점에서 실패로 끊는다. 시도 회전이 몇 배 빨라진다.
            if abs(s_now - r.get("s_prog", -1.0)) > 0.003:
                r["s_prog"], r["prog_mark"] = s_now, step
            elif step - r.get("prog_mark", step) > FAIL_S * PHYSICS_HZ:
                print(f"[{r['name']:8s}] ❌ 조기 중단 — {FAIL_S:.0f}초째 "
                      f"제자리 (s={s_now * 1000:.0f}mm, "
                      f"상태 {r.get('ctl_state', '?')}). 실패로 판정")
                drive(r, 0.0)
                r["dead"] = True
                continue
            if r.get("ctl_state") == "DONE":
                # 🚨 **가짜 DONE 검문** (2026-08-07) — 복귀가 끝 구간(코스 끝
                #    − END_S)을 벗어나지도 않았는데 거리 적산이 0 에 닿아
                #    DONE 이 나면, 재출발 즉시 recall 이 다시 걸려 랩만 헛돈다
                #    (실측: 물리 3랩에 로그 155랩). 숨기지 않고 실패로 끝낸다.
                if s_now >= r["cl"].total - END_S - 0.02:
                    print(f"[{r['name']:8s}] ❌ 복귀 실패 — DONE 시점에 아직 "
                          f"끝 구간 (s={s_now * 1000:.0f}mm ≥ "
                          f"{(r['cl'].total - END_S) * 1000:.0f}mm). "
                          f"거리 적산이 실주행보다 빨리 소진된 것")
                    drive(r, 0.0)
                    r["dead"] = True
                    continue
                r["lap"] += 1
                r["recall"] = False
                r["started"] = False
                # 🎯 재출발 유예 (2026-08-07) — 복귀 종점이 접합부 모서리라,
                #    비틀린 몸으로 곧장 재진입을 걸면 쐐기가 된다(실측: 1랩은
                #    안착 직후 곧은 몸이라 성공, 2랩 재출발만 s=178 RECOVER
                #    동결 사망). 2초간 관절을 펴고 나서 출발한다.
                r["straighten_n"] = int(2.0 * PHYSICS_HZ)
                r["s_prog"], r["prog_mark"] = -1.0, step
                _ctl_kn = dict(
                    branch_rule=os.environ.get("BRANCH_RULE", "right"))
                for _env, _key in (("CENTER_GAIN", "center_gain"),
                                   ("CENTER_GAIN_CURVE", "center_gain_curve")):
                    if os.environ.get(_env) is not None:
                        _ctl_kn[_key] = float(os.environ[_env])
                r["ctl"] = DriveController(_ctl_kn)
                print(f"[{r['name']:8s}] 🔁 복귀 완료 (왕복 {r['lap']})"
                      f" — 다시 간다")
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
            if r["stuck"] <= 2:          # 처음 두 번만 자세히 (로그 폭주 방지)
                diag_stuck(r)
            continue

        # 끝 / 시작에 닿으면 방향 전환 — 이것이 '왕복 1회'
        # 🚨 **코스 끝은 관의 열린 끝단이다.** 마지막 점까지 가면 로봇 앞
        #    절반(94mm)이 관 밖으로 나가고, 벽을 잃은 다리가 스트로크 한계
        #    (도달 75mm)까지 뻗어 **관 밖으로 튀어나온다**(GUI 로 확인).
        #    → 출발과 같은 여유(END_S)를 남기고 되돌아간다.
        if r["dir"] > 0 and s_now >= r["cl"].total - END_S:
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

    # 🎯 전멸 즉시 종료 (2026-08-07 사용자 지시) — 죽은 로봇을 두고 스텝
    #    예산을 태우지 않는다. 실패는 이미 났고, 다음 시도가 더 싸다.
    if robots and all(r["dead"] for r in robots):
        print(f"[중단] 로봇 전멸 (step {step}/{STEPS}) — 즉시 종료")
        break

    if step - _last_report >= REPORT_S * PHYSICS_HZ:
        _last_report = step
        _line = []
        for r in robots:
            p1 = wpos(r["seg1"])
            s_now, off_now, _i = r["cl"].nearest(p1)
            if NAV == "vision":
                _c = r.get("cond")
                _line.append(
                    f"{r['name']} s={r['cl'].nearest(wpos(r['seg1']))[0]*1000:5.0f}"
                    f" {r.get('ctl_state','?')}"
                    f" v={r.get('v_cmd',0)*1000:3.0f}mm/s"
                    f" 시각속도={'--' if r.get('vis_mps') is None else format(r['vis_mps']*1000,'.0f')}"
                    f" 슬립={r.get('slip',0):.2f}"
                    f" 판정={'--' if _c is None else _c.state}"
                    f"(개방{0 if _c is None else _c.aperture_px}px"
                    f" 관경{0 if _c is None else _c.bore_mm:.0f}mm"
                    f" 분기{0 if _c is None else _c.branch_ratio * 100:.0f}%"
                    f"@{0 if _c is None else _c.branch_deg:+.0f}°"
                    f" 입사{0 if _c is None else _c.incidence_deg:.0f}°"
                    f"@{0 if _c is None else _c.incidence_clock_deg:+.0f}°"
                    f" 오프셋{0 if _c is None else _c.offset_mm:.0f}mm"
                    f"@{0 if _c is None else _c.joint_range_m:.2f}m)")
                continue
            _w = r.get("want", (0.0, 0.0))
            _x = (f" {r.get('v_cmd', 0) * 1000:3.0f}mm/s"
                  f" 목표P{_w[0]:+.0f}/Y{_w[1]:+.0f}"
                  f" 지령{r.get('cmd_pitch', 0):+.0f}/{r.get('cmd_yaw', 0):+.0f}"
                  f" 관절합P{r.get('q_pitch', 0):+.0f}/Y{r.get('q_yaw', 0):+.0f}"
                  f" 다리편차{r.get('leg_spread_now', 0) * 1000:.1f}"
                  f"/기준벗어남{r.get('leg_dev', 0) * 1000:.1f}mm"
                  f" 배율{r.get('want_f', 1):.2f}"
                  f" 실제P{r.get('act_pitch', 0):+.0f}"
                  f"/Y{r.get('act_yaw', 0):+.0f}")
            _line.append(f"{r['name']} s={s_now * 1000:5.0f}"
                         f"{'→' if r['dir'] > 0 else '←'}"
                         f" 왕복{r['lap']} 끼임{r['stuck']}{_x}"
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
          f"s={s_now * 1000:.0f}mm  이탈 {off_now * 1000:.1f}mm "
          f"(평균 {np.mean(r.get('off_hist', [0])) * 1000:.1f} / 최대 "
          f"{np.max(r.get('off_hist', [0])) * 1000:.1f}mm)  실속도/지령 "
          f"{np.mean(r.get('gov_hist', [1])) * 100:.1f}%  최대 꺾임 "
          f"{r.get('bend_max', 0):.1f}°  조향오차 최대 "
          f"{r.get('steer_max', 0):.1f}°  뜬다리 최대 "
          f"{r.get('leg_free_max', 0)}개  다리편차 최대 "
          f"{r.get('leg_spread_max', 0) * 1000:.1f}mm"
          f"  다리 최대신장 {r.get('leg_ext_max', 0) * 1000:.1f}mm"
          f"(도달 {TUNE['wheel_center_mm'] + r.get('leg_ext_max', 0) * 1000:.0f}"
          f" / 관벽 {(PIPE_IR - WHEEL_R) * 1000:.0f}mm)"
          + ("  ❌ 이탈로 정지" if r["dead"] else ""))
sys.stdout.flush()
simulation_app.close()
