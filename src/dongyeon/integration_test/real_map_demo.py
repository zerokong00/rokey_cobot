"""[Isaac 3.11] 실전 맵(화장실 배관망) 점검·수리 시연 — 샤워 배수구로 진입한다.

`repair_demo.py` 의 **로직과 동작을 그대로** 쓰고 코스만 실전 맵으로 바꾼 것이다.
층마다 로봇 1대가 자기 층 수평망을 돈다(설계: 층 이동 없음).

  --floor2 (기본)  진입 → 곡관 → 결함 2건 수리·검증 → **관 단절 감지 → 복귀**
  --floor1          진입 → 곡관 → **T자 분기 판단(오른손 법칙)**

맵  maps/restroom_pipe150_final.usd  (CATIA STEP 변환본, MAP_USD 로 교체 가능)
    🚨 **이 파일은 읽기 전용으로 다룬다.** Isaac 에서 Ctrl+S 하면 형상 없는
       껍데기가 원본을 덮어써 실제로 한 번 날아갔다(기록된 사고).

━━ 실측으로 확정한 맵 기하 (map mm, upAxis Z, metersPerUnit 0.001) ━━
  진입   사각 거름망 개구 130×130   floor2 z +100~+85 / floor1 z −2390.23~−2405.23
         수직 라이저 내반경 50(ø100), 직선 185mm
  코스   전 곡관 굽힘 R=150 (LR). 수평망 floor2 z=−250 / floor1 z=−2740.23
  floor2 (330,850) → (680,850) → (680,1400) → (1200,1400) → (1200,600) → (1500,600)
         총 2,533mm. **끝 (1500,600) 이 관 끝 캡 = 단절**
  floor1 (330,850) → (730,850) ★T★ → (730,100) → (1300,100) → (1300,600) → (1500,600)

🚨 **스케일** — 맵 usd 는 metersPerUnit 0.001 인데 이 스테이지는 1.0(m) 이다.
   USD 참조는 단위를 자동 변환하지 않으므로 그냥 얹으면 좌표 2500 이 2500 m 로
   들어와 **건물이 2.5km** 가 된다. 참조 Xform 에 scale 0.001 을 명시한다.

🚨 **원점** — 건물이 z −2980~+2100mm 에 걸쳐 있어 z=0 평면이 한가운데를 자른다.
   활성 층의 **수평망이 월드 z=0** 에 오도록 맵을 Z 로 옮긴다. 그래야 좌표가
   ±2m 안에 들어오고 지면과도 안 겹친다.

🚨 **인스턴스 프로토타입이라 그냥은 메시가 안 잡힌다.** floor2/floor1/aisle 이
   instanceable 이라 `Stage.Traverse()` 로 Mesh 가 0개 나온다. 참조 직후
   `SetInstanceable(False)` 를 해야 메시 편집(관통 개구 절단)과 콜라이더 부여가
   된다. 실측으로 확인: 해제 후 Mesh 15개 노출, 삼각형 편집·cook 전부 통과.

━━ 이 맵에서 **미리 알고 시작하는 두 가지 제약** (실측, 추측 아님) ━━
① floor2 경로 s 960~1660mm 구간이 **ø90(내반경 45)** 이다. 메모리의 "관R 45
   곡관 2 + 직관 1(x 830~1050, y 1250~1400)" 이 그것이고 **우회로가 없다.**
   휠 접지 반경이 40→35mm 로 들어가야 해서 피스톤이 5mm 더 압축된다
   (스트로크 0~10mm 라 여지는 있으나 예압이 레그당 +15N). 통과 여부는 실행으로만
   가린다 — `[관경]` 로그가 매 구간 벽거리를 찍는다.
② floor1 의 T 분기는 **날카로운 T 다(필렛 없음).** 실측: x 550~680 에서 ±Y 가
   50.0mm 로 막혀 있다가 x≈700 에서 갑자기 열리고 x=730 에서 +X 가 50.0 로 막힌다.
   급회전 90° 는 두 세그먼트가 **동시에** 90° 를 이뤄야 하는데 벨로우즈는 4관절
   ×±27.5° 라 한 평면 ±55°(롤 45°로 두 축을 같이 써도 ~78°) 다. **조향을 넣어도
   안 풀리는 기하 한계다.** R150 곡관이 되는 것은 90°를 235mm 호에 나눠 쓰기
   때문이다(세그먼트 간 필요각 약 38°).
   → 이 시연은 T 를 **감지하고 오른손 법칙으로 갈 가지를 고르는 것까지** 하고,
     진입을 시도한 뒤 결과를 정직하게 기록한다. 성공을 가정하지 않는다.

실행 (Isaac 내장 rclpy 환경 — HANDOFF.md 참조):
  isaac_python real_map_demo.py --floor2 --headless
  DISPLAY=:1 isaac_python real_map_demo.py --floor2 --glass --hold
  DISPLAY=:1 isaac_python real_map_demo.py --floor1 --hold --shots
"""

import json
import math
import os
import struct
import sys
from pathlib import Path as _P

HEADLESS = "--headless" in sys.argv
SHOTS = "--shots" in sys.argv
HOLD = "--hold" in sys.argv
NO_TORCH = "--no-torch" in sys.argv
TORCH_NOCOLLIDE = "--torch-nocollide" in sys.argv
WATER = "--water" in sys.argv
FLUID = "--fluid" in sys.argv
FLOODED = WATER or FLUID
GLASS = WATER or "--glass" in sys.argv
FLOOR = "floor1" if "--floor1" in sys.argv else "floor2"
if FLUID and WATER:
    print("[경고] --fluid 와 --water 를 같이 줬다 — 항력 이중 계상이라 --fluid 끔")
    FLUID = False
STEPS = 60000
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

# 🚨 headless 에서는 카메라 프레임이 안 나와(HANDOFF.md 기록된 함정) 발행할
#    내용이 없다 — --no-ros 로 명시적으로도 끌 수 있게 해 둔다.
# 🚨 rclpy 는 그냥 import 만 하면 안 잡힌다 — 처음엔 pyver.py 가 문서화한 대로
#    IsaacSim-ros_workspaces 로 3.11 용 ROS2 를 직접 빌드해야 하는 줄 알았는데,
#    실측해보니 **번들된 `isaacsim.ros2.bridge` 확장에 Humble/Python 3.11 용
#    rclpy 가 이미 들어 있다**(rclpy-3.3.17-py3.11). `enable_extension(
#    "isaacsim.ros2.bridge")` 로 그 확장을 먼저 로드해야 내부 경로가 잡혀
#    import 가 된다(실측: 그냥 import → ModuleNotFoundError, enable_extension
#    먼저 → import 성공 + rclpy.init() 성공). 이 확장의 .so 를 찾으려면 실행
#    전 셸에서 LD_LIBRARY_PATH 에 `<isaac설치>/exts/isaacsim.ros2.bridge/
#    humble/lib` 가 잡혀 있어야 한다 — 이 저장소 사용자의 `~/.bashrc` 에
#    `isaac_ros` alias 로 이미 있다(실행 전에 그 alias 를 먼저 칠 것).
NO_ROS = HEADLESS or "--no-ros" in sys.argv
NO_ROS_REASON = "headless" if HEADLESS else "--no-ros"
if not NO_ROS:
    try:
        from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
        enable_extension("isaacsim.ros2.bridge")
        import rclpy                                      # noqa: E402
        from rclpy.node import Node                        # noqa: E402
        from rclpy.qos import (QoSProfile, ReliabilityPolicy,  # noqa: E402
                               HistoryPolicy)
        from sensor_msgs.msg import CompressedImage          # noqa: E402
        from std_msgs.msg import String, Bool                # noqa: E402
    except ImportError as _rclpy_exc:
        print(f"[경고] rclpy 없음 — ROS2 카메라 발행 끔 ({_rclpy_exc}). "
              f"LD_LIBRARY_PATH 에 isaacsim.ros2.bridge/humble/lib 있는지 "
              f"확인할 것(예: `isaac_ros` alias 먼저 실행)")
        NO_ROS = True
        NO_ROS_REASON = f"rclpy 없음: {_rclpy_exc}"

MM = 0.001
PHYSICS_HZ_PRE = float(os.environ.get("PHYSICS_HZ", 240))
SON = _P(__file__).resolve().parent
ROBOT_USDA = str(SON / "robot_bellows" / "robot_bellows.usda")
# 🔑 **수정본을 기본으로 쓴다.** 원본 CAD 에는 로봇이 못 지나는 기하 결함이
#    둘 있다 — ① 같은 배관이 두 번 모델링됐고 `PartBody` 사본이 4.7mm 더 좁다
#    (곡관2 에서 45.3mm) ② floor2 의 y=1400 가지가 ø90 이다. 둘 다
#    `tools/fix_map.py` 가 고친다(원본은 안 건드리고 `_fixed.usd` 를 만든다).
#    실측: 최악 내반경 44.6 → 48.8mm (기준 배관 pipe_elbow_lr150 과 동일).
#    맵은 `src/son/maps/` 안에 둔다(2026-08-06 사용자 지시 — 레포 밖 참조 제거).
#    ⚠ `.gitignore` 의 `*.usd` 때문에 **git 으로는 안 따라간다** — 받는 사람은
#    STEP 을 변환해 `maps/` 에 넣고 `tools/fix_map.py` 를 직접 돌려야 한다.
_MAPS = SON / "maps"
_MAP_FIXED = _MAPS / "restroom_pipe150_final_fixed.usd"
_MAP_ORIG = _MAPS / "restroom_pipe150_final.usd"
MAP_USD = os.environ.get(
    "MAP_USD", str(_MAP_FIXED if _MAP_FIXED.is_file() else _MAP_ORIG))
if MAP_USD == str(_MAP_ORIG):
    print("[경고] 원본 맵을 쓴다 — 곡관에 좁은 중복 사본(45.3mm)과 ø90 구간이 "
          "있어 로봇이 못 지난다. `python3 tools/fix_map.py` 를 먼저 돌릴 것")
OUT = SON / "out" / f"real_map_{FLOOR}"

META = json.loads((SON / "spec" / "parts_meta.json").read_text())
TORCH = META["torch"]
sys.path.insert(0, str(SON / "welder"))
import torch_spec                                         # noqa: E402

# ── 층별 코스 데이터 (map mm) ───────────────────────────────────────
# 코너는 **직선의 교점**이다. 필렛 R=150 이 자동으로 끼워진다.
# 결함은 map 좌표로 준다 — 실행 시 중심선에 투영해 s 와 시계각을 찍어 준다.
#   🚨 결함은 **직선 구간**에 두어야 한다. 물 주머니·개구 절단이 그 지점의
#      접선을 축으로 하는 원통이라고 가정하기 때문이다(곡관이면 틀어진다).
#   🚨 시계각 180° = 관 바닥. 물이 새는 것을 보려면 중력 방향이어야 한다.
FLOORS = {
    "floor2": {
        "z_net": -250.0,                 # 수평망 중심선 z
        "corners": [(330, 850, 85), (330, 850, -250), (680, 850, -250),
                    (680, 1400, -250), (1200, 1400, -250),
                    (1200, 600, -250), (1500, 600, -250)],
        # D1 은 x=680 의 +Y 직선(250mm), D2 는 x=1200 의 −Y 직선(500mm).
        # 둘 다 ø100 구간이다(ø90 구간 s 960~1660 을 피했다).
        "defects": [{"p": (680, 1130, -250), "clock": 180.0},
                    {"p": (1200, 1000, -250), "clock": 180.0}],
        "end": "disconnect",             # 코스 끝이 관 끝 캡 = 단절
        "note": "s 960~1660mm 는 ø90(내반경 45) — 우회로 없음",
    },
    "floor1": {
        "z_net": -2740.23,
        "corners": [(330, 850, -2405.23), (330, 850, -2740.23),
                    (730, 850, -2740.23), (730, 100, -2740.23),
                    (1300, 100, -2740.23), (1300, 600, -2740.23),
                    (1500, 600, -2740.23)],
        # 🚨 T 앞 직선이 100mm(x 480→580) 뿐이라 결함을 놓을 자리가 없다.
        #    INSPECT 는 결함 120mm 앞에서 서야 하는데 그 자리가 곡관 안이라
        #    카메라가 결함이 아니라 곡관 바깥벽을 본다. 억지로 넣지 않는다.
        #    T 너머 가지에 두려면 아래 목록에 좌표를 넣고 --start-s 로 T 를
        #    건너뛴 지점에서 출발시킬 것(그건 "T 를 통과했다" 가 아니다).
        "defects": [],
        "end": "junction",               # T 분기에서 판단
        "note": "T 분기(730,850)는 날카로운 T — 벨로우즈 굽힘 한계로 통과 불가",
    },
}
CFG = FLOORS[FLOOR]
Z_NET = CFG["z_net"]
BEND_R = 0.150                            # LR 곡관 굽힘 반경 (m)
PIPE_IR = 0.050                           # 설계 내반경 (ø90 구간은 0.045)

# 출발 지점 — 라이저 직선 안. 로봇 전장 약 140mm, 라이저 직선 185mm 이라
# 통째로 들어간다. 0 은 라이저 상단(사각 개구 아래면)이다.
START_S_MM = float(os.environ.get("START_S", 35.0))
if "--start-s" in sys.argv:
    START_S_MM = float(sys.argv[sys.argv.index("--start-s") + 1])

# ── repair_demo 와 같은 상수들 ──────────────────────────────────────
LEAK_PORT_D = 0.0381        # 입자 설정에서 유도된 값 (PCO 8 / 입자지름 9.6)
RHO, G = 1000.0, 9.81
V_FLOW_DESIGN, A_FRONTAL, CD_EFF, V_DISP = 0.855, 2.7e-3, 2.32, 1.8e-4
WHEEL_R, SEG_GAP = 0.010, 0.076
TARGET_SPEED_MPS = float(os.environ.get("SPEED_MPS", 0.05))   # 설계값. 카메라 10Hz 검출과 맞물린다(repair_demo 주석 참조)
SPIN_DEG_S = math.degrees(TARGET_SPEED_MPS / WHEEL_R)
CONTACT_OFFSET = max(0.0005, 1.2 * TARGET_SPEED_MPS / PHYSICS_HZ_PRE)
if WATER:
    CONTACT_OFFSET = max(0.02, CONTACT_OFFSET)   # GPU 하한 (기록된 함정)
REST_OFFSET = 0.0
RING_R_OUT = TORCH["ring_r_out"] * MM
ROD_ORIGIN_R = TORCH["rod_origin_r"] * MM
ROD_LEN = TORCH["rod_len"] * MM
J2_STROKE = TORCH["j2_stroke_mm"] * MM
STOW_R = TORCH["stow_radius_mm"] * MM
REACH_R = TORCH["reach_radius_mm"] * MM
WELD_GAP = 0.002
TIP_TARGET_R = (PIPE_IR - WELD_GAP) * 1000
J2_KP, J2_TOL = 0.6, 0.15
# J1 도 같은 이유로 폐루프다 — 관절값이 아니라 **측정한 팁 시계각**
# 으로 닫는다. 0.6° = 관벽 호길이 0.52mm (허용 1.5mm 의 1/3).
J1_KP, J1_TOL_DEG = 0.5, 0.6
PHYSICS_HZ = PHYSICS_HZ_PRE
PHYSICS_DT = 1.0 / PHYSICS_HZ

ROBOT, MAPP = "/World/Robot", "/World/Map"

world = World(stage_units_in_meters=1.0,
              physics_dt=PHYSICS_DT, rendering_dt=1.0 / 60.0)
stage = world.stage
if WATER:
    _pc = world.get_physics_context()
    _pc.enable_gpu_dynamics(True)
    _pc.set_broadphase_type("GPU")
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage, "/World")


def rot(deg, axis):
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(Gf.Vec3d(*axis), deg))
    return m


def trans(x, y, z):
    m = Gf.Matrix4d(1.0)
    m.SetTranslate(Gf.Vec3d(x, y, z))
    return m


def quat_from_matrix_wxyz(M):
    """3x3 회전행렬(열=변환된 기저벡터) → USD "usd" 축계 사원수(w,x,y,z).

    카메라 set_local_pose(camera_axes="usd") 는 이 스칼라-우선 규약을 그대로
    쓴다. torch_camera 를 실시간으로 재조준하는 데 쓴다(아래 aim_camera_at).
    """
    t = np.trace(M)
    if t > 0:
        S = np.sqrt(t + 1.0) * 2
        w = 0.25 * S
        x, y, z = (M[2, 1]-M[1, 2])/S, (M[0, 2]-M[2, 0])/S, (M[1, 0]-M[0, 1])/S
    elif M[0, 0] > M[1, 1] and M[0, 0] > M[2, 2]:
        S = np.sqrt(1.0+M[0, 0]-M[1, 1]-M[2, 2])*2
        w, x = (M[2, 1]-M[1, 2])/S, 0.25*S
        y, z = (M[0, 1]+M[1, 0])/S, (M[0, 2]+M[2, 0])/S
    elif M[1, 1] > M[2, 2]:
        S = np.sqrt(1.0+M[1, 1]-M[0, 0]-M[2, 2])*2
        w, x = (M[0, 2]-M[2, 0])/S, (M[0, 1]+M[1, 0])/S
        y, z = 0.25*S, (M[1, 2]+M[2, 1])/S
    else:
        S = np.sqrt(1.0+M[2, 2]-M[0, 0]-M[1, 1])*2
        w, x = (M[1, 0]-M[0, 1])/S, (M[0, 2]+M[2, 0])/S
        y, z = (M[1, 2]+M[2, 1])/S, 0.25*S
    return np.array([w, x, y, z])


def W(p_mm):
    """map mm → 월드 m. 활성 층 수평망이 z=0 이 되도록 옮긴다."""
    return np.array([p_mm[0] * MM, p_mm[1] * MM, (p_mm[2] - Z_NET) * MM])


# ── 중심선 ──────────────────────────────────────────────────────────
class CenterLine:
    """코너 목록 + 필렛 반경 → 직선/원호가 교대하는 중심선.

    🚨 원호의 두 번째 기저는 **들어오는 접선 t1** 이다(나가는 t2 가 아니다).
       t2 로 쓰면 a=0 에서 접선이 안 맞아 중심선이 관 밖으로 나간다 — 실제로
       한 번 그렇게 짜서 검증이 전 구간 ★ 로 떨어졌다.
    """

    def __init__(self, corners_world, R):
        C = [np.asarray(c, float) for c in corners_world]
        self.segs, cur = [], C[0]
        for i in range(1, len(C) - 1):
            B = C[i]
            t1 = B - C[i - 1]; t1 /= np.linalg.norm(t1)
            t2 = C[i + 1] - B; t2 /= np.linalg.norm(t2)
            p_in, p_out = B - t1 * R, B + t2 * R
            ctr = p_in + (p_out - B)
            L = float(np.linalg.norm(p_in - cur))
            if L > 1e-9:
                self.segs.append(("line", cur, t1, L))
            ang = float(np.arccos(np.clip(np.dot(t1, t2), -1.0, 1.0)))
            self.segs.append(("arc", ctr, (p_in - ctr) / R, t1, R, R * ang))
            cur = p_out
        L = float(np.linalg.norm(C[-1] - cur))
        self.segs.append(("line", cur, (C[-1] - cur) / L, L))
        self.cum = np.cumsum([s[-1] for s in self.segs])
        self.total = float(self.cum[-1])

    def point_tangent(self, s):
        s = min(max(float(s), 0.0), self.total)
        i = min(int(np.searchsorted(self.cum, s, side="right")),
                len(self.segs) - 1)
        u = s - (self.cum[i - 1] if i else 0.0)
        sg = self.segs[i]
        if sg[0] == "line":
            return sg[1] + sg[2] * u, sg[2]
        _, ctr, e1, t1, R, _ = sg
        a = u / R
        p = ctr + R * (math.cos(a) * e1 + math.sin(a) * t1)
        t = -math.sin(a) * e1 + math.cos(a) * t1
        return p, t / np.linalg.norm(t)

    def frame(self, s):
        """(점, 접선, e_up, e_side). 시계각은 e_up→e_side, **180° = 바닥.**

        e_up 은 월드 +Z 의 접선 수직 성분이다. 수직관에서는 퇴화하므로 +X 로
        떨어뜨린다(그 구간에는 결함을 두지 않는다).
        """
        p, t = self.point_tangent(s)
        u = np.array([0.0, 0.0, 1.0]) - t * t[2]
        if np.linalg.norm(u) < 1e-6:
            u = np.array([1.0, 0.0, 0.0]) - t * t[0]
        u /= np.linalg.norm(u)
        return p, t, u, np.cross(u, t)

    def radial(self, s, clock_deg):
        _, _, u, w = self.frame(s)
        a = math.radians(clock_deg)
        return math.cos(a) * u + math.sin(a) * w

    def clock_of(self, s, d):
        """방향벡터 d 의 시계각(도). radial() 의 역이다."""
        _, _, u, w = self.frame(s)
        return math.degrees(math.atan2(float(np.dot(d, w)), float(np.dot(d, u))))

    def tabulate(self, ds=0.002):
        """조밀 표본표 — 투영을 numpy 한 방으로 끝내려고 미리 굽는다.

        🚨 표 없이 매 호출마다 `point_tangent` 를 파이썬 루프로 수백 번 돌리면
           **투영이 물리보다 비싸진다.** 메인 루프가 스텝마다 링·팁·본체를
           투영하므로 이건 선택이 아니다.
        """
        self.tab_s = np.arange(0.0, self.total + 1e-9, ds)
        self.tab_p = np.array([self.point_tangent(x)[0] for x in self.tab_s])
        self.tab_ds = ds
        return self

    def project(self, p, hint=None, win=0.30):
        """가장 가까운 s 와 그때의 중심선 거리. hint 가 있으면 그 둘레만 본다."""
        if hint is None:
            lo_i, hi_i = 0, len(self.tab_s)
        else:
            lo_i = max(0, int((hint - win) / self.tab_ds))
            hi_i = min(len(self.tab_s), int((hint + win) / self.tab_ds) + 1)
            if hi_i - lo_i < 2:
                lo_i, hi_i = 0, len(self.tab_s)
        d2 = np.sum((self.tab_p[lo_i:hi_i] - p) ** 2, axis=1)
        k = lo_i + int(np.argmin(d2))
        # 표 간격 안쪽을 한 번 더 좁힌다(정렬 판정이 0.4mm 를 다툰다)
        lo = max(0.0, self.tab_s[k] - self.tab_ds)
        hi = min(self.total, self.tab_s[k] + self.tab_ds)
        ss = np.linspace(lo, hi, 9)
        P = np.array([self.point_tangent(x)[0] for x in ss])
        k2 = int(np.argmin(np.sum((P - p) ** 2, axis=1)))
        return float(ss[k2]), float(np.linalg.norm(P[k2] - p))


PATH = CenterLine([W(c) for c in CFG["corners"]], BEND_R).tabulate()
print("=" * 78)
print(f"[코스] {FLOOR} — 중심선 {PATH.total * 1000:.1f}mm, 코너 "
      f"{len(CFG['corners'])}개, 곡관 R{BEND_R * 1000:.0f}")
print(f"       {CFG['note']}")

# ── 맵 적재 ─────────────────────────────────────────────────────────
map_root = stage.DefinePrim(MAPP, "Xform")
map_root.GetReferences().AddReference(MAP_USD)
_xf = UsdGeom.Xformable(map_root)
_xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -Z_NET * MM))
_xf.AddScaleOp().Set(Gf.Vec3f(MM, MM, MM))
_n_inst = 0
for _p in stage.Traverse():
    if _p.IsInstance():
        _p.SetInstanceable(False)
        _n_inst += 1
map_meshes = [p for p in stage.Traverse()
              if p.IsA(UsdGeom.Mesh) and str(p.GetPath()).startswith(MAPP)]
print(f"[준비] 맵 {_P(MAP_USD).name} — instanceable 해제 {_n_inst}개, "
      f"메시 {len(map_meshes)}개, 스케일 {MM} (metersPerUnit 0.001 보정), "
      f"z 오프셋 {-Z_NET:.2f}mm → 수평망이 월드 z=0")

# 다른 층·샤프트는 렌더만 남기고 **충돌에서 뺀다.** 로봇이 갈 수 없는 곳이라
# 삼각형만 늘리고, 무엇보다 다른 층 관을 잘못 잡을 위험이 있다.
_this, _other = [], []
for p in map_meshes:
    (_this if f"/{FLOOR}/" in str(p.GetPath()) else _other).append(p)

_XCB = UsdGeom.XformCache()


def mesh_world(prim):
    m = UsdGeom.Mesh(prim)
    pts = np.array(m.GetPointsAttr().Get(), dtype=np.float64)
    M = np.array(_XCB.GetLocalToWorldTransform(prim), dtype=np.float64)
    idx = np.array(m.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
    return pts @ M[:3, :3] + M[3, :3], idx


# 레이캐스트용 삼각형 (활성 층만, 월드 m)
_TRI = []
for p in _this:
    w, idx = mesh_world(p)
    _TRI.append(w[idx])
_TRI = np.vstack(_TRI)
_T0, _TE1, _TE2 = _TRI[:, 0], _TRI[:, 1] - _TRI[:, 0], _TRI[:, 2] - _TRI[:, 0]


def raycast(o, d):
    """Möller–Trumbore — 가장 가까운 교차 거리(m). 없으면 inf."""
    d = np.asarray(d, float); d = d / np.linalg.norm(d)
    h = np.cross(d, _TE2)
    a = np.einsum("ij,ij->i", _TE1, h)
    ok = np.abs(a) > 1e-15
    f = np.zeros_like(a); f[ok] = 1.0 / a[ok]
    s = o - _T0
    u = f * np.einsum("ij,ij->i", s, h)
    q = np.cross(s, _TE1)
    v = f * (q @ d)
    t = f * np.einsum("ij,ij->i", _TE2, q)
    g = ok & (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9) & (t > 1e-9)
    return float(t[g].min()) if g.any() else float("inf")


def dist_to_path(pts, chunk=400):
    """점 무리 → 중심선까지 최단거리(m). 표를 써서 벡터로 잰다."""
    out = np.empty(len(pts))
    tab = PATH.tab_p
    for i in range(0, len(pts), chunk):
        q = pts[i:i + chunk]
        d = np.linalg.norm(q[:, None, :] - tab[None, :, :], axis=2)
        out[i:i + chunk] = d.min(axis=1)
    return out


def bore_radius(s, n=12):
    """중심선 s 에서 관 내반경(mm) 중앙값 — 관경 변화를 실측으로 잡는다."""
    p, t, u, w = PATH.frame(s)
    h = [raycast(p, math.cos(math.radians(30 * k)) * u
                 + math.sin(math.radians(30 * k)) * w) for k in range(n)]
    h = np.array(h)
    fin = np.isfinite(h)
    return (float(np.median(h[fin])) * 1000) if fin.any() else -1.0


print("[관경] 중심선 따라 실측 내반경 —")
_prev, _seg0 = None, 0.0
for _s in np.arange(0, PATH.total + 1e-9, 0.020):
    _r = bore_radius(_s)
    _tag = "ø100" if _r > 47 else ("ø90" if _r > 42 else "열림/분기")
    if _tag != _prev:
        if _prev is not None:
            print(f"       s {_seg0 * 1000:7.1f} ~ {_s * 1000:7.1f}mm  {_prev}")
        _prev, _seg0 = _tag, _s
print(f"       s {_seg0 * 1000:7.1f} ~ {PATH.total * 1000:7.1f}mm  {_prev}")

# ── 결함 배치 (중심선 투영) ─────────────────────────────────────────
DEFECTS = []
for k, d in enumerate(CFG["defects"]):
    wp = W(d["p"])
    s_d, off = PATH.project(wp)
    r_bore = bore_radius(s_d)
    DEFECTS.append({"i": k, "s": s_d, "clock": d["clock"],
                    "p_axis": PATH.point_tangent(s_d)[0],
                    "bore_mm": r_bore})
    print(f"[준비] 결함 {k + 1}: map {d['p']} → s={s_d * 1000:.1f}mm "
          f"시계각 {d['clock']:.0f}°(바닥), 중심선 이탈 {off * 1000:.1f}mm, "
          f"실측 내반경 {r_bore:.1f}mm"
          + ("" if r_bore > 47 else "  ⚠ ø100 구간이 아니다"))
for a, b in zip(DEFECTS, DEFECTS[1:]):
    if b["s"] - a["s"] < 0.30:
        print(f"[경고] 결함 {a['i'] + 1}·{b['i'] + 1} 간격 "
              f"{(b['s'] - a['s']) * 1000:.0f}mm — 접근/후진 구간이 겹친다")

def refine_near(prim, centre_w, radius_w, target_w, max_pass=16, cap=80000):
    """개구 자리 삼각형만 국소 세분한다 (최장변 이등분).

    🚨 **맵 메시는 개구를 뚫기에 너무 성기다.** CATIA 변환본의 Sweep3 는
       250mm 직관을 **원주 64분할 × 축방향 1밴드** 로 덮어서 삼각형 한 장이
       250mm 를 가로지른다. 삼각형을 통째로 지우는 방식은 "중심이 개구 안" 인
       장이 하나도 없어 **0개가 잘린다**(실측). repair_demo 의 배관은 같은
       규모를 18,624장으로 덮었다 — 150배 차이다.

    🚨 **1→4 중점 분할을 쓰면 안 된다.** 원본이 250×4.9mm 슬리버라 최장변을
       3mm 까지 줄이면 최단변이 0.04mm 가 되어 삼각형이 폭발한다 — 실측으로
       19만 장까지 갔고 개구 하나에 4.6만 장이 잘렸다(정상은 수백 장).
    → **최장변 이등분(1→2)** 을 쓴다. 긴 쪽만 잘라서 종횡비가 저절로 좋아진다.
      바깥 이웃은 안 쪼개므로 T-정점이 생기지만, 새 정점이 이웃 변 **위에**
      정확히 놓이므로 틈(누수 경로)은 안 생긴다.
    """
    m = UsdGeom.Mesh(prim)
    pts = np.array(m.GetPointsAttr().Get(), dtype=np.float64)
    idx = np.array(m.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
    M = np.array(_XCB.GetLocalToWorldTransform(prim), dtype=np.float64)
    R, T = M[:3, :3], M[3, :3]
    Rinv = np.linalg.inv(R)
    c_l = (centre_w - T) @ Rinv
    scale = float(np.linalg.norm(R[0]))          # 균등 스케일 가정 (0.001)
    r_l, tg_l = radius_w / scale, target_w / scale
    grew = 0
    for _ in range(max_pass):
        v = pts[idx]
        # 🚨 **정점 거리로 고르면 안 된다.** 이 맵의 Sweep3 는 원주 64분할 ×
        #    축방향 1밴드라 삼각형 한 장이 250mm 를 가로지른다. 개구 바로 위를
        #    지나는 장인데도 정점은 120mm 밖에 있어 하나도 안 골렸다(실측).
        #    → 꼭짓점 3 + 변 중점 3 + 무게중심 7점의 최소거리로 고른다.
        mid01 = (v[:, 0] + v[:, 1]) * 0.5
        mid12 = (v[:, 1] + v[:, 2]) * 0.5
        mid20 = (v[:, 2] + v[:, 0]) * 0.5
        cen = v.mean(axis=1)
        probe = np.stack([v[:, 0], v[:, 1], v[:, 2],
                          mid01, mid12, mid20, cen], axis=1)
        near = (np.linalg.norm(probe - c_l, axis=2) < r_l).any(axis=1)
        edge = np.max([np.linalg.norm(v[:, 1] - v[:, 0], axis=1),
                       np.linalg.norm(v[:, 2] - v[:, 1], axis=1),
                       np.linalg.norm(v[:, 0] - v[:, 2], axis=1)], axis=0)
        sel = near & (edge > tg_l)
        if not sel.any() or len(idx) > cap:
            break
        # 어느 변이 최장인지 (0: v0-v1, 1: v1-v2, 2: v2-v0)
        elen = np.stack([np.linalg.norm(v[:, 1] - v[:, 0], axis=1),
                         np.linalg.norm(v[:, 2] - v[:, 1], axis=1),
                         np.linalg.norm(v[:, 0] - v[:, 2], axis=1)], axis=1)
        which = np.argmax(elen, axis=1)
        pts_list = list(pts)
        mid = {}

        def mp(a, b):
            k = (min(a, b), max(a, b))
            if k not in mid:
                mid[k] = len(pts_list)
                pts_list.append((pts[a] + pts[b]) * 0.5)
            return mid[k]

        new = []
        for tri, s, wch in zip(idx, sel, which):
            a, b, c = int(tri[0]), int(tri[1]), int(tri[2])
            if not s:
                new.append((a, b, c))
                continue
            if wch == 1:            # v1-v2 가 최장 → 회전시켜 v0-v1 로 맞춘다
                a, b, c = b, c, a
            elif wch == 2:          # v2-v0 가 최장
                a, b, c = c, a, b
            m_ab = mp(a, b)
            new += [(a, m_ab, c), (m_ab, b, c)]
        pts = np.array(pts_list)
        idx = np.array(new)
        grew += 1
    if grew:
        m.GetPointsAttr().Set([Gf.Vec3f(float(a), float(b), float(c))
                               for a, b, c in pts])
        m.GetFaceVertexIndicesAttr().Set(idx.reshape(-1).tolist())
        m.GetFaceVertexCountsAttr().Set([3] * len(idx))
        m.GetExtentAttr().Set([Gf.Vec3f(*[float(x) for x in pts.min(0)]),
                               Gf.Vec3f(*[float(x) for x in pts.max(0)])])
    return grew, len(idx)


# ── 관통 개구 절단 (결함마다) ───────────────────────────────────────
# 🚨 결함/비드 프림은 시각 전용이라 그것만으로는 물이 안 샌다. 배관 충돌 메시가
#    approximation="none" 삼각 메시이므로 개구 자리 삼각형을 지우면 그대로
#    관통 구멍이 된다. 개구 축은 **결함 지점의 반경 방향**이다.
# 🚨 반쪽을 안 자르면 반대쪽(천장)에도 구멍이 뚫린다 — 축선은 관을 두 번 지난다.
#    실측으로 걸린 적이 있다(삼각형 98개 = 바닥 49 + 천장 49).
plug_tris = []          # 결함마다 (정점, 인덱스) — 메꿈 조각
for d in DEFECTS:
    axis_p = d["p_axis"]
    axis_d = PATH.radial(d["s"], d["clock"])
    # 개구 둘레 60mm 안을 변 3mm 이하로 쪼갠다(ø38 개구가 둥글게 나오는 크기).
    wall_pt = axis_p + axis_d * (d["bore_mm"] * MM)
    for prim in _this:
        # 어느 메시가 개구를 덮는지 미리 거르지 않는다 — 삼각형이 길어서
        # 정점 거리로는 못 가른다(위 refine_near 주석). 안 걸리면 0회로 끝난다.
        g, n_t = refine_near(prim, wall_pt, 0.035, 0.003)
        if g:
            print(f"[준비] 결함 {d['i'] + 1} 개구 자리 세분 — "
                  f"{str(prim.GetPath()).split('/')[-2]} {g}회 → 삼각형 {n_t:,}")
    n_cut, n_far = 0, 0
    for prim in _this:
        m = UsdGeom.Mesh(prim)
        wpts, idx = mesh_world(prim)
        cent = wpts[idx].mean(axis=1)
        rel = cent - axis_p
        along = rel @ axis_d
        perp = np.linalg.norm(rel - np.outer(along, axis_d), axis=1)
        hit = perp < LEAK_PORT_D / 2
        n_far += int(hit.sum())
        # 🚨 개구 축은 **무한 직선**이라 그대로 두면 관 반대쪽 벽은 물론 그
        #    아래 바닥 슬래브까지 뚫는다. 관벽 두께 언저리로 묶는다.
        #    (반대쪽 벽을 안 자르는 것이 첫째 이유 — repair_demo 에서 구멍이
        #     바닥·천장 두 곳에 뚫렸던 그 함정이다.)
        r_w = d["bore_mm"] * MM
        hit &= (along > r_w - 0.015) & (along < r_w + 0.020)
        if not hit.any():
            continue
        n_cut += int(hit.sum())
        plug_tris.append((wpts[idx[hit]], prim))
        keep = idx[~hit]
        m.GetFaceVertexIndicesAttr().Set(keep.reshape(-1).tolist())
        m.GetFaceVertexCountsAttr().Set([3] * len(keep))
    print(f"[준비] 결함 {d['i'] + 1} 관통 개구 ø{LEAK_PORT_D * 1000:.1f}mm — "
          f"삼각형 {n_cut}개 제거 (반대쪽 {n_far - n_cut}개는 남겼다)")
    if n_cut == 0:
        print("       ⚠ 하나도 안 잘렸다 — 결함 좌표나 층 선택을 의심할 것")

# ── 충돌 · 물리 재질 ────────────────────────────────────────────────
# 🔧 진단 손잡이 — 기본은 설계값(v3 §12.3). env 로만 바꾼다.
#    비교 기준: 벨로우즈 12륜이 실제로 완주한 `test_pipe_elbow.usda` 는
#    마찰 0.9/0.8 이었다. 설계 배수값 0.40/0.35 와 2배 이상 차이가 난다.
FRICTION_STATIC = float(os.environ.get(
    "FRICTION_S", 0.30 if FLOODED else 0.40))
FRICTION_DYNAMIC = float(os.environ.get(
    "FRICTION_D", 0.25 if FLOODED else 0.35))
if "FRICTION_S" in os.environ or "FRICTION_D" in os.environ:
    print(f"[진단] 마찰을 설계값이 아닌 {FRICTION_STATIC}/{FRICTION_DYNAMIC} "
          f"로 덮어썼다")
_pm = UsdPhysics.MaterialAPI.Apply(
    UsdShade.Material.Define(stage, "/World/PipePhysMat").GetPrim())
_pm.CreateStaticFrictionAttr(FRICTION_STATIC)
_pm.CreateDynamicFrictionAttr(FRICTION_DYNAMIC)
_pm.CreateRestitutionAttr(0.0)

if GLASS:
    _gl = UsdShade.Material.Define(stage, "/World/Glass")
    _gs = UsdShade.Shader.Define(stage, "/World/Glass/Shader")
    _gs.CreateIdAttr("UsdPreviewSurface")
    _gs.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.55, 0.58, 0.60))
    _gs.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.25)
    _gs.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
    _gl.CreateSurfaceOutput().ConnectToSource(_gs.ConnectableAPI(), "surface")

for p in _this:
    UsdPhysics.CollisionAPI.Apply(p)
    # 🚨 배관은 반드시 approximation="none". convexHull 이면 관 내부가 꽉 찬다.
    UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr("none")
    UsdShade.MaterialBindingAPI.Apply(p).Bind(
        UsdShade.Material.Get(stage, "/World/PipePhysMat"),
        bindingStrength=UsdShade.Tokens.weakerThanDescendants,
        materialPurpose="physics")
for p in _other:
    UsdPhysics.CollisionAPI.Apply(p).CreateCollisionEnabledAttr(False)
# ── 유리(--glass) 는 **배관 면에만** 건다 ──────────────────────────
# 🚨 메시 단위로 바르면 안 된다. 이 맵은 `PartBody1` 한 장에 **벽·바닥·천장이
#    다 들어 있고**, 반대로 진입 라이저(z +85~0)는 `Sweep` 이 아니라 그
#    `PartBody1` 안에만 있다. 메시로 가르면 건물이 통째로 투명해지거나
#    라이저가 불투명하게 남는다 — 둘 다 틀렸다.
# → 이름이 Sweep/Trim/Join 이면 통째로 배관이고(메모리의 분류 규칙),
#   PartBody 는 **중심선까지 거리 35~75mm 인 면만** 뽑아 GeomSubset 으로 칠한다.
if GLASS:
    _n_pipe_mesh, _n_pipe_face = 0, 0
    for p in _this:
        nm = str(p.GetPath()).split("/")[-2]
        if any(k in nm for k in ("Sweep", "Trim", "Join")):
            UsdShade.MaterialBindingAPI.Apply(p).Bind(
                _gl, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
            _n_pipe_mesh += 1
            continue
        wpts, idx = mesh_world(p)
        cen = wpts[idx].mean(axis=1)
        dd = dist_to_path(cen)
        sel = np.where((dd > 0.035) & (dd < 0.075))[0]
        if len(sel) == 0:
            continue
        sub = UsdGeom.Subset.CreateGeomSubset(
            UsdGeom.Mesh(p), "pipe_glass", UsdGeom.Tokens.face,
            Vt.IntArray(sel.tolist()), "materialBind",
            UsdGeom.Tokens.nonOverlapping)
        UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(
            _gl, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
        _n_pipe_face += len(sel)
    print(f"[준비] 유리 — 배관 메시 {_n_pipe_mesh}개 통째 + PartBody 안에 숨은 "
          f"배관 면 {_n_pipe_face:,}개(GeomSubset). 벽·바닥은 불투명 유지")

print(f"[준비] 충돌 — {FLOOR} 메시 {len(_this)}개 삼각 메시(approximation=none), "
      f"다른 층·샤프트 {len(_other)}개는 렌더만(충돌 끔). 관 상태 "
      f"{'만관' if FLOODED else '배수'} 마찰 {FRICTION_STATIC}/{FRICTION_DYNAMIC}"
      + ("   표시 반투명(유리)" if GLASS else ""))
if GLASS:
    print("[경고] --glass 는 사람이 밖에서 보기 위한 표시 옵션이다. 반투명이면 "
          "로봇 조명이 벽에 반사돼 돌아오지 않아 **검출·판정을 믿을 수 없다**")

light = UsdLux.SphereLight.Define(stage, "/World/Light")
light.CreateIntensityAttr(3e6)
light.CreateRadiusAttr(0.05)
_lp = PATH.point_tangent(PATH.total * 0.5)[0]
UsdGeom.Xformable(light).AddTranslateOp().Set(
    Gf.Vec3d(float(_lp[0]), float(_lp[1]), float(_lp[2]) + 0.6))


def load_stl(path):
    data = _P(path).read_bytes()
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


def defect_xform(d):
    """결함 STL(관 축 X, 패치 +Z)을 중심선 위 결함 자리로 보내는 변환.

    회전은 (X→접선, Z→반경) 을 만드는 정규직교 기저로 준다.
    """
    p, t, _, _ = PATH.frame(d["s"])
    z = PATH.radial(d["s"], d["clock"])       # STL 의 +Z 를 반경 방향으로
    x = t                                     # STL 의 +X 를 관 축으로
    y = np.cross(z, x)
    m = Gf.Matrix4d(1.0)
    m.SetRow3(0, Gf.Vec3d(*x))
    m.SetRow3(1, Gf.Vec3d(*y))
    m.SetRow3(2, Gf.Vec3d(*z))
    return m * trans(float(p[0]), float(p[1]), float(p[2]))


# ── 결함 / 비드 / 메꿈 프림 ─────────────────────────────────────────
for d in DEFECTS:
    base = f"/World/Defect{d['i']}"
    UsdGeom.Xform.Define(stage, base)
    xfm = defect_xform(d)
    d["xf"] = xfm
    d["hole"] = make_mesh(f"{base}/hole", SON / "pipe/meshes/defect_hole.stl",
                          color=(0.05, 0.05, 0.06), xform=xfm)
    d["bead"] = make_mesh(f"{base}/bead", SON / "pipe/meshes/bead_hole.stl",
                          color=(0.85, 0.55, 0.20), xform=xfm)
    d["bead_op"] = UsdGeom.Xformable(d["bead"]).GetOrderedXformOps()[0]
    UsdGeom.Imageable(d["bead"]).MakeInvisible()
    # 🔑 벽면 점은 **실측 내반경**으로 만든다. 설계 50mm 를 그대로 쓰면 ø90
    #    구간에서 5mm 틀어지고, 그 오차가 그대로 투영 화소와 정렬 판정에 간다.
    d["r_wall"] = d["bore_mm"] * MM if d["bore_mm"] > 0 else PIPE_IR
    d["world"] = d["p_axis"] + PATH.radial(d["s"], d["clock"]) * d["r_wall"]
    d["tangent"] = PATH.point_tangent(d["s"])[1]

# 메꿈 조각 — 잘라낸 벽 삼각형을 그대로 되살린다.
# 🚨 강체로 만들지 않는다. 강체는 PhysX 가 자세를 쥐고 있어 시뮬 시작 뒤 USD
#    변환이 안 먹고, 키네마틱이면 reset 때 `setLinearVelocity: Body must be
#    non-kinematic!` 가 뜬다. **옮기지 않고** 제자리에 두고 가시성·충돌만 켠다.
for d in DEFECTS:
    tris = [t for t, prim in plug_tris
            if np.linalg.norm(t.reshape(-1, 3).mean(axis=0) - d["world"]) < 0.08]
    if not tris:
        d["plug"] = None
        continue
    T = np.vstack(tris)
    pts, inv = np.unique(np.round(T.reshape(-1, 3), 7), axis=0,
                         return_inverse=True)
    g = UsdGeom.Mesh.Define(stage, f"/World/LeakPlug{d['i']}")
    g.CreatePointsAttr([Gf.Vec3f(float(a), float(b), float(c))
                        for a, b, c in pts])
    g.CreateFaceVertexCountsAttr([3] * len(T))
    g.CreateFaceVertexIndicesAttr(inv.reshape(-1).tolist())
    g.CreateExtentAttr([Gf.Vec3f(*[float(v) for v in pts.min(0)]),
                        Gf.Vec3f(*[float(v) for v in pts.max(0)])])
    g.CreateSubdivisionSchemeAttr("none")
    g.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.55, 0.20)])
    gp = g.GetPrim()
    d["plug"] = gp
    d["plug_coll"] = UsdPhysics.CollisionAPI.Apply(gp)
    d["plug_coll"].CreateCollisionEnabledAttr(False)
    UsdPhysics.MeshCollisionAPI.Apply(gp).CreateApproximationAttr("none")
    UsdGeom.Imageable(gp).MakeInvisible()
    print(f"[준비] 결함 {d['i'] + 1} 메꿈 조각 대기 — 삼각형 {len(T)}개 "
          f"(숨김+충돌끔, 용접 성공 시 켠다)")

# ── 물 (--water) : 결함 주변만 국소 충수 ────────────────────────────
# 🚨 입자·물 값은 새로 만들지 않는다(사용자 지시). PCO 8 / fluid_rest 4.8 /
#    입자-강체 contact 0.004·rest 0.0025 / 간격 9. **채우는 범위만** 줄인다.
#    코스 전장(2.5m)을 채우면 입자가 4만 개를 넘어 감당이 안 된다.
FLOW_V, FLOW_BLEND = -0.10, 0.20
W_SPACING, W_PCO, W_FLUID_REST = 0.009, 0.008, 0.0048
W_R_MAX = 0.045
POCKET_HALF = float(os.environ.get("POCKET_MM", 150.0)) * MM   # 결함 ±150mm
RECYCLE_HZ = float(os.environ.get("RECYCLE_HZ", 15))
RECYCLE_EVERY = max(1, int(round(PHYSICS_HZ / RECYCLE_HZ)))
ISO_PASSES = int(os.environ.get("ISO_PASSES", 2))
water_instancer, n_particles = None, 0
_pockets = []       # (원점, 접선, 반길이) — 직선 가정
if WATER and DEFECTS:
    from omni.physx.scripts import particleUtils            # noqa: E402
    _psys = Sdf.Path("/World/ParticleSystem")
    particleUtils.add_physx_particle_system(
        stage, _psys, particle_contact_offset=W_PCO,
        fluid_rest_offset=W_FLUID_REST, contact_offset=0.004,
        rest_offset=0.0025, max_velocity=5.0, wind=Gf.Vec3f(0, 0, 0))
    particleUtils.add_physx_particle_isosurface(
        stage, _psys, enabled=ISO_PASSES > 0, grid_spacing=W_FLUID_REST * 1.5,
        surface_distance=W_FLUID_REST * 1.6,
        grid_smoothing_radius=W_FLUID_REST * 2.0,
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

    pos, vel = [], []
    for d in DEFECTS:
        p0, t0, _, _ = PATH.frame(d["s"])
        _pockets.append((p0, t0, POCKET_HALF))
        n_ax = int(2 * POCKET_HALF / W_SPACING)
        n_rd = int(2 * W_R_MAX / W_SPACING)
        _a = np.linspace(-POCKET_HALF, POCKET_HALF, n_ax)
        _b = np.linspace(-W_R_MAX, W_R_MAX, n_rd)
        _, _, u0, w0 = PATH.frame(d["s"])
        A, B, C = np.meshgrid(_a, _b, _b, indexing="ij")
        keep = (B ** 2 + C ** 2) <= W_R_MAX ** 2
        P = (p0 + np.outer(A[keep], t0) + np.outer(B[keep], u0)
             + np.outer(C[keep], w0))
        for q in P:
            pos.append(Gf.Vec3f(*[float(v) for v in q]))
            vel.append(Gf.Vec3f(*[float(v) for v in (-t0 * abs(FLOW_V))]))
    n_particles = len(pos)
    _inst = particleUtils.add_physx_particleset_pointinstancer(
        stage, Sdf.Path("/World/WaterParticles"),
        Vt.Vec3fArray(pos), Vt.Vec3fArray(vel), particle_system_path=_psys,
        self_collision=True, fluid=True, particle_group=0,
        particle_mass=0.0, density=1000.0)
    UsdGeom.Sphere(stage.GetPrimAtPath(
        "/World/WaterParticles/particlePrototype0")).GetRadiusAttr().Set(
        W_FLUID_REST)
    water_instancer = UsdGeom.PointInstancer(_inst)
    UsdGeom.Imageable(_inst).MakeInvisible()
    print(f"[준비] 물 — **결함 주변만 국소 충수** {len(DEFECTS)}군데 "
          f"(각 ±{POCKET_HALF * 1000:.0f}mm), 입자 {n_particles:,}개. "
          f"주행 구간은 마른 관이다")
elif WATER:
    print("[준비] --water 를 줬으나 이 층에 결함이 없어 물 주머니를 안 만든다")

# ── 로봇 ────────────────────────────────────────────────────────────
# 🚨 배치는 각 링크 로컬 변환에 굽는다(부모 Xform·articulation root 에 걸면
#    PhysX 가 `Invalid PhysX transform` + NaN 으로 발산한다 — 실측).
# 라이저는 아래로 내려가는 구간이라 진행 방향이 월드 −Z 다. 이 로봇의 전방은
# **로컬 −Z** 이므로 회전이 항등이다(repair_demo 의 Ry(-90) 은 축이 X 인 코스용).
_p_start, _t_start, _u_start, _w_start = PATH.frame(START_S_MM * MM)
_fwd = _t_start
_zc = -_fwd                       # 로봇 로컬 +Z 가 향할 월드 방향
_xc_ = _u_start                   # 로봇 로컬 +X (토치 J1=0 방향)
_yc = np.cross(_zc, _xc_)
_R = Gf.Matrix4d(1.0)
_R.SetRow3(0, Gf.Vec3d(*_xc_))
_R.SetRow3(1, Gf.Vec3d(*_yc))
_R.SetRow3(2, Gf.Vec3d(*_zc))
PLACE = _R * trans(float(_p_start[0]), float(_p_start[1]), float(_p_start[2]))

robot_prim = stage.DefinePrim(ROBOT, "Xform")
robot_prim.GetReferences().AddReference(ROBOT_USDA)
for child in list(robot_prim.GetChildren()):
    if not child.HasAPI(UsdPhysics.RigidBodyAPI):
        continue
    xf = UsdGeom.Xformable(child)
    local = xf.GetLocalTransformation()
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(local * PLACE)
print(f"[준비] 로봇 배치 s={START_S_MM:.0f}mm "
      f"({_p_start[0] * 1000:.0f}, {_p_start[1] * 1000:.0f}, "
      f"{_p_start[2] * 1000:.0f})mm  진행방향 "
      f"({_fwd[0]:+.2f},{_fwd[1]:+.2f},{_fwd[2]:+.2f}) — 수직 하강으로 진입")

# ── 토치 ────────────────────────────────────────────────────────────
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
arc_light = None
if NO_TORCH:
    print("[진단] 토치 없음 — 주행 대조군")
else:
    torch_link("torch_ring", trans(0, 0, RING_Z), WM / "torch_ring.stl",
               torch_spec.MASS_RING_KG, (0.35, 0.35, 0.40))
    torch_link("torch_rod", trans(ROD_ORIGIN_R, 0, RING_Z),
               WM / "torch_rod.stl", torch_spec.MASS_ROD_KG, (0.60, 0.60, 0.65))
    torch_link("torch_tip", trans(ROD_ORIGIN_R + ROD_LEN, 0, RING_Z),
               WM / "torch_tip.stl", torch_spec.MASS_TIP_KG, (0.90, 0.75, 0.30))

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
    _k1, _c1, _f1 = torch_spec.usd_angular()
    d1.CreateStiffnessAttr(_k1)
    d1.CreateDampingAttr(_c1)
    d1.CreateMaxForceAttr(_f1)
    d1.CreateTargetPositionAttr(0.0)

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

    arc_light = UsdLux.SphereLight.Define(stage, f"{ROBOT}/torch_tip/arc")
    arc_light.CreateRadiusAttr(0.002)
    arc_light.CreateIntensityAttr(0.0)
    arc_light.CreateColorAttr(Gf.Vec3f(0.75, 0.85, 1.0))
    UsdGeom.Xformable(arc_light).AddTranslateOp().Set(Gf.Vec3d(0.005, 0, 0))
    print(f"[준비] 토치 — J1 {_k1:.4f} N·m/deg / J2 {_k2:.0f} N/m, "
          f"질량 {torch_spec.MASS_TORCH_KG * 1000:.0f}g")

# ── 카메라 ──────────────────────────────────────────────────────────
_res = os.environ.get("CAM_RES", "640x360").lower().split("x")
CAM_W, CAM_H, CAM_HFOV = int(_res[0]), int(_res[1]), 140.0
CAM_AREA_SCALE = (CAM_W * CAM_H) / (1280.0 * 720.0)
F_PX = (CAM_W / 2.0) / math.radians(CAM_HFOV / 2.0)
CM = SON / "camera" / "meshes"
CAM_SPECS = [("front_camera", "seg1_body", -0.045, True),
             ("back_camera", "seg0_body", +0.037, False)]
# 토치 카메라 — 90° 표준화각으로 시도했다가 결함(관 바닥, 시계각 180°처럼
# 가파른 각도)이 프레임 밖으로 나가 못 잡는 게 실측으로 확인됐다(ARC 중인데도
# 깊이값이 균일 300mm — 빈 통로만 보임). front_camera 와 같은 140° 어안으로
# 맞춘다 — 왜곡은 늘지만 실제로 결함을 잡는 게 우선이다.
TORCH_CAM_HFOV = CAM_HFOV
TORCH_CAM_F_PX = F_PX


def cam_prim_path(nm, seg):
    return f"{ROBOT}/{seg}/{nm}_rig/{nm}"


FRONT_CAM = cam_prim_path("front_camera", "seg1_body")
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
    UsdGeom.Xformable(hm).AddTransformOp().Set(
        rot(90 if fwd else -90, (0, 1, 0))
        * trans(0, 0, z + (0.005 if fwd else -0.005)))
    for k in range(2):
        lg = UsdLux.SphereLight.Define(stage, f"{base}/light_{k}")
        lg.CreateIntensityAttr(4.0e5)
        lg.CreateRadiusAttr(0.002)
        UsdGeom.Xformable(lg).AddTranslateOp().Set(Gf.Vec3d(
            0.012 * (1 if k == 0 else -1), 0.0,
            z + (-0.004 if fwd else 0.004)))

# ── 감지 밴드 ───────────────────────────────────────────────────────
# 🚨 **반드시 여기서 돌린다** — 맵·로봇·토치가 전부 올라온 뒤, world.reset()
#    (cook) 앞이다. 맵 직후에 돌리면 로봇이 아직 없어 휠 콜라이더에 안 닿는다.
_n_off = 0
for _p in stage.Traverse():
    if _p.HasAPI(UsdPhysics.CollisionAPI) or _p.IsA(UsdGeom.Mesh):
        _px = PhysxSchema.PhysxCollisionAPI.Apply(_p)
        _px.CreateContactOffsetAttr(CONTACT_OFFSET)
        _px.CreateRestOffsetAttr(REST_OFFSET)
        _n_off += 1
_n_wheel = sum(1 for _p in stage.Traverse() if "_wheel_" in str(_p.GetPath())
               and _p.HasAPI(PhysxSchema.PhysxCollisionAPI))
print(f"[준비] 주행 {TARGET_SPEED_MPS * 1000:.0f} mm/s → contactOffset "
      f"{CONTACT_OFFSET * 1000:.2f}mm, 프림 {_n_off}개"
      + ("  ※ 물 모드 GPU 하한" if WATER else ""))
print(f"         그중 휠 콜라이더 {_n_wheel}/12"
      + ("" if _n_wheel == 12 else "  ⚠ 12 가 아니다 — 순서가 틀어졌다"))

# 🚨 리셋 뒤에는 USD 드라이브 속성 쓰기가 PhysX 로 안 간다 — 반드시 여기,
#    world.reset() 전에 건다. floor1 진입부(수직 하강 직후 s≈95mm)와 floor2
#    후진 코너에서 겪는 "휠은 도는데 진행이 없다" 끼임에 대한 대응책 —
#    맵의 중복 배관 메시(fix_map.py 이슈 ①, 미해결) 때문에 접지압이 설계보다
#    약할 걸로 보고 피스톤 예압을 올려 붙잡는 힘을 키운다.
PISTON_FORCE_BOOST = float(os.environ.get("PISTON_FORCE_BOOST", 1.0))
if PISTON_FORCE_BOOST != 1.0:
    _n_boost = 0
    for _sg in (0, 1):
        for _i in range(3):
            _jp = stage.GetPrimAtPath(f"{ROBOT}/joints/seg{_sg}_piston_{_i}")
            if not _jp.IsValid():
                continue
            _d = UsdPhysics.DriveAPI.Apply(_jp, "linear")
            _base_force = _d.GetMaxForceAttr().Get() or 9.0
            _d.CreateMaxForceAttr(_base_force * PISTON_FORCE_BOOST)
            _n_boost += 1
    print(f"[준비] 피스톤 예압 {PISTON_FORCE_BOOST:.1f}배 — {_n_boost}개 조인트 "
          f"maxForce {9.0:.1f}N → {9.0 * PISTON_FORCE_BOOST:.1f}N "
          "(real_map_demo.py 전용, robot_bellows.usda 원본 무수정)")

# 🔧 진단용 국소 부스트 — [RETURN진단-다리]로 확인한 s=369mm 정지 지점에서
#    i=1 다리(seg0_piston_1, seg1_piston_1) 둘만 피스톤 0.2/10mm(거의
#    안 눌림)로 접지 상실하는 게 확인됐다(2026-08-07). 스프링이 무부하면
#    자동으로 최대까지 뻗는 드라이브인데 0.2mm에 멈춰 있다는 건 뭔가가
#    누르고 있다는 뜻 — 그 두 다리만 힘을 키워서 뚫고 나가는지 실험한다.
#    (겹친 메시 같은 진짜 고체 콜라이더면 힘을 올려도 안 뚫릴 수 있다.)
LEG1_FORCE_BOOST = float(os.environ.get("LEG1_FORCE_BOOST", 1.0))
if LEG1_FORCE_BOOST != 1.0:
    _n_leg1 = 0
    for _sg in (0, 1):
        _jp = stage.GetPrimAtPath(f"{ROBOT}/joints/seg{_sg}_piston_1")
        if not _jp.IsValid():
            continue
        _d = UsdPhysics.DriveAPI.Apply(_jp, "linear")
        _base_force = _d.GetMaxForceAttr().Get() or 9.0
        _d.CreateMaxForceAttr(_base_force * LEG1_FORCE_BOOST)
        _n_leg1 += 1
    print(f"[준비] i=1 다리 국소 예압 {LEG1_FORCE_BOOST:.1f}배 — {_n_leg1}개 "
          f"조인트 maxForce {9.0:.1f}N → {9.0 * LEG1_FORCE_BOOST:.1f}N (진단용)")

art = SingleArticulation(prim_path=ROBOT, name="bellows_welder")
world.scene.add(art)
world.reset()

dof = list(art.dof_names or [])
wheel_idx = [k for k, n in enumerate(dof) if "_wheel_" in n]
piston_idx = [k for k, n in enumerate(dof) if "_piston_" in n]
piston_name = [n for n in dof if "_piston_" in n]
bel_idx = [k for k, n in enumerate(dof) if n.startswith("bellows_")]
j1_idx = dof.index("torch_j1") if "torch_j1" in dof else None
j2_idx = dof.index("torch_j2") if "torch_j2" in dof else None
print(f"조립  DOF {len(dof)} = 휠 {len(wheel_idx)} / 피스톤 {len(piston_idx)}"
      f" / 벨로우즈 {len(bel_idx)} / 토치 J1,J2   물리 1/{PHYSICS_HZ:.0f}")

_XC = UsdGeom.XformCache()
_seg0 = stage.GetPrimAtPath(f"{ROBOT}/seg0_body")
_seg1 = stage.GetPrimAtPath(f"{ROBOT}/seg1_body")
_ring = stage.GetPrimAtPath(f"{ROBOT}/torch_ring") if not NO_TORCH else _seg1
_tip = stage.GetPrimAtPath(f"{ROBOT}/torch_tip") if not NO_TORCH else _seg1
TIP_END_LOCAL = Gf.Vec3d(0.00497, 0.0, 0.0)


def wpos(prim):
    _XC.Clear()
    t = _XC.GetLocalToWorldTransform(prim).ExtractTranslation()
    return np.array([float(t[0]), float(t[1]), float(t[2])])


def wmat(prim):
    _XC.Clear()
    m = _XC.GetLocalToWorldTransform(prim)
    return np.array([[m[i][j] for j in range(4)] for i in range(4)], float)


s_hint = START_S_MM * MM


def s_of(prim, hint=None):
    global s_hint
    s, _ = PATH.project(wpos(prim), hint if hint is not None else s_hint)
    return s


def radial_r(p, s):
    """중심선 s 를 기준으로 한 점 p 의 반경(mm)."""
    c, t = PATH.point_tangent(s)
    d = p - c
    return float(np.linalg.norm(d - t * np.dot(d, t))) * 1000


def tip_end_world():
    _XC.Clear()
    w = _XC.GetLocalToWorldTransform(_tip).Transform(TIP_END_LOCAL)
    return np.array([float(w[0]), float(w[1]), float(w[2])])


def tip_end_r(s):
    return radial_r(tip_end_world(), s)


def make_spark_confine(s0, half=0.15, step=0.005):
    """결함 주변 중심선을 샘플링해 스파크 가둠 함수를 만든다.

    🔑 직선 원기둥으로 근사하면 안 된다 — 이 맵은 결함 120mm 앞에서 관이
       꺾이는 자리도 있어서, 무한 직선으로 가두면 스패터가 곡관 벽을 뚫고
       건물 안으로 날아간다. 중심선 점 61개(±150mm, 5mm 간격)에 대한 최근접
       거리로 재면 굽은 구간도 그대로 따라간다.
    샘플 범위를 벗어난 스패터는 끝점 기준 거리가 커져 저절로 되튕긴다.
    `bore_radius` 는 레이캐스트 12발이라 **여기서 한 번만** 부른다.
    """
    ss = np.arange(max(0.0, s0 - half),
                   min(PATH.total, s0 + half) + 1e-9, step)
    pts = np.array([PATH.point_tangent(float(s))[0] for s in ss])
    R = bore_radius(s0) * MM

    def confine(p):
        d = p[:, None, :] - pts[None, :, :]
        j = np.argmin(np.einsum("ijk,ijk->ij", d, d), axis=1)
        v = p - pts[j]
        r = np.linalg.norm(v, axis=1)
        return r, v / np.maximum(r, 1e-12)[:, None], R

    return confine


def wall_inward(p, s):
    """점 p 에서 **관 안쪽을 향하는** 단위벡터. 스패터가 튀어 나가는 쪽이다."""
    c, t = PATH.point_tangent(s)
    d = np.asarray(p, dtype=float) - c
    rad = d - t * float(np.dot(d, t))
    n = float(np.linalg.norm(rad))
    return (-rad / n) if n > 1e-9 else np.array([0.0, 0.0, 1.0])


def drive(deg_s):
    art.apply_action(ArticulationAction(
        joint_velocities=np.array([math.radians(deg_s)] * len(wheel_idx)),
        joint_indices=np.array(wheel_idx)))


def set_torch(j1_deg=None, j2_m=None):
    if j1_idx is None:
        return
    idx, val = [], []
    if j1_deg is not None:
        idx.append(j1_idx); val.append(math.radians(j1_deg))
    if j2_m is not None:
        idx.append(j2_idx); val.append(j2_m)
    art.apply_action(ArticulationAction(joint_positions=np.array(val),
                                        joint_indices=np.array(idx)))


def j1_target_for(d):
    """결함 반경 방향을 **로봇 seg1 로컬** 로 옮겨 J1 지령을 만든다.

    🚨 repair_demo 는 `set_torch(j1_deg=DEFECT_CLOCK_DEG)` 로 월드 시계각을
       그대로 줬는데, 그건 배치가 고정(Ry(-90))이고 관 축이 X 라서 로봇 롤이
       0 으로 유지됐기 때문이다. 실전 맵은 하강 → 곡관 → 수평으로 자세가 바뀌고
       롤도 물리가 정한다. **측정한 자세로 매번 다시 계산해야 한다.**
    """
    M = wmat(_seg1)
    R = M[:3, :3]                       # 행벡터 규약: world = local @ R
    dwld = PATH.radial(d["s"], d["clock"])
    dloc = R @ dwld                     # R 이 직교라 R⁻¹ = Rᵀ, local = R @ world
    return math.degrees(math.atan2(float(dloc[1]), float(dloc[0])))


# ── 유체력 (--fluid) ────────────────────────────────────────────────
_fluid_view = None
if FLUID:
    from isaacsim.core.prims import RigidPrim               # noqa: E402
    _fluid_view = RigidPrim([f"{ROBOT}/seg0_body", f"{ROBOT}/seg1_body"],
                            name="fluid_body")
    _fluid_view.initialize()
    F_BUOY = RHO * G * V_DISP
    F_DRAG_DESIGN = 0.5 * RHO * CD_EFF * A_FRONTAL * V_FLOW_DESIGN ** 2
    F_TRACTION = 6 * 9.0 * FRICTION_STATIC
    print(f"[준비] 만관 유체력(해석적) — 부력 {F_BUOY:.2f}N, 항력 "
          f"{F_DRAG_DESIGN:.2f}N vs 견인 {F_TRACTION:.2f}N")


def apply_fluid(s_now):
    """항력(코스 접선) + 부력(연직 위). apply_forces 는 1스텝만 유효하다."""
    if _fluid_view is None:
        return 0.0
    pos = _fluid_view.get_world_poses()[0]
    vel = _fluid_view.get_linear_velocities()
    f_all = []
    for k in range(len(pos)):
        s, _ = PATH.project(pos[k], s_now)
        _, t = PATH.point_tangent(s)
        v_r = float(np.dot(vel[k], t))
        rel = (-V_FLOW_DESIGN) - v_r
        f = 0.5 * RHO * CD_EFF * (A_FRONTAL / 2.0) * rel * abs(rel)
        f_all.append(np.array([f * t[0], f * t[1], f * t[2] + F_BUOY / 2.0]))
    _fluid_view.apply_forces(np.array(f_all, dtype=np.float32), is_global=True)
    return float(sum(np.dot(f[:3], [1, 1, 0]) for f in f_all))


# ── 카메라 활성 ─────────────────────────────────────────────────────
rigs = []
TORCH_CAM_OK = False        # aim_torch_camera 가 정의됐을 때만 True
try:
    import omni.replicator.core as rep
    from isaacsim.sensors.camera import Camera
    OUT.mkdir(parents=True, exist_ok=True)
    for nm, seg, z, fwd in CAM_SPECS:
        cam = Camera(prim_path=cam_prim_path(nm, seg),
                     translation=np.array([0.0, 0.0, z]),
                     frequency=10, resolution=(CAM_W, CAM_H))
        cam.initialize()
        # 화면 위 = 로봇 로컬 +X 가 되도록 롤만 준다(광축은 그대로).
        _SQ = 0.70710678
        cam.set_local_pose(
            orientation=np.array([_SQ, 0.0, 0.0, -_SQ] if fwd
                                 else [0.0, _SQ, _SQ, 0.0]), camera_axes="usd")
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
        dep = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
        dep.attach(rp)
        rigs.append((nm, ann, dep))
        if nm == "front_camera" and os.environ.get("CAM_DIAG"):
            # 🔎 대조군: 이미 실전에서 검증된 front_camera 로 "camera_axes=usd"
            #   사원수 → 실제 월드 광축" 변환 관계 자체를 실측 검산한다.
            _fcp = stage.GetPrimAtPath(cam_prim_path(nm, seg))
            _fc_R = wmat(_fcp)[:3, :3]
            _fc_fwd = -_fc_R[:, 2]; _fc_fwd /= np.linalg.norm(_fc_fwd)
            _seg1_fwd = wmat(_seg1)[:3, :3] @ np.array([0, 0, -1.0])
            _seg1_fwd /= np.linalg.norm(_seg1_fwd)
            _ang = math.degrees(math.acos(
                np.clip(np.dot(_fc_fwd, _seg1_fwd), -1, 1)))
            print(f"[진단-대조군] front_camera 광축 {_fc_fwd}, "
                  f"seg1 로컬-Z(기대 전진방향) {_seg1_fwd}, 각도차 {_ang:.1f}°")
            _fc_ops = [p for p in _fcp.GetPropertyNames() if p.startswith("xformOp")]
            print(f"[진단-xformOp] front_camera ops={_fc_ops} "
                  f"orient={_fcp.GetAttribute('xformOp:orient').Get() if 'xformOp:orient' in _fc_ops else 'MISSING'} "
                  f"orderAttr={_fcp.GetAttribute('xformOpOrder').Get()}")
    print(f"[준비] 카메라 2대 (어안 {CAM_HFOV:.0f}°, {CAM_W}x{CAM_H}, "
          f"f={F_PX:.1f}px) + RGB/Depth 어노테이터")

    if not NO_TORCH:
        # 🚨 2026-08-07 두 번째 재설계 — torch_tip(조인트 체인 하류) 부착을
        #   포기하고 **seg1_body**(front_camera 와 같은 부착점)로 옮긴다.
        #   torch_ring/torch_tip 둘 다 시도했지만(STL 형상 추측 48.7° 오차,
        #   SETTLE 후 실측 R_tip 재확인해도 69.8° 오차) 매번 광축이 로컬
        #   회전과 무관하게 거의 고정된 방향으로 나왔다 — front_camera(같은
        #   set_local_pose 패턴, 대조군 각도차 0.0°)는 정상 작동하는 것으로
        #   보아, Isaac Sim/PhysX 에서 **articulation 조인트 체인 하류
        #   링크(torch_ring/torch_tip)에 붙인 자식 프림은 로컬 xformOp:orient
        #   가 렌더/월드 합성에 반영되지 않는** 구조적 한계로 보고 우회한다
        #   (사용자 확인 후 결정 — J1 을 따라 돌지는 못하지만 확실히
        #   작동하는 쪽을 선택).
        #   front_camera 와 똑같이 검증된 마운트 패턴(seg1_body 자식, 같은
        #   "전진" 사원수)을 그대로 재사용한다 — 다만 화각은 핀홀 90°로
        #   좁혀서(TORCH_CAM_HFOV) 용접부를 더 가깝게 당겨 보여준다.
        _tc_base = f"{ROBOT}/seg1_body/torch_camera_rig"
        UsdGeom.Xform.Define(stage, _tc_base)
        # 🚨 front/back 카메라는 CAM_SPECS 루프에서 전용 조명(SphereLight 2개,
        #   4e5) 을 같이 붙이는데, 이 카메라는 그 루프 밖에서 따로 만들어서
        #   조명이 없었다 — 계속 어둡게 나온 원인 중 하나(사용자 지적).
        #   같은 레시피로 조명을 붙였는데도 front_camera 보다 훨씬 어둡게
        #   나왔다(실측: 결함 테두리·스파크 하나만 겨우 보임) — 4e5 로는
        #   부족해 5배(2e6) 로 올린다.
        # 🚨 z=-0.040 (front_camera 의 -0.045 보다 5mm 몸체 쪽으로 더 안쪽) 로
        #   뒀더니 화면 대부분이 아주 가까운 거리(평균 2.3mm)로 나왔다 —
        #   seg1_body 자체(또는 그 앞의 다른 부품)에 파고든 것으로 보인다
        #   (사용자가 GUI 로 직접 보고 지적). front_camera 자리엔 카메라가
        #   내다볼 수 있는 자리(하우징/개구부)가 확실히 있는 반면, 5mm 어긋난
        #   지점엔 그게 없을 수 있다 — front_camera 와 완전히 같은 z 로 맞춘다.
        for _k in range(2):
            _lg = UsdLux.SphereLight.Define(stage, f"{_tc_base}/light_{_k}")
            _lg.CreateIntensityAttr(0.8e6)   # 한 단계 더 낮춤 (2e6→1.2e6→0.8e6)
            _lg.CreateRadiusAttr(0.002)
            UsdGeom.Xformable(_lg).AddTranslateOp().Set(
                Gf.Vec3d(0.012 * (1 if _k == 0 else -1), 0.0, -0.049))
        TORCH_CAM_PATH = f"{_tc_base}/torch_camera"
        tcam = Camera(prim_path=TORCH_CAM_PATH,
                     translation=np.array([0.0, 0.0, -0.045]),
                     frequency=10, resolution=(CAM_W, CAM_H))
        tcam.initialize()
        _SQ2 = 0.70710678
        tcam.set_local_pose(
            orientation=np.array([_SQ2, 0.0, 0.0, -_SQ2]), camera_axes="usd")
        tcam.set_clipping_range(0.005, 5.0)
        tcam.set_focal_length(3.0 * TORCH_CAM_F_PX * 1e-6)
        tcam.set_horizontal_aperture(3.0 * CAM_W * 1e-6)
        try:
            tcam.set_opencv_fisheye_properties(
                cx=CAM_W / 2, cy=CAM_H / 2, fx=TORCH_CAM_F_PX, fy=TORCH_CAM_F_PX,
                fisheye=[0.0, 0.0, 0.0, 0.0])
        except Exception as exc:
            print(f"[경고] torch_camera 어안 설정 실패({exc}) — 핀홀로 진행")
        trp = tcam.get_render_product_path()
        tann = rep.AnnotatorRegistry.get_annotator("rgb")
        tann.attach(trp)
        tdep = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
        tdep.attach(trp)
        rigs.append(("torch_camera", tann, tdep))
        print(f"[준비] 토치 카메라 1대 (어안 {TORCH_CAM_HFOV:.0f}°, {CAM_W}x{CAM_H}, "
              f"f={TORCH_CAM_F_PX:.1f}px) — seg1_body 부착, ALIGN/EXTEND/ARC "
              f"동안 매 틱 팁을 실시간 재조준")

        # 🚨 22mm 로는 토치 링(반경 30mm) 바로 옆이라 링 자체가 시야를 가려
        #   용접점이 프레임 가장자리로 밀려났다(사용자가 스크린샷으로 확인,
        #   /home/rokey/Pictures/torch.png) — 링을 확실히 벗어나도록 38mm 로.
        TORCH_CAM_BODY_RADIUS = 0.038

        def aim_torch_camera(target_world):
            """seg1 에 붙은 torch_camera 를 target_world 쪽으로 매 틱 재조준한다.

            🚨 torch_ring/torch_tip(조인트 체인 하류)에 직접 붙이면
               set_local_pose 의 로컬 회전이 렌더에 반영되지 않는 Isaac Sim
               한계에 부딪혔다(48.7~69.8° 오차, 실측 확인). seg1_body 는
               articulation 루트에 가까운 바디라 방향 지정이 정상 동작한다
               (front_camera 대조군 0.0°) — 대신 **정지된 로컬 오프셋** 하나로
               는 로봇이 정렬·용접하며 자세를 바꾸는 동안 계속 못 맞으므로
               (사용자 지적: "정확히 토치쪽을 바라봐야함"), ALIGN/EXTEND/ARC
               매 틱 target_world(팁 끝 실측 위치)를 다시 계산해 unroll 한다.
            🚨 첫 시도는 방향을 **seg1 원점**에서 계산했는데, 원점이 토치
               장착 위치(RING_Z, 축방향 -108mm)에서 한참 떨어져 있어 반경
               성분(~45mm)보다 축방향 성분(~108mm)이 훨씬 커서 사실상 계속
               정면(전진 방향)을 보고 있었다 — 사용자가 GUI 로 보고 "조준점이
               벗어난다"고 지적해서 확인함. 토치가 실제로 달린 **축방향
               위치**(seg1 로컬 z=RING_Z)를 기준점으로 삼아야 반경 방향이
               제대로 나온다.
            """
            R_seg1 = wmat(_seg1)[:3, :3]
            ref_local = np.array([0.0, 0.0, RING_Z])   # 토치 장착 축방향 위치
            ref_world = wpos(_seg1) + R_seg1 @ ref_local
            f_world = target_world - ref_world
            dist = np.linalg.norm(f_world)
            if dist < 1e-6:
                return
            f_world = f_world / dist
            u_world = np.array([0.0, 0.0, 1.0])
            if abs(np.dot(f_world, u_world)) > 0.9:   # 거의 수직이면 대체 up
                u_world = np.array([1.0, 0.0, 0.0])
            # 🚨 팁을 정확히 화면 중앙에 두면 화면이 살짝 아래로 처져 보인다는
            #    지적(스크린샷 torch1.png) — 광축을 up 쪽으로 살짝 틀어(0.28,
            #    ≈16°) 결함이 화면 아래쪽에 오도록 했다. 더 크게(0.55) 틀어
            #    보라는 줄 알았는데 — 아니었다: "기울임은 원복하고, 카메라를
            #    **회전**시켜서 결함이 아래로 오게" 였다(torch3.png) — 광축
            #    방향(pitch) 이 아니라 광축을 축으로 한 롤(roll) 회전이 필요
            #    하다는 뜻. 0.28 로 되돌리고, 별도로 롤을 추가한다.
            f_world = f_world + u_world * 0.28
            f_world /= np.linalg.norm(f_world)
            right_world = np.cross(f_world, u_world)
            right_world /= np.linalg.norm(right_world)
            up_world2 = np.cross(right_world, f_world)
            # 🚨 롤(roll) — 광축(f_world)을 축으로 right/up 을 같이 돌린다
            #    (Rodrigues, 둘 다 f_world 에 수직이라 회전은 cos/sin 항만).
            #    사용자 지정: "반시계 방향으로 90도" — 부호가 반대로 나오면
            #    (실제로 시계방향으로 보이면) 이 각도만 -90 으로 뒤집으면 된다.
            _roll = math.radians(90.0)
            _rc, _rs = math.cos(_roll), math.sin(_roll)
            _right_rolled = right_world * _rc + np.cross(f_world, right_world) * _rs
            _up_rolled = up_world2 * _rc + np.cross(f_world, up_world2) * _rs
            M_world = np.column_stack([_right_rolled, _up_rolled, -f_world])
            M_local = R_seg1.T @ M_world
            local_pos = ref_local + R_seg1.T @ (f_world * TORCH_CAM_BODY_RADIUS)
            tcam.set_local_pose(
                translation=local_pos,
                orientation=quat_from_matrix_wxyz(M_local), camera_axes="usd")
        TORCH_CAM_OK = True
except Exception as exc:
    print(f"[경고] 카메라 초기화 실패 — 검출·판정 불가 ({exc})")
    rigs = []

# ── 상태별 활성 카메라 ROS2 발행 ────────────────────────────────────
# 🚨 camera/rig.py 의 CameraBridge 와 달리 카메라마다 별도 토픽을 쓰지
#    않는다 — 관리자가 볼 화면은 하나면 되니, **한 토픽에 그때그때 활성
#    카메라 영상을 갈아 끼워서** 내보낸다(전진→front, 후진→back, 용접
#    관련(ALIGN~VERIFY)→torch). rig.py 처럼 GUI 필수(headless 는 프레임이
#    안 나옴, HANDOFF.md 기록된 함정) — NO_ROS 로 이미 그 경우를 걸렀다.
_RIGS_BY_NAME = {nm: (ann, dep) for nm, ann, dep in rigs}


def active_camera_name(fsm_state):
    if fsm_state in ("RETURN", "RECOVER"):
        return "back_camera"
    # 🚨 용접(ARC)이 끝나면 바로 front_camera 로 돌아온다(사용자 지적) —
    #    COOL/REPOSITION/VERIFY 는 식히고 물러나 검증 촬영하는 사후 단계라
    #    torch_camera 에 계속 머물 이유가 없다. torch_camera 는 정렬~용접
    #    (ALIGN/EXTEND/ARC) 동안만 쓴다.
    if fsm_state in ("ALIGN", "EXTEND", "ARC"):
        return "torch_camera"
    return "front_camera"


cam_bridge = None
if not NO_ROS:
    class ActiveCamBridge(Node):
        """그때그때 활성 카메라 하나만 압축 JPEG 로 발행한다."""

        def __init__(self, rigs_by_name):
            super().__init__("repair_robot_active_cam")
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
            self.rigs = rigs_by_name
            self.pub_rgb = self.create_publisher(
                CompressedImage, "/repair_robot/active_cam/rgb/compressed", qos)
            self.pub_which = self.create_publisher(
                String, "/repair_robot/active_cam/which", qos)
            # 🚨 2026-08-08 — OpenCV(find_wall_hole/find_weld_bead) 판정을
            #    시각화해 발행한다. YOLO 디버그(/defect/debug_image/
            #    compressed, active_cam_vision_node.py)와는 별개 토픽 —
            #    이쪽은 Depth 필터·expect_px 매칭까지 반영된 **실제 판정
            #    근거**고, INSPECT/RECHECK/VERIFY 촬영 순간에만 갱신된다
            #    (active_cam 처럼 매 프레임 나오지 않는다).
            self.pub_opencv_debug = self.create_publisher(
                CompressedImage, "/repair_robot/opencv_debug/compressed", qos)
            # 🚨 2026-08-08 — 위 이미지는 판정 순간의 정지 스냅샷이라 뷰어가
            #    그걸로 화면을 통째로 갈아치우면 "얼어붙는" 느낌이 난다
            #    (사용자 지적). 뷰어가 **실시간 raw 프레임 위에 직접 겹쳐
            #    그릴 수 있게** hole/bead 판정 자체(좌표·반경·매칭 여부)를
            #    JSON 으로도 따로 발행한다 — 이미지 토픽은 archival/--shots
            #    용으로 남겨둔다.
            self.pub_opencv_judge = self.create_publisher(
                String, "/repair_robot/opencv_judgement/json", qos)
            self.n = 0
            # 🚨 2026-08-07 — YOLO 2차검증(active_cam_vision_node.py, PC2)의
            #    의견을 받는다. PipeVisionBridge(front_camera 고정, Depth/
            #    Odom/IMU 동기화 필요) 를 걷어내면서 이쪽으로 옮겼다 — 한
            #    로봇이 정찰+수리를 동시에 하니 축 위치 추적이 필요 없어져,
            #    active_cam(전진→front/후진→back/용접→torch)에 YOLO만 얹은
            #    가벼운 노드로 충분하다(설계 상의, 2026-08-07). 절대 이걸로
            #    OpenCV 판정을 대체하지 않는다 — 항상 참고용 비동기 의견.
            self.sub_detected = self.create_subscription(
                Bool, "/defect/detected", self._on_detected, qos)
            self.sub_report = self.create_subscription(
                String, "/defect/report_json", self._on_report, 10)
            self.last_detected, self.last_detected_t = None, 0.0
            self.last_report, self.last_report_t = None, 0.0

        def _on_detected(self, msg):
            self.last_detected, self.last_detected_t = bool(msg.data), time.time()

        def _on_report(self, msg):
            try:
                self.last_report = json.loads(msg.data)
                self.last_report_t = time.time()
            except (ValueError, TypeError):
                pass

        def yolo_opinion(self, max_age_s=3.0):
            """최근 max_age_s 초 이내 YOLO 의견 — (detected: bool|None, report: dict|None).

            너무 오래된(YOLO 가 안 돌고 있거나 느린) 값은 None 으로 취급해
            "의견 없음"과 "결함 없다고 봄" 을 헷갈리지 않게 한다.
            """
            now = time.time()
            detected = (self.last_detected
                       if (now - self.last_detected_t) <= max_age_s else None)
            report = (self.last_report
                     if (now - self.last_report_t) <= max_age_s else None)
            return detected, report

        def publish(self, name):
            rig = self.rigs.get(name)
            if rig is None:
                return
            ann, _dep = rig
            c = ann.get_data()
            if c is None or not getattr(c, "size", 0):
                return
            a = np.asarray(c)[:, :, :3].astype(np.uint8)
            ok, buf = cv2.imencode(".jpg", a[:, :, ::-1],
                                   [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                return
            stamp = self.get_clock().now().to_msg()
            m = CompressedImage()
            m.header.stamp = stamp
            m.header.frame_id = name
            m.format = "jpeg"
            m.data = buf.tobytes()
            self.pub_rgb.publish(m)
            wm = String(); wm.data = name
            self.pub_which.publish(wm)
            self.n += 1

        def publish_opencv_debug(self, img_rgb, frame_name):
            """draw_inspect_debug() 가 그린 이미지를 그대로 발행한다(archival/--shots 대응)."""
            ok, buf = cv2.imencode(
                ".jpg", np.ascontiguousarray(img_rgb)[:, :, ::-1],
                [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                return
            m = CompressedImage()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = frame_name
            m.format = "jpeg"
            m.data = buf.tobytes()
            self.pub_opencv_debug.publish(m)

        def publish_opencv_judgement(self, payload):
            """연속(실시간) OpenCV 판정 좌표·근거를 JSON 으로 발행한다.

            뷰어가 이걸로 **살아있는 raw 프레임 위에 직접** 원을 그린다 —
            표시 전용이다. 임무가 실제로 결함을 등록·확정하는 시점은 지금도
            INSPECT/RECHECK/VERIFY 세 번뿐이고(§5.3 절차, inspect_defect()),
            이 실시간 스트림은 그 판정을 바꾸지 않는다.
            """
            m = String()
            m.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
            self.pub_opencv_judge.publish(m)

    rclpy.init()
    cam_bridge = ActiveCamBridge(_RIGS_BY_NAME)
    print("[준비] ROS2 활성 카메라 발행 — /repair_robot/active_cam/rgb/compressed "
          "(전진→front_camera, 후진→back_camera, 용접 관련→torch_camera)")
else:
    print(f"[경고] ROS2 카메라 발행 끔 ({NO_ROS_REASON})")

CAM_PUBLISH_HZ = 10.0
_cam_pub_last = 0.0
_judge_pub_last = 0.0
_judge_exc_last_print = 0.0

# 🚨 2026-08-07 — 예전엔 여기 pipe_inspect_demo::pipe_vision_node(YOLO)용
#    RGB+Depth+CameraInfo+Odom+IMU 5-토픽 동기화 묶음(PipeVisionBridge,
#    front_camera 고정)이 있었다. 정찰+수리를 한 로봇이 동시에 하면서 축
#    위치 추적(Odom/IMU)이 필요 없어져 — active_cam(전진→front/후진→back/
#    용접→torch)에 YOLO만 얹은 가벼운 노드(active_cam_vision_node.py, PC2)
#    로 교체했다. YOLO 2차검증 의견(/defect/detected,/defect/report_json)
#    구독은 ActiveCamBridge 로 옮겼다(위 클래스, yolo_opinion() 참고).


def _log_yolo_opinion(d_cur, tag):
    """OpenCV 가 방금 찾은 결함(d_cur)에 대해 YOLO(active_cam_vision_node) 의
    최근 의견을 나란히 찍는다 — **판정을 바꾸지 않는다**, 참고용 로그일 뿐이다.
    YOLO 가 안 돌고 있거나(PC2 미실행) 응답이 오래됐으면(3초 초과) "의견
    없음"으로 찍고 그대로 진행한다 — OpenCV 판정 흐름은 절대 안 막는다.
    """
    if cam_bridge is None or d_cur is None:
        return
    detected, report = cam_bridge.yolo_opinion()
    if detected is None:
        print(f"  [YOLO 2차검증·{tag}] 의견 없음 (최근 3초 내 수신 안 됨 "
              f"— PC2/pipe_vision 미실행이거나 느림)")
        return
    line = f"  [YOLO 2차검증·{tag}] {'동의(검출함)' if detected else '불일치(YOLO 는 못 봄)'}"
    if report is not None:
        try:
            yolo_s_m = float(report["defect"]["axial_position_from_entry_m"])
            diff_mm = abs(yolo_s_m - d_cur["s"]) * 1000.0
            line += (f", YOLO 축위치 {yolo_s_m * 1000:.0f}mm "
                    f"(OpenCV {d_cur['s'] * 1000:.0f}mm, 차 {diff_mm:.0f}mm)")
        except (KeyError, TypeError, ValueError):
            pass
    print(line)


sys.path.insert(0, str(SON))
from condition.detector import PipeConditionDetector      # noqa: E402
from welder.weld import WeldSequencer                     # noqa: E402
from welder.spark_fx import SparkFX                       # noqa: E402
from driver.rod_budget import RodBudget                   # noqa: E402

# ── 아크 스파크 (시각 전용) ────────────────────────────────────────
# 🚨 헤드리스에서는 만들지 않는다 — 렌더가 없어 보이지도 않는데 계산만 들고,
#    판정 경로(INSPECT/VERIFY 촬영)를 건드릴 이유가 없다. 물리에는 어느
#    모드에서도 관여하지 않는다(콜라이더 없는 PointInstancer 하나).
# 🚨 **매질 판정은 FLOODED 가 아니라 "입자 물이 실제로 있는가"(WATER) 다**
#    (2026-08-06, repair_demo 와 같은 이유). --fluid 는 해석적 힘뿐이라 화면에
#    물이 없는데 스패터만 수중 거동이면 꺼진 것처럼 보인다. SPARK_WET=1 로 강제.
SPARK_WET = os.environ.get("SPARK_WET", "1" if WATER else "0") == "1"
sparks = None if (HEADLESS or NO_TORCH) else SparkFX(
    stage, "/World/weld_sparks", flooded=SPARK_WET)
if sparks is not None:
    print(f"[준비] 아크 스파크 — {'수중(급냉·단거리)' if SPARK_WET else '기중'} "
          f"조건, 시각 전용(물리·판정 무관)"
          + ("" if SPARK_WET == FLOODED else
             "   ※ 관은 충수지만 보이는 물이 없어 기중 거동으로 그린다"))

INTR = {"fx": F_PX, "fy": F_PX, "ppx": CAM_W / 2.0, "ppy": CAM_H / 2.0,
        "f_fish": F_PX}
detector = PipeConditionDetector(
    INTR, params={"aperture_min_px": 400 * CAM_AREA_SCALE})
sequencer = WeldSequencer()
ALIGN_TOL_MM = float(sequencer.k["align_tol_mm"])
# 🚨 이 맵의 DEFECTS 는 위치·시계각만 갖고 길이/폭/깊이가 없다(§8.1 공식을
#    쓸 실측치가 없다) — default_repair_mm 근사 경로로 예산을 추적한다.
#    ROD_TOTAL_MM 은 시험용(작게 줘서 소진→복귀를 짧은 런에서 재현한다).
rod_budget = RodBudget(dict(
    total_mm=float(os.environ.get("ROD_TOTAL_MM", 1880.0)),
    default_repair_mm=float(os.environ.get("ROD_DEFAULT_MM", 100.0))))
print(f"[준비] 용접봉 예산 — 총 {rod_budget.k['total_mm']:.0f}mm, "
      f"예비 {rod_budget.k['reserve_mm']:.0f}mm, 결함당 근사 "
      f"{rod_budget.k['default_repair_mm']:.0f}mm (§8.1 미계측 근사)")
defect_geom = {}
HOLE_DARK_FRAC = 0.12
HOLE_MIN_PX = max(60, 300 * CAM_AREA_SCALE)
BEAD_SAT_MIN, BEAD_VAL_MIN = 60, 40
BEAD_MIN_PX = max(20, 100 * CAM_AREA_SCALE)
print(f"[준비] 검출·판정 활성 — 정렬 허용 {ALIGN_TOL_MM}mm")


def find_wall_hole(rgb, expect_px=None, depth=None, near_m=0.22,
                   win_px=70):
    """근접 벽면의 구멍 — 밝은 벽에 둘러싸인 어두운 덩어리.

    🚨 **전방 개구부(관 저 끝)를 반드시 걸러야 한다.** repair_demo 는 "화면
       중앙 덩어리" 를 빼는 방식이었는데, 그건 **저 끝이 화면 중앙에 있다는
       전제**다. 실전 맵에서는 결함 120mm 앞에서 관이 꺾여 저 끝이 화면
       옆으로 밀리고, 그러면 그게 통째로 "구멍" 으로 잡힌다 — 실측에서
       8,182px 짜리 저 끝(221,206)이 결함으로 보고됐다.
    → **Depth 로 가른다.** 벽면 구멍은 테두리가 근접 벽(≈0.1m)이고, 관 저
       끝은 테두리가 먼 벽(0.3~0.5m)이다. 이건 관이 꺾이든 말든 성립한다.
       Depth 가 없으면 예전 방식(중앙 덩어리 제외)으로 물러난다.
    """
    g_full = cv2.medianBlur(cv2.cvtColor(
        np.asarray(rgb)[:, :, :3].astype(np.uint8), cv2.COLOR_RGB2GRAY), 5)
    H, Wd = g_full.shape
    # 🚨 **전역 이진화는 이 맵에서 성립하지 않는다.** 실전 맵은 관이 곧지 않고
    #    조명이 한쪽으로 몰려 화면 절반이 통째로 어둡다. 그러면 결함 덩어리가
    #    그 어두운 배경과 **하나로 이어져** 25% 상한에 걸려 버려진다(실측:
    #    구멍이 눈에 보이는데 "구멍 없음" 이 나왔다).
    # → 결함이 보여야 할 자리 둘레만 잘라서 **그 창의 분포로** 임계를 잡는다.
    #   창 밖은 아예 안 본다. (자기충족이 아니다 — 창 안에 어두운 덩어리가
    #   실제로 없으면 아무것도 안 잡히고, 수리가 되면 정확히 그렇게 된다.)
    if expect_px is not None:
        x0 = int(max(0, expect_px[0] - win_px))
        x1 = int(min(Wd, expect_px[0] + win_px))
        y0 = int(max(0, expect_px[1] - win_px))
        y1 = int(min(H, expect_px[1] + win_px))
    else:
        x0, y0, x1, y1 = 0, 0, Wd, H
    g = g_full[y0:y1, x0:x1]
    if g.size == 0:
        return None
    h, w = g.shape
    lo, hi = np.percentile(g, 5), np.percentile(g, 90)
    thr = lo + HOLE_DARK_FRAC * (hi - lo)
    n, lab, st, ce = cv2.connectedComponentsWithStats((g < thr).astype(np.uint8), 8)
    bore = set()
    rr = max(4, int(0.10 * min(h, w))) if expect_px is None else 0
    for yy in range(max(0, h // 2 - rr), min(h, h // 2 + rr), 3):
        for xx in range(max(0, w // 2 - rr), min(w, w // 2 + rr), 3):
            if lab[yy, xx]:
                bore.add(int(lab[yy, xx]))
    dz = None
    if depth is not None:
        _d = np.asarray(depth, dtype=np.float32)
        if _d.shape == g_full.shape:
            dz = _d[y0:y1, x0:x1]
    big, best = 0.60 * h * w, None
    for i in range(1, n):
        # 두 겹을 다 쓴다 — 중앙 덩어리 제외(관이 곧을 때)와 테두리 깊이
        # (관이 꺾일 때). 어느 하나만으로는 이 맵에서 저 끝을 못 가른다.
        if i in bore:
            continue
        area = int(st[i, cv2.CC_STAT_AREA])
        if area < HOLE_MIN_PX or area > big:
            continue
        rim_m = None
        if dz is not None:
            m_i = (lab == i).astype(np.uint8)
            ring = cv2.dilate(m_i, np.ones((9, 9), np.uint8)) - m_i
            v = dz[(ring > 0) & np.isfinite(dz) & (dz > 0)]
            if v.size < 20:
                continue
            rim_m = float(np.median(v))
            if rim_m > near_m:          # 테두리가 멀다 = 관 저 끝
                continue
        cand = {"area_px": area, "cx": float(ce[i][0]) + x0,
                "cy": float(ce[i][1]) + y0, "thr": float(thr), "rim_m": rim_m,
                "r_eq_px": math.sqrt(area / math.pi)}
        # 🚨 **가장 큰 덩어리를 고르면 안 된다.** 관이 곧지 않으면 관 저 끝이
        #    화면 어디에든 오고, 그건 늘 결함보다 크다(실측 39,603px vs 결함
        #    수천 px). 우리는 결함이 **어느 화소에 보여야 하는지** 를 기하로
        #    알고 있으므로 그 자리에 가장 가까운 덩어리를 고른다.
        #    자기충족이 아니다 — 그 자리에 어두운 덩어리가 실제로 없으면
        #    아무것도 안 잡히고, 수리가 되면 정확히 그렇게 된다.
        if expect_px is not None:
            cand["dist_px"] = math.hypot(cand["cx"] - expect_px[0],
                                         cand["cy"] - expect_px[1])
            if best is None or cand["dist_px"] < best["dist_px"]:
                best = cand
        elif best is None or area > best["area_px"]:
            best = cand
    if best and expect_px is not None:
        best["matched"] = best["dist_px"] <= max(2.0 * best["r_eq_px"],
                                                 0.06 * Wd)
    return best


def find_weld_bead(rgb, expect_px=None):
    """비드를 **색(HSV 채도)** 으로 찾는다. 관벽(회색)·관 저 끝(검정)은 채도 0."""
    hsv = cv2.cvtColor(np.asarray(rgb)[:, :, :3].astype(np.uint8),
                       cv2.COLOR_RGB2HSV)
    mask = ((hsv[:, :, 1] > BEAD_SAT_MIN)
            & (hsv[:, :, 2] > BEAD_VAL_MIN)).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, st, ce = cv2.connectedComponentsWithStats(mask, 8)
    best = None
    for i in range(1, n):
        area = int(st[i, cv2.CC_STAT_AREA])
        if area < BEAD_MIN_PX:
            continue
        if best is None or area > best["area_px"]:
            best = {"area_px": area, "cx": float(ce[i][0]),
                    "cy": float(ce[i][1]), "r_eq_px": math.sqrt(area / math.pi)}
    if best and expect_px is not None:
        dd = math.hypot(best["cx"] - expect_px[0], best["cy"] - expect_px[1])
        best["dist_px"] = dd
        best["matched"] = dd <= max(3.0 * best["r_eq_px"], 0.08 * hsv.shape[1])
    return best


def draw_inspect_debug(rgb, hole, bead, expect_px, win_px=70):
    """find_wall_hole/find_weld_bead 가 실제로 무엇을 보고 확정했는지 그린다.

    YOLO 와 무관한 순수 OpenCV 판정 시각화 — 탐색 창(회색 사각), 결함이
    있어야 할 자리(청록 십자, expect_px), 채택/기각된 후보의 윤곽과 판정
    근거(임계값·거리·테두리 depth)를 함께 그린다. hole/bead 는 matched 로
    걸러지기 **전** 원본을 넘겨야 기각된 후보(빨강)도 보인다 — 호출자가
    matched 로 None 처리하기 전의 값을 따로 챙겨서 넘길 것.
    """
    img = np.ascontiguousarray(np.asarray(rgb)[:, :, :3].astype(np.uint8)).copy()
    if expect_px is not None:
        ex, ey = int(expect_px[0]), int(expect_px[1])
        cv2.rectangle(img, (ex - win_px, ey - win_px), (ex + win_px, ey + win_px),
                     (160, 160, 160), 1)
        cv2.drawMarker(img, (ex, ey), (0, 255, 255), cv2.MARKER_CROSS, 18, 2)

    def _label(pos, text, color):
        cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                   (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                   color, 1, cv2.LINE_AA)

    if hole is not None:
        cx, cy = int(hole["cx"]), int(hole["cy"])
        r = max(4, int(hole["r_eq_px"]))
        matched = hole.get("matched", True)
        # 🚨 이 img 는 아직 RGB 채널 순서다(발행 직전에야 BGR 로 뒤집는다,
        #    ActiveCamBridge.publish_opencv_debug 참고) — cv2 관례대로
        #    (0,0,255) 를 "빨강" 삼으면 실제로는 파랑이 그려진다. RGB 순서
        #    기준 빨강은 (255,0,0).
        color = (0, 255, 0) if matched else (255, 0, 0)
        cv2.circle(img, (cx, cy), r, color, 2)
        text = (f"hole {hole['area_px']}px thr={hole['thr']:.0f} "
               f"d={hole.get('dist_px', 0):.0f}px "
               f"{'MATCH' if matched else 'REJECT'}")
        if hole.get("rim_m"):
            text += f" rim={hole['rim_m'] * 1000:.0f}mm"
        _label((cx + r + 4, max(12, cy)), text, color)

    if bead is not None:
        cx, cy = int(bead["cx"]), int(bead["cy"])
        r = max(4, int(bead["r_eq_px"]))
        matched = bead.get("matched", True)
        color = (255, 140, 0) if matched else (255, 0, 0)
        cv2.circle(img, (cx, cy), r, color, 2)
        text = f"bead {bead['area_px']}px d={bead.get('dist_px', 0):.0f}px"
        if "matched" in bead:
            text += " MATCH" if matched else " REJECT"
        _label((cx + r + 4, max(12, cy) + 16), text, color)

    return img


def front_frames(warm=200):
    """전방 RGB + Depth.

    🚨 **어노테이터는 마지막으로 렌더된 프레임을 그대로 들고 있다.** 헤드리스는
       메인 루프가 render=False 라 아예 렌더가 안 일어난다. "데이터가 있으면
       그만" 으로 짜면 두 번째 호출부터 낡은 영상을 본다(실측으로 걸린 최대
       시간 낭비). → **조건 없이 먼저 굽고** 읽는다.
    """
    for nm, ann, dep in rigs:
        if nm != "front_camera":
            continue
        for _ in range(warm):
            world.step(render=True)
        c, z = ann.get_data(), dep.get_data()
        rgb = (np.asarray(c)[:, :, :3].astype(np.uint8)
               if c is not None and getattr(c, "size", 0) else None)
        depth = (np.asarray(z, dtype=np.float32)
                 if z is not None and getattr(z, "size", 0) else None)
        return rgb, depth
    return None, None


def defect_pixel(d, cam_path=None):
    """결함 벽면 점을 지정 카메라의 어안 화소로 투영 (등거리 r = f·θ).

    cam_path 를 안 주면 FRONT_CAM(front_camera). torch_camera 는 front 와
    똑같은 내부파라미터로 구성돼 있다(TORCH_CAM_F_PX == F_PX, 같은 HFOV,
    같은 CAM_W/CAM_H — 위 torch_camera 조립부 참고) — 그래서 INTR 를 그대로
    재사용해도 되고, 프림 경로(cam_path)만 바꾸면 된다. torch_camera 는
    aim_torch_camera() 가 매 틱 재조준하므로, 여기서 매번 현재 월드 변환을
    새로 읽는 이 함수 구조가 그대로 맞는다(재조준된 방향이 자동 반영됨).
    """
    cam_prim = stage.GetPrimAtPath(cam_path or FRONT_CAM)
    if not cam_prim.IsValid():
        return None
    _XC.Clear()
    m = _XC.GetLocalToWorldTransform(cam_prim)
    p = m.GetInverse().Transform(Gf.Vec3d(*[float(v) for v in d["world"]]))
    x, y, z = float(p[0]), float(p[1]), float(p[2])
    if -z <= 1e-9:
        return None
    theta = math.atan2(math.hypot(x, y), -z)
    r_px = INTR["f_fish"] * theta
    rho = math.hypot(x, y)
    defect_geom.update(fwd_mm=-z * 1000.0, lat_mm=rho * 1000.0,
                       theta_deg=math.degrees(theta))
    if rho < 1e-9:
        return INTR["ppx"], INTR["ppy"]
    return (INTR["ppx"] + r_px * x / rho, INTR["ppy"] - r_px * y / rho)


def wall_uv(p, s, ref_clock):
    """관벽을 펼친 좌표 (축방향 mm, 원주 호길이 mm).

    반경 성분을 일부러 버린다 — 토치는 설계상 관벽에서 2mm 떨어져 있어서
    반경까지 넣으면 그 간극이 오차로 둔갑해 허용치 1.5mm 를 늘 넘는다.

    🚨 **시계각은 기준각(결함) 에 맞춰 ±180° 로 감아야 한다.** 절대 시계각을
       그대로 호길이로 바꾸면 −172° 와 +180° 가 352° 차이로 잡혀 정렬 오차가
       **311mm** 로 나온다(실측). 실제 어긋남은 8° = 7mm 다.
    """
    s_p, _ = PATH.project(p, s)
    c, t = PATH.point_tangent(s_p)
    d = p - c
    d = d - t * np.dot(d, t)
    nrm = np.linalg.norm(d)
    clock = PATH.clock_of(s_p, d / nrm) if nrm > 1e-9 else 0.0
    rel = (clock - ref_clock + 180.0) % 360.0 - 180.0
    return np.array([s_p * 1000.0, PIPE_IR * 1000.0 * math.radians(rel), 0.0])


def inspect_defect(d, tag):
    """카메라로 실제로 본다 → (관상태, 구멍, 비드, depth, 결함화소).

    depth·화소 좌표를 같이 돌려주는 이유 — weld.py 의 `rod_precheck` 가
    §8.1 등가직경 계산에 그대로 쓴다. `front_frames()` 는 headless 에서
    프레임을 다시 굽느라 무겁다(200스텝) — 이미 여기서 한 번 구운 걸
    호출자가 또 구하려고 다시 부르면 비용이 배가 된다.
    """
    rgb, z = front_frames()
    if rgb is None:
        print(f"  [검출] {tag}: 카메라 프레임 없음 — 판정 불가")
        return None, None, None, None, None
    cond = None
    if z is not None:
        joint = (math.degrees(float(np.max(np.abs(
            art.get_joint_positions()[bel_idx])))) if bel_idx else 0.0)
        cond = detector.run(z, joint_angle_deg=joint)
        print(f"  [관상태] {tag} {cond.state} ({cond.speed})  원형도 "
              f"{cond.circularity:.3f}  거칠기 {cond.roughness:.4f}  "
              f"오프셋 {cond.offset_mm:.2f}mm")
    px = defect_pixel(d)
    if px is None:
        print(f"  [검출] {tag}: 결함이 전방 카메라 화각 밖")
        return cond, None, None, z, None
    hole = find_wall_hole(rgb, expect_px=px, depth=z)
    bead = find_weld_bead(rgb, expect_px=px)
    hole_raw, bead_raw = hole, bead   # matched=False 로 걸러지기 전 원본 — 디버그 시각화용
    if hole is None:
        print(f"  [검출] {tag} 구멍 없음 — 벽면이 이어져 있다 "
              f"(예상 {px[0]:.0f},{px[1]:.0f} / 카메라 "
              f"{defect_geom.get('fwd_mm', 0):.0f}mm 앞)")
    else:
        print(f"  [검출] {tag} 어두운 덩어리 {hole['area_px']:,}px "
              f"중심({hole['cx']:.0f},{hole['cy']:.0f}) 예상자리와 "
              f"{hole['dist_px']:.0f}px → "
              f"{'구멍(결함)' if hole['matched'] else '위치 안 맞음 — 무시'}"
              f"  [밝기임계 {hole['thr']:.0f}"
              + (f", 테두리 깊이 {hole['rim_m'] * 1000:.0f}mm]"
                 if hole.get('rim_m') else "]"))
        if not hole["matched"]:
            hole = None
    if bead is not None:
        print(f"  [검출] {tag} 비드(주황) {bead['area_px']:,}px 예상자리와 "
              f"{bead['dist_px']:.0f}px → "
              f"{'결함 자리' if bead['matched'] else '엉뚱한 자리'}")
        if not bead["matched"]:
            bead = None
    else:
        print(f"  [검출] {tag} 비드(주황) 없음")
    _publish_opencv_debug(rgb, hole_raw, bead_raw, px, tag)
    return cond, hole, bead, z, px


def _judgement_payload(hole, bead, px, tag, cond=None, joint_deg=None):
    """find_wall_hole/find_weld_bead(+ 선택적 관상태) 결과를 JSON 발행용 dict 로 포장한다.

    hole/bead 는 이미 python float/int/bool/None 만 담고 있어(각 함수 안에서
    float()/int() 캐스팅됨) json.dumps 에 그대로 들어간다. cond 는
    PipeConditionDetector.run() 결과 중 표시에 필요한 필드만 뽑은 dict —
    front_camera 에서만 채워 보낸다(§ below, 관절 비틀림 판단 과정).
    """
    payload = {
        "tag": str(tag),
        "px": [float(px[0]), float(px[1])] if px is not None else None,
        "win_px": 70,
        "hole": hole,
        "bead": bead,
    }
    if cond is not None:
        payload["cond"] = cond
    if joint_deg is not None:
        payload["joint_deg"] = float(joint_deg)
    return payload


def _publish_opencv_debug(rgb, hole, bead, px, tag):
    """OpenCV 판정 시각화 — ROS 로 실시간 발행하고(--shots 면) PNG 로도 남긴다.

    hole/bead 는 matched 로 걸러지기 전 원본이어야 기각 사례도 보인다
    (inspect_defect() 의 hole_raw/bead_raw). YOLO 와 완전히 무관하다.
    """
    if rgb is None:
        return
    img = draw_inspect_debug(rgb, hole, bead, px)
    if cam_bridge is not None:
        cam_bridge.publish_opencv_debug(img, "front_camera")
    if SHOTS:
        safe_tag = tag.replace(" ", "_")
        _save_png(img, OUT / f"{safe_tag}_opencv_debug.png")


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
    if not rigs or not SHOTS:
        return
    # 🚨 torch_camera 는 ALIGN/EXTEND/ARC 중에만 의미 있게 조준돼 있다 —
    #    INSPECT/VERIFY 시점의 before/after 스냅에는 그 순간 프레임이 남을
    #    이유가 없다(사용자 지적). front/back 만 저장한다.
    for nm, ann, dep in rigs:
        if nm == "torch_camera":
            continue
        c = ann.get_data()
        if c is not None and getattr(c, "size", 0):
            img = np.asarray(c)[:, :, :3].astype(np.uint8)
            q = _save_png(img, OUT / f"{tag}_{nm}_rgb.png")
            print(f"  [사진] {q.name}  평균밝기 {img.mean():.1f}/255")
        z = dep.get_data()
        if z is None or getattr(z, "size", 0) == 0:
            continue
        a = np.asarray(z, dtype=np.float32)
        np.save(OUT / f"{tag}_{nm}_depth.npy", a)
        fin = np.isfinite(a) & (a > 0)
        g = np.zeros(a.shape, np.uint8)
        if fin.any():
            v = a[fin]; lo, hi = float(v.min()), float(v.max())
            if hi > lo:
                g[fin] = ((a[fin] - lo) / (hi - lo) * 255).astype(np.uint8)
        img = np.dstack([g, g, g]); img[~fin] = (255, 0, 0)
        _save_png(img, OUT / f"{tag}_{nm}_depth.png")


# ── 물 재투입 (국소 주머니) ────────────────────────────────────────
_rng = np.random.default_rng(3)
_recycled, _flow_now = 0, 0.0


def recycle_particles():
    """주머니를 벗어난 입자를 상류 끝으로 재투입.

    주머니는 **직선 구간**이라 축을 하나로 고정해도 된다(결함을 직선에만
    두는 이유). 개구로 빠져나간 입자도 여기서 잡아 되돌리므로 누수가 안 끊긴다.
    """
    global _recycled, _flow_now
    pts = np.array(water_instancer.GetPositionsAttr().Get())
    vels = np.array(water_instancer.GetVelocitiesAttr().Get())
    spd = abs(FLOW_V)
    n_p = len(pts)
    # 어느 주머니 소속인지 한 번에 정한다 — 파이썬 루프는 입자 수만큼 돌면
    # 재투입이 물리보다 비싸진다(호출이 초당 15회다).
    ax_all = np.empty((len(_pockets), n_p))
    rad_all = np.empty((len(_pockets), n_p))
    for m, (p0, t0, half) in enumerate(_pockets):
        rel = pts - p0
        ax_all[m] = rel @ t0
        rad_all[m] = np.linalg.norm(rel - np.outer(ax_all[m], t0), axis=1)
    own = np.argmin(np.abs(ax_all) + rad_all, axis=0)
    ax = ax_all[own, np.arange(n_p)]
    rad = rad_all[own, np.arange(n_p)]
    T0 = np.array([q[1] for q in _pockets])[own]        # (N,3) 소속 접선
    P0 = np.array([q[0] for q in _pockets])[own]
    HALF = np.array([q[2] for q in _pockets])[own]
    vels += FLOW_BLEND * (-T0 * spd - vels)
    _flow_now = float(np.mean(-np.einsum("ij,ij->i", vels, T0)))
    out = (ax < -HALF) | (rad > W_R_MAX + 0.008) | (ax > HALF * 1.5)
    n = int(out.sum())
    if n:
        # 상류 끝으로 되돌린다. 개구로 빠져나간 입자도 여기서 잡히므로
        # 누수가 끊기지 않고 계속 보인다.
        pts[out] = (P0[out] + T0[out] * HALF[out, None] * 0.95
                    + _rng.normal(0.0, 0.008, (n, 3)))
        vels[out] = -T0[out] * spd
    water_instancer.GetPositionsAttr().Set(
        Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))
    water_instancer.GetVelocitiesAttr().Set(
        Vt.Vec3fArray.FromNumpy(vels.astype(np.float32)))
    _recycled += n


def leak_count(d):
    """결함 개구 **바깥** 으로 빠져나간 입자 수 — 누수의 정량 지표."""
    if water_instancer is None:
        return 0
    pts = np.array(water_instancer.GetPositionsAttr().Get())
    c, t = PATH.point_tangent(d["s"])
    rel = pts - c
    ax = rel @ t
    rad = np.linalg.norm(rel - np.outer(ax, t), axis=1)
    return int(((np.abs(ax) < POCKET_HALF) & (rad > PIPE_IR + 0.010)).sum())


# ── 전방 판정: 단절 / 분기 ──────────────────────────────────────────
def classify_ahead():
    """전방을 보고 NORMAL / JUNCTION / DISCONNECTED 를 가른다.

    - 관 상태 판정기(Depth 무효 비율)는 **단절과 분기를 못 가른다** — 둘 다
      반사가 안 돌아오는 빈 공간이 생긴다. 그래서 한 겹 더 본다:
      정면(화면 중앙)에 **가까운 벽**이 있으면 분기, 뻥 뚫려 있으면 단절이다.
    """
    rgb, z = front_frames(warm=8)
    if z is None:
        return "NONE", None, None, None
    joint = (math.degrees(float(np.max(np.abs(
        art.get_joint_positions()[bel_idx])))) if bel_idx else 0.0)
    cond = detector.run(z, joint_angle_deg=joint)
    h, w = z.shape
    c = z[int(h * 0.42):int(h * 0.58), int(w * 0.42):int(w * 0.58)]
    fin = np.isfinite(c) & (c > 0)
    centre_m = float(np.median(c[fin])) if fin.any() else float("inf")
    inval = float((~(np.isfinite(z) & (z > 0))).mean())
    if cond.state == "DISCONNECTED" or inval > 0.02:
        # 🚨 이 맵에 실제 T 분기가 있는 층은 CFG["end"]=="junction" 뿐이다
        #    (floor1). floor2 는 "disconnect" 로 끝난다 — 실제 분기가 없다.
        #    DO_SMOOTH 를 끈 각진 곡관(정점 투영 보정 미해결 — 물리 폭발
        #    회피용 임시 조치, 2026-08-06)이 가까운 벽처럼 보여 JUNCTION 으로
        #    오검출됐다(실측: floor2 s=458mm 에서 오검출 → 없는 분기에
        #    서스펜션 비대칭을 걸다 끼임). 분기가 없는 층에서는 애초에
        #    JUNCTION 을 낼 수 없게 막는다.
        kind = ("JUNCTION" if (centre_m < 0.16 and CFG.get("end") == "junction")
               else "DISCONNECTED")
    else:
        kind = "NORMAL"
    return kind, cond, centre_m, inval


def right_branch_clock():
    """오른손 법칙 — 진행 방향 기준 '오른쪽' 의 시계각(도).

    right = forward × up (월드). up 은 +Z. 시계각 규약은 PATH.frame 과 같다
    (e_up 에서 e_side 로, 180° = 바닥).
    """
    s = s_of(_seg1)
    _, t, u, w = PATH.frame(s)
    r = np.cross(t, np.array([0.0, 0.0, 1.0]))
    if np.linalg.norm(r) < 1e-6:
        return None
    r /= np.linalg.norm(r)
    return math.degrees(math.atan2(float(np.dot(r, w)), float(np.dot(r, u))))


# 서스펜션 예압 비대칭 — T 분기에서 전방 세그먼트를 원하는 가지로 민다.
# ❓ **미검증이다.** 이 로봇에 조향이 없어서 남은 유일한 수단이고(설계 한계
#    "T분기 경로 선택 불가"), 새 액추에이터를 안 넣는다는 조건에서 고른 것이다.
#    게다가 이 맵의 T 는 날카로워 굽힘 한계(평면당 ±55°)로 통과 자체가 안 될
#    가능성이 크다. 되는지 안 되는지를 **로그로 남기는 것**이 이 코드의 목적이다.
PISTON_BASE = 0.020        # usda 기본 target (상한 밖 = 예압)
PISTON_BIAS = 0.004


def leg_clock_deg(sg, i):
    """레그 i 의 시계각(월드, 중심선 프레임 기준)."""
    p = wpos(stage.GetPrimAtPath(f"{ROBOT}/seg{sg}_wheel_{i}_0"))
    s = s_of(_seg1)
    c, t = PATH.point_tangent(s)
    d = p - c
    d = d - t * np.dot(d, t)
    n = np.linalg.norm(d)
    return PATH.clock_of(s, d / n) if n > 1e-9 else 0.0


_bias_cache = None


def bias_pistons(clock_deg, rebuild=False):
    """clock_deg 방향으로 전방 세그먼트를 민다 — 반대쪽 레그를 더 뻗는다.

    지령은 **분기 진입 시점에 한 번만** 만든다. 레그 시계각을 매 스텝 다시
    재면 투영이 스텝마다 세 번씩 더 돌아 시연이 눈에 띄게 느려진다.
    """
    global _bias_cache
    if not piston_idx:
        return
    if rebuild or _bias_cache is None:
        idx, val = [], []
        for k, nm in zip(piston_idx, piston_name):
            if not nm.startswith("seg1"):
                idx.append(k); val.append(PISTON_BASE)
                continue
            th = leg_clock_deg(1, int(nm.split("_")[-1]))
            idx.append(k)
            val.append(PISTON_BASE
                       - PISTON_BIAS * math.cos(math.radians(th - clock_deg)))
        _bias_cache = (np.array(val), np.array(idx))
        print("           피스톤 지령(mm) " + " ".join(
            f"{v * 1000:.1f}" for v in _bias_cache[0]))
    art.apply_action(ArticulationAction(joint_positions=_bias_cache[0],
                                        joint_indices=_bias_cache[1]))


def release_pistons():
    if not piston_idx:
        return
    art.apply_action(ArticulationAction(
        joint_positions=np.array([PISTON_BASE] * len(piston_idx)),
        joint_indices=np.array(piston_idx)))


# ── 시퀀스 ──────────────────────────────────────────────────────────
STOP_TOL = 0.003
ALIGN_AXIAL_TOL = 0.4
INSPECT_BACK_MM = 120.0
# 🔑 용접 직전 재확인 거리 — REPOSITION 의 검증촬영 기본값(90mm)과 같은
#    값을 쓴다(카메라 원리적 한계 18.2mm 보다 충분히 크고, 이미 이 코드
#    베이스에서 "보인다"고 검증된 거리다). INSPECT(120mm) 와는 다른
#    거리·각도에서 다시 재서, 첫 촬영의 각도 의존 오차(2026-08-07 관통
#    구멍 Depth 오측정 사건 참고)를 두 번째 독립 측정으로 잡아낸다.
RECHECK_BACK_MM = 90.0
ARC_STEPS = int(2.0 * PHYSICS_HZ)
COOL_STEPS = int(1.0 * PHYSICS_HZ)
CHECK_EVERY = int(1.0 * PHYSICS_HZ)     # 전방 판정 주기 (렌더 8프레임)
SETTLE_STEPS = int((6.0 if WATER else 1.5) * PHYSICS_HZ)

state, t_state = "SETTLE", 0
cur = 0                    # 지금 다루는 결함 번호
j2_cmd, j1_cmd = 0.0, 0.0
inspected = False
rechecked = False
repos_from, inspect_fwd_mm = None, 0.0
pre_hole, pre_bead, verdicts = None, None, []
rod_required_mm = 0.0
end_reason = None
_last_check = 0
_stuck_s, _stuck_t, stuck_retry = None, 0, 0

import time                                              # noqa: E402
_t_mark, _step_mark, _f_drag = time.time(), 0, 0.0
print("-" * 78)
REPLAY = HOLD and not HEADLESS
_spark_cyl = None       # 스파크 가둠 캐시 (결함 s, 가둠 함수)
step, was_playing, reported = 0, True, False

# RETURN 라이저 등반 진단 — s<450mm 구간에서 [RETURN진단] 으로 매 20스텝(~83ms)
# 휠접지/피스톤/틀어짐(축·롤)/스텝당 실시간(솔버 부하 근사)을 남긴다.
# (2026-08-07, floor2 후진 라이저 등반 실패 재현 조사용, src/son 에서 포팅)
RETURN_DIAG_S_MM = 450
RETURN_DIAG_EVERY = 20
_prev_loop_t = time.time()
_diag_dt = []


def report():
    print("=" * 78)
    s = s_of(_seg1)
    print(f"결과  층 {FLOOR}  최종 상태 {state}  진행 {s * 1000:.1f} / "
          f"{PATH.total * 1000:.0f}mm  ({s / PATH.total * 100:.0f}%)")
    for v in verdicts:
        print(f"      결함 {v['i'] + 1}: {v['msg']}")
    if not verdicts and DEFECTS:
        print("      결함 처리 이력 없음")
    if end_reason:
        print(f"      종료 사유 — {end_reason}")
    if WATER and DEFECTS:
        print(f"      물 입자 {n_particles:,}개, 유동 {_flow_now:.3f} m/s, "
              f"재투입 누계 {_recycled:,}")
    ok = [v for v in verdicts if v["ok"] is True]
    print(f"판정  수리 성공 {len(ok)}건 / 결함 {len(DEFECTS)}건")
    sys.stdout.flush()


def restart_demo():
    global state, t_state, cur, j2_cmd, inspected, repos_from, pre_hole
    global verdicts, end_reason, inspect_fwd_mm, _last_check
    for d in DEFECTS:
        UsdGeom.Imageable(d["hole"]).MakeVisible()
        UsdGeom.Imageable(d["bead"]).MakeInvisible()
        d["bead_op"].Set(d["xf"])
        if d.get("plug") is not None:
            UsdGeom.Imageable(d["plug"]).MakeInvisible()
            d["plug_coll"].CreateCollisionEnabledAttr(False)
    if arc_light is not None:
        arc_light.GetIntensityAttr().Set(0.0)
    if sparks is not None:
        sparks.clear()
    globals()['_spark_cyl'] = None
    world.reset()
    state, t_state, cur, j2_cmd = "SETTLE", 0, 0, 0.0
    globals()['j1_cmd'] = 0.0
    inspected, repos_from, pre_hole = False, None, None
    globals()['pre_bead'] = None
    globals()['rechecked'] = False
    globals()['rod_required_mm'] = 0.0
    verdicts, end_reason, inspect_fwd_mm, _last_check = [], None, 0.0, 0
    print("=" * 78)
    print("[재시작] Stop → Play 감지 — 처음부터 다시 돈다")


while True:
    world.step(render=not HEADLESS)
    step += 1
    _now_loop = time.time()
    _loop_dt = _now_loop - _prev_loop_t
    _prev_loop_t = _now_loop
    if not HEADLESS:
        _playing = world.is_playing()
        if _playing and not was_playing:
            restart_demo()
            step, reported = 0, False
            _t_mark, _step_mark = time.time(), 0
        was_playing = _playing
        if not simulation_app.is_running():
            break
        if not _playing:
            continue

    if cam_bridge is not None and _now_loop - _cam_pub_last >= 1.0 / CAM_PUBLISH_HZ:
        _cam_pub_last = _now_loop
        cam_bridge.publish(active_camera_name(state))
        rclpy.spin_once(cam_bridge, timeout_sec=0.0)

    if state == "DONE":
        if not reported:
            drive(0.0)
            report()
            reported = True
            if REPLAY:
                print("GUI 실행 중 — Stop→Play 로 재시작, 창을 닫으면 종료")
        if not REPLAY:
            break
        continue
    if step > STEPS and not REPLAY:
        end_reason = f"스텝 상한 {STEPS} 도달"
        state = "DONE"
        continue

    if step - _step_mark >= 5 * PHYSICS_HZ:
        _dt = time.time() - _t_mark
        _rate = (step - _step_mark) / max(_dt, 1e-9)
        print(f"  [속도] 물리 {_rate:6.1f} step/s (실시간 {_rate / PHYSICS_HZ:4.2f}x)"
              + (f"  입자 {n_particles:,}" if WATER else "")
              + (f"  항력 {_f_drag:+.2f}N" if FLUID else ""))
        _t_mark, _step_mark = time.time(), step
    if WATER and water_instancer is not None and step % RECYCLE_EVERY == 0:
        recycle_particles()

    t_state += 1
    s_hint = s_of(_seg1)
    if FLUID:
        _f_drag = apply_fluid(s_hint)
    d_cur = DEFECTS[cur] if cur < len(DEFECTS) else None
    s_ring = s_of(_ring, s_hint)
    tip_r = tip_end_r(s_ring) if not NO_TORCH else 0.0
    # 🚨 예전엔 ALIGN/EXTEND/ARC 상태에서만 재조준했는데, 사용자가 "처음부터
    #    이 각도에서 찍게 할 수 있나" 라고 물어서 — 상태와 무관하게 매 틱
    #    무조건 돈다(SETTLE 부터 끝까지, 로드가 수축해 있어도 팁 위치는
    #    항상 정의돼 있다). 비용은 삼각함수 몇 번뿐이라 무시할 만하다.
    if TORCH_CAM_OK:
        aim_torch_camera(tip_end_world())

    # 🚨 2026-08-08 — 표시 전용 실시간 판정(사용자 요청: "스냅샷 말고 실시간
    #    으로", 이어서 "front_camera에만 표시하고 torch/back 은 빼자"로
    #    범위를 다시 좁혔다). inspect_defect() 의 3번짜리 스냅샷(INSPECT/
    #    RECHECK/VERIFY, 각각 front_frames(warm=200) 로 200스텝 재렌더)과
    #    달리, 이건 이미 ActiveCamBridge 가 매 틱 공짜로 읽는 것과 **같은**
    #    rig 애노테이터(GUI 모드라 매 틱 render=True 로 항상 최신)를 그대로
    #    읽어서 추가 렌더 비용이 없다. find_wall_hole/find_weld_bead 자체는
    #    inspect_defect() 와 동일 함수라 판정 로직이 갈라지지 않는다 —
    #    여기서 계산한 결과는 **결코 임무 판정(결함 등록, 용접봉 예산 확정)
    #    에 쓰이지 않는다**, 화면 표시용 JSON 발행뿐이다.
    # 🚨 SETTLE 은 로봇이 아직 안착 중이라 카메라/Depth 가 초기화 직후의
    #    불완전한 프레임일 수 있다 — inspect_defect() 는 이 타이밍에 한 번도
    #    불린 적이 없어서 검증되지 않은 입력이다(cur=0 이라 d_cur 는 SETTLE
    #    부터 이미 유효해서, 막지 않으면 첫 틱부터 돈다). 표시 전용 기능이
    #    이런 초기 프레임 때문에 시뮬레이션 전체를 죽이면 안 되므로 SETTLE
    #    은 아예 빼고, 그래도 모를 예외는 아래 try/except 로 한 번 더 막는다
    #    (실패해도 그 틱만 건너뛰고 계속 돈다 — 2026-08-08 실측: 아래
    #    curve_ahead AttributeError 가 이 방어 덕에 미션을 안 죽였다).
    _active_cam = active_camera_name(state)
    if (cam_bridge is not None and d_cur is not None and state != "SETTLE"
            and _active_cam == "front_camera"
            and _now_loop - _judge_pub_last >= 1.0 / CAM_PUBLISH_HZ):
        _judge_pub_last = _now_loop
        try:
            _live_rig = _RIGS_BY_NAME.get(_active_cam)
            if _live_rig is not None:
                _live_ann, _live_dep = _live_rig
                _live_c, _live_z = _live_ann.get_data(), _live_dep.get_data()
                if _live_c is not None and getattr(_live_c, "size", 0):
                    _live_rgb = np.asarray(_live_c)[:, :, :3].astype(np.uint8)
                    _live_depth = (np.asarray(_live_z, dtype=np.float32)
                                  if _live_z is not None and getattr(_live_z, "size", 0)
                                  else None)
                    _live_px = defect_pixel(d_cur)
                    _live_hole = find_wall_hole(_live_rgb, expect_px=_live_px, depth=_live_depth)
                    _live_bead = find_weld_bead(_live_rgb, expect_px=_live_px)
                    # 🚨 "front 카메라에 조인트 비틀림 판단 과정도"(사용자 요청).
                    #    PipeConditionDetector.run() 이 판정 전에 로봇 자체
                    #    벨로우즈 관절의 굽힘각(joint_angle_deg, §1 inspect_
                    #    defect() 와 동일 계산)을 먼저 보고, 그게 bend_angle_deg
                    #    문턱을 넘으면 "곡관 접근 중이라 원형도 판정을 생략한다"고
                    #    스스로 밝힌다(c.reason 에 그대로 찍힘) — 이게 "조인트
                    #    비틀림 판단"이다.
                    # 🚨 2026-08-08 실측 — condition/detector.py 가 **이 저장소에
                    #    두 벌**이다: src/son/condition/detector.py(Condition 에
                    #    curve_ahead 있음) 와 dongyeon 자신의 integration_test/
                    #    condition/detector.py(없음). real_map_demo.py 는
                    #    sys.path 상 후자를 실제로 import 하고 있어서 .curve_ahead
                    #    를 그냥 읽으면 AttributeError 로 매 틱 실패했다(로그로
                    #    확인, try/except 가 미션은 지켜줬다) — getattr 로
                    #    양쪽 버전 다 안전하게 만든다.
                    _live_joint = (math.degrees(float(np.max(np.abs(
                        art.get_joint_positions()[bel_idx])))) if bel_idx else 0.0)
                    _live_cond_payload = None
                    if _live_depth is not None:
                        _live_cond = detector.run(_live_depth, joint_angle_deg=_live_joint)
                        _live_cond_payload = {
                            "state": _live_cond.state,
                            "speed": _live_cond.speed,
                            "circularity": _live_cond.circularity,
                            "roughness": _live_cond.roughness,
                            "offset_mm": _live_cond.offset_mm,
                            "curve_ahead": getattr(_live_cond, "curve_ahead", False),
                            "reason": _live_cond.reason,
                        }
                    cam_bridge.publish_opencv_judgement(
                        _judgement_payload(_live_hole, _live_bead, _live_px, state,
                                          cond=_live_cond_payload, joint_deg=_live_joint))
        except Exception as _live_exc:
            # 표시 전용 기능이라 실패해도 미션(주행·용접·판정)은 절대 멈추면
            # 안 된다 — 이 틱만 건너뛰고 다음 틱에 다시 시도한다. 반복 실패 시
            # 콘솔이 10Hz 로 도배되지 않도록 5초에 한 번만 찍는다.
            if _now_loop - _judge_exc_last_print >= 5.0:
                _judge_exc_last_print = _now_loop
                print(f"[경고] 실시간 판정 표시 실패({_active_cam}, {state}): "
                      f"{type(_live_exc).__name__}: {_live_exc}")

    if state == "SETTLE":
        if t_state > SETTLE_STEPS:
            p = wpos(_seg1)
            wr = [radial_r(wpos(stage.GetPrimAtPath(
                f"{ROBOT}/seg{sg}_wheel_{i}_{jj}")), s_hint)
                for sg in (0, 1) for i in range(3) for jj in (0, 1)]
            print(f"[SETTLE] 안착 seg1 ({p[0] * 1000:.0f}, {p[1] * 1000:.0f}, "
                  f"{p[2] * 1000:.0f})mm  진행 {s_hint * 1000:.1f}mm")
            print(f"  휠 중심 반경 {min(wr):.1f}~{max(wr):.1f}mm "
                  f"밀착 {sum(1 for r in wr if r > 39.0)}/12  (관 내반경 "
                  f"{bore_radius(s_hint):.1f}mm)")
            if "--start-return" in sys.argv:
                # 🔧 진단용 — 전진·용접 시퀀스를 다 안 거치고 후진(RETURN)
                #    구간만 바로 재현한다(--start-s 로 위치를 잡아 준다).
                end_reason = "진단 — --start-return 강제 진입"
                state, t_state = "RETURN", 0
            else:
                state, t_state = "CRUISE", 0

    elif state == "CRUISE":
        drive(SPIN_DEG_S)
        set_torch(j1_deg=0.0, j2_m=0.0)
        # 다음 결함 앞에서 촬영
        if d_cur is not None and not inspected \
                and s_ring >= d_cur["s"] - INSPECT_BACK_MM * MM:
            inspected = True
            drive(0.0)
            state, t_state = "INSPECT", 0
        elif d_cur is not None and inspected and not rechecked \
                and s_ring >= d_cur["s"] - RECHECK_BACK_MM * MM:
            rechecked = True
            drive(0.0)
            state, t_state = "RECHECK", 0
        elif d_cur is not None and inspected and rechecked \
                and s_ring >= d_cur["s"] - STOP_TOL:
            drive(0.0)
            print(f"[APPROACH] 결함 {cur + 1} 도달 — 링 s={s_ring * 1000:.1f}mm "
                  f"(목표 {d_cur['s'] * 1000:.1f})")
            state, t_state = "ALIGN", 0
        elif step - _last_check >= CHECK_EVERY:
            _last_check = step
            kind, cond, centre, inval = classify_ahead()
            if kind == "DISCONNECTED":
                drive(0.0)
                print(f"[판정] ★ 관 단절 — Depth 무효 {inval * 100:.1f}%, "
                      f"정면 거리 {centre:.3f}m (벽이 없다). 진행 "
                      f"{s_hint * 1000:.1f}mm")
                state, t_state = "DISCONNECTED", 0
            elif kind == "JUNCTION":
                drive(0.0)
                print(f"[판정] ★ 분기 감지 — Depth 무효 {inval * 100:.1f}%, "
                      f"정면에 벽 {centre * 1000:.0f}mm. 진행 "
                      f"{s_hint * 1000:.1f}mm")
                state, t_state = "JUNCTION", 0
            elif s_hint >= PATH.total - 0.05:
                drive(0.0)
                end_reason = "코스 끝 도달(단절·분기 미검출)"
                print(f"[END] 코스 끝 — 진행 {s_hint * 1000:.1f}mm")
                state, t_state = "DONE", 0
        if t_state % 600 == 0:
            q = np.asarray(art.get_joint_positions(), dtype=float)
            v = np.asarray(art.get_joint_velocities(), dtype=float)
            wv = math.degrees(float(np.mean([v[k] for k in wheel_idx])))
            pis = [q[k] * 1000 for k in piston_idx]
            bel = [math.degrees(q[k]) for k in bel_idx]
            print(f"  주행 s{s_hint * 1000:8.1f}mm  휠 {wv:7.1f}deg/s "
                  f"(지령 {SPIN_DEG_S:.0f})  피스톤 {min(pis):4.1f}~{max(pis):4.1f}"
                  f"  벨로우즈 {min(bel):+5.1f}~{max(bel):+5.1f}"
                  f"  관 {bore_radius(s_hint):.1f}mm")
            # 끼임 감지 — 휠은 도는데 진행이 없다
            if _stuck_s is not None and abs(s_hint - _stuck_s) < 0.002:
                _stuck_t += 1
                if _stuck_t >= 3:
                    end_reason = (f"끼임 — s {s_hint * 1000:.1f}mm 에서 "
                                  f"진행이 멎었다(휠 {wv:.0f}deg/s)")
                    print(f"[판정] ★ {end_reason}")
                    # 어느 레그가 물렸는지 남긴다 — 이게 없으면 "예압 부족"
                    # 으로 오독하고 파라미터를 헛돌게 된다(기록된 함정).
                    for sg in (0, 1):
                        rr = [radial_r(wpos(stage.GetPrimAtPath(
                            f"{ROBOT}/seg{sg}_wheel_{i}_{jj}")), s_hint)
                            for i in range(3) for jj in (0, 1)]
                        pp = [q[k] * 1000 for k, nm in zip(piston_idx,
                                                           piston_name)
                              if nm.startswith(f"seg{sg}")]
                        cl = [leg_clock_deg(sg, i) for i in range(3)]
                        print(f"       seg{sg} 휠반경 "
                              + " ".join(f"{r:.1f}" for r in rr)
                              + "  피스톤 " + " ".join(f"{v:.1f}" for v in pp)
                              + "  레그시계각 "
                              + " ".join(f"{c:+.0f}°" for c in cl))
                    print(f"       벨로우즈 4관절 "
                          + " ".join(f"{math.degrees(q[k]):+.1f}°"
                                     for k in bel_idx)
                          + "  (관절당 한계 ±27.5°)")
                    print(f"       중심선 이탈 seg0 "
                          f"{PATH.project(wpos(_seg0), s_hint)[1] * 1000:.1f}mm"
                          f" / seg1 "
                          f"{PATH.project(wpos(_seg1), s_hint)[1] * 1000:.1f}mm"
                          f"  관 내반경 {bore_radius(s_hint):.1f}mm")
                    # 🔑 **끼임과 관경 축소를 가른다.** 설계는 서스펜션
                    #    스트로크를 관경 변화 감지 보조로 쓴다(위치추정 절).
                    #    피스톤이 압축 한계에 붙어 있으면 관이 좁아진 것이고,
                    #    그건 후진 재시도로 풀리지 않는다 — 보고하고 복귀한다.
                    _pmin = min(q[k] * 1000 for k in piston_idx)
                    _ahead = bore_radius(min(s_hint + 0.10, PATH.total))
                    _narrow = _ahead < bore_radius(s_hint) - 2.0
                    if _pmin < 1.0 and _narrow:
                        end_reason = (
                            f"관경 축소로 통행 불가 — s {s_hint * 1000:.0f}mm "
                            f"에서 관이 {bore_radius(s_hint):.1f} → "
                            f"{_ahead:.1f}mm 로 좁아지는데 피스톤이 이미 "
                            f"{_pmin:.1f}mm(압축 한계)다")
                        print(f"[판정] ★ {end_reason}")
                        state, t_state = "DISCONNECTED", 0   # 보고 후 복귀
                    elif stuck_retry < 3:
                        stuck_retry += 1
                        print(f"[RECOVER] 끼임 — 후진 재시도 "
                              f"{stuck_retry}/3 (임무 규칙 8)")
                        state, t_state = "RECOVER", 0
                    else:
                        end_reason = (f"끼임 — 후진 재시도 3회 후에도 s "
                                      f"{s_hint * 1000:.0f}mm 에서 못 나갔다")
                        state, t_state = "DISCONNECTED", 0
            else:
                _stuck_t = 0
            _stuck_s = s_hint

    elif state == "INSPECT":
        drive(0.0)
        if t_state > 0.7 * PHYSICS_HZ:
            gap = (d_cur["s"] - s_ring) * 1000.0
            print(f"[INSPECT] 결함 {cur + 1} 촬영 정지 — 링 기준 {gap:.1f}mm "
                  f"(목표 {INSPECT_BACK_MM:.0f}mm)")
            _, pre_hole, pre_bead, pre_z, pre_px = inspect_defect(
                d_cur, f"결함{cur + 1} 수리 전")
            inspect_fwd_mm = defect_geom.get("fwd_mm", 0.0)
            snap(f"d{cur + 1}_1_before")
            if pre_bead is not None:
                print(f"  ⚠ **수리 전인데 비드가 잡혔다** "
                      f"({pre_bead['area_px']:,}px) — 판정 근거가 "
                      f"깨진다. 색 검출 조건을 의심할 것")
            print(f"  [판정] 수리 대상 등록 — 참고 어두운 덩어리 "
                  + (f"{pre_hole['area_px']:,}px" if pre_hole
                     else "안 잡힘"))
            # 🚨 §8.3/임무 규칙 8 — 용접을 시도하기 *전에* 이 결함을 감당할
            #    잔량이 있는지 먼저 묻는다. weld.py 의 rod_precheck 가
            #    이번 결함의 Depth 프로파일에서 등가직경(mm)을 실측해
            #    §8.1 L_req 로 소요량을 뽑고, 잔량과 비교한다(치수 계측
            #    실패시 default_repair_mm 근사로 물러난다) — 시도 후
            #    사후 판정이면 이미 팔이 나가 있어 §8.3을 못 지킨다.
            # 🚨 Depth 프로파일이 아니라 OpenCV 밝기검출 면적(area_px)으로
            #    잰다 — 관통 구멍은 Depth 로는 카메라 각도만으로 값이
            #    수십~수백 배 널뛴다(구멍 뒤 배경까지의 거리를 보게 되어,
            #    실측: 결함1 875mm vs 결함2 5mm — 같은 ø38.1mm인데). 반면
            #    area_px 는 두 결함이 2,049px/2,105px로 거의 일치했다.
            _hole_px = pre_hole["area_px"] if pre_hole else None
            req, rod_ok = sequencer.rod_precheck(
                rod_budget, hole_area_px=_hole_px, centre_px=pre_px, intr=INTR)
            rod_required_mm = req
            print(f"  [용접봉] 이번 결함 예상 소요 {req:.0f}mm  "
                  f"(잔량 {rod_budget.remaining_mm:.0f}mm)")
            _log_yolo_opinion(d_cur, "INSPECT")
            if not rod_ok:
                rod_budget.mark_exhausted_if_needed(req)
                end_reason = rod_budget.s.reason
                print(f"[STOP·보고] {end_reason} — 임무 규칙 8 에 따라 "
                      f"즉시 정지·보고 후 **복귀**한다 "
                      f"(결함{cur + 1} 용접 시도 안 함)")
                snap("stop_report")
                state, t_state = "RETURN", 0
            else:
                state, t_state = "CRUISE", 0

    elif state == "RECHECK":
        # 🔑 용접 직전, INSPECT 와는 다른 거리·각도(90mm vs 120mm)에서
        #    한 번 더 본다 — §5.3 절차 4 "목표 지점에서 재검출", weld.py
        #    의 RECHECK 상태와 같은 개념이다. 첫 촬영(INSPECT)의 각도
        #    의존 오차를 두 번째 독립 측정으로 잡으려는 것 — 2026-08-07
        #    관통 구멍 Depth 오측정처럼 각도에 따라 크게 튀는 사례가
        #    있었다. 재검출 안 되면(조명·각도 일시적 문제) INSPECT 값을
        #    그대로 믿고 진행한다 — 여기서 결함 자체를 취소하지 않는다
        #    (같은 로봇이 방금 등록한 결함이라, 두 로봇 설계의 "반전
        #    판정"과는 성격이 다르다).
        drive(0.0)
        if t_state > 0.7 * PHYSICS_HZ:
            gap = (d_cur["s"] - s_ring) * 1000.0
            print(f"[RECHECK] 결함 {cur + 1} 재확인 — 링 기준 {gap:.1f}mm "
                  f"(목표 {RECHECK_BACK_MM:.0f}mm)")
            _, re_hole, _, _, re_px = inspect_defect(
                d_cur, f"결함{cur + 1} 재확인")
            if re_hole is not None:
                req, _ = sequencer.rod_precheck(
                    None, hole_area_px=re_hole["area_px"],
                    centre_px=re_px, intr=INTR)
                # 🚨 평균이 아니라 **최댓값**을 쓴다. 두 측정 차이가 랜덤
                #    잡음이 아니라 거리(90 vs 120mm)에 따라 체계적으로
                #    벌어지는 것으로 실측됐다 — 평균은 "참값 근처에서
                #    양쪽으로 흩어진다"는 전제가 있어야 의미가 있는데,
                #    여기선 그 전제가 깨졌다. §8.3 은 원래 예비량(reserve_mm)
                #    으로 과소평가 위험을 막는 보수적 설계이므로, 두 추정
                #    중 더 큰 쪽(더 안전한 쪽)을 그대로 따른다.
                merged = max(rod_required_mm, req)
                print(f"  [용접봉] 재확인 예상 소요 {req:.0f}mm "
                      f"(최초 {rod_required_mm:.0f}mm → 최댓값 {merged:.0f}mm "
                      f"채택, 잔량 {rod_budget.remaining_mm:.0f}mm)")
                rod_required_mm = merged
                rod_ok = rod_budget.can_repair(rod_required_mm)
            else:
                rod_ok = rod_budget.can_repair(rod_required_mm)
                print(f"  [용접봉] 재확인에서 결함 재검출 안 됨 — 최초 "
                      f"추정 {rod_required_mm:.0f}mm 유지")
            _log_yolo_opinion(d_cur, "RECHECK")
            if not rod_ok:
                rod_budget.mark_exhausted_if_needed(rod_required_mm)
                end_reason = rod_budget.s.reason
                print(f"[STOP·보고] {end_reason} — 임무 규칙 8 에 따라 "
                      f"즉시 정지·보고 후 **복귀**한다 "
                      f"(결함{cur + 1} 용접 시도 안 함)")
                snap("stop_report")
                state, t_state = "RETURN", 0
            else:
                state, t_state = "CRUISE", 0

    elif state == "ALIGN":
        # 축방향은 바퀴를 저속으로 되물려 맞춘다(J1 만으로는 안 맞는다).
        s_tip = s_of(_tip, s_hint)
        ax_mm = (d_cur["s"] - s_tip) * 1000.0
        if abs(ax_mm) > ALIGN_AXIAL_TOL:
            drive(math.copysign(SPIN_DEG_S * 0.12, ax_mm))
        else:
            drive(0.0)
        # 🚨 **J1 관절값으로 닫으면 안 된다.** 스프링 드라이브라 지령과 실제가
        #    벌어지고(J2 에서 이미 겪었다: 지령 1.49 vs 관절 3.22mm), 게다가
        #    로봇이 관 중심에서 편심해 있어 같은 J1 이어도 팁이 가리키는
        #    **시계각**이 달라진다. 실측: J1 오차 1.14° 인데 정렬 오차 2.45mm.
        # → 측정한 **팁의 시계각**으로 닫는다(P 제어). J2 와 같은 이유·같은 방식.
        _tw = tip_end_world()
        _c, _t = PATH.point_tangent(s_tip)
        _dv = _tw - _c
        _dv = _dv - _t * np.dot(_dv, _t)
        _nn = np.linalg.norm(_dv)
        clock_tip = PATH.clock_of(s_tip, _dv / _nn) if _nn > 1e-9 else 0.0
        err_deg = (d_cur["clock"] - clock_tip + 180.0) % 360.0 - 180.0
        if t_state == 1:
            j1_cmd = j1_target_for(d_cur)          # 되먹임의 출발점
        j1_cmd = max(-TORCH["j1_limit_deg"],
                     min(TORCH["j1_limit_deg"], j1_cmd + J1_KP * err_deg))
        set_torch(j1_deg=j1_cmd, j2_m=0.0)
        ok = abs(err_deg) <= J1_TOL_DEG and abs(ax_mm) <= ALIGN_AXIAL_TOL
        if ok and t_state > 0.5 * PHYSICS_HZ:
            print(f"[ALIGN] 팁 시계각 {clock_tip:+.2f}° (목표 "
                  f"{d_cur['clock']:+.0f}°, 오차 {err_deg:+.2f}° = "
                  f"{PIPE_IR * 1000 * math.radians(abs(err_deg)):.2f}mm), "
                  f"축방향 {ax_mm:+.2f}mm, J1 지령 {j1_cmd:+.2f}°")
            state, t_state = "EXTEND", 0
        elif t_state > 10 * PHYSICS_HZ:
            print(f"[ALIGN] ⚠ 시간 초과 — 시계각 오차 {err_deg:+.2f}°, "
                  f"축방향 {ax_mm:+.2f}mm 남은 채로 진행")
            state, t_state = "EXTEND", 0

    elif state == "EXTEND":
        drive(0.0)
        # 🚨 고정 스트로크로는 간극이 원리적으로 안 맞는다 — 로봇이 관 중심에
        #    있지 않기 때문이다(피스톤 6개가 균형 잡은 자리에 선다). 측정한
        #    **팁 끝 반경**으로 닫는다.
        err = tip_r - TIP_TARGET_R
        j2_cmd = min(max(j2_cmd - J2_KP * err * MM, 0.0), J2_STROKE)
        set_torch(j1_deg=j1_cmd, j2_m=j2_cmd)
        if (abs(err) <= J2_TOL and t_state > 0.3 * PHYSICS_HZ) \
                or t_state > 5 * PHYSICS_HZ:
            j2_now = float(art.get_joint_positions()[j2_idx])
            print(f"[EXTEND] J2 = {j2_now * 1000:.2f}mm (지령 "
                  f"{j2_cmd * 1000:.2f})  팁 끝 반경 {tip_r:.2f}mm "
                  f"(관벽 {bore_radius(d_cur['s']):.1f} → 간극 "
                  f"{bore_radius(d_cur['s']) - tip_r:.2f}mm, 설계 2mm)")
            if abs(err) > J2_TOL:
                print(f"         ⚠ 목표 {TIP_TARGET_R:.1f}mm 에 {err:+.2f}mm "
                      f"못 맞췄다")
            state, t_state = "ARC", 0

    elif state == "ARC":
        # 🚨 **아크 동안 축을 계속 잡아 준다.** 지령을 0 으로 줘도 2초 사이에
        #    흘러가서 정렬이 문턱(1.5mm)을 오르내렸다 — repair_demo 에서
        #    미해결로 남겨 뒀던 바로 그 항목이다(1.13 / 1.45 / 1.60mm).
        #    ALIGN 과 같은 저속 되물림을 아크 중에도 유지한다.
        _ax = (d_cur["s"] - s_of(_tip, s_hint)) * 1000.0
        if abs(_ax) > ALIGN_AXIAL_TOL:
            drive(math.copysign(SPIN_DEG_S * 0.08, _ax))
        else:
            drive(0.0)
        set_torch(j1_deg=j1_cmd, j2_m=j2_cmd)
        if arc_light is not None and sparks is None:
            arc_light.GetIntensityAttr().Set(3.0e5)   # 깜빡임은 SparkFX 담당
        if t_state == 1:
            print(f"[ARC] 아크 {ARC_STEPS / PHYSICS_HZ:.1f}초"
                  + ("" if sparks is None else " — 스패터 발생"))
        if os.environ.get("ARC_CHECK") and t_state == ARC_STEPS // 2:
            snap("arc_mid")
            print("[ARC_CHECK] 아크 중 스냅 완료 — 종료")
            simulation_app.close()
            sys.exit(0)
        if t_state > ARC_STEPS:
            if arc_light is not None:
                arc_light.GetIntensityAttr().Set(0.0)
            if sparks is not None:
                sparks.clear()
            # 🚨 **링크 원점이 아니라 팁 끝으로 재야 한다.** 아크가 일어나는
            #    곳은 팁 끝이고, 로봇이 편심해 있어서 같은 로드 위의 두 점이
            #    관 축 기준으로 **다른 시계각**을 갖는다(실측: 원주 오차가
            #    링크 원점 1.43mm vs 팁 끝 0.44mm — ALIGN 은 팁 끝으로 닫는데
            #    SWAP 만 원점으로 재서 판정이 어긋났다).
            tipw = tip_end_world()
            s_tip, _ = PATH.project(tipw, s_hint)
            # 🚨 결함 쪽은 **다시 투영하지 말 것.** s 와 시계각을 이미 정확히
            #    알고 있는데 월드 점을 되투영하면 표 간격(2mm)만큼 잡음이
            #    섞여 정렬 오차가 문턱(1.5mm)을 넘나든다. 기준각을 결함
            #    시계각으로 잡았으므로 결함의 펼친 좌표는 (s, 0) 이 정확하다.
            _rc = d_cur["clock"]
            _uv = wall_uv(tipw, s_tip, _rc)
            align_mm = sequencer.align_error(
                _uv, np.array([d_cur["s"] * 1000.0, 0.0, 0.0]))
            # 성분을 남긴다 — 합만 보면 축방향인지 원주인지 못 가른다.
            print(f"  [정렬] 축방향 {_uv[0] - d_cur['s'] * 1000.0:+.2f}mm  "
                  f"원주 {_uv[1]:+.2f}mm  합 {align_mm:.2f}mm   "
                  f"(팁링크 s {s_tip * 1000:.2f} / 팁끝 s "
                  f"{s_of(_tip, s_hint) * 1000:.2f} / 결함 s "
                  f"{d_cur['s'] * 1000:.2f})")
            # 🚨 정렬 성패와 무관하게 아크는 이미 일어났다 — 로드는 소모됐다
            #    (§8.1). 정렬 실패는 "결함을 못 없앴다"이지 "로드를 안 썼다"가
            #    아니다.
            rod_budget.consume(rod_required_mm)
            print(f"  [용접봉] 이번 {rod_required_mm:.0f}mm 소모 "
                  f"— 잔량 {rod_budget.remaining_mm:.0f}/{rod_budget.k['total_mm']:.0f}mm")
            aligned = align_mm <= ALIGN_TOL_MM
            # 🔑 비드는 **토치가 실제로 있던 자리**에 놓는다. 결함 자리에 고정으로
            #    띄우면 정렬이 실패해도 검증이 뚫린다(설계 7.2).
            # 🚨 어긋남은 **관 축(접선) 방향**으로 준다. repair_demo 는 월드 X 로
            #    줬는데 그 코스는 관 축이 X 였기 때문이다 — 여기서 그대로 쓰면
            #    엉뚱한 방향으로 밀린다.
            _off = d_cur["tangent"] * (s_tip - d_cur["s"])
            d_cur["bead_op"].Set(d_cur["xf"] * trans(float(_off[0]),
                                                     float(_off[1]),
                                                     float(_off[2])))
            UsdGeom.Imageable(d_cur["bead"]).MakeVisible()
            if aligned:
                UsdGeom.Imageable(d_cur["hole"]).MakeInvisible()
                before = leak_count(d_cur) if WATER else 0
                if d_cur.get("plug") is not None:
                    d_cur["plug_coll"].CreateCollisionEnabledAttr(True)
                    UsdGeom.Imageable(d_cur["plug"]).MakeVisible()
                print(f"[SWAP] 정렬 오차 {align_mm:.2f}mm ≤ {ALIGN_TOL_MM}mm "
                      f"— 결함 → 비드 + 메꿈 착좌"
                      + (f" (직전 누수 {before}개)" if WATER else ""))
            else:
                print(f"[SWAP] ⚠ 정렬 오차 {align_mm:.2f}mm > {ALIGN_TOL_MM}mm "
                      f"— 결함을 남긴다. 비드만 어긋난 자리에(설계 7.2)")
            state, t_state = "COOL", 0

    elif state == "COOL":
        drive(0.0)
        set_torch(j1_deg=j1_cmd, j2_m=0.0)
        if t_state > COOL_STEPS:
            print(f"[COOL/RETRACT] J2 수납")
            repos_from = s_ring
            state, t_state = "REPOSITION", 0

    elif state == "REPOSITION":
        # 🔑 **촬영했던 거리까지만** 물러난다. 더 멀어지면 구멍의 어두운 영역이
        #    관 저 끝과 이어져 중앙 덩어리로 흡수돼 놓친다(실측).
        drive(-SPIN_DEG_S)
        set_torch(j1_deg=0.0, j2_m=0.0)
        cam_fwd = (d_cur["s"] - s_of(stage.GetPrimAtPath(FRONT_CAM),
                                     s_hint)) * 1000.0
        target = inspect_fwd_mm if inspect_fwd_mm > 1.0 else 90.0
        if cam_fwd >= target or (repos_from - s_ring) >= 0.30:
            drive(0.0)
            print(f"[REPOSITION] 후진 {(repos_from - s_ring) * 1000:.1f}mm — "
                  f"카메라가 결함 {cam_fwd:.1f}mm 앞 (촬영 때 "
                  f"{inspect_fwd_mm:.1f}mm)")
            state, t_state = "VERIFY", 0

    elif state == "VERIFY":
        drive(0.0)
        if t_state > 1.0 * PHYSICS_HZ:
            _, post_hole, post_bead, _, _ = inspect_defect(
                d_cur, f"결함{cur + 1} 수리 후")
            snap(f"d{cur + 1}_2_after")
            # 🔑 **판정은 비드(색)로만 한다** (2026-08-05 사용자 결정).
            #    어둠으로 어둠을 가르지 않는다 — 구멍도 어둡고, 관 저 끝도
            #    어둡고, 조명이 한쪽으로 몰린 실전 맵에서는 그늘진 관벽까지
            #    어둡다. 실측에서 수리 뒤 51px 떨어진 그늘이 4,216px 짜리
            #    "구멍" 으로 보고돼 판정이 뒤집혔다. 비드만 주황(채도 높음)이라
            #    관벽(회색)·관 저 끝(검정)과 절대 안 헷갈린다.
            #    구멍 면적은 **참고로만** 남긴다. 판정에 안 쓴다.
            #
            #    ⚠ 이것이 자기충족이 아닌 이유: 비드는 ① 정렬 오차가 실측
            #      1.5mm 이내일 때만 켜지고 ② **토치가 실제로 있던 자리**에
            #      놓이며(결함 자리에 고정으로 띄우지 않는다) ③ 카메라가 실제로
            #      렌더해서 그 화소에서 찾아야 한다. 정렬이 빗나가면 비드가
            #      엉뚱한 곳에 남아 `matched` 가 깨진다.
            if post_bead is not None:
                ok = True
                msg = (f"✅ 수리 확인 — 비드가 결함 자리에 있다 "
                       f"({post_bead['area_px']:,}px, 예상자리와 "
                       f"{post_bead['dist_px']:.0f}px)")
            else:
                ok = False
                msg = "❌ 미수리 — 결함 자리에 비드가 없다"
            if pre_bead is not None:
                ok = None
                msg = ("△ 판정 불가 — **수리 전에 이미** 비드가 잡혔다"
                       "(색 검출 조건을 의심할 것)")
            _h = (f"{pre_hole['area_px']:,}px" if pre_hole else "안 잡힘")
            _h2 = (f"{post_hole['area_px']:,}px" if post_hole else "안 잡힘")
            msg += f"   [참고: 어두운 덩어리 {_h} → {_h2}]"
            if WATER:
                msg += f"  누수 입자 {leak_count(d_cur)}개"
            print(f"[VERIFY] 결함 {cur + 1} 판정 {msg}")
            # 🚨 용접 완료도 YOLO 2차검증 — 판정은 그대로 비드(색)가 하고,
            #    YOLO 는 "수리 후에도 크랙을 여전히 보는가" 를 나란히 로그로
            #    남긴다(사용자 지시: "대체 말고 같이"). 기대: 진짜 고쳐졌으면
            #    YOLO 도 더는 못 봐야(detected=False) 일치.
            if cam_bridge is not None:
                _yv_det, _ = cam_bridge.yolo_opinion()
                if _yv_det is None:
                    print("  [YOLO 용접완료·2차검증] 의견 없음 "
                          "(최근 3초 내 수신 안 됨)")
                else:
                    _yolo_fixed = not _yv_det
                    _bead_fixed = bool(ok) if ok is not None else None
                    if _bead_fixed is None:
                        _cmp = "비드 판정 불가라 비교 생략"
                    elif _yolo_fixed == _bead_fixed:
                        _cmp = "비드 판정과 동의"
                    else:
                        _cmp = "⚠ 비드 판정과 불일치"
                    print(f"  [YOLO 용접완료·2차검증] "
                          f"{'미검출(수리됨 의견)' if _yolo_fixed else '검출함(안 고쳐짐 의견)'}"
                          f" — {_cmp}")
            verdicts.append({"i": cur, "ok": ok, "msg": msg})
            # 🚨 §8.3 / 임무 규칙 8 — 복귀 사유는 관 단절과 용접봉 소진뿐이다.
            #    수리했다고 복귀하지 않는다 — 소진 여부는 이제 *다음* 결함을
            #    실제로 찾았을 때(INSPECT 의 mark_exhausted_if_needed) 판단한다.
            #    여기서 판단하면 다음 결함을 발견하기도 전에, 그리고 그
            #    시점의 잔량이 그 결함을 감당할 수 있는지도 모른 채 복귀해
            #    버린다 — INSPECT 의 사전 확인과 값은 같아도(둘 다 잔량 대비
            #    default_repair_mm) 순서상 여기가 먼저라 항상 여기서 이겨
            #    버려서 "결함을 발견했는데 부족해서 복귀" 시나리오 자체가
            #    영영 재현되지 않았다(2026-08-07 수정).
            state, t_state = "RESUME", 0

    elif state == "RESUME":
        # 🚨 **수리했다고 복귀하지 않는다.** 복귀 사유는 관 단절과 용접봉
        #    소진뿐이다(임무 규칙 8). 다음 결함 / 코스 끝까지 계속 간다.
        drive(SPIN_DEG_S)
        set_torch(j1_deg=0.0, j2_m=0.0)
        if t_state == 1:
            cur += 1
            inspected, rechecked, pre_hole, pre_bead = False, False, None, None
            nxt = (f"다음 결함 {cur + 1} (s "
                   f"{DEFECTS[cur]['s'] * 1000:.0f}mm)" if cur < len(DEFECTS)
                   else "남은 결함 없음 — 코스 끝까지 점검")
            print(f"[RESUME] 점검 계속 (복귀 아님) — {nxt}")
        if t_state > 0.5 * PHYSICS_HZ:
            state, t_state = "CRUISE", 0

    elif state == "RECOVER":
        # 임무 규칙 8 — 끼임은 후진 후 재시도(최대 3회).
        drive(-SPIN_DEG_S)
        set_torch(j1_deg=0.0, j2_m=0.0)
        if t_state == 1:
            globals()["_rec_s0"] = s_hint
        if (_rec_s0 - s_hint) > 0.100 or t_state > 8 * PHYSICS_HZ:
            print(f"  [RECOVER] 후진 {(_rec_s0 - s_hint) * 1000:.0f}mm — 재진입")
            _stuck_s, _stuck_t = None, 0
            state, t_state = "CRUISE", 0

    elif state == "JUNCTION":
        # 오른손 법칙으로 갈 가지를 고른다. 실행 수단은 서스펜션 예압 비대칭뿐.
        if t_state == 1:
            rc = right_branch_clock()
            print(f"[JUNCTION] 오른손 법칙 — 오른쪽 = 시계각 {rc:.0f}° 방향. "
                  f"서스펜션 예압을 그쪽으로 비대칭으로 준다")
            print("           ❓ 이 맵의 T 는 **날카로운 T(필렛 없음)** 이고 "
                  "벨로우즈 굽힘은 평면당 ±55° 라 급회전 90° 는 원리적으로 "
                  "부족하다. 되는지 안 되는지를 여기서 기록한다")
            globals()["_junc_s0"] = s_hint
            globals()["_junc_clock"] = rc
            bias_pistons(rc, rebuild=True)
        bias_pistons(_junc_clock)
        drive(SPIN_DEG_S * 0.5)
        if t_state > 12 * PHYSICS_HZ:
            release_pistons()
            drive(0.0)
            adv = (s_hint - _junc_s0) * 1000.0
            bel = ([math.degrees(float(q)) for q in
                    np.asarray(art.get_joint_positions())[bel_idx]]
                   if bel_idx else [0.0])
            end_reason = (f"T 분기 진입 시도 12초 — 진행 {adv:+.1f}mm, "
                          f"벨로우즈 {min(bel):+.1f}~{max(bel):+.1f}° "
                          f"(한계 ±27.5°/관절)")
            print(f"[JUNCTION] {end_reason}")
            print("           " + ("가지에 들어갔다" if adv > 60 else
                                   "★ 통과 실패 — 예상대로 굽힘이 모자란다"))
            state, t_state = "DONE", 0

    elif state == "DISCONNECTED":
        # 임무 규칙 8 — 단절은 즉시 정지·보고·복귀다.
        if t_state == 1:
            # 사유를 그대로 찍는다 — 단절도, 관경 축소도, 끼임 3회 실패도
            # 여기로 온다. "관 단절 확인" 으로 고정해 두면 로그가 거짓말한다.
            print(f"[STOP·보고] {end_reason or '관 단절'} — 임무 규칙 8 에 따라 "
                  f"즉시 정지·보고 후 **복귀**한다")
            snap("stop_report")
        if t_state > 1.0 * PHYSICS_HZ:
            state, t_state = "RETURN", 0

    elif state == "RETURN":
        drive(-SPIN_DEG_S)
        set_torch(j1_deg=0.0, j2_m=0.0)
        if t_state == 1:
            # 🚨 진입 사유를 여기서 붙잡아 둔다 — 안 그러면 도착 시점에 아래
            #    end_reason 을 덮어쓰면서 "관 단절"로 고정돼, 용접봉 소진으로
            #    들어온 경우도 거짓으로 그렇게 찍힌다.
            globals()["_return_cause"] = end_reason or "복귀"
            print(f"[RETURN] 복귀 시작 — 진입점까지 {s_hint * 1000:.0f}mm")
        if t_state % 1200 == 0:
            print(f"  복귀 중  s {s_hint * 1000:7.1f}mm")
        if s_hint * 1000 < RETURN_DIAG_S_MM:
            _diag_dt.append(_loop_dt)
            if t_state % RETURN_DIAG_EVERY == 0:
                # 휠별 시계각(결함 표기와 같은 규약, PATH.clock_of)까지 남긴다
                # — 접지 상실이 특정 시계각 쪽으로 쏠리는지 보려는 것.
                _c_now, _t_now2 = PATH.point_tangent(s_hint)
                wheel_detail = []
                for sg in (0, 1):
                    for i in range(3):
                        for jj in (0, 1):
                            p = wpos(stage.GetPrimAtPath(
                                f"{ROBOT}/seg{sg}_wheel_{i}_{jj}"))
                            d = p - _c_now
                            d_perp = d - _t_now2 * np.dot(d, _t_now2)
                            r_mm = float(np.linalg.norm(d_perp)) * 1000
                            _nn2 = np.linalg.norm(d_perp)
                            clk = (PATH.clock_of(s_hint, d_perp / _nn2)
                                   if _nn2 > 1e-9 else 0.0)
                            wheel_detail.append((f"s{sg}i{i}j{jj}", clk, r_mm))
                wr = [r for _, _, r in wheel_detail]
                wheel_detail.sort(key=lambda w: w[1])
                wline = "  ".join(
                    f"{lbl}{clk:+4.0f}°{r:4.1f}mm{'' if r > 39.0 else '✗'}"
                    for lbl, clk, r in wheel_detail)
                q = np.asarray(art.get_joint_positions(), dtype=float)
                pis = [q[k] * 1000 for k in piston_idx]
                # 다리(피스톤)별 — 힘 부족(더 뻗을 여지 있음)인지 스트로크
                # 부족(이미 상한 10mm 근처인데도 안 닿음)인지 구분하려는 것.
                _pmm_by_name = dict(zip(piston_name, pis))
                _leg_lines = []
                for sg in (0, 1):
                    for i in range(3):
                        _ws = [(clk, r) for lbl, clk, r in wheel_detail
                               if lbl.startswith(f"s{sg}i{i}j")]
                        _avg_clk = sum(c for c, _ in _ws) / len(_ws)
                        _avg_r = sum(r for _, r in _ws) / len(_ws)
                        _pmm = _pmm_by_name.get(f"seg{sg}_piston_{i}", float("nan"))
                        _leg_lines.append((_avg_clk,
                            f"s{sg}i{i}{_avg_clk:+4.0f}° 피스톤{_pmm:4.1f}"
                            f"/10mm 반경{_avg_r:4.1f}mm"
                            f"{'' if _avg_r > 39.0 else '✗'}"))
                _leg_lines.sort(key=lambda w: w[0])
                legline = "  ".join(s for _, s in _leg_lines)
                _p_now, _t_now, _u_now, _w_now = PATH.frame(s_hint)
                _m = wmat(_seg1)
                _fwd_now, _xax_now = -_m[2, :3], _m[0, :3]
                _axial = math.degrees(math.acos(np.clip(np.dot(
                    _fwd_now, _t_now) / (np.linalg.norm(_fwd_now) + 1e-9),
                    -1, 1)))
                _x_perp = _xax_now - _t_now * np.dot(_xax_now, _t_now)
                _x_perp /= (np.linalg.norm(_x_perp) + 1e-9)
                _roll = math.degrees(math.acos(np.clip(
                    np.dot(_x_perp, _u_now), -1, 1)))
                _dts = np.array(_diag_dt) if _diag_dt else np.array([0.0])
                print(f"  [RETURN진단] s{s_hint * 1000:7.1f}mm  휠접지 "
                      f"{min(wr):.1f}~{max(wr):.1f}mm 밀착 "
                      f"{sum(1 for r in wr if r > 39.0)}/12  피스톤 "
                      f"{min(pis):4.1f}~{max(pis):4.1f}mm  축틀어짐 "
                      f"{_axial:5.1f}°  롤 {_roll:5.1f}°  스텝시간 "
                      f"{_dts.mean() * 1000:.2f}(최대{_dts.max() * 1000:.2f})ms")
                print(f"    [RETURN진단-휠] {wline}")
                print(f"    [RETURN진단-다리] {legline}")
                _diag_dt.clear()
        if s_hint <= START_S_MM * MM + 0.02:
            drive(0.0)
            end_reason = f"{_return_cause} — 진입점 복귀 완료"
            print(f"[RETURN] 진입점 도착 — s {s_hint * 1000:.1f}mm")
            state, t_state = "DONE", 0
        elif t_state > 400 * PHYSICS_HZ:
            end_reason = f"{_return_cause} — 복귀 시간 초과, s {s_hint * 1000:.1f}mm 에서 멈춤"
            state, t_state = "DONE", 0

    # ── 아크 스패터 (시각 전용) ────────────────────────────────────
    # 🔑 **FSM 뒤에 둔다.** 아크 조명 깜빡임을 여기서 걸기 때문이다 — 앞에
    #    두면 ARC 분기의 `Set(3.0e5)` 가 매 스텝 덮어써서 깜빡임이 안 보인다.
    # 🚨 `confine` 을 반드시 준다. 안 주면 스패터가 관벽을 뚫고 건물 안으로
    #    날아간다(기중 정지거리 0.5m 이상 vs 관 내반경 0.05m).
    if sparks is not None:
        if state == "ARC" and d_cur is not None:
            _sd = d_cur["s"]
            # 🚨 가둠 함수 만들기는 중심선 샘플링 + 레이캐스트 12발이다.
            #    매 스텝 하면 안 된다 — 결함마다 한 번만 만들고 캐시한다.
            if _spark_cyl is None or _spark_cyl[0] != _sd:
                _spark_cyl = (_sd, make_spark_confine(_sd))
            _spo = tip_end_world()
            sparks.step(1.0 / PHYSICS_HZ, emitting=True, origin=_spo,
                        normal=wall_inward(_spo, _sd), light=arc_light,
                        confine=_spark_cyl[1])
        else:
            _spark_cyl = None
            sparks.step(1.0 / PHYSICS_HZ, emitting=False)

if not reported:
    report()

if cam_bridge is not None:
    print(f"[종료] ROS2 활성 카메라 발행 — 총 {cam_bridge.n} 프레임")
    cam_bridge.destroy_node()
    rclpy.shutdown()

import threading                                          # noqa: E402

threading.Thread(target=simulation_app.close, daemon=True).start()
threading.Event().wait(8.0)
sys.stdout.flush()
os._exit(0)
