#!/usr/bin/env python3
"""pipe_robot_v6 + restroom_pipeR150 GUI 라이브 스모크 테스트 준비.

미션 로직이 전혀 없다 — 맵과 로봇을 스테이지에 올려놓고 물리를 이미 돌린
채로 GUI 를 띄워둔다. Window > Script Editor 에 robot_final/final_script.py
내용을 붙여넣으면(PIPE/JUNCTION 상수만 고치고) 바로 go() 까지 자동 실행된다
— Play 버튼을 따로 누를 필요 없다(아래 참고).

실행:
    DISPLAY=:1 isaac_python robot_final/gui_smoke_test.py

🚨 **raw omni.usd + GUI Play 버튼 조합은 실측으로 실패했다** — 첫 실행에서
   로봇의 RigidBody 29개 전부에 "Invalid PhysX transform" 경고가 뜨고
   프로세스가 죽었다. real_map_demo.py 를 다시 보니 **`isaacsim.core.api.World`
   로 만든 stage 에, 배치를 다 구운 뒤 `world.reset()` 을 호출**하는 순서를
   쓴다(raw omni.usd 스테이지가 아니다) — 그 초기화 경로가 GUI 의 Play 버튼이
   타는 경로와 다른 것으로 보인다. 이번 버전은 그 패턴을 그대로 따른다.

Script Editor 에서 할 일:
  - final_script.py 를 붙여넣기 전에 PIPE 상수를 고칠 것 — 이 스테이지엔
    "/World/test_pipe_tee_ID100" 가 없다. JUNCTION = (x,y,z) 세계좌표를
    직접 주거나 PIPE 를 다른 프림 경로로 바꾼다.
  - floor2 코스는 real_map_demo.py 기준 실제 T분기가 없다("disconnect"로
    끝남) — 분기 조향 자체를 보려면 floor1(730,850mm 부근 T분기) 좌표가
    필요할 수 있다.
"""
import math
import sys
from pathlib import Path as _P

import numpy as np

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import World                        # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics        # noqa: E402

SON = _P(__file__).resolve().parent.parent
MAP_USD = str(SON / "maps" / "restroom_pipeR150.usd")
ROBOT_USDA = str(SON / "robot_final" / "pipe_robot_v6.usda")
MM = 0.001
FLOOR = "floor2"
Z_NET = -250.0                 # real_map_demo.py FLOORS["floor2"]["z_net"] 그대로
# real_map_demo.py FLOORS["floor2"]["corners"] 그대로 — 같은 CAD 계열이라
# 좌표를 그대로 재사용한다.
CORNERS_MM = [(330, 850, 85), (330, 850, -250), (680, 850, -250),
              (680, 1400, -250), (1200, 1400, -250),
              (1200, 600, -250), (1500, 600, -250)]
BEND_R = 0.150
START_S_MM = 120.0              # 진입부보다 z 로 더 내려간 지점(사용자 확인 2회)
PHYSICS_HZ = 240.0              # pipe_robot_v6.usda 의 PhysicsScene 설정과 동일
PIPE_IR = 0.050                  # real_map_demo.py 와 동일 설계 내반경(m)


def W(p_mm):
    return np.array([p_mm[0] * MM, p_mm[1] * MM, (p_mm[2] - Z_NET) * MM])


def trans(x, y, z):
    m = Gf.Matrix4d(1.0)
    m.SetTranslate(Gf.Vec3d(x, y, z))
    return m


class CenterLine:
    """real_map_demo.py::CenterLine 그대로(직선/원호 교대 중심선). 여기선
    시작 지점의 위치+접선만 필요해 project()/tabulate() 정밀 표는 생략."""

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
        i = min(int(np.searchsorted(self.cum, s, side="right")), len(self.segs) - 1)
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
        p, t = self.point_tangent(s)
        u = np.array([0.0, 0.0, 1.0]) - t * t[2]
        if np.linalg.norm(u) < 1e-6:
            u = np.array([1.0, 0.0, 0.0]) - t * t[0]
        u /= np.linalg.norm(u)
        return p, t, u, np.cross(u, t)


PATH = CenterLine([W(c) for c in CORNERS_MM], BEND_R)

# 🚨 real_map_demo.py 와 동일 — raw omni.usd 스테이지가 아니라 Isaac 의
#    World 로 만든다. world.reset() 이전에 배치를 다 굽는다.
world = World(stage_units_in_meters=1.0,
              physics_dt=1.0 / PHYSICS_HZ, rendering_dt=1.0 / 60.0)
stage = world.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

# ── 맵 ──────────────────────────────────────────────────────────────
map_root = stage.DefinePrim("/World/Map", "Xform")
map_root.GetReferences().AddReference(MAP_USD)
_xf = UsdGeom.Xformable(map_root)
_xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -Z_NET * MM))
_xf.AddScaleOp().Set(Gf.Vec3f(MM, MM, MM))

n_inst = 0
for p in stage.Traverse():
    if p.IsInstance():
        p.SetInstanceable(False)
        n_inst += 1

map_meshes = [p for p in stage.Traverse()
              if p.IsA(UsdGeom.Mesh) and str(p.GetPath()).startswith("/World/Map")]
this_floor = [p for p in map_meshes if f"/{FLOOR}/" in str(p.GetPath())]
other = [p for p in map_meshes if p not in this_floor]

for p in this_floor:
    UsdPhysics.CollisionAPI.Apply(p)
    # 🚨 배관은 반드시 approximation="none" — convexHull 이면 관 내부가 꽉 찬다
    #    (real_map_demo.py 에서 실측으로 확인된 함정, 그대로 적용).
    UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr("none")
for p in other:
    UsdPhysics.CollisionAPI.Apply(p).CreateCollisionEnabledAttr(False)

print(f"[준비] 맵 {_P(MAP_USD).name} — instanceable 해제 {n_inst}개, "
      f"메시 {len(map_meshes)}개, {FLOOR} 충돌 {len(this_floor)}개(approximation=none), "
      f"나머지 {len(other)}개는 렌더만")

# ── 로봇 ────────────────────────────────────────────────────────────
# 🚨 부모 Xform(articulation root)에 변환을 걸면 안 된다 — 각 RigidBody
#    자식의 로컬 변환에 개별적으로 굽는다(real_map_demo.py 와 동일 패턴).
#    final_script.py 는 로봇 루트의 authored 회전 중 로컬 **+X 축을 진행
#    방향(travel)**으로 읽는다(_setup() 의 ax0[0]).
# 🔁 사용자 확인: 처음 배치가 "뒤집힌" 것처럼 보여 진행방향을 반대로 뒤집었다
#    (DiscF 가 이끄는 게 아니라 반대쪽이 먼저 내려가던 상태였을 가능성).
_p_start, _t_start, _u_start, _w_start = PATH.frame(START_S_MM * MM)
_fwd = -_t_start                  # 뒤집기(사용자 확인) — 반대쪽 끝이 앞장서게
_xc = _fwd                        # 로봇 로컬 +X = 진행 방향(final_script.py 규약)
_yc = -_u_start                   # X 를 뒤집었으니 오른손계 유지하려고 Y 도 같이 뒤집는다
_zc = np.cross(_xc, _yc)          # 오른손 좌표계 유지
_R = Gf.Matrix4d(1.0)
_R.SetRow3(0, Gf.Vec3d(*_xc))
_R.SetRow3(1, Gf.Vec3d(*_yc))
_R.SetRow3(2, Gf.Vec3d(*_zc))
PLACE = _R * trans(float(_p_start[0]), float(_p_start[1]), float(_p_start[2]))

ROBOT_PATH = "/World/pipe_robot_v6"
robot_prim = stage.DefinePrim(ROBOT_PATH, "Xform")
robot_prim.GetReferences().AddReference(ROBOT_USDA)
# 🚨 robot_prim 의 바로 밑 자식이 아니라(defaultPrim 이 "World" 라 "Robot"
#    Xform 한 겹이 더 있다 — final_script.py 의 "%s/Robot/DiscF" 참조와
#    일치), 서브트리 전체에서 RigidBody 를 찾아야 한다.
_n_baked = 0
for p in stage.Traverse():
    if not str(p.GetPath()).startswith(ROBOT_PATH + "/"):
        continue
    if not p.HasAPI(UsdPhysics.RigidBodyAPI):
        continue
    xf = UsdGeom.Xformable(p)
    local = xf.GetLocalTransformation()
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(local * PLACE)
    _n_baked += 1
print(f"[준비] 로봇 {ROBOT_PATH} — PATH.frame(s={START_S_MM:.0f}mm) 기준 배치(진행방향 뒤집음), "
      f"RigidBody {_n_baked}개 로컬 변환에 구움, 위치 "
      f"({_p_start[0] * 1000:.0f},{_p_start[1] * 1000:.0f},{_p_start[2] * 1000:.0f})mm "
      f"진행방향 ({_fwd[0]:+.2f},{_fwd[1]:+.2f},{_fwd[2]:+.2f})")

light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
light.CreateIntensityAttr(800.0)

# 🚨 real_map_demo.py 와 동일 순서: 배치를 다 구운 **뒤에** reset() —
#    이게 raw omni.usd+Play 조합에서 실패했던 "Invalid PhysX transform"
#    을 피하는 핵심이다(World 가 물리 attach 를 이 시점에 제대로 한다).
world.reset()
print("[준비] world.reset() 완료 — 안착 추이를 여러 체크포인트로 지켜본다")

# ── 안착 확인 — 휠 12개가 파이프 벽에 걸쳐있는지 반경으로 판정 ──────────
# real_map_demo.py::radial_r() 와 같은 방식(중심선 s 기준 반경, mm).
_WHEEL_NAMES = ["B_A0", "B_A1", "B_A2", "B_B0", "B_B1", "B_B2",
                "DF_A0", "DF_A1", "DF_A2", "DR_A0", "DR_A1", "DR_A2"]
_XC = UsdGeom.XformCache()


def _wpos(prim):
    _XC.Clear()
    t = _XC.GetLocalToWorldTransform(prim).ExtractTranslation()
    return np.array([float(t[0]), float(t[1]), float(t[2])])


def _radial_r(p, s):
    c, t = PATH.point_tangent(s)
    d = p - c
    return float(np.linalg.norm(d - t * np.dot(d, t))) * 1000.0


def _check_settle(tag):
    # 로봇이 s=120mm 부근에서 시작했지만 안착 중 밀릴 수 있으니, seg 중심
    # (대략 Body 위치)으로 s 를 다시 투영해서 쓴다 — 이 구간은 직선이라
    # 거친 스캔으로도 오차가 크지 않다.
    _body_prim = stage.GetPrimAtPath(f"{ROBOT_PATH}/Robot/Body")
    _body_p = _wpos(_body_prim)
    _s_scan = np.arange(max(0.0, START_S_MM * MM - 0.08), START_S_MM * MM + 0.08, 0.001)
    _s_cur = min(_s_scan, key=lambda s: float(np.linalg.norm(PATH.point_tangent(s)[0] - _body_p)))
    _radii = []
    for nm in _WHEEL_NAMES:
        prim = stage.GetPrimAtPath(f"{ROBOT_PATH}/Robot/{nm}_Wheel")
        if not prim.IsValid():
            print(f"[안착·{tag}] ⚠ {nm}_Wheel 프림을 못 찾음")
            continue
        _radii.append(_radial_r(_wpos(prim), _s_cur))
    if not _radii:
        print(f"[안착·{tag}] 데이터 없음")
        return
    _fmt = lambda xs: " ".join(f"{r:.1f}" for r in xs)  # noqa: E731
    print(f"[안착·{tag}] s={_s_cur * 1000:.1f}mm  "
          f"Body[{_fmt(_radii[0:6])}]  DiscF[{_fmt(_radii[6:9])}]  "
          f"DiscR[{_fmt(_radii[9:12])}]")


_tick = 0
for _tick_stop in (240, 480, 720, 960):
    while _tick < _tick_stop:
        world.step(render=True)
        _tick += 1
    _check_settle(f"{_tick_stop / 240.0:.1f}s")

print("=" * 78)
print("[준비 완료] 이미 물리 재생 중이다(Play 버튼 안 눌러도 됨). "
      "Window > Script Editor 에 robot_final/final_script.py 붙여넣고 "
      "PIPE/JUNCTION 상수만 고쳐서 Run — go() 까지 자동 실행됨")
print("=" * 78)

while simulation_app.is_running():
    world.step(render=True)
simulation_app.close()
