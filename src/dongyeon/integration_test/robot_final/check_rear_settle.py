#!/usr/bin/env python3
"""gui_smoke_test.py 와 같은 씬을 만들고, final_script.py 의 reset()(rear bias
포함)까지 실행해서 DiscR 진동이 잡히는지 확인한다. go()(주행 시작)는 부르지
않는다 — 안착 자체만 보려는 목적.

실행:
    DISPLAY=:1 isaac_python robot_final/check_rear_settle.py
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
FINAL_SCRIPT = _P(__file__).resolve().parent / "final_script.py"
MM = 0.001
FLOOR = "floor2"
Z_NET = -250.0
CORNERS_MM = [(330, 850, 85), (330, 850, -250), (680, 850, -250),
              (680, 1400, -250), (1200, 1400, -250),
              (1200, 600, -250), (1500, 600, -250)]
BEND_R = 0.150
START_S_MM = 120.0
PHYSICS_HZ = 240.0
PIPE_IR = 0.050
ROBOT_PATH = "/World/pipe_robot_v6"


def W(p_mm):
    return np.array([p_mm[0] * MM, p_mm[1] * MM, (p_mm[2] - Z_NET) * MM])


def trans(x, y, z):
    m = Gf.Matrix4d(1.0)
    m.SetTranslate(Gf.Vec3d(x, y, z))
    return m


class CenterLine:
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

    def tabulate(self, ds=0.002):
        self.tab_s = np.arange(0.0, self.total + 1e-9, ds)
        self.tab_p = np.array([self.point_tangent(x)[0] for x in self.tab_s])
        self.tab_ds = ds
        return self

    def project(self, p, hint=None, win=0.30):
        """real_map_demo.py::CenterLine.project() 그대로 — hint 없으면 전체를
        본다(휠마다 실제로 가장 가까운 s 를 따로 찾아야 굽은 구간에 걸친
        로봇을 잘못 측정하지 않는다)."""
        if hint is None:
            lo_i, hi_i = 0, len(self.tab_s)
        else:
            lo_i = max(0, int((hint - win) / self.tab_ds))
            hi_i = min(len(self.tab_s), int((hint + win) / self.tab_ds) + 1)
            if hi_i - lo_i < 2:
                lo_i, hi_i = 0, len(self.tab_s)
        d2 = np.sum((self.tab_p[lo_i:hi_i] - p) ** 2, axis=1)
        k = lo_i + int(np.argmin(d2))
        lo = max(0.0, self.tab_s[k] - self.tab_ds)
        hi = min(self.total, self.tab_s[k] + self.tab_ds)
        ss = np.linspace(lo, hi, 9)
        P = np.array([self.point_tangent(x)[0] for x in ss])
        k2 = int(np.argmin(np.sum((P - p) ** 2, axis=1)))
        s_best = float(ss[k2])
        return s_best, float(np.linalg.norm(P[k2] - p))


PATH = CenterLine([W(c) for c in CORNERS_MM], BEND_R).tabulate()
print(f"[코스] 직선(진입) 구간 총 {PATH.total * 1000:.0f}mm 중 첫 굽음 시작 지점 확인용 — "
      f"세그먼트: " + ", ".join(f"{seg[0]}({seg[-1]*1000:.0f}mm)" for seg in PATH.segs))

world = World(stage_units_in_meters=1.0,
              physics_dt=1.0 / PHYSICS_HZ, rendering_dt=1.0 / 60.0)
stage = world.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

map_root = stage.DefinePrim("/World/Map", "Xform")
map_root.GetReferences().AddReference(MAP_USD)
_xf = UsdGeom.Xformable(map_root)
_xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -Z_NET * MM))
_xf.AddScaleOp().Set(Gf.Vec3f(MM, MM, MM))

for p in stage.Traverse():
    if p.IsInstance():
        p.SetInstanceable(False)

map_meshes = [p for p in stage.Traverse()
              if p.IsA(UsdGeom.Mesh) and str(p.GetPath()).startswith("/World/Map")]
this_floor = [p for p in map_meshes if f"/{FLOOR}/" in str(p.GetPath())]
other = [p for p in map_meshes if p not in this_floor]
for p in this_floor:
    UsdPhysics.CollisionAPI.Apply(p)
    UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr("none")
for p in other:
    UsdPhysics.CollisionAPI.Apply(p).CreateCollisionEnabledAttr(False)
print(f"[준비] 맵 {len(map_meshes)}개 메시, {FLOOR} 충돌 {len(this_floor)}개")

_p_start, _t_start, _u_start, _w_start = PATH.frame(START_S_MM * MM)
_fwd = -_t_start
_xc = _fwd
_yc = -_u_start
_zc = np.cross(_xc, _yc)
_R = Gf.Matrix4d(1.0)
_R.SetRow3(0, Gf.Vec3d(*_xc))
_R.SetRow3(1, Gf.Vec3d(*_yc))
_R.SetRow3(2, Gf.Vec3d(*_zc))
PLACE = _R * trans(float(_p_start[0]), float(_p_start[1]), float(_p_start[2]))

robot_prim = stage.DefinePrim(ROBOT_PATH, "Xform")
robot_prim.GetReferences().AddReference(ROBOT_USDA)
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
print(f"[준비] 로봇 배치 완료 — RigidBody {_n_baked}개")

light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
light.CreateIntensityAttr(800.0)

world.reset()
print("[준비] world.reset() 완료(final_script.py 실행 전)")

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


def check_settle(tag):
    # 🚨 휠마다 **따로** 투영한다 — 로봇 길이(DiscF~DiscR 약 228mm+)가 직선
    # 구간(185mm)보다 길어서, Body 기준 s 하나로 전부 재면 굽은 구간에 걸친
    # 쪽의 반경이 실제와 다르게 나온다(사용자 지적).
    _s_list, _radii = [], []
    for nm in _WHEEL_NAMES:
        p = _wpos(stage.GetPrimAtPath(f"{ROBOT_PATH}/Robot/{nm}_Wheel"))
        s, _ = PATH.project(p)
        _s_list.append(s)
        _radii.append(_radial_r(p, s))
    _fmt = lambda xs: " ".join(f"{r:.1f}" for r in xs)  # noqa: E731
    _fmts = lambda xs: " ".join(f"{r * 1000:.0f}" for r in xs)  # noqa: E731
    print(f"[안착·{tag}] s(mm)  Body[{_fmts(_s_list[0:6])}]  "
          f"DiscF[{_fmts(_s_list[6:9])}]  DiscR[{_fmts(_s_list[9:12])}]")
    print(f"[안착·{tag}] r(mm)  Body[{_fmt(_radii[0:6])}]  "
          f"DiscF[{_fmt(_radii[6:9])}]  DiscR[{_fmt(_radii[9:12])}]")


_tick = 0
for _stop in (240, 480):
    while _tick < _stop:
        world.step(render=True)
        _tick += 1
    check_settle(f"reset전 {_stop / 240.0:.1f}s")

# ── final_script.py 를 module-level 자동실행부(v8_start()/go() 등) 없이 로드 ──
_src = FINAL_SCRIPT.read_text()
_marker = "v8_start()\nplane_check()"
_idx = _src.index(_marker)
_src_defs = _src[:_idx]           # CONFIG + 함수/클래스 정의까지만
_ns = {}
exec(compile(_src_defs, str(FINAL_SCRIPT), "exec"), _ns)

_ns["JUNCTION"] = tuple(float(v) for v in _p_start)   # PIPE 프림 없이 좌표 직접 지정
_ns["v8_start"]()
print("[final_script] v8_start() 완료 — reset() 호출(rear bias 적용)")
_ns["reset"]()

_tick2 = 0
for _stop in (240, 480, 720):
    while _tick2 < _stop:
        world.step(render=True)
        _tick2 += 1
    check_settle(f"reset후 {_stop / 240.0:.1f}s")

print("=" * 78)
print("[대기] go() 는 호출 안 함 — 안착만 확인. 필요하면 Script Editor 에서 "
      "go() 직접 호출해 주행 테스트 이어가면 된다.")
print("=" * 78)

while simulation_app.is_running():
    world.step(render=True)
simulation_app.close()
