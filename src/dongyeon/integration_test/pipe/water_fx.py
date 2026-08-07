"""[Isaac 3.11] --fluid 시각 물 층 — 렌더 전용 (2026-08-06 사용자 요청).

--fluid 는 해석적 힘 두 개(항력·부력)만 있고 물 프림이 하나도 없어 화면에
아무것도 안 보였다. 사용자 지적 그대로 **유체압을 거는 이상 물은 있는 것**이고,
파티클이 없을 뿐이다. 그 물을 표면으로 그린다:

  ① 수체    — 코스를 따라 수위까지 채운 반투명 물덩어리 메시 (정적)
  ② 흐름    — 수면 위 줄무늬가 설계 유속으로 흘러감 (PointInstancer)
  ③ 누수    — 개구 아래로 떨어지는 물줄기. **마개가 착좌하면 멎는다**

🚨 전부 렌더 전용이다 — 콜라이더·강체·파티클 없음, 물리에 영향 0.
   유체력은 여전히 데모의 apply_fluid() 가 담당한다. 진짜 입자 물(실제로
   새는 물)은 --water 다. 헤드리스에서는 만들지 않는다(spark_fx 와 같은 이유).

**재질은 갈아끼울 수 있게 뒀다** (`material_path` 인자). 기본은 로컬
UsdPreviewSurface 반투명이고, vMaterials LIQUIDS(`Water_Blue_Ocean_*` 등)를
받아 오면 그 Material 경로만 넘기면 된다. 문서 「Material Library로 환경에
재질 입히기」의 원칙 그대로 — 재질은 **Mesh** 에 붙이고, PhysicsMaterial 과는
완전히 별개다.

pxr 는 함수 안 지연 import — 3.10 오프라인에서 기하 함수를 시험할 수 있다.
"""

import math

import numpy as np

WATER_RGB = (0.15, 0.45, 0.85)          # water_sim(입자 물) 과 같은 파랑
GLOW_RGB = (0.05, 0.16, 0.35)           # 흐름 줄무늬 전용 — 수면에는 쓰지 말 것
WATER_OPACITY = 0.35                    # 물속 구멍이 비쳐 보이는 최소선(실기)


def elbow_centerline(s_in, in_y, arc_r, out_x, s_out, ds=0.01):
    """이 데모 코스(입구 직관 → LR 곡관 → 출구 직관)의 중심선 표본.

    repair_demo 의 `path_dist_tangent` 와 **같은 기하**를 쓴다:
      직관1  y=in_y, x ∈ [-s_in, 0]        접선 (+1, 0)
      곡관   중심 원점, 각 −90°→0°          접선 (−sin, +cos)
      직관2  x=out_x, y ∈ [0, s_out]       접선 (0, +1)
    반환 (points (N,3), tangents (N,3)). z=0 평면.

    🚨 `np.arange` 로 만들지 말 것 — 끝점이 구간을 넘어 중심선이 관 밖으로
    삐져나간다(실측 7mm, 오프라인 시험이 잡았다). 세 구간을 `linspace` 로
    **끝점을 정확히 맞춰** 만들고, 이음매의 중복점만 하나씩 뺀다.
    """
    n1 = max(2, int(round(s_in / ds)) + 1)
    n2 = max(2, int(round(arc_r * (math.pi / 2) / ds)) + 1)
    n3 = max(2, int(round(s_out / ds)) + 1)
    pts, tans = [], []
    for x in np.linspace(-s_in, 0.0, n1)[:-1]:      # 끝점은 곡관 시작과 같다
        pts.append((x, in_y, 0.0))
        tans.append((1.0, 0.0, 0.0))
    for a in np.linspace(-math.pi / 2, 0.0, n2):    # 양 끝 포함
        pts.append((arc_r * math.cos(a), arc_r * math.sin(a), 0.0))
        tans.append((-math.sin(a), math.cos(a), 0.0))
    for y in np.linspace(0.0, s_out, n3)[1:]:       # 시작점은 곡관 끝과 같다
        pts.append((out_x, y, 0.0))
        tans.append((0.0, 1.0, 0.0))
    return np.array(pts, float), np.array(tans, float)


def half_width(level, radius):
    """수위 level 에서 수면의 반폭 √(r²−level²) [m]."""
    return math.sqrt(max(radius * radius - level * level, 1e-12))


def surface_ribbon(pts, tans, level, radius):
    """**수면만** — 중심선을 따라 z=level 에 깔린 띠 (points, counts, indices).

    🚨 **속이 찬 수체를 만들면 안 된다** (2026-08-06 실기로 확정). 부피로
    만들었더니 두 가지가 한꺼번에 깨졌다:
      ① 로봇 바퀴가 반경 40mm 라 **물덩어리 속에 잠겨** 수면 높이에서 잘린
         것처럼 렌더됐다.
      ② 시연 때 시점을 관 안으로 넣으면 **카메라가 덩어리 안으로 들어가**
         화면 전체가 파랗게 떴다("바다처럼 보인다").
    `doubleSided=False` 로 안쪽 면을 걸러내려 했지만 RTX 실시간 렌더가 그
    힌트를 따르지 않아 방어가 안 먹었다. → 부피를 버리고 **면 하나**만 둔다.
    감쌀 부피가 없으니 로봇을 침범할 수도, 카메라를 가둘 수도 없다.
    (로봇 bbox 를 도려내는 방안도 있으나 로봇이 매 스텝 움직여 물 메시를
     매번 다시 구워야 한다 — 같은 결과를 훨씬 비싸게 얻는 셈이다.)

    관 안에서 보면 수면이 앞으로 뻗어 "물이 차 있다" 가 그대로 읽힌다.
    코스는 XY 평면(관 축 수평) 전제 — 접선 z 성분은 버린다.
    """
    pts = np.asarray(pts, float)
    tans = np.asarray(tans, float)
    n = len(pts)
    w = half_width(level, radius)

    h = np.stack([-tans[:, 1], tans[:, 0]], axis=1)
    h /= np.maximum(np.linalg.norm(h, axis=1, keepdims=True), 1e-12)
    points = np.empty((n * 2, 3))
    points[0::2, 0] = pts[:, 0] - h[:, 0] * w
    points[0::2, 1] = pts[:, 1] - h[:, 1] * w
    points[1::2, 0] = pts[:, 0] + h[:, 0] * w
    points[1::2, 1] = pts[:, 1] + h[:, 1] * w
    points[:, 2] = level

    counts, idx = [], []
    for i in range(n - 1):
        a0, b0, a1, b1 = 2 * i, 2 * i + 1, 2 * i + 2, 2 * i + 3
        counts.append(4)
        idx += [a0, a1, b1, b0]        # 법선 +z (위에서 보인다)
    return points, counts, idx


def advect_s(s, dt, v, total):
    """줄무늬 호길이 전진 + 순환. v 음수 = s 감소 방향(출구→입구 역류)."""
    return (s + v * dt) % total


def drop_z(t, z0, v0, g=9.81):
    """누수 물방울 z(t) — 토리첼리 초속 v0 로 낙하."""
    return z0 - v0 * t - 0.5 * g * t * t


def fall_time(dist, v0, g=9.81):
    """dist 만큼 떨어지는 시간(물방울 순환 주기)."""
    return (-v0 + math.sqrt(v0 * v0 + 2.0 * g * dist)) / g


def torricelli(level, z_hole, g=9.81):
    """수면 level 아래 z_hole 구멍의 유출 속도 √(2gh) [m/s]."""
    return math.sqrt(max(0.0, 2.0 * g * (level - z_hole)))


class WaterFX:
    """--fluid 시각 물 층 하나. 생성은 GUI 에서만, step() 은 매 물리 스텝."""

    def __init__(self, stage, pts, tans, *, level, radius, flow_v,
                 hole_xyz, v_out, parent="/World/WaterFX",
                 material_path=None, n_streaks=36, n_drops=24,
                 fall_m=0.45, seed=7):
        from pxr import Gf, Sdf, UsdGeom, UsdShade

        self._UsdGeom = UsdGeom
        self.pts, self.tans = np.asarray(pts, float), np.asarray(tans, float)
        # 표본 간 호길이를 누적해 s 표를 만든다(줄무늬를 흘려보낼 좌표계).
        seg = np.linalg.norm(np.diff(self.pts, axis=0), axis=1)
        self.tab_s = np.concatenate([[0.0], np.cumsum(seg)])
        self.total = float(self.tab_s[-1])
        self.level, self.flow_v = level, flow_v
        self.hole = np.asarray(hole_xyz, float)
        self.v_out, self.fall_m = v_out, fall_m
        self._leak_shown = True
        rng = np.random.default_rng(seed)

        self._root = UsdGeom.Xform.Define(stage, parent).GetPrim()

        def _mat(path, rgb, glow, opa):
            m = UsdShade.Material.Define(stage, path)
            s = UsdShade.Shader.Define(stage, path + "/Shader")
            s.CreateIdAttr("UsdPreviewSurface")
            s.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
                Gf.Vec3f(*rgb))
            s.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
                Gf.Vec3f(*glow))
            s.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opa)
            s.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.15)
            m.CreateSurfaceOutput().ConnectToSource(
                s.ConnectableAPI(), "surface")
            return m

        # 🔑 외부 재질(vMaterials LIQUIDS 등)을 넘기면 그것을 쓰고, 없으면
        #    로컬 반투명으로 간다. 원격 라이브러리가 없어도 물은 보인다.
        _m_body = (UsdShade.Material.Get(stage, material_path)
                   if material_path else None)
        self.material_used = material_path if (_m_body and _m_body.GetPrim()
                                               .IsValid()) else None
        if self.material_used is None:
            # 🚨 **수면에 자체발광을 주면 안 된다** (2026-08-06 실기).
            #    반투명 + emissive 가 블룸으로 번지고, 렌더 해상도 320×180 을
            #    DLSS 가 업스케일하면서 그 번짐이 화면 전체로 퍼진다 — 관
            #    밖에서도 물이 보이고, 각도가 맞으면 **온 화면이 파래진다**
            #    (씬에 바닥 프림이 아예 없는데 "ground 가 물이 됐다" 로
            #     보인 것이 이것이다). 물은 스스로 빛나지 않는다.
            _m_body = _mat(parent + "/BodyMat", WATER_RGB, (0.0, 0.0, 0.0),
                           WATER_OPACITY)
        _m_flow = _mat(parent + "/FlowMat", (0.45, 0.70, 1.0),
                       (0.18, 0.38, 0.75), 0.8)

        # ── ① 수면 ─────────────────────────────────────────────────
        p, c, ix = surface_ribbon(self.pts, self.tans, level, radius)
        body = UsdGeom.Mesh.Define(stage, parent + "/Surface")
        body.CreatePointsAttr([Gf.Vec3f(*map(float, q)) for q in p])
        body.CreateFaceVertexCountsAttr(c)
        body.CreateFaceVertexIndicesAttr(ix)
        body.CreateExtentAttr([Gf.Vec3f(*map(float, p.min(0))),
                               Gf.Vec3f(*map(float, p.max(0)))])
        body.CreateSubdivisionSchemeAttr("none")
        # 면 하나뿐이라 양면으로 둔다 — 아래에서 올려다봐도 수면이 보여야
        # 하고, 부피가 없으니 카메라를 가둘 위험이 없다.
        body.CreateDoubleSidedAttr(True)
        UsdShade.MaterialBindingAPI.Apply(body.GetPrim()).Bind(_m_body)
        self.n_body_faces = len(c)

        # ── ② 흐름 줄무늬 ──────────────────────────────────────────
        half_w = half_width(level, radius)
        self.s_streak = rng.uniform(0.0, self.total, n_streaks)
        self.off_streak = rng.uniform(-0.6, 0.6, n_streaks) * half_w
        self.streaks = self._instancer(stage, parent + "/Flow", n_streaks,
                                       _m_flow, axis="X", radius=0.0015,
                                       height=0.022)

        # ── ③ 누수 물줄기 ──────────────────────────────────────────
        self.t_cycle = fall_time(fall_m, v_out)
        self.t_drop = rng.uniform(0.0, self.t_cycle, n_drops)
        self.jit_drop = rng.uniform(-0.004, 0.004, (n_drops, 2))
        self.drops = self._instancer(stage, parent + "/Leak", n_drops,
                                     _m_flow, axis="Z", radius=0.002,
                                     height=0.014)
        self.step(0.0, leaking=True)

    def _instancer(self, stage, path, n, mat, *, axis, radius, height):
        """인스턴서 하나 — 프로토타입 캡슐 + 위치·회전 배열.

        🚨 **회전을 반드시 항등으로 초기화할 것** (2026-08-06 실기로 확정).
        `Vt.QuathArray(n)` 은 **0 으로 채운 쿼터니언**(0,0,0,0)을 준다. 크기가
        0 이라 회전 행렬이 무너지고, 인스턴스가 화면 전체로 퍼져 **배경이
        통째로 파랗게 뜬다.** 흐름 줄무늬는 `step()` 이 매 스텝 회전을 다시
        써서 멀쩡했고, **누수 물방울만 위치만 쓰고 회전을 안 써서** 그 상태로
        남아 있었다 — 사용자가 Stage 에서 `Leak` 만 껐다 켜며 잡아냈다.

        🚨 extent 는 **축을 따라** 잡아야 한다. 전에는 축과 무관하게 X 성분에만
        길이를 더해, axis="Z" 인 물방울의 경계상자가 틀렸다.
        """
        from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt
        inst = UsdGeom.PointInstancer.Define(stage, path)
        proto = UsdGeom.Capsule.Define(stage, path + "/proto")
        proto.CreateAxisAttr(axis)
        proto.CreateRadiusAttr(radius)
        proto.CreateHeightAttr(height)
        _h = [radius, radius, radius]
        _h["XYZ".index(axis)] += height / 2.0
        proto.CreateExtentAttr([Gf.Vec3f(-_h[0], -_h[1], -_h[2]),
                                Gf.Vec3f(_h[0], _h[1], _h[2])])
        UsdShade.MaterialBindingAPI.Apply(proto.GetPrim()).Bind(mat)
        inst.CreatePrototypesRel().SetTargets([Sdf.Path(path + "/proto")])
        inst.CreateProtoIndicesAttr(Vt.IntArray(n, 0))
        inst.CreatePositionsAttr(Vt.Vec3fArray(n))
        inst.CreateOrientationsAttr(
            Vt.QuathArray(n, Gf.Quath(1.0, 0.0, 0.0, 0.0)))   # 항등
        return inst

    def set_visible(self, on):
        """물 층 전체를 보이거나 감춘다 (루트 Xform 하나로).

        🔑 **로봇 카메라가 촬영하는 순간에만 감춘다.** 결함은 관 바닥이고
        수위는 그 위라 **결함이 물에 잠겨 있다** — 카메라는 수면 너머로 보게
        되고, 그 탓에 구멍이 충분히 어둡게 안 잡혀 검출이 늦어진다(실측:
        정지 지점이 결함 81mm 앞 → **33mm 앞**으로 밀려 촬영 거리가 무너졌다).
        색으로 물만 골라내는 것은 원리적으로 안 된다 — 결함도 같이 파랗다.
        → 판정 프레임에서만 물을 빼고, 사람이 보는 화면에는 그대로 둔다.
        """
        img = self._UsdGeom.Imageable(self._root)
        (img.MakeVisible if on else img.MakeInvisible)()

    def _interp(self, s):
        x = np.interp(s, self.tab_s, self.pts[:, 0])
        y = np.interp(s, self.tab_s, self.pts[:, 1])
        tx = np.interp(s, self.tab_s, self.tans[:, 0])
        ty = np.interp(s, self.tab_s, self.tans[:, 1])
        return x, y, tx, ty

    def step(self, dt, leaking):
        from pxr import Gf, Vt
        # 흐름 줄무늬 — 유속 방향(음수 = 출구→입구)으로 흘려보낸다
        self.s_streak = advect_s(self.s_streak, dt, self.flow_v, self.total)
        x, y, tx, ty = self._interp(self.s_streak)
        nrm = np.maximum(np.hypot(tx, ty), 1e-12)
        tx, ty = tx / nrm, ty / nrm
        px = x + (-ty) * self.off_streak
        py = y + tx * self.off_streak
        self.streaks.GetPositionsAttr().Set(Vt.Vec3fArray(
            [Gf.Vec3f(float(a), float(b), float(self.level) + 0.002)
             for a, b in zip(px, py)]))
        yaw2 = np.arctan2(ty, tx) * 0.5
        self.streaks.GetOrientationsAttr().Set(Vt.QuathArray(
            [Gf.Quath(float(math.cos(w)), 0.0, 0.0, float(math.sin(w)))
             for w in yaw2]))

        # 누수 물줄기 — 마개가 착좌하면 감춘다
        if leaking != self._leak_shown:
            img = self._UsdGeom.Imageable(self.drops.GetPrim())
            (img.MakeVisible if leaking else img.MakeInvisible)()
            self._leak_shown = leaking
        if leaking:
            self.t_drop = (self.t_drop + dt) % self.t_cycle
            z = drop_z(self.t_drop, self.hole[2] - 0.004, self.v_out)
            self.drops.GetPositionsAttr().Set(Vt.Vec3fArray(
                [Gf.Vec3f(float(self.hole[0] + jx),
                          float(self.hole[1] + jy), float(zz))
                 for (jx, jy), zz in zip(self.jit_drop, z)]))
