"""[오프라인] 실전 맵(restroom_pipe150_final.usd) 기하 결함을 고쳐 사본을 만든다.

🚨 **원본은 절대 덮어쓰지 않는다.** Isaac 에서 이 USD 에 저장했다가 형상 없는
   껍데기(3.4KB)가 원본을 덮어써 실제로 한 번 날아간 적이 있다. 이 스크립트는
   원본을 읽기만 하고 `<이름>_fixed.usd` 를 새로 쓴다.

Isaac 이 필요 없다. usd-core 만 있으면 된다:
    python3 -m pip install --user --no-deps usd-core
    python3 tools/fix_map.py [원본.usd] [출력.usd]

━━ 왜 고치는가 (전부 실측, 추측 아님) ━━
로봇이 곡관을 못 넘던 것은 마찰이 아니라 **맵 형상**이었다. 같은 R150·ø100
조건인 `pipe/pipe_elbow_lr150.usda`(벨로우즈가 실제로 완주한 코스)와 나란히
재 보면 조건이 같지 않다.

  ① **같은 배관이 두 번 모델링돼 있고 사본의 곡관이 각져서 더 좁다.**
     CAD 가 배관을 `Sweep*` 표면으로도 만들고 건물 솔리드 `PartBody*` 안에도
     같은 관을 뚫어 놨다. floor2 곡관2(s 540~660) 실측:
         Sweep2     48.8 ~ 50.1mm   ← 기준 배관(48.75~50.00)과 동일
         PartBody1  45.3 ~ 50.5mm   ← 4.7mm 더 좁다
     둘 다 콜라이더면 로봇은 **좁은 쪽**에 걸린다. s=560 에서 끼이던 원인이다.
     정점 자체는 정확한데(반경 50.00/55.00) 곡관의 **호 분할이 성겨서**
     (5.6°/스텝, 설계 요구 2.5°) 면이 현을 그어 안쪽을 파고든다.
     → **사본을 지우지 않는다.** 한 번 지워 봤다가 라이저에 구멍이 생겨
       로봇이 s=120 에서 관을 뚫고 나갔다(휠 반경 62mm, 중심선 이탈 19mm).
       솔리드 사본은 표면 배관(두께 0)의 뒷받침 역할을 한다.
     → 대신 **파고드는 면만 쪼개서 참 반경으로 투영한다.** 그러면 사본이
       Sweep 보다 넓어져 좁은 쪽이 아니게 되고, 뒷받침도 유지된다.

  ② **floor2 의 y=1400 가지가 ø90 이다(내반경 45).**
     s 960~1620 구간은 `PartBody2` 하나뿐이고 44.7~45.1mm 다. 여기엔 Sweep 이
     없으니 중복이 아니라 **진짜로 관이 작다.** 정점 분포도 내 45 / 외 50 으로
     벽 5mm 인 ø90 관이다(1259 + 1196개). 나머지 배관이 전부 DN100 인데 이
     가지만 다르다 — 메모리에 "관R 45 짜리 곡관 2 + 직관 1 이 남아 있다" 로
     적혀 있던 그 잔재다. 로봇은 여기서 피스톤 압축 한계에 걸려 못 지난다.
     → 중심선 기준 반경을 **+5mm** 밀어 내 45/50 → 50/55(DN100) 로 만든다.

  ③ 표면 배관(`Sweep*`)은 두께 없는 한 겹이라 `doubleSided` 를 켜 둔다.
     PartBody 사본을 지우고 나면 관 안쪽을 이 면이 혼자 감당하기 때문이다.

floor1 은 배관이 전부 49.x~50.0 으로 깨끗하다(①만 해당). s 600·2280 의 이상치는
T 분기라 정상이다.
"""

import sys
from pathlib import Path as _P

import numpy as np
from pxr import Gf, Usd, UsdGeom

HERE = _P(__file__).resolve().parent
sys.path.insert(0, str(HERE))

BEND_R = 150.0
PROTO = "/restroom_pipe150_final/Prototypes"
# 로봇이 실제로 도는 경로(코너 = 직선의 교점, 필렛 R150 자동). map mm, **월드**.
FLOOR_CORNERS = {
    "floor2": [(330, 850, 85), (330, 850, -250), (680, 850, -250),
               (680, 1400, -250), (1200, 1400, -250), (1200, 600, -250),
               (1500, 600, -250)],
    "floor1": [(330, 850, -2405.23), (330, 850, -2740.23),
               (730, 850, -2740.23), (730, 100, -2740.23),
               (1300, 100, -2740.23), (1300, 600, -2740.23),
               (1500, 600, -2740.23)],
}
PIPE_NAMES = ("Sweep", "Trim", "Join")     # 이름만으로 배관인 것 (메모리 규칙)
BAND = (30.0, 80.0)        # 중심선에서 이 거리 안이면 '배관 면'
GROW_MM = 5.0              # ø90 → ø100
GROW_BAND = (38.0, 60.0)
# 진단 손잡이 — 두 수정을 따로 껐다 켜서 원인을 가른다.
import os as _os
DO_GROW = _os.environ.get("FIX_GROW", "1") != "0"
DO_SMOOTH = _os.environ.get("FIX_SMOOTH", "1") != "0"


class Path:
    """코너 + 필렛 R → 직선/원호 중심선. real_map_demo 의 CenterLine 과 같다."""

    def __init__(self, corners, R=BEND_R):
        C = [np.asarray(c, float) for c in corners]
        self.segs, cur = [], C[0]
        for i in range(1, len(C) - 1):
            B = C[i]
            t1 = B - C[i - 1]; t1 /= np.linalg.norm(t1)
            t2 = C[i + 1] - B; t2 /= np.linalg.norm(t2)
            pin, pout = B - t1 * R, B + t2 * R
            ctr = pin + (pout - B)
            L = float(np.linalg.norm(pin - cur))
            if L > 1e-9:
                self.segs.append(("line", cur, t1, L))
            ang = float(np.arccos(np.clip(np.dot(t1, t2), -1.0, 1.0)))
            self.segs.append(("arc", ctr, (pin - ctr) / R, t1, R, R * ang))
            cur = pout
        L = float(np.linalg.norm(C[-1] - cur))
        self.segs.append(("line", cur, (C[-1] - cur) / L, L))
        self.cum = np.cumsum([s[-1] for s in self.segs])
        self.total = float(self.cum[-1])
        self.tab_s = np.arange(0.0, self.total + 1e-9, 2.0)
        self.tab_p = np.array([self.pt(x)[0] for x in self.tab_s])

    def pt(self, s):
        s = min(max(float(s), 0.0), self.total)
        i = min(int(np.searchsorted(self.cum, s, side="right")),
                len(self.segs) - 1)
        u = s - (self.cum[i - 1] if i else 0.0)
        sg = self.segs[i]
        if sg[0] == "line":
            return sg[1] + sg[2] * u, sg[2]
        _, ctr, e1, t1, R, _ = sg
        a = u / R
        p = ctr + R * (np.cos(a) * e1 + np.sin(a) * t1)
        t = -np.sin(a) * e1 + np.cos(a) * t1
        return p, t / np.linalg.norm(t)

    def dist_s(self, pts, chunk=400):
        """→ (중심선까지 거리, 진행거리 s). 표로 벡터 계산."""
        d_out = np.empty(len(pts)); s_out = np.empty(len(pts))
        for i in range(0, len(pts), chunk):
            q = pts[i:i + chunk]
            d = np.linalg.norm(q[:, None, :] - self.tab_p[None, :, :], axis=2)
            k = np.argmin(d, axis=1)
            d_out[i:i + chunk] = d[np.arange(len(q)), k]
            s_out[i:i + chunk] = self.tab_s[k]
        return d_out, s_out

    def radial_of(self, pts, ss):
        """각 점의 반경 방향 단위벡터(중심선에서 바깥)."""
        out = np.empty((len(pts), 3))
        for i, (p, s) in enumerate(zip(pts, ss)):
            c, t = self.pt(s)
            d = p - c
            d = d - t * np.dot(d, t)
            n = np.linalg.norm(d)
            out[i] = d / n if n > 1e-9 else np.array([0.0, 0.0, 1.0])
        return out


def mesh_prims(stage, floor):
    """{짧은이름: (Mesh prim, 인스턴스 오프셋 벡터)}"""
    inst = stage.GetPrimAtPath(f"/restroom_pipe150_final/"
                               f"restroom_pipe150_final/{floor}")
    off = np.zeros(3)
    if inst and inst.IsValid():
        M = np.array(UsdGeom.XformCache().GetLocalToWorldTransform(inst),
                     dtype=np.float64)
        off = M[3, :3]
    out = {}
    root = stage.GetPrimAtPath(f"{PROTO}/{floor}")
    if not root.IsValid():
        raise SystemExit(f"[중단] {PROTO}/{floor} 이 없다 — 맵 구조가 다르다")
    # 🚨 `GetChildren()` 은 프로토타입 하위를 안 돌려준다(기본 술어에서 빠진다).
    #    실측: GetChildren() → [] / GetAllChildren() → 메시 6개.
    for body in root.GetAllChildren():
        for m in body.GetAllChildren():
            if m.GetTypeName() == "Mesh":
                out[body.GetName().replace("tn__", "")] = (m, off)
    return out


def get_mesh(prim):
    m = UsdGeom.Mesh(prim)
    pts = np.array(m.GetPointsAttr().Get(), dtype=np.float64)
    idx = np.array(m.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
    return m, pts, idx


def set_faces(m, idx):
    m.GetFaceVertexIndicesAttr().Set(idx.reshape(-1).tolist())
    m.GetFaceVertexCountsAttr().Set([3] * len(idx))


def round_pipe(prim, off, P, sag_tol=0.25, max_pass=6):
    """관벽 면이 현을 그어 파고드는 것을 편다 → 쪼갠 면 수.

    🚨 정점은 정확한데(반경 50.00 / 55.00) **호 분할이 성겨서**(이 CAD 는
       5.6°/스텝, 설계 요구는 2.5°) 면이 두 정점 사이를 직선으로 잇는다.
       그 현이 곡관 안쪽으로 최대 4.7mm 파고들어 로봇이 걸린다.
    → 파고드는 면만 **최장변 이등분**으로 쪼개고, 새로 생긴 중점을 중심선
      기준 **참 반경**으로 밀어 낸다. 중점을 그냥 두면 현이 그대로 남는다.
      (1→4 중점분할은 쓰지 않는다 — 원본이 슬리버라 삼각형이 폭발한다.)
    """
    m, pts, idx = get_mesh(prim)
    grew = 0
    for _ in range(max_pass):
        w = pts + off
        d, s = P.dist_s(w[idx].reshape(-1, 3))
        d = d.reshape(-1, 3); s = s.reshape(-1, 3)
        # 🚨 **진짜 관벽 면만 고른다.** `PartBody1` 은 벽·바닥·천장이 한 메시라
        #    밴드를 우연히 가로지르는 건물 면이 섞인다. 그걸 "배관" 으로 보고
        #    참 반경으로 밀어 내면 건물이 뒤틀리고, 그 위에서 물리가 NaN 으로
        #    발산한다(실측: seg1 이 (2.2e7, 1.6e7, 1.0e6)mm 로 날아갔다).
        # → 세 정점이 모두 관벽 반경대(46~58mm)에 있고 **서로 2mm 안**일 때만
        #   관벽으로 본다. 관을 가로지르는 벽면은 반경이 제각각이라 걸러진다.
        band = (((d > BAND[0]) & (d < BAND[1])).all(axis=1)
                & (d >= 46.0).all(axis=1) & (d <= 58.0).all(axis=1)
                & ((d.max(axis=1) - d.min(axis=1)) < 2.0))
        if not band.any():
            break
        # 면 중심이 참 반경보다 얼마나 안쪽인가 = 현이 파고든 깊이
        cen = w[idx].mean(axis=1)
        dc, sc = P.dist_s(cen)
        target = d.mean(axis=1)          # 세 정점의 반경 (거의 같다)
        sag = target - dc
        sel = band & (sag > sag_tol)
        if not sel.any():
            break
        elen = np.stack([np.linalg.norm(w[idx[:, 1]] - w[idx[:, 0]], axis=1),
                         np.linalg.norm(w[idx[:, 2]] - w[idx[:, 1]], axis=1),
                         np.linalg.norm(w[idx[:, 0]] - w[idx[:, 2]], axis=1)],
                        axis=1)
        which = np.argmax(elen, axis=1)
        pts_list = list(pts); mid = {}

        def mp(a, b):
            k = (min(a, b), max(a, b))
            if k not in mid:
                q = (pts[a] + pts[b]) * 0.5
                # 참 반경으로 밀어 낸다 (a·b 정점 반경의 평균이 목표)
                dq, sq = P.dist_s(np.array([q + off]))
                c, t = P.pt(float(sq[0]))
                v = (q + off) - c
                v = v - t * np.dot(v, t)
                nv = np.linalg.norm(v)
                # 🚨 **v 가 아주 작을 때 방향이 실수 오차에 휘둘린다** — nv 가
                #    1e-9 근처면 정규화(v/nv)가 사실상 잡음 방향을 증폭해
                #    엉뚱한 쪽으로 정점을 튕겨 낸다(관찰: PhysX 콜리전 쿠킹이
                #    이렇게 튄 정점 근처에서 위치가 수백만 mm 로 발산했다 —
                #    후진 주행 중 코너에서만 재현). 문턱을 훨씬 넉넉히 잡고,
                #    보정량 자체도 물리적으로 말이 되는 범위로 자른다(실측
                #    최대 파고듦이 4.7mm 였으니 20mm 를 넘을 이유가 없다).
                if nv > 1e-3:
                    da, _ = P.dist_s(np.array([pts[a] + off]))
                    db, _ = P.dist_s(np.array([pts[b] + off]))
                    tgt = 0.5 * (float(da[0]) + float(db[0]))
                    disp = max(-20.0, min(20.0, tgt - nv))
                    q = q + (v / nv) * disp
                mid[k] = len(pts_list)
                pts_list.append(q)
            return mid[k]

        new = []
        for tri, sl, wch in zip(idx, sel, which):
            a, b, c = int(tri[0]), int(tri[1]), int(tri[2])
            if not sl:
                new.append((a, b, c)); continue
            if wch == 1:
                a, b, c = b, c, a
            elif wch == 2:
                a, b, c = c, a, b
            mab = mp(a, b)
            new += [(a, mab, c), (mab, b, c)]
        pts = np.array(pts_list); idx = np.array(new)
        grew += int(sel.sum())
    if grew:
        m.GetPointsAttr().Set([Gf.Vec3f(float(a), float(b), float(c))
                               for a, b, c in pts])
        set_faces(m, idx)
        m.GetExtentAttr().Set([Gf.Vec3f(*[float(v) for v in pts.min(0)]),
                               Gf.Vec3f(*[float(v) for v in pts.max(0)])])
    return grew


def main(src, dst):
    stage = Usd.Stage.Open(src)
    if stage is None:
        raise SystemExit(f"[중단] 못 열었다: {src}")
    print(f"[맵수정] 원본 {src}")
    print(f"         → 사본 {dst}   (원본은 읽기만 한다)")
    total_dropped = total_grown = 0

    for floor, corners in FLOOR_CORNERS.items():
        P = Path(corners)
        M = mesh_prims(stage, floor)
        print(f"\n── {floor}  중심선 {P.total:.0f}mm, 메시 {len(M)}개 "
              f"(인스턴스 오프셋 {M[list(M)[0]][1].round(2)}) ──")

        # ── ② ø90 → ø100 : Sweep 이 없는 구간의 좁은 관을 넓힌다 ──
        #    (floor2 의 PartBody2 뿐이지만 규칙으로 판정한다)
        for name, (prim, off) in M.items():
            if any(k in name for k in PIPE_NAMES) or not DO_GROW:
                continue
            m, pts, idx = get_mesh(prim)
            w = pts + off
            d, s = P.dist_s(w)
            band = (d > GROW_BAND[0]) & (d < GROW_BAND[1])
            if band.sum() < 200:
                continue
            # 이 메시의 배관이 ø90 인가 — 안쪽 벽 반경의 최빈값으로 본다
            inner = d[band & (d < 48.0)]
            if inner.size < 100 or float(np.median(inner)) > 47.0:
                continue
            rad = P.radial_of(w[band], s[band])
            pts[band] = pts[band] + rad * GROW_MM
            m.GetPointsAttr().Set([Gf.Vec3f(float(a), float(b), float(c))
                                   for a, b, c in pts])
            m.GetExtentAttr().Set([Gf.Vec3f(*[float(v) for v in pts.min(0)]),
                                   Gf.Vec3f(*[float(v) for v in pts.max(0)])])
            total_grown += int(band.sum())
            print(f"   ø90→ø100  {name}: 정점 {int(band.sum()):,}개를 반경 "
                  f"+{GROW_MM:.0f}mm (안쪽벽 중앙값 "
                  f"{float(np.median(inner)):.1f} → "
                  f"{float(np.median(inner)) + GROW_MM:.1f}mm)")

        # ③ 표면 배관은 양면으로 렌더
        for name, (prim, off) in M.items():
            if any(k in name for k in PIPE_NAMES):
                UsdGeom.Mesh(prim).CreateDoubleSidedAttr(True)

        # ── ① 각진 곡관 펴기 : 파고드는 면을 쪼개 참 반경으로 투영 ──
        for name, (prim, off) in M.items():
            n = round_pipe(prim, off, P) if DO_SMOOTH else 0
            if n:
                total_dropped += n
                print(f"   곡관 펴기  {name}: 면 {n:,}개 세분·투영")

    stage.GetRootLayer().Export(dst)
    print(f"\n[맵수정] 완료 — 곡관 펴기 {total_dropped:,}면, "
          f"ø90 정점 {total_grown:,}개 확장")
    print(f"         저장: {dst}")
    print("         확인: tools/check_map.py 로 내반경 프로파일을 다시 잰다")


if __name__ == "__main__":
    # 맵은 `src/son/maps/` 안에 있다 (2026-08-06 — 레포 밖 참조 제거)
    _src = (sys.argv[1] if len(sys.argv) > 1
            else str(_P(__file__).resolve().parent.parent
                     / "maps" / "restroom_pipe150_final.usd"))
    _dst = (sys.argv[2] if len(sys.argv) > 2
            else _src.replace(".usd", "_fixed.usd"))
    if _P(_dst).resolve() == _P(_src).resolve():
        raise SystemExit("[중단] 출력이 원본과 같다 — 원본을 덮어쓰지 않는다")
    main(_src, _dst)
