"""[자산생성] 배관 점검 로봇 — 링크별 STL 생성 (설계확정본 v3 기준)

기존 build_robot_stl.py 는 전체를 단일 메시로 합쳐 출력한다. articulation 을
만들려면 링크마다 별도 파일이어야 하므로 부품 단위로 나누어 내보낸다.

출력 (parts/):
  body_rear.stl  body_front.stl  bellows.stl  arm.stl  wheel.stl
  pipe_straight.stl  pipe_elbow_sr.stl

기준 자세는 관절 굽힘 0도, 암 각도 44.4도(DN100 접촉)이다.
실행:  python3 build_robot_parts.py
"""

import argparse
import json
from pathlib import Path

import numpy as np
import trimesh
from trimesh.creation import annulus, box, cylinder
from trimesh.transformations import rotation_matrix, translation_matrix

P = dict(
    seg_len=62.0,
    joint_len=26.0,
    body_w=50.0,
    body_h=40.0,
    bellows_body=36.0,
    bellows_crest=40.0,
    bellows_folds=4,
    pivot_r=12.0,
    arm_len=40.0,
    arm_angle=44.4,
    arm_thick=6.0,
    arm_width=10.0,
    wheel_dia=20.0,
    wheel_width=15.0,
    # 설계확정본 "휠 Ø20 x 폭 15mm, **크라운 반경 50mm**". 관 내반경과 같은
    # 값이라 트레드가 관벽 곡률에 그대로 맞물린다. 이 값을 빠뜨려 2026-08-04
    # 까지 휠이 원통으로 만들어졌고, 트레드 양끝 2점으로만 닿았다.
    wheel_crown_r=50.0,
    stagger=7.0,          # 10mm 였으나 링(x58)과 암 앞끝(피벗+3) 여유가 1mm 뿐이라 축소.
                          # 휠 접촉점 퍼짐도 줄어 곡관 필요 스트로크가 함께 감소한다
    torch_rot_r=9.0,
    ring_r_in=26.0,       # 본체 최대 반경 25 + 여유 1
    ring_r_out=30.0,
    ring_w=10.0,          # 링 폭. 암이 없는 선단 구간(21mm)에 들어간다
    ring_x=63.0,          # 링 중심 축 위치
    # 링이 이미 r=30 까지 실어 나르므로 직동 스트로크가 짧아도 된다.
    # 설계의 35mm 는 토치를 축 근처(r=20)에 달았을 때 값이다.
    # 수납 시 r<=42 여야 곡관을 지난다(곡관에서 본체·암 최대 42.0mm 실측).
    torch_rod_len=5.0,    # 링 외곽 30 -> 35
    torch_tip_len=5.0,    # 35 -> 40 (수납),  40 -> 48 (신장)
    torch_stroke=8.0,     # 수납 40mm. 곡관에서 본체·암이 41.7mm 까지 가므로
                          #  토치가 그보다 안쪽에 있어야 걸리지 않는다
    torch_arm_w=7.0,
    cam_dia=16.0,
    cam_depth=4.0,
    front_arms=[0.0, 120.0, 240.0],
    rear_arms=[60.0, 180.0, 300.0],
    sections=48,
)

PIPE_ID = 100.0
DEFECT = dict(
    patch_len_mm=30.0,
    patch_ang_deg=34.0,
    crack_len_mm=18.0,
    crack_width_mm=1.6,
    crack_depth_mm=2.2,
    hole_dia_mm=6.0,
    bead_proud_mm=0.8,
    bead_margin_mm=1.6,
)
PIPE_OD = 114.0
PIPE_STRAIGHT_LEN = 600.0
ELBOW_R = 100.0
ELBOW_SWEEP = 90.0
ELBOW_STEPS = 72

X = np.array([1.0, 0.0, 0.0])
Y = np.array([0.0, 1.0, 0.0])


def T(x=0.0, y=0.0, z=0.0):
    return translation_matrix([x, y, z])


def R(deg, axis, point=(0, 0, 0)):
    return rotation_matrix(np.radians(deg), axis, point)


def elliptic_cylinder(w, h, length, sections):
    m = cylinder(radius=1.0, height=length, sections=sections)
    m.apply_scale([w / 2.0, h / 2.0, 1.0])
    m.apply_transform(R(90, Y))
    return m


def rounded_body(w, h, length, sections):
    core = elliptic_cylinder(w, h, length - 6.0, sections)
    caps = []
    for sgn in (-1, 1):
        c = elliptic_cylinder(w - 5.0, h - 5.0, 6.0, sections)
        c.apply_transform(T(x=sgn * (length - 6.0) / 2.0 + sgn * 3.0))
        caps.append(c)
    return trimesh.util.concatenate([core] + caps)


def bellows(inner_d, crest_d, length, folds, sections):
    parts = []
    tube = cylinder(radius=inner_d / 2.0, height=length, sections=sections)
    tube.apply_transform(R(90, Y))
    parts.append(tube)
    pitch = length / folds
    for i in range(folds):
        x = -length / 2.0 + pitch * (i + 0.5)
        ring = annulus(r_min=inner_d / 2.0 - 0.5, r_max=crest_d / 2.0,
                       height=pitch * 0.45, sections=sections)
        ring.apply_transform(R(90, Y))
        ring.apply_transform(T(x=x))
        parts.append(ring)
    return trimesh.util.concatenate(parts)


def camera(x_pos, facing, p):
    lens = cylinder(radius=p['cam_dia'] / 2.0, height=p['cam_depth'],
                    sections=p['sections'])
    lens.apply_transform(R(90, Y))
    lens.apply_transform(T(x=x_pos + facing * p['cam_depth'] / 2.0))
    ring = annulus(r_min=p['cam_dia'] / 2.0, r_max=p['cam_dia'] / 2.0 + 3.0,
                   height=2.0, sections=p['sections'])
    ring.apply_transform(R(90, Y))
    ring.apply_transform(T(x=x_pos + facing * 1.0))
    return trimesh.util.concatenate([lens, ring])


def arm_mesh(p):
    psi = np.radians(p['arm_angle'])
    dx = p['arm_len'] * np.cos(psi)
    dr = p['arm_len'] * np.sin(psi)
    m = box(extents=[p['arm_len'], p['arm_width'], p['arm_thick']])
    tilt = np.degrees(np.arctan2(dr, dx))
    m.apply_transform(R(tilt, Y))
    m.apply_transform(T(x=-dx / 2.0, z=dr / 2.0))
    hub = cylinder(radius=p['arm_thick'] / 2.0, height=p['arm_width'],
                   sections=24)
    hub.apply_transform(R(90, X))
    m = trimesh.util.concatenate([m, hub])
    return m, dx, dr


def revolve_profile(ys, radii, sections):
    """(y, 반경) 프로파일을 Y 축 둘레로 돌려 회전체를 만든다.

    trimesh.creation.cylinder 는 반경이 일정해야 해서 크라운을 못 만든다.
    """
    ang = np.linspace(0.0, 2.0 * np.pi, sections, endpoint=False)
    c, sn = np.cos(ang), np.sin(ang)
    rings = [np.stack([r * c, np.full(sections, y), r * sn], axis=-1)
             for y, r in zip(ys, radii)]
    verts = np.concatenate(rings, axis=0)
    faces = []
    for i in range(len(ys) - 1):
        a, b = i * sections, (i + 1) * sections
        for j in range(sections):
            k = (j + 1) % sections
            faces += [[a + j, b + j, a + k], [a + k, b + j, b + k]]
    # 양 끝 뚜껑
    lo = len(verts)
    verts = np.vstack([verts, [[0.0, ys[0], 0.0], [0.0, ys[-1], 0.0]]])
    top = (len(ys) - 1) * sections
    for j in range(sections):
        k = (j + 1) % sections
        faces += [[lo, j, k], [lo + 1, top + k, top + j]]
    return trimesh.Trimesh(vertices=verts, faces=np.array(faces),
                           process=False)


def wheel_mesh(p):
    """크라운(볼록 배럴) 트레드를 가진 휠.

    ⚠ **원통으로 만들면 안 된다.** 관 내벽은 원주 방향으로 휘어 있어서
    평면 트레드는 양끝 모서리 2점으로만 닿는다. 접촉이 날카로운 에지에
    몰리고 휠 중심이 설계보다 0.566mm 안쪽으로 밀린다.

    크라운 반경을 관 내반경(50mm)과 같게 두면 트레드 전폭이 벽에 맞물린다.
    폭 방향 위치 y 에서 휠 반경을 이렇게 깎는다.

        r(y) = wheel_r - (crown_r - sqrt(crown_r^2 - y^2))

    가장자리(y=±7.5)에서 0.566mm 얕아진다. 그래야 y=0 이 벽에 닿을 때
    가장자리도 같이 닿는다.
    """
    hw = p['wheel_width'] / 2.0
    cr = p.get('wheel_crown_r')
    ys = np.linspace(-hw, hw, p.get('crown_sections', 17))
    if cr:
        drop = cr - np.sqrt(np.maximum(cr ** 2 - ys ** 2, 0.0))
    else:
        drop = np.zeros_like(ys)
    radii = p['wheel_dia'] / 2.0 - drop
    wh = revolve_profile(ys, radii, p['sections'])
    hub = cylinder(radius=p['wheel_dia'] / 2.0 - 3.0,
                   height=p['wheel_width'] + 1.2, sections=p['sections'])
    hub.apply_transform(R(90, X))
    return trimesh.util.concatenate([wh, hub])


def torch_links(p):
    """용접 토치 2자유도 모듈을 링크별로 만든다.

    J1 은 **본체를 감싸는 링**이다. 바퀴는 전진·후진만 되고 조향이 없어
    로봇 자체를 결함 방향으로 돌릴 수 없다. 대신 링이 돌아 토치를 원주
    방향 어디로든 겨눈다.

    회전축을 관 중심축과 일치시키면 어느 방향으로 돌려도 토치와 관벽 사이
    간극이 일정하다. 관절식(회전 2개)은 직선 팔의 끝점이 원통 벽면을
    따라가지 않아 방향에 따라 간극이 변한다.

    ±180° 이지 360° 가 아니다. 360° 연속 회전은 슬립링이 필요하고
    슬립링은 용접 전류를 흘리기에 부적합하다. ±180° 로 전 방향에 도달하며
    케이블도 안 꼬인다. 용접봉은 회전축 중심의 중공으로 공급한다.

    articulation 을 걸어야 하므로 ring / rod / tip 을 따로 낸다.
    각 링크의 원점은 그 링크의 조인트 위치다.
    """
    out = {}

    # 링. 본체(타원 50x40, 최대 반경 25)를 감싸므로 안지름이 그보다 커야 한다.
    ring = annulus(r_min=p['ring_r_in'], r_max=p['ring_r_out'],
                   height=p['ring_w'], sections=p['sections'])
    ring.apply_transform(R(90, Y))
    out['torch_ring'] = ring

    # J2 직동. 링 외곽에서 관벽 쪽으로 뻗는다. 원점은 조인트 위치다.
    lr = p['torch_rod_len']
    rod = box(extents=[p['torch_arm_w'], p['torch_arm_w'], lr])
    rod.apply_transform(T(z=lr / 2.0))
    out['torch_rod'] = rod

    lt = p['torch_tip_len']
    tip = cylinder(radius=3.0, height=lt * 0.7, sections=24)
    tip.apply_transform(T(z=lt * 0.35))
    nozzle = cylinder(radius=1.6, height=lt * 0.35, sections=16)
    nozzle.apply_transform(T(z=lt * 0.82))
    out['torch_tip'] = trimesh.util.concatenate([tip, nozzle])
    return out


def _wall_patch(r_in_fn, half_len, half_ang, r_out, nu=40, nv=40,
                drop_fn=None):
    """관벽 조각. 결함·비드를 같은 개구부에 끼우기 위한 공통 형상.

    로컬 프레임은 배관과 같다. 축 = X, 조각 중심은 +Z 방향(각도 0).
    r_in_fn(u, v) 가 안쪽 반경을 준다. drop_fn 이 True 인 셀은 뚫는다.
    """
    us = np.linspace(-half_len, half_len, nu + 1)
    vs = np.linspace(-half_ang, half_ang, nv + 1)
    U, V = np.meshgrid(us, vs, indexing='ij')
    RI = r_in_fn(U, V)

    def ring(r):
        return np.stack([U, r * np.sin(V), r * np.cos(V)], axis=-1)

    inner = ring(RI)
    outer = ring(np.full_like(RI, r_out))
    verts = np.concatenate([inner.reshape(-1, 3), outer.reshape(-1, 3)])
    n = (nu + 1) * (nv + 1)

    def idx(i, j):
        return i * (nv + 1) + j

    faces = []
    live = np.ones((nu, nv), dtype=bool)
    if drop_fn is not None:
        cu = 0.5 * (us[:-1] + us[1:])
        cv = 0.5 * (vs[:-1] + vs[1:])
        CU, CV = np.meshgrid(cu, cv, indexing='ij')
        live = ~drop_fn(CU, CV)

    for i in range(nu):
        for j in range(nv):
            if not live[i, j]:
                continue
            a, b = idx(i, j), idx(i, j + 1)
            c, d = idx(i + 1, j), idx(i + 1, j + 1)
            faces += [[a, c, b], [b, c, d]]                    # 안쪽 면
            faces += [[n + a, n + b, n + c], [n + b, n + d, n + c]]  # 바깥 면

    # 뚫린 구멍의 옆벽과 조각 테두리를 막는다
    def wall(a, b):
        faces.append([a, b, n + a])
        faces.append([b, n + b, n + a])

    for i in range(nu):
        for j in range(nv):
            if not live[i, j]:
                continue
            if i == 0 or not live[i - 1, j]:
                wall(idx(i, j + 1), idx(i, j))
            if i == nu - 1 or not live[i + 1, j]:
                wall(idx(i + 1, j), idx(i + 1, j + 1))
            if j == 0 or not live[i, j - 1]:
                wall(idx(i, j), idx(i + 1, j))
            if j == nv - 1 or not live[i, j + 1]:
                wall(idx(i + 1, j + 1), idx(i, j + 1))

    m = trimesh.Trimesh(vertices=verts, faces=np.array(faces))
    m.remove_unreferenced_vertices()
    m.fix_normals()
    return m


def defect_and_bead(kind, d, bore_r, wall_r):
    """결함 조각과 그것을 메운 비드 조각. 같은 개구부에 끼우는 한 쌍이다.

    가시성만 전환하면 형상이 실제로 바뀐다. 색상 덮어쓰기를 배제한 이유가
    여기 있다 — Depth 는 여전히 홈을 읽으므로 "안 보이는데 파여 있는" 모순이
    생긴다. 비드는 홈을 메우고 내벽보다 살짝 돌출시켜 Depth 로도 확인된다.
    """
    hl = d['patch_len_mm'] / 2.0
    ha = np.radians(d['patch_ang_deg'] / 2.0)
    proud = d['bead_proud_mm']

    if kind == 'crack':
        # 축 방향으로 난 V 홈. 반경이 커지는 쪽이 파인 것이다.
        half_w = d['crack_width_mm'] / 2.0
        half_l = d['crack_len_mm'] / 2.0
        dep = d['crack_depth_mm']

        def groove(u, v):
            arc = v * bore_r
            prof = np.clip(1.0 - np.abs(arc) / half_w, 0.0, 1.0)
            along = np.clip(1.0 - (np.abs(u) / half_l) ** 6, 0.0, 1.0)
            return bore_r + dep * prof * along

        def bead(u, v):
            arc = v * bore_r
            w = half_w + d['bead_margin_mm']
            l = half_l + d['bead_margin_mm']
            prof = np.clip(1.0 - (np.abs(arc) / w) ** 2, 0.0, 1.0)
            along = np.clip(1.0 - (np.abs(u) / l) ** 8, 0.0, 1.0)
            return bore_r - proud * prof * along

        return (_wall_patch(groove, hl, ha, wall_r),
                _wall_patch(bead, hl, ha, wall_r))

    # 관통 구멍. Isaac Sim 에는 불리언 연산이 없으므로 테셀레이션 단계에서
    # 셀을 빼고 옆벽을 세워 만든다. 설계 12.5 의 "불리언 컷" 은 쓸 수 없다.
    rad = d['hole_dia_mm'] / 2.0

    def hole(u, v):
        return np.hypot(u, v * bore_r) < rad

    def flat(u, v):
        return np.full_like(u, bore_r)

    def cap(u, v):
        r = np.hypot(u, v * bore_r)
        w = rad + d['bead_margin_mm']
        prof = np.clip(1.0 - (r / w) ** 2, 0.0, 1.0)
        return bore_r - proud * prof

    return (_wall_patch(flat, hl, ha, wall_r, drop_fn=hole),
            _wall_patch(cap, hl, ha, wall_r))


def camera_housing(p):
    """카메라 하우징 — RealSense 외형(가로로 긴 바 + 렌즈 3개)을 로봇 스케일로.

    실제 D455 는 124x26x29mm 라 세그먼트 62mm 안에 안 들어간다. 스펙상으로도
    최소 측정 거리 52cm 라 적용 불가 판정이 났다. 형상만 따오고 치수는
    본체 단면(50x40)에 맞춘다. 시각 전용이며 콜라이더를 걸지 않는다.
    """
    w, h, d = 44.0, 13.0, 8.0
    body = box(extents=[d, w, h])
    parts = [body]
    for i, (y, r) in enumerate(((-15.0, 4.2), (0.0, 3.0), (15.0, 4.2))):
        lens = cylinder(radius=r, height=2.5, sections=32)
        lens.apply_transform(R(90, Y))
        lens.apply_transform(T(x=d / 2.0 + 1.0, y=y))
        parts.append(lens)
    ring = annulus(r_min=4.2, r_max=5.2, height=1.6, sections=32)
    ring.apply_transform(R(90, Y))
    ring.apply_transform(T(x=d / 2.0 + 0.6, y=-15.0))
    parts.append(ring)
    return trimesh.util.concatenate(parts)


def _sweep_tube(centers, us, vs, axes, r_in_fn, r_out, sections):
    """중심선을 따라 관을 훑어 만든다.

    centers / us / vs / axes 는 링마다의 원점과 단면 기저·축이다.
    r_in_fn(i, phi) 가 그 지점의 안쪽 반경을 준다 — 여기에 균열을 새긴다.
    """
    phi = np.linspace(0.0, 2.0 * np.pi, sections, endpoint=False)
    n = len(centers)
    verts, faces = [], []

    for surf, is_inner in ((r_in_fn, True), (None, False)):
        base = len(verts)
        for i in range(n):
            r = (surf(i, phi) if is_inner
                 else np.full(sections, r_out))
            pts = (centers[i]
                   + r[:, None] * (np.cos(phi)[:, None] * us[i]
                                   + np.sin(phi)[:, None] * vs[i]))
            verts.extend(pts)
        for i in range(n - 1):
            for j in range(sections):
                a = base + i * sections + j
                b = base + i * sections + (j + 1) % sections
                c = base + (i + 1) * sections + j
                d = base + (i + 1) * sections + (j + 1) % sections
                if is_inner:
                    faces += [[a, c, b], [b, c, d]]
                else:
                    faces += [[a, b, c], [b, d, c]]

    m = n * sections
    for k, i in enumerate((0, n - 1)):
        for j in range(sections):
            a = i * sections + j
            b = i * sections + (j + 1) % sections
            c = m + i * sections + j
            d = m + i * sections + (j + 1) % sections
            if k == 0:
                faces += [[a, b, c], [b, d, c]]
            else:
                faces += [[a, c, b], [b, c, d]]

    mesh = trimesh.Trimesh(vertices=np.array(verts), faces=np.array(faces))
    mesh.fix_normals()
    return mesh


def _refined(lo, hi, site, half, coarse_mm=20.0, fine_mm=0.5, pad=3.0):
    """균열 근처만 조밀하게 표본한다.

    관 전체를 균등하게 잘게 쪼개면 메시가 쓸데없이 커진다(600mm 직관에
    400링이면 7.7MB). 균열 폭이 1.6mm, 길이 18mm 라 그 구간만 0.5mm 로
    잡고 나머지는 20mm 로 둔다.
    """
    a, b = site - half - pad, site + half + pad
    a, b = max(lo, a), min(hi, b)
    parts = [np.arange(lo, a, coarse_mm), np.arange(a, b, fine_mm),
             np.arange(b, hi, coarse_mm), np.array([hi])]
    return np.unique(np.concatenate([x for x in parts if len(x)]))


def _crack_profile(d, bore_r, du_mm, arc_mm):
    """축방향 du, 원주방향 arc 위치에서 파인 깊이(mm)."""
    hw = d['crack_width_mm'] / 2.0
    hl = d['crack_len_mm'] / 2.0
    prof = np.clip(1.0 - np.abs(arc_mm) / hw, 0.0, 1.0)
    along = np.clip(1.0 - (np.abs(du_mm) / hl) ** 6, 0.0, 1.0)
    return d['crack_depth_mm'] * prof * along


def tube_straight_crack(inner_r, outer_r, length, sections, d,
                        site_x=0.0, site_deg=0.0):
    """균열이 새겨진 직관. 패치를 끼우는 방식이 아니라 관 자체에 판 것이다."""
    xs = _refined(-length / 2.0, length / 2.0, site_x,
                  d['crack_len_mm'] / 2.0)
    nu = len(xs)
    centers = np.stack([xs, np.zeros(nu), np.zeros(nu)], axis=-1)
    us = np.tile(np.array([0.0, 0.0, 1.0]), (nu, 1))
    vs = np.tile(np.array([0.0, 1.0, 0.0]), (nu, 1))
    axes = np.tile(np.array([1.0, 0.0, 0.0]), (nu, 1))
    site = np.radians(site_deg)

    def r_in(i, phi):
        dphi = (phi - site + np.pi) % (2 * np.pi) - np.pi
        return inner_r + _crack_profile(d, inner_r, xs[i] - site_x,
                                        dphi * inner_r)

    return _sweep_tube(centers, us, vs, axes, r_in, outer_r, sections)


def tube_elbow_crack(inner_r, outer_r, bend_r, sweep_deg, steps, sections, d,
                     site_frac=0.5, site_deg=0.0):
    """균열이 새겨진 SR 곡관. 균열은 호 중간에 둔다."""
    total = np.radians(sweep_deg) * bend_r
    ss = _refined(0.0, total, total * site_frac, d['crack_len_mm'] / 2.0,
                  coarse_mm=total / steps, fine_mm=0.5)
    ts = ss / bend_r
    n = len(ts)
    centers = np.stack([bend_r * np.sin(ts), np.zeros(n),
                        bend_r * (1.0 - np.cos(ts))], axis=-1)
    us = np.stack([-np.sin(ts), np.zeros(n), np.cos(ts)], axis=-1)
    vs = np.tile(np.array([0.0, 1.0, 0.0]), (n, 1))
    axes = np.stack([np.cos(ts), np.zeros(n), np.sin(ts)], axis=-1)
    arc = ts * bend_r
    site_s = arc[-1] * site_frac
    site = np.radians(site_deg)

    def r_in(i, phi):
        dphi = (phi - site + np.pi) % (2 * np.pi) - np.pi
        return inner_r + _crack_profile(d, inner_r, arc[i] - site_s,
                                        dphi * inner_r)

    return _sweep_tube(centers, us, vs, axes, r_in, outer_r, sections)


def tube_straight(inner_r, outer_r, length, sections):
    m = annulus(r_min=inner_r, r_max=outer_r, height=length, sections=sections)
    m.apply_transform(R(90, Y))
    return m


def tube_elbow(inner_r, outer_r, bend_r, sweep_deg, steps, sections):
    ts = np.radians(np.linspace(0.0, sweep_deg, steps + 1))
    phi = np.linspace(0.0, 2.0 * np.pi, sections, endpoint=False)
    verts, faces = [], []

    def ring(t, r):
        cx = bend_r * np.sin(t)
        cz = bend_r * (1.0 - np.cos(t))
        ax = np.array([np.cos(t), 0.0, np.sin(t)])
        rad = np.array([-np.sin(t), 0.0, np.cos(t)])
        tan = np.array([0.0, 1.0, 0.0])
        c = np.array([cx, 0.0, cz])
        return c + r * (np.cos(phi)[:, None] * rad + np.sin(phi)[:, None] * tan), ax

    for r in (inner_r, outer_r):
        base = len(verts)
        for t in ts:
            pts, _ = ring(t, r)
            verts.extend(pts)
        for i in range(len(ts) - 1):
            for j in range(sections):
                a = base + i * sections + j
                b = base + i * sections + (j + 1) % sections
                c = base + (i + 1) * sections + j
                d = base + (i + 1) * sections + (j + 1) % sections
                if r == inner_r:
                    faces += [[a, c, b], [b, c, d]]
                else:
                    faces += [[a, b, c], [b, d, c]]
    n_ring = len(ts) * sections
    for k, i in enumerate((0, len(ts) - 1)):
        for j in range(sections):
            a = i * sections + j
            b = i * sections + (j + 1) % sections
            c = n_ring + i * sections + j
            d = n_ring + i * sections + (j + 1) % sections
            if k == 0:
                faces += [[a, b, c], [b, d, c]]
            else:
                faces += [[a, c, b], [b, c, d]]
    m = trimesh.Trimesh(vertices=np.array(verts), faces=np.array(faces))
    m.fix_normals()
    return m


def arm_angle_for_contact(contact_r, p):
    s = (contact_r - p['wheel_dia'] / 2.0 - p['pivot_r']) / p['arm_len']
    return np.degrees(np.arcsin(np.clip(s, -1.0, 1.0)))


def wheel_seat_radius(bore_r, p, samples=201):
    """휠이 관벽에 닿을 때 **휠 중심**이 관축에서 갖는 반경.

    휠 축은 원주 방향이다. 트레드 폭 방향 위치 y, 그 자리의 휠 반경 r(y) 인
    점은 관축에서 sqrt(y^2 + (R_c + r(y))^2) 떨어져 있다. 이 최댓값이 관 내반경
    과 같아지는 R_c 를 찾는다.

      크라운 없음  r(y)=10 고정 → 최댓값이 **가장자리**(y=±7.5)에서 나온다.
                   R_c = sqrt(50^2 - 7.5^2) - 10 = 39.434mm — 0.566mm 밀린다.
      크라운 R50   폭 전체에서 정확히 50.0 이 된다(크라운 반경 = 관 내반경).
                   R_c = 50 - 10 = 40.0mm — 설계값 그대로.
    """
    hw = p['wheel_width'] / 2.0
    cr = p.get('wheel_crown_r')
    ys = np.linspace(-hw, hw, samples)
    drop = (cr - np.sqrt(np.maximum(cr ** 2 - ys ** 2, 0.0))) if cr else 0.0
    r = p['wheel_dia'] / 2.0 - drop
    # max_y sqrt(y^2 + (R_c + r(y))^2) = bore_r  →  R_c = min_y (...)
    return float(np.min(np.sqrt(np.maximum(bore_r ** 2 - ys ** 2, 0.0)) - r))


def arm_angle_seated(bore_r, p):
    """휠이 관벽에 착좌하는 암 각도."""
    r_wheel = wheel_seat_radius(bore_r, p)
    return np.degrees(np.arcsin((r_wheel - p['pivot_r']) / p['arm_len']))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=str(Path(__file__).resolve().parents[1]))
    ap.add_argument('--keep-doc-angle', action='store_true',
                    help='암 각도를 설계 문서값 44.4도로 고정 (림 간섭 0.55mm 발생)')
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    p = dict(P)

    arm_angle_doc = p['arm_angle']
    if not args.keep_doc_angle:
        p['arm_angle'] = float(arm_angle_seated(PIPE_ID / 2.0, p))

    L, J = p['seg_len'], p['joint_len']
    arm, dx, dr = arm_mesh(p)
    wheel = wheel_mesh(p)

    CATEGORY = {
        'body_rear': 'robot', 'body_front': 'robot', 'bellows': 'robot',
        'arm': 'robot', 'wheel': 'robot',
        'camera_housing': 'camera',
        'pipe_straight': 'pipe', 'pipe_elbow_sr': 'pipe',
        'pipe_straight_crack': 'pipe', 'pipe_elbow_crack': 'pipe',
        'defect_crack': 'pipe', 'defect_hole': 'pipe',
        'bead_crack': 'pipe', 'bead_hole': 'pipe',
        'torch_ring': 'welder', 'torch_rod': 'welder', 'torch_tip': 'welder',
    }

    meshes = {
        'body_rear': trimesh.util.concatenate(
            [rounded_body(p['body_w'], p['body_h'], L, p['sections']),
             camera(-L / 2.0, -1, p)]),
        'body_front': trimesh.util.concatenate(
            [rounded_body(p['body_w'], p['body_h'], L, p['sections']),
             camera(L / 2.0, +1, p)]),
        'bellows': bellows(p['bellows_body'], p['bellows_crest'], J,
                           p['bellows_folds'], p['sections']),
        'arm': arm,
        'wheel': wheel,
        'camera_housing': camera_housing(p),
        'pipe_straight': tube_straight(PIPE_ID / 2.0, PIPE_OD / 2.0,
                                       PIPE_STRAIGHT_LEN, p['sections'] * 2),
        'pipe_elbow_sr': tube_elbow(PIPE_ID / 2.0, PIPE_OD / 2.0, ELBOW_R,
                                    ELBOW_SWEEP, ELBOW_STEPS, p['sections'] * 2),
        # 균열이 관 자체에 새겨진 판. 패치 방식(defect_/bead_)과 별개로,
        # 결함 있는 배관을 통짜로 쓰고 싶을 때(YOLO 학습 영상, 눈 확인).
        'pipe_straight_crack': tube_straight_crack(
            PIPE_ID / 2.0, PIPE_OD / 2.0, PIPE_STRAIGHT_LEN,
            p['sections'] * 2, DEFECT),
        'pipe_elbow_crack': tube_elbow_crack(
            PIPE_ID / 2.0, PIPE_OD / 2.0, ELBOW_R, ELBOW_SWEEP,
            ELBOW_STEPS, p['sections'] * 2, DEFECT),
    }
    meshes.update(torch_links(p))
    for kind in ('crack', 'hole'):
        dm, bm = defect_and_bead(kind, DEFECT, PIPE_ID / 2.0, PIPE_OD / 2.0)
        meshes[f'defect_{kind}'] = dm
        meshes[f'bead_{kind}'] = bm

    print('=' * 78)
    print('STL 출력 — 용도별로 나눠 저장한다')
    print('=' * 78)
    for cat in ('robot', 'camera', 'pipe', 'welder'):
        (out / cat / 'meshes').mkdir(parents=True, exist_ok=True)
        print(f'  [{cat}/meshes/]')
        for name, m in meshes.items():
            if CATEGORY[name] != cat:
                continue
            m.merge_vertices()
            m.export(str(out / cat / 'meshes' / f'{name}.stl'))
            b = m.bounds
            print(f'    {name:16s} 삼각형 {len(m.faces):6,d}  '
                  f'bbox {np.round(b[1] - b[0], 1)}  '
                  f'watertight={m.is_watertight}')

    wheel_r = p['wheel_dia'] / 2.0
    contact_r = p['pivot_r'] + dr + wheel_r
    meta = dict(
        axis='X',
        unit='mm',
        seg_len=L,
        joint_len=J,
        seg_center_offset=J / 2.0 + L / 2.0,
        pivot_r=p['pivot_r'],
        arm_len=p['arm_len'],
        arm_angle_nominal=p['arm_angle'],
        arm_dx_nominal=dx,
        arm_dr_nominal=dr,
        wheel_r=wheel_r,
        wheel_width=p['wheel_width'],
        wheel_crown_r=p.get('wheel_crown_r'),
        stagger=p['stagger'],
        front_arms=p['front_arms'],
        rear_arms=p['rear_arms'],
        contact_r_nominal=contact_r,
        arm_angle_compressed=arm_angle_for_contact(contact_r - 6.0, p),
        arm_angle_extended=arm_angle_for_contact(contact_r + 6.0, p),
        pipe_id=PIPE_ID,
        pipe_od=PIPE_OD,
        elbow_r=ELBOW_R,
    )
    meta['layout'] = {k: f'{v}/meshes/{k}.stl' for k, v in CATEGORY.items()}
    meta['defect'] = dict(DEFECT)
    meta['torch'] = dict(
        mount='ring', ring_r_in=p['ring_r_in'], ring_r_out=p['ring_r_out'],
        ring_w=p['ring_w'], ring_x=p['ring_x'],
        rod_len=p['torch_rod_len'], tip_len=p['torch_tip_len'],
        arm_w=p['torch_arm_w'], rod_origin_r=p['ring_r_out'],
        j1_limit_deg=180.0, j2_stroke_mm=p['torch_stroke'],
        stow_radius_mm=p['ring_r_out'] + p['torch_rod_len'] + p['torch_tip_len'],
        reach_radius_mm=(p['ring_r_out'] + p['torch_rod_len']
                         + p['torch_tip_len'] + p['torch_stroke']))
    (out / 'spec').mkdir(parents=True, exist_ok=True)
    (out / 'spec' / 'parts_meta.json').write_text(json.dumps(meta, indent=2))

    print('=' * 74)
    print('기준 자세 (articulation 스크립트가 그대로 읽는 값)')
    print('=' * 74)
    print(f'  세그먼트 중심 간격      ±{meta["seg_center_offset"]:.1f} mm')
    print(f'  암 피벗 반경            {p["pivot_r"]:.1f} mm')
    print(f'  암 기준 각도            {p["arm_angle"]:.2f}°  '
          f'(축방향 {dx:.2f} / 반경방향 {dr:.2f} mm)')
    cr = p.get('wheel_crown_r')
    print(f'  휠 크라운 반경          '
          f'{f"{cr:.1f} mm (관 내반경 {PIPE_ID / 2.0:.1f} 과 일치 → 전폭 밀착)" if cr else "없음 — 원통 트레드, 에지 2점 접촉"}')
    if abs(p['arm_angle'] - arm_angle_doc) > 0.05:
        print(f'    ※ 문서값 {arm_angle_doc:.2f}° 와 {p["arm_angle"] - arm_angle_doc:+.2f}° 차이. '
              f'크라운 없이 유도하면 이런 편차가 난다')
    else:
        print(f'    문서값 {arm_angle_doc:.1f}° 와 일치 '
              f'(차이 {p["arm_angle"] - arm_angle_doc:+.3f}°)')
        r_centre = contact_r - p['wheel_dia'] / 2.0      # 휠 중심의 관축 반경
    rim = max(np.hypot(y, r_centre + p['wheel_dia'] / 2.0 - (
        (p['wheel_crown_r'] - np.sqrt(max(p['wheel_crown_r'] ** 2 - y ** 2, 0.0)))
        if p.get('wheel_crown_r') else 0.0))
        for y in np.linspace(-p['wheel_width'] / 2.0, p['wheel_width'] / 2.0, 101))
    print(f'    트레드 최대 반경        {rim:.3f} mm   (관벽 {PIPE_ID / 2.0:.1f} 이하여야 함)')
    print(f'  휠 접촉 반경            {contact_r:.2f} mm   (DN100 내반경 50.0)')
    print(f'  암 각도 한계            {meta["arm_angle_compressed"] - p["arm_angle"]:+.2f}° '
          f"~ {meta['arm_angle_extended'] - p['arm_angle']:+.2f}°  (스트로크 ±6mm)")
    print(f'  곡관 곡률반경           {ELBOW_R:.0f} mm (SR = 1.0 x DN)')
    for length, label in ((L, '세그먼트 62mm'), (2 * L + J, '일체형 150mm')):
        sag = ELBOW_R - np.sqrt(ELBOW_R ** 2 - (length / 2.0) ** 2)
        print(f'    {label:14s} 새그 {sag:6.2f} mm  →  필요 스트로크 ±{sag / 2.0:5.2f} mm')
    print(f'\n저장 위치: {out}/<robot|camera|pipe>/meshes/  + {out}/spec/')


if __name__ == '__main__':
    main()
