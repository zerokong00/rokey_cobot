"""
배관 점검 및 수리 로봇 — STL 생성 및 렌더링

설계 확정본 기준으로 로봇 형상을 파라메트릭하게 생성한다.
  - 2단 1관절, 전장 150mm (62 + 26 + 62)
  - 본체 단면 50 x 40mm (타원 기둥)
  - 6륜, 세그먼트당 3암 120도 (전방 0/120/240, 후방 60/180/300)
  - 전후 카메라, 수리기는 토치 액추에이터 2자유도

실행:
  python3 build_robot_stl.py --variant welder --bend 12 --out ./out
"""

import argparse
import os
import numpy as np
import trimesh
from trimesh.creation import cylinder, box, annulus, icosphere
from trimesh.transformations import rotation_matrix, translation_matrix, concatenate_matrices

# ── 설계 파라미터 (단위 mm) ────────────────────────────────────────────
P = dict(
    seg_len       = 62.0,    # 세그먼트 길이
    joint_len     = 26.0,    # 벨로우즈 유연부 길이
    body_w        = 50.0,    # 본체 폭
    body_h        = 40.0,    # 본체 높이
    bellows_body  = 36.0,    # 벨로우즈 몸통 외경
    bellows_crest = 40.0,    # 벨로우즈 주름 정점 외경
    bellows_folds = 4,       # 주름 산 개수
    pivot_r       = 12.0,    # 암 피벗 반경
    arm_len       = 40.0,    # 암 길이
    arm_angle     = 44.4,    # 암 각도 (정상 상태)
    arm_thick     = 6.0,     # 암 두께
    arm_width     = 10.0,    # 암 폭
    wheel_dia     = 20.0,    # 휠 직경
    wheel_width   = 15.0,    # 휠 폭
    stagger       = 10.0,    # 축 방향 스태거
    cam_dia       = 16.0,    # 카메라 렌즈 직경
    cam_depth     = 4.0,
    torch_rot_r   = 9.0,     # 토치 회전부 반경
    torch_arm_len = 28.0,    # 토치 직동 신장량 (수납 20 -> 48)
    torch_arm_w   = 7.0,
    front_arms    = [0.0, 120.0, 240.0],
    rear_arms     = [60.0, 180.0, 300.0],
    sections      = 48,      # 원통 분할 수
)

Z = np.array([0.0, 0.0, 1.0])
X = np.array([1.0, 0.0, 0.0])
Y = np.array([0.0, 1.0, 0.0])


def T(x=0.0, y=0.0, z=0.0):
    """평행이동 행렬을 만든다."""
    return translation_matrix([x, y, z])


def R(deg, axis, point=(0, 0, 0)):
    """point를 지나는 axis 축 기준 회전 행렬을 만든다."""
    return rotation_matrix(np.radians(deg), axis, point)


def elliptic_cylinder(w, h, length, sections):
    """
    타원 단면 기둥을 만든다. 축은 X 방향, 원점 중심.
      w       : 폭 (Y 방향 전체 치수)
      h       : 높이 (Z 방향 전체 치수)
      length  : 축 방향 길이
      sections: 원주 분할 수
    """
    m = cylinder(radius=1.0, height=length, sections=sections)
    m.apply_scale([w / 2.0, h / 2.0, 1.0])       # 단면을 타원으로
    m.apply_transform(R(90, Y))                   # Z축 기둥 -> X축 기둥
    return m


def rounded_body(w, h, length, sections):
    """세그먼트 본체. 타원 기둥 양 끝을 살짝 챔퍼링한 형상을 만든다."""
    core = elliptic_cylinder(w, h, length - 6.0, sections)
    caps = []
    for sgn in (-1, 1):
        c = elliptic_cylinder(w - 5.0, h - 5.0, 6.0, sections)
        c.apply_transform(T(x=sgn * (length - 6.0) / 2.0 + sgn * 3.0))
        caps.append(c)
    return trimesh.util.concatenate([core] + caps)


def bellows(inner_d, crest_d, length, folds, sections):
    """
    벨로우즈(주름 외피)를 만든다. 축은 X 방향, 원점 중심.
      inner_d : 몸통 외경
      crest_d : 주름 정점 외경
      length  : 유연부 길이
      folds   : 주름 산 개수
    """
    parts = []
    tube = cylinder(radius=inner_d / 2.0, height=length, sections=sections)
    tube.apply_transform(R(90, Y))
    parts.append(tube)

    pitch = length / folds
    for i in range(folds):
        x = -length / 2.0 + pitch * (i + 0.5)
        ring = annulus(r_min=inner_d / 2.0 - 0.5,
                       r_max=crest_d / 2.0,
                       height=pitch * 0.45,
                       sections=sections)
        ring.apply_transform(R(90, Y))
        ring.apply_transform(T(x=x))
        parts.append(ring)
    return trimesh.util.concatenate(parts)


def wheel_assembly(angle_deg, x_offset, p):
    """
    암 1개와 휠 1개로 이루어진 서스펜션 유닛을 만든다.
      angle_deg: 원주 방향 배치 각도 (0도 = 12시, 시계 방향)
      x_offset : 세그먼트 중심 기준 축 방향 위치
    """
    psi = np.radians(p['arm_angle'])
    r_pivot = p['pivot_r']
    r_wheel = r_pivot + p['arm_len'] * np.sin(psi)      # 휠 중심 반경
    dx_arm = p['arm_len'] * np.cos(psi)                 # 암의 축 방향 성분

    # 암 — 피벗에서 휠 중심까지 잇는 막대
    arm = box(extents=[p['arm_len'], p['arm_width'], p['arm_thick']])
    tilt = np.degrees(np.arctan2(r_wheel - r_pivot, dx_arm))
    arm.apply_transform(R(tilt, Y))
    arm.apply_transform(T(x=-dx_arm / 2.0, z=(r_pivot + r_wheel) / 2.0))

    # 휠 — 축이 Y 방향인 원통
    wh = cylinder(radius=p['wheel_dia'] / 2.0,
                  height=p['wheel_width'],
                  sections=p['sections'])
    wh.apply_transform(R(90, X))
    wh.apply_transform(T(z=r_wheel))

    # 허브
    hub = cylinder(radius=p['wheel_dia'] / 2.0 - 3.0,
                   height=p['wheel_width'] + 1.2,
                   sections=p['sections'])
    hub.apply_transform(R(90, X))
    hub.apply_transform(T(z=r_wheel))

    unit = trimesh.util.concatenate([arm, wh, hub])
    unit.apply_transform(R(-angle_deg, X))              # 원주 방향 배치
    unit.apply_transform(T(x=x_offset))
    return unit


def camera(x_pos, facing, p):
    """단부 카메라(렌즈 + 하우징)를 만든다. facing은 +1 전방, -1 후방."""
    lens = cylinder(radius=p['cam_dia'] / 2.0, height=p['cam_depth'],
                    sections=p['sections'])
    lens.apply_transform(R(90, Y))
    lens.apply_transform(T(x=x_pos + facing * p['cam_depth'] / 2.0))

    ring = annulus(r_min=p['cam_dia'] / 2.0, r_max=p['cam_dia'] / 2.0 + 3.0,
                   height=2.0, sections=p['sections'])
    ring.apply_transform(R(90, Y))
    ring.apply_transform(T(x=x_pos + facing * 1.0))
    return trimesh.util.concatenate([lens, ring])


def torch_actuator(x_pos, roll_deg, extend, p):
    """
    용접 토치 액추에이터(회전 + 직동 2자유도)를 만든다.
      roll_deg: J1 원주 방향 회전각
      extend  : J2 신장량 0 ~ torch_arm_len
    """
    base = cylinder(radius=p['torch_rot_r'], height=10.0, sections=p['sections'])
    base.apply_transform(R(90, Y))
    base.apply_transform(T(x=x_pos + 5.0))

    reach = 20.0 + extend                                # 축에서의 반경
    rod = box(extents=[p['torch_arm_w'], p['torch_arm_w'], reach])
    rod.apply_transform(T(x=x_pos + 12.0, z=reach / 2.0))

    tip = cylinder(radius=3.0, height=9.0, sections=24)
    tip.apply_transform(T(x=x_pos + 12.0, z=reach + 3.0))

    arm = trimesh.util.concatenate([rod, tip])
    arm.apply_transform(R(-roll_deg, X, point=(x_pos, 0, 0)))
    return trimesh.util.concatenate([base, arm])


def build_robot(variant='welder', bend_deg=12.0, torch_roll=210.0, torch_extend=18.0, p=P):
    """
    로봇 전체 형상을 조립한다.
      variant     : 'scout' 정찰기 / 'welder' 수리기
      bend_deg    : 관절 굽힘각 (양수 = 전방이 위로)
      torch_roll  : 토치 회전각 (수리기 전용)
      torch_extend: 토치 신장량 (수리기 전용)
    """
    L, J = p['seg_len'], p['joint_len']
    rear_cx = -(J / 2.0 + L / 2.0)                       # 후방 세그먼트 중심
    front_cx = +(J / 2.0 + L / 2.0)
    joint_x = 0.0

    # ── 후방 세그먼트 ──
    rear = [rounded_body(p['body_w'], p['body_h'], L, p['sections'])]
    for i, a in enumerate(p['rear_arms']):
        rear.append(wheel_assembly(a, (i - 1) * p['stagger'], p))
    rear.append(camera(-L / 2.0, -1, p))
    rear = trimesh.util.concatenate(rear)
    rear.apply_transform(T(x=rear_cx))

    # ── 전방 세그먼트 ──
    front = [rounded_body(p['body_w'], p['body_h'], L, p['sections'])]
    for i, a in enumerate(p['front_arms']):
        front.append(wheel_assembly(a, (i - 1) * p['stagger'], p))
    front.append(camera(L / 2.0, +1, p))
    if variant == 'welder':
        front.append(torch_actuator(L / 2.0, torch_roll, torch_extend, p))
    front = trimesh.util.concatenate(front)
    front.apply_transform(T(x=front_cx))
    front.apply_transform(R(bend_deg, Y, point=(joint_x, 0, 0)))   # 관절 굽힘

    # ── 벨로우즈 (굽힘의 절반만 적용해 자연스럽게) ──
    bel = bellows(p['bellows_body'], p['bellows_crest'], J,
                  p['bellows_folds'], p['sections'])
    bel.apply_transform(R(bend_deg / 2.0, Y, point=(joint_x, 0, 0)))

    robot = trimesh.util.concatenate([rear, bel, front])
    robot.merge_vertices()
    return robot


def render(mesh, path, elev=22, azim=-58, title=None):
    """
    메시를 STL 뷰어 스타일(단일 회색 재질, 평면 음영)로 렌더링한다.
    """
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    tri = mesh.triangles
    nrm = mesh.face_normals

    # 광원 방향에 대한 램버트 음영
    light = np.array([-0.45, -0.75, 0.50])
    light /= np.linalg.norm(light)
    shade = np.clip(nrm @ light, 0.0, 1.0) * 0.72 + 0.28
    base = np.array([0.62, 0.66, 0.70])
    colors = np.clip(shade[:, None] * base[None, :], 0, 1)
    colors = np.hstack([colors, np.ones((len(colors), 1))])

    fig = plt.figure(figsize=(12, 7.5), dpi=170)
    ax = fig.add_subplot(111, projection='3d')
    coll = Poly3DCollection(tri, facecolors=colors,
                            edgecolors=(0.32, 0.34, 0.37, 0.16), linewidths=0.18)
    ax.add_collection3d(coll)

    b = mesh.bounds
    ctr = mesh.centroid
    span = float(np.max(b[1] - b[0])) * 0.56
    ax.set_xlim(ctr[0] - span, ctr[0] + span)
    ax.set_ylim(ctr[1] - span, ctr[1] + span)
    ax.set_zlim(ctr[2] - span, ctr[2] + span)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    ax.set_facecolor('#f2f3f5')
    fig.patch.set_facecolor('#f2f3f5')
    if title:
        ax.set_title(title, fontsize=11, color='#3a3f45', pad=2)
    fig.tight_layout(pad=0.2)
    fig.savefig(path, facecolor=fig.get_facecolor())
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--variant', choices=['scout', 'welder'], default='welder')
    ap.add_argument('--bend', type=float, default=12.0, help='관절 굽힘각 (도)')
    ap.add_argument('--torch-roll', type=float, default=210.0)
    ap.add_argument('--torch-extend', type=float, default=18.0)
    ap.add_argument('--out', default='./out')
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    m = build_robot(args.variant, args.bend, args.torch_roll, args.torch_extend)

    stl = os.path.join(args.out, f'pipe_robot_{args.variant}.stl')
    m.export(stl)
    print(f'STL 저장: {stl}')
    print(f'  삼각형 {len(m.faces):,}개 / 정점 {len(m.vertices):,}개')
    print(f'  경계 상자 {np.round(m.bounds[1] - m.bounds[0], 1)} mm')
    print(f'  워터타이트: {m.is_watertight}')

    views = [(22, -58, '3/4 view'), (0, -90, 'side view'),
             (88, -90, 'top view'), (6, 0, 'front view')]
    for elev, azim, name in views:
        png = os.path.join(args.out, f'render_{args.variant}_{name.split()[0].replace("/", "")}.png')
        render(m, png, elev, azim, f'{args.variant} — {name}')
        print(f'렌더 저장: {png}')


if __name__ == '__main__':
    main()
