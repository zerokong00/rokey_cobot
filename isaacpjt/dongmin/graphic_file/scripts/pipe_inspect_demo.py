"""배관 내부 결함(크랙/구멍) 카메라 탐지 데모.

로봇 본체 앞끝에 전방 카메라 + 조명을 부착하고 100mm 직관을 주행하며,
내벽에 심어 둔 결함을 컴퓨터비전으로 탐지해 위치와 함께 기록한다.

CLAUDE.md 크리티컬 포인트 대응:
  C1  배관 = 정적 collider + 삼각메시. 카메라/조명/결함은 collider 없는 시각 프림
      -> 주행에 물리적 간섭 0
  C2  배관 내부는 무광. DomeLight 끄고 로봇 부착 조명만 사용, 프레임 평균 밝기 실측
  C3  결함을 직접 심는다: 크랙 2(지그재그 어두운 띠) + 구멍 1(검은 원판),
      정답 위치를 ground_truth.csv 로 저장
  C7  탐지 레코드 = (프레임, 시뮬시각, 로봇 X 오도메트리, blob 정보) + 프레임 PNG
  C8  임계값은 추측하지 않는다 — 주행 초반 프레임의 실측 통계(중앙값)로 산출.
      탐지 실패 시 fallback 없이 FAIL 로 드러낸다.

실행:
  PYTHONUNBUFFERED=1 isaac_python pipe_inspect_demo.py            # GUI
  PYTHONUNBUFFERED=1 isaac_python pipe_inspect_demo.py --headless # 배치 탐지
"""

import csv
import math
import sys
from pathlib import Path

HEADLESS = "--headless" in sys.argv

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": HEADLESS})

import cv2
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.sensors.camera import Camera
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

HERE = Path(__file__).resolve().parent
ROBOT_USD = str(HERE / "robot_assembled.usd")
PIPE_USD = str(HERE.parent / "pipe" / "pipe.usd")
OUT_DIR = HERE / "inspect_out"
OUT_DIR.mkdir(exist_ok=True)
for _old in OUT_DIR.glob("*.png"):
    _old.unlink()                      # 이전 실행 프레임이 섞이면 결과 해석을 오염시킨다
for _old in OUT_DIR.glob("*.csv"):
    _old.unlink()

# ── 로봇 (robot_articulated.py 와 같은 값) ──
ROBOT_SCALE = 90.0 / 302.0
WHEEL_R_M = 23.0 * 0.001 * ROBOT_SCALE
WHEEL_OUT_M = (132.0 + 23.0) * 0.001 * ROBOT_SCALE
LEG_HI_M = 15.0 * 0.001 * ROBOT_SCALE
ROBOT_LEN_M = 393.5 * 0.001 * ROBOT_SCALE
CAM_OFF_M = 0.062                     # 본체 중심 -> 카메라 (본체 앞끝 58.6mm + 여유)

# ── 배관: 100mm 직관 4구간 (pipe_drive_demo 와 동일 구성) ──
PIPE_BORE_MM = 100.0
PIPE_SCALE = (PIPE_BORE_MM / 45.0) * 0.001
PIPE_SEC_LEN_M = 200.0 * PIPE_SCALE
PIPE_SECTIONS = 4
PIPE_LEN_M = PIPE_SECTIONS * PIPE_SEC_LEN_M
PIPE_FRICTION = 0.7
PIPE_IN_R_M = PIPE_BORE_MM * 0.0005

# ── 결함 정의 (C3). 전부 시각 전용 — collider 를 붙이지 않는다 ──
DEFECTS = [
    {"kind": "crack", "x_m": 0.55, "angle_deg": 40.0},
    {"kind": "crack", "x_m": 0.95, "angle_deg": 200.0},
    {"kind": "hole",  "x_m": 1.25, "angle_deg": 320.0},
]

# ── 주행/촬영 ──
SPIN_DEG_S = 1080.0
CAPTURE_EVERY = 6                     # 6 물리스텝(0.1s)마다 1프레임
CAM_RES = (320, 320)
CALIB_FRAMES = 12                     # 임계값 산출용 초반 프레임 수 (C8)
DRIVE_STEP_CAP = 3000
# 출구 0.3m 전에 정지: 출구 밖 어둠이 화면에 들어오면 통짜 오탐이 된다(실측: X>1.27 에서
# 39프레임 연속 오탐). 실전에서는 '관 끝 감지' 로직이 따로 필요하다는 뜻이다.
END_X_M = PIPE_LEN_M - 0.30

world = World(stage_units_in_meters=1.0)
stage = world.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage, "/World")
# C2: 외부 조명 없음 — 배관 내부가 실제처럼 어둡도록 DomeLight 를 두지 않는다.

# ══ 배관 ══════════════════════════════════════════════════════════
pipe_mat_path = "/World/Pipe/PipeMaterial"
UsdGeom.Xform.Define(stage, "/World/Pipe")
pm = UsdPhysics.MaterialAPI.Apply(
    UsdShade.Material.Define(stage, pipe_mat_path).GetPrim())
pm.CreateStaticFrictionAttr(PIPE_FRICTION)
pm.CreateDynamicFrictionAttr(PIPE_FRICTION)
pm.CreateRestitutionAttr(0.0)

for i in range(PIPE_SECTIONS):
    xf = UsdGeom.Xform.Define(stage, f"/World/Pipe/section_{i}")
    s = Gf.Matrix4d(1.0)
    s.SetScale(Gf.Vec3d(PIPE_SCALE, PIPE_SCALE, PIPE_SCALE))
    r = Gf.Matrix4d(1.0)
    r.SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), -90.0))
    t = Gf.Matrix4d(1.0)
    t.SetTranslate(Gf.Vec3d(i * PIPE_SEC_LEN_M, 0.0, 0.0))
    xf.AddTransformOp().Set(s * r * t)
    xf.GetPrim().GetReferences().AddReference(PIPE_USD)

n_pipe = 0
for p in stage.Traverse():
    if str(p.GetPath()).startswith("/World/Pipe/section_") and p.IsA(UsdGeom.Mesh):
        UsdPhysics.CollisionAPI.Apply(p)
        UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr("none")   # C1
        UsdShade.MaterialBindingAPI.Apply(p).Bind(
            UsdShade.Material.Get(stage, pipe_mat_path),
            bindingStrength=UsdShade.Tokens.weakerThanDescendants,
            materialPurpose="physics",
        )
        n_pipe += 1
if n_pipe == 0:
    raise RuntimeError("배관 Mesh 를 못 찾음")

# ══ 결함 프림 (C3) — collider 없음, displayColor 로 어둡게 ═════════
def radial_frame(angle_deg):
    """배관 단면(YZ)에서 각도 -> (반경방향 단위벡터 y,z)."""
    a = math.radians(angle_deg)
    return math.cos(a), math.sin(a)


UsdGeom.Xform.Define(stage, "/World/Defects")
gt_rows = []
for di, d in enumerate(DEFECTS):
    ry, rz = radial_frame(d["angle_deg"])
    base = f"/World/Defects/defect_{di}"
    UsdGeom.Xform.Define(stage, base)
    if d["kind"] == "crack":
        # 지그재그 3세그먼트: 축방향으로 길고(30mm) 접선방향 2mm, 내벽에서 1mm 돌출
        for si in range(3):
            seg = UsdGeom.Cube.Define(stage, f"{base}/seg_{si}")
            seg.CreateSizeAttr(1.0)
            seg.CreateDisplayColorAttr([Gf.Vec3f(0.02, 0.02, 0.02)])
            m = Gf.Matrix4d(1.0)
            m.SetScale(Gf.Vec3d(0.030, 0.002, 0.001))          # X:길이 Y:접선 Z:반경
            rot = Gf.Matrix4d(1.0)
            # 반경방향(로컬 Z)을 (0, ry, rz) 로 돌린다
            rot.SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), Gf.Vec3d(0.0, ry, rz)))
            tt = Gf.Matrix4d(1.0)
            r_pos = PIPE_IN_R_M - 0.0005
            tang = (-rz, ry)                                    # 접선 방향
            zig = (si - 1) * 0.004                              # ±4mm 지그재그
            tt.SetTranslate(Gf.Vec3d(
                d["x_m"] + (si - 1) * 0.028,
                r_pos * ry + zig * tang[0],
                r_pos * rz + zig * tang[1]))
            UsdGeom.Xformable(seg.GetPrim()).AddTransformOp().Set(m * rot * tt)
        size_note = "84x10mm zigzag"
    else:
        hole = UsdGeom.Cylinder.Define(stage, f"{base}/disc")
        hole.CreateRadiusAttr(0.008)
        hole.CreateHeightAttr(0.001)
        hole.CreateAxisAttr("Z")
        hole.CreateDisplayColorAttr([Gf.Vec3f(0.003, 0.003, 0.003)])
        rot = Gf.Matrix4d(1.0)
        rot.SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), Gf.Vec3d(0.0, ry, rz)))
        tt = Gf.Matrix4d(1.0)
        r_pos = PIPE_IN_R_M - 0.0003
        tt.SetTranslate(Gf.Vec3d(d["x_m"], r_pos * ry, r_pos * rz))
        UsdGeom.Xformable(hole.GetPrim()).AddTransformOp().Set(rot * tt)
        size_note = "d16mm disc"
    gt_rows.append([di, d["kind"], f"{d['x_m'] * 1000:.0f}", f"{d['angle_deg']:.0f}",
                    size_note])

with open(OUT_DIR / "ground_truth.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["id", "kind", "x_mm", "angle_deg", "size"])
    w.writerows(gt_rows)

# ══ 로봇 + 카메라 + 조명 ══════════════════════════════════════════
START_X = 0.5 * ROBOT_LEN_M + 0.02
m = Gf.Matrix4d(1.0)
m.SetRotate(Gf.Rotation(Gf.Vec3d(0, 1, 0), 90.0))
t = Gf.Matrix4d(1.0)
t.SetTranslate(Gf.Vec3d(START_X, 0.0, 0.0))
UsdGeom.Xform.Define(stage, "/World/RobotRoot").AddTransformOp().Set(m * t)
add_reference_to_stage(usd_path=ROBOT_USD, prim_path="/World/RobotRoot/Robot")

BODY = "/World/RobotRoot/Robot/Robot/body"
# 카메라 마운트: 본체 로컬 +Z(전방) 62mm. 질량/충돌 없음 -> 주행 간섭 0 (C1)
mount = UsdGeom.Xform.Define(stage, BODY + "/cam_mount")
# Isaac Camera 클래스의 시선은 프림 로컬 +X 다 (USD -Z 아님 — 색큐브 프로브로 실측).
# 본체 로컬 전방은 +Z 이므로 rotY(-90) 으로 +X -> +Z.
mm_ = Gf.Matrix4d(1.0)
mm_.SetRotate(Gf.Rotation(Gf.Vec3d(0, 1, 0), -90.0))
tt_ = Gf.Matrix4d(1.0)
tt_.SetTranslate(Gf.Vec3d(0.0, 0.0, CAM_OFF_M))
mount.AddTransformOp().Set(mm_ * tt_)

# 조명(C2): 카메라 바로 뒤. 배관 내부 유일 광원
lamp = UsdLux.SphereLight.Define(stage, BODY + "/cam_mount/lamp")
lamp.CreateRadiusAttr(0.004)
lamp.CreateIntensityAttr(3.0e7)   # 3e5 에서 중앙값 3/255 실측 -> 100배
UsdGeom.Xformable(lamp.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.008))

art = SingleArticulation(prim_path="/World/RobotRoot/Robot/Robot", name="pipe_robot")
world.scene.add(art)
world.reset()

# 파지 초기화 (levelup 데모에서 확립한 순서 그대로)
dof = list(art.dof_names)
leg_idx = [k for k, n in enumerate(dof) if n.startswith("joint_leg_")]
wheel_idx = [k for k, n in enumerate(dof) if n.startswith("joint_wheel_")]
init = art.get_joint_positions()
grip_m = PIPE_IN_R_M - WHEEL_OUT_M - 0.0001
for k in leg_idx:
    init[k] = grip_m
art.set_joint_positions(init)
free_len = LEG_HI_M + 0.010 * ROBOT_SCALE
for k in leg_idx:
    jp = stage.GetPrimAtPath(f"/World/RobotRoot/Robot/Robot/{dof[k]}")
    UsdPhysics.DriveAPI.Get(jp, "linear").GetTargetPositionAttr().Set(free_len)

camera = Camera(prim_path=BODY + "/cam_mount/camera", resolution=CAM_RES)
camera.initialize()
# USD 기본 clippingRange 는 (1, 1e6)m — near 1m 라서 배관 벽(25~60mm)이 전부
# 잘려 프레임이 완전 검정이 된다 (실측). 근접 촬영용으로 좁힌다.
camera.set_clipping_range(0.003, 10.0)

body = SingleRigidPrim(prim_path=BODY, name="body")

for _ in range(120):
    world.step(render=True)          # 카메라 워밍업 겸 정착

if not HEADLESS:
    set_camera_view(eye=[START_X - 0.3, -0.35, 0.2], target=[0.6, 0.0, 0.0])

# ══ 탐지 파이프라인 ═══════════════════════════════════════════════
# 전방 카메라 화면: 중앙 = 먼 어둠(항상 어두움), 주변부 = 가까운 벽(밝음).
# 따라서 중앙 원판은 제외하고 고리(annulus) 영역만 본다.
H, W = CAM_RES[1], CAM_RES[0]
yy, xx = np.mgrid[0:H, 0:W]
rr = np.hypot(yy - H / 2, xx - W / 2) / (W / 2)
# 안쪽 0.45: 화면 중앙의 '먼 어둠'(조명이 못 닿는 원거리)이 0.30 까지 새어 들어와
# 캘리브 잡음 blob 이 7075px 로 튀었다(실측). 결함은 가까워지면 어차피 바깥 고리로 나온다.
ANNULUS = (rr > 0.45) & (rr < 0.92)

# C8: 캘리브레이션은 "결함 없는 배경" 통계여야 한다. 첫 실행에서 크랙이 캘리브 프레임에
# 이미 보여 잡음 최대 blob(477px)에 포함됐고, 그 2배로 잡은 최소 면적이 크랙 자체를
# 걸러버렸다. 시뮬레이션은 정답을 아므로 캘리브 동안만 결함을 숨긴다.
defects_prim = UsdGeom.Imageable(stage.GetPrimAtPath("/World/Defects"))
defects_prim.MakeInvisible()

def grab_gray():
    rgba = camera.get_rgba()
    if rgba is None or rgba.size == 0:
        return None, None
    img = rgba[:, :, :3]
    if img.dtype != np.uint8:
        img = (np.clip(img, 0, 1) * 255).astype(np.uint8)
    return img, cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)


print("\n" + "=" * 80)
print("배관 내부 결함 탐지 주행")
print(f"  결함(정답): " + ", ".join(
    f"#{r[0]} {r[1]}@X={r[2]}mm/{r[3]}deg" for r in gt_rows))
print(f"  기록: {OUT_DIR}")
print("=" * 80)

# ── 캘리브레이션: 출발 전 정지 상태에서 (C8) ──
# 처음엔 주행하면서 캘리브를 했더니, 결함을 숨겨 둔 캘리브 구간이 결함 #0 의
# 탐지 창(전방 260~530mm 를 지나는 로봇 X 0~290mm)과 겹쳐 #0 을 통째로 놓쳤다.
calib_medians, calib_blob_max = [], 0
for ci in range(CALIB_FRAMES):
    for _ in range(3):
        world.step(render=True)
    img, gray = grab_gray()
    if gray is None:
        continue
    calib_medians.append(float(np.median(gray[ANNULUS])))
    if ci in (0, CALIB_FRAMES - 1):
        cv2.imwrite(str(OUT_DIR / f"calib_{ci:02d}.png"),
                    cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    dark0 = (gray < 0.45 * max(calib_medians[-1], 1.0)) & ANNULUS
    n, labels0, stats0, _ = cv2.connectedComponentsWithStats(
        dark0.astype(np.uint8), connectivity=8)
    for k in range(1, n):
        if float(rr[labels0 == k].min()) < 0.465:   # 중앙 어둠 연장은 잡음 통계서 제외
            continue
        calib_blob_max = max(calib_blob_max, int(stats0[k, cv2.CC_STAT_AREA]))

calib_med = float(np.median(calib_medians))
if calib_med < 5.0:
    print(f"[FAIL] C2 위반 — 캘리브레이션 밝기 중앙값 {calib_med:.1f}/255. 조명 확인.")
    simulation_app.close()
    sys.exit(1)
thr = 0.45 * calib_med
area_min = max(40, 2 * calib_blob_max)
print(f"[C8] 정지 캘리브레이션 {CALIB_FRAMES}프레임(결함 숨김): 벽 밝기 중앙값 {calib_med:.1f}/255")
print(f"     -> 어두움 임계 {thr:.1f}, 최소 blob 면적 {area_min}px "
      f"(잡음 최대 {calib_blob_max}px 의 2배)")
defects_prim.MakeVisible()

# ── 주행 시작 ──
for k in wheel_idx:
    jp = stage.GetPrimAtPath(f"/World/RobotRoot/Robot/Robot/{dof[k]}")
    UsdPhysics.DriveAPI.Get(jp, "angular").GetTargetVelocityAttr().Set(SPIN_DEG_S)

detections = []          # C7 레코드
dt = world.get_physics_dt()
frame_i = 0

for step in range(1, DRIVE_STEP_CAP + 1):
    capture = (step % CAPTURE_EVERY == 0)
    world.step(render=capture or not HEADLESS)
    x_rob = float(body.get_world_pose()[0][0])
    if x_rob > END_X_M:
        break
    if not capture:
        continue

    img, gray = grab_gray()
    if gray is None:
        continue
    frame_i += 1

    dark = (gray < thr) & ANNULUS
    n, labels, stats, cents = cv2.connectedComponentsWithStats(
        dark.astype(np.uint8), connectivity=8)
    hits = []
    for k in range(1, n):
        if stats[k, cv2.CC_STAT_AREA] < area_min:
            continue
        # 화면 중앙의 먼 어둠(원거리/출구 밖)은 annulus 안쪽 경계에 붙은 채로 들어온다.
        # 안쪽 경계(0.45)에 닿아 있는 blob 은 중앙 어둠의 연장이므로 기각한다.
        if float(rr[labels == k].min()) < 0.465:
            continue
        hits.append(k)
    if hits:
        big = max(hits, key=lambda k: stats[k, cv2.CC_STAT_AREA])
        bx, by, bw, bh, ba = stats[big]
        detections.append({
            "frame": frame_i, "t_s": step * dt, "x_rob_mm": x_rob * 1000.0,
            "area_px": int(ba), "cx": float(cents[big][0]), "cy": float(cents[big][1]),
        })
        vis = img.copy()
        cv2.rectangle(vis, (bx, by), (bx + bw, by + bh), (255, 0, 0), 2)
        cv2.imwrite(str(OUT_DIR / f"det_f{frame_i:04d}_x{x_rob * 1000:.0f}mm.png"),
                    cv2.cvtColor(vis, cv2.COLOR_RGB2BGR))

x_end = float(body.get_world_pose()[0][0])

# ══ C7 기록 + 정답 대조 ═══════════════════════════════════════════
with open(OUT_DIR / "detections.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["frame", "sim_time_s", "robot_x_mm", "blob_area_px", "cx", "cy"])
    for d in detections:
        w.writerow([d["frame"], f"{d['t_s']:.3f}", f"{d['x_rob_mm']:.1f}",
                    d["area_px"], f"{d['cx']:.1f}", f"{d['cy']:.1f}"])

print("\n" + "=" * 80)
print(f"주행 종료 X={x_end * 1000:.0f}mm, 촬영 {frame_i}프레임, 탐지 {len(detections)}건")

# 탐지 이벤트(연속 프레임 묶음) -> 결함 위치 추정 = 마지막 목격 시 카메라 위치
events = []
for d in detections:
    if events and d["frame"] - events[-1]["last_f"] <= 3:
        events[-1]["last_f"] = d["frame"]
        events[-1]["last_x"] = d["x_rob_mm"]
        events[-1]["n"] += 1
    else:
        events.append({"first_f": d["frame"], "last_f": d["frame"],
                       "first_x": d["x_rob_mm"], "last_x": d["x_rob_mm"], "n": 1})

TOL_MM = 120.0
ok = True
print(f"\n탐지 이벤트 {len(events)}개 (추정 위치 = 마지막 목격 시 로봇 X + 카메라 오프셋):")
matched = set()
for e in events:
    # 마지막 목격 = 결함이 좁은 FOV(실측 ~24deg, 기본 focal 50mm) 가장자리를 벗어난 시점
    # -> 카메라 전방 약 190~200mm (이벤트 소실 거리 실측 182/206mm)
    est = e["last_x"] + (CAM_OFF_M + 0.19) * 1000.0
    best, best_d = None, 1e9
    for di, d in enumerate(DEFECTS):
        dd = abs(est - d["x_m"] * 1000.0)
        if dd < best_d:
            best, best_d = di, dd
    tag = f"-> 결함 #{best} (오차 {best_d:.0f}mm)" if best_d <= TOL_MM else "-> 대응 없음(오탐?)"
    if best_d <= TOL_MM:
        matched.add(best)
    e["matched"] = best_d <= TOL_MM
    print(f"  프레임 {e['first_f']}~{e['last_f']} ({e['n']}컷)  "
          f"로봇X {e['first_x']:.0f}~{e['last_x']:.0f}mm  추정 {est:.0f}mm  {tag}")

for di, d in enumerate(DEFECTS):
    if di in matched:
        print(f"[OK] 결함 #{di} ({d['kind']} @ {d['x_m'] * 1000:.0f}mm) 탐지됨")
    else:
        print(f"[FAIL] 결함 #{di} ({d['kind']} @ {d['x_m'] * 1000:.0f}mm) 미탐지 — "
              "fallback 없이 실패로 보고한다 (C8)")
        ok = False
n_fp = sum(1 for e in events if not e.get("matched"))
if n_fp:
    print(f"[경고] 정답과 대응 안 되는 이벤트 {n_fp}개 — 오탐 검토 필요")
print("=" * 80)

if not HEADLESS:
    print("\nGUI 실행 중 — 창을 닫으면 종료됩니다.")
    while simulation_app.is_running():
        world.step(render=True)

simulation_app.close()
