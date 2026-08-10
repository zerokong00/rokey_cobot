"""[Isaac 3.11] T자 분기관 — 로봇 없이 카메라만 본관을 훑으며 **분기지점 판별**.

목적은 하나다: 카메라가 관 안을 이동하다가 분기가 나오면 **"여기는
분기지점이다"** 를 비전으로 확인하는 것. 위치 정밀 추정은 하지 않는다.
검출 시 분기가 갈라지는 **방향**(위/아래/왼쪽/오른쪽)도 함께 판별한다.

배관  test_pipe_tee_ID100.usda (워크스페이스 루트, **미터 단위**)
      본관: 축 X, x=-0.8→+0.25, 중심선 y=z=0, 내반경 50mm, 벽 6mm
      분기: x=0 에서 +Z 로 0.6m, 같은 ID100 → 개구는 본관 천장 ø100 = "위"
카메라 son 규약 그대로 — 어안 140° 등거리, 640x360, f=261.9px
      (zigzag/vision.py 와 동일 파라미터·동일 사원수. 전방 +X, 화면 위 +Z)
조명  카메라 리그에 SphereLight 2개 — 관 내부는 이것이 유일한 광원

## 검출 — 어둠 덩어리가 아니라 **기대 깊이 편차**다. 이유를 남긴다

1차 시도는 vision.py 의 어둠 덩어리 방식이었다. **실측 실패** (17지점 중
1지점만 검출): 분기 개구는 조명이 못 닿는 원거리에서 관 저 끝의 어둠과
**한 덩어리로 붙어** 중앙(관저끝) 제외 필터에 같이 쓸려나간다. 어둠 검출은
"밝은 벽에 둘러싸인 가까운 구멍" 전제라 ø100 원거리 개구에는 성립하지 않는다.

여기서는 본관이 직관 + 카메라가 중심선 위라는 기하를 그대로 쓴다:
  기대 벽 거리  t_exp = R / ρ   (ρ = 광선의 축직교 성분 — 해석적, 조명 무관)
  개구 화소     실측 깊이 > t_exp + 30mm
  관 저 끝 제외 기대 착벽점 x 가 관 범위 밖인 화소는 **애초에 후보가 아니다**
               (어둠 방식처럼 라벨 연결에 기대지 않고 화소 단위로 잘린다)
  분기 방향     개구 화소 평균 광선의 축직교 성분 (y,z) → 위/아래/왼쪽/오른쪽

🚨 vision.py 는 "깊이로 결함 위치를 정하지 말라" 고 남겼다 — 분기 **판별**은
   결함 정렬이 아니라서 그 두 근거(관통 화소의 inf, ±5mm 흩어짐 vs 허용치
   1.5mm)가 해당하지 않는다. 관통이 여기서는 오히려 신호다.

실측 (2026-08-07, 50mm 간격 17지점):
  분기 1.05m 앞(요구 40px 면적)부터 잡히기 시작, 0.5m 앞부터는 매 지점 연속
  검출, 개구 바로 밑(-0.05)까지 유지. 지나간 뒤(+0.00 이후) 오검출 0.

실행:
  PYTHONUNBUFFERED=1 isaac_python tee_branch_cam.py --headless
  PYTHONUNBUFFERED=1 isaac_python tee_branch_cam.py            # GUI

프레임은 _out/tee_cam/ 에 저장된다 (지점별: RGB 위 + 깊이 컬러맵 아래).
"""

import math
import sys
from pathlib import Path

HEADLESS = "--headless" in sys.argv
# --roll DEG: 배관을 본관 축(X) 둘레로 굴린다. 본관은 그대로, 분기만 돈다.
#   0=위(기본), 90=오른쪽, -90(=270)=왼쪽, 180=아래 — 전 방향 검출 검증용
ROLL = (float(sys.argv[sys.argv.index("--roll") + 1])
        if "--roll" in sys.argv else 0.0)

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": HEADLESS})

import cv2
import numpy as np
import omni.replicator.core as rep
from isaacsim.core.api import World
from isaacsim.sensors.camera import Camera
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade

WS = Path(__file__).resolve().parents[4]
PIPE_USD = WS / "test_pipe_tee_ID100.usda"
OUT = Path(__file__).resolve().parent / "_out" / (
    "tee_cam" if ROLL == 0.0 else f"tee_cam_roll{ROLL:+.0f}")
OUT.mkdir(parents=True, exist_ok=True)

# ── 배관 기하 (usda 실측 — 정점 파싱으로 확인한 값) ──────────────────
R_IN = 0.050                  # 내반경
BRANCH_X = 0.0                # 분기 중심 (정답, 판정에만 사용)
PIPE_X0, PIPE_X1 = -0.8, 0.25

# ── 카메라 제원 (son camera.yaml 규약 = zigzag/vision.py 와 동일) ────
CAM_HFOV = 140.0
W, H = 640, 360
F_PX = (W / 2.0) / math.radians(CAM_HFOV / 2.0)      # 261.9
PPX, PPY = W / 2.0, H / 2.0

# ── 주행 계획: 본관 중심선을 +X 로 훑는다 ────────────────────────────
SCAN_FROM, SCAN_TO, SCAN_STEP = -0.65, 0.15, 0.05

# ── 검출 임계 ────────────────────────────────────────────────────────
DEPTH_MARGIN = 0.030          # 실측 − 기대 (m). 깊이 흩어짐 ±5mm 의 6배
MIN_AREA = 40                 # 원거리에선 개구가 가는 호(弧)로 맺힌다
CLIP_FAR = 5.0

world = World(stage_units_in_meters=1.0)
stage = world.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

pipe = UsdGeom.Xform.Define(stage, "/World/Pipe")
pipe.AddRotateXOp().Set(ROLL)
pipe.GetPrim().GetReferences().AddReference(str(PIPE_USD))

if not HEADLESS:
    # 관전용 — 밖에서 배관이 보이게 돔라이트를 깐다. 검출은 깊이 기반이라
    # 조명과 무관하고, 관 내부 RGB 는 개구부로 새는 빛만 밝아진다
    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(800.0)
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=[0.7, -1.1, 0.7], target=[-0.15, 0.0, 0.15])

# ── 카메라 리그: Xform(이동) + 카메라(고정 자세) + 조명 2 ────────────
rig = UsdGeom.Xform.Define(stage, "/World/CamRig")
rig_t = rig.AddTranslateOp()
rig_t.Set(Gf.Vec3d(SCAN_FROM, 0.0, 0.0))
for k in range(2):
    lg = UsdLux.SphereLight.Define(stage, f"/World/CamRig/light_{k}")
    lg.CreateIntensityAttr(1.5e6)          # 시각 확인용 — 검출은 조명 무관
    lg.CreateRadiusAttr(0.002)
    UsdGeom.Xformable(lg).AddTranslateOp().Set(
        Gf.Vec3d(0.004, 0.012 * (1 if k == 0 else -1), 0.0))

if not HEADLESS:
    # 리그 위치 표식 — 카메라 프림은 지오메트리가 없어 GUI 에서 안 보인다.
    # 관 밖(중심 위 150mm)에 발광 구를 띄워 리그를 따라가게 한다.
    # 카메라 기준 정확히 90° 위라 시야각(반화각 70°) 밖 — 검출·영상 무영향
    mk = UsdGeom.Sphere.Define(stage, "/World/CamRig/marker")
    mk.CreateRadiusAttr(0.015)
    UsdGeom.Xformable(mk).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.15))
    mmat = UsdShade.Material.Define(stage, "/World/MarkerMat")
    msh = UsdShade.Shader.Define(stage, "/World/MarkerMat/sh")
    msh.CreateIdAttr("UsdPreviewSurface")
    msh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(4.0, 0.4, 0.2))
    mmat.CreateSurfaceOutput().ConnectToSource(msh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(mk.GetPrim()).Bind(mmat)

cam = Camera(prim_path="/World/CamRig/cam",
             translation=np.array([0.0, 0.0, 0.0]),
             frequency=10, resolution=(W, H))
world.reset()
cam.initialize()
# USD 카메라는 로컬 −Z 를 본다. 전방 +X, 화면 위 +Z 가 되는 사원수는
# (w,x,y,z) = (0.5, 0.5, −0.5, −0.5) — zigzag/vision.py 가 실측 대조한 값
cam.set_local_pose(orientation=np.array([0.5, 0.5, -0.5, -0.5]),
                   camera_axes="usd")
cam.set_clipping_range(0.005, CLIP_FAR)
cam.set_focal_length(3.0 * F_PX * 1e-6)
cam.set_horizontal_aperture(3.0 * W * 1e-6)
try:
    cam.set_opencv_fisheye_properties(cx=PPX, cy=PPY, fx=F_PX, fy=F_PX,
                                      fisheye=[0.0, 0.0, 0.0, 0.0])
except Exception as exc:
    print(f"[경고] 어안 설정 실패({exc}) — 핀홀로 진행")
rp = cam.get_render_product_path()
ann_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
ann_rgb.attach(rp)
# 관 내부는 방사형 — distance_to_camera 가 맞다 (vision.py 규약)
ann_dep = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
ann_dep.attach(rp)

# ── 자세 실측 대조 — 사원수 부호를 틀리면 조용히 죽는다(vision.py 교훈) ──
xc = UsdGeom.XformCache()
for _ in range(30 if HEADLESS else 8):
    world.step(render=True)
m = xc.GetLocalToWorldTransform(stage.GetPrimAtPath("/World/CamRig/cam"))
M = np.array([[m[i][j] for j in range(4)] for i in range(4)], dtype=float)
fwd = -M[2, :3] / np.linalg.norm(M[2, :3])
up = M[1, :3] / np.linalg.norm(M[1, :3])
e_fwd = math.degrees(math.acos(np.clip(fwd @ np.array([1.0, 0, 0]), -1, 1)))
e_up = math.degrees(math.acos(np.clip(up @ np.array([0, 0, 1.0]), -1, 1)))
print(f"카메라 자세 실측: 전방 오차 {e_fwd:.2f}°, 위쪽 오차 {e_up:.2f}°")
if e_fwd > 1.0 or e_up > 1.0:
    raise RuntimeError("카메라 자세가 틀렸다 — 사원수를 확인할 것")

# ── 화소별 광선 (월드 프레임) — 등거리 어안 역투영, 한 번만 계산 ─────
# 화면 오른쪽 −Y, 화면 위 +Z (화소 y 는 아래로 크므로 위 = −dy_px)
_px, _py = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
_dx_px, _dy_px = _px - PPX, _py - PPY
_r_px = np.hypot(_dx_px, _dy_px)
_theta = _r_px / F_PX
_s = np.where(_r_px > 1e-6, np.sin(_theta) / np.maximum(_r_px, 1e-6), 0.0)
RAY = np.stack([np.cos(_theta), -_dx_px * _s, -_dy_px * _s], axis=-1)
RHO = np.hypot(RAY[..., 1], RAY[..., 2])             # 축직교 성분
T_EXP = R_IN / np.maximum(RHO, 1e-6)                 # 기대 벽 거리
T_EXP = np.minimum(T_EXP, 2.0 * CLIP_FAR)


def frames(warm):
    """조건 없이 먼저 굽고 읽는다 — 낡은 프레임 함정(vision.py 실측)."""
    for _ in range(warm):
        world.step(render=True)
    c, z = ann_rgb.get_data(), ann_dep.get_data()
    rgb = (np.asarray(c)[:, :, :3].astype(np.uint8)
           if c is not None and getattr(c, "size", 0) else None)
    dep = (np.asarray(z, dtype=np.float32)
           if z is not None and getattr(z, "size", 0) else None)
    return rgb, dep


def branch_dir(vy, vz):
    """개구 평균 광선의 축직교 성분 → 갈라지는 방향. 전방 +X 기준
    왼쪽 = +Y, 오른쪽 = −Y, 위 = +Z, 아래 = −Z."""
    if abs(vz) >= abs(vy):
        return "위" if vz > 0 else "아래"
    return "왼쪽" if vy > 0 else "오른쪽"


# 정답 방향: 분기 +Z 를 롤만큼 돌린 것 — Rx(θ): (0,0,1) → (0, −sinθ, cosθ)
EXPECT_DIR = branch_dir(-math.sin(math.radians(ROLL)),
                        math.cos(math.radians(ROLL)))


def find_branch(dep, cam_x):
    """분기 개구 → dict 또는 None. 조명 무관 — 깊이 편차만 쓴다."""
    meas = np.where(np.isfinite(dep), dep, CLIP_FAR)
    # 기대 착벽점이 관 범위 안인 화소만 후보 — 관 저 끝(개구 아님)은 여기서
    # 화소 단위로 잘려 나가므로 어둠 방식의 라벨 연결 함정이 없다
    x_hit = cam_x + T_EXP * RAY[..., 0]
    valid = (x_hit > PIPE_X0 + 0.01) & (x_hit < PIPE_X1 - 0.01)
    mask = (valid & (meas > T_EXP + DEPTH_MARGIN)).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, st, ce = cv2.connectedComponentsWithStats(mask, 8)

    best = None
    for i in range(1, n):
        area = int(st[i, cv2.CC_STAT_AREA])
        if area < MIN_AREA:
            continue
        sel = lab == i
        vy = float(RAY[..., 1][sel].mean())
        vz = float(RAY[..., 2][sel].mean())
        gap = float(np.median(meas[sel] - T_EXP[sel]))
        # 개구까지 대략 거리 — 기대 착벽 거리의 중앙값 (보고용)
        dist = float(np.median(T_EXP[sel]))
        if best is None or area > best["area"]:
            best = {"area": area, "cx": float(ce[i][0]), "cy": float(ce[i][1]),
                    "gap": gap, "dist": dist, "dir": branch_dir(vy, vz),
                    "mask": sel}
    return best


def save_station(rgb, dep, hit, x):
    """RGB(위) + 깊이 시각화(아래) 세로 붙임 한 장."""
    vis_r = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    d = np.where(np.isfinite(dep), dep, CLIP_FAR)
    d8 = np.clip(d / 0.6 * 255.0, 0, 255).astype(np.uint8)
    vis_d = cv2.applyColorMap(d8, cv2.COLORMAP_TURBO)
    if hit is not None:
        cont, _ = cv2.findContours(hit["mask"].astype(np.uint8),
                                   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for v in (vis_r, vis_d):
            cv2.drawContours(v, cont, -1, (0, 255, 0), 2)
        cv2.putText(vis_r, f"BRANCH! dir={hit['dir']} ~{hit['dist']:.2f}m",
                    (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(vis_r, f"cam x={x:+.2f}m", (8, H - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    cv2.imwrite(str(OUT / f"station_{x:+.2f}.png"), np.vstack([vis_r, vis_d]))


print("=" * 70)
print("T자 분기관 카메라 단독 주행 — 분기지점 판별")
print(f"코스 x {SCAN_FROM:+.2f} → {SCAN_TO:+.2f} m (간격 {SCAN_STEP*1e3:.0f}mm), "
      f"롤 {ROLL:+.0f}° → 분기는 x={BRANCH_X:+.2f} 에서 '{EXPECT_DIR}' 방향이 정답")
print("=" * 70)
print("cam_x = 카메라 위치. ★ = 그 위치에서 **전방의** 분기를 인식했다는 뜻")
print(f"{'cam_x':>7} {'판별':>12} {'방향':>4} {'분기까지m':>8} "
      f"{'면적px':>7} {'깊이차m':>7}")

if not HEADLESS:
    # WebRTC 클라이언트가 붙을 시간을 준다 — 렌더는 계속 돌려야 스트림이 산다
    print("스캔 시작 전 15초 대기 (WebRTC 접속 시간)...")
    for _ in range(15 * 60):
        world.step(render=True)

results = []
x = SCAN_FROM
while x <= SCAN_TO + 1e-9:
    rig_t.Set(Gf.Vec3d(x, 0.0, 0.0))
    rgb, dep = frames(8 if HEADLESS else 4)
    hit = find_branch(dep, x) if dep is not None else None
    if hit:
        print(f"{x:+7.2f} {'★분기지점':>12} {hit['dir']:>4} {hit['dist']:>8.2f} "
              f"{hit['area']:>7d} {hit['gap']:>7.3f}")
    else:
        print(f"{x:+7.2f} {'직관':>12}")
    results.append((round(x, 3), hit))
    if rgb is not None and dep is not None:
        save_station(rgb, dep, hit, x)
    if not HEADLESS:
        for _ in range(45):              # 지점당 ~0.75s 머물러 눈으로 따라간다
            world.step(render=True)
    x += SCAN_STEP

# ── 합부 ─────────────────────────────────────────────────────────────
# ① 분기에 도달하기 전에 분기지점임을 인식했는가 (접근 중 연속 검출)
# ② 방향을 맞혔는가 ③ 분기를 지나 직관만 보일 때 오검출이 없는가
# "통과" 는 개구 뒷전(x=+50mm)을 지난 뒤부터다. 분기 폭 안(0~+50mm)에서는
# 옆 분기가 정당하게 계속 보인다 — 가로 반화각(70°)이 세로(39.4°)보다 커서
# 옆 개구는 위/아래 개구보다 더 늦게까지 시야에 남는다(실측)
approach = [(x, h) for x, h in results if x < BRANCH_X]
after = [(x, h) for x, h in results if x > BRANCH_X + 0.05 + 1e-9]
det = [(x, h) for x, h in approach if h]
first = det[0] if det else None
streak = 0
for _, h in reversed(approach):          # 분기 직전부터 거슬러 연속 검출 수
    if not h:
        break
    streak += 1
dirs_ok = all(h["dir"] == EXPECT_DIR for _, h in det)
false_after = [x for x, h in after if h]
print("-" * 70)
if first:
    print(f"최초 인식: cam x={first[0]:+.2f} (분기 {BRANCH_X - first[0]:.2f}m 앞), "
          f"이후 분기 직전까지 연속 {streak}지점 유지")
print(f"방향 판별: {f'전부 정답({EXPECT_DIR})' if dirs_ok and det else '오답 있음'}"
      f" / 통과 후 오검출: {false_after if false_after else '없음'}")
ok = first is not None and streak >= 3 and dirs_ok and not false_after
print(("[PASS] " if ok else "[FAIL] ") + f"분기지점 판별 — 프레임: {OUT}")
print("=" * 70)

if not HEADLESS:
    # 스캔이 끝나도 앱을 유지한다 — WebRTC 로 장면을 둘러볼 시간을 준다.
    # 종료는 터미널 Ctrl+C
    print("GUI 유지 중 — 뷰포트 카메라를 /World/CamRig/cam 으로 바꾸면 "
          "카메라 시점 그대로 보인다. 종료: Ctrl+C")
    try:
        while simulation_app.is_running():
            world.step(render=True)
    except KeyboardInterrupt:
        pass

simulation_app.close()
