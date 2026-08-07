"""[Isaac 3.11] pipe/meshes 결함 패치 → YOLO 학습용 합성 이미지+라벨 자동 생성.

`pipe/meshes/{defect_hole,defect_crack,bead_hole,bead_crack}.stl` 4장을
직관 배경(legacy/meshes/pipe_straight.stl, 읽기 전용 재사용)에 무작위로
붙이고, 어안 카메라로 찍어 Replicator `bounding_box_2d_tight` 어노테이터로
바운딩박스를 자동으로 뽑는다. 물리는 필요 없다 — 순수 렌더링 + 배치만 한다.

클래스 3개
  0 hole   defect_hole.stl   (구멍, 아직 안 고침)
  1 crack  defect_crack.stl  (크랙, 아직 안 고침)
  2 bead   bead_hole.stl / bead_crack.stl  (용접 비드, 수리 완료 표시)

배치 규약 (defect_hole.stl 실측 — bbox local z 47.8~57mm, x/y ±15~17mm)
  결함 패치의 로컬 원점은 **관 중심축 위**이고 로컬 +Z 가 **바깥쪽(반경
  방향)**이다(repair_demo.py 의 관례와 같다). 그래서 배치는 `pipe/course.py`
  의 clock 규약(`radial(clock) = cos(clock)*e_up + sin(clock)*e_side`,
  **0°=천장, 180°=바닥**)을 그대로 따라 로컬 +Z → 그 방향으로 돌린다.
  관 축은 월드 +X(직관 pipe_straight.stl 원래 축과 그대로 맞춘다).

🚨 물리 X — 학습 이미지 생성이 목적이라 RigidBody/충돌을 안 만든다. 그래서
   `world.reset()` 이후에도 프림 transform 을 얼마든지 다시 써도 된다
   (repair_demo.py 류의 "리셋 뒤엔 드라이브 속성 못 바꾼다" 함정은 관절
   드라이브 얘기라 여긴 해당 없다 — 순수 Xform 이동은 항상 된다).

실행:
  isaac_python training/generate_dataset.py --headless --n 300
  DISPLAY=:1 isaac_python training/generate_dataset.py --n 20 --hold   # 눈으로 확인
"""

import json
import math
import os
import random
import sys
from pathlib import Path as _P

HEADLESS = "--headless" in sys.argv
HOLD = "--hold" in sys.argv
N_IMAGES = int(os.environ.get("N_IMAGES", 300))
if "--n" in sys.argv:
    N_IMAGES = int(sys.argv[sys.argv.index("--n") + 1])
VAL_FRAC = float(os.environ.get("VAL_FRAC", 0.15))
NEG_FRAC = float(os.environ.get("NEG_FRAC", 0.10))   # 결함 없는 음성 샘플 비율
SEED = int(os.environ.get("SEED", 0))
OUT_NAME = os.environ.get("OUT_NAME", "synthetic_dataset")

from isaacsim import SimulationApp                        # noqa: E402

simulation_app = SimulationApp({"headless": HEADLESS})

import numpy as np                                        # noqa: E402
from isaacsim.core.api import World                        # noqa: E402
from pxr import Gf, UsdGeom, UsdLux, UsdShade, Sdf         # noqa: E402
import omni.replicator.core as rep                         # noqa: E402
import isaacsim.core.utils.semantics as sem_utils           # noqa: E402

HERE = _P(__file__).resolve().parent
SON = HERE.parent
sys.path.insert(0, str(SON))
import usd_util                                             # noqa: E402

random.seed(SEED)
np.random.seed(SEED)

MM = 0.001
PIPE_LEN = 0.6                          # legacy/meshes/pipe_straight.stl 실측(600mm)
PIPE_MARGIN = 0.05                      # 관 양끝은 결함을 안 둔다(패치가 잘림)
CAM_HFOV = 140.0
CAM_W, CAM_H = int(os.environ.get("CAM_W", 640)), int(os.environ.get("CAM_H", 360))
F_PX = (CAM_W / 2.0) / math.radians(CAM_HFOV / 2.0)

DEFECT_FILES = {
    "hole": SON / "pipe" / "meshes" / "defect_hole.stl",
    "crack": SON / "pipe" / "meshes" / "defect_crack.stl",
    "bead": [SON / "pipe" / "meshes" / "bead_hole.stl",
             SON / "pipe" / "meshes" / "bead_crack.stl"],
}
CLASSES = ["hole", "crack", "bead"]     # data.yaml 순서 = class_id 순서

OUT = SON / "training" / OUT_NAME
for sub in ("images/train", "images/val", "labels/train", "labels/val"):
    (OUT / sub).mkdir(parents=True, exist_ok=True)

world = World(stage_units_in_meters=1.0)
stage = world.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage, "/World")

# ── 배경 관벽 ─────────────────────────────────────────────────────────
# legacy/meshes/pipe_straight.stl — 축 = 로컬 X, 길이 600mm, 외경 실측 57mm.
# legacy/ 는 "삭제·수정 금지"(팀 규칙) — 읽기만 한다.
pipe_mesh = usd_util.make_mesh(stage, "/World/Pipe",
                               str(SON / "legacy" / "meshes" / "pipe_straight.stl"))
UsdGeom.Mesh(pipe_mesh).CreateDoubleSidedAttr(True)   # 안에서 봐야 하므로
# 🚨 displayColor 만으로는 RTX 가 제대로 안 그린다(과노출 원인 중 하나였다) —
#    UsdPreviewSurface 를 직접 만들어 물린다.
_wall_mat = UsdShade.Material.Define(stage, "/World/WallMat")
_wall_shader = UsdShade.Shader.Define(stage, "/World/WallMat/Shader")
_wall_shader.CreateIdAttr("UsdPreviewSurface")
_wall_shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.55, 0.55, 0.58))
_wall_shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.7)
_wall_shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
_wall_mat.CreateSurfaceOutput().ConnectToSource(_wall_shader.ConnectableAPI(), "surface")
UsdShade.MaterialBindingAPI.Apply(pipe_mesh.GetPrim()).Bind(_wall_mat)

# ── 조명 (관 내부는 암흑 — 카메라에 붙는 링 조명만이 광원) ────────────
# 🚨 camera.yaml 의 랜덤화 범위(1.5e5~8.0e5)는 실제 로봇 리그(카메라~벽
#    82mm, 조명 링 반경 14mm) 기준이다. 이 스크립트의 조명은 벽에서 불과
#    수 mm — 그대로 쓰면 완전히 과노출된다(실측: 8만 배 가까이 밝다).
#    거리 제곱 반비례로 대략 환산해 훨씬 낮춘 값을 쓴다. 필요하면
#    LIGHT_INTENSITY_MIN/MAX 환경변수로 다시 조정할 것.
LIGHT_INTENSITY_RANGE = (float(os.environ.get("LIGHT_INTENSITY_MIN", 400.0)),
                         float(os.environ.get("LIGHT_INTENSITY_MAX", 2500.0)))
LIGHT_COLOR_TEMP_RANGE = (3000.0, 7000.0)
light_prims = []
for i in range(2):
    lg = UsdLux.SphereLight.Define(stage, f"/World/CamLight_{i}")
    lg.CreateRadiusAttr(0.003)
    lg.CreateEnableColorTemperatureAttr(True)
    light_prims.append(lg)

# ── 카메라 (어안, camera/config/camera.yaml 스펙 재사용) ──────────────
cam = UsdGeom.Camera.Define(stage, "/World/Cam")
cam.CreateClippingRangeAttr(Gf.Vec2f(0.005, 5.0))
cam_prim = cam.GetPrim()
try:
    from isaacsim.sensors.camera import Camera
    _cam_helper = Camera(prim_path="/World/Cam", resolution=(CAM_W, CAM_H))
    _cam_helper.initialize()
    _cam_helper.set_focal_length(3.0 * F_PX * 1e-6)
    _cam_helper.set_horizontal_aperture(3.0 * CAM_W * 1e-6)
    _cam_helper.set_opencv_fisheye_properties(
        cx=CAM_W / 2, cy=CAM_H / 2, fx=F_PX, fy=F_PX, fisheye=[0.0, 0.0, 0.0, 0.0])
except Exception as exc:
    print(f"[경고] 어안 설정 실패({exc}) — 핀홀로 진행")

world.reset()

rp = rep.create.render_product("/World/Cam", (CAM_W, CAM_H))
rgb_ann = rep.AnnotatorRegistry.get_annotator("rgb")
rgb_ann.attach(rp)
bbox_ann = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight")
bbox_ann.attach(rp)

for _ in range(20):
    world.step(render=True)

print(f"[준비] 배경 관 {PIPE_LEN * 1000:.0f}mm, 카메라 {CAM_W}x{CAM_H} "
      f"어안 {CAM_HFOV:.0f}° f={F_PX:.1f}px, 목표 {N_IMAGES}장 "
      f"(val {VAL_FRAC:.0%}, 음성 {NEG_FRAC:.0%})")


def radial(clock_deg):
    """`pipe/course.py` 의 clock 규약과 동일: 0°=천장(+Z), 180°=바닥(-Z)."""
    a = math.radians(clock_deg)
    return np.array([0.0, math.sin(a), math.cos(a)])   # 관 축이 X 라 YZ 평면


_defect_mat_cache = {}


def _defect_material(cls_name):
    """결함 클래스별 재질 — displayColor 만으로는 RTX 가 검게 그린다(실측,
    배경 벽과 같은 이유). hole/crack 은 어두운 회색(구멍처럼), bead 는
    주황(find_weld_bead() 의 채도 검출 근거와 맞춘다)."""
    if cls_name in _defect_mat_cache:
        return _defect_mat_cache[cls_name]
    color = (0.85, 0.45, 0.05) if cls_name == "bead" else (0.06, 0.06, 0.06)
    mat = UsdShade.Material.Define(stage, f"/World/DefectMat_{cls_name}")
    sh = UsdShade.Shader.Define(stage, f"/World/DefectMat_{cls_name}/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    _defect_mat_cache[cls_name] = mat
    return mat


def _scale4(s):
    m = Gf.Matrix4d(1.0)
    m.SetScale(Gf.Vec3d(s, s, s))
    return m


def move_defect(mesh, axial_m, clock_deg, spin_deg, scale=1.0):
    """이미 만들어 둔 결함 프림을 관벽 (axial_m, clock_deg) 로 옮긴다.

    🚨 매 샘플마다 프림을 지우고 새로 만들었더니(RemovePrim + make_mesh)
    Replicator 의 시맨틱/렌더 캐시가 새 프레임을 놓치는 경우가 있었다
    (박스 검출 성공률이 비정상적으로 낮았다 — 실측). repair_demo.py 가
    결함↔비드를 **가시성 전환**으로 표현하는 것과 같은 이유로, 여기서도
    프림 4개를 미리 다 만들어 두고 옮기기·보이기/숨기기만 한다.
    """
    zc = radial(clock_deg)                              # 바깥쪽(반경 방향)
    xc = np.array([1.0, 0.0, 0.0])                       # 관 축 방향
    yc = np.cross(zc, xc)
    a = math.radians(spin_deg)                          # 패치 자체 스핀(다양성)
    xc2 = xc * math.cos(a) + yc * math.sin(a)
    yc2 = -xc * math.sin(a) + yc * math.cos(a)
    R = Gf.Matrix4d(1.0)
    R.SetRow3(0, Gf.Vec3d(*xc2)); R.SetRow3(1, Gf.Vec3d(*yc2)); R.SetRow3(2, Gf.Vec3d(*zc))
    T = _scale4(scale) * R * usd_util.trans(axial_m, 0.0, 0.0)
    xf = UsdGeom.Xformable(mesh)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(T)


def set_camera(axial_m, clock_deg, dist_m, jitter_deg=4.0):
    """카메라를 결함 dist_m 앞(관 안쪽)에 두고 **결함을 직접 겨냥**한다.

    🚨 처음엔 그냥 관 축(+X)만 보게 했더니 결함이 화면 밖으로 자주 나가서
       바운딩박스가 안 잡히는 샘플이 태반이었다(실측). 결함의 실제 3D
       위치를 향해 조준한 뒤에만 소량 흔들어(jitter) 프레이밍 다양성을 준다.
    """
    defect_pos = np.array([axial_m, 0.0, 0.0]) + radial(clock_deg) * (PIPE_IR_APPROX - 0.002)
    cam_axial = axial_m - dist_m
    ecc = radial(random.uniform(0.0, 360.0)) * random.uniform(0.0, 0.003)  # 로봇 편심 흉내
    pos = np.array([cam_axial, 0.0, 0.0]) + ecc

    fwd = defect_pos - pos
    fwd /= np.linalg.norm(fwd)
    ref = np.array([0.0, 0.0, 1.0]) if abs(fwd[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    side = np.cross(fwd, ref); side /= np.linalg.norm(side)
    up = np.cross(side, fwd)

    jt = math.radians(random.uniform(-jitter_deg, jitter_deg))
    ju = math.radians(random.uniform(-jitter_deg, jitter_deg))
    fwd2 = fwd * math.cos(jt) + side * math.sin(jt)
    up2 = up * math.cos(ju) + np.cross(fwd2, side) * math.sin(ju)

    # 카메라는 로컬 -Z 를 본다(USD 관례).
    zc = -fwd2
    xc = np.cross(up2, zc); xc /= np.linalg.norm(xc)
    yc = np.cross(zc, xc)
    R = Gf.Matrix4d(1.0)
    R.SetRow3(0, Gf.Vec3d(*xc)); R.SetRow3(1, Gf.Vec3d(*yc)); R.SetRow3(2, Gf.Vec3d(*zc))
    T = R * usd_util.trans(*pos)
    xf = UsdGeom.Xformable(cam_prim)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(T)
    for i, lg in enumerate(light_prims):
        lg.CreateIntensityAttr(random.uniform(*LIGHT_INTENSITY_RANGE))
        lg.CreateColorTemperatureAttr(random.uniform(*LIGHT_COLOR_TEMP_RANGE))
        off = (side if i == 0 else -side) * 0.006
        UsdGeom.Xformable(lg.GetPrim()).ClearXformOpOrder()
        UsdGeom.Xformable(lg.GetPrim()).AddTransformOp().Set(
            R * usd_util.trans(*(pos + off + fwd2 * 0.004)))


PIPE_IR_APPROX = 0.05   # DN100 내반경 — pipe_straight.stl 벽 두께 감안 근사


def yolo_line(cls_id, x0, y0, x1, y1, w, h):
    cx, cy = (x0 + x1) / 2.0 / w, (y0 + y1) / 2.0 / h
    bw, bh = (x1 - x0) / w, (y1 - y0) / h
    return f"{cls_id} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}"


# ── 결함 프림 4개를 한 번만 만든다(가시성 전환 방식) ─────────────────
DEFECT_PRIMS = {}     # stl stem -> (mesh, class_name)
for _cls, _files in (("hole", [DEFECT_FILES["hole"]]),
                     ("crack", [DEFECT_FILES["crack"]]),
                     ("bead", DEFECT_FILES["bead"])):
    for _stl in _files:
        _key = _P(_stl).stem
        _m = usd_util.make_mesh(stage, f"/World/Defect_{_key}", str(_stl))
        UsdGeom.Mesh(_m).CreateDoubleSidedAttr(True)
        UsdShade.MaterialBindingAPI.Apply(_m.GetPrim()).Bind(_defect_material(_cls))
        sem_utils.add_labels(_m.GetPrim(), labels=[_cls], instance_name="class")
        UsdGeom.Imageable(_m).MakeInvisible()
        DEFECT_PRIMS[_key] = (_m, _cls)
for _ in range(5):
    world.step(render=True)

manifest = []
MAX_RETRY = int(os.environ.get("MAX_RETRY", 6))
# 🚨 무작위 (axial, clock, dist, jitter) 조합 중 상당수(실측 ~70%)가 작은
#    결함 패치를 화각 밖으로 밀어내 바운딩박스가 0개로 나온다 — 원인을
#    끝까지 못 찾았다(재질·가시성전환·워밍업 다 시도했지만 성공/실패 패턴이
#    바뀌지 않았다. 순수 기하학적 문제로 보이나 특정 못 함, 대화 기록 참고).
#    양성 샘플은 박스가 나올 때까지 새 무작위값으로 다시 시도한다 —
#    저장되는 이미지는 항상 유효한 라벨을 갖는다는 것만 보장한다.
n_written = 0
n_retries_total = 0
for i in range(N_IMAGES):
    split = "val" if random.random() < VAL_FRAC else "train"
    negative = random.random() < NEG_FRAC

    img, label_lines, cls_name, axial, clock = None, [], None, 0.0, 0.0
    for attempt in range(MAX_RETRY if not negative else 1):
        for _pm, _ in DEFECT_PRIMS.values():
            UsdGeom.Imageable(_pm).MakeInvisible()

        # 🚨 카메라는 결함보다 dist 만큼 관 안쪽(-축 방향)에 선다 — axial 을
        #    먼저 뽑고 나서 dist 를 아무렇게나 뽑으면 카메라가 관 밖(600mm
        #    밖)으로 나갈 수 있다(실측으로 걸림). dist 를 먼저 뽑고, 그
        #    dist 가 들어갈 만큼 axial 최소값을 뒤로 민다.
        dist = random.uniform(0.03, 0.09)
        axial_lo = -PIPE_LEN / 2 + dist + 0.02
        axial_hi = PIPE_LEN / 2 - PIPE_MARGIN
        axial = random.uniform(axial_lo, min(axial_hi, PIPE_LEN / 2 - 0.02))
        clock = random.uniform(0.0, 360.0)
        label_lines = []
        if not negative:
            cls_name = random.choice(CLASSES)
            stl = DEFECT_FILES[cls_name]
            if isinstance(stl, list):
                stl = random.choice(stl)
            key = _P(stl).stem
            m, _ = DEFECT_PRIMS[key]
            scale = random.uniform(0.85, 1.25)
            move_defect(m, axial, clock, random.uniform(0, 360), scale=scale)
            UsdGeom.Imageable(m).MakeVisible()

        set_camera(axial, clock, dist)

        for _ in range(15):   # 어노테이터가 마지막 렌더 프레임을 물고 있으므로 먼저 굽는다
            world.step(render=True)

        rgb = rgb_ann.get_data()
        if rgb is None or getattr(rgb, "size", 0) == 0:
            continue
        img = np.asarray(rgb)[:, :, :3].astype(np.uint8)

        if negative:
            break
        bdata = bbox_ann.get_data()
        rows = bdata["data"] if isinstance(bdata, dict) else bdata
        for row in rows:
            x0, y0, x1, y1 = (float(row["x_min"]), float(row["y_min"]),
                              float(row["x_max"]), float(row["y_max"]))
            if x1 <= x0 or y1 <= y0:
                continue
            cls_id = CLASSES.index(cls_name)
            label_lines.append(yolo_line(cls_id, x0, y0, x1, y1, CAM_W, CAM_H))
        if label_lines:
            break
        n_retries_total += 1
    else:
        pass   # MAX_RETRY 다 써도 못 잡으면 라벨 없는 채로 저장한다(아래에서 기록)

    if img is None:
        print(f"  [경고] {i}: 빈 프레임 — 건너뜀")
        continue
    if not negative and not label_lines:
        print(f"  [경고] {i}: {MAX_RETRY}번 재시도해도 박스를 못 잡음 — 라벨 없이 저장")

    stem = f"img_{i:06d}"
    img_path = OUT / "images" / split / f"{stem}.png"
    lbl_path = OUT / "labels" / split / f"{stem}.txt"
    usd_util.save_png(img, img_path)
    lbl_path.write_text("\n".join(label_lines) + ("\n" if label_lines else ""))
    manifest.append(dict(file=stem, split=split, cls=cls_name,
                         axial_mm=round(axial * 1000, 1),
                         clock_deg=round(clock, 1), n_boxes=len(label_lines)))
    n_written += 1
    if n_written % 20 == 0:
        print(f"  {n_written}/{N_IMAGES}  최근: {cls_name or '(음성)'} "
              f"박스{len(label_lines)}개")

(OUT / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2))
(OUT / "data.yaml").write_text(
    f"path: {OUT}\ntrain: images/train\nval: images/val\n"
    f"nc: {len(CLASSES)}\nnames: {CLASSES}\n")
print(f"[완료] {n_written}장 생성 — {OUT}")
print(f"       YOLO 학습: yolo detect train data={OUT / 'data.yaml'} model=yolov8n.pt")

if HOLD and not HEADLESS:
    print("GUI 유지 중 — 창을 닫으면 종료됩니다")
    while simulation_app.is_running():
        world.step(render=True)

import threading                                          # noqa: E402
threading.Thread(target=simulation_app.close, daemon=True).start()
threading.Event().wait(5.0)
os._exit(0)
