"""배관 체인(가변관 + 직관) 주행 + RGB/Depth 카메라 ROS2 발행.

배관 구성 (월드 X, 로봇은 +X 로 주행):
  X 0.000 ~ 0.950   pipe_levelup  (내경 190 -> 테이퍼 -> 95mm, 좁은 쪽이 +X)
  X 0.950 ~ 2.283   pipe x3       (내경 100mm 직관)
  경계(X=0.95)에서 내경 95 -> 100: 다리가 "바깥으로" 2.5mm 신장하는 방향이라
  턱에 걸리지 않는다 (반대 방향이면 2.5mm 단차를 기어올라야 해서 걸릴 수 있음).

카메라: 본체 앞끝 전방 주시 RGB + Depth(distance_to_image_plane).
  - in-process 로 depth 통계를 찍어 depth 가 실제로 나오는지 검증하고
  - replicator writer(ROS2PublishImage)를 카메라 render product 에 attach 해
    /rgb /depth 를 발행한다 (KB NOTES/05 §5 Python 방식). /camera_info 는
    writer 가 이미지 파이프라인을 깨는 문제로 발행하지 않고 K 를 출력만 한다.
    ※ ROS2CameraHelper(OmniGraph) 방식은 이 구성에서 빈 프레임만 발행됐다 — 코드 주석 참조.

ROS2 확인 (CLAUDE.md E1~E3):
  - Isaac Python 에선 rclpy 불가 -> 발행은 브릿지 writer 로만
  - 실행 전 반드시 isaac_ros (LD_LIBRARY_PATH). 안 하면 브릿지 기동 실패
  - ※ E2: headless 에서는 토픽은 보여도 프레임이 오지 않는다.
    **토픽 확인은 반드시 GUI 로 실행할 것.**
  - 다른 터미널: ros_set 후
      ros2 topic hz /rgb
      ros2 run pipe_inspect camera_check   (src/dongmin 의 패키지)
    isaac_ros 를 먼저 실행한 셸에서 이 스크립트를 띄워야 한다
    ROS_DOMAIN_ID 가 시뮬 쪽과 같아야 한다 (.bashrc 기본 143).

실행:
  PYTHONUNBUFFERED=1 isaac_python pipe_chain_ros_demo.py            # GUI (토픽 확인용)
  PYTHONUNBUFFERED=1 isaac_python pipe_chain_ros_demo.py --headless # 주행/depth 자체 검증
"""

import math
import sys
from pathlib import Path

HEADLESS = "--headless" in sys.argv

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": HEADLESS})

import os

# ROS2 브릿지는 Isaac 내장 humble 라이브러리를 dlopen 한다. LD_LIBRARY_PATH 는
# 프로세스 시작 시점에 로더가 읽으므로 여기서 고칠 수 없다 — 실행 전 셸에서
# isaac_ros (CLAUDE.md E1) 를 먼저 실행해야 한다. 안 하면 브릿지가 조용히 죽고
# 그래프 생성에서 "unrecognized type" 으로 터진다. 미리 검사해서 바로 알려준다.
_BRIDGE_LIB = ("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/"
               "exts/isaacsim.ros2.bridge/humble/lib")
if _BRIDGE_LIB not in os.environ.get("LD_LIBRARY_PATH", ""):
    print("\n[FAIL] LD_LIBRARY_PATH 에 Isaac 내장 humble lib 가 없다.")
    print("       실행 전에 다음을 먼저 하세요 (CLAUDE.md E1):")
    print("         isaac_ros")
    print(f"       (= export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:{_BRIDGE_LIB})\n")
    simulation_app.close()
    sys.exit(1)
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

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
LEVELUP_USD = str(HERE.parent / "pipe" / "pipe_levelup.usd")

# ── 로봇 (robot_articulated.py 와 같은 값) ──
ROBOT_SCALE = 90.0 / 302.0
WHEEL_OUT_M = (132.0 + 23.0) * 0.001 * ROBOT_SCALE
LEG_HI_M = 15.0 * 0.001 * ROBOT_SCALE
ROBOT_LEN_M = 393.5 * 0.001 * ROBOT_SCALE
CAM_OFF_M = 0.062

# ── 배관 체인 ──
LV_NARROW_MM = 95.0                       # levelup 좁은 구간 내경
LV_S = (LV_NARROW_MM / 40.0) * 0.001      # levelup CAD -> m
LV_LEN_M = 400.0 * LV_S                   # 0.95 m
P100_S = (100.0 / 45.0) * 0.001           # pipe.usd CAD -> m (내경 100mm)
P100_LEN_M = 200.0 * P100_S               # 0.4444 m
P100_N = 3
CHAIN_END_M = LV_LEN_M + P100_N * P100_LEN_M
PIPE_FRICTION = 0.7

SPIN_DEG_S = 1080.0
START_X = 0.70                            # levelup 좁은 구간(0.475~0.95) 안
END_X_M = CHAIN_END_M - 0.30              # 출구 어둠 전 정지
DRIVE_STEP_CAP = 2500
DEPTH_REPORT_EVERY = 120

world = World(stage_units_in_meters=1.0)
stage = world.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage, "/World")
# 배관 내부 리얼리즘: 외부 조명 없음 (C2). 로봇 조명이 유일 광원.

# ══ 배관 체인 ═════════════════════════════════════════════════════
pipe_mat_path = "/World/Pipe/PipeMaterial"
UsdGeom.Xform.Define(stage, "/World/Pipe")
pm = UsdPhysics.MaterialAPI.Apply(
    UsdShade.Material.Define(stage, pipe_mat_path).GetPrim())
pm.CreateStaticFrictionAttr(PIPE_FRICTION)
pm.CreateDynamicFrictionAttr(PIPE_FRICTION)
pm.CreateRestitutionAttr(0.0)


def add_pipe(path, usd, scale_r, scale_ax, x_offset):
    """CAD 축(+Y)을 월드 +X 로 눕혀 x_offset 에 배치. 반환: 붙인 Mesh 수."""
    xf = UsdGeom.Xform.Define(stage, path)
    s = Gf.Matrix4d(1.0)
    s.SetScale(Gf.Vec3d(scale_r, scale_ax, scale_r))
    r = Gf.Matrix4d(1.0)
    r.SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), -90.0))   # +Y -> +X (실측 확인된 부호)
    t = Gf.Matrix4d(1.0)
    t.SetTranslate(Gf.Vec3d(x_offset, 0.0, 0.0))
    xf.AddTransformOp().Set(s * r * t)
    xf.GetPrim().GetReferences().AddReference(usd)
    n = 0
    for p in stage.Traverse():
        if str(p.GetPath()).startswith(path) and p.IsA(UsdGeom.Mesh) \
                and not p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(p)
            UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr("none")  # C1
            UsdShade.MaterialBindingAPI.Apply(p).Bind(
                UsdShade.Material.Get(stage, pipe_mat_path),
                bindingStrength=UsdShade.Tokens.weakerThanDescendants,
                materialPurpose="physics",
            )
            n += 1
    return n


n_mesh = 0
# levelup: CAD Y=-200(넓음)..+200(좁음) -> X -0.475..+0.475, +0.475 이동으로 0..0.95
n_mesh += add_pipe("/World/Pipe/levelup", LEVELUP_USD, LV_S, LV_S, LV_LEN_M / 2.0)
# 100mm 직관 3개: pipe.usd 는 CAD Y=0..200 -> X 0..0.4444
for i in range(P100_N):
    n_mesh += add_pipe(f"/World/Pipe/p100_{i}", PIPE_USD, P100_S, P100_S,
                       LV_LEN_M + i * P100_LEN_M)
if n_mesh < P100_N + 1:
    raise RuntimeError(f"배관 collider {n_mesh}개 — 참조 실패 의심")

# ══ 결함 (100mm 구간, 시각 전용) — 카메라로 볼 거리 제공 ══════════
UsdGeom.Xform.Define(stage, "/World/Defects")
DEFECTS = [("crack", 1.35, 60.0), ("hole", 1.75, 250.0)]
for di, (kind, x_m, ang) in enumerate(DEFECTS):
    ry, rz = math.cos(math.radians(ang)), math.sin(math.radians(ang))
    base = f"/World/Defects/defect_{di}"
    UsdGeom.Xform.Define(stage, base)
    if kind == "crack":
        for si in range(3):
            seg = UsdGeom.Cube.Define(stage, f"{base}/seg_{si}")
            seg.CreateSizeAttr(1.0)
            seg.CreateDisplayColorAttr([Gf.Vec3f(0.02, 0.02, 0.02)])
            m = Gf.Matrix4d(1.0)
            m.SetScale(Gf.Vec3d(0.030, 0.002, 0.001))
            rot = Gf.Matrix4d(1.0)
            rot.SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), Gf.Vec3d(0.0, ry, rz)))
            tt = Gf.Matrix4d(1.0)
            zig = (si - 1) * 0.004
            tt.SetTranslate(Gf.Vec3d(x_m + (si - 1) * 0.028,
                                     0.0495 * ry - zig * rz,
                                     0.0495 * rz + zig * ry))
            UsdGeom.Xformable(seg.GetPrim()).AddTransformOp().Set(m * rot * tt)
    else:
        hole = UsdGeom.Cylinder.Define(stage, f"{base}/disc")
        hole.CreateRadiusAttr(0.008)
        hole.CreateHeightAttr(0.001)
        hole.CreateAxisAttr("Z")
        hole.CreateDisplayColorAttr([Gf.Vec3f(0.003, 0.003, 0.003)])
        rot = Gf.Matrix4d(1.0)
        rot.SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), Gf.Vec3d(0.0, ry, rz)))
        tt = Gf.Matrix4d(1.0)
        tt.SetTranslate(Gf.Vec3d(x_m, 0.0497 * ry, 0.0497 * rz))
        UsdGeom.Xformable(hole.GetPrim()).AddTransformOp().Set(rot * tt)

# ══ 로봇 + 카메라 + 조명 ══════════════════════════════════════════
m = Gf.Matrix4d(1.0)
m.SetRotate(Gf.Rotation(Gf.Vec3d(0, 1, 0), 90.0))
t = Gf.Matrix4d(1.0)
t.SetTranslate(Gf.Vec3d(START_X, 0.0, 0.0))
UsdGeom.Xform.Define(stage, "/World/RobotRoot").AddTransformOp().Set(m * t)
add_reference_to_stage(usd_path=ROBOT_USD, prim_path="/World/RobotRoot/Robot")

BODY = "/World/RobotRoot/Robot/Robot/body"
mount = UsdGeom.Xform.Define(stage, BODY + "/cam_mount")
mm_ = Gf.Matrix4d(1.0)
mm_.SetRotate(Gf.Rotation(Gf.Vec3d(0, 1, 0), -90.0))   # Isaac Camera 시선 = 로컬 +X
tt_ = Gf.Matrix4d(1.0)
tt_.SetTranslate(Gf.Vec3d(0.0, 0.0, CAM_OFF_M))
mount.AddTransformOp().Set(mm_ * tt_)

lamp = UsdLux.SphereLight.Define(stage, BODY + "/cam_mount/lamp")
lamp.CreateRadiusAttr(0.004)
lamp.CreateIntensityAttr(3.0e7)
UsdGeom.Xformable(lamp.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.008))

art = SingleArticulation(prim_path="/World/RobotRoot/Robot/Robot", name="pipe_robot")
world.scene.add(art)
world.reset()

dof = list(art.dof_names)
leg_idx = [k for k, n in enumerate(dof) if n.startswith("joint_leg_")]
wheel_idx = [k for k, n in enumerate(dof) if n.startswith("joint_wheel_")]
init = art.get_joint_positions()
grip_m = LV_NARROW_MM * 0.5 * 0.001 - WHEEL_OUT_M - 0.0001
for k in leg_idx:
    init[k] = grip_m
art.set_joint_positions(init)
free_len = LEG_HI_M + 0.010 * ROBOT_SCALE
for k in leg_idx:                          # set_joint_positions 가 target 을 덮어씀
    jp = stage.GetPrimAtPath(f"/World/RobotRoot/Robot/Robot/{dof[k]}")
    UsdPhysics.DriveAPI.Get(jp, "linear").GetTargetPositionAttr().Set(free_len)

CAM_PRIM = BODY + "/cam_mount/camera"
camera = Camera(prim_path=CAM_PRIM, resolution=(320, 320))
camera.initialize()
camera.set_clipping_range(0.003, 10.0)     # 기본 near 1m — 배관 벽이 전부 잘린다
camera.add_distance_to_image_plane_to_frame()   # depth (m 단위)

body = SingleRigidPrim(prim_path=BODY, name="body")

# ══ ROS2 발행: replicator writer 를 카메라 render product 에 직접 attach ══
# (KB NOTES/05 §5 "Python 방식")
#
# ROS2CameraHelper(OmniGraph) 방식은 두 가지로 다 실패했다:
#   - IsaacCreateRenderProduct 로 새 RP 생성: 토픽 70~90Hz 로 흐르지만 내용 전부 0
#   - in-process Camera 의 RP 재사용:        마찬가지로 전부 0
#   같은 순간 in-process get_rgba() 는 밝기 ~180 으로 정상이었으므로 렌더는 문제가
#   없고, helper 의 SDG 배선이 데이터를 못 받는 것이다
#   (로그의 "SdRenderVarPtr missing valid input renderVar ...SDhost" 경고와 일치).
import omni.replicator.core as rep
import omni.syntheticdata
import omni.syntheticdata._syntheticdata as sd

RP_PATH = str(camera._render_product_path)

rv_rgb = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
    sd.SensorType.Rgb.name)
w_rgb = rep.writers.get(rv_rgb + "ROS2PublishImage")
w_rgb.initialize(frameId="inspect_cam", nodeNamespace="", queueSize=1,
                 topicName="rgb")
w_rgb.attach([RP_PATH])

rv_depth = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
    sd.SensorType.DistanceToImagePlane.name)
w_depth = rep.writers.get(rv_depth + "ROS2PublishImage")
w_depth.initialize(frameId="inspect_cam", nodeNamespace="", queueSize=1,
                   topicName="depth")
w_depth.attach([RP_PATH])

# camera_info: 토픽으로 발행하지 않는다.
# 실측 — ROS2PublishCameraInfo writer 를 같은 render product 에 attach 하면
# rgb/depth 파이프라인이 깨져 빈 프레임이 발행된다 (rgb 가 뷰포트 값으로 바뀌고
# depth 무효화). depth -> 3D 복원에 필요한 내부 파라미터는 아래처럼 출력만 한다.
try:
    from isaacsim.ros2.bridge import read_camera_info

    _ci = read_camera_info(render_product_path=RP_PATH)
    if isinstance(_ci, (tuple, list)):
        _ci = _ci[0]
    _K = list(_ci.k)
    print(f"[카메라 내부 파라미터] {_ci.width}x{_ci.height}  "
          f"fx={_K[0]:.1f} fy={_K[4]:.1f} cx={_K[2]:.1f} cy={_K[5]:.1f}")
except Exception as exc:  # noqa: BLE001
    print(f"[경고] camera_info 조회 실패 ({type(exc).__name__}: {exc})")
print("[OK] ROS2 발행 writer 부착: /rgb /depth")
if HEADLESS:
    print("    ※ E2: headless 에서는 프레임이 발행되지 않는다. 토픽 확인은 GUI 로!")

for _ in range(120):
    world.step(render=True)

if not HEADLESS:
    set_camera_view(eye=[START_X - 0.3, -0.4, 0.25], target=[1.2, 0.0, 0.0])

for k in wheel_idx:
    jp = stage.GetPrimAtPath(f"/World/RobotRoot/Robot/Robot/{dof[k]}")
    UsdPhysics.DriveAPI.Get(jp, "angular").GetTargetVelocityAttr().Set(SPIN_DEG_S)

print("\n" + "=" * 78)
print("배관 체인 주행 (95mm 가변관 -> 100mm 직관, RGB/Depth 발행 중)")
print(f"  체인: levelup 0~{LV_LEN_M:.2f}m + 100mm x{P100_N} ~{CHAIN_END_M:.2f}m")
print(f"  결함: crack@1.35m, hole@1.75m")
print("=" * 78)
print(f"{'스텝':>6} {'X위치':>8} {'다리':>9} {'RGB평균':>8} "
      f"{'depth중앙':>10} {'depth최소':>10}")

ok = True
for step in range(1, DRIVE_STEP_CAP + 1):
    world.step(render=True)               # ROS 발행은 렌더 프레임에 실림
    x_rob = float(body.get_world_pose()[0][0])
    if x_rob > END_X_M:
        break
    if step % DEPTH_REPORT_EVERY:
        continue
    pos = art.get_joint_positions()
    leg = sum(float(pos[k]) for k in leg_idx) / 3.0
    frame = camera.get_current_frame()
    rgba = camera.get_rgba()
    rgb_mean = float(np.asarray(rgba)[:, :, :3].mean()) if rgba is not None else -1
    depth = frame.get("distance_to_image_plane") if frame else None
    if depth is not None and np.asarray(depth).size:
        d = np.asarray(depth, dtype=np.float32)
        finite = d[np.isfinite(d) & (d > 0)]
        d_med = float(np.median(finite)) if finite.size else float("nan")
        d_min = float(finite.min()) if finite.size else float("nan")
    else:
        d_med = d_min = float("nan")
    print(f"{step:>6} {x_rob * 1000:>6.0f}mm {leg * 1000:>+7.3f}mm {rgb_mean:>8.1f} "
          f"{d_med:>9.4f}m {d_min:>9.4f}m")

x_end = float(body.get_world_pose()[0][0])
pos = art.get_joint_positions()
leg_end = sum(float(pos[k]) for k in leg_idx) / 3.0

print("=" * 78)
print(f"최종 X = {x_end * 1000:.0f} mm")
if x_end < LV_LEN_M + 0.1:
    print("[FAIL] 95->100mm 경계를 통과하지 못했다")
    ok = False
else:
    print(f"[OK] 경계(950mm) 통과, 100mm 구간 주행 (다리 {leg_end * 1000:+.3f} mm)")

frame = camera.get_current_frame()
depth = frame.get("distance_to_image_plane") if frame else None
if depth is None or not np.asarray(depth).size:
    print("[FAIL] depth 프레임이 비어 있다 — annotator 확인")
    ok = False
else:
    d = np.asarray(depth, dtype=np.float32)
    finite = d[np.isfinite(d) & (d > 0)]
    print(f"[OK] depth {d.shape} 유효픽셀 {finite.size}/{d.size}, "
          f"범위 {finite.min():.4f}~{finite.max():.4f} m")
print("=" * 78)

if not HEADLESS:
    print("\n다른 터미널에서 확인:")
    print("  ros_set   (source /opt/ros/humble/setup.bash)")
    print("  ros2 topic list          # /rgb /depth")
    print("  ros2 topic hz /rgb")
    print("  ros2 run pipe_inspect camera_check")
    print("GUI 실행 중 — 창을 닫으면 종료됩니다.")
    while simulation_app.is_running():
        world.step(render=True)

simulation_app.close()
