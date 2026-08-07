"""[Isaac 3.11] 카메라 2대 + 조명 부착, RGB·Depth·CameraInfo 를 ROS 2 로 발행.

구조 (한 프로세스 안에서 전부 일어난다)

    Isaac Sim RTX 렌더러
        └ Camera prim (OpenCV fisheye 왜곡)
            └ render product          ← 이 카메라가 그리는 버퍼 묶음
                ├ annotator "rgb"                 → uint8 HxWx4
                └ annotator "distance_to_camera"  → float32 HxW (m)
                        │
                        │ 시뮬 스텝마다 numpy 로 읽음
                        ▼
                    rclpy 퍼블리셔 (이 프로세스가 곧 ROS 노드)
                        ├ /front/rgb          Image or CompressedImage
                        ├ /front/depth        Image 32FC1
                        └ /front/camera_info  CameraInfo
                        │
                        ▼  DDS
                    PC2 의 pipe_condition_node

Depth 는 물리 센서가 아니라 **광선 추적 결과**다. 스테레오 매칭도 IR 프로젝터도
없고, 렌더러가 "이 픽셀의 광선이 얼마나 날아가서 무엇에 맞았나"를 그대로
돌려준다. 그래서 RealSense 의 최소 측정 거리 52cm 같은 제약이 없고 25mm 도
측정된다. 반대로 **실물 센서보다 낙관적**이라는 뜻이기도 하다.

발행을 OmniGraph writer 가 아니라 rclpy 로 직접 하는 이유
  - 우리가 쓰는 양은 distance_to_image_plane 이 아니라 distance_to_camera 인데,
    rendervar writer 경로는 공식 문서에 DistanceToImagePlane 만 나와 있다.
  - CameraInfo writer 를 rgb/depth 와 같은 render product 에 붙이면 파이프라인이
    깨진다는 팀 실측이 있다. 직접 발행하면 그 충돌 자체가 없다.
  - annotator 를 numpy 로 받으므로 문제가 생겼을 때 어디서 끊겼는지 바로 보인다.

⚠ headless 에서는 카메라 프레임이 발행되지 않는다. 반드시 GUI 로 실행할 것.

⚠ `--save` 는 제거됐다. 예전에는 카메라를 붙인 상태를 `robot_2seg_cam.usd` 로
구워 두고 주행 쪽이 그것을 불러 썼는데, **그 파일 왕복이 세대 불일치·낡은 자산
사고의 근원**이었다(robot/assemble.py 머리말). 지금은 파일이 없다.

쓰는 법 (두 진입점)
  build_cameras(stage, robot)  조립 직후·`world.reset()` **전에** 카메라를 만든다.
                               주행 스크립트가 부른다.
  attach_existing(robot)       이미 만들어진 카메라 프림에 annotator 를 다시 붙인다.
                               annotator·render product 는 런타임 자원이라
                               `reset()` 뒤에 붙여야 한다.

자산 관계
  tools/build_parts.py     -> */meshes/*.stl + spec/parts_meta.json
  robot/assemble.py        -> 씬에 직접 조립 (파일 산출물 없음)
  pipe/curve_demo.py       -> 조립 + 카메라 + 주행 (Isaac 기동 1회)

실행:
  PYTHONUNBUFFERED=1 isaac_python camera/rig.py                    # 단독 확인 (GUI)
  PYTHONUNBUFFERED=1 isaac_python camera/rig.py --robot /World/Robot --ns /scout
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pyver import hard_exit, require_isaac

require_isaac(__file__, needs_rclpy=True)

HEADLESS = "--headless" in sys.argv
SAVE = "--save" in sys.argv
ROBOT = "/World/Robot"
NS = ""
if "--robot" in sys.argv:
    ROBOT = sys.argv[sys.argv.index("--robot") + 1]
if "--ns" in sys.argv:
    NS = sys.argv[sys.argv.index("--ns") + 1].rstrip("/")

from isaacsim import SimulationApp

# 다른 스크립트가 이 파일을 import 할 때(pipe_curve_demo.py --cameras) 는
# 그쪽이 이미 SimulationApp 을 만든 상태다. 두 번 만들면 실패하므로
# 직접 실행일 때만 생성한다.
simulation_app = None
if __name__ == "__main__":
    simulation_app = SimulationApp({"headless": HEADLESS})

import json
import math
import struct
import time
from pathlib import Path

import numpy as np
import yaml
from isaacsim.core.api import World
from isaacsim.sensors.camera import Camera
from pxr import Gf, UsdGeom, UsdLux

import omni.replicator.core as rep

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

HERE = Path(__file__).resolve().parent
SON = HERE.parent
CFG = yaml.safe_load((HERE / "config" / "camera.yaml").read_text())
CAM = CFG["camera"]
LIGHT = CFG["light"]

MM = 0.001
W, H = int(CAM["width"]), int(CAM["height"])
NEAR, FAR = CAM["clipping_range_m"]
HZ = float(CAM["publish_hz"])
HFOV = float(CAM["hfov_deg"])

# 등거리 어안 r = f*theta.  화면 가로 끝에서 theta = HFOV/2
F_PX = (W / 2.0) / math.radians(HFOV / 2.0)
PPX, PPY = W / 2.0, H / 2.0

# USD 카메라의 물리 파라미터. 픽셀 크기를 정하면 초점거리·센서폭이 따라 나온다.
PIXEL_SIZE_UM = 3.0
HORIZ_APERTURE_M = PIXEL_SIZE_UM * W * 1e-6
FOCAL_LENGTH_M = PIXEL_SIZE_UM * F_PX * 1e-6

if HEADLESS:
    print("[경고] headless 에서는 카메라 프레임이 발행되지 않는다. GUI 로 실행할 것.")


def load_stl(path):
    data = Path(path).read_bytes()
    n = struct.unpack("<I", data[80:84])[0]
    a = np.frombuffer(data[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    tri = a[:, 12:48].copy().view("<f4").reshape(n * 3, 3).astype(np.float64)
    pts, inv = np.unique(np.round(tri, 5), axis=0, return_inverse=True)
    return pts * MM, inv.reshape(n, 3)


# 하우징(깊이 8mm)을 센서 뒤로 물리는 거리. 4mm 면 앞면이 센서와 같은 평면이라
# 스치므로 1mm 여유를 더한다.
HOUSING_BACK_M = 5.0 * MM


def add_housing(stage, path, centre, flip):
    """RealSense 형태 하우징. 시각 전용이라 콜라이더를 걸지 않는다."""
    stl = HERE / "meshes" / "camera_housing.stl"
    if not stl.is_file():
        print(f"[참고] {stl} 없음 — 하우징 생략 (build_robot_parts.py 먼저 실행)")
        return
    pts, idx = load_stl(stl)
    if flip:
        pts = pts * np.array([-1.0, 1.0, 1.0])
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*centre))
    mesh = UsdGeom.Mesh.Define(stage, path + "/mesh")
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    mesh.CreateFaceVertexCountsAttr([3] * len(idx))
    mesh.CreateFaceVertexIndicesAttr(idx.reshape(-1).tolist())
    mesh.CreateExtentAttr([Gf.Vec3f(*pts.min(0)), Gf.Vec3f(*pts.max(0))])
    mesh.CreateSubdivisionSchemeAttr("none")


def add_lights(stage, base, centre, forward):
    """카메라 광축 둘레에 대칭 배치. 관 내부는 로봇 조명이 유일한 광원이다.
    intensity 3e5 로는 밝기 중앙값 3/255 라 이 스케일에서는 3e7 이 필요하다."""
    rr = LIGHT["ring_radius_mm"] * MM
    n = int(LIGHT["count_per_camera"])
    for i in range(n):
        a = 2.0 * math.pi * i / n
        light = UsdLux.SphereLight.Define(stage, f"{base}/light_{i}")
        light.CreateIntensityAttr(float(LIGHT["intensity"]))
        light.CreateRadiusAttr(LIGHT["radius_mm"] * MM)
        light.CreateColorTemperatureAttr(float(LIGHT["color_temp_k"]))
        light.CreateEnableColorTemperatureAttr(True)
        UsdGeom.Xformable(light).AddTranslateOp().Set(Gf.Vec3d(
            centre[0] + forward * 0.004,
            centre[1] + rr * math.cos(a),
            centre[2] + rr * math.sin(a)))


def make_camera(stage, name, spec):
    off = np.array(spec["offset_mm"], dtype=float) * MM
    centre = np.array([spec["mount_x_mm"] * MM, 0.0, 0.0]) + off
    forward = 1.0 if spec["axis"] == "+X" else -1.0

    base = f"{ROBOT}/{spec['link']}/{name}"
    UsdGeom.Xform.Define(stage, base)
    # 🚨 하우징을 센서와 같은 좌표에 두면 안 된다. 깊이 8mm 상자라 centre 를
    # 중심으로 ±4mm 를 감싸고 조명까지 그 안에 갇혀 렌즈 앞을 통째로 막는다.
    # 2026-08-04 실측: 하우징만 숨겨도 RGB 평균 0.00 → 91.42.
    # (Depth 는 불변이었다 — 시야를 막은 것은 본체 메시로 별개 결함이었다)
    add_housing(stage, f"{base}/housing",
                centre - np.array([forward * HOUSING_BACK_M, 0.0, 0.0]),
                forward < 0)
    add_lights(stage, base, centre, forward)

    cam = Camera(prim_path=f"{base}/sensor",
                 position=np.array(centre),
                 frequency=int(HZ),
                 resolution=(W, H))
    cam.initialize()
    # Isaac Camera 의 시선은 프림 로컬 +X. 후방 카메라만 180도 돌린다.
    if forward < 0:
        cam.set_local_pose(orientation=np.array([0.0, 0.0, 1.0, 0.0]),
                           camera_axes="usd")
    # near 를 관벽 25mm 보다 작게 두지 않으면 근접 벽면이 통째로 잘린다.
    cam.set_clipping_range(float(NEAR), float(FAR))

    # 어안. fisheyePolynomial 은 Isaac Sim 5.0 부터 폐기됐고 OmniLensDistortion
    # 스키마가 대체한다. Camera 클래스에서는 OpenCV fisheye 를 쓴다.
    # 왜곡 계수 0 이면 순수 등거리 r = f*theta 가 되어 오프라인 판정기 모델과
    # 정확히 일치한다. 1차 구현에서는 왜곡 보정 없이 이 상태로 간다.
    cam.set_focal_length(FOCAL_LENGTH_M)
    cam.set_horizontal_aperture(HORIZ_APERTURE_M)
    try:
        cam.set_opencv_fisheye_properties(cx=PPX, cy=PPY, fx=F_PX, fy=F_PX,
                                          fisheye=[0.0, 0.0, 0.0, 0.0])
    except Exception as exc:
        print(f"[경고] {name}: set_opencv_fisheye_properties 실패 ({exc}). "
              "핀홀로 동작한다 — 판정기 projection 도 pinhole 로 맞출 것")

    return dict(name=name, cam=cam, rp=None, rgb=None, depth=None, frame=name)


def attach_annotators(rig):
    """render product 에 rgb / distance_to_camera 를 붙인다.

    annotator 와 render product 는 런타임 자원이라 USD 에 저장되지 않는다.
    저장된 USD 를 불러 쓸 때도 이 함수를 다시 불러야 영상이 나온다.
    """
    rig["rp"] = rig["cam"].get_render_product_path()
    rig["rgb"] = rep.AnnotatorRegistry.get_annotator("rgb")
    rig["rgb"].attach(rig["rp"])
    rig["depth"] = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
    rig["depth"].attach(rig["rp"])
    return rig


def build_cameras(stage, robot_path=ROBOT):
    """씬에 카메라를 만들어 붙인다. 조립 직후·`world.reset()` 전에 부를 것.

    예전에는 `rig.py --save` 가 `robot_2seg_cam.usd` 를 구워 두고 주행 쪽이
    그것을 불러 썼다. 그 파일 왕복이 사고의 근원이라 없앴다 —
    이제 주행 스크립트가 조립 후 이 함수를 부른다.
    """
    return [make_camera(stage, "front_camera", CAM["front"]),
            make_camera(stage, "rear_camera", CAM["rear"])]


def attach_existing(robot_path=ROBOT):
    """이미 만들어진 카메라 프림에 annotator 를 붙인다.

    `build_cameras()` 로 카메라를 만든 뒤 `world.reset()` 을 지나고 나서
    호출한다 — annotator 와 render product 는 런타임 자원이라 reset 전에
    붙여 두면 살아남지 않는다.
    """
    rigs = []
    for name, spec in (("front_camera", CAM["front"]),
                       ("rear_camera", CAM["rear"])):
        path = f"{robot_path}/{spec['link']}/{name}/sensor"
        cam = Camera(prim_path=path, frequency=int(HZ), resolution=(W, H))
        cam.initialize()
        rigs.append(attach_annotators(
            dict(name=name, cam=cam, rp=None, rgb=None, depth=None,
                 frame=name)))
    return rigs


def topic_sets():
    t = CAM["topics"]
    return [dict(rgb=t["rgb"], depth=t["depth"], info=t["camera_info"]),
            dict(rgb=t["rear_rgb"], depth=t["rear_depth"],
                 info=t["rear_camera_info"])]


class CameraBridge(Node):
    """annotator 버퍼를 그대로 ROS 2 로 내보낸다."""

    def __init__(self, rigs, topics):
        super().__init__("isaac_camera_rig")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pubs = {}
        self.compressed = CAM["rgb_transport"] == "compressed"
        for rig, t in zip(rigs, topics):
            pre = f"{NS}/" if NS else ""
            self.pubs[rig["name"]] = dict(
                rgb=self.create_publisher(
                    CompressedImage if self.compressed else Image,
                    f"{pre}{t['rgb']}" + ("/compressed" if self.compressed else ""),
                    qos),
                depth=self.create_publisher(Image, f"{pre}{t['depth']}", qos),
                info=self.create_publisher(CameraInfo, f"{pre}{t['info']}", qos),
                frame=rig["frame"])
        self.n = 0

    def info_msg(self, stamp, frame):
        m = CameraInfo()
        m.header.stamp = stamp
        m.header.frame_id = frame
        m.width, m.height = W, H
        # 어안이라 K 는 엄밀한 핀홀 행렬이 아니다. K[0] 에 등거리 어안의
        # f (r = f*theta) 를 싣는 규약을 쓴다. 판정기가 그 값으로 입사각을 역산한다.
        m.k = [F_PX, 0.0, PPX, 0.0, F_PX, PPY, 0.0, 0.0, 1.0]
        m.p = [F_PX, 0.0, PPX, 0.0, 0.0, F_PX, PPY, 0.0, 0.0, 0.0, 1.0, 0.0]
        m.distortion_model = "equidistant"
        m.d = [0.0, 0.0, 0.0, 0.0]
        return m

    def publish(self, rig):
        p = self.pubs[rig["name"]]
        stamp = self.get_clock().now().to_msg()

        rgb = rig["rgb"].get_data()
        if rgb is not None and getattr(rgb, "size", 0):
            a = np.asarray(rgb)
            if a.ndim == 3 and a.shape[2] == 4:
                a = a[:, :, :3]
            a = np.ascontiguousarray(a.astype(np.uint8))
            if self.compressed:
                import cv2
                ok, buf = cv2.imencode(".jpg", a[:, :, ::-1],
                                       [cv2.IMWRITE_JPEG_QUALITY, 85])
                if ok:
                    m = CompressedImage()
                    m.header.stamp = stamp
                    m.header.frame_id = p["frame"]
                    m.format = "jpeg"
                    m.data = buf.tobytes()
                    p["rgb"].publish(m)
            else:
                m = Image()
                m.header.stamp = stamp
                m.header.frame_id = p["frame"]
                m.height, m.width = a.shape[0], a.shape[1]
                m.encoding = "rgb8"
                m.step = a.shape[1] * 3
                m.data = a.tobytes()
                p["rgb"].publish(m)

        d = rig["depth"].get_data()
        if d is not None and getattr(d, "size", 0):
            # 손실 압축은 깊이값을 훼손하므로 무압축 32FC1 로만 보낸다.
            a = np.ascontiguousarray(np.asarray(d, dtype=np.float32))
            m = Image()
            m.header.stamp = stamp
            m.header.frame_id = p["frame"]
            m.height, m.width = a.shape[0], a.shape[1]
            m.encoding = "32FC1"
            m.step = a.shape[1] * 4
            m.data = a.tobytes()
            p["depth"].publish(m)

        p["info"].publish(self.info_msg(stamp, p["frame"]))
        self.n += 1


def main():
    world = World(stage_units_in_meters=1.0)
    stage = world.stage
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/World")

    # 로봇은 씬에 직접 조립한다(robot/assemble.py). USD 파일을 거치지 않으므로
    # "로봇 없는 cam USD" 나 "낡은 로봇이 든 cam USD" 같은 사고가 성립하지 않는다.
    sys.path.insert(0, str(SON / "robot"))
    import assemble                                   # noqa: E402
    assemble.build(stage, ROBOT, verbose=False)

    rigs = build_cameras(stage)
    topics = topic_sets()

    world.reset()
    for _ in range(40):
        world.step(render=True)

    print("\n" + "=" * 76)
    print("카메라 리그" + ("  [저장 모드]" if SAVE else ""))
    print("=" * 76)
    print(f"  해상도 {W}x{H}   HFOV {HFOV:.0f}°   f {F_PX:.2f} px   "
          f"clipping {NEAR}~{FAR} m   {HZ:.0f} Hz")
    print(f"  초점거리 {FOCAL_LENGTH_M * 1000:.4f} mm  "
          f"센서폭 {HORIZ_APERTURE_M * 1000:.3f} mm  (픽셀 {PIXEL_SIZE_UM} um 가정)")
    for rig, tp in zip(rigs, topics):
        print(f"  {rig['name']:14s} {rig['cam'].prim_path}")

    for rig in rigs:
        attach_annotators(rig)
    for _ in range(30):
        world.step(render=True)

    print("-" * 76)
    ok = True
    for rig in rigs:
        r, d = rig["rgb"].get_data(), rig["depth"].get_data()
        rn = int(np.asarray(r).size) if r is not None else 0
        dn = int(np.asarray(d).size) if d is not None else 0
        good = rn > 0 and dn > 0
        ok = ok and good
        print(f"  {rig['name']:14s} rgb {rn:,} px   depth {dn:,} px   "
              f"{'OK' if good else 'FAIL — GUI 로 실행했는지 확인'}")
        if dn:
            a = np.asarray(d, dtype=np.float64)
            fin = np.isfinite(a) & (a > 0)
            print(f"      depth 유효 {int(fin.sum()):,}/{a.size:,}  "
                  + (f"범위 {a[fin].min():.4f}~{a[fin].max():.4f} m"
                     if fin.any() else "전부 무효"))

    out = HERE / "camera_intrinsics.json"
    out.write_text(json.dumps(dict(
        width=W, height=H, f_px=F_PX, ppx=PPX, ppy=PPY, hfov_deg=HFOV,
        model="equidistant_fisheye", clipping=[NEAR, FAR], hz=HZ), indent=2))
    print("-" * 76)
    print(f"  내부 파라미터 저장: {out}")
    print(f"  → condition/config 의 fallback_fx/ppx/ppy 와 일치해야 한다 "
          f"({F_PX:.1f} / {PPX:.0f} / {PPY:.0f})")

    # ── 저장 모드 ────────────────────────────────────────────────
    # --save 는 제거됐다. 예전에는 여기서 robot_2seg_cam.usd 를 구웠는데,
    # 그 파일 왕복이 세대 불일치·낡은 자산 사고의 근원이었다. 이제 주행
    # 스크립트가 조립 직후 build_cameras() 를 부른다.
    if SAVE:
        print("[중단] --save 는 제거됐다. USD 파일 왕복을 없앴다.")
        print("       주행 스크립트가 조립 후 rig.build_cameras() 를 부른다:")
        print("       PYTHONUNBUFFERED=1 isaac_python pipe/curve_demo.py --cameras")
        hard_exit(simulation_app)

    # ── 발행 모드 ────────────────────────────────────────────────
    rclpy.init()
    bridge = CameraBridge(rigs, topics)
    for rig, tp in zip(rigs, topics):
        pre = f"{NS}/" if NS else "/"
        print(f"  {rig['name']:14s} rgb → {pre}{tp['rgb']}"
              + ("/compressed" if bridge.compressed else ""))
        print(f"  {'':14s} depth → {pre}{tp['depth']}  (32FC1)")
        print(f"  {'':14s} info  → {pre}{tp['info']}")
    print("=" * 76)
    if not ok:
        print("\n[경고] 프레임을 못 받았다. headless 여부와 GUI 렌더를 확인할 것.\n")

    period = 1.0 / HZ
    last = 0.0
    try:
        while simulation_app.is_running():
            world.step(render=True)
            now = time.time()
            if now - last >= period:
                last = now
                for rig in rigs:
                    bridge.publish(rig)
                rclpy.spin_once(bridge, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"\n발행 종료 — 총 {bridge.n} 프레임")
        bridge.destroy_node()
        rclpy.shutdown()
        simulation_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
