"""[공용] 데모 공용 USD·자산 유틸 — repair_demo / real_map_demo 중복 제거.

두 데모에 글자까지 같게 복붙돼 있던 8개 함수를 합쳤다 (2026-08-06).
pxr/isaacsim 은 함수 안에서 import 한다(crack_inject 의 반쪽 3.11 방식) —
`load_stl`·`save_png` 는 순수 파이썬이라 3.10 오프라인에서도 돌고
`test_code/util/` 에서 시험한다.

전역에 기대던 것은 인자·팩토리로 풀었다:
  make_mesh(stage, ...)       ← stage 를 첫 인자로
  cam_prim_path(root, ...)    ← 로봇 루트 경로를 첫 인자로
  make_drive(art, wheel_idx)  ← 데모가 조립 후 한 번 묶어 drive() 를 얻는다
"""

import struct
from pathlib import Path

import numpy as np


def rot(deg, axis):
    """회전 행렬 Gf.Matrix4d."""
    from pxr import Gf
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(Gf.Vec3d(*axis), deg))
    return m


def trans(x, y, z):
    """평행이동 행렬 Gf.Matrix4d."""
    from pxr import Gf
    m = Gf.Matrix4d(1.0)
    m.SetTranslate(Gf.Vec3d(x, y, z))
    return m


def load_stl(path, scale=0.001):
    """바이너리 STL → (정점 [m], 삼각형 인덱스). 기본 scale=mm→m.

    같은 좌표 정점을 소수 5자리에서 합쳐 공유 정점 메시로 만든다.
    """
    data = Path(path).read_bytes()
    n = struct.unpack("<I", data[80:84])[0]
    a = np.frombuffer(data[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    tri = a[:, 12:48].copy().view("<f4").reshape(n * 3, 3).astype(np.float64)
    pts, inv = np.unique(np.round(tri, 5), axis=0, return_inverse=True)
    return pts * scale, inv.reshape(n, 3)


def make_mesh(stage, path, stl, color=None, xform=None, scale=0.001):
    """STL 을 UsdGeom.Mesh 로 저작. 변환은 transform op 하나로 건다."""
    from pxr import Gf, UsdGeom
    pts, idx = load_stl(stl, scale)
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    mesh.CreateFaceVertexCountsAttr([3] * len(idx))
    mesh.CreateFaceVertexIndicesAttr(idx.reshape(-1).tolist())
    mesh.CreateExtentAttr([Gf.Vec3f(*pts.min(0)), Gf.Vec3f(*pts.max(0))])
    mesh.CreateSubdivisionSchemeAttr("none")
    if color:
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if xform is not None:
        UsdGeom.Xformable(mesh).AddTransformOp().Set(xform)
    return mesh


def cam_prim_path(root, nm, seg):
    """카메라 리그 프림 경로 규약: <root>/<seg>/<nm>_rig/<nm>."""
    return f"{root}/{seg}/{nm}_rig/{nm}"


_XC = None      # XformCache — 첫 사용 때 만든다 (모듈 import 는 3.10 에서도 됨)


def wpos(prim):
    """프림 월드 위치 (numpy 3벡터). 매 호출 캐시를 비워 최신 자세를 읽는다."""
    global _XC
    from pxr import UsdGeom
    if _XC is None:
        _XC = UsdGeom.XformCache()
    _XC.Clear()
    t = _XC.GetLocalToWorldTransform(prim).ExtractTranslation()
    return np.array([float(t[0]), float(t[1]), float(t[2])])


def wmat(prim):
    """프림 월드 변환 행렬 (numpy 4x4, 행벡터 규약)."""
    global _XC
    from pxr import UsdGeom
    if _XC is None:
        _XC = UsdGeom.XformCache()
    _XC.Clear()
    m = _XC.GetLocalToWorldTransform(prim)
    return np.array([[m[i][j] for j in range(4)] for i in range(4)],
                    dtype=float)


def make_drive(art, wheel_idx):
    """휠 속도 지령 함수를 만든다: drive(deg_s) — 전 휠 동일 각속도.

    🚨 시뮬 시작 뒤에는 USD 드라이브 속성 쓰기가 PhysX 로 안 간다 —
       반드시 이 apply_action 경로를 쓸 것 (기록된 함정 ③).
    """
    import math
    from isaacsim.core.utils.types import ArticulationAction

    def drive(deg_s):
        art.apply_action(ArticulationAction(
            joint_velocities=np.array([math.radians(deg_s)]
                                      * len(wheel_idx)),
            joint_indices=np.array(wheel_idx)))
    return drive


def save_png(img, path):
    """PNG 저장. PIL 이 없으면 PPM 으로 물러난다(확장자 바꿔서 반환)."""
    try:
        from PIL import Image
        Image.fromarray(img).save(path)
        return path
    except Exception:
        path = path.with_suffix(".ppm")
        with open(path, "wb") as f:
            f.write(b"P6\n%d %d\n255\n" % (img.shape[1], img.shape[0]))
            f.write(img.tobytes())
        return path
