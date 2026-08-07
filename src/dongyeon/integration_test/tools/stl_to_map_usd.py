"""[Isaac 3.11] restroom_wpipe150.stl(통짜 메시) → real_map_demo.py 가

**플래그 없이** 바로 쓰는 `restroom_pipe150_final_fixed.usd` 로 변환한다.

왜 필요한가 — `real_map_demo.py`(원래 경로, `--map-stl` 없이)는 USD 참조 뒤
`f"/{FLOOR}/" in 경로` 로 floor2/floor1 콜리전을 가른다. STL 은 층 구분이
없는 삼각형 하나뿐이라 이 필터를 그냥은 통과 못 한다. 이 스크립트가 **같은
지오메트리를 `/Root/floor2/`와 `/Root/floor1/` 밑에 두 벌** 만들어 그 필터가
있는 그대로 통해도 각 층 실행에서 "이번 층" 콜리전을 잡게 만든다(중복 저장
이지만 STL 이 1.6MB 라 대수롭지 않다).

🚨 **이건 CAD 원본을 고친 `_fixed.usd`(tools/fix_map.py 산출물)와 다르다.**
   그쪽은 Sweep/PartBody 중복 배관을 프림 단위로 식별해 고치는데, 이 STL은
   그 정보(이름별 서브프림)가 아예 없다. 대신 실측(2026-08-06, 대화 기록)
   으로 `[관경]` 레이캐스트가 문서와 정확히 일치하는 것과 결함 투영 오차
   0.0~0.2mm 를 확인했다 — 중복 배관 결함의 흔적은 못 찾았지만, 진짜
   CATIA 원본을 고친 파일이 생기면 **이 파일을 지우고 그쪽으로 바꿀 것**
   (파일명이 같아서 그냥 덮어써도 된다).

산출물 이름을 `restroom_pipe150_final_fixed.usd` 로 고정한 이유 — real_map_
demo.py 의 `_MAP_FIXED` 우선순위 규칙이 그 이름을 최우선으로 찾는다. 그래야
아래 두 명령이 **아무 플래그 추가 없이** 그대로 된다:
    DISPLAY=:1 isaac_python real_map_demo.py --floor2 --glass --hold
    DISPLAY=:1 isaac_python real_map_demo.py --floor1 --hold --shots

실행 (pxr 는 Isaac 내장 것을 쓴다 — SimulationApp 기동만 하고 물리는 안 돈다):
    isaac_python tools/stl_to_map_usd.py
    isaac_python tools/stl_to_map_usd.py --stl ~/다른경로.stl --out ~/다른출력.usd
"""

import sys
from pathlib import Path as _P

STL_IN = str(_P.home() / "Downloads" / "restroom_wpipe150.stl")
USD_OUT = str(_P.home() / "Downloads" / "restroom_pipe150_final_fixed.usd")
if "--stl" in sys.argv:
    STL_IN = sys.argv[sys.argv.index("--stl") + 1]
if "--out" in sys.argv:
    USD_OUT = sys.argv[sys.argv.index("--out") + 1]

from isaacsim import SimulationApp                        # noqa: E402

simulation_app = SimulationApp({"headless": True})

from pxr import Gf, Usd, UsdGeom                          # noqa: E402

HERE = _P(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import usd_util                                            # noqa: E402

if not _P(STL_IN).is_file():
    print(f"[중단] STL 이 없다: {STL_IN}")
    simulation_app.close()
    sys.exit(1)

# scale=1.0 — raw mm 그대로 저장한다. real_map_demo.py 의 map_root 가
# AddScaleOp(0.001) 로 mm→m 변환을 하므로 여기서 미리 스케일하면 이중 변환된다
# (문서의 "USD 참조는 단위를 자동 변환하지 않는다" 함정과 같은 이유).
pts, idx = usd_util.load_stl(STL_IN, scale=1.0)
print(f"[준비] STL 로드 — 정점 {len(pts):,}개, 삼각형 {len(idx):,}개 (raw mm)")

stage = Usd.Stage.CreateNew(USD_OUT)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
root = UsdGeom.Xform.Define(stage, "/Root")
stage.SetDefaultPrim(root.GetPrim())


def _write_mesh(path):
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    m.CreateFaceVertexCountsAttr([3] * len(idx))
    m.CreateFaceVertexIndicesAttr(idx.reshape(-1).tolist())
    m.CreateExtentAttr([Gf.Vec3f(*pts.min(0)), Gf.Vec3f(*pts.max(0))])
    m.CreateSubdivisionSchemeAttr("none")
    return m


# 같은 지오메트리를 두 층 경로에 각각 저장한다(중복 저장, 6번 항목 참고).
_write_mesh("/Root/floor2/wpipe_mesh")
_write_mesh("/Root/floor1/wpipe_mesh")

stage.GetRootLayer().customLayerData = {
    "source": "stl_to_map_usd.py",
    "note": "restroom_wpipe150.stl 통짜 변환 — CAD 원본을 고친 정식 fixed.usd 아님",
}
stage.GetRootLayer().Save()
print(f"[완료] {USD_OUT} 저장 — /Root/floor2, /Root/floor1 에 각각 "
      f"{len(idx):,}개 삼각형")
print("       real_map_demo.py 가 --map-stl 없이도 이 파일을 기본으로 찾는다"
      "(_MAP_FIXED 우선순위)")

simulation_app.close()
