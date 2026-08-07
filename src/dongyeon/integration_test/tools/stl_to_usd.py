"""[Isaac 3.11] STL -> USD 단순 변환기. 재질/조명/배치 없이 지오메트리 그대로 옮긴다.

    isaac_python tools/stl_to_usd.py <stl경로> [<stl경로> ...] [--outdir DIR]

기본 출력 위치: 각 STL과 같은 이름으로 training/ 아래(--outdir 로 변경 가능).
각 파일은 PascalCase 이름의 전용 루트 prim(예: /DefectCrack) 아래에
지오메트리(.../Geom)를 두고, 그 루트를 stage defaultPrim 으로 지정한다 —
다른 스테이지에서 references.AddReference() 로 가져다 쓸 때 어떤 prim이
붙는지 명확해진다.
"""
import sys
from pathlib import Path

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))
import usd_util

args = sys.argv[1:]
OUT_DIR = ROOT / "training"
if "--outdir" in args:
    i = args.index("--outdir")
    OUT_DIR = Path(args[i + 1]).expanduser()
    del args[i:i + 2]
OUT_DIR.mkdir(parents=True, exist_ok=True)


def _pascal(stem):
    return "".join(w.capitalize() for w in stem.split("_"))


stl_paths = [Path(a).expanduser() for a in args if not a.startswith("--")]
if not stl_paths:
    print("[오류] 변환할 STL 경로를 하나 이상 넘겨주세요.")
else:
    for stl_path in stl_paths:
        out_path = OUT_DIR / f"{stl_path.stem}.usd"
        prim_path = f"/{_pascal(stl_path.stem)}"

        stage = Usd.Stage.CreateNew(str(out_path))
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        root = UsdGeom.Xform.Define(stage, prim_path)
        stage.SetDefaultPrim(root.GetPrim())
        usd_util.make_mesh(stage, f"{prim_path}/Geom", str(stl_path))
        stage.GetRootLayer().Save()
        print(f"[완료] {stl_path.name} -> {out_path}  (prim_path={prim_path}, defaultPrim)")

import os
import threading
threading.Thread(target=simulation_app.close, daemon=True).start()
threading.Event().wait(3.0)
os._exit(0)
