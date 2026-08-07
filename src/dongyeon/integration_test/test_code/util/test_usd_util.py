"""[오프라인] 공용 유틸(usd_util.py) 중 순수 파이썬 부분 검증.

pxr 이 필요한 함수(rot/trans/make_mesh/wpos/wmat/make_drive)는 Isaac 에서만
돌므로, 여기서는 3.10 으로 되는 것만 본다: 모듈 import 자체(지연 import 가
깨지지 않았는가) + load_stl + save_png.

실행:  python3 test_usd_util.py
"""

import struct
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SON = HERE.parents[1]
sys.path.insert(0, str(SON))

import usd_util  # noqa: E402


def tiny_stl(path):
    """정사각형(삼각형 2개, 정점 4개 공유) STL 을 만든다. 단위 mm."""
    tris = [((0, 0, 0), (10, 0, 0), (10, 10, 0)),
            ((0, 0, 0), (10, 10, 0), (0, 10, 0))]
    with open(path, "wb") as f:
        f.write(b"\0" * 80)
        f.write(struct.pack("<I", len(tris)))
        for t in tris:
            f.write(struct.pack("<3f", 0, 0, 1))
            for v in t:
                f.write(struct.pack("<3f", *v))
            f.write(struct.pack("<H", 0))


def main():
    ok = []

    def check(name, cond, extra=""):
        ok.append(bool(cond))
        print(f"  [{len(ok):2d}] {name:<44} {'OK' if cond else 'FAIL'} {extra}")

    print("=" * 78)
    print("① 모듈 — 3.10 에서 import 가 되는가 (pxr 지연 import 확인)")
    check("import usd_util 성공 (여기까지 왔으면 성공)", True)
    check("pxr 함수도 이름은 있다",
          all(hasattr(usd_util, f) for f in
              ("rot", "trans", "make_mesh", "wpos", "wmat", "make_drive")))

    print("② load_stl — 바이너리 STL 파서")
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "sq.stl"
        tiny_stl(p)
        pts, idx = usd_util.load_stl(p)
        check("삼각형 6정점 → 공유 4정점", len(pts) == 4, f"({len(pts)})")
        check("삼각형 2개", idx.shape == (2, 3), f"({idx.shape})")
        check("mm → m 스케일 (10mm → 0.010)",
              abs(pts.max() - 0.010) < 1e-12, f"({pts.max()})")
        check("인덱스가 정점 범위 안", idx.min() >= 0 and idx.max() < len(pts))
        pts2, _ = usd_util.load_stl(p, scale=1.0)
        check("scale 인자 동작 (1.0 → 10.0)", abs(pts2.max() - 10.0) < 1e-9)

        print("③ save_png — PIL 저장 / PPM 폴백")
        img = np.zeros((8, 8, 3), dtype=np.uint8)
        img[2:6, 2:6] = (255, 0, 0)
        out = usd_util.save_png(img, Path(td) / "t.png")
        check("파일이 생겼다", out.exists(), f"({out.name})")
        check("확장자 png 또는 ppm", out.suffix in (".png", ".ppm"))

    print("=" * 78)
    print(f"전체 판정: {'통과' if all(ok) else '실패'}  ({sum(ok)}/{len(ok)})")
    return 0 if all(ok) else 1


if __name__ == "__main__":
    sys.exit(main())
