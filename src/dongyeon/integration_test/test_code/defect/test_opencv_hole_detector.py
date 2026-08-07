"""[오프라인] OpenCV 결함 검출 시험 — ROS·Isaac 불필요.

`defect/opencv_hole_detector.py`(= repair_demo.py find_wall_hole/
find_weld_bead 이식)를 합성 영상으로 검증한다.

핵심 질문 셋
  - 밝은 벽에 둘러싸인 어두운 덩어리(구멍)를 잡는가
  - 화면 중앙의 전방 개구부(관 저 끝)는 구멍으로 오검출하지 않는가
  - 채도 높은 주황(비드)만 걸리고 회색 벽·검은 개구부는 안 걸리는가

실행:  python3 test_opencv_hole_detector.py
"""

import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SON = HERE.parents[1]
sys.path.insert(0, str(SON / "defect"))

from opencv_hole_detector import OpenCvHoleDetector

W, H = 640, 360


def wall_scene(hole_at=None, hole_r=18, bore_r=40, bead_at=None, bead_r=14):
    """회색 벽 + 중앙 어두운 원형 개구부(관 저 끝) + 선택적 구멍/비드."""
    rgb = np.full((H, W, 3), 170, dtype=np.uint8)          # 밝은 회색 벽
    yy, xx = np.mgrid[0:H, 0:W]
    bore = (xx - W // 2) ** 2 + (yy - H // 2) ** 2 <= bore_r ** 2
    rgb[bore] = (20, 20, 20)                                # 전방 개구부(검정, 채도 0)
    if hole_at is not None:
        cx, cy = hole_at
        m = (xx - cx) ** 2 + (yy - cy) ** 2 <= hole_r ** 2
        rgb[m] = (15, 15, 15)                               # 벽면 결함(어둠)
    if bead_at is not None:
        cx, cy = bead_at
        m = (xx - cx) ** 2 + (yy - cy) ** 2 <= bead_r ** 2
        rgb[m] = (230, 130, 20)                              # 주황 비드
    return rgb


def main():
    ok = []
    det = OpenCvHoleDetector()

    print("=" * 78)
    print("① 벽면 구멍 검출 — 중앙 개구부와 떨어진 자리")
    print("=" * 78)
    rgb = wall_scene(hole_at=(160, 90))
    found = det.find_wall_hole(rgb)
    good = len(found) == 1 and abs(found[0].center_pixel[0] - 160) < 3 and abs(found[0].center_pixel[1] - 90) < 3
    print(f"  검출 {len(found)}개  중심 {found[0].center_pixel if found else None}  {'OK' if good else 'FAIL'}")
    ok.append(good)

    print("=" * 78)
    print("② 전방 개구부만 있을 때 — 구멍으로 오검출하면 안 된다")
    print("=" * 78)
    rgb = wall_scene()
    found = det.find_wall_hole(rgb)
    good = len(found) == 0
    print(f"  검출 {len(found)}개 (기대 0)  {'OK' if good else 'FAIL'}")
    ok.append(good)

    print("=" * 78)
    print("③ 너무 작은 어두운 덩어리는 노이즈로 무시")
    print("=" * 78)
    rgb = wall_scene(hole_at=(160, 90), hole_r=3)   # 최소 픽셀 미달
    found = det.find_wall_hole(rgb)
    good = len(found) == 0
    print(f"  검출 {len(found)}개 (기대 0)  {'OK' if good else 'FAIL'}")
    ok.append(good)

    print("=" * 78)
    print("④ 용접 비드 검출 — 회색 벽·검은 개구부는 안 걸린다")
    print("=" * 78)
    rgb = wall_scene(bead_at=(480, 200))
    found = det.find_weld_bead(rgb)
    good = len(found) == 1 and abs(found[0].center_pixel[0] - 480) < 3
    print(f"  검출 {len(found)}개  중심 {found[0].center_pixel if found else None}  {'OK' if good else 'FAIL'}")
    ok.append(good)

    print("=" * 78)
    print("⑤ mask 는 실제 area_px 와 픽셀 수가 일치해야 한다")
    print("=" * 78)
    rgb = wall_scene(hole_at=(160, 90))
    found = det.find_wall_hole(rgb)
    good = bool(found) and int(found[0].mask.sum()) == found[0].area_px
    print(f"  mask 합 {int(found[0].mask.sum()) if found else None} == area_px {found[0].area_px if found else None}  {'OK' if good else 'FAIL'}")
    ok.append(good)

    print("=" * 78)
    print(f"전체 판정: {'통과' if all(ok) else '실패'}  ({sum(ok)}/{len(ok)})")
    return 0 if all(ok) else 1


if __name__ == "__main__":
    sys.exit(main())
