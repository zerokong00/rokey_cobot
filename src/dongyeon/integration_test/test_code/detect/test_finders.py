"""[오프라인] 공용 검출기(detect/finders.py) 검증 — 합성 프레임으로.

repair_demo·real_map_demo 에 복붙으로 갈라져 있던 검출 함수를 합친 것이
양쪽 요구를 다 만족하는지 본다. 특히:
  - 관 저 끝 거르기 두 겹 (중앙 덩어리 / Depth 테두리)
  - 물(파랑) 화소 제외 — 2026-08-06 물결 오탐 수정
  - expect 창 잘라 임계 + 가장 가까운 덩어리 선택
  - 비드 색 검출

실행:  python3 test_finders.py
"""

import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SON = HERE.parents[1]
sys.path.insert(0, str(SON))

from detect import finders  # noqa: E402

W, H = 320, 240
SCALE = (W * H) / (1280.0 * 720.0)   # 면적 하한: 구멍 max(60,25)=60px


def wall(gray=140):
    """회색 관벽 배경 프레임."""
    return np.full((H, W, 3), gray, dtype=np.uint8)


def put(img, cx, cy, r, color):
    yy, xx = np.mgrid[0:H, 0:W]
    img[(xx - cx) ** 2 + (yy - cy) ** 2 <= r ** 2] = color
    return img


def main():
    ok = []

    def check(name, cond, extra=""):
        ok.append(bool(cond))
        print(f"  [{len(ok):2d}] {name:<44} {'OK' if cond else 'FAIL'} {extra}")

    print("=" * 78)
    print("① 구멍 검출 기본 — 밝은 벽 위 어두운 덩어리")
    img = put(wall(), 80, 120, 12, (20, 20, 22))
    h = finders.find_wall_hole(img, area_scale=SCALE)
    check("어두운 덩어리를 구멍으로", h is not None)
    if h:
        check("중심 위치 (±3px)",
              abs(h["cx"] - 80) < 3 and abs(h["cy"] - 120) < 3,
              f"({h['cx']:.0f},{h['cy']:.0f})")

    print("② 관 저 끝 거르기 — 화면 중앙 큰 검정 (expect 없음)")
    img = put(wall(), W // 2, H // 2, 60, (5, 5, 5))
    h = finders.find_wall_hole(img, area_scale=SCALE)
    check("중앙 덩어리는 무시", h is None)

    print("③ 25% 상한 — 화면 절반이 어두워도 구멍 아님 (expect 없음)")
    img = wall()
    img[:, : W // 2] = (10, 10, 10)          # 중앙 원판을 비껴가는 큰 어둠
    h = finders.find_wall_hole(img, area_scale=SCALE)
    check("화면 25% 초과 덩어리 무시", h is None or h["area_px"] < 0.25 * W * H)

    print("④ 물(파랑) 제외 — 어둡지만 파란 물결 그림자")
    img = put(wall(), 80, 120, 12, (18, 38, 78))     # B-R=60, 어두움
    h = finders.find_wall_hole(img, area_scale=SCALE)
    check("파란 어둠은 구멍 아님", h is None)
    img2 = put(put(wall(), 80, 120, 12, (18, 38, 78)),
               220, 120, 10, (20, 20, 22))
    h2 = finders.find_wall_hole(img2, area_scale=SCALE)
    check("옆의 무채색 구멍은 그대로 잡음",
          h2 is not None and abs(h2["cx"] - 220) < 3)

    print("⑤ expect 창 — 가장 큰 게 아니라 가장 가까운 덩어리")
    img = put(wall(), 100, 120, 10, (20, 20, 22))    # 결함 (작다)
    img = put(img, 160, 120, 25, (5, 5, 5))          # 저 끝 (크다, 창 언저리)
    h = finders.find_wall_hole(img, expect_px=(100, 120), area_scale=SCALE)
    check("expect 에 가까운 쪽 선택",
          h is not None and abs(h["cx"] - 100) < 5 and h["matched"])

    print("⑥ Depth 테두리 — 테두리가 멀면 저 끝, 가까우면 구멍")
    img = put(wall(), 100, 120, 12, (20, 20, 22))
    far = np.full((H, W), 0.45, dtype=np.float32)    # 테두리 깊이 0.45m > 0.22
    h = finders.find_wall_hole(img, expect_px=(100, 120), depth=far,
                               area_scale=SCALE)
    check("먼 테두리 → 관 저 끝으로 배제", h is None)
    near = np.full((H, W), 0.10, dtype=np.float32)
    h = finders.find_wall_hole(img, expect_px=(100, 120), depth=near,
                               area_scale=SCALE)
    check("가까운 테두리 → 구멍 유지",
          h is not None and h["rim_m"] is not None
          and abs(h["rim_m"] - 0.10) < 0.01)

    print("⑦ 비드 — 주황(채도)만 잡고 회색·검정은 무시")
    img = put(wall(), 150, 100, 10, (215, 140, 40))  # 주황
    b = finders.find_weld_bead(img, expect_px=(150, 100), area_scale=SCALE)
    check("주황 비드 검출 + matched",
          b is not None and b["matched"] and abs(b["cx"] - 150) < 3)
    b2 = finders.find_weld_bead(put(wall(), 150, 100, 10, (30, 30, 30)),
                                area_scale=SCALE)
    check("무채색 덩어리는 비드 아님", b2 is None)

    print("=" * 78)
    print(f"전체 판정: {'통과' if all(ok) else '실패'}  ({sum(ok)}/{len(ok)})")
    return 0 if all(ok) else 1


if __name__ == "__main__":
    sys.exit(main())
