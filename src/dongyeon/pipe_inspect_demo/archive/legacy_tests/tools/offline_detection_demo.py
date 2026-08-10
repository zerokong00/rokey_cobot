"""ROS 없이 합성 배관 영상으로 dark-blob 검출 결과를 시각화한다."""

import argparse
from pathlib import Path
import sys

import cv2
import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from pipe_inspect_demo.defect_detection import DarkBlobDetector  # noqa: E402


SIZE = 320


def pipe_background():
    """배관 벽과 중앙 원거리 암부를 단순화한 RGB 합성 영상."""
    yy, xx = np.mgrid[0:SIZE, 0:SIZE]
    radius = np.hypot(yy - SIZE / 2, xx - SIZE / 2) / (SIZE / 2)
    wall = 178.0 + 14.0 * np.cos(radius * np.pi)
    wall += 5.0 * np.sin(xx / 23.0) + 3.0 * np.cos(yy / 19.0)
    wall[radius < 0.34] = 28.0 + radius[radius < 0.34] * 45.0
    image = np.clip(wall, 0, 255).astype(np.uint8)
    return np.repeat(image[:, :, None], 3, axis=2)


def make_cases():
    normal = pipe_background()
    cases = {"01_normal": normal.copy()}

    hole = normal.copy()
    cv2.circle(hole, (252, 160), 13, (7, 7, 7), -1)
    cases["02_hole"] = hole

    crack = normal.copy()
    points = np.array([[224, 102], [232, 120], [229, 140], [242, 162], [238, 184]])
    cv2.polylines(crack, [points], False, (5, 5, 5), 6, cv2.LINE_AA)
    cases["03_crack"] = crack

    center_dark = normal.copy()
    cv2.circle(center_dark, (160, 160), 52, (0, 0, 0), -1)
    cases["04_center_dark"] = center_dark

    noise = normal.copy()
    for x, y in [(248, 130), (250, 190), (82, 146), (110, 245)]:
        cv2.circle(noise, (x, y), 2, (0, 0, 0), -1)
    cases["05_small_noise"] = noise

    shadow = normal.copy()
    shadow_layer = shadow.copy()
    cv2.ellipse(shadow_layer, (245, 160), (35, 62), 0, 0, 360, (35, 35, 35), -1)
    shadow = cv2.addWeighted(shadow, 0.25, shadow_layer, 0.75, 0.0)
    cases["06_shadow_false_positive"] = shadow

    uneven = normal.astype(np.float32)
    gradient = np.linspace(0.28, 1.0, SIZE, dtype=np.float32)[None, :, None]
    uneven = np.clip(uneven * gradient, 0, 255).astype(np.uint8)
    cases["07_uneven_lighting"] = uneven
    return cases


def new_detector():
    detector = DarkBlobDetector(
        calibration_frames=6,
        dark_ratio=0.45,
        min_area=40,
        noise_area_scale=2.0,
        confirm_frames=3,
        release_frames=3,
    )
    normal = pipe_background()
    for _ in range(detector.calibration_frames):
        detector.process(normal)
    if not detector.calibrated:
        raise RuntimeError("합성 정상 영상 캘리브레이션 실패")
    return detector


def visualize(image, detector, result, title):
    view = image.copy()
    overlay = np.zeros_like(view)
    overlay[detector.mask] = (0, 75, 0)
    view = cv2.addWeighted(view, 1.0, overlay, 0.28, 0.0)
    if result.candidate is not None:
        x, y, width, height = result.candidate.box
        cv2.rectangle(view, (x, y), (x + width, y + height), (255, 40, 40), 3)
        cv2.circle(
            view,
            (round(result.candidate.center_x), round(result.candidate.center_y)),
            4,
            (255, 255, 0),
            -1,
        )
    state = "DETECTED" if result.raw_detected else "CLEAR"
    cv2.rectangle(view, (0, 0), (SIZE, 58), (0, 0, 0), -1)
    cv2.putText(view, title, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    cv2.putText(
        view,
        f"{state}  area={result.candidate.area if result.candidate else 0}",
        (8, 48),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (255, 80, 80) if result.raw_detected else (80, 255, 80),
        2,
    )
    return view


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=PACKAGE_ROOT / "test_output",
        help="결과 이미지 출력 폴더",
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    panels = []
    summary = []
    for name, image in make_cases().items():
        detector = new_detector()
        result = detector.process(image)
        panel = visualize(image, detector, result, name)
        cv2.imwrite(str(args.output / f"{name}.png"), cv2.cvtColor(panel, cv2.COLOR_RGB2BGR))
        panels.append(panel)
        summary.append(
            f"{name}: {'DETECTED' if result.raw_detected else 'CLEAR'} "
            f"area={result.candidate.area if result.candidate else 0} "
            f"threshold={result.threshold:.1f}"
        )

    blank = np.full_like(panels[0], 25)
    while len(panels) % 2:
        panels.append(blank.copy())
    rows = [np.hstack(panels[index:index + 2]) for index in range(0, len(panels), 2)]
    montage = np.vstack(rows)
    cv2.imwrite(str(args.output / "montage.png"), cv2.cvtColor(montage, cv2.COLOR_RGB2BGR))
    (args.output / "summary.txt").write_text("\n".join(summary) + "\n", encoding="utf-8")
    print("\n".join(summary))
    print(f"결과 저장: {args.output.resolve()}")


if __name__ == "__main__":
    main()
