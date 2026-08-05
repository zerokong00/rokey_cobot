from pathlib import Path

from setuptools import find_packages, setup

package_name = "pipe_inspect_demo"
model_path = Path("resource/yolov8n_seg_best.pt")
model_data = [(f"share/{package_name}/resource", [str(model_path)])] if model_path.is_file() else []

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ] + model_data,
    install_requires=["setuptools", "ultralytics"],
    zip_safe=True,
    maintainer="dongyeon",
    maintainer_email="loik1235@gmail.com",
    description="배관 점검 로봇 — Isaac Sim 압축 RGB·Depth 토픽 기반 YOLO Seg 결함 탐지·위치 추적·수리 판정 (PC B 측)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pipe_vision = pipe_inspect_demo.pipe_vision_node:main",
            "pipe_report = pipe_inspect_demo.pipe_report_node:main",
            "pipe_coordinator = pipe_inspect_demo.pipe_coordinator_node:main",
            "yolo_viewer = pipe_inspect_demo.yolo_debug_viewer:main",
            "repair_target_test = pipe_inspect_demo.repair_target_test_node:main",
        ],
    },
)
