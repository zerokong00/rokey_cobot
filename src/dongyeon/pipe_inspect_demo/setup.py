from setuptools import find_packages, setup

package_name = "pipe_inspect_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dongyeon",
    maintainer_email="loik1235@gmail.com",
    description="배관 점검 로봇 — Isaac Sim 카메라 토픽 확인/결함 탐지 (PC B 측)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_check = pipe_inspect_demo.camera_check:main",
            "defect_detector = pipe_inspect_demo.defect_detector:main",
        ],
    },
)
