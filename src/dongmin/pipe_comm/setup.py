from glob import glob

from setuptools import find_packages, setup

package_name = "pipe_comm"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dongmin",
    maintainer_email="loik1235@gmail.com",
    description="배관 점검 로봇 — Isaac Sim(PC1) ↔ ROS 2(PC2) 통신 규약과 감시·지령 노드",
    license="MIT",
    entry_points={
        "console_scripts": [
            "camera_monitor = pipe_comm.camera_monitor:main",
            "drive_monitor = pipe_comm.drive_monitor:main",
            "mission_cli = pipe_comm.mission_cli:main",
        ],
    },
)
