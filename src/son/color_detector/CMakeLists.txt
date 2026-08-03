cmake_minimum_required(VERSION 3.8)
project(color_detector)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  color_detector/color_detector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME color_detector
)

ament_package()
