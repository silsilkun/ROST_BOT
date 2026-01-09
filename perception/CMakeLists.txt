cmake_minimum_required(VERSION 3.8)
project(perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/to_estimation.srv"
  DEPENDENCIES sensor_msgs
)

ament_package()
