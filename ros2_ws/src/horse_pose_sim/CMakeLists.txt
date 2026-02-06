cmake_minimum_required(VERSION 3.8)
project(horse_pose_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HorseKeypoint.msg"
  "msg/HorseDetection.msg"
  "msg/HorseDetections.msg"
  DEPENDENCIES std_msgs
)

install(
  DIRECTORY
    config
    launch
    models
    rviz
    worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/horse_pose_node.py
    scripts/keypoint_overlay_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
