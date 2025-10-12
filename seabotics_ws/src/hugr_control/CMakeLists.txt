cmake_minimum_required(VERSION 3.8)
project(hugr_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(asv_kinematics src/asv_kinematics.cpp)
ament_target_dependencies(asv_kinematics rclcpp)

install(TARGETS asv_kinematics
  DESTINATION lib/${PROJECT_NAME})

ament_package()
