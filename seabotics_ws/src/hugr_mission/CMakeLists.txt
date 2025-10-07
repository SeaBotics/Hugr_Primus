cmake_minimum_required(VERSION 3.8)
project(hugr_mission)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Legg til noder her (eksempel):
# add_executable(${PROJECT_NAME}_node src/main.cpp)
# ament_target_dependencies(${PROJECT_NAME}_node rclcpp)

# install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config urdf rviz meshes DESTINATION share/${PROJECT_NAME})

ament_package()
