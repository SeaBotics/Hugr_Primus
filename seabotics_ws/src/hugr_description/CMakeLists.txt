cmake_minimum_required(VERSION 3.8)
project(hugr_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf ros2_control DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

ament_package()
