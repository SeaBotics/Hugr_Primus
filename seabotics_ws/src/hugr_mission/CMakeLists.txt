cmake_minimum_required(VERSION 3.8)
project(hugr_mission)

find_package(ament_cmake REQUIRED)

# Tåler at mappen mangler:
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()
