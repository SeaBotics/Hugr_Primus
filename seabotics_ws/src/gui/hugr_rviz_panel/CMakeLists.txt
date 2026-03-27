cmake_minimum_required(VERSION 3.8)
project(hugr_rviz_panel)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)

add_library(hugr_rviz_panel SHARED
  src/hugr_status_panel.cpp
  src/hugr_gyro_display.cpp
  src/hugr_compass_panel.cpp
  src/hugr_camera_subscriber_panel.cpp
  src/hugr_leak_panel.cpp
  include/hugr_rviz_panel/hugr_status_panel.hpp
  include/hugr_rviz_panel/hugr_gyro_display.hpp
  include/hugr_rviz_panel/hugr_compass_panel.hpp
  include/hugr_rviz_panel/hugr_camera_subscriber_panel.hpp
  include/hugr_rviz_panel/hugr_leak_panel.hpp
)

set_target_properties(hugr_rviz_panel PROPERTIES AUTOMOC ON)

target_include_directories(hugr_rviz_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(hugr_rviz_panel
  rclcpp
  rviz_common
  rviz_rendering
  pluginlib
  std_msgs
  sensor_msgs
  geometry_msgs
  image_transport
  cv_bridge
)

target_link_libraries(hugr_rviz_panel
  Qt5::Widgets
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS hugr_rviz_panel
  EXPORT export_hugr_rviz_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
