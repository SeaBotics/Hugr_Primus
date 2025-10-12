cmake_minimum_required(VERSION 3.8)
project(hugr_hardware)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)


# ---- Hardware interface (shared lib / plugin) ----
add_library(serial_thruster_interface SHARED
  src/serial_thruster_interface.cpp
)
target_include_directories(serial_thruster_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(serial_thruster_interface
  rclcpp rclcpp_lifecycle hardware_interface pluginlib)
# Export plugin description
pluginlib_export_plugin_description_file(hardware_interface hugr_hardware_plugin.xml)

# ---- Helper node: twist -> thruster commands ----
add_executable(twist_to_thrust_node
  src/twist_to_thrust_node.cpp
)
target_include_directories(twist_to_thrust_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(twist_to_thrust_node
  rclcpp geometry_msgs std_msgs)

# ---- Install ----
install(TARGETS serial_thruster_interface twist_to_thrust_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(FILES hugr_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
