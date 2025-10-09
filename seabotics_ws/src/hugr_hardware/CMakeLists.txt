cmake_minimum_required(VERSION 3.8)
project(hugr_hardware)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# === Library: ros2_control SystemInterface ===
add_library(serial_thruster_interface SHARED
  src/serial_thruster_interface.cpp          # <== bruk propulsion_control/src/... hvis du ikke flytter
)
ament_target_dependencies(serial_thruster_interface rclcpp hardware_interface pluginlib)

# Public headers
target_include_directories(serial_thruster_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# pluginlib descriptor
pluginlib_export_plugin_description_file(hardware_interface serial_thruster_interface.xml)

# === Node: /cmd_vel -> /thruster_controller/commands ===
add_executable(twist_to_thrust_node
  src/twist_to_thrust_node.cpp              # <== bruk propulsion_control/src/... hvis du ikke flytter
)
ament_target_dependencies(twist_to_thrust_node rclcpp geometry_msgs std_msgs)

# Install
install(TARGETS serial_thruster_interface twist_to_thrust_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(FILES serial_thruster_interface.xml DESTINATION share/${PROJECT_NAME})

ament_package()
