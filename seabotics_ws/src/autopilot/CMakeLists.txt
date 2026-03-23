cmake_minimum_required(VERSION 3.8)
project(autopilot)

# Bruk C++17 eller nyere (standard for Humble)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Finn avhengigheter
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 2. Legg til selve kjørefilen (executable)
add_executable(autopilot_node 
  src/autopilot_node.cpp
  src/main.cpp
)

# 3. Fortell CMake hvor header-filene dine bor (Dette fikser Squiggles i VS Code!)
target_include_directories(autopilot_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 4. Koble bibliotekene sammen
ament_target_dependencies(autopilot_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
)

# 5. Installasjon (Viktig for at "ros2 run" skal virke)
install(TARGETS
  autopilot_node
  DESTINATION lib/${PROJECT_NAME}
)

# Testing (behold denne nederst)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()