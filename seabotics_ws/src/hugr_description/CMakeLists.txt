cmake_minimum_required(VERSION 3.8)
project(hugr_description)


find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)


# Xacro check target (optional convenience)
find_program(XACRO_EXECUTABLE xacro)
if(NOT XACRO_EXECUTABLE)
message(FATAL_ERROR "xacro not found")
endif()


# Install resources
install(DIRECTORY urdf ros2_control launch
DESTINATION share/${PROJECT_NAME}
)


# Validate main xacro at build time (optional)
add_custom_target(check_urdf ALL
COMMAND ${XACRO_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/urdf/hugr.urdf.xacro > /dev/null
COMMENT "Validating hugr.urdf.xacro"
)


ament_package()