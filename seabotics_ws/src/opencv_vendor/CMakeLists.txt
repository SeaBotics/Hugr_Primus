cmake_minimum_required(VERSION 3.16)
project(opencv_vendor)

include(ExternalProject)
include(GNUInstallDirs)

# Choose OpenCV version
set(OPENCV_VERSION 4.10.0)

# Download OpenCV source
set(OPENCV_URL https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip)

# Where OpenCV will be built inside the workspace
set(OPENCV_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/opencv_install)

ExternalProject_Add(opencv_src
  URL ${OPENCV_URL}
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/opencv_src
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${OPENCV_INSTALL_DIR}
    -DCMAKE_BUILD_TYPE=Release
    -DBUILD_TESTS=OFF
    -DBUILD_PERF_TESTS=OFF
    -DBUILD_opencv_python=ON
    -DBUILD_opencv_java=OFF
    -DBUILD_opencv_apps=OFF
    -DBUILD_opencv_js=OFF
)

# Export include + lib dirs
install(
  DIRECTORY ${OPENCV_INSTALL_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  DIRECTORY ${OPENCV_INSTALL_DIR}/lib/
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# Export config file for downstream packages
install(
  FILES cmake/opencv_vendor-extras.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(include)
ament_export_libraries(opencv_core opencv_imgproc opencv_highgui)

ament_package()
