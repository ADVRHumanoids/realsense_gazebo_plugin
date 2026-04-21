cmake_minimum_required(VERSION 3.10)
project(realsense_gz_plugin)

# ---------------------------------------------------------------------------
# C++ standard
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# ament
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)

# ---------------------------------------------------------------------------
# Gazebo Harmonic dependencies
# gz-sensors8 must be built with camera and depth_camera components.
# ---------------------------------------------------------------------------
find_package(gz-plugin2  REQUIRED)
find_package(gz-rendering8 REQUIRED)
find_package(gz-sim8     REQUIRED)
find_package(gz-sensors8 REQUIRED COMPONENTS camera depth_camera)

# ---------------------------------------------------------------------------
# Shared library
# ---------------------------------------------------------------------------
add_library(ros2_gz_camera SHARED
  src/Ros2Camera.cpp
  # src/ros2_depth_camera_sensor.cpp
  # src/Ros2CameraSystem.cpp
  # src/realsense_gz_plugin.cpp
)

add_library(Ros2CameraSystem SHARED
  # src/ros2_sensor_config.cpp
  # src/Ros2Camera.cpp
  # src/ros2_depth_camera_sensor.cpp
  src/Ros2CameraSystem.cpp
  # src/realsense_gz_plugin.cpp
)

target_include_directories(ros2_gz_camera PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(Ros2CameraSystem PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(ros2_gz_camera
  gz-plugin2::gz-plugin2
  gz-sensors8::gz-sensors8
  gz-sensors8::gz-sensors8-camera
  # gz-sensors8::gz-sensors8-depth_camera
)

target_link_libraries(Ros2CameraSystem
  gz-plugin2::gz-plugin2
  gz-rendering8::gz-rendering8
  gz-sim8::gz-sim8
  ros2_gz_camera
  # gz-sensors8::gz-sensors8-depth_camera
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(
  TARGETS ros2_gz_camera
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS Ros2CameraSystem
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/realsense_gz_plugin.dsv.in"
)

ament_package()
