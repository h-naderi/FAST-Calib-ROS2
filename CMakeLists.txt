cmake_minimum_required(VERSION 3.8)
project(fast_calib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(PCL 1.10 REQUIRED)
# OpenCV >= 4.7, where ArUco is part of objdetect -- no contrib build needed.
# On the rover (JetPack 7.2) that is NVIDIA's 4.8 in /usr/lib/cmake/opencv4. Ubuntu's
# 4.6 runtime is also installed (cv_bridge links it); do not add cv_bridge here, or
# both OpenCVs load into one process.
find_package(OpenCV 4.7 REQUIRED COMPONENTS core imgproc imgcodecs calib3d objdetect)
message(STATUS "fast_calib: OpenCV ${OpenCV_VERSION} from ${OpenCV_DIR}")

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(CMAKE_BUILD_TYPE Release)
if (CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

# Create executable
add_executable(fast_calib src/main.cpp)

# Dependencies for the executable
ament_target_dependencies(fast_calib
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  rosbag2_cpp
  rosbag2_storage
  ament_index_cpp
)

target_link_libraries(fast_calib
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Install targets
install(TARGETS
  fast_calib
  DESTINATION lib/${PROJECT_NAME}
)

# Install other directories
install(DIRECTORY
  launch
  config
  rviz_cfg
  calib_data
  output
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()