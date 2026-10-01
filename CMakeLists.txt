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
# Workspace-local OpenCV 4.5.4 built WITH the contrib aruco module.
# The system OpenCV (NVIDIA JetPack 4.5.4) has no aruco. Override with -DOpenCV_DIR=... if needed.
if(NOT DEFINED OpenCV_DIR)
  set(OpenCV_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/opencv454/lib/cmake/opencv4")
endif()
find_package(OpenCV REQUIRED)
message(STATUS "fast_calib: OpenCV ${OpenCV_VERSION} from ${OpenCV_DIR}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

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