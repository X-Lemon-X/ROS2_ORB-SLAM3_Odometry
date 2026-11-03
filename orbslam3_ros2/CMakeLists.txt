cmake_minimum_required(VERSION 3.18)
project(orbslam3)

# You should set the PYTHONPATH to your own python site-packages path
# set(ENV{PYTHONPATH} "/opt/ros/jazzy/lib/python3.10/site-packages")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
)

link_directories(
  include
)

add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)
ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin OpenCV nav_msgs geometry_msgs tf2_ros)

add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)
ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin OpenCV nav_msgs geometry_msgs tf2_ros)

add_executable(stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
)
ament_target_dependencies(stereo rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin OpenCV nav_msgs geometry_msgs tf2_ros)

add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
)

ament_target_dependencies(stereo-inertial rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin OpenCV nav_msgs geometry_msgs tf2_ros)


add_executable(stereo-inertial-fix
  src/stereo-inertial-fix/stereo-inertial.cpp
  src/stereo-inertial-fix/stereo-inertial-node.cpp
)

ament_target_dependencies(stereo-inertial-fix rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin OpenCV nav_msgs geometry_msgs tf2_ros)




install(TARGETS mono rgbd stereo stereo-inertial stereo-inertial-fix DESTINATION lib/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Stereo-inertial SLAM node with control services (recommended entry point)
# ---------------------------------------------------------------------------
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(stereo_inertial_slam
  src/stereo_inertial/main.cpp
  src/stereo_inertial/stereo_inertial_slam_node.cpp
)
target_include_directories(stereo_inertial_slam PRIVATE src/stereo_inertial)
set_target_properties(stereo_inertial_slam PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
ament_target_dependencies(stereo_inertial_slam
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs std_srvs
  cv_bridge tf2 tf2_ros tf2_eigen ORB_SLAM3 Pangolin OpenCV)
target_link_libraries(stereo_inertial_slam Eigen3::Eigen)
install(TARGETS stereo_inertial_slam DESTINATION lib/${PROJECT_NAME})

# Unit tests for the ROS-free sync / pose logic (colcon test)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_stereo_inertial_core test/test_stereo_inertial_core.cpp)
  target_include_directories(test_stereo_inertial_core PRIVATE src/stereo_inertial)
  set_target_properties(test_stereo_inertial_core PROPERTIES CXX_STANDARD 17)
  target_link_libraries(test_stereo_inertial_core Eigen3::Eigen)
endif()

# Vocabulary: unpack the shipped archive at build time so the launch file can find it.
set(ORBVOC_TXT ${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/ORBvoc.txt)
if(NOT EXISTS ${ORBVOC_TXT})
  set(ORBVOC_TXT ${CMAKE_CURRENT_BINARY_DIR}/vocabulary/ORBvoc.txt)
  if(NOT EXISTS ${ORBVOC_TXT})
    message(STATUS "Extracting ORBvoc.txt.tar.gz ...")
    file(ARCHIVE_EXTRACT
      INPUT ${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/ORBvoc.txt.tar.gz
      DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/vocabulary)
  endif()
endif()
install(FILES ${ORBVOC_TXT} DESTINATION share/${PROJECT_NAME}/vocabulary)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)
install(PROGRAMS scripts/make_orbslam3_config.py DESTINATION lib/${PROJECT_NAME})

ament_package()

