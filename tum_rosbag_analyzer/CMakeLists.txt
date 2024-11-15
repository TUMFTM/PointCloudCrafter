cmake_minimum_required(VERSION 3.12)
set(PROJECT_NAME tum_rosbag_analyzer)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)

add_executable(rosbag_analyzer
  rosbag_analyzer.cpp
  timestamp_analyzer.cpp)
target_compile_features(rosbag_analyzer PUBLIC c_std_99 cxx_std_20)
target_include_directories(rosbag_analyzer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PCL_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rosbag_analyzer
  rclcpp
  sensor_msgs
  pcl_conversions
  ament_index_cpp
  rosbag2_cpp
)
target_link_libraries(rosbag_analyzer   # always after ament_target_dependencies
  ${PCL_LIBRARIES}
)

install(
  DIRECTORY ./include/
  DESTINATION include
)

install(
  TARGETS rosbag_analyzer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
