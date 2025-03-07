# Copyright 2024 Markus Pielmeier, Florian Sauerbeck,
# Dominik Kulmer, Maximilian Leitenstern

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(pointcloudcrafter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)

include_directories(SYSTEM
  ${PCL_INCLUDE_DIRS}
)

# Library
add_library(pointcloudcrafter_lib SHARED
  src/pointcloudcrafter.cpp)

target_include_directories(pointcloudcrafter_lib
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(pointcloudcrafter_lib PRIVATE cxx_std_20)
ament_target_dependencies(pointcloudcrafter_lib PUBLIC
  rclcpp
  rosbag2_cpp
  sensor_msgs
  pcl_conversions
  tf2_ros)

target_link_libraries(pointcloudcrafter_lib PRIVATE
  ${PCL_LIBRARIES})

install(
  TARGETS pointcloudcrafter_lib
  EXPORT export_pointcloudcrafter_lib
  DESTINATION lib/pointcloudcrafter_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_pointcloudcrafter_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rosbag2_cpp
  sensor_msgs
  pcl_conversions
  tf2_ros)

# Executable
add_executable(pointcloudcrafter
  src/pointcloudcrafter_main.cpp)

target_link_libraries(pointcloudcrafter
  pointcloudcrafter_lib)

target_compile_features(pointcloudcrafter PRIVATE cxx_std_20)

install(TARGETS pointcloudcrafter RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
