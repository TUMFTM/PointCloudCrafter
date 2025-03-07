# Copyright (C) 2024 Markus Pielmeier, Florian Sauerbeck, Dominik Kulmer, Maximilian Leitenstern
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

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
find_package(gps_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(fmt REQUIRED)
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
  gps_msgs
  sensor_msgs
  pcl_conversions
  tf2_ros)
target_link_libraries(pointcloudcrafter_lib PRIVATE
  ${PCL_LIBRARIES}
  fmt)

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
  gps_msgs
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
