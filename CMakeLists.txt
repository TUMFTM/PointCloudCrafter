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

cmake_minimum_required(VERSION 3.10)
project(pointcloudcrafter VERSION 1.0.0)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(fmt REQUIRED)

option(BUILD_TESTS "Build unit tests" ON)

# Subdirectory for cpp library
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/pointcloudmodifier/)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/pointcloudmodifier)
else()
  message(FATAL_ERROR "pointcloudmodifier directory not found by cmake")
endif()

# Library
add_library(${PROJECT_NAME} SHARED
  pointcloudcrafter_rosbag/src/pointcloudcrafter_rosbag.cpp
  pointcloudcrafter_pcd/src/pointcloudcrafter_pcd.cpp)

target_include_directories(${PROJECT_NAME}
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/pointcloudcrafter_rosbag/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/pointcloudcrafter_pcd/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/utils>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  rosbag2_cpp
  sensor_msgs
  pcl_conversions
  tf2_ros)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${PCL_LIBRARIES}
  fmt::fmt
  pointcloudmodifierlib)

# Installation rules
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    PUBLIC_HEADER DESTINATION include
)


# Executable
add_executable(rosbag
  pointcloudcrafter_rosbag/src/pointcloudcrafter_rosbag_main.cpp)

target_link_libraries(rosbag
  ${PROJECT_NAME})

target_compile_features(rosbag PRIVATE cxx_std_20)

install(TARGETS rosbag
  RUNTIME DESTINATION lib/${PROJECT_NAME})

add_executable(pcd
  pointcloudcrafter_pcd/src/pointcloudcrafter_pcd_main.cpp)

target_link_libraries(pcd
  ${PROJECT_NAME})

target_compile_features(pcd PRIVATE cxx_std_20)

install(TARGETS pcd
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
