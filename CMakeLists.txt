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
project(pointcloudcrafter VERSION 1.1.0)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

# Options
option(BUILD_TESTS "Build unit tests" OFF)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Add subdirectories
add_subdirectory(pointcloudcrafter_pcd)
add_subdirectory(pointcloudcrafter_rosbag)
add_subdirectory(pointcloudmodifier)
add_subdirectory(utils)

ament_package()