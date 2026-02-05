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

cmake_minimum_required(VERSION 3.22.1...4.2.1)


# Suppress PCL warnings
set(CMAKE_POLICY_DEFAULT_CMP0144 OLD)
set(CMAKE_POLICY_DEFAULT_CMP0167 OLD)

project(pointcloudcrafter VERSION 1.2.4)

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
find_package(PCL REQUIRED QUIET)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage_mcap REQUIRED)
find_package(rosbag2_storage_default_plugins REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)

# Add subdirectories
add_subdirectory(pointcloudcrafter_file)
add_subdirectory(pointcloudcrafter_rosbag)
add_subdirectory(pointcloudmodifier)
add_subdirectory(utils)

ament_package()

# Python wheel install
set(CMAKE_INSTALL_BINDIR "bin")
set(CMAKE_INSTALL_LIBDIR "lib")

# Baking dependencies into wheel
if(SKBUILD)
    install(CODE [[
        execute_process(
            COMMAND /bin/bash
                    "${CMAKE_SOURCE_DIR}/package/bake_libs.sh"
                    "${CMAKE_INSTALL_PREFIX}"
        )
    ]])
endif()
