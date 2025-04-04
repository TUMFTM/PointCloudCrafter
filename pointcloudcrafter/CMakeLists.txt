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

cmake_minimum_required(VERSION 3.10)
project(pointcloudcrafter)

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
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)

include_directories(SYSTEM
  ${PCL_INCLUDE_DIRS}
)

# Subdirectory for cpp library
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../pointcloudmodifyer/)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../pointcloudmodifyer ${CMAKE_CURRENT_BINARY_DIR}/pointcloudcrafter)
else()
  message(FATAL_ERROR "pointcloudmodifyer directory not found by cmake")
endif()


# Library
add_library(${PROJECT_NAME} SHARED
  src/pointcloudcrafter.cpp)

target_include_directories(${PROJECT_NAME}
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  rosbag2_cpp
  sensor_msgs
  pcl_conversions
  tf2_ros)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${PCL_LIBRARIES}
  pointcloudmodifyer)

# Installation rules
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    INCLUDES DESTINATION include
)

# Executable
add_executable(crafter
  src/pointcloudcrafter_main.cpp)

target_link_libraries(crafter
  ${PROJECT_NAME})

target_compile_features(crafter PRIVATE cxx_std_20)

install(TARGETS crafter
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
