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

# Library
add_library(pointcloudcrafter_rosbag SHARED
  src/pointcloudcrafter_rosbag.cpp)

target_include_directories(pointcloudcrafter_rosbag
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS})

ament_target_dependencies(pointcloudcrafter_rosbag PUBLIC
  rclcpp
  rosbag2_cpp
  sensor_msgs
  pcl_conversions
  tf2_ros
  urdf)

target_link_libraries(pointcloudcrafter_rosbag PUBLIC
  ${PCL_LIBRARIES}
  fmt::fmt
  pointcloudmodifierlib
  utils)

# Executable
add_executable(rosbag
  src/pointcloudcrafter_rosbag_main.cpp)

target_link_libraries(rosbag PRIVATE
  pointcloudcrafter_rosbag)

# Installation
install(TARGETS pointcloudcrafter_rosbag
  LIBRARY DESTINATION lib)

# ROS2
install(TARGETS rosbag
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# Python wheel
install(TARGETS rosbag
  RUNTIME DESTINATION bin)
