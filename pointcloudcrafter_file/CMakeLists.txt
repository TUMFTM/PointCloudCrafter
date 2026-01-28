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
add_library(pointcloudcrafter_file SHARED
  src/pointcloudcrafter_file.cpp)

target_include_directories(pointcloudcrafter_file
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS})

ament_target_dependencies(pointcloudcrafter_file PUBLIC
  rclcpp)

target_link_libraries(pointcloudcrafter_file PUBLIC
  ${PCL_LIBRARIES}
  fmt::fmt
  pointcloudmodifierlib
  utils)

# Executable
add_executable(file
  src/pointcloudcrafter_file_main.cpp)

target_link_libraries(file PRIVATE
  pointcloudcrafter_file)

# Installation
install(TARGETS pointcloudcrafter_file
  LIBRARY DESTINATION lib)

# ROS2
install(TARGETS file
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# Python wheel
install(TARGETS file
  RUNTIME DESTINATION bin)