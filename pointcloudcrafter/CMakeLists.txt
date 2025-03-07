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

# find dependencies
# find_package(autoware_cmake REQUIRED)
# autoware_package()

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
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)
# find_package(GeographicLib REQUIRED)
# find_package(novatel_oem7_msgs REQUIRED)
# find_package(vectornav_msgs REQUIRED)

include_directories(SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

# ament_auto_add_executable(rosbag_to_pcd src/rosbag_to_pcd_main.cpp src/rosbag_to_pcd.cpp src/utils.cpp src/pcl_utils.cpp)
# ament_target_dependencies(rosbag_to_pcd rclcpp rosbag2_cpp gps_msgs sensor_msgs pcl_conversions tf2_ros yaml-cpp)

add_executable(rosbag_to_pcd ${CMAKE_CURRENT_SOURCE_DIR}/src/rosbag_to_pcd_main.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/rosbag_to_pcd.cpp src/utils.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/pcl_utils.cpp)
ament_target_dependencies(rosbag_to_pcd rclcpp rosbag2_cpp gps_msgs sensor_msgs pcl_conversions tf2_ros yaml-cpp)

# add_executable(rosbag_to_gnss ${CMAKE_CURRENT_SOURCE_DIR}/src/rosbag_to_gnss_main.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/rosbag_to_gnss.cpp src/utils.cpp)
# ament_target_dependencies(rosbag_to_gnss rclcpp rosbag2_cpp gps_msgs sensor_msgs tf2_ros novatel_oem7_msgs vectornav_msgs nav_msgs GeographicLib)

target_include_directories(rosbag_to_pcd PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
# target_include_directories(rosbag_to_gnss PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(rosbag_to_pcd ${PCL_LIBRARIES} fmt)
# target_link_libraries(rosbag_to_gnss ${PCL_LIBRARIES} fmt ${GeographicLib_LIBRARIES})

target_compile_features(rosbag_to_pcd PRIVATE cxx_std_17)
# target_compile_features(rosbag_to_gnss PRIVATE cxx_std_17)

install(TARGETS rosbag_to_pcd RUNTIME DESTINATION lib/${PROJECT_NAME})
# install(TARGETS rosbag_to_gnss RUNTIME DESTINATION lib/${PROJECT_NAME})
# install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME}/)

# ament_auto_package()
ament_package()
