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
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
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
  DIRECTORY ../include/
  DESTINATION include
)

install(
  TARGETS rosbag_analyzer
  DESTINATION lib/${PROJECT_NAME}
)
