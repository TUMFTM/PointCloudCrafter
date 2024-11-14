cmake_minimum_required(VERSION 3.12)
set(PROJECT_NAME tum_rosbag_analyzer)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_lint_auto REQUIRED)


add_subdirectory(src)

# if launchfiles
#install(
#    DIRECTORY launch
#    DESTINATION share/${PROJECT_NAME}
#)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   ament_add_gtest(unit_tests test/unit_test.cpp) # unit tests
# endif()

ament_package()
