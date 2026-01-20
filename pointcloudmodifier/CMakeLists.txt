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

# Suppress VTK deprecation warnings
add_definitions(-DVTK_DEPRECATION_LEVEL=0)

# Library
add_library(pointcloudmodifierlib INTERFACE)

target_include_directories(pointcloudmodifierlib
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS})

target_link_libraries(pointcloudmodifierlib
  INTERFACE
    ${PCL_LIBRARIES})

# Installation
install(TARGETS pointcloudmodifierlib
  EXPORT pointcloudmodifierlibTargets)

install(FILES include/pointcloudmodifier.hpp
  DESTINATION include)

# Examples
option(BUILD_EXAMPLES "Build example applications" OFF)
if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

# Tests
if(BUILD_TESTS)
  add_subdirectory(tests)
endif()