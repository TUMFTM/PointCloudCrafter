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
project(pointcloudmodifyer VERSION 1.0.0)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# Suppress VTK deprecation warnings
add_definitions(-DVTK_DEPRECATION_LEVEL=0)

# Find PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Include project headers
include_directories(include)

# Define library sources
set(LIBRARY_SOURCES
    src/pointcloudmodifyer.cpp
)

# Define library headers
set(LIBRARY_HEADERS
    include/pointcloudmodifyer.hpp
)

# Create shared library
add_library(${PROJECT_NAME} SHARED ${LIBRARY_SOURCES})
target_include_directories(${PROJECT_NAME}
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
# Link PCL libraries
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

# Set the library version
set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    PUBLIC_HEADER "${LIBRARY_HEADERS}"
)

# Installation rules
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    PUBLIC_HEADER DESTINATION include
)

# Turn on examples by default
option(BUILD_EXAMPLES "Build example applications" OFF)
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()