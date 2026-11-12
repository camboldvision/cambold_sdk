# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)
if(TARGET cambold_slam_protocol)
    return()
endif()
if(NOT TARGET nlohmann_json::nlohmann_json)
    add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../../../third_party/nlohmann_json"
                     "${CMAKE_CURRENT_BINARY_DIR}/json")
endif()
find_package(Threads REQUIRED)
add_library(cambold_slam_protocol STATIC src/protocol.cpp src/transport.cpp src/server.cpp src/map_stream.cpp)
target_compile_features(cambold_slam_protocol PUBLIC cxx_std_17)
target_include_directories(cambold_slam_protocol PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>"
    "$<INSTALL_INTERFACE:include/cambold/detail>")
target_link_libraries(cambold_slam_protocol PUBLIC nlohmann_json::nlohmann_json Threads::Threads)
if(WIN32)
    target_link_libraries(cambold_slam_protocol PRIVATE setupapi advapi32)
endif()
set_target_properties(cambold_slam_protocol PROPERTIES CXX_EXTENSIONS OFF)
