# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)
if(TARGET camera_client)
    return()
endif()
find_package(Threads REQUIRED)
add_library(camera_ipc STATIC camera_ipc.cpp stereo_local.cpp)
target_compile_features(camera_ipc PUBLIC cxx_std_17)
target_include_directories(camera_ipc PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
    "$<INSTALL_INTERFACE:include/cambold/detail>")
target_link_libraries(camera_ipc PUBLIC Threads::Threads)
set_target_properties(camera_ipc PROPERTIES CXX_EXTENSIONS OFF)
add_library(camera_client STATIC camera_client.cpp)
target_compile_features(camera_client PUBLIC cxx_std_17)
target_include_directories(camera_client PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
    "$<INSTALL_INTERFACE:include/cambold/detail>")
target_link_libraries(camera_client PUBLIC camera_ipc)
install(TARGETS camera_client camera_ipc EXPORT CamBoldSDKTargets ARCHIVE DESTINATION lib)
install(FILES camera_client.h camera_broker_protocol.h camera_ipc.hpp stereo_local.hpp
    slam_envelope.hpp slam_socket.hpp calibration_ipc.hpp DESTINATION include/cambold/detail)
