# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)
project(cambold_sdk VERSION 1.0.0 LANGUAGES C CXX)

if(TARGET cb_sdk)
    return()
endif()
set(_cb_default_backend USB)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(_cb_default_backend LOCAL)
endif()
set(CB_SDK_BACKEND "${_cb_default_backend}" CACHE STRING "Camera transport: USB or LOCAL")
set_property(CACHE CB_SDK_BACKEND PROPERTY STRINGS USB LOCAL)
if(NOT CB_SDK_BACKEND MATCHES "^(USB|LOCAL)$")
    message(FATAL_ERROR "CB_SDK_BACKEND must be USB or LOCAL")
endif()
if(CB_SDK_BACKEND STREQUAL "LOCAL" AND NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR "The LOCAL backend requires Linux")
endif()
option(CB_SDK_BUILD_EXAMPLES "Build the portable C SDK example" OFF)
include(GNUInstallDirs)
find_package(Threads REQUIRED)
add_subdirectory(src/slam/protocol)
add_library(cb_sdk STATIC src/cb_sdk.c src/cb_runtime.cpp src/cb_video.cpp src/cb_jpeg_metadata.cpp
    src/slam/client/client.cpp
    src/slam/model/scene.cpp src/slam/model/map_scene.cpp)
add_library(CamBold::SDK ALIAS cb_sdk)
target_compile_features(cb_sdk PRIVATE c_std_11 cxx_std_17)
set_target_properties(cb_sdk PROPERTIES C_EXTENSIONS OFF CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON EXPORT_NAME SDK)
target_include_directories(cb_sdk PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
    PRIVATE src src/slam src/local/ipc)
target_link_libraries(cb_sdk PRIVATE cambold_slam_protocol Threads::Threads)
if(CB_SDK_BACKEND STREQUAL "USB")
    include(cmake/usb.cmake)
    target_sources(cb_sdk PRIVATE src/cb_transport_usb.cpp src/cb_native_video.cpp)
    target_link_libraries(cb_sdk PRIVATE cb_libusb cb_libuvc)
    if(WIN32)
        target_link_libraries(cb_sdk PRIVATE setupapi cfgmgr32 winusb mf mfplat mfreadwrite mfuuid)
    elseif(APPLE)
        target_link_libraries(cb_sdk PRIVATE
            "-framework IOKit" "-framework CoreFoundation" "-framework Security")
    else()
        target_link_libraries(cb_sdk PRIVATE rt)
    endif()
else()
    add_subdirectory(src/local/ipc)
    target_sources(cb_sdk PRIVATE src/local/cb_transport_local.cpp src/local/cb_video_local.cpp)
    target_link_libraries(cb_sdk PRIVATE camera_client util)
    target_compile_definitions(cb_sdk PRIVATE CB_SDK_LOCAL=1)
endif()
if(CB_SDK_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
install(TARGETS cb_sdk cambold_slam_protocol EXPORT CamBoldSDKTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
get_target_property(_cb_json_imported nlohmann_json::nlohmann_json IMPORTED)
if(NOT _cb_json_imported)
    get_target_property(_cb_json_target nlohmann_json::nlohmann_json ALIASED_TARGET)
    if(NOT _cb_json_target)
        set(_cb_json_target nlohmann_json::nlohmann_json)
    endif()
    set_target_properties(${_cb_json_target} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlohmann_json/include>;$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/cambold/detail>")
    set_target_properties(${_cb_json_target} PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlohmann_json/include>;$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/cambold/detail>")
    install(TARGETS ${_cb_json_target} EXPORT CamBoldSDKTargets)
endif()
install(DIRECTORY src/slam/protocol/include/ third_party/nlohmann_json/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/cambold/detail)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES README.md LICENSE THIRD_PARTY.md DESTINATION ${CMAKE_INSTALL_DATADIR}/cambold-sdk)
install(FILES third_party/nlohmann_json/LICENSE.MIT
    DESTINATION ${CMAKE_INSTALL_DATADIR}/cambold-sdk/licenses RENAME nlohmann-json-LICENSE)
include(CMakePackageConfigHelpers)
configure_package_config_file(cmake/CamBoldSDKConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/CamBoldSDKConfig.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/CamBoldSDK")
write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/CamBoldSDKConfigVersion.cmake"
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(EXPORT CamBoldSDKTargets NAMESPACE CamBold::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/CamBoldSDK")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/CamBoldSDKConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/CamBoldSDKConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/CamBoldSDK")
