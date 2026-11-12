# SPDX-License-Identifier: Apache-2.0
set(LIBUSB_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/libusb)
set(LIBUSB_COMMON_SOURCES
    ${LIBUSB_ROOT}/libusb/core.c
    ${LIBUSB_ROOT}/libusb/descriptor.c
    ${LIBUSB_ROOT}/libusb/hotplug.c
    ${LIBUSB_ROOT}/libusb/io.c
    ${LIBUSB_ROOT}/libusb/strerror.c
    ${LIBUSB_ROOT}/libusb/sync.c)
if(WIN32)
    set(LIBUSB_CONFIG ${CMAKE_CURRENT_SOURCE_DIR}/cmake/libusb_config_windows.h)
    list(APPEND LIBUSB_SOURCES
        ${LIBUSB_ROOT}/libusb/os/events_windows.c
        ${LIBUSB_ROOT}/libusb/os/threads_windows.c
        ${LIBUSB_ROOT}/libusb/os/windows_common.c
        ${LIBUSB_ROOT}/libusb/os/windows_usbdk.c
        ${LIBUSB_ROOT}/libusb/os/windows_winusb.c)
elseif(APPLE)
    set(LIBUSB_CONFIG ${CMAKE_CURRENT_SOURCE_DIR}/cmake/libusb_config_posix.h)
    list(APPEND LIBUSB_SOURCES
        ${LIBUSB_ROOT}/libusb/os/events_posix.c
        ${LIBUSB_ROOT}/libusb/os/threads_posix.c
        ${LIBUSB_ROOT}/libusb/os/darwin_usb.c)
else()
    set(LIBUSB_CONFIG ${CMAKE_CURRENT_SOURCE_DIR}/cmake/libusb_config_posix.h)
    list(APPEND LIBUSB_SOURCES
        ${LIBUSB_ROOT}/libusb/os/events_posix.c
        ${LIBUSB_ROOT}/libusb/os/threads_posix.c
        ${LIBUSB_ROOT}/libusb/os/linux_usbfs.c
        ${LIBUSB_ROOT}/libusb/os/linux_netlink.c)
endif()

add_library(cb_libusb STATIC ${LIBUSB_COMMON_SOURCES} ${LIBUSB_SOURCES})
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/libusb-config)
target_include_directories(cb_libusb PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/libusb-config
    ${LIBUSB_ROOT}/libusb)
target_compile_definitions(cb_libusb PRIVATE LIBUSB_STATIC=1)
configure_file(${LIBUSB_CONFIG}
    ${CMAKE_CURRENT_BINARY_DIR}/libusb-config/config.h COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/libusb_version_describe.h
    ${CMAKE_CURRENT_BINARY_DIR}/libusb-config/version_describe.h COPYONLY)

# Native UVC capture belongs to the public host SDK.  The desktop executable
# consumes its C callback and owns only preview/rosbag policy.
set(LIBUVC_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/libuvc)
set(LIBUVC_CONFIG_DIR ${CMAKE_CURRENT_BINARY_DIR}/libuvc-config)
file(MAKE_DIRECTORY ${LIBUVC_CONFIG_DIR}/include/libuvc)
set(libuvc_VERSION_MAJOR 0)
set(libuvc_VERSION_MINOR 0)
set(libuvc_VERSION_PATCH 8)
set(libuvc_VERSION "0.0.8")
set(LIBUVC_HAS_JPEG OFF)
configure_file(${LIBUVC_ROOT}/include/libuvc/libuvc_config.h.in
    ${LIBUVC_CONFIG_DIR}/include/libuvc/libuvc_config.h @ONLY)
add_library(cb_libuvc STATIC
    ${LIBUVC_ROOT}/src/ctrl.c
    ${LIBUVC_ROOT}/src/ctrl-gen.c
    ${LIBUVC_ROOT}/src/device.c
    ${LIBUVC_ROOT}/src/diag.c
    ${LIBUVC_ROOT}/src/frame.c
    ${LIBUVC_ROOT}/src/init.c
    ${LIBUVC_ROOT}/src/stream.c
    ${LIBUVC_ROOT}/src/misc.c)
target_include_directories(cb_libuvc PRIVATE
    ${LIBUVC_ROOT}/include
    ${LIBUVC_CONFIG_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}/libusb-config
    ${LIBUSB_ROOT}/libusb)
target_compile_definitions(cb_libuvc PRIVATE LIBUSB_STATIC=1)
target_link_libraries(cb_libuvc PRIVATE
    cb_libusb
    Threads::Threads)

target_include_directories(cb_sdk PRIVATE
    "${LIBUSB_ROOT}/libusb" "${LIBUVC_ROOT}/include"
    "${LIBUVC_CONFIG_DIR}/include")
install(TARGETS cb_libusb cb_libuvc EXPORT CamBoldSDKTargets ARCHIVE DESTINATION lib)
install(FILES third_party/libusb/COPYING
    DESTINATION share/cambold-sdk/licenses RENAME libusb-COPYING)
install(FILES third_party/libuvc/LICENSE.txt
    DESTINATION share/cambold-sdk/licenses RENAME libuvc-LICENSE)
