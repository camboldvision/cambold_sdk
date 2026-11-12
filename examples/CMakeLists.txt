# SPDX-License-Identifier: Apache-2.0
add_executable(cb_camera_example camera.c)
target_link_libraries(cb_camera_example PRIVATE CamBold::SDK)
target_compile_features(cb_camera_example PRIVATE c_std_11)
if(MINGW)
    target_link_options(cb_camera_example PRIVATE -static -static-libgcc -static-libstdc++)
endif()
