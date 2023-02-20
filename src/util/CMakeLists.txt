# Copyright 2023, Moshi Turner
# SPDX-License-Identifier: BSL-1.0



add_library(meow_util STATIC CameraIdxGuesser.cpp CameraIdxGuesser.hpp CameraIdxGuesser_win_capture.h driver_log.cpp driver_log.h)


target_include_directories(meow_util INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(
        meow_util
        PRIVATE
        aux_util
        xrt-external-openvr
        ${OpenCV_LIBRARIES}
)
