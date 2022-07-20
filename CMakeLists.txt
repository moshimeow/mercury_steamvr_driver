# Copyright 2018-2022, Collabora, Ltd.
# SPDX-License-Identifier: BSL-1.0

cmake_minimum_required(VERSION 3.10.2)
project(thingyTHING VERSION 0.0.1)

message("${CMAKE_CURRENT_SOURCE_DIR}")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")



find_package(
	OpenCV
	REQUIRED
	COMPONENTS
		core
		calib3d
		highgui
		imgproc
		imgcodecs
		features2d
		video
		CONFIG
	)

# Dear god what is this
function(ocv_add_external_target name inc link def)
  if(BUILD_SHARED_LIBS AND link)
    set(imp IMPORTED)
  endif()
  add_library(ocv.3rdparty.${name} INTERFACE ${imp})
  if(def)
    if(NOT (CMAKE_VERSION VERSION_LESS "3.11.0"))  # https://gitlab.kitware.com/cmake/cmake/-/merge_requests/1264 : eliminates "Cannot specify compile definitions for imported target" error message
      target_compile_definitions(ocv.3rdparty.${name} INTERFACE "${def}")
    else()
      set_target_properties(ocv.3rdparty.${name} PROPERTIES INTERFACE_COMPILE_DEFINITIONS "${def}")
    endif()
  endif()
  if(inc)
    if(NOT (CMAKE_VERSION VERSION_LESS "3.11.0"))  # https://gitlab.kitware.com/cmake/cmake/-/merge_requests/1264 : eliminates "Cannot specify compile definitions for imported target" error message
      target_include_directories(ocv.3rdparty.${name} SYSTEM INTERFACE "$<BUILD_INTERFACE:${inc}>")
    else()
      set_target_properties(ocv.3rdparty.${name} PROPERTIES
          INTERFACE_INCLUDE_DIRECTORIES "$<BUILD_INTERFACE:${inc}>"
          INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "$<BUILD_INTERFACE:${inc}>"
      )
    endif()
  endif()
  if(link)
    # When cmake version is greater than or equal to 3.11, INTERFACE_LINK_LIBRARIES no longer applies to interface library
    # See https://github.com/opencv/opencv/pull/18658
    if(CMAKE_VERSION VERSION_LESS 3.11)
      set_target_properties(ocv.3rdparty.${name} PROPERTIES
        INTERFACE_LINK_LIBRARIES "${link}")
    else()
      target_link_libraries(ocv.3rdparty.${name} INTERFACE ${link})
    endif()
  endif()
  # to install used target only upgrade CMake
  if(NOT BUILD_SHARED_LIBS
      AND CMAKE_VERSION VERSION_LESS "3.13.0"  # https://gitlab.kitware.com/cmake/cmake/-/merge_requests/2152
  )
    install(TARGETS ocv.3rdparty.${name} EXPORT OpenCVModules)
  endif()
endfunction()

# Stolen from opencv/modules/videoio/cmake/detect_dshow.cmake
# --- VideoInput/DirectShow ---
if(NOT HAVE_DSHOW AND MSVC AND NOT MSVC_VERSION LESS 1500)
  set(HAVE_DSHOW TRUE)
endif()

if(NOT HAVE_DSHOW)
  check_include_file(dshow.h HAVE_DSHOW)
endif()

if(HAVE_DSHOW)
  ocv_add_external_target(dshow "" "" "HAVE_DSHOW")
endif()

#add_library(xrt-pthreads INTERFACE)
#if(WIN32)
#	find_package(pthreads_windows REQUIRED)
#	target_link_libraries(xrt-pthreads INTERFACE PThreads4W::PThreads4W_CXXEXC)
#else()
#	set(CMAKE_THREAD_PREFER_PTHREAD ON)
#	find_package(Threads)
#	target_link_libraries(xrt-pthreads INTERFACE Threads::Threads)
#	target_compile_definitions(xrt-pthreads INTERFACE _GNU_SOURCE)
#endif()

add_executable(
	THINGY 	src/main.cpp src/videoInput.h src/videoInput.cpp # src/cap_dshow.cpp src/cap_dshow.hpp
)

target_link_libraries(
	THINGY
	PRIVATE
	${OpenCV_LIBRARIES}
)

#target_include_directories(
#	THINGY
#	PRIVATE
#	${OpenCV_HEADERS}
#)

