# Copyright 2018-2022, Collabora, Ltd.
# SPDX-License-Identifier: BSL-1.0

cmake_minimum_required(VERSION 3.10.2)
project(thingyTHING VERSION 0.0.1)


#"-DCMAKE_TOOLCHAIN_FILE=C:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake"

# DELETE THIS - MOSES SPECIFIC HACK:
#set(CMAKE_TOOLCHAIN_FILE "C:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake")
find_package(OpenCV CONFIG REQUIRED)

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
	THINGY 	src/main.cpp
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

