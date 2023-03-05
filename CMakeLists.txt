# Copyright 2018-2022, Collabora, Ltd.
# SPDX-License-Identifier: BSL-1.0

cmake_minimum_required(VERSION 3.10.2)
project(thingyTHING VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Get Monado's CMake modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/monado/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/monado/cmake/sanitizers")


if(NOT MSVC)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pedantic -Wall -Wextra -Wno-unused-parameter -Werror=incompatible-pointer-types")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter")
	set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")
	set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -Wl,--no-undefined")
endif()

# Default to PIC code
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(MONADO_SRC_XRT ${CMAKE_CURRENT_SOURCE_DIR}/monado/src/xrt)

### Set configuration options for Monado
set(XRT_HAVE_JPEG OFF) # No we don't
set(ZLIB_FOUND OFF) # No we didn't

set(XRT_FEATURE_COMPOSITOR_MAIN OFF) # we are not using this
set(XRT_FEATURE_COMPOSITOR_NULL ON) # Whatever
set(XRT_FEATURE_OPENXR OFF) # we need to replace this to pull together a different combination of modules
set(XRT_FEATURE_IPC OFF)
set(XRT_FEATURE_RENDERDOC OFF)
set(XRT_FEATURE_SERVICE OFF)
set(XRT_FEATURE_SERVICE_SYSTEMD OFF)
set(XRT_FEATURE_STEAMVR_PLUGIN OFF) # for now. we'll see what transpires
set(XRT_FEATURE_TRACING OFF) #ditto
set(XRT_FEATURE_SLAM OFF) #Not using it

set(XRT_FEATURE_CLIENT_DEBUG_GUI OFF)


set(XRT_MODULE_AUX_VIVE ON)
set(XRT_MODULE_MERCURY_HANDTRACKING ON)
# set(XRT_FEATURE_PROBER OFF)
set(XRT_MODULE_MONADO_CLI OFF)
set(XRT_MODULE_MONADO_GUI OFF)

set(XRT_HAVE_D3D11 OFF)
set(XRT_HAVE_D3D12 OFF)

# No drivers!
set(XRT_BUILD_DRIVER_ANDROID OFF)
set(XRT_BUILD_DRIVER_ARDUINO OFF)
set(XRT_BUILD_DRIVER_DAYDREAM OFF)
set(XRT_BUILD_DRIVER_DEPTHAI OFF)
set(XRT_BUILD_DRIVER_EUROC OFF)
set(XRT_BUILD_DRIVER_HANDTRACKING OFF)
set(XRT_BUILD_DRIVER_TWRAP OFF)
set(XRT_BUILD_DRIVER_HDK OFF)
set(XRT_BUILD_DRIVER_HYDRA OFF)
set(XRT_BUILD_DRIVER_ILLIXR OFF)
set(XRT_BUILD_DRIVER_NS OFF)
set(XRT_BUILD_DRIVER_OHMD OFF)
set(XRT_BUILD_DRIVER_OPENGLOVES OFF)
set(XRT_BUILD_DRIVER_PSMV OFF)
set(XRT_BUILD_DRIVER_PSVR OFF)
set(XRT_BUILD_DRIVER_QWERTY OFF)
set(XRT_BUILD_DRIVER_REALSENSE OFF)
set(XRT_BUILD_DRIVER_REMOTE OFF)
set(XRT_BUILD_DRIVER_RIFT_S OFF)
set(XRT_BUILD_DRIVER_SURVIVE OFF)
set(XRT_BUILD_DRIVER_ULV2 OFF)
set(XRT_BUILD_DRIVER_VF OFF)
set(XRT_BUILD_DRIVER_VIVE OFF)
set(XRT_BUILD_DRIVER_WMR OFF)
set(XRT_BUILD_DRIVER_SIMULAVR OFF)

# There is no clean way to disable the prober/builders (as far as I can see?)
# So we enable one driver so that the legacy builder does not die
set(XRT_BUILD_DRIVER_SIMULATED ON)

set(XRT_BUILD_SAMPLES OFF)

set(BUILD_TESTING OFF)
set(BUILD_DOC OFF)


# Make sure we have pretty colours
option (FORCE_COLORED_OUTPUT "Always produce ANSI-colored output (GNU/Clang only)." TRUE)

set(ONNXRuntime_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}\\deps\\onnxruntime\\onnxruntime-win-x64-1.13.1\\lib\\onnxruntime.lib")
set(ONNXRuntime_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}\\deps\\onnxruntime\\onnxruntime-win-x64-1.13.1\\include")

message("${CMAKE_CURRENT_SOURCE_DIR}")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")


find_package(
	OpenCV
	REQUIRED
	COMPONENTS
		core
		#calib3d
		#highgui
		#imgproc
		# imgcodecs
		#features2d
		video
		CONFIG
	)
find_package(
	OPENVR
	REQUIRED
)

add_subdirectory(monado)

add_subdirectory(src)
add_subdirectory(attic/cs_test)