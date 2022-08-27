# Copyright 2020-2021, Collabora, Ltd.
# SPDX-License-Identifier: BSL-1.0

add_library(driver_mercury MODULE driver_factory.cpp device_provider.h device_provider.cpp mercury_device.h mercury_device.cpp util/driver_log.h util/driver_log.cpp util/win_utils.h util/win_utils.cpp util/vr_utils.h util/vr_utils.cpp util/video_input.h util/video_input.cpp util/bones.h util/bones.cpp)


target_link_libraries(
        driver_mercury
        PRIVATE
        xrt-external-openvr
        aux_util
        ${OpenCV_LIBRARIES}
        xrt-interfaces
        aux_vive
        t_ht_mercury # XXX meowses, 8/27: this results in link failure? Really shouldn't... 
        # st_ovrd
        # st_prober
        # target_lists
        # target_instance_no_comp
)
# copied from project_northstar_openvr_driver
target_compile_definitions(driver_mercury PUBLIC -D_EXPORT)

# openvr/samples/driver_sample
# note - what is CMAKE_DL_LIBS?
# i THINK THESE ARE getting expanded to just nothing
target_link_libraries(driver_mercury
        ${OPENVR_LIBRARIES}
        ${CMAKE_DL_LIBS}
        )

# # meta data that the steamvr plugin needs in the base directory of the steamvr plugin
file(COPY mercury/driver.vrdrivermanifest DESTINATION ${PROJECT_BINARY_DIR}/mercury)
file(COPY mercury/resources DESTINATION ${PROJECT_BINARY_DIR}/mercury)

# determine the output directory for the steamvr plugin
# if(WIN32)
# 	# FIXME need to account for different architectures
# 	if("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
# 		set(PLUGINDIR "${PROJECT_BINARY_DIR}/steamvr-monado/bin/win64")
# 	else()
# 		set(PLUGINDIR "${PROJECT_BINARY_DIR}/steamvr-monado/bin/win32")
# 	endif()
# elseif(APPLE)
# 	if("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
# 		set(PLUGINDIR "${PROJECT_BINARY_DIR}/steamvr-monado/bin/osx64")
# 	else()
# 		set(PLUGINDIR "${PROJECT_BINARY_DIR}/steamvr-monado/bin/osx32")
# 	endif()
# elseif(NOT ANDROID)
# 	if("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
# 		set(PLUGINDIR "${PROJECT_BINARY_DIR}/steamvr-monado/bin/linux64")
# 	else()
# 		set(PLUGINDIR "${PROJECT_BINARY_DIR}/steamvr-monado/bin/linux32")
# 	endif()
# endif()

# didn't make much sense but SteamVR seemed to want win32. please replace if you figure this out
# what? ok it should definitely be win64. what the heck
set(PLUGINDIR "${PROJECT_BINARY_DIR}/mercury/bin/win64")

# set


# message("SteamVR plugin path: ${PLUGINDIR}")
set_target_properties(driver_mercury PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${PLUGINDIR}")

# don't add lib prefix to driver_monado.so
set_target_properties(driver_mercury PROPERTIES PREFIX "")

install(
        DIRECTORY "${PROJECT_BINARY_DIR}/mercury"
        DESTINATION "${CMAKE_INSTALL_PREFIX}/share"
)
