# Copyright 2020-2021, Collabora, Ltd.
# SPDX-License-Identifier: BSL-1.0

add_library(meow_oxr_sdl2 STATIC oxr_sdl2_hack.c)
target_link_libraries(meow_oxr_sdl2 PRIVATE aux_util)
if(XRT_HAVE_OPENGL)
	target_link_libraries(meow_oxr_sdl2 PUBLIC aux_ogl)
endif()
if(XRT_HAVE_SDL2)
	target_link_libraries(meow_oxr_sdl2 PRIVATE st_gui xrt-external-imgui-sdl2 ${SDL2_LIBRARIES})

	if(XRT_BUILD_DRIVER_QWERTY)
		target_link_libraries(meow_oxr_sdl2 PRIVATE drv_qwerty drv_qwerty_includes)
	endif()
endif()




add_library(driver_mercury MODULE driver_factory.cpp 
                device_provider.h device_provider.cpp
                mercury_device.h mercury_device.cpp
                util/win_utils.h util/win_utils.cpp
                util/path_utils.h util/path_utils.cpp
                util/bones.h util/bones.cpp)


target_link_libraries(
        driver_mercury
        PRIVATE
        xrt-external-openvr
        aux_util
        ${OpenCV_LIBRARIES}
        xrt-interfaces
        aux_vive
        t_ht_mercury # XXX meowses, 8/27: this results in link failure? Really shouldn't... 
        meow_oxr_sdl2
        meow_util
        # st_ovrd
        # st_prober
        # target_lists
        # target_instance_no_comp
)
# copied from project_northstar_openvr_driver
target_compile_definitions(driver_mercury PUBLIC -D_EXPORT)


# meta data that the steamvr plugin needs in the base directory of the steamvr plugin
file(COPY mercury/driver.vrdrivermanifest DESTINATION ${PROJECT_BINARY_DIR}/mercury)
file(COPY mercury/resources DESTINATION ${PROJECT_BINARY_DIR}/mercury)


set(PLUGINDIR "${PROJECT_BINARY_DIR}/mercury/bin/win64")

# message("SteamVR plugin path: ${PLUGINDIR}")
set_target_properties(driver_mercury PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${PLUGINDIR}")

set_target_properties(driver_mercury PROPERTIES PREFIX "")

install(
        DIRECTORY "${PROJECT_BINARY_DIR}/mercury"
        DESTINATION "${CMAKE_INSTALL_PREFIX}/share"
)
