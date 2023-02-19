
add_executable(
	THINGY 	camera_demo.cpp videoInput.h videoInput.cpp # cap_dshow.cpp cap_dshow.hpp
)

target_link_libraries(
	THINGY
	PRIVATE
	${OpenCV_LIBRARIES}
  xrt-interfaces aux_util aux_vive t_ht_mercury
  meow_oxr_sdl2
)

target_include_directories(
	THINGY
	PRIVATE
  # xrt-interfaces
  # aux_vive
)




#target_include_directories(
#	THINGY
#	PRIVATE
#	${OpenCV_HEADERS}
#)




add_subdirectory(steamvr_driver)