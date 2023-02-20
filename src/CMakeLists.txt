
add_executable(
	THINGY 	camera_demo.cpp videoInput.h videoInput.cpp # cap_dshow.cpp cap_dshow.hpp
)

target_link_libraries(
	THINGY
	PRIVATE
	${OpenCV_LIBRARIES}
  xrt-interfaces aux_util aux_vive t_ht_mercury
  meow_oxr_sdl2
  hand_async
  #escapi_dll
  #escapi_interface
)



target_include_directories(
	THINGY
	PRIVATE
  # xrt-interfaces
  # aux_vive
)




add_executable(
	THINGY2 	camera_demo_2.cpp 
)

target_link_libraries(
	THINGY2
	PRIVATE
	${OpenCV_LIBRARIES}
  xrt-interfaces aux_util aux_vive t_ht_mercury
  meow_oxr_sdl2
  hand_async
  #escapi_dll
  #escapi_interface
)



target_include_directories(
	THINGY2
	PRIVATE
  # xrt-interfaces
  # aux_vive
)


add_executable(
	ESCAPI_TEST 	escapi_test.cpp 
)

target_link_libraries(
	ESCAPI_TEST
	PRIVATE
	${OpenCV_LIBRARIES}
  xrt-interfaces aux_util aux_vive t_ht_mercury
  meow_oxr_sdl2
  hand_async
  meow_escapi
  #escapi_interface
)






add_subdirectory(steamvr_driver)