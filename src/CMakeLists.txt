
add_executable(
	simple_test simple_test.cpp
)

target_link_libraries(
	simple_test
	PRIVATE
	${OpenCV_LIBRARIES}
  xrt-interfaces aux_util aux_vive t_ht_mercury
  meow_oxr_sdl2
  hand_async
  meow_util
)

target_include_directories(
	simple_test
	PRIVATE
)

add_subdirectory(util)
add_subdirectory(steamvr_driver)
