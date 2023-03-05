add_executable(cs_test cs_test.cpp)

target_link_libraries(cs_test PRIVATE
                        xrt-interfaces
                      )

add_executable(cs_test_subprocess cs_test_subprocess.cpp)

target_link_libraries(cs_test_subprocess PRIVATE 
  ${OpenCV_LIBRARIES}
  OpenVR::OpenVR
  xrt-interfaces
  aux_util
  aux_vive
  t_ht_mercury
  meow_oxr_sdl2
  hand_async
  meow_util
  target_instance_no_comp
)

