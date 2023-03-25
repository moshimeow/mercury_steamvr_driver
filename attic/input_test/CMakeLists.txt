add_executable(server_input_test server_input_test.cpp)

target_link_libraries(server_input_test PRIVATE
                        xrt-interfaces
                        aux_util
                      )

