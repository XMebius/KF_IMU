add_subdirectory(CSerial)

add_library(rt_imu SHARED rt_wheeltec.cpp )
target_link_libraries(rt_imu c_serial_port)