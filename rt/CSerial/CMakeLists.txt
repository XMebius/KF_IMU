include_directories(include)
file(GLOB SOURCES "src/*.cpp")

add_library(c_serial_port SHARED ${SOURCES})
target_link_libraries(c_serial_port pthread)