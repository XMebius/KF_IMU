cmake_minimum_required(VERSION 3.10)
project(Workspace)

set(CMAKE_CXX_STANDARD 11)

include_directories("./rt/")
include_directories("./KarmanFilter/")
include_directories("./CSerial/include/")
include_directories("/usr/local/include/eigen3")
include_directories("/usr/local/include/lcm/")
include_directories("./lcm")

add_subdirectory(rt)
add_subdirectory(KarmanFilter)

add_executable(KF_IMU main.cpp)
target_link_libraries(KF_IMU KF rt_imu)