add_library(KF SHARED KarmanFilter.cpp)
target_link_libraries(KF PUBLIC lcm)