add_library(KF SHARED KalmanFilter.cpp)
target_link_libraries(KF PUBLIC lcm)