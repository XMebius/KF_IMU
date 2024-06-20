#include <iostream>
#include <rt_wheeltec.h>
#include <KalmanFilter.h>
#include "lcm_original.hpp"
#include <thread>
#include <chrono>

int main() {

    void *pSerialPort = CSerialPortMalloc();

    lcm_original imu_data_original;
    KalmanFilter kf;

    init_port(pSerialPort);

    while (true) {
        read_imu(pSerialPort, imu_data_original);
        kf.KFrun(imu_data_original);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));  // delay
    }

    CSerialPortFree(pSerialPort);
    return 0;
}