/*
 * @file KalmanFilter.h
 * @brief 
 */

#ifndef WORKSPACE_KALMANFILTER_H
#define WORKSPACE_KALMANFILTER_H

#include <fstream>
#include <iostream>
#include "lcm_processed.hpp"
#include "lcm_original.hpp"
#include <lcm/lcm-cpp.hpp>
#include "Eigen/Dense"

Eigen::MatrixXd X = Eigen::MatrixXd(3, 1); //创建一个3*1矩阵

using namespace std;

class KalmanFilter {
public:
    KalmanFilter();

    Eigen::MatrixXd predictAndupdate(Eigen::MatrixXd x, Eigen::MatrixXd z);

    void KFrun(lcm_original imu_data_original);

    ~KalmanFilter();

    lcm::LCM kf_lcm;
private:
    Eigen::MatrixXd A; //系统状态矩阵
    Eigen::MatrixXd P; //协方差
    Eigen::MatrixXd Q; //测量过程噪音（预测）
    Eigen::MatrixXd R; //真实传感器噪音
    Eigen::MatrixXd H; //测量矩阵

    Eigen::MatrixXd X = Eigen::MatrixXd(3, 1); //创建一个3*1矩阵

    bool firstRun = true;

    lcm_original _imu_data_original;
    lcm_processed _imu_data_processed;

    bool isinitized = false; //判断是否进行了初始化
};


#endif //WORKSPACE_KALMANFILTER_H
