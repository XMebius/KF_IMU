/*
 * @file KalmanFilter.cpp
 * @brief 
 */

#include "KalmanFilter.h"

KalmanFilter::KalmanFilter():kf_lcm("udpm://239.255.76.67:7667?ttl=1&recv_buf_size=8388608") {
//参数初始化设置
    A = Eigen::MatrixXd(3, 3); //系统状态矩阵
    A << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
    H = Eigen::MatrixXd(3, 3); //测量矩阵
    H << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
    Q = Eigen::MatrixXd(3, 3); //（预测）过程噪音
    Q << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
    R = Eigen::MatrixXd(3, 3); //真实传感器噪音，查wheeltec的手册可得知
    R << 0.05, 0, 0,            //R太大，卡尔曼滤波响应会变慢
            0, 0.05, 0,
            0, 0, 0.05;

    if (!kf_lcm.good()) {
        throw std::runtime_error("LCM initialization failed!");
    }
}

void KalmanFilter::KFrun(lcm_original imu_data_original) {
    _imu_data_original = imu_data_original;
    // 发布话题
    auto result = kf_lcm.publish("IMU_DATA_ORIGINAL", &_imu_data_original);

    if (firstRun) {
        float ax = imu_data_original.accx_o;
        float ay = imu_data_original.accy_o;
        float az = imu_data_original.accz_o;
        X << ax, ay, az;
        firstRun = false;
    }

    float ax = imu_data_original.accx_o;
    float ay = imu_data_original.accy_o;
    float az = imu_data_original.accz_o;
    Eigen::MatrixXd z;
    z = Eigen::MatrixXd(3, 1);
    z << ax, ay, az;
    X = predictAndupdate(X, z);
    _imu_data_processed.accx_p = X(0, 0);
    _imu_data_processed.accy_p = X(1, 0);
    _imu_data_processed.accz_p = X(2, 0);
    kf_lcm.publish("IMU_DATA_PROCESSED", &_imu_data_processed);
    // debug
//    std::cout << "Published IMU_DATA_PROCESSED: " << _imu_data_processed.accx_p << ", "
//              << _imu_data_processed.accy_p << ", " << _imu_data_processed.accz_p << std::endl;
//    kf_lcm.handle();
}

Eigen::MatrixXd KalmanFilter::predictAndupdate(Eigen::MatrixXd x, Eigen::MatrixXd z) {
    if (!isinitized) {
        P = Eigen::MatrixXd(3, 3); //协方差 3*3矩阵
        P << 1, 0, 0,
                0, 1, 0,
                0, 0, 1; //协方差的初始化
        isinitized = true;
    }
    x = A * x; // 状态预测方程（先验）
    P = A * P * (A.transpose()) + Q; // 预测协方差矩阵
    Eigen::MatrixXd K = P * (H.transpose()) * ((H * P * (H.transpose()) + R).inverse()); // kalman增益
    x = x + K * (z - H * x); // 状态更新（后验）
    int x_size = x.size();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
    P = (I - K * H) * P; // 协方差阵更新：
    return x;
}

KalmanFilter::~KalmanFilter() {}
