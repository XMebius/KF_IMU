# KalmanFilter在机器人姿态估计中的应用——以加速度为例

作者：陈逸轩

主题：同济大学2021级传感器与检测技术口试

## 硬件

使用wheeltec轮趣公司的[N100 mini](https://github.com/NDHANA94/ros2_wheeltec_n100_imu)

## 项目架构说明

1. `rt`文件夹中含有与imu通信的代码，主要包括串口通信第三方库以及imu的数据解包
2. `lcm`中含有用于数据可视化与话题订阅收发的数据格式定义
3. `KalmanFilter`中包含卡尔曼滤波实现代码

## 运行


```bash
mkdir build && cd build
cmake ..
make
```
编译好后新开终端在build内执行
```bash
./KF_IMU
```
若想观察可视化数据，打开另一终端运行根目录下脚本
```bash
./launch_lcm-spy.sh
```
