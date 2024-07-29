#define 她喜欢的人 我

#include "myMPU6050.h"

// IMU初始化,传入参数为IIC_6050的地址
bool MyMPU6050::mpuInit(TwoWire* IIC_6050) {

    if (!mpu.begin(MPU6050_I2CADDR_DEFAULT, IIC_6050, 0)) {
        return false;
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
    return true;

}

// 设置MPU6050数据更新频率
void MyMPU6050::begin(float frequency) {
    this->frequency = frequency;
    filter.begin(frequency);
}

// 得到静止状态下的测量的100个误差值
void MyMPU6050::getDataErrorSum() {
    for(int i = 0; i < 100; i++) {
        mpu.getEvent(&a, &g, &temp);
        MPU6050ERROR[0] += a.acceleration.x;
        MPU6050ERROR[1] += a.acceleration.y;
        MPU6050ERROR[2] += a.acceleration.z - 9.8;
        MPU6050ERROR[3] += g.gyro.x;
        MPU6050ERROR[4] += g.gyro.y;
        MPU6050ERROR[5] += g.gyro.z;
        delay(10);
    }
}

// 将100个误差值取均值
void MyMPU6050::getDataError() {
    for(int i = 0; i < 6; i++) {
        MPU6050ERROR[i] /= 100.0;
    }
}

// 得到滤波之后的准确数值
void MyMPU6050::dataGetAndFilter() {
    mpu.getEvent(&a, &g, &temp);
    ax = a.acceleration.x - MPU6050ERROR[0];
    ay = a.acceleration.y - MPU6050ERROR[1];
    az = a.acceleration.z - MPU6050ERROR[2];
    gx = (g.gyro.x - MPU6050ERROR[3]) * RAD_TO_DEG;
    gy = (g.gyro.y - MPU6050ERROR[4]) * RAD_TO_DEG;
    gz = (g.gyro.z - MPU6050ERROR[5]) * RAD_TO_DEG;
    temperature = temp.temperature;
} 

// 计算欧拉角
void MyMPU6050::IMUupdate() {
    dataGetAndFilter();
    filter.updateIMU(gx, gy, gz, ax, ay, az);
    Yaw = filter.getYaw();
    Roll = filter.getRoll();
    Pitch = filter.getPitch();
}

// 拿到yaw角的值
double MyMPU6050::getYaw() const {
    return Yaw;
}

// 拿到pitch角的值
double MyMPU6050::getPitch() const { 
    return Pitch;
}

// 拿到Roll角的值 
double MyMPU6050::getRoll() const {
    return Roll;
}


// 直接对z轴角速度积分得到的Yaw，仅供测试使用
void MyMPU6050::updataMyYaw() {
    MyYaw += gz / frequency;
}

// 得到z轴角速度积分得到的Yaw，仅供测试使用
double MyMPU6050::GetMyYaw() {
    return MyYaw;
}

// 加速度二重积分得位移，仅供测试使用
void MyMPU6050::calculateDisplacement() {
    vx += ax / frequency;
    vy += ay / frequency;
    vz += (az - 9.8) / frequency;

    sx += vx / frequency;
    sy += vy / frequency;
    sz += vz / frequency;
}
