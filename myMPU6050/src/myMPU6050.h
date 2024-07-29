#define 她喜欢的人 我

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <MadgwickAHRS.h>

class MyMPU6050 {
public:
    // 默认构造函数
    MyMPU6050() {}

    // IMU初始化,IIC通讯协议，填入你的IIC地址
    bool mpuInit(TwoWire* IIC_6050);

    void begin(float frequency);

    // 得到静止状态下的测量的100个误差值
    void getDataErrorSum();

    // 将100个误差值取均值
    void getDataError();

    // 得到滤波之后的准确数值
    void dataGetAndFilter();

    // 计算欧拉角
    void IMUupdate();

    // 拿到yaw角的值
    double getYaw() const;

    // 拿到pitch角的值
    double getPitch() const;

    // 拿到Roll角的值 
    double getRoll() const;

    // 析构函数
    ~MyMPU6050() {}
    void updataMyYaw();
    double GetMyYaw();
    void calculateDisplacement();
private:

    sensors_event_t a, g, temp;
    double ax, ay, az, gx, gy, gz, temperature;
    double MyYaw;
    double vx, vy, vz, sx, sy, sz;
    double YawErrorK;
    float frequency = 0.01;
    double MPU6050ERROR[6] = {0.0};
    double Yaw = 0.0f, Pitch = 0.0f, Roll = 0.0f;                     // 偏航角，俯仰角，翻滚角
    
    Adafruit_MPU6050 mpu;
    Madgwick filter;
};

// MPU6050的一些参数
typedef struct {
    unsigned int MPU6050dt;            // 记录MPU6050的数据更新时间间隔，这里为10ms
    unsigned long preMPU6050Millis;    // 记录上一次MPU6050的数据更新时间
    unsigned int firstGetYawTime;      // 记录开始进行MPU6050的Yaw数据滤波优化时间
    unsigned long previousprintMillis; // 记录上一次打印数据时的时间

} globalMPU6050Params;
