#include <Arduino.h>
#include <myMPU6050.h>
#include <HardwareSerial.h>

globalMPU6050Params MPU6050Params = {10, 0, 0, 0};
MyMPU6050 My6050;

// 使用STM32F407的USART1硬件接口
HardwareSerial Serial1 = HardwareSerial(USART1);
// NPU6050的IIC地址
TwoWire IIC_6050 = TwoWire(PB7, PB6);


void setup() {

    Serial1.begin(115200);

    // MPU6050初始化
    while(!My6050.mpuInit(&IIC_6050)) {
        Serial1.println("MPU6050 NOT_FOUND!");
        delay(3000);
    }
    Serial1.println("MPU6050 found!");
    
    // 设置MPU6050数据更新频率
    My6050.begin(1000.0f / (float)MPU6050Params.MPU6050dt);

    // MPU6050数据滤波优化
    My6050.getDataErrorSum();
    My6050.getDataError();
    Serial1.println("MPU6050 datafilter succeed!");

}

void loop() {
    if(millis() - MPU6050Params.preMPU6050Millis >= MPU6050Params.MPU6050dt) {
        MPU6050Params.preMPU6050Millis = millis();
        My6050.IMUupdate();
    }

    if(millis() - MPU6050Params.previousprintMillis >= 1000) {
        MPU6050Params.previousprintMillis = millis();
        Serial1.print("Roll: ");
        Serial1.print(My6050.getRoll());
        Serial1.print(", Pitch: ");
        Serial1.print(My6050.getPitch());
        Serial1.print(", Yaw: ");
        Serial1.println(My6050.getYaw());
    }
}

