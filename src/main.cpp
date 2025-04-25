
#include <cmath>
#include "Motor.hpp"
#include "HXCEncoder.hpp"

HXC::Encoder encoder(4, 6);
Motor motor(1, 2);

void encoder_loop(void *parameter) {
    Serial.print(encoder.get_count());
    Serial.print(",");
    Serial.println(encoder.get_speed());
    delay(300);
}

void motor_loop(void *parameter) {

    while (1) {
        Serial.println("Set  speed to 20");
        motor.setSpeed(20);
        delay(3000);

        Serial.println("Set  speed to 50");
        motor.setSpeed(50);
        delay(3000);

        Serial.println("Set  speed to 80");
        motor.setSpeed(80);
        delay(3000);

        Serial.println("Set  speed to 30");
        motor.setSpeed(30);
        delay(3000);
        
        Serial.println("Set  speed to 0");
        motor.setSpeed(20);
        delay(5000);
    }
}

void setup() {
    //调试串口初始化
    Serial.begin(115200);
    Serial.println("start");   
    encoder.setup();//初始化 
    encoder.set_filter(1);//设置脉冲去抖动滤波器的时间常数,单位纳秒

    // 创建遥控数据处理任务
    // xTaskCreate(
    //     encoder_loop,   // 任务函数
    //     "encoder_loop",     // 任务名称
    //     2048,             // 任务堆栈大小
    //     NULL,             // 传递给任务的参数
    //     1,                // 任务优先级
    //     NULL              // 任务句柄
    // );

    xTaskCreate(
        motor_loop,   // 任务函数
        "motor_loop",     // 任务名称
        2048,             // 任务堆栈大小
        NULL,             // 传递给任务的参数
        1,                // 任务优先级
        NULL              // 任务句柄
    );
}

void loop() {
   // 
   Serial.print(encoder.get_count());
   Serial.print(",");
   Serial.print(encoder.get_speed());
   Serial.print(",");
   uint32_t speed_kHz = static_cast<int>(round(encoder.get_speed() / 1000));
   Serial.println(speed_kHz);
   delay(100);
}