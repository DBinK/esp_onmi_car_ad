
#include <Arduino.h>

#include <QuickPID.h>

#include "Motor.hpp"
#include "HXCEncoder.hpp"
#include "vofa.hpp"

HXC::Encoder encoder_lf(4, 6);
Motor motor_lf(1, 2, 0, 1, 1000);

//Define Variables we'll be connecting to
float ms, out, tg;
float Kp = 0.19, Ki = 0.19, Kd = 0.0;

QuickPID lfPID(&ms, &out, &tg, Kp, Ki, Kd, QuickPID::Action::direct);
uint16_t i = 0;
int8_t reverse = 1;

VOFA vofa;   // 串口增强类, 用于接收 vofa+ 调试 PID 参数

float Kp2 = 0.2, Ki2 = 0.08701, Kd2 = 0.000;

void setup() {
    Serial.begin(115200);
    Serial.printf("Start!\n");    
    vofa.begin(Serial);  // 初始化串口增强类
    
    encoder_lf.set_filter(100); // 设置脉冲去抖动滤波器的时间常数,单位纳秒
    encoder_lf.setup(50);       // 初始化编码器并设置编码器采样频率,单位Hz 

    motor_lf.setSpeedLimit(0, 1023);
    motor_lf.setSpeed(0);

    lfPID.SetMode(1);
    lfPID.SetOutputLimits(-950, 950);
    tg = 1;
};

void loop() {

    delay(10);

    int64_t count = encoder_lf.get_count();
    float speed = encoder_lf.get_speed();

    // ms = count;
    ms = speed;
    lfPID.Compute();

    motor_lf.setSpeed(out);

    // Serial.printf("ms:%f, out:%f, tg:%f\n", ms, out, tg);
    // Serial.printf("count:%ld, speed:%f\n",count,speed);

    // i++;
    // if (i > 400) {
    //     tg += 3000 * reverse;
    //     i = 0;
    //     // tg = -tg;
    //     // lfPID.Reset();

    //     if (tg > 12000 || tg < 3000){
    //         reverse = -reverse;
    //     }
    // }

    Serial.printf("%f,%f,%f,%d\n", ms, out, tg, i);

    // 更新 PID 参数
    if (vofa.UpdatePidParams(Kp, Ki, Kd, Kp2, Ki2, Kd2, tg))
    {
      lfPID.SetTunings(Kp, Ki, Kd);

      Serial.println("PID参数更新成功");
    }
};