
#include <Arduino.h>

#include <QuickPID.h>

#include "Motor.hpp"
#include "HXCEncoder.hpp"

HXC::Encoder encoder_lf(4, 6);
Motor motor_lf(1, 2);

//Define Variables we'll be connecting to
float ms, out, tg;
float Kp = 0.3, Ki = 0, Kd = 0;

QuickPID lfPID(&ms, &out, &tg, Kp, Ki, Kd, QuickPID::Action::direct);

void setup() {
    Serial.begin(115200);
    Serial.printf("Start!\n");    
    
    encoder_lf.set_filter(100); // 设置脉冲去抖动滤波器的时间常数,单位纳秒
    encoder_lf.setup(50);       // 初始化编码器并设置编码器采样频率,单位Hz 

    motor_lf.setSpeedLimit(0, 1023);
    motor_lf.setSpeed(0);

    lfPID.SetMode(1);
    // lfPID.SetTunings(Kp, Ki, Kd);
    lfPID.SetOutputLimits(-90, 90);
    tg = -100;
    // motor_lf.setSpeed(-80);
};

void loop() {

    int64_t count = encoder_lf.get_count();
    float speed = encoder_lf.get_speed();

    ms = count;
    lfPID.Compute();

    out = out+random(-2,2);
    motor_lf.setSpeed(out);

    Serial.printf("ms:%f, out:%f, tg:%f\n", ms, out, tg);
    // Serial.printf("count:%ld, speed:%f\n",count,speed);

    delay(100);
};