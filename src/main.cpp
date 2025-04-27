
#include <Arduino.h>

#include <QuickPID.h>

#include "Motor.hpp"
#include "HXCEncoder.hpp"
#include "vofa.hpp"
#include "config.hpp"

HXC::Encoder encoder_lf(4, 6);
Motor motor_lf(1, 2, 0, 1, 1000);

PIDConfig POS = {0.196,  // Kp
                 0.229,  // Ki
                 0.0};   // Kd

PIDConfig RATE = {0.19,  // Kp  
                  0.19,  // Ki
                  0.0};  // Kd

PIDCtrlVal rateLF, rateRF, rateRB, rateLB;
PIDCtrlVal posLF, posRF, posRB, posLB;


QuickPID pidRateLF(&rateLF.ms, &rateLF.out, &rateLF.tg,
                    RATE.P, RATE.I, RATE.D,
                    QuickPID::Action::direct);

QuickPID pidPosLF(&posLF.ms, &posLF.out, &posLF.tg,
                  POS.P, POS.I, POS.D,
                  QuickPID::Action::direct);

VOFA vofa;   // 串口增强类, 用于接收 vofa+ 调试 PID 参数


void setup() {
    Serial.begin(115200);
    Serial.printf("Start!\n");    
    vofa.begin(Serial);  // 初始化串口增强类
    
    encoder_lf.set_filter(100); // 设置脉冲去抖动滤波器的时间常数,单位纳秒
    encoder_lf.setup(50);       // 初始化编码器并设置编码器采样频率,单位Hz 

    motor_lf.setSpeedLimit(0, 1023);
    motor_lf.setSpeed(0);

    pidRateLF.SetMode(1);
    pidRateLF.SetOutputLimits(-950, 950);

    pidPosLF.SetMode(1);
    pidPosLF.SetOutputLimits(-9500, 9500);

    rateLF.tg = 0;
    posLF.tg = 0;
};

void loop() {

    delay(10);

    int64_t count = encoder_lf.get_count();
    float speed = encoder_lf.get_speed();

    posLF.ms = count;
    rateLF.ms = speed;
    
    pidPosLF.Compute();

    rateLF.tg  = posLF.out;  // 让位置环的输出作为速度环的输入
    pidRateLF.Compute();

    motor_lf.setSpeed(rateLF.out);

    Serial.printf("%f,%f,%f,%f,%f,%f\n", 
      rateLF.ms, rateLF.out, rateLF.tg, posLF.ms, posLF.out, posLF.tg);

    // 更新 PID 参数
    if (vofa.UpdatePidParams(POS.P, POS.I, POS.D, RATE.P, RATE.I, RATE.D, posLF.tg))
    {
      pidRateLF.SetTunings(RATE.P, RATE.I, RATE.D);
      pidPosLF.SetTunings(POS.P, POS.I, POS.D);

      Serial.println("PID参数更新成功");
    }
};