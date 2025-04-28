
#include <Arduino.h>

#include <QuickPID.h>

#include "Motor.hpp"
#include "HXCEncoder.hpp"
#include "vofa.hpp"
#include "config.hpp"

// 全局变量
PIDConfig POS = {25.0, // Kp
                 0.0,  // Ki
                 0.60}; // Kd

PIDConfig RATE = {0.10, // Kp
                  0.67, // Ki
                  0.001}; // Kd

PIDCtrlVal rateLF, rateRF, rateRB, rateLB;
PIDCtrlVal posLF, posRF, posRB, posLB;

// VOFA vofa; // 串口增强类, 用于接收 vofa+ 调试 PID 参数

uint8_t SampleTimeMS = 10;  // PID 和 控制循环计算频率 

// 电机用对象
HXC::Encoder encoder_lf(4, 6);
Motor motor_lf(1, 2, 0, 1, 10000);

QuickPID pidRateLF(&rateLF.ms, &rateLF.out, &rateLF.tg,
                   RATE.P, RATE.I, RATE.D,
                   QuickPID::Action::direct);

QuickPID pidPosLF(&posLF.ms, &posLF.out, &posLF.tg,
                  POS.P, POS.I, POS.D,
                  QuickPID::Action::direct);


void motor_control(void *parameter)
{
  static TickType_t xLastWakeTime = 0; // 用于 vTaskDelayUntil 的变量

  while (1)
  {
    if (xLastWakeTime == 0)  // 修正 vTaskDelayUntil 的使用
    {
      xLastWakeTime = xTaskGetTickCount();
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SampleTimeMS)); // 10ms

    posLF.ms  = encoder_lf.get_count();
    rateLF.ms = encoder_lf.get_speed();

    pidPosLF.Compute();   
    rateLF.tg = posLF.out; // 让位置环的输出作为速度环的输入
    pidRateLF.Compute();
    motor_lf.setSpeed(rateLF.out);
  }
};

void setup()
{
  Serial.begin(115200);
  Serial.printf("Start!\n");

  // encoder_lf.set_filter(100); // 设置脉冲去抖动滤波器的时间常数,单位纳秒
  encoder_lf.setup(50);       // 初始化编码器并设置编码器采样频率,单位Hz

  motor_lf.setSpeedLimit(0, 1023);
  motor_lf.setSpeed(0);

  pidRateLF.SetMode(1);
  pidRateLF.SetSampleTimeUs(SampleTimeMS * 1000);  
  pidRateLF.SetOutputLimits(-950, 950);

  pidPosLF.SetMode(1);
  pidPosLF.SetSampleTimeUs(SampleTimeMS * 1000);
  pidPosLF.SetOutputLimits(-9500, 9500);

  rateLF.tg = 0;
  posLF.tg = 0;

  xTaskCreate(motor_control, "motor_control", 2048, NULL, 1, NULL);
};



void loop()
{
  delay(1000);
};