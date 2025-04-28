
#include <Arduino.h>

#include <QuickPID.h>

#include "Motor.hpp"
#include "HXCEncoder.hpp"
#include "vofa.hpp"
#include "config.hpp"

HXC::Encoder encoder_lf(4, 6);
Motor motor_lf(1, 2, 0, 1, 1000);

PIDConfig POS = {25.0, // Kp
                 0.0,  // Ki
                 0.60}; // Kd

PIDConfig RATE = {0.10, // Kp
                  0.67, // Ki
                  0.001}; // Kd

PIDCtrlVal rateLF, rateRF, rateRB, rateLB;
PIDCtrlVal posLF, posRF, posRB, posLB;

QuickPID pidRateLF(&rateLF.ms, &rateLF.out, &rateLF.tg,
                   RATE.P, RATE.I, RATE.D,
                   QuickPID::Action::direct);

QuickPID pidPosLF(&posLF.ms, &posLF.out, &posLF.tg,
                  POS.P, POS.I, POS.D,
                  QuickPID::Action::direct);

VOFA vofa; // 串口增强类, 用于接收 vofa+ 调试 PID 参数

uint8_t SampleTimeMS = 10;  // PID 和 控制循环计算频率 


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

    // 检查 PID 参数更新
    if (vofa.UpdatePidParams(POS.P, POS.I, POS.D, RATE.P, RATE.I, RATE.D, posLF.tg, rateLF.tg))
    {
      pidRateLF.SetTunings(RATE.P, RATE.I, RATE.D);
      pidPosLF.SetTunings(POS.P, POS.I, POS.D);
      Serial.printf("PID参数更新 pos: %f, %f, %f ; rate: %f, %f, %f \n", POS.P, POS.I, POS.D, RATE.P, RATE.I, RATE.D);
    }

    // 获取编码器数据
    int64_t count = encoder_lf.get_count();
    float speed = encoder_lf.get_speed();

    posLF.ms = count;
    rateLF.ms = speed;

    // 计算 PID
    if (POS.P > 0 && RATE.P > 0) {  // 位置环和速度环同时开启
      pidPosLF.Compute();   
      rateLF.tg = posLF.out; // 让位置环的输出作为速度环的输入
      pidRateLF.Compute();
      motor_lf.setSpeed(rateLF.out);
    }
    else if (POS.P > 0 && RATE.P == 0) {   // 仅开启位置环
      pidPosLF.Compute();   
      motor_lf.setSpeed(posLF.out);
    }
    else if (RATE.P > 0 && POS.P == 0) {    // 仅开启速度环
      pidRateLF.Compute(); 
      motor_lf.setSpeed(rateLF.out);
    }
    else { // 位置环和速度环都关闭
      motor_lf.setSpeed(0);
      Serial.printf("已关闭 PID 解算, 请设置位置或速度环P值开始调试\n");
      pidRateLF.Reset();
      pidPosLF.Reset();
      posLF.ms = 0;
      delay(1000);
    }
      
    // VOFA 串口输出
    Serial.printf("%f,%f,%f,%f,%f,%f\n",
                  rateLF.ms, rateLF.out, rateLF.tg, posLF.ms, posLF.out, posLF.tg);

  }
};

void setup()
{
  Serial.begin(115200);
  Serial.printf("Start!\n");
  vofa.begin(Serial); // 初始化串口增强类

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