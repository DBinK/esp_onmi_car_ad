
#include <Arduino.h>

#include <QuickPID.h>

// #include "Motor.hpp"
// #include "HXCEncoder.hpp"

#include "MotorController.hpp"
#include "vofa.hpp"
#include "config.hpp"

// 全局变量
PIDConfig POS = {25.0, // Kp
                 0.0,  // Ki
                 0.6}; // Kd

PIDConfig RATE = {0.10,  // Kp
                  0.67,  // Ki
                  0.001};// Kd

uint8_t SampleTimeMS = 10;  // PID 和 控制循环计算频率 

EncoderConfig encoder_lf_Cfg = {4, 6, 50, 10};
EncoderConfig encoder_rf_Cfg = {39, 40, 50, 10};

MotorConfig motor_lf_Cfg = {1, 2, 0, 1, 10000, 0, 1023};
MotorConfig motor_rf_Cfg = {14, 13, 2, 3, 10000, 0, 1023};

MotorController motor_lf(POS, RATE, encoder_lf_Cfg, motor_lf_Cfg, SampleTimeMS*1000);
MotorController motor_rf(POS, RATE, encoder_rf_Cfg, motor_rf_Cfg, SampleTimeMS*1000);

VOFA vofa; // 串口增强类, 用于接收 vofa+ 调试 PID 参数


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
    if (vofa.UpdatePidParams(POS.P, POS.I, POS.D, RATE.P, RATE.I, RATE.D, 
                            motor_lf.posVal.tg, motor_lf.rateVal.tg))
    {
      motor_rf.posVal.tg = motor_lf.posVal.tg;
      motor_rf.posVal.tg = motor_lf.posVal.tg;

      motor_lf.setPIDcfg(POS, RATE);
      motor_rf.setPIDcfg(POS, RATE);
    }

    motor_lf.PIDCompute();
    // motor_rf.PIDCompute();
    motor_rf.setMotorSpeedDirect(motor_lf.rateVal.out);

    Serial.printf("%f,%f,%f,%f,%f,%f\n",   // VOFA 串口输出
        motor_lf.rateVal.ms, motor_lf.rateVal.out, motor_lf.rateVal.tg, 
        motor_lf.posVal.ms, motor_lf.posVal.out, motor_lf.posVal.tg);
  }
};

void setup()
{
  Serial.begin(115200);
  Serial.printf("Start!\n");
  vofa.begin(Serial); // 初始化串口增强类

  xTaskCreate(motor_control, "motor_control", 4096, NULL, 1, NULL);
};


void loop()
{
  delay(1000);
};