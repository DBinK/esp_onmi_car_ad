
#include <Arduino.h>

#include <QuickPID.h>

// #include "Motor.hpp"
// #include "HXCEncoder.hpp"

#include "MotorController.hpp"
#include "vofa.hpp"
#include "config.hpp"

// 全局变量
// PIDConfig POS = {25.0, // Kp
//                  0.0,  // Ki
//                  0.6}; // Kd

// PIDConfig RATE = {0.10, // Kp
//                   0.67, // Ki
//                   0.001};// Kd


PIDConfig POS = {0.0, // Kp
                 0.0,  // Ki
                 0.0}; // Kd

PIDConfig RATE = {0.0, // Kp
                  0.0, // Ki
                  0.0};// Kd

PIDCtrlVal rateLF, rateRF, rateRB, rateLB;
PIDCtrlVal posLF, posRF, posRB, posLB;

EncoderConfig encoderCfg = {4, 6, 50, 10};
MotorConfig motorCfg = {1, 2, 0, 1, 10000, 0, 1023};

uint8_t SampleTimeMS = 10;  // PID 和 控制循环计算频率 

MotorController motor_lf(POS, RATE, encoderCfg, motorCfg, SampleTimeMS*1000);

VOFA vofa; // 串口增强类, 用于接收 vofa+ 调试 PID 参数

// Motor motor_rf(14, 13, 4, 5, 1000);

// // 电机用对象
// HXC::Encoder encoder_lf(4, 6);
// Motor motor_lf(1, 2, 0, 1);

// QuickPID pidRateLF(&rateLF.ms, &rateLF.out, &rateLF.tg,
//                    RATE.P, RATE.I, RATE.D,
//                    QuickPID::Action::direct);

// QuickPID pidPosLF(&posLF.ms, &posLF.out, &posLF.tg,
//                   POS.P, POS.I, POS.D,
//                   QuickPID::Action::direct);


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
      motor_lf.setPIDcfg(POS, RATE);
      // Serial.printf("PID参数更新 pos: %f, %f, %f ; rate: %f, %f, %f \n", POS.P, POS.I, POS.D, RATE.P, RATE.I, RATE.D);
    }

    motor_lf.PIDCompute(1);
    // motor_lf.setMotorSpeedDirect(motor_lf.posVal.tg);
    // motor_lf.setMotorSpeedDirect(500);
    // motor_rf.setSpeed(motor_lf.rateVal.tg);
    
    // motor_rf.setSpeed(700);
    
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
  
  // motor_rf.setSpeed(700);
};


void loop()
{
  delay(1000);
};