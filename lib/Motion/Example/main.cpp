
#include <Arduino.h>
#include "Motion.hpp"

// 定义引脚
const int encoderPins[] = {4, 6, 39, 40, 21, 34, 12, 11}; // 编码器引脚
const int motorPins[]   = {1, 2, 14, 13, 38, 36, 8, 10};  // 电机引脚

// 创建Encoders和Motors对象
Encoders encoders(encoderPins);
Motors motors(motorPins);

void setup() {
  Serial.begin(115200);              // 初始化串口
  encoders.setup();                  // 初始化编码器
  encoders.set_encoder_filter(10);  // 设置编码器滤波器

  // 设置电机速度
  int16_t motorSpeeds[4] = {0, 222, 50, 80};
  motors.set_motor_speed(motorSpeeds);
}

void loop() {

  // 读取编码器计数
  int64_t encoderCounts[4];
  encoders.get_encoder_counts(encoderCounts);
  Serial.print("\nEncoder Counts: LF=");
  Serial.print(encoderCounts[LF]);
  Serial.print(", RF=");
  Serial.print(encoderCounts[RF]);
  Serial.print(", LB=");
  Serial.print(encoderCounts[LB]);
  Serial.print(", RB=");
  Serial.println(encoderCounts[RB]);

  // 读取编码器速度
  int32_t encoderSpeeds[4];
  encoders.get_encoder_speeds(encoderSpeeds);
  Serial.print("Encoder Speeds: LF=");
  Serial.print(encoderSpeeds[LF]);
  Serial.print(", RF=");
  Serial.print(encoderSpeeds[RF]);
  Serial.print(", LB=");
  Serial.print(encoderSpeeds[LB]);
  Serial.print(", RB=");
  Serial.println(encoderSpeeds[RB]);

  delay(1000); // 每秒读取一次
}