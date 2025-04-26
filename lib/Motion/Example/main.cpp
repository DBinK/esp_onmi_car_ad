// #include <Arduino.h>
// #include "Motion.hpp"

// Motion motionController= Motion(encoder_pins, motor_pins, posVals, rateVals, msPos, msRate);

// void setup() {
//     Serial.begin(115200);
//     Serial.println("Motion Controller Initialized");


//     // 设置目标位置
//     motionController.set_pos_target_value(MotorIndex::LF, 100.0);
//     motionController.set_pos_target_value(MotorIndex::RF, 100.0);
//     motionController.set_pos_target_value(MotorIndex::LB, 100.0);
//     motionController.set_pos_target_value(MotorIndex::RB, 100.0);
// }

// void loop() {
//     // 主循环中可以添加其他逻辑
//     delay(1000); // 每秒打印一次状态
//     Serial.print("LF Position: ");
//     Serial.print(msPos.LF);
//     Serial.print(", LF Speed: ");
//     Serial.println(msRate.LF);
// }