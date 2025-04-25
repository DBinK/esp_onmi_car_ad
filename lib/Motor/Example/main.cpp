#include "Motor.hpp"

Motor motor(1, 2);

void setup() {
    //调试串口初始化
    Serial.begin(115200);
    Serial.println("start");
}
void loop() {
    Serial.println("Set  speed to 20");
    motor.setSpeed(20);
    delay(1000);

    Serial.println("Set  speed to 50");
    motor.setSpeed(50);
    delay(1000);

    Serial.println("Set  speed to 80");
    motor.setSpeed(80);
    delay(1000);

    Serial.println("Set  speed to 30");
    motor.setSpeed(30);
    delay(1000);

    Serial.println("Set  speed to 0");
    motor.setSpeed(0);
    delay(3000);
}