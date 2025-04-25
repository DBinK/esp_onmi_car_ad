#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <Arduino.h>

class Motor {
    public:
        /**
         * @brief Motor 电机控制类
         * @note  支持 2 Pin 控制的 H 桥芯片, 如 AT8236 、DRV8833 等
         * @param FW_PIN   前进引脚
         * @param BW_PIN   后退引脚
         * @param THR_MIN  速度最小PWM值
         * @param THR_MAX  速度最大PWM值
         */
        Motor(uint8_t FW_PIN, uint8_t BW_PIN, uint16_t THR_MIN = 0, uint16_t THR_MAX = 1023) {
            _FW_PIN = FW_PIN;
            _BW_PIN = BW_PIN;
            _THR_MIN = THR_MIN;
            _THR_MAX = THR_MAX;
    
            pinMode(_FW_PIN, OUTPUT);
            pinMode(_BW_PIN, OUTPUT);

            analogWriteResolution(10);  // 设置PWM分辨率为10位 
        }
    
        /**
         * @brief 设置电机的转速
         * @param rate 
         */
        void setSpeed(int8_t rate) {
            int8_t pwmValue = map(abs(rate), 0, 100, _THR_MIN, _THR_MAX);
            pwmValue = constrain(pwmValue, _THR_MIN, _THR_MAX);
    
            if (rate > 0) {
                analogWrite(_FW_PIN, pwmValue);
                digitalWrite(_BW_PIN, LOW);
            } else if (rate < 0) {
                digitalWrite(_FW_PIN, LOW);
                analogWrite(_BW_PIN, pwmValue);
            } else {
                digitalWrite(_FW_PIN, LOW);
                digitalWrite(_BW_PIN, LOW);
            }
        }

        /**
         * @brief 设置电机的上下限
         * @param rate 
         */
        void setSpeedLimit(uint16_t THR_MIN, uint16_t THR_MAX) {
            _THR_MIN = THR_MIN;
            _THR_MAX = THR_MAX;
        }
    
    private:
        uint8_t _FW_PIN;
        uint8_t _BW_PIN;
        int8_t _THR_MIN;
        int8_t _THR_MAX;
    };
    
    #endif