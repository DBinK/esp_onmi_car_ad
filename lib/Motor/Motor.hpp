#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <Arduino.h>

class Motor {
    public:
        /**
         * @brief Motor 电机控制类
         * @note  支持 2 Pin 控制的 H 桥芯片, 如 AT8236 、DRV8833 等
         * @param FW_PIN     前进引脚
         * @param BW_PIN     后退引脚
         * @param FW_CHANNEL 前进引脚的通道
         * @param BW_CHANNEL 后退引脚的通道
         * @param freq
         * @param THR_MIN    速度最小PWM值
         * @param THR_MAX    速度最大PWM值
         */
        Motor(uint16_t FW_PIN, uint16_t BW_PIN, 
            uint16_t FW_CHANNEL, uint16_t BW_CHANNEL, uint16_t freq=10000,
            uint16_t THR_MIN = 0, uint16_t THR_MAX = 1023) {
            _FW_PIN = FW_PIN;
            _BW_PIN = BW_PIN;
            _FW_CHANNEL = FW_CHANNEL;
            _BW_CHANNEL = BW_CHANNEL;
            _freq = freq;
            _THR_MIN = THR_MIN;
            _THR_MAX = THR_MAX;


            pinMode(_FW_PIN, OUTPUT);
            pinMode(_BW_PIN, OUTPUT);

            ledcSetup(_FW_CHANNEL, _freq, 10);  // 设置通道、频率和分辨率
            ledcAttachPin(_FW_PIN, _FW_CHANNEL);  // 将通道与引脚关联

            ledcSetup(_BW_CHANNEL, _freq, 10);  // 设置通道、频率和分辨率
            ledcAttachPin(_BW_PIN, _BW_CHANNEL);  // 将通道与引脚关联
        }
    
        /**
         * @brief 设置电机的转速
         * @param rate 
         */
        void setSpeed(int16_t rate) {
            // Serial.println(rate);
            // uint16_t pwmValue = map(abs(rate), 0, 100, _THR_MIN, _THR_MAX);
            uint16_t pwmValue = abs(rate);
            pwmValue = constrain(pwmValue, _THR_MIN, _THR_MAX);

            if (rate > 0) {
                ledcWrite(_FW_CHANNEL, pwmValue);
                ledcWrite(_BW_CHANNEL, 0);
                // Serial.printf("+%ld\n", pwmValue);

            } else if (rate < 0) {
                ledcWrite(_FW_CHANNEL, 0);
                ledcWrite(_BW_CHANNEL, pwmValue);
                // Serial.printf("-%ld\n", pwmValue);
                
            } else {
                ledcWrite(_FW_CHANNEL, 0);
                ledcWrite(_BW_CHANNEL, 0);
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
        uint8_t _FW_CHANNEL;
        uint8_t _BW_CHANNEL;
        uint16_t _freq;
        uint16_t _THR_MIN;
        uint16_t _THR_MAX;
    };
    
    #endif