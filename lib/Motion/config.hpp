#ifndef __CONFIG_HPP__
#define __CONFIG_HPP__

#include <Arduino.h>
#include <QuickPID.h>

enum MotorIndex : uint8_t {LF, RF, RB, LB};

/* PID参数结构体 */
typedef struct {
    float P;
    float I;
    float D;
} PIDConfig;

typedef struct {
    uint16_t FW_PIN;
    uint16_t BW_PIN;
    uint16_t FW_CHANNEL;
    uint16_t BW_CHANNEL;
    uint16_t freq; // = 10000;
    uint16_t THR_MIN; // = 0;
    uint16_t THR_MAX; // = 1023;
} MotorConfig;

typedef struct { 
    uint8_t PINA;
    uint8_t PINB;
    uint16_t frc; // = 50;
    uint16_t filter_ns; // = 10;
} EncoderConfig;

/* 预定义电机PID 目标值、测量值、输出值 结构体 */
typedef struct {  
    float tg;  // target
    float ms;  // measure
    float out;   // output
} PIDCtrlVal; 

typedef struct {  // 多编码器测量值结构体
    int64_t LF = 0;
    int64_t LB = 0;
    int64_t RF = 0;
    int64_t RB = 0;
} MeasureVal;    


// 定义编码器和电机的引脚
// const uint8_t encoder_pins[8] = {4, 6, 39, 40, 21, 34, 12, 11}; // 根据实际情况修改引脚
// const uint8_t motor_pins[8] = {1, 2, 14, 13, 38, 36, 8, 10}; // 根据实际情况修改引脚


#endif