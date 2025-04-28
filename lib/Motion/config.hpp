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
    uint8_t FW_PIN;
    uint8_t BW_PIN;
    uint8_t FW_CHANNEL;
    uint8_t BW_CHANNEL;
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


/* 电机PID参数 */
// PIDConfig POS = {0.1, 0, 0};   // 位置 PID 参数
// PIDConfig RATE = {0, 0, 0};  // 速度 PID 参数

// PIDCtrlVal posLF, posRF, posRB, posLB;
// PIDCtrlVal rateLF, rateRF, rateRB, rateLB;

// PIDCtrlVal posVals[4] = {posLF, posRF, posRB, posLB};
// PIDCtrlVal rateVals[4] = {rateLF, rateRF, rateRB, rateLB};

// // 构造数组元素
// PIDCtrlVal posVals[4] = {
//     {posLF.tg, posLF.ms, posLF.out},
//     {posRF.tg, posRF.ms, posRF.out},
//     {posRB.tg, posRB.ms, posRB.out},
//     {posLB.tg, posLB.ms, posLB.out}
// };

// PIDCtrlVal rateVals[4] = {
//     {rateLF.tg, rateLF.ms, rateLF.out},
//     {rateRF.tg, rateRF.ms, rateRF.out},
//     {rateRB.tg, rateRB.ms, rateRB.out},
//     {rateLB.tg, rateLB.ms, rateLB.out}
// };

// MeasureVal msPos;   // 实际位置 测量值
// MeasureVal msRate;  // 实际速度 测量值