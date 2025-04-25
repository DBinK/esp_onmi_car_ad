#include <Arduino.h>

/* PID参数结构体 */
typedef struct {
    float P;
    float I;
    float D;
} PIDConfig;

/* 电机PID参数 */
PIDConfig POS = {0, 0, 0};   // 位置PID参数
PIDConfig RATE = {0, 0, 0};  // 速度PID参数



/* 预定义 目标值、测量值、输出值 结构体 */

typedef struct {  // 目标值结构体
    int32_t LF = 0;
    int32_t LB = 0;
    int32_t RF = 0;
    int32_t RB = 0;
} TargetValue;     

TargetValue tgPos;   // 目标 位置增量
TargetValue tgRate;  // 目标 速度


typedef struct {  // 测量值结构体
    int64_t LF = 0;
    int64_t LB = 0;
    int64_t RF = 0;
    int64_t RB = 0;
} MeasureValue;    

MeasureValue msPos;   // 实际位置 测量值
MeasureValue msRate;  // 实际速度 测量值


typedef struct {  // 输出值结构体
    float LF = 0;
    float LB = 0;
    float RF = 0;
    float RB = 0;
} OutputValue;       

OutputValue outPos;   // 位置 PID 输出控制量
OutputValue outRate;  // 速度 PID 输出控制量