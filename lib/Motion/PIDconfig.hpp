#include <Arduino.h>
#include <QuickPID.h>

enum MotorIndex : uint8_t {LF, RF, RB, LB};

/* PID参数结构体 */
typedef struct {
    float P;
    float I;
    float D;
} PIDConfig;

/* 电机PID参数 */
PIDConfig POS = {0, 0, 0};   // 位置 PID 参数
PIDConfig RATE = {0, 0, 0};  // 速度 PID 参数


/* 预定义电机PID 目标值、测量值、输出值 结构体 */
typedef struct {  
    float tg;  // target
    float ms;  // measure
    float out;   // output
} PIDCtrlVal; 

PIDCtrlVal posLF, posRF, posRB, posLB;
PIDCtrlVal rateLF, rateRF, rateRB, rateLB;

PIDCtrlVal posVals[4] = {posLF, posRF, posRB, posLB};
PIDCtrlVal rateVals[4] = {rateLF, rateRF, rateRB, rateLB};


typedef struct {  // 编码器测量值结构体
    int64_t LF = 0;
    int64_t LB = 0;
    int64_t RF = 0;
    int64_t RB = 0;
} MeasureVal;    

MeasureVal msPos;   // 实际位置 测量值
MeasureVal msRate;  // 实际速度 测量值


class PIDControllers       // 位置环 PID 控制类集合
{
public:
    PIDControllers(PIDCtrlVal PIDCtrlVals[4]):
        lf(&vals[LF].ms, &vals[LF].tg, &vals[LF].out ),
        rf(&vals[RF].ms, &vals[RF].tg, &vals[RF].out ),
        rb(&vals[RB].ms, &vals[RB].tg, &vals[RB].out ),
        lb(&vals[LB].ms, &vals[LB].tg, &vals[LB].out )
    {
        for (int i = 0; i < 4; ++i) {
            vals[i] = PIDCtrlVals[i];
        }
        SetMode(1);
    }

    void UpdateMeasure(MeasureVal MeasureVals) {
        vals[LF].ms = MeasureVals.LF;
        vals[RF].ms = MeasureVals.RF;
        vals[RB].ms = MeasureVals.RB;
        vals[LB].ms = MeasureVals.LB;
    }
        
    void SetTunings(PIDConfig pid) {
        lf.SetTunings(pid.P, pid.I, pid.D);
        rf.SetTunings(pid.P, pid.I, pid.D);
        rb.SetTunings(pid.P, pid.I, pid.D);
        lb.SetTunings(pid.P, pid.I, pid.D);
    }

    void SetOutputLimits(float Min, float Max) {
        lf.SetOutputLimits(Min, Max);
        rf.SetOutputLimits(Min, Max);
        rb.SetOutputLimits(Min, Max);
        lb.SetOutputLimits(Min, Max);
    }

    void SetMode(uint8_t Mode = 1) {   
        // enum class Control : uint8_t {manual, automatic, timer, toggle};
        lf.SetMode(Mode); rf.SetMode(Mode); rb.SetMode(Mode); lb.SetMode(Mode);
    }

    void Compute() {
        lf.Compute();  rf.Compute();  rb.Compute();  lb.Compute();
    }

    void Reset() {
        lf.Reset();  rf.Reset();  rb.Reset();  lb.Reset();
    }

protected:
    QuickPID lf;
    QuickPID rf;
    QuickPID rb;
    QuickPID lb;

    PIDCtrlVal vals[4];
};

PIDControllers pidPos;
PIDControllers pidRate;

