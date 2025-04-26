
#include "PIDconfig.hpp"

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