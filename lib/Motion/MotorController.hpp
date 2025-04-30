#ifndef __MOTOR_CONTROLLER_H__
#define __MOTOR_CONTROLLER_H__

#include <Arduino.h>
#include <QuickPID.h>

#include "Motor.hpp"
#include "HXCEncoder.hpp"
#include "config.hpp"

class MotorController
{
public:
    MotorController(PIDConfig _POS, PIDConfig _RATE,
        EncoderConfig _encoderCfg, MotorConfig _motorCfg, 
        uint32_t SampleTimeUs):

        // PID配置和中间变量
        POS(_POS), RATE(_RATE),
        encoderCfg(_encoderCfg), motorCfg(_motorCfg),

        // 电机用对象
        encoder(encoderCfg.PINA, encoderCfg.PINB),
        motor(motorCfg.FW_PIN, motorCfg.BW_PIN,
                motorCfg.FW_CHANNEL, motorCfg.BW_CHANNEL,
                motorCfg.freq, motorCfg.THR_MIN, motorCfg.THR_MAX),

        pidRate(&rateVal.ms, &rateVal.out, &rateVal.tg,
                RATE.P, RATE.I, RATE.D,
                QuickPID::Action::direct),

        pidPos(&posVal.ms, &posVal.out, &posVal.tg,
                POS.P, POS.I, POS.D,
                QuickPID::Action::direct)

    {
        encoder.setup(encoderCfg.frc);
        encoder.set_filter(encoderCfg.filter_ns);

        motor.setSpeedLimit(0, motorCfg.THR_MAX);
        motor.setSpeed(0);

        pidRate.SetMode(1);
        pidRate.SetSampleTimeUs(SampleTimeUs);
        pidRate.SetOutputLimits(-motorCfg.THR_MAX, motorCfg.THR_MAX);

        pidPos.SetMode(1);
        pidPos.SetSampleTimeUs(SampleTimeUs);
        pidPos.SetOutputLimits(-motorCfg.THR_MAX*10, motorCfg.THR_MAX*10);

        rateVal.tg = 0;
        posVal.tg = 0;
    }

    PIDConfig POS, RATE;
    PIDCtrlVal posVal, rateVal;
    EncoderConfig encoderCfg;
    MotorConfig motorCfg;
    
    HXC::Encoder encoder;
    Motor motor;
    QuickPID pidRate, pidPos;

    void PIDCompute() {
        posVal.ms  = encoder.get_count();
        rateVal.ms = encoder.get_speed();

        if (POS.P > 0 && RATE.P > 0) {  // 位置环和速度环一起控制
            pidPos.Compute();   
            rateVal.tg = posVal.out;    // 让位置环的输出作为速度环的输入
            pidRate.Compute();   
            motor.setSpeed(rateVal.out);
            Serial.printf("位置环和速度环一起控制 %f, %f \n", posVal.out, rateVal.out);
        }
        else if (RATE.P > 0 && POS.P == 0) {  // 仅速度环控制
            pidRate.Compute();
            motor.setSpeed(rateVal.out);
            Serial.printf("仅速度环控制\n");
        }
        else if (POS.P > 0 && RATE.P == 0) {  // 仅位置环控制
            pidPos.Compute();
            motor.setSpeed(posVal.out);
            Serial.printf("仅位置环控制\n");
        }
        else {   // 关闭 PID 控制
            motor.setSpeed(0);
            pidRate.Reset();
            pidPos.Reset();
            // posVal.ms = 0;
            encoder.reset_count();
        }

        // Serial.printf("%f, %f, %f , %f, %f , %f \n", 
        //     rateVal.ms, rateVal.out, rateVal.tg, posVal.ms, posVal.out, posVal.tg);
    } 

    void setPIDcfg(PIDConfig _POS, PIDConfig _RATE) {

        POS = _POS;
        RATE = _RATE;

        pidPos.SetTunings(POS.P, POS.I, POS.D);
        pidRate.SetTunings(RATE.P, RATE.I, RATE.D);

        Serial.printf("PID参数更新 pos: %f, %f, %f ; rate: %f, %f, %f \n", POS.P, POS.I, POS.D, RATE.P, RATE.I, RATE.D);
    }    
    
    void setSpeedLimit(uint16_t THR_MIN, uint16_t THR_MAX) {
        motor.setSpeedLimit(THR_MIN, THR_MAX);
    }

    void setPostion(float _tg) {
        posVal.tg = _tg;
    }

    void setRate(float _tg) {
        rateVal.tg = _tg;
    }

    void getPostion(int64_t &Pos) {
        Pos = encoder.get_count();
    }

    void getRate(float &Rate) {
        Rate = encoder.get_speed();
    }

    void reset() {
        pidRate.Reset();
        pidPos.Reset();
        posVal.ms = 0;
        rateVal.ms = 0;
        encoder.reset_count();
        motor.setSpeed(0);
        Serial.println("PID reset");
    }

    void setMotorSpeedDirect(int16_t rate) {
        Serial.printf("直接设置电机速度 %d \n", rate);
        motor.setSpeed(rate);
        motor.print();
    }
    
};

#endif
