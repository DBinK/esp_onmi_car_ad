#include <Arduino.h>
#include <QuickPID.h>

#include "Motor.hpp"
#include "HXCEncoder.hpp"
#include "config.hpp"

class MotorController
{
public:
    MotorController(PIDConfig _POS, PIDConfig _RATE,
        PIDCtrlVal _posVal, PIDCtrlVal _rateVal,
        EncoderConfig _encoderCfg, MotorConfig _motorCfg, 
        uint32_t SampleTimeUs):

        // 全局变量
        POS(_POS), RATE(_RATE),
        posVal(_posVal), rateVal(_rateVal),
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

        motor.setSpeedLimit(motorCfg.THR_MIN, motorCfg.THR_MAX);
        motor.setSpeed(0);

        pidRate.SetMode(1);
        pidRate.SetSampleTimeUs(SampleTimeUs);
        pidRate.SetOutputLimits(-motorCfg.THR_MAX, motorCfg.THR_MAX);

        pidPos.SetMode(1);
        pidPos.SetSampleTimeUs(SampleTimeUs);
        pidPos.SetOutputLimits(-motorCfg.THR_MAX, motorCfg.THR_MAX);

        rateVal.tg = 0;
        posVal.tg = 0;
    }

    PIDConfig POS, RATE;
    PIDCtrlVal posVal, rateVal;
    EncoderConfig encoderCfg;
    MotorConfig motorCfg;

    
protected:
    HXC::Encoder encoder;
    Motor motor;

    QuickPID pidRate, pidPos;
};