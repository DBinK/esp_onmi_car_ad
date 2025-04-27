#ifndef MOTION_HPP
#define MOTION_HPP

#include <cmath>

#include <Arduino.h>
#include <QuickPID.h>

#include "HXCthread.hpp"

#include "HXCEncoder.hpp"
#include "Motor.hpp"
#include "PIDconfig.hpp"
#include "PIDControllers.hpp"


class Encoders
{
public:
    /**
     * @brief 编码器集合类
     * @param const int pins[] , 总共8个编码器，每个编码器由两个引脚组成
     */
    Encoders(const int pins[8])
        : encoder_lf(pins[0], pins[1]),
          encoder_rf(pins[2], pins[3]),
          encoder_lb(pins[4], pins[5]),
          encoder_rb(pins[6], pins[7]),

          counts{0, 0, 0, 0}, // 初始化 脉冲计数(位置)，默认4倍计数
          speeds{0, 0, 0, 0}  // 初始化 脉冲频率(速度)，单位Hz
    {
        setup();
    }

    void setup(int frc = 1000)
    {
        encoder_lf.setup(frc);
        encoder_rf.setup(frc);
        encoder_lb.setup(frc);
        encoder_rb.setup(frc);
    }

    void set_encoder_filter(uint16_t ns)
    {
        encoder_lf.set_filter(ns);
        encoder_rf.set_filter(ns);
        encoder_lb.set_filter(ns);
        encoder_rb.set_filter(ns);
    }

    void get_encoder_counts(MeasureVal &msPos)
    {
        msPos.LF = encoder_lf.get_count();
        msPos.RF = encoder_rf.get_count();
        msPos.RB = encoder_rb.get_count();
        msPos.LB = encoder_lb.get_count();
    }

    void get_encoder_speeds(MeasureVal &msRate)
    {
        msRate.LF = encoder_lf.get_speed();
        msRate.RF = encoder_rf.get_speed();
        msRate.RB = encoder_rb.get_speed();
        msRate.LB = encoder_lb.get_speed();
    }

protected:
    HXC::Encoder encoder_lf; // 左前编码器
    HXC::Encoder encoder_rf; // 右前编码器
    HXC::Encoder encoder_lb; // 左后编码器
    HXC::Encoder encoder_rb; // 右后编码器

    int64_t counts[4]; // 脉冲计数，默认4倍计数
    int32_t speeds[4]; // 脉冲频率，单位Hz
};

class Motors
{
public:
    Motors(const int pins[8])
        : motor_lf(pins[0], pins[1]),
          motor_rf(pins[2], pins[3]),
          motor_lb(pins[4], pins[5]),
          motor_rb(pins[6], pins[7])
    {
        set_motor_limit(0, 1023);  // 设置默认电机速度限制
    }

    void set_motor_limit(uint16_t THR_MIN, uint16_t THR_MAX)
    {
        motor_lf.setSpeedLimit(THR_MIN, THR_MAX);
        motor_rf.setSpeedLimit(THR_MIN, THR_MAX);
        motor_lb.setSpeedLimit(THR_MIN, THR_MAX);
        motor_rb.setSpeedLimit(THR_MIN, THR_MAX);
    }

    void set_motor_speed(int16_t rate_in[4])
    {
        motor_lf.setSpeed(rate_in[LF]);
        motor_rf.setSpeed(rate_in[RF]);
        motor_rb.setSpeed(rate_in[RB]);
        motor_lb.setSpeed(rate_in[LB]);
    }

    void set_motor_speed_lf(int16_t rate_in)
    {
        motor_lf.setSpeed(rate_in);
    }

    void set_motor_speed_rf(int16_t rate_in)
    {
        motor_rf.setSpeed(rate_in);
    }

    void set_motor_speed_lb(int16_t rate_in)
    {
        motor_lb.setSpeed(rate_in);
    }

    void set_motor_speed_rb(int16_t rate_in)
    {
        motor_rb.setSpeed(rate_in);
    }


protected:
    Motor motor_lf;
    Motor motor_rf;
    Motor motor_lb;
    Motor motor_rb;
};


// class Motion {
//     public:
//         Motion(const uint8_t encoder_pins[8], const uint8_t motor_pins[8],
//                PIDCtrlVal posVals[4], PIDCtrlVal rateVals[4],
//                MeasureVal msPos, MeasureVal msRate)
//             : encoders(encoder_pins),
//               motors(motor_pins),
//               pidPos(posVals),
//               pidRate(rateVals),
//               msPos(msPos),
//               msRate(msRate), 
//               pidControlThread([]() {}) // 显式初始化线程成员变量为空任务
//         {
//             encoders.setup(50);
//             encoders.set_encoder_filter(10);
//             motors.set_motor_limit(0, 1023);

//             start_pid_control(); // 启动PID控制任务
//         }
    
//         void start_pid_control() {
//             pidControlThread = HXC::thread<void>([this]() { pid_control_task(); });
//             pidControlThread.start();
//         }
    
//         void syncOutToTarget() {
//             for (int i = 0; i < 4; i++) {
//                 rateVals[i].tg = posVals[i].out;
//             }
//         }

//         void set_pos_target_value(uint8_t index, float pos_target_value) {
//             posVals[index].tg = pos_target_value;
//         }

//         void set_rate_target_value(uint8_t index, float rate_target_value) {
//             rateVals[index].tg = rate_target_value;
//         }
    
//         void pid_control_task() {
//             static TickType_t xLastWakeTime = 0;
//             set_pos_target_value(LF, 0);
    
//             while (1) {
//                 if (xLastWakeTime == 0) {
//                     xLastWakeTime = xTaskGetTickCount();
//                 }
//                 vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    
//                 encoders.get_encoder_counts(msPos);
//                 encoders.get_encoder_speeds(msRate);
//                 pidPos.UpdateMeasure(msPos);
//                 pidRate.UpdateMeasure(msRate);
    
//                 pidPos.Compute();
//                 syncOutToTarget();
//                 pidRate.Compute();
    
//                 motors.set_motor_speed_lf(posVals[LF].out);
//                 // motors.set_motor_speed_rf(rateRF.out);
//                 // motors.set_motor_speed_lb(rateLB.out);
//                 // motors.set_motor_speed_rb(rateRB.out);
//             }
//         }
    
//     protected:
//         Encoders encoders;
//         Motors motors;
//         PIDControllers pidPos, pidRate;
//         MeasureVal msPos, msRate;
//         PIDCtrlVal posVals[4], rateVals[4];
    
//         HXC::thread<void> pidControlThread; // 添加线程成员变量
//     };
    

#endif