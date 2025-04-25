
#include <cmath>

#include "Motor.hpp"
#include "HXCEncoder.hpp"


class Encoders {
    public:
        /**
         * @brief 编码器集合类
         * @param const int pins[] , 总共8个编码器，每个编码器由两个引脚组成
         */
        Encoders(const int pins[]) 
            : encoder_lf(pins[0], pins[1]), 
              encoder_rf(pins[2], pins[3]), 
              encoder_lb(pins[4], pins[5]), 
              encoder_rb(pins[6], pins[7]) {}
    
        void set_encoder_filter(uint16_t ns) {
            encoder_lf.set_filter(ns);
            encoder_rf.set_filter(ns);
            encoder_lb.set_filter(ns);
            encoder_rb.set_filter(ns);
        }

        void get_encoder_counts(int64_t (&counts)[4]) {
                counts[0] = encoder_lf.get_count();
                counts[1] = encoder_rf.get_count();
                counts[2] = encoder_lb.get_count();
                counts[3] = encoder_rb.get_count();
            }
        
        void get_encoder_speeds(float (&speeds)[4]) {
            speeds[0] = encoder_lf.get_speed();
            speeds[1] = encoder_rf.get_speed();
            speeds[2] = encoder_lb.get_speed();
            speeds[3] = encoder_rb.get_speed();
        }
    
    protected:
        HXC::Encoder encoder_lf; // 左前编码器
        HXC::Encoder encoder_rf; // 右前编码器
        HXC::Encoder encoder_lb; // 左后编码器
        HXC::Encoder encoder_rb; // 右后编码器
    
        int64_t counts[4]; // 脉冲计数，默认4倍计数
        float speeds[4];   // 脉冲频率，单位Hz
    };


class Motors {
    public:
        Motors(const int pins[]) 
            : motor_lf(pins[0], pins[1]), 
              motor_rf(pins[2], pins[3]), 
              motor_lb(pins[4], pins[5]), 
              motor_rb(pins[6], pins[7]) {}
    
        void set_motor_speed(int8_t rates[4]) {
            motor_lf.setSpeed(rates[0]);
            motor_rf.setSpeed(rates[1]);
            motor_lb.setSpeed(rates[2]);
            motor_rb.setSpeed(rates[3]);
        }

        void set_motor_limit(uint16_t THR_MIN, uint16_t THR_MAX) {
            motor_lf.setSpeedLimit(THR_MIN, THR_MAX);
            motor_rf.setSpeedLimit(THR_MIN, THR_MAX);
            motor_lb.setSpeedLimit(THR_MIN, THR_MAX);
            motor_rb.setSpeedLimit(THR_MIN, THR_MAX);
        }
    
    protected:
        Motor motor_lf;
        Motor motor_rf;
        Motor motor_lb;
        Motor motor_rb;

        int8_t rates[4];

};
