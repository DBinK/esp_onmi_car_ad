#ifndef MOTION_HPP
#define MOTION_HPP

#include <cmath>
#include <stdexcept> // 异常处理
#include <mutex>     // 线程安全

#include "Motor.hpp"
#include "HXCEncoder.hpp"

// 定义枚举以提高可读性
enum EncoderIndex
{
    LF = 0,
    RF = 1,
    LB = 2,
    RB = 3
};

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
          encoder_rb(pins[6], pins[7])
    {
        // 初始化 counts 和 speeds
        for (int i = 0; i < 4; ++i)
        {
            counts[i] = 0;
            speeds[i] = 0.0f;
        }

        setup();
    }

    void setup() {
        encoder_lf.setup();
        encoder_rf.setup();
        encoder_lb.setup();
        encoder_rb.setup();
    }

    void set_encoder_filter(uint16_t ns)
    {
        std::lock_guard<std::mutex> lock(mutex); // 线程安全
        encoder_lf.set_filter(ns);
        encoder_rf.set_filter(ns);
        encoder_lb.set_filter(ns);
        encoder_rb.set_filter(ns);
    }

    void get_encoder_counts(int64_t (&counts_out)[4])
    {
        std::lock_guard<std::mutex> lock(mutex); // 线程安全
        counts_out[LF] = encoder_lf.get_count();
        counts_out[RF] = encoder_rf.get_count();
        counts_out[LB] = encoder_lb.get_count();
        counts_out[RB] = encoder_rb.get_count();
    }

    void get_encoder_speeds(int32_t (&speeds_out)[4])
    {
        std::lock_guard<std::mutex> lock(mutex); // 线程安全
        speeds_out[LF] = static_cast<int32_t>(round(encoder_lf.get_speed() / 1000));
        speeds_out[RF] = static_cast<int32_t>(round(encoder_rf.get_speed() / 1000));
        speeds_out[LB] = static_cast<int32_t>(round(encoder_lb.get_speed() / 1000));
        speeds_out[RB] = static_cast<int32_t>(round(encoder_rb.get_speed() / 1000));
    }

protected:
    HXC::Encoder encoder_lf; // 左前编码器
    HXC::Encoder encoder_rf; // 右前编码器
    HXC::Encoder encoder_lb; // 左后编码器
    HXC::Encoder encoder_rb; // 右后编码器

    int64_t counts[4]; // 脉冲计数，默认4倍计数
    int32_t speeds[4]; // 脉冲频率，单位Hz

    std::mutex mutex; // 用于线程安全
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
        // 初始化 rates
        for (int i = 0; i < 4; ++i)
        {
            rates[i] = 0;
        }
    }

    void set_motor_speed(int16_t rates_in[4])
    {
        std::lock_guard<std::mutex> lock(mutex); // 线程安全
        motor_lf.setSpeed(rates_in[LF]);
        motor_rf.setSpeed(rates_in[RF]);
        motor_lb.setSpeed(rates_in[LB]);
        motor_rb.setSpeed(rates_in[RB]);
    }

    void set_motor_limit(uint16_t THR_MIN, uint16_t THR_MAX)
    {
        std::lock_guard<std::mutex> lock(mutex); // 线程安全
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

    int16_t rates[4];

    std::mutex mutex; // 用于线程安全
};

#endif