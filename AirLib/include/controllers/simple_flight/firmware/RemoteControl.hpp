#pragma once

#include <vector>
#include <cstdint>
#include "Board.hpp"

namespace simple_flight {

class RemoteControl {
public:
    RemoteControl(const Board* board, const Params* params)
        : board_(board), params_(params)
    {
        pwm_range = params_->max_pwm - params_->min_pwm;
    }

    void reset()
    {
        rc_channels_.assign(params_->rc_channel_count, 0);
        last_rec_read_ = 0;
    }


    void update()
    {
        uint64_t time = board_->millis();

        //don't keep reading if not updated
        if (time - last_rec_read_ <= params_->rc_read_interval_ms)
            return;

        last_rec_read_ = time;

        //read rc
        for (int channel = 0; channel < params_->rc_channel_count; ++channel)
            rc_channels_[channel] = board_->readChannel(channel);

        thrust = normalizePwm(rc_channels_[params_->rc_thrust_channel]);
        pitch = normalizePwm(rc_channels_[params_->rc_pitch_channel]);
        roll = normalizePwm(rc_channels_[params_->rc_roll_channel]);
        yaw = normalizePwm(rc_channels_[params_->rc_yaw_channel]);
    }


private:
    float normalizePwm(uint16_t pwm)
    {
        return (pwm - params_->min_pwm) / pwm_range;
    }
private:
    const Board* board_;
    const Params* params_;

    std::vector<uint16_t> rc_channels_;
    uint64_t last_rec_read_;
    float thrust, pitch, roll, yaw;

    float pwm_range;
};


} //namespace