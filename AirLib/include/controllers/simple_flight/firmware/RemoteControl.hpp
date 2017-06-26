#pragma once

#include <vector>
#include <cstdint>
#include "Board.hpp"
#include "CommonStructs.hpp"

namespace simple_flight {

class RemoteControl {
public:
    RemoteControl(const Params* params, const IBoardClock* clock, const IBoardInputPins* board_inputs)
        : clock_(clock), board_inputs_(board_inputs), params_(params)
    {
    }

    void reset()
    {
        rc_channels_.assign(params_->rc_channel_count, 0);
        last_rec_read_ = 0;
        rate_controls_ = Controls();
    }
    
    void update()
    {
        uint64_t time = clock_->millis();

        //don't keep reading if not updated
        if (time - last_rec_read_ <= params_->rc_read_interval_ms)
            return;

        last_rec_read_ = time;

        //read rc
        for (int channel = 0; channel < params_->rc_channel_count; ++channel)
            rc_channels_[channel] = board_inputs_->readChannel(channel);

        rate_controls_.throttle = rc_channels_[params_->rc_thrust_channel];
        rate_controls_.pitch = params_->max_pitch_rate * rc_channels_[params_->rc_pitch_channel];
        rate_controls_.roll = params_->max_roll_rate * rc_channels_[params_->rc_roll_channel];
        rate_controls_.yaw = params_->max_yaw_rate * rc_channels_[params_->rc_yaw_channel];
    }

    const Controls& getGoalRateControls() const 
    {
        return rate_controls_;
    }


private:
    const IBoardClock* clock_;
    const IBoardInputPins* board_inputs_;
    const Params* params_;

    Controls rate_controls_;

    std::vector<float> rc_channels_;
    uint64_t last_rec_read_;
};


} //namespace