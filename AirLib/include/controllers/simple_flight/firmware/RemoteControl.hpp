#pragma once

#include <vector>
#include <cstdint>
#include "Board.hpp"
#include "CommonStructs.hpp"

namespace simple_flight {

class RemoteControl {
public:
    RemoteControl(const Board* board, const Params* params)
        : board_(board), params_(params)
    {
    }

    void reset()
    {
        rc_channels_.assign(params_->rc_channel_count, 0);
        last_rec_read_ = 0;
        controls_ = Controls();
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

        controls_.thrust = rc_channels_[params_->rc_thrust_channel];
        controls_.pitch = rc_channels_[params_->rc_pitch_channel];
        controls_.roll = rc_channels_[params_->rc_roll_channel];
        controls_.yaw = rc_channels_[params_->rc_yaw_channel];
    }

    const Controls& getDesiredRates() const 
    {
        return controls_;
    }


private:
    const Board* board_;
    const Params* params_;

    Controls controls_;

    std::vector<float> rc_channels_;
    uint64_t last_rec_read_;
};


} //namespace