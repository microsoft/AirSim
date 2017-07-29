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
        controls_ = Controls();
        control_mode_ = params_->default_control_mode;
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

        controls_.throttle = rc_channels_[params_->rc_thrust_channel];
        if (rc_channels_[params_->rc_rate_angle_channel] < 0.1f) { //approximately 0
            control_mode_ = ControlMode::getStandardAngleMode();

            //we are in control-by-angles mode
            controls_.angles.pitch = params_->max_pitch_angle * rc_channels_[params_->rc_pitch_channel];
            controls_.angles.roll = params_->max_roll_angle * rc_channels_[params_->rc_roll_channel];
            controls_.angles.yaw = params_->max_yaw_angle * rc_channels_[params_->rc_yaw_channel];
        }
        else { //we are in control-by-rate mode
            control_mode_ = ControlMode::getAllRateMode();

            controls_.angles.pitch = params_->max_pitch_rate * rc_channels_[params_->rc_pitch_channel];
            controls_.angles.roll = params_->max_roll_rate * rc_channels_[params_->rc_roll_channel];
            controls_.angles.yaw = params_->max_yaw_rate * rc_channels_[params_->rc_yaw_channel];
        }

    }

    ControlMode getControlMode() const
    {
        return control_mode_;
    }

    const Controls& getControls() const 
    {
        return controls_;
    }


private:
    const IBoardClock* clock_;
    const IBoardInputPins* board_inputs_;
    const Params* params_;

    ControlMode control_mode_;
    Controls controls_;

    std::vector<float> rc_channels_;
    uint64_t last_rec_read_;
};


} //namespace