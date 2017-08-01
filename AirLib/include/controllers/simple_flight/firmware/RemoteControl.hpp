#pragma once

#include <vector>
#include <cstdint>
#include "interfaces/IBoardClock.hpp"
#include "interfaces/IBoardInputPins.hpp"
#include "interfaces/IGoalInput.hpp"
#include "interfaces/CommonStructs.hpp"

namespace simple_flight {

class RemoteControl : public IGoalInput {
public:
    RemoteControl(const Params* params, const IBoardClock* clock, const IBoardInputPins* board_inputs)
        : clock_(clock), board_inputs_(board_inputs), params_(params)
    {
    }

    virtual void reset() override
    {
        IGoalInput::reset();

        goal_ = Axis4r();
        goal_mode_ = params_->default_goal_mode;
        channel_vals_.assign(params_->rc.channel_count, 0);
        last_rec_read_ = 0;
    }
    
    virtual void update() override
    {
        IGoalInput::update();

        uint64_t time = clock_->millis();

        //don't keep reading if not updated
        if (time - last_rec_read_ <= params_->rc.read_interval_ms)
            return;

        last_rec_read_ = time;

        //read rc
        for (uint16_t channel = 0; channel < params_->rc.channel_count; ++channel)
            channel_vals_[channel] = board_inputs_->readChannel(channel);

        goal_.throttle = channel_vals_[params_->rc.channels.throttle];
        if (channel_vals_[params_->rc.rate_level_mode_channel] < 0.1f) { //approximately 0
            goal_mode_ = GoalMode::getStandardAngleMode();

            //we are in control-by-angles mode
            goal_.axis3.pitch() = params_->max_angle_level.pitch() * channel_vals_[params_->rc.channels.axis3.pitch()];
            goal_.axis3.roll() = params_->max_angle_level.roll() * channel_vals_[params_->rc_roll_channel];
            goal_.axis3.yaw() = params_->max_angle_level.yaw() * channel_vals_[params_->rc_yaw_channel];
        }
        else { //we are in control-by-rate mode
            goal_mode_ = GoalMode::getAllRateMode();

            goal_.axis3.pitch() = params_->max_angle_rate.pitch() * channel_vals_[params_->rc_pitch_channel];
            goal_.axis3.roll() = params_->max_angle_rate.roll() * channel_vals_[params_->rc_roll_channel];
            goal_.axis3.yaw() = params_->max_angle_rate.yaw() * channel_vals_[params_->rc_yaw_channel];
        }

    }

    virtual const Axis4r& getGoal() const override
    {
        return goal_;
    }

    virtual const GoalMode& getGoalMode() const override
    {
        return goal_mode_;
    }

private:
    enum class RcRequestType {
        None, ArmRequest, DisarmRequest
    };

    RcRequestType getRcRequest()
    {

    }

private:
    const IBoardClock* clock_;
    const IBoardInputPins* board_inputs_;
    const Params* params_;

    Axis4r goal_;
    GoalMode goal_mode_;

    std::vector<float> channel_vals_;
    uint64_t last_rec_read_;
};


} //namespace