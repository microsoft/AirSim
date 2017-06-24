#pragma once

#include "CommonStructs.hpp"
#include "Params.hpp"
#include "common/common_utils/Utils.hpp"

namespace simple_flight {

class Stabilizer {
public:
    Stabilizer(const Params* params)
        : params_(params)
    {
    }

    void reset()
    {
        goal_rate_controls_ 
            = measured_rate_controls_ 
            = output_controls_
            = Controls();
    }

    void update()
    {
        output_controls_.throttle = goal_rate_controls_.throttle;
        output_controls_.pitch = params_->p_pitch_rate * (goal_rate_controls_.pitch - measured_rate_controls_.pitch);
        output_controls_.roll = params_->p_roll_rate * (goal_rate_controls_.roll - measured_rate_controls_.roll);
        output_controls_.yaw = params_->p_yaw_rate * (goal_rate_controls_.yaw - measured_rate_controls_.yaw);

        common_utils::Utils::log(
        common_utils::Utils::stringf("(%f, %f, %f) - (%f, %f, %f)", 
            goal_rate_controls_.pitch, goal_rate_controls_.roll, goal_rate_controls_.yaw,
            measured_rate_controls_.pitch, measured_rate_controls_.roll, measured_rate_controls_.yaw
            ));
    }

    void setGoalRateControls(const Controls& controls)
    {
        goal_rate_controls_ = controls;
    }

    void setMeasuredRateControls(const Controls& controls)
    {
        measured_rate_controls_ = controls;
    }

    const Controls& getOutputControls() const
    {
        return output_controls_;
    }

private:
    Controls output_controls_;
    Controls goal_rate_controls_, measured_rate_controls_;

    const Params* params_;
};


} //namespace