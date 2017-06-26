#pragma once

#include "CommonStructs.hpp"
#include "Params.hpp"
#include "common/common_utils/Utils.hpp"
#include "PidController.hpp"
#include "CommonStructs.hpp"

namespace simple_flight {

class Stabilizer {
public:
    Stabilizer(const Params* params, const IBoardClock* clock = nullptr)
        : params_(params), 
          pid_rate_pitch_(clock, 
              PidController<float>::Config(params_->p_pitch_rate, 0, 0)), 
          pid_rate_roll_(clock, 
              PidController<float>::Config(params_->p_roll_rate, 0, 0)), 
          pid_rate_yaw_(clock, 
              PidController<float>::Config(params_->p_yaw_rate, 0, 0))
    {
    }

    void reset()
    {
        pid_rate_pitch_.reset();
        pid_rate_roll_.reset();
        pid_rate_yaw_.reset();
    }

    void update()
    {
        pid_rate_pitch_.update();
        pid_rate_roll_.update();
        pid_rate_yaw_.update();

        output_controls_.throttle = goal_throttle_;
        output_controls_.pitch = pid_rate_pitch_.getOutput();
        output_controls_.roll = pid_rate_roll_.getOutput();
        output_controls_.yaw = pid_rate_yaw_.getOutput();

        //common_utils::Utils::log(
        //common_utils::Utils::stringf("(%f, %f, %f) - (%f, %f, %f)", 
        //    goal_rate_controls_.pitch, goal_rate_controls_.roll, goal_rate_controls_.yaw,
        //    measured_rate_controls_.pitch, measured_rate_controls_.roll, measured_rate_controls_.yaw
        //    ));
    }

    void setGoalRateControls(const Controls& controls)
    {
        goal_throttle_ = controls.throttle;
        pid_rate_pitch_.setGoal(controls.pitch);
        pid_rate_roll_.setGoal(controls.roll);
        pid_rate_yaw_.setGoal(controls.yaw);
    }

    void setMeasuredRateControls(const Controls& controls)
    {
        pid_rate_pitch_.setMeasured(controls.pitch);
        pid_rate_roll_.setMeasured(controls.roll);
        pid_rate_yaw_.setMeasured(controls.yaw);
    }

    const Controls& getOutputControls() const
    {
        return output_controls_;
    }

private:
    Controls output_controls_;
    float goal_throttle_;

    PidController<float> pid_rate_pitch_, pid_rate_roll_, pid_rate_yaw_;

    const Params* params_;
};


} //namespace