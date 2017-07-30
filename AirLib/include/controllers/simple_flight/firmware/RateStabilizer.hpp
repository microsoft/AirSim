
#pragma once

#include "interfaces/CommonStructs.hpp"
#include "Params.hpp"
#include "PidController.hpp"
//#include "common/common_utils/Utils.hpp"

namespace simple_flight {

class RateStabilizer {
public:
    RateStabilizer(const Params* params, const IBoardClock* clock = nullptr)
        : params_(params), 
          pid_rate_pitch_(clock, 
              PidController<float>::Config(params->p_pitch_rate, 0, 0)), 
          pid_rate_roll_(clock, 
              PidController<float>::Config(params->p_roll_rate, 0, 0)), 
          pid_rate_yaw_(clock, 
              PidController<float>::Config(params->p_yaw_rate, 0, 0))
    {
        unused(params_);
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

        output_.pitch = pid_rate_pitch_.getOutput();
        output_.roll = pid_rate_roll_.getOutput();
        output_.yaw = pid_rate_yaw_.getOutput();

        //common_utils::Utils::log(
        //common_utils::Utils::stringf("RAT (%f, %f, %f) - (%f, %f, %f) - (%f, %f, %f)", 
        //    pid_rate_pitch_.getGoal(), pid_rate_roll_.getGoal(), pid_rate_yaw_.getGoal(),
        //    pid_rate_pitch_.getMeasured(), pid_rate_roll_.getMeasured(), pid_rate_yaw_.getMeasured(),
        //    pid_rate_pitch_.getOutput(), pid_rate_roll_.getOutput(), pid_rate_yaw_.getOutput()
        //    ));
    }

    void setGoalRate(const Angles& controls)
    {
        pid_rate_pitch_.setGoal(controls.pitch);
        pid_rate_roll_.setGoal(controls.roll);
        pid_rate_yaw_.setGoal(controls.yaw);
    }

    void setMeasuredRate(const Angles& controls)
    {
        pid_rate_pitch_.setMeasured(controls.pitch);
        pid_rate_roll_.setMeasured(controls.roll);
        pid_rate_yaw_.setMeasured(controls.yaw);
    }

    const Angles& getOutput() const
    {
        return output_;
    }

private:
    Angles output_;

    const Params* params_;
    PidController<float> pid_rate_pitch_, pid_rate_roll_, pid_rate_yaw_;
};


} //namespace