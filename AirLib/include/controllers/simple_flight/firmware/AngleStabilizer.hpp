
#pragma once

#include "interfaces/IUpdatable.hpp"
#include "interfaces/IBoardClock.hpp"
#include "interfaces/CommonStructs.hpp"
#include "Params.hpp"
#include "PidController.hpp"
//#include "common/common_utils/Utils.hpp"

namespace simple_flight {

class AngleStabilizer : public IUpdatable {
public:
    AngleStabilizer(const Params* params, const IBoardClock* clock = nullptr)
        : params_(params), 
        pid_angle_pitch_(clock, 
            PidController<float>::Config(params->p_pitch_angle, 0, 0)), 
        pid_angle_roll_(clock, 
            PidController<float>::Config(params->p_roll_angle, 0, 0)), 
        pid_angle_yaw_(clock, 
            PidController<float>::Config(params->p_yaw_angle, 0, 0))
    {
        unused(params_);
    }

    virtual void reset() override
    {
        pid_angle_pitch_.reset();
        pid_angle_roll_.reset();
        pid_angle_yaw_.reset();
    }

    virtual void update() override
    {
        pid_angle_pitch_.update();
        pid_angle_roll_.update();
        pid_angle_yaw_.update();

        output_.pitch = pid_angle_pitch_.getOutput();
        output_.roll = pid_angle_roll_.getOutput();
        output_.yaw = pid_angle_yaw_.getOutput();

        //common_utils::Utils::log(
        //common_utils::Utils::stringf("ANG (%f, %f, %f) - (%f, %f, %f) - (%f, %f, %f)", 
        //    pid_angle_pitch_.getGoal(), pid_angle_roll_.getGoal(), pid_angle_yaw_.getGoal(),
        //    pid_angle_pitch_.getMeasured(), pid_angle_roll_.getMeasured(), pid_angle_yaw_.getMeasured(),
        //    pid_angle_pitch_.getOutput(), pid_angle_roll_.getOutput(), pid_angle_yaw_.getOutput()
        //    ));
    }

    void setGoalAngles(const Angles& angles)
    {
        pid_angle_pitch_.setGoal(angles.pitch);
        pid_angle_roll_.setGoal(angles.roll);
        pid_angle_yaw_.setGoal(angles.yaw);
    }

    void setMeasuredAngles(const Angles& angles)
    {
        pid_angle_pitch_.setMeasured(angles.pitch);
        pid_angle_roll_.setMeasured(angles.roll);
        pid_angle_yaw_.setMeasured(angles.yaw);
    }

    const Angles& getOutput() const
    {
        return output_;
    }

private:
    Angles output_;
    const Params* params_;
    PidController<float> pid_angle_pitch_, pid_angle_roll_, pid_angle_yaw_;
};


} //namespace