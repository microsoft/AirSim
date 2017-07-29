#pragma once

#include "RateStabilizer.hpp"
#include "AngleStabilizer.hpp"

namespace simple_flight {

class Stabilizer {
public:
    Stabilizer(const Params* params, const IBoardSensors* sensors, const IBoardClock* clock, const IAngleEstimator* angle_estimator)
        : params_(params), clock_(clock), sensors_(sensors),
          angle_estimator_(angle_estimator),
          rate_stabilizer_(params, clock), angle_stabilizer_(params, clock)
    {
        unused(clock_);
    }

    void reset()
    {
        control_mode_ = params_->default_control_mode;
        output_controls_ = Controls();
        goal_input_ = goal_rates_ = measured_rates_ = Angles();
        goal_throttle_ = 0;
        rate_stabilizer_.reset();
        angle_stabilizer_.reset();
    }

    void update()
    {
        angle_stabilizer_.setGoalAngles(goal_input_);
        angle_stabilizer_.setMeasuredAngles(angle_estimator_->getAngles());

        angle_stabilizer_.update();

        //Output is from -1 to 1 so we need to renormalize to max range
        goal_from_angles_ = angle_stabilizer_.getOutput();
        goal_from_angles_.pitch *= params_->max_pitch_rate;
        goal_from_angles_.roll *= params_->max_roll_rate;
        goal_from_angles_.yaw *= params_->max_yaw_rate;

        //figure out how each axis is controller. By angle or rate?
        goal_rates_.pitch = control_mode_.PitchByRate ? goal_input_.pitch : goal_from_angles_.pitch;
        goal_rates_.roll = control_mode_.RollByRate ? goal_input_.roll : goal_from_angles_.roll;
        goal_rates_.yaw = control_mode_.YawByRate ? goal_input_.yaw : goal_from_angles_.yaw;
        
        rate_stabilizer_.setGoalRate(goal_rates_);

        //update measured controls
        sensors_->readGyro(gyro_readout);
        measured_rates_.pitch = gyro_readout[1];
        measured_rates_.roll = gyro_readout[0];
        measured_rates_.yaw = gyro_readout[2];
        rate_stabilizer_.setMeasuredRate(measured_rates_);

        //rate_stabilizer computes the output from error in goal vs measured
        rate_stabilizer_.update();
        output_controls_.angles = rate_stabilizer_.getOutput();
        output_controls_.throttle = goal_throttle_;
    }

    void setControlMode(const ControlMode& control_mode)
    {
        control_mode_ = control_mode;
    }
    ControlMode getControlMode()
    {
        return control_mode_;
    }

    void setGoal(const Controls& controls)
    {
        goal_throttle_ = controls.throttle;
        goal_input_ = controls.angles;
    }

    const Controls& getOutput() const
    {
        return output_controls_;
    }

private:
    ControlMode control_mode_;

    Controls output_controls_;
    
    const Params* params_;
    const IBoardClock* clock_;
    const IBoardSensors* sensors_;
    const IAngleEstimator* angle_estimator_;

    float gyro_readout[3];
    Angles goal_input_, goal_from_angles_, goal_rates_;
    Angles measured_angles_, measured_rates_;
    float goal_throttle_;

    RateStabilizer rate_stabilizer_;
    AngleStabilizer angle_stabilizer_;
};

}