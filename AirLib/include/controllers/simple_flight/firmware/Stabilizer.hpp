#pragma once

#include "RateStabilizer.hpp"
#include "AngleStabilizer.hpp"

namespace simple_flight {

class Stabilizer {
public:
    Stabilizer(const Params* params, const IBoardSensors* sensors, const IBoardClock* clock, const IAngleEstimator* angle_estimator)
        : params_(params), clock_(clock), sensors_(sensors),
          rate_stabilizer_(params, clock), angle_stabilizer_(params, clock),
          angle_estimator_(angle_estimator)
    {
    }

    void reset()
    {
        control_mode_ = params_->default_control_mode;
        output_controls_ = Controls();
        goal_rates_ = measured_rates_ = Angles();
        goal_angles_ = measured_angles_ = Angles();
        goal_throttle_ = 0;
        rate_stabilizer_.reset();
        angle_stabilizer_.reset();
    }

    void update()
    {
        if (control_mode_ == ControlMode::Angle) {
            angle_stabilizer_.setGoalAngles(goal_angles_);
            angle_stabilizer_.setMeasuredAngles(angle_estimator_->getAngles());

            angle_stabilizer_.update();

            goal_rates_ = angle_stabilizer_.getOutput();
            goal_rates_.pitch *= params_->max_pitch_rate;
            goal_rates_.roll *= params_->max_roll_rate;
            goal_rates_.yaw *= params_->max_yaw_rate;
        }

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

    void setControlMode(ControlMode control_mode)
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

        if (control_mode_ == ControlMode::Angle)
            goal_angles_ = controls.angles;
        else
            goal_rates_ = controls.angles;
    }

    const Controls& getOutput() const
    {
        return output_controls_;
    }

private:
    ControlMode control_mode_;

    Controls output_controls_;
    
    const IBoardSensors* sensors_;
    const IBoardClock* clock_;
    const Params* params_;
    const IAngleEstimator* angle_estimator_;

    float gyro_readout[3];
    Angles goal_rates_, measured_rates_;
    Angles goal_angles_, measured_angles_;
    float goal_throttle_;

    RateStabilizer rate_stabilizer_;
    AngleStabilizer angle_stabilizer_;
};

}