#pragma once

#include <vector>
#include "Board.hpp"
#include "CommLink.hpp"
#include "Params.hpp"
#include "RemoteControl.hpp"
#include "Stabilizer.hpp"
#include "CommonStructs.hpp"
#include "Mixer.hpp"

namespace simple_flight {

class Firmware {
public:
    Firmware(Board* board, CommLink* comm_link, const Params* params)
        : board_(board), comm_link_(comm_link), params_(params), 
          rc_(board, params), mixer_(params), stabilizer_(params)
    {
    }

    void reset()
    {
        board_->reset();
        comm_link_->reset();
        rc_.reset();
        stabilizer_.reset();

        motor_outputs_.assign(params_->motor_count, 0);
    }

    void update()
    {
        board_->update();
        comm_link_->update();

        //get latest from RC
        rc_.update();

        //use RC values as set point for stabilizer
        const auto& goal_rate_controls = rc_.getGoalRateControls();
        stabilizer_.setGoalRateControls(goal_rate_controls);

        //update measured controls
        board_->readGyro(gyro_readout);
        measured_rate_controls_.pitch = gyro_readout[1];
        measured_rate_controls_.roll = gyro_readout[0];
        measured_rate_controls_.yaw = gyro_readout[2];
        stabilizer_.setMeasuredRateControls(measured_rate_controls_);

        //stabilizer computes the output from error in goal vs measured
        stabilizer_.update();
        const auto& output_controls = stabilizer_.getOutputControls();

        //convert stabilizer output in to motor outputs
        mixer_.getMotorOutput(output_controls, motor_outputs_);

        //finally write the motor outputs
        for (int motor_index = 0; motor_index < params_->motor_count; ++motor_index)
            board_->writeOutput(motor_index, motor_outputs_.at(motor_index));
    }

private:
    //objects we use
    Board* board_;
    CommLink* comm_link_;

    float gyro_readout[3];
    Controls measured_rate_controls_;
    std::vector<float> motor_outputs_;

    const Params* params_;
    RemoteControl rc_;
    Stabilizer stabilizer_;
    Mixer mixer_;
};


} //namespace

