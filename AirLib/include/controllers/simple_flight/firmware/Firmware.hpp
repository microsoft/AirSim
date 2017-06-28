#pragma once

#include <vector>
#include "CommonStructs.hpp"
#include "Board.hpp"
#include "CommLink.hpp"
#include "Params.hpp"
#include "RemoteControl.hpp"
#include "Mixer.hpp"
#include "Stabilizer.hpp"


namespace simple_flight {

class Firmware {
public:
    Firmware(Board* board, CommLink* comm_link, IAngleEstimator* angle_estimator, const Params* params)
        : board_(board), comm_link_(comm_link), params_(params), 
          rc_(params, board, board), mixer_(params), stabilizer_(params, board, board, angle_estimator)
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

        stabilizer_.setControlMode(rc_.getControlMode());
        stabilizer_.setGoal(rc_.getControls());
        stabilizer_.update();

        const Controls& output_controls = stabilizer_.getOutput();

        //convert rate_stabilizer output in to motor outputs
        mixer_.getMotorOutput(output_controls, motor_outputs_);

        //finally write the motor outputs
        for (int motor_index = 0; motor_index < params_->motor_count; ++motor_index)
            board_->writeOutput(motor_index, motor_outputs_.at(motor_index));
    }

private:
    //objects we use
    Board* board_;
    CommLink* comm_link_;

    std::vector<float> motor_outputs_;

    const Params* params_;
    RemoteControl rc_;
    Mixer mixer_;
    Stabilizer stabilizer_;
};


} //namespace

