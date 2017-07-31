#pragma once

#include <vector>
#include "interfaces/CommonStructs.hpp"
#include "interfaces/IBoard.hpp"
#include "interfaces/ICommLink.hpp"
#include "interfaces/IStateEstimator.hpp"
#include "Params.hpp"
#include "RemoteControl.hpp"
#include "Mixer.hpp"
#include "CascadeController.hpp"


namespace simple_flight {

class Firmware {
public:
    Firmware(IBoard* board, ICommLink* comm_link, IStateEstimator* state_estimator, const Params* params)
        : board_(board), comm_link_(comm_link), params_(params), 
          rc_(params, board, board), mixer_(params), controller_(params, board)
    {
        controller_.initialize(&rc_, state_estimator);
    }

    void reset()
    {
        board_->reset();
        comm_link_->reset();
        rc_.reset();
        controller_.reset();

        motor_outputs_.assign(params_->motor_count, 0);
    }

    void update()
    {
        board_->update();
        comm_link_->update();

        //get latest from RC
        rc_.update();
        controller_.update();

        const Axis4r& output_controls = controller_.getOutput();

        //convert rate_stabilizer output in to motor outputs
        mixer_.getMotorOutput(output_controls, motor_outputs_);

        //finally write the motor outputs
        for (int motor_index = 0; motor_index < params_->motor_count; ++motor_index)
            board_->writeOutput(motor_index, motor_outputs_.at(motor_index));
    }

private:
    //objects we use
    IBoard* board_;
    ICommLink* comm_link_;

    std::vector<float> motor_outputs_;

    const Params* params_;
    RemoteControl rc_;
    Mixer mixer_;
    CascadeController controller_;
};


} //namespace

