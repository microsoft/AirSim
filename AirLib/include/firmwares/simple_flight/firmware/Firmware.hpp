#pragma once

#include <vector>
#include "interfaces/CommonStructs.hpp"
#include "interfaces/IBoard.hpp"
#include "interfaces/ICommLink.hpp"
#include "interfaces/IStateEstimator.hpp"
#include "interfaces/IFirmware.hpp"
#include "Params.hpp"
#include "RemoteControl.hpp"
#include "CombinedGoalInput.hpp"
#include "Mixer.hpp"
#include "CascadeController.hpp"


namespace simple_flight {

class Firmware : public IFirmware {
public:
    Firmware(const Params* params, IBoard* board, ICommLink* comm_link, IStateEstimator* state_estimator)
        : params_(params), board_(board), comm_link_(comm_link), state_estimator_(state_estimator),
          combined_goal_input_(params, board, board, comm_link), mixer_(params), controller_(params, board, comm_link)
    {
        controller_.initialize(&combined_goal_input_, state_estimator_);
    }

    virtual void reset() override
    {
        IFirmware::reset();

        board_->reset();
        comm_link_->reset();
        controller_.reset();
        combined_goal_input_.reset();

        motor_outputs_.assign(params_->motor.motor_count, 0);
    }

    virtual void update() override
    {
        IFirmware::update();

        board_->update();
        combined_goal_input_.update();
        controller_.update();

        const Axis4r& output_controls = controller_.getOutput();

        //convert controller output in to motor outputs
        mixer_.getMotorOutput(output_controls, motor_outputs_);

        //finally write the motor outputs
        for (uint16_t motor_index = 0; motor_index < params_->motor.motor_count; ++motor_index)
            board_->writeOutput(motor_index, motor_outputs_.at(motor_index));

        comm_link_->update();
    }

    virtual IGoalInput& getGoalInput() override
    {
        return combined_goal_input_;
    }

    virtual const IStateEstimator& getStateEstimator() override
    {
        return *state_estimator_;
    }

private:
    //objects we use
    const Params* params_;
    IBoard* board_;
    ICommLink* comm_link_;
    IStateEstimator* state_estimator_;

    CombinedGoalInput combined_goal_input_;
    Mixer mixer_;
    CascadeController controller_;

    std::vector<float> motor_outputs_;
};


} //namespace

