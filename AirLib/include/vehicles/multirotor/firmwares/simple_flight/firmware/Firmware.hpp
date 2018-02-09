#pragma once

#include <vector>
#include "interfaces/CommonStructs.hpp"
#include "interfaces/IBoard.hpp"
#include "interfaces/ICommLink.hpp"
#include "interfaces/IStateEstimator.hpp"
#include "interfaces/IFirmware.hpp"
#include "Params.hpp"
#include "RemoteControl.hpp"
#include "OffboardApi.hpp"
#include "Mixer.hpp"
#include "CascadeController.hpp"
#include "physics/Kinematics.hpp"
#include "AdaptiveController.h"



namespace simple_flight {

class Firmware : public IFirmware {
public:
    Firmware(const Params* params, IBoard* board, ICommLink* comm_link, IStateEstimator* state_estimator)
        : params_(params), board_(board), comm_link_(comm_link), state_estimator_(state_estimator),
          offboard_api_(params, board, board, state_estimator, comm_link), mixer_(params), 
          controller_(params, board, comm_link)
    {
        controller_.initialize(&offboard_api_, state_estimator_);
    }
		
    virtual void reset() override
    {
        IFirmware::reset();

        board_->reset();
        comm_link_->reset();
        controller_.reset();
        offboard_api_.reset();

        motor_outputs_.assign(params_->motor.motor_count, 0);
    }

	virtual void update() override
	{
		IFirmware::update();

		board_->update();
		offboard_api_.update();
		controller_.update();
		
		Axis4r output_controls = controller_.getOutput();

		bool adaptive = true; //toggle adaptive controller

		// Create vectors to store state variables
		Axis3r angle = 0;
		Axis3r position = 0;
		Axis3r angle_rate = 0; 
		Axis3r velocity = 0;
				
		if (adaptive)
		{
			// Assign state variables to placeholder variables
			angle = state_estimator_->getAngles();
			angle_rate = state_estimator_->getAngulerVelocity();
			position = state_estimator_->getPosition();
			velocity = state_estimator_->getLinearVelocity();
				
			// Send state variables to the adaptive controller
			adaptive_.update_state_vals(position[0], velocity[0], position[1], velocity[1], position[2], velocity[2], 
										angle[0], angle_rate[0], angle[1], angle_rate[1], angle[2], angle_rate[2]);

			adaptive_.run();
			
			// Replace PID control variables with Adaptive control variables
			output_controls[0] = adaptive_.get_U1();
			output_controls[1] = adaptive_.get_U2();
			output_controls[2] = adaptive_.get_U3();
			output_controls[3] = adaptive_.get_U4();
		}
	
        //convert controller output in to motor outputs
        mixer_.getMotorOutput(output_controls, motor_outputs_,t);
        //finally write the motor outputs
        for (uint16_t motor_index = 0; motor_index < params_->motor.motor_count; ++motor_index)
            board_->writeOutput(motor_index, motor_outputs_.at(motor_index));

        comm_link_->update();
		// Iterate t counter for use in adaptive controller and propeller breaking
		t += 0.003;
    }

    virtual IOffboardApi& offboardApi() override
    {
        return offboard_api_;
    }
	
	

private:
    //objects we use
    const Params* params_;
    IBoard* board_;
    ICommLink* comm_link_;
    IStateEstimator* state_estimator_;
	const Kinematics::State* kinematics_;

    OffboardApi offboard_api_;
    Mixer mixer_;
    CascadeController controller_;
	AdaptiveController adaptive_;

    std::vector<float> motor_outputs_;
};


} //namespace

