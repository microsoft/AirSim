// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_air_copter_sim_MotorDirectController_hpp
#define msr_air_copter_sim_MotorDirectController_hpp

#include <exception>
#include "controllers/ControllerBase.hpp"
#include "common/common_utils/Utils.hpp"
#include "MotorDirectControllerParams.hpp"

namespace msr { namespace airlib {

class MotorDirectController : public ControllerBase {
private:
    vector<real_T> motor_control_signals_;
    MotorDirectControllerParams params_;

public:
    MotorDirectController()
    {
        MotorDirectController::reset();
    }
    MotorDirectController(const MotorDirectControllerParams& params)
    {
        initialize(params);
    }
    void initialize(const MotorDirectControllerParams& params)
    {
        params_ = params;
        motor_control_signals_.resize(params_.rotor_count);
        MotorDirectController::reset();
    }


    //*** Start ControllerBase implementation ****//
    virtual void reset() override
    {
        motor_control_signals_.assign(params_.rotor_count, 0);
    }

	virtual void update(real_T dt) override
	{
		//nothing to update in direct motor control
	}

    virtual real_T getVertexControlSignal(unsigned int rotor_index) override
    {
        return motor_control_signals_.at(rotor_index);
    }
    virtual size_t getVertexCount() override
    {
        return 4;
    }
    //*** End ControllerBase implementation ****//

    void setRotorControlSignal(unsigned int rotor_index, real_T value)
    {
        motor_control_signals_[rotor_index] = value;
    }

    virtual ~MotorDirectController() = default;

};

}} //namespace
#endif
