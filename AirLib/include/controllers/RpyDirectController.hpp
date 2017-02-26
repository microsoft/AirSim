// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_air_copter_sim_RpyDirectController_hpp
#define msr_air_copter_sim_RpyDirectController_hpp

#include <exception>
#include "controllers/ControllerBase.hpp"
#include "RpyDirectControllerParams.hpp"
#include "common/common_utils/Utils.hpp"

namespace msr { namespace airlib {

class RpyDirectController : public ControllerBase {
public:
    RpyDirectController()
    {
        RpyDirectController::reset();
    }
    RpyDirectController(const RpyDirectControllerParams& params)
    {
        initialize(params);
    }
    void initialize(const RpyDirectControllerParams& params)
    {
        params_ = params;
        motor_control_signals_.resize(params_.rotor_count);

        if (params_.rotor_count != 4)
            throw std::invalid_argument(Utils::stringf("Rotor count of %d is not supported yet", params_.rotor_count));
        RpyDirectController::reset();
    }

    //*** Start ControllerBase implementation ****//
	virtual void reset() override
	{
        motor_control_signals_.assign(params_.rotor_count, 0);
	}

	virtual void update(real_T dt) override
	{
		real_T throttle_speed = scale(throttle_, 1.0f / params_.throttle_scale);
		real_T roll_speed = scale(roll_, throttle_speed / params_.roll_scale);
		real_T pitch_speed = scale(pitch_, throttle_speed / params_.pitch_scale);
		real_T yaw_speed = scale(yaw_, throttle_speed / params_.yaw_scale);

        motor_control_signals_[0] = Utils::clip(throttle_speed - pitch_speed + roll_speed + yaw_speed, 0.0f, 1.0f);
        motor_control_signals_[1] = Utils::clip(throttle_speed + pitch_speed + roll_speed - yaw_speed, 0.0f, 1.0f);
        motor_control_signals_[2] = Utils::clip(throttle_speed + pitch_speed - roll_speed + yaw_speed, 0.0f, 1.0f);
        motor_control_signals_[3] = Utils::clip(throttle_speed - pitch_speed - roll_speed - yaw_speed, 0.0f, 1.0f);
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


    //RPY range -1 to 1, throttle 0 to 1
    void setDesired(real_T throttle, real_T roll, real_T pitch, real_T yaw)
    {
        throttle_ = throttle; roll_ = roll; pitch_ = pitch; yaw_ = yaw;
    }

    virtual ~RpyDirectController() = default;

private:
	//input between -1 to 1
	real_T scale(real_T input, real_T factor)
	{
		return input * factor;
	}

private:
    real_T roll_ = 0, pitch_ = 0, yaw_ = 0, throttle_ = 0;
    vector<real_T> motor_control_signals_;
    RpyDirectControllerParams params_;
};

}} //namespace
#endif
