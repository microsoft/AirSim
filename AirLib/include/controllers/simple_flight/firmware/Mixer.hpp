#pragma once

#include <vector>
#include <algorithm>
#include "Params.hpp"
#include "interfaces/CommonStructs.hpp"

namespace simple_flight {

class Mixer {
public:
    Mixer(const Params* params)
        : params_(params)
    {
    }

    void getMotorOutput(const Axis4r& controls, std::vector<float>& motor_outputs) const
    {
        //if throttle is too low then set all motors to same value as throttle because
        //otherwise values in pitch/roll/yaw would get clipped randomly and can produce random results
        //in other words: we can't do angling if throttle is too low
        if (controls.throttle < params_->min_armed_output) {
            motor_outputs.assign(params_->motor_count, controls.throttle);
            return;
        }


        for (int motor_index = 0; motor_index < kMotorCount; ++motor_index) {
            motor_outputs[motor_index] =
                controls.throttle * mixerQuadX[motor_index].throttle
                + controls.axis3.pitch() * mixerQuadX[motor_index].pitch
                + controls.axis3.roll() * mixerQuadX[motor_index].roll
                + controls.axis3.yaw() * mixerQuadX[motor_index].yaw
                ;
        }

        float min_motor = *std::min_element(motor_outputs.begin(), motor_outputs.begin() + kMotorCount);
        if (min_motor < params_->min_motor_output) {
            float undershoot = params_->min_motor_output - min_motor;
            for (int motor_index = 0; motor_index < kMotorCount; ++motor_index)
                motor_outputs[motor_index] += undershoot;
        }

        float max_motor = *std::max_element(motor_outputs.begin(), motor_outputs.begin() + kMotorCount);
        float scale = max_motor / params_->max_motor_output;
        if (scale > params_->max_motor_output) {
            for (int motor_index = 0; motor_index < kMotorCount; ++motor_index)
                motor_outputs[motor_index] /= scale;
        }

        for (int motor_index = 0; motor_index < kMotorCount; ++motor_index)
            motor_outputs[motor_index] = std::max(params_->min_motor_output, std::min(motor_outputs[motor_index], params_->max_motor_output));

        //common_utils::Utils::log(
        //    common_utils::Utils::stringf("(%f, %f, %f, %f)", 
        //        motor_outputs[0], motor_outputs[1], motor_outputs[2], motor_outputs[3]
        //    ));
    }

private:
    const int kMotorCount = 4;

    const Params* params_;

    // Custom mixer data per motor
    typedef struct motorMixer_t {
        float throttle;
        float roll;
        float pitch;
        float yaw;
    } motorMixer_t;

    //only thing that this matrix does is change the sign
    const motorMixer_t mixerQuadX[4] = { //QuadX config
        { 1.0f, -1.0f, 1.0f, 1.0f },          // FRONT_R
        { 1.0f, 1.0f, -1.0f, 1.0f },          // REAR_L
        { 1.0f, 1.0f, 1.0f, -1.0f },          // FRONT_L
        { 1.0f, -1.0f, -1.0f, -1.0f },          // REAR_R
    };

};


} //namespace