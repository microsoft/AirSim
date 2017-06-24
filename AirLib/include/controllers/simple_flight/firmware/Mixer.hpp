#pragma once

#include <vector>
#include <algorithm>
#include "Params.hpp"
#include "CommonStructs.hpp"

namespace simple_flight {

class Mixer {
public:
    Mixer(const Params* params)
        : params_(params)
    {
    }

    void getMotorOutput(const Controls& controls, std::vector<float>& motor_outputs) const
    {
        for (int motor_index = 0; motor_index < kMotorCount; ++motor_index) {
            if (controls.throttle > 0) {
                motor_outputs[motor_index] =
                    controls.throttle * mixerQuadX[motor_index].throttle
                    + controls.pitch * mixerQuadX[motor_index].pitch
                    + controls.roll * mixerQuadX[motor_index].roll
                    - controls.yaw * mixerQuadX[motor_index].yaw
                    ;
            } 
            else {
                motor_outputs[motor_index] = params_->min_armed_output;
            }
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

        common_utils::Utils::log(
            common_utils::Utils::stringf("(%f, %f, %f, %f)", 
                motor_outputs[0], motor_outputs[1], motor_outputs[2], motor_outputs[3]
            ));
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
    static constexpr motorMixer_t mixerQuadX[] = { //QuadX config
        { 1.0f, 1.0f, 1.0f,  1.0f },          // FRONT_R
        { 1.0f, -1.0f, -1.0f,  1.0f },          // REAR_L
        { 1.0f, -1.0f, 1.0f, -1.0f },          // FRONT_L
        { 1.0f, 1.0f, -1.0f, -1.0f },          // REAR_R
    };

};


} //namespace