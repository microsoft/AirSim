#pragma once

#include "vehicles/multirotor/RotorParams.hpp"

namespace msr { namespace airlib { namespace helper_func {

class Converter {
public:
    void getCommandArgumentsFromSpeed(const simple_flight::Axis4r& input_command_args, simple_flight::Axis4r& output_command_args)
    {
        simple_flight::Axis4r motor_outputs;
        rotor_params_.calculateMaxThrust();
        motor_outputs[0] = input_command_args.roll() / rotor_params_.max_speed;
        motor_outputs[1] = input_command_args.pitch() / rotor_params_.max_speed;
        motor_outputs[2] = input_command_args.yaw() / rotor_params_.max_speed;
        motor_outputs[3] = input_command_args.throttle() / rotor_params_.max_speed;

        simple_flight::Axis4r output_command_args_temp;
        for (int motor_index = 0; motor_index < kMotorCount; ++motor_index) {
            output_command_args_temp[motor_index] =
                motor_outputs[0] * mixerQuadXTransposeInv[motor_index].throttle
                + motor_outputs[1] * mixerQuadXTransposeInv[motor_index].pitch
                + motor_outputs[2] * mixerQuadXTransposeInv[motor_index].roll
                + motor_outputs[3] * mixerQuadXTransposeInv[motor_index].yaw
                ;
        }
        output_command_args = simple_flight::Axis4r(
            output_command_args_temp[2],
            output_command_args_temp[1],
            output_command_args_temp[3],
            output_command_args_temp[0]
        );
    }

private:
    const int kMotorCount = 4;
    msr::airlib::RotorParams rotor_params_;

    // Custom mixer data per motor
    typedef struct motorMixer_t {
        float throttle;
        float roll;
        float pitch;
        float yaw;
    } motorMixer_t;

    //only thing that this matrix does is change the sign
    const motorMixer_t mixerQuadXTransposeInv[4] = { //QuadX config
        { 0.25f, 0.25f, 0.25f, 0.25f },          // FRONT_R
        { -0.25f, 0.25f, 0.25f, -0.25f },          // REAR_L
        { 0.25f, -0.25f, 0.25f, -0.25f },          // FRONT_L
        { 0.25f, 0.25f, -0.25f, -0.25f },          // REAR_R
    };
};

}}} //namespace
