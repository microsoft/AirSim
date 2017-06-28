#pragma once

#include "CommonStructs.hpp"

namespace simple_flight {

struct Params {
public:
    int16_t rc_channel_count = 12;
    int16_t rc_read_interval_ms = 10;
    int16_t rc_thrust_channel = 2;
    int16_t rc_pitch_channel = 3;
    int16_t rc_roll_channel = 0;
    int16_t rc_yaw_channel = 1;

    int16_t rc_rate_angle_channel = 6; //corresponds to switch 3


    //this should match up with target board
    //simulation board should respect possible values
    uint16_t motor_count = 4;
    float min_motor_output = 0;
    float max_motor_output = 1;
    float min_armed_output = 0.2f;

    static constexpr float pi = 3.14159265359f; //180-degrees
    float max_pitch_rate = 0.5f, max_roll_rate = 0.5f, max_yaw_rate = 3.0f; //in radians/sec
    float max_pitch_angle = pi / 6, max_roll_angle = pi / 6, max_yaw_angle = pi; // in radians

    //stabilizer params
    float p_pitch_rate = 0.2f, p_roll_rate = 0.2f, p_yaw_rate = 0.2f;
    float p_pitch_angle = 0.5f, p_roll_angle = 0.5f, p_yaw_angle = 0.5f;

    ControlMode default_control_mode = ControlMode::Angle;
};


} //namespace