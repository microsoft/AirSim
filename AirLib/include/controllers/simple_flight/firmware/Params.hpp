#pragma once

namespace simple_flight {

struct Params {
public:
    int16_t rc_channel_count = 12;
    int16_t rc_read_interval_ms = 10;
    int16_t rc_thrust_channel = 2;
    int16_t rc_pitch_channel = 3;
    int16_t rc_roll_channel = 0;
    int16_t rc_yaw_channel = 1;

    //this should match up with target board
    //simulation board should respect possible values
    uint16_t motor_count = 16;
    float min_motor_output = 0;
    float max_motor_output = 1;
    float min_armed_output = 0.05f;
};


} //namespace