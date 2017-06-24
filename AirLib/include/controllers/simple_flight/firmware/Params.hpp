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
    uint16_t motor_count = 4;
    float min_motor_output = 0;
    float max_motor_output = 1;
    float min_armed_output = 0.2f;

    //in radians/sec
    float max_pitch_rate = 0.1, max_roll_rate = 0.1, max_yaw_rate = 3;

    //stabilizer params
    float p_pitch_rate = 0.01f, p_roll_rate = 0.01f, p_yaw_rate = 0.01f;
};


} //namespace