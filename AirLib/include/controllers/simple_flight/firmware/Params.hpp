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


    uint16_t motor_count = 16;
    uint16_t min_pwm = 1000;
    uint16_t max_pwm = 2000;
};


} //namespace