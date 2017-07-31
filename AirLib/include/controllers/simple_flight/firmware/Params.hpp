#pragma once

#include "interfaces/CommonStructs.hpp"

namespace simple_flight {

struct Params {
public:
    uint16_t rc_channel_count = 12;
    uint16_t rc_read_interval_ms = 10;
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
    //if min_armed_output too low then noise in pitch/roll can destabilize quad copter when throttle is zero
    float min_armed_output = 0.2f;

    static constexpr float pi = 3.14159265359f; //180-degrees

    //max_xxx_rate > 5 would introduce wobble/oscillations
    static constexpr float kMaxAngleRateDefault = 2.5f;
    Axis3r max_angle_rate = Axis3r(kMaxAngleRateDefault, kMaxAngleRateDefault, kMaxAngleRateDefault); //roll, pitch, yaw - in radians/sec

    //max_pitch/roll_angle > 5.5 would produce verticle thrust that is not enough to keep vehicle in air at extremeities of controls
    Axis3r max_angle_level = Axis3r(pi / 5.5f, pi / 5.5f, pi); //roll, pitch, yaw - in radians/sec

    //stabilizer params
    //p_xxx_rate params are sensetive to gyro noise. Values higher than 0.5 would require 
    //noise filteration
    static constexpr float kPidPAngleRateDefault = 0.5f;
    Axis3r pid_p_angle_rate = Axis3r(kPidPAngleRateDefault, kPidPAngleRateDefault, kPidPAngleRateDefault);
    
    static constexpr float kPidPAngleLevelDefault = 2.5f;
    Axis3r pid_p_angle_level = Axis3r(kPidPAngleLevelDefault, kPidPAngleLevelDefault, kPidPAngleLevelDefault);

    GoalMode default_goal_mode = GoalMode::getStandardAngleMode();
};


} //namespace