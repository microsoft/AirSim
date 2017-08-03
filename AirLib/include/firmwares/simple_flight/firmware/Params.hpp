#pragma once

#include "interfaces/CommonStructs.hpp"

namespace simple_flight {

struct Params {
public:
    static constexpr float min_armed_throttle = 0.1f;

    //this should match up with target board
    //simulation board should respect possible values
    struct Motor {
        uint16_t motor_count = 4;
        float min_motor_output = 0;
        float max_motor_output = 1;
        //if min_armed_output too low then noise in pitch/roll can destabilize quad copter when throttle is zero
        float min_angling_throttle = Params::min_armed_throttle / 2;
    } motor;

    struct Rc {
        uint16_t channel_count = 12;
        uint16_t read_interval_ms = 10;
        int16_t rate_level_mode_channel = 6; //corresponds to switch 3 in rc_data
        int16_t allow_api_control_channel = 7; //corresponds to switch 4 in rc_data

        //When actions such as arming/unarming, how much tolerance can be allowed in stick positions from 0 to 1?
        float action_request_tolerance = 0.1f;

        //milliseconds while sticks should stay in position
        uint64_t arm_duration = 100; 
        uint64_t disarm_duration = 100; 
        uint64_t neutral_duration = 100;
        
        Axis4<int16_t> channels = Axis4<int16_t>(2, 0, 3, 1);

        TReal max_angle_level_switch = 0.3f;

        //should be >= motor.min_angling_throttle
        float min_angling_throttle = Params::min_armed_throttle / 1.5f;
    } rc;

    struct AngleRatePid {
        //max_xxx_rate > 5 would introduce wobble/oscillations
        const float kMaxLimit = 2.5f;
        Axis3r max_limit = Axis3r(kMaxLimit, kMaxLimit, kMaxLimit); //roll, pitch, yaw - in radians/sec

        //p_xxx_rate params are sensetive to gyro noise. Values higher than 0.5 would require 
        //noise filteration
        const float kP = 0.5f;
        Axis3r p = Axis3r(kP, kP, kP);
    } angle_rate_pid;

    struct AngleLevelPid {
        const float pi = 3.14159265359f; //180-degrees
        
        //max_pitch/roll_angle > 5.5 would produce verticle thrust that is not enough to keep vehicle in air at extremeities of controls
        Axis3r max_limit = Axis3r(pi / 5.5f, pi / 5.5f, pi); //roll, pitch, yaw - in radians/sec

        const float kP = 2.5f;
        Axis3r p = Axis3r(kP, kP, kP);
    } angle_level_pid;

    GoalMode default_goal_mode = GoalMode::getStandardAngleMode();
    bool default_allow_api_control = false;
    VehicleState default_vehicle_state = VehicleState::Inactive;
};


} //namespace