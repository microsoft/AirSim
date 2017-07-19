#pragma once

#include <cstdint>
#include "sensors.hpp"
#include "rc.hpp"
#include "commonstate.hpp"
#include "param.hpp"


namespace ros_flight {

class Mode {
public:
    typedef enum
    {
        INVALID_CONTROL_MODE,
        INVALID_ARMED_STATE,
    } error_state_t;

    void init(Board* _board, CommLink* _comm_link, CommonState* _common_state, Sensors* _sensors, RC* _rc, Params* _params);
    bool check_mode(uint64_t now);

private:
    bool arm(void);
    void disarm(void);
    bool check_failsafe(void);
    void updateCommLinkArmStatus();

private:
    error_state_t _error_state;
    CommonState* common_state;
    Sensors* sensors;
    RC* rc;
    Params* params;
    Board* board;
    CommLink* comm_link;

    bool started_gyro_calibration = false; //arm
    uint8_t blink_count = 0; //check_failsafe
    uint64_t prev_time = 0; //check_mode
    uint32_t time_sticks_have_been_in_arming_position = 0; //check_mode
};


/************************************************** Implementation ***************************************************************/
void Mode::init(Board* _board, CommLink* _comm_link, CommonState* _common_state, Sensors* _sensors, RC* _rc, Params* _params)
{
    unused(_error_state);

    board = _board;
    comm_link = _comm_link;
    params = _params;
    common_state = _common_state;
    sensors = _sensors;
    rc = _rc;

    common_state->set_disarm();
}

bool Mode::arm(void)
{
    bool success = false;
    if (!started_gyro_calibration)
    {
        if (common_state->is_disarmed())
            comm_link->log_message("Cannot arm because gyro calibration is not complete", 1);
        
        sensors->start_gyro_calibration();
        started_gyro_calibration = true;
    } else if (sensors->gyro_calibration_complete())
    {
        started_gyro_calibration = false;
        common_state->set_arm();
        board->set_led(0, true);
        success = true;
    }

    updateCommLinkArmStatus();

    return success;
}

void Mode::disarm(void)
{
    common_state->set_disarm();
    board->set_led(0, true);

    updateCommLinkArmStatus();
}

/// TODO: Be able to tell if the RC has become disconnected during flight
bool Mode::check_failsafe(void)
{
    for (int8_t i = 0; i < params->get_param_int(Params::PARAM_RC_NUM_CHANNELS); i++)
    {
        if (board->pwmRead(i) < 900 || board->pwmRead(i) > 2100)
        {
            if (common_state->is_armed() || common_state->is_disarmed())
            {
                comm_link->log_message("Switching to failsafe mode because of invalid PWM RC inputs", 1);
                common_state->setArmedState(CommonState::FAILSAFE_DISARMED);
            }

            // blink LED
            if (blink_count > 25)
            {
                board->toggle_led(1);
                blink_count = 0;
            }
            blink_count++;
            return true;
        }
    }

    // we got a valid RC measurement for all channels
    if (common_state->get_armed_state() == CommonState::FAILSAFE_ARMED || common_state->get_armed_state() == CommonState::FAILSAFE_DISARMED)
    {
        // return to appropriate mode
        common_state->setArmedState(
            (common_state->get_armed_state() == CommonState::FAILSAFE_ARMED) ? CommonState::ARMED :CommonState::DISARMED
        );
    }
    return false;
}

void Mode::updateCommLinkArmStatus()
{
    if (common_state->is_armed())
        comm_link->log_message("Vehicle is now armed", 0);
    else if (common_state->is_disarmed())
        comm_link->log_message("Vehicle is now disarmed", 0);
    else
        comm_link->log_message("Attempt to arm or disarm failed", 0);

}

bool Mode::check_mode(uint64_t now)
{
    // see it has been at least 20 ms
    uint32_t dt = static_cast<uint32_t>(now - prev_time);
    if (dt < 20000)
    {
        return false;
    }

    // if it has, then do stuff
    prev_time = now;

    // check for failsafe mode
    if (check_failsafe())
    {
        return true;
    } else
    {
        // check for arming switch
        if (params->get_param_int(Params::PARAM_ARM_STICKS))
        {
            if (common_state->get_armed_state() == CommonState::DISARMED)
            {
                // if left stick is down and to the right
                if (board->pwmRead(params->get_param_int(Params::PARAM_RC_F_CHANNEL)) < params->get_param_int(Params::PARAM_RC_F_BOTTOM) + params->get_param_int(Params::PARAM_ARM_THRESHOLD)
                    && board->pwmRead(params->get_param_int(Params::PARAM_RC_Z_CHANNEL)) > (params->get_param_int(Params::PARAM_RC_Z_CENTER) + params->get_param_int(Params::PARAM_RC_Z_RANGE) / 2)
                    - params->get_param_int(Params::PARAM_ARM_THRESHOLD))
                {
                    time_sticks_have_been_in_arming_position += dt;
                } else
                {
                    time_sticks_have_been_in_arming_position = 0;
                }
                if (time_sticks_have_been_in_arming_position > 500000)
                {
                    if (arm())
                        time_sticks_have_been_in_arming_position = 0;
                }
            } else // _armed_state is ARMED
            {
                // if left stick is down and to the left
                if (board->pwmRead(params->get_param_int(Params::PARAM_RC_F_CHANNEL)) < params->get_param_int(Params::PARAM_RC_F_BOTTOM) +
                    params->get_param_int(Params::PARAM_ARM_THRESHOLD)
                    && board->pwmRead(params->get_param_int(Params::PARAM_RC_Z_CHANNEL)) < (params->get_param_int(Params::PARAM_RC_Z_CENTER) - params->get_param_int(Params::PARAM_RC_Z_RANGE) / 2)
                    + params->get_param_int(Params::PARAM_ARM_THRESHOLD))
                {
                    time_sticks_have_been_in_arming_position += dt;
                } else
                {
                    time_sticks_have_been_in_arming_position = 0;
                }
                if (time_sticks_have_been_in_arming_position > 500000)
                {
                    disarm();
                    time_sticks_have_been_in_arming_position = 0;
                }
            }
        } else
        {
            if (rc->rc_switch(params->get_param_int(Params::PARAM_ARM_CHANNEL)))
            {
                if (common_state->is_disarmed())
                    arm();
            } else
            {
                if (common_state->is_armed())
                    disarm();
            }
        }
    }
    return true;
}



} //namespace