#pragma once

#include <cstdbool>
#include <cstdint>
#include "commonstate.hpp"
#include "param.hpp"
#include "board.hpp"


namespace ros_flight {

class Mux {
public:
    enum class control_type_t
    {
        RATE, // Channel is is in rate mode (mrad/s)
        ANGLE, // Channel command is in angle mode (mrad)
        THROTTLE, // Channel is direcly controlling throttle max/1000
        ALTITUDE, // Channel is commanding a specified altitude in cm
        MOTOR_DIRECT // Channel directly passes PWM input to the mixer
    };

    struct control_channel_t
    {
        bool active; // Whether or not the channel is active
        control_type_t type;  // What type the channel is
        float value; // The value of the channel
    };

    struct control_t
    {
        control_channel_t x;
        control_channel_t y;
        control_channel_t z;
        control_channel_t F;
    } ;
public:
    void init(CommonState* _common_state, Board* _board, Params* _params);
    bool mux_inputs();

    control_t& rc_control() { return _rc_control; }
    control_t& offboard_control() { return _offboard_control; }
    control_t& combined_control() { return _combined_control; }

    void set_new_command(bool val) { _new_command = val; }

private:
    CommonState* common_state;
    Params* params;
    Board* board;

    control_t _rc_control;
    control_t _offboard_control;
    control_t _combined_control;

    bool _new_command;

    control_t _failsafe_control = {
        {true, Mux::control_type_t::ANGLE, 0.0},
        {true, Mux::control_type_t::ANGLE, 0.0},
        {true, Mux::control_type_t::RATE, 0.0},
        {true, Mux::control_type_t::THROTTLE, 0.0}};
};


/************************************************** Implementation ***************************************************************/
void Mux::init(CommonState* _common_state, Board* _board, Params* _params)
{
    params = _params;
    common_state = _common_state;
    board = _board;

}

bool Mux::mux_inputs()
{
    if (!_new_command)
    {
        // we haven't received any new commands, so we shouldn't do anything
        return false;
    }
    // otherwise combine the new commands

    if(common_state->get_armed_state() == CommonState::FAILSAFE_ARMED || common_state->get_armed_state() == CommonState::FAILSAFE_DISARMED)
    {
        _combined_control = _failsafe_control;  
    }
    else
    {
        if (_rc_control.x.active)
        {
            _combined_control.x = _rc_control.x;
        }
        else if (_offboard_control.x.active)
        {
            _combined_control.x = _offboard_control.x;
        }
        else
        {
            // default to taking RC if neither is publishing
            _combined_control.x = _rc_control.x;
            _combined_control.x.active = true;
        }


        if (_rc_control.y.active)
        {
            _combined_control.y = _rc_control.y;
        }
        else if (_offboard_control.y.active)
        {
            _combined_control.y = _offboard_control.y;
        }
        else
        {
            // default to taking RC if neither is publishing
            _combined_control.y = _rc_control.y;
            _combined_control.y.active = true;
        }


        if (_rc_control.z.active)
        {
            _combined_control.z = _rc_control.z;
        }
        else if (_offboard_control.z.active)
        {
            _combined_control.z = _offboard_control.z;
        }
        else
        {
            _combined_control.z = _rc_control.z;
            _combined_control.z.active = true;
        }

        if (params->get_param_int(Params::PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE))
        {
            if (_offboard_control.F.active)
            {
                if (_rc_control.F.type == Mux::control_type_t::THROTTLE && _offboard_control.F.type == Mux::control_type_t::THROTTLE)
                {
                    _combined_control.F.value = (_rc_control.F.value > _offboard_control.F.value) ?
                        _offboard_control.F.value : _rc_control.F.value;
                    _combined_control.F.type = Mux::control_type_t::THROTTLE;
                    _combined_control.F.active = true;
                }
                else
                {
                    // I'm still not quite sure how to handle the mixed altitude/throttle cases
                    // for now, just pass the rc along.  I expect that what we really need to do
                    // is run the altitude controller here so we can compare throttle to throttle
                    _combined_control.F = _rc_control.F;
                }
            }
        }
        else // no min throttle check
        {
            if (_rc_control.F.active)
            {
                _combined_control.F = _rc_control.F;
            }
            else if (_offboard_control.F.active)
            {
                _combined_control.F = _offboard_control.F;
            }
            else
            {
                _combined_control.F = _rc_control.F;
                _combined_control.F.active = true;
            }
        }

        // Light to indicate override
        if (_rc_control.x.active || _rc_control.y.active || _rc_control.z.active || _rc_control.F.active)
        {
            board->set_led(0, true);
        }
        else
        {
            board->set_led(0, false);
        }
    }

    // reset the new command flag
    _new_command = false;
    return true;
}



} //namespace