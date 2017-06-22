#ifndef RC_H_
#define RC_H_

#include "mux.hpp"
#include "param.hpp"
#include "board.hpp"
#include "commlink.hpp"

namespace ros_flight {

class RC {
public:
    void init(CommonState* _common_state, Board* _board, Mux* _mux, Params* _params, CommLink* _comm_link);
    bool rc_switch(int16_t channel);
    bool receive_rc(uint64_t now);

private:
    void calibrate_rc();
    void convertPWMtoRad();

private:
    typedef struct
    {
        int16_t channel;
        int16_t direction;
    } rc_switch_t;

    typedef enum
    {
        PARALLEL_PWM,
        CPPM,
    } rc_type_t;


    bool _calibrate_rc;
    rc_switch_t switches[4];

    CommonState* common_state;
    Mux* mux;
    Params* params;
    Board* board;
    CommLink* comm_link;

    uint64_t last_rc_receive_time = 0; //receive_rc
    uint64_t time_of_last_stick_deviation = 0; //receive_rc
    int32_t calib_max[4] = {0, 0, 0, 0};
    int32_t calib_min[4] = {10000, 10000, 10000, 10000};
    int32_t calib_sum[4] = {0, 0, 0, 0};
    int32_t calib_count[4] = {0, 0, 0, 0};

    bool last_is_angle_control, last_is_altitude_control;
};


/************************************************** Implementation ***************************************************************/

void RC::init(CommonState* _common_state, Board* _board, Mux* _mux, Params* _params, CommLink* _comm_link)
{
    common_state = _common_state;
    mux = _mux;
    params = _params;
    board = _board;
    comm_link = _comm_link;

    _calibrate_rc = false;

    Mux::control_t& rc_control = mux->rc_control();
    rc_control.x.type = Mux::control_type_t::ANGLE;
    rc_control.y.type = Mux::control_type_t::ANGLE;
    rc_control.z.type = Mux::control_type_t::RATE;
    rc_control.F.type = Mux::control_type_t::THROTTLE;

    rc_control.x.value = 0;
    rc_control.y.value = 0;
    rc_control.z.value = 0;
    rc_control.F.value = 0;

    Mux::control_t& offboard_control = mux->offboard_control();
    offboard_control.x.active = false;
    offboard_control.y.active = false;
    offboard_control.z.active = false;
    offboard_control.F.active = false;

    switches[0].channel = 4;
    switches[0].direction = params->get_param_int(Params::PARAM_RC_SWITCH_5_DIRECTION);
    switches[1].channel = 5;
    switches[1].direction = params->get_param_int(Params::PARAM_RC_SWITCH_6_DIRECTION);
    switches[2].channel = 6;
    switches[2].direction = params->get_param_int(Params::PARAM_RC_SWITCH_7_DIRECTION);
    switches[3].channel = 7;
    switches[3].direction = params->get_param_int(Params::PARAM_RC_SWITCH_8_DIRECTION);

    bool is_angle_control = rc_switch(params->get_param_int(Params::PARAM_RC_ATT_CONTROL_TYPE_CHANNEL));
    bool is_altitude_control = rc_switch(params->get_param_int(Params::PARAM_RC_F_CONTROL_TYPE_CHANNEL));
    comm_link->log_message(CommonState::stringf("Is angle/rate control = %i", is_angle_control).c_str(), 0);
    last_is_angle_control = is_angle_control;
    comm_link->log_message(CommonState::stringf("Is altitude/throttle control = %i", is_altitude_control).c_str(), 0);
    last_is_altitude_control = is_altitude_control;
}

bool RC::rc_switch(int16_t channel)
{
    if(channel < 4 || channel > 8)
    {
        return false;
    }
    if(switches[channel - 4].direction < 0)
    {
        return board->pwmRead(channel) < 1500;
    }
    else
    {
        return board->pwmRead(channel) > 1500;
    }
}

void RC::convertPWMtoRad()
{
    Mux::control_t& rc_control = mux->rc_control();

    // Get Roll control command out of RC
    if (rc_control.x.type == Mux::control_type_t::ANGLE)
    {
        rc_control.x.value = (float)((board->pwmRead(params->get_param_int(Params::PARAM_RC_X_CHANNEL)) - 1500)
            *2.0f*params->get_param_float(Params::PARAM_RC_MAX_ROLL))/(float)params->get_param_int(Params::PARAM_RC_X_RANGE);
    }
    else if (rc_control.x.type == Mux::control_type_t::RATE)
    {
        rc_control.x.value = (float)((board->pwmRead(params->get_param_int(Params::PARAM_RC_X_CHANNEL)) - 1500)
            *2.0f*params->get_param_float(Params::PARAM_RC_MAX_ROLLRATE))/(float)params->get_param_int(Params::PARAM_RC_X_RANGE);
    }
    else if (rc_control.x.type == Mux::control_type_t::MOTOR_DIRECT)
    {
        rc_control.x.value = static_cast<float>(
            board->pwmRead(params->get_param_int(Params::PARAM_RC_X_CHANNEL)) - params->get_param_int(Params::PARAM_RC_X_CENTER)
            );
    }

    // Get Pitch control command out of RC
    if (rc_control.y.type == Mux::control_type_t::ANGLE)
    {
        rc_control.y.value = ((board->pwmRead(params->get_param_int(Params::PARAM_RC_Y_CHANNEL)) - 1500)
            *2.0f*params->get_param_float(Params::PARAM_RC_MAX_PITCH))/(float)params->get_param_int(Params::PARAM_RC_Y_RANGE);
    }
    else if (rc_control.y.type == Mux::control_type_t::RATE)
    {
        rc_control.y.value = (float)((board->pwmRead(params->get_param_int(Params::PARAM_RC_Y_CHANNEL)) - 1500)
            *2.0f*params->get_param_float(Params::PARAM_RC_MAX_PITCHRATE))/(float)params->get_param_int(Params::PARAM_RC_Y_RANGE);
    }
    else if (rc_control.y.type == Mux::control_type_t::MOTOR_DIRECT)
    {
        rc_control.y.value = static_cast<float>(
            board->pwmRead(params->get_param_int(Params::PARAM_RC_Y_CHANNEL)) - 1500
            );
    }

    // Get the Yaw control command type out of RC
    if (rc_control.z.type == Mux::control_type_t::RATE)
    {
        rc_control.z.value = ((board->pwmRead(params->get_param_int(Params::PARAM_RC_Z_CHANNEL)) - 1500)
            *2.0f*params->get_param_float(Params::PARAM_RC_MAX_YAWRATE))/(float)params->get_param_int(Params::PARAM_RC_Z_RANGE);
    }
    else if (rc_control.z.type == Mux::control_type_t::MOTOR_DIRECT)
    {
        rc_control.z.value = static_cast<float>(board->pwmRead(params->get_param_int(Params::PARAM_RC_Z_CHANNEL)) - 1500);
    }

    // Finally, the Mux::control_type_t::THROTTLE command
    rc_control.F.value = (float)((board->pwmRead(params->get_param_int(Params::PARAM_RC_F_CHANNEL)) - params->get_param_int(Params::PARAM_RC_F_BOTTOM)))
        / (float)params->get_param_int(Params::PARAM_RC_F_RANGE);
}


bool RC::receive_rc(uint64_t now)
{
    if(_calibrate_rc)
    {
        calibrate_rc();
    }

    // if it has been more than 20ms then look for new RC values and parse them
    if (now - last_rc_receive_time < 20000)
    {
        return false;
    }
    last_rc_receive_time = now;

    // Get timestamp for deadband control lag
    Mux::control_t& rc_control = mux->rc_control();

    // Figure out the desired control type from the switches and params
    if (params->get_param_int(Params::PARAM_FIXED_WING))
    {
        // for using fixedwings
        rc_control.x.type = rc_control.y.type = rc_control.z.type = Mux::control_type_t::MOTOR_DIRECT;
        rc_control.F.type = Mux::control_type_t::THROTTLE;
    }
    else
    {
        bool is_angle_control = rc_switch(params->get_param_int(Params::PARAM_RC_ATT_CONTROL_TYPE_CHANNEL));
        bool is_altitude_control = rc_switch(params->get_param_int(Params::PARAM_RC_F_CONTROL_TYPE_CHANNEL));

        if (last_is_angle_control != is_angle_control || last_is_altitude_control != is_altitude_control) {
            comm_link->log_message(CommonState::stringf("Is angle or rate control = %i", is_angle_control).c_str(), 0);
            last_is_angle_control = is_angle_control;
        }
        if (last_is_altitude_control != is_altitude_control) {
            comm_link->log_message(CommonState::stringf("Is altitude or throttle control = %i", is_altitude_control).c_str(), 0);
            last_is_altitude_control = is_altitude_control;
        }

        rc_control.x.type = rc_control.y.type = is_angle_control ? Mux::control_type_t::ANGLE : Mux::control_type_t::RATE;
        rc_control.z.type = Mux::control_type_t::RATE;
        rc_control.F.type = is_altitude_control ? Mux::control_type_t::ALTITUDE : Mux::control_type_t::THROTTLE;
    }

    // Interpret PWM Values from RC
    convertPWMtoRad();

    // Set flags for attitude channels
    if (rc_switch(params->get_param_int(Params::PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL))
        || now - time_of_last_stick_deviation < (uint32_t)(params->get_param_int(Params::PARAM_OVERRIDE_LAG_TIME))*1000)
    {
        // Pilot is in full control
        rc_control.x.active = true;
        rc_control.y.active = true;
        rc_control.z.active = true;
    }
    else
    {
        // Check for stick deviation - if so, then the channel is active
        rc_control.x.active = rc_control.y.active  = rc_control.z.active =
            abs(board->pwmRead(params->get_param_int(Params::PARAM_RC_X_CHANNEL)) - params->get_param_int(Params::PARAM_RC_X_CENTER)) >
            params->get_param_int(Params::PARAM_RC_OVERRIDE_DEVIATION)
            || abs(board->pwmRead(params->get_param_int(Params::PARAM_RC_Y_CHANNEL)) - params->get_param_int(Params::PARAM_RC_Y_CENTER)) >
            params->get_param_int(Params::PARAM_RC_OVERRIDE_DEVIATION)
            || abs(board->pwmRead(params->get_param_int(Params::PARAM_RC_Z_CHANNEL)) - params->get_param_int(Params::PARAM_RC_Z_CENTER)) >
            params->get_param_int(Params::PARAM_RC_OVERRIDE_DEVIATION);
        if (rc_control.x.active)
        {
            // reset override lag
            time_of_last_stick_deviation = now;
        }
    }


    // Set flags for Mux::control_type_t::THROTTLE channel
    if (rc_switch(params->get_param_int(Params::PARAM_RC_THROTTLE_OVERRIDE_CHANNEL)))
    {
        // RC Pilot is in full control
        rc_control.F.active = true;
    }
    else
    {
        // Onboard Control - min Mux::control_type_t::THROTTLE Checking will be done in mux and in the controller.
        rc_control.F.active = false;
    }

    mux->set_new_command(true);
    return true;
}

void RC::calibrate_rc()
{
    if(common_state->is_armed())
    {
        comm_link->log_message("Cannot calibrate RC when FCU is armed", 5);
    }
    else
    {
        // Calibrate Extents of RC Transmitter
        comm_link->log_message("Calibrating RC, move sticks to full extents", 1);
        comm_link->log_message("in the next 10s", 1);
        uint64_t now = board->micros();

        while(board->micros() - now < 1e7)
        {
            for(int16_t i = 0; i < 4; i++)
            {
                int32_t read_value = (int32_t)board->pwmRead(i);
                if(read_value > calib_max[i])
                {
                    calib_max[i] = read_value;
                }
                if(read_value < calib_min[i])
                {
                    calib_min[i] = read_value;
                }
            }
            board->delay_millis(10);
        }
        params->set_param_int(Params::PARAM_RC_X_RANGE, calib_max[params->get_param_int(Params::PARAM_RC_X_CHANNEL)] - calib_min[params->get_param_int(Params::PARAM_RC_X_CHANNEL)]);
        params->set_param_int(Params::PARAM_RC_Y_RANGE, calib_max[params->get_param_int(Params::PARAM_RC_Y_CHANNEL)] - calib_min[params->get_param_int(Params::PARAM_RC_Y_CHANNEL)]);
        params->set_param_int(Params::PARAM_RC_Z_RANGE, calib_max[params->get_param_int(Params::PARAM_RC_Z_CHANNEL)] - calib_min[params->get_param_int(Params::PARAM_RC_Z_CHANNEL)]);
        params->set_param_int(Params::PARAM_RC_F_RANGE, calib_max[params->get_param_int(Params::PARAM_RC_F_CHANNEL)] - calib_min[params->get_param_int(Params::PARAM_RC_F_CHANNEL)]);

        // Calibrate Trimmed Centers
        comm_link->log_message("Calibrating RC, leave sticks at center", 1);
        comm_link->log_message("and Mux::control_type_t::THROTTLE low for next 10 seconds", 1);
        board->delay_millis(5000);
        now = board->micros();

        while(board->micros() - now < 5e6)
        {
            for(int16_t i = 0; i < 4; i++)
            {
                int32_t read_value = (int32_t)board->pwmRead(i);
                calib_sum[i] = calib_sum[i] + read_value;
                calib_count[i] = calib_count[i] + 1;
            }
            board->delay_millis(20); // RC is updated at 50 Hz
        }

        params->set_param_int(Params::PARAM_RC_X_CENTER, calib_sum[params->get_param_int(Params::PARAM_RC_X_CHANNEL)]/calib_count[params->get_param_int(Params::PARAM_RC_X_CHANNEL)]);
        params->set_param_int(Params::PARAM_RC_Y_CENTER, calib_sum[params->get_param_int(Params::PARAM_RC_Y_CHANNEL)]/calib_count[params->get_param_int(Params::PARAM_RC_Y_CHANNEL)]);
        params->set_param_int(Params::PARAM_RC_Z_CENTER, calib_sum[params->get_param_int(Params::PARAM_RC_Z_CHANNEL)]/calib_count[params->get_param_int(Params::PARAM_RC_Z_CHANNEL)]);
        params->set_param_int(Params::PARAM_RC_F_BOTTOM, calib_sum[params->get_param_int(Params::PARAM_RC_F_CHANNEL)]/calib_count[params->get_param_int(Params::PARAM_RC_F_CHANNEL)]);
    }

    // calculate Trim values (in terms of SI units)
    if(rc_switch(params->get_param_int(Params::PARAM_RC_ATT_CONTROL_TYPE_CHANNEL)))
    {
        // in angle mode
        params->set_param_float(Params::PARAM_ROLL_ANGLE_TRIM, (float)(params->get_param_int(Params::PARAM_RC_X_CENTER) - 1500)*2.0f*params->get_param_float(Params::PARAM_RC_MAX_ROLL)
            /(float)params->get_param_int(Params::PARAM_RC_X_RANGE));
        params->set_param_float(Params::PARAM_PITCH_ANGLE_TRIM, (float)(params->get_param_int(Params::PARAM_RC_Y_CENTER) - 1500)*2.0f*params->get_param_float(Params::PARAM_RC_MAX_PITCH)
            /(float)params->get_param_int(Params::PARAM_RC_Y_RANGE));
    }
    else
    {
        // in rate mode
        params->set_param_float(Params::PARAM_ROLL_RATE_TRIM, (float)(params->get_param_int(Params::PARAM_RC_X_CENTER) - 1500)*2.0f*params->get_param_float(Params::PARAM_RC_MAX_ROLLRATE)
            /(float)params->get_param_int(Params::PARAM_RC_X_RANGE));
        params->set_param_float(Params::PARAM_PITCH_RATE_TRIM, (float)(params->get_param_int(Params::PARAM_RC_Y_CENTER) - 1500)*2.0f*params->get_param_float(Params::PARAM_RC_MAX_PITCHRATE)
            /(float)params->get_param_int(Params::PARAM_RC_Y_RANGE));
    }
    params->set_param_float(Params::PARAM_YAW_RATE_TRIM, (float)(params->get_param_int(Params::PARAM_RC_Z_CENTER) - 1500)*2.0f*params->get_param_float(Params::PARAM_RC_MAX_YAWRATE)
        /(float)params->get_param_int(Params::PARAM_RC_Z_RANGE));

    params->write_params();

    comm_link->log_message("Completed RC calibration", 0);
    _calibrate_rc = false;
}



} //namespace
#endif
