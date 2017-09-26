#pragma once

#include <cstdint>
#include <cstdbool>

#include "mux.hpp"
#include "param.hpp"
#include "estimator.hpp"
#include "mixer.hpp"
#include "commonstate.hpp"


namespace ros_flight {

class Controller {
public:
    void run_controller();
    void init(CommonState* _common_state, Board* _board, Mux* _mux, Mixer* _mixer, Estimator* _estimator, Params* _params, CommLink* _comm_link);

private:
    typedef struct
    {
        Params::param_id_t kp_param_id;
        Params::param_id_t ki_param_id;
        Params::param_id_t kd_param_id;

        float* current_x;
        float* current_xdot;
        float* commanded_x;
        float* output;

        float max;
        float min;

        float integrator;
        float prev_time;
        float prev_x;
        float differentiator;
        float tau;
    } pid_t;

    pid_t pid_roll;
    pid_t pid_roll_rate;
    pid_t pid_pitch;
    pid_t pid_pitch_rate;
    pid_t pid_yaw_rate;
    pid_t pid_altitude;
    
    Estimator* estimator;
    CommonState* common_state;
    Mux* mux;
    Mixer* mixer;
    Params* params;
    Board* board;
    CommLink* comm_link;

    void init_pid(pid_t* pid, Params::param_id_t kp_param_id, Params::param_id_t ki_param_id, Params::param_id_t kd_param_id, float* current_x, float* current_xdot, float* commanded_x, float* output, float max, float min);
    void run_pid(pid_t* pid, float dt);

    //variables used by methods
    float prev_time = 0.0f; //run_controller

};


/************************************************** Implementation ***************************************************************/
void Controller::init(CommonState* _common_state, Board* _board, Mux* _mux, Mixer* _mixer, Estimator* _estimator, Params* _params, CommLink* _comm_link)
{
    estimator = _estimator;
    mux = _mux;
    mixer = _mixer;
    common_state = _common_state;
    params = _params;
    board = _board;
    comm_link = _comm_link;

    Mux::control_t& combined_control = mux->combined_control();
    Mixer::command_t& mixer_command = mixer->getCommand();

    init_pid(&pid_roll,
        Params::PARAM_PID_ROLL_ANGLE_P,
        Params::PARAM_PID_ROLL_ANGLE_I,
        Params::PARAM_PID_ROLL_ANGLE_D,
        &_estimator->state().euler.x,
        &_estimator->state().omega.x,
        &combined_control.x.value,
        &mixer_command.x,
        params->get_param_int(Params::PARAM_MAX_COMMAND) / 2.0f,
        -1.0f*params->get_param_int(Params::PARAM_MAX_COMMAND) / 2.0f);

    init_pid(&pid_pitch,
        Params::PARAM_PID_PITCH_ANGLE_P,
        Params::PARAM_PID_PITCH_ANGLE_I,
        Params::PARAM_PID_PITCH_ANGLE_D,
        &_estimator->state().euler.y,
        &_estimator->state().omega.y,
        &combined_control.y.value,
        &mixer_command.y,
        params->get_param_int(Params::PARAM_MAX_COMMAND) / 2.0f,
        -1.0f*params->get_param_int(Params::PARAM_MAX_COMMAND) / 2.0f);

    init_pid(&pid_roll_rate,
        Params::PARAM_PID_ROLL_RATE_P,
        Params::PARAM_PID_ROLL_RATE_I,
        Params::PARAM_PID_ROLL_RATE_D,
        &_estimator->state().omega.x,
        NULL,
        &combined_control.x.value,
        &mixer_command.x,
        params->get_param_int(Params::PARAM_MAX_COMMAND) / 2.0f,
        -1.0f*params->get_param_int(Params::PARAM_MAX_COMMAND) / 2.0f);

    init_pid(&pid_pitch_rate,
        Params::PARAM_PID_PITCH_RATE_P,
        Params::PARAM_PID_PITCH_RATE_I,
        Params::PARAM_PID_PITCH_RATE_D,
        &_estimator->state().omega.y,
        NULL,
        &combined_control.y.value,
        &mixer_command.y,
        params->get_param_int(Params::PARAM_MAX_COMMAND) / 2.0f,
        -1.0f*params->get_param_int(Params::PARAM_MAX_COMMAND) / 2.0f);

    init_pid(&pid_yaw_rate,
        Params::PARAM_PID_YAW_RATE_P,
        Params::PARAM_PID_YAW_RATE_I,
        Params::PARAM_PID_YAW_RATE_D,
        &_estimator->state().omega.z,
        NULL,
        &combined_control.z.value,
        &mixer_command.z,
        params->get_param_int(Params::PARAM_MAX_COMMAND) / 2.0f,
        -1.0f*params->get_param_int(Params::PARAM_MAX_COMMAND) / 2.0f);

    init_pid(&pid_altitude,
        Params::PARAM_PID_ALT_P,
        Params::PARAM_PID_ALT_I,
        Params::PARAM_PID_ALT_D,
        &_estimator->state().altitude,
        NULL,
        &combined_control.F.value,
        &mixer_command.F,
        static_cast<float>(params->get_param_int(Params::PARAM_MAX_COMMAND)),
        0.0f);
}

void Controller::init_pid(pid_t* pid, Params::param_id_t kp_param_id, Params::param_id_t ki_param_id, Params::param_id_t kd_param_id, float *current_x, float *current_xdot, float *commanded_x, float *output, float max, float min)
{
    pid->kp_param_id = kp_param_id;
    pid->ki_param_id = ki_param_id;
    pid->kd_param_id = kd_param_id;
    pid->current_x = current_x;
    pid->current_xdot = current_xdot;
    pid->commanded_x = commanded_x;
    pid->output = output;
    pid->max = max;
    pid->min = min;
    pid->integrator = 0.0f;
    pid->prev_time = board->micros()*1e-6f;
    pid->differentiator = 0.0f;
    pid->prev_x = 0.0f;
    pid->tau = params->get_param_float(Params::PARAM_PID_TAU);
}


void Controller::run_pid(pid_t *pid, float dt)
{
    if (dt > 0.010 || common_state->is_disarmed())
    {
        // This means that this is a ''stale'' controller and needs to be reset.
        // This would happen if we have been operating in a different mode for a while
        // and will result in some enormous integrator.
        // Or, it means we are disarmed and shouldn't integrate
        // Setting dt for this loop will mean that the integrator and dirty derivative
        // doesn't do anything this time but will keep it from exploding.
        dt = 0.0;
        pid->differentiator = 0.0;
    }

    // Calculate Error (make sure to de-reference pointers)
    float error = (*pid->commanded_x) - (*pid->current_x);

    // Initialize Terms
    float p_term = error * params->get_param_float(pid->kp_param_id);
    float i_term = 0.0;
    float d_term = 0.0;

    // If there is a derivative term
    if (pid->kd_param_id < Params::PARAMS_COUNT)
    {
        // calculate D term (use dirty derivative if we don't have access to a measurement of the derivative)
        // The dirty derivative is a sort of low-pass filtered version of the derivative.
        // (Be sure to de-reference pointers)
        if (pid->current_xdot == NULL)
        {
            if (dt > 0.0f) {
                pid->differentiator = (2.0f*pid->tau - dt) / (2.0f*pid->tau + dt)*pid->differentiator + 2.0f / (2.0f*pid->tau + dt)*((*pid->current_x) - pid->prev_x);
                pid->prev_x = *pid->current_x;
                d_term = params->get_param_float(pid->kd_param_id)*pid->differentiator;
            }
            else
                d_term = 0;
        } else
        {
            d_term = params->get_param_float(pid->kd_param_id) * (*pid->current_xdot);
        }
    }

    // If there is an integrator, we are armed, and throttle is high
    /// TODO: better way to figure out if throttle is high
    if ((pid->ki_param_id < Params::PARAMS_COUNT) && (common_state->is_armed()) && (board->pwmRead(params->get_param_int(Params::PARAM_RC_F_CHANNEL) > 1200)))
    {
        if (params->get_param_float(pid->ki_param_id) > 0.0)
        {
            // integrate
            pid->integrator += error*dt;
            // calculate I term (be sure to de-reference pointer to gain)
            i_term = params->get_param_float(pid->ki_param_id) * pid->integrator;
        }
    }

    // sum three terms
    float u = p_term + i_term - d_term;

    // Integrator anti-windup
    float u_sat = (u > pid->max) ? pid->max : (u < pid->min) ? pid->min : u;
    if (u != u_sat && fabs(i_term) > fabs(u - p_term + d_term))
        pid->integrator = (u_sat - p_term + d_term) / params->get_param_float(pid->ki_param_id);

    // Set output
    (*pid->output) = u_sat;

    return;
}

void Controller::run_controller()
{
    Mux::control_t& combined_control = mux->combined_control();
    Mixer::command_t& mixer_command = mixer->getCommand();

    if (prev_time < 0.0001f)
    {
        prev_time = estimator->state().now_us * 1e-6f;
        return;
    }

    float now = estimator->state().now_us * 1e-6f;
    float dt = now - prev_time;
    prev_time = now;

    // ROLL
    if (combined_control.x.type == Mux::control_type_t::RATE)
        run_pid(&pid_roll_rate, dt);
    else if (combined_control.x.type == Mux::control_type_t::ANGLE)
        run_pid(&pid_roll, dt);
    else // MOTOR_DIRECT
        mixer_command.x = combined_control.x.value;

    // PITCH
    if (combined_control.y.type == Mux::control_type_t::RATE)
        run_pid(&pid_pitch_rate, dt);
    else if (combined_control.y.type == Mux::control_type_t::ANGLE)
        run_pid(&pid_pitch, dt);
    else // MOTOR_DIRECT
        mixer_command.y = combined_control.y.value;

    // YAW
    if (combined_control.z.type == Mux::control_type_t::RATE)
        run_pid(&pid_yaw_rate, dt);
    else// MOTOR_DIRECT
        mixer_command.z = combined_control.z.value;

    // THROTTLE
    //  if(combined_control.F.type == ALTITUDE)
    //    run_pid(&pid_altitude);
    //  else // MOTOR_DIRECT
    mixer_command.F = combined_control.F.value;

    comm_link->notify_controller_updated();
}


} //namespace