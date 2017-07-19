#pragma once

#include <cstdint>
#include <cstdbool>
#include "turbotrig/turbotrig.h"
#include "turbotrig/turbovec.h"
#include "param.hpp"


namespace ros_flight {

class Estimator {
public:
    struct state_t
    {
        quaternion_t q;
        vector_t euler;
        vector_t omega;

        //  float p;
        //  float q;
        //  float r;

        //  float phi;
        //  float theta;
        //  float psi;

        float altitude;
        uint64_t now_us;

    };

public:
    void init(Params* _params, bool use_matrix_exponential, bool use_quadratic_integration, bool use_accelerometer);
    void reset_state();
    void reset_adaptive_bias();
    void run_estimator(const vector_t& accel, const vector_t& gyro, const uint64_t& imu_time);
    state_t& state() { return _current_state; };

private:
    void run_LPF(const vector_t& accel, const vector_t& gyro);


private:
    state_t _current_state;
    Params* params;
    Board* board;

    vector_t _adaptive_gyro_bias;

    vector_t w1;
    vector_t w2;
    vector_t wbar;
    vector_t wfinal;
    vector_t w_acc;
    const vector_t g = {0.0f, 0.0f, -1.0f}; // 'static constexpr' here results in link error.
    vector_t b;
    quaternion_t q_tilde;
    quaternion_t q_hat;
    uint64_t last_time;

    bool mat_exp;
    bool quad_int;
    bool use_acc;

    vector_t _accel_LPF;
    vector_t _gyro_LPF;

    float kp = 0, ki = 0; //run_estimator
};


/************************************************** Implementation ***************************************************************/


void Estimator::init(Params* _params, bool use_matrix_exponential, bool use_quadratic_integration, bool use_accelerometer)
{
    unused(board);
    unused(_adaptive_gyro_bias);
    
    params = _params;

    mat_exp = use_matrix_exponential;
    quad_int = use_quadratic_integration;
    use_acc = use_accelerometer;

    last_time = 0;

    reset_state();
}

void Estimator::reset_state()
{
    _current_state.q.w = 1.0f;
    _current_state.q.x = 0.0f;
    _current_state.q.y = 0.0f;
    _current_state.q.z = 0.0f;
    _current_state.omega.x = 0.0f;
    _current_state.omega.y = 0.0f;
    _current_state.omega.z = 0.0f;
    _current_state.euler.x = 0.0f;
    _current_state.euler.y = 0.0f;
    _current_state.euler.z = 0.0f;

    q_hat.w = 1.0f;
    q_hat.x = 0.0f;
    q_hat.y = 0.0f;
    q_hat.z = 0.0f;

    w1.x = 0.0f;
    w1.y = 0.0f;
    w1.z = 0.0f;

    w2.x = 0.0f;
    w2.y = 0.0f;
    w2.z = 0.0f;

    b.x = 0.0f;
    b.y = 0.0f;
    b.z = 0.0f;

    w_acc.x = 0.0f;
    w_acc.y = 0.0f;
    w_acc.z = 0.0f;

    q_tilde.w = 1.0f;
    q_tilde.x = 0.0f;
    q_tilde.y = 0.0f;
    q_tilde.z = 0.0f;

    _accel_LPF.x = 0;
    _accel_LPF.y = 0;
    _accel_LPF.z = -9.80665f;

    _gyro_LPF.x = 0;
    _gyro_LPF.y = 0;
    _gyro_LPF.z = 0;
}

void Estimator::reset_adaptive_bias()
{
    b.x = 0;
    b.y = 0;
    b.z = 0;
}

void Estimator::run_LPF(const vector_t& accel, const vector_t& gyro)
{
    float alpha_acc = params->get_param_float(Params::PARAM_ACC_ALPHA);
    _accel_LPF.x = (1.0f-alpha_acc)*accel.x + alpha_acc*_accel_LPF.x;
    _accel_LPF.y = (1.0f-alpha_acc)*accel.y + alpha_acc*_accel_LPF.y;
    _accel_LPF.z = (1.0f-alpha_acc)*accel.z + alpha_acc*_accel_LPF.z;

    float alpha_gyro = params->get_param_float(Params::PARAM_GYRO_ALPHA);
    _gyro_LPF.x = (1.0f-alpha_gyro)*gyro.x + alpha_gyro*_gyro_LPF.x;
    _gyro_LPF.y = (1.0f-alpha_gyro)*gyro.y + alpha_gyro*_gyro_LPF.y;
    _gyro_LPF.z = (1.0f-alpha_gyro)*gyro.z + alpha_gyro*_gyro_LPF.z;
}


void Estimator::run_estimator(const vector_t& accel, const vector_t& gyro, const uint64_t& imu_time)
{
    _current_state.now_us = imu_time;
    if (last_time == 0 || _current_state.now_us <= last_time)
    {
        last_time = _current_state.now_us;
        return;
    }

    float dt = (_current_state.now_us - last_time) * 1e-6f;
    last_time = _current_state.now_us;

    // Crank up the gains for the first few seconds for quick convergence
    if (imu_time < (uint64_t)params->get_param_int(Params::PARAM_INIT_TIME)*1000)
    {
        kp = params->get_param_float(Params::PARAM_FILTER_KP)*10.0f;
        ki = params->get_param_float(Params::PARAM_FILTER_KI)*10.0f;
    }
    else
    {
        kp = params->get_param_float(Params::PARAM_FILTER_KP);
        ki = params->get_param_float(Params::PARAM_FILTER_KI);
    }

    // Run LPF to reject a lot of noise
    run_LPF(accel, gyro);

    // add in accelerometer
    float a_sqrd_norm = _accel_LPF.x*_accel_LPF.x + _accel_LPF.y*_accel_LPF.y + _accel_LPF.z*_accel_LPF.z;

    if (use_acc && a_sqrd_norm < 1.15f*1.15f*9.80665f*9.80665f && a_sqrd_norm > 0.85f*0.85f*9.80665f*9.80665f)
    {
        // Get error estimated by accelerometer measurement
        vector_t a = vector_normalize(_accel_LPF);
        // Get the quaternion from accelerometer (low-frequency measure q)
        // (Not in either paper)
        quaternion_t q_acc_inv = quaternion_inverse(quat_from_two_vectors(a, g));
        // Get the error quaternion between observer and low-freq q
        // Below Eq. 45 Mahoney Paper
        q_tilde = quaternion_multiply(q_acc_inv, q_hat);
        // Correction Term of Eq. 47a and 47b Mahoney Paper
        // w_acc = 2*s_tilde*v_tilde
        w_acc.x = -2.0f*q_tilde.w*q_tilde.x;
        w_acc.y = -2.0f*q_tilde.w*q_tilde.y;
        w_acc.z = 0.0f; // Don't correct z, because it's unobservable from the accelerometer

                        // integrate biases from accelerometer feedback
                        // (eq 47b Mahoney Paper, using correction term w_acc found above)
        b.x -= ki*w_acc.x*dt;
        b.y -= ki*w_acc.y*dt;
        b.z = 0.0;  // Don't integrate z bias, because it's unobservable
    }
    else
    {
        w_acc.x = 0.0f;
        w_acc.y = 0.0f;
        w_acc.z = 0.0f;
    }

    // Pull out Gyro measurements
    if (quad_int)
    {
        // Quadratic Integration (Eq. 14 Casey Paper)
        // this integration step adds 12 us on the STM32F10x chips
        wbar = vector_add(vector_add(scalar_multiply(-1.0f/12.0f,w2), scalar_multiply(8.0f/12.0f,w1)),
            scalar_multiply(5.0f/12.0f,_gyro_LPF));
        w2 = w1;
        w1 = _gyro_LPF;
    }
    else
    {
        wbar = _gyro_LPF;
    }

    // Build the composite omega vector for kinematic propagation
    // This the stuff inside the p function in eq. 47a - Mahoney Paper
    wfinal = vector_add(vector_sub(wbar, b), scalar_multiply(kp, w_acc));

    // Propagate Dynamics (only if we've moved)
    float sqrd_norm_w = sqrd_norm(wfinal);
    if (sqrd_norm_w > 0.0f)
    {
        float p = wfinal.x;
        float q = wfinal.y;
        float r = wfinal.z;

        if (mat_exp)
        {
            // Matrix Exponential Approximation (From Attitude Representation and Kinematic
            // Propagation for Low-Cost UAVs by Robert T. Casey)
            // (Eq. 12 Casey Paper)
            // This adds 90 us on STM32F10x chips
            float norm_w = sqrtf(sqrd_norm_w);
            quaternion_t qhat_np1;
            float t1 = cosf((norm_w*dt)/2.0f);
            float t2 = 1.0f/norm_w * sinf((norm_w*dt)/2.0f);
            qhat_np1.w = t1*q_hat.w   + t2*(          - p*q_hat.x - q*q_hat.y - r*q_hat.z);
            qhat_np1.x = t1*q_hat.x   + t2*(p*q_hat.w             + r*q_hat.y - q*q_hat.z);
            qhat_np1.y = t1*q_hat.y   + t2*(q*q_hat.w - r*q_hat.x             + p*q_hat.z);
            qhat_np1.z = t1*q_hat.z   + t2*(r*q_hat.w + q*q_hat.x - p*q_hat.y);
            q_hat = quaternion_normalize(qhat_np1);
        }
        else
        {
            // Euler Integration
            // (Eq. 47a Mahoney Paper), but this is pretty straight-forward
            quaternion_t qdot = {0.5f * (           - p*q_hat.x - q*q_hat.y - r*q_hat.z),
                0.5f * ( p*q_hat.w             + r*q_hat.y - q*q_hat.z),
                0.5f * ( q*q_hat.w - r*q_hat.x             + p*q_hat.z),
                0.5f * ( r*q_hat.w + q*q_hat.x - p*q_hat.y)
            };
            q_hat.w += qdot.w*dt;
            q_hat.x += qdot.x*dt;
            q_hat.y += qdot.y*dt;
            q_hat.z += qdot.z*dt;
            q_hat = quaternion_normalize(q_hat);
        }
    }

    // Save attitude estimate
    _current_state.q = q_hat;

    // Extract Euler Angles for controller
    euler_from_quat(_current_state.q, &_current_state.euler.x, &_current_state.euler.y, &_current_state.euler.z);

    // Save off adjust gyro measurements with estimated biases for control
    _current_state.omega = vector_sub(_gyro_LPF, b);
}



} //namespace