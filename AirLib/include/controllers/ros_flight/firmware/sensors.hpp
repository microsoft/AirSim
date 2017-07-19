#pragma once

#include "turbotrig/turbovec.h"
#include <cstdint>
#include <cstdbool>
#include "estimator.hpp"
#include "commonstate.hpp"
#include "param.hpp"
#include "board.hpp"
#include "commlink.hpp"

namespace ros_flight {

class Sensors {
public:
    // function declarations
    void init(CommonState* _common_state, Board* _board, Estimator* _estimator, Params* _params, CommLink* _comm_link);
    bool update_sensors();

    bool start_imu_calibration(void);
    bool start_gyro_calibration(void);
    bool gyro_calibration_complete(void);

    void get_imu_measurements(vector_t& accel, vector_t& gyro, uint64_t& imu_time);

private:
    volatile uint8_t accel_status, gyro_status, temp_status;
    float accel_scale;
    float gyro_scale;
    bool calibrating_acc_flag;
    bool calibrating_gyro_flag;
    void calibrate_accel(void);
    void calibrate_gyro(void);
    void correct_imu(void);
    void imu_ISR(void);
    bool update_imu(void);

private:
    // IMU
    vector_t _accel;
    vector_t _gyro;
    float _imu_temperature;
    uint64_t _imu_time;
    bool new_imu_data = false;

    // Airspeed
    bool _diff_pressure_present = false;
    float _pitot_velocity, _pitot_diff_pressure, _pitot_temp;

    // Barometer
    bool _baro_present = false;
    float _baro_altitude;
    float _baro_pressure;
    float _baro_temperature;

    // Sonar
    bool _sonar_present = false;
    float _sonar_range;

    // Magnetometer
    bool _mag_present = false;
    vector_t _mag;

    // IMU stuff
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t temp_raw;

    Estimator* estimator;
    CommonState* common_state;
    Params* params;
    Board* board;
    CommLink* comm_link;

    uint32_t last_time_look_for_disarmed_sensors = 0;
    uint32_t last_imu_update_ms = 0;
    uint16_t calib_gyro_count = 0;
    vector_t calib_gyro_sum = { 0.0f, 0.0f, 0.0f };
    uint16_t calib_accel_count = 0;
    vector_t calib_accel_sum = { 0.0f, 0.0f, 0.0f };
    vector_t gravity = { 0.0f, 0.0f, 9.80665f };
    float acc_temp_sum = 0.0f;
};


/************************************************** Implementation ***************************************************************/


void Sensors::init(CommonState* _common_state, Board* _board, Estimator* _estimator, Params* _params, CommLink* _comm_link)
{
    unused(accel_status);
    unused(gyro_status);
    unused(temp_status);

    common_state = _common_state;
    estimator = _estimator;
    params = _params;
    board = _board;
    comm_link = _comm_link;

    uint16_t acc1G;
    board->init_sensors(acc1G, gyro_scale, params->get_param_int(Params::PARAM_BOARD_REVISION), std::bind(&Sensors::imu_ISR, this));
    accel_scale = 9.80665f / acc1G * params->get_param_float(Params::PARAM_ACCEL_SCALE);
}

void Sensors::get_imu_measurements(vector_t& accel, vector_t& gyro, uint64_t& imu_time)
{
    accel = _accel;
    gyro = _gyro;
    imu_time = _imu_time;
}

bool Sensors::update_sensors()
{
    // Look for disabled sensors while disarmed (poll every 10 seconds)
    // These sensors need power to respond, so they might not have been
    // detected on startup, but will be detected whenever power is applied
    // to the 5V rail.
    if (common_state->is_disarmed() && (!_sonar_present))//|| !_diff_pressure_present))
    {
        uint32_t now = board->millis();
        if (now > (last_time_look_for_disarmed_sensors + 500))
        {
            last_time_look_for_disarmed_sensors = now;
            if (!_sonar_present)
            {
                _sonar_present = board->is_sensor_present(board->SensorType::Sonar);
                if (_sonar_present)
                {
                    comm_link->log_message("FOUND SONAR", 0);
                }
            }
            if (!_diff_pressure_present)
            {
                _diff_pressure_present = board->is_sensor_present(board->SensorType::DiffPressure);
                if (_diff_pressure_present)
                {
                    comm_link->log_message("FOUND DIFF PRESS", 0);
                }
            }
        }
    }

    if (_baro_present)
    {
        board->read_baro(_baro_altitude, _baro_pressure, _baro_temperature);
    }

    if (_diff_pressure_present)
    {
        board->read_diff_pressure(_pitot_diff_pressure, _pitot_temp, _pitot_velocity);
    }

    if (_sonar_present)
    {
        _sonar_range = board->read_sonar();
    }

    if (_mag_present)
    {
        int16_t raw_mag[3] = { 0,0,0 };
        board->read_mag(raw_mag);
        _mag.x = (float)raw_mag[0];
        _mag.y = (float)raw_mag[1];
        _mag.z = (float)raw_mag[2];
    }

    // Return whether or not we got new IMU data
    return update_imu();
}


bool Sensors::start_imu_calibration(void)
{
    comm_link->log_message("Starting IMU calibration...", 0);

    start_gyro_calibration();

    calibrating_acc_flag = true;
    params->set_param_float(Params::PARAM_ACC_X_BIAS, 0.0);
    params->set_param_float(Params::PARAM_ACC_Y_BIAS, 0.0);
    params->set_param_float(Params::PARAM_ACC_Z_BIAS, 0.0);
    return true;
}

bool Sensors::start_gyro_calibration(void)
{
    comm_link->log_message("Starting gyro calibration...", 0);

    calibrating_gyro_flag = true;
    params->set_param_float(Params::PARAM_GYRO_X_BIAS, 0.0);
    params->set_param_float(Params::PARAM_GYRO_Y_BIAS, 0.0);
    params->set_param_float(Params::PARAM_GYRO_Z_BIAS, 0.0);
    return true;
}

bool Sensors::gyro_calibration_complete(void)
{
    return !calibrating_gyro_flag;
}


void Sensors::imu_ISR(void)
{
    _imu_time = board->micros();
    new_imu_data = true;
}


bool Sensors::update_imu(void)
{
    if (new_imu_data)
    {
        last_imu_update_ms = board->millis();
        board->read_accel(accel_raw);
        board->read_gyro(gyro_raw);
        board->read_temperature(temp_raw);
        new_imu_data = false;

        // convert temperature SI units (degC, m/s^2, rad/s)
        _imu_temperature = temp_raw / 340.0f + 36.53f;

        // convert to NED and SI units
        _accel.x = accel_raw[0] * accel_scale;
        _accel.y = -accel_raw[1] * accel_scale;
        _accel.z = -accel_raw[2] * accel_scale;

        _gyro.x = gyro_raw[0] * gyro_scale;
        _gyro.y = -gyro_raw[1] * gyro_scale;
        _gyro.z = -gyro_raw[2] * gyro_scale;

        if (calibrating_acc_flag == true)
            calibrate_accel();
        if (calibrating_gyro_flag)
            calibrate_gyro();

        correct_imu();
        return true;
    } else
    {
        // if we have lost 1000 IMU messages something is wrong
        if (board->millis() > last_imu_update_ms + 1000)
        {
            comm_link->log_message("Lost too many IMU messages! Reinitializing IMU...", 1);

            // change board revision and reset IMU
            last_imu_update_ms = board->millis();
            params->set_param_int(Params::PARAM_BOARD_REVISION, (params->get_param_int(Params::PARAM_BOARD_REVISION) >= 4) ? 5 : 2);
            uint16_t acc1G;
            board->init_imu(acc1G, gyro_scale, params->get_param_int(Params::PARAM_BOARD_REVISION));
            accel_scale = 9.80665f / acc1G * params->get_param_float(Params::PARAM_ACCEL_SCALE);
        }
        return false;
    }
}


void Sensors::calibrate_gyro()
{
    calib_gyro_sum = vector_add(calib_gyro_sum, _gyro);
    calib_gyro_count++;

    if (calib_gyro_count > 100)
    {
        // Gyros are simple.  Just find the average during the calibration
        vector_t gyro_bias = scalar_multiply(1.0f / (float)calib_gyro_count, calib_gyro_sum);

        if (sqrd_norm(gyro_bias) < 1.0)
        {
            params->set_param_float(Params::PARAM_GYRO_X_BIAS, gyro_bias.x);
            params->set_param_float(Params::PARAM_GYRO_Y_BIAS, gyro_bias.y);
            params->set_param_float(Params::PARAM_GYRO_Z_BIAS, gyro_bias.z);

            // Tell the estimator to reset it's bias estimate, because it should be zero now
            estimator->reset_adaptive_bias();
        } else
        {
            comm_link->log_message("Too much movement for gyro cal", 3);
        }

        // reset calibration in case we do it again
        calibrating_gyro_flag = false;
        calib_gyro_count = 0;
        calib_gyro_sum.x = 0.0f;
        calib_gyro_sum.y = 0.0f;
        calib_gyro_sum.z = 0.0f;
    }
}


void Sensors::calibrate_accel(void)
{
    calib_accel_sum = vector_add(vector_add(calib_accel_sum, _accel), gravity);
    acc_temp_sum += _imu_temperature;
    calib_accel_count++;

    if (calib_accel_count > 1000)
    {
        // The temperature bias is calculated using a least-squares regression.
        // This is computationally intensive, so it is done by the onboard computer in
        // fcu_io and shipped over to the flight controller.
        vector_t accel_temp_bias = {
            params->get_param_float(Params::PARAM_ACC_X_TEMP_COMP),
            params->get_param_float(Params::PARAM_ACC_Y_TEMP_COMP),
            params->get_param_float(Params::PARAM_ACC_Z_TEMP_COMP)
        };

        // Figure out the proper accel bias.
        // We have to consider the contribution of temperature during the calibration,
        // Which is why this line is so confusing. What we are doing, is first removing
        // the contribution of temperature to the measurements during the calibration,
        // Then we are dividing by the number of measurements.
        vector_t accel_bias = scalar_multiply(1.0f / (float)calib_accel_count, vector_sub(calib_accel_sum, scalar_multiply(acc_temp_sum, accel_temp_bias)));

        // Sanity Check -
        // If the accelerometer is upside down or being spun around during the calibration,
        // then don't do anything
        if (sqrd_norm(accel_bias) < 4.5)
        {
            params->set_param_float(Params::PARAM_ACC_X_BIAS, accel_bias.x);
            params->set_param_float(Params::PARAM_ACC_Y_BIAS, accel_bias.y);
            params->set_param_float(Params::PARAM_ACC_Z_BIAS, accel_bias.z);
            comm_link->log_message("IMU offsets captured", 0);

            // reset the estimated state
            estimator->reset_state();
            calibrating_acc_flag = false;
        } else
        {
            // check for bad _accel_scale
            if (sqrd_norm(accel_bias) > 4.5*4.5 && sqrd_norm(accel_bias) < 5.5*5.5)
            {
                comm_link->log_message("Detected bad IMU accel scale value", 4);
                params->set_param_float(Params::PARAM_ACCEL_SCALE, 2.0f * params->get_param_float(Params::PARAM_ACCEL_SCALE));
                accel_scale *= params->get_param_float(Params::PARAM_ACCEL_SCALE);
                params->write_params();
            } else if (sqrd_norm(accel_bias) > 9.0f*9.0f && sqrd_norm(accel_bias) < 11.0*11.0)
            {
                comm_link->log_message("Detected bad IMU accel scale value", 4);
                params->set_param_float(Params::PARAM_ACCEL_SCALE, 0.5f * params->get_param_float(Params::PARAM_ACCEL_SCALE));
                accel_scale *= params->get_param_float(Params::PARAM_ACCEL_SCALE);
                params->write_params();
            } else
            {
                comm_link->log_message("Too much movement for IMU cal", 2);
                calibrating_acc_flag = false;
            }
        }

        // reset calibration in case we do it again
        calib_accel_count = 0;
        calib_accel_sum.x = 0.0f;
        calib_accel_sum.y = 0.0f;
        calib_accel_sum.z = 0.0f;
        acc_temp_sum = 0.0f;
    }
}


void Sensors::correct_imu(void)
{
    // correct according to known biases and temperature compensation
    _accel.x -= params->get_param_float(Params::PARAM_ACC_X_TEMP_COMP)*_imu_temperature + params->get_param_float(Params::PARAM_ACC_X_BIAS);
    _accel.y -= params->get_param_float(Params::PARAM_ACC_Y_TEMP_COMP)*_imu_temperature + params->get_param_float(Params::PARAM_ACC_Y_BIAS);
    _accel.z -= params->get_param_float(Params::PARAM_ACC_Z_TEMP_COMP)*_imu_temperature + params->get_param_float(Params::PARAM_ACC_Z_BIAS);

    _gyro.x -= params->get_param_float(Params::PARAM_GYRO_X_BIAS);
    _gyro.y -= params->get_param_float(Params::PARAM_GYRO_Y_BIAS);
    _gyro.z -= params->get_param_float(Params::PARAM_GYRO_Z_BIAS);
}



} //namespace