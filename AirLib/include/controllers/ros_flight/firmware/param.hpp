#pragma once

#include <cstdbool>
#include <cstdint>
#include "param.hpp"
#include "board.hpp"
#include "commlink.hpp"
#include "commonstate.hpp"

namespace ros_flight {

class Params {
public:
    static constexpr uint8_t PARAMS_NAME_LENGTH = 15;

    typedef enum : uint16_t
    {
        /******************************/
        /*** HARDWARE CONFIGURATION ***/
        /******************************/
        PARAM_BOARD_REVISION = 0,
        PARAM_BAUD_RATE,

        /*****************************/
        /*** COMM LINK CONFIGURATION ***/
        /*****************************/
        PARAM_SYSTEM_ID,
        PARAM_STREAM_HEARTBEAT_RATE,

        PARAM_STREAM_ATTITUDE_RATE,
        PARAM_STREAM_IMU_RATE,
        PARAM_STREAM_MAG_RATE,
        PARAM_STREAM_BARO_RATE,
        PARAM_STREAM_AIRSPEED_RATE,
        PARAM_STREAM_GPS_RATE,
        PARAM_STREAM_SONAR_RATE,

        PARAM_STREAM_SERVO_OUTPUT_RAW_RATE,
        PARAM_STREAM_RC_RAW_RATE,

        /********************************/
        /*** CONTROLLER CONFIGURATION ***/
        /********************************/
        PARAM_MAX_COMMAND,

        PARAM_PID_ROLL_RATE_P,
        PARAM_PID_ROLL_RATE_I,
        PARAM_PID_ROLL_RATE_D,
        PARAM_ROLL_RATE_TRIM,
        PARAM_MAX_ROLL_RATE,

        PARAM_PID_PITCH_RATE_P,
        PARAM_PID_PITCH_RATE_I,
        PARAM_PID_PITCH_RATE_D,
        PARAM_PITCH_RATE_TRIM,
        PARAM_MAX_PITCH_RATE,

        PARAM_PID_YAW_RATE_P,
        PARAM_PID_YAW_RATE_I,
        PARAM_PID_YAW_RATE_D,
        PARAM_YAW_RATE_TRIM,
        PARAM_MAX_YAW_RATE,

        PARAM_PID_ROLL_ANGLE_P,
        PARAM_PID_ROLL_ANGLE_I,
        PARAM_PID_ROLL_ANGLE_D,
        PARAM_ROLL_ANGLE_TRIM,
        PARAM_MAX_ROLL_ANGLE,

        PARAM_PID_PITCH_ANGLE_P,
        PARAM_PID_PITCH_ANGLE_I,
        PARAM_PID_PITCH_ANGLE_D,
        PARAM_PITCH_ANGLE_TRIM,
        PARAM_MAX_PITCH_ANGLE,

        PARAM_PID_ALT_P,
        PARAM_PID_ALT_I,
        PARAM_PID_ALT_D,
        PARAM_HOVER_THROTTLE,

        PARAM_PID_TAU,

        /*************************/
        /*** PWM CONFIGURATION ***/
        /*************************/
        PARAM_MOTOR_PWM_SEND_RATE,
        PARAM_MOTOR_IDLE_PWM,
        PARAM_SPIN_MOTORS_WHEN_ARMED,

        /*******************************/
        /*** ESTIMATOR CONFIGURATION ***/
        /*******************************/
        PARAM_INIT_TIME,
        PARAM_FILTER_KP,
        PARAM_FILTER_KI,

        PARAM_GYRO_ALPHA,
        PARAM_ACC_ALPHA,

        PARAM_ACCEL_SCALE,

        PARAM_GYRO_X_BIAS,
        PARAM_GYRO_Y_BIAS,
        PARAM_GYRO_Z_BIAS,
        PARAM_ACC_X_BIAS,
        PARAM_ACC_Y_BIAS,
        PARAM_ACC_Z_BIAS,
        PARAM_ACC_X_TEMP_COMP,
        PARAM_ACC_Y_TEMP_COMP,
        PARAM_ACC_Z_TEMP_COMP,

        /************************/
        /*** RC CONFIGURATION ***/
        /************************/
        PARAM_RC_TYPE,
        PARAM_RC_X_CHANNEL,
        PARAM_RC_Y_CHANNEL,
        PARAM_RC_Z_CHANNEL,
        PARAM_RC_F_CHANNEL,
        PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL,
        PARAM_RC_THROTTLE_OVERRIDE_CHANNEL,
        PARAM_RC_ATT_CONTROL_TYPE_CHANNEL,
        PARAM_RC_F_CONTROL_TYPE_CHANNEL,
        PARAM_RC_NUM_CHANNELS,

        PARAM_RC_X_CENTER,
        PARAM_RC_Y_CENTER,
        PARAM_RC_Z_CENTER,
        PARAM_RC_F_BOTTOM,
        PARAM_RC_X_RANGE,
        PARAM_RC_Y_RANGE,
        PARAM_RC_Z_RANGE,
        PARAM_RC_F_RANGE,
        PARAM_RC_SWITCH_5_DIRECTION,
        PARAM_RC_SWITCH_6_DIRECTION,
        PARAM_RC_SWITCH_7_DIRECTION,
        PARAM_RC_SWITCH_8_DIRECTION,

        PARAM_RC_OVERRIDE_DEVIATION,
        PARAM_OVERRIDE_LAG_TIME,
        PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE,

        PARAM_RC_MAX_ROLL,
        PARAM_RC_MAX_PITCH,
        PARAM_RC_MAX_ROLLRATE,
        PARAM_RC_MAX_PITCHRATE,
        PARAM_RC_MAX_YAWRATE,

        /***************************/
        /*** FRAME CONFIGURATION ***/
        /***************************/
        PARAM_MIXER,

        PARAM_FIXED_WING,
        PARAM_ELEVATOR_REVERSE,
        PARAM_AILERON_REVERSE,
        PARAM_RUDDER_REVERSE,

        /********************/
        /*** ARMING SETUP ***/
        /********************/
        PARAM_ARM_STICKS,
        PARAM_ARM_CHANNEL,
        PARAM_ARM_THRESHOLD,

        // keep track of size of params array
        PARAMS_COUNT
    } param_id_t;

    typedef enum
    {
        PARAM_TYPE_INT32,
        PARAM_TYPE_FLOAT,
        PARAM_TYPE_INVALID
    } param_type_t;

    // function declarations
    /**
     * @brief Initialize parameter values
     */
    void init(Board* _board, CommLink* _comm_link);

    /**
     * @brief Set all parameters to default values
     */
    void set_param_defaults(void);

    /**
     * @brief Read parameter values from non-volatile memory
     * @return True if successful, false otherwise
     */
    bool read_params(void);

    /**
     * @brief Write current parameter values to non-volatile memory
     * @return True if successful, false otherwise
     */
    bool write_params(void);

    /**
     * @brief Callback for executing actions that need to be taken when a parameter value changes
     * @param id The ID of the parameter that was changed
     */
    void param_change_callback(param_id_t id);

    /**
     * @brief Gets the id of a parameter from its name
     * @param name The name of the parameter
     * @return The ID of the parameter if the name is valid, PARAMS_COUNT otherwise (invalid ID)
     */
    param_id_t lookup_param_id(const char name[PARAMS_NAME_LENGTH]);

    /**
     * @brief Get the value of an integer parameter by id
     * @param id The ID of the parameter
     * @return The value of the parameter
     */
    int get_param_int(param_id_t id);

    /**
     * @brief Get the value of a floating point parameter by id
     * @param id The ID of the parameter
     * @return The value of the parameter
     */
    float get_param_float(param_id_t id);

    /**
     * @brief Get the name of a parameter
     * @param id The ID of the parameter
     * @return The name of the parameter
     */
    char * get_param_name(param_id_t id);

    /**
     * @brief Get the type of a parameter
     * @param id The ID of the parameter
     * @return The type of the parameter
     * This returns one of three possible types
     * PARAM_TYPE_INT32, PARAM_TYPE_FLOAT, or PARAM_TYPE_INVALID
     * See line 165
     */
    param_type_t get_param_type(param_id_t id);

    /**
     * @brief Sets the value of a parameter by ID and calls the parameter change callback
     * @param id The ID of the parameter
     * @param value The new value
     * @return True if a parameter value was changed, false otherwise
     */
    bool set_param_int(param_id_t id, int32_t value);

    /**
     * @brief Sets the value of a floating point parameter by ID and calls the parameter callback
     * @param id The ID of the parameter
     * @param value The new value
     * @return  True if a parameter was changed, false otherwise
     */
    bool set_param_float(param_id_t id, float value);

    /**
     * @brief Sets the value of a parameter by name and calls the parameter change callback
     * @param name The name of the parameter
     * @param value The new value
     * @return True if a parameter value was changed, false otherwise
     */
    bool set_param_by_name_int(const char name[PARAMS_NAME_LENGTH], int32_t value);

    /**
     * @brief Sets the value of a floating point parameter by name and calls the parameter change callback
     * @param name The name of the parameter
     * @param value The new value
     * @return True if a parameter value was changed, false otherwise
     */
    bool set_param_by_name_float(const char name[PARAMS_NAME_LENGTH], float value);

private:
    void init_param_int(param_id_t id, const char name[PARAMS_NAME_LENGTH], int32_t value);
    void init_param_float(param_id_t id, const char name[PARAMS_NAME_LENGTH], float value);


private:
    // type definitions
    typedef struct
    {
        uint8_t version;
        uint16_t size;
        uint8_t magic_be;                       // magic number, should be 0xBE

        int32_t values[PARAMS_COUNT];
        char names[PARAMS_COUNT][PARAMS_NAME_LENGTH];
        param_type_t types[PARAMS_COUNT];

        uint8_t magic_ef;                       // magic number, should be 0xEF
        uint8_t chk;                            // XOR checksum
    } params_t;

    params_t _params;
    Board* board;
    CommLink* comm_link;

};



/************************************************** Implementation ***************************************************************/


// function definitions
void Params::init(Board* _board, CommLink* _comm_link)
{
    board = _board;
    comm_link = _comm_link;

    for (uint8_t i = 0; i < PARAMS_COUNT; i++)
    {
        init_param_int(static_cast<param_id_t>(i), "DEFAULT", 0);
    }
    board->init_params();
    if (!read_params())
    {
        set_param_defaults();
        write_params();
    }

    for (uint16_t id = 0; id < PARAMS_COUNT; id++)
        param_change_callback((param_id_t)id);
}

// local function definitions
void Params::init_param_int(Params::param_id_t id, const char name[Params::PARAMS_NAME_LENGTH], int32_t value)
{
    memcpy(_params.names[id], name, PARAMS_NAME_LENGTH);
    _params.values[id] = value;
    _params.types[id] = PARAM_TYPE_INT32;
}

void Params::init_param_float(Params::param_id_t id, const char name[Params::PARAMS_NAME_LENGTH], float value)
{
    memcpy(_params.names[id], name, PARAMS_NAME_LENGTH);
    _params.values[id] = *((int32_t *)&value);
    _params.types[id] = PARAM_TYPE_FLOAT;
}

void Params::set_param_defaults(void)
{
    /******************************/
    /*** HARDWARE CONFIGURATION ***/
    /******************************/
    init_param_int(PARAM_BOARD_REVISION, "BOARD_REV", 2); // Major board revision of naze32/flip32 | 1 | 6
    init_param_int(PARAM_BAUD_RATE, "BAUD_RATE", 921600); // Baud rate of communication with onboard computer | 9600 | 921600

    /*****************************/
    /*** MAVLINK CONFIGURATION ***/
    /*****************************/
    init_param_int(PARAM_SYSTEM_ID, "SYS_ID", 1); // Mavlink System ID  | 1 | 255
    init_param_int(PARAM_STREAM_HEARTBEAT_RATE, "STRM_HRTBT", 1); // Rate of heartbeat streaming (Hz) | 0 | 1000

    init_param_int(PARAM_STREAM_ATTITUDE_RATE, "STRM_ATTITUDE", 100); // Rate of attitude stream (Hz) | 0 | 1000
    init_param_int(PARAM_STREAM_IMU_RATE, "STRM_IMU", 500); // Rate of IMU stream (Hz) | 0 | 1000
    init_param_int(PARAM_STREAM_MAG_RATE, "STRM_MAG", 75); // Rate of magnetometer stream (Hz) | 0 | 75
    init_param_int(PARAM_STREAM_BARO_RATE, "STRM_BARO", 100); // Rate of barometer stream (Hz) | 0 | 100
    init_param_int(PARAM_STREAM_AIRSPEED_RATE, "STRM_AIRSPEED", 20); // Rate of airspeed stream (Hz) | 0 |  50
    init_param_int(PARAM_STREAM_GPS_RATE, "STRM_GPS", 0); // Rate of GPS stream (Hz) | 0 | 1
    init_param_int(PARAM_STREAM_SONAR_RATE, "STRM_SONAR", 40); // Rate of sonar stream (Hz) | 0 | 40

    init_param_int(PARAM_STREAM_SERVO_OUTPUT_RAW_RATE, "STRM_SERVO", 50); // Rate of raw output stream | 0 |  490
    init_param_int(PARAM_STREAM_RC_RAW_RATE, "STRM_RC", 50); // Rate of raw RC input stream | 0 | 50

    /********************************/
    /*** CONTROLLER CONFIGURATION ***/
    /********************************/
    init_param_int(PARAM_MAX_COMMAND, "PARAM_MAX_CMD", 1000); // saturation point for PID controller output | 0 | 1000

    init_param_float(PARAM_PID_ROLL_RATE_P, "PID_ROLL_RATE_P", 0.070f); // Roll Rate Proportional Gain | 0.0 | 1000.0
    init_param_float(PARAM_PID_ROLL_RATE_I, "PID_ROLL_RATE_I", 0.000f); // Roll Rate Integral Gain | 0.0 | 1000.0
    init_param_float(PARAM_PID_ROLL_RATE_D, "PID_ROLL_RATE_D", 0.000f); // Rall Rate Derivative Gain | 0.0 | 1000.0
    init_param_float(PARAM_ROLL_RATE_TRIM, "ROLL_RATE_TRIM", 0.0f); // Roll Rate Trim - See RC calibration | -1000.0 | 1000.0
    init_param_float(PARAM_MAX_ROLL_RATE, "MAX_ROLL_RATE", 3.14159f); // Maximum Roll Rate command accepted into PID controllers | 0.0 | 1000.0

    init_param_float(PARAM_PID_PITCH_RATE_P, "PID_PITCH_RATE_P", 0.070f);  // Pitch Rate Proporitional Gain | 0.0 | 1000.0
    init_param_float(PARAM_PID_PITCH_RATE_I, "PID_PITCH_RATE_I", 0.0000f); // Pitch Rate Integral Gain | 0.0 | 1000.0
    init_param_float(PARAM_PID_PITCH_RATE_D, "PID_PITCH_RATE_D", 0.0000f); // Pitch Rate Derivative Gain | 0.0 | 1000.0
    init_param_float(PARAM_PITCH_RATE_TRIM, "PITCH_RATE_TRIM", 0.0f); // Pitch Rate Trim - See RC calibration | -1000.0 | 1000.0
    init_param_float(PARAM_MAX_PITCH_RATE, "MAX_PITCH_RATE", 3.14159f);  // Maximum Pitch Rate command accepted into PID controllers | 0.0 | 1000.0

    init_param_float(PARAM_PID_YAW_RATE_P, "PID_YAW_RATE_P", 0.25f);   // Yaw Rate Proporitional Gain | 0.0 | 1000.0
    init_param_float(PARAM_PID_YAW_RATE_I, "PID_YAW_RATE_I", 0.0f);  // Yaw Rate Integral Gain | 0.0 | 1000.0
    init_param_float(PARAM_PID_YAW_RATE_D, "PID_YAW_RATE_D", 0.0f);  // Yaw Rate Derivative Gain | 0.0 | 1000.0
    init_param_float(PARAM_YAW_RATE_TRIM, "YAW_RATE_TRIM", 0.0f);  // Yaw Rate Trim - See RC calibration | -1000.0 | 1000.0
    init_param_float(PARAM_MAX_YAW_RATE, "MAX_YAW_RATE", 6.283f);   // Maximum Yaw Rate command accepted into PID controllers | 0.0 | 1000.0

    init_param_float(PARAM_PID_ROLL_ANGLE_P, "PID_ROLL_ANG_P", 0.15f);   // Roll Angle Proporitional Gain | 0.0 | 1000.0
    init_param_float(PARAM_PID_ROLL_ANGLE_I, "PID_ROLL_ANG_I", 0.0f);   // Roll Angle Integral Gain | 0.0 | 1000.0
    init_param_float(PARAM_PID_ROLL_ANGLE_D, "PID_ROLL_ANG_D", 0.07f);  // Roll Angle Derivative Gain | 0.0 | 1000.0
    init_param_float(PARAM_ROLL_ANGLE_TRIM, "ROLL_TRIM", 0.0f);  // Roll Angle Trim - See RC calibration | -1000.0 | 1000.0
    init_param_float(PARAM_MAX_ROLL_ANGLE, "MAX_ROLL_ANG", 0.786f);   // Maximum Roll Angle command accepted into PID controllers | 0.0 | 1000.0

    init_param_float(PARAM_PID_PITCH_ANGLE_P, "PID_PITCH_ANG_P", 0.15f);  // Pitch Angle Proporitional Gain | 0.0 | 1000.0 
    init_param_float(PARAM_PID_PITCH_ANGLE_I, "PID_PITCH_ANG_I", 0.0f);  // Pitch Angle Integral Gain | 0.0 | 1000.0
    init_param_float(PARAM_PID_PITCH_ANGLE_D, "PID_PITCH_ANG_D", 0.07f); // Pitch Angle Derivative Gain | 0.0 | 1000.0 
    init_param_float(PARAM_PITCH_ANGLE_TRIM, "PITCH_TRIM", 0.0f);  // Pitch Angle Trim - See RC calibration | -1000.0 | 1000.0
    init_param_float(PARAM_MAX_PITCH_ANGLE, "MAX_PITCH_ANG", 0.786f);   // Maximum Pitch Angle command accepted into PID controllers | 0.0 | 1000.0

    init_param_float(PARAM_PID_ALT_P, "PID_ALT_P", 0.0f); // Altitude Proporitional Gain | 0.0 | 1000.0 
    init_param_float(PARAM_PID_ALT_I, "PID_ALT_I", 0.0f); // Altitude Integral Gain | 0.0 | 1000.0 
    init_param_float(PARAM_PID_ALT_D, "PID_ALT_D", 0.0f); // Altitude Derivative Gain | 0.0 | 1000.0 
    init_param_float(PARAM_HOVER_THROTTLE, "HOVER_THR", 0.5); // Hover Throttle - See RC calibration | 0.0 | 1.0

    init_param_float(PARAM_PID_TAU, "PID_TAU", 0.05f); // Dirty Derivative time constant - See controller documentation | 0.0 | 1.0


    /*************************/
    /*** PWM CONFIGURATION ***/
    /*************************/
    init_param_int(PARAM_MOTOR_PWM_SEND_RATE, "MOTOR_PWM_UPDATE", 490); // Refresh rate of motor commands to motors - See motor documentation | 0 | 1000
    init_param_int(PARAM_MOTOR_IDLE_PWM, "MOTOR_IDLE_PWM", 1100); // Idle PWM sent to motors at zero throttle (Set above 1100 to spin when armed) | 1000 | 2000
    init_param_int(PARAM_SPIN_MOTORS_WHEN_ARMED, "ARM_SPIN_MOTORS", true); // Enforce MOTOR_IDLE_PWM | 0 | 1

    /*******************************/
    /*** ESTIMATOR CONFIGURATION ***/
    /*******************************/
    init_param_int(PARAM_INIT_TIME, "FILTER_INIT_T", 3000); // Time in ms to initialize estimator | 0 | 100000
    init_param_float(PARAM_FILTER_KP, "FILTER_KP", 1.0f); // estimator proportional gain - See estimator documentation | 0 | 10.0
    init_param_float(PARAM_FILTER_KI, "FILTER_KI", 0.1f); // estimator integral gain - See estimator documentation | 0 | 1.0

    init_param_float(PARAM_GYRO_ALPHA, "GYRO_LPF_ALPHA", 0.888f); // Low-pass filter constant - See estimator documentation | 0 | 1.0
    init_param_float(PARAM_ACC_ALPHA, "ACC_LPF_ALPHA", 0.888f); // Low-pass filter constant - See estimator documentation | 0 | 1.0

    init_param_float(PARAM_ACCEL_SCALE, "ACCEL_SCALE", 1.0f); // Scale factor to apply to IMU measurements - Read-Only | 0.5 | 2.0

    init_param_float(PARAM_GYRO_X_BIAS, "GYRO_X_BIAS", 0.0f); // Constant x-bias of gyroscope readings | -1.0 | 1.0
    init_param_float(PARAM_GYRO_Y_BIAS, "GYRO_Y_BIAS", 0.0f); // Constant y-bias of gyroscope readings | -1.0 | 1.0
    init_param_float(PARAM_GYRO_Z_BIAS, "GYRO_Z_BIAS", 0.0f); // Constant z-bias of gyroscope readings | -1.0 | 1.0
    init_param_float(PARAM_ACC_X_BIAS, "ACC_X_BIAS", 0.0f); // Constant x-bias of accelerometer readings | -2.0 | 2.0
    init_param_float(PARAM_ACC_Y_BIAS, "ACC_Y_BIAS", 0.0f); // Constant y-bias of accelerometer readings | -2.0 | 2.0
    init_param_float(PARAM_ACC_Z_BIAS, "ACC_Z_BIAS", 0.0f); // Constant z-bias of accelerometer readings | -2.0 | 2.0
    init_param_float(PARAM_ACC_X_TEMP_COMP, "ACC_X_TEMP_COMP", 0.0f); // Linear x-axis temperature compensation constant | -2.0 | 2.0
    init_param_float(PARAM_ACC_Y_TEMP_COMP, "ACC_Y_TEMP_COMP", 0.0f); // Linear y-axis temperature compensation constant | -2.0 | 2.0
    init_param_float(PARAM_ACC_Z_TEMP_COMP, "ACC_Z_TEMP_COMP", 0.0f); // Linear z-axis temperature compensation constant | -2.0 | 2.0

    /************************/
    /*** RC CONFIGURATION ***/
    /************************/
    init_param_int(PARAM_RC_TYPE, "RC_TYPE", 1); // Type of RC input 0 - Parallel PWM (PWM), 1 - Pulse-Position Modulation (PPM) | 0 | 1
    init_param_int(PARAM_RC_X_CHANNEL, "RC_X_CHN", 0); // RC input channel mapped to x-axis commands [0 - indexed] | 0 | 3 
    init_param_int(PARAM_RC_Y_CHANNEL, "RC_Y_CHN", 1); // RC input channel mapped to y-axis commands [0 - indexed] | 0 | 3
    init_param_int(PARAM_RC_Z_CHANNEL, "RC_Z_CHN", 3); // RC input channel mapped to z-axis commands [0 - indexed] | 0 | 3
    init_param_int(PARAM_RC_F_CHANNEL, "RC_F_CHN", 2); // RC input channel mapped to F-axis commands [0 - indexed] | 0 | 3
    init_param_int(PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL, "RC_ATT_OVRD_CHN", 4); // RC switch mapped to attitude override [0 -indexed] | 4 | 7
    init_param_int(PARAM_RC_THROTTLE_OVERRIDE_CHANNEL, "RC_THR_OVRD_CHN", 5); // RC switch hannel mapped to throttle override [0 -indexed] | 4 | 7
    init_param_int(PARAM_RC_ATT_CONTROL_TYPE_CHANNEL, "RC_ATT_CTRL_CHN", 6); // RC switch channel mapped to attitude control type [0 -indexed] | 4 | 7
    init_param_int(PARAM_RC_F_CONTROL_TYPE_CHANNEL, "RC_F_CTRL_CHN", 7); // RC switch channel mapped to throttle control type override [0 -indexed] | 4 | 7
    init_param_int(PARAM_RC_NUM_CHANNELS, "RC_NUM_CHN", 9); // number of RC input channels | 1 | 8

    init_param_int(PARAM_RC_X_CENTER, "RC_X_CENTER", 1500); // RC calibration x-axis center (us) | 1000 | 2000
    init_param_int(PARAM_RC_Y_CENTER, "RC_Y_CENTER", 1500); // RC calibration y-axis center (us) | 1000 | 2000
    init_param_int(PARAM_RC_Z_CENTER, "RC_Z_CENTER", 1500); // RC calibration z-axis center (us) | 1000 | 2000
    init_param_int(PARAM_RC_F_BOTTOM, "RC_F_BOTTOM", 1000); // RC calibration F-axis center (us) | 1000 | 2000
    init_param_int(PARAM_RC_X_RANGE, "RC_X_RANGE", 1000); // RC calibration x-axis range (us) | 500 | 2500
    init_param_int(PARAM_RC_Y_RANGE, "RC_Y_RANGE", 1000); // RC calibration y-axis range (us) | 500 | 2500
    init_param_int(PARAM_RC_Z_RANGE, "RC_Z_RANGE", 1000); // RC calibration z-axis range (us) | 500 | 2500
    init_param_int(PARAM_RC_F_RANGE, "RC_F_RANGE", 1000); // RC calibration F-axis range (us) | 500 | 2500
    init_param_int(PARAM_RC_SWITCH_5_DIRECTION, "SWITCH_5_DIR", 1); // RC switch 5 toggle direction | 0 | 1
    init_param_int(PARAM_RC_SWITCH_6_DIRECTION, "SWITCH_6_DIR", 1); // RC switch 6 toggle direction | 0 | 1
    init_param_int(PARAM_RC_SWITCH_7_DIRECTION, "SWITCH_7_DIR", 1); // RC switch 7 toggle direction | 0 | 1
    init_param_int(PARAM_RC_SWITCH_8_DIRECTION, "SWITCH_8_DIR", 1); // RC switch 8 toggle direction | 0 | 1

    init_param_int(PARAM_RC_OVERRIDE_DEVIATION, "RC_OVRD_DEV", 100); // RC stick deviation from center for overrride (us) | 0 | 1000
    init_param_int(PARAM_OVERRIDE_LAG_TIME, "OVRD_LAG_TIME", 1000); // RC stick deviation lag time before returning control (ms) | 0 | 100000
    init_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE, "MIN_THROTTLE", false); // Take minimum throttle between RC and computer at all times | 0 | 1

    init_param_float(PARAM_RC_MAX_ROLL, "RC_MAX_ROLL", 0.786f); // Maximum roll angle command sent by full deflection of RC sticks | 0.0 | 3.14159
    init_param_float(PARAM_RC_MAX_PITCH, "RC_MAX_PITCH", 0.786f); // Maximum pitch angle command sent by full stick deflection of RC sticks | 0.0 | 3.14159
    init_param_float(PARAM_RC_MAX_ROLLRATE, "RC_MAX_ROLLRATE", 3.14159f); // Maximum roll rate command sent by full stick deflection of RC sticks | 0.0 | 9.42477796077
    init_param_float(PARAM_RC_MAX_PITCHRATE, "RC_MAX_PITCHRATE", 3.14159f); // Maximum pitch command sent by full stick deflection of RC sticks | 0.0 | 3.14159
    init_param_float(PARAM_RC_MAX_YAWRATE, "RC_MAX_YAWRATE", 0.786f); // Maximum pitch command sent by full stick deflection of RC sticks | 0.0 | 3.14159

    /***************************/
    /*** FRAME CONFIGURATION ***/
    /***************************/
    init_param_int(PARAM_MIXER, "MIXER", 1); // Which mixer to choose, 1 = QUADCOPTER_X - See Mixer documentation | 0 | 5

    init_param_int(PARAM_FIXED_WING, "FIXED_WING", false); // switches on passthrough commands for fixedwing operation | 0 | 1
    init_param_int(PARAM_ELEVATOR_REVERSE, "ELEVATOR_REV", 0); // reverses elevator servo output | 0 | 1
    init_param_int(PARAM_AILERON_REVERSE, "AIL_REV", 0); // reverses aileron servo output | 0 | 1
    init_param_int(PARAM_RUDDER_REVERSE, "RUDDER_REV", 0); // reverses rudder servo output | 0 | 1

    /********************/
    /*** ARMING SETUP ***/
    /********************/
    init_param_int(PARAM_ARM_STICKS, "ARM_STICKS", false); // use RC sticks to arm vehicle (disables arm RC switch if enabled) | 0 | 1
    init_param_int(PARAM_ARM_CHANNEL, "ARM_CHANNEL", 4); // RC switch mapped to arm/disarm [0 -indexed] | 4 | 7
    init_param_int(PARAM_ARM_THRESHOLD, "ARM_THRESHOLD", 150); // RC deviation from max/min in yaw and throttle for arming and disarming check (us) | 0 | 500
}

bool Params::read_params(void)
{
    return board->read_params();
}

bool Params::write_params(void)
{
    return board->write_params(true);
}

void Params::param_change_callback(param_id_t id)
{
    switch (id)
    {
    case PARAM_SYSTEM_ID:
        comm_link->set_sys_id(get_param_int(PARAM_SYSTEM_ID));
        break;
    case PARAM_STREAM_HEARTBEAT_RATE:
        comm_link->set_streaming_rate(PARAM_STREAM_HEARTBEAT_RATE, get_param_int(PARAM_STREAM_HEARTBEAT_RATE));
        break;

    case PARAM_STREAM_ATTITUDE_RATE:
        comm_link->set_streaming_rate(PARAM_STREAM_ATTITUDE_RATE, get_param_int(PARAM_STREAM_ATTITUDE_RATE));
        break;

    case PARAM_STREAM_IMU_RATE:
        comm_link->set_streaming_rate(PARAM_STREAM_IMU_RATE, get_param_int(PARAM_STREAM_IMU_RATE));
        break;
    case PARAM_STREAM_AIRSPEED_RATE:
        comm_link->set_streaming_rate(PARAM_STREAM_AIRSPEED_RATE, get_param_int(PARAM_STREAM_AIRSPEED_RATE));
        break;
    case PARAM_STREAM_SONAR_RATE:
        comm_link->set_streaming_rate(PARAM_STREAM_SONAR_RATE, get_param_int(PARAM_STREAM_SONAR_RATE));
        break;
    case  PARAM_STREAM_BARO_RATE:
        comm_link->set_streaming_rate(PARAM_STREAM_BARO_RATE, get_param_int(PARAM_STREAM_BARO_RATE));
        break;
    case  PARAM_STREAM_MAG_RATE:
        comm_link->set_streaming_rate(PARAM_STREAM_MAG_RATE, get_param_int(PARAM_STREAM_MAG_RATE));
        break;

    case PARAM_STREAM_SERVO_OUTPUT_RAW_RATE:
        comm_link->set_streaming_rate(PARAM_STREAM_SERVO_OUTPUT_RAW_RATE, get_param_int(PARAM_STREAM_SERVO_OUTPUT_RAW_RATE));
        break;
    case PARAM_STREAM_RC_RAW_RATE:
        comm_link->set_streaming_rate(PARAM_STREAM_RC_RAW_RATE, get_param_int(PARAM_STREAM_RC_RAW_RATE));
        break;

    //information messages
    case PARAM_RC_TYPE:
        comm_link->log_message(CommonState::stringf("RC type = %i", get_param_int(PARAM_RC_TYPE)).c_str(), 0);
    case PARAM_MIXER:
        comm_link->log_message(CommonState::stringf("Mixer = %i", get_param_int(PARAM_MIXER)).c_str(), 0);
    case PARAM_FIXED_WING:
        comm_link->log_message(CommonState::stringf("Fixed wing = %i", get_param_int(PARAM_FIXED_WING)).c_str(), 0);
    case PARAM_ARM_STICKS:
        comm_link->log_message(CommonState::stringf("Can use RC to arm = %i", get_param_int(PARAM_ARM_STICKS)).c_str(), 0);


        //TODO: need better design so components can listen to their param changes
        //case PARAM_RC_TYPE:
        //    mixer->init_PWM();
        //    break;
        //case PARAM_MOTOR_PWM_SEND_RATE:
        //    mixer->init_PWM();
        //    break;
        //case PARAM_MIXER:
        //    mixer->init(common_state);
        //    break;

    default:
        // no action needed for this parameter
        break;
    }
}

Params::param_id_t Params::lookup_param_id(const char name[PARAMS_NAME_LENGTH])
{
    for (uint16_t id = 0; id < PARAMS_COUNT; id++)
    {
        bool match = true;
        for (uint8_t i = 0; i < PARAMS_NAME_LENGTH; i++)
        {
            // compare each character
            if (name[i] != _params.names[id][i])
            {
                match = false;
                break;
            }

            // stop comparing if end of string is reached
            if (_params.names[id][i] == '\0')
                break;
        }

        if (match)
            return (param_id_t)id;
    }

    return PARAMS_COUNT;
}

int Params::get_param_int(param_id_t id)
{
    return _params.values[id];
}

float Params::get_param_float(param_id_t id)
{
    return *(float *)&_params.values[id];
}

char * Params::get_param_name(param_id_t id)
{
    return _params.names[id];
}

Params::param_type_t Params::get_param_type(param_id_t id)
{
    return _params.types[id];
}

bool Params::set_param_int(param_id_t id, int32_t value)
{
    if (id < PARAMS_COUNT && value != _params.values[id])
    {
        _params.values[id] = value;
        param_change_callback(id);
        comm_link->notify_param_change(id, value);
        return true;
    }
    return false;
}

bool Params::set_param_float(param_id_t id, float value)
{
    return set_param_int(id, *(int32_t *)&value);
}

bool Params::set_param_by_name_int(const char name[PARAMS_NAME_LENGTH], int32_t value)
{
    param_id_t id = lookup_param_id(name);
    return set_param_int(id, value);
}

bool Params::set_param_by_name_float(const char name[PARAMS_NAME_LENGTH], float value)
{
    return set_param_by_name_int(name, *(int32_t *)&value);
}


} //namespace