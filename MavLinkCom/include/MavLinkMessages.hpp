// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#ifndef MavLinkCom_MavLinkMessages_hpp
#define MavLinkCom_MavLinkMessages_hpp

#include <stdint.h>
#include <string>
#include "MavLinkMessageBase.hpp"

namespace mavlinkcom
{

enum class MavLinkMessageIds {
    MAVLINK_MSG_ID_HEARTBEAT = 0,
    MAVLINK_MSG_ID_SYS_STATUS = 1,
    MAVLINK_MSG_ID_SYSTEM_TIME = 2,
    MAVLINK_MSG_ID_PING = 4,
    MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL = 5,
    MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK = 6,
    MAVLINK_MSG_ID_AUTH_KEY = 7,
    MAVLINK_MSG_ID_SET_MODE = 11,
    MAVLINK_MSG_ID_PARAM_REQUEST_READ = 20,
    MAVLINK_MSG_ID_PARAM_REQUEST_LIST = 21,
    MAVLINK_MSG_ID_PARAM_VALUE = 22,
    MAVLINK_MSG_ID_PARAM_SET = 23,
    MAVLINK_MSG_ID_GPS_RAW_INT = 24,
    MAVLINK_MSG_ID_GPS_STATUS = 25,
    MAVLINK_MSG_ID_SCALED_IMU = 26,
    MAVLINK_MSG_ID_RAW_IMU = 27,
    MAVLINK_MSG_ID_RAW_PRESSURE = 28,
    MAVLINK_MSG_ID_SCALED_PRESSURE = 29,
    MAVLINK_MSG_ID_ATTITUDE = 30,
    MAVLINK_MSG_ID_ATTITUDE_QUATERNION = 31,
    MAVLINK_MSG_ID_LOCAL_POSITION_NED = 32,
    MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 33,
    MAVLINK_MSG_ID_RC_CHANNELS_SCALED = 34,
    MAVLINK_MSG_ID_RC_CHANNELS_RAW = 35,
    MAVLINK_MSG_ID_SERVO_OUTPUT_RAW = 36,
    MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST = 37,
    MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST = 38,
    MAVLINK_MSG_ID_MISSION_ITEM = 39,
    MAVLINK_MSG_ID_MISSION_REQUEST = 40,
    MAVLINK_MSG_ID_MISSION_SET_CURRENT = 41,
    MAVLINK_MSG_ID_MISSION_CURRENT = 42,
    MAVLINK_MSG_ID_MISSION_REQUEST_LIST = 43,
    MAVLINK_MSG_ID_MISSION_COUNT = 44,
    MAVLINK_MSG_ID_MISSION_CLEAR_ALL = 45,
    MAVLINK_MSG_ID_MISSION_ITEM_REACHED = 46,
    MAVLINK_MSG_ID_MISSION_ACK = 47,
    MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN = 48,
    MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN = 49,
    MAVLINK_MSG_ID_PARAM_MAP_RC = 50,
    MAVLINK_MSG_ID_MISSION_REQUEST_INT = 51,
    MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA = 54,
    MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA = 55,
    MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV = 61,
    MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT = 62,
    MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV = 63,
    MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV = 64,
    MAVLINK_MSG_ID_RC_CHANNELS = 65,
    MAVLINK_MSG_ID_REQUEST_DATA_STREAM = 66,
    MAVLINK_MSG_ID_DATA_STREAM = 67,
    MAVLINK_MSG_ID_MANUAL_CONTROL = 69,
    MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE = 70,
    MAVLINK_MSG_ID_MISSION_ITEM_INT = 73,
    MAVLINK_MSG_ID_VFR_HUD = 74,
    MAVLINK_MSG_ID_COMMAND_INT = 75,
    MAVLINK_MSG_ID_COMMAND_LONG = 76,
    MAVLINK_MSG_ID_COMMAND_ACK = 77,
    MAVLINK_MSG_ID_MANUAL_SETPOINT = 81,
    MAVLINK_MSG_ID_SET_ATTITUDE_TARGET = 82,
    MAVLINK_MSG_ID_ATTITUDE_TARGET = 83,
    MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED = 84,
    MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED = 85,
    MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT = 86,
    MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT = 87,
    MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET = 89,
    MAVLINK_MSG_ID_HIL_STATE = 90,
    MAVLINK_MSG_ID_HIL_CONTROLS = 91,
    MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW = 92,
    MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS = 93,
    MAVLINK_MSG_ID_OPTICAL_FLOW = 100,
    MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE = 101,
    MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE = 102,
    MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE = 103,
    MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE = 104,
    MAVLINK_MSG_ID_HIGHRES_IMU = 105,
    MAVLINK_MSG_ID_OPTICAL_FLOW_RAD = 106,
    MAVLINK_MSG_ID_HIL_SENSOR = 107,
    MAVLINK_MSG_ID_SIM_STATE = 108,
    MAVLINK_MSG_ID_RADIO_STATUS = 109,
    MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL = 110,
    MAVLINK_MSG_ID_TIMESYNC = 111,
    MAVLINK_MSG_ID_CAMERA_TRIGGER = 112,
    MAVLINK_MSG_ID_HIL_GPS = 113,
    MAVLINK_MSG_ID_HIL_OPTICAL_FLOW = 114,
    MAVLINK_MSG_ID_HIL_STATE_QUATERNION = 115,
    MAVLINK_MSG_ID_SCALED_IMU2 = 116,
    MAVLINK_MSG_ID_LOG_REQUEST_LIST = 117,
    MAVLINK_MSG_ID_LOG_ENTRY = 118,
    MAVLINK_MSG_ID_LOG_REQUEST_DATA = 119,
    MAVLINK_MSG_ID_LOG_DATA = 120,
    MAVLINK_MSG_ID_LOG_ERASE = 121,
    MAVLINK_MSG_ID_LOG_REQUEST_END = 122,
    MAVLINK_MSG_ID_GPS_INJECT_DATA = 123,
    MAVLINK_MSG_ID_GPS2_RAW = 124,
    MAVLINK_MSG_ID_POWER_STATUS = 125,
    MAVLINK_MSG_ID_SERIAL_CONTROL = 126,
    MAVLINK_MSG_ID_GPS_RTK = 127,
    MAVLINK_MSG_ID_GPS2_RTK = 128,
    MAVLINK_MSG_ID_SCALED_IMU3 = 129,
    MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE = 130,
    MAVLINK_MSG_ID_ENCAPSULATED_DATA = 131,
    MAVLINK_MSG_ID_DISTANCE_SENSOR = 132,
    MAVLINK_MSG_ID_TERRAIN_REQUEST = 133,
    MAVLINK_MSG_ID_TERRAIN_DATA = 134,
    MAVLINK_MSG_ID_TERRAIN_CHECK = 135,
    MAVLINK_MSG_ID_TERRAIN_REPORT = 136,
    MAVLINK_MSG_ID_SCALED_PRESSURE2 = 137,
    MAVLINK_MSG_ID_ATT_POS_MOCAP = 138,
    MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET = 139,
    MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET = 140,
    MAVLINK_MSG_ID_ALTITUDE = 141,
    MAVLINK_MSG_ID_RESOURCE_REQUEST = 142,
    MAVLINK_MSG_ID_SCALED_PRESSURE3 = 143,
    MAVLINK_MSG_ID_FOLLOW_TARGET = 144,
    MAVLINK_MSG_ID_CONTROL_SYSTEM_STATE = 146,
    MAVLINK_MSG_ID_BATTERY_STATUS = 147,
    MAVLINK_MSG_ID_AUTOPILOT_VERSION = 148,
    MAVLINK_MSG_ID_LANDING_TARGET = 149,
    MAVLINK_MSG_ID_ESTIMATOR_STATUS = 230,
    MAVLINK_MSG_ID_WIND_COV = 231,
    MAVLINK_MSG_ID_GPS_INPUT = 232,
    MAVLINK_MSG_ID_GPS_RTCM_DATA = 233,
    MAVLINK_MSG_ID_HIGH_LATENCY = 234,
    MAVLINK_MSG_ID_VIBRATION = 241,
    MAVLINK_MSG_ID_HOME_POSITION = 242,
    MAVLINK_MSG_ID_SET_HOME_POSITION = 243,
    MAVLINK_MSG_ID_MESSAGE_INTERVAL = 244,
    MAVLINK_MSG_ID_EXTENDED_SYS_STATE = 245,
    MAVLINK_MSG_ID_ADSB_VEHICLE = 246,
    MAVLINK_MSG_ID_COLLISION = 247,
    MAVLINK_MSG_ID_V2_EXTENSION = 248,
    MAVLINK_MSG_ID_MEMORY_VECT = 249,
    MAVLINK_MSG_ID_DEBUG_VECT = 250,
    MAVLINK_MSG_ID_NAMED_VALUE_FLOAT = 251,
    MAVLINK_MSG_ID_NAMED_VALUE_INT = 252,
    MAVLINK_MSG_ID_STATUSTEXT = 253,
    MAVLINK_MSG_ID_DEBUG = 254,
    MAVLINK_MSG_ID_PROTOCOL_VERSION = 300
};
// Micro air vehicle / autopilot classes. This identifies the individual model.
enum class MAV_AUTOPILOT {
    // Generic autopilot, full support for everything
    MAV_AUTOPILOT_GENERIC = 0,
    // Reserved for future use.
    MAV_AUTOPILOT_RESERVED = 1,
    // SLUGS autopilot, http://slugsuav.soe.ucsc.edu
    MAV_AUTOPILOT_SLUGS = 2,
    // ArduPilotMega / ArduCopter, http://diydrones.com
    MAV_AUTOPILOT_ARDUPILOTMEGA = 3,
    // OpenPilot, http://openpilot.org
    MAV_AUTOPILOT_OPENPILOT = 4,
    // Generic autopilot only supporting simple waypoints
    MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY = 5,
    // Generic autopilot supporting waypoints and other simple navigation commands
    MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6,
    // Generic autopilot supporting the full mission command set
    MAV_AUTOPILOT_GENERIC_MISSION_FULL = 7,
    // No valid autopilot, e.g. a GCS or other MAVLink component
    MAV_AUTOPILOT_INVALID = 8,
    // PPZ UAV - http://nongnu.org/paparazzi
    MAV_AUTOPILOT_PPZ = 9,
    // UAV Dev Board
    MAV_AUTOPILOT_UDB = 10,
    // FlexiPilot
    MAV_AUTOPILOT_FP = 11,
    // PX4 Autopilot - http://pixhawk.ethz.ch/px4/
    MAV_AUTOPILOT_PX4 = 12,
    // SMACCMPilot - http://smaccmpilot.org
    MAV_AUTOPILOT_SMACCMPILOT = 13,
    // AutoQuad -- http://autoquad.org
    MAV_AUTOPILOT_AUTOQUAD = 14,
    // Armazila -- http://armazila.com
    MAV_AUTOPILOT_ARMAZILA = 15,
    // Aerob -- http://aerob.ru
    MAV_AUTOPILOT_AEROB = 16,
    // ASLUAV autopilot -- http://www.asl.ethz.ch
    MAV_AUTOPILOT_ASLUAV = 17
};

enum class MAV_TYPE {
    // Generic micro air vehicle.
    MAV_TYPE_GENERIC = 0,
    // Fixed wing aircraft.
    MAV_TYPE_FIXED_WING = 1,
    // Quadrotor
    MAV_TYPE_QUADROTOR = 2,
    // Coaxial helicopter
    MAV_TYPE_COAXIAL = 3,
    // Normal helicopter with tail rotor.
    MAV_TYPE_HELICOPTER = 4,
    // Ground installation
    MAV_TYPE_ANTENNA_TRACKER = 5,
    // Operator control unit / ground control station
    MAV_TYPE_GCS = 6,
    // Airship, controlled
    MAV_TYPE_AIRSHIP = 7,
    // Free balloon, uncontrolled
    MAV_TYPE_FREE_BALLOON = 8,
    // Rocket
    MAV_TYPE_ROCKET = 9,
    // Ground rover
    MAV_TYPE_GROUND_ROVER = 10,
    // Surface vessel, boat, ship
    MAV_TYPE_SURFACE_BOAT = 11,
    // Submarine
    MAV_TYPE_SUBMARINE = 12,
    // Hexarotor
    MAV_TYPE_HEXAROTOR = 13,
    // Octorotor
    MAV_TYPE_OCTOROTOR = 14,
    // Tricopter
    MAV_TYPE_TRICOPTER = 15,
    // Flapping wing
    MAV_TYPE_FLAPPING_WING = 16,
    // Kite
    MAV_TYPE_KITE = 17,
    // Onboard companion controller
    MAV_TYPE_ONBOARD_CONTROLLER = 18,
    // Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter.
    MAV_TYPE_VTOL_DUOROTOR = 19,
    // Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter.
    MAV_TYPE_VTOL_QUADROTOR = 20,
    // Tiltrotor VTOL
    MAV_TYPE_VTOL_TILTROTOR = 21,
    // VTOL reserved 2
    MAV_TYPE_VTOL_RESERVED2 = 22,
    // VTOL reserved 3
    MAV_TYPE_VTOL_RESERVED3 = 23,
    // VTOL reserved 4
    MAV_TYPE_VTOL_RESERVED4 = 24,
    // VTOL reserved 5
    MAV_TYPE_VTOL_RESERVED5 = 25,
    // Onboard gimbal
    MAV_TYPE_GIMBAL = 26,
    // Onboard ADSB peripheral
    MAV_TYPE_ADSB = 27
};

// These values define the type of firmware release. These values indicate the first
// version or release of this type. For example the first alpha release would be 64,
// the second would be 65.
enum class FIRMWARE_VERSION_TYPE {
    // development release
    FIRMWARE_VERSION_TYPE_DEV = 0,
    // alpha release
    FIRMWARE_VERSION_TYPE_ALPHA = 64,
    // beta release
    FIRMWARE_VERSION_TYPE_BETA = 128,
    // release candidate
    FIRMWARE_VERSION_TYPE_RC = 192,
    // official stable release
    FIRMWARE_VERSION_TYPE_OFFICIAL = 255
};

// These flags encode the MAV mode.
enum class MAV_MODE_FLAG {
    // 0b10000000 MAV safety set to armed. Motors are enabled / running / can start.
    // Ready to fly. Additional note: this flag is to be ignore when sent in the command
    // MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM shall be used instead.
    // The flag can still be used to report the armed state.
    MAV_MODE_FLAG_SAFETY_ARMED = 128,
    // 0b01000000 remote control input is enabled.
    MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64,
    // 0b00100000 hardware in the loop simulation. All motors / actuators are blocked,
    // but internal software is full operational.
    MAV_MODE_FLAG_HIL_ENABLED = 32,
    // 0b00010000 system stabilizes electronically its attitude (and optionally position).
    // It needs however further control inputs to move around.
    MAV_MODE_FLAG_STABILIZE_ENABLED = 16,
    // 0b00001000 guided mode enabled, system flies MISSIONs / mission items.
    MAV_MODE_FLAG_GUIDED_ENABLED = 8,
    // 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided
    // flag can be set or not, depends on the actual implementation.
    MAV_MODE_FLAG_AUTO_ENABLED = 4,
    // 0b00000010 system has a test mode enabled. This flag is intended for temporary
    // system tests and should not be used for stable implementations.
    MAV_MODE_FLAG_TEST_ENABLED = 2,
    // 0b00000001 Reserved for future use.
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
};

// These values encode the bit positions of the decode position. These values can
// be used to read the value of a flag bit by combining the base_mode variable with
// AND with the flag position value. The result will be either 0 or 1, depending on
// if the flag is set or not.
enum class MAV_MODE_FLAG_DECODE_POSITION {
    // First bit: 10000000
    MAV_MODE_FLAG_DECODE_POSITION_SAFETY = 128,
    // Second bit: 01000000
    MAV_MODE_FLAG_DECODE_POSITION_MANUAL = 64,
    // Third bit: 00100000
    MAV_MODE_FLAG_DECODE_POSITION_HIL = 32,
    // Fourth bit: 00010000
    MAV_MODE_FLAG_DECODE_POSITION_STABILIZE = 16,
    // Fifth bit: 00001000
    MAV_MODE_FLAG_DECODE_POSITION_GUIDED = 8,
    // Sixt bit: 00000100
    MAV_MODE_FLAG_DECODE_POSITION_AUTO = 4,
    // Seventh bit: 00000010
    MAV_MODE_FLAG_DECODE_POSITION_TEST = 2,
    // Eighth bit: 00000001
    MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE = 1
};

// Override command, pauses current mission execution and moves immediately to a position
enum class MAV_GOTO {
    // Hold at the current position.
    MAV_GOTO_DO_HOLD = 0,
    // Continue with the next item in mission execution.
    MAV_GOTO_DO_CONTINUE = 1,
    // Hold at the current position of the system
    MAV_GOTO_HOLD_AT_CURRENT_POSITION = 2,
    // Hold at the position specified in the parameters of the DO_HOLD action
    MAV_GOTO_HOLD_AT_SPECIFIED_POSITION = 3
};

// These defines are predefined OR-combined mode flags. There is no need to use values
// from this enum, but it simplifies the use of the mode flags. Note that manual input
// is enabled in all modes as a safety override.
enum class MAV_MODE {
    // System is not ready to fly, booting, calibrating, etc. No flag is set.
    MAV_MODE_PREFLIGHT = 0,
    // System is allowed to be active, under assisted RC control.
    MAV_MODE_STABILIZE_DISARMED = 80,
    // System is allowed to be active, under assisted RC control.
    MAV_MODE_STABILIZE_ARMED = 208,
    // System is allowed to be active, under manual (RC) control, no stabilization
    MAV_MODE_MANUAL_DISARMED = 64,
    // System is allowed to be active, under manual (RC) control, no stabilization
    MAV_MODE_MANUAL_ARMED = 192,
    // System is allowed to be active, under autonomous control, manual setpoint
    MAV_MODE_GUIDED_DISARMED = 88,
    // System is allowed to be active, under autonomous control, manual setpoint
    MAV_MODE_GUIDED_ARMED = 216,
    // System is allowed to be active, under autonomous control and navigation (the
    // trajectory is decided onboard and not pre-programmed by MISSIONs)
    MAV_MODE_AUTO_DISARMED = 92,
    // System is allowed to be active, under autonomous control and navigation (the
    // trajectory is decided onboard and not pre-programmed by MISSIONs)
    MAV_MODE_AUTO_ARMED = 220,
    // UNDEFINED mode. This solely depends on the autopilot - use with caution, intended
    // for developers only.
    MAV_MODE_TEST_DISARMED = 66,
    // UNDEFINED mode. This solely depends on the autopilot - use with caution, intended
    // for developers only.
    MAV_MODE_TEST_ARMED = 194
};

enum class MAV_STATE {
    // Uninitialized system, state is unknown.
    MAV_STATE_UNINIT = 0,
    // System is booting up.
    MAV_STATE_BOOT,
    // System is calibrating and not flight-ready.
    MAV_STATE_CALIBRATING,
    // System is grounded and on standby. It can be launched any time.
    MAV_STATE_STANDBY,
    // System is active and might be already airborne. Motors are engaged.
    MAV_STATE_ACTIVE,
    // System is in a non-normal flight mode. It can however still navigate.
    MAV_STATE_CRITICAL,
    // System is in a non-normal flight mode. It lost control over parts or over the
    // whole airframe. It is in mayday and going down.
    MAV_STATE_EMERGENCY,
    // System just initialized its power-down sequence, will shut down now.
    MAV_STATE_POWEROFF
};

enum class MAV_COMPONENT {
    MAV_COMP_ID_ALL = 0,
    MAV_COMP_ID_GPS = 220,
    MAV_COMP_ID_MISSIONPLANNER = 190,
    MAV_COMP_ID_PATHPLANNER = 195,
    MAV_COMP_ID_MAPPER = 180,
    MAV_COMP_ID_CAMERA = 100,
    MAV_COMP_ID_IMU = 200,
    MAV_COMP_ID_IMU_2 = 201,
    MAV_COMP_ID_IMU_3 = 202,
    MAV_COMP_ID_UDP_BRIDGE = 240,
    MAV_COMP_ID_UART_BRIDGE = 241,
    MAV_COMP_ID_SYSTEM_CONTROL = 250,
    MAV_COMP_ID_SERVO1 = 140,
    MAV_COMP_ID_SERVO2 = 141,
    MAV_COMP_ID_SERVO3 = 142,
    MAV_COMP_ID_SERVO4 = 143,
    MAV_COMP_ID_SERVO5 = 144,
    MAV_COMP_ID_SERVO6 = 145,
    MAV_COMP_ID_SERVO7 = 146,
    MAV_COMP_ID_SERVO8 = 147,
    MAV_COMP_ID_SERVO9 = 148,
    MAV_COMP_ID_SERVO10 = 149,
    MAV_COMP_ID_SERVO11 = 150,
    MAV_COMP_ID_SERVO12 = 151,
    MAV_COMP_ID_SERVO13 = 152,
    MAV_COMP_ID_SERVO14 = 153,
    MAV_COMP_ID_GIMBAL = 154,
    MAV_COMP_ID_LOG = 155,
    MAV_COMP_ID_ADSB = 156,
    // On Screen Display (OSD) devices for video links
    MAV_COMP_ID_OSD = 157,
    // Generic autopilot peripheral component ID. Meant for devices that do not implement
    // the parameter sub-protocol
    MAV_COMP_ID_PERIPHERAL = 158,
    MAV_COMP_ID_QX1_GIMBAL = 159
};

// These encode the sensors whose status is sent as part of the SYS_STATUS message.
enum class MAV_SYS_STATUS_SENSOR {
    // 0x01 3D gyro
    MAV_SYS_STATUS_SENSOR_3D_GYRO = 1,
    // 0x02 3D accelerometer
    MAV_SYS_STATUS_SENSOR_3D_ACCEL = 2,
    // 0x04 3D magnetometer
    MAV_SYS_STATUS_SENSOR_3D_MAG = 4,
    // 0x08 absolute pressure
    MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE = 8,
    // 0x10 differential pressure
    MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE = 16,
    // 0x20 GPS
    MAV_SYS_STATUS_SENSOR_GPS = 32,
    // 0x40 optical flow
    MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW = 64,
    // 0x80 computer vision position
    MAV_SYS_STATUS_SENSOR_VISION_POSITION = 128,
    // 0x100 laser based position
    MAV_SYS_STATUS_SENSOR_LASER_POSITION = 256,
    // 0x200 external ground truth (Vicon or Leica)
    MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH = 512,
    // 0x400 3D angular rate control
    MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL = 1024,
    // 0x800 attitude stabilization
    MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION = 2048,
    // 0x1000 yaw position
    MAV_SYS_STATUS_SENSOR_YAW_POSITION = 4096,
    // 0x2000 z/altitude control
    MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL = 8192,
    // 0x4000 x/y position control
    MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL = 16384,
    // 0x8000 motor outputs / control
    MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS = 32768,
    // 0x10000 rc receiver
    MAV_SYS_STATUS_SENSOR_RC_RECEIVER = 65536,
    // 0x20000 2nd 3D gyro
    MAV_SYS_STATUS_SENSOR_3D_GYRO2 = 131072,
    // 0x40000 2nd 3D accelerometer
    MAV_SYS_STATUS_SENSOR_3D_ACCEL2 = 262144,
    // 0x80000 2nd 3D magnetometer
    MAV_SYS_STATUS_SENSOR_3D_MAG2 = 524288,
    // 0x100000 geofence
    MAV_SYS_STATUS_GEOFENCE = 1048576,
    // 0x200000 AHRS subsystem health
    MAV_SYS_STATUS_AHRS = 2097152,
    // 0x400000 Terrain subsystem health
    MAV_SYS_STATUS_TERRAIN = 4194304,
    // 0x800000 Motors are reversed
    MAV_SYS_STATUS_REVERSE_MOTOR = 8388608,
    // 0x1000000 Logging
    MAV_SYS_STATUS_LOGGING = 16777216
};

enum class MAV_FRAME {
    // Global coordinate frame, WGS84 coordinate system. First value / x: latitude,
    // second value / y: longitude, third value / z: positive altitude over mean sea
    // level (MSL)
    MAV_FRAME_GLOBAL = 0,
    // Local coordinate frame, Z-up (x: north, y: east, z: down).
    MAV_FRAME_LOCAL_NED = 1,
    // NOT a coordinate frame, indicates a mission command.
    MAV_FRAME_MISSION = 2,
    // Global coordinate frame, WGS84 coordinate system, relative altitude over ground
    // with respect to the home position. First value / x: latitude, second value
    // / y: longitude, third value / z: positive altitude with 0 being at the altitude
    // of the home location.
    MAV_FRAME_GLOBAL_RELATIVE_ALT = 3,
    // Local coordinate frame, Z-down (x: east, y: north, z: up)
    MAV_FRAME_LOCAL_ENU = 4,
    // Global coordinate frame, WGS84 coordinate system. First value / x: latitude
    // in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value
    // / z: positive altitude over mean sea level (MSL)
    MAV_FRAME_GLOBAL_INT = 5,
    // Global coordinate frame, WGS84 coordinate system, relative altitude over ground
    // with respect to the home position. First value / x: latitude in degrees*10e-7,
    // second value / y: longitude in degrees*10e-7, third value / z: positive altitude
    // with 0 being at the altitude of the home location.
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6,
    // Offset to the current local frame. Anything expressed in this frame should
    // be added to the current local frame position.
    MAV_FRAME_LOCAL_OFFSET_NED = 7,
    // Setpoint in body NED frame. This makes sense if all position control is externalized
    // - e.g. useful to command 2 m/s^2 acceleration to the right.
    MAV_FRAME_BODY_NED = 8,
    // Offset in body NED frame. This makes sense if adding setpoints to the current
    // flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration
    // to the east.
    MAV_FRAME_BODY_OFFSET_NED = 9,
    // Global coordinate frame with above terrain level altitude. WGS84 coordinate
    // system, relative altitude over terrain with respect to the waypoint coordinate.
    // First value / x: latitude in degrees, second value / y: longitude in degrees,
    // third value / z: positive altitude in meters with 0 being at ground level in
    // terrain model.
    MAV_FRAME_GLOBAL_TERRAIN_ALT = 10,
    // Global coordinate frame with above terrain level altitude. WGS84 coordinate
    // system, relative altitude over terrain with respect to the waypoint coordinate.
    // First value / x: latitude in degrees*10e-7, second value / y: longitude in
    // degrees*10e-7, third value / z: positive altitude in meters with 0 being at
    // ground level in terrain model.
    MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
};

enum class MAVLINK_DATA_STREAM_TYPE {
    MAVLINK_DATA_STREAM_IMG_JPEG = 0,
    MAVLINK_DATA_STREAM_IMG_BMP,
    MAVLINK_DATA_STREAM_IMG_RAW8U,
    MAVLINK_DATA_STREAM_IMG_RAW32U,
    MAVLINK_DATA_STREAM_IMG_PGM,
    MAVLINK_DATA_STREAM_IMG_PNG
};

enum class FENCE_ACTION {
    // Disable fenced mode
    FENCE_ACTION_NONE = 0,
    // Switched to guided mode to return point (fence point 0)
    FENCE_ACTION_GUIDED = 1,
    // Report fence breach, but don't take action
    FENCE_ACTION_REPORT = 2,
    // Switched to guided mode to return point (fence point 0) with manual throttle
    // control
    FENCE_ACTION_GUIDED_THR_PASS = 3,
    // Switch to RTL (return to launch) mode and head for the return point.
    FENCE_ACTION_RTL = 4
};

enum class FENCE_BREACH {
    // No last fence breach
    FENCE_BREACH_NONE = 0,
    // Breached minimum altitude
    FENCE_BREACH_MINALT = 1,
    // Breached maximum altitude
    FENCE_BREACH_MAXALT = 2,
    // Breached fence boundary
    FENCE_BREACH_BOUNDARY = 3
};

// Enumeration of possible mount operation modes
enum class MAV_MOUNT_MODE {
    // Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization
    MAV_MOUNT_MODE_RETRACT = 0,
    // Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory.
    MAV_MOUNT_MODE_NEUTRAL = 1,
    // Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization
    MAV_MOUNT_MODE_MAVLINK_TARGETING = 2,
    // Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
    MAV_MOUNT_MODE_RC_TARGETING = 3,
    // Load neutral position and start to point to Lat,Lon,Alt
    MAV_MOUNT_MODE_GPS_POINT = 4
};

// Commands to be executed by the MAV. They can be executed on user request, or as
// part of a mission script. If the action is used in a mission, the parameter mapping
// to the waypoint/mission message is as follows: Param 1, Param 2, Param 3, Param
// 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what ARINC 424
// is for commercial aircraft: A data format how to interpret waypoint/mission data.
enum class MAV_CMD {
    // Navigate to MISSION.
    MAV_CMD_NAV_WAYPOINT = 16,
    // Loiter around this MISSION an unlimited amount of time
    MAV_CMD_NAV_LOITER_UNLIM = 17,
    // Loiter around this MISSION for X turns
    MAV_CMD_NAV_LOITER_TURNS = 18,
    // Loiter around this MISSION for X seconds
    MAV_CMD_NAV_LOITER_TIME = 19,
    // Return to launch location
    MAV_CMD_NAV_RETURN_TO_LAUNCH = 20,
    // Land at location
    MAV_CMD_NAV_LAND = 21,
    // Takeoff from ground / hand
    MAV_CMD_NAV_TAKEOFF = 22,
    // Land at local position (local frame only)
    MAV_CMD_NAV_LAND_LOCAL = 23,
    // Takeoff from local position (local frame only)
    MAV_CMD_NAV_TAKEOFF_LOCAL = 24,
    // Vehicle following, i.e. this waypoint represents the position of a moving vehicle
    MAV_CMD_NAV_FOLLOW = 25,
    // Continue on the current course and climb/descend to specified altitude. When
    // the altitude is reached continue to the next command (i.e., don't proceed to
    // the next command until the desired altitude is reached.
    MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT = 30,
    // Begin loiter at the specified Latitude and Longitude. If Lat=Lon=0, then loiter
    // at the current position. Don't consider the navigation command complete (don't
    // leave loiter) until the altitude has been reached. Additionally, if the Heading
    // Required parameter is non-zero the aircraft will not leave the loiter until
    // heading toward the next waypoint.
    MAV_CMD_NAV_LOITER_TO_ALT = 31,
    // Being following a target
    MAV_CMD_DO_FOLLOW = 32,
    // Reposition the MAV after a follow target command has been sent
    MAV_CMD_DO_FOLLOW_REPOSITION = 33,
    // Sets the region of interest (ROI) for a sensor set or the vehicle itself. This
    // can then be used by the vehicles control system to control the vehicle attitude
    // and the attitude of various sensors such as cameras.
    MAV_CMD_NAV_ROI = 80,
    // Control autonomous path planning on the MAV.
    MAV_CMD_NAV_PATHPLANNING = 81,
    // Navigate to MISSION using a spline path.
    MAV_CMD_NAV_SPLINE_WAYPOINT = 82,
    // Takeoff from ground using VTOL mode
    MAV_CMD_NAV_VTOL_TAKEOFF = 84,
    // Land using VTOL mode
    MAV_CMD_NAV_VTOL_LAND = 85,
    // hand control over to an external controller
    MAV_CMD_NAV_GUIDED_ENABLE = 92,
    // Delay the next navigation command a number of seconds or until a specified
    // time
    MAV_CMD_NAV_DELAY = 93,
    // NOP - This command is only used to mark the upper limit of the NAV/ACTION commands
    // in the enumeration
    MAV_CMD_NAV_LAST = 95,
    // Delay mission state machine.
    MAV_CMD_CONDITION_DELAY = 112,
    // Ascend/descend at rate. Delay mission state machine until desired altitude
    // reached.
    MAV_CMD_CONDITION_CHANGE_ALT = 113,
    // Delay mission state machine until within desired distance of next NAV point.
    MAV_CMD_CONDITION_DISTANCE = 114,
    // Reach a certain target angle.
    MAV_CMD_CONDITION_YAW = 115,
    // NOP - This command is only used to mark the upper limit of the CONDITION commands
    // in the enumeration
    MAV_CMD_CONDITION_LAST = 159,
    // Set system mode.
    MAV_CMD_DO_SET_MODE = 176,
    // Jump to the desired command in the mission list. Repeat this action only the
    // specified number of times
    MAV_CMD_DO_JUMP = 177,
    // Change speed and/or throttle set points.
    MAV_CMD_DO_CHANGE_SPEED = 178,
    // Changes the home location either to the current location or a specified location.
    MAV_CMD_DO_SET_HOME = 179,
    // Set a system parameter. Caution! Use of this command requires knowledge of
    // the numeric enumeration value of the parameter.
    MAV_CMD_DO_SET_PARAMETER = 180,
    // Set a relay to a condition.
    MAV_CMD_DO_SET_RELAY = 181,
    // Cycle a relay on and off for a desired number of cyles with a desired period.
    MAV_CMD_DO_REPEAT_RELAY = 182,
    // Set a servo to a desired PWM value.
    MAV_CMD_DO_SET_SERVO = 183,
    // Cycle a between its nominal setting and a desired PWM for a desired number
    // of cycles with a desired period.
    MAV_CMD_DO_REPEAT_SERVO = 184,
    // Terminate flight immediately
    MAV_CMD_DO_FLIGHTTERMINATION = 185,
    // Change altitude set point.
    MAV_CMD_DO_CHANGE_ALTITUDE = 186,
    // Mission command to perform a landing. This is used as a marker in a mission
    // to tell the autopilot where a sequence of mission items that represents a landing
    // starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which
    // case the nearest (geographically) landing sequence in the mission will be used.
    // The Latitude/Longitude is optional, and may be set to 0/0 if not needed. If
    // specified then it will be used to help find the closest landing sequence.
    MAV_CMD_DO_LAND_START = 189,
    // Mission command to perform a landing from a rally point.
    MAV_CMD_DO_RALLY_LAND = 190,
    // Mission command to safely abort an autonmous landing.
    MAV_CMD_DO_GO_AROUND = 191,
    // Reposition the vehicle to a specific WGS84 global position.
    MAV_CMD_DO_REPOSITION = 192,
    // If in a GPS controlled position mode, hold the current position or continue.
    MAV_CMD_DO_PAUSE_CONTINUE = 193,
    // Set moving direction to forward or reverse.
    MAV_CMD_DO_SET_REVERSE = 194,
    // Control onboard camera system.
    MAV_CMD_DO_CONTROL_VIDEO = 200,
    // Sets the region of interest (ROI) for a sensor set or the vehicle itself. This
    // can then be used by the vehicles control system to control the vehicle attitude
    // and the attitude of various sensors such as cameras.
    MAV_CMD_DO_SET_ROI = 201,
    // Mission command to configure an on-board camera controller system.
    MAV_CMD_DO_DIGICAM_CONFIGURE = 202,
    // Mission command to control an on-board camera controller system.
    MAV_CMD_DO_DIGICAM_CONTROL = 203,
    // Mission command to configure a camera or antenna mount
    MAV_CMD_DO_MOUNT_CONFIGURE = 204,
    // Mission command to control a camera or antenna mount
    MAV_CMD_DO_MOUNT_CONTROL = 205,
    // Mission command to set CAM_TRIGG_DIST for this flight
    MAV_CMD_DO_SET_CAM_TRIGG_DIST = 206,
    // Mission command to enable the geofence
    MAV_CMD_DO_FENCE_ENABLE = 207,
    // Mission command to trigger a parachute
    MAV_CMD_DO_PARACHUTE = 208,
    // Mission command to perform motor test
    MAV_CMD_DO_MOTOR_TEST = 209,
    // Change to/from inverted flight
    MAV_CMD_DO_INVERTED_FLIGHT = 210,
    // Sets a desired vehicle turn angle and thrust change
    MAV_CMD_DO_SET_POSITION_YAW_THRUST = 213,
    // Mission command to control a camera or antenna mount, using a quaternion as
    // reference.
    MAV_CMD_DO_MOUNT_CONTROL_QUAT = 220,
    // set id of master controller
    MAV_CMD_DO_GUIDED_MASTER = 221,
    // set limits for external control
    MAV_CMD_DO_GUIDED_LIMITS = 222,
    // Control vehicle engine. This is interpreted by the vehicles engine controller
    // to change the target engine state. It is intended for vehicles with internal
    // combustion engines
    MAV_CMD_DO_ENGINE_CONTROL = 223,
    // NOP - This command is only used to mark the upper limit of the DO commands
    // in the enumeration
    MAV_CMD_DO_LAST = 240,
    // Trigger calibration. This command will be only accepted if in pre-flight mode.
    MAV_CMD_PREFLIGHT_CALIBRATION = 241,
    // Set sensor offsets. This command will be only accepted if in pre-flight mode.
    MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS = 242,
    // Trigger UAVCAN config. This command will be only accepted if in pre-flight
    // mode.
    MAV_CMD_PREFLIGHT_UAVCAN = 243,
    // Request storage of different parameter values and logs. This command will be
    // only accepted if in pre-flight mode.
    MAV_CMD_PREFLIGHT_STORAGE = 245,
    // Request the reboot or shutdown of system components.
    MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246,
    // Hold / continue the current action
    MAV_CMD_OVERRIDE_GOTO = 252,
    // start running a mission
    MAV_CMD_MISSION_START = 300,
    // Arms / Disarms a component
    MAV_CMD_COMPONENT_ARM_DISARM = 400,
    // Request the home position from the vehicle.
    MAV_CMD_GET_HOME_POSITION = 410,
    // Starts receiver pairing
    MAV_CMD_START_RX_PAIR = 500,
    // Request the interval between messages for a particular MAVLink message ID
    MAV_CMD_GET_MESSAGE_INTERVAL = 510,
    // Request the interval between messages for a particular MAVLink message ID.
    // This interface replaces REQUEST_DATA_STREAM
    MAV_CMD_SET_MESSAGE_INTERVAL = 511,
    // Request MAVLink protocol version compatibility
    MAV_CMD_REQUEST_PROTOCOL_VERSION = 519,
    // Request autopilot capabilities
    MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520,
    // WIP: Request camera information (CAMERA_INFORMATION)
    MAV_CMD_REQUEST_CAMERA_INFORMATION = 521,
    // WIP: Request camera settings (CAMERA_SETTINGS)
    MAV_CMD_REQUEST_CAMERA_SETTINGS = 522,
    // WIP: Set the camera settings part 1 (CAMERA_SETTINGS)
    MAV_CMD_SET_CAMERA_SETTINGS_1 = 523,
    // WIP: Set the camera settings part 2 (CAMERA_SETTINGS)
    MAV_CMD_SET_CAMERA_SETTINGS_2 = 524,
    // WIP: Request storage information (STORAGE_INFORMATION)
    MAV_CMD_REQUEST_STORAGE_INFORMATION = 525,
    // WIP: Format a storage medium
    MAV_CMD_STORAGE_FORMAT = 526,
    // WIP: Request camera capture status (CAMERA_CAPTURE_STATUS)
    MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS = 527,
    // WIP: Request flight information (FLIGHT_INFORMATION)
    MAV_CMD_REQUEST_FLIGHT_INFORMATION = 528,
    // Start image capture sequence
    MAV_CMD_IMAGE_START_CAPTURE = 2000,
    // Stop image capture sequence
    MAV_CMD_IMAGE_STOP_CAPTURE = 2001,
    // Enable or disable on-board camera triggering system.
    MAV_CMD_DO_TRIGGER_CONTROL = 2003,
    // Starts video capture
    MAV_CMD_VIDEO_START_CAPTURE = 2500,
    // Stop the current video capture
    MAV_CMD_VIDEO_STOP_CAPTURE = 2501,
    // Request to start streaming logging data over MAVLink (see also LOGGING_DATA
    // message)
    MAV_CMD_LOGGING_START = 2510,
    // Request to stop streaming log data over MAVLink
    MAV_CMD_LOGGING_STOP = 2511,
    MAV_CMD_AIRFRAME_CONFIGURATION = 2520,
    // Create a panorama at the current position
    MAV_CMD_PANORAMA_CREATE = 2800,
    // Request VTOL transition
    MAV_CMD_DO_VTOL_TRANSITION = 3000,
    // This command sets the submode to standard guided when vehicle is in guided
    // mode. The vehicle holds position and altitude and the user can input the desired
    // velocites along all three axes.
    MAV_CMD_SET_GUIDED_SUBMODE_STANDARD = 4000,
    // This command sets submode circle when vehicle is in guided mode. Vehicle flies
    // along a circle facing the center of the circle. The user can input the velocity
    // along the circle and change the radius. If no input is given the vehicle will
    // hold position.
    MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE = 4001,
    // Deploy payload on a Lat / Lon / Alt position. This includes the navigation
    // to reach the required release position and velocity.
    MAV_CMD_PAYLOAD_PREPARE_DEPLOY = 30001,
    // Control the payload deployment.
    MAV_CMD_PAYLOAD_CONTROL_DEPLOY = 30002,
    // User defined waypoint item. Ground Station will show the Vehicle as flying
    // through this item.
    MAV_CMD_WAYPOINT_USER_1 = 31000,
    // User defined waypoint item. Ground Station will show the Vehicle as flying
    // through this item.
    MAV_CMD_WAYPOINT_USER_2 = 31001,
    // User defined waypoint item. Ground Station will show the Vehicle as flying
    // through this item.
    MAV_CMD_WAYPOINT_USER_3 = 31002,
    // User defined waypoint item. Ground Station will show the Vehicle as flying
    // through this item.
    MAV_CMD_WAYPOINT_USER_4 = 31003,
    // User defined waypoint item. Ground Station will show the Vehicle as flying
    // through this item.
    MAV_CMD_WAYPOINT_USER_5 = 31004,
    // User defined spatial item. Ground Station will not show the Vehicle as flying
    // through this item. Example: ROI item.
    MAV_CMD_SPATIAL_USER_1 = 31005,
    // User defined spatial item. Ground Station will not show the Vehicle as flying
    // through this item. Example: ROI item.
    MAV_CMD_SPATIAL_USER_2 = 31006,
    // User defined spatial item. Ground Station will not show the Vehicle as flying
    // through this item. Example: ROI item.
    MAV_CMD_SPATIAL_USER_3 = 31007,
    // User defined spatial item. Ground Station will not show the Vehicle as flying
    // through this item. Example: ROI item.
    MAV_CMD_SPATIAL_USER_4 = 31008,
    // User defined spatial item. Ground Station will not show the Vehicle as flying
    // through this item. Example: ROI item.
    MAV_CMD_SPATIAL_USER_5 = 31009,
    // User defined command. Ground Station will not show the Vehicle as flying through
    // this item. Example: MAV_CMD_DO_SET_PARAMETER item.
    MAV_CMD_USER_1 = 31010,
    // User defined command. Ground Station will not show the Vehicle as flying through
    // this item. Example: MAV_CMD_DO_SET_PARAMETER item.
    MAV_CMD_USER_2 = 31011,
    // User defined command. Ground Station will not show the Vehicle as flying through
    // this item. Example: MAV_CMD_DO_SET_PARAMETER item.
    MAV_CMD_USER_3 = 31012,
    // User defined command. Ground Station will not show the Vehicle as flying through
    // this item. Example: MAV_CMD_DO_SET_PARAMETER item.
    MAV_CMD_USER_4 = 31013,
    // User defined command. Ground Station will not show the Vehicle as flying through
    // this item. Example: MAV_CMD_DO_SET_PARAMETER item.
    MAV_CMD_USER_5 = 31014
};

// THIS INTERFACE IS DEPRECATED AS OF JULY 2015. Please use MESSAGE_INTERVAL instead.
// A data stream is not a fixed set of messages, but rather a recommendation to the
// autopilot software. Individual autopilots may or may not obey the recommended messages.
enum class MAV_DATA_STREAM {
    // Enable all data streams
    MAV_DATA_STREAM_ALL = 0,
    // Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
    MAV_DATA_STREAM_RAW_SENSORS = 1,
    // Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
    MAV_DATA_STREAM_EXTENDED_STATUS = 2,
    // Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
    MAV_DATA_STREAM_RC_CHANNELS = 3,
    // Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
    MAV_DATA_STREAM_RAW_CONTROLLER = 4,
    // Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
    MAV_DATA_STREAM_POSITION = 6,
    // Dependent on the autopilot
    MAV_DATA_STREAM_EXTRA1 = 10,
    // Dependent on the autopilot
    MAV_DATA_STREAM_EXTRA2 = 11,
    // Dependent on the autopilot
    MAV_DATA_STREAM_EXTRA3 = 12
};

// The ROI (region of interest) for the vehicle. This can be be used by the vehicle
// for camera/vehicle attitude alignment (see MAV_CMD_NAV_ROI).
enum class MAV_ROI {
    // No region of interest.
    MAV_ROI_NONE = 0,
    // Point toward next MISSION.
    MAV_ROI_WPNEXT = 1,
    // Point toward given MISSION.
    MAV_ROI_WPINDEX = 2,
    // Point toward fixed location.
    MAV_ROI_LOCATION = 3,
    // Point toward of given id.
    MAV_ROI_TARGET = 4
};

// ACK / NACK / ERROR values as a result of MAV_CMDs and for mission item transmission.
enum class MAV_CMD_ACK {
    // Command / mission item is ok.
    MAV_CMD_ACK_OK = 0,
    // Generic error message if none of the other reasons fails or if no detailed
    // error reporting is implemented.
    MAV_CMD_ACK_ERR_FAIL,
    // The system is refusing to accept this command from this source / communication
    // partner.
    MAV_CMD_ACK_ERR_ACCESS_DENIED,
    // Command or mission item is not supported, other commands would be accepted.
    MAV_CMD_ACK_ERR_NOT_SUPPORTED,
    // The coordinate frame of this command / mission item is not supported.
    MAV_CMD_ACK_ERR_COORDINATE_FRAME_NOT_SUPPORTED,
    // The coordinate frame of this command is ok, but he coordinate values exceed
    // the safety limits of this system. This is a generic error, please use the more
    // specific error messages below if possible.
    MAV_CMD_ACK_ERR_COORDINATES_OUT_OF_RANGE,
    // The X or latitude value is out of range.
    MAV_CMD_ACK_ERR_X_LAT_OUT_OF_RANGE,
    // The Y or longitude value is out of range.
    MAV_CMD_ACK_ERR_Y_LON_OUT_OF_RANGE,
    // The Z or altitude value is out of range.
    MAV_CMD_ACK_ERR_Z_ALT_OUT_OF_RANGE
};

// Specifies the datatype of a MAVLink parameter.
enum class MAV_PARAM_TYPE {
    // 8-bit unsigned integer
    MAV_PARAM_TYPE_UINT8 = 1,
    // 8-bit signed integer
    MAV_PARAM_TYPE_INT8 = 2,
    // 16-bit unsigned integer
    MAV_PARAM_TYPE_UINT16 = 3,
    // 16-bit signed integer
    MAV_PARAM_TYPE_INT16 = 4,
    // 32-bit unsigned integer
    MAV_PARAM_TYPE_UINT32 = 5,
    // 32-bit signed integer
    MAV_PARAM_TYPE_INT32 = 6,
    // 64-bit unsigned integer
    MAV_PARAM_TYPE_UINT64 = 7,
    // 64-bit signed integer
    MAV_PARAM_TYPE_INT64 = 8,
    // 32-bit floating-point
    MAV_PARAM_TYPE_REAL32 = 9,
    // 64-bit floating-point
    MAV_PARAM_TYPE_REAL64 = 10
};

// result from a mavlink command
enum class MAV_RESULT {
    // Command ACCEPTED and EXECUTED
    MAV_RESULT_ACCEPTED = 0,
    // Command TEMPORARY REJECTED/DENIED
    MAV_RESULT_TEMPORARILY_REJECTED = 1,
    // Command PERMANENTLY DENIED
    MAV_RESULT_DENIED = 2,
    // Command UNKNOWN/UNSUPPORTED
    MAV_RESULT_UNSUPPORTED = 3,
    // Command executed, but failed
    MAV_RESULT_FAILED = 4
};

// result in a mavlink mission ack
enum class MAV_MISSION_RESULT {
    // mission accepted OK
    MAV_MISSION_ACCEPTED = 0,
    // generic error / not accepting mission commands at all right now
    MAV_MISSION_ERROR = 1,
    // coordinate frame is not supported
    MAV_MISSION_UNSUPPORTED_FRAME = 2,
    // command is not supported
    MAV_MISSION_UNSUPPORTED = 3,
    // mission item exceeds storage space
    MAV_MISSION_NO_SPACE = 4,
    // one of the parameters has an invalid value
    MAV_MISSION_INVALID = 5,
    // param1 has an invalid value
    MAV_MISSION_INVALID_PARAM1 = 6,
    // param2 has an invalid value
    MAV_MISSION_INVALID_PARAM2 = 7,
    // param3 has an invalid value
    MAV_MISSION_INVALID_PARAM3 = 8,
    // param4 has an invalid value
    MAV_MISSION_INVALID_PARAM4 = 9,
    // x/param5 has an invalid value
    MAV_MISSION_INVALID_PARAM5_X = 10,
    // y/param6 has an invalid value
    MAV_MISSION_INVALID_PARAM6_Y = 11,
    // param7 has an invalid value
    MAV_MISSION_INVALID_PARAM7 = 12,
    // received waypoint out of sequence
    MAV_MISSION_INVALID_SEQUENCE = 13,
    // not accepting any mission commands from this communication partner
    MAV_MISSION_DENIED = 14
};

// Indicates the severity level, generally used for status messages to indicate their
// relative urgency. Based on RFC-5424 using expanded definitions at: http://www.kiwisyslog.com/kb/info:-syslog-message-levels/.
enum class MAV_SEVERITY {
    // System is unusable. This is a "panic" condition.
    MAV_SEVERITY_EMERGENCY = 0,
    // Action should be taken immediately. Indicates error in non-critical systems.
    MAV_SEVERITY_ALERT = 1,
    // Action must be taken immediately. Indicates failure in a primary system.
    MAV_SEVERITY_CRITICAL = 2,
    // Indicates an error in secondary/redundant systems.
    MAV_SEVERITY_ERROR = 3,
    // Indicates about a possible future error if this is not resolved within a given
    // timeframe. Example would be a low battery warning.
    MAV_SEVERITY_WARNING = 4,
    // An unusual event has occured, though not an error condition. This should be
    // investigated for the root cause.
    MAV_SEVERITY_NOTICE = 5,
    // Normal operational messages. Useful for logging. No action is required for
    // these messages.
    MAV_SEVERITY_INFO = 6,
    // Useful non-operational messages that can assist in debugging. These should
    // not occur during normal operation.
    MAV_SEVERITY_DEBUG = 7
};

// Power supply status flags (bitmask)
enum class MAV_POWER_STATUS {
    // main brick power supply valid
    MAV_POWER_STATUS_BRICK_VALID = 1,
    // main servo power supply valid for FMU
    MAV_POWER_STATUS_SERVO_VALID = 2,
    // USB power is connected
    MAV_POWER_STATUS_USB_CONNECTED = 4,
    // peripheral supply is in over-current state
    MAV_POWER_STATUS_PERIPH_OVERCURRENT = 8,
    // hi-power peripheral supply is in over-current state
    MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT = 16,
    // Power status has changed since boot
    MAV_POWER_STATUS_CHANGED = 32
};

// SERIAL_CONTROL device types
enum class SERIAL_CONTROL_DEV {
    // First telemetry port
    SERIAL_CONTROL_DEV_TELEM1 = 0,
    // Second telemetry port
    SERIAL_CONTROL_DEV_TELEM2 = 1,
    // First GPS port
    SERIAL_CONTROL_DEV_GPS1 = 2,
    // Second GPS port
    SERIAL_CONTROL_DEV_GPS2 = 3,
    // system shell
    SERIAL_CONTROL_DEV_SHELL = 10
};

// SERIAL_CONTROL flags (bitmask)
enum class SERIAL_CONTROL_FLAG {
    // Set if this is a reply
    SERIAL_CONTROL_FLAG_REPLY = 1,
    // Set if the sender wants the receiver to send a response as another SERIAL_CONTROL
    // message
    SERIAL_CONTROL_FLAG_RESPOND = 2,
    // Set if access to the serial port should be removed from whatever driver is
    // currently using it, giving exclusive access to the SERIAL_CONTROL protocol.
    // The port can be handed back by sending a request without this flag set
    SERIAL_CONTROL_FLAG_EXCLUSIVE = 4,
    // Block on writes to the serial port
    SERIAL_CONTROL_FLAG_BLOCKING = 8,
    // Send multiple replies until port is drained
    SERIAL_CONTROL_FLAG_MULTI = 16
};

// Enumeration of distance sensor types
enum class MAV_DISTANCE_SENSOR {
    // Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units
    MAV_DISTANCE_SENSOR_LASER = 0,
    // Ultrasound rangefinder, e.g. MaxBotix units
    MAV_DISTANCE_SENSOR_ULTRASOUND = 1,
    // Infrared rangefinder, e.g. Sharp units
    MAV_DISTANCE_SENSOR_INFRARED = 2
};

// Enumeration of sensor orientation, according to its rotations
enum class MAV_SENSOR_ORIENTATION {
    // Roll: 0, Pitch: 0, Yaw: 0
    MAV_SENSOR_ROTATION_NONE = 0,
    // Roll: 0, Pitch: 0, Yaw: 45
    MAV_SENSOR_ROTATION_YAW_45 = 1,
    // Roll: 0, Pitch: 0, Yaw: 90
    MAV_SENSOR_ROTATION_YAW_90 = 2,
    // Roll: 0, Pitch: 0, Yaw: 135
    MAV_SENSOR_ROTATION_YAW_135 = 3,
    // Roll: 0, Pitch: 0, Yaw: 180
    MAV_SENSOR_ROTATION_YAW_180 = 4,
    // Roll: 0, Pitch: 0, Yaw: 225
    MAV_SENSOR_ROTATION_YAW_225 = 5,
    // Roll: 0, Pitch: 0, Yaw: 270
    MAV_SENSOR_ROTATION_YAW_270 = 6,
    // Roll: 0, Pitch: 0, Yaw: 315
    MAV_SENSOR_ROTATION_YAW_315 = 7,
    // Roll: 180, Pitch: 0, Yaw: 0
    MAV_SENSOR_ROTATION_ROLL_180 = 8,
    // Roll: 180, Pitch: 0, Yaw: 45
    MAV_SENSOR_ROTATION_ROLL_180_YAW_45 = 9,
    // Roll: 180, Pitch: 0, Yaw: 90
    MAV_SENSOR_ROTATION_ROLL_180_YAW_90 = 10,
    // Roll: 180, Pitch: 0, Yaw: 135
    MAV_SENSOR_ROTATION_ROLL_180_YAW_135 = 11,
    // Roll: 0, Pitch: 180, Yaw: 0
    MAV_SENSOR_ROTATION_PITCH_180 = 12,
    // Roll: 180, Pitch: 0, Yaw: 225
    MAV_SENSOR_ROTATION_ROLL_180_YAW_225 = 13,
    // Roll: 180, Pitch: 0, Yaw: 270
    MAV_SENSOR_ROTATION_ROLL_180_YAW_270 = 14,
    // Roll: 180, Pitch: 0, Yaw: 315
    MAV_SENSOR_ROTATION_ROLL_180_YAW_315 = 15,
    // Roll: 90, Pitch: 0, Yaw: 0
    MAV_SENSOR_ROTATION_ROLL_90 = 16,
    // Roll: 90, Pitch: 0, Yaw: 45
    MAV_SENSOR_ROTATION_ROLL_90_YAW_45 = 17,
    // Roll: 90, Pitch: 0, Yaw: 90
    MAV_SENSOR_ROTATION_ROLL_90_YAW_90 = 18,
    // Roll: 90, Pitch: 0, Yaw: 135
    MAV_SENSOR_ROTATION_ROLL_90_YAW_135 = 19,
    // Roll: 270, Pitch: 0, Yaw: 0
    MAV_SENSOR_ROTATION_ROLL_270 = 20,
    // Roll: 270, Pitch: 0, Yaw: 45
    MAV_SENSOR_ROTATION_ROLL_270_YAW_45 = 21,
    // Roll: 270, Pitch: 0, Yaw: 90
    MAV_SENSOR_ROTATION_ROLL_270_YAW_90 = 22,
    // Roll: 270, Pitch: 0, Yaw: 135
    MAV_SENSOR_ROTATION_ROLL_270_YAW_135 = 23,
    // Roll: 0, Pitch: 90, Yaw: 0
    MAV_SENSOR_ROTATION_PITCH_90 = 24,
    // Roll: 0, Pitch: 270, Yaw: 0
    MAV_SENSOR_ROTATION_PITCH_270 = 25,
    // Roll: 0, Pitch: 180, Yaw: 90
    MAV_SENSOR_ROTATION_PITCH_180_YAW_90 = 26,
    // Roll: 0, Pitch: 180, Yaw: 270
    MAV_SENSOR_ROTATION_PITCH_180_YAW_270 = 27,
    // Roll: 90, Pitch: 90, Yaw: 0
    MAV_SENSOR_ROTATION_ROLL_90_PITCH_90 = 28,
    // Roll: 180, Pitch: 90, Yaw: 0
    MAV_SENSOR_ROTATION_ROLL_180_PITCH_90 = 29,
    // Roll: 270, Pitch: 90, Yaw: 0
    MAV_SENSOR_ROTATION_ROLL_270_PITCH_90 = 30,
    // Roll: 90, Pitch: 180, Yaw: 0
    MAV_SENSOR_ROTATION_ROLL_90_PITCH_180 = 31,
    // Roll: 270, Pitch: 180, Yaw: 0
    MAV_SENSOR_ROTATION_ROLL_270_PITCH_180 = 32,
    // Roll: 90, Pitch: 270, Yaw: 0
    MAV_SENSOR_ROTATION_ROLL_90_PITCH_270 = 33,
    // Roll: 180, Pitch: 270, Yaw: 0
    MAV_SENSOR_ROTATION_ROLL_180_PITCH_270 = 34,
    // Roll: 270, Pitch: 270, Yaw: 0
    MAV_SENSOR_ROTATION_ROLL_270_PITCH_270 = 35,
    // Roll: 90, Pitch: 180, Yaw: 90
    MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90 = 36,
    // Roll: 90, Pitch: 0, Yaw: 270
    MAV_SENSOR_ROTATION_ROLL_90_YAW_270 = 37,
    // Roll: 315, Pitch: 315, Yaw: 315
    MAV_SENSOR_ROTATION_ROLL_315_PITCH_315_YAW_315 = 38
};

// Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot
// supports this capability.
enum class MAV_PROTOCOL_CAPABILITY {
    // Autopilot supports MISSION float message type.
    MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT = 1,
    // Autopilot supports the new param float message type.
    MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT = 2,
    // Autopilot supports MISSION_INT scaled integer message type.
    MAV_PROTOCOL_CAPABILITY_MISSION_INT = 4,
    // Autopilot supports COMMAND_INT scaled integer message type.
    MAV_PROTOCOL_CAPABILITY_COMMAND_INT = 8,
    // Autopilot supports the new param union message type.
    MAV_PROTOCOL_CAPABILITY_PARAM_UNION = 16,
    // Autopilot supports the new FILE_TRANSFER_PROTOCOL message type.
    MAV_PROTOCOL_CAPABILITY_FTP = 32,
    // Autopilot supports commanding attitude offboard.
    MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET = 64,
    // Autopilot supports commanding position and velocity targets in local NED frame.
    MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED = 128,
    // Autopilot supports commanding position and velocity targets in global scaled
    // integers.
    MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT = 256,
    // Autopilot supports terrain protocol / data handling.
    MAV_PROTOCOL_CAPABILITY_TERRAIN = 512,
    // Autopilot supports direct actuator control.
    MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET = 1024,
    // Autopilot supports the flight termination command.
    MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION = 2048,
    // Autopilot supports onboard compass calibration.
    MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION = 4096,
    // Autopilot supports mavlink version 2.
    MAV_PROTOCOL_CAPABILITY_MAVLINK2 = 8192
};

// Enumeration of estimator types
enum class MAV_ESTIMATOR_TYPE {
    // This is a naive estimator without any real covariance feedback.
    MAV_ESTIMATOR_TYPE_NAIVE = 1,
    // Computer vision based estimate. Might be up to scale.
    MAV_ESTIMATOR_TYPE_VISION = 2,
    // Visual-inertial estimate.
    MAV_ESTIMATOR_TYPE_VIO = 3,
    // Plain GPS estimate.
    MAV_ESTIMATOR_TYPE_GPS = 4,
    // Estimator integrating GPS and inertial sensing.
    MAV_ESTIMATOR_TYPE_GPS_INS = 5
};

// Enumeration of battery types
enum class MAV_BATTERY_TYPE {
    // Not specified.
    MAV_BATTERY_TYPE_UNKNOWN = 0,
    // Lithium polymer battery
    MAV_BATTERY_TYPE_LIPO = 1,
    // Lithium-iron-phosphate battery
    MAV_BATTERY_TYPE_LIFE = 2,
    // Lithium-ION battery
    MAV_BATTERY_TYPE_LION = 3,
    // Nickel metal hydride battery
    MAV_BATTERY_TYPE_NIMH = 4
};

// Enumeration of battery functions
enum class MAV_BATTERY_FUNCTION {
    // Battery function is unknown
    MAV_BATTERY_FUNCTION_UNKNOWN = 0,
    // Battery supports all flight systems
    MAV_BATTERY_FUNCTION_ALL = 1,
    // Battery for the propulsion system
    MAV_BATTERY_FUNCTION_PROPULSION = 2,
    // Avionics battery
    MAV_BATTERY_FUNCTION_AVIONICS = 3,
    // Payload battery
    MAV_BATTERY_TYPE_PAYLOAD = 4
};

// Enumeration of VTOL states
enum class MAV_VTOL_STATE {
    // MAV is not configured as VTOL
    MAV_VTOL_STATE_UNDEFINED = 0,
    // VTOL is in transition from multicopter to fixed-wing
    MAV_VTOL_STATE_TRANSITION_TO_FW = 1,
    // VTOL is in transition from fixed-wing to multicopter
    MAV_VTOL_STATE_TRANSITION_TO_MC = 2,
    // VTOL is in multicopter state
    MAV_VTOL_STATE_MC = 3,
    // VTOL is in fixed-wing state
    MAV_VTOL_STATE_FW = 4
};

// Enumeration of landed detector states
enum class MAV_LANDED_STATE {
    // MAV landed state is unknown
    MAV_LANDED_STATE_UNDEFINED = 0,
    // MAV is landed (on ground)
    MAV_LANDED_STATE_ON_GROUND = 1,
    // MAV is in air
    MAV_LANDED_STATE_IN_AIR = 2
};

// Enumeration of the ADSB altimeter types
enum class ADSB_ALTITUDE_TYPE {
    // Altitude reported from a Baro source using QNH reference
    ADSB_ALTITUDE_TYPE_PRESSURE_QNH = 0,
    // Altitude reported from a GNSS source
    ADSB_ALTITUDE_TYPE_GEOMETRIC = 1
};

// ADSB classification for the type of vehicle emitting the transponder signal
enum class ADSB_EMITTER_TYPE {
    ADSB_EMITTER_TYPE_NO_INFO = 0,
    ADSB_EMITTER_TYPE_LIGHT = 1,
    ADSB_EMITTER_TYPE_SMALL = 2,
    ADSB_EMITTER_TYPE_LARGE = 3,
    ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE = 4,
    ADSB_EMITTER_TYPE_HEAVY = 5,
    ADSB_EMITTER_TYPE_HIGHLY_MANUV = 6,
    ADSB_EMITTER_TYPE_ROTOCRAFT = 7,
    ADSB_EMITTER_TYPE_UNASSIGNED = 8,
    ADSB_EMITTER_TYPE_GLIDER = 9,
    ADSB_EMITTER_TYPE_LIGHTER_AIR = 10,
    ADSB_EMITTER_TYPE_PARACHUTE = 11,
    ADSB_EMITTER_TYPE_ULTRA_LIGHT = 12,
    ADSB_EMITTER_TYPE_UNASSIGNED2 = 13,
    ADSB_EMITTER_TYPE_UAV = 14,
    ADSB_EMITTER_TYPE_SPACE = 15,
    ADSB_EMITTER_TYPE_UNASSGINED3 = 16,
    ADSB_EMITTER_TYPE_EMERGENCY_SURFACE = 17,
    ADSB_EMITTER_TYPE_SERVICE_SURFACE = 18,
    ADSB_EMITTER_TYPE_POINT_OBSTACLE = 19
};

// These flags indicate status such as data validity of each data source. Set = data
// valid
enum class ADSB_FLAGS {
    ADSB_FLAGS_VALID_COORDS = 1,
    ADSB_FLAGS_VALID_ALTITUDE = 2,
    ADSB_FLAGS_VALID_HEADING = 4,
    ADSB_FLAGS_VALID_VELOCITY = 8,
    ADSB_FLAGS_VALID_CALLSIGN = 16,
    ADSB_FLAGS_VALID_SQUAWK = 32,
    ADSB_FLAGS_SIMULATED = 64
};

// Bitmask of options for the MAV_CMD_DO_REPOSITION
enum class MAV_DO_REPOSITION_FLAGS {
    // The aircraft should immediately transition into guided. This should not be
    // set for follow me applications
    MAV_DO_REPOSITION_FLAGS_CHANGE_MODE = 1
};

// Flags in EKF_STATUS message
enum class ESTIMATOR_STATUS_FLAGS {
    // True if the attitude estimate is good
    ESTIMATOR_ATTITUDE = 1,
    // True if the horizontal velocity estimate is good
    ESTIMATOR_VELOCITY_HORIZ = 2,
    // True if the vertical velocity estimate is good
    ESTIMATOR_VELOCITY_VERT = 4,
    // True if the horizontal position (relative) estimate is good
    ESTIMATOR_POS_HORIZ_REL = 8,
    // True if the horizontal position (absolute) estimate is good
    ESTIMATOR_POS_HORIZ_ABS = 16,
    // True if the vertical position (absolute) estimate is good
    ESTIMATOR_POS_VERT_ABS = 32,
    // True if the vertical position (above ground) estimate is good
    ESTIMATOR_POS_VERT_AGL = 64,
    // True if the EKF is in a constant position mode and is not using external measurements
    // (eg GPS or optical flow)
    ESTIMATOR_CONST_POS_MODE = 128,
    // True if the EKF has sufficient data to enter a mode that will provide a (relative)
    // position estimate
    ESTIMATOR_PRED_POS_HORIZ_REL = 256,
    // True if the EKF has sufficient data to enter a mode that will provide a (absolute)
    // position estimate
    ESTIMATOR_PRED_POS_HORIZ_ABS = 512,
    // True if the EKF has detected a GPS glitch
    ESTIMATOR_GPS_GLITCH = 1024
};

enum class MOTOR_TEST_THROTTLE_TYPE {
    // throttle as a percentage from 0 ~ 100
    MOTOR_TEST_THROTTLE_PERCENT = 0,
    // throttle as an absolute PWM value (normally in range of 1000~2000)
    MOTOR_TEST_THROTTLE_PWM = 1,
    // throttle pass-through from pilot's transmitter
    MOTOR_TEST_THROTTLE_PILOT = 2
};

enum class GPS_INPUT_IGNORE_FLAGS {
    // ignore altitude field
    GPS_INPUT_IGNORE_FLAG_ALT = 1,
    // ignore hdop field
    GPS_INPUT_IGNORE_FLAG_HDOP = 2,
    // ignore vdop field
    GPS_INPUT_IGNORE_FLAG_VDOP = 4,
    // ignore horizontal velocity field (vn and ve)
    GPS_INPUT_IGNORE_FLAG_VEL_HORIZ = 8,
    // ignore vertical velocity field (vd)
    GPS_INPUT_IGNORE_FLAG_VEL_VERT = 16,
    // ignore speed accuracy field
    GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY = 32,
    // ignore horizontal accuracy field
    GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY = 64,
    // ignore vertical accuracy field
    GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY = 128
};

// Possible actions an aircraft can take to avoid a collision.
enum class MAV_COLLISION_ACTION {
    // Ignore any potential collisions
    MAV_COLLISION_ACTION_NONE = 0,
    // Report potential collision
    MAV_COLLISION_ACTION_REPORT = 1,
    // Ascend or Descend to avoid thread
    MAV_COLLISION_ACTION_ASCEND_OR_DESCEND = 2,
    // Ascend or Descend to avoid thread
    MAV_COLLISION_ACTION_MOVE_HORIZONTALLY = 3,
    // Aircraft to move perpendicular to the collision's velocity vector
    MAV_COLLISION_ACTION_MOVE_PERPENDICULAR = 4,
    // Aircraft to fly directly back to its launch point
    MAV_COLLISION_ACTION_RTL = 5,
    // Aircraft to stop in place
    MAV_COLLISION_ACTION_HOVER = 6
};

// Aircraft-rated danger from this threat.
enum class MAV_COLLISION_THREAT_LEVEL {
    // Not a threat
    MAV_COLLISION_THREAT_LEVEL_NONE = 0,
    // Craft is mildly concerned about this threat
    MAV_COLLISION_THREAT_LEVEL_LOW = 1,
    // Craft is panicing, and may take actions to avoid threat
    MAV_COLLISION_THREAT_LEVEL_HIGH = 2
};

// Source of information about this collision.
enum class MAV_COLLISION_SRC {
    // ID field references ADSB_VEHICLE packets
    MAV_COLLISION_SRC_ADSB = 0,
    // ID field references MAVLink SRC ID
    MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT = 1
};

// Type of GPS fix
enum class GPS_FIX_TYPE {
    // No GPS connected
    GPS_FIX_TYPE_NO_GPS = 0,
    // No position information, GPS is connected
    GPS_FIX_TYPE_NO_FIX = 1,
    // 2D position
    GPS_FIX_TYPE_2D_FIX = 2,
    // 3D position
    GPS_FIX_TYPE_3D_FIX = 3,
    // DGPS/SBAS aided 3D position
    GPS_FIX_TYPE_DGPS = 4,
    // RTK float, 3D position
    GPS_FIX_TYPE_RTK_FLOAT = 5,
    // RTK Fixed, 3D position
    GPS_FIX_TYPE_RTK_FIXED = 6,
    // Static fixed, typically used for base stations
    GPS_FIX_TYPE_STATIC = 7
};

// The heartbeat message shows that a system is present and responding. The type of
// the MAV and Autopilot hardware allow the receiving system to treat further messages
// from this system appropriate (e.g. by laying out the user interface based on the
// autopilot).
class MavLinkHeartbeat : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 0;
    MavLinkHeartbeat() { msgid = kMessageId; }
    // A bitfield for use for autopilot-specific flags.
    uint32_t custom_mode = 0;
    // Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE
    // ENUM)
    uint8_t type = 0;
    // Autopilot type / class. defined in MAV_AUTOPILOT ENUM
    uint8_t autopilot = 0;
    // System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
    uint8_t base_mode = 0;
    // System status flag, see MAV_STATE ENUM
    uint8_t system_status = 0;
    // MAVLink version, not writable by user, gets added by protocol because of magic
    // data type: uint8_t_mavlink_version
    uint8_t mavlink_version = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The general system state. If the system is following the MAVLink standard, the
// system state is mainly defined by three orthogonal states/modes: The system mode,
// which is either LOCKED (motors shut down and locked), MANUAL (system under RC control),
// GUIDED (system with autonomous position control, position setpoint controlled manually)
// or AUTO (system guided by path/waypoint planner). The NAV_MODE defined the current
// flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR.
// This represents the internal navigation state machine. The system status shows
// wether the system is currently active or not and if an emergency occured. During
// the CRITICAL and EMERGENCY states the MAV is still considered to be active, but
// should start emergency procedures autonomously. After a failure occured it should
// first move from active to critical to allow manual intervention and then move to
// emergency after a certain timeout.
class MavLinkSysStatus : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 1;
    MavLinkSysStatus() { msgid = kMessageId; }
    // Bitmask showing which onboard controllers and sensors are present. Value of
    // 0: not present. Value of 1: present. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
    uint32_t onboard_control_sensors_present = 0;
    // Bitmask showing which onboard controllers and sensors are enabled: Value of
    // 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
    uint32_t onboard_control_sensors_enabled = 0;
    // Bitmask showing which onboard controllers and sensors are operational or have
    // an error: Value of 0: not enabled. Value of 1: enabled. Indices defined by
    // ENUM MAV_SYS_STATUS_SENSOR
    uint32_t onboard_control_sensors_health = 0;
    // Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be
    // always below 1000
    uint16_t load = 0;
    // Battery voltage, in millivolts (1 = 1 millivolt)
    uint16_t voltage_battery = 0;
    // Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does
    // not measure the current
    int16_t current_battery = 0;
    // Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN),
    // dropped packets on all links (packets that were corrupted on reception on the
    // MAV)
    uint16_t drop_rate_comm = 0;
    // Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets
    // that were corrupted on reception on the MAV)
    uint16_t errors_comm = 0;
    // Autopilot-specific errors
    uint16_t errors_count1 = 0;
    // Autopilot-specific errors
    uint16_t errors_count2 = 0;
    // Autopilot-specific errors
    uint16_t errors_count3 = 0;
    // Autopilot-specific errors
    uint16_t errors_count4 = 0;
    // Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining
    // battery
    int8_t battery_remaining = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The system time is the time of the master clock, typically the computer clock of
// the main onboard computer.
class MavLinkSystemTime : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 2;
    MavLinkSystemTime() { msgid = kMessageId; }
    // Timestamp of the master clock in microseconds since UNIX epoch.
    uint64_t time_unix_usec = 0;
    // Timestamp of the component clock since boot time in milliseconds.
    uint32_t time_boot_ms = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// A ping message either requesting or responding to a ping. This allows to measure
// the system latencies, including serial port, radio modem and UDP connections.
class MavLinkPing : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 4;
    MavLinkPing() { msgid = kMessageId; }
    // Unix timestamp in microseconds or since system boot if smaller than MAVLink
    // epoch (1.1.2009)
    uint64_t time_usec = 0;
    // PING sequence
    uint32_t seq = 0;
    // 0: request ping from all receiving systems, if greater than 0: message is a
    // ping response and number is the system id of the requesting system
    uint8_t target_system = 0;
    // 0: request ping from all receiving components, if greater than 0: message is
    // a ping response and number is the system id of the requesting system
    uint8_t target_component = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Request to control this MAV
class MavLinkChangeOperatorControl : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 5;
    MavLinkChangeOperatorControl() { msgid = kMessageId; }
    // System the GCS requests control for
    uint8_t target_system = 0;
    // 0: request control of this MAV, 1: Release control of this MAV
    uint8_t control_request = 0;
    // 0: key as plaintext, 1-255: future, different hashing/encryption variants.
    // The GCS should in general use the safest mode possible initially and then gradually
    // move down the encryption level if it gets a NACK message indicating an encryption
    // mismatch.
    uint8_t version = 0;
    // Password / Key, depending on version plaintext or encrypted. 25 or less characters,
    // NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
    char passkey[25] = { 0 };
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Accept / deny control of this MAV
class MavLinkChangeOperatorControlAck : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 6;
    MavLinkChangeOperatorControlAck() { msgid = kMessageId; }
    // ID of the GCS this message
    uint8_t gcs_system_id = 0;
    // 0: request control of this MAV, 1: Release control of this MAV
    uint8_t control_request = 0;
    // 0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method,
    // 3: NACK: Already under control
    uint8_t ack = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol
// has been kept simple, so transmitting the key requires an encrypted channel for
// true safety.
class MavLinkAuthKey : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 7;
    MavLinkAuthKey() { msgid = kMessageId; }
    // key
    char key[32] = { 0 };
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// THIS INTERFACE IS DEPRECATED. USE COMMAND_LONG with MAV_CMD_DO_SET_MODE INSTEAD.
// Set the system mode, as defined by enum MAV_MODE. There is no target component
// id as the mode is by definition for the overall aircraft, not only for one component.
class MavLinkSetMode : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 11;
    MavLinkSetMode() { msgid = kMessageId; }
    // The new autopilot-specific mode. This field can be ignored by an autopilot.
    uint32_t custom_mode = 0;
    // The system setting the mode
    uint8_t target_system = 0;
    // The new base mode
    uint8_t base_mode = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Request to read the onboard parameter with the param_id string id. Onboard parameters
// are stored as key[const char*] -> value[float]. This allows to send a parameter
// to any other component (such as the GCS) without the need of previous knowledge
// of possible parameter names. Thus the same GCS can store different parameters for
// different autopilots. See also http://qgroundcontrol.org/parameter_interface for
// a full documentation of QGroundControl and IMU code.
class MavLinkParamRequestRead : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 20;
    MavLinkParamRequestRead() { msgid = kMessageId; }
    // Parameter index. Send -1 to use the param ID field as identifier (else the
    // param id will be ignored)
    int16_t param_index = 0;
    // System ID
    uint8_t target_system = 0;
    // Component ID
    uint8_t target_component = 0;
    // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable
    // chars and WITHOUT null termination (NULL) byte if the length is exactly 16
    // chars - applications have to provide 16+1 bytes storage if the ID is stored
    // as string
    char param_id[16] = { 0 };
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Request all parameters of this component. After this request, all parameters are
// emitted.
class MavLinkParamRequestList : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 21;
    MavLinkParamRequestList() { msgid = kMessageId; }
    // System ID
    uint8_t target_system = 0;
    // Component ID
    uint8_t target_component = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Emit the value of a onboard parameter. The inclusion of param_count and param_index
// in the message allows the recipient to keep track of received parameters and allows
// him to re-request missing parameters after a loss or timeout.
class MavLinkParamValue : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 22;
    MavLinkParamValue() { msgid = kMessageId; }
    // Onboard parameter value
    float param_value = 0;
    // Total number of onboard parameters
    uint16_t param_count = 0;
    // Index of this onboard parameter
    uint16_t param_index = 0;
    // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable
    // chars and WITHOUT null termination (NULL) byte if the length is exactly 16
    // chars - applications have to provide 16+1 bytes storage if the ID is stored
    // as string
    char param_id[16] = { 0 };
    // Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
    uint8_t param_type = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Set a parameter value TEMPORARILY to RAM. It will be reset to default on system
// reboot. Send the ACTION MAV_ACTION_STORAGE_WRITE to PERMANENTLY write the RAM contents
// to EEPROM. IMPORTANT: The receiving component should acknowledge the new parameter
// value by sending a param_value message to all communication partners. This will
// also ensure that multiple GCS all have an up-to-date list of all parameters. If
// the sending GCS did not receive a PARAM_VALUE message within its timeout time,
// it should re-send the PARAM_SET message.
class MavLinkParamSet : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 23;
    MavLinkParamSet() { msgid = kMessageId; }
    // Onboard parameter value
    float param_value = 0;
    // System ID
    uint8_t target_system = 0;
    // Component ID
    uint8_t target_component = 0;
    // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable
    // chars and WITHOUT null termination (NULL) byte if the length is exactly 16
    // chars - applications have to provide 16+1 bytes storage if the ID is stored
    // as string
    char param_id[16] = { 0 };
    // Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
    uint8_t param_type = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The global position, as returned by the Global Positioning System (GPS). This is
// NOT the global position estimate of the system, but rather a RAW sensor value.
// See message GLOBAL_POSITION for the global position estimate. Coordinate frame
// is right-handed, Z-axis up (GPS frame).
class MavLinkGpsRawInt : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 24;
    MavLinkGpsRawInt() { msgid = kMessageId; }
    // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    uint64_t time_usec = 0;
    // Latitude (WGS84), in degrees * 1E7
    int32_t lat = 0;
    // Longitude (WGS84), in degrees * 1E7
    int32_t lon = 0;
    // Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually
    // all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
    int32_t alt = 0;
    // GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
    uint16_t eph = 0;
    // GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
    uint16_t epv = 0;
    // GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
    uint16_t vel = 0;
    // Course over ground (NOT heading, but direction of movement) in degrees * 100,
    // 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
    uint16_t cog = 0;
    // See the GPS_FIX_TYPE enum.
    uint8_t fix_type = 0;
    // Number of satellites visible. If unknown, set to 255
    uint8_t satellites_visible = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The positioning status, as reported by GPS. This message is intended to display
// status information about each satellite visible to the receiver. See message GLOBAL_POSITION
// for the global position estimate. This message can contain information for up to
// 20 satellites.
class MavLinkGpsStatus : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 25;
    MavLinkGpsStatus() { msgid = kMessageId; }
    // Number of satellites visible
    uint8_t satellites_visible = 0;
    // Global satellite ID
    uint8_t satellite_prn[20] = { 0 };
    // 0: Satellite not used, 1: used for localization
    uint8_t satellite_used[20] = { 0 };
    // Elevation (0: right on top of receiver, 90: on the horizon) of satellite
    uint8_t satellite_elevation[20] = { 0 };
    // Direction of satellite, 0: 0 deg, 255: 360 deg.
    uint8_t satellite_azimuth[20] = { 0 };
    // Signal to noise ratio of satellite
    uint8_t satellite_snr[20] = { 0 };
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The RAW IMU readings for the usual 9DOF sensor setup. This message should contain
// the scaled values to the described units
class MavLinkScaledImu : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 26;
    MavLinkScaledImu() { msgid = kMessageId; }
    // Timestamp (milliseconds since system boot)
    uint32_t time_boot_ms = 0;
    // X acceleration (mg)
    int16_t xacc = 0;
    // Y acceleration (mg)
    int16_t yacc = 0;
    // Z acceleration (mg)
    int16_t zacc = 0;
    // Angular speed around X axis (millirad /sec)
    int16_t xgyro = 0;
    // Angular speed around Y axis (millirad /sec)
    int16_t ygyro = 0;
    // Angular speed around Z axis (millirad /sec)
    int16_t zgyro = 0;
    // X Magnetic field (milli tesla)
    int16_t xmag = 0;
    // Y Magnetic field (milli tesla)
    int16_t ymag = 0;
    // Z Magnetic field (milli tesla)
    int16_t zmag = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The RAW IMU readings for the usual 9DOF sensor setup. This message should always
// contain the true raw values without any scaling to allow data capture and system
// debugging.
class MavLinkRawImu : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 27;
    MavLinkRawImu() { msgid = kMessageId; }
    // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    uint64_t time_usec = 0;
    // X acceleration (raw)
    int16_t xacc = 0;
    // Y acceleration (raw)
    int16_t yacc = 0;
    // Z acceleration (raw)
    int16_t zacc = 0;
    // Angular speed around X axis (raw)
    int16_t xgyro = 0;
    // Angular speed around Y axis (raw)
    int16_t ygyro = 0;
    // Angular speed around Z axis (raw)
    int16_t zgyro = 0;
    // X Magnetic field (raw)
    int16_t xmag = 0;
    // Y Magnetic field (raw)
    int16_t ymag = 0;
    // Z Magnetic field (raw)
    int16_t zmag = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The RAW pressure readings for the typical setup of one absolute pressure and one
// differential pressure sensor. The sensor values should be the raw, UNSCALED ADC
// values.
class MavLinkRawPressure : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 28;
    MavLinkRawPressure() { msgid = kMessageId; }
    // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    uint64_t time_usec = 0;
    // Absolute pressure (raw)
    int16_t press_abs = 0;
    // Differential pressure 1 (raw, 0 if nonexistant)
    int16_t press_diff1 = 0;
    // Differential pressure 2 (raw, 0 if nonexistant)
    int16_t press_diff2 = 0;
    // Raw Temperature measurement (raw)
    int16_t temperature = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The pressure readings for the typical setup of one absolute and differential pressure
// sensor. The units are as specified in each field.
class MavLinkScaledPressure : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 29;
    MavLinkScaledPressure() { msgid = kMessageId; }
    // Timestamp (milliseconds since system boot)
    uint32_t time_boot_ms = 0;
    // Absolute pressure (hectopascal)
    float press_abs = 0;
    // Differential pressure 1 (hectopascal)
    float press_diff = 0;
    // Temperature measurement (0.01 degrees celsius)
    int16_t temperature = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
class MavLinkAttitude : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 30;
    MavLinkAttitude() { msgid = kMessageId; }
    // Timestamp (milliseconds since system boot)
    uint32_t time_boot_ms = 0;
    // Roll angle (rad, -pi..+pi)
    float roll = 0;
    // Pitch angle (rad, -pi..+pi)
    float pitch = 0;
    // Yaw angle (rad, -pi..+pi)
    float yaw = 0;
    // Roll angular speed (rad/s)
    float rollspeed = 0;
    // Pitch angular speed (rad/s)
    float pitchspeed = 0;
    // Yaw angular speed (rad/s)
    float yawspeed = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right),
// expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would
// be expressed as (1 0 0 0).
class MavLinkAttitudeQuaternion : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 31;
    MavLinkAttitudeQuaternion() { msgid = kMessageId; }
    // Timestamp (milliseconds since system boot)
    uint32_t time_boot_ms = 0;
    // Quaternion component 1, w (1 in null-rotation)
    float q1 = 0;
    // Quaternion component 2, x (0 in null-rotation)
    float q2 = 0;
    // Quaternion component 3, y (0 in null-rotation)
    float q3 = 0;
    // Quaternion component 4, z (0 in null-rotation)
    float q4 = 0;
    // Roll angular speed (rad/s)
    float rollspeed = 0;
    // Pitch angular speed (rad/s)
    float pitchspeed = 0;
    // Yaw angular speed (rad/s)
    float yawspeed = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The filtered local position (e.g. fused computer vision and accelerometers). Coordinate
// frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
class MavLinkLocalPositionNed : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 32;
    MavLinkLocalPositionNed() { msgid = kMessageId; }
    // Timestamp (milliseconds since system boot)
    uint32_t time_boot_ms = 0;
    // X Position
    float x = 0;
    // Y Position
    float y = 0;
    // Z Position
    float z = 0;
    // X Speed
    float vx = 0;
    // Y Speed
    float vy = 0;
    // Z Speed
    float vz = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The filtered global position (e.g. fused GPS and accelerometers). The position
// is in GPS-frame (right-handed, Z-up). It is designed as scaled integer message
// since the resolution of float is not sufficient.
class MavLinkGlobalPositionInt : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 33;
    MavLinkGlobalPositionInt() { msgid = kMessageId; }
    // Timestamp (milliseconds since system boot)
    uint32_t time_boot_ms = 0;
    // Latitude, expressed as degrees * 1E7
    int32_t lat = 0;
    // Longitude, expressed as degrees * 1E7
    int32_t lon = 0;
    // Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note
    // that virtually all GPS modules provide the AMSL as well)
    int32_t alt = 0;
    // Altitude above ground in meters, expressed as * 1000 (millimeters)
    int32_t relative_alt = 0;
    // Ground X Speed (Latitude, positive north), expressed as m/s * 100
    int16_t vx = 0;
    // Ground Y Speed (Longitude, positive east), expressed as m/s * 100
    int16_t vy = 0;
    // Ground Z Speed (Altitude, positive down), expressed as m/s * 100
    int16_t vz = 0;
    // Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown,
    // set to: UINT16_MAX
    uint16_t hdg = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The scaled values of the RC channels received. (-100%) -10000, (0%) 0, (100%) 10000.
// Channels that are inactive should be set to UINT16_MAX.
class MavLinkRcChannelsScaled : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 34;
    MavLinkRcChannelsScaled() { msgid = kMessageId; }
    // Timestamp (milliseconds since system boot)
    uint32_t time_boot_ms = 0;
    // RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid)
    // INT16_MAX.
    int16_t chan1_scaled = 0;
    // RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid)
    // INT16_MAX.
    int16_t chan2_scaled = 0;
    // RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid)
    // INT16_MAX.
    int16_t chan3_scaled = 0;
    // RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid)
    // INT16_MAX.
    int16_t chan4_scaled = 0;
    // RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid)
    // INT16_MAX.
    int16_t chan5_scaled = 0;
    // RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid)
    // INT16_MAX.
    int16_t chan6_scaled = 0;
    // RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid)
    // INT16_MAX.
    int16_t chan7_scaled = 0;
    // RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid)
    // INT16_MAX.
    int16_t chan8_scaled = 0;
    // Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one,
    // but this allows for more than 8 servos.
    uint8_t port = 0;
    // Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
    uint8_t rssi = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The RAW values of the RC channels received. The standard PPM modulation is as follows:
// 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters
// might violate this specification.
class MavLinkRcChannelsRaw : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 35;
    MavLinkRcChannelsRaw() { msgid = kMessageId; }
    // Timestamp (milliseconds since system boot)
    uint32_t time_boot_ms = 0;
    // RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel
    // is unused.
    uint16_t chan1_raw = 0;
    // RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel
    // is unused.
    uint16_t chan2_raw = 0;
    // RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel
    // is unused.
    uint16_t chan3_raw = 0;
    // RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel
    // is unused.
    uint16_t chan4_raw = 0;
    // RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel
    // is unused.
    uint16_t chan5_raw = 0;
    // RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel
    // is unused.
    uint16_t chan6_raw = 0;
    // RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel
    // is unused.
    uint16_t chan7_raw = 0;
    // RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel
    // is unused.
    uint16_t chan8_raw = 0;
    // Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one,
    // but this allows for more than 8 servos.
    uint8_t port = 0;
    // Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
    uint8_t rssi = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The RAW values of the servo outputs (for RC input from the remote, use the RC_CHANNELS
// messages). The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000
// microseconds: 100%.
class MavLinkServoOutputRaw : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 36;
    MavLinkServoOutputRaw() { msgid = kMessageId; }
    // Timestamp (microseconds since system boot)
    uint32_t time_usec = 0;
    // Servo output 1 value, in microseconds
    uint16_t servo1_raw = 0;
    // Servo output 2 value, in microseconds
    uint16_t servo2_raw = 0;
    // Servo output 3 value, in microseconds
    uint16_t servo3_raw = 0;
    // Servo output 4 value, in microseconds
    uint16_t servo4_raw = 0;
    // Servo output 5 value, in microseconds
    uint16_t servo5_raw = 0;
    // Servo output 6 value, in microseconds
    uint16_t servo6_raw = 0;
    // Servo output 7 value, in microseconds
    uint16_t servo7_raw = 0;
    // Servo output 8 value, in microseconds
    uint16_t servo8_raw = 0;
    // Servo output 9 value, in microseconds
    uint16_t servo9_raw = 0;
    // Servo output 10 value, in microseconds
    uint16_t servo10_raw = 0;
    // Servo output 11 value, in microseconds
    uint16_t servo11_raw = 0;
    // Servo output 12 value, in microseconds
    uint16_t servo12_raw = 0;
    // Servo output 13 value, in microseconds
    uint16_t servo13_raw = 0;
    // Servo output 14 value, in microseconds
    uint16_t servo14_raw = 0;
    // Servo output 15 value, in microseconds
    uint16_t servo15_raw = 0;
    // Servo output 16 value, in microseconds
    uint16_t servo16_raw = 0;
    // Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one,
    // but this allows to encode more than 8 servos.
    uint8_t port = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Request a partial list of mission items from the system/component. http://qgroundcontrol.org/mavlink/waypoint_protocol.
// If start and end index are the same, just send one waypoint.
class MavLinkMissionRequestPartialList : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 37;
    MavLinkMissionRequestPartialList() { msgid = kMessageId; }
    // Start index, 0 by default
    int16_t start_index = 0;
    // End index, -1 by default (-1: send list to end). Else a valid index of the
    // list
    int16_t end_index = 0;
    // System ID
    uint8_t target_system = 0;
    // Component ID
    uint8_t target_component = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// This message is sent to the MAV to write a partial list. If start index == end
// index, only one item will be transmitted / updated. If the start index is NOT 0
// and above the current list size, this request should be REJECTED!
class MavLinkMissionWritePartialList : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 38;
    MavLinkMissionWritePartialList() { msgid = kMessageId; }
    // Start index, 0 by default and smaller / equal to the largest index of the current
    // onboard list.
    int16_t start_index = 0;
    // End index, equal or greater than start index.
    int16_t end_index = 0;
    // System ID
    uint8_t target_system = 0;
    // Component ID
    uint8_t target_component = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Message encoding a mission item. This message is emitted to announce the presence
// of a mission item and to set a mission item on the system. The mission item can
// be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame
// is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also
// http://qgroundcontrol.org/mavlink/waypoint_protocol.
class MavLinkMissionItem : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 39;
    MavLinkMissionItem() { msgid = kMessageId; }
    // PARAM1, see MAV_CMD enum
    float param1 = 0;
    // PARAM2, see MAV_CMD enum
    float param2 = 0;
    // PARAM3, see MAV_CMD enum
    float param3 = 0;
    // PARAM4, see MAV_CMD enum
    float param4 = 0;
    // PARAM5 / local: x position, global: latitude
    float x = 0;
    // PARAM6 / y position: global: longitude
    float y = 0;
    // PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
    float z = 0;
    // Sequence
    uint16_t seq = 0;
    // The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
    uint16_t command = 0;
    // System ID
    uint8_t target_system = 0;
    // Component ID
    uint8_t target_component = 0;
    // The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
    uint8_t frame = 0;
    // false:0, true:1
    uint8_t current = 0;
    // autocontinue to next wp
    uint8_t autocontinue = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Request the information of the mission item with the sequence number seq. The response
// of the system to this message should be a MISSION_ITEM message. http://qgroundcontrol.org/mavlink/waypoint_protocol
class MavLinkMissionRequest : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 40;
    MavLinkMissionRequest() { msgid = kMessageId; }
    // Sequence
    uint16_t seq = 0;
    // System ID
    uint8_t target_system = 0;
    // Component ID
    uint8_t target_component = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Set the mission item with sequence number seq as current item. This means that
// the MAV will continue to this mission item on the shortest path (not following
// the mission items in-between).
class MavLinkMissionSetCurrent : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 41;
    MavLinkMissionSetCurrent() { msgid = kMessageId; }
    // Sequence
    uint16_t seq = 0;
    // System ID
    uint8_t target_system = 0;
    // Component ID
    uint8_t target_component = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Message that announces the sequence number of the current active mission item.
// The MAV will fly towards this mission item.
class MavLinkMissionCurrent : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 42;
    MavLinkMissionCurrent() { msgid = kMessageId; }
    // Sequence
    uint16_t seq = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Request the overall list of mission items from the system/component.
class MavLinkMissionRequestList : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 43;
    MavLinkMissionRequestList() { msgid = kMessageId; }
    // System ID
    uint8_t target_system = 0;
    // Component ID
    uint8_t target_component = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate
// a write transaction. The GCS can then request the individual mission item based
// on the knowledge of the total number of MISSIONs.
class MavLinkMissionCount : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 44;
    MavLinkMissionCount() { msgid = kMessageId; }
    // Number of mission items in the sequence
    uint16_t count = 0;
    // System ID
    uint8_t target_system = 0;
    // Component ID
    uint8_t target_component = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Delete all mission items at once.
class MavLinkMissionClearAll : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 45;
    MavLinkMissionClearAll() { msgid = kMessageId; }
    // System ID
    uint8_t target_system = 0;
    // Component ID
    uint8_t target_component = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// A certain mission item has been reached. The system will either hold this position
// (or circle on the orbit) or (if the autocontinue on the WP was set) continue to
// the next MISSION.
class MavLinkMissionItemReached : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 46;
    MavLinkMissionItemReached() { msgid = kMessageId; }
    // Sequence
    uint16_t seq = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Ack message during MISSION handling. The type field states if this message is a
// positive ack (type=0) or if an error happened (type=non-zero).
class MavLinkMissionAck : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 47;
    MavLinkMissionAck() { msgid = kMessageId; }
    // System ID
    uint8_t target_system = 0;
    // Component ID
    uint8_t target_component = 0;
    // See MAV_MISSION_RESULT enum
    uint8_t type = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// As local waypoints exist, the global MISSION reference allows to transform between
// the local coordinate frame and the global (GPS) coordinate frame. This can be necessary
// when e.g. in- and outdoor settings are connected and the MAV should move from in-
// to outdoor.
class MavLinkSetGpsGlobalOrigin : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 48;
    MavLinkSetGpsGlobalOrigin() { msgid = kMessageId; }
    // Latitude (WGS84), in degrees * 1E7
    int32_t latitude = 0;
    // Longitude (WGS84, in degrees * 1E7
    int32_t longitude = 0;
    // Altitude (AMSL), in meters * 1000 (positive for up)
    int32_t altitude = 0;
    // System ID
    uint8_t target_system = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Once the MAV sets a new GPS-Local correspondence, this message announces the origin
// (0,0,0) position
class MavLinkGpsGlobalOrigin : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 49;
    MavLinkGpsGlobalOrigin() { msgid = kMessageId; }
    // Latitude (WGS84), in degrees * 1E7
    int32_t latitude = 0;
    // Longitude (WGS84), in degrees * 1E7
    int32_t longitude = 0;
    // Altitude (AMSL), in meters * 1000 (positive for up)
    int32_t altitude = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Bind a RC channel to a parameter. The parameter should change accoding to the RC
// channel value.
class MavLinkParamMapRc : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 50;
    MavLinkParamMapRc() { msgid = kMessageId; }
    // Initial parameter value
    float param_value0 = 0;
    // Scale, maps the RC range [-1, 1] to a parameter value
    float scale = 0;
    // Minimum param value. The protocol does not define if this overwrites an onboard
    // minimum value. (Depends on implementation)
    float param_value_min = 0;
    // Maximum param value. The protocol does not define if this overwrites an onboard
    // maximum value. (Depends on implementation)
    float param_value_max = 0;
    // Parameter index. Send -1 to use the param ID field as identifier (else the
    // param id will be ignored), send -2 to disable any existing map for this rc_channel_index.
    int16_t param_index = 0;
    // System ID
    uint8_t target_system = 0;
    // Component ID
    uint8_t target_component = 0;
    // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable
    // chars and WITHOUT null termination (NULL) byte if the length is exactly 16
    // chars - applications have to provide 16+1 bytes storage if the ID is stored
    // as string
    char param_id[16] = { 0 };
    // Index of parameter RC channel. Not equal to the RC channel id. Typically correpsonds
    // to a potentiometer-knob on the RC.
    uint8_t parameter_rc_channel_index = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Request the information of the mission item with the sequence number seq. The response
// of the system to this message should be a MISSION_ITEM_INT message. http://qgroundcontrol.org/mavlink/waypoint_protocol
class MavLinkMissionRequestInt : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 51;
    MavLinkMissionRequestInt() { msgid = kMessageId; }
    // Sequence
    uint16_t seq = 0;
    // System ID
    uint8_t target_system = 0;
    // Component ID
    uint8_t target_component = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Set a safety zone (volume), which is defined by two corners of a cube. This message
// can be used to tell the MAV which setpoints/MISSIONs to accept and which to reject.
// Safety areas are often enforced by national or competition regulations.
class MavLinkSafetySetAllowedArea : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 54;
    MavLinkSafetySetAllowedArea() { msgid = kMessageId; }
    // x position 1 / Latitude 1
    float p1x = 0;
    // y position 1 / Longitude 1
    float p1y = 0;
    // z position 1 / Altitude 1
    float p1z = 0;
    // x position 2 / Latitude 2
    float p2x = 0;
    // y position 2 / Longitude 2
    float p2y = 0;
    // z position 2 / Altitude 2
    float p2z = 0;
    // System ID
    uint8_t target_system = 0;
    // Component ID
    uint8_t target_component = 0;
    // Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either
    // global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
    uint8_t frame = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Read out the safety zone the MAV currently assumes.
class MavLinkSafetyAllowedArea : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 55;
    MavLinkSafetyAllowedArea() { msgid = kMessageId; }
    // x position 1 / Latitude 1
    float p1x = 0;
    // y position 1 / Longitude 1
    float p1y = 0;
    // z position 1 / Altitude 1
    float p1z = 0;
    // x position 2 / Latitude 2
    float p2x = 0;
    // y position 2 / Longitude 2
    float p2y = 0;
    // z position 2 / Altitude 2
    float p2z = 0;
    // Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either
    // global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
    uint8_t frame = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right),
// expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would
// be expressed as (1 0 0 0).
class MavLinkAttitudeQuaternionCov : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 61;
    MavLinkAttitudeQuaternionCov() { msgid = kMessageId; }
    // Timestamp (microseconds since system boot or since UNIX epoch)
    uint64_t time_usec = 0;
    // Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
    float q[4] = { 0 };
    // Roll angular speed (rad/s)
    float rollspeed = 0;
    // Pitch angular speed (rad/s)
    float pitchspeed = 0;
    // Yaw angular speed (rad/s)
    float yawspeed = 0;
    // Attitude covariance
    float covariance[9] = { 0 };
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The state of the fixed wing navigation and position controller.
class MavLinkNavControllerOutput : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 62;
    MavLinkNavControllerOutput() { msgid = kMessageId; }
    // Current desired roll in degrees
    float nav_roll = 0;
    // Current desired pitch in degrees
    float nav_pitch = 0;
    // Current altitude error in meters
    float alt_error = 0;
    // Current airspeed error in meters/second
    float aspd_error = 0;
    // Current crosstrack error on x-y plane in meters
    float xtrack_error = 0;
    // Current desired heading in degrees
    int16_t nav_bearing = 0;
    // Bearing to current MISSION/target in degrees
    int16_t target_bearing = 0;
    // Distance to active MISSION in meters
    uint16_t wp_dist = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The filtered global position (e.g. fused GPS and accelerometers). The position
// is in GPS-frame (right-handed, Z-up). It is designed as scaled integer message
// since the resolution of float is not sufficient. NOTE: This message is intended
// for onboard networks / companion computers and higher-bandwidth links and optimized
// for accuracy and completeness. Please use the GLOBAL_POSITION_INT message for a
// minimal subset.
class MavLinkGlobalPositionIntCov : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 63;
    MavLinkGlobalPositionIntCov() { msgid = kMessageId; }
    // Timestamp (microseconds since system boot or since UNIX epoch)
    uint64_t time_usec = 0;
    // Latitude, expressed as degrees * 1E7
    int32_t lat = 0;
    // Longitude, expressed as degrees * 1E7
    int32_t lon = 0;
    // Altitude in meters, expressed as * 1000 (millimeters), above MSL
    int32_t alt = 0;
    // Altitude above ground in meters, expressed as * 1000 (millimeters)
    int32_t relative_alt = 0;
    // Ground X Speed (Latitude), expressed as m/s
    float vx = 0;
    // Ground Y Speed (Longitude), expressed as m/s
    float vy = 0;
    // Ground Z Speed (Altitude), expressed as m/s
    float vz = 0;
    // Covariance matrix (first six entries are the first ROW, next six entries are
    // the second row, etc.)
    float covariance[36] = { 0 };
    // Class id of the estimator this estimate originated from.
    uint8_t estimator_type = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The filtered local position (e.g. fused computer vision and accelerometers). Coordinate
// frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
class MavLinkLocalPositionNedCov : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 64;
    MavLinkLocalPositionNedCov() { msgid = kMessageId; }
    // Timestamp (microseconds since system boot or since UNIX epoch)
    uint64_t time_usec = 0;
    // X Position
    float x = 0;
    // Y Position
    float y = 0;
    // Z Position
    float z = 0;
    // X Speed (m/s)
    float vx = 0;
    // Y Speed (m/s)
    float vy = 0;
    // Z Speed (m/s)
    float vz = 0;
    // X Acceleration (m/s^2)
    float ax = 0;
    // Y Acceleration (m/s^2)
    float ay = 0;
    // Z Acceleration (m/s^2)
    float az = 0;
    // Covariance matrix upper right triangular (first nine entries are the first
    // ROW, next eight entries are the second row, etc.)
    float covariance[45] = { 0 };
    // Class id of the estimator this estimate originated from.
    uint8_t estimator_type = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The PPM values of the RC channels received. The standard PPM modulation is as follows:
// 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters
// might violate this specification.
class MavLinkRcChannels : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 65;
    MavLinkRcChannels() { msgid = kMessageId; }
    // Timestamp (milliseconds since system boot)
    uint32_t time_boot_ms = 0;
    // RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel
    // is unused.
    uint16_t chan1_raw = 0;
    // RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel
    // is unused.
    uint16_t chan2_raw = 0;
    // RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel
    // is unused.
    uint16_t chan3_raw = 0;
    // RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel
    // is unused.
    uint16_t chan4_raw = 0;
    // RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel
    // is unused.
    uint16_t chan5_raw = 0;
    // RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel
    // is unused.
    uint16_t chan6_raw = 0;
    // RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel
    // is unused.
    uint16_t chan7_raw = 0;
    // RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel
    // is unused.
    uint16_t chan8_raw = 0;
    // RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel
    // is unused.
    uint16_t chan9_raw = 0;
    // RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel
    // is unused.
    uint16_t chan10_raw = 0;
    // RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel
    // is unused.
    uint16_t chan11_raw = 0;
    // RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel
    // is unused.
    uint16_t chan12_raw = 0;
    // RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel
    // is unused.
    uint16_t chan13_raw = 0;
    // RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel
    // is unused.
    uint16_t chan14_raw = 0;
    // RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel
    // is unused.
    uint16_t chan15_raw = 0;
    // RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel
    // is unused.
    uint16_t chan16_raw = 0;
    // RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel
    // is unused.
    uint16_t chan17_raw = 0;
    // RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel
    // is unused.
    uint16_t chan18_raw = 0;
    // Total number of RC channels being received. This can be larger than 18, indicating
    // that more channels are available but not given in this message. This value
    // should be 0 when no RC channels are available.
    uint8_t chancount = 0;
    // Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
    uint8_t rssi = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// THIS INTERFACE IS DEPRECATED. USE SET_MESSAGE_INTERVAL INSTEAD.
class MavLinkRequestDataStream : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 66;
    MavLinkRequestDataStream() { msgid = kMessageId; }
    // The requested message rate
    uint16_t req_message_rate = 0;
    // The target requested to send the message stream.
    uint8_t target_system = 0;
    // The target requested to send the message stream.
    uint8_t target_component = 0;
    // The ID of the requested data stream
    uint8_t req_stream_id = 0;
    // 1 to start sending, 0 to stop sending.
    uint8_t start_stop = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// THIS INTERFACE IS DEPRECATED. USE MESSAGE_INTERVAL INSTEAD.
class MavLinkDataStream : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 67;
    MavLinkDataStream() { msgid = kMessageId; }
    // The message rate
    uint16_t message_rate = 0;
    // The ID of the requested data stream
    uint8_t stream_id = 0;
    // 1 stream is enabled, 0 stream is stopped.
    uint8_t on_off = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// This message provides an API for manually controlling the vehicle using standard
// joystick axes nomenclature, along with a joystick-like input device. Unused axes
// can be disabled an buttons are also transmit as boolean values of their
class MavLinkManualControl : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 69;
    MavLinkManualControl() { msgid = kMessageId; }
    // X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates
    // that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000)
    // movement on a joystick and the pitch of a vehicle.
    int16_t x = 0;
    // Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates
    // that this axis is invalid. Generally corresponds to left(-1000)-right(1000)
    // movement on a joystick and the roll of a vehicle.
    int16_t y = 0;
    // Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates
    // that this axis is invalid. Generally corresponds to a separate slider movement
    // with maximum being 1000 and minimum being -1000 on a joystick and the thrust
    // of a vehicle. Positive values are positive thrust, negative values are negative
    // thrust.
    int16_t z = 0;
    // R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates
    // that this axis is invalid. Generally corresponds to a twisting of the joystick,
    // with counter-clockwise being 1000 and clockwise being -1000, and the yaw of
    // a vehicle.
    int16_t r = 0;
    // A bitfield corresponding to the joystick buttons' current state, 1 for pressed,
    // 0 for released. The lowest bit corresponds to Button 1.
    uint16_t buttons = 0;
    // The system to be controlled.
    uint8_t target = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The RAW values of the RC channels sent to the MAV to override info received from
// the RC radio. A value of UINT16_MAX means no change to that channel. A value of
// 0 means control of that channel should be released back to the RC radio. The standard
// PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual
// receivers/transmitters might violate this specification.
class MavLinkRcChannelsOverride : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 70;
    MavLinkRcChannelsOverride() { msgid = kMessageId; }
    // RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore
    // this field.
    uint16_t chan1_raw = 0;
    // RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore
    // this field.
    uint16_t chan2_raw = 0;
    // RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore
    // this field.
    uint16_t chan3_raw = 0;
    // RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore
    // this field.
    uint16_t chan4_raw = 0;
    // RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore
    // this field.
    uint16_t chan5_raw = 0;
    // RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore
    // this field.
    uint16_t chan6_raw = 0;
    // RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore
    // this field.
    uint16_t chan7_raw = 0;
    // RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore
    // this field.
    uint16_t chan8_raw = 0;
    // System ID
    uint8_t target_system = 0;
    // Component ID
    uint8_t target_component = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Message encoding a mission item. This message is emitted to announce the presence
// of a mission item and to set a mission item on the system. The mission item can
// be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame
// is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See alsohttp://qgroundcontrol.org/mavlink/waypoint_protocol.
class MavLinkMissionItemInt : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 73;
    MavLinkMissionItemInt() { msgid = kMessageId; }
    // PARAM1, see MAV_CMD enum
    float param1 = 0;
    // PARAM2, see MAV_CMD enum
    float param2 = 0;
    // PARAM3, see MAV_CMD enum
    float param3 = 0;
    // PARAM4, see MAV_CMD enum
    float param4 = 0;
    // PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
    int32_t x = 0;
    // PARAM6 / y position: local: x position in meters * 1e4, global: longitude in
    // degrees *10^7
    int32_t y = 0;
    // PARAM7 / z position: global: altitude in meters (relative or absolute, depending
    // on frame.
    float z = 0;
    // Waypoint ID (sequence number). Starts at zero. Increases monotonically for
    // each waypoint, no gaps in the sequence (0,1,2,3,4).
    uint16_t seq = 0;
    // The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
    uint16_t command = 0;
    // System ID
    uint8_t target_system = 0;
    // Component ID
    uint8_t target_component = 0;
    // The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
    uint8_t frame = 0;
    // false:0, true:1
    uint8_t current = 0;
    // autocontinue to next wp
    uint8_t autocontinue = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Metrics typically displayed on a HUD for fixed wing aircraft
class MavLinkVfrHud : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 74;
    MavLinkVfrHud() { msgid = kMessageId; }
    // Current airspeed in m/s
    float airspeed = 0;
    // Current ground speed in m/s
    float groundspeed = 0;
    // Current altitude (MSL), in meters
    float alt = 0;
    // Current climb rate in meters/second
    float climb = 0;
    // Current heading in degrees, in compass units (0..360, 0=north)
    int16_t heading = 0;
    // Current throttle setting in integer percent, 0 to 100
    uint16_t throttle = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Message encoding a command with parameters as scaled integers. Scaling depends
// on the actual command value.
class MavLinkCommandInt : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 75;
    MavLinkCommandInt() { msgid = kMessageId; }
    // PARAM1, see MAV_CMD enum
    float param1 = 0;
    // PARAM2, see MAV_CMD enum
    float param2 = 0;
    // PARAM3, see MAV_CMD enum
    float param3 = 0;
    // PARAM4, see MAV_CMD enum
    float param4 = 0;
    // PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
    int32_t x = 0;
    // PARAM6 / local: y position in meters * 1e4, global: longitude in degrees *
    // 10^7
    int32_t y = 0;
    // PARAM7 / z position: global: altitude in meters (relative or absolute, depending
    // on frame.
    float z = 0;
    // The scheduled action for the mission item. see MAV_CMD in common.xml MAVLink
    // specs
    uint16_t command = 0;
    // System ID
    uint8_t target_system = 0;
    // Component ID
    uint8_t target_component = 0;
    // The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.h
    uint8_t frame = 0;
    // false:0, true:1
    uint8_t current = 0;
    // autocontinue to next wp
    uint8_t autocontinue = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Send a command with up to seven parameters to the MAV
class MavLinkCommandLong : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 76;
    MavLinkCommandLong() { msgid = kMessageId; }
    // Parameter 1, as defined by MAV_CMD enum.
    float param1 = 0;
    // Parameter 2, as defined by MAV_CMD enum.
    float param2 = 0;
    // Parameter 3, as defined by MAV_CMD enum.
    float param3 = 0;
    // Parameter 4, as defined by MAV_CMD enum.
    float param4 = 0;
    // Parameter 5, as defined by MAV_CMD enum.
    float param5 = 0;
    // Parameter 6, as defined by MAV_CMD enum.
    float param6 = 0;
    // Parameter 7, as defined by MAV_CMD enum.
    float param7 = 0;
    // Command ID, as defined by MAV_CMD enum.
    uint16_t command = 0;
    // System which should execute the command
    uint8_t target_system = 0;
    // Component which should execute the command, 0 for all components
    uint8_t target_component = 0;
    // 0: First transmission of this command. 1-255: Confirmation transmissions (e.g.
    // for kill command)
    uint8_t confirmation = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Report status of a command. Includes feedback wether the command was executed.
class MavLinkCommandAck : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 77;
    MavLinkCommandAck() { msgid = kMessageId; }
    // Command ID, as defined by MAV_CMD enum.
    uint16_t command = 0;
    // See MAV_RESULT enum
    uint8_t result = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Setpoint in roll, pitch, yaw and thrust from the operator
class MavLinkManualSetpoint : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 81;
    MavLinkManualSetpoint() { msgid = kMessageId; }
    // Timestamp in milliseconds since system boot
    uint32_t time_boot_ms = 0;
    // Desired roll rate in radians per second
    float roll = 0;
    // Desired pitch rate in radians per second
    float pitch = 0;
    // Desired yaw rate in radians per second
    float yaw = 0;
    // Collective thrust, normalized to 0 .. 1
    float thrust = 0;
    // Flight mode switch position, 0.. 255
    uint8_t mode_switch = 0;
    // Override mode switch position, 0.. 255
    uint8_t manual_override_switch = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Sets a desired vehicle attitude. Used by an external controller to command the
// vehicle (manual controller or other system).
class MavLinkSetAttitudeTarget : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 82;
    MavLinkSetAttitudeTarget() { msgid = kMessageId; }
    // Timestamp in milliseconds since system boot
    uint32_t time_boot_ms = 0;
    // Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
    float q[4] = { 0 };
    // Body roll rate in radians per second
    float body_roll_rate = 0;
    // Body roll rate in radians per second
    float body_pitch_rate = 0;
    // Body roll rate in radians per second
    float body_yaw_rate = 0;
    // Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse
    // trust)
    float thrust = 0;
    // System ID
    uint8_t target_system = 0;
    // Component ID
    uint8_t target_component = 0;
    // Mappings: If any of these bits are set, the corresponding input should be ignored:
    // bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit
    // 6: reserved, bit 7: throttle, bit 8: attitude
    uint8_t type_mask = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Reports the current commanded attitude of the vehicle as specified by the autopilot.
// This should match the commands sent in a SET_ATTITUDE_TARGET message if the vehicle
// is being controlled this way.
class MavLinkAttitudeTarget : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 83;
    MavLinkAttitudeTarget() { msgid = kMessageId; }
    // Timestamp in milliseconds since system boot
    uint32_t time_boot_ms = 0;
    // Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
    float q[4] = { 0 };
    // Body roll rate in radians per second
    float body_roll_rate = 0;
    // Body roll rate in radians per second
    float body_pitch_rate = 0;
    // Body roll rate in radians per second
    float body_yaw_rate = 0;
    // Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse
    // trust)
    float thrust = 0;
    // Mappings: If any of these bits are set, the corresponding input should be ignored:
    // bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit
    // 7: reserved, bit 8: attitude
    uint8_t type_mask = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Sets a desired vehicle position in a local north-east-down coordinate frame. Used
// by an external controller to command the vehicle (manual controller or other system).
class MavLinkSetPositionTargetLocalNed : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 84;
    MavLinkSetPositionTargetLocalNed() { msgid = kMessageId; }
    // Timestamp in milliseconds since system boot
    uint32_t time_boot_ms = 0;
    // X Position in NED frame in meters
    float x = 0;
    // Y Position in NED frame in meters
    float y = 0;
    // Z Position in NED frame in meters (note, altitude is negative in NED)
    float z = 0;
    // X velocity in NED frame in meter / s
    float vx = 0;
    // Y velocity in NED frame in meter / s
    float vy = 0;
    // Z velocity in NED frame in meter / s
    float vz = 0;
    // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter
    // / s^2 or N
    float afx = 0;
    // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter
    // / s^2 or N
    float afy = 0;
    // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter
    // / s^2 or N
    float afz = 0;
    // yaw setpoint in rad
    float yaw = 0;
    // yaw rate setpoint in rad/s
    float yaw_rate = 0;
    // Bitmask to indicate which dimensions should be ignored by the vehicle: a value
    // of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint
    // dimensions should be ignored. If bit 10 is set the floats afx afy afz should
    // be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2:
    // y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9:
    // az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
    uint16_t type_mask = 0;
    // System ID
    uint8_t target_system = 0;
    // Component ID
    uint8_t target_component = 0;
    // Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7,
    // MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
    uint8_t coordinate_frame = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Reports the current commanded vehicle position, velocity, and acceleration as specified
// by the autopilot. This should match the commands sent in SET_POSITION_TARGET_LOCAL_NED
// if the vehicle is being controlled this way.
class MavLinkPositionTargetLocalNed : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 85;
    MavLinkPositionTargetLocalNed() { msgid = kMessageId; }
    // Timestamp in milliseconds since system boot
    uint32_t time_boot_ms = 0;
    // X Position in NED frame in meters
    float x = 0;
    // Y Position in NED frame in meters
    float y = 0;
    // Z Position in NED frame in meters (note, altitude is negative in NED)
    float z = 0;
    // X velocity in NED frame in meter / s
    float vx = 0;
    // Y velocity in NED frame in meter / s
    float vy = 0;
    // Z velocity in NED frame in meter / s
    float vz = 0;
    // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter
    // / s^2 or N
    float afx = 0;
    // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter
    // / s^2 or N
    float afy = 0;
    // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter
    // / s^2 or N
    float afz = 0;
    // yaw setpoint in rad
    float yaw = 0;
    // yaw rate setpoint in rad/s
    float yaw_rate = 0;
    // Bitmask to indicate which dimensions should be ignored by the vehicle: a value
    // of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint
    // dimensions should be ignored. If bit 10 is set the floats afx afy afz should
    // be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2:
    // y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9:
    // az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
    uint16_t type_mask = 0;
    // Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7,
    // MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
    uint8_t coordinate_frame = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate
// system (WGS84). Used by an external controller to command the vehicle (manual controller
// or other system).
class MavLinkSetPositionTargetGlobalInt : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 86;
    MavLinkSetPositionTargetGlobalInt() { msgid = kMessageId; }
    // Timestamp in milliseconds since system boot. The rationale for the timestamp
    // in the setpoint is to allow the system to compensate for the transport delay
    // of the setpoint. This allows the system to compensate processing latency.
    uint32_t time_boot_ms = 0;
    // X Position in WGS84 frame in 1e7 * meters
    int32_t lat_int = 0;
    // Y Position in WGS84 frame in 1e7 * meters
    int32_t lon_int = 0;
    // Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above
    // terrain if GLOBAL_TERRAIN_ALT_INT
    float alt = 0;
    // X velocity in NED frame in meter / s
    float vx = 0;
    // Y velocity in NED frame in meter / s
    float vy = 0;
    // Z velocity in NED frame in meter / s
    float vz = 0;
    // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter
    // / s^2 or N
    float afx = 0;
    // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter
    // / s^2 or N
    float afy = 0;
    // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter
    // / s^2 or N
    float afz = 0;
    // yaw setpoint in rad
    float yaw = 0;
    // yaw rate setpoint in rad/s
    float yaw_rate = 0;
    // Bitmask to indicate which dimensions should be ignored by the vehicle: a value
    // of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint
    // dimensions should be ignored. If bit 10 is set the floats afx afy afz should
    // be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2:
    // y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9:
    // az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
    uint16_t type_mask = 0;
    // System ID
    uint8_t target_system = 0;
    // Component ID
    uint8_t target_component = 0;
    // Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    // = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
    uint8_t coordinate_frame = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Reports the current commanded vehicle position, velocity, and acceleration as specified
// by the autopilot. This should match the commands sent in SET_POSITION_TARGET_GLOBAL_INT
// if the vehicle is being controlled this way.
class MavLinkPositionTargetGlobalInt : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 87;
    MavLinkPositionTargetGlobalInt() { msgid = kMessageId; }
    // Timestamp in milliseconds since system boot. The rationale for the timestamp
    // in the setpoint is to allow the system to compensate for the transport delay
    // of the setpoint. This allows the system to compensate processing latency.
    uint32_t time_boot_ms = 0;
    // X Position in WGS84 frame in 1e7 * meters
    int32_t lat_int = 0;
    // Y Position in WGS84 frame in 1e7 * meters
    int32_t lon_int = 0;
    // Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above
    // terrain if GLOBAL_TERRAIN_ALT_INT
    float alt = 0;
    // X velocity in NED frame in meter / s
    float vx = 0;
    // Y velocity in NED frame in meter / s
    float vy = 0;
    // Z velocity in NED frame in meter / s
    float vz = 0;
    // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter
    // / s^2 or N
    float afx = 0;
    // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter
    // / s^2 or N
    float afy = 0;
    // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter
    // / s^2 or N
    float afz = 0;
    // yaw setpoint in rad
    float yaw = 0;
    // yaw rate setpoint in rad/s
    float yaw_rate = 0;
    // Bitmask to indicate which dimensions should be ignored by the vehicle: a value
    // of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint
    // dimensions should be ignored. If bit 10 is set the floats afx afy afz should
    // be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2:
    // y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9:
    // az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
    uint16_t type_mask = 0;
    // Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    // = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
    uint8_t coordinate_frame = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X
// and the global coordinate frame in NED coordinates. Coordinate frame is right-handed,
// Z-axis down (aeronautical frame, NED / north-east-down convention)
class MavLinkLocalPositionNedSystemGlobalOffset : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 89;
    MavLinkLocalPositionNedSystemGlobalOffset() { msgid = kMessageId; }
    // Timestamp (milliseconds since system boot)
    uint32_t time_boot_ms = 0;
    // X Position
    float x = 0;
    // Y Position
    float y = 0;
    // Z Position
    float z = 0;
    // Roll
    float roll = 0;
    // Pitch
    float pitch = 0;
    // Yaw
    float yaw = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// DEPRECATED PACKET! Suffers from missing airspeed fields and singularities due to
// Euler angles. Please use HIL_STATE_QUATERNION instead. Sent from simulation to
// autopilot. This packet is useful for high throughput applications such as hardware
// in the loop simulations.
class MavLinkHilState : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 90;
    MavLinkHilState() { msgid = kMessageId; }
    // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    uint64_t time_usec = 0;
    // Roll angle (rad)
    float roll = 0;
    // Pitch angle (rad)
    float pitch = 0;
    // Yaw angle (rad)
    float yaw = 0;
    // Body frame roll / phi angular speed (rad/s)
    float rollspeed = 0;
    // Body frame pitch / theta angular speed (rad/s)
    float pitchspeed = 0;
    // Body frame yaw / psi angular speed (rad/s)
    float yawspeed = 0;
    // Latitude, expressed as * 1E7
    int32_t lat = 0;
    // Longitude, expressed as * 1E7
    int32_t lon = 0;
    // Altitude in meters, expressed as * 1000 (millimeters)
    int32_t alt = 0;
    // Ground X Speed (Latitude), expressed as m/s * 100
    int16_t vx = 0;
    // Ground Y Speed (Longitude), expressed as m/s * 100
    int16_t vy = 0;
    // Ground Z Speed (Altitude), expressed as m/s * 100
    int16_t vz = 0;
    // X acceleration (mg)
    int16_t xacc = 0;
    // Y acceleration (mg)
    int16_t yacc = 0;
    // Z acceleration (mg)
    int16_t zacc = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Sent from autopilot to simulation. Hardware in the loop control outputs
class MavLinkHilControls : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 91;
    MavLinkHilControls() { msgid = kMessageId; }
    // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    uint64_t time_usec = 0;
    // Control output -1 .. 1
    float roll_ailerons = 0;
    // Control output -1 .. 1
    float pitch_elevator = 0;
    // Control output -1 .. 1
    float yaw_rudder = 0;
    // Throttle 0 .. 1
    float throttle = 0;
    // Aux 1, -1 .. 1
    float aux1 = 0;
    // Aux 2, -1 .. 1
    float aux2 = 0;
    // Aux 3, -1 .. 1
    float aux3 = 0;
    // Aux 4, -1 .. 1
    float aux4 = 0;
    // System mode (MAV_MODE)
    uint8_t mode = 0;
    // Navigation mode (MAV_NAV_MODE)
    uint8_t nav_mode = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Sent from simulation to autopilot. The RAW values of the RC channels received.
// The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds:
// 100%. Individual receivers/transmitters might violate this specification.
class MavLinkHilRcInputsRaw : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 92;
    MavLinkHilRcInputsRaw() { msgid = kMessageId; }
    // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    uint64_t time_usec = 0;
    // RC channel 1 value, in microseconds
    uint16_t chan1_raw = 0;
    // RC channel 2 value, in microseconds
    uint16_t chan2_raw = 0;
    // RC channel 3 value, in microseconds
    uint16_t chan3_raw = 0;
    // RC channel 4 value, in microseconds
    uint16_t chan4_raw = 0;
    // RC channel 5 value, in microseconds
    uint16_t chan5_raw = 0;
    // RC channel 6 value, in microseconds
    uint16_t chan6_raw = 0;
    // RC channel 7 value, in microseconds
    uint16_t chan7_raw = 0;
    // RC channel 8 value, in microseconds
    uint16_t chan8_raw = 0;
    // RC channel 9 value, in microseconds
    uint16_t chan9_raw = 0;
    // RC channel 10 value, in microseconds
    uint16_t chan10_raw = 0;
    // RC channel 11 value, in microseconds
    uint16_t chan11_raw = 0;
    // RC channel 12 value, in microseconds
    uint16_t chan12_raw = 0;
    // Receive signal strength indicator, 0: 0%, 255: 100%
    uint8_t rssi = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Sent from autopilot to simulation. Hardware in the loop control outputs (replacement
// for HIL_CONTROLS)
class MavLinkHilActuatorControls : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 93;
    MavLinkHilActuatorControls() { msgid = kMessageId; }
    // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    uint64_t time_usec = 0;
    // Flags as bitfield, reserved for future use.
    uint64_t flags = 0;
    // Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
    float controls[16] = { 0 };
    // System mode (MAV_MODE), includes arming state.
    uint8_t mode = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Optical flow from a flow sensor (e.g. optical mouse sensor)
class MavLinkOpticalFlow : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 100;
    MavLinkOpticalFlow() { msgid = kMessageId; }
    // Timestamp (UNIX)
    uint64_t time_usec = 0;
    // Flow in meters in x-sensor direction, angular-speed compensated
    float flow_comp_m_x = 0;
    // Flow in meters in y-sensor direction, angular-speed compensated
    float flow_comp_m_y = 0;
    // Ground distance in meters. Positive value: distance known. Negative value:
    // Unknown distance
    float ground_distance = 0;
    // Flow in pixels * 10 in x-sensor direction (dezi-pixels)
    int16_t flow_x = 0;
    // Flow in pixels * 10 in y-sensor direction (dezi-pixels)
    int16_t flow_y = 0;
    // Sensor ID
    uint8_t sensor_id = 0;
    // Optical flow quality / confidence. 0: bad, 255: maximum quality
    uint8_t quality = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

class MavLinkGlobalVisionPositionEstimate : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 101;
    MavLinkGlobalVisionPositionEstimate() { msgid = kMessageId; }
    // Timestamp (microseconds, synced to UNIX time or since system boot)
    uint64_t usec = 0;
    // Global X position
    float x = 0;
    // Global Y position
    float y = 0;
    // Global Z position
    float z = 0;
    // Roll angle in rad
    float roll = 0;
    // Pitch angle in rad
    float pitch = 0;
    // Yaw angle in rad
    float yaw = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

class MavLinkVisionPositionEstimate : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 102;
    MavLinkVisionPositionEstimate() { msgid = kMessageId; }
    // Timestamp (microseconds, synced to UNIX time or since system boot)
    uint64_t usec = 0;
    // Global X position
    float x = 0;
    // Global Y position
    float y = 0;
    // Global Z position
    float z = 0;
    // Roll angle in rad
    float roll = 0;
    // Pitch angle in rad
    float pitch = 0;
    // Yaw angle in rad
    float yaw = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

class MavLinkVisionSpeedEstimate : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 103;
    MavLinkVisionSpeedEstimate() { msgid = kMessageId; }
    // Timestamp (microseconds, synced to UNIX time or since system boot)
    uint64_t usec = 0;
    // Global X speed
    float x = 0;
    // Global Y speed
    float y = 0;
    // Global Z speed
    float z = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

class MavLinkViconPositionEstimate : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 104;
    MavLinkViconPositionEstimate() { msgid = kMessageId; }
    // Timestamp (microseconds, synced to UNIX time or since system boot)
    uint64_t usec = 0;
    // Global X position
    float x = 0;
    // Global Y position
    float y = 0;
    // Global Z position
    float z = 0;
    // Roll angle in rad
    float roll = 0;
    // Pitch angle in rad
    float pitch = 0;
    // Yaw angle in rad
    float yaw = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The IMU readings in SI units in NED body frame
class MavLinkHighresImu : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 105;
    MavLinkHighresImu() { msgid = kMessageId; }
    // Timestamp (microseconds, synced to UNIX time or since system boot)
    uint64_t time_usec = 0;
    // X acceleration (m/s^2)
    float xacc = 0;
    // Y acceleration (m/s^2)
    float yacc = 0;
    // Z acceleration (m/s^2)
    float zacc = 0;
    // Angular speed around X axis (rad / sec)
    float xgyro = 0;
    // Angular speed around Y axis (rad / sec)
    float ygyro = 0;
    // Angular speed around Z axis (rad / sec)
    float zgyro = 0;
    // X Magnetic field (Gauss)
    float xmag = 0;
    // Y Magnetic field (Gauss)
    float ymag = 0;
    // Z Magnetic field (Gauss)
    float zmag = 0;
    // Absolute pressure in millibar
    float abs_pressure = 0;
    // Differential pressure in millibar
    float diff_pressure = 0;
    // Altitude calculated from pressure
    float pressure_alt = 0;
    // Temperature in degrees celsius
    float temperature = 0;
    // Bitmask for fields that have updated since last message, bit 0 = xacc, bit
    // 12: temperature
    uint16_t fields_updated = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)
class MavLinkOpticalFlowRad : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 106;
    MavLinkOpticalFlowRad() { msgid = kMessageId; }
    // Timestamp (microseconds, synced to UNIX time or since system boot)
    uint64_t time_usec = 0;
    // Integration time in microseconds. Divide integrated_x and integrated_y by the
    // integration time to obtain average flow. The integration time also indicates
    // the.
    uint32_t integration_time_us = 0;
    // Flow in radians around X axis (Sensor RH rotation about the X axis induces
    // a positive flow. Sensor linear motion along the positive Y axis induces a negative
    // flow.)
    float integrated_x = 0;
    // Flow in radians around Y axis (Sensor RH rotation about the Y axis induces
    // a positive flow. Sensor linear motion along the positive X axis induces a positive
    // flow.)
    float integrated_y = 0;
    // RH rotation around X axis (rad)
    float integrated_xgyro = 0;
    // RH rotation around Y axis (rad)
    float integrated_ygyro = 0;
    // RH rotation around Z axis (rad)
    float integrated_zgyro = 0;
    // Time in microseconds since the distance was sampled.
    uint32_t time_delta_distance_us = 0;
    // Distance to the center of the flow field in meters. Positive value (including
    // zero): distance known. Negative value: Unknown distance.
    float distance = 0;
    // Temperature * 100 in centi-degrees Celsius
    int16_t temperature = 0;
    // Sensor ID
    uint8_t sensor_id = 0;
    // Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
    uint8_t quality = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The IMU readings in SI units in NED body frame
class MavLinkHilSensor : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 107;
    MavLinkHilSensor() { msgid = kMessageId; }
    // Timestamp (microseconds, synced to UNIX time or since system boot)
    uint64_t time_usec = 0;
    // X acceleration (m/s^2)
    float xacc = 0;
    // Y acceleration (m/s^2)
    float yacc = 0;
    // Z acceleration (m/s^2)
    float zacc = 0;
    // Angular speed around X axis in body frame (rad / sec)
    float xgyro = 0;
    // Angular speed around Y axis in body frame (rad / sec)
    float ygyro = 0;
    // Angular speed around Z axis in body frame (rad / sec)
    float zgyro = 0;
    // X Magnetic field (Gauss)
    float xmag = 0;
    // Y Magnetic field (Gauss)
    float ymag = 0;
    // Z Magnetic field (Gauss)
    float zmag = 0;
    // Absolute pressure in millibar
    float abs_pressure = 0;
    // Differential pressure (airspeed) in millibar
    float diff_pressure = 0;
    // Altitude calculated from pressure
    float pressure_alt = 0;
    // Temperature in degrees celsius
    float temperature = 0;
    // Bitmask for fields that have updated since last message, bit 0 = xacc, bit
    // 12: temperature, bit 31: full reset of attitude/position/velocities/etc was
    // performed in sim.
    uint32_t fields_updated = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Status of simulation environment, if used
class MavLinkSimState : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 108;
    MavLinkSimState() { msgid = kMessageId; }
    // True attitude quaternion component 1, w (1 in null-rotation)
    float q1 = 0;
    // True attitude quaternion component 2, x (0 in null-rotation)
    float q2 = 0;
    // True attitude quaternion component 3, y (0 in null-rotation)
    float q3 = 0;
    // True attitude quaternion component 4, z (0 in null-rotation)
    float q4 = 0;
    // Attitude roll expressed as Euler angles, not recommended except for human-readable
    // outputs
    float roll = 0;
    // Attitude pitch expressed as Euler angles, not recommended except for human-readable
    // outputs
    float pitch = 0;
    // Attitude yaw expressed as Euler angles, not recommended except for human-readable
    // outputs
    float yaw = 0;
    // X acceleration m/s/s
    float xacc = 0;
    // Y acceleration m/s/s
    float yacc = 0;
    // Z acceleration m/s/s
    float zacc = 0;
    // Angular speed around X axis rad/s
    float xgyro = 0;
    // Angular speed around Y axis rad/s
    float ygyro = 0;
    // Angular speed around Z axis rad/s
    float zgyro = 0;
    // Latitude in degrees
    float lat = 0;
    // Longitude in degrees
    float lon = 0;
    // Altitude in meters
    float alt = 0;
    // Horizontal position standard deviation
    float std_dev_horz = 0;
    // Vertical position standard deviation
    float std_dev_vert = 0;
    // True velocity in m/s in NORTH direction in earth-fixed NED frame
    float vn = 0;
    // True velocity in m/s in EAST direction in earth-fixed NED frame
    float ve = 0;
    // True velocity in m/s in DOWN direction in earth-fixed NED frame
    float vd = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Status generated by radio and injected into MAVLink stream.
class MavLinkRadioStatus : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 109;
    MavLinkRadioStatus() { msgid = kMessageId; }
    // Receive errors
    uint16_t rxerrors = 0;
    // Count of error corrected packets
    uint16_t fixed = 0;
    // Local signal strength
    uint8_t rssi = 0;
    // Remote signal strength
    uint8_t remrssi = 0;
    // Remaining free buffer space in percent.
    uint8_t txbuf = 0;
    // Background noise level
    uint8_t noise = 0;
    // Remote background noise level
    uint8_t remnoise = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// File transfer message
class MavLinkFileTransferProtocol : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 110;
    MavLinkFileTransferProtocol() { msgid = kMessageId; }
    // Network ID (0 for broadcast)
    uint8_t target_network = 0;
    // System ID (0 for broadcast)
    uint8_t target_system = 0;
    // Component ID (0 for broadcast)
    uint8_t target_component = 0;
    // Variable length payload. The length is defined by the remaining message length
    // when subtracting the header and other fields. The entire content of this block
    // is opaque unless you understand any the encoding message_type. The particular
    // encoding used can be extension specific and might not always be documented
    // as part of the mavlink specification.
    uint8_t payload[251] = { 0 };
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Time synchronization message.
class MavLinkTimesync : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 111;
    MavLinkTimesync() { msgid = kMessageId; }
    // Time sync timestamp 1
    int64_t tc1 = 0;
    // Time sync timestamp 2
    int64_t ts1 = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Camera-IMU triggering and synchronisation message.
class MavLinkCameraTrigger : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 112;
    MavLinkCameraTrigger() { msgid = kMessageId; }
    // Timestamp for the image frame in microseconds
    uint64_t time_usec = 0;
    // Image frame sequence
    uint32_t seq = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The global position, as returned by the Global Positioning System (GPS). This is
// NOT the global position estimate of the sytem, but rather a RAW sensor value. See
// message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed,
// Z-axis up (GPS frame).
class MavLinkHilGps : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 113;
    MavLinkHilGps() { msgid = kMessageId; }
    // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    uint64_t time_usec = 0;
    // Latitude (WGS84), in degrees * 1E7
    int32_t lat = 0;
    // Longitude (WGS84), in degrees * 1E7
    int32_t lon = 0;
    // Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
    int32_t alt = 0;
    // GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to:
    // 65535
    uint16_t eph = 0;
    // GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
    uint16_t epv = 0;
    // GPS ground speed (m/s * 100). If unknown, set to: 65535
    uint16_t vel = 0;
    // GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
    int16_t vn = 0;
    // GPS velocity in cm/s in EAST direction in earth-fixed NED frame
    int16_t ve = 0;
    // GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
    int16_t vd = 0;
    // Course over ground (NOT heading, but direction of movement) in degrees * 100,
    // 0.0..359.99 degrees. If unknown, set to: 65535
    uint16_t cog = 0;
    // 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value
    // of this field unless it is at least two, so always correctly fill in the fix.
    uint8_t fix_type = 0;
    // Number of satellites visible. If unknown, set to 255
    uint8_t satellites_visible = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)
class MavLinkHilOpticalFlow : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 114;
    MavLinkHilOpticalFlow() { msgid = kMessageId; }
    // Timestamp (microseconds, synced to UNIX time or since system boot)
    uint64_t time_usec = 0;
    // Integration time in microseconds. Divide integrated_x and integrated_y by the
    // integration time to obtain average flow. The integration time also indicates
    // the.
    uint32_t integration_time_us = 0;
    // Flow in radians around X axis (Sensor RH rotation about the X axis induces
    // a positive flow. Sensor linear motion along the positive Y axis induces a negative
    // flow.)
    float integrated_x = 0;
    // Flow in radians around Y axis (Sensor RH rotation about the Y axis induces
    // a positive flow. Sensor linear motion along the positive X axis induces a positive
    // flow.)
    float integrated_y = 0;
    // RH rotation around X axis (rad)
    float integrated_xgyro = 0;
    // RH rotation around Y axis (rad)
    float integrated_ygyro = 0;
    // RH rotation around Z axis (rad)
    float integrated_zgyro = 0;
    // Time in microseconds since the distance was sampled.
    uint32_t time_delta_distance_us = 0;
    // Distance to the center of the flow field in meters. Positive value (including
    // zero): distance known. Negative value: Unknown distance.
    float distance = 0;
    // Temperature * 100 in centi-degrees Celsius
    int16_t temperature = 0;
    // Sensor ID
    uint8_t sensor_id = 0;
    // Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
    uint8_t quality = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities.
// This packet is useful for high throughput applications such as hardware in the
// loop simulations.
class MavLinkHilStateQuaternion : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 115;
    MavLinkHilStateQuaternion() { msgid = kMessageId; }
    // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    uint64_t time_usec = 0;
    // Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with
    // 1 0 0 0 being the null-rotation)
    float attitude_quaternion[4] = { 0 };
    // Body frame roll / phi angular speed (rad/s)
    float rollspeed = 0;
    // Body frame pitch / theta angular speed (rad/s)
    float pitchspeed = 0;
    // Body frame yaw / psi angular speed (rad/s)
    float yawspeed = 0;
    // Latitude, expressed as * 1E7
    int32_t lat = 0;
    // Longitude, expressed as * 1E7
    int32_t lon = 0;
    // Altitude in meters, expressed as * 1000 (millimeters)
    int32_t alt = 0;
    // Ground X Speed (Latitude), expressed as m/s * 100
    int16_t vx = 0;
    // Ground Y Speed (Longitude), expressed as m/s * 100
    int16_t vy = 0;
    // Ground Z Speed (Altitude), expressed as m/s * 100
    int16_t vz = 0;
    // Indicated airspeed, expressed as m/s * 100
    uint16_t ind_airspeed = 0;
    // True airspeed, expressed as m/s * 100
    uint16_t true_airspeed = 0;
    // X acceleration (mg)
    int16_t xacc = 0;
    // Y acceleration (mg)
    int16_t yacc = 0;
    // Z acceleration (mg)
    int16_t zacc = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The RAW IMU readings for secondary 9DOF sensor setup. This message should contain
// the scaled values to the described units
class MavLinkScaledImu2 : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 116;
    MavLinkScaledImu2() { msgid = kMessageId; }
    // Timestamp (milliseconds since system boot)
    uint32_t time_boot_ms = 0;
    // X acceleration (mg)
    int16_t xacc = 0;
    // Y acceleration (mg)
    int16_t yacc = 0;
    // Z acceleration (mg)
    int16_t zacc = 0;
    // Angular speed around X axis (millirad /sec)
    int16_t xgyro = 0;
    // Angular speed around Y axis (millirad /sec)
    int16_t ygyro = 0;
    // Angular speed around Z axis (millirad /sec)
    int16_t zgyro = 0;
    // X Magnetic field (milli tesla)
    int16_t xmag = 0;
    // Y Magnetic field (milli tesla)
    int16_t ymag = 0;
    // Z Magnetic field (milli tesla)
    int16_t zmag = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Request a list of available logs. On some systems calling this may stop on-board
// logging until LOG_REQUEST_END is called.
class MavLinkLogRequestList : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 117;
    MavLinkLogRequestList() { msgid = kMessageId; }
    // First log id (0 for first available)
    uint16_t start = 0;
    // Last log id (0xffff for last available)
    uint16_t end = 0;
    // System ID
    uint8_t target_system = 0;
    // Component ID
    uint8_t target_component = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Reply to LOG_REQUEST_LIST
class MavLinkLogEntry : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 118;
    MavLinkLogEntry() { msgid = kMessageId; }
    // UTC timestamp of log in seconds since 1970, or 0 if not available
    uint32_t time_utc = 0;
    // Size of the log (may be approximate) in bytes
    uint32_t size = 0;
    // Log id
    uint16_t id = 0;
    // Total number of logs
    uint16_t num_logs = 0;
    // High log number
    uint16_t last_log_num = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Request a chunk of a log
class MavLinkLogRequestData : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 119;
    MavLinkLogRequestData() { msgid = kMessageId; }
    // Offset into the log
    uint32_t ofs = 0;
    // Number of bytes
    uint32_t count = 0;
    // Log id (from LOG_ENTRY reply)
    uint16_t id = 0;
    // System ID
    uint8_t target_system = 0;
    // Component ID
    uint8_t target_component = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Reply to LOG_REQUEST_DATA
class MavLinkLogData : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 120;
    MavLinkLogData() { msgid = kMessageId; }
    // Offset into the log
    uint32_t ofs = 0;
    // Log id (from LOG_ENTRY reply)
    uint16_t id = 0;
    // Number of bytes (zero for end of log)
    uint8_t count = 0;
    // log data
    uint8_t data[90] = { 0 };
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Erase all logs
class MavLinkLogErase : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 121;
    MavLinkLogErase() { msgid = kMessageId; }
    // System ID
    uint8_t target_system = 0;
    // Component ID
    uint8_t target_component = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Stop log transfer and resume normal logging
class MavLinkLogRequestEnd : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 122;
    MavLinkLogRequestEnd() { msgid = kMessageId; }
    // System ID
    uint8_t target_system = 0;
    // Component ID
    uint8_t target_component = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// data for injecting into the onboard GPS (used for DGPS)
class MavLinkGpsInjectData : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 123;
    MavLinkGpsInjectData() { msgid = kMessageId; }
    // System ID
    uint8_t target_system = 0;
    // Component ID
    uint8_t target_component = 0;
    // data length
    uint8_t len = 0;
    // raw data (110 is enough for 12 satellites of RTCMv2)
    uint8_t data[110] = { 0 };
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Second GPS data. Coordinate frame is right-handed, Z-axis up (GPS frame).
class MavLinkGps2Raw : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 124;
    MavLinkGps2Raw() { msgid = kMessageId; }
    // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    uint64_t time_usec = 0;
    // Latitude (WGS84), in degrees * 1E7
    int32_t lat = 0;
    // Longitude (WGS84), in degrees * 1E7
    int32_t lon = 0;
    // Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
    int32_t alt = 0;
    // Age of DGPS info
    uint32_t dgps_age = 0;
    // GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to:
    // UINT16_MAX
    uint16_t eph = 0;
    // GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
    uint16_t epv = 0;
    // GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
    uint16_t vel = 0;
    // Course over ground (NOT heading, but direction of movement) in degrees * 100,
    // 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
    uint16_t cog = 0;
    // See the GPS_FIX_TYPE enum.
    uint8_t fix_type = 0;
    // Number of satellites visible. If unknown, set to 255
    uint8_t satellites_visible = 0;
    // Number of DGPS satellites
    uint8_t dgps_numch = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Power supply status
class MavLinkPowerStatus : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 125;
    MavLinkPowerStatus() { msgid = kMessageId; }
    // 5V rail voltage in millivolts
    uint16_t Vcc = 0;
    // servo rail voltage in millivolts
    uint16_t Vservo = 0;
    // power supply status flags (see MAV_POWER_STATUS enum)
    uint16_t flags = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Control a serial port. This can be used for raw access to an onboard serial peripheral
// such as a GPS or telemetry radio. It is designed to make it possible to update
// the devices firmware via MAVLink messages or change the devices settings. A message
// with zero bytes can be used to change just the baudrate.
class MavLinkSerialControl : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 126;
    MavLinkSerialControl() { msgid = kMessageId; }
    // Baudrate of transfer. Zero means no change.
    uint32_t baudrate = 0;
    // Timeout for reply data in milliseconds
    uint16_t timeout = 0;
    // See SERIAL_CONTROL_DEV enum
    uint8_t device = 0;
    // See SERIAL_CONTROL_FLAG enum
    uint8_t flags = 0;
    // how many bytes in this transfer
    uint8_t count = 0;
    // serial data
    uint8_t data[70] = { 0 };
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// RTK GPS data. Gives information on the relative baseline calculation the GPS is
// reporting
class MavLinkGpsRtk : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 127;
    MavLinkGpsRtk() { msgid = kMessageId; }
    // Time since boot of last baseline message received in ms.
    uint32_t time_last_baseline_ms = 0;
    // GPS Time of Week of last baseline
    uint32_t tow = 0;
    // Current baseline in ECEF x or NED north component in mm.
    int32_t baseline_a_mm = 0;
    // Current baseline in ECEF y or NED east component in mm.
    int32_t baseline_b_mm = 0;
    // Current baseline in ECEF z or NED down component in mm.
    int32_t baseline_c_mm = 0;
    // Current estimate of baseline accuracy.
    uint32_t accuracy = 0;
    // Current number of integer ambiguity hypotheses.
    int32_t iar_num_hypotheses = 0;
    // GPS Week Number of last baseline
    uint16_t wn = 0;
    // Identification of connected RTK receiver.
    uint8_t rtk_receiver_id = 0;
    // GPS-specific health report for RTK data.
    uint8_t rtk_health = 0;
    // Rate of baseline messages being received by GPS, in HZ
    uint8_t rtk_rate = 0;
    // Current number of sats used for RTK calculation.
    uint8_t nsats = 0;
    // Coordinate system of baseline. 0 == ECEF, 1 == NED
    uint8_t baseline_coords_type = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// RTK GPS data. Gives information on the relative baseline calculation the GPS is
// reporting
class MavLinkGps2Rtk : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 128;
    MavLinkGps2Rtk() { msgid = kMessageId; }
    // Time since boot of last baseline message received in ms.
    uint32_t time_last_baseline_ms = 0;
    // GPS Time of Week of last baseline
    uint32_t tow = 0;
    // Current baseline in ECEF x or NED north component in mm.
    int32_t baseline_a_mm = 0;
    // Current baseline in ECEF y or NED east component in mm.
    int32_t baseline_b_mm = 0;
    // Current baseline in ECEF z or NED down component in mm.
    int32_t baseline_c_mm = 0;
    // Current estimate of baseline accuracy.
    uint32_t accuracy = 0;
    // Current number of integer ambiguity hypotheses.
    int32_t iar_num_hypotheses = 0;
    // GPS Week Number of last baseline
    uint16_t wn = 0;
    // Identification of connected RTK receiver.
    uint8_t rtk_receiver_id = 0;
    // GPS-specific health report for RTK data.
    uint8_t rtk_health = 0;
    // Rate of baseline messages being received by GPS, in HZ
    uint8_t rtk_rate = 0;
    // Current number of sats used for RTK calculation.
    uint8_t nsats = 0;
    // Coordinate system of baseline. 0 == ECEF, 1 == NED
    uint8_t baseline_coords_type = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the
// scaled values to the described units
class MavLinkScaledImu3 : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 129;
    MavLinkScaledImu3() { msgid = kMessageId; }
    // Timestamp (milliseconds since system boot)
    uint32_t time_boot_ms = 0;
    // X acceleration (mg)
    int16_t xacc = 0;
    // Y acceleration (mg)
    int16_t yacc = 0;
    // Z acceleration (mg)
    int16_t zacc = 0;
    // Angular speed around X axis (millirad /sec)
    int16_t xgyro = 0;
    // Angular speed around Y axis (millirad /sec)
    int16_t ygyro = 0;
    // Angular speed around Z axis (millirad /sec)
    int16_t zgyro = 0;
    // X Magnetic field (milli tesla)
    int16_t xmag = 0;
    // Y Magnetic field (milli tesla)
    int16_t ymag = 0;
    // Z Magnetic field (milli tesla)
    int16_t zmag = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

class MavLinkDataTransmissionHandshake : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 130;
    MavLinkDataTransmissionHandshake() { msgid = kMessageId; }
    // total data size in bytes (set on ACK only)
    uint32_t size = 0;
    // Width of a matrix or image
    uint16_t width = 0;
    // Height of a matrix or image
    uint16_t height = 0;
    // number of packets beeing sent (set on ACK only)
    uint16_t packets = 0;
    // type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h)
    uint8_t type = 0;
    // payload size per packet (normally 253 byte, see DATA field size in message
    // ENCAPSULATED_DATA) (set on ACK only)
    uint8_t payload = 0;
    // JPEG quality out of [1,100]
    uint8_t jpg_quality = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

class MavLinkEncapsulatedData : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 131;
    MavLinkEncapsulatedData() { msgid = kMessageId; }
    // sequence number (starting with 0 on every transmission)
    uint16_t seqnr = 0;
    // image data bytes
    uint8_t data[253] = { 0 };
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

class MavLinkDistanceSensor : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 132;
    MavLinkDistanceSensor() { msgid = kMessageId; }
    // Time since system boot
    uint32_t time_boot_ms = 0;
    // Minimum distance the sensor can measure in centimeters
    uint16_t min_distance = 0;
    // Maximum distance the sensor can measure in centimeters
    uint16_t max_distance = 0;
    // Current distance reading
    uint16_t current_distance = 0;
    // Type from MAV_DISTANCE_SENSOR enum.
    uint8_t type = 0;
    // Onboard ID of the sensor
    uint8_t id = 0;
    // Direction the sensor faces from MAV_SENSOR_ORIENTATION enum.
    uint8_t orientation = 0;
    // Measurement covariance in centimeters, 0 for unknown / invalid readings
    uint8_t covariance = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Request for terrain data and terrain status
class MavLinkTerrainRequest : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 133;
    MavLinkTerrainRequest() { msgid = kMessageId; }
    // Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
    uint64_t mask = 0;
    // Latitude of SW corner of first grid (degrees *10^7)
    int32_t lat = 0;
    // Longitude of SW corner of first grid (in degrees *10^7)
    int32_t lon = 0;
    // Grid spacing in meters
    uint16_t grid_spacing = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a
// lat/lon from a TERRAIN_REQUEST
class MavLinkTerrainData : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 134;
    MavLinkTerrainData() { msgid = kMessageId; }
    // Latitude of SW corner of first grid (degrees *10^7)
    int32_t lat = 0;
    // Longitude of SW corner of first grid (in degrees *10^7)
    int32_t lon = 0;
    // Grid spacing in meters
    uint16_t grid_spacing = 0;
    // Terrain data in meters AMSL
    int16_t data[16] = { 0 };
    // bit within the terrain request mask
    uint8_t gridbit = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Request that the vehicle report terrain height at the given location. Used by GCS
// to check if vehicle has all terrain data needed for a mission.
class MavLinkTerrainCheck : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 135;
    MavLinkTerrainCheck() { msgid = kMessageId; }
    // Latitude (degrees *10^7)
    int32_t lat = 0;
    // Longitude (degrees *10^7)
    int32_t lon = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Response from a TERRAIN_CHECK request
class MavLinkTerrainReport : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 136;
    MavLinkTerrainReport() { msgid = kMessageId; }
    // Latitude (degrees *10^7)
    int32_t lat = 0;
    // Longitude (degrees *10^7)
    int32_t lon = 0;
    // Terrain height in meters AMSL
    float terrain_height = 0;
    // Current vehicle height above lat/lon terrain height (meters)
    float current_height = 0;
    // grid spacing (zero if terrain at this location unavailable)
    uint16_t spacing = 0;
    // Number of 4x4 terrain blocks waiting to be received or read from disk
    uint16_t pending = 0;
    // Number of 4x4 terrain blocks in memory
    uint16_t loaded = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Barometer readings for 2nd barometer
class MavLinkScaledPressure2 : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 137;
    MavLinkScaledPressure2() { msgid = kMessageId; }
    // Timestamp (milliseconds since system boot)
    uint32_t time_boot_ms = 0;
    // Absolute pressure (hectopascal)
    float press_abs = 0;
    // Differential pressure 1 (hectopascal)
    float press_diff = 0;
    // Temperature measurement (0.01 degrees celsius)
    int16_t temperature = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Motion capture attitude and position
class MavLinkAttPosMocap : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 138;
    MavLinkAttPosMocap() { msgid = kMessageId; }
    // Timestamp (micros since boot or Unix epoch)
    uint64_t time_usec = 0;
    // Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
    float q[4] = { 0 };
    // X position in meters (NED)
    float x = 0;
    // Y position in meters (NED)
    float y = 0;
    // Z position in meters (NED)
    float z = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Set the vehicle attitude and body angular rates.
class MavLinkSetActuatorControlTarget : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 139;
    MavLinkSetActuatorControlTarget() { msgid = kMessageId; }
    // Timestamp (micros since boot or Unix epoch)
    uint64_t time_usec = 0;
    // Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for
    // single rotation direction motors is 0..1, negative range for reverse direction.
    // Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch,
    // yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
    // mixer to repurpose them as generic outputs.
    float controls[8] = { 0 };
    // Actuator group. The "_mlx" indicates this is a multi-instance message and a
    // MAVLink parser should use this field to difference between instances.
    uint8_t group_mlx = 0;
    // System ID
    uint8_t target_system = 0;
    // Component ID
    uint8_t target_component = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Set the vehicle attitude and body angular rates.
class MavLinkActuatorControlTarget : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 140;
    MavLinkActuatorControlTarget() { msgid = kMessageId; }
    // Timestamp (micros since boot or Unix epoch)
    uint64_t time_usec = 0;
    // Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for
    // single rotation direction motors is 0..1, negative range for reverse direction.
    // Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch,
    // yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
    // mixer to repurpose them as generic outputs.
    float controls[8] = { 0 };
    // Actuator group. The "_mlx" indicates this is a multi-instance message and a
    // MAVLink parser should use this field to difference between instances.
    uint8_t group_mlx = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The current system altitude.
class MavLinkAltitude : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 141;
    MavLinkAltitude() { msgid = kMessageId; }
    // Timestamp (micros since boot or Unix epoch)
    uint64_t time_usec = 0;
    // This altitude measure is initialized on system boot and monotonic (it is never
    // reset, but represents the local altitude change). The only guarantee on this
    // field is that it will never be reset and is consistent within a flight. The
    // recommended value for this field is the uncorrected barometric altitude at
    // boot time. This altitude will also drift and vary between flights.
    float altitude_monotonic = 0;
    // This altitude measure is strictly above mean sea level and might be non-monotonic
    // (it might reset on events like GPS lock or when a new QNH value is set). It
    // should be the altitude to which global altitude waypoints are compared to.
    // Note that it is *not* the GPS altitude, however, most GPS modules already output
    // AMSL by default and not the WGS84 altitude.
    float altitude_amsl = 0;
    // This is the local altitude in the local coordinate frame. It is not the altitude
    // above home, but in reference to the coordinate origin (0, 0, 0). It is up-positive.
    float altitude_local = 0;
    // This is the altitude above the home position. It resets on each change of the
    // current home position.
    float altitude_relative = 0;
    // This is the altitude above terrain. It might be fed by a terrain database or
    // an altimeter. Values smaller than -1000 should be interpreted as unknown.
    float altitude_terrain = 0;
    // This is not the altitude, but the clear space below the system according to
    // the fused clearance estimate. It generally should max out at the maximum range
    // of e.g. the laser altimeter. It is generally a moving target. A negative value
    // indicates no measurement available.
    float bottom_clearance = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The autopilot is requesting a resource (file, binary, other type of data)
class MavLinkResourceRequest : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 142;
    MavLinkResourceRequest() { msgid = kMessageId; }
    // Request ID. This ID should be re-used when sending back URI contents
    uint8_t request_id = 0;
    // The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
    uint8_t uri_type = 0;
    // The requested unique resource identifier (URI). It is not necessarily a straight
    // domain name (depends on the URI type enum)
    uint8_t uri[120] = { 0 };
    // The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary
    // stream.
    uint8_t transfer_type = 0;
    // The storage path the autopilot wants the URI to be stored in. Will only be
    // valid if the transfer_type has a storage associated (e.g. MAVLink FTP).
    uint8_t storage[120] = { 0 };
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Barometer readings for 3rd barometer
class MavLinkScaledPressure3 : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 143;
    MavLinkScaledPressure3() { msgid = kMessageId; }
    // Timestamp (milliseconds since system boot)
    uint32_t time_boot_ms = 0;
    // Absolute pressure (hectopascal)
    float press_abs = 0;
    // Differential pressure 1 (hectopascal)
    float press_diff = 0;
    // Temperature measurement (0.01 degrees celsius)
    int16_t temperature = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// current motion information from a designated system
class MavLinkFollowTarget : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 144;
    MavLinkFollowTarget() { msgid = kMessageId; }
    // Timestamp in milliseconds since system boot
    uint64_t timestamp = 0;
    // button states or switches of a tracker device
    uint64_t custom_state = 0;
    // Latitude (WGS84), in degrees * 1E7
    int32_t lat = 0;
    // Longitude (WGS84), in degrees * 1E7
    int32_t lon = 0;
    // AMSL, in meters
    float alt = 0;
    // target velocity (0,0,0) for unknown
    float vel[3] = { 0 };
    // linear target acceleration (0,0,0) for unknown
    float acc[3] = { 0 };
    // (1 0 0 0 for unknown)
    float attitude_q[4] = { 0 };
    // (0 0 0 for unknown)
    float rates[3] = { 0 };
    // eph epv
    float position_cov[3] = { 0 };
    // bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL =
    // 2, ATT + RATES = 3)
    uint8_t est_capabilities = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The smoothed, monotonic system state used to feed the control loops of the system.
class MavLinkControlSystemState : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 146;
    MavLinkControlSystemState() { msgid = kMessageId; }
    // Timestamp (micros since boot or Unix epoch)
    uint64_t time_usec = 0;
    // X acceleration in body frame
    float x_acc = 0;
    // Y acceleration in body frame
    float y_acc = 0;
    // Z acceleration in body frame
    float z_acc = 0;
    // X velocity in body frame
    float x_vel = 0;
    // Y velocity in body frame
    float y_vel = 0;
    // Z velocity in body frame
    float z_vel = 0;
    // X position in local frame
    float x_pos = 0;
    // Y position in local frame
    float y_pos = 0;
    // Z position in local frame
    float z_pos = 0;
    // Airspeed, set to -1 if unknown
    float airspeed = 0;
    // Variance of body velocity estimate
    float vel_variance[3] = { 0 };
    // Variance in local position
    float pos_variance[3] = { 0 };
    // The attitude, represented as Quaternion
    float q[4] = { 0 };
    // Angular rate in roll axis
    float roll_rate = 0;
    // Angular rate in pitch axis
    float pitch_rate = 0;
    // Angular rate in yaw axis
    float yaw_rate = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Battery information
class MavLinkBatteryStatus : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 147;
    MavLinkBatteryStatus() { msgid = kMessageId; }
    // Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide
    // mAh consumption estimate
    int32_t current_consumed = 0;
    // Consumed energy, in 100*Joules (intergrated U*I*dt) (1 = 100 Joule), -1: autopilot
    // does not provide energy consumption estimate
    int32_t energy_consumed = 0;
    // Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown
    // temperature.
    int16_t temperature = 0;
    // Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the
    // valid cell count for this battery should have the UINT16_MAX value.
    uint16_t voltages[10] = { 0 };
    // Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does
    // not measure the current
    int16_t current_battery = 0;
    // Battery ID
    uint8_t id = 0;
    // Function of the battery
    uint8_t battery_function = 0;
    // Type (chemistry) of the battery
    uint8_t type = 0;
    // Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate
    // the remaining battery
    int8_t battery_remaining = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Version and capability of autopilot software
class MavLinkAutopilotVersion : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 148;
    MavLinkAutopilotVersion() { msgid = kMessageId; }
    // bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
    uint64_t capabilities = 0;
    // UID if provided by hardware
    uint64_t uid = 0;
    // Firmware version number
    uint32_t flight_sw_version = 0;
    // Middleware version number
    uint32_t middleware_sw_version = 0;
    // Operating system version number
    uint32_t os_sw_version = 0;
    // HW / board version (last 8 bytes should be silicon ID, if any)
    uint32_t board_version = 0;
    // ID of the board vendor
    uint16_t vendor_id = 0;
    // ID of the product
    uint16_t product_id = 0;
    // Custom version field, commonly the first 8 bytes of the git hash. This is not
    // an unique identifier, but should allow to identify the commit using the main
    // version number even for very large code bases.
    uint8_t flight_custom_version[8] = { 0 };
    // Custom version field, commonly the first 8 bytes of the git hash. This is not
    // an unique identifier, but should allow to identify the commit using the main
    // version number even for very large code bases.
    uint8_t middleware_custom_version[8] = { 0 };
    // Custom version field, commonly the first 8 bytes of the git hash. This is not
    // an unique identifier, but should allow to identify the commit using the main
    // version number even for very large code bases.
    uint8_t os_custom_version[8] = { 0 };
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The location of a landing area captured from a downward facing camera
class MavLinkLandingTarget : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 149;
    MavLinkLandingTarget() { msgid = kMessageId; }
    // Timestamp (micros since boot or Unix epoch)
    uint64_t time_usec = 0;
    // X-axis angular offset (in radians) of the target from the center of the image
    float angle_x = 0;
    // Y-axis angular offset (in radians) of the target from the center of the image
    float angle_y = 0;
    // Distance to the target from the vehicle in meters
    float distance = 0;
    // Size in radians of target along x-axis
    float size_x = 0;
    // Size in radians of target along y-axis
    float size_y = 0;
    // The ID of the target if multiple targets are present
    uint8_t target_num = 0;
    // MAV_FRAME enum specifying the whether the following feilds are earth-frame,
    // body-frame, etc.
    uint8_t frame = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Estimator status message including flags, innovation test ratios and estimated
// accuracies. The flags message is an integer bitmask containing information on which
// EKF outputs are valid. See the ESTIMATOR_STATUS_FLAGS enum definition for further
// information. The innovaton test ratios show the magnitude of the sensor innovation
// divided by the innovation check threshold. Under normal operation the innovaton
// test ratios should be below 0.5 with occasional values up to 1.0. Values greater
// than 1.0 should be rare under normal operation and indicate that a measurement
// has been rejected by the filter. The user should be notified if an innovation test
// ratio greater than 1.0 is recorded. Notifications for values in the range between
// 0.5 and 1.0 should be optional and controllable by the user.
class MavLinkEstimatorStatus : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 230;
    MavLinkEstimatorStatus() { msgid = kMessageId; }
    // Timestamp (micros since boot or Unix epoch)
    uint64_t time_usec = 0;
    // Velocity innovation test ratio
    float vel_ratio = 0;
    // Horizontal position innovation test ratio
    float pos_horiz_ratio = 0;
    // Vertical position innovation test ratio
    float pos_vert_ratio = 0;
    // Magnetometer innovation test ratio
    float mag_ratio = 0;
    // Height above terrain innovation test ratio
    float hagl_ratio = 0;
    // True airspeed innovation test ratio
    float tas_ratio = 0;
    // Horizontal position 1-STD accuracy relative to the EKF local origin (m)
    float pos_horiz_accuracy = 0;
    // Vertical position 1-STD accuracy relative to the EKF local origin (m)
    float pos_vert_accuracy = 0;
    // Integer bitmask indicating which EKF outputs are valid. See definition for
    // ESTIMATOR_STATUS_FLAGS.
    uint16_t flags = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

class MavLinkWindCov : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 231;
    MavLinkWindCov() { msgid = kMessageId; }
    // Timestamp (micros since boot or Unix epoch)
    uint64_t time_usec = 0;
    // Wind in X (NED) direction in m/s
    float wind_x = 0;
    // Wind in Y (NED) direction in m/s
    float wind_y = 0;
    // Wind in Z (NED) direction in m/s
    float wind_z = 0;
    // Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
    float var_horiz = 0;
    // Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
    float var_vert = 0;
    // AMSL altitude (m) this measurement was taken at
    float wind_alt = 0;
    // Horizontal speed 1-STD accuracy
    float horiz_accuracy = 0;
    // Vertical speed 1-STD accuracy
    float vert_accuracy = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// GPS sensor input message. This is a raw sensor value sent by the GPS. This is NOT
// the global position estimate of the sytem.
class MavLinkGpsInput : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 232;
    MavLinkGpsInput() { msgid = kMessageId; }
    // Timestamp (micros since boot or Unix epoch)
    uint64_t time_usec = 0;
    // GPS time (milliseconds from start of GPS week)
    uint32_t time_week_ms = 0;
    // Latitude (WGS84), in degrees * 1E7
    int32_t lat = 0;
    // Longitude (WGS84), in degrees * 1E7
    int32_t lon = 0;
    // Altitude (AMSL, not WGS84), in m (positive for up)
    float alt = 0;
    // GPS HDOP horizontal dilution of position in m
    float hdop = 0;
    // GPS VDOP vertical dilution of position in m
    float vdop = 0;
    // GPS velocity in m/s in NORTH direction in earth-fixed NED frame
    float vn = 0;
    // GPS velocity in m/s in EAST direction in earth-fixed NED frame
    float ve = 0;
    // GPS velocity in m/s in DOWN direction in earth-fixed NED frame
    float vd = 0;
    // GPS speed accuracy in m/s
    float speed_accuracy = 0;
    // GPS horizontal accuracy in m
    float horiz_accuracy = 0;
    // GPS vertical accuracy in m
    float vert_accuracy = 0;
    // Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).
    // All other fields must be provided.
    uint16_t ignore_flags = 0;
    // GPS week number
    uint16_t time_week = 0;
    // ID of the GPS for multiple GPS inputs
    uint8_t gps_id = 0;
    // 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
    uint8_t fix_type = 0;
    // Number of satellites visible.
    uint8_t satellites_visible = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// WORK IN PROGRESS! RTCM message for injecting into the onboard GPS (used for DGPS)
class MavLinkGpsRtcmData : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 233;
    MavLinkGpsRtcmData() { msgid = kMessageId; }
    // LSB: 1 means message is fragmented
    uint8_t flags = 0;
    // data length
    uint8_t len = 0;
    // RTCM message (may be fragmented)
    uint8_t data[180] = { 0 };
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Message appropriate for high latency connections like Iridium
class MavLinkHighLatency : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 234;
    MavLinkHighLatency() { msgid = kMessageId; }
    // Timestamp (microseconds since UNIX epoch)
    uint64_t time_usec = 0;
    // A bitfield for use for autopilot-specific flags.
    uint32_t custom_mode = 0;
    // Latitude, expressed as degrees * 1E7
    int32_t latitude = 0;
    // Longitude, expressed as degrees * 1E7
    int32_t longitude = 0;
    // roll (centidegrees)
    int16_t roll = 0;
    // pitch (centidegrees)
    int16_t pitch = 0;
    // heading (centidegrees)
    uint16_t heading = 0;
    // roll setpoint (centidegrees)
    int16_t roll_sp = 0;
    // pitch setpoint (centidegrees)
    int16_t pitch_sp = 0;
    // heading setpoint (centidegrees)
    int16_t heading_sp = 0;
    // Altitude above the home position (meters)
    int16_t altitude_home = 0;
    // Altitude above mean sea level (meters)
    int16_t altitude_amsl = 0;
    // Altitude setpoint relative to the home position (meters)
    int16_t altitude_sp = 0;
    // distance to target (meters)
    uint16_t wp_distance = 0;
    // System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
    uint8_t base_mode = 0;
    // The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
    uint8_t landed_state = 0;
    // throttle (percentage)
    int8_t throttle = 0;
    // airspeed (m/s)
    uint8_t airspeed = 0;
    // airspeed setpoint (m/s)
    uint8_t airspeed_sp = 0;
    // groundspeed (m/s)
    uint8_t groundspeed = 0;
    // climb rate (m/s)
    int8_t climb_rate = 0;
    // Number of satellites visible. If unknown, set to 255
    uint8_t gps_nsat = 0;
    // See the GPS_FIX_TYPE enum.
    uint8_t gps_fix_type = 0;
    // Remaining battery (percentage)
    uint8_t battery_remaining = 0;
    // Autopilot temperature (degrees C)
    int8_t temperature = 0;
    // Air temperature (degrees C) from airspeed sensor
    int8_t temperature_air = 0;
    // failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC,
    // bit1:batt, bit2:GPS, bit3:GCS, bit4:fence)
    uint8_t failsafe = 0;
    // current waypoint number
    uint8_t wp_num = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Vibration levels and accelerometer clipping
class MavLinkVibration : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 241;
    MavLinkVibration() { msgid = kMessageId; }
    // Timestamp (micros since boot or Unix epoch)
    uint64_t time_usec = 0;
    // Vibration levels on X-axis
    float vibration_x = 0;
    // Vibration levels on Y-axis
    float vibration_y = 0;
    // Vibration levels on Z-axis
    float vibration_z = 0;
    // first accelerometer clipping count
    uint32_t clipping_0 = 0;
    // second accelerometer clipping count
    uint32_t clipping_1 = 0;
    // third accelerometer clipping count
    uint32_t clipping_2 = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// This message can be requested by sending the MAV_CMD_GET_HOME_POSITION command.
// The position the system will return to and land on. The position is set automatically
// by the system during the takeoff in case it was not explicitely set by the operator
// before or after. The position the system will return to and land on. The global
// and local positions encode the position in the respective coordinate frames, while
// the q parameter encodes the orientation of the surface. Under normal conditions
// it describes the heading and terrain slope, which can be used by the aircraft to
// adjust the approach. The approach 3D vector describes the point to which the system
// should fly in normal flight mode and then perform a landing sequence along the
// vector.
class MavLinkHomePosition : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 242;
    MavLinkHomePosition() { msgid = kMessageId; }
    // Latitude (WGS84), in degrees * 1E7
    int32_t latitude = 0;
    // Longitude (WGS84, in degrees * 1E7
    int32_t longitude = 0;
    // Altitude (AMSL), in meters * 1000 (positive for up)
    int32_t altitude = 0;
    // Local X position of this position in the local coordinate frame
    float x = 0;
    // Local Y position of this position in the local coordinate frame
    float y = 0;
    // Local Z position of this position in the local coordinate frame
    float z = 0;
    // World to surface normal and heading transformation of the takeoff position.
    // Used to indicate the heading and slope of the ground
    float q[4] = { 0 };
    // Local X position of the end of the approach vector. Multicopters should set
    // this position based on their takeoff path. Grass-landing fixed wing aircraft
    // should set it the same way as multicopters. Runway-landing fixed wing aircraft
    // should set it to the opposite direction of the takeoff, assuming the takeoff
    // happened from the threshold / touchdown zone.
    float approach_x = 0;
    // Local Y position of the end of the approach vector. Multicopters should set
    // this position based on their takeoff path. Grass-landing fixed wing aircraft
    // should set it the same way as multicopters. Runway-landing fixed wing aircraft
    // should set it to the opposite direction of the takeoff, assuming the takeoff
    // happened from the threshold / touchdown zone.
    float approach_y = 0;
    // Local Z position of the end of the approach vector. Multicopters should set
    // this position based on their takeoff path. Grass-landing fixed wing aircraft
    // should set it the same way as multicopters. Runway-landing fixed wing aircraft
    // should set it to the opposite direction of the takeoff, assuming the takeoff
    // happened from the threshold / touchdown zone.
    float approach_z = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The position the system will return to and land on. The position is set automatically
// by the system during the takeoff in case it was not explicitely set by the operator
// before or after. The global and local positions encode the position in the respective
// coordinate frames, while the q parameter encodes the orientation of the surface.
// Under normal conditions it describes the heading and terrain slope, which can be
// used by the aircraft to adjust the approach. The approach 3D vector describes the
// point to which the system should fly in normal flight mode and then perform a landing
// sequence along the vector.
class MavLinkSetHomePosition : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 243;
    MavLinkSetHomePosition() { msgid = kMessageId; }
    // Latitude (WGS84), in degrees * 1E7
    int32_t latitude = 0;
    // Longitude (WGS84, in degrees * 1E7
    int32_t longitude = 0;
    // Altitude (AMSL), in meters * 1000 (positive for up)
    int32_t altitude = 0;
    // Local X position of this position in the local coordinate frame
    float x = 0;
    // Local Y position of this position in the local coordinate frame
    float y = 0;
    // Local Z position of this position in the local coordinate frame
    float z = 0;
    // World to surface normal and heading transformation of the takeoff position.
    // Used to indicate the heading and slope of the ground
    float q[4] = { 0 };
    // Local X position of the end of the approach vector. Multicopters should set
    // this position based on their takeoff path. Grass-landing fixed wing aircraft
    // should set it the same way as multicopters. Runway-landing fixed wing aircraft
    // should set it to the opposite direction of the takeoff, assuming the takeoff
    // happened from the threshold / touchdown zone.
    float approach_x = 0;
    // Local Y position of the end of the approach vector. Multicopters should set
    // this position based on their takeoff path. Grass-landing fixed wing aircraft
    // should set it the same way as multicopters. Runway-landing fixed wing aircraft
    // should set it to the opposite direction of the takeoff, assuming the takeoff
    // happened from the threshold / touchdown zone.
    float approach_y = 0;
    // Local Z position of the end of the approach vector. Multicopters should set
    // this position based on their takeoff path. Grass-landing fixed wing aircraft
    // should set it the same way as multicopters. Runway-landing fixed wing aircraft
    // should set it to the opposite direction of the takeoff, assuming the takeoff
    // happened from the threshold / touchdown zone.
    float approach_z = 0;
    // System ID.
    uint8_t target_system = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// This interface replaces DATA_STREAM
class MavLinkMessageInterval : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 244;
    MavLinkMessageInterval() { msgid = kMessageId; }
    // The interval between two messages, in microseconds. A value of -1 indicates
    // this stream is disabled, 0 indicates it is not available, > 0 indicates the
    // interval at which it is sent.
    int32_t interval_us = 0;
    // The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
    uint16_t message_id = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Provides state for additional features
class MavLinkExtendedSysState : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 245;
    MavLinkExtendedSysState() { msgid = kMessageId; }
    // The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is
    // not in VTOL configuration.
    uint8_t vtol_state = 0;
    // The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
    uint8_t landed_state = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// The location and information of an ADSB vehicle
class MavLinkAdsbVehicle : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 246;
    MavLinkAdsbVehicle() { msgid = kMessageId; }
    // ICAO address
    uint32_t ICAO_address = 0;
    // Latitude, expressed as degrees * 1E7
    int32_t lat = 0;
    // Longitude, expressed as degrees * 1E7
    int32_t lon = 0;
    // Altitude(ASL) in millimeters
    int32_t altitude = 0;
    // Course over ground in centidegrees
    uint16_t heading = 0;
    // The horizontal velocity in centimeters/second
    uint16_t hor_velocity = 0;
    // The vertical velocity in centimeters/second, positive is up
    int16_t ver_velocity = 0;
    // Flags to indicate various statuses including valid data fields
    uint16_t flags = 0;
    // Squawk code
    uint16_t squawk = 0;
    // Type from ADSB_ALTITUDE_TYPE enum
    uint8_t altitude_type = 0;
    // The callsign, 8+null
    char callsign[9] = { 0 };
    // Type from ADSB_EMITTER_TYPE enum
    uint8_t emitter_type = 0;
    // Time since last communication in seconds
    uint8_t tslc = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Information about a potential collision
class MavLinkCollision : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 247;
    MavLinkCollision() { msgid = kMessageId; }
    // Unique identifier, domain based on src field
    uint32_t id = 0;
    // Estimated time until collision occurs (seconds)
    float time_to_minimum_delta = 0;
    // Closest vertical distance in meters between vehicle and object
    float altitude_minimum_delta = 0;
    // Closest horizontal distance in meteres between vehicle and object
    float horizontal_minimum_delta = 0;
    // Collision data source
    uint8_t src = 0;
    // Action that is being taken to avoid this collision
    uint8_t action = 0;
    // How concerned the aircraft is about this collision
    uint8_t threat_level = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Message implementing parts of the V2 payload specs in V1 frames for transitional
// support.
class MavLinkV2Extension : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 248;
    MavLinkV2Extension() { msgid = kMessageId; }
    // A code that identifies the software component that understands this message
    // (analogous to usb device classes or mime type strings). If this code is less
    // than 32768, it is considered a 'registered' protocol extension and the corresponding
    // entry should be added to https://github.com/mavlink/mavlink/extension-message-ids.xml.
    // Software creators can register blocks of message IDs as needed (useful for
    // GCS specific metadata, etc...). Message_types greater than 32767 are considered
    // local experiments and should not be checked in to any widely distributed codebase.
    uint16_t message_type = 0;
    // Network ID (0 for broadcast)
    uint8_t target_network = 0;
    // System ID (0 for broadcast)
    uint8_t target_system = 0;
    // Component ID (0 for broadcast)
    uint8_t target_component = 0;
    // Variable length payload. The length is defined by the remaining message length
    // when subtracting the header and other fields. The entire content of this block
    // is opaque unless you understand any the encoding message_type. The particular
    // encoding used can be extension specific and might not always be documented
    // as part of the mavlink specification.
    uint8_t payload[249] = { 0 };
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Send raw controller memory. The use of this message is discouraged for normal packets,
// but a quite efficient way for testing new messages and getting experimental debug
// output.
class MavLinkMemoryVect : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 249;
    MavLinkMemoryVect() { msgid = kMessageId; }
    // Starting address of the debug variables
    uint16_t address = 0;
    // Version code of the type variable. 0=unknown, type ignored and assumed int16_t.
    // 1=as below
    uint8_t ver = 0;
    // Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t,
    // 2=16 x Q15, 3=16 x 1Q14
    uint8_t type = 0;
    // Memory contents at specified address
    int8_t value[32] = { 0 };
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

class MavLinkDebugVect : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 250;
    MavLinkDebugVect() { msgid = kMessageId; }
    // Timestamp
    uint64_t time_usec = 0;
    // x
    float x = 0;
    // y
    float y = 0;
    // z
    float z = 0;
    // Name
    char name[10] = { 0 };
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Send a key-value pair as float. The use of this message is discouraged for normal
// packets, but a quite efficient way for testing new messages and getting experimental
// debug output.
class MavLinkNamedValueFloat : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 251;
    MavLinkNamedValueFloat() { msgid = kMessageId; }
    // Timestamp (milliseconds since system boot)
    uint32_t time_boot_ms = 0;
    // Floating point value
    float value = 0;
    // Name of the debug variable
    char name[10] = { 0 };
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Send a key-value pair as integer. The use of this message is discouraged for normal
// packets, but a quite efficient way for testing new messages and getting experimental
// debug output.
class MavLinkNamedValueInt : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 252;
    MavLinkNamedValueInt() { msgid = kMessageId; }
    // Timestamp (milliseconds since system boot)
    uint32_t time_boot_ms = 0;
    // Signed integer value
    int32_t value = 0;
    // Name of the debug variable
    char name[10] = { 0 };
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Status text message. These messages are printed in yellow in the COMM console of
// QGroundControl. WARNING: They consume quite some bandwidth, so use only for important
// status and error messages. If implemented wisely, these messages are buffered on
// the MCU and sent only at a limited rate (e.g. 10 Hz).
class MavLinkStatustext : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 253;
    MavLinkStatustext() { msgid = kMessageId; }
    // Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
    uint8_t severity = 0;
    // Status text message, without null termination character
    char text[50] = { 0 };
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Send a debug value. The index is used to discriminate between values. These values
// show up in the plot of QGroundControl as DEBUG N.
class MavLinkDebug : public MavLinkMessageBase {
public:
    const static uint8_t kMessageId = 254;
    MavLinkDebug() { msgid = kMessageId; }
    // Timestamp (milliseconds since system boot)
    uint32_t time_boot_ms = 0;
    // DEBUG value
    float value = 0;
    // index of debug variable
    uint8_t ind = 0;
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Version and capability of protocol version. This message is the response to REQUEST_PROTOCOL_VERSION 
// and is used as part of the handshaking to establish which MAVLink version should be used on the network. 
// Every node should respond to REQUEST_PROTOCOL_VERSION to enable the handshaking. Library implementers 
// should consider adding this into the default decoding state machine to allow the protocol core to respond directly.
class MavLinkProtocolVersion : public MavLinkMessageBase {
public:
    const static uint32_t kMessageId = 300;
    MavLinkProtocolVersion() { msgid = kMessageId; }
    // Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc.
    uint16_t version = 0;
    uint16_t min_version = 0;
    uint16_t max_version = 0;
    uint8_t spec_version_hash[8] = { 0 };
    uint8_t library_version_hash[8] = { 0 };
    virtual std::string toJSon();
protected:
    virtual int pack(char* buffer) const;
    virtual int unpack(const char* buffer);
};

// Navigate to MISSION.
class MavCmdNavWaypoint : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 16;
    MavCmdNavWaypoint() { command = kCommandId; }
    // Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION
    // for rotary wing)
    float HoldTimeDecimal = 0;
    // Acceptance radius in meters (if the sphere with this radius is hit, the MISSION
    // counts as reached)
    float AcceptanceRadiusMeters = 0;
    // 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value
    // for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory
    // control.
    float p0ToPass = 0;
    // Desired yaw angle at MISSION (rotary wing)
    float DesiredYawAngle = 0;
    // Latitude
    float Latitude = 0;
    // Longitude
    float Longitude = 0;
    // Altitude
    float Altitude = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Loiter around this MISSION an unlimited amount of time
class MavCmdNavLoiterUnlim : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 17;
    MavCmdNavLoiterUnlim() { command = kCommandId; }
    // Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise
    float RadiusAroundMission = 0;
    // Desired yaw angle.
    float DesiredYawAngle = 0;
    // Latitude
    float Latitude = 0;
    // Longitude
    float Longitude = 0;
    // Altitude
    float Altitude = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Loiter around this MISSION for X turns
class MavCmdNavLoiterTurns : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 18;
    MavCmdNavLoiterTurns() { command = kCommandId; }
    // Turns
    float Turns = 0;
    // Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise
    float RadiusAroundMission = 0;
    // Forward moving aircraft this sets exit xtrack location: 0 for center of loiter
    // wp, 1 for exit location. Else, this is desired yaw angle
    float ExitXtrackLocation = 0;
    // Latitude
    float Latitude = 0;
    // Longitude
    float Longitude = 0;
    // Altitude
    float Altitude = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Loiter around this MISSION for X seconds
class MavCmdNavLoiterTime : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 19;
    MavCmdNavLoiterTime() { command = kCommandId; }
    // Seconds (decimal)
    float Seconds = 0;
    // Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise
    float RadiusAroundMission = 0;
    // Forward moving aircraft this sets exit xtrack location: 0 for center of loiter
    // wp, 1 for exit location. Else, this is desired yaw angle
    float ExitXtrackLocation = 0;
    // Latitude
    float Latitude = 0;
    // Longitude
    float Longitude = 0;
    // Altitude
    float Altitude = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Return to launch location
class MavCmdNavReturnToLaunch : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 20;
    MavCmdNavReturnToLaunch() { command = kCommandId; }
protected:
    virtual void pack();
    virtual void unpack();
};
// Land at location
class MavCmdNavLand : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 21;
    MavCmdNavLand() { command = kCommandId; }
    // Abort Alt
    float AbortAlt = 0;
    // Desired yaw angle
    float DesiredYawAngle = 0;
    // Latitude
    float Latitude = 0;
    // Longitude
    float Longitude = 0;
    // Altitude
    float Altitude = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Takeoff from ground / hand
class MavCmdNavTakeoff : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 22;
    MavCmdNavTakeoff() { command = kCommandId; }
    // Minimum pitch (if airspeed sensor present), desired pitch without sensor
    float MinimumPitch = 0;
    // Yaw angle (if magnetometer present), ignored without magnetometer
    float YawAngle = 0;
    // Latitude
    float Latitude = 0;
    // Longitude
    float Longitude = 0;
    // Altitude
    float Altitude = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Land at local position (local frame only)
class MavCmdNavLandLocal : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 23;
    MavCmdNavLandLocal() { command = kCommandId; }
    // Landing target number (if available)
    float LandingTargetNumber = 0;
    // Maximum accepted offset from desired landing position [m] - computed magnitude
    // from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum
    // accepted distance between the desired landing position and the position where
    // the vehicle is about to land
    float MaximumAcceptedOffset = 0;
    // Landing descend rate [ms^-1]
    float LandingDescendRate = 0;
    // Desired yaw angle [rad]
    float DesiredYawAngle = 0;
    // Y-axis position [m]
    float Y = 0;
    // X-axis position [m]
    float X = 0;
    // Z-axis / ground level position [m]
    float Z = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Takeoff from local position (local frame only)
class MavCmdNavTakeoffLocal : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 24;
    MavCmdNavTakeoffLocal() { command = kCommandId; }
    // Minimum pitch (if airspeed sensor present), desired pitch without sensor [rad]
    float MinimumPitch = 0;
    // Takeoff ascend rate [ms^-1]
    float TakeoffAscendRate = 0;
    // Yaw angle [rad] (if magnetometer or another yaw estimation source present),
    // ignored without one of these
    float YawAngle = 0;
    // Y-axis position [m]
    float Y = 0;
    // X-axis position [m]
    float X = 0;
    // Z-axis position [m]
    float Z = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Vehicle following, i.e. this waypoint represents the position of a moving vehicle
class MavCmdNavFollow : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 25;
    MavCmdNavFollow() { command = kCommandId; }
    // Following logic to use (e.g. loitering or sinusoidal following) - depends on
    // specific autopilot implementation
    float FollowingLogicTo = 0;
    // Ground speed of vehicle to be followed
    float GroundSpeedOf = 0;
    // Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise
    float RadiusAroundMission = 0;
    // Desired yaw angle.
    float DesiredYawAngle = 0;
    // Latitude
    float Latitude = 0;
    // Longitude
    float Longitude = 0;
    // Altitude
    float Altitude = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Continue on the current course and climb/descend to specified altitude. When the
// altitude is reached continue to the next command (i.e., don't proceed to the next
// command until the desired altitude is reached.
class MavCmdNavContinueAndChangeAlt : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 30;
    MavCmdNavContinueAndChangeAlt() { command = kCommandId; }
    // Climb or Descend (0 = Neutral, command completes when within 5m of this command's
    // altitude, 1 = Climbing, command completes when at or above this command's altitude,
    // 2 = Descending, command completes when at or below this command's altitude.
    float ClimbOrDescend = 0;
    // Desired altitude in meters
    float DesiredAltitudeMeters = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Begin loiter at the specified Latitude and Longitude. If Lat=Lon=0, then loiter
// at the current position. Don't consider the navigation command complete (don't
// leave loiter) until the altitude has been reached. Additionally, if the Heading
// Required parameter is non-zero the aircraft will not leave the loiter until heading
// toward the next waypoint.
class MavCmdNavLoiterToAlt : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 31;
    MavCmdNavLoiterToAlt() { command = kCommandId; }
    // Heading Required (0 = False)
    float HeadingRequired = 0;
    // Radius in meters. If positive loiter clockwise, negative counter-clockwise,
    // 0 means no change to standard loiter.
    float RadiusMeters = 0;
    // Forward moving aircraft this sets exit xtrack location: 0 for center of loiter
    // wp, 1 for exit location
    float ExitXtrackLocation = 0;
    // Latitude
    float Latitude = 0;
    // Longitude
    float Longitude = 0;
    // Altitude
    float Altitude = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Being following a target
class MavCmdDoFollow : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 32;
    MavCmdDoFollow() { command = kCommandId; }
    // System ID (the system ID of the FOLLOW_TARGET beacon). Send 0 to disable follow-me
    // and return to the default position hold mode
    float SystemId = 0;
    // altitude flag: 0: Keep current altitude, 1: keep altitude difference to target,
    // 2: go to a fixed altitude above home
    float AltitudeFlag = 0;
    // altitude
    float Altitude = 0;
    // TTL in seconds in which the MAV should go to the default position hold mode
    // after a message rx timeout
    float TtlSecondsWhich = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Reposition the MAV after a follow target command has been sent
class MavCmdDoFollowReposition : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 33;
    MavCmdDoFollowReposition() { command = kCommandId; }
    // Camera q1 (where 0 is on the ray from the camera to the tracking device)
    float CameraQ1 = 0;
    // Camera q2
    float CameraQ2 = 0;
    // Camera q3
    float CameraQ3 = 0;
    // Camera q4
    float CameraQ4 = 0;
    // altitude offset from target (m)
    float AltitudeOffsetTarget = 0;
    // X offset from target (m)
    float XOffsetTarget = 0;
    // Y offset from target (m)
    float YOffsetTarget = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Sets the region of interest (ROI) for a sensor set or the vehicle itself. This
// can then be used by the vehicles control system to control the vehicle attitude
// and the attitude of various sensors such as cameras.
class MavCmdNavRoi : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 80;
    MavCmdNavRoi() { command = kCommandId; }
    // Region of intereset mode. (see MAV_ROI enum)
    float RegionOfIntereset = 0;
    // MISSION index/ target ID. (see MAV_ROI enum)
    float MissionIndexTarget = 0;
    // ROI index (allows a vehicle to manage multiple ROI's)
    float RoiIndex = 0;
    // x the location of the fixed ROI (see MAV_FRAME)
    float XLocationOf = 0;
    // y
    float Y = 0;
    // z
    float Z = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Control autonomous path planning on the MAV.
class MavCmdNavPathplanning : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 81;
    MavCmdNavPathplanning() { command = kCommandId; }
    // 0: Disable local obstacle avoidance / local path planning (without resetting
    // map), 1: Enable local path planning, 2: Enable and reset local path planning
    float p0 = 0;
    // 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable
    // and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy
    // grid
    float p02 = 0;
    // Yaw angle at goal, in compass degrees, [0..360]
    float YawAngleAt = 0;
    // Latitude/X of goal
    float LatitudexOfGoal = 0;
    // Longitude/Y of goal
    float LongitudeyOfGoal = 0;
    // Altitude/Z of goal
    float AltitudezOfGoal = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Navigate to MISSION using a spline path.
class MavCmdNavSplineWaypoint : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 82;
    MavCmdNavSplineWaypoint() { command = kCommandId; }
    // Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION
    // for rotary wing)
    float HoldTimeDecimal = 0;
    // Latitude/X of goal
    float LatitudexOfGoal = 0;
    // Longitude/Y of goal
    float LongitudeyOfGoal = 0;
    // Altitude/Z of goal
    float AltitudezOfGoal = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Takeoff from ground using VTOL mode
class MavCmdNavVtolTakeoff : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 84;
    MavCmdNavVtolTakeoff() { command = kCommandId; }
    // Yaw angle in degrees
    float YawAngleDegrees = 0;
    // Latitude
    float Latitude = 0;
    // Longitude
    float Longitude = 0;
    // Altitude
    float Altitude = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Land using VTOL mode
class MavCmdNavVtolLand : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 85;
    MavCmdNavVtolLand() { command = kCommandId; }
    // Yaw angle in degrees
    float YawAngleDegrees = 0;
    // Latitude
    float Latitude = 0;
    // Longitude
    float Longitude = 0;
    // Altitude
    float Altitude = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// hand control over to an external controller
class MavCmdNavGuidedEnable : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 92;
    MavCmdNavGuidedEnable() { command = kCommandId; }
    // On / Off (> 0.5f on)
    float OnOff = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Delay the next navigation command a number of seconds or until a specified time
class MavCmdNavDelay : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 93;
    MavCmdNavDelay() { command = kCommandId; }
    // Delay in seconds (decimal, -1 to enable time-of-day fields)
    float DelaySeconds = 0;
    // hour (24h format, UTC, -1 to ignore)
    float Hour = 0;
    // minute (24h format, UTC, -1 to ignore)
    float Minute = 0;
    // second (24h format, UTC)
    float Second = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// NOP - This command is only used to mark the upper limit of the NAV/ACTION commands
// in the enumeration
class MavCmdNavLast : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 95;
    MavCmdNavLast() { command = kCommandId; }
protected:
    virtual void pack();
    virtual void unpack();
};
// Delay mission state machine.
class MavCmdConditionDelay : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 112;
    MavCmdConditionDelay() { command = kCommandId; }
    // Delay in seconds (decimal)
    float DelaySeconds = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Ascend/descend at rate. Delay mission state machine until desired altitude reached.
class MavCmdConditionChangeAlt : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 113;
    MavCmdConditionChangeAlt() { command = kCommandId; }
    // Descent / Ascend rate (m/s)
    float DescentAscend = 0;
    // Finish Altitude
    float FinishAltitude = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Delay mission state machine until within desired distance of next NAV point.
class MavCmdConditionDistance : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 114;
    MavCmdConditionDistance() { command = kCommandId; }
    // Distance (meters)
    float Distance = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Reach a certain target angle.
class MavCmdConditionYaw : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 115;
    MavCmdConditionYaw() { command = kCommandId; }
    // target angle: [0-360], 0 is north
    float TargetAngle = 0;
    // speed during yaw change:[deg per second]
    float SpeedDuringYaw = 0;
    // direction: negative: counter clockwise, positive: clockwise [-1,1]
    float Direction = 0;
    // relative offset or absolute angle: [ 1,0]
    float RelativeOffsetOr = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// NOP - This command is only used to mark the upper limit of the CONDITION commands
// in the enumeration
class MavCmdConditionLast : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 159;
    MavCmdConditionLast() { command = kCommandId; }
protected:
    virtual void pack();
    virtual void unpack();
};
// Set system mode.
class MavCmdDoSetMode : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 176;
    MavCmdDoSetMode() { command = kCommandId; }
    // Mode, as defined by ENUM MAV_MODE
    float Mode = 0;
    // Custom mode - this is system specific, please refer to the individual autopilot
    // specifications for details.
    float CustomMode = 0;
    // Custom sub mode - this is system specific, please refer to the individual autopilot
    // specifications for details.
    float CustomSubMode = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Jump to the desired command in the mission list. Repeat this action only the specified
// number of times
class MavCmdDoJump : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 177;
    MavCmdDoJump() { command = kCommandId; }
    // Sequence number
    float SequenceNumber = 0;
    // Repeat count
    float RepeatCount = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Change speed and/or throttle set points.
class MavCmdDoChangeSpeed : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 178;
    MavCmdDoChangeSpeed() { command = kCommandId; }
    // Speed type (0=Airspeed, 1=Ground Speed)
    float SpeedType = 0;
    // Speed (m/s, -1 indicates no change)
    float Speed = 0;
    // Throttle ( Percent, -1 indicates no change)
    float Throttle = 0;
    // absolute or relative [0,1]
    float AbsoluteOrRelative = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Changes the home location either to the current location or a specified location.
class MavCmdDoSetHome : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 179;
    MavCmdDoSetHome() { command = kCommandId; }
    // Use current (1=use current location, 0=use specified location)
    float UseCurrent = 0;
    // Latitude
    float Latitude = 0;
    // Longitude
    float Longitude = 0;
    // Altitude
    float Altitude = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Set a system parameter. Caution! Use of this command requires knowledge of the
// numeric enumeration value of the parameter.
class MavCmdDoSetParameter : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 180;
    MavCmdDoSetParameter() { command = kCommandId; }
    // Parameter number
    float ParameterNumber = 0;
    // Parameter value
    float ParameterValue = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Set a relay to a condition.
class MavCmdDoSetRelay : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 181;
    MavCmdDoSetRelay() { command = kCommandId; }
    // Relay number
    float RelayNumber = 0;
    // Setting (1=on, 0=off, others possible depending on system hardware)
    float Setting = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Cycle a relay on and off for a desired number of cyles with a desired period.
class MavCmdDoRepeatRelay : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 182;
    MavCmdDoRepeatRelay() { command = kCommandId; }
    // Relay number
    float RelayNumber = 0;
    // Cycle count
    float CycleCount = 0;
    // Cycle time (seconds, decimal)
    float CycleTime = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Set a servo to a desired PWM value.
class MavCmdDoSetServo : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 183;
    MavCmdDoSetServo() { command = kCommandId; }
    // Servo number
    float ServoNumber = 0;
    // PWM (microseconds, 1000 to 2000 typical)
    float Pwm = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Cycle a between its nominal setting and a desired PWM for a desired number of cycles
// with a desired period.
class MavCmdDoRepeatServo : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 184;
    MavCmdDoRepeatServo() { command = kCommandId; }
    // Servo number
    float ServoNumber = 0;
    // PWM (microseconds, 1000 to 2000 typical)
    float Pwm = 0;
    // Cycle count
    float CycleCount = 0;
    // Cycle time (seconds)
    float CycleTime = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Terminate flight immediately
class MavCmdDoFlighttermination : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 185;
    MavCmdDoFlighttermination() { command = kCommandId; }
    // Flight termination activated if > 0.5
    float FlightTerminationActivated = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Change altitude set point.
class MavCmdDoChangeAltitude : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 186;
    MavCmdDoChangeAltitude() { command = kCommandId; }
    // Altitude in meters
    float AltitudeMeters = 0;
    // Mav frame of new altitude (see MAV_FRAME)
    float MavFrameOf = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Mission command to perform a landing. This is used as a marker in a mission to
// tell the autopilot where a sequence of mission items that represents a landing
// starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case
// the nearest (geographically) landing sequence in the mission will be used. The
// Latitude/Longitude is optional, and may be set to 0/0 if not needed. If specified
// then it will be used to help find the closest landing sequence.
class MavCmdDoLandStart : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 189;
    MavCmdDoLandStart() { command = kCommandId; }
    // Latitude
    float Latitude = 0;
    // Longitude
    float Longitude = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Mission command to perform a landing from a rally point.
class MavCmdDoRallyLand : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 190;
    MavCmdDoRallyLand() { command = kCommandId; }
    // Break altitude (meters)
    float BreakAltitude = 0;
    // Landing speed (m/s)
    float LandingSpeed = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Mission command to safely abort an autonmous landing.
class MavCmdDoGoAround : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 191;
    MavCmdDoGoAround() { command = kCommandId; }
    // Altitude (meters)
    float Altitude = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Reposition the vehicle to a specific WGS84 global position.
class MavCmdDoReposition : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 192;
    MavCmdDoReposition() { command = kCommandId; }
    // Ground speed, less than 0 (-1) for default
    float GroundSpeed = 0;
    // Bitmask of option flags, see the MAV_DO_REPOSITION_FLAGS enum.
    float BitmaskOfOption = 0;
    // Yaw heading, NaN for unchanged. For planes indicates loiter direction (0: clockwise,
    // 1: counter clockwise)
    float YawHeading = 0;
    // Latitude (deg * 1E7)
    float Latitude = 0;
    // Longitude (deg * 1E7)
    float Longitude = 0;
    // Altitude (meters)
    float Altitude = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// If in a GPS controlled position mode, hold the current position or continue.
class MavCmdDoPauseContinue : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 193;
    MavCmdDoPauseContinue() { command = kCommandId; }
    // 0: Pause current mission or reposition command, hold current position. 1: Continue
    // mission. A VTOL capable vehicle should enter hover mode (multicopter and VTOL
    // planes). A plane should loiter with the default loiter radius.
    float p0 = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Set moving direction to forward or reverse.
class MavCmdDoSetReverse : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 194;
    MavCmdDoSetReverse() { command = kCommandId; }
    // Direction (0=Forward, 1=Reverse)
    float Direction = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Control onboard camera system.
class MavCmdDoControlVideo : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 200;
    MavCmdDoControlVideo() { command = kCommandId; }
    // Camera ID (-1 for all)
    float CameraId = 0;
    // Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw
    float Transmission = 0;
    // Transmission mode: 0: video stream, >0: single images every n seconds (decimal)
    float TransmissionMode = 0;
    // Recording: 0: disabled, 1: enabled compressed, 2: enabled raw
    float Recording = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Sets the region of interest (ROI) for a sensor set or the vehicle itself. This
// can then be used by the vehicles control system to control the vehicle attitude
// and the attitude of various sensors such as cameras.
class MavCmdDoSetRoi : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 201;
    MavCmdDoSetRoi() { command = kCommandId; }
    // Region of intereset mode. (see MAV_ROI enum)
    float RegionOfIntereset = 0;
    // MISSION index/ target ID. (see MAV_ROI enum)
    float MissionIndexTarget = 0;
    // ROI index (allows a vehicle to manage multiple ROI's)
    float RoiIndex = 0;
    // x the location of the fixed ROI (see MAV_FRAME)
    float XLocationOf = 0;
    // y
    float Y = 0;
    // z
    float Z = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Mission command to configure an on-board camera controller system.
class MavCmdDoDigicamConfigure : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 202;
    MavCmdDoDigicamConfigure() { command = kCommandId; }
    // Modes: P, TV, AV, M, Etc
    float Modes = 0;
    // Shutter speed: Divisor number for one second
    float ShutterSpeed = 0;
    // Aperture: F stop number
    float Aperture = 0;
    // ISO number e.g. 80, 100, 200, Etc
    float IsoNumberE = 0;
    // Exposure type enumerator
    float ExposureTypeEnumerator = 0;
    // Command Identity
    float CommandIdentity = 0;
    // Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)
    float MainEngineCut = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Mission command to control an on-board camera controller system.
class MavCmdDoDigicamControl : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 203;
    MavCmdDoDigicamControl() { command = kCommandId; }
    // Session control e.g. show/hide lens
    float SessionControlE = 0;
    // Zoom's absolute position
    float ZoomsAbsolutePosition = 0;
    // Zooming step value to offset zoom from the current position
    float ZoomingStepValue = 0;
    // Focus Locking, Unlocking or Re-locking
    float FocusLocking = 0;
    // Shooting Command
    float ShootingCommand = 0;
    // Command Identity
    float CommandIdentity = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Mission command to configure a camera or antenna mount
class MavCmdDoMountConfigure : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 204;
    MavCmdDoMountConfigure() { command = kCommandId; }
    // Mount operation mode (see MAV_MOUNT_MODE enum)
    float MountOperationMode = 0;
    // stabilize roll? (1 = yes, 0 = no)
    float StabilizeRoll = 0;
    // stabilize pitch? (1 = yes, 0 = no)
    float StabilizePitch = 0;
    // stabilize yaw? (1 = yes, 0 = no)
    float StabilizeYaw = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Mission command to control a camera or antenna mount
class MavCmdDoMountControl : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 205;
    MavCmdDoMountControl() { command = kCommandId; }
    // pitch (WIP: DEPRECATED: or lat in degrees) depending on mount mode.
    float Pitch = 0;
    // roll (WIP: DEPRECATED: or lon in degrees) depending on mount mode.
    float Roll = 0;
    // yaw (WIP: DEPRECATED: or alt in meters) depending on mount mode.
    float Yaw = 0;
    // WIP: alt in meters depending on mount mode.
    float Wip = 0;
    // WIP: latitude in degrees * 1E7, set if appropriate mount mode.
    float Wip2 = 0;
    // WIP: longitude in degrees * 1E7, set if appropriate mount mode.
    float Wip3 = 0;
    // MAV_MOUNT_MODE enum value
    float MavMountModeEnumValue = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Mission command to set CAM_TRIGG_DIST for this flight
class MavCmdDoSetCamTriggDist : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 206;
    MavCmdDoSetCamTriggDist() { command = kCommandId; }
    // Camera trigger distance (meters)
    float CameraTriggerDistance = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Mission command to enable the geofence
class MavCmdDoFenceEnable : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 207;
    MavCmdDoFenceEnable() { command = kCommandId; }
    // enable? (0=disable, 1=enable, 2=disable_floor_only)
    float Enable = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Mission command to trigger a parachute
class MavCmdDoParachute : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 208;
    MavCmdDoParachute() { command = kCommandId; }
    // action (0=disable, 1=enable, 2=release, for some systems see PARACHUTE_ACTION
    // enum, not in general message set.)
    float Action = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Mission command to perform motor test
class MavCmdDoMotorTest : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 209;
    MavCmdDoMotorTest() { command = kCommandId; }
    // motor sequence number (a number from 1 to max number of motors on the vehicle)
    float MotorSequenceNumber = 0;
    // throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through.
    // See MOTOR_TEST_THROTTLE_TYPE enum)
    float ThrottleType = 0;
    // throttle
    float Throttle = 0;
    // timeout (in seconds)
    float Timeout = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Change to/from inverted flight
class MavCmdDoInvertedFlight : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 210;
    MavCmdDoInvertedFlight() { command = kCommandId; }
    // inverted (0=normal, 1=inverted)
    float Inverted = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Sets a desired vehicle turn angle and thrust change
class MavCmdDoSetPositionYawThrust : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 213;
    MavCmdDoSetPositionYawThrust() { command = kCommandId; }
    // yaw angle to adjust steering by in centidegress
    float YawAngleTo = 0;
    // Thrust - normalized to -2 .. 2
    float Thrust = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Mission command to control a camera or antenna mount, using a quaternion as reference.
class MavCmdDoMountControlQuat : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 220;
    MavCmdDoMountControlQuat() { command = kCommandId; }
    // q1 - quaternion param #1, w (1 in null-rotation)
    float Q1 = 0;
    // q2 - quaternion param #2, x (0 in null-rotation)
    float Q2 = 0;
    // q3 - quaternion param #3, y (0 in null-rotation)
    float Q3 = 0;
    // q4 - quaternion param #4, z (0 in null-rotation)
    float Q4 = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// set id of master controller
class MavCmdDoGuidedMaster : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 221;
    MavCmdDoGuidedMaster() { command = kCommandId; }
    // System ID
    float SystemId = 0;
    // Component ID
    float ComponentId = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// set limits for external control
class MavCmdDoGuidedLimits : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 222;
    MavCmdDoGuidedLimits() { command = kCommandId; }
    // timeout - maximum time (in seconds) that external controller will be allowed
    // to control vehicle. 0 means no timeout
    float Timeout = 0;
    // absolute altitude min (in meters, AMSL) - if vehicle moves below this alt,
    // the command will be aborted and the mission will continue. 0 means no lower
    // altitude limit
    float AbsoluteAltitudeMin = 0;
    // absolute altitude max (in meters)- if vehicle moves above this alt, the command
    // will be aborted and the mission will continue. 0 means no upper altitude limit
    float AbsoluteAltitudeMax = 0;
    // horizontal move limit (in meters, AMSL) - if vehicle moves more than this distance
    // from it's location at the moment the command was executed, the command will
    // be aborted and the mission will continue. 0 means no horizontal altitude limit
    float HorizontalMoveLimit = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Control vehicle engine. This is interpreted by the vehicles engine controller to
// change the target engine state. It is intended for vehicles with internal combustion
// engines
class MavCmdDoEngineControl : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 223;
    MavCmdDoEngineControl() { command = kCommandId; }
    // 0: Stop engine, 1:Start Engine
    float p0 = 0;
    // 0: Warm start, 1:Cold start. Controls use of choke where applicable
    float p02 = 0;
    // Height delay (meters). This is for commanding engine start only after the vehicle
    // has gained the specified height. Used in VTOL vehicles during takeoff to start
    // engine after the aircraft is off the ground. Zero for no delay.
    float HeightDelay = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// NOP - This command is only used to mark the upper limit of the DO commands in the
// enumeration
class MavCmdDoLast : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 240;
    MavCmdDoLast() { command = kCommandId; }
protected:
    virtual void pack();
    virtual void unpack();
};
// Trigger calibration. This command will be only accepted if in pre-flight mode.
class MavCmdPreflightCalibration : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 241;
    MavCmdPreflightCalibration() { command = kCommandId; }
    // Gyro calibration: 0: no, 1: yes
    float GyroCalibration = 0;
    // Magnetometer calibration: 0: no, 1: yes
    float MagnetometerCalibration = 0;
    // Ground pressure: 0: no, 1: yes
    float GroundPressure = 0;
    // Radio calibration: 0: no, 1: yes
    float RadioCalibration = 0;
    // Accelerometer calibration: 0: no, 1: yes
    float AccelerometerCalibration = 0;
    // Compass/Motor interference calibration: 0: no, 1: yes
    float CompassmotorInterferenceCalibration = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Set sensor offsets. This command will be only accepted if in pre-flight mode.
class MavCmdPreflightSetSensorOffsets : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 242;
    MavCmdPreflightSetSensorOffsets() { command = kCommandId; }
    // Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer,
    // 3: barometer, 4: optical flow, 5: second magnetometer, 6: third magnetometer
    float SensorToAdjust = 0;
    // X axis offset (or generic dimension 1), in the sensor's raw units
    float XAxisOffset = 0;
    // Y axis offset (or generic dimension 2), in the sensor's raw units
    float YAxisOffset = 0;
    // Z axis offset (or generic dimension 3), in the sensor's raw units
    float ZAxisOffset = 0;
    // Generic dimension 4, in the sensor's raw units
    float GenericDimension4 = 0;
    // Generic dimension 5, in the sensor's raw units
    float GenericDimension5 = 0;
    // Generic dimension 6, in the sensor's raw units
    float GenericDimension6 = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Trigger UAVCAN config. This command will be only accepted if in pre-flight mode.
class MavCmdPreflightUavcan : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 243;
    MavCmdPreflightUavcan() { command = kCommandId; }
    // 1: Trigger actuator ID assignment and direction mapping.
    float p1 = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Request storage of different parameter values and logs. This command will be only
// accepted if in pre-flight mode.
class MavCmdPreflightStorage : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 245;
    MavCmdPreflightStorage() { command = kCommandId; }
    // Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM,
    // 2: Reset to defaults
    float ParameterStorage = 0;
    // Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM,
    // 2: Reset to defaults
    float MissionStorage = 0;
    // Onboard logging: 0: Ignore, 1: Start default rate logging, -1: Stop logging,
    // > 1: start logging with rate of param 3 in Hz (e.g. set to 1000 for 1000 Hz
    // logging)
    float OnboardLogging = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Request the reboot or shutdown of system components.
class MavCmdPreflightRebootShutdown : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 246;
    MavCmdPreflightRebootShutdown() { command = kCommandId; }
    // 0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3:
    // Reboot autopilot and keep it in the bootloader until upgraded.
    float p0 = 0;
    // 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown
    // onboard computer, 3: Reboot onboard computer and keep it in the bootloader
    // until upgraded.
    float p02 = 0;
    // WIP: 0: Do nothing for camera, 1: Reboot onboard camera, 2: Shutdown onboard
    // camera, 3: Reboot onboard camera and keep it in the bootloader until upgraded
    float Wip = 0;
    // WIP: 0: Do nothing for mount (e.g. gimbal), 1: Reboot mount, 2: Shutdown mount,
    // 3: Reboot mount and keep it in the bootloader until upgraded
    float Wip2 = 0;
    // WIP: ID (e.g. camera ID -1 for all IDs)
    float Wip3 = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Hold / continue the current action
class MavCmdOverrideGoto : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 252;
    MavCmdOverrideGoto() { command = kCommandId; }
    // MAV_GOTO_DO_HOLD: hold MAV_GOTO_DO_CONTINUE: continue with next item in mission
    // plan
    float MavGotoDoHold = 0;
    // MAV_GOTO_HOLD_AT_CURRENT_POSITION: Hold at current position MAV_GOTO_HOLD_AT_SPECIFIED_POSITION:
    // hold at specified position
    float MavGotoHoldAtCurrentPosition = 0;
    // MAV_FRAME coordinate frame of hold point
    float MavFrameCoordinateFrame = 0;
    // Desired yaw angle in degrees
    float DesiredYawAngle = 0;
    // Latitude / X position
    float LatitudeX = 0;
    // Longitude / Y position
    float LongitudeY = 0;
    // Altitude / Z position
    float AltitudeZ = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// start running a mission
class MavCmdMissionStart : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 300;
    MavCmdMissionStart() { command = kCommandId; }
    // first_item: the first mission item to run
    float FirstItem = 0;
    // last_item: the last mission item to run (after this item is run, the mission
    // ends)
    float LastItem = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Arms / Disarms a component
class MavCmdComponentArmDisarm : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 400;
    MavCmdComponentArmDisarm() { command = kCommandId; }
    // 1 to arm, 0 to disarm
    float p1ToArm = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Request the home position from the vehicle.
class MavCmdGetHomePosition : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 410;
    MavCmdGetHomePosition() { command = kCommandId; }
protected:
    virtual void pack();
    virtual void unpack();
};
// Starts receiver pairing
class MavCmdStartRxPair : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 500;
    MavCmdStartRxPair() { command = kCommandId; }
    // 0:Spektrum
    float p0 = 0;
    // 0:Spektrum DSM2, 1:Spektrum DSMX
    float p02 = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Request the interval between messages for a particular MAVLink message ID
class MavCmdGetMessageInterval : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 510;
    MavCmdGetMessageInterval() { command = kCommandId; }
    // The MAVLink message ID
    float TheMavlinkMessage = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Request the interval between messages for a particular MAVLink message ID. This
// interface replaces REQUEST_DATA_STREAM
class MavCmdSetMessageInterval : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 511;
    MavCmdSetMessageInterval() { command = kCommandId; }
    // The MAVLink message ID
    float TheMavlinkMessage = 0;
    // The interval between two messages, in microseconds. Set to -1 to disable and
    // 0 to request default rate.
    float TheIntervalBetween = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Request autopilot capabilities
class MavCmdRequestAutopilotCapabilities : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 520;
    MavCmdRequestAutopilotCapabilities() { command = kCommandId; }
    // 1: Request autopilot version
    float p1 = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// WIP: Request camera information (CAMERA_INFORMATION)
class MavCmdRequestCameraInformation : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 521;
    MavCmdRequestCameraInformation() { command = kCommandId; }
    // 1: Request camera capabilities
    float p1 = 0;
    // Camera ID
    float CameraId = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// WIP: Request camera settings (CAMERA_SETTINGS)
class MavCmdRequestCameraSettings : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 522;
    MavCmdRequestCameraSettings() { command = kCommandId; }
    // 1: Request camera settings
    float p1 = 0;
    // Camera ID
    float CameraId = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// WIP: Set the camera settings part 1 (CAMERA_SETTINGS)
class MavCmdSetCameraSettings1 : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 523;
    MavCmdSetCameraSettings1() { command = kCommandId; }
    // Camera ID
    float CameraId = 0;
    // Aperture (1/value)
    float Aperture = 0;
    // Aperture locked (0: auto, 1: locked)
    float ApertureLocked = 0;
    // Shutter speed in s
    float ShutterSpeedS = 0;
    // Shutter speed locked (0: auto, 1: locked)
    float ShutterSpeedLocked = 0;
    // ISO sensitivity
    float IsoSensitivity = 0;
    // ISO sensitivity locked (0: auto, 1: locked)
    float IsoSensitivityLocked = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// WIP: Set the camera settings part 2 (CAMERA_SETTINGS)
class MavCmdSetCameraSettings2 : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 524;
    MavCmdSetCameraSettings2() { command = kCommandId; }
    // Camera ID
    float CameraId = 0;
    // White balance locked (0: auto, 1: locked)
    float WhiteBalanceLocked = 0;
    // White balance (color temperature in K)
    float WhiteBalance = 0;
    // Reserved for camera mode ID
    float ReservedForCamera = 0;
    // Reserved for color mode ID
    float ReservedForColor = 0;
    // Reserved for image format ID
    float ReservedForImage = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// WIP: Request storage information (STORAGE_INFORMATION)
class MavCmdRequestStorageInformation : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 525;
    MavCmdRequestStorageInformation() { command = kCommandId; }
    // 1: Request storage information
    float p1 = 0;
    // Storage ID
    float StorageId = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// WIP: Format a storage medium
class MavCmdStorageFormat : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 526;
    MavCmdStorageFormat() { command = kCommandId; }
    // 1: Format storage
    float p1 = 0;
    // Storage ID
    float StorageId = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// WIP: Request camera capture status (CAMERA_CAPTURE_STATUS)
class MavCmdRequestCameraCaptureStatus : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 527;
    MavCmdRequestCameraCaptureStatus() { command = kCommandId; }
    // 1: Request camera capture status
    float p1 = 0;
    // Camera ID
    float CameraId = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// WIP: Request flight information (FLIGHT_INFORMATION)
class MavCmdRequestFlightInformation : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 528;
    MavCmdRequestFlightInformation() { command = kCommandId; }
    // 1: Request flight information
    float p1 = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Start image capture sequence
class MavCmdImageStartCapture : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 2000;
    MavCmdImageStartCapture() { command = kCommandId; }
    // Duration between two consecutive pictures (in seconds)
    float DurationBetweenTwo = 0;
    // Number of images to capture total - 0 for unlimited capture
    float NumberOfImages = 0;
    // Resolution in megapixels (0.3 for 640x480, 1.3 for 1280x720, etc), set to 0
    // if param 4/5 are used
    float ResolutionMegapixels = 0;
    // WIP: Resolution horizontal in pixels
    float Wip = 0;
    // WIP: Resolution horizontal in pixels
    float Wip2 = 0;
    // WIP: Camera ID
    float Wip3 = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Stop image capture sequence
class MavCmdImageStopCapture : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 2001;
    MavCmdImageStopCapture() { command = kCommandId; }
    // Camera ID
    float CameraId = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Enable or disable on-board camera triggering system.
class MavCmdDoTriggerControl : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 2003;
    MavCmdDoTriggerControl() { command = kCommandId; }
    // Trigger enable/disable (0 for disable, 1 for start)
    float TriggerEnabledisable = 0;
    // Shutter integration time (in ms)
    float ShutterIntegrationTime = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Starts video capture
class MavCmdVideoStartCapture : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 2500;
    MavCmdVideoStartCapture() { command = kCommandId; }
    // Camera ID (0 for all cameras), 1 for first, 2 for second, etc.
    float CameraId = 0;
    // Frames per second
    float FramesPerSecond = 0;
    // Resolution in megapixels (0.3 for 640x480, 1.3 for 1280x720, etc), set to 0
    // if param 4/5 are used
    float ResolutionMegapixels = 0;
    // WIP: Resolution horizontal in pixels
    float Wip = 0;
    // WIP: Resolution horizontal in pixels
    float Wip2 = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Stop the current video capture
class MavCmdVideoStopCapture : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 2501;
    MavCmdVideoStopCapture() { command = kCommandId; }
    // WIP: Camera ID
    float Wip = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Request to start streaming logging data over MAVLink (see also LOGGING_DATA message)
class MavCmdLoggingStart : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 2510;
    MavCmdLoggingStart() { command = kCommandId; }
    // Format: 0: ULog
    float Format = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Request to stop streaming log data over MAVLink
class MavCmdLoggingStop : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 2511;
    MavCmdLoggingStop() { command = kCommandId; }
protected:
    virtual void pack();
    virtual void unpack();
};
class MavCmdAirframeConfiguration : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 2520;
    MavCmdAirframeConfiguration() { command = kCommandId; }
    // Landing gear ID (default: 0, -1 for all)
    float LandingGearId = 0;
    // Landing gear position (Down: 0, Up: 1, NAN for no change)
    float LandingGearPosition = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Create a panorama at the current position
class MavCmdPanoramaCreate : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 2800;
    MavCmdPanoramaCreate() { command = kCommandId; }
    // Viewing angle horizontal of the panorama (in degrees, +- 0.5 the total angle)
    float ViewingAngleHorizontal = 0;
    // Viewing angle vertical of panorama (in degrees)
    float ViewingAngleVertical = 0;
    // Speed of the horizontal rotation (in degrees per second)
    float SpeedOfHorizontal = 0;
    // Speed of the vertical rotation (in degrees per second)
    float SpeedOfVertical = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Request VTOL transition
class MavCmdDoVtolTransition : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 3000;
    MavCmdDoVtolTransition() { command = kCommandId; }
    // The target VTOL state, as defined by ENUM MAV_VTOL_STATE. Only MAV_VTOL_STATE_MC
    // and MAV_VTOL_STATE_FW can be used.
    float TheTargetVtol = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// This command sets the submode to standard guided when vehicle is in guided mode.
// The vehicle holds position and altitude and the user can input the desired velocites
// along all three axes.
class MavCmdSetGuidedSubmodeStandard : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 4000;
    MavCmdSetGuidedSubmodeStandard() { command = kCommandId; }
protected:
    virtual void pack();
    virtual void unpack();
};
// This command sets submode circle when vehicle is in guided mode. Vehicle flies
// along a circle facing the center of the circle. The user can input the velocity
// along the circle and change the radius. If no input is given the vehicle will hold
// position.
class MavCmdSetGuidedSubmodeCircle : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 4001;
    MavCmdSetGuidedSubmodeCircle() { command = kCommandId; }
    // Radius of desired circle in CIRCLE_MODE
    float RadiusOfDesired = 0;
    // User defined
    float UserDefined = 0;
    // User defined
    float UserDefined2 = 0;
    // User defined
    float UserDefined3 = 0;
    // Unscaled target latitude of center of circle in CIRCLE_MODE
    float UnscaledTargetLatitude = 0;
    // Unscaled target longitude of center of circle in CIRCLE_MODE
    float UnscaledTargetLongitude = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach
// the required release position and velocity.
class MavCmdPayloadPrepareDeploy : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 30001;
    MavCmdPayloadPrepareDeploy() { command = kCommandId; }
    // Operation mode. 0: prepare single payload deploy (overwriting previous requests),
    // but do not execute it. 1: execute payload deploy immediately (rejecting further
    // deploy commands during execution, but allowing abort). 2: add payload deploy
    // to existing deployment list.
    float OperationMode = 0;
    // Desired approach vector in degrees compass heading (0..360). A negative value
    // indicates the system can define the approach vector at will.
    float DesiredApproachVector = 0;
    // Desired ground speed at release time. This can be overriden by the airframe
    // in case it needs to meet minimum airspeed. A negative value indicates the system
    // can define the ground speed at will.
    float DesiredGroundSpeed = 0;
    // Minimum altitude clearance to the release position in meters. A negative value
    // indicates the system can define the clearance at will.
    float MinimumAltitudeClearance = 0;
    // Latitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT
    float LatitudeUnscaledFor = 0;
    // Longitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT
    float LongitudeUnscaledFor = 0;
    // Altitude, in meters AMSL
    float Altitude = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// Control the payload deployment.
class MavCmdPayloadControlDeploy : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 30002;
    MavCmdPayloadControlDeploy() { command = kCommandId; }
    // Operation mode. 0: Abort deployment, continue normal mission. 1: switch to
    // payload deploment mode. 100: delete first payload deployment request. 101:
    // delete all payload deployment requests.
    float OperationMode = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// User defined waypoint item. Ground Station will show the Vehicle as flying through
// this item.
class MavCmdWaypointUser1 : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 31000;
    MavCmdWaypointUser1() { command = kCommandId; }
    // User defined
    float UserDefined = 0;
    // User defined
    float UserDefined2 = 0;
    // User defined
    float UserDefined3 = 0;
    // User defined
    float UserDefined4 = 0;
    // Latitude unscaled
    float LatitudeUnscaled = 0;
    // Longitude unscaled
    float LongitudeUnscaled = 0;
    // Altitude, in meters AMSL
    float Altitude = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// User defined waypoint item. Ground Station will show the Vehicle as flying through
// this item.
class MavCmdWaypointUser2 : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 31001;
    MavCmdWaypointUser2() { command = kCommandId; }
    // User defined
    float UserDefined = 0;
    // User defined
    float UserDefined2 = 0;
    // User defined
    float UserDefined3 = 0;
    // User defined
    float UserDefined4 = 0;
    // Latitude unscaled
    float LatitudeUnscaled = 0;
    // Longitude unscaled
    float LongitudeUnscaled = 0;
    // Altitude, in meters AMSL
    float Altitude = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// User defined waypoint item. Ground Station will show the Vehicle as flying through
// this item.
class MavCmdWaypointUser3 : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 31002;
    MavCmdWaypointUser3() { command = kCommandId; }
    // User defined
    float UserDefined = 0;
    // User defined
    float UserDefined2 = 0;
    // User defined
    float UserDefined3 = 0;
    // User defined
    float UserDefined4 = 0;
    // Latitude unscaled
    float LatitudeUnscaled = 0;
    // Longitude unscaled
    float LongitudeUnscaled = 0;
    // Altitude, in meters AMSL
    float Altitude = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// User defined waypoint item. Ground Station will show the Vehicle as flying through
// this item.
class MavCmdWaypointUser4 : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 31003;
    MavCmdWaypointUser4() { command = kCommandId; }
    // User defined
    float UserDefined = 0;
    // User defined
    float UserDefined2 = 0;
    // User defined
    float UserDefined3 = 0;
    // User defined
    float UserDefined4 = 0;
    // Latitude unscaled
    float LatitudeUnscaled = 0;
    // Longitude unscaled
    float LongitudeUnscaled = 0;
    // Altitude, in meters AMSL
    float Altitude = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// User defined waypoint item. Ground Station will show the Vehicle as flying through
// this item.
class MavCmdWaypointUser5 : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 31004;
    MavCmdWaypointUser5() { command = kCommandId; }
    // User defined
    float UserDefined = 0;
    // User defined
    float UserDefined2 = 0;
    // User defined
    float UserDefined3 = 0;
    // User defined
    float UserDefined4 = 0;
    // Latitude unscaled
    float LatitudeUnscaled = 0;
    // Longitude unscaled
    float LongitudeUnscaled = 0;
    // Altitude, in meters AMSL
    float Altitude = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// User defined spatial item. Ground Station will not show the Vehicle as flying through
// this item. Example: ROI item.
class MavCmdSpatialUser1 : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 31005;
    MavCmdSpatialUser1() { command = kCommandId; }
    // User defined
    float UserDefined = 0;
    // User defined
    float UserDefined2 = 0;
    // User defined
    float UserDefined3 = 0;
    // User defined
    float UserDefined4 = 0;
    // Latitude unscaled
    float LatitudeUnscaled = 0;
    // Longitude unscaled
    float LongitudeUnscaled = 0;
    // Altitude, in meters AMSL
    float Altitude = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// User defined spatial item. Ground Station will not show the Vehicle as flying through
// this item. Example: ROI item.
class MavCmdSpatialUser2 : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 31006;
    MavCmdSpatialUser2() { command = kCommandId; }
    // User defined
    float UserDefined = 0;
    // User defined
    float UserDefined2 = 0;
    // User defined
    float UserDefined3 = 0;
    // User defined
    float UserDefined4 = 0;
    // Latitude unscaled
    float LatitudeUnscaled = 0;
    // Longitude unscaled
    float LongitudeUnscaled = 0;
    // Altitude, in meters AMSL
    float Altitude = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// User defined spatial item. Ground Station will not show the Vehicle as flying through
// this item. Example: ROI item.
class MavCmdSpatialUser3 : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 31007;
    MavCmdSpatialUser3() { command = kCommandId; }
    // User defined
    float UserDefined = 0;
    // User defined
    float UserDefined2 = 0;
    // User defined
    float UserDefined3 = 0;
    // User defined
    float UserDefined4 = 0;
    // Latitude unscaled
    float LatitudeUnscaled = 0;
    // Longitude unscaled
    float LongitudeUnscaled = 0;
    // Altitude, in meters AMSL
    float Altitude = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// User defined spatial item. Ground Station will not show the Vehicle as flying through
// this item. Example: ROI item.
class MavCmdSpatialUser4 : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 31008;
    MavCmdSpatialUser4() { command = kCommandId; }
    // User defined
    float UserDefined = 0;
    // User defined
    float UserDefined2 = 0;
    // User defined
    float UserDefined3 = 0;
    // User defined
    float UserDefined4 = 0;
    // Latitude unscaled
    float LatitudeUnscaled = 0;
    // Longitude unscaled
    float LongitudeUnscaled = 0;
    // Altitude, in meters AMSL
    float Altitude = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// User defined spatial item. Ground Station will not show the Vehicle as flying through
// this item. Example: ROI item.
class MavCmdSpatialUser5 : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 31009;
    MavCmdSpatialUser5() { command = kCommandId; }
    // User defined
    float UserDefined = 0;
    // User defined
    float UserDefined2 = 0;
    // User defined
    float UserDefined3 = 0;
    // User defined
    float UserDefined4 = 0;
    // Latitude unscaled
    float LatitudeUnscaled = 0;
    // Longitude unscaled
    float LongitudeUnscaled = 0;
    // Altitude, in meters AMSL
    float Altitude = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// User defined command. Ground Station will not show the Vehicle as flying through
// this item. Example: MAV_CMD_DO_SET_PARAMETER item.
class MavCmdUser1 : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 31010;
    MavCmdUser1() { command = kCommandId; }
    // User defined
    float UserDefined = 0;
    // User defined
    float UserDefined2 = 0;
    // User defined
    float UserDefined3 = 0;
    // User defined
    float UserDefined4 = 0;
    // User defined
    float UserDefined5 = 0;
    // User defined
    float UserDefined6 = 0;
    // User defined
    float UserDefined7 = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// User defined command. Ground Station will not show the Vehicle as flying through
// this item. Example: MAV_CMD_DO_SET_PARAMETER item.
class MavCmdUser2 : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 31011;
    MavCmdUser2() { command = kCommandId; }
    // User defined
    float UserDefined = 0;
    // User defined
    float UserDefined2 = 0;
    // User defined
    float UserDefined3 = 0;
    // User defined
    float UserDefined4 = 0;
    // User defined
    float UserDefined5 = 0;
    // User defined
    float UserDefined6 = 0;
    // User defined
    float UserDefined7 = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// User defined command. Ground Station will not show the Vehicle as flying through
// this item. Example: MAV_CMD_DO_SET_PARAMETER item.
class MavCmdUser3 : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 31012;
    MavCmdUser3() { command = kCommandId; }
    // User defined
    float UserDefined = 0;
    // User defined
    float UserDefined2 = 0;
    // User defined
    float UserDefined3 = 0;
    // User defined
    float UserDefined4 = 0;
    // User defined
    float UserDefined5 = 0;
    // User defined
    float UserDefined6 = 0;
    // User defined
    float UserDefined7 = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// User defined command. Ground Station will not show the Vehicle as flying through
// this item. Example: MAV_CMD_DO_SET_PARAMETER item.
class MavCmdUser4 : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 31013;
    MavCmdUser4() { command = kCommandId; }
    // User defined
    float UserDefined = 0;
    // User defined
    float UserDefined2 = 0;
    // User defined
    float UserDefined3 = 0;
    // User defined
    float UserDefined4 = 0;
    // User defined
    float UserDefined5 = 0;
    // User defined
    float UserDefined6 = 0;
    // User defined
    float UserDefined7 = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
// User defined command. Ground Station will not show the Vehicle as flying through
// this item. Example: MAV_CMD_DO_SET_PARAMETER item.
class MavCmdUser5 : public MavLinkCommand {
public:
    const static uint16_t kCommandId = 31014;
    MavCmdUser5() { command = kCommandId; }
    // User defined
    float UserDefined = 0;
    // User defined
    float UserDefined2 = 0;
    // User defined
    float UserDefined3 = 0;
    // User defined
    float UserDefined4 = 0;
    // User defined
    float UserDefined5 = 0;
    // User defined
    float UserDefined6 = 0;
    // User defined
    float UserDefined7 = 0;
protected:
    virtual void pack();
    virtual void unpack();
};
}

#endif
