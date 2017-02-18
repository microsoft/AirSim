/** @file
 *  @brief MAVLink comm protocol generated from common.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_COMMON_H
#define MAVLINK_COMMON_H

#ifndef MAVLINK_H
#error Wrong include order: MAVLINK_COMMON.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 1

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {9, 31, 12, 0, 14, 28, 3, 32, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 20, 2, 25, 23, 30, 101, 22, 26, 16, 14, 28, 32, 28, 28, 22, 22, 21, 6, 6, 37, 4, 4, 2, 2, 4, 2, 2, 3, 13, 12, 37, 4, 0, 0, 27, 25, 0, 0, 0, 0, 0, 68, 26, 185, 229, 42, 6, 4, 0, 11, 18, 0, 0, 37, 20, 35, 33, 3, 0, 0, 0, 22, 39, 37, 53, 51, 53, 51, 0, 28, 56, 42, 33, 81, 0, 0, 0, 0, 0, 0, 26, 32, 32, 20, 32, 62, 44, 64, 84, 9, 254, 16, 12, 36, 44, 64, 22, 6, 14, 12, 97, 2, 2, 113, 35, 6, 79, 35, 35, 22, 13, 255, 14, 18, 43, 8, 22, 14, 36, 43, 41, 32, 243, 14, 93, 0, 100, 36, 60, 30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 42, 40, 63, 182, 54, 0, 0, 0, 0, 0, 0, 32, 52, 53, 6, 2, 38, 19, 254, 36, 30, 18, 18, 51, 9, 0}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {50, 124, 137, 0, 237, 217, 104, 119, 0, 0, 0, 89, 0, 0, 0, 0, 0, 0, 0, 0, 214, 159, 220, 168, 24, 23, 170, 144, 67, 115, 39, 246, 185, 104, 237, 244, 222, 212, 9, 254, 230, 28, 28, 132, 221, 232, 11, 153, 41, 39, 78, 196, 0, 0, 15, 3, 0, 0, 0, 0, 0, 153, 183, 51, 59, 118, 148, 21, 0, 243, 124, 0, 0, 38, 20, 158, 152, 143, 0, 0, 0, 106, 49, 22, 143, 140, 5, 150, 0, 231, 183, 63, 54, 47, 0, 0, 0, 0, 0, 0, 175, 102, 158, 208, 56, 93, 138, 108, 32, 185, 84, 34, 174, 124, 237, 4, 76, 128, 56, 116, 134, 237, 203, 250, 87, 203, 220, 25, 226, 46, 29, 223, 85, 6, 229, 203, 1, 195, 109, 168, 181, 47, 72, 131, 127, 0, 103, 154, 178, 200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 163, 105, 151, 35, 179, 0, 0, 0, 0, 0, 0, 90, 104, 85, 95, 130, 184, 81, 8, 204, 49, 170, 44, 83, 46, 0}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_COMMON

// ENUM DEFINITIONS


/** @brief Micro air vehicle / autopilot classes. This identifies the individual model. */
#ifndef HAVE_ENUM_MAV_AUTOPILOT
#define HAVE_ENUM_MAV_AUTOPILOT
typedef enum MAV_AUTOPILOT {
    MAV_AUTOPILOT_GENERIC=0, /* Generic autopilot, full support for everything | */
    MAV_AUTOPILOT_RESERVED=1, /* Reserved for future use. | */
    MAV_AUTOPILOT_SLUGS=2, /* SLUGS autopilot, http://slugsuav.soe.ucsc.edu | */
    MAV_AUTOPILOT_ARDUPILOTMEGA=3, /* ArduPilotMega / ArduCopter, http://diydrones.com | */
    MAV_AUTOPILOT_OPENPILOT=4, /* OpenPilot, http://openpilot.org | */
    MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY=5, /* Generic autopilot only supporting simple waypoints | */
    MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY=6, /* Generic autopilot supporting waypoints and other simple navigation commands | */
    MAV_AUTOPILOT_GENERIC_MISSION_FULL=7, /* Generic autopilot supporting the full mission command set | */
    MAV_AUTOPILOT_INVALID=8, /* No valid autopilot, e.g. a GCS or other MAVLink component | */
    MAV_AUTOPILOT_PPZ=9, /* PPZ UAV - http://nongnu.org/paparazzi | */
    MAV_AUTOPILOT_UDB=10, /* UAV Dev Board | */
    MAV_AUTOPILOT_FP=11, /* FlexiPilot | */
    MAV_AUTOPILOT_PX4=12, /* PX4 Autopilot - http://pixhawk.ethz.ch/px4/ | */
    MAV_AUTOPILOT_SMACCMPILOT=13, /* SMACCMPilot - http://smaccmpilot.org | */
    MAV_AUTOPILOT_AUTOQUAD=14, /* AutoQuad -- http://autoquad.org | */
    MAV_AUTOPILOT_ARMAZILA=15, /* Armazila -- http://armazila.com | */
    MAV_AUTOPILOT_AEROB=16, /* Aerob -- http://aerob.ru | */
    MAV_AUTOPILOT_ASLUAV=17, /* ASLUAV autopilot -- http://www.asl.ethz.ch | */
    MAV_AUTOPILOT_ENUM_END=18, /*  | */
} MAV_AUTOPILOT;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_TYPE
#define HAVE_ENUM_MAV_TYPE
typedef enum MAV_TYPE {
    MAV_TYPE_GENERIC=0, /* Generic micro air vehicle. | */
    MAV_TYPE_FIXED_WING=1, /* Fixed wing aircraft. | */
    MAV_TYPE_QUADROTOR=2, /* Quadrotor | */
    MAV_TYPE_COAXIAL=3, /* Coaxial helicopter | */
    MAV_TYPE_HELICOPTER=4, /* Normal helicopter with tail rotor. | */
    MAV_TYPE_ANTENNA_TRACKER=5, /* Ground installation | */
    MAV_TYPE_GCS=6, /* Operator control unit / ground control station | */
    MAV_TYPE_AIRSHIP=7, /* Airship, controlled | */
    MAV_TYPE_FREE_BALLOON=8, /* Free balloon, uncontrolled | */
    MAV_TYPE_ROCKET=9, /* Rocket | */
    MAV_TYPE_GROUND_ROVER=10, /* Ground rover | */
    MAV_TYPE_SURFACE_BOAT=11, /* Surface vessel, boat, ship | */
    MAV_TYPE_SUBMARINE=12, /* Submarine | */
    MAV_TYPE_HEXAROTOR=13, /* Hexarotor | */
    MAV_TYPE_OCTOROTOR=14, /* Octorotor | */
    MAV_TYPE_TRICOPTER=15, /* Tricopter | */
    MAV_TYPE_FLAPPING_WING=16, /* Flapping wing | */
    MAV_TYPE_KITE=17, /* Kite | */
    MAV_TYPE_ONBOARD_CONTROLLER=18, /* Onboard companion controller | */
    MAV_TYPE_VTOL_DUOROTOR=19, /* Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter. | */
    MAV_TYPE_VTOL_QUADROTOR=20, /* Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter. | */
    MAV_TYPE_VTOL_TILTROTOR=21, /* Tiltrotor VTOL | */
    MAV_TYPE_VTOL_RESERVED2=22, /* VTOL reserved 2 | */
    MAV_TYPE_VTOL_RESERVED3=23, /* VTOL reserved 3 | */
    MAV_TYPE_VTOL_RESERVED4=24, /* VTOL reserved 4 | */
    MAV_TYPE_VTOL_RESERVED5=25, /* VTOL reserved 5 | */
    MAV_TYPE_GIMBAL=26, /* Onboard gimbal | */
    MAV_TYPE_ADSB=27, /* Onboard ADSB peripheral | */
    MAV_TYPE_ENUM_END=28, /*  | */
} MAV_TYPE;
#endif

/** @brief These values define the type of firmware release.  These values indicate the first version or release of this type.  For example the first alpha release would be 64, the second would be 65. */
#ifndef HAVE_ENUM_FIRMWARE_VERSION_TYPE
#define HAVE_ENUM_FIRMWARE_VERSION_TYPE
typedef enum FIRMWARE_VERSION_TYPE {
    FIRMWARE_VERSION_TYPE_DEV=0, /* development release | */
    FIRMWARE_VERSION_TYPE_ALPHA=64, /* alpha release | */
    FIRMWARE_VERSION_TYPE_BETA=128, /* beta release | */
    FIRMWARE_VERSION_TYPE_RC=192, /* release candidate | */
    FIRMWARE_VERSION_TYPE_OFFICIAL=255, /* official stable release | */
    FIRMWARE_VERSION_TYPE_ENUM_END=256, /*  | */
} FIRMWARE_VERSION_TYPE;
#endif

/** @brief These flags encode the MAV mode. */
#ifndef HAVE_ENUM_MAV_MODE_FLAG
#define HAVE_ENUM_MAV_MODE_FLAG
typedef enum MAV_MODE_FLAG {
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED=1, /* 0b00000001 Reserved for future use. | */
    MAV_MODE_FLAG_TEST_ENABLED=2, /* 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. | */
    MAV_MODE_FLAG_AUTO_ENABLED=4, /* 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. | */
    MAV_MODE_FLAG_GUIDED_ENABLED=8, /* 0b00001000 guided mode enabled, system flies MISSIONs / mission items. | */
    MAV_MODE_FLAG_STABILIZE_ENABLED=16, /* 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. | */
    MAV_MODE_FLAG_HIL_ENABLED=32, /* 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. | */
    MAV_MODE_FLAG_MANUAL_INPUT_ENABLED=64, /* 0b01000000 remote control input is enabled. | */
    MAV_MODE_FLAG_SAFETY_ARMED=128, /* 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM shall be used instead. The flag can still be used to report the armed state. | */
    MAV_MODE_FLAG_ENUM_END=129, /*  | */
} MAV_MODE_FLAG;
#endif

/** @brief These values encode the bit positions of the decode position. These values can be used to read the value of a flag bit by combining the base_mode variable with AND with the flag position value. The result will be either 0 or 1, depending on if the flag is set or not. */
#ifndef HAVE_ENUM_MAV_MODE_FLAG_DECODE_POSITION
#define HAVE_ENUM_MAV_MODE_FLAG_DECODE_POSITION
typedef enum MAV_MODE_FLAG_DECODE_POSITION {
    MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE=1, /* Eighth bit: 00000001 | */
    MAV_MODE_FLAG_DECODE_POSITION_TEST=2, /* Seventh bit: 00000010 | */
    MAV_MODE_FLAG_DECODE_POSITION_AUTO=4, /* Sixt bit:   00000100 | */
    MAV_MODE_FLAG_DECODE_POSITION_GUIDED=8, /* Fifth bit:  00001000 | */
    MAV_MODE_FLAG_DECODE_POSITION_STABILIZE=16, /* Fourth bit: 00010000 | */
    MAV_MODE_FLAG_DECODE_POSITION_HIL=32, /* Third bit:  00100000 | */
    MAV_MODE_FLAG_DECODE_POSITION_MANUAL=64, /* Second bit: 01000000 | */
    MAV_MODE_FLAG_DECODE_POSITION_SAFETY=128, /* First bit:  10000000 | */
    MAV_MODE_FLAG_DECODE_POSITION_ENUM_END=129, /*  | */
} MAV_MODE_FLAG_DECODE_POSITION;
#endif

/** @brief Override command, pauses current mission execution and moves immediately to a position */
#ifndef HAVE_ENUM_MAV_GOTO
#define HAVE_ENUM_MAV_GOTO
typedef enum MAV_GOTO {
    MAV_GOTO_DO_HOLD=0, /* Hold at the current position. | */
    MAV_GOTO_DO_CONTINUE=1, /* Continue with the next item in mission execution. | */
    MAV_GOTO_HOLD_AT_CURRENT_POSITION=2, /* Hold at the current position of the system | */
    MAV_GOTO_HOLD_AT_SPECIFIED_POSITION=3, /* Hold at the position specified in the parameters of the DO_HOLD action | */
    MAV_GOTO_ENUM_END=4, /*  | */
} MAV_GOTO;
#endif

/** @brief These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it
               simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override. */
#ifndef HAVE_ENUM_MAV_MODE
#define HAVE_ENUM_MAV_MODE
typedef enum MAV_MODE {
    MAV_MODE_PREFLIGHT=0, /* System is not ready to fly, booting, calibrating, etc. No flag is set. | */
    MAV_MODE_MANUAL_DISARMED=64, /* System is allowed to be active, under manual (RC) control, no stabilization | */
    MAV_MODE_TEST_DISARMED=66, /* UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. | */
    MAV_MODE_STABILIZE_DISARMED=80, /* System is allowed to be active, under assisted RC control. | */
    MAV_MODE_GUIDED_DISARMED=88, /* System is allowed to be active, under autonomous control, manual setpoint | */
    MAV_MODE_AUTO_DISARMED=92, /* System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs) | */
    MAV_MODE_MANUAL_ARMED=192, /* System is allowed to be active, under manual (RC) control, no stabilization | */
    MAV_MODE_TEST_ARMED=194, /* UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. | */
    MAV_MODE_STABILIZE_ARMED=208, /* System is allowed to be active, under assisted RC control. | */
    MAV_MODE_GUIDED_ARMED=216, /* System is allowed to be active, under autonomous control, manual setpoint | */
    MAV_MODE_AUTO_ARMED=220, /* System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs) | */
    MAV_MODE_ENUM_END=221, /*  | */
} MAV_MODE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_STATE
#define HAVE_ENUM_MAV_STATE
typedef enum MAV_STATE {
    MAV_STATE_UNINIT=0, /* Uninitialized system, state is unknown. | */
    MAV_STATE_BOOT=1, /* System is booting up. | */
    MAV_STATE_CALIBRATING=2, /* System is calibrating and not flight-ready. | */
    MAV_STATE_STANDBY=3, /* System is grounded and on standby. It can be launched any time. | */
    MAV_STATE_ACTIVE=4, /* System is active and might be already airborne. Motors are engaged. | */
    MAV_STATE_CRITICAL=5, /* System is in a non-normal flight mode. It can however still navigate. | */
    MAV_STATE_EMERGENCY=6, /* System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down. | */
    MAV_STATE_POWEROFF=7, /* System just initialized its power-down sequence, will shut down now. | */
    MAV_STATE_ENUM_END=8, /*  | */
} MAV_STATE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_COMPONENT
#define HAVE_ENUM_MAV_COMPONENT
typedef enum MAV_COMPONENT {
    MAV_COMP_ID_ALL=0, /*  | */
    MAV_COMP_ID_CAMERA=100, /*  | */
    MAV_COMP_ID_SERVO1=140, /*  | */
    MAV_COMP_ID_SERVO2=141, /*  | */
    MAV_COMP_ID_SERVO3=142, /*  | */
    MAV_COMP_ID_SERVO4=143, /*  | */
    MAV_COMP_ID_SERVO5=144, /*  | */
    MAV_COMP_ID_SERVO6=145, /*  | */
    MAV_COMP_ID_SERVO7=146, /*  | */
    MAV_COMP_ID_SERVO8=147, /*  | */
    MAV_COMP_ID_SERVO9=148, /*  | */
    MAV_COMP_ID_SERVO10=149, /*  | */
    MAV_COMP_ID_SERVO11=150, /*  | */
    MAV_COMP_ID_SERVO12=151, /*  | */
    MAV_COMP_ID_SERVO13=152, /*  | */
    MAV_COMP_ID_SERVO14=153, /*  | */
    MAV_COMP_ID_GIMBAL=154, /*  | */
    MAV_COMP_ID_LOG=155, /*  | */
    MAV_COMP_ID_ADSB=156, /*  | */
    MAV_COMP_ID_OSD=157, /* On Screen Display (OSD) devices for video links | */
    MAV_COMP_ID_PERIPHERAL=158, /* Generic autopilot peripheral component ID. Meant for devices that do not implement the parameter sub-protocol | */
    MAV_COMP_ID_QX1_GIMBAL=159, /*  | */
    MAV_COMP_ID_MAPPER=180, /*  | */
    MAV_COMP_ID_MISSIONPLANNER=190, /*  | */
    MAV_COMP_ID_PATHPLANNER=195, /*  | */
    MAV_COMP_ID_IMU=200, /*  | */
    MAV_COMP_ID_IMU_2=201, /*  | */
    MAV_COMP_ID_IMU_3=202, /*  | */
    MAV_COMP_ID_GPS=220, /*  | */
    MAV_COMP_ID_UDP_BRIDGE=240, /*  | */
    MAV_COMP_ID_UART_BRIDGE=241, /*  | */
    MAV_COMP_ID_SYSTEM_CONTROL=250, /*  | */
    MAV_COMPONENT_ENUM_END=251, /*  | */
} MAV_COMPONENT;
#endif

/** @brief These encode the sensors whose status is sent as part of the SYS_STATUS message. */
#ifndef HAVE_ENUM_MAV_SYS_STATUS_SENSOR
#define HAVE_ENUM_MAV_SYS_STATUS_SENSOR
typedef enum MAV_SYS_STATUS_SENSOR {
    MAV_SYS_STATUS_SENSOR_3D_GYRO=1, /* 0x01 3D gyro | */
    MAV_SYS_STATUS_SENSOR_3D_ACCEL=2, /* 0x02 3D accelerometer | */
    MAV_SYS_STATUS_SENSOR_3D_MAG=4, /* 0x04 3D magnetometer | */
    MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE=8, /* 0x08 absolute pressure | */
    MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE=16, /* 0x10 differential pressure | */
    MAV_SYS_STATUS_SENSOR_GPS=32, /* 0x20 GPS | */
    MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW=64, /* 0x40 optical flow | */
    MAV_SYS_STATUS_SENSOR_VISION_POSITION=128, /* 0x80 computer vision position | */
    MAV_SYS_STATUS_SENSOR_LASER_POSITION=256, /* 0x100 laser based position | */
    MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH=512, /* 0x200 external ground truth (Vicon or Leica) | */
    MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL=1024, /* 0x400 3D angular rate control | */
    MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION=2048, /* 0x800 attitude stabilization | */
    MAV_SYS_STATUS_SENSOR_YAW_POSITION=4096, /* 0x1000 yaw position | */
    MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL=8192, /* 0x2000 z/altitude control | */
    MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL=16384, /* 0x4000 x/y position control | */
    MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS=32768, /* 0x8000 motor outputs / control | */
    MAV_SYS_STATUS_SENSOR_RC_RECEIVER=65536, /* 0x10000 rc receiver | */
    MAV_SYS_STATUS_SENSOR_3D_GYRO2=131072, /* 0x20000 2nd 3D gyro | */
    MAV_SYS_STATUS_SENSOR_3D_ACCEL2=262144, /* 0x40000 2nd 3D accelerometer | */
    MAV_SYS_STATUS_SENSOR_3D_MAG2=524288, /* 0x80000 2nd 3D magnetometer | */
    MAV_SYS_STATUS_GEOFENCE=1048576, /* 0x100000 geofence | */
    MAV_SYS_STATUS_AHRS=2097152, /* 0x200000 AHRS subsystem health | */
    MAV_SYS_STATUS_TERRAIN=4194304, /* 0x400000 Terrain subsystem health | */
    MAV_SYS_STATUS_REVERSE_MOTOR=8388608, /* 0x800000 Motors are reversed | */
    MAV_SYS_STATUS_LOGGING=16777216, /* 0x1000000 Logging | */
    MAV_SYS_STATUS_SENSOR_ENUM_END=16777217, /*  | */
} MAV_SYS_STATUS_SENSOR;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_FRAME
#define HAVE_ENUM_MAV_FRAME
typedef enum MAV_FRAME {
    MAV_FRAME_GLOBAL=0, /* Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL) | */
    MAV_FRAME_LOCAL_NED=1, /* Local coordinate frame, Z-up (x: north, y: east, z: down). | */
    MAV_FRAME_MISSION=2, /* NOT a coordinate frame, indicates a mission command. | */
    MAV_FRAME_GLOBAL_RELATIVE_ALT=3, /* Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location. | */
    MAV_FRAME_LOCAL_ENU=4, /* Local coordinate frame, Z-down (x: east, y: north, z: up) | */
    MAV_FRAME_GLOBAL_INT=5, /* Global coordinate frame, WGS84 coordinate system. First value / x: latitude in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL) | */
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT=6, /* Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with 0 being at the altitude of the home location. | */
    MAV_FRAME_LOCAL_OFFSET_NED=7, /* Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position. | */
    MAV_FRAME_BODY_NED=8, /* Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right. | */
    MAV_FRAME_BODY_OFFSET_NED=9, /* Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east. | */
    MAV_FRAME_GLOBAL_TERRAIN_ALT=10, /* Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model. | */
    MAV_FRAME_GLOBAL_TERRAIN_ALT_INT=11, /* Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground level in terrain model. | */
    MAV_FRAME_ENUM_END=12, /*  | */
} MAV_FRAME;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAVLINK_DATA_STREAM_TYPE
#define HAVE_ENUM_MAVLINK_DATA_STREAM_TYPE
typedef enum MAVLINK_DATA_STREAM_TYPE {
    MAVLINK_DATA_STREAM_IMG_JPEG=1, /*  | */
    MAVLINK_DATA_STREAM_IMG_BMP=2, /*  | */
    MAVLINK_DATA_STREAM_IMG_RAW8U=3, /*  | */
    MAVLINK_DATA_STREAM_IMG_RAW32U=4, /*  | */
    MAVLINK_DATA_STREAM_IMG_PGM=5, /*  | */
    MAVLINK_DATA_STREAM_IMG_PNG=6, /*  | */
    MAVLINK_DATA_STREAM_TYPE_ENUM_END=7, /*  | */
} MAVLINK_DATA_STREAM_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_FENCE_ACTION
#define HAVE_ENUM_FENCE_ACTION
typedef enum FENCE_ACTION {
    FENCE_ACTION_NONE=0, /* Disable fenced mode | */
    FENCE_ACTION_GUIDED=1, /* Switched to guided mode to return point (fence point 0) | */
    FENCE_ACTION_REPORT=2, /* Report fence breach, but don't take action | */
    FENCE_ACTION_GUIDED_THR_PASS=3, /* Switched to guided mode to return point (fence point 0) with manual throttle control | */
    FENCE_ACTION_RTL=4, /* Switch to RTL (return to launch) mode and head for the return point. | */
    FENCE_ACTION_ENUM_END=5, /*  | */
} FENCE_ACTION;
#endif

/** @brief  */
#ifndef HAVE_ENUM_FENCE_BREACH
#define HAVE_ENUM_FENCE_BREACH
typedef enum FENCE_BREACH {
    FENCE_BREACH_NONE=0, /* No last fence breach | */
    FENCE_BREACH_MINALT=1, /* Breached minimum altitude | */
    FENCE_BREACH_MAXALT=2, /* Breached maximum altitude | */
    FENCE_BREACH_BOUNDARY=3, /* Breached fence boundary | */
    FENCE_BREACH_ENUM_END=4, /*  | */
} FENCE_BREACH;
#endif

/** @brief Enumeration of possible mount operation modes */
#ifndef HAVE_ENUM_MAV_MOUNT_MODE
#define HAVE_ENUM_MAV_MOUNT_MODE
typedef enum MAV_MOUNT_MODE {
    MAV_MOUNT_MODE_RETRACT=0, /* Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization | */
    MAV_MOUNT_MODE_NEUTRAL=1, /* Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory. | */
    MAV_MOUNT_MODE_MAVLINK_TARGETING=2, /* Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization | */
    MAV_MOUNT_MODE_RC_TARGETING=3, /* Load neutral position and start RC Roll,Pitch,Yaw control with stabilization | */
    MAV_MOUNT_MODE_GPS_POINT=4, /* Load neutral position and start to point to Lat,Lon,Alt | */
    MAV_MOUNT_MODE_ENUM_END=5, /*  | */
} MAV_MOUNT_MODE;
#endif

/** @brief Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data. */
#ifndef HAVE_ENUM_MAV_CMD
#define HAVE_ENUM_MAV_CMD
typedef enum MAV_CMD {
    MAV_CMD_NAV_WAYPOINT=16, /* Navigate to MISSION. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)| Acceptance radius in meters (if the sphere with this radius is hit, the MISSION counts as reached)| 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| Desired yaw angle at MISSION (rotary wing)| Latitude| Longitude| Altitude|  */
    MAV_CMD_NAV_LOITER_UNLIM=17, /* Loiter around this MISSION an unlimited amount of time |Empty| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
    MAV_CMD_NAV_LOITER_TURNS=18, /* Loiter around this MISSION for X turns |Turns| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle| Latitude| Longitude| Altitude|  */
    MAV_CMD_NAV_LOITER_TIME=19, /* Loiter around this MISSION for X seconds |Seconds (decimal)| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle| Latitude| Longitude| Altitude|  */
    MAV_CMD_NAV_RETURN_TO_LAUNCH=20, /* Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
    MAV_CMD_NAV_LAND=21, /* Land at location |Abort Alt| Empty| Empty| Desired yaw angle| Latitude| Longitude| Altitude|  */
    MAV_CMD_NAV_TAKEOFF=22, /* Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer| Latitude| Longitude| Altitude|  */
    MAV_CMD_NAV_LAND_LOCAL=23, /* Land at local position (local frame only) |Landing target number (if available)| Maximum accepted offset from desired landing position [m] - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position and the position where the vehicle is about to land| Landing descend rate [ms^-1]| Desired yaw angle [rad]| Y-axis position [m]| X-axis position [m]| Z-axis / ground level position [m]|  */
    MAV_CMD_NAV_TAKEOFF_LOCAL=24, /* Takeoff from local position (local frame only) |Minimum pitch (if airspeed sensor present), desired pitch without sensor [rad]| Empty| Takeoff ascend rate [ms^-1]| Yaw angle [rad] (if magnetometer or another yaw estimation source present), ignored without one of these| Y-axis position [m]| X-axis position [m]| Z-axis position [m]|  */
    MAV_CMD_NAV_FOLLOW=25, /* Vehicle following, i.e. this waypoint represents the position of a moving vehicle |Following logic to use (e.g. loitering or sinusoidal following) - depends on specific autopilot implementation| Ground speed of vehicle to be followed| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
    MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT=30, /* Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached. |Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing, command completes when at or above this command's altitude, 2 = Descending, command completes when at or below this command's altitude. | Empty| Empty| Empty| Empty| Empty| Desired altitude in meters|  */
    MAV_CMD_NAV_LOITER_TO_ALT=31, /* Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached.  Additionally, if the Heading Required parameter is non-zero the  aircraft will not leave the loiter until heading toward the next waypoint.  |Heading Required (0 = False)| Radius in meters. If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.| Empty| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location| Latitude| Longitude| Altitude|  */
    MAV_CMD_DO_FOLLOW=32, /* Being following a target |System ID (the system ID of the FOLLOW_TARGET beacon). Send 0 to disable follow-me and return to the default position hold mode| RESERVED| RESERVED| altitude flag: 0: Keep current altitude, 1: keep altitude difference to target, 2: go to a fixed altitude above home| altitude| RESERVED| TTL in seconds in which the MAV should go to the default position hold mode after a message rx timeout|  */
    MAV_CMD_DO_FOLLOW_REPOSITION=33, /* Reposition the MAV after a follow target command has been sent |Camera q1 (where 0 is on the ray from the camera to the tracking device)| Camera q2| Camera q3| Camera q4| altitude offset from target (m)| X offset from target (m)| Y offset from target (m)|  */
    MAV_CMD_NAV_ROI=80, /* Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of intereset mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  */
    MAV_CMD_NAV_PATHPLANNING=81, /* Control autonomous path planning on the MAV. |0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning| 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid| Empty| Yaw angle at goal, in compass degrees, [0..360]| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  */
    MAV_CMD_NAV_SPLINE_WAYPOINT=82, /* Navigate to MISSION using a spline path. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)| Empty| Empty| Empty| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  */
    MAV_CMD_NAV_VTOL_TAKEOFF=84, /* Takeoff from ground using VTOL mode |Empty| Empty| Empty| Yaw angle in degrees| Latitude| Longitude| Altitude|  */
    MAV_CMD_NAV_VTOL_LAND=85, /* Land using VTOL mode |Empty| Empty| Empty| Yaw angle in degrees| Latitude| Longitude| Altitude|  */
    MAV_CMD_NAV_GUIDED_ENABLE=92, /* hand control over to an external controller |On / Off (> 0.5f on)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    MAV_CMD_NAV_DELAY=93, /* Delay the next navigation command a number of seconds or until a specified time |Delay in seconds (decimal, -1 to enable time-of-day fields)| hour (24h format, UTC, -1 to ignore)| minute (24h format, UTC, -1 to ignore)| second (24h format, UTC)| Empty| Empty| Empty|  */
    MAV_CMD_NAV_LAST=95, /* NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
    MAV_CMD_CONDITION_DELAY=112, /* Delay mission state machine. |Delay in seconds (decimal)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    MAV_CMD_CONDITION_CHANGE_ALT=113, /* Ascend/descend at rate.  Delay mission state machine until desired altitude reached. |Descent / Ascend rate (m/s)| Empty| Empty| Empty| Empty| Empty| Finish Altitude|  */
    MAV_CMD_CONDITION_DISTANCE=114, /* Delay mission state machine until within desired distance of next NAV point. |Distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    MAV_CMD_CONDITION_YAW=115, /* Reach a certain target angle. |target angle: [0-360], 0 is north| speed during yaw change:[deg per second]| direction: negative: counter clockwise, positive: clockwise [-1,1]| relative offset or absolute angle: [ 1,0]| Empty| Empty| Empty|  */
    MAV_CMD_CONDITION_LAST=159, /* NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
    MAV_CMD_DO_SET_MODE=176, /* Set system mode. |Mode, as defined by ENUM MAV_MODE| Custom mode - this is system specific, please refer to the individual autopilot specifications for details.| Custom sub mode - this is system specific, please refer to the individual autopilot specifications for details.| Empty| Empty| Empty| Empty|  */
    MAV_CMD_DO_JUMP=177, /* Jump to the desired command in the mission list.  Repeat this action only the specified number of times |Sequence number| Repeat count| Empty| Empty| Empty| Empty| Empty|  */
    MAV_CMD_DO_CHANGE_SPEED=178, /* Change speed and/or throttle set points. |Speed type (0=Airspeed, 1=Ground Speed)| Speed  (m/s, -1 indicates no change)| Throttle  ( Percent, -1 indicates no change)| absolute or relative [0,1]| Empty| Empty| Empty|  */
    MAV_CMD_DO_SET_HOME=179, /* Changes the home location either to the current location or a specified location. |Use current (1=use current location, 0=use specified location)| Empty| Empty| Empty| Latitude| Longitude| Altitude|  */
    MAV_CMD_DO_SET_PARAMETER=180, /* Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. |Parameter number| Parameter value| Empty| Empty| Empty| Empty| Empty|  */
    MAV_CMD_DO_SET_RELAY=181, /* Set a relay to a condition. |Relay number| Setting (1=on, 0=off, others possible depending on system hardware)| Empty| Empty| Empty| Empty| Empty|  */
    MAV_CMD_DO_REPEAT_RELAY=182, /* Cycle a relay on and off for a desired number of cyles with a desired period. |Relay number| Cycle count| Cycle time (seconds, decimal)| Empty| Empty| Empty| Empty|  */
    MAV_CMD_DO_SET_SERVO=183, /* Set a servo to a desired PWM value. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Empty| Empty| Empty| Empty| Empty|  */
    MAV_CMD_DO_REPEAT_SERVO=184, /* Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Cycle count| Cycle time (seconds)| Empty| Empty| Empty|  */
    MAV_CMD_DO_FLIGHTTERMINATION=185, /* Terminate flight immediately |Flight termination activated if > 0.5| Empty| Empty| Empty| Empty| Empty| Empty|  */
    MAV_CMD_DO_CHANGE_ALTITUDE=186, /* Change altitude set point. |Altitude in meters| Mav frame of new altitude (see MAV_FRAME)| Empty| Empty| Empty| Empty| Empty|  */
    MAV_CMD_DO_LAND_START=189, /* Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0/0 if not needed. If specified then it will be used to help find the closest landing sequence. |Empty| Empty| Empty| Empty| Latitude| Longitude| Empty|  */
    MAV_CMD_DO_RALLY_LAND=190, /* Mission command to perform a landing from a rally point. |Break altitude (meters)| Landing speed (m/s)| Empty| Empty| Empty| Empty| Empty|  */
    MAV_CMD_DO_GO_AROUND=191, /* Mission command to safely abort an autonmous landing. |Altitude (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    MAV_CMD_DO_REPOSITION=192, /* Reposition the vehicle to a specific WGS84 global position. |Ground speed, less than 0 (-1) for default| Bitmask of option flags, see the MAV_DO_REPOSITION_FLAGS enum.| Reserved| Yaw heading, NaN for unchanged. For planes indicates loiter direction (0: clockwise, 1: counter clockwise)| Latitude (deg * 1E7)| Longitude (deg * 1E7)| Altitude (meters)|  */
    MAV_CMD_DO_PAUSE_CONTINUE=193, /* If in a GPS controlled position mode, hold the current position or continue. |0: Pause current mission or reposition command, hold current position. 1: Continue mission. A VTOL capable vehicle should enter hover mode (multicopter and VTOL planes). A plane should loiter with the default loiter radius.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
    MAV_CMD_DO_SET_REVERSE=194, /* Set moving direction to forward or reverse. |Direction (0=Forward, 1=Reverse)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    MAV_CMD_DO_CONTROL_VIDEO=200, /* Control onboard camera system. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|  */
    MAV_CMD_DO_SET_ROI=201, /* Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of intereset mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  */
    MAV_CMD_DO_DIGICAM_CONFIGURE=202, /* Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|  */
    MAV_CMD_DO_DIGICAM_CONTROL=203, /* Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Empty|  */
    MAV_CMD_DO_MOUNT_CONFIGURE=204, /* Mission command to configure a camera or antenna mount |Mount operation mode (see MAV_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| Empty| Empty| Empty|  */
    MAV_CMD_DO_MOUNT_CONTROL=205, /* Mission command to control a camera or antenna mount |pitch or lat in degrees, depending on mount mode.| roll or lon in degrees depending on mount mode| yaw or alt (in meters) depending on mount mode| reserved| reserved| reserved| MAV_MOUNT_MODE enum value|  */
    MAV_CMD_DO_SET_CAM_TRIGG_DIST=206, /* Mission command to set CAM_TRIGG_DIST for this flight |Camera trigger distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    MAV_CMD_DO_FENCE_ENABLE=207, /* Mission command to enable the geofence |enable? (0=disable, 1=enable, 2=disable_floor_only)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    MAV_CMD_DO_PARACHUTE=208, /* Mission command to trigger a parachute |action (0=disable, 1=enable, 2=release, for some systems see PARACHUTE_ACTION enum, not in general message set.)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    MAV_CMD_DO_MOTOR_TEST=209, /* Mission command to perform motor test |motor sequence number (a number from 1 to max number of motors on the vehicle)| throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)| throttle| timeout (in seconds)| Empty| Empty| Empty|  */
    MAV_CMD_DO_INVERTED_FLIGHT=210, /* Change to/from inverted flight |inverted (0=normal, 1=inverted)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    MAV_CMD_DO_SET_POSITION_YAW_THRUST=213, /* Sets a desired vehicle turn angle and thrust change |yaw angle to adjust steering by in centidegress| Thrust - normalized to -2 .. 2| Empty| Empty| Empty| Empty| Empty|  */
    MAV_CMD_DO_MOUNT_CONTROL_QUAT=220, /* Mission command to control a camera or antenna mount, using a quaternion as reference. |q1 - quaternion param #1, w (1 in null-rotation)| q2 - quaternion param #2, x (0 in null-rotation)| q3 - quaternion param #3, y (0 in null-rotation)| q4 - quaternion param #4, z (0 in null-rotation)| Empty| Empty| Empty|  */
    MAV_CMD_DO_GUIDED_MASTER=221, /* set id of master controller |System ID| Component ID| Empty| Empty| Empty| Empty| Empty|  */
    MAV_CMD_DO_GUIDED_LIMITS=222, /* set limits for external control |timeout - maximum time (in seconds) that external controller will be allowed to control vehicle. 0 means no timeout| absolute altitude min (in meters, AMSL) - if vehicle moves below this alt, the command will be aborted and the mission will continue.  0 means no lower altitude limit| absolute altitude max (in meters)- if vehicle moves above this alt, the command will be aborted and the mission will continue.  0 means no upper altitude limit| horizontal move limit (in meters, AMSL) - if vehicle moves more than this distance from it's location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal altitude limit| Empty| Empty| Empty|  */
    MAV_CMD_DO_ENGINE_CONTROL=223, /* Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine state. It is intended for vehicles with internal combustion engines |0: Stop engine, 1:Start Engine| 0: Warm start, 1:Cold start. Controls use of choke where applicable| Height delay (meters). This is for commanding engine start only after the vehicle has gained the specified height. Used in VTOL vehicles during takeoff to start engine after the aircraft is off the ground. Zero for no delay.| Empty| Empty| Empty| Empty| Empty|  */
    MAV_CMD_DO_LAST=240, /* NOP - This command is only used to mark the upper limit of the DO commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
    MAV_CMD_PREFLIGHT_CALIBRATION=241, /* Trigger calibration. This command will be only accepted if in pre-flight mode. |Gyro calibration: 0: no, 1: yes| Magnetometer calibration: 0: no, 1: yes| Ground pressure: 0: no, 1: yes| Radio calibration: 0: no, 1: yes| Accelerometer calibration: 0: no, 1: yes| Compass/Motor interference calibration: 0: no, 1: yes| Empty|  */
    MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS=242, /* Set sensor offsets. This command will be only accepted if in pre-flight mode. |Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow, 5: second magnetometer, 6: third magnetometer| X axis offset (or generic dimension 1), in the sensor's raw units| Y axis offset (or generic dimension 2), in the sensor's raw units| Z axis offset (or generic dimension 3), in the sensor's raw units| Generic dimension 4, in the sensor's raw units| Generic dimension 5, in the sensor's raw units| Generic dimension 6, in the sensor's raw units|  */
    MAV_CMD_PREFLIGHT_UAVCAN=243, /* Trigger UAVCAN config. This command will be only accepted if in pre-flight mode. |1: Trigger actuator ID assignment and direction mapping.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
    MAV_CMD_PREFLIGHT_STORAGE=245, /* Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. |Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults| Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults| Onboard logging: 0: Ignore, 1: Start default rate logging, -1: Stop logging, > 1: start logging with rate of param 3 in Hz (e.g. set to 1000 for 1000 Hz logging)| Reserved| Empty| Empty| Empty|  */
    MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN=246, /* Request the reboot or shutdown of system components. |0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep it in the bootloader until upgraded.| 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer, 3: Reboot onboard computer and keep it in the bootloader until upgraded.| Reserved, send 0| Reserved, send 0| Reserved, send 0| Reserved, send 0| Reserved, send 0|  */
    MAV_CMD_OVERRIDE_GOTO=252, /* Hold / continue the current action |MAV_GOTO_DO_HOLD: hold MAV_GOTO_DO_CONTINUE: continue with next item in mission plan| MAV_GOTO_HOLD_AT_CURRENT_POSITION: Hold at current position MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position| MAV_FRAME coordinate frame of hold point| Desired yaw angle in degrees| Latitude / X position| Longitude / Y position| Altitude / Z position|  */
    MAV_CMD_MISSION_START=300, /* start running a mission |first_item: the first mission item to run| last_item:  the last mission item to run (after this item is run, the mission ends)|  */
    MAV_CMD_COMPONENT_ARM_DISARM=400, /* Arms / Disarms a component |1 to arm, 0 to disarm|  */
    MAV_CMD_GET_HOME_POSITION=410, /* Request the home position from the vehicle. |Reserved| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
    MAV_CMD_START_RX_PAIR=500, /* Starts receiver pairing |0:Spektrum| 0:Spektrum DSM2, 1:Spektrum DSMX|  */
    MAV_CMD_GET_MESSAGE_INTERVAL=510, /* Request the interval between messages for a particular MAVLink message ID |The MAVLink message ID|  */
    MAV_CMD_SET_MESSAGE_INTERVAL=511, /* Request the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM |The MAVLink message ID| The interval between two messages, in microseconds. Set to -1 to disable and 0 to request default rate.|  */
    MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES=520, /* Request autopilot capabilities |1: Request autopilot version| Reserved (all remaining params)|  */
    MAV_CMD_IMAGE_START_CAPTURE=2000, /* Start image capture sequence |Duration between two consecutive pictures (in seconds)| Number of images to capture total - 0 for unlimited capture| Resolution in megapixels (0.3 for 640x480, 1.3 for 1280x720, etc)|  */
    MAV_CMD_IMAGE_STOP_CAPTURE=2001, /* Stop image capture sequence |Reserved| Reserved|  */
    MAV_CMD_DO_TRIGGER_CONTROL=2003, /* Enable or disable on-board camera triggering system. |Trigger enable/disable (0 for disable, 1 for start)| Shutter integration time (in ms)| Reserved|  */
    MAV_CMD_VIDEO_START_CAPTURE=2500, /* Starts video capture |Camera ID (0 for all cameras), 1 for first, 2 for second, etc.| Frames per second| Resolution in megapixels (0.3 for 640x480, 1.3 for 1280x720, etc)|  */
    MAV_CMD_VIDEO_STOP_CAPTURE=2501, /* Stop the current video capture |Reserved| Reserved|  */
    MAV_CMD_LOGGING_START=2510, /* Request to start streaming logging data over MAVLink (see also LOGGING_DATA message) |Format: 0: ULog| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)|  */
    MAV_CMD_LOGGING_STOP=2511, /* Request to stop streaming log data over MAVLink |Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)|  */
    MAV_CMD_AIRFRAME_CONFIGURATION=2520, /*  |Landing gear ID (default: 0, -1 for all)| Landing gear position (Down: 0, Up: 1, NAN for no change)| Reserved, set to NAN| Reserved, set to NAN| Reserved, set to NAN| Reserved, set to NAN| Reserved, set to NAN|  */
    MAV_CMD_PANORAMA_CREATE=2800, /* Create a panorama at the current position |Viewing angle horizontal of the panorama (in degrees, +- 0.5 the total angle)| Viewing angle vertical of panorama (in degrees)| Speed of the horizontal rotation (in degrees per second)| Speed of the vertical rotation (in degrees per second)|  */
    MAV_CMD_DO_VTOL_TRANSITION=3000, /* Request VTOL transition |The target VTOL state, as defined by ENUM MAV_VTOL_STATE. Only MAV_VTOL_STATE_MC and MAV_VTOL_STATE_FW can be used.|  */
    MAV_CMD_SET_GUIDED_SUBMODE_STANDARD=4000, /* This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocites along all three axes.
                   | */
    MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE=4001, /* This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.
                   |Radius of desired circle in CIRCLE_MODE| User defined| User defined| User defined| Unscaled target latitude of center of circle in CIRCLE_MODE| Unscaled target longitude of center of circle in CIRCLE_MODE|  */
    MAV_CMD_PAYLOAD_PREPARE_DEPLOY=30001, /* Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity. |Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing abort). 2: add payload deploy to existing deployment list.| Desired approach vector in degrees compass heading (0..360). A negative value indicates the system can define the approach vector at will.| Desired ground speed at release time. This can be overridden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will.| Minimum altitude clearance to the release position in meters. A negative value indicates the system can define the clearance at will.| Latitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT| Longitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT| Altitude, in meters AMSL|  */
    MAV_CMD_PAYLOAD_CONTROL_DEPLOY=30002, /* Control the payload deployment. |Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deploment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
    MAV_CMD_WAYPOINT_USER_1=31000, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    MAV_CMD_WAYPOINT_USER_2=31001, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    MAV_CMD_WAYPOINT_USER_3=31002, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    MAV_CMD_WAYPOINT_USER_4=31003, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    MAV_CMD_WAYPOINT_USER_5=31004, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    MAV_CMD_SPATIAL_USER_1=31005, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    MAV_CMD_SPATIAL_USER_2=31006, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    MAV_CMD_SPATIAL_USER_3=31007, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    MAV_CMD_SPATIAL_USER_4=31008, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    MAV_CMD_SPATIAL_USER_5=31009, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    MAV_CMD_USER_1=31010, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
    MAV_CMD_USER_2=31011, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
    MAV_CMD_USER_3=31012, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
    MAV_CMD_USER_4=31013, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
    MAV_CMD_USER_5=31014, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
    MAV_CMD_ENUM_END=31015, /*  | */
} MAV_CMD;
#endif

/** @brief THIS INTERFACE IS DEPRECATED AS OF JULY 2015. Please use MESSAGE_INTERVAL instead. A data stream is not a fixed set of messages, but rather a
     recommendation to the autopilot software. Individual autopilots may or may not obey
     the recommended messages. */
#ifndef HAVE_ENUM_MAV_DATA_STREAM
#define HAVE_ENUM_MAV_DATA_STREAM
typedef enum MAV_DATA_STREAM {
    MAV_DATA_STREAM_ALL=0, /* Enable all data streams | */
    MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets. | */
    MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS | */
    MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW | */
    MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT. | */
    MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages. | */
    MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot | */
    MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot | */
    MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot | */
    MAV_DATA_STREAM_ENUM_END=13, /*  | */
} MAV_DATA_STREAM;
#endif

/** @brief  The ROI (region of interest) for the vehicle. This can be
                be used by the vehicle for camera/vehicle attitude alignment (see
                MAV_CMD_NAV_ROI). */
#ifndef HAVE_ENUM_MAV_ROI
#define HAVE_ENUM_MAV_ROI
typedef enum MAV_ROI {
    MAV_ROI_NONE=0, /* No region of interest. | */
    MAV_ROI_WPNEXT=1, /* Point toward next MISSION. | */
    MAV_ROI_WPINDEX=2, /* Point toward given MISSION. | */
    MAV_ROI_LOCATION=3, /* Point toward fixed location. | */
    MAV_ROI_TARGET=4, /* Point toward of given id. | */
    MAV_ROI_ENUM_END=5, /*  | */
} MAV_ROI;
#endif

/** @brief ACK / NACK / ERROR values as a result of MAV_CMDs and for mission item transmission. */
#ifndef HAVE_ENUM_MAV_CMD_ACK
#define HAVE_ENUM_MAV_CMD_ACK
typedef enum MAV_CMD_ACK {
    MAV_CMD_ACK_OK=1, /* Command / mission item is ok. | */
    MAV_CMD_ACK_ERR_FAIL=2, /* Generic error message if none of the other reasons fails or if no detailed error reporting is implemented. | */
    MAV_CMD_ACK_ERR_ACCESS_DENIED=3, /* The system is refusing to accept this command from this source / communication partner. | */
    MAV_CMD_ACK_ERR_NOT_SUPPORTED=4, /* Command or mission item is not supported, other commands would be accepted. | */
    MAV_CMD_ACK_ERR_COORDINATE_FRAME_NOT_SUPPORTED=5, /* The coordinate frame of this command / mission item is not supported. | */
    MAV_CMD_ACK_ERR_COORDINATES_OUT_OF_RANGE=6, /* The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of this system. This is a generic error, please use the more specific error messages below if possible. | */
    MAV_CMD_ACK_ERR_X_LAT_OUT_OF_RANGE=7, /* The X or latitude value is out of range. | */
    MAV_CMD_ACK_ERR_Y_LON_OUT_OF_RANGE=8, /* The Y or longitude value is out of range. | */
    MAV_CMD_ACK_ERR_Z_ALT_OUT_OF_RANGE=9, /* The Z or altitude value is out of range. | */
    MAV_CMD_ACK_ENUM_END=10, /*  | */
} MAV_CMD_ACK;
#endif

/** @brief Specifies the datatype of a MAVLink parameter. */
#ifndef HAVE_ENUM_MAV_PARAM_TYPE
#define HAVE_ENUM_MAV_PARAM_TYPE
typedef enum MAV_PARAM_TYPE {
    MAV_PARAM_TYPE_UINT8=1, /* 8-bit unsigned integer | */
    MAV_PARAM_TYPE_INT8=2, /* 8-bit signed integer | */
    MAV_PARAM_TYPE_UINT16=3, /* 16-bit unsigned integer | */
    MAV_PARAM_TYPE_INT16=4, /* 16-bit signed integer | */
    MAV_PARAM_TYPE_UINT32=5, /* 32-bit unsigned integer | */
    MAV_PARAM_TYPE_INT32=6, /* 32-bit signed integer | */
    MAV_PARAM_TYPE_UINT64=7, /* 64-bit unsigned integer | */
    MAV_PARAM_TYPE_INT64=8, /* 64-bit signed integer | */
    MAV_PARAM_TYPE_REAL32=9, /* 32-bit floating-point | */
    MAV_PARAM_TYPE_REAL64=10, /* 64-bit floating-point | */
    MAV_PARAM_TYPE_ENUM_END=11, /*  | */
} MAV_PARAM_TYPE;
#endif

/** @brief result from a mavlink command */
#ifndef HAVE_ENUM_MAV_RESULT
#define HAVE_ENUM_MAV_RESULT
typedef enum MAV_RESULT {
    MAV_RESULT_ACCEPTED=0, /* Command ACCEPTED and EXECUTED | */
    MAV_RESULT_TEMPORARILY_REJECTED=1, /* Command TEMPORARY REJECTED/DENIED | */
    MAV_RESULT_DENIED=2, /* Command PERMANENTLY DENIED | */
    MAV_RESULT_UNSUPPORTED=3, /* Command UNKNOWN/UNSUPPORTED | */
    MAV_RESULT_FAILED=4, /* Command executed, but failed | */
    MAV_RESULT_ENUM_END=5, /*  | */
} MAV_RESULT;
#endif

/** @brief result in a mavlink mission ack */
#ifndef HAVE_ENUM_MAV_MISSION_RESULT
#define HAVE_ENUM_MAV_MISSION_RESULT
typedef enum MAV_MISSION_RESULT {
    MAV_MISSION_ACCEPTED=0, /* mission accepted OK | */
    MAV_MISSION_ERROR=1, /* generic error / not accepting mission commands at all right now | */
    MAV_MISSION_UNSUPPORTED_FRAME=2, /* coordinate frame is not supported | */
    MAV_MISSION_UNSUPPORTED=3, /* command is not supported | */
    MAV_MISSION_NO_SPACE=4, /* mission item exceeds storage space | */
    MAV_MISSION_INVALID=5, /* one of the parameters has an invalid value | */
    MAV_MISSION_INVALID_PARAM1=6, /* param1 has an invalid value | */
    MAV_MISSION_INVALID_PARAM2=7, /* param2 has an invalid value | */
    MAV_MISSION_INVALID_PARAM3=8, /* param3 has an invalid value | */
    MAV_MISSION_INVALID_PARAM4=9, /* param4 has an invalid value | */
    MAV_MISSION_INVALID_PARAM5_X=10, /* x/param5 has an invalid value | */
    MAV_MISSION_INVALID_PARAM6_Y=11, /* y/param6 has an invalid value | */
    MAV_MISSION_INVALID_PARAM7=12, /* param7 has an invalid value | */
    MAV_MISSION_INVALID_SEQUENCE=13, /* received waypoint out of sequence | */
    MAV_MISSION_DENIED=14, /* not accepting any mission commands from this communication partner | */
    MAV_MISSION_RESULT_ENUM_END=15, /*  | */
} MAV_MISSION_RESULT;
#endif

/** @brief Indicates the severity level, generally used for status messages to indicate their relative urgency. Based on RFC-5424 using expanded definitions at: http://www.kiwisyslog.com/kb/info:-syslog-message-levels/. */
#ifndef HAVE_ENUM_MAV_SEVERITY
#define HAVE_ENUM_MAV_SEVERITY
typedef enum MAV_SEVERITY {
    MAV_SEVERITY_EMERGENCY=0, /* System is unusable. This is a "panic" condition. | */
    MAV_SEVERITY_ALERT=1, /* Action should be taken immediately. Indicates error in non-critical systems. | */
    MAV_SEVERITY_CRITICAL=2, /* Action must be taken immediately. Indicates failure in a primary system. | */
    MAV_SEVERITY_ERROR=3, /* Indicates an error in secondary/redundant systems. | */
    MAV_SEVERITY_WARNING=4, /* Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning. | */
    MAV_SEVERITY_NOTICE=5, /* An unusual event has occurred, though not an error condition. This should be investigated for the root cause. | */
    MAV_SEVERITY_INFO=6, /* Normal operational messages. Useful for logging. No action is required for these messages. | */
    MAV_SEVERITY_DEBUG=7, /* Useful non-operational messages that can assist in debugging. These should not occur during normal operation. | */
    MAV_SEVERITY_ENUM_END=8, /*  | */
} MAV_SEVERITY;
#endif

/** @brief Power supply status flags (bitmask) */
#ifndef HAVE_ENUM_MAV_POWER_STATUS
#define HAVE_ENUM_MAV_POWER_STATUS
typedef enum MAV_POWER_STATUS {
    MAV_POWER_STATUS_BRICK_VALID=1, /* main brick power supply valid | */
    MAV_POWER_STATUS_SERVO_VALID=2, /* main servo power supply valid for FMU | */
    MAV_POWER_STATUS_USB_CONNECTED=4, /* USB power is connected | */
    MAV_POWER_STATUS_PERIPH_OVERCURRENT=8, /* peripheral supply is in over-current state | */
    MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT=16, /* hi-power peripheral supply is in over-current state | */
    MAV_POWER_STATUS_CHANGED=32, /* Power status has changed since boot | */
    MAV_POWER_STATUS_ENUM_END=33, /*  | */
} MAV_POWER_STATUS;
#endif

/** @brief SERIAL_CONTROL device types */
#ifndef HAVE_ENUM_SERIAL_CONTROL_DEV
#define HAVE_ENUM_SERIAL_CONTROL_DEV
typedef enum SERIAL_CONTROL_DEV {
    SERIAL_CONTROL_DEV_TELEM1=0, /* First telemetry port | */
    SERIAL_CONTROL_DEV_TELEM2=1, /* Second telemetry port | */
    SERIAL_CONTROL_DEV_GPS1=2, /* First GPS port | */
    SERIAL_CONTROL_DEV_GPS2=3, /* Second GPS port | */
    SERIAL_CONTROL_DEV_SHELL=10, /* system shell | */
    SERIAL_CONTROL_DEV_ENUM_END=11, /*  | */
} SERIAL_CONTROL_DEV;
#endif

/** @brief SERIAL_CONTROL flags (bitmask) */
#ifndef HAVE_ENUM_SERIAL_CONTROL_FLAG
#define HAVE_ENUM_SERIAL_CONTROL_FLAG
typedef enum SERIAL_CONTROL_FLAG {
    SERIAL_CONTROL_FLAG_REPLY=1, /* Set if this is a reply | */
    SERIAL_CONTROL_FLAG_RESPOND=2, /* Set if the sender wants the receiver to send a response as another SERIAL_CONTROL message | */
    SERIAL_CONTROL_FLAG_EXCLUSIVE=4, /* Set if access to the serial port should be removed from whatever driver is currently using it, giving exclusive access to the SERIAL_CONTROL protocol. The port can be handed back by sending a request without this flag set | */
    SERIAL_CONTROL_FLAG_BLOCKING=8, /* Block on writes to the serial port | */
    SERIAL_CONTROL_FLAG_MULTI=16, /* Send multiple replies until port is drained | */
    SERIAL_CONTROL_FLAG_ENUM_END=17, /*  | */
} SERIAL_CONTROL_FLAG;
#endif

/** @brief Enumeration of distance sensor types */
#ifndef HAVE_ENUM_MAV_DISTANCE_SENSOR
#define HAVE_ENUM_MAV_DISTANCE_SENSOR
typedef enum MAV_DISTANCE_SENSOR {
    MAV_DISTANCE_SENSOR_LASER=0, /* Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units | */
    MAV_DISTANCE_SENSOR_ULTRASOUND=1, /* Ultrasound rangefinder, e.g. MaxBotix units | */
    MAV_DISTANCE_SENSOR_INFRARED=2, /* Infrared rangefinder, e.g. Sharp units | */
    MAV_DISTANCE_SENSOR_ENUM_END=3, /*  | */
} MAV_DISTANCE_SENSOR;
#endif

/** @brief Enumeration of sensor orientation, according to its rotations */
#ifndef HAVE_ENUM_MAV_SENSOR_ORIENTATION
#define HAVE_ENUM_MAV_SENSOR_ORIENTATION
typedef enum MAV_SENSOR_ORIENTATION {
    MAV_SENSOR_ROTATION_NONE=0, /* Roll: 0, Pitch: 0, Yaw: 0 | */
    MAV_SENSOR_ROTATION_YAW_45=1, /* Roll: 0, Pitch: 0, Yaw: 45 | */
    MAV_SENSOR_ROTATION_YAW_90=2, /* Roll: 0, Pitch: 0, Yaw: 90 | */
    MAV_SENSOR_ROTATION_YAW_135=3, /* Roll: 0, Pitch: 0, Yaw: 135 | */
    MAV_SENSOR_ROTATION_YAW_180=4, /* Roll: 0, Pitch: 0, Yaw: 180 | */
    MAV_SENSOR_ROTATION_YAW_225=5, /* Roll: 0, Pitch: 0, Yaw: 225 | */
    MAV_SENSOR_ROTATION_YAW_270=6, /* Roll: 0, Pitch: 0, Yaw: 270 | */
    MAV_SENSOR_ROTATION_YAW_315=7, /* Roll: 0, Pitch: 0, Yaw: 315 | */
    MAV_SENSOR_ROTATION_ROLL_180=8, /* Roll: 180, Pitch: 0, Yaw: 0 | */
    MAV_SENSOR_ROTATION_ROLL_180_YAW_45=9, /* Roll: 180, Pitch: 0, Yaw: 45 | */
    MAV_SENSOR_ROTATION_ROLL_180_YAW_90=10, /* Roll: 180, Pitch: 0, Yaw: 90 | */
    MAV_SENSOR_ROTATION_ROLL_180_YAW_135=11, /* Roll: 180, Pitch: 0, Yaw: 135 | */
    MAV_SENSOR_ROTATION_PITCH_180=12, /* Roll: 0, Pitch: 180, Yaw: 0 | */
    MAV_SENSOR_ROTATION_ROLL_180_YAW_225=13, /* Roll: 180, Pitch: 0, Yaw: 225 | */
    MAV_SENSOR_ROTATION_ROLL_180_YAW_270=14, /* Roll: 180, Pitch: 0, Yaw: 270 | */
    MAV_SENSOR_ROTATION_ROLL_180_YAW_315=15, /* Roll: 180, Pitch: 0, Yaw: 315 | */
    MAV_SENSOR_ROTATION_ROLL_90=16, /* Roll: 90, Pitch: 0, Yaw: 0 | */
    MAV_SENSOR_ROTATION_ROLL_90_YAW_45=17, /* Roll: 90, Pitch: 0, Yaw: 45 | */
    MAV_SENSOR_ROTATION_ROLL_90_YAW_90=18, /* Roll: 90, Pitch: 0, Yaw: 90 | */
    MAV_SENSOR_ROTATION_ROLL_90_YAW_135=19, /* Roll: 90, Pitch: 0, Yaw: 135 | */
    MAV_SENSOR_ROTATION_ROLL_270=20, /* Roll: 270, Pitch: 0, Yaw: 0 | */
    MAV_SENSOR_ROTATION_ROLL_270_YAW_45=21, /* Roll: 270, Pitch: 0, Yaw: 45 | */
    MAV_SENSOR_ROTATION_ROLL_270_YAW_90=22, /* Roll: 270, Pitch: 0, Yaw: 90 | */
    MAV_SENSOR_ROTATION_ROLL_270_YAW_135=23, /* Roll: 270, Pitch: 0, Yaw: 135 | */
    MAV_SENSOR_ROTATION_PITCH_90=24, /* Roll: 0, Pitch: 90, Yaw: 0 | */
    MAV_SENSOR_ROTATION_PITCH_270=25, /* Roll: 0, Pitch: 270, Yaw: 0 | */
    MAV_SENSOR_ROTATION_PITCH_180_YAW_90=26, /* Roll: 0, Pitch: 180, Yaw: 90 | */
    MAV_SENSOR_ROTATION_PITCH_180_YAW_270=27, /* Roll: 0, Pitch: 180, Yaw: 270 | */
    MAV_SENSOR_ROTATION_ROLL_90_PITCH_90=28, /* Roll: 90, Pitch: 90, Yaw: 0 | */
    MAV_SENSOR_ROTATION_ROLL_180_PITCH_90=29, /* Roll: 180, Pitch: 90, Yaw: 0 | */
    MAV_SENSOR_ROTATION_ROLL_270_PITCH_90=30, /* Roll: 270, Pitch: 90, Yaw: 0 | */
    MAV_SENSOR_ROTATION_ROLL_90_PITCH_180=31, /* Roll: 90, Pitch: 180, Yaw: 0 | */
    MAV_SENSOR_ROTATION_ROLL_270_PITCH_180=32, /* Roll: 270, Pitch: 180, Yaw: 0 | */
    MAV_SENSOR_ROTATION_ROLL_90_PITCH_270=33, /* Roll: 90, Pitch: 270, Yaw: 0 | */
    MAV_SENSOR_ROTATION_ROLL_180_PITCH_270=34, /* Roll: 180, Pitch: 270, Yaw: 0 | */
    MAV_SENSOR_ROTATION_ROLL_270_PITCH_270=35, /* Roll: 270, Pitch: 270, Yaw: 0 | */
    MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90=36, /* Roll: 90, Pitch: 180, Yaw: 90 | */
    MAV_SENSOR_ROTATION_ROLL_90_YAW_270=37, /* Roll: 90, Pitch: 0, Yaw: 270 | */
    MAV_SENSOR_ROTATION_ROLL_315_PITCH_315_YAW_315=38, /* Roll: 315, Pitch: 315, Yaw: 315 | */
    MAV_SENSOR_ORIENTATION_ENUM_END=39, /*  | */
} MAV_SENSOR_ORIENTATION;
#endif

/** @brief Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this capability. */
#ifndef HAVE_ENUM_MAV_PROTOCOL_CAPABILITY
#define HAVE_ENUM_MAV_PROTOCOL_CAPABILITY
typedef enum MAV_PROTOCOL_CAPABILITY {
    MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT=1, /* Autopilot supports MISSION float message type. | */
    MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT=2, /* Autopilot supports the new param float message type. | */
    MAV_PROTOCOL_CAPABILITY_MISSION_INT=4, /* Autopilot supports MISSION_INT scaled integer message type. | */
    MAV_PROTOCOL_CAPABILITY_COMMAND_INT=8, /* Autopilot supports COMMAND_INT scaled integer message type. | */
    MAV_PROTOCOL_CAPABILITY_PARAM_UNION=16, /* Autopilot supports the new param union message type. | */
    MAV_PROTOCOL_CAPABILITY_FTP=32, /* Autopilot supports the new FILE_TRANSFER_PROTOCOL message type. | */
    MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET=64, /* Autopilot supports commanding attitude offboard. | */
    MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED=128, /* Autopilot supports commanding position and velocity targets in local NED frame. | */
    MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT=256, /* Autopilot supports commanding position and velocity targets in global scaled integers. | */
    MAV_PROTOCOL_CAPABILITY_TERRAIN=512, /* Autopilot supports terrain protocol / data handling. | */
    MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET=1024, /* Autopilot supports direct actuator control. | */
    MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION=2048, /* Autopilot supports the flight termination command. | */
    MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION=4096, /* Autopilot supports onboard compass calibration. | */
    MAV_PROTOCOL_CAPABILITY_MAVLINK2=8192, /* Autopilot supports mavlink version 2. | */
    MAV_PROTOCOL_CAPABILITY_ENUM_END=8193, /*  | */
} MAV_PROTOCOL_CAPABILITY;
#endif

/** @brief Enumeration of estimator types */
#ifndef HAVE_ENUM_MAV_ESTIMATOR_TYPE
#define HAVE_ENUM_MAV_ESTIMATOR_TYPE
typedef enum MAV_ESTIMATOR_TYPE {
    MAV_ESTIMATOR_TYPE_NAIVE=1, /* This is a naive estimator without any real covariance feedback. | */
    MAV_ESTIMATOR_TYPE_VISION=2, /* Computer vision based estimate. Might be up to scale. | */
    MAV_ESTIMATOR_TYPE_VIO=3, /* Visual-inertial estimate. | */
    MAV_ESTIMATOR_TYPE_GPS=4, /* Plain GPS estimate. | */
    MAV_ESTIMATOR_TYPE_GPS_INS=5, /* Estimator integrating GPS and inertial sensing. | */
    MAV_ESTIMATOR_TYPE_ENUM_END=6, /*  | */
} MAV_ESTIMATOR_TYPE;
#endif

/** @brief Enumeration of battery types */
#ifndef HAVE_ENUM_MAV_BATTERY_TYPE
#define HAVE_ENUM_MAV_BATTERY_TYPE
typedef enum MAV_BATTERY_TYPE {
    MAV_BATTERY_TYPE_UNKNOWN=0, /* Not specified. | */
    MAV_BATTERY_TYPE_LIPO=1, /* Lithium polymer battery | */
    MAV_BATTERY_TYPE_LIFE=2, /* Lithium-iron-phosphate battery | */
    MAV_BATTERY_TYPE_LION=3, /* Lithium-ION battery | */
    MAV_BATTERY_TYPE_NIMH=4, /* Nickel metal hydride battery | */
    MAV_BATTERY_TYPE_ENUM_END=5, /*  | */
} MAV_BATTERY_TYPE;
#endif

/** @brief Enumeration of battery functions */
#ifndef HAVE_ENUM_MAV_BATTERY_FUNCTION
#define HAVE_ENUM_MAV_BATTERY_FUNCTION
typedef enum MAV_BATTERY_FUNCTION {
    MAV_BATTERY_FUNCTION_UNKNOWN=0, /* Battery function is unknown | */
    MAV_BATTERY_FUNCTION_ALL=1, /* Battery supports all flight systems | */
    MAV_BATTERY_FUNCTION_PROPULSION=2, /* Battery for the propulsion system | */
    MAV_BATTERY_FUNCTION_AVIONICS=3, /* Avionics battery | */
    MAV_BATTERY_TYPE_PAYLOAD=4, /* Payload battery | */
    MAV_BATTERY_FUNCTION_ENUM_END=5, /*  | */
} MAV_BATTERY_FUNCTION;
#endif

/** @brief Enumeration of VTOL states */
#ifndef HAVE_ENUM_MAV_VTOL_STATE
#define HAVE_ENUM_MAV_VTOL_STATE
typedef enum MAV_VTOL_STATE {
    MAV_VTOL_STATE_UNDEFINED=0, /* MAV is not configured as VTOL | */
    MAV_VTOL_STATE_TRANSITION_TO_FW=1, /* VTOL is in transition from multicopter to fixed-wing | */
    MAV_VTOL_STATE_TRANSITION_TO_MC=2, /* VTOL is in transition from fixed-wing to multicopter | */
    MAV_VTOL_STATE_MC=3, /* VTOL is in multicopter state | */
    MAV_VTOL_STATE_FW=4, /* VTOL is in fixed-wing state | */
    MAV_VTOL_STATE_ENUM_END=5, /*  | */
} MAV_VTOL_STATE;
#endif

/** @brief Enumeration of landed detector states */
#ifndef HAVE_ENUM_MAV_LANDED_STATE
#define HAVE_ENUM_MAV_LANDED_STATE
typedef enum MAV_LANDED_STATE {
    MAV_LANDED_STATE_UNDEFINED=0, /* MAV landed state is unknown | */
    MAV_LANDED_STATE_ON_GROUND=1, /* MAV is landed (on ground) | */
    MAV_LANDED_STATE_IN_AIR=2, /* MAV is in air | */
    MAV_LANDED_STATE_ENUM_END=3, /*  | */
} MAV_LANDED_STATE;
#endif

/** @brief Enumeration of the ADSB altimeter types */
#ifndef HAVE_ENUM_ADSB_ALTITUDE_TYPE
#define HAVE_ENUM_ADSB_ALTITUDE_TYPE
typedef enum ADSB_ALTITUDE_TYPE {
    ADSB_ALTITUDE_TYPE_PRESSURE_QNH=0, /* Altitude reported from a Baro source using QNH reference | */
    ADSB_ALTITUDE_TYPE_GEOMETRIC=1, /* Altitude reported from a GNSS source | */
    ADSB_ALTITUDE_TYPE_ENUM_END=2, /*  | */
} ADSB_ALTITUDE_TYPE;
#endif

/** @brief ADSB classification for the type of vehicle emitting the transponder signal */
#ifndef HAVE_ENUM_ADSB_EMITTER_TYPE
#define HAVE_ENUM_ADSB_EMITTER_TYPE
typedef enum ADSB_EMITTER_TYPE {
    ADSB_EMITTER_TYPE_NO_INFO=0, /*  | */
    ADSB_EMITTER_TYPE_LIGHT=1, /*  | */
    ADSB_EMITTER_TYPE_SMALL=2, /*  | */
    ADSB_EMITTER_TYPE_LARGE=3, /*  | */
    ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE=4, /*  | */
    ADSB_EMITTER_TYPE_HEAVY=5, /*  | */
    ADSB_EMITTER_TYPE_HIGHLY_MANUV=6, /*  | */
    ADSB_EMITTER_TYPE_ROTOCRAFT=7, /*  | */
    ADSB_EMITTER_TYPE_UNASSIGNED=8, /*  | */
    ADSB_EMITTER_TYPE_GLIDER=9, /*  | */
    ADSB_EMITTER_TYPE_LIGHTER_AIR=10, /*  | */
    ADSB_EMITTER_TYPE_PARACHUTE=11, /*  | */
    ADSB_EMITTER_TYPE_ULTRA_LIGHT=12, /*  | */
    ADSB_EMITTER_TYPE_UNASSIGNED2=13, /*  | */
    ADSB_EMITTER_TYPE_UAV=14, /*  | */
    ADSB_EMITTER_TYPE_SPACE=15, /*  | */
    ADSB_EMITTER_TYPE_UNASSGINED3=16, /*  | */
    ADSB_EMITTER_TYPE_EMERGENCY_SURFACE=17, /*  | */
    ADSB_EMITTER_TYPE_SERVICE_SURFACE=18, /*  | */
    ADSB_EMITTER_TYPE_POINT_OBSTACLE=19, /*  | */
    ADSB_EMITTER_TYPE_ENUM_END=20, /*  | */
} ADSB_EMITTER_TYPE;
#endif

/** @brief These flags indicate status such as data validity of each data source. Set = data valid */
#ifndef HAVE_ENUM_ADSB_FLAGS
#define HAVE_ENUM_ADSB_FLAGS
typedef enum ADSB_FLAGS {
    ADSB_FLAGS_VALID_COORDS=1, /*  | */
    ADSB_FLAGS_VALID_ALTITUDE=2, /*  | */
    ADSB_FLAGS_VALID_HEADING=4, /*  | */
    ADSB_FLAGS_VALID_VELOCITY=8, /*  | */
    ADSB_FLAGS_VALID_CALLSIGN=16, /*  | */
    ADSB_FLAGS_VALID_SQUAWK=32, /*  | */
    ADSB_FLAGS_SIMULATED=64, /*  | */
    ADSB_FLAGS_ENUM_END=65, /*  | */
} ADSB_FLAGS;
#endif

/** @brief Bitmask of options for the MAV_CMD_DO_REPOSITION */
#ifndef HAVE_ENUM_MAV_DO_REPOSITION_FLAGS
#define HAVE_ENUM_MAV_DO_REPOSITION_FLAGS
typedef enum MAV_DO_REPOSITION_FLAGS {
    MAV_DO_REPOSITION_FLAGS_CHANGE_MODE=1, /* The aircraft should immediately transition into guided. This should not be set for follow me applications | */
    MAV_DO_REPOSITION_FLAGS_ENUM_END=2, /*  | */
} MAV_DO_REPOSITION_FLAGS;
#endif

/** @brief Flags in EKF_STATUS message */
#ifndef HAVE_ENUM_ESTIMATOR_STATUS_FLAGS
#define HAVE_ENUM_ESTIMATOR_STATUS_FLAGS
typedef enum ESTIMATOR_STATUS_FLAGS {
    ESTIMATOR_ATTITUDE=1, /* True if the attitude estimate is good | */
    ESTIMATOR_VELOCITY_HORIZ=2, /* True if the horizontal velocity estimate is good | */
    ESTIMATOR_VELOCITY_VERT=4, /* True if the  vertical velocity estimate is good | */
    ESTIMATOR_POS_HORIZ_REL=8, /* True if the horizontal position (relative) estimate is good | */
    ESTIMATOR_POS_HORIZ_ABS=16, /* True if the horizontal position (absolute) estimate is good | */
    ESTIMATOR_POS_VERT_ABS=32, /* True if the vertical position (absolute) estimate is good | */
    ESTIMATOR_POS_VERT_AGL=64, /* True if the vertical position (above ground) estimate is good | */
    ESTIMATOR_CONST_POS_MODE=128, /* True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical flow) | */
    ESTIMATOR_PRED_POS_HORIZ_REL=256, /* True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate | */
    ESTIMATOR_PRED_POS_HORIZ_ABS=512, /* True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate | */
    ESTIMATOR_GPS_GLITCH=1024, /* True if the EKF has detected a GPS glitch | */
    ESTIMATOR_STATUS_FLAGS_ENUM_END=1025, /*  | */
} ESTIMATOR_STATUS_FLAGS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MOTOR_TEST_THROTTLE_TYPE
#define HAVE_ENUM_MOTOR_TEST_THROTTLE_TYPE
typedef enum MOTOR_TEST_THROTTLE_TYPE {
    MOTOR_TEST_THROTTLE_PERCENT=0, /* throttle as a percentage from 0 ~ 100 | */
    MOTOR_TEST_THROTTLE_PWM=1, /* throttle as an absolute PWM value (normally in range of 1000~2000) | */
    MOTOR_TEST_THROTTLE_PILOT=2, /* throttle pass-through from pilot's transmitter | */
    MOTOR_TEST_THROTTLE_TYPE_ENUM_END=3, /*  | */
} MOTOR_TEST_THROTTLE_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GPS_INPUT_IGNORE_FLAGS
#define HAVE_ENUM_GPS_INPUT_IGNORE_FLAGS
typedef enum GPS_INPUT_IGNORE_FLAGS {
    GPS_INPUT_IGNORE_FLAG_ALT=1, /* ignore altitude field | */
    GPS_INPUT_IGNORE_FLAG_HDOP=2, /* ignore hdop field | */
    GPS_INPUT_IGNORE_FLAG_VDOP=4, /* ignore vdop field | */
    GPS_INPUT_IGNORE_FLAG_VEL_HORIZ=8, /* ignore horizontal velocity field (vn and ve) | */
    GPS_INPUT_IGNORE_FLAG_VEL_VERT=16, /* ignore vertical velocity field (vd) | */
    GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY=32, /* ignore speed accuracy field | */
    GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY=64, /* ignore horizontal accuracy field | */
    GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY=128, /* ignore vertical accuracy field | */
    GPS_INPUT_IGNORE_FLAGS_ENUM_END=129, /*  | */
} GPS_INPUT_IGNORE_FLAGS;
#endif

/** @brief Possible actions an aircraft can take to avoid a collision. */
#ifndef HAVE_ENUM_MAV_COLLISION_ACTION
#define HAVE_ENUM_MAV_COLLISION_ACTION
typedef enum MAV_COLLISION_ACTION {
    MAV_COLLISION_ACTION_NONE=0, /* Ignore any potential collisions | */
    MAV_COLLISION_ACTION_REPORT=1, /* Report potential collision | */
    MAV_COLLISION_ACTION_ASCEND_OR_DESCEND=2, /* Ascend or Descend to avoid thread | */
    MAV_COLLISION_ACTION_MOVE_HORIZONTALLY=3, /* Ascend or Descend to avoid thread | */
    MAV_COLLISION_ACTION_MOVE_PERPENDICULAR=4, /* Aircraft to move perpendicular to the collision's velocity vector | */
    MAV_COLLISION_ACTION_RTL=5, /* Aircraft to fly directly back to its launch point | */
    MAV_COLLISION_ACTION_HOVER=6, /* Aircraft to stop in place | */
    MAV_COLLISION_ACTION_ENUM_END=7, /*  | */
} MAV_COLLISION_ACTION;
#endif

/** @brief Aircraft-rated danger from this threat. */
#ifndef HAVE_ENUM_MAV_COLLISION_THREAT_LEVEL
#define HAVE_ENUM_MAV_COLLISION_THREAT_LEVEL
typedef enum MAV_COLLISION_THREAT_LEVEL {
    MAV_COLLISION_THREAT_LEVEL_NONE=0, /* Not a threat | */
    MAV_COLLISION_THREAT_LEVEL_LOW=1, /* Craft is mildly concerned about this threat | */
    MAV_COLLISION_THREAT_LEVEL_HIGH=2, /* Craft is panicing, and may take actions to avoid threat | */
    MAV_COLLISION_THREAT_LEVEL_ENUM_END=3, /*  | */
} MAV_COLLISION_THREAT_LEVEL;
#endif

/** @brief Source of information about this collision. */
#ifndef HAVE_ENUM_MAV_COLLISION_SRC
#define HAVE_ENUM_MAV_COLLISION_SRC
typedef enum MAV_COLLISION_SRC {
    MAV_COLLISION_SRC_ADSB=0, /* ID field references ADSB_VEHICLE packets | */
    MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT=1, /* ID field references MAVLink SRC ID | */
    MAV_COLLISION_SRC_ENUM_END=2, /*  | */
} MAV_COLLISION_SRC;
#endif

/** @brief Type of GPS fix */
#ifndef HAVE_ENUM_GPS_FIX_TYPE
#define HAVE_ENUM_GPS_FIX_TYPE
typedef enum GPS_FIX_TYPE {
    GPS_FIX_TYPE_NO_GPS=0, /* No GPS connected | */
    GPS_FIX_TYPE_NO_FIX=1, /* No position information, GPS is connected | */
    GPS_FIX_TYPE_2D_FIX=2, /* 2D position | */
    GPS_FIX_TYPE_3D_FIX=3, /* 3D position | */
    GPS_FIX_TYPE_DGPS=4, /* DGPS/SBAS aided 3D position | */
    GPS_FIX_TYPE_RTK_FLOAT=5, /* RTK float, 3D position | */
    GPS_FIX_TYPE_RTK_FIXED=6, /* RTK Fixed, 3D position | */
    GPS_FIX_TYPE_STATIC=7, /* Static fixed, typically used for base stations | */
    GPS_FIX_TYPE_ENUM_END=8, /*  | */
} GPS_FIX_TYPE;
#endif

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 3
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 3
#endif


// base include


#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 1

#if MAVLINK_THIS_XML_IDX == MAVLINK_PRIMARY_XML_IDX
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_HEARTBEAT, MAVLINK_MESSAGE_INFO_SYS_STATUS, MAVLINK_MESSAGE_INFO_SYSTEM_TIME, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, MAVLINK_MESSAGE_INFO_PING, MAVLINK_MESSAGE_INFO_CHANGE_OPERATOR_CONTROL, MAVLINK_MESSAGE_INFO_CHANGE_OPERATOR_CONTROL_ACK, MAVLINK_MESSAGE_INFO_AUTH_KEY, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, MAVLINK_MESSAGE_INFO_SET_MODE, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, MAVLINK_MESSAGE_INFO_PARAM_REQUEST_READ, MAVLINK_MESSAGE_INFO_PARAM_REQUEST_LIST, MAVLINK_MESSAGE_INFO_PARAM_VALUE, MAVLINK_MESSAGE_INFO_PARAM_SET, MAVLINK_MESSAGE_INFO_GPS_RAW_INT, MAVLINK_MESSAGE_INFO_GPS_STATUS, MAVLINK_MESSAGE_INFO_SCALED_IMU, MAVLINK_MESSAGE_INFO_RAW_IMU, MAVLINK_MESSAGE_INFO_RAW_PRESSURE, MAVLINK_MESSAGE_INFO_SCALED_PRESSURE, MAVLINK_MESSAGE_INFO_ATTITUDE, MAVLINK_MESSAGE_INFO_ATTITUDE_QUATERNION, MAVLINK_MESSAGE_INFO_LOCAL_POSITION_NED, MAVLINK_MESSAGE_INFO_GLOBAL_POSITION_INT, MAVLINK_MESSAGE_INFO_RC_CHANNELS_SCALED, MAVLINK_MESSAGE_INFO_RC_CHANNELS_RAW, MAVLINK_MESSAGE_INFO_SERVO_OUTPUT_RAW, MAVLINK_MESSAGE_INFO_MISSION_REQUEST_PARTIAL_LIST, MAVLINK_MESSAGE_INFO_MISSION_WRITE_PARTIAL_LIST, MAVLINK_MESSAGE_INFO_MISSION_ITEM, MAVLINK_MESSAGE_INFO_MISSION_REQUEST, MAVLINK_MESSAGE_INFO_MISSION_SET_CURRENT, MAVLINK_MESSAGE_INFO_MISSION_CURRENT, MAVLINK_MESSAGE_INFO_MISSION_REQUEST_LIST, MAVLINK_MESSAGE_INFO_MISSION_COUNT, MAVLINK_MESSAGE_INFO_MISSION_CLEAR_ALL, MAVLINK_MESSAGE_INFO_MISSION_ITEM_REACHED, MAVLINK_MESSAGE_INFO_MISSION_ACK, MAVLINK_MESSAGE_INFO_SET_GPS_GLOBAL_ORIGIN, MAVLINK_MESSAGE_INFO_GPS_GLOBAL_ORIGIN, MAVLINK_MESSAGE_INFO_PARAM_MAP_RC, MAVLINK_MESSAGE_INFO_MISSION_REQUEST_INT, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, MAVLINK_MESSAGE_INFO_SAFETY_SET_ALLOWED_AREA, MAVLINK_MESSAGE_INFO_SAFETY_ALLOWED_AREA, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, MAVLINK_MESSAGE_INFO_ATTITUDE_QUATERNION_COV, MAVLINK_MESSAGE_INFO_NAV_CONTROLLER_OUTPUT, MAVLINK_MESSAGE_INFO_GLOBAL_POSITION_INT_COV, MAVLINK_MESSAGE_INFO_LOCAL_POSITION_NED_COV, MAVLINK_MESSAGE_INFO_RC_CHANNELS, MAVLINK_MESSAGE_INFO_REQUEST_DATA_STREAM, MAVLINK_MESSAGE_INFO_DATA_STREAM, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, MAVLINK_MESSAGE_INFO_MANUAL_CONTROL, MAVLINK_MESSAGE_INFO_RC_CHANNELS_OVERRIDE, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, MAVLINK_MESSAGE_INFO_MISSION_ITEM_INT, MAVLINK_MESSAGE_INFO_VFR_HUD, MAVLINK_MESSAGE_INFO_COMMAND_INT, MAVLINK_MESSAGE_INFO_COMMAND_LONG, MAVLINK_MESSAGE_INFO_COMMAND_ACK, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, MAVLINK_MESSAGE_INFO_MANUAL_SETPOINT, MAVLINK_MESSAGE_INFO_SET_ATTITUDE_TARGET, MAVLINK_MESSAGE_INFO_ATTITUDE_TARGET, MAVLINK_MESSAGE_INFO_SET_POSITION_TARGET_LOCAL_NED, MAVLINK_MESSAGE_INFO_POSITION_TARGET_LOCAL_NED, MAVLINK_MESSAGE_INFO_SET_POSITION_TARGET_GLOBAL_INT, MAVLINK_MESSAGE_INFO_POSITION_TARGET_GLOBAL_INT, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, MAVLINK_MESSAGE_INFO_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET, MAVLINK_MESSAGE_INFO_HIL_STATE, MAVLINK_MESSAGE_INFO_HIL_CONTROLS, MAVLINK_MESSAGE_INFO_HIL_RC_INPUTS_RAW, MAVLINK_MESSAGE_INFO_HIL_ACTUATOR_CONTROLS, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, MAVLINK_MESSAGE_INFO_OPTICAL_FLOW, MAVLINK_MESSAGE_INFO_GLOBAL_VISION_POSITION_ESTIMATE, MAVLINK_MESSAGE_INFO_VISION_POSITION_ESTIMATE, MAVLINK_MESSAGE_INFO_VISION_SPEED_ESTIMATE, MAVLINK_MESSAGE_INFO_VICON_POSITION_ESTIMATE, MAVLINK_MESSAGE_INFO_HIGHRES_IMU, MAVLINK_MESSAGE_INFO_OPTICAL_FLOW_RAD, MAVLINK_MESSAGE_INFO_HIL_SENSOR, MAVLINK_MESSAGE_INFO_SIM_STATE, MAVLINK_MESSAGE_INFO_RADIO_STATUS, MAVLINK_MESSAGE_INFO_FILE_TRANSFER_PROTOCOL, MAVLINK_MESSAGE_INFO_TIMESYNC, MAVLINK_MESSAGE_INFO_CAMERA_TRIGGER, MAVLINK_MESSAGE_INFO_HIL_GPS, MAVLINK_MESSAGE_INFO_HIL_OPTICAL_FLOW, MAVLINK_MESSAGE_INFO_HIL_STATE_QUATERNION, MAVLINK_MESSAGE_INFO_SCALED_IMU2, MAVLINK_MESSAGE_INFO_LOG_REQUEST_LIST, MAVLINK_MESSAGE_INFO_LOG_ENTRY, MAVLINK_MESSAGE_INFO_LOG_REQUEST_DATA, MAVLINK_MESSAGE_INFO_LOG_DATA, MAVLINK_MESSAGE_INFO_LOG_ERASE, MAVLINK_MESSAGE_INFO_LOG_REQUEST_END, MAVLINK_MESSAGE_INFO_GPS_INJECT_DATA, MAVLINK_MESSAGE_INFO_GPS2_RAW, MAVLINK_MESSAGE_INFO_POWER_STATUS, MAVLINK_MESSAGE_INFO_SERIAL_CONTROL, MAVLINK_MESSAGE_INFO_GPS_RTK, MAVLINK_MESSAGE_INFO_GPS2_RTK, MAVLINK_MESSAGE_INFO_SCALED_IMU3, MAVLINK_MESSAGE_INFO_DATA_TRANSMISSION_HANDSHAKE, MAVLINK_MESSAGE_INFO_ENCAPSULATED_DATA, MAVLINK_MESSAGE_INFO_DISTANCE_SENSOR, MAVLINK_MESSAGE_INFO_TERRAIN_REQUEST, MAVLINK_MESSAGE_INFO_TERRAIN_DATA, MAVLINK_MESSAGE_INFO_TERRAIN_CHECK, MAVLINK_MESSAGE_INFO_TERRAIN_REPORT, MAVLINK_MESSAGE_INFO_SCALED_PRESSURE2, MAVLINK_MESSAGE_INFO_ATT_POS_MOCAP, MAVLINK_MESSAGE_INFO_SET_ACTUATOR_CONTROL_TARGET, MAVLINK_MESSAGE_INFO_ACTUATOR_CONTROL_TARGET, MAVLINK_MESSAGE_INFO_ALTITUDE, MAVLINK_MESSAGE_INFO_RESOURCE_REQUEST, MAVLINK_MESSAGE_INFO_SCALED_PRESSURE3, MAVLINK_MESSAGE_INFO_FOLLOW_TARGET, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, MAVLINK_MESSAGE_INFO_CONTROL_SYSTEM_STATE, MAVLINK_MESSAGE_INFO_BATTERY_STATUS, MAVLINK_MESSAGE_INFO_AUTOPILOT_VERSION, MAVLINK_MESSAGE_INFO_LANDING_TARGET, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, MAVLINK_MESSAGE_INFO_ESTIMATOR_STATUS, MAVLINK_MESSAGE_INFO_WIND_COV, MAVLINK_MESSAGE_INFO_GPS_INPUT, MAVLINK_MESSAGE_INFO_GPS_RTCM_DATA, MAVLINK_MESSAGE_INFO_HIGH_LATENCY, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, MAVLINK_MESSAGE_INFO_VIBRATION, MAVLINK_MESSAGE_INFO_HOME_POSITION, MAVLINK_MESSAGE_INFO_SET_HOME_POSITION, MAVLINK_MESSAGE_INFO_MESSAGE_INTERVAL, MAVLINK_MESSAGE_INFO_EXTENDED_SYS_STATE, MAVLINK_MESSAGE_INFO_ADSB_VEHICLE, MAVLINK_MESSAGE_INFO_COLLISION, MAVLINK_MESSAGE_INFO_V2_EXTENSION, MAVLINK_MESSAGE_INFO_MEMORY_VECT, MAVLINK_MESSAGE_INFO_DEBUG_VECT, MAVLINK_MESSAGE_INFO_NAMED_VALUE_FLOAT, MAVLINK_MESSAGE_INFO_NAMED_VALUE_INT, MAVLINK_MESSAGE_INFO_STATUSTEXT, MAVLINK_MESSAGE_INFO_DEBUG, {"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_COMMON_H
