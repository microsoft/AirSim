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
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{0, 50, 9, 9, 0, 0, 0}, {1, 124, 31, 31, 0, 0, 0}, {2, 137, 12, 12, 0, 0, 0}, {4, 237, 14, 14, 3, 12, 13}, {5, 217, 28, 28, 1, 0, 0}, {6, 104, 3, 3, 0, 0, 0}, {7, 119, 32, 32, 0, 0, 0}, {8, 117, 36, 36, 0, 0, 0}, {11, 89, 6, 6, 1, 4, 0}, {20, 214, 20, 20, 3, 2, 3}, {21, 159, 2, 2, 3, 0, 1}, {22, 220, 25, 25, 0, 0, 0}, {23, 168, 23, 23, 3, 4, 5}, {24, 24, 30, 50, 0, 0, 0}, {25, 23, 101, 101, 0, 0, 0}, {26, 170, 22, 24, 0, 0, 0}, {27, 144, 26, 29, 0, 0, 0}, {28, 67, 16, 16, 0, 0, 0}, {29, 115, 14, 14, 0, 0, 0}, {30, 39, 28, 28, 0, 0, 0}, {31, 246, 32, 48, 0, 0, 0}, {32, 185, 28, 28, 0, 0, 0}, {33, 104, 28, 28, 0, 0, 0}, {34, 237, 22, 22, 0, 0, 0}, {35, 244, 22, 22, 0, 0, 0}, {36, 222, 21, 37, 0, 0, 0}, {37, 212, 6, 7, 3, 4, 5}, {38, 9, 6, 7, 3, 4, 5}, {39, 254, 37, 38, 3, 32, 33}, {40, 230, 4, 5, 3, 2, 3}, {41, 28, 4, 4, 3, 2, 3}, {42, 28, 2, 2, 0, 0, 0}, {43, 132, 2, 3, 3, 0, 1}, {44, 221, 4, 5, 3, 2, 3}, {45, 232, 2, 3, 3, 0, 1}, {46, 11, 2, 2, 0, 0, 0}, {47, 153, 3, 4, 3, 0, 1}, {48, 41, 13, 21, 1, 12, 0}, {49, 39, 12, 20, 0, 0, 0}, {50, 78, 37, 37, 3, 18, 19}, {51, 196, 4, 5, 3, 2, 3}, {52, 132, 7, 7, 0, 0, 0}, {54, 15, 27, 27, 3, 24, 25}, {55, 3, 25, 25, 0, 0, 0}, {61, 167, 72, 72, 0, 0, 0}, {62, 183, 26, 26, 0, 0, 0}, {63, 119, 181, 181, 0, 0, 0}, {64, 191, 225, 225, 0, 0, 0}, {65, 118, 42, 42, 0, 0, 0}, {66, 148, 6, 6, 3, 2, 3}, {67, 21, 4, 4, 0, 0, 0}, {69, 243, 11, 11, 1, 10, 0}, {70, 124, 18, 38, 3, 16, 17}, {73, 38, 37, 38, 3, 32, 33}, {74, 20, 20, 20, 0, 0, 0}, {75, 158, 35, 35, 3, 30, 31}, {76, 152, 33, 33, 3, 30, 31}, {77, 143, 3, 10, 3, 8, 9}, {81, 106, 22, 22, 0, 0, 0}, {82, 49, 39, 39, 3, 36, 37}, {83, 22, 37, 37, 0, 0, 0}, {84, 143, 53, 53, 3, 50, 51}, {85, 140, 51, 51, 0, 0, 0}, {86, 5, 53, 53, 3, 50, 51}, {87, 150, 51, 51, 0, 0, 0}, {89, 231, 28, 28, 0, 0, 0}, {90, 183, 56, 56, 0, 0, 0}, {91, 63, 42, 42, 0, 0, 0}, {92, 54, 33, 33, 0, 0, 0}, {93, 47, 81, 81, 0, 0, 0}, {100, 175, 26, 34, 0, 0, 0}, {101, 102, 32, 117, 0, 0, 0}, {102, 158, 32, 117, 0, 0, 0}, {103, 208, 20, 57, 0, 0, 0}, {104, 56, 32, 116, 0, 0, 0}, {105, 93, 62, 63, 0, 0, 0}, {106, 138, 44, 44, 0, 0, 0}, {107, 108, 64, 64, 0, 0, 0}, {108, 32, 84, 84, 0, 0, 0}, {109, 185, 9, 9, 0, 0, 0}, {110, 84, 254, 254, 3, 1, 2}, {111, 34, 16, 16, 0, 0, 0}, {112, 174, 12, 12, 0, 0, 0}, {113, 124, 36, 36, 0, 0, 0}, {114, 237, 44, 44, 0, 0, 0}, {115, 4, 64, 64, 0, 0, 0}, {116, 76, 22, 24, 0, 0, 0}, {117, 128, 6, 6, 3, 4, 5}, {118, 56, 14, 14, 0, 0, 0}, {119, 116, 12, 12, 3, 10, 11}, {120, 134, 97, 97, 0, 0, 0}, {121, 237, 2, 2, 3, 0, 1}, {122, 203, 2, 2, 3, 0, 1}, {123, 250, 113, 113, 3, 0, 1}, {124, 87, 35, 35, 0, 0, 0}, {125, 203, 6, 6, 0, 0, 0}, {126, 220, 79, 79, 0, 0, 0}, {127, 25, 35, 35, 0, 0, 0}, {128, 226, 35, 35, 0, 0, 0}, {129, 46, 22, 24, 0, 0, 0}, {130, 29, 13, 13, 0, 0, 0}, {131, 223, 255, 255, 0, 0, 0}, {132, 85, 14, 38, 0, 0, 0}, {133, 6, 18, 18, 0, 0, 0}, {134, 229, 43, 43, 0, 0, 0}, {135, 203, 8, 8, 0, 0, 0}, {136, 1, 22, 22, 0, 0, 0}, {137, 195, 14, 14, 0, 0, 0}, {138, 109, 36, 120, 0, 0, 0}, {139, 168, 43, 43, 3, 41, 42}, {140, 181, 41, 41, 0, 0, 0}, {141, 47, 32, 32, 0, 0, 0}, {142, 72, 243, 243, 0, 0, 0}, {143, 131, 14, 14, 0, 0, 0}, {144, 127, 93, 93, 0, 0, 0}, {146, 103, 100, 100, 0, 0, 0}, {147, 154, 36, 41, 0, 0, 0}, {148, 178, 60, 78, 0, 0, 0}, {149, 200, 30, 60, 0, 0, 0}, {162, 189, 8, 9, 0, 0, 0}, {230, 163, 42, 42, 0, 0, 0}, {231, 105, 40, 40, 0, 0, 0}, {232, 151, 63, 65, 0, 0, 0}, {233, 35, 182, 182, 0, 0, 0}, {234, 150, 40, 40, 0, 0, 0}, {235, 179, 42, 42, 0, 0, 0}, {241, 90, 32, 32, 0, 0, 0}, {242, 104, 52, 60, 0, 0, 0}, {243, 85, 53, 61, 1, 52, 0}, {244, 95, 6, 6, 0, 0, 0}, {245, 130, 2, 2, 0, 0, 0}, {246, 184, 38, 38, 0, 0, 0}, {247, 81, 19, 19, 0, 0, 0}, {248, 8, 254, 254, 3, 3, 4}, {249, 204, 36, 36, 0, 0, 0}, {250, 49, 30, 30, 0, 0, 0}, {251, 170, 18, 18, 0, 0, 0}, {252, 44, 18, 18, 0, 0, 0}, {253, 83, 51, 51, 0, 0, 0}, {254, 46, 9, 9, 0, 0, 0}, {256, 71, 42, 42, 3, 8, 9}, {257, 131, 9, 9, 0, 0, 0}, {258, 187, 32, 232, 3, 0, 1}, {259, 92, 235, 235, 0, 0, 0}, {260, 146, 5, 13, 0, 0, 0}, {261, 179, 27, 27, 0, 0, 0}, {262, 12, 18, 18, 0, 0, 0}, {263, 133, 255, 255, 0, 0, 0}, {264, 49, 28, 28, 0, 0, 0}, {265, 26, 16, 20, 0, 0, 0}, {266, 193, 255, 255, 3, 2, 3}, {267, 35, 255, 255, 3, 2, 3}, {268, 14, 4, 4, 3, 2, 3}, {269, 109, 213, 213, 0, 0, 0}, {270, 59, 19, 19, 0, 0, 0}, {299, 19, 96, 96, 0, 0, 0}, {300, 217, 22, 22, 0, 0, 0}, {301, 243, 58, 58, 0, 0, 0}, {310, 28, 17, 17, 0, 0, 0}, {311, 95, 116, 116, 0, 0, 0}, {320, 243, 20, 20, 3, 2, 3}, {321, 88, 2, 2, 3, 0, 1}, {322, 243, 149, 149, 0, 0, 0}, {323, 78, 147, 147, 3, 0, 1}, {324, 132, 146, 146, 0, 0, 0}, {330, 23, 158, 167, 0, 0, 0}, {331, 91, 230, 232, 0, 0, 0}, {332, 236, 239, 239, 0, 0, 0}, {333, 231, 109, 109, 0, 0, 0}, {334, 135, 14, 14, 0, 0, 0}, {335, 225, 24, 24, 0, 0, 0}, {340, 99, 70, 70, 0, 0, 0}, {350, 232, 20, 252, 0, 0, 0}, {360, 11, 25, 25, 0, 0, 0}, {365, 36, 255, 255, 0, 0, 0}, {370, 98, 73, 73, 0, 0, 0}, {371, 161, 50, 50, 0, 0, 0}, {375, 251, 140, 140, 0, 0, 0}, {380, 232, 20, 20, 0, 0, 0}, {385, 147, 133, 133, 3, 2, 3}, {390, 156, 238, 238, 0, 0, 0}, {395, 231, 222, 222, 0, 0, 0}, {400, 110, 254, 254, 3, 4, 5}, {401, 183, 6, 6, 3, 4, 5}, {9000, 113, 137, 137, 0, 0, 0}, {12900, 197, 22, 22, 0, 0, 0}, {12901, 16, 37, 37, 0, 0, 0}, {12902, 181, 31, 31, 0, 0, 0}, {12903, 149, 24, 24, 0, 0, 0}, {12904, 238, 21, 21, 0, 0, 0}, {12905, 56, 21, 21, 0, 0, 0}, {12915, 67, 252, 252, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_COMMON

// ENUM DEFINITIONS


/** @brief Micro air vehicle / autopilot classes. This identifies the individual model. */
#ifndef HAVE_ENUM_MAV_AUTOPILOT
#define HAVE_ENUM_MAV_AUTOPILOT
typedef enum MAV_AUTOPILOT
{
   MAV_AUTOPILOT_GENERIC=0, /* Generic autopilot, full support for everything | */
   MAV_AUTOPILOT_RESERVED=1, /* Reserved for future use. | */
   MAV_AUTOPILOT_SLUGS=2, /* SLUGS autopilot, http://slugsuav.soe.ucsc.edu | */
   MAV_AUTOPILOT_ARDUPILOTMEGA=3, /* ArduPilot - Plane/Copter/Rover/Sub/Tracker, http://ardupilot.org | */
   MAV_AUTOPILOT_OPENPILOT=4, /* OpenPilot, http://openpilot.org | */
   MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY=5, /* Generic autopilot only supporting simple waypoints | */
   MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY=6, /* Generic autopilot supporting waypoints and other simple navigation commands | */
   MAV_AUTOPILOT_GENERIC_MISSION_FULL=7, /* Generic autopilot supporting the full mission command set | */
   MAV_AUTOPILOT_INVALID=8, /* No valid autopilot, e.g. a GCS or other MAVLink component | */
   MAV_AUTOPILOT_PPZ=9, /* PPZ UAV - http://nongnu.org/paparazzi | */
   MAV_AUTOPILOT_UDB=10, /* UAV Dev Board | */
   MAV_AUTOPILOT_FP=11, /* FlexiPilot | */
   MAV_AUTOPILOT_PX4=12, /* PX4 Autopilot - http://px4.io/ | */
   MAV_AUTOPILOT_SMACCMPILOT=13, /* SMACCMPilot - http://smaccmpilot.org | */
   MAV_AUTOPILOT_AUTOQUAD=14, /* AutoQuad -- http://autoquad.org | */
   MAV_AUTOPILOT_ARMAZILA=15, /* Armazila -- http://armazila.com | */
   MAV_AUTOPILOT_AEROB=16, /* Aerob -- http://aerob.ru | */
   MAV_AUTOPILOT_ASLUAV=17, /* ASLUAV autopilot -- http://www.asl.ethz.ch | */
   MAV_AUTOPILOT_SMARTAP=18, /* SmartAP Autopilot - http://sky-drones.com | */
   MAV_AUTOPILOT_AIRRAILS=19, /* AirRails - http://uaventure.com | */
   MAV_AUTOPILOT_ENUM_END=20, /*  | */
} MAV_AUTOPILOT;
#endif

/** @brief MAVLINK component type reported in HEARTBEAT message. Flight controllers must report the type of the vehicle on which they are mounted (e.g. MAV_TYPE_OCTOROTOR). All other components must report a value appropriate for their type (e.g. a camera must use MAV_TYPE_CAMERA). */
#ifndef HAVE_ENUM_MAV_TYPE
#define HAVE_ENUM_MAV_TYPE
typedef enum MAV_TYPE
{
   MAV_TYPE_GENERIC=0, /* Generic micro air vehicle | */
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
   MAV_TYPE_GIMBAL=26, /* Gimbal | */
   MAV_TYPE_ADSB=27, /* ADSB system | */
   MAV_TYPE_PARAFOIL=28, /* Steerable, nonrigid airfoil | */
   MAV_TYPE_DODECAROTOR=29, /* Dodecarotor | */
   MAV_TYPE_CAMERA=30, /* Camera | */
   MAV_TYPE_CHARGING_STATION=31, /* Charging station | */
   MAV_TYPE_FLARM=32, /* FLARM collision avoidance system | */
   MAV_TYPE_SERVO=33, /* Servo | */
   MAV_TYPE_ENUM_END=34, /*  | */
} MAV_TYPE;
#endif

/** @brief These values define the type of firmware release.  These values indicate the first version or release of this type.  For example the first alpha release would be 64, the second would be 65. */
#ifndef HAVE_ENUM_FIRMWARE_VERSION_TYPE
#define HAVE_ENUM_FIRMWARE_VERSION_TYPE
typedef enum FIRMWARE_VERSION_TYPE
{
   FIRMWARE_VERSION_TYPE_DEV=0, /* development release | */
   FIRMWARE_VERSION_TYPE_ALPHA=64, /* alpha release | */
   FIRMWARE_VERSION_TYPE_BETA=128, /* beta release | */
   FIRMWARE_VERSION_TYPE_RC=192, /* release candidate | */
   FIRMWARE_VERSION_TYPE_OFFICIAL=255, /* official stable release | */
   FIRMWARE_VERSION_TYPE_ENUM_END=256, /*  | */
} FIRMWARE_VERSION_TYPE;
#endif

/** @brief Flags to report failure cases over the high latency telemtry. */
#ifndef HAVE_ENUM_HL_FAILURE_FLAG
#define HAVE_ENUM_HL_FAILURE_FLAG
typedef enum HL_FAILURE_FLAG
{
   HL_FAILURE_FLAG_GPS=1, /* GPS failure. | */
   HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE=2, /* Differential pressure sensor failure. | */
   HL_FAILURE_FLAG_ABSOLUTE_PRESSURE=4, /* Absolute pressure sensor failure. | */
   HL_FAILURE_FLAG_3D_ACCEL=8, /* Accelerometer sensor failure. | */
   HL_FAILURE_FLAG_3D_GYRO=16, /* Gyroscope sensor failure. | */
   HL_FAILURE_FLAG_3D_MAG=32, /* Magnetometer sensor failure. | */
   HL_FAILURE_FLAG_TERRAIN=64, /* Terrain subsystem failure. | */
   HL_FAILURE_FLAG_BATTERY=128, /* Battery failure/critical low battery. | */
   HL_FAILURE_FLAG_RC_RECEIVER=256, /* RC receiver failure/no rc connection. | */
   HL_FAILURE_FLAG_OFFBOARD_LINK=512, /* Offboard link failure. | */
   HL_FAILURE_FLAG_ENGINE=1024, /* Engine failure. | */
   HL_FAILURE_FLAG_GEOFENCE=2048, /* Geofence violation. | */
   HL_FAILURE_FLAG_ESTIMATOR=4096, /* Estimator failure, for example measurement rejection or large variances. | */
   HL_FAILURE_FLAG_MISSION=8192, /* Mission failure. | */
   HL_FAILURE_FLAG_ENUM_END=8193, /*  | */
} HL_FAILURE_FLAG;
#endif

/** @brief These flags encode the MAV mode. */
#ifndef HAVE_ENUM_MAV_MODE_FLAG
#define HAVE_ENUM_MAV_MODE_FLAG
typedef enum MAV_MODE_FLAG
{
   MAV_MODE_FLAG_CUSTOM_MODE_ENABLED=1, /* 0b00000001 Reserved for future use. | */
   MAV_MODE_FLAG_TEST_ENABLED=2, /* 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. | */
   MAV_MODE_FLAG_AUTO_ENABLED=4, /* 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. | */
   MAV_MODE_FLAG_GUIDED_ENABLED=8, /* 0b00001000 guided mode enabled, system flies waypoints / mission items. | */
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
typedef enum MAV_MODE_FLAG_DECODE_POSITION
{
   MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE=1, /* Eighth bit: 00000001 | */
   MAV_MODE_FLAG_DECODE_POSITION_TEST=2, /* Seventh bit: 00000010 | */
   MAV_MODE_FLAG_DECODE_POSITION_AUTO=4, /* Sixth bit:   00000100 | */
   MAV_MODE_FLAG_DECODE_POSITION_GUIDED=8, /* Fifth bit:  00001000 | */
   MAV_MODE_FLAG_DECODE_POSITION_STABILIZE=16, /* Fourth bit: 00010000 | */
   MAV_MODE_FLAG_DECODE_POSITION_HIL=32, /* Third bit:  00100000 | */
   MAV_MODE_FLAG_DECODE_POSITION_MANUAL=64, /* Second bit: 01000000 | */
   MAV_MODE_FLAG_DECODE_POSITION_SAFETY=128, /* First bit:  10000000 | */
   MAV_MODE_FLAG_DECODE_POSITION_ENUM_END=129, /*  | */
} MAV_MODE_FLAG_DECODE_POSITION;
#endif

/** @brief Actions that may be specified in MAV_CMD_OVERRIDE_GOTO to override mission execution. */
#ifndef HAVE_ENUM_MAV_GOTO
#define HAVE_ENUM_MAV_GOTO
typedef enum MAV_GOTO
{
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
typedef enum MAV_MODE
{
   MAV_MODE_PREFLIGHT=0, /* System is not ready to fly, booting, calibrating, etc. No flag is set. | */
   MAV_MODE_MANUAL_DISARMED=64, /* System is allowed to be active, under manual (RC) control, no stabilization | */
   MAV_MODE_TEST_DISARMED=66, /* UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. | */
   MAV_MODE_STABILIZE_DISARMED=80, /* System is allowed to be active, under assisted RC control. | */
   MAV_MODE_GUIDED_DISARMED=88, /* System is allowed to be active, under autonomous control, manual setpoint | */
   MAV_MODE_AUTO_DISARMED=92, /* System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints) | */
   MAV_MODE_MANUAL_ARMED=192, /* System is allowed to be active, under manual (RC) control, no stabilization | */
   MAV_MODE_TEST_ARMED=194, /* UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. | */
   MAV_MODE_STABILIZE_ARMED=208, /* System is allowed to be active, under assisted RC control. | */
   MAV_MODE_GUIDED_ARMED=216, /* System is allowed to be active, under autonomous control, manual setpoint | */
   MAV_MODE_AUTO_ARMED=220, /* System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints) | */
   MAV_MODE_ENUM_END=221, /*  | */
} MAV_MODE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_STATE
#define HAVE_ENUM_MAV_STATE
typedef enum MAV_STATE
{
   MAV_STATE_UNINIT=0, /* Uninitialized system, state is unknown. | */
   MAV_STATE_BOOT=1, /* System is booting up. | */
   MAV_STATE_CALIBRATING=2, /* System is calibrating and not flight-ready. | */
   MAV_STATE_STANDBY=3, /* System is grounded and on standby. It can be launched any time. | */
   MAV_STATE_ACTIVE=4, /* System is active and might be already airborne. Motors are engaged. | */
   MAV_STATE_CRITICAL=5, /* System is in a non-normal flight mode. It can however still navigate. | */
   MAV_STATE_EMERGENCY=6, /* System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down. | */
   MAV_STATE_POWEROFF=7, /* System just initialized its power-down sequence, will shut down now. | */
   MAV_STATE_FLIGHT_TERMINATION=8, /* System is terminating itself. | */
   MAV_STATE_ENUM_END=9, /*  | */
} MAV_STATE;
#endif

/** @brief Component ids (values) for the different types and instances of onboard hardware/software that might make up a MAVLink system (autopilot, cameras, servos, GPS systems, avoidance systems etc.).
      Components must use the appropriate ID in their source address when sending messages. Components can also use IDs to determine if they are the intended recipient of an incoming message. The MAV_COMP_ID_ALL value is used to indicate messages that must be processed by all components.
      When creating new entries, components that can have multiple instances (e.g. cameras, servos etc.) should be allocated sequential values. An appropriate number of values should be left free after these components to allow the number of instances to be expanded. */
#ifndef HAVE_ENUM_MAV_COMPONENT
#define HAVE_ENUM_MAV_COMPONENT
typedef enum MAV_COMPONENT
{
   MAV_COMP_ID_ALL=0, /* Target id (target_component) used to broadcast messages to all components of the receiving system. Components should attempt to process messages with this component ID and forward to components on any other interfaces. Note: This is not a valid *source* component id for a message. | */
   MAV_COMP_ID_AUTOPILOT1=1, /* System flight controller component ("autopilot"). Only one autopilot is expected in a particular system. | */
   MAV_COMP_ID_USER1=25, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER2=26, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER3=27, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER4=28, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER5=29, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER6=30, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER7=31, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER8=32, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER9=33, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER10=34, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER11=35, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER12=36, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER13=37, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER14=38, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER15=39, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USE16=40, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER17=41, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER18=42, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER19=43, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER20=44, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER21=45, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER22=46, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER23=47, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER24=48, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER25=49, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER26=50, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER27=51, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER28=52, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER29=53, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER30=54, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER31=55, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER32=56, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER33=57, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER34=58, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER35=59, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER36=60, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER37=61, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER38=62, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER39=63, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER40=64, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER41=65, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER42=66, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER43=67, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER44=68, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER45=69, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER46=70, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER47=71, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER48=72, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER49=73, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER50=74, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER51=75, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER52=76, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER53=77, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER54=78, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER55=79, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER56=80, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER57=81, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER58=82, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER59=83, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER60=84, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER61=85, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER62=86, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER63=87, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER64=88, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER65=89, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER66=90, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER67=91, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER68=92, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER69=93, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER70=94, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER71=95, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER72=96, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER73=97, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER74=98, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_USER75=99, /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | */
   MAV_COMP_ID_CAMERA=100, /* Camera #1. | */
   MAV_COMP_ID_CAMERA2=101, /* Camera #2. | */
   MAV_COMP_ID_CAMERA3=102, /* Camera #3. | */
   MAV_COMP_ID_CAMERA4=103, /* Camera #4. | */
   MAV_COMP_ID_CAMERA5=104, /* Camera #5. | */
   MAV_COMP_ID_CAMERA6=105, /* Camera #6. | */
   MAV_COMP_ID_SERVO1=140, /* Servo #1. | */
   MAV_COMP_ID_SERVO2=141, /* Servo #2. | */
   MAV_COMP_ID_SERVO3=142, /* Servo #3. | */
   MAV_COMP_ID_SERVO4=143, /* Servo #4. | */
   MAV_COMP_ID_SERVO5=144, /* Servo #5. | */
   MAV_COMP_ID_SERVO6=145, /* Servo #6. | */
   MAV_COMP_ID_SERVO7=146, /* Servo #7. | */
   MAV_COMP_ID_SERVO8=147, /* Servo #8. | */
   MAV_COMP_ID_SERVO9=148, /* Servo #9. | */
   MAV_COMP_ID_SERVO10=149, /* Servo #10. | */
   MAV_COMP_ID_SERVO11=150, /* Servo #11. | */
   MAV_COMP_ID_SERVO12=151, /* Servo #12. | */
   MAV_COMP_ID_SERVO13=152, /* Servo #13. | */
   MAV_COMP_ID_SERVO14=153, /* Servo #14. | */
   MAV_COMP_ID_GIMBAL=154, /* Gimbal #1. | */
   MAV_COMP_ID_LOG=155, /* Logging component. | */
   MAV_COMP_ID_ADSB=156, /* Automatic Dependent Surveillance-Broadcast (ADS-B) component. | */
   MAV_COMP_ID_OSD=157, /* On Screen Display (OSD) devices for video links. | */
   MAV_COMP_ID_PERIPHERAL=158, /* Generic autopilot peripheral component ID. Meant for devices that do not implement the parameter microservice. | */
   MAV_COMP_ID_QX1_GIMBAL=159, /* Gimbal ID for QX1. | */
   MAV_COMP_ID_FLARM=160, /* FLARM collision alert component. | */
   MAV_COMP_ID_GIMBAL2=171, /* Gimbal #2. | */
   MAV_COMP_ID_GIMBAL3=172, /* Gimbal #3. | */
   MAV_COMP_ID_GIMBAL4=173, /* Gimbal #4 | */
   MAV_COMP_ID_GIMBAL5=174, /* Gimbal #5. | */
   MAV_COMP_ID_GIMBAL6=175, /* Gimbal #6. | */
   MAV_COMP_ID_MISSIONPLANNER=190, /* Component that can generate/supply a mission flight plan (e.g. GCS or developer API). | */
   MAV_COMP_ID_PATHPLANNER=195, /* Component that finds an optimal path between points based on a certain constraint (e.g. minimum snap, shortest path, cost, etc.). | */
   MAV_COMP_ID_OBSTACLE_AVOIDANCE=196, /* Component that plans a collision free path between two points. | */
   MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY=197, /* Component that provides position estimates using VIO techniques. | */
   MAV_COMP_ID_IMU=200, /* Inertial Measurement Unit (IMU) #1. | */
   MAV_COMP_ID_IMU_2=201, /* Inertial Measurement Unit (IMU) #2. | */
   MAV_COMP_ID_IMU_3=202, /* Inertial Measurement Unit (IMU) #3. | */
   MAV_COMP_ID_GPS=220, /* GPS #1. | */
   MAV_COMP_ID_GPS2=221, /* GPS #2. | */
   MAV_COMP_ID_UDP_BRIDGE=240, /* Component to bridge MAVLink to UDP (i.e. from a UART). | */
   MAV_COMP_ID_UART_BRIDGE=241, /* Component to bridge to UART (i.e. from UDP). | */
   MAV_COMP_ID_TUNNEL_NODE=242, /* Component handling TUNNEL messages (e.g. vendor specific GUI of a component). | */
   MAV_COMP_ID_SYSTEM_CONTROL=250, /* Component for handling system messages (e.g. to ARM, takeoff, etc.). | */
   MAV_COMPONENT_ENUM_END=251, /*  | */
} MAV_COMPONENT;
#endif

/** @brief These encode the sensors whose status is sent as part of the SYS_STATUS message. */
#ifndef HAVE_ENUM_MAV_SYS_STATUS_SENSOR
#define HAVE_ENUM_MAV_SYS_STATUS_SENSOR
typedef enum MAV_SYS_STATUS_SENSOR
{
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
   MAV_SYS_STATUS_SENSOR_BATTERY=33554432, /* 0x2000000 Battery | */
   MAV_SYS_STATUS_SENSOR_PROXIMITY=67108864, /* 0x4000000 Proximity | */
   MAV_SYS_STATUS_SENSOR_SATCOM=134217728, /* 0x8000000 Satellite Communication  | */
   MAV_SYS_STATUS_SENSOR_ENUM_END=134217729, /*  | */
} MAV_SYS_STATUS_SENSOR;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_FRAME
#define HAVE_ENUM_MAV_FRAME
typedef enum MAV_FRAME
{
   MAV_FRAME_GLOBAL=0, /* Global (WGS84) coordinate frame + MSL altitude. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL). | */
   MAV_FRAME_LOCAL_NED=1, /* Local coordinate frame, Z-down (x: north, y: east, z: down). | */
   MAV_FRAME_MISSION=2, /* NOT a coordinate frame, indicates a mission command. | */
   MAV_FRAME_GLOBAL_RELATIVE_ALT=3, /* Global (WGS84) coordinate frame + altitude relative to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location. | */
   MAV_FRAME_LOCAL_ENU=4, /* Local coordinate frame, Z-up (x: east, y: north, z: up). | */
   MAV_FRAME_GLOBAL_INT=5, /* Global (WGS84) coordinate frame (scaled) + MSL altitude. First value / x: latitude in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL). | */
   MAV_FRAME_GLOBAL_RELATIVE_ALT_INT=6, /* Global (WGS84) coordinate frame (scaled) + altitude relative to the home position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with 0 being at the altitude of the home location. | */
   MAV_FRAME_LOCAL_OFFSET_NED=7, /* Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position. | */
   MAV_FRAME_BODY_NED=8, /* Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right. | */
   MAV_FRAME_BODY_OFFSET_NED=9, /* Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east. | */
   MAV_FRAME_GLOBAL_TERRAIN_ALT=10, /* Global (WGS84) coordinate frame with AGL altitude (at the waypoint coordinate). First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model. | */
   MAV_FRAME_GLOBAL_TERRAIN_ALT_INT=11, /* Global (WGS84) coordinate frame (scaled) with AGL altitude (at the waypoint coordinate). First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground level in terrain model. | */
   MAV_FRAME_BODY_FRD=12, /* Body fixed frame of reference, Z-down (x: forward, y: right, z: down). | */
   MAV_FRAME_BODY_FLU=13, /* Body fixed frame of reference, Z-up (x: forward, y: left, z: up). | */
   MAV_FRAME_MOCAP_NED=14, /* Odometry local coordinate frame of data given by a motion capture system, Z-down (x: north, y: east, z: down). | */
   MAV_FRAME_MOCAP_ENU=15, /* Odometry local coordinate frame of data given by a motion capture system, Z-up (x: east, y: north, z: up). | */
   MAV_FRAME_VISION_NED=16, /* Odometry local coordinate frame of data given by a vision estimation system, Z-down (x: north, y: east, z: down). | */
   MAV_FRAME_VISION_ENU=17, /* Odometry local coordinate frame of data given by a vision estimation system, Z-up (x: east, y: north, z: up). | */
   MAV_FRAME_ESTIM_NED=18, /* Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-down (x: north, y: east, z: down). | */
   MAV_FRAME_ESTIM_ENU=19, /* Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-up (x: east, y: noth, z: up). | */
   MAV_FRAME_LOCAL_FRD=20, /* Forward, Right, Down coordinate frame. This is a local frame with Z-down and arbitrary F/R alignment (i.e. not aligned with NED/earth frame). | */
   MAV_FRAME_LOCAL_FLU=21, /* Forward, Left, Up coordinate frame. This is a local frame with Z-up and arbitrary F/L alignment (i.e. not aligned with ENU/earth frame). | */
   MAV_FRAME_ENUM_END=22, /*  | */
} MAV_FRAME;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAVLINK_DATA_STREAM_TYPE
#define HAVE_ENUM_MAVLINK_DATA_STREAM_TYPE
typedef enum MAVLINK_DATA_STREAM_TYPE
{
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
typedef enum FENCE_ACTION
{
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
typedef enum FENCE_BREACH
{
   FENCE_BREACH_NONE=0, /* No last fence breach | */
   FENCE_BREACH_MINALT=1, /* Breached minimum altitude | */
   FENCE_BREACH_MAXALT=2, /* Breached maximum altitude | */
   FENCE_BREACH_BOUNDARY=3, /* Breached fence boundary | */
   FENCE_BREACH_ENUM_END=4, /*  | */
} FENCE_BREACH;
#endif

/** @brief Actions being taken to mitigate/prevent fence breach */
#ifndef HAVE_ENUM_FENCE_MITIGATE
#define HAVE_ENUM_FENCE_MITIGATE
typedef enum FENCE_MITIGATE
{
   FENCE_MITIGATE_UNKNOWN=0, /* Unknown | */
   FENCE_MITIGATE_NONE=1, /* No actions being taken | */
   FENCE_MITIGATE_VEL_LIMIT=2, /* Velocity limiting active to prevent breach | */
   FENCE_MITIGATE_ENUM_END=3, /*  | */
} FENCE_MITIGATE;
#endif

/** @brief Enumeration of possible mount operation modes */
#ifndef HAVE_ENUM_MAV_MOUNT_MODE
#define HAVE_ENUM_MAV_MOUNT_MODE
typedef enum MAV_MOUNT_MODE
{
   MAV_MOUNT_MODE_RETRACT=0, /* Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization | */
   MAV_MOUNT_MODE_NEUTRAL=1, /* Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory. | */
   MAV_MOUNT_MODE_MAVLINK_TARGETING=2, /* Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization | */
   MAV_MOUNT_MODE_RC_TARGETING=3, /* Load neutral position and start RC Roll,Pitch,Yaw control with stabilization | */
   MAV_MOUNT_MODE_GPS_POINT=4, /* Load neutral position and start to point to Lat,Lon,Alt | */
   MAV_MOUNT_MODE_ENUM_END=5, /*  | */
} MAV_MOUNT_MODE;
#endif

/** @brief Generalized UAVCAN node health */
#ifndef HAVE_ENUM_UAVCAN_NODE_HEALTH
#define HAVE_ENUM_UAVCAN_NODE_HEALTH
typedef enum UAVCAN_NODE_HEALTH
{
   UAVCAN_NODE_HEALTH_OK=0, /* The node is functioning properly. | */
   UAVCAN_NODE_HEALTH_WARNING=1, /* A critical parameter went out of range or the node has encountered a minor failure. | */
   UAVCAN_NODE_HEALTH_ERROR=2, /* The node has encountered a major failure. | */
   UAVCAN_NODE_HEALTH_CRITICAL=3, /* The node has suffered a fatal malfunction. | */
   UAVCAN_NODE_HEALTH_ENUM_END=4, /*  | */
} UAVCAN_NODE_HEALTH;
#endif

/** @brief Generalized UAVCAN node mode */
#ifndef HAVE_ENUM_UAVCAN_NODE_MODE
#define HAVE_ENUM_UAVCAN_NODE_MODE
typedef enum UAVCAN_NODE_MODE
{
   UAVCAN_NODE_MODE_OPERATIONAL=0, /* The node is performing its primary functions. | */
   UAVCAN_NODE_MODE_INITIALIZATION=1, /* The node is initializing; this mode is entered immediately after startup. | */
   UAVCAN_NODE_MODE_MAINTENANCE=2, /* The node is under maintenance. | */
   UAVCAN_NODE_MODE_SOFTWARE_UPDATE=3, /* The node is in the process of updating its software. | */
   UAVCAN_NODE_MODE_OFFLINE=7, /* The node is no longer available online. | */
   UAVCAN_NODE_MODE_ENUM_END=8, /*  | */
} UAVCAN_NODE_MODE;
#endif

/** @brief Flags to indicate the status of camera storage. */
#ifndef HAVE_ENUM_STORAGE_STATUS
#define HAVE_ENUM_STORAGE_STATUS
typedef enum STORAGE_STATUS
{
   STORAGE_STATUS_EMPTY=0, /* Storage is missing (no microSD card loaded for example.) | */
   STORAGE_STATUS_UNFORMATTED=1, /* Storage present but unformatted. | */
   STORAGE_STATUS_READY=2, /* Storage present and ready. | */
   STORAGE_STATUS_NOT_SUPPORTED=3, /* Camera does not supply storage status information. Capacity information in STORAGE_INFORMATION fields will be ignored. | */
   STORAGE_STATUS_ENUM_END=4, /*  | */
} STORAGE_STATUS;
#endif

/** @brief Yaw behaviour during orbit flight. */
#ifndef HAVE_ENUM_ORBIT_YAW_BEHAVIOUR
#define HAVE_ENUM_ORBIT_YAW_BEHAVIOUR
typedef enum ORBIT_YAW_BEHAVIOUR
{
   ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER=0, /* Vehicle front points to the center (default). | */
   ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING=1, /* Vehicle front holds heading when message received. | */
   ORBIT_YAW_BEHAVIOUR_UNCONTROLLED=2, /* Yaw uncontrolled. | */
   ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE=3, /* Vehicle front follows flight path (tangential to circle). | */
   ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED=4, /* Yaw controlled by RC input. | */
   ORBIT_YAW_BEHAVIOUR_ENUM_END=5, /*  | */
} ORBIT_YAW_BEHAVIOUR;
#endif

/** @brief Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data. See https://mavlink.io/en/guide/xml_schema.html#MAV_CMD for information about the structure of the MAV_CMD entries */
#ifndef HAVE_ENUM_MAV_CMD
#define HAVE_ENUM_MAV_CMD
typedef enum MAV_CMD
{
   MAV_CMD_NAV_WAYPOINT=16, /* Navigate to waypoint. |Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)| Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached)| 0 to pass through the WP, if > 0 radius to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| Desired yaw angle at waypoint (rotary wing). NaN for unchanged.| Latitude| Longitude| Altitude|  */
   MAV_CMD_NAV_LOITER_UNLIM=17, /* Loiter around this waypoint an unlimited amount of time |Empty| Empty| Radius around waypoint. If positive loiter clockwise, else counter-clockwise| Desired yaw angle. NaN for unchanged.| Latitude| Longitude| Altitude|  */
   MAV_CMD_NAV_LOITER_TURNS=18, /* Loiter around this waypoint for X turns |Number of turns.| Empty| Radius around waypoint. If positive loiter clockwise, else counter-clockwise| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle. NaN for unchanged.| Latitude| Longitude| Altitude|  */
   MAV_CMD_NAV_LOITER_TIME=19, /* Loiter around this waypoint for X seconds |Loiter time.| Empty| Radius around waypoint. If positive loiter clockwise, else counter-clockwise.| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle.  NaN for unchanged.| Latitude| Longitude| Altitude|  */
   MAV_CMD_NAV_RETURN_TO_LAUNCH=20, /* Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_NAV_LAND=21, /* Land at location. |Minimum target altitude if landing is aborted (0 = undefined/use system default).| Precision land mode.| Empty.| Desired yaw angle. NaN for unchanged.| Latitude.| Longitude.| Landing altitude (ground level in current frame).|  */
   MAV_CMD_NAV_TAKEOFF=22, /* Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer. NaN for unchanged.| Latitude| Longitude| Altitude|  */
   MAV_CMD_NAV_LAND_LOCAL=23, /* Land at local position (local frame only) |Landing target number (if available)| Maximum accepted offset from desired landing position - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position and the position where the vehicle is about to land| Landing descend rate| Desired yaw angle| Y-axis position| X-axis position| Z-axis / ground level position|  */
   MAV_CMD_NAV_TAKEOFF_LOCAL=24, /* Takeoff from local position (local frame only) |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Takeoff ascend rate| Yaw angle (if magnetometer or another yaw estimation source present), ignored without one of these| Y-axis position| X-axis position| Z-axis position|  */
   MAV_CMD_NAV_FOLLOW=25, /* Vehicle following, i.e. this waypoint represents the position of a moving vehicle |Following logic to use (e.g. loitering or sinusoidal following) - depends on specific autopilot implementation| Ground speed of vehicle to be followed| Radius around waypoint. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
   MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT=30, /* Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached. |Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing, command completes when at or above this command's altitude, 2 = Descending, command completes when at or below this command's altitude.| Empty| Empty| Empty| Empty| Empty| Desired altitude|  */
   MAV_CMD_NAV_LOITER_TO_ALT=31, /* Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached.  Additionally, if the Heading Required parameter is non-zero the  aircraft will not leave the loiter until heading toward the next waypoint. |Heading Required (0 = False)| Radius. If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.| Empty| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location| Latitude| Longitude| Altitude|  */
   MAV_CMD_DO_FOLLOW=32, /* Begin following a target |System ID (of the FOLLOW_TARGET beacon). Send 0 to disable follow-me and return to the default position hold mode.| RESERVED| RESERVED| Altitude mode: 0: Keep current altitude, 1: keep altitude difference to target, 2: go to a fixed altitude above home.| Altitude above home. (used if mode=2)| RESERVED| Time to land in which the MAV should go to the default position hold mode after a message RX timeout.|  */
   MAV_CMD_DO_FOLLOW_REPOSITION=33, /* Reposition the MAV after a follow target command has been sent |Camera q1 (where 0 is on the ray from the camera to the tracking device)| Camera q2| Camera q3| Camera q4| altitude offset from target| X offset from target| Y offset from target|  */
   MAV_CMD_DO_ORBIT=34, /* Start orbiting on the circumference of a circle defined by the parameters. Setting any value NaN results in using defaults. |Radius of the circle. positive: Orbit clockwise. negative: Orbit counter-clockwise.| Tangential Velocity. NaN: Vehicle configuration default.| Yaw behavior of the vehicle.| Reserved (e.g. for dynamic center beacon options)| Center point latitude (if no MAV_FRAME specified) / X coordinate according to MAV_FRAME. NaN: Use current vehicle position or current center if already orbiting.| Center point longitude (if no MAV_FRAME specified) / Y coordinate according to MAV_FRAME. NaN: Use current vehicle position or current center if already orbiting.| Center point altitude (MSL) (if no MAV_FRAME specified) / Z coordinate according to MAV_FRAME. NaN: Use current vehicle position or current center if already orbiting.|  */
   MAV_CMD_NAV_ROI=80, /* Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of interest mode.| Waypoint index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  */
   MAV_CMD_NAV_PATHPLANNING=81, /* Control autonomous path planning on the MAV. |0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning| 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid| Empty| Yaw angle at goal| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  */
   MAV_CMD_NAV_SPLINE_WAYPOINT=82, /* Navigate to waypoint using a spline path. |Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)| Empty| Empty| Empty| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  */
   MAV_CMD_NAV_VTOL_TAKEOFF=84, /* Takeoff from ground using VTOL mode, and transition to forward flight with specified heading. |Empty| Front transition heading.| Empty| Yaw angle. NaN for unchanged.| Latitude| Longitude| Altitude|  */
   MAV_CMD_NAV_VTOL_LAND=85, /* Land using VTOL mode |Empty| Empty| Approach altitude (with the same reference as the Altitude field). NaN if unspecified.| Yaw angle. NaN for unchanged.| Latitude| Longitude| Altitude (ground level)|  */
   MAV_CMD_NAV_GUIDED_ENABLE=92, /* hand control over to an external controller |On / Off (> 0.5f on)| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_NAV_DELAY=93, /* Delay the next navigation command a number of seconds or until a specified time |Delay (-1 to enable time-of-day fields)| hour (24h format, UTC, -1 to ignore)| minute (24h format, UTC, -1 to ignore)| second (24h format, UTC)| Empty| Empty| Empty|  */
   MAV_CMD_NAV_PAYLOAD_PLACE=94, /* Descend and place payload. Vehicle moves to specified location, descends until it detects a hanging payload has reached the ground, and then releases the payload. If ground is not detected before the reaching the maximum descent value (param1), the command will complete without releasing the payload. |Maximum distance to descend.| Empty| Empty| Empty| Latitude| Longitude| Altitude|  */
   MAV_CMD_NAV_LAST=95, /* NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_CONDITION_DELAY=112, /* Delay mission state machine. |Delay| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_CONDITION_CHANGE_ALT=113, /* Ascend/descend at rate.  Delay mission state machine until desired altitude reached. |Descent / Ascend rate.| Empty| Empty| Empty| Empty| Empty| Finish Altitude|  */
   MAV_CMD_CONDITION_DISTANCE=114, /* Delay mission state machine until within desired distance of next NAV point. |Distance.| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_CONDITION_YAW=115, /* Reach a certain target angle. |target angle, 0 is north| angular speed| direction: -1: counter clockwise, 1: clockwise| 0: absolute angle, 1: relative offset| Empty| Empty| Empty|  */
   MAV_CMD_CONDITION_LAST=159, /* NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_SET_MODE=176, /* Set system mode. |Mode| Custom mode - this is system specific, please refer to the individual autopilot specifications for details.| Custom sub mode - this is system specific, please refer to the individual autopilot specifications for details.| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_JUMP=177, /* Jump to the desired command in the mission list.  Repeat this action only the specified number of times |Sequence number| Repeat count| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_CHANGE_SPEED=178, /* Change speed and/or throttle set points. |Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)| Speed (-1 indicates no change)| Throttle (-1 indicates no change)| 0: absolute, 1: relative| Empty| Empty| Empty|  */
   MAV_CMD_DO_SET_HOME=179, /* Changes the home location either to the current location or a specified location. |Use current (1=use current location, 0=use specified location)| Empty| Empty| Empty| Latitude| Longitude| Altitude|  */
   MAV_CMD_DO_SET_PARAMETER=180, /* Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. |Parameter number| Parameter value| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_SET_RELAY=181, /* Set a relay to a condition. |Relay instance number.| Setting. (1=on, 0=off, others possible depending on system hardware)| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_REPEAT_RELAY=182, /* Cycle a relay on and off for a desired number of cycles with a desired period. |Relay instance number.| Cycle count.| Cycle time.| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_SET_SERVO=183, /* Set a servo to a desired PWM value. |Servo instance number.| Pulse Width Modulation.| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_REPEAT_SERVO=184, /* Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. |Servo instance number.| Pulse Width Modulation.| Cycle count.| Cycle time.| Empty| Empty| Empty|  */
   MAV_CMD_DO_FLIGHTTERMINATION=185, /* Terminate flight immediately |Flight termination activated if > 0.5| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_CHANGE_ALTITUDE=186, /* Change altitude set point. |Altitude.| Frame of new altitude.| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_LAND_START=189, /* Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0 if not needed. If specified then it will be used to help find the closest landing sequence. |Empty| Empty| Empty| Empty| Latitude| Longitude| Empty|  */
   MAV_CMD_DO_RALLY_LAND=190, /* Mission command to perform a landing from a rally point. |Break altitude| Landing speed| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_GO_AROUND=191, /* Mission command to safely abort an autonomous landing. |Altitude| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_REPOSITION=192, /* Reposition the vehicle to a specific WGS84 global position. |Ground speed, less than 0 (-1) for default| Bitmask of option flags.| Reserved| Yaw heading, NaN for unchanged. For planes indicates loiter direction (0: clockwise, 1: counter clockwise)| Latitude| Longitude| Altitude (meters)|  */
   MAV_CMD_DO_PAUSE_CONTINUE=193, /* If in a GPS controlled position mode, hold the current position or continue. |0: Pause current mission or reposition command, hold current position. 1: Continue mission. A VTOL capable vehicle should enter hover mode (multicopter and VTOL planes). A plane should loiter with the default loiter radius.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
   MAV_CMD_DO_SET_REVERSE=194, /* Set moving direction to forward or reverse. |Direction (0=Forward, 1=Reverse)| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_SET_ROI_LOCATION=195, /* Sets the region of interest (ROI) to a location. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Empty| Empty| Empty| Empty| Latitude| Longitude| Altitude|  */
   MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET=196, /* Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Empty| Empty| Empty| Empty| pitch offset from next waypoint| roll offset from next waypoint| yaw offset from next waypoint|  */
   MAV_CMD_DO_SET_ROI_NONE=197, /* Cancels any previous ROI command returning the vehicle/sensors to default flight characteristics. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_CONTROL_VIDEO=200, /* Control onboard camera system. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|  */
   MAV_CMD_DO_SET_ROI=201, /* Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of interest mode.| Waypoint index/ target ID (depends on param 1).| Region of interest index. (allows a vehicle to manage multiple ROI's)| Empty| MAV_ROI_WPNEXT: pitch offset from next waypoint, MAV_ROI_LOCATION: latitude| MAV_ROI_WPNEXT: roll offset from next waypoint, MAV_ROI_LOCATION: longitude| MAV_ROI_WPNEXT: yaw offset from next waypoint, MAV_ROI_LOCATION: altitude|  */
   MAV_CMD_DO_DIGICAM_CONFIGURE=202, /* Configure digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ). |Modes: P, TV, AV, M, Etc.| Shutter speed: Divisor number for one second.| Aperture: F stop number.| ISO number e.g. 80, 100, 200, Etc.| Exposure type enumerator.| Command Identity.| Main engine cut-off time before camera trigger. (0 means no cut-off)|  */
   MAV_CMD_DO_DIGICAM_CONTROL=203, /* Control digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ). |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Test shot identifier. If set to 1, image will only be captured, but not counted towards internal frame count.|  */
   MAV_CMD_DO_MOUNT_CONFIGURE=204, /* Mission command to configure a camera or antenna mount |Mount operation mode| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| roll input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)| pitch input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)| yaw input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)|  */
   MAV_CMD_DO_MOUNT_CONTROL=205, /* Mission command to control a camera or antenna mount |pitch depending on mount mode (degrees or degrees/second depending on pitch input).| roll depending on mount mode (degrees or degrees/second depending on roll input).| yaw depending on mount mode (degrees or degrees/second depending on yaw input).| altitude depending on mount mode.| latitude, set if appropriate mount mode.| longitude, set if appropriate mount mode.| Mount mode.|  */
   MAV_CMD_DO_SET_CAM_TRIGG_DIST=206, /* Mission command to set camera trigger distance for this flight. The camera is triggered each time this distance is exceeded. This command can also be used to set the shutter integration time for the camera. |Camera trigger distance. 0 to stop triggering.| Camera shutter integration time. -1 or 0 to ignore| Trigger camera once immediately. (0 = no trigger, 1 = trigger)| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_FENCE_ENABLE=207, /* Mission command to enable the geofence |enable? (0=disable, 1=enable, 2=disable_floor_only)| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_PARACHUTE=208, /* Mission command to trigger a parachute |action| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_MOTOR_TEST=209, /* Mission command to perform motor test. |Motor instance number. (from 1 to max number of motors on the vehicle)| Throttle type.| Throttle.| Timeout.| Motor count. (number of motors to test to test in sequence, waiting for the timeout above between them; 0=1 motor, 1=1 motor, 2=2 motors...)| Motor test order.| Empty|  */
   MAV_CMD_DO_INVERTED_FLIGHT=210, /* Change to/from inverted flight. |Inverted flight. (0=normal, 1=inverted)| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_NAV_SET_YAW_SPEED=213, /* Sets a desired vehicle turn angle and speed change. |Yaw angle to adjust steering by.| Speed.| Final angle. (0=absolute, 1=relative)| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL=214, /* Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is triggered each time this interval expires. This command can also be used to set the shutter integration time for the camera. |Camera trigger cycle time. -1 or 0 to ignore.| Camera shutter integration time. Should be less than trigger cycle time. -1 or 0 to ignore.| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_MOUNT_CONTROL_QUAT=220, /* Mission command to control a camera or antenna mount, using a quaternion as reference. |quaternion param q1, w (1 in null-rotation)| quaternion param q2, x (0 in null-rotation)| quaternion param q3, y (0 in null-rotation)| quaternion param q4, z (0 in null-rotation)| Empty| Empty| Empty|  */
   MAV_CMD_DO_GUIDED_MASTER=221, /* set id of master controller |System ID| Component ID| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_GUIDED_LIMITS=222, /* Set limits for external control |Timeout - maximum time that external controller will be allowed to control vehicle. 0 means no timeout.| Altitude (MSL) min - if vehicle moves below this alt, the command will be aborted and the mission will continue. 0 means no lower altitude limit.| Altitude (MSL) max - if vehicle moves above this alt, the command will be aborted and the mission will continue. 0 means no upper altitude limit.| Horizontal move limit - if vehicle moves more than this distance from its location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal move limit.| Empty| Empty| Empty|  */
   MAV_CMD_DO_ENGINE_CONTROL=223, /* Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine state. It is intended for vehicles with internal combustion engines |0: Stop engine, 1:Start Engine| 0: Warm start, 1:Cold start. Controls use of choke where applicable| Height delay. This is for commanding engine start only after the vehicle has gained the specified height. Used in VTOL vehicles during takeoff to start engine after the aircraft is off the ground. Zero for no delay.| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_SET_MISSION_CURRENT=224, /* Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between). |Mission sequence value to set| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_LAST=240, /* NOP - This command is only used to mark the upper limit of the DO commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_PREFLIGHT_CALIBRATION=241, /* Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature Calibration, only one sensor should be set in a single message and all others should be zero. |1: gyro calibration, 3: gyro temperature calibration| 1: magnetometer calibration| 1: ground pressure calibration| 1: radio RC calibration, 2: RC trim calibration| 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration| 1: APM: compass/motor interference calibration (PX4: airspeed calibration, deprecated), 2: airspeed calibration| 1: ESC calibration, 3: barometer temperature calibration|  */
   MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS=242, /* Set sensor offsets. This command will be only accepted if in pre-flight mode. |Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow, 5: second magnetometer, 6: third magnetometer| X axis offset (or generic dimension 1), in the sensor's raw units| Y axis offset (or generic dimension 2), in the sensor's raw units| Z axis offset (or generic dimension 3), in the sensor's raw units| Generic dimension 4, in the sensor's raw units| Generic dimension 5, in the sensor's raw units| Generic dimension 6, in the sensor's raw units|  */
   MAV_CMD_PREFLIGHT_UAVCAN=243, /* Trigger UAVCAN config. This command will be only accepted if in pre-flight mode. |1: Trigger actuator ID assignment and direction mapping.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
   MAV_CMD_PREFLIGHT_STORAGE=245, /* Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. |Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults| Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults| Onboard logging: 0: Ignore, 1: Start default rate logging, -1: Stop logging, > 1: logging rate (e.g. set to 1000 for 1000 Hz logging)| Reserved| Empty| Empty| Empty|  */
   MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN=246, /* Request the reboot or shutdown of system components. |0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep it in the bootloader until upgraded.| 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer, 3: Reboot onboard computer and keep it in the bootloader until upgraded.| WIP: 0: Do nothing for camera, 1: Reboot onboard camera, 2: Shutdown onboard camera, 3: Reboot onboard camera and keep it in the bootloader until upgraded| WIP: 0: Do nothing for mount (e.g. gimbal), 1: Reboot mount, 2: Shutdown mount, 3: Reboot mount and keep it in the bootloader until upgraded| Reserved, send 0| Reserved, send 0| WIP: ID (e.g. camera ID -1 for all IDs)|  */
   MAV_CMD_OVERRIDE_GOTO=252, /* Override current mission with command to pause mission, pause mission and move to position, continue/resume mission. When param 1 indicates that the mission is paused (MAV_GOTO_DO_HOLD), param 2 defines whether it holds in place or moves to another position. |MAV_GOTO_DO_HOLD: pause mission and either hold or move to specified position (depending on param2), MAV_GOTO_DO_CONTINUE: resume mission.| MAV_GOTO_HOLD_AT_CURRENT_POSITION: hold at current position, MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position.| Coordinate frame of hold point.| Desired yaw angle.| Latitude / X position.| Longitude / Y position.| Altitude / Z position.|  */
   MAV_CMD_MISSION_START=300, /* start running a mission |first_item: the first mission item to run| last_item:  the last mission item to run (after this item is run, the mission ends)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_COMPONENT_ARM_DISARM=400, /* Arms / Disarms a component |0: disarm, 1: arm| 0: arm-disarm unless prevented by safety checks (i.e. when landed), 21196: force arming/disarming (e.g. allow arming to override preflight checks and disarming in flight)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_ILLUMINATOR_ON_OFF=405, /* Turns illuminators ON/OFF. An illuminator is a light source that is used for lighting up dark areas external to the sytstem: e.g. a torch or searchlight (as opposed to a light source for illuminating the system itself, e.g. an indicator light). |0: Illuminators OFF, 1: Illuminators ON| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_GET_HOME_POSITION=410, /* Request the home position from the vehicle. |Reserved| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
   MAV_CMD_START_RX_PAIR=500, /* Starts receiver pairing. |0:Spektrum.| RC type.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_GET_MESSAGE_INTERVAL=510, /* Request the interval between messages for a particular MAVLink message ID. The receiver should ACK the command and then emit its response in a MESSAGE_INTERVAL message. |The MAVLink message ID| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_SET_MESSAGE_INTERVAL=511, /* Set the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM. |The MAVLink message ID| The interval between two messages. Set to -1 to disable and 0 to request default rate.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.|  */
   MAV_CMD_REQUEST_MESSAGE=512, /* Request the target system(s) emit a single instance of a specified message (i.e. a "one-shot" version of MAV_CMD_SET_MESSAGE_INTERVAL). |The MAVLink message ID of the requested message.| Index id (if appropriate). The use of this parameter (if any), must be defined in the requested message.| The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).| The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).| The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).| The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).| Target address for requested message (if message has target address fields). 0: Flight-stack default, 1: address of requestor, 2: broadcast.|  */
   MAV_CMD_REQUEST_PROTOCOL_VERSION=519, /* Request MAVLink protocol version compatibility |1: Request supported protocol versions by all nodes on the network| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES=520, /* Request autopilot capabilities. The receiver should ACK the command and then emit its capabilities in an AUTOPILOT_VERSION message |1: Request autopilot version| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_REQUEST_CAMERA_INFORMATION=521, /* Request camera information (CAMERA_INFORMATION). |0: No action 1: Request camera capabilities| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_REQUEST_CAMERA_SETTINGS=522, /* Request camera settings (CAMERA_SETTINGS). |0: No Action 1: Request camera settings| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_REQUEST_STORAGE_INFORMATION=525, /* Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a specific component's storage. |Storage ID (0 for all, 1 for first, 2 for second, etc.)| 0: No Action 1: Request storage information| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_STORAGE_FORMAT=526, /* Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the command's target_component to target a specific component's storage. |Storage ID (1 for first, 2 for second, etc.)| 0: No action 1: Format storage| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS=527, /* Request camera capture status (CAMERA_CAPTURE_STATUS) |0: No Action 1: Request camera capture status| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_REQUEST_FLIGHT_INFORMATION=528, /* Request flight information (FLIGHT_INFORMATION) |1: Request flight information| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_RESET_CAMERA_SETTINGS=529, /* Reset all camera settings to Factory Default |0: No Action 1: Reset all settings| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_SET_CAMERA_MODE=530, /* Set camera running mode. Use NaN for reserved values. GCS will send a MAV_CMD_REQUEST_VIDEO_STREAM_STATUS command after a mode change if the camera supports video streaming. |Reserved (Set to 0)| Camera mode| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_SET_CAMERA_ZOOM=531, /* Set camera zoom. Camera must respond with a CAMERA_SETTINGS message (on success). Use NaN for reserved values. |Zoom type| Zoom value. The range of valid values depend on the zoom type.| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_SET_CAMERA_FOCUS=532, /* Set camera focus. Camera must respond with a CAMERA_SETTINGS message (on success). Use NaN for reserved values. |Focus type| Focus value| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_JUMP_TAG=600, /* Tagged jump target. Can be jumped to with MAV_CMD_DO_JUMP_TAG. |Tag.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_DO_JUMP_TAG=601, /* Jump to the matching tag in the mission list. Repeat this action for the specified number of times. A mission should contain a single matching tag for each jump. If this is not the case then a jump to a missing tag should complete the mission, and a jump where there are multiple matching tags should always select the one with the lowest mission sequence number. |Target tag to jump to.| Repeat count.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_IMAGE_START_CAPTURE=2000, /* Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NaN for reserved values. |Reserved (Set to 0)| Desired elapsed time between two consecutive pictures (in seconds). Minimum values depend on hardware (typically greater than 2 seconds).| Total number of images to capture. 0 to capture forever/until MAV_CMD_IMAGE_STOP_CAPTURE.| Capture sequence number starting from 1. This is only valid for single-capture (param3 == 1). Increment the capture ID for each capture command to prevent double captures when a command is re-transmitted. Use 0 to ignore it.| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_IMAGE_STOP_CAPTURE=2001, /* Stop image capture sequence Use NaN for reserved values. |Reserved (Set to 0)| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE=2002, /* Re-request a CAMERA_IMAGE_CAPTURE message. Use NaN for reserved values. |Sequence number for missing CAMERA_IMAGE_CAPTURE message| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_DO_TRIGGER_CONTROL=2003, /* Enable or disable on-board camera triggering system. |Trigger enable/disable (0 for disable, 1 for start), -1 to ignore| 1 to reset the trigger sequence, -1 or 0 to ignore| 1 to pause triggering, but without switching the camera off or retracting it. -1 to ignore| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_VIDEO_START_CAPTURE=2500, /* Starts video capture (recording). Use NaN for reserved values. |Video Stream ID (0 for all streams)| Frequency CAMERA_CAPTURE_STATUS messages should be sent while recording (0 for no messages, otherwise frequency)| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_VIDEO_STOP_CAPTURE=2501, /* Stop the current video capture (recording). Use NaN for reserved values. |Video Stream ID (0 for all streams)| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_VIDEO_START_STREAMING=2502, /* Start video streaming |Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)| Reserved| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_VIDEO_STOP_STREAMING=2503, /* Stop the given video stream |Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)| Reserved| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION=2504, /* Request video stream information (VIDEO_STREAM_INFORMATION) |Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_REQUEST_VIDEO_STREAM_STATUS=2505, /* Request video stream status (VIDEO_STREAM_STATUS) |Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_LOGGING_START=2510, /* Request to start streaming logging data over MAVLink (see also LOGGING_DATA message) |Format: 0: ULog| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)|  */
   MAV_CMD_LOGGING_STOP=2511, /* Request to stop streaming log data over MAVLink |Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)|  */
   MAV_CMD_AIRFRAME_CONFIGURATION=2520, /*  |Landing gear ID (default: 0, -1 for all)| Landing gear position (Down: 0, Up: 1, NaN for no change)| Reserved, set to NaN| Reserved, set to NaN| Reserved, set to NaN| Reserved, set to NaN| Reserved, set to NaN|  */
   MAV_CMD_CONTROL_HIGH_LATENCY=2600, /* Request to start/stop transmitting over the high latency telemetry |Control transmission over high latency telemetry (0: stop, 1: start)| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_PANORAMA_CREATE=2800, /* Create a panorama at the current position |Viewing angle horizontal of the panorama (+- 0.5 the total angle)| Viewing angle vertical of panorama.| Speed of the horizontal rotation.| Speed of the vertical rotation.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_DO_VTOL_TRANSITION=3000, /* Request VTOL transition |The target VTOL state. Only MAV_VTOL_STATE_MC and MAV_VTOL_STATE_FW can be used.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_ARM_AUTHORIZATION_REQUEST=3001, /* Request authorization to arm the vehicle to a external entity, the arm authorizer is responsible to request all data that is needs from the vehicle before authorize or deny the request. If approved the progress of command_ack message should be set with period of time that this authorization is valid in seconds or in case it was denied it should be set with one of the reasons in ARM_AUTH_DENIED_REASON.
         |Vehicle system id, this way ground station can request arm authorization on behalf of any vehicle| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_SET_GUIDED_SUBMODE_STANDARD=4000, /* This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocities along all three axes.
                   |Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE=4001, /* This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.
                   |Radius of desired circle in CIRCLE_MODE| User defined| User defined| User defined| Unscaled target latitude of center of circle in CIRCLE_MODE| Unscaled target longitude of center of circle in CIRCLE_MODE| Reserved (default:0)|  */
   MAV_CMD_CONDITION_GATE=4501, /* Delay mission state machine until gate has been reached. |Geometry: 0: orthogonal to path between previous and next waypoint.| Altitude: 0: ignore altitude| Empty| Empty| Latitude| Longitude| Altitude|  */
   MAV_CMD_NAV_FENCE_RETURN_POINT=5000, /* Fence return point. There can only be one fence return point.
         |Reserved| Reserved| Reserved| Reserved| Latitude| Longitude| Altitude|  */
   MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION=5001, /* Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must stay within this area. Minimum of 3 vertices required.
         |Polygon vertex count| Reserved| Reserved| Reserved| Latitude| Longitude| Reserved|  */
   MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION=5002, /* Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must stay outside this area. Minimum of 3 vertices required.
         |Polygon vertex count| Reserved| Reserved| Reserved| Latitude| Longitude| Reserved|  */
   MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION=5003, /* Circular fence area. The vehicle must stay inside this area.
         |Radius.| Reserved| Reserved| Reserved| Latitude| Longitude| Reserved|  */
   MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION=5004, /* Circular fence area. The vehicle must stay outside this area.
         |Radius.| Reserved| Reserved| Reserved| Latitude| Longitude| Reserved|  */
   MAV_CMD_NAV_RALLY_POINT=5100, /* Rally point. You can have multiple rally points defined.
         |Reserved| Reserved| Reserved| Reserved| Latitude| Longitude| Altitude|  */
   MAV_CMD_UAVCAN_GET_NODE_INFO=5200, /* Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN node that is online. Note that some of the response messages can be lost, which the receiver can detect easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO received earlier; if not, this command should be sent again in order to request re-transmission of the node information messages. |Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)|  */
   MAV_CMD_PAYLOAD_PREPARE_DEPLOY=30001, /* Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity. |Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing abort). 2: add payload deploy to existing deployment list.| Desired approach vector in compass heading. A negative value indicates the system can define the approach vector at will.| Desired ground speed at release time. This can be overridden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will.| Minimum altitude clearance to the release position. A negative value indicates the system can define the clearance at will.| Latitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT| Longitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT| Altitude (MSL), in meters|  */
   MAV_CMD_PAYLOAD_CONTROL_DEPLOY=30002, /* Control the payload deployment. |Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deployment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
   MAV_CMD_WAYPOINT_USER_1=31000, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL), in meters|  */
   MAV_CMD_WAYPOINT_USER_2=31001, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL), in meters|  */
   MAV_CMD_WAYPOINT_USER_3=31002, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL), in meters|  */
   MAV_CMD_WAYPOINT_USER_4=31003, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL), in meters|  */
   MAV_CMD_WAYPOINT_USER_5=31004, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL), in meters|  */
   MAV_CMD_SPATIAL_USER_1=31005, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL), in meters|  */
   MAV_CMD_SPATIAL_USER_2=31006, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL), in meters|  */
   MAV_CMD_SPATIAL_USER_3=31007, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL), in meters|  */
   MAV_CMD_SPATIAL_USER_4=31008, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL), in meters|  */
   MAV_CMD_SPATIAL_USER_5=31009, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL), in meters|  */
   MAV_CMD_USER_1=31010, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
   MAV_CMD_USER_2=31011, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
   MAV_CMD_USER_3=31012, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
   MAV_CMD_USER_4=31013, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
   MAV_CMD_USER_5=31014, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
   MAV_CMD_ENUM_END=31015, /*  | */
} MAV_CMD;
#endif

/** @brief A data stream is not a fixed set of messages, but rather a
     recommendation to the autopilot software. Individual autopilots may or may not obey
     the recommended messages. */
#ifndef HAVE_ENUM_MAV_DATA_STREAM
#define HAVE_ENUM_MAV_DATA_STREAM
typedef enum MAV_DATA_STREAM
{
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

/** @brief The ROI (region of interest) for the vehicle. This can be
                be used by the vehicle for camera/vehicle attitude alignment (see
                MAV_CMD_NAV_ROI). */
#ifndef HAVE_ENUM_MAV_ROI
#define HAVE_ENUM_MAV_ROI
typedef enum MAV_ROI
{
   MAV_ROI_NONE=0, /* No region of interest. | */
   MAV_ROI_WPNEXT=1, /* Point toward next waypoint, with optional pitch/roll/yaw offset. | */
   MAV_ROI_WPINDEX=2, /* Point toward given waypoint. | */
   MAV_ROI_LOCATION=3, /* Point toward fixed location. | */
   MAV_ROI_TARGET=4, /* Point toward of given id. | */
   MAV_ROI_ENUM_END=5, /*  | */
} MAV_ROI;
#endif

/** @brief ACK / NACK / ERROR values as a result of MAV_CMDs and for mission item transmission. */
#ifndef HAVE_ENUM_MAV_CMD_ACK
#define HAVE_ENUM_MAV_CMD_ACK
typedef enum MAV_CMD_ACK
{
   MAV_CMD_ACK_OK=1, /* Command / mission item is ok. |Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_ACK_ERR_FAIL=2, /* Generic error message if none of the other reasons fails or if no detailed error reporting is implemented. |Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_ACK_ERR_ACCESS_DENIED=3, /* The system is refusing to accept this command from this source / communication partner. |Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_ACK_ERR_NOT_SUPPORTED=4, /* Command or mission item is not supported, other commands would be accepted. |Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_ACK_ERR_COORDINATE_FRAME_NOT_SUPPORTED=5, /* The coordinate frame of this command / mission item is not supported. |Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_ACK_ERR_COORDINATES_OUT_OF_RANGE=6, /* The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of this system. This is a generic error, please use the more specific error messages below if possible. |Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_ACK_ERR_X_LAT_OUT_OF_RANGE=7, /* The X or latitude value is out of range. |Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_ACK_ERR_Y_LON_OUT_OF_RANGE=8, /* The Y or longitude value is out of range. |Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_ACK_ERR_Z_ALT_OUT_OF_RANGE=9, /* The Z or altitude value is out of range. |Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_ACK_ENUM_END=10, /*  | */
} MAV_CMD_ACK;
#endif

/** @brief Specifies the datatype of a MAVLink parameter. */
#ifndef HAVE_ENUM_MAV_PARAM_TYPE
#define HAVE_ENUM_MAV_PARAM_TYPE
typedef enum MAV_PARAM_TYPE
{
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

/** @brief Specifies the datatype of a MAVLink extended parameter. */
#ifndef HAVE_ENUM_MAV_PARAM_EXT_TYPE
#define HAVE_ENUM_MAV_PARAM_EXT_TYPE
typedef enum MAV_PARAM_EXT_TYPE
{
   MAV_PARAM_EXT_TYPE_UINT8=1, /* 8-bit unsigned integer | */
   MAV_PARAM_EXT_TYPE_INT8=2, /* 8-bit signed integer | */
   MAV_PARAM_EXT_TYPE_UINT16=3, /* 16-bit unsigned integer | */
   MAV_PARAM_EXT_TYPE_INT16=4, /* 16-bit signed integer | */
   MAV_PARAM_EXT_TYPE_UINT32=5, /* 32-bit unsigned integer | */
   MAV_PARAM_EXT_TYPE_INT32=6, /* 32-bit signed integer | */
   MAV_PARAM_EXT_TYPE_UINT64=7, /* 64-bit unsigned integer | */
   MAV_PARAM_EXT_TYPE_INT64=8, /* 64-bit signed integer | */
   MAV_PARAM_EXT_TYPE_REAL32=9, /* 32-bit floating-point | */
   MAV_PARAM_EXT_TYPE_REAL64=10, /* 64-bit floating-point | */
   MAV_PARAM_EXT_TYPE_CUSTOM=11, /* Custom Type | */
   MAV_PARAM_EXT_TYPE_ENUM_END=12, /*  | */
} MAV_PARAM_EXT_TYPE;
#endif

/** @brief Result from a MAVLink command (MAV_CMD) */
#ifndef HAVE_ENUM_MAV_RESULT
#define HAVE_ENUM_MAV_RESULT
typedef enum MAV_RESULT
{
   MAV_RESULT_ACCEPTED=0, /* Command is valid (is supported and has valid parameters), and was executed. | */
   MAV_RESULT_TEMPORARILY_REJECTED=1, /* Command is valid, but cannot be executed at this time. This is used to indicate a problem that should be fixed just by waiting (e.g. a state machine is busy, can't arm because have not got GPS lock, etc.). Retrying later should work. | */
   MAV_RESULT_DENIED=2, /* Command is invalid (is supported but has invalid parameters). Retrying same command and parameters will not work. | */
   MAV_RESULT_UNSUPPORTED=3, /* Command is not supported (unknown). | */
   MAV_RESULT_FAILED=4, /* Command is valid, but execution has failed. This is used to indicate any non-temporary or unexpected problem, i.e. any problem that must be fixed before the command can succeed/be retried. For example, attempting to write a file when out of memory, attempting to arm when sensors are not calibrated, etc. | */
   MAV_RESULT_IN_PROGRESS=5, /* Command is valid and is being executed. This will be followed by further progress updates, i.e. the component may send further COMMAND_ACK messages with result MAV_RESULT_IN_PROGRESS (at a rate decided by the implementation), and must terminate by sending a COMMAND_ACK message with final result of the operation. The COMMAND_ACK.progress field can be used to indicate the progress of the operation. There is no need for the sender to retry the command, but if done during execution, the component will return MAV_RESULT_IN_PROGRESS with an updated progress. | */
   MAV_RESULT_ENUM_END=6, /*  | */
} MAV_RESULT;
#endif

/** @brief Result of mission operation (in a MISSION_ACK message). */
#ifndef HAVE_ENUM_MAV_MISSION_RESULT
#define HAVE_ENUM_MAV_MISSION_RESULT
typedef enum MAV_MISSION_RESULT
{
   MAV_MISSION_ACCEPTED=0, /* mission accepted OK | */
   MAV_MISSION_ERROR=1, /* Generic error / not accepting mission commands at all right now. | */
   MAV_MISSION_UNSUPPORTED_FRAME=2, /* Coordinate frame is not supported. | */
   MAV_MISSION_UNSUPPORTED=3, /* Command is not supported. | */
   MAV_MISSION_NO_SPACE=4, /* Mission item exceeds storage space. | */
   MAV_MISSION_INVALID=5, /* One of the parameters has an invalid value. | */
   MAV_MISSION_INVALID_PARAM1=6, /* param1 has an invalid value. | */
   MAV_MISSION_INVALID_PARAM2=7, /* param2 has an invalid value. | */
   MAV_MISSION_INVALID_PARAM3=8, /* param3 has an invalid value. | */
   MAV_MISSION_INVALID_PARAM4=9, /* param4 has an invalid value. | */
   MAV_MISSION_INVALID_PARAM5_X=10, /* x / param5 has an invalid value. | */
   MAV_MISSION_INVALID_PARAM6_Y=11, /* y / param6 has an invalid value. | */
   MAV_MISSION_INVALID_PARAM7=12, /* z / param7 has an invalid value. | */
   MAV_MISSION_INVALID_SEQUENCE=13, /* Mission item received out of sequence | */
   MAV_MISSION_DENIED=14, /* Not accepting any mission commands from this communication partner. | */
   MAV_MISSION_OPERATION_CANCELLED=15, /* Current mission operation cancelled (e.g. mission upload, mission download). | */
   MAV_MISSION_RESULT_ENUM_END=16, /*  | */
} MAV_MISSION_RESULT;
#endif

/** @brief Indicates the severity level, generally used for status messages to indicate their relative urgency. Based on RFC-5424 using expanded definitions at: http://www.kiwisyslog.com/kb/info:-syslog-message-levels/. */
#ifndef HAVE_ENUM_MAV_SEVERITY
#define HAVE_ENUM_MAV_SEVERITY
typedef enum MAV_SEVERITY
{
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
typedef enum MAV_POWER_STATUS
{
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
typedef enum SERIAL_CONTROL_DEV
{
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
typedef enum SERIAL_CONTROL_FLAG
{
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
typedef enum MAV_DISTANCE_SENSOR
{
   MAV_DISTANCE_SENSOR_LASER=0, /* Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units | */
   MAV_DISTANCE_SENSOR_ULTRASOUND=1, /* Ultrasound rangefinder, e.g. MaxBotix units | */
   MAV_DISTANCE_SENSOR_INFRARED=2, /* Infrared rangefinder, e.g. Sharp units | */
   MAV_DISTANCE_SENSOR_RADAR=3, /* Radar type, e.g. uLanding units | */
   MAV_DISTANCE_SENSOR_UNKNOWN=4, /* Broken or unknown type, e.g. analog units | */
   MAV_DISTANCE_SENSOR_ENUM_END=5, /*  | */
} MAV_DISTANCE_SENSOR;
#endif

/** @brief Enumeration of sensor orientation, according to its rotations */
#ifndef HAVE_ENUM_MAV_SENSOR_ORIENTATION
#define HAVE_ENUM_MAV_SENSOR_ORIENTATION
typedef enum MAV_SENSOR_ORIENTATION
{
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
   MAV_SENSOR_ROTATION_ROLL_90_PITCH_68_YAW_293=38, /* Roll: 90, Pitch: 68, Yaw: 293 | */
   MAV_SENSOR_ROTATION_PITCH_315=39, /* Pitch: 315 | */
   MAV_SENSOR_ROTATION_ROLL_90_PITCH_315=40, /* Roll: 90, Pitch: 315 | */
   MAV_SENSOR_ROTATION_CUSTOM=100, /* Custom orientation | */
   MAV_SENSOR_ORIENTATION_ENUM_END=101, /*  | */
} MAV_SENSOR_ORIENTATION;
#endif

/** @brief Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this capability. */
#ifndef HAVE_ENUM_MAV_PROTOCOL_CAPABILITY
#define HAVE_ENUM_MAV_PROTOCOL_CAPABILITY
typedef enum MAV_PROTOCOL_CAPABILITY
{
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
   MAV_PROTOCOL_CAPABILITY_MAVLINK2=8192, /* Autopilot supports MAVLink version 2. | */
   MAV_PROTOCOL_CAPABILITY_MISSION_FENCE=16384, /* Autopilot supports mission fence protocol. | */
   MAV_PROTOCOL_CAPABILITY_MISSION_RALLY=32768, /* Autopilot supports mission rally point protocol. | */
   MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION=65536, /* Autopilot supports the flight information protocol. | */
   MAV_PROTOCOL_CAPABILITY_ENUM_END=65537, /*  | */
} MAV_PROTOCOL_CAPABILITY;
#endif

/** @brief Type of mission items being requested/sent in mission protocol. */
#ifndef HAVE_ENUM_MAV_MISSION_TYPE
#define HAVE_ENUM_MAV_MISSION_TYPE
typedef enum MAV_MISSION_TYPE
{
   MAV_MISSION_TYPE_MISSION=0, /* Items are mission commands for main mission. | */
   MAV_MISSION_TYPE_FENCE=1, /* Specifies GeoFence area(s). Items are MAV_CMD_NAV_FENCE_ GeoFence items. | */
   MAV_MISSION_TYPE_RALLY=2, /* Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are MAV_CMD_NAV_RALLY_POINT rally point items. | */
   MAV_MISSION_TYPE_ALL=255, /* Only used in MISSION_CLEAR_ALL to clear all mission types. | */
   MAV_MISSION_TYPE_ENUM_END=256, /*  | */
} MAV_MISSION_TYPE;
#endif

/** @brief Enumeration of estimator types */
#ifndef HAVE_ENUM_MAV_ESTIMATOR_TYPE
#define HAVE_ENUM_MAV_ESTIMATOR_TYPE
typedef enum MAV_ESTIMATOR_TYPE
{
   MAV_ESTIMATOR_TYPE_UNKNOWN=0, /* Unknown type of the estimator. | */
   MAV_ESTIMATOR_TYPE_NAIVE=1, /* This is a naive estimator without any real covariance feedback. | */
   MAV_ESTIMATOR_TYPE_VISION=2, /* Computer vision based estimate. Might be up to scale. | */
   MAV_ESTIMATOR_TYPE_VIO=3, /* Visual-inertial estimate. | */
   MAV_ESTIMATOR_TYPE_GPS=4, /* Plain GPS estimate. | */
   MAV_ESTIMATOR_TYPE_GPS_INS=5, /* Estimator integrating GPS and inertial sensing. | */
   MAV_ESTIMATOR_TYPE_MOCAP=6, /* Estimate from external motion capturing system. | */
   MAV_ESTIMATOR_TYPE_LIDAR=7, /* Estimator based on lidar sensor input. | */
   MAV_ESTIMATOR_TYPE_AUTOPILOT=8, /* Estimator on autopilot. | */
   MAV_ESTIMATOR_TYPE_ENUM_END=9, /*  | */
} MAV_ESTIMATOR_TYPE;
#endif

/** @brief Enumeration of battery types */
#ifndef HAVE_ENUM_MAV_BATTERY_TYPE
#define HAVE_ENUM_MAV_BATTERY_TYPE
typedef enum MAV_BATTERY_TYPE
{
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
typedef enum MAV_BATTERY_FUNCTION
{
   MAV_BATTERY_FUNCTION_UNKNOWN=0, /* Battery function is unknown | */
   MAV_BATTERY_FUNCTION_ALL=1, /* Battery supports all flight systems | */
   MAV_BATTERY_FUNCTION_PROPULSION=2, /* Battery for the propulsion system | */
   MAV_BATTERY_FUNCTION_AVIONICS=3, /* Avionics battery | */
   MAV_BATTERY_TYPE_PAYLOAD=4, /* Payload battery | */
   MAV_BATTERY_FUNCTION_ENUM_END=5, /*  | */
} MAV_BATTERY_FUNCTION;
#endif

/** @brief Enumeration for battery charge states. */
#ifndef HAVE_ENUM_MAV_BATTERY_CHARGE_STATE
#define HAVE_ENUM_MAV_BATTERY_CHARGE_STATE
typedef enum MAV_BATTERY_CHARGE_STATE
{
   MAV_BATTERY_CHARGE_STATE_UNDEFINED=0, /* Low battery state is not provided | */
   MAV_BATTERY_CHARGE_STATE_OK=1, /* Battery is not in low state. Normal operation. | */
   MAV_BATTERY_CHARGE_STATE_LOW=2, /* Battery state is low, warn and monitor close. | */
   MAV_BATTERY_CHARGE_STATE_CRITICAL=3, /* Battery state is critical, return or abort immediately. | */
   MAV_BATTERY_CHARGE_STATE_EMERGENCY=4, /* Battery state is too low for ordinary abort sequence. Perform fastest possible emergency stop to prevent damage. | */
   MAV_BATTERY_CHARGE_STATE_FAILED=5, /* Battery failed, damage unavoidable. | */
   MAV_BATTERY_CHARGE_STATE_UNHEALTHY=6, /* Battery is diagnosed to be defective or an error occurred, usage is discouraged / prohibited. | */
   MAV_BATTERY_CHARGE_STATE_CHARGING=7, /* Battery is charging. | */
   MAV_BATTERY_CHARGE_STATE_ENUM_END=8, /*  | */
} MAV_BATTERY_CHARGE_STATE;
#endif

/** @brief Smart battery supply status/fault flags (bitmask) for health indication. */
#ifndef HAVE_ENUM_MAV_SMART_BATTERY_FAULT
#define HAVE_ENUM_MAV_SMART_BATTERY_FAULT
typedef enum MAV_SMART_BATTERY_FAULT
{
   MAV_SMART_BATTERY_FAULT_DEEP_DISCHARGE=1, /* Battery has deep discharged. | */
   MAV_SMART_BATTERY_FAULT_SPIKES=2, /* Voltage spikes. | */
   MAV_SMART_BATTERY_FAULT_SINGLE_CELL_FAIL=4, /* Single cell has failed. | */
   MAV_SMART_BATTERY_FAULT_OVER_CURRENT=8, /* Over-current fault. | */
   MAV_SMART_BATTERY_FAULT_OVER_TEMPERATURE=16, /* Over-temperature fault. | */
   MAV_SMART_BATTERY_FAULT_UNDER_TEMPERATURE=32, /* Under-temperature fault. | */
   MAV_SMART_BATTERY_FAULT_ENUM_END=33, /*  | */
} MAV_SMART_BATTERY_FAULT;
#endif

/** @brief Enumeration of VTOL states */
#ifndef HAVE_ENUM_MAV_VTOL_STATE
#define HAVE_ENUM_MAV_VTOL_STATE
typedef enum MAV_VTOL_STATE
{
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
typedef enum MAV_LANDED_STATE
{
   MAV_LANDED_STATE_UNDEFINED=0, /* MAV landed state is unknown | */
   MAV_LANDED_STATE_ON_GROUND=1, /* MAV is landed (on ground) | */
   MAV_LANDED_STATE_IN_AIR=2, /* MAV is in air | */
   MAV_LANDED_STATE_TAKEOFF=3, /* MAV currently taking off | */
   MAV_LANDED_STATE_LANDING=4, /* MAV currently landing | */
   MAV_LANDED_STATE_ENUM_END=5, /*  | */
} MAV_LANDED_STATE;
#endif

/** @brief Enumeration of the ADSB altimeter types */
#ifndef HAVE_ENUM_ADSB_ALTITUDE_TYPE
#define HAVE_ENUM_ADSB_ALTITUDE_TYPE
typedef enum ADSB_ALTITUDE_TYPE
{
   ADSB_ALTITUDE_TYPE_PRESSURE_QNH=0, /* Altitude reported from a Baro source using QNH reference | */
   ADSB_ALTITUDE_TYPE_GEOMETRIC=1, /* Altitude reported from a GNSS source | */
   ADSB_ALTITUDE_TYPE_ENUM_END=2, /*  | */
} ADSB_ALTITUDE_TYPE;
#endif

/** @brief ADSB classification for the type of vehicle emitting the transponder signal */
#ifndef HAVE_ENUM_ADSB_EMITTER_TYPE
#define HAVE_ENUM_ADSB_EMITTER_TYPE
typedef enum ADSB_EMITTER_TYPE
{
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
typedef enum ADSB_FLAGS
{
   ADSB_FLAGS_VALID_COORDS=1, /*  | */
   ADSB_FLAGS_VALID_ALTITUDE=2, /*  | */
   ADSB_FLAGS_VALID_HEADING=4, /*  | */
   ADSB_FLAGS_VALID_VELOCITY=8, /*  | */
   ADSB_FLAGS_VALID_CALLSIGN=16, /*  | */
   ADSB_FLAGS_VALID_SQUAWK=32, /*  | */
   ADSB_FLAGS_SIMULATED=64, /*  | */
   ADSB_FLAGS_VERTICAL_VELOCITY_VALID=128, /*  | */
   ADSB_FLAGS_BARO_VALID=256, /*  | */
   ADSB_FLAGS_SOURCE_UAT=32768, /*  | */
   ADSB_FLAGS_ENUM_END=32769, /*  | */
} ADSB_FLAGS;
#endif

/** @brief Bitmap of options for the MAV_CMD_DO_REPOSITION */
#ifndef HAVE_ENUM_MAV_DO_REPOSITION_FLAGS
#define HAVE_ENUM_MAV_DO_REPOSITION_FLAGS
typedef enum MAV_DO_REPOSITION_FLAGS
{
   MAV_DO_REPOSITION_FLAGS_CHANGE_MODE=1, /* The aircraft should immediately transition into guided. This should not be set for follow me applications | */
   MAV_DO_REPOSITION_FLAGS_ENUM_END=2, /*  | */
} MAV_DO_REPOSITION_FLAGS;
#endif

/** @brief Flags in EKF_STATUS message */
#ifndef HAVE_ENUM_ESTIMATOR_STATUS_FLAGS
#define HAVE_ENUM_ESTIMATOR_STATUS_FLAGS
typedef enum ESTIMATOR_STATUS_FLAGS
{
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
   ESTIMATOR_ACCEL_ERROR=2048, /* True if the EKF has detected bad accelerometer data | */
   ESTIMATOR_STATUS_FLAGS_ENUM_END=2049, /*  | */
} ESTIMATOR_STATUS_FLAGS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MOTOR_TEST_ORDER
#define HAVE_ENUM_MOTOR_TEST_ORDER
typedef enum MOTOR_TEST_ORDER
{
   MOTOR_TEST_ORDER_DEFAULT=0, /* default autopilot motor test method | */
   MOTOR_TEST_ORDER_SEQUENCE=1, /* motor numbers are specified as their index in a predefined vehicle-specific sequence | */
   MOTOR_TEST_ORDER_BOARD=2, /* motor numbers are specified as the output as labeled on the board | */
   MOTOR_TEST_ORDER_ENUM_END=3, /*  | */
} MOTOR_TEST_ORDER;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MOTOR_TEST_THROTTLE_TYPE
#define HAVE_ENUM_MOTOR_TEST_THROTTLE_TYPE
typedef enum MOTOR_TEST_THROTTLE_TYPE
{
   MOTOR_TEST_THROTTLE_PERCENT=0, /* throttle as a percentage from 0 ~ 100 | */
   MOTOR_TEST_THROTTLE_PWM=1, /* throttle as an absolute PWM value (normally in range of 1000~2000) | */
   MOTOR_TEST_THROTTLE_PILOT=2, /* throttle pass-through from pilot's transmitter | */
   MOTOR_TEST_COMPASS_CAL=3, /* per-motor compass calibration test | */
   MOTOR_TEST_THROTTLE_TYPE_ENUM_END=4, /*  | */
} MOTOR_TEST_THROTTLE_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GPS_INPUT_IGNORE_FLAGS
#define HAVE_ENUM_GPS_INPUT_IGNORE_FLAGS
typedef enum GPS_INPUT_IGNORE_FLAGS
{
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
typedef enum MAV_COLLISION_ACTION
{
   MAV_COLLISION_ACTION_NONE=0, /* Ignore any potential collisions | */
   MAV_COLLISION_ACTION_REPORT=1, /* Report potential collision | */
   MAV_COLLISION_ACTION_ASCEND_OR_DESCEND=2, /* Ascend or Descend to avoid threat | */
   MAV_COLLISION_ACTION_MOVE_HORIZONTALLY=3, /* Move horizontally to avoid threat | */
   MAV_COLLISION_ACTION_MOVE_PERPENDICULAR=4, /* Aircraft to move perpendicular to the collision's velocity vector | */
   MAV_COLLISION_ACTION_RTL=5, /* Aircraft to fly directly back to its launch point | */
   MAV_COLLISION_ACTION_HOVER=6, /* Aircraft to stop in place | */
   MAV_COLLISION_ACTION_ENUM_END=7, /*  | */
} MAV_COLLISION_ACTION;
#endif

/** @brief Aircraft-rated danger from this threat. */
#ifndef HAVE_ENUM_MAV_COLLISION_THREAT_LEVEL
#define HAVE_ENUM_MAV_COLLISION_THREAT_LEVEL
typedef enum MAV_COLLISION_THREAT_LEVEL
{
   MAV_COLLISION_THREAT_LEVEL_NONE=0, /* Not a threat | */
   MAV_COLLISION_THREAT_LEVEL_LOW=1, /* Craft is mildly concerned about this threat | */
   MAV_COLLISION_THREAT_LEVEL_HIGH=2, /* Craft is panicking, and may take actions to avoid threat | */
   MAV_COLLISION_THREAT_LEVEL_ENUM_END=3, /*  | */
} MAV_COLLISION_THREAT_LEVEL;
#endif

/** @brief Source of information about this collision. */
#ifndef HAVE_ENUM_MAV_COLLISION_SRC
#define HAVE_ENUM_MAV_COLLISION_SRC
typedef enum MAV_COLLISION_SRC
{
   MAV_COLLISION_SRC_ADSB=0, /* ID field references ADSB_VEHICLE packets | */
   MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT=1, /* ID field references MAVLink SRC ID | */
   MAV_COLLISION_SRC_ENUM_END=2, /*  | */
} MAV_COLLISION_SRC;
#endif

/** @brief Type of GPS fix */
#ifndef HAVE_ENUM_GPS_FIX_TYPE
#define HAVE_ENUM_GPS_FIX_TYPE
typedef enum GPS_FIX_TYPE
{
   GPS_FIX_TYPE_NO_GPS=0, /* No GPS connected | */
   GPS_FIX_TYPE_NO_FIX=1, /* No position information, GPS is connected | */
   GPS_FIX_TYPE_2D_FIX=2, /* 2D position | */
   GPS_FIX_TYPE_3D_FIX=3, /* 3D position | */
   GPS_FIX_TYPE_DGPS=4, /* DGPS/SBAS aided 3D position | */
   GPS_FIX_TYPE_RTK_FLOAT=5, /* RTK float, 3D position | */
   GPS_FIX_TYPE_RTK_FIXED=6, /* RTK Fixed, 3D position | */
   GPS_FIX_TYPE_STATIC=7, /* Static fixed, typically used for base stations | */
   GPS_FIX_TYPE_PPP=8, /* PPP, 3D position. | */
   GPS_FIX_TYPE_ENUM_END=9, /*  | */
} GPS_FIX_TYPE;
#endif

/** @brief RTK GPS baseline coordinate system, used for RTK corrections */
#ifndef HAVE_ENUM_RTK_BASELINE_COORDINATE_SYSTEM
#define HAVE_ENUM_RTK_BASELINE_COORDINATE_SYSTEM
typedef enum RTK_BASELINE_COORDINATE_SYSTEM
{
   RTK_BASELINE_COORDINATE_SYSTEM_ECEF=0, /* Earth-centered, Earth-fixed | */
   RTK_BASELINE_COORDINATE_SYSTEM_NED=1, /* RTK basestation centered, north, east, down | */
   RTK_BASELINE_COORDINATE_SYSTEM_ENUM_END=2, /*  | */
} RTK_BASELINE_COORDINATE_SYSTEM;
#endif

/** @brief Type of landing target */
#ifndef HAVE_ENUM_LANDING_TARGET_TYPE
#define HAVE_ENUM_LANDING_TARGET_TYPE
typedef enum LANDING_TARGET_TYPE
{
   LANDING_TARGET_TYPE_LIGHT_BEACON=0, /* Landing target signaled by light beacon (ex: IR-LOCK) | */
   LANDING_TARGET_TYPE_RADIO_BEACON=1, /* Landing target signaled by radio beacon (ex: ILS, NDB) | */
   LANDING_TARGET_TYPE_VISION_FIDUCIAL=2, /* Landing target represented by a fiducial marker (ex: ARTag) | */
   LANDING_TARGET_TYPE_VISION_OTHER=3, /* Landing target represented by a pre-defined visual shape/feature (ex: X-marker, H-marker, square) | */
   LANDING_TARGET_TYPE_ENUM_END=4, /*  | */
} LANDING_TARGET_TYPE;
#endif

/** @brief Direction of VTOL transition */
#ifndef HAVE_ENUM_VTOL_TRANSITION_HEADING
#define HAVE_ENUM_VTOL_TRANSITION_HEADING
typedef enum VTOL_TRANSITION_HEADING
{
   VTOL_TRANSITION_HEADING_VEHICLE_DEFAULT=0, /* Respect the heading configuration of the vehicle. | */
   VTOL_TRANSITION_HEADING_NEXT_WAYPOINT=1, /* Use the heading pointing towards the next waypoint. | */
   VTOL_TRANSITION_HEADING_TAKEOFF=2, /* Use the heading on takeoff (while sitting on the ground). | */
   VTOL_TRANSITION_HEADING_SPECIFIED=3, /* Use the specified heading in parameter 4. | */
   VTOL_TRANSITION_HEADING_ANY=4, /* Use the current heading when reaching takeoff altitude (potentially facing the wind when weather-vaning is active). | */
   VTOL_TRANSITION_HEADING_ENUM_END=5, /*  | */
} VTOL_TRANSITION_HEADING;
#endif

/** @brief Camera capability flags (Bitmap) */
#ifndef HAVE_ENUM_CAMERA_CAP_FLAGS
#define HAVE_ENUM_CAMERA_CAP_FLAGS
typedef enum CAMERA_CAP_FLAGS
{
   CAMERA_CAP_FLAGS_CAPTURE_VIDEO=1, /* Camera is able to record video | */
   CAMERA_CAP_FLAGS_CAPTURE_IMAGE=2, /* Camera is able to capture images | */
   CAMERA_CAP_FLAGS_HAS_MODES=4, /* Camera has separate Video and Image/Photo modes (MAV_CMD_SET_CAMERA_MODE) | */
   CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE=8, /* Camera can capture images while in video mode | */
   CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE=16, /* Camera can capture videos while in Photo/Image mode | */
   CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE=32, /* Camera has image survey mode (MAV_CMD_SET_CAMERA_MODE) | */
   CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM=64, /* Camera has basic zoom control (MAV_CMD_SET_CAMERA_ZOOM) | */
   CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS=128, /* Camera has basic focus control (MAV_CMD_SET_CAMERA_FOCUS) | */
   CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM=256, /* Camera has video streaming capabilities (use MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION for video streaming info) | */
   CAMERA_CAP_FLAGS_ENUM_END=257, /*  | */
} CAMERA_CAP_FLAGS;
#endif

/** @brief Stream status flags (Bitmap) */
#ifndef HAVE_ENUM_VIDEO_STREAM_STATUS_FLAGS
#define HAVE_ENUM_VIDEO_STREAM_STATUS_FLAGS
typedef enum VIDEO_STREAM_STATUS_FLAGS
{
   VIDEO_STREAM_STATUS_FLAGS_RUNNING=1, /* Stream is active (running) | */
   VIDEO_STREAM_STATUS_FLAGS_THERMAL=2, /* Stream is thermal imaging | */
   VIDEO_STREAM_STATUS_FLAGS_ENUM_END=3, /*  | */
} VIDEO_STREAM_STATUS_FLAGS;
#endif

/** @brief Video stream types */
#ifndef HAVE_ENUM_VIDEO_STREAM_TYPE
#define HAVE_ENUM_VIDEO_STREAM_TYPE
typedef enum VIDEO_STREAM_TYPE
{
   VIDEO_STREAM_TYPE_RTSP=0, /* Stream is RTSP | */
   VIDEO_STREAM_TYPE_RTPUDP=1, /* Stream is RTP UDP (URI gives the port number) | */
   VIDEO_STREAM_TYPE_TCP_MPEG=2, /* Stream is MPEG on TCP | */
   VIDEO_STREAM_TYPE_MPEG_TS_H264=3, /* Stream is h.264 on MPEG TS (URI gives the port number) | */
   VIDEO_STREAM_TYPE_ENUM_END=4, /*  | */
} VIDEO_STREAM_TYPE;
#endif

/** @brief Zoom types for MAV_CMD_SET_CAMERA_ZOOM */
#ifndef HAVE_ENUM_CAMERA_ZOOM_TYPE
#define HAVE_ENUM_CAMERA_ZOOM_TYPE
typedef enum CAMERA_ZOOM_TYPE
{
   ZOOM_TYPE_STEP=0, /* Zoom one step increment (-1 for wide, 1 for tele) | */
   ZOOM_TYPE_CONTINUOUS=1, /* Continuous zoom up/down until stopped (-1 for wide, 1 for tele, 0 to stop zooming) | */
   ZOOM_TYPE_RANGE=2, /* Zoom value as proportion of full camera range (a value between 0.0 and 100.0) | */
   ZOOM_TYPE_FOCAL_LENGTH=3, /* Zoom value/variable focal length in milimetres. Note that there is no message to get the valid zoom range of the camera, so this can type can only be used for cameras where the zoom range is known (implying that this cannot reliably be used in a GCS for an arbitrary camera) | */
   CAMERA_ZOOM_TYPE_ENUM_END=4, /*  | */
} CAMERA_ZOOM_TYPE;
#endif

/** @brief Focus types for MAV_CMD_SET_CAMERA_FOCUS */
#ifndef HAVE_ENUM_SET_FOCUS_TYPE
#define HAVE_ENUM_SET_FOCUS_TYPE
typedef enum SET_FOCUS_TYPE
{
   FOCUS_TYPE_STEP=0, /* Focus one step increment (-1 for focusing in, 1 for focusing out towards infinity). | */
   FOCUS_TYPE_CONTINUOUS=1, /* Continuous focus up/down until stopped (-1 for focusing in, 1 for focusing out towards infinity, 0 to stop focusing) | */
   FOCUS_TYPE_RANGE=2, /* Focus value as proportion of full camera focus range (a value between 0.0 and 100.0) | */
   FOCUS_TYPE_METERS=3, /* Focus value in metres. Note that there is no message to get the valid focus range of the camera, so this can type can only be used for cameras where the range is known (implying that this cannot reliably be used in a GCS for an arbitrary camera). | */
   SET_FOCUS_TYPE_ENUM_END=4, /*  | */
} SET_FOCUS_TYPE;
#endif

/** @brief Result from a PARAM_EXT_SET message. */
#ifndef HAVE_ENUM_PARAM_ACK
#define HAVE_ENUM_PARAM_ACK
typedef enum PARAM_ACK
{
   PARAM_ACK_ACCEPTED=0, /* Parameter value ACCEPTED and SET | */
   PARAM_ACK_VALUE_UNSUPPORTED=1, /* Parameter value UNKNOWN/UNSUPPORTED | */
   PARAM_ACK_FAILED=2, /* Parameter failed to set | */
   PARAM_ACK_IN_PROGRESS=3, /* Parameter value received but not yet validated or set. A subsequent PARAM_EXT_ACK will follow once operation is completed with the actual result. These are for parameters that may take longer to set. Instead of waiting for an ACK and potentially timing out, you will immediately receive this response to let you know it was received. | */
   PARAM_ACK_ENUM_END=4, /*  | */
} PARAM_ACK;
#endif

/** @brief Camera Modes. */
#ifndef HAVE_ENUM_CAMERA_MODE
#define HAVE_ENUM_CAMERA_MODE
typedef enum CAMERA_MODE
{
   CAMERA_MODE_IMAGE=0, /* Camera is in image/photo capture mode. | */
   CAMERA_MODE_VIDEO=1, /* Camera is in video capture mode. | */
   CAMERA_MODE_IMAGE_SURVEY=2, /* Camera is in image survey capture mode. It allows for camera controller to do specific settings for surveys. | */
   CAMERA_MODE_ENUM_END=3, /*  | */
} CAMERA_MODE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ARM_AUTH_DENIED_REASON
#define HAVE_ENUM_MAV_ARM_AUTH_DENIED_REASON
typedef enum MAV_ARM_AUTH_DENIED_REASON
{
   MAV_ARM_AUTH_DENIED_REASON_GENERIC=0, /* Not a specific reason | */
   MAV_ARM_AUTH_DENIED_REASON_NONE=1, /* Authorizer will send the error as string to GCS | */
   MAV_ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT=2, /* At least one waypoint have a invalid value | */
   MAV_ARM_AUTH_DENIED_REASON_TIMEOUT=3, /* Timeout in the authorizer process(in case it depends on network) | */
   MAV_ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE=4, /* Airspace of the mission in use by another vehicle, second result parameter can have the waypoint id that caused it to be denied. | */
   MAV_ARM_AUTH_DENIED_REASON_BAD_WEATHER=5, /* Weather is not good to fly | */
   MAV_ARM_AUTH_DENIED_REASON_ENUM_END=6, /*  | */
} MAV_ARM_AUTH_DENIED_REASON;
#endif

/** @brief RC type */
#ifndef HAVE_ENUM_RC_TYPE
#define HAVE_ENUM_RC_TYPE
typedef enum RC_TYPE
{
   RC_TYPE_SPEKTRUM_DSM2=0, /* Spektrum DSM2 | */
   RC_TYPE_SPEKTRUM_DSMX=1, /* Spektrum DSMX | */
   RC_TYPE_ENUM_END=2, /*  | */
} RC_TYPE;
#endif

/** @brief Bitmap to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 9 is set the floats afx afy afz should be interpreted as force instead of acceleration. */
#ifndef HAVE_ENUM_POSITION_TARGET_TYPEMASK
#define HAVE_ENUM_POSITION_TARGET_TYPEMASK
typedef enum POSITION_TARGET_TYPEMASK
{
   POSITION_TARGET_TYPEMASK_X_IGNORE=1, /* Ignore position x | */
   POSITION_TARGET_TYPEMASK_Y_IGNORE=2, /* Ignore position y | */
   POSITION_TARGET_TYPEMASK_Z_IGNORE=4, /* Ignore position z | */
   POSITION_TARGET_TYPEMASK_VX_IGNORE=8, /* Ignore velocity x | */
   POSITION_TARGET_TYPEMASK_VY_IGNORE=16, /* Ignore velocity y | */
   POSITION_TARGET_TYPEMASK_VZ_IGNORE=32, /* Ignore velocity z | */
   POSITION_TARGET_TYPEMASK_AX_IGNORE=64, /* Ignore acceleration x | */
   POSITION_TARGET_TYPEMASK_AY_IGNORE=128, /* Ignore acceleration y | */
   POSITION_TARGET_TYPEMASK_AZ_IGNORE=256, /* Ignore acceleration z | */
   POSITION_TARGET_TYPEMASK_FORCE_SET=512, /* Use force instead of acceleration | */
   POSITION_TARGET_TYPEMASK_YAW_IGNORE=1024, /* Ignore yaw | */
   POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE=2048, /* Ignore yaw rate | */
   POSITION_TARGET_TYPEMASK_ENUM_END=2049, /*  | */
} POSITION_TARGET_TYPEMASK;
#endif

/** @brief Airborne status of UAS. */
#ifndef HAVE_ENUM_UTM_FLIGHT_STATE
#define HAVE_ENUM_UTM_FLIGHT_STATE
typedef enum UTM_FLIGHT_STATE
{
   UTM_FLIGHT_STATE_UNKNOWN=1, /* The flight state can't be determined. | */
   UTM_FLIGHT_STATE_GROUND=2, /* UAS on ground. | */
   UTM_FLIGHT_STATE_AIRBORNE=3, /* UAS airborne. | */
   UTM_FLIGHT_STATE_EMERGENCY=16, /* UAS is in an emergency flight state. | */
   UTM_FLIGHT_STATE_NOCTRL=32, /* UAS has no active controls. | */
   UTM_FLIGHT_STATE_ENUM_END=33, /*  | */
} UTM_FLIGHT_STATE;
#endif

/** @brief Flags for the global position report. */
#ifndef HAVE_ENUM_UTM_DATA_AVAIL_FLAGS
#define HAVE_ENUM_UTM_DATA_AVAIL_FLAGS
typedef enum UTM_DATA_AVAIL_FLAGS
{
   UTM_DATA_AVAIL_FLAGS_TIME_VALID=1, /* The field time contains valid data. | */
   UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE=2, /* The field uas_id contains valid data. | */
   UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE=4, /* The fields lat, lon and h_acc contain valid data. | */
   UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE=8, /* The fields alt and v_acc contain valid data. | */
   UTM_DATA_AVAIL_FLAGS_RELATIVE_ALTITUDE_AVAILABLE=16, /* The field relative_alt contains valid data. | */
   UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE=32, /* The fields vx and vy contain valid data. | */
   UTM_DATA_AVAIL_FLAGS_VERTICAL_VELO_AVAILABLE=64, /* The field vz contains valid data. | */
   UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE=128, /* The fields next_lat, next_lon and next_alt contain valid data. | */
   UTM_DATA_AVAIL_FLAGS_ENUM_END=129, /*  | */
} UTM_DATA_AVAIL_FLAGS;
#endif

/** @brief Cellular network radio type */
#ifndef HAVE_ENUM_CELLULAR_NETWORK_RADIO_TYPE
#define HAVE_ENUM_CELLULAR_NETWORK_RADIO_TYPE
typedef enum CELLULAR_NETWORK_RADIO_TYPE
{
   CELLULAR_NETWORK_RADIO_TYPE_NONE=0, /*  | */
   CELLULAR_NETWORK_RADIO_TYPE_GSM=1, /*  | */
   CELLULAR_NETWORK_RADIO_TYPE_CDMA=2, /*  | */
   CELLULAR_NETWORK_RADIO_TYPE_WCDMA=3, /*  | */
   CELLULAR_NETWORK_RADIO_TYPE_LTE=4, /*  | */
   CELLULAR_NETWORK_RADIO_TYPE_ENUM_END=5, /*  | */
} CELLULAR_NETWORK_RADIO_TYPE;
#endif

/** @brief These flags encode the cellular network status */
#ifndef HAVE_ENUM_CELLULAR_NETWORK_STATUS_FLAG
#define HAVE_ENUM_CELLULAR_NETWORK_STATUS_FLAG
typedef enum CELLULAR_NETWORK_STATUS_FLAG
{
   CELLULAR_NETWORK_STATUS_FLAG_ROAMING=1, /* Roaming is active | */
   CELLULAR_NETWORK_STATUS_FLAG_ENUM_END=2, /*  | */
} CELLULAR_NETWORK_STATUS_FLAG;
#endif

/** @brief Precision land modes (used in MAV_CMD_NAV_LAND). */
#ifndef HAVE_ENUM_PRECISION_LAND_MODE
#define HAVE_ENUM_PRECISION_LAND_MODE
typedef enum PRECISION_LAND_MODE
{
   PRECISION_LAND_MODE_DISABLED=0, /* Normal (non-precision) landing. | */
   PRECISION_LAND_MODE_OPPORTUNISTIC=1, /* Use precision landing if beacon detected when land command accepted, otherwise land normally. | */
   PRECISION_LAND_MODE_REQUIRED=2, /* Use precision landing, searching for beacon if not found when land command accepted (land normally if beacon cannot be found). | */
   PRECISION_LAND_MODE_ENUM_END=3, /*  | */
} PRECISION_LAND_MODE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_PARACHUTE_ACTION
#define HAVE_ENUM_PARACHUTE_ACTION
typedef enum PARACHUTE_ACTION
{
   PARACHUTE_DISABLE=0, /* Disable parachute release. | */
   PARACHUTE_ENABLE=1, /* Enable parachute release. | */
   PARACHUTE_RELEASE=2, /* Release parachute. | */
   PARACHUTE_ACTION_ENUM_END=3, /*  | */
} PARACHUTE_ACTION;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_TUNNEL_PAYLOAD_TYPE
#define HAVE_ENUM_MAV_TUNNEL_PAYLOAD_TYPE
typedef enum MAV_TUNNEL_PAYLOAD_TYPE
{
   MAV_TUNNEL_PAYLOAD_TYPE_UNKNOWN=0, /* Encoding of payload unknown. | */
   MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED0=200, /* Registered for STorM32 gimbal controller. | */
   MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED1=201, /* Registered for STorM32 gimbal controller. | */
   MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED2=202, /* Registered for STorM32 gimbal controller. | */
   MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED3=203, /* Registered for STorM32 gimbal controller. | */
   MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED4=204, /* Registered for STorM32 gimbal controller. | */
   MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED5=205, /* Registered for STorM32 gimbal controller. | */
   MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED6=206, /* Registered for STorM32 gimbal controller. | */
   MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED7=207, /* Registered for STorM32 gimbal controller. | */
   MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED8=208, /* Registered for STorM32 gimbal controller. | */
   MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED9=209, /* Registered for STorM32 gimbal controller. | */
   MAV_TUNNEL_PAYLOAD_TYPE_ENUM_END=210, /*  | */
} MAV_TUNNEL_PAYLOAD_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_ID_TYPE
#define HAVE_ENUM_MAV_ODID_ID_TYPE
typedef enum MAV_ODID_ID_TYPE
{
   MAV_ODID_ID_TYPE_NONE=0, /* No type defined. | */
   MAV_ODID_ID_TYPE_SERIAL_NUMBER=1, /* Manufacturer Serial Number (ANSI/CTA-2063 format). | */
   MAV_ODID_ID_TYPE_CAA_REGISTRATION_ID=2, /* CAA (Civil Aviation Authority) registered ID. Format: [ICAO Country Code].[CAA Assigned ID]. | */
   MAV_ODID_ID_TYPE_UTM_ASSIGNED_UUID=3, /* UTM (Unmanned Traffic Management) assigned UUID (RFC4122). | */
   MAV_ODID_ID_TYPE_ENUM_END=4, /*  | */
} MAV_ODID_ID_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_UA_TYPE
#define HAVE_ENUM_MAV_ODID_UA_TYPE
typedef enum MAV_ODID_UA_TYPE
{
   MAV_ODID_UA_TYPE_NONE=0, /* No UA (Unmanned Aircraft) type defined. | */
   MAV_ODID_UA_TYPE_AEROPLANE=1, /* Aeroplane/Airplane. Fixed wing. | */
   MAV_ODID_UA_TYPE_ROTORCRAFT=2, /* Rotorcraft (including Multirotor). | */
   MAV_ODID_UA_TYPE_GYROPLANE=3, /* Gyroplane. | */
   MAV_ODID_UA_TYPE_VTOL=4, /* VTOL (Vertical Take-Off and Landing). Fixed wing aircraft that can take off vertically. | */
   MAV_ODID_UA_TYPE_ORNITHOPTER=5, /* Ornithopter. | */
   MAV_ODID_UA_TYPE_GLIDER=6, /* Glider. | */
   MAV_ODID_UA_TYPE_KITE=7, /* Kite. | */
   MAV_ODID_UA_TYPE_FREE_BALLOON=8, /* Free Balloon. | */
   MAV_ODID_UA_TYPE_CAPTIVE_BALLOON=9, /* Captive Balloon. | */
   MAV_ODID_UA_TYPE_AIRSHIP=10, /* Airship. E.g. a blimp. | */
   MAV_ODID_UA_TYPE_FREE_FALL_PARACHUTE=11, /* Free Fall/Parachute. | */
   MAV_ODID_UA_TYPE_ROCKET=12, /* Rocket. | */
   MAV_ODID_UA_TYPE_TETHERED_POWERED_AIRCRAFT=13, /* Tethered powered aircraft. | */
   MAV_ODID_UA_TYPE_GROUND_OBSTACLE=14, /* Ground Obstacle. | */
   MAV_ODID_UA_TYPE_OTHER=15, /* Other type of aircraft not listed earlier. | */
   MAV_ODID_UA_TYPE_ENUM_END=16, /*  | */
} MAV_ODID_UA_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_STATUS
#define HAVE_ENUM_MAV_ODID_STATUS
typedef enum MAV_ODID_STATUS
{
   MAV_ODID_STATUS_UNDECLARED=0, /* The status of the (UA) Unmanned Aircraft is undefined. | */
   MAV_ODID_STATUS_GROUND=1, /* The UA is on the ground. | */
   MAV_ODID_STATUS_AIRBORNE=2, /* The UA is in the air. | */
   MAV_ODID_STATUS_ENUM_END=3, /*  | */
} MAV_ODID_STATUS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_HEIGHT_REF
#define HAVE_ENUM_MAV_ODID_HEIGHT_REF
typedef enum MAV_ODID_HEIGHT_REF
{
   MAV_ODID_HEIGHT_REF_OVER_TAKEOFF=0, /* The height field is relative to the take-off location. | */
   MAV_ODID_HEIGHT_REF_OVER_GROUND=1, /* The height field is relative to ground. | */
   MAV_ODID_HEIGHT_REF_ENUM_END=2, /*  | */
} MAV_ODID_HEIGHT_REF;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_HOR_ACC
#define HAVE_ENUM_MAV_ODID_HOR_ACC
typedef enum MAV_ODID_HOR_ACC
{
   MAV_ODID_HOR_ACC_UNKNOWN=0, /* The horizontal accuracy is unknown. | */
   MAV_ODID_HOR_ACC_10NM=1, /* The horizontal accuracy is smaller than 10 Nautical Miles. 18.52 km. | */
   MAV_ODID_HOR_ACC_4NM=2, /* The horizontal accuracy is smaller than 4 Nautical Miles. 7.408 km. | */
   MAV_ODID_HOR_ACC_2NM=3, /* The horizontal accuracy is smaller than 2 Nautical Miles. 3.704 km. | */
   MAV_ODID_HOR_ACC_1NM=4, /* The horizontal accuracy is smaller than 1 Nautical Miles. 1.852 km. | */
   MAV_ODID_HOR_ACC_0_5NM=5, /* The horizontal accuracy is smaller than 0.5 Nautical Miles. 926 m. | */
   MAV_ODID_HOR_ACC_0_3NM=6, /* The horizontal accuracy is smaller than 0.3 Nautical Miles. 555.6 m. | */
   MAV_ODID_HOR_ACC_0_1NM=7, /* The horizontal accuracy is smaller than 0.1 Nautical Miles. 185.2 m. | */
   MAV_ODID_HOR_ACC_0_05NM=8, /* The horizontal accuracy is smaller than 0.05 Nautical Miles. 92.6 m. | */
   MAV_ODID_HOR_ACC_30_METER=9, /* The horizontal accuracy is smaller than 30 meter. | */
   MAV_ODID_HOR_ACC_10_METER=10, /* The horizontal accuracy is smaller than 10 meter. | */
   MAV_ODID_HOR_ACC_3_METER=11, /* The horizontal accuracy is smaller than 3 meter. | */
   MAV_ODID_HOR_ACC_1_METER=12, /* The horizontal accuracy is smaller than 1 meter. | */
   MAV_ODID_HOR_ACC_ENUM_END=13, /*  | */
} MAV_ODID_HOR_ACC;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_VER_ACC
#define HAVE_ENUM_MAV_ODID_VER_ACC
typedef enum MAV_ODID_VER_ACC
{
   MAV_ODID_VER_ACC_UNKNOWN=0, /* The vertical accuracy is unknown. | */
   MAV_ODID_VER_ACC_150_METER=1, /* The vertical accuracy is smaller than 150 meter. | */
   MAV_ODID_VER_ACC_45_METER=2, /* The vertical accuracy is smaller than 45 meter. | */
   MAV_ODID_VER_ACC_25_METER=3, /* The vertical accuracy is smaller than 25 meter. | */
   MAV_ODID_VER_ACC_10_METER=4, /* The vertical accuracy is smaller than 10 meter. | */
   MAV_ODID_VER_ACC_3_METER=5, /* The vertical accuracy is smaller than 3 meter. | */
   MAV_ODID_VER_ACC_1_METER=6, /* The vertical accuracy is smaller than 1 meter. | */
   MAV_ODID_VER_ACC_ENUM_END=7, /*  | */
} MAV_ODID_VER_ACC;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_SPEED_ACC
#define HAVE_ENUM_MAV_ODID_SPEED_ACC
typedef enum MAV_ODID_SPEED_ACC
{
   MAV_ODID_SPEED_ACC_UNKNOWN=0, /* The speed accuracy is unknown. | */
   MAV_ODID_SPEED_ACC_10_METERS_PER_SECOND=1, /* The speed accuracy is smaller than 10 meters per second. | */
   MAV_ODID_SPEED_ACC_3_METERS_PER_SECOND=2, /* The speed accuracy is smaller than 3 meters per second. | */
   MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND=3, /* The speed accuracy is smaller than 1 meters per second. | */
   MAV_ODID_SPEED_ACC_0_3_METERS_PER_SECOND=4, /* The speed accuracy is smaller than 0.3 meters per second. | */
   MAV_ODID_SPEED_ACC_ENUM_END=5, /*  | */
} MAV_ODID_SPEED_ACC;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_TIME_ACC
#define HAVE_ENUM_MAV_ODID_TIME_ACC
typedef enum MAV_ODID_TIME_ACC
{
   MAV_ODID_TIME_ACC_UNKNOWN=0, /* The timestamp accuracy is unknown. | */
   MAV_ODID_TIME_ACC_0_1_SECOND=1, /* The timestamp accuracy is smaller than 0.1 second. | */
   MAV_ODID_TIME_ACC_0_2_SECOND=2, /* The timestamp accuracy is smaller than 0.2 second. | */
   MAV_ODID_TIME_ACC_0_3_SECOND=3, /* The timestamp accuracy is smaller than 0.3 second. | */
   MAV_ODID_TIME_ACC_0_4_SECOND=4, /* The timestamp accuracy is smaller than 0.4 second. | */
   MAV_ODID_TIME_ACC_0_5_SECOND=5, /* The timestamp accuracy is smaller than 0.5 second. | */
   MAV_ODID_TIME_ACC_0_6_SECOND=6, /* The timestamp accuracy is smaller than 0.6 second. | */
   MAV_ODID_TIME_ACC_0_7_SECOND=7, /* The timestamp accuracy is smaller than 0.7 second. | */
   MAV_ODID_TIME_ACC_0_8_SECOND=8, /* The timestamp accuracy is smaller than 0.8 second. | */
   MAV_ODID_TIME_ACC_0_9_SECOND=9, /* The timestamp accuracy is smaller than 0.9 second. | */
   MAV_ODID_TIME_ACC_1_0_SECOND=10, /* The timestamp accuracy is smaller than 1.0 second. | */
   MAV_ODID_TIME_ACC_1_1_SECOND=11, /* The timestamp accuracy is smaller than 1.1 second. | */
   MAV_ODID_TIME_ACC_1_2_SECOND=12, /* The timestamp accuracy is smaller than 1.2 second. | */
   MAV_ODID_TIME_ACC_1_3_SECOND=13, /* The timestamp accuracy is smaller than 1.3 second. | */
   MAV_ODID_TIME_ACC_1_4_SECOND=14, /* The timestamp accuracy is smaller than 1.4 second. | */
   MAV_ODID_TIME_ACC_1_5_SECOND=15, /* The timestamp accuracy is smaller than 1.5 second. | */
   MAV_ODID_TIME_ACC_ENUM_END=16, /*  | */
} MAV_ODID_TIME_ACC;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_AUTH_TYPE
#define HAVE_ENUM_MAV_ODID_AUTH_TYPE
typedef enum MAV_ODID_AUTH_TYPE
{
   MAV_ODID_AUTH_TYPE_NONE=0, /* No authentication type is specified. | */
   MAV_ODID_AUTH_TYPE_UAS_ID_SIGNATURE=1, /* Signature for the UAS (Unmanned Aircraft System) ID. | */
   MAV_ODID_AUTH_TYPE_OPERATOR_ID_SIGNATURE=2, /* Signature for the Operator ID. | */
   MAV_ODID_AUTH_TYPE_MESSAGE_SET_SIGNATURE=3, /* Signature for the entire message set. | */
   MAV_ODID_AUTH_TYPE_NETWORK_REMOTE_ID=4, /* Authentication is provided by Network Remote ID. | */
   MAV_ODID_AUTH_TYPE_ENUM_END=5, /*  | */
} MAV_ODID_AUTH_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_DESC_TYPE
#define HAVE_ENUM_MAV_ODID_DESC_TYPE
typedef enum MAV_ODID_DESC_TYPE
{
   MAV_ODID_DESC_TYPE_TEXT=0, /* Free-form text description of the purpose of the flight. | */
   MAV_ODID_DESC_TYPE_ENUM_END=1, /*  | */
} MAV_ODID_DESC_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_LOCATION_SRC
#define HAVE_ENUM_MAV_ODID_LOCATION_SRC
typedef enum MAV_ODID_LOCATION_SRC
{
   MAV_ODID_LOCATION_SRC_TAKEOFF=0, /* The location of the operator is the same as the take-off location. | */
   MAV_ODID_LOCATION_SRC_LIVE_GNSS=1, /* The location of the operator is based on live GNSS data. | */
   MAV_ODID_LOCATION_SRC_FIXED=2, /* The location of the operator is a fixed location. | */
   MAV_ODID_LOCATION_SRC_ENUM_END=3, /*  | */
} MAV_ODID_LOCATION_SRC;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_OPERATOR_ID_TYPE
#define HAVE_ENUM_MAV_ODID_OPERATOR_ID_TYPE
typedef enum MAV_ODID_OPERATOR_ID_TYPE
{
   MAV_ODID_OPERATOR_ID_TYPE_CAA=0, /* CAA (Civil Aviation Authority) registered operator ID. | */
   MAV_ODID_OPERATOR_ID_TYPE_ENUM_END=1, /*  | */
} MAV_ODID_OPERATOR_ID_TYPE;
#endif

/** @brief Tune formats (used for vehicle buzzer/tone generation). */
#ifndef HAVE_ENUM_TUNE_FORMAT
#define HAVE_ENUM_TUNE_FORMAT
typedef enum TUNE_FORMAT
{
   TUNE_FORMAT_QBASIC1_1=1, /* Format is QBasic 1.1 Play: https://www.qbasic.net/en/reference/qb11/Statement/PLAY-006.htm. | */
   TUNE_FORMAT_MML_MODERN=2, /* Format is Modern Music Markup Language (MML): https://en.wikipedia.org/wiki/Music_Macro_Language#Modern_MML. | */
   TUNE_FORMAT_ENUM_END=3, /*  | */
} TUNE_FORMAT;
#endif

/** @brief Component capability flags (Bitmap) */
#ifndef HAVE_ENUM_COMPONENT_CAP_FLAGS
#define HAVE_ENUM_COMPONENT_CAP_FLAGS
typedef enum COMPONENT_CAP_FLAGS
{
   COMPONENT_CAP_FLAGS_PARAM=1, /* Component has parameters, and supports the parameter protocol (PARAM messages). | */
   COMPONENT_CAP_FLAGS_PARAM_EXT=2, /* Component has parameters, and supports the extended parameter protocol (PARAM_EXT messages). | */
   COMPONENT_CAP_FLAGS_ENUM_END=3, /*  | */
} COMPONENT_CAP_FLAGS;
#endif

/** @brief Type of AIS vessel, enum duplicated from AIS standard, https://gpsd.gitlab.io/gpsd/AIVDM.html */
#ifndef HAVE_ENUM_AIS_TYPE
#define HAVE_ENUM_AIS_TYPE
typedef enum AIS_TYPE
{
   AIS_TYPE_UNKNOWN=0, /* Not available (default). | */
   AIS_TYPE_RESERVED_1=1, /*  | */
   AIS_TYPE_RESERVED_2=2, /*  | */
   AIS_TYPE_RESERVED_3=3, /*  | */
   AIS_TYPE_RESERVED_4=4, /*  | */
   AIS_TYPE_RESERVED_5=5, /*  | */
   AIS_TYPE_RESERVED_6=6, /*  | */
   AIS_TYPE_RESERVED_7=7, /*  | */
   AIS_TYPE_RESERVED_8=8, /*  | */
   AIS_TYPE_RESERVED_9=9, /*  | */
   AIS_TYPE_RESERVED_10=10, /*  | */
   AIS_TYPE_RESERVED_11=11, /*  | */
   AIS_TYPE_RESERVED_12=12, /*  | */
   AIS_TYPE_RESERVED_13=13, /*  | */
   AIS_TYPE_RESERVED_14=14, /*  | */
   AIS_TYPE_RESERVED_15=15, /*  | */
   AIS_TYPE_RESERVED_16=16, /*  | */
   AIS_TYPE_RESERVED_17=17, /*  | */
   AIS_TYPE_RESERVED_18=18, /*  | */
   AIS_TYPE_RESERVED_19=19, /*  | */
   AIS_TYPE_WIG=20, /* Wing In Ground effect. | */
   AIS_TYPE_WIG_HAZARDOUS_A=21, /*  | */
   AIS_TYPE_WIG_HAZARDOUS_B=22, /*  | */
   AIS_TYPE_WIG_HAZARDOUS_C=23, /*  | */
   AIS_TYPE_WIG_HAZARDOUS_D=24, /*  | */
   AIS_TYPE_WIG_RESERVED_1=25, /*  | */
   AIS_TYPE_WIG_RESERVED_2=26, /*  | */
   AIS_TYPE_WIG_RESERVED_3=27, /*  | */
   AIS_TYPE_WIG_RESERVED_4=28, /*  | */
   AIS_TYPE_WIG_RESERVED_5=29, /*  | */
   AIS_TYPE_FISHING=30, /*  | */
   AIS_TYPE_TOWING=31, /*  | */
   AIS_TYPE_TOWING_LARGE=32, /* Towing: length exceeds 200m or breadth exceeds 25m. | */
   AIS_TYPE_DREDGING=33, /* Dredging or other underwater ops. | */
   AIS_TYPE_DIVING=34, /*  | */
   AIS_TYPE_MILITARY=35, /*  | */
   AIS_TYPE_SAILING=36, /*  | */
   AIS_TYPE_PLEASURE=37, /*  | */
   AIS_TYPE_RESERVED_20=38, /*  | */
   AIS_TYPE_RESERVED_21=39, /*  | */
   AIS_TYPE_HSC=40, /* High Speed Craft. | */
   AIS_TYPE_HSC_HAZARDOUS_A=41, /*  | */
   AIS_TYPE_HSC_HAZARDOUS_B=42, /*  | */
   AIS_TYPE_HSC_HAZARDOUS_C=43, /*  | */
   AIS_TYPE_HSC_HAZARDOUS_D=44, /*  | */
   AIS_TYPE_HSC_RESERVED_1=45, /*  | */
   AIS_TYPE_HSC_RESERVED_2=46, /*  | */
   AIS_TYPE_HSC_RESERVED_3=47, /*  | */
   AIS_TYPE_HSC_RESERVED_4=48, /*  | */
   AIS_TYPE_HSC_UNKNOWN=49, /*  | */
   AIS_TYPE_PILOT=50, /*  | */
   AIS_TYPE_SAR=51, /* Search And Rescue vessel. | */
   AIS_TYPE_TUG=52, /*  | */
   AIS_TYPE_PORT_TENDER=53, /*  | */
   AIS_TYPE_ANTI_POLLUTION=54, /* Anti-pollution equipment. | */
   AIS_TYPE_LAW_ENFORCEMENT=55, /*  | */
   AIS_TYPE_SPARE_LOCAL_1=56, /*  | */
   AIS_TYPE_SPARE_LOCAL_2=57, /*  | */
   AIS_TYPE_MEDICAL_TRANSPORT=58, /*  | */
   AIS_TYPE_NONECOMBATANT=59, /* Noncombatant ship according to RR Resolution No. 18. | */
   AIS_TYPE_PASSENGER=60, /*  | */
   AIS_TYPE_PASSENGER_HAZARDOUS_A=61, /*  | */
   AIS_TYPE_PASSENGER_HAZARDOUS_B=62, /*  | */
   AIS_TYPE_AIS_TYPE_PASSENGER_HAZARDOUS_C=63, /*  | */
   AIS_TYPE_PASSENGER_HAZARDOUS_D=64, /*  | */
   AIS_TYPE_PASSENGER_RESERVED_1=65, /*  | */
   AIS_TYPE_PASSENGER_RESERVED_2=66, /*  | */
   AIS_TYPE_PASSENGER_RESERVED_3=67, /*  | */
   AIS_TYPE_AIS_TYPE_PASSENGER_RESERVED_4=68, /*  | */
   AIS_TYPE_PASSENGER_UNKNOWN=69, /*  | */
   AIS_TYPE_CARGO=70, /*  | */
   AIS_TYPE_CARGO_HAZARDOUS_A=71, /*  | */
   AIS_TYPE_CARGO_HAZARDOUS_B=72, /*  | */
   AIS_TYPE_CARGO_HAZARDOUS_C=73, /*  | */
   AIS_TYPE_CARGO_HAZARDOUS_D=74, /*  | */
   AIS_TYPE_CARGO_RESERVED_1=75, /*  | */
   AIS_TYPE_CARGO_RESERVED_2=76, /*  | */
   AIS_TYPE_CARGO_RESERVED_3=77, /*  | */
   AIS_TYPE_CARGO_RESERVED_4=78, /*  | */
   AIS_TYPE_CARGO_UNKNOWN=79, /*  | */
   AIS_TYPE_TANKER=80, /*  | */
   AIS_TYPE_TANKER_HAZARDOUS_A=81, /*  | */
   AIS_TYPE_TANKER_HAZARDOUS_B=82, /*  | */
   AIS_TYPE_TANKER_HAZARDOUS_C=83, /*  | */
   AIS_TYPE_TANKER_HAZARDOUS_D=84, /*  | */
   AIS_TYPE_TANKER_RESERVED_1=85, /*  | */
   AIS_TYPE_TANKER_RESERVED_2=86, /*  | */
   AIS_TYPE_TANKER_RESERVED_3=87, /*  | */
   AIS_TYPE_TANKER_RESERVED_4=88, /*  | */
   AIS_TYPE_TANKER_UNKNOWN=89, /*  | */
   AIS_TYPE_OTHER=90, /*  | */
   AIS_TYPE_OTHER_HAZARDOUS_A=91, /*  | */
   AIS_TYPE_OTHER_HAZARDOUS_B=92, /*  | */
   AIS_TYPE_OTHER_HAZARDOUS_C=93, /*  | */
   AIS_TYPE_OTHER_HAZARDOUS_D=94, /*  | */
   AIS_TYPE_OTHER_RESERVED_1=95, /*  | */
   AIS_TYPE_OTHER_RESERVED_2=96, /*  | */
   AIS_TYPE_OTHER_RESERVED_3=97, /*  | */
   AIS_TYPE_OTHER_RESERVED_4=98, /*  | */
   AIS_TYPE_OTHER_UNKNOWN=99, /*  | */
   AIS_TYPE_ENUM_END=100, /*  | */
} AIS_TYPE;
#endif

/** @brief Navigational status of AIS vessel, enum duplicated from AIS standard, https://gpsd.gitlab.io/gpsd/AIVDM.html */
#ifndef HAVE_ENUM_AIS_NAV_STATUS
#define HAVE_ENUM_AIS_NAV_STATUS
typedef enum AIS_NAV_STATUS
{
   UNDER_WAY=0, /* Under way using engine. | */
   AIS_NAV_ANCHORED=1, /*  | */
   AIS_NAV_UN_COMMANDED=2, /*  | */
   AIS_NAV_RESTRICTED_MANOEUVERABILITY=3, /*  | */
   AIS_NAV_DRAUGHT_CONSTRAINED=4, /*  | */
   AIS_NAV_MOORED=5, /*  | */
   AIS_NAV_AGROUND=6, /*  | */
   AIS_NAV_FISHING=7, /*  | */
   AIS_NAV_SAILING=8, /*  | */
   AIS_NAV_RESERVED_HSC=9, /*  | */
   AIS_NAV_RESERVED_WIG=10, /*  | */
   AIS_NAV_RESERVED_1=11, /*  | */
   AIS_NAV_RESERVED_2=12, /*  | */
   AIS_NAV_RESERVED_3=13, /*  | */
   AIS_NAV_AIS_SART=14, /* Search And Rescue Transponder. | */
   AIS_NAV_UNKNOWN=15, /* Not available (default). | */
   AIS_NAV_STATUS_ENUM_END=16, /*  | */
} AIS_NAV_STATUS;
#endif

/** @brief These flags are used in the AIS_VESSEL.fields bitmask to indicate validity of data in the other message fields. When set, the data is valid. */
#ifndef HAVE_ENUM_AIS_FLAGS
#define HAVE_ENUM_AIS_FLAGS
typedef enum AIS_FLAGS
{
   AIS_FLAGS_POSITION_ACCURACY=1, /* 1 = Position accuracy less than 10m, 0 = position accuracy greater than 10m. | */
   AIS_FLAGS_VALID_COG=2, /*  | */
   AIS_FLAGS_VALID_VELOCITY=4, /*  | */
   AIS_FLAGS_HIGH_VELOCITY=8, /* 1 = Velocity over 52.5765m/s (102.2 knots) | */
   AIS_FLAGS_VALID_TURN_RATE=16, /*  | */
   AIS_FLAGS_TURN_RATE_SIGN_ONLY=32, /* Only the sign of the returned turn rate value is valid, either greater than 5deg/30s or less than -5deg/30s | */
   AIS_FLAGS_VALID_DIMENSIONS=64, /*  | */
   AIS_FLAGS_LARGE_BOW_DIMENSION=128, /* Distance to bow is larger than 511m | */
   AIS_FLAGS_LARGE_STERN_DIMENSION=256, /* Distance to stern is larger than 511m | */
   AIS_FLAGS_LARGE_PORT_DIMENSION=512, /* Distance to port side is larger than 63m | */
   AIS_FLAGS_LARGE_STARBOARD_DIMENSION=1024, /* Distance to starboard side is larger than 63m | */
   AIS_FLAGS_VALID_CALLSIGN=2048, /*  | */
   AIS_FLAGS_VALID_NAME=4096, /*  | */
   AIS_FLAGS_ENUM_END=4097, /*  | */
} AIS_FLAGS;
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
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_HEARTBEAT, MAVLINK_MESSAGE_INFO_SYS_STATUS, MAVLINK_MESSAGE_INFO_SYSTEM_TIME, MAVLINK_MESSAGE_INFO_PING, MAVLINK_MESSAGE_INFO_CHANGE_OPERATOR_CONTROL, MAVLINK_MESSAGE_INFO_CHANGE_OPERATOR_CONTROL_ACK, MAVLINK_MESSAGE_INFO_AUTH_KEY, MAVLINK_MESSAGE_INFO_LINK_NODE_STATUS, MAVLINK_MESSAGE_INFO_SET_MODE, MAVLINK_MESSAGE_INFO_PARAM_REQUEST_READ, MAVLINK_MESSAGE_INFO_PARAM_REQUEST_LIST, MAVLINK_MESSAGE_INFO_PARAM_VALUE, MAVLINK_MESSAGE_INFO_PARAM_SET, MAVLINK_MESSAGE_INFO_GPS_RAW_INT, MAVLINK_MESSAGE_INFO_GPS_STATUS, MAVLINK_MESSAGE_INFO_SCALED_IMU, MAVLINK_MESSAGE_INFO_RAW_IMU, MAVLINK_MESSAGE_INFO_RAW_PRESSURE, MAVLINK_MESSAGE_INFO_SCALED_PRESSURE, MAVLINK_MESSAGE_INFO_ATTITUDE, MAVLINK_MESSAGE_INFO_ATTITUDE_QUATERNION, MAVLINK_MESSAGE_INFO_LOCAL_POSITION_NED, MAVLINK_MESSAGE_INFO_GLOBAL_POSITION_INT, MAVLINK_MESSAGE_INFO_RC_CHANNELS_SCALED, MAVLINK_MESSAGE_INFO_RC_CHANNELS_RAW, MAVLINK_MESSAGE_INFO_SERVO_OUTPUT_RAW, MAVLINK_MESSAGE_INFO_MISSION_REQUEST_PARTIAL_LIST, MAVLINK_MESSAGE_INFO_MISSION_WRITE_PARTIAL_LIST, MAVLINK_MESSAGE_INFO_MISSION_ITEM, MAVLINK_MESSAGE_INFO_MISSION_REQUEST, MAVLINK_MESSAGE_INFO_MISSION_SET_CURRENT, MAVLINK_MESSAGE_INFO_MISSION_CURRENT, MAVLINK_MESSAGE_INFO_MISSION_REQUEST_LIST, MAVLINK_MESSAGE_INFO_MISSION_COUNT, MAVLINK_MESSAGE_INFO_MISSION_CLEAR_ALL, MAVLINK_MESSAGE_INFO_MISSION_ITEM_REACHED, MAVLINK_MESSAGE_INFO_MISSION_ACK, MAVLINK_MESSAGE_INFO_SET_GPS_GLOBAL_ORIGIN, MAVLINK_MESSAGE_INFO_GPS_GLOBAL_ORIGIN, MAVLINK_MESSAGE_INFO_PARAM_MAP_RC, MAVLINK_MESSAGE_INFO_MISSION_REQUEST_INT, MAVLINK_MESSAGE_INFO_MISSION_CHANGED, MAVLINK_MESSAGE_INFO_SAFETY_SET_ALLOWED_AREA, MAVLINK_MESSAGE_INFO_SAFETY_ALLOWED_AREA, MAVLINK_MESSAGE_INFO_ATTITUDE_QUATERNION_COV, MAVLINK_MESSAGE_INFO_NAV_CONTROLLER_OUTPUT, MAVLINK_MESSAGE_INFO_GLOBAL_POSITION_INT_COV, MAVLINK_MESSAGE_INFO_LOCAL_POSITION_NED_COV, MAVLINK_MESSAGE_INFO_RC_CHANNELS, MAVLINK_MESSAGE_INFO_REQUEST_DATA_STREAM, MAVLINK_MESSAGE_INFO_DATA_STREAM, MAVLINK_MESSAGE_INFO_MANUAL_CONTROL, MAVLINK_MESSAGE_INFO_RC_CHANNELS_OVERRIDE, MAVLINK_MESSAGE_INFO_MISSION_ITEM_INT, MAVLINK_MESSAGE_INFO_VFR_HUD, MAVLINK_MESSAGE_INFO_COMMAND_INT, MAVLINK_MESSAGE_INFO_COMMAND_LONG, MAVLINK_MESSAGE_INFO_COMMAND_ACK, MAVLINK_MESSAGE_INFO_MANUAL_SETPOINT, MAVLINK_MESSAGE_INFO_SET_ATTITUDE_TARGET, MAVLINK_MESSAGE_INFO_ATTITUDE_TARGET, MAVLINK_MESSAGE_INFO_SET_POSITION_TARGET_LOCAL_NED, MAVLINK_MESSAGE_INFO_POSITION_TARGET_LOCAL_NED, MAVLINK_MESSAGE_INFO_SET_POSITION_TARGET_GLOBAL_INT, MAVLINK_MESSAGE_INFO_POSITION_TARGET_GLOBAL_INT, MAVLINK_MESSAGE_INFO_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET, MAVLINK_MESSAGE_INFO_HIL_STATE, MAVLINK_MESSAGE_INFO_HIL_CONTROLS, MAVLINK_MESSAGE_INFO_HIL_RC_INPUTS_RAW, MAVLINK_MESSAGE_INFO_HIL_ACTUATOR_CONTROLS, MAVLINK_MESSAGE_INFO_OPTICAL_FLOW, MAVLINK_MESSAGE_INFO_GLOBAL_VISION_POSITION_ESTIMATE, MAVLINK_MESSAGE_INFO_VISION_POSITION_ESTIMATE, MAVLINK_MESSAGE_INFO_VISION_SPEED_ESTIMATE, MAVLINK_MESSAGE_INFO_VICON_POSITION_ESTIMATE, MAVLINK_MESSAGE_INFO_HIGHRES_IMU, MAVLINK_MESSAGE_INFO_OPTICAL_FLOW_RAD, MAVLINK_MESSAGE_INFO_HIL_SENSOR, MAVLINK_MESSAGE_INFO_SIM_STATE, MAVLINK_MESSAGE_INFO_RADIO_STATUS, MAVLINK_MESSAGE_INFO_FILE_TRANSFER_PROTOCOL, MAVLINK_MESSAGE_INFO_TIMESYNC, MAVLINK_MESSAGE_INFO_CAMERA_TRIGGER, MAVLINK_MESSAGE_INFO_HIL_GPS, MAVLINK_MESSAGE_INFO_HIL_OPTICAL_FLOW, MAVLINK_MESSAGE_INFO_HIL_STATE_QUATERNION, MAVLINK_MESSAGE_INFO_SCALED_IMU2, MAVLINK_MESSAGE_INFO_LOG_REQUEST_LIST, MAVLINK_MESSAGE_INFO_LOG_ENTRY, MAVLINK_MESSAGE_INFO_LOG_REQUEST_DATA, MAVLINK_MESSAGE_INFO_LOG_DATA, MAVLINK_MESSAGE_INFO_LOG_ERASE, MAVLINK_MESSAGE_INFO_LOG_REQUEST_END, MAVLINK_MESSAGE_INFO_GPS_INJECT_DATA, MAVLINK_MESSAGE_INFO_GPS2_RAW, MAVLINK_MESSAGE_INFO_POWER_STATUS, MAVLINK_MESSAGE_INFO_SERIAL_CONTROL, MAVLINK_MESSAGE_INFO_GPS_RTK, MAVLINK_MESSAGE_INFO_GPS2_RTK, MAVLINK_MESSAGE_INFO_SCALED_IMU3, MAVLINK_MESSAGE_INFO_DATA_TRANSMISSION_HANDSHAKE, MAVLINK_MESSAGE_INFO_ENCAPSULATED_DATA, MAVLINK_MESSAGE_INFO_DISTANCE_SENSOR, MAVLINK_MESSAGE_INFO_TERRAIN_REQUEST, MAVLINK_MESSAGE_INFO_TERRAIN_DATA, MAVLINK_MESSAGE_INFO_TERRAIN_CHECK, MAVLINK_MESSAGE_INFO_TERRAIN_REPORT, MAVLINK_MESSAGE_INFO_SCALED_PRESSURE2, MAVLINK_MESSAGE_INFO_ATT_POS_MOCAP, MAVLINK_MESSAGE_INFO_SET_ACTUATOR_CONTROL_TARGET, MAVLINK_MESSAGE_INFO_ACTUATOR_CONTROL_TARGET, MAVLINK_MESSAGE_INFO_ALTITUDE, MAVLINK_MESSAGE_INFO_RESOURCE_REQUEST, MAVLINK_MESSAGE_INFO_SCALED_PRESSURE3, MAVLINK_MESSAGE_INFO_FOLLOW_TARGET, MAVLINK_MESSAGE_INFO_CONTROL_SYSTEM_STATE, MAVLINK_MESSAGE_INFO_BATTERY_STATUS, MAVLINK_MESSAGE_INFO_AUTOPILOT_VERSION, MAVLINK_MESSAGE_INFO_LANDING_TARGET, MAVLINK_MESSAGE_INFO_FENCE_STATUS, MAVLINK_MESSAGE_INFO_ESTIMATOR_STATUS, MAVLINK_MESSAGE_INFO_WIND_COV, MAVLINK_MESSAGE_INFO_GPS_INPUT, MAVLINK_MESSAGE_INFO_GPS_RTCM_DATA, MAVLINK_MESSAGE_INFO_HIGH_LATENCY, MAVLINK_MESSAGE_INFO_HIGH_LATENCY2, MAVLINK_MESSAGE_INFO_VIBRATION, MAVLINK_MESSAGE_INFO_HOME_POSITION, MAVLINK_MESSAGE_INFO_SET_HOME_POSITION, MAVLINK_MESSAGE_INFO_MESSAGE_INTERVAL, MAVLINK_MESSAGE_INFO_EXTENDED_SYS_STATE, MAVLINK_MESSAGE_INFO_ADSB_VEHICLE, MAVLINK_MESSAGE_INFO_COLLISION, MAVLINK_MESSAGE_INFO_V2_EXTENSION, MAVLINK_MESSAGE_INFO_MEMORY_VECT, MAVLINK_MESSAGE_INFO_DEBUG_VECT, MAVLINK_MESSAGE_INFO_NAMED_VALUE_FLOAT, MAVLINK_MESSAGE_INFO_NAMED_VALUE_INT, MAVLINK_MESSAGE_INFO_STATUSTEXT, MAVLINK_MESSAGE_INFO_DEBUG, MAVLINK_MESSAGE_INFO_SETUP_SIGNING, MAVLINK_MESSAGE_INFO_BUTTON_CHANGE, MAVLINK_MESSAGE_INFO_PLAY_TUNE, MAVLINK_MESSAGE_INFO_CAMERA_INFORMATION, MAVLINK_MESSAGE_INFO_CAMERA_SETTINGS, MAVLINK_MESSAGE_INFO_STORAGE_INFORMATION, MAVLINK_MESSAGE_INFO_CAMERA_CAPTURE_STATUS, MAVLINK_MESSAGE_INFO_CAMERA_IMAGE_CAPTURED, MAVLINK_MESSAGE_INFO_FLIGHT_INFORMATION, MAVLINK_MESSAGE_INFO_MOUNT_ORIENTATION, MAVLINK_MESSAGE_INFO_LOGGING_DATA, MAVLINK_MESSAGE_INFO_LOGGING_DATA_ACKED, MAVLINK_MESSAGE_INFO_LOGGING_ACK, MAVLINK_MESSAGE_INFO_VIDEO_STREAM_INFORMATION, MAVLINK_MESSAGE_INFO_VIDEO_STREAM_STATUS, MAVLINK_MESSAGE_INFO_WIFI_CONFIG_AP, MAVLINK_MESSAGE_INFO_PROTOCOL_VERSION, MAVLINK_MESSAGE_INFO_AIS_VESSEL, MAVLINK_MESSAGE_INFO_UAVCAN_NODE_STATUS, MAVLINK_MESSAGE_INFO_UAVCAN_NODE_INFO, MAVLINK_MESSAGE_INFO_PARAM_EXT_REQUEST_READ, MAVLINK_MESSAGE_INFO_PARAM_EXT_REQUEST_LIST, MAVLINK_MESSAGE_INFO_PARAM_EXT_VALUE, MAVLINK_MESSAGE_INFO_PARAM_EXT_SET, MAVLINK_MESSAGE_INFO_PARAM_EXT_ACK, MAVLINK_MESSAGE_INFO_OBSTACLE_DISTANCE, MAVLINK_MESSAGE_INFO_ODOMETRY, MAVLINK_MESSAGE_INFO_TRAJECTORY_REPRESENTATION_WAYPOINTS, MAVLINK_MESSAGE_INFO_TRAJECTORY_REPRESENTATION_BEZIER, MAVLINK_MESSAGE_INFO_CELLULAR_STATUS, MAVLINK_MESSAGE_INFO_ISBD_LINK_STATUS, MAVLINK_MESSAGE_INFO_UTM_GLOBAL_POSITION, MAVLINK_MESSAGE_INFO_DEBUG_FLOAT_ARRAY, MAVLINK_MESSAGE_INFO_ORBIT_EXECUTION_STATUS, MAVLINK_MESSAGE_INFO_STATUSTEXT_LONG, MAVLINK_MESSAGE_INFO_SMART_BATTERY_INFO, MAVLINK_MESSAGE_INFO_SMART_BATTERY_STATUS, MAVLINK_MESSAGE_INFO_ACTUATOR_OUTPUT_STATUS, MAVLINK_MESSAGE_INFO_TIME_ESTIMATE_TO_TARGET, MAVLINK_MESSAGE_INFO_TUNNEL, MAVLINK_MESSAGE_INFO_ONBOARD_COMPUTER_STATUS, MAVLINK_MESSAGE_INFO_COMPONENT_INFORMATION, MAVLINK_MESSAGE_INFO_PLAY_TUNE_V2, MAVLINK_MESSAGE_INFO_SUPPORTED_TUNES, MAVLINK_MESSAGE_INFO_WHEEL_DISTANCE, MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_BASIC_ID, MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_LOCATION, MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_AUTHENTICATION, MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_SELF_ID, MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_SYSTEM, MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_OPERATOR_ID, MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_MESSAGE_PACK}
# define MAVLINK_MESSAGE_NAMES {{ "ACTUATOR_CONTROL_TARGET", 140 }, { "ACTUATOR_OUTPUT_STATUS", 375 }, { "ADSB_VEHICLE", 246 }, { "AIS_VESSEL", 301 }, { "ALTITUDE", 141 }, { "ATTITUDE", 30 }, { "ATTITUDE_QUATERNION", 31 }, { "ATTITUDE_QUATERNION_COV", 61 }, { "ATTITUDE_TARGET", 83 }, { "ATT_POS_MOCAP", 138 }, { "AUTH_KEY", 7 }, { "AUTOPILOT_VERSION", 148 }, { "BATTERY_STATUS", 147 }, { "BUTTON_CHANGE", 257 }, { "CAMERA_CAPTURE_STATUS", 262 }, { "CAMERA_IMAGE_CAPTURED", 263 }, { "CAMERA_INFORMATION", 259 }, { "CAMERA_SETTINGS", 260 }, { "CAMERA_TRIGGER", 112 }, { "CELLULAR_STATUS", 334 }, { "CHANGE_OPERATOR_CONTROL", 5 }, { "CHANGE_OPERATOR_CONTROL_ACK", 6 }, { "COLLISION", 247 }, { "COMMAND_ACK", 77 }, { "COMMAND_INT", 75 }, { "COMMAND_LONG", 76 }, { "COMPONENT_INFORMATION", 395 }, { "CONTROL_SYSTEM_STATE", 146 }, { "DATA_STREAM", 67 }, { "DATA_TRANSMISSION_HANDSHAKE", 130 }, { "DEBUG", 254 }, { "DEBUG_FLOAT_ARRAY", 350 }, { "DEBUG_VECT", 250 }, { "DISTANCE_SENSOR", 132 }, { "ENCAPSULATED_DATA", 131 }, { "ESTIMATOR_STATUS", 230 }, { "EXTENDED_SYS_STATE", 245 }, { "FENCE_STATUS", 162 }, { "FILE_TRANSFER_PROTOCOL", 110 }, { "FLIGHT_INFORMATION", 264 }, { "FOLLOW_TARGET", 144 }, { "GLOBAL_POSITION_INT", 33 }, { "GLOBAL_POSITION_INT_COV", 63 }, { "GLOBAL_VISION_POSITION_ESTIMATE", 101 }, { "GPS2_RAW", 124 }, { "GPS2_RTK", 128 }, { "GPS_GLOBAL_ORIGIN", 49 }, { "GPS_INJECT_DATA", 123 }, { "GPS_INPUT", 232 }, { "GPS_RAW_INT", 24 }, { "GPS_RTCM_DATA", 233 }, { "GPS_RTK", 127 }, { "GPS_STATUS", 25 }, { "HEARTBEAT", 0 }, { "HIGHRES_IMU", 105 }, { "HIGH_LATENCY", 234 }, { "HIGH_LATENCY2", 235 }, { "HIL_ACTUATOR_CONTROLS", 93 }, { "HIL_CONTROLS", 91 }, { "HIL_GPS", 113 }, { "HIL_OPTICAL_FLOW", 114 }, { "HIL_RC_INPUTS_RAW", 92 }, { "HIL_SENSOR", 107 }, { "HIL_STATE", 90 }, { "HIL_STATE_QUATERNION", 115 }, { "HOME_POSITION", 242 }, { "ISBD_LINK_STATUS", 335 }, { "LANDING_TARGET", 149 }, { "LINK_NODE_STATUS", 8 }, { "LOCAL_POSITION_NED", 32 }, { "LOCAL_POSITION_NED_COV", 64 }, { "LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET", 89 }, { "LOGGING_ACK", 268 }, { "LOGGING_DATA", 266 }, { "LOGGING_DATA_ACKED", 267 }, { "LOG_DATA", 120 }, { "LOG_ENTRY", 118 }, { "LOG_ERASE", 121 }, { "LOG_REQUEST_DATA", 119 }, { "LOG_REQUEST_END", 122 }, { "LOG_REQUEST_LIST", 117 }, { "MANUAL_CONTROL", 69 }, { "MANUAL_SETPOINT", 81 }, { "MEMORY_VECT", 249 }, { "MESSAGE_INTERVAL", 244 }, { "MISSION_ACK", 47 }, { "MISSION_CHANGED", 52 }, { "MISSION_CLEAR_ALL", 45 }, { "MISSION_COUNT", 44 }, { "MISSION_CURRENT", 42 }, { "MISSION_ITEM", 39 }, { "MISSION_ITEM_INT", 73 }, { "MISSION_ITEM_REACHED", 46 }, { "MISSION_REQUEST", 40 }, { "MISSION_REQUEST_INT", 51 }, { "MISSION_REQUEST_LIST", 43 }, { "MISSION_REQUEST_PARTIAL_LIST", 37 }, { "MISSION_SET_CURRENT", 41 }, { "MISSION_WRITE_PARTIAL_LIST", 38 }, { "MOUNT_ORIENTATION", 265 }, { "NAMED_VALUE_FLOAT", 251 }, { "NAMED_VALUE_INT", 252 }, { "NAV_CONTROLLER_OUTPUT", 62 }, { "OBSTACLE_DISTANCE", 330 }, { "ODOMETRY", 331 }, { "ONBOARD_COMPUTER_STATUS", 390 }, { "OPEN_DRONE_ID_AUTHENTICATION", 12902 }, { "OPEN_DRONE_ID_BASIC_ID", 12900 }, { "OPEN_DRONE_ID_LOCATION", 12901 }, { "OPEN_DRONE_ID_MESSAGE_PACK", 12915 }, { "OPEN_DRONE_ID_OPERATOR_ID", 12905 }, { "OPEN_DRONE_ID_SELF_ID", 12903 }, { "OPEN_DRONE_ID_SYSTEM", 12904 }, { "OPTICAL_FLOW", 100 }, { "OPTICAL_FLOW_RAD", 106 }, { "ORBIT_EXECUTION_STATUS", 360 }, { "PARAM_EXT_ACK", 324 }, { "PARAM_EXT_REQUEST_LIST", 321 }, { "PARAM_EXT_REQUEST_READ", 320 }, { "PARAM_EXT_SET", 323 }, { "PARAM_EXT_VALUE", 322 }, { "PARAM_MAP_RC", 50 }, { "PARAM_REQUEST_LIST", 21 }, { "PARAM_REQUEST_READ", 20 }, { "PARAM_SET", 23 }, { "PARAM_VALUE", 22 }, { "PING", 4 }, { "PLAY_TUNE", 258 }, { "PLAY_TUNE_V2", 400 }, { "POSITION_TARGET_GLOBAL_INT", 87 }, { "POSITION_TARGET_LOCAL_NED", 85 }, { "POWER_STATUS", 125 }, { "PROTOCOL_VERSION", 300 }, { "RADIO_STATUS", 109 }, { "RAW_IMU", 27 }, { "RAW_PRESSURE", 28 }, { "RC_CHANNELS", 65 }, { "RC_CHANNELS_OVERRIDE", 70 }, { "RC_CHANNELS_RAW", 35 }, { "RC_CHANNELS_SCALED", 34 }, { "REQUEST_DATA_STREAM", 66 }, { "RESOURCE_REQUEST", 142 }, { "SAFETY_ALLOWED_AREA", 55 }, { "SAFETY_SET_ALLOWED_AREA", 54 }, { "SCALED_IMU", 26 }, { "SCALED_IMU2", 116 }, { "SCALED_IMU3", 129 }, { "SCALED_PRESSURE", 29 }, { "SCALED_PRESSURE2", 137 }, { "SCALED_PRESSURE3", 143 }, { "SERIAL_CONTROL", 126 }, { "SERVO_OUTPUT_RAW", 36 }, { "SETUP_SIGNING", 256 }, { "SET_ACTUATOR_CONTROL_TARGET", 139 }, { "SET_ATTITUDE_TARGET", 82 }, { "SET_GPS_GLOBAL_ORIGIN", 48 }, { "SET_HOME_POSITION", 243 }, { "SET_MODE", 11 }, { "SET_POSITION_TARGET_GLOBAL_INT", 86 }, { "SET_POSITION_TARGET_LOCAL_NED", 84 }, { "SIM_STATE", 108 }, { "SMART_BATTERY_INFO", 370 }, { "SMART_BATTERY_STATUS", 371 }, { "STATUSTEXT", 253 }, { "STATUSTEXT_LONG", 365 }, { "STORAGE_INFORMATION", 261 }, { "SUPPORTED_TUNES", 401 }, { "SYSTEM_TIME", 2 }, { "SYS_STATUS", 1 }, { "TERRAIN_CHECK", 135 }, { "TERRAIN_DATA", 134 }, { "TERRAIN_REPORT", 136 }, { "TERRAIN_REQUEST", 133 }, { "TIMESYNC", 111 }, { "TIME_ESTIMATE_TO_TARGET", 380 }, { "TRAJECTORY_REPRESENTATION_BEZIER", 333 }, { "TRAJECTORY_REPRESENTATION_WAYPOINTS", 332 }, { "TUNNEL", 385 }, { "UAVCAN_NODE_INFO", 311 }, { "UAVCAN_NODE_STATUS", 310 }, { "UTM_GLOBAL_POSITION", 340 }, { "V2_EXTENSION", 248 }, { "VFR_HUD", 74 }, { "VIBRATION", 241 }, { "VICON_POSITION_ESTIMATE", 104 }, { "VIDEO_STREAM_INFORMATION", 269 }, { "VIDEO_STREAM_STATUS", 270 }, { "VISION_POSITION_ESTIMATE", 102 }, { "VISION_SPEED_ESTIMATE", 103 }, { "WHEEL_DISTANCE", 9000 }, { "WIFI_CONFIG_AP", 299 }, { "WIND_COV", 231 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_COMMON_H
