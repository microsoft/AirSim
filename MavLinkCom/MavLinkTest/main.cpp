// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// PX4.cpp : Defines the entry point for the console application.

#include "Utils.hpp"
#include "MavLinkConnection.hpp"
#include "MavLinkVehicle.hpp"
#include "MavLinkMessages.hpp"
#include "MavLinkLog.hpp"
#include "Commands.h"
#include <iostream>
#include <vector>
#include <string.h>
#include <functional>
#include <map>
#include <ctime>
#include "UnitTests.h"
STRICT_MODE_OFF
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
STRICT_MODE_ON
/* enable math defines on Windows */

#define M_PI_2     1.57079632679489661923   // pi/2

static const int pixhawkVendorId = 9900;   ///< Vendor ID for Pixhawk board (V2 and V1) and PX4 Flow
static const int pixhawkFMUV4ProductId = 18;     ///< Product ID for Pixhawk V2 board
static const int pixhawkFMUV2ProductId = 17;     ///< Product ID for Pixhawk V2 board
static const int pixhawkFMUV2OldBootloaderProductId = 22;     ///< Product ID for Bootloader on older Pixhawk V2 boards
static const int pixhawkFMUV1ProductId = 16;     ///< Product ID for PX4 FMU V1 board

#define MAV_AUTOPILOT_ENUM_END (static_cast<uint8_t>(MAV_AUTOPILOT::MAV_AUTOPILOT_ASLUAV)+1)
#define MAV_TYPE_ENUM_END (static_cast<uint8_t>(MAV_TYPE::MAV_TYPE_ADSB)+1)
#define MAV_STATE_ENUM_END (static_cast<uint8_t>(MAV_STATE::MAV_STATE_POWEROFF)+1)

typedef common_utils::Utils Utils;
using namespace mavlinkcom;

struct FlagName {
  public:
    int Flag;
    const char* Name;
};

class PortAddress {
  public:
    std::string addr;
    int port;
};

#ifdef _WIN32
#include <Windows.h>
void DebugOutput(const char* message, ...) {
    va_list args;
    va_start(args, message);

    char* buffer = new char[8000];
    vsprintf(buffer, message, args);
    OutputDebugStringA(buffer);
    OutputDebugStringA("\n");
    fflush(stdout);

    va_end(args);
    delete buffer;
}
#else
// how do you write to the debug output windows on Unix ?
void DebugOutput(const char* message, ...) {
    va_list args;
    va_start(args, message);

    char* buffer = new char[8000];
    vsprintf(buffer, message, args);
    fflush(stdout);

    va_end(args);
    delete buffer;
}
#endif

const int LocalSystemId = 166;
const int LocalLogViewerSystemId = 167;
const int LocalComponentId = 1;

#define DEFAULT_BUFLEN 512
std::string defaultLocalAddress { "127.0.0.1" };

// The remote app is connected to Pixhawk, and is also "serving" UDP packets, this tells us what remote
// connection to create to talke to that server.
bool offboard = false;
PortAddress offboardEndPoint;
#define DEFAULT_OFFBOARD_PORT 14560

// this is used if you want to connect MavLinkTest to the serial port of the Pixhawk directly
bool serial = false;
std::string comPort;
int baudRate = 115200;

// server mode is when you want another app to connect to Pixhawk and publish data back to this process.
// this server will be listening for UDP packets, this is mutually exclusive with 'offboard' as this
// server will become the primary "droneConnection".  For example, jMAVSim can talk to this server
// using their the -qgc option.
bool server = false;
PortAddress serverEndPoint;
#define DEFAULT_SERVER_PORT 14550

bool connectLogViewer = false;
PortAddress logViewerEndPoint;
#define DEFAULT_LOGVIEWER_PORT 14570

// These are used to echo the mavlink messages to other 3rd party apps like QGC or LogViewer.
std::vector<PortAddress> proxyEndPoints;
#define DEFAULT_PROXY_PORT 14580

// this switch controls whether we turn off the RC remote active link loss detection
// if you do not have radio connected this is needed to stop "failsafe" override in pixhawk
// from kicking in when you try and fly.
bool noRadio = false;
bool unitTest = false;
bool verbose = false;
bool nsh = false;
bool noparams = false;
std::string logDirectory;
std::shared_ptr<MavLinkLog> inLogFile;
std::shared_ptr<MavLinkLog> outLogFile;

void OpenLogFiles() {
    if (logDirectory.size() > 0) {
        std::time_t result = std::time(nullptr);
        auto local = std::localtime(&result);

        auto path = boost::filesystem::system_complete(logDirectory);
        if (!boost::filesystem::is_directory(path)) {
            if (!boost::filesystem::create_directory(path)) {
                throw std::runtime_error(Utils::stringf("Failed to create log file directory '%s'.", path.generic_string().c_str()));
            }
        }

        path.append("logs", boost::filesystem::path::codecvt());
        if (!boost::filesystem::is_directory(path)) {
            if (!boost::filesystem::create_directory(path)) {
                throw std::runtime_error(Utils::stringf("Failed to create log file directory '%s'.", path.generic_string().c_str()));
            }
        }

        std::string today = Utils::stringf("%04d-%02d-%02d", local->tm_year + 1900, local->tm_mon + 1, local->tm_mday);
        path.append(today, boost::filesystem::path::codecvt());
        if (!boost::filesystem::is_directory(path)) {
            if (!boost::filesystem::create_directory(path)) {
                throw std::runtime_error(Utils::stringf("Failed to create log file directory '%s'.", path.generic_string().c_str()));
            }
        }

        std::string input = Utils::stringf("%02d-%02d-%02d-input.mavlink", local->tm_hour, local->tm_min, local->tm_sec);
        auto infile = boost::filesystem::system_complete(path);
        infile.append(input, boost::filesystem::path::codecvt());
        inLogFile = std::make_shared<MavLinkLog>();
        inLogFile->openForWriting(infile.generic_string());

        std::string output = Utils::stringf("%02d-%02d-%02d-output.mavlink", local->tm_hour, local->tm_min, local->tm_sec);
        auto outfile = boost::filesystem::system_complete(path);
        outfile.append(output, boost::filesystem::path::codecvt());
        outLogFile = std::make_shared<MavLinkLog>();
        outLogFile->openForWriting(outfile.generic_string());

    }
}
void CloseLogFiles() {
    if (inLogFile != nullptr) {
        inLogFile->close();
        inLogFile = nullptr;
    }
    if (outLogFile != nullptr) {
        outLogFile->close();
        outLogFile = nullptr;
    }
}


const static FlagName MavSysSensorFlags[] = {
    { static_cast<int>(MAV_SYS_STATUS_SENSOR::MAV_SYS_STATUS_SENSOR_3D_GYRO), "MAV_SYS_STATUS_SENSOR_3D_GYRO - 0x01 3D gyro" },
    { static_cast<int>(MAV_SYS_STATUS_SENSOR::MAV_SYS_STATUS_SENSOR_3D_ACCEL), "MAV_SYS_STATUS_SENSOR_3D_ACCEL - 0x02 3D accelerometer" },
    { static_cast<int>(MAV_SYS_STATUS_SENSOR::MAV_SYS_STATUS_SENSOR_3D_MAG), "MAV_SYS_STATUS_SENSOR_3D_MAG - 0x04 3D magnetometer" },
    { static_cast<int>(MAV_SYS_STATUS_SENSOR::MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE), "MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE - 0x08 absolute pressure" },
    { static_cast<int>(MAV_SYS_STATUS_SENSOR::MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE), "MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE - 0x10 differential pressure" },
    { static_cast<int>(MAV_SYS_STATUS_SENSOR::MAV_SYS_STATUS_SENSOR_GPS), "MAV_SYS_STATUS_SENSOR_GPS - 0x20 GPS" },
    { static_cast<int>(MAV_SYS_STATUS_SENSOR::MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW), "MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW - 0x40 optical flow" },
    { static_cast<int>(MAV_SYS_STATUS_SENSOR::MAV_SYS_STATUS_SENSOR_VISION_POSITION), "MAV_SYS_STATUS_SENSOR_VISION_POSITION - 0x80 computer vision position" },
    { static_cast<int>(MAV_SYS_STATUS_SENSOR::MAV_SYS_STATUS_SENSOR_LASER_POSITION), "MAV_SYS_STATUS_SENSOR_LASER_POSITION - 0x100 laser based position" },
    { static_cast<int>(MAV_SYS_STATUS_SENSOR::MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH), "MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH - 0x200 external ground truth (Vicon or Leica)" },
    { static_cast<int>(MAV_SYS_STATUS_SENSOR::MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL), "MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL - 0x400 3D angular rate control" },
    { static_cast<int>(MAV_SYS_STATUS_SENSOR::MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION), "MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION - 0x800 attitude stabilization" },
    { static_cast<int>(MAV_SYS_STATUS_SENSOR::MAV_SYS_STATUS_SENSOR_YAW_POSITION), "MAV_SYS_STATUS_SENSOR_YAW_POSITION - 0x1000 yaw position" },
    { static_cast<int>(MAV_SYS_STATUS_SENSOR::MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL), "MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL - 0x2000 z/altitude control" },
    { static_cast<int>(MAV_SYS_STATUS_SENSOR::MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL), "MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL - 0x4000 x/y position control" },
    { static_cast<int>(MAV_SYS_STATUS_SENSOR::MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS), "MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS - 0x8000 motor outputs / control" },
    { static_cast<int>(MAV_SYS_STATUS_SENSOR::MAV_SYS_STATUS_SENSOR_RC_RECEIVER), "MAV_SYS_STATUS_SENSOR_RC_RECEIVER - 0x10000 rc receiver" },
    { static_cast<int>(MAV_SYS_STATUS_SENSOR::MAV_SYS_STATUS_SENSOR_3D_GYRO2), "MAV_SYS_STATUS_SENSOR_3D_GYRO2 - 0x20000 2nd 3D gyro" },
    { static_cast<int>(MAV_SYS_STATUS_SENSOR::MAV_SYS_STATUS_SENSOR_3D_ACCEL2), "MAV_SYS_STATUS_SENSOR_3D_ACCEL2 - 0x40000 2nd 3D accelerometer" },
    { static_cast<int>(MAV_SYS_STATUS_SENSOR::MAV_SYS_STATUS_SENSOR_3D_MAG2), "MAV_SYS_STATUS_SENSOR_3D_MAG2 - 0x80000 2nd 3D magnetometer" },
    { static_cast<int>(MAV_SYS_STATUS_SENSOR::MAV_SYS_STATUS_GEOFENCE), "MAV_SYS_STATUS_GEOFENCE - 0x100000 geofence" },
    { static_cast<int>(MAV_SYS_STATUS_SENSOR::MAV_SYS_STATUS_AHRS), "MAV_SYS_STATUS_AHRS - 0x200000 AHRS subsystem health" },
    { static_cast<int>(MAV_SYS_STATUS_SENSOR::MAV_SYS_STATUS_TERRAIN), "MAV_SYS_STATUS_TERRAIN - 0x400000 Terrain subsystem health" },
    { static_cast<int>(MAV_SYS_STATUS_SENSOR::MAV_SYS_STATUS_REVERSE_MOTOR), "MAV_SYS_STATUS_REVERSE_MOTOR - 0x800000 Motors are reversed" },
    { 0, NULL }
};

const static char* AutoPilotNames[] {
    "MAV_AUTOPILOT_GENERIC, Generic autopilot, full support for everything",
    "MAV_AUTOPILOT_RESERVED, Reserved for future use.",
    "MAV_AUTOPILOT_SLUGS, SLUGS autopilot, http://slugsuav.soe.ucsc.edu",
    "MAV_AUTOPILOT_ARDUPILOTMEGA, ArduPilotMega / ArduCopter, http://diydrones.com",
    "MAV_AUTOPILOT_OPENPILOT, OpenPilot, http://openpilot.org",
    "MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY, Generic autopilot only supporting simple waypoints",
    "MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY, Generic autopilot supporting waypoints and other simple navigation commands",
    "MAV_AUTOPILOT_GENERIC_MISSION_FULL, Generic autopilot supporting the full mission command set",
    "MAV_AUTOPILOT_INVALID,  No valid autopilot, e.g. a GCS or other MAVLink component",
    "MAV_AUTOPILOT_PPZ, PPZ UAV - http://nongnu.org/paparazzi",
    "MAV_AUTOPILOT_UDB, UAV Dev Board",
    "MAV_AUTOPILOT_FP, FlexiPilot",
    "MAV_AUTOPILOT_PX4, PX4 Autopilot - http://pixhawk.ethz.ch/px4/",
    "MAV_AUTOPILOT_SMACCMPILOT, SMACCMPilot - http://smaccmpilot.org",
    "MAV_AUTOPILOT_AUTOQUAD, AutoQuad -- http://autoquad.org",
    "MAV_AUTOPILOT_ARMAZILA,  Armazila -- http://armazila.com",
    "MAV_AUTOPILOT_AEROB, Aerob -- http://aerob.ru",
    "MAV_AUTOPILOT_ASLUAV, ASLUAV autopilot -- http://www.asl.ethz.ch"
};

const static char* MavTypeNames[] {
    "MAV_TYPE_GENERIC-  Generic micro air vehicle.",
    "MAV_TYPE_FIXED_WING-  Fixed wing aircraft.",
    "MAV_TYPE_QUADROTOR-  Quadrotor",
    "MAV_TYPE_COAXIAL-  Coaxial helicopter",
    "MAV_TYPE_HELICOPTER-  Normal helicopter with tail rotor.",
    "MAV_TYPE_ANTENNA_TRACKER-  Ground installation",
    "MAV_TYPE_GCS-  Operator control unit / ground control station",
    "MAV_TYPE_AIRSHIP-  Airship, controlled",
    "MAV_TYPE_FREE_BALLOON-  Free balloon, uncontrolled",
    "MAV_TYPE_ROCKET-  Rocket",
    "MAV_TYPE_GROUND_ROVER-  Ground rover",
    "MAV_TYPE_SURFACE_BOAT-  Surface vessel, boat, ship",
    "MAV_TYPE_SUBMARINE-  Submarine",
    "MAV_TYPE_HEXAROTOR-  Hexarotor",
    "MAV_TYPE_OCTOROTOR-  Octorotor",
    "MAV_TYPE_TRICOPTER-  Octorotor",
    "MAV_TYPE_FLAPPING_WING-  Flapping wing",
    "MAV_TYPE_KITE-  Flapping wing",
    "MAV_TYPE_ONBOARD_CONTROLLER-  Onboard companion controller",
    "MAV_TYPE_VTOL_DUOROTOR-  Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter.",
    "MAV_TYPE_VTOL_QUADROTOR-  Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter.",
    "MAV_TYPE_VTOL_TILTROTOR-  Tiltrotor VTOL",
    "MAV_TYPE_VTOL_RESERVED2-  VTOL reserved 2",
    "MAV_TYPE_VTOL_RESERVED3-  VTOL reserved 3",
    "MAV_TYPE_VTOL_RESERVED4-  VTOL reserved 4",
    "MAV_TYPE_VTOL_RESERVED5-  VTOL reserved 5",
    "MAV_TYPE_GIMBAL-  Onboard gimbal",
    "MAV_TYPE_ADSB-  Onboard ADSB peripheral"
};

const static FlagName ModeFlagNames[] {
    { static_cast<int>(MAV_MODE_FLAG::MAV_MODE_FLAG_TEST_ENABLED), "MAV_MODE_FLAG_TEST_ENABLED - system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations." },
    { static_cast<int>(MAV_MODE_FLAG::MAV_MODE_FLAG_AUTO_ENABLED), "MAV_MODE_FLAG_AUTO_ENABLED - autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation." },
    { static_cast<int>(MAV_MODE_FLAG::MAV_MODE_FLAG_GUIDED_ENABLED), "MAV_MODE_FLAG_GUIDED_ENABLED - guided mode enabled, system flies MISSIONs / mission items." },
    { static_cast<int>(MAV_MODE_FLAG::MAV_MODE_FLAG_STABILIZE_ENABLED), "MAV_MODE_FLAG_STABILIZE_ENABLED - system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around." },
    { static_cast<int>(MAV_MODE_FLAG::MAV_MODE_FLAG_HIL_ENABLED), "MAV_MODE_FLAG_HIL_ENABLED - hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational." },
    { static_cast<int>(MAV_MODE_FLAG::MAV_MODE_FLAG_MANUAL_INPUT_ENABLED), "MAV_MODE_FLAG_MANUAL_INPUT_ENABLED - remote control input is enabled." },
    { static_cast<int>(MAV_MODE_FLAG::MAV_MODE_FLAG_SAFETY_ARMED), "MAV_MODE_FLAG_SAFETY_ARMED - MAV safety set to armed. Motors are enabled / running / can start. Ready to fly." },
    { 0, NULL }
};

const static char* MavStateNames[] {
    "MAV_STATE_UNINIT - Uninitialized system, state is unknown.",
    "MAV_STATE_BOOT - System is booting up.",
    "MAV_STATE_CALIBRATING - System is calibrating and not flight-ready.",
    "MAV_STATE_STANDBY - System is grounded and on standby. It can be launched any time.",
    "MAV_STATE_ACTIVE - System is active and might be already airborne. Motors are engaged.",
    "MAV_STATE_CRITICAL - System is in a non-normal flight mode. It can however still navigate.",
    "MAV_STATE_EMERGENCY - System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down.",
    "MAV_STATE_POWEROFF - System just initialized its power-down sequence, will shut down now.",
    "MAV_STATE_ENUM_END - ",
};


enum PX4_CUSTOM_MAIN_MODE {
    PX4_CUSTOM_MAIN_MODE_MANUAL = 1,
    PX4_CUSTOM_MAIN_MODE_ALTCTL,
    PX4_CUSTOM_MAIN_MODE_POSCTL,
    PX4_CUSTOM_MAIN_MODE_AUTO,
    PX4_CUSTOM_MAIN_MODE_ACRO,
    PX4_CUSTOM_MAIN_MODE_OFFBOARD,
    PX4_CUSTOM_MAIN_MODE_STABILIZED,
    PX4_CUSTOM_MAIN_MODE_RATTITUDE
};

enum PX4_CUSTOM_SUB_MODE_AUTO {
    PX4_CUSTOM_SUB_MODE_AUTO_READY = 1,
    PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF,
    PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
    PX4_CUSTOM_SUB_MODE_AUTO_MISSION,
    PX4_CUSTOM_SUB_MODE_AUTO_RTL,
    PX4_CUSTOM_SUB_MODE_AUTO_LAND,
    PX4_CUSTOM_SUB_MODE_AUTO_RTGS,
    PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET
};

const static FlagName  CustomModeNames[] {
    { PX4_CUSTOM_MAIN_MODE_MANUAL,		     "PX4_CUSTOM_MAIN_MODE_MANUAL"},
    { PX4_CUSTOM_MAIN_MODE_ALTCTL,		     "PX4_CUSTOM_MAIN_MODE_ALTCTL"},
    { PX4_CUSTOM_MAIN_MODE_POSCTL,		     "PX4_CUSTOM_MAIN_MODE_POSCTL"},
    { PX4_CUSTOM_MAIN_MODE_AUTO,			 "PX4_CUSTOM_MAIN_MODE_AUTO"},
    { PX4_CUSTOM_MAIN_MODE_ACRO,			 "PX4_CUSTOM_MAIN_MODE_ACRO"},
    { PX4_CUSTOM_MAIN_MODE_OFFBOARD,		 "PX4_CUSTOM_MAIN_MODE_OFFBOARD"},
    { PX4_CUSTOM_MAIN_MODE_STABILIZED,	     "PX4_CUSTOM_MAIN_MODE_STABILIZED"},
    { PX4_CUSTOM_MAIN_MODE_RATTITUDE,		 "PX4_CUSTOM_MAIN_MODE_RATTITUDE"},
    { 0,		 nullptr }
};

const static FlagName CustomSubModeNames[] {
    { PX4_CUSTOM_SUB_MODE_AUTO_READY,			 "PX4_CUSTOM_SUB_MODE_AUTO_READY"},
    { PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF,		     "PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF"},
    { PX4_CUSTOM_SUB_MODE_AUTO_LOITER,		     "PX4_CUSTOM_SUB_MODE_AUTO_LOITER"},
    { PX4_CUSTOM_SUB_MODE_AUTO_MISSION,		     "PX4_CUSTOM_SUB_MODE_AUTO_MISSION"},
    { PX4_CUSTOM_SUB_MODE_AUTO_RTL,			     "PX4_CUSTOM_SUB_MODE_AUTO_RTL"},
    { PX4_CUSTOM_SUB_MODE_AUTO_LAND,			 "PX4_CUSTOM_SUB_MODE_AUTO_LAND"},
    { PX4_CUSTOM_SUB_MODE_AUTO_RTGS,			 "PX4_CUSTOM_SUB_MODE_AUTO_RTGS"},
    { PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET,	 "PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET"},
    { 0,		 nullptr }
};


void PrintFlags(const FlagName* flagNames, int value) {

    for (int i = 0; ; i++) {
        if (flagNames[i].Name == NULL)
            break;

        if ((value & flagNames[i].Flag) != 0) {
            printf("    %s\n", flagNames[i].Name);
        }
    }
}

void PrintEnum(const FlagName* enumNames, int value) {

    for (int i = 0; ; i++) {
        if (enumNames[i].Name == NULL)
            break;

        if (value == enumNames[i].Flag) {
            printf("    %s\n", enumNames[i].Name);
        }
    }
}

void PrintSystemStatus(MavLinkSysStatus& status) {
    printf("System status:\n");

    printf("    onboard sensors present:\n");
    PrintFlags(MavSysSensorFlags, status.onboard_control_sensors_present);

    printf("    onboard sensors enabled:\n");
    PrintFlags(MavSysSensorFlags, status.onboard_control_sensors_enabled);

    printf("    CPU load %d\n", static_cast<int>(status.load));

    printf("    battery voltage %d millivolts\n", static_cast<int>(status.voltage_battery));
    printf("    battery current %d milliamps\n", static_cast<int>(status.current_battery));
    printf("    drop_rate_comm = %d\n", static_cast<int>(status.drop_rate_comm));
    printf("    errors_comm = %d\n", static_cast<int>(status.errors_comm));
    printf("    battery_remaining = %d\n", static_cast<int>(status.battery_remaining));

}

void PrintCustomMode(const MavLinkHeartbeat& heartbeat) {
    if (heartbeat.autopilot == static_cast<uint8_t>(MAV_AUTOPILOT::MAV_AUTOPILOT_PX4)) {
        int custom = (heartbeat.custom_mode >> 16);
        int mode = (custom & 0xff);
        int submode = (custom >> 8);
        PrintEnum(CustomModeNames, mode);
        PrintEnum(CustomSubModeNames, submode);
    } else {
        Utils::logMessage("    Custom mode=%x", heartbeat.custom_mode);
    }
}

void PrintHeartbeat(const MavLinkMessage& msg) {

    MavLinkHeartbeat heartbeat;
    heartbeat.decode(msg);

    Utils::logMessage("Connected:");
    Utils::logMessage("    Version=%d", static_cast<int>(heartbeat.mavlink_version));

    if (heartbeat.type < MAV_TYPE_ENUM_END) {
        Utils::logMessage("    Type=%s", MavTypeNames[heartbeat.type]);
    }

    if (heartbeat.autopilot < MAV_AUTOPILOT_ENUM_END) {
        Utils::logMessage("    Autopilot=%s", AutoPilotNames[heartbeat.autopilot]);
    }

    if (heartbeat.system_status < MAV_STATE_ENUM_END) {
        Utils::logMessage("    State=%s", MavStateNames[heartbeat.system_status]);
    }

    Utils::logMessage("    Base mode:");
    PrintFlags(ModeFlagNames, heartbeat.base_mode);

    PrintCustomMode(heartbeat);

    Utils::logMessage("    VEHICLE SYSTEM ID: %i", msg.sysid);
    Utils::logMessage("    VEHICLE COMPONENT ID: %i", msg.compid);

}

uint32_t gCustom = 0;

void CheckHeartbeat(const MavLinkMessage& msg) {
    MavLinkHeartbeat heartbeat;
    heartbeat.decode(msg);

    if (gCustom != heartbeat.custom_mode) {
        gCustom = heartbeat.custom_mode;
        PrintCustomMode(heartbeat);
    }
}

void mavlink_quaternion_to_dcm(const float quaternion[4], float dcm[3][3]) {
    double a = quaternion[0];
    double b = quaternion[1];
    double c = quaternion[2];
    double d = quaternion[3];
    double aSq = a * a;
    double bSq = b * b;
    double cSq = c * c;
    double dSq = d * d;
    dcm[0][0] = aSq + bSq - cSq - dSq;
    dcm[0][1] = 2 * (b * c - a * d);
    dcm[0][2] = 2 * (a * c + b * d);
    dcm[1][0] = 2 * (b * c + a * d);
    dcm[1][1] = aSq - bSq + cSq - dSq;
    dcm[1][2] = 2 * (c * d - a * b);
    dcm[2][0] = 2 * (b * d - a * c);
    dcm[2][1] = 2 * (a * b + c * d);
    dcm[2][2] = aSq - bSq - cSq + dSq;
}
void mavlink_dcm_to_euler(const float dcm[3][3], float* roll, float* pitch, float* yaw) {
    float phi, theta, psi;
    theta = asin(-dcm[2][0]);

    if (fabsf(theta - static_cast<float>(M_PI_2)) < 1.0e-3f) {
        phi = 0.0f;
        psi = (atan2f(dcm[1][2] - dcm[0][1],
                      dcm[0][2] + dcm[1][1]) + phi);

    } else if (fabsf(theta + static_cast<float>(M_PI_2)) < 1.0e-3f) {
        phi = 0.0f;
        psi = atan2f(dcm[1][2] - dcm[0][1],
                     dcm[0][2] + dcm[1][1] - phi);

    } else {
        phi = atan2f(dcm[2][1], dcm[2][2]);
        psi = atan2f(dcm[1][0], dcm[0][0]);
    }

    *roll = phi;
    *pitch = theta;
    *yaw = psi;
}
void mavlink_quaternion_to_euler(const float quaternion[4], float* roll, float* pitch, float* yaw) {
    float dcm[3][3];
    mavlink_quaternion_to_dcm(quaternion, dcm);
    mavlink_dcm_to_euler(dcm, roll, pitch, yaw);
}

extern void mavlink_euler_to_quaternion(float roll, float pitch, float yaw, float quaternion[4]);

void PrintUsage() {
    printf("Usage: PX4 options\n");
    printf("Connects to PX4 either over udp or serial COM port\n");
    printf("Options: \n");
    printf("    -udp:ipaddr[:port]]                    - connect to remote drone via this udp address, the remote app is the UDP server (default port 14550)\n");
    printf("    -server:ipaddr[:port]                  - start mavlink server on this local port (so jMAVSim can connect to it using -udp)\n");
    printf("    -serial[:comPortName][, baudrate]]     - open serial port\n");
    printf("    -logviewer:ipaddr[:port]               - for sending mavlink information to Log Viewer\n");
    printf("    -proxy:ipaddr[:port]                   - send all mavlink messages to and from remote node\n");
    printf("    -local:ipaddr                          - specify local NIC address (default 127.0.0.1)\n");
    printf("    -logdir:filename                       - specify local directory where mavlink logs are stored (default is no log files)\n");
    printf("    -noradio							   - disables RC link loss failsafe\n");
    printf("    -nsh                                   - enter NuttX shell immediately on connecting with PX4\n");
    printf("If no arguments it will find a COM port matching the name 'PX4'\n");
    printf("You can specify -proxy multiple times with different port numbers to proxy drone messages out to multiple listeners\n");
}

bool ParseCommandLine(int argc, const char* argv[]) {
    const char* logDirOption = "logdir";
    const char* outLogFileOption = "outlogfile";

    // parse command line
    for (int i = 1; i < argc; i++) {
        const char* arg = argv[i];
        if (arg[0] == '-' || arg[0] == '/') {
            std::string option(arg + 1);
            std::vector<std::string> parts;
            boost::algorithm::split(parts, option, boost::is_any_of(":,"));
            std::string lower = boost::algorithm::to_lower_copy(parts[0]);
            if (lower == "udp") {
                offboard = true;
                offboardEndPoint.port = DEFAULT_OFFBOARD_PORT;
                if (parts.size() > 1) {
                    offboardEndPoint.addr = parts[1];
                    if (parts.size() > 2) {
                        offboardEndPoint.port = atoi(parts[2].c_str());
                    }
                }
            } else if (lower == "server") {
                server = true;
                serverEndPoint.port = DEFAULT_SERVER_PORT;
                if (parts.size() > 1) {
                    serverEndPoint.addr = parts[1];
                    if (parts.size() > 2) {
                        serverEndPoint.port = atoi(parts[2].c_str());
                    }
                }
            } else if (lower == "proxy") {
                PortAddress ep;
                ep.port = DEFAULT_PROXY_PORT;

                if (parts.size() > 1) {
                    ep.addr = parts[1];
                    if (parts.size() > 2) {
                        ep.port = atoi(parts[2].c_str());
                    }
                }
                proxyEndPoints.push_back(ep);
            } else if (lower == "logviewer") {
                connectLogViewer = true;
                logViewerEndPoint.port = DEFAULT_LOGVIEWER_PORT;
                if (parts.size() > 1) {
                    logViewerEndPoint.addr = parts[1];
                    if (parts.size() > 2) {
                        logViewerEndPoint.port = atoi(parts[2].c_str());
                    }
                }
            } else if (lower == logDirOption) {
                if (parts.size() > 1) {
                    std::string fileName(arg + 1 + strlen(logDirOption) + 1);
                    logDirectory = fileName;
                }
            } else if (lower == "local") {
                if (parts.size() > 1) {
                    defaultLocalAddress = parts[1];
                }
            } else if (lower == "serial") {
                serial = true;
                if (parts.size() > 1) {
                    comPort = parts[1];
                    if (parts.size() > 2) {
                        baudRate = atoi(parts[2].c_str());
                        if (baudRate == 0) {
                            printf("### Error: invalid baud rate in -serial argument\n");
                            return false;
                        }
                    }
                }
            } else if (lower == "h" || lower == "?" || lower == "help" || lower == "-help") {
                return false;
            } else if (lower == "noradio") {
                noRadio = true;
            } else if (lower == "test") {
                unitTest = true;
            } else if (lower == "verbose") {
                verbose = true;
            } else if (lower == "nsh") {
                nsh = true;
            } else if (lower == "noparams") {
                noparams = true;
            } else {
                printf("### Error: unexpected argument: %s\n", arg);
                return false;
            }
        } else {
            printf("### Error: unexpected argument: %s\n", arg);
            return false;
        }
    }
    return true;
}


std::vector<std::string> parseArgs(std::string s) {
    auto start = s.begin();
    std::vector<std::string> result;
    auto theEnd = s.end();
    auto it = s.begin();
    while (it != theEnd) {
        char ch = *it;
        if (ch == ' ' || ch == '\t' || ch == ',') {
            if (start < it) {
                result.push_back(std::string(start, it));
            }
            it++;
            start = it;
        } else if (*it == '"') {
            // treat literals as one word
            it++;
            start = it;
            while (*it != '"' && it != theEnd) {
                it++;
            }
            auto end = it;
            if (start < it) {
                result.push_back(std::string(start, end));
            }
            if (*it == '"') {
                it++;
            }
            start = it;
        } else {
            it++;
        }
    }
    if (start < theEnd) {
        result.push_back(std::string(start, s.end()));
    }
    return result;
}

void HexDump(uint8_t *buffer, int len) {
    for (int i = 0; i < len; i += 16) {

        int j = 0;
        for (j = i; j < i + 16 && j < len; j++) {
            uint8_t b = buffer[i + j];
            printf("%02x ", b);
        }
        while (j < 16) {
            printf("   ");
        }
        for (j = i; j < i + 16 && j < len; j++) {
            uint8_t b = buffer[j];
            if (b < 0x20 || b == 0x7f || b == 0xfe) {
                b = '.';
            }
            printf("%c", b);
        }
        while (j < 16) {
            printf(" ");
        }
        printf("\n");
    }
}

std::shared_ptr<MavLinkConnection> connectProxy(std::shared_ptr<MavLinkConnection> droneConnection, const PortAddress& endPoint, std::string name) {
    printf("Connecting to UDP Proxy address %s:%d\n", endPoint.addr.c_str(), endPoint.port);

    std::shared_ptr<MavLinkConnection> proxyConnection = MavLinkConnection::connectRemoteUdp(name, defaultLocalAddress, endPoint.addr, endPoint.port);

    // forward all PX4 messages to the remote proxy and all messages from remote proxy to PX4.
    droneConnection->join(proxyConnection);

    if (verbose) {
        proxyConnection->subscribe([=](std::shared_ptr<MavLinkConnection> con, const MavLinkMessage& msg) {
            printf("Received msg %d from proxy\n", static_cast<int>(msg.msgid));
        });
    }

    return proxyConnection;
}

std::string findPixhawk() {

    auto result = MavLinkConnection::findSerialPorts(0, 0);
    for (auto iter = result.begin(); iter != result.end(); iter++) {
        SerialPortInfo info = *iter;
        if (info.vid == pixhawkVendorId) {
            if (info.pid == pixhawkFMUV4ProductId || info.pid == pixhawkFMUV2ProductId || info.pid == pixhawkFMUV2OldBootloaderProductId) {
                printf("Auto Selecting COM port: %S\n", info.displayName.c_str());
                return std::string(info.portName.begin(), info.portName.end());
            }
        }
    }
    return "";
}

std::shared_ptr<MavLinkConnection> connectSerial() {
    std::string name = comPort;
    printf("Connecting to serial port %s, baudrate=%d\n", name.c_str(), baudRate);
    return MavLinkConnection::connectSerial("drone", name, baudRate, "sh /etc/init.d/rc.usb\n");
}


std::shared_ptr<MavLinkConnection> connectOffboard() {
    if (offboardEndPoint.addr == "") {
        offboardEndPoint.addr = defaultLocalAddress;
    }
    printf("Connecting to offboard drone at address %s:%d\n", offboardEndPoint.addr.c_str(), offboardEndPoint.port);
    return MavLinkConnection::connectRemoteUdp("drone", defaultLocalAddress, offboardEndPoint.addr, offboardEndPoint.port);
}

std::shared_ptr<MavLinkConnection> connectServer(const PortAddress& endPoint, std::string name) {
    printf("Connecting to UDP Server address %s:%d\n", endPoint.addr.c_str(), endPoint.port);

    std::shared_ptr<MavLinkConnection> serverConnection = MavLinkConnection::connectLocalUdp(name, endPoint.addr, endPoint.port);

    return serverConnection;
}

std::shared_ptr<MavLinkConnection> droneConnection;
std::shared_ptr<MavLinkConnection> logConnection;

bool connect(std::shared_ptr<MavLinkVehicle> mavLinkVehicle) {
    if (offboard && serial) {
        printf("Cannot connect to local serial pixhawk and -offboard drone at the same time \n");
        return false;
    }
    if (!offboard && !serial && !server) {
        printf("Must specify one of -serial, -offboard or -server otherwise we don't have a drone connection\n");
        return false;
    }
    if (offboard && server) {
        printf("Cannot have offboard and server, must pick one of this as the primary drone connection\n");
        return false;
    }

    std::vector<PortAddress> usedPorts;

    if (serial) {
        droneConnection = connectSerial();
    } else if (offboard) {
        droneConnection = connectOffboard();
        usedPorts.push_back(offboardEndPoint);
    }

    if (server) {
        if (serverEndPoint.addr == "") {
            serverEndPoint.addr = defaultLocalAddress;
        }

        std::shared_ptr<MavLinkConnection> serverConnection = connectServer(serverEndPoint, "server");
        usedPorts.push_back(serverEndPoint);

        if (droneConnection != nullptr) {
            // then we have a serial connection as the primary droneConnection, so publish all PX4 messages out to the server
            droneConnection->join(serverConnection);
        } else {
            // no local serial connection, so this is the primary droneConnection.
            droneConnection = serverConnection;
        }
    }

    if (droneConnection == nullptr) {
        // failed to connect
        return false;
    }

    if (outLogFile != nullptr) {
        droneConnection->startLoggingSendMessage(outLogFile);
    }
    mavLinkVehicle->connect(droneConnection);

    if (serial) {
        // local connection, then we own sending the heartbeat.
        mavLinkVehicle->startHeartbeat();
    }

    if (connectLogViewer) {
        if (logViewerEndPoint.addr == "") {
            logViewerEndPoint.addr = defaultLocalAddress;
        }
        logConnection = connectProxy(droneConnection, logViewerEndPoint, "log");
        usedPorts.push_back(logViewerEndPoint);
    } else {
        logConnection = nullptr;
    }

    for (auto ptr = proxyEndPoints.begin(), end = proxyEndPoints.end(); ptr != end; ptr++) {
        PortAddress proxyEndPoint = *ptr;
        for (auto ep = usedPorts.begin(), endep = usedPorts.end(); ep != endep; ep++) {
            PortAddress used = *ep;
            if (used.addr == proxyEndPoint.addr && used.port == proxyEndPoint.port) {
                printf("Cannot proxy to address that is already used: %s:%d\n", used.addr.c_str(), used.port);
                return false;
            }
        }
        usedPorts.push_back(proxyEndPoint);
        connectProxy(droneConnection, proxyEndPoint, "proxy");
    }


    return true;
}

void checkPulse(std::shared_ptr<MavLinkVehicle> mavLinkVehicle) {
    MavLinkHeartbeat heartbeat;
    if (!mavLinkVehicle->waitForHeartbeat().wait(100000, &heartbeat)) {
        throw std::runtime_error("Received no heartbeat from PX4 after 100 seconds");
    }
}

int console() {

    std::string line;
    std::shared_ptr<MavLinkVehicle> mavLinkVehicle = std::make_shared<MavLinkVehicle>(LocalSystemId, LocalComponentId);
    std::shared_ptr<MavLinkNode> logViewer = nullptr;
    Command* currentCommand = nullptr;
    std::stringstream initScript;
    OrbitCommand* orbit = new OrbitCommand();
    SendImageCommand* sendImage = nullptr;
    NshCommand* nshCommand = new NshCommand();

    std::vector<Command*> cmdTable;
    cmdTable.push_back(new ArmDisarmCommand());
    cmdTable.push_back(new TakeOffCommand());
    cmdTable.push_back(new LandCommand());
    cmdTable.push_back(new MissionCommand());
    cmdTable.push_back(new LoiterCommand());
    cmdTable.push_back(new CapabilitiesCommand());
    cmdTable.push_back(new RtlCommand());
    cmdTable.push_back(new GetParamsCommand());
    cmdTable.push_back(new GetSetParamCommand());
    cmdTable.push_back(new StatusCommand());
    cmdTable.push_back(new PositionCommand());
    cmdTable.push_back(new RequestImageCommand());
    cmdTable.push_back(new FtpCommand());
    cmdTable.push_back(nshCommand);
    cmdTable.push_back(new AltHoldCommand());
    cmdTable.push_back(sendImage = new SendImageCommand());

    if (!connect(mavLinkVehicle)) {
        return 1;
    }

    droneConnection->subscribe([=](std::shared_ptr<MavLinkConnection> connection, const MavLinkMessage& message) {
        if (inLogFile != nullptr && inLogFile->isOpen()) {
            inLogFile->write(message);
        }
        if (message.msgid == MavLinkHeartbeat::kMessageId) {
            CheckHeartbeat(message);
        } else if (message.msgid == MavLinkAttitudeTarget::kMessageId) {
            MavLinkAttitudeTarget target;
            target.decode(message);

            float pitch, roll, yaw;
            mavlink_quaternion_to_euler(target.q, &roll, &pitch, &yaw);
            /*
            float q2[4];
            mavlink_euler_to_quaternion(roll, pitch, yaw, q2);*/

            //DebugOutput("q1 : %f\t%f\t%f\t%g", target.q[0], target.q[1], target.q[2], target.q[3]);
            //DebugOutput("q2 : %f\t%f\t%f\t%g", q2[0], q2[1], q2[2], q2[3]);
            //DebugOutput("target roll: %f\tpitch: %f\tyaw:%f\tthrust: %f", roll, pitch, yaw, target.thrust);

        }
    });

    if (logConnection != nullptr) {
        logViewer = std::make_shared<MavLinkNode>(LocalLogViewerSystemId, LocalComponentId);
        logViewer->connect(logConnection);
        orbit->setLogViewer(logViewer);
        sendImage->setLogViewer(logViewer);
    }

    checkPulse(mavLinkVehicle);

    int retries = 0;
    while (retries++ < 5) {
        try {
            if (mavLinkVehicle->isLocalControlSupported()) {
                cmdTable.push_back(new GotoCommand());
                cmdTable.push_back(new RotateCommand());
                cmdTable.push_back(orbit);
                cmdTable.push_back(new WiggleCommand());
                cmdTable.push_back(new IdleCommand());
            }
            break;
        } catch (std::runtime_error e) {
            printf("%s\n", e.what());
        }
    }

    if (noRadio) {
        MavLinkParameter p = mavLinkVehicle->getCachedParameter("NAV_RCL_ACT");
        if (p.value != 0) {
            p.value = 0;
            mavLinkVehicle->setParameter(p);
        }
    }

    if (nsh) {
        currentCommand = nshCommand;
        currentCommand->Execute(mavLinkVehicle);
    } else {
        if (!noparams) {
            printf("Downloading drone parameters so we know how to control it properly...\n");
            mavLinkVehicle->getParamList();
        }
        mavLinkVehicle->setStabilizedFlightMode();
    }


    printf("Ready...\n");
    initScript  << "status\n";

    while (!std::cin.eof()) {

        if (!initScript.eof()) {
            std::getline(initScript, line);
        } else {
            std::getline(std::cin, line);
        }

        line = common_utils::Utils::trim(line, ' ');

        if (line.length() > 0) {
            if (line.length() == 0) {
                continue;
            }
            std::vector<std::string> args = parseArgs(line);
            std::string cmd = args[0];

            if (cmd == "x") {
                break;
            } else if (cmd == "disconnect") {
                mavLinkVehicle->close();
            } else if (cmd == "connect") {
                connect(mavLinkVehicle);
            } else if (cmd == "?" || cmd == "help") {
                for (size_t i = 0; i < cmdTable.size(); i++) {
                    Command* command = cmdTable[i];
                    if (args.size() > 1 && args[1] == command->Name) {
                        command->PrintHelp();
                        break;
                    } else {
                        printf("%s\n", command->Name.c_str());
                    }

                }
            } else {
                Command* selected = nullptr;
                for (size_t i = 0; i < cmdTable.size(); i++) {
                    Command* command = cmdTable[i];
                    if (command->Parse(args)) {
                        // found it!
                        selected = command;
                        break;
                    }
                }

                if (currentCommand != nullptr && currentCommand != selected) {
                    // close previous command.
                    currentCommand->Close();
                }
                currentCommand = selected;

                if (currentCommand != NULL) {
                    try {
                        currentCommand->Execute(mavLinkVehicle);
                    } catch (const std::exception& e) {
                        printf("Error: %s\n", e.what());
                    }
                } else {
                    printf("Unknown command.  Type '?' to get list of commands\n");
                }

            }

        }

    }

    logViewer = nullptr;
    droneConnection = nullptr;
    logConnection = nullptr;

    CloseLogFiles();
    return 0;
}

void completion(int state) {

}

int main(int argc, const char* argv[]) {
    if (!ParseCommandLine(argc, argv)) {
        PrintUsage();
        return 1;
    }

    OpenLogFiles();

    if (serial) {
        if (comPort.size() == 0 || comPort == "*") {
            comPort = findPixhawk();
            if (comPort == "") {
                printf("### Error: PX4 not found on your SerialPort, or it is not available");
                return 1;
            }
        }
    }


    if (unitTest) {
        UnitTests test;
        test.RunAll(comPort, baudRate);
        return 0;
    }

    try {
        return console();
    } catch (const std::exception& e) {
        printf("Exception: %s\n", e.what());
        return 1;
    }

    CloseLogFiles();
}

