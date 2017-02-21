// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//if using Unreal Build system then include precompiled header file first
#ifdef AIRLIB_PCH
#include "AirSim.h"
#endif

#include "control/MavLinkDroneControl.hpp"
#include "common/common_utils/Utils.hpp"
#include "common/Common.hpp"
#include <chrono>

using namespace mavlinkcom;
using namespace common_utils;

namespace msr { namespace airlib {

MavLinkDroneControl::MavLinkDroneControl(const Parameters& params)
    : MavLinkDroneControl(params_.system_id, params_.component_id, createConnection(params))
{
}

MavLinkDroneControl::MavLinkDroneControl(int local_system_id, int local_component_id, std::shared_ptr<MavLinkConnection> connection)
    : state_version(0)
{
    //For debugging, disable buffering
    //setbuf(stdout, NULL);

    is_offboard_mode_ = false;

    if (connection == nullptr)
        throw std::invalid_argument("MavLinkDroneControl was supplied nullptr connection");

    // create MavLinkVehicle that uses this serial connection to talk to the PX4.
    drone_ = std::make_shared<MavLinkVehicle>(params_.system_id, params_.component_id);
    drone_->connect(connection);
}

void MavLinkDroneControl::reportTelemetry(long renderTime)
{
	MavLinkTelemetry telemetry;
	auto con = drone_->getConnection();
	if (con != nullptr) {
		con->getTelemetry(telemetry);
		telemetry.renderTime = renderTime;				
		drone_->sendMessage(telemetry);
	}
}

shared_ptr<mavlinkcom::MavLinkConnection> MavLinkDroneControl::createConnection(const Parameters& params)
{
    std::shared_ptr<MavLinkConnection> connection;

    if (params.connectionType == "udp")
    {
        if (params.udpAddress == "") {
            throw std::invalid_argument("drone_udp parameter is not set, or has an invalid value.  Please run 'rosparam' to set '/AirNode/drone_udp' to the local UDP address to start listening on for the mavlink messages, for example: '127.0.0.1' or '192.168.1.64'.\n");
        }

        if (params.udpPort == 0) {
            throw std::invalid_argument("drone_udp_port parameter is not set, or has an invalid value.  Please run 'rosparam' to set '/AirNode/drone_udp_port' to to the local UDP port start listening on for mavlink messages, for example: 14550.\n");
        }

        connection = MavLinkConnection::connectLocalUdp( "drone", params.udpAddress, params.udpPort);
    }
    else if (params.connectionType == "serial")
    {
        if (params.serialDevice == "") {
            throw std::invalid_argument("drone_serial parameter is not set, or has an invalid value.  Please run 'rosparam' to set '/AirNode/drone_serial' to the name of the serial device, for example '/dev/ttyACM0'.  Note you can ls -l /dev/serial/by-id to find which device your PX4 is showing up as.\n");
        }

        if (params.baudRate == 0) {
            throw std::invalid_argument("drone_params.baudRate parameter is not set, or has an invalid value.  Please run 'rosparam' to set '/AirNode/drone_params.baudRate' to to the correct value, for example: 115200 or 57600\n");
        }

        connection = MavLinkConnection::connectSerial("drone", params.serialDevice.c_str(), params.baudRate, "sh /etc/init.d/rc.usb\n");
    }
    else
    {
        throw std::invalid_argument("drone_connection parameter is not set, or has an invalid value.  Please run 'rosparam' to set '/AirNode/drone_connection' to 'serial' or 'udp'\n");
    }

    return connection;
}

void MavLinkDroneControl::updateState()
{
    StatusLock(this);
    int version = drone_->getVehicleStateVersion();
    if (version != state_version)
    {
        current_state = drone_->getVehicleState();
        state_version = version;
    }
}
//sensors
Vector3r MavLinkDroneControl::getPosition()
{
    updateState();
    return Vector3r(current_state.local_est.pos.x, current_state.local_est.pos.y, current_state.local_est.pos.z);
}

Vector3r MavLinkDroneControl::getVelocity()
{
    updateState();
    return Vector3r(current_state.local_est.vel.vx, current_state.local_est.vel.vy, current_state.local_est.vel.vz);
}

GeoPoint MavLinkDroneControl::getHomePoint()
{
    updateState();
    if (current_state.home.is_set)
        return GeoPoint(current_state.home.global_pos.lat, current_state.home.global_pos.lon, current_state.home.global_pos.alt);
    else
        return GeoPoint(Utils::nan<double>(), Utils::nan<double>(), Utils::nan<float>());
}

GeoPoint MavLinkDroneControl::getGpsLocation()
{
    updateState();
    return GeoPoint(current_state.global_est.pos.lat, current_state.global_est.pos.lon, current_state.global_est.pos.alt);
}

Quaternionr MavLinkDroneControl::getOrientation()
{
    updateState();
    return VectorMath::toQuaternion(current_state.attitude.pitch, current_state.attitude.roll, current_state.attitude.yaw);
}

double MavLinkDroneControl::timestampNow()
{
    return static_cast<double>(drone_->getTimeStamp());
}

//administrative

bool MavLinkDroneControl::armDisarm(bool arm, CancelableActionBase& cancelable_action)
{
    bool rc = false;
    drone_->armDisarm(arm).wait(10000, &rc);
    return rc;
}

bool MavLinkDroneControl::isOffboardMode()
{
    return is_offboard_mode_;
}

bool MavLinkDroneControl::requestControl(CancelableActionBase& cancelable_action)
{
    drone_->requestControl();
    is_offboard_mode_ = true;
    return true;
}
bool MavLinkDroneControl::releaseControl(CancelableActionBase& cancelable_action)
{
    drone_->releaseControl();
    is_offboard_mode_ = false;
    return true;
}
bool MavLinkDroneControl::takeoff(float max_wait_seconds, CancelableActionBase& cancelable_action)
{
    bool rc = false;
    if (!drone_->takeoff(getTakeoffZ(), 0.0f, 0.0f).wait(static_cast<int>(max_wait_seconds * 1000), &rc))
    {
        throw MoveException("TakeOff command - timeout waiting for response");
    }
    if (!rc) {
        throw MoveException("TakeOff command rejected by drone");
    }

    bool success = waitForZ(max_wait_seconds, getTakeoffZ(), getDistanceAccuracy(), cancelable_action);
    return success;
}

bool MavLinkDroneControl::hover(CancelableActionBase& cancelable_action)
{
    bool rc = false;
    AsyncResult<bool> result = drone_->loiter();
    auto start_time = std::chrono::system_clock::now();
    while (!cancelable_action.isCancelled())
    {
        if (result.wait(100, &rc))
        {
            break;
        }
    }
    return rc;
}

bool MavLinkDroneControl::land(CancelableActionBase& cancelable_action)
{
    // bugbug: really need a downward pointing distance to ground sensor to do this properly, for now
    // we assume the ground is relatively flat an we are landing roughly at the home altitude.
    updateState();
    if (current_state.home.is_set)
    {
        bool rc = false;
        if (!drone_->land(current_state.global_est.pos.lat, current_state.global_est.pos.lon, current_state.home.global_pos.alt).wait(10000, &rc))
        {
            throw MoveException("Landing command - timeout waiting for response from drone");
        }
        else if(!rc) {
            throw MoveException("Landing command rejected by drone");
        }
    }
    else 
    {
        throw MoveException("Cannot land safely with out a home position that tells us the home altitude.  Could fix this if we hook up a distance to ground sensor...");
    }

    float max_wait = 60;
    if (!waitForFunction([&]() {
        updateState();
        return current_state.controls.landed;
    }, max_wait, cancelable_action))
    {
        throw MoveException("Drone hasn't reported a landing state");
    }
    return true;
}

bool MavLinkDroneControl::goHome(CancelableActionBase& cancelable_action)
{
    bool rc = false;
    if (!drone_->returnToHome().wait(10000, &rc)) {
        throw MoveException("goHome - timeout waiting for response from drone");
    }
    return rc;
}

void MavLinkDroneControl::commandRollPitchZ(float pitch, float roll, float z, float yaw)
{
    if (target_height_ != -z) {
        // these PID values were calculated experimentally using AltHoldCommand n MavLinkTest, this provides the best
        // control over thrust to achieve minimal over/under shoot in a reasonable amount of time, but it has not
        // been tested on a real drone outside jMavSim, so it may need recalibrating...
        thrust_controller_.setPoint(-z, .05f, .005f, 0.09f);
        target_height_ = -z;
    }
    auto state = drone_->getVehicleState();
    float thrust = 0.21f + thrust_controller_.control(-state.local_est.pos.z);
    drone_->moveByAttitude(roll, pitch, yaw, 0, 0, 0, thrust);
}
void MavLinkDroneControl::commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode)
{
    float yaw = yaw_mode.yaw_or_rate * M_PIf / 180;
    drone_->moveByLocalVelocity(vx, vy, vz, !yaw_mode.is_rate, yaw);
}
void MavLinkDroneControl::commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode)
{
    float yaw = yaw_mode.yaw_or_rate * M_PIf / 180;
    drone_->moveByLocalVelocityWithAltHold(vx, vy, z, !yaw_mode.is_rate, yaw);
}
void MavLinkDroneControl::commandPosition(float x, float y, float z, const YawMode& yaw_mode)
{
    float yaw = yaw_mode.yaw_or_rate * M_PIf / 180;
    drone_->moveToLocalPosition(x, y, z, !yaw_mode.is_rate, yaw);
}

//virtual RC mode
RCData MavLinkDroneControl::getRCData()
{
    RCData rc_data;
    return rc_data;
}

bool MavLinkDroneControl::validateRCData(const RCData& rc_data)
{
    return true;
}

void MavLinkDroneControl::commandVirtualRC(const RCData& rc_data)
{
    throw MoveException("commandVirtualRC is not implemented yet");
}
void MavLinkDroneControl::commandEnableVirtualRC(bool enable)
{
    throw MoveException("commandVirtualRC is not implemented yet");
}

//drone parameters
float MavLinkDroneControl::getCommandPeriod() 
{
    return 1.0f/50; //1 period of 50hz
}
float MavLinkDroneControl::getTakeoffZ()
{
    // pick a number, PX4 doesn't have a fixed limit here, but 3 meters is probably safe 
    // enough to get out of the backwash turbulance.  Negative due to NED coordinate system.
    return -3.0f;    
}
float MavLinkDroneControl::getDistanceAccuracy() 
{
    return 0.5f;    //measured in simulator by firing commands "MoveToLocation -x 0 -y 0" multiple times and looking at distance travelled
}
const VehicleParams& MavLinkDroneControl::getVehicleParams()
{
    return MavLinkDroneControl::getInternalVehicleParams(); //defaults are good for PX4 generic quadrocopter.
}
//TODO: decouple DroneControlBase, VehicalParams and SafetyEval
const VehicleParams& MavLinkDroneControl::getInternalVehicleParams()
{
    static const VehicleParams vehicle_params_;
    return vehicle_params_; //defaults are good for DJI Matrice 100
}


}} //namespace
#endif
