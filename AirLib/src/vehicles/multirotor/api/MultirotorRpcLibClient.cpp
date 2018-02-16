// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include precompiled header file first

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

#include "common/Common.hpp"
#include <thread>
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#undef check
#ifdef nil
#undef nil
#endif // nil
#include "rpc/client.h"
#include "vehicles/multirotor/api/MultirotorRpcLibAdapators.hpp"
STRICT_MODE_ON
#ifdef _MSC_VER
__pragma(warning( disable : 4239))
#endif			  


namespace msr { namespace airlib {


typedef msr::airlib_rpclib::MultirotorRpcLibAdapators MultirotorRpcLibAdapators;

MultirotorRpcLibClient::MultirotorRpcLibClient(const string&  ip_address, uint16_t port, uint timeout_ms)
    : RpcLibClientBase(ip_address, port, timeout_ms)
{
}

MultirotorRpcLibClient::~MultirotorRpcLibClient()
{}

bool MultirotorRpcLibClient::armDisarm(bool arm)
{
    return static_cast<rpc::client*>(getClient())->call("armDisarm", arm).as<bool>();
}
void MultirotorRpcLibClient::setSimulationMode(bool is_set)
{
    static_cast<rpc::client*>(getClient())->call("setSimulationMode", is_set);
}
bool MultirotorRpcLibClient::takeoff(float max_wait_seconds)
{
    return static_cast<rpc::client*>(getClient())->call("takeoff", max_wait_seconds).as<bool>();
}
bool MultirotorRpcLibClient::land(float max_wait_seconds)
{
    return static_cast<rpc::client*>(getClient())->call("land", max_wait_seconds).as<bool>();
}
bool MultirotorRpcLibClient::goHome()
{
    return static_cast<rpc::client*>(getClient())->call("goHome").as<bool>();
}

bool MultirotorRpcLibClient::moveByAngle(float pitch, float roll, float z, float yaw, float duration)
{
    return static_cast<rpc::client*>(getClient())->call("moveByAngle", pitch, roll, z, yaw, duration).as<bool>();
}

bool MultirotorRpcLibClient::moveByVelocity(float vx, float vy, float vz, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode)
{
    return static_cast<rpc::client*>(getClient())->call("moveByVelocity", vx, vy, vz, duration, drivetrain, MultirotorRpcLibAdapators::YawMode(yaw_mode)).as<bool>();
}

bool MultirotorRpcLibClient::moveByVelocityZ(float vx, float vy, float z, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode)
{
    return static_cast<rpc::client*>(getClient())->call("moveByVelocityZ", vx, vy, z, duration, drivetrain, MultirotorRpcLibAdapators::YawMode(yaw_mode)).as<bool>();
}

bool MultirotorRpcLibClient::moveOnPath(const vector<Vector3r>& path, float velocity, float max_wait_seconds, DrivetrainType drivetrain, const YawMode& yaw_mode, float lookahead, float adaptive_lookahead)
{
    vector<MultirotorRpcLibAdapators::Vector3r> conv_path;
    MultirotorRpcLibAdapators::from(path, conv_path);
    return static_cast<rpc::client*>(getClient())->call("moveOnPath", conv_path, velocity, max_wait_seconds, drivetrain, MultirotorRpcLibAdapators::YawMode(yaw_mode), lookahead, adaptive_lookahead).as<bool>();
}

bool MultirotorRpcLibClient::moveToPosition(float x, float y, float z, float velocity, float max_wait_seconds, DrivetrainType drivetrain, const YawMode& yaw_mode, float lookahead, float adaptive_lookahead)
{
    return static_cast<rpc::client*>(getClient())->call("moveToPosition", x, y, z, velocity, max_wait_seconds, drivetrain, MultirotorRpcLibAdapators::YawMode(yaw_mode), lookahead, adaptive_lookahead).as<bool>();
}

bool MultirotorRpcLibClient::moveToZ(float z, float velocity, float max_wait_seconds, const YawMode& yaw_mode, float lookahead, float adaptive_lookahead)
{
    return static_cast<rpc::client*>(getClient())->call("moveToZ", z, velocity, max_wait_seconds, MultirotorRpcLibAdapators::YawMode(yaw_mode), lookahead, adaptive_lookahead).as<bool>();
}

bool MultirotorRpcLibClient::moveByManual(float vx_max, float vy_max, float z_min, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode)
{
    return static_cast<rpc::client*>(getClient())->call("moveByManual", vx_max, vy_max, z_min, duration, drivetrain, MultirotorRpcLibAdapators::YawMode(yaw_mode)).as<bool>();
}

bool MultirotorRpcLibClient::rotateToYaw(float yaw, float max_wait_seconds, float margin)
{
    return static_cast<rpc::client*>(getClient())->call("rotateToYaw", yaw, max_wait_seconds, margin).as<bool>();
}

bool MultirotorRpcLibClient::rotateByYawRate(float yaw_rate, float duration)
{
    return static_cast<rpc::client*>(getClient())->call("rotateByYawRate", yaw_rate, duration).as<bool>();
}

bool MultirotorRpcLibClient::hover()
{
    return static_cast<rpc::client*>(getClient())->call("hover").as<bool>();
}

bool MultirotorRpcLibClient::setSafety(SafetyEval::SafetyViolationType enable_reasons, float obs_clearance, SafetyEval::ObsAvoidanceStrategy obs_startegy,
    float obs_avoidance_vel, const Vector3r& origin, float xy_length, float max_z, float min_z)
{
    return static_cast<rpc::client*>(getClient())->call("setSafety", static_cast<uint>(enable_reasons), obs_clearance, obs_startegy,
        obs_avoidance_vel, MultirotorRpcLibAdapators::Vector3r(origin), xy_length, max_z, min_z).as<bool>();
}

//status getters
MultirotorState MultirotorRpcLibClient::getMultirotorState()
{
    return static_cast<rpc::client*>(getClient())->call("getMultirotorState").
        as<MultirotorRpcLibAdapators::MultirotorState>().to();
}

Vector3r MultirotorRpcLibClient::getPosition()
{
    return static_cast<rpc::client*>(getClient())->call("getPosition").as<MultirotorRpcLibAdapators::Vector3r>().to();
}
Vector3r MultirotorRpcLibClient::getVelocity()
{
    return static_cast<rpc::client*>(getClient())->call("getVelocity").as<MultirotorRpcLibAdapators::Vector3r>().to();
}
Quaternionr MultirotorRpcLibClient::getOrientation()
{
    return static_cast<rpc::client*>(getClient())->call("getOrientation").as<MultirotorRpcLibAdapators::Quaternionr>().to();
}

DroneControllerBase::LandedState MultirotorRpcLibClient::getLandedState()
{
    int result = static_cast<rpc::client*>(getClient())->call("getLandedState").as<int>();
    return static_cast<DroneControllerBase::LandedState>(result);
}

RCData MultirotorRpcLibClient::getRCData()
{
    return static_cast<rpc::client*>(getClient())->call("getRCData").as<MultirotorRpcLibAdapators::RCData>().to();
}
void MultirotorRpcLibClient::setRCData(const RCData& rc_data)
{
    static_cast<rpc::client*>(getClient())->call("setRCData", MultirotorRpcLibAdapators::RCData(rc_data));
}

GeoPoint MultirotorRpcLibClient::getGpsLocation()
{
    return static_cast<rpc::client*>(getClient())->call("getGpsLocation").as<MultirotorRpcLibAdapators::GeoPoint>().to();
}

bool MultirotorRpcLibClient::isSimulationMode()
{
    return static_cast<rpc::client*>(getClient())->call("isSimulationMode").as<bool>();
}


}} //namespace

#endif
#endif
