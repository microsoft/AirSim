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

#ifdef nil
#undef nil
#endif // nil

#include "common/common_utils/WindowsApisCommonPre.hpp"
#undef FLOAT
#undef check
#include "rpc/client.h"
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
#ifndef check
#define check(expr) (static_cast<void>((expr)))
#endif
#include "common/common_utils/WindowsApisCommonPost.hpp"

#include "vehicles/multirotor/api/MultirotorRpcLibAdapators.hpp"

STRICT_MODE_ON
#ifdef _MSC_VER
__pragma(warning( disable : 4239))
#endif			  


namespace msr { namespace airlib {


typedef msr::airlib_rpclib::MultirotorRpcLibAdapators MultirotorRpcLibAdapators;

struct MultirotorRpcLibClient::impl {
public:
    std::future<RPCLIB_MSGPACK::object_handle> last_future;
};


MultirotorRpcLibClient::MultirotorRpcLibClient(const string&  ip_address, uint16_t port, float timeout_sec)
    : RpcLibClientBase(ip_address, port, timeout_sec)
{
    pimpl_.reset(new impl());
}

MultirotorRpcLibClient::~MultirotorRpcLibClient()
{}

MultirotorRpcLibClient* MultirotorRpcLibClient::takeoffAsync(float timeout_sec, const std::string& vehicle_name)
{
    pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("takeoff", timeout_sec, vehicle_name);
    return this;
}
MultirotorRpcLibClient* MultirotorRpcLibClient::landAsync(float timeout_sec, const std::string& vehicle_name)
{
    pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("land", timeout_sec, vehicle_name);
    return this;
}
MultirotorRpcLibClient* MultirotorRpcLibClient::goHomeAsync(float timeout_sec, const std::string& vehicle_name)
{
    pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("goHome", timeout_sec, vehicle_name);
    return this;
}

MultirotorRpcLibClient* MultirotorRpcLibClient::moveByAngleZAsync(float pitch, float roll, float z, float yaw, float duration, const std::string& vehicle_name)
{
    pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("moveByAngleZ", pitch, roll, z, yaw, duration, vehicle_name);
    return this;
}

MultirotorRpcLibClient* MultirotorRpcLibClient::moveByAngleThrottleAsync(float pitch, float roll, float throttle, float yaw_rate, float duration, const std::string& vehicle_name)
{
    pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("moveByAngleThrottle", pitch, roll, throttle, yaw_rate, duration, vehicle_name);
    return this;
}

MultirotorRpcLibClient* MultirotorRpcLibClient::moveByVelocityAsync(float vx, float vy, float vz, float duration, 
    DrivetrainType drivetrain, const YawMode& yaw_mode, const std::string& vehicle_name)
{
    pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("moveByVelocity", vx, vy, vz, duration, 
        drivetrain, MultirotorRpcLibAdapators::YawMode(yaw_mode), vehicle_name);
    return this;
}

MultirotorRpcLibClient* MultirotorRpcLibClient::moveByVelocityZAsync(float vx, float vy, float z, float duration, 
    DrivetrainType drivetrain, const YawMode& yaw_mode, const std::string& vehicle_name)
{
    pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("moveByVelocityZ", vx, vy, z, duration, 
        drivetrain, MultirotorRpcLibAdapators::YawMode(yaw_mode), vehicle_name);
    return this;
}

MultirotorRpcLibClient* MultirotorRpcLibClient::moveOnPathAsync(const vector<Vector3r>& path, float velocity, float duration, 
    DrivetrainType drivetrain, const YawMode& yaw_mode, float lookahead, float adaptive_lookahead, const std::string& vehicle_name)
{
    vector<MultirotorRpcLibAdapators::Vector3r> conv_path;
    MultirotorRpcLibAdapators::from(path, conv_path);
    pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("moveOnPath", conv_path, velocity, duration, 
        drivetrain, MultirotorRpcLibAdapators::YawMode(yaw_mode), lookahead, adaptive_lookahead, vehicle_name);
    return this;
}

MultirotorRpcLibClient* MultirotorRpcLibClient::moveToPositionAsync(float x, float y, float z, float velocity, float timeout_sec, 
    DrivetrainType drivetrain, const YawMode& yaw_mode, float lookahead, float adaptive_lookahead, const std::string& vehicle_name)
{
    pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("moveToPosition", x, y, z, velocity, timeout_sec, 
        drivetrain, MultirotorRpcLibAdapators::YawMode(yaw_mode), lookahead, adaptive_lookahead, vehicle_name);
    return this;
}

MultirotorRpcLibClient* MultirotorRpcLibClient::moveToZAsync(float z, float velocity, float timeout_sec, const 
    YawMode& yaw_mode, float lookahead, float adaptive_lookahead, const std::string& vehicle_name)
{
    pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("moveToZ", z, velocity, timeout_sec, 
        MultirotorRpcLibAdapators::YawMode(yaw_mode), lookahead, adaptive_lookahead, vehicle_name);
    return this;
}

MultirotorRpcLibClient* MultirotorRpcLibClient::moveByManualAsync(float vx_max, float vy_max, float z_min, float duration, 
    DrivetrainType drivetrain, const YawMode& yaw_mode, const std::string& vehicle_name)
{
    pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("moveByManual", vx_max, vy_max, z_min, duration, 
        drivetrain, MultirotorRpcLibAdapators::YawMode(yaw_mode), vehicle_name);
    return this;
}

MultirotorRpcLibClient* MultirotorRpcLibClient::rotateToYawAsync(float yaw, float timeout_sec, float margin, const std::string& vehicle_name)
{
    pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("rotateToYaw", yaw, timeout_sec, margin, vehicle_name);
    return this;
}

MultirotorRpcLibClient* MultirotorRpcLibClient::rotateByYawRateAsync(float yaw_rate, float duration, const std::string& vehicle_name)
{
    pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("rotateByYawRate", yaw_rate, duration, vehicle_name);
    return this;
}

MultirotorRpcLibClient* MultirotorRpcLibClient::hoverAsync(const std::string& vehicle_name)
{
    pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("hover", vehicle_name);
    return this;
}

bool MultirotorRpcLibClient::setSafety(SafetyEval::SafetyViolationType enable_reasons, float obs_clearance, SafetyEval::ObsAvoidanceStrategy obs_startegy,
    float obs_avoidance_vel, const Vector3r& origin, float xy_length, float max_z, float min_z, const std::string& vehicle_name)
{
    return static_cast<rpc::client*>(getClient())->call("setSafety", static_cast<uint>(enable_reasons), obs_clearance, obs_startegy,
        obs_avoidance_vel, MultirotorRpcLibAdapators::Vector3r(origin), xy_length, max_z, min_z, vehicle_name).as<bool>();
}

//status getters
MultirotorState MultirotorRpcLibClient::getMultirotorState(const std::string& vehicle_name)
{
    return static_cast<rpc::client*>(getClient())->call("getMultirotorState", vehicle_name).
        as<MultirotorRpcLibAdapators::MultirotorState>().to();
}

void MultirotorRpcLibClient::moveByRC(const RCData& rc_data, const std::string& vehicle_name)
{
    static_cast<rpc::client*>(getClient())->call("moveByRC", MultirotorRpcLibAdapators::RCData(rc_data), vehicle_name);
}

//return value of last task. It should be true if task completed without
//cancellation or timeout
MultirotorRpcLibClient* MultirotorRpcLibClient::waitOnLastTask(bool* task_result, float timeout_sec)
{
    bool result;
    if (std::isnan(timeout_sec) || timeout_sec == Utils::max<float>())
        result = pimpl_->last_future.get().as<bool>();
    else {
        auto future_status = pimpl_->last_future.wait_for(std::chrono::duration<double>(timeout_sec));
        if (future_status == std::future_status::ready)
            result = pimpl_->last_future.get().as<bool>();
        else
            result = false;
    }

    if (task_result)
        *task_result = result;

    return this;
}

}} //namespace

#endif
#endif
