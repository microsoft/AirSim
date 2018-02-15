// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include precompiled header file first

#include "vehicles/multirotor/api/MultirotorRpcLibServer.hpp"


#include "common/Common.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "common/common_utils/MinWinDefines.hpp"
#undef NOUSER
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
//#undef check
#include "rpc/server.h"
#include "vehicles/multirotor/api/MultirotorRpcLibAdapators.hpp"
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
#define check(expr) (static_cast<void>((expr)))
STRICT_MODE_ON


namespace msr { namespace airlib {

typedef msr::airlib_rpclib::MultirotorRpcLibAdapators MultirotorRpcLibAdapators;

MultirotorRpcLibServer::MultirotorRpcLibServer(MultirotorApi* drone, string server_address, uint16_t port)
        : RpcLibServerBase(drone, server_address, port)
{
    (static_cast<rpc::server*>(getServer()))->
        bind("armDisarm", [&](bool arm) -> bool { return getDroneApi()->armDisarm(arm); });
    (static_cast<rpc::server*>(getServer()))->
        bind("setSimulationMode", [&](bool is_set) -> void { getDroneApi()->setSimulationMode(is_set); });
    (static_cast<rpc::server*>(getServer()))->
        bind("takeoff", [&](float max_wait_seconds) -> bool { return getDroneApi()->takeoff(max_wait_seconds); });
    (static_cast<rpc::server*>(getServer()))->
        bind("land", [&](float max_wait_seconds) -> bool { return getDroneApi()->land(max_wait_seconds); });
    (static_cast<rpc::server*>(getServer()))->
        bind("goHome", [&]() -> bool { return getDroneApi()->goHome(); });


    (static_cast<rpc::server*>(getServer()))->
        bind("moveByAngle", [&](float pitch, float roll, float z, float yaw, float duration) -> 
        bool { return getDroneApi()->moveByAngle(pitch, roll, z, yaw, duration); });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveByVelocity", [&](float vx, float vy, float vz, float duration, DrivetrainType drivetrain, const MultirotorRpcLibAdapators::YawMode& yaw_mode) -> 
        bool { return getDroneApi()->moveByVelocity(vx, vy, vz, duration, drivetrain, yaw_mode.to()); });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveByVelocityZ", [&](float vx, float vy, float z, float duration, DrivetrainType drivetrain, const MultirotorRpcLibAdapators::YawMode& yaw_mode) -> 
        bool { return getDroneApi()->moveByVelocityZ(vx, vy, z, duration, drivetrain, yaw_mode.to()); });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveOnPath", [&](const vector<MultirotorRpcLibAdapators::Vector3r>& path, float velocity, float max_wait_seconds, DrivetrainType drivetrain, const MultirotorRpcLibAdapators::YawMode& yaw_mode,
        float lookahead, float adaptive_lookahead) ->
        bool { 
            vector<Vector3r> conv_path;
            MultirotorRpcLibAdapators::to(path, conv_path);
            return getDroneApi()->moveOnPath(conv_path, velocity, max_wait_seconds, drivetrain, yaw_mode.to(), lookahead, adaptive_lookahead);
        });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveToPosition", [&](float x, float y, float z, float velocity, float max_wait_seconds, DrivetrainType drivetrain,
        const MultirotorRpcLibAdapators::YawMode& yaw_mode, float lookahead, float adaptive_lookahead) -> 
        bool { return getDroneApi()->moveToPosition(x, y, z, velocity, max_wait_seconds, drivetrain, yaw_mode.to(), lookahead, adaptive_lookahead); });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveToZ", [&](float z, float velocity, float max_wait_seconds, const MultirotorRpcLibAdapators::YawMode& yaw_mode, float lookahead, float adaptive_lookahead) ->
        bool { return getDroneApi()->moveToZ(z, velocity, max_wait_seconds, yaw_mode.to(), lookahead, adaptive_lookahead); });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveByManual", [&](float vx_max, float vy_max, float z_min, float duration, DrivetrainType drivetrain, const MultirotorRpcLibAdapators::YawMode& yaw_mode) ->
        bool { return getDroneApi()->moveByManual(vx_max, vy_max, z_min, duration, drivetrain, yaw_mode.to()); });

    (static_cast<rpc::server*>(getServer()))->
        bind("rotateToYaw", [&](float yaw, float max_wait_seconds, float margin) ->
        bool { return getDroneApi()->rotateToYaw(yaw, max_wait_seconds, margin); });
    (static_cast<rpc::server*>(getServer()))->
        bind("rotateByYawRate", [&](float yaw_rate, float duration) -> 
        bool { return getDroneApi()->rotateByYawRate(yaw_rate, duration); });
    (static_cast<rpc::server*>(getServer()))->
        bind("hover", [&]() -> bool { return getDroneApi()->hover(); });

    (static_cast<rpc::server*>(getServer()))->
        bind("setSafety", [&](uint enable_reasons, float obs_clearance, const SafetyEval::ObsAvoidanceStrategy& obs_startegy,
        float obs_avoidance_vel, const MultirotorRpcLibAdapators::Vector3r& origin, float xy_length, float max_z, float min_z) -> 
        bool { return getDroneApi()->setSafety(SafetyEval::SafetyViolationType(enable_reasons), obs_clearance, obs_startegy,
            obs_avoidance_vel, origin.to(), xy_length, max_z, min_z); });
	(static_cast<rpc::server*>(getServer()))->
		bind("setRCData", [&](const MultirotorRpcLibAdapators::RCData& data) ->
		void { getDroneApi()->setRCData(data.to()); });


    //getters
    (static_cast<rpc::server*>(getServer()))->
        bind("getMultirotorState", [&]() -> MultirotorRpcLibAdapators::MultirotorState { 
        return MultirotorRpcLibAdapators::MultirotorState(getDroneApi()->getMultirotorState()); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("getPosition", [&]() -> MultirotorRpcLibAdapators::Vector3r { 
        return MultirotorRpcLibAdapators::Vector3r(getDroneApi()->getPosition()); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("getVelocity", [&]() -> MultirotorRpcLibAdapators::Vector3r { 
        return MultirotorRpcLibAdapators::Vector3r(getDroneApi()->getVelocity()); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("getOrientation", [&]() -> MultirotorRpcLibAdapators::Quaternionr { 
        return MultirotorRpcLibAdapators::Quaternionr(getDroneApi()->getOrientation()); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("getLandedState", [&]() -> int { 
        return static_cast<int>(getDroneApi()->getLandedState()); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("getRCData", [&]() -> MultirotorRpcLibAdapators::RCData { 
        return MultirotorRpcLibAdapators::RCData(getDroneApi()->getRCData()); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("getGpsLocation", [&]() -> MultirotorRpcLibAdapators::GeoPoint { 
        return MultirotorRpcLibAdapators::GeoPoint(getDroneApi()->getGpsLocation()); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("isSimulationMode", [&]() -> bool { 
        return getDroneApi()->isSimulationMode(); 
    });

}

//required for pimpl
MultirotorRpcLibServer::~MultirotorRpcLibServer()
{
}

MultirotorApi* MultirotorRpcLibServer::getDroneApi()
{
    return static_cast<MultirotorApi*>(RpcLibServerBase::getVehicleApi());
}

}} //namespace


#endif
#endif
