// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include precompiled header file first
#ifdef AIRLIB_PCH
#include "AirSim.h"
#endif

#include "common/Common.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK

#include "rpc/server.h"
#include "rpc/RpcLibAdapators.hpp"
STRICT_MODE_ON
#include "rpc/RpcLibServer.hpp"


namespace msr { namespace airlib {

struct RpcLibServer::impl {
    impl(string server_address, uint16_t port)
        : server(server_address, port)
    {}

    rpc::server server;
};

typedef msr::airlib_rpclib::RpcLibAdapators RpcLibAdapators;

RpcLibServer::RpcLibServer(DroneControllerCancelable* drone, string server_address, uint16_t port)
        : drone_(drone)
{
    pimpl_.reset(new impl(server_address, port));
    pimpl_->server.bind("armDisarm", [&](bool arm) -> bool { return drone_->armDisarm(arm); });
    pimpl_->server.bind("setOffboardMode", [&](bool is_set) -> void { drone_->setOffboardMode(is_set); });
    pimpl_->server.bind("setSimulationMode", [&](bool is_set) -> void { drone_->setSimulationMode(is_set); });
    pimpl_->server.bind("setUserInputs", [&](const vector<float>& inputs) -> void { drone_->setUserInputs(inputs); });
    pimpl_->server.bind("takeoff", [&](float max_wait_seconds) -> bool { return drone_->takeoff(max_wait_seconds); });
    pimpl_->server.bind("land", [&]() -> bool { return drone_->land(); });
    pimpl_->server.bind("goHome", [&]() -> bool { return drone_->goHome(); });
    pimpl_->server.bind("start", [&]() -> void { drone_->start(); });
    pimpl_->server.bind("stop", [&]() -> void { drone_->stop(); });


    pimpl_->server.bind("moveByAngle", [&](float pitch, float roll, float z, float yaw, float duration) -> 
        bool { return drone_->moveByAngle(pitch, roll, z, yaw, duration); });
    pimpl_->server.bind("moveByVelocity", [&](float vx, float vy, float vz, float duration, DrivetrainType drivetrain, const RpcLibAdapators::YawMode& yaw_mode) -> 
        bool { return drone_->moveByVelocity(vx, vy, vz, duration, drivetrain, yaw_mode.to()); });
    pimpl_->server.bind("moveByVelocityZ", [&](float vx, float vy, float z, float duration, DrivetrainType drivetrain, const RpcLibAdapators::YawMode& yaw_mode) -> 
        bool { return drone_->moveByVelocityZ(vx, vy, z, duration, drivetrain, yaw_mode.to()); });
    pimpl_->server.bind("moveOnPath", [&](const vector<RpcLibAdapators::Vector3r>& path, float velocity, DrivetrainType drivetrain, const RpcLibAdapators::YawMode& yaw_mode,
        float lookahead, float adaptive_lookahead) ->
        bool { 
            vector<Vector3r> conv_path;
            RpcLibAdapators::to(path, conv_path);
            return drone_->moveOnPath(conv_path, velocity, drivetrain, yaw_mode.to(), lookahead, adaptive_lookahead); 
        });
    pimpl_->server.bind("moveToPosition", [&](float x, float y, float z, float velocity, DrivetrainType drivetrain,
        const RpcLibAdapators::YawMode& yaw_mode, float lookahead, float adaptive_lookahead) -> 
        bool { return drone_->moveToPosition(x, y, z, velocity, drivetrain, yaw_mode.to(), lookahead, adaptive_lookahead); });
    pimpl_->server.bind("moveToZ", [&](float z, float velocity, const RpcLibAdapators::YawMode& yaw_mode, float lookahead, float adaptive_lookahead) -> 
        bool { return drone_->moveToZ(z, velocity, yaw_mode.to(), lookahead, adaptive_lookahead); });
    pimpl_->server.bind("moveByManual", [&](float vx_max, float vy_max, float z_min, DrivetrainType drivetrain, const RpcLibAdapators::YawMode& yaw_mode, float duration) -> 
        bool { return drone_->moveByManual(vx_max, vy_max, z_min, drivetrain, yaw_mode.to(), duration); });

    pimpl_->server.bind("rotateToYaw", [&](float yaw, float margin) -> 
        bool { return drone_->rotateToYaw(yaw, margin); });
    pimpl_->server.bind("rotateByYawRate", [&](float yaw_rate, float duration) -> 
        bool { return drone_->rotateByYawRate(yaw_rate, duration); });
    pimpl_->server.bind("hover", [&]() -> bool { return drone_->hover(); });

    pimpl_->server.bind("setSafety", [&](uint enable_reasons, float obs_clearance, SafetyEval::ObsAvoidanceStrategy obs_startegy,
        float obs_avoidance_vel, const RpcLibAdapators::Vector3r& origin, float xy_length, float max_z, float min_z) -> 
        bool { return drone_->setSafety(SafetyEval::SafetyViolationType(enable_reasons), obs_clearance, obs_startegy,
            obs_avoidance_vel, origin.to(), xy_length, max_z, min_z); });
    pimpl_->server.bind("setImageTypeForCamera", [&](int camera_id, DroneControllerBase::ImageType type) -> void { drone_->setImageTypeForCamera(camera_id, type); });
    pimpl_->server.bind("getImageTypeForCamera", [&](int camera_id) -> DroneControllerBase::ImageType { return drone_->getImageTypeForCamera(camera_id); });
    pimpl_->server.bind("getImageForCamera", [&](int camera_id, DroneControllerBase::ImageType type) -> vector<uint8_t> { return drone_->getImageForCamera(camera_id, type); });


    //getters
    pimpl_->server.bind("getPosition", [&]() -> RpcLibAdapators::Vector3r { return drone_->getPosition(); });
    pimpl_->server.bind("getVelocity", [&]() -> RpcLibAdapators::Vector3r { return drone_->getVelocity(); });
    pimpl_->server.bind("getOrientation", [&]() -> RpcLibAdapators::Quaternionr { return drone_->getOrientation(); });
    pimpl_->server.bind("getRCData", [&]() -> RpcLibAdapators::RCData { return drone_->getRCData(); });
    pimpl_->server.bind("timestampNow", [&]() -> double { return drone_->timestampNow(); });
    pimpl_->server.bind("getHomePoint", [&]() -> RpcLibAdapators::GeoPoint { return drone_->getHomePoint(); });
    pimpl_->server.bind("getGpsLocation", [&]() -> RpcLibAdapators::GeoPoint { return drone_->getGpsLocation(); });
    pimpl_->server.bind("isOffboardMode", [&]() -> bool { return drone_->isOffboardMode(); });
    pimpl_->server.bind("isSimulationMode", [&]() -> bool { return drone_->isSimulationMode(); });
    pimpl_->server.bind("getServerDebugInfo", [&]() -> std::string { return drone_->getServerDebugInfo(); });

    pimpl_->server.suppress_exceptions(true);
}

//required for pimpl
RpcLibServer::~RpcLibServer()
{}

void RpcLibServer::start(bool block)
{
    if (block)
        pimpl_->server.run();
    else
        pimpl_->server.async_run(2);   //2 threads
}

void RpcLibServer::stop()
{
    pimpl_->server.stop();
}

}} //namespace


#endif
#endif
