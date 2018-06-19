// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include precompiled header file first

#include "api/RpcLibServerBase.hpp"


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
#include "api/RpcLibAdapatorsBase.hpp"
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
#define check(expr) (static_cast<void>((expr)))
STRICT_MODE_ON


namespace msr { namespace airlib {

struct RpcLibServerBase::impl {
    impl(string server_address, uint16_t port)
        : server(server_address, port)
    {}

    impl(uint16_t port)
        : server(port)
    {}

    ~impl() {
    }

    rpc::server server;
};

typedef msr::airlib_rpclib::RpcLibAdapatorsBase RpcLibAdapatorsBase;

RpcLibServerBase::RpcLibServerBase(ApiProvider* api_provider, const std::string& server_address, uint16_t port)
    : api_provider_(api_provider)
{
    if (server_address == "")
        pimpl_.reset(new impl(port));
    else
        pimpl_.reset(new impl(server_address, port));
    pimpl_->server.bind("ping", [&]() -> bool { return true; });
    pimpl_->server.bind("getServerVersion", []() -> int {
        return 1;
    });
    pimpl_->server.bind("getMinRequiredClientVersion", []() -> int {
        return 1;
    });
       
    pimpl_->server.bind("simPause", [&](bool is_paused) -> void { 
        getWorldSimApi()->pause(is_paused); 
    });
    pimpl_->server.bind("simIsPaused", [&]() -> bool { 
        return getWorldSimApi()->isPaused(); 
    });
    pimpl_->server.bind("simContinueForTime", [&](double seconds) -> void { 
        getWorldSimApi()->continueForTime(seconds); 
    });

    pimpl_->server.bind("enableApiControl", [&](bool is_enabled, const std::string& vehicle_name) -> void { 
        getVehicleApi(vehicle_name)->enableApiControl(is_enabled);
    });
    pimpl_->server.bind("isApiControlEnabled", [&](const std::string& vehicle_name) -> bool { 
        return getVehicleApi(vehicle_name)->isApiControlEnabled();
    });
    pimpl_->server.bind("armDisarm", [&](bool arm, const std::string& vehicle_name) -> bool { 
        return getVehicleApi(vehicle_name)->armDisarm(arm);
    });

    pimpl_->server.bind("simGetImages", [&](const std::vector<RpcLibAdapatorsBase::ImageRequest>& request_adapter, const std::string& vehicle_name) -> 
        vector<RpcLibAdapatorsBase::ImageResponse> {
            const auto& response = getVehicleSimApi(vehicle_name)->getImages(RpcLibAdapatorsBase::ImageRequest::to(request_adapter));
            return RpcLibAdapatorsBase::ImageResponse::from(response);
    });
    pimpl_->server.bind("simGetImage", [&](const std::string& camera_name, ImageCaptureBase::ImageType type, const std::string& vehicle_name) -> vector<uint8_t> {
        auto result = getVehicleSimApi(vehicle_name)->getImage(camera_name, type);
        if (result.size() == 0) {
            // rpclib has a bug with serializing empty vectors, so we return a 1 byte vector instead.
            result.push_back(0);
        }
        return result;
    });

    pimpl_->server.
        bind("simSetVehiclePose", [&](const RpcLibAdapatorsBase::Pose &pose, bool ignore_collision, const std::string& vehicle_name) -> void {
        getVehicleSimApi(vehicle_name)->setPose(pose.to(), ignore_collision);
    });
    pimpl_->server.bind("simGetVehiclePose", [&](const std::string& vehicle_name) -> RpcLibAdapatorsBase::Pose {
        const auto& pose = getVehicleSimApi(vehicle_name)->getPose();
        return RpcLibAdapatorsBase::Pose(pose);
    });

    pimpl_->server.
        bind("simSetSegmentationObjectID", [&](const std::string& mesh_name, int object_id, bool is_name_regex) -> bool {
        return getWorldSimApi()->setSegmentationObjectID(mesh_name, object_id, is_name_regex);
    });
    pimpl_->server.
        bind("simGetSegmentationObjectID", [&](const std::string& mesh_name) -> int {
        return getWorldSimApi()->getSegmentationObjectID(mesh_name);
    });    

    pimpl_->server.bind("reset", [&]() -> void {
        auto* sim_world_api = getWorldSimApi();
        if (sim_world_api)
            sim_world_api->reset();
        else
            getVehicleApi("")->reset();
    });

    pimpl_->server.bind("simPrintLogMessage", [&](const std::string& message, const std::string& message_param, unsigned char severity) -> void {
        getWorldSimApi()->printLogMessage(message, message_param, severity);
    });

    pimpl_->server.bind("getHomeGeoPoint", [&](const std::string& vehicle_name) -> RpcLibAdapatorsBase::GeoPoint {
        const auto& geo_point = getVehicleApi(vehicle_name)->getHomeGeoPoint();
        return RpcLibAdapatorsBase::GeoPoint(geo_point);
    });

    pimpl_->server.bind("simGetCameraInfo", [&](const std::string& camera_name, const std::string& vehicle_name) -> RpcLibAdapatorsBase::CameraInfo {
        const auto& camera_info = getVehicleSimApi(vehicle_name)->getCameraInfo(camera_name);
        return RpcLibAdapatorsBase::CameraInfo(camera_info);
    });

    pimpl_->server.bind("simSetCameraOrientation", [&](const std::string& camera_name, const RpcLibAdapatorsBase::Quaternionr& orientation, 
        const std::string& vehicle_name) -> void {
        getVehicleSimApi(vehicle_name)->setCameraOrientation(camera_name, orientation.to());
    });

    pimpl_->server.bind("simGetCollisionInfo", [&](const std::string& vehicle_name) -> RpcLibAdapatorsBase::CollisionInfo {
        const auto& collision_info = getVehicleSimApi(vehicle_name)->getCollisionInfo(); 
        return RpcLibAdapatorsBase::CollisionInfo(collision_info);
    });

    pimpl_->server.bind("simGetObjectPose", [&](const std::string& object_name) -> RpcLibAdapatorsBase::Pose {
        const auto& pose = getWorldSimApi()->getObjectPose(object_name); 
        return RpcLibAdapatorsBase::Pose(pose);
    });

    pimpl_->server.bind("simGetGroundTruthKinematics", [&](const std::string& vehicle_name) -> RpcLibAdapatorsBase::KinematicsState {
        const Kinematics::State& result = *getVehicleSimApi(vehicle_name)->getGroundTruthKinematics();
        return RpcLibAdapatorsBase::KinematicsState(result);
    });

    pimpl_->server.bind("simGetGroundTruthEnvironment", [&](const std::string& vehicle_name) -> RpcLibAdapatorsBase::EnvironmentState {
        const Environment::State& result = (*getVehicleSimApi(vehicle_name)->getGroundTruthEnvironment()).getState();
        return RpcLibAdapatorsBase::EnvironmentState(result);
    });

    pimpl_->server.bind("cancelLastTask", [&](const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->cancelLastTask();
    });

    //if we don't suppress then server will bomb out for exceptions raised by any method
    pimpl_->server.suppress_exceptions(true);
}

//required for pimpl
RpcLibServerBase::~RpcLibServerBase()
{
    stop();
}

void RpcLibServerBase::start(bool block)
{
    if (block)
        pimpl_->server.run();
    else
        pimpl_->server.async_run(4);   //4 threads
}

void RpcLibServerBase::stop()
{
    pimpl_->server.stop();
}

void* RpcLibServerBase::getServer() const
{
    return &pimpl_->server;
}

}} //namespace
#endif
#endif
