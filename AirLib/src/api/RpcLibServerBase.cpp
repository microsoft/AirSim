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

RpcLibServerBase::RpcLibServerBase(const std::string& server_address, uint16_t port)
{
    pimpl_->server.bind("getServerVersion", []() -> int {
        return 1;
    });
    pimpl_->server.bind("getMinRequiredClientVersion", []() -> int {
        return 1;
    });


    if (server_address == "")
        pimpl_.reset(new impl(port));
    else
        pimpl_.reset(new impl(server_address, port));
    pimpl_->server.bind("ping", [&]() -> bool { return true; });
    pimpl_->server.bind("enableApiControl", [&](bool is_enabled) -> void { getVehicleApi()->enableApiControl(is_enabled); });
    pimpl_->server.bind("isApiControlEnabled", [&]() -> bool { return getVehicleApi()->isApiControlEnabled(); });
    pimpl_->server.bind("armDisarm", [&](bool arm) -> bool { return getVehicleApi()->armDisarm(arm); });

       
    pimpl_->server.bind("simPause", [&](bool is_paused) -> void { 
        getWorldSimApi()->pause(is_paused); 
    });
    pimpl_->server.bind("simIsPaused", [&]() -> bool { 
        return getWorldSimApi()->isPaused(); 
    });
    pimpl_->server.bind("simContinueForTime", [&](double seconds) -> void { 
        getWorldSimApi()->continueForTime(seconds); 
    });


    pimpl_->server.bind("simGetImages", [&](const std::vector<RpcLibAdapatorsBase::ImageRequest>& request_adapter) -> vector<RpcLibAdapatorsBase::ImageResponse> {
        const auto& response = getVehicleSimApi()->getImages(RpcLibAdapatorsBase::ImageRequest::to(request_adapter));
        return RpcLibAdapatorsBase::ImageResponse::from(response);
    });
    pimpl_->server.bind("simGetImage", [&](uint8_t camera_id, ImageCaptureBase::ImageType type) -> vector<uint8_t> {
        auto result = getVehicleSimApi()->getImage(camera_id, type);
        if (result.size() == 0) {
            // rpclib has a bug with serializing empty vectors, so we return a 1 byte vector instead.
            result.push_back(0);
        }
        return result;
    });

    pimpl_->server.
        bind("simSetPose", [&](const RpcLibAdapatorsBase::Pose &pose, bool ignore_collision) -> void {
        getVehicleSimApi()->setPose(pose.to(), ignore_collision);
    });
    pimpl_->server.bind("simGetPose", [&]() -> RpcLibAdapatorsBase::Pose { 
        const auto& pose = getVehicleSimApi()->getPose();
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

    pimpl_->server.bind("simResetWorld", [&]() -> void {
        getWorldSimApi()->reset();
    });
    pimpl_->server.bind("resetVehicle", [&]() -> void {
        auto* sim_vehicle_api = getVehicleSimApi();
        if (sim_vehicle_api)
            sim_vehicle_api->reset();
        else
            getVehicleApi()->reset();
    });

    pimpl_->server.bind("simPrintLogMessage", [&](const std::string& message, const std::string& message_param, unsigned char severity) -> void {
        getWorldSimApi()->printLogMessage(message, message_param, severity);
    });

    pimpl_->server.bind("getHomeGeoPoint", [&]() -> RpcLibAdapatorsBase::GeoPoint {
        const auto& geo_point = getVehicleApi()->getHomeGeoPoint();
        return RpcLibAdapatorsBase::GeoPoint(geo_point);
    });

    pimpl_->server.bind("getCameraInfo", [&](int camera_id) -> RpcLibAdapatorsBase::CameraInfo {
        const auto& camera_info = getVehicleApi()->getCameraInfo(camera_id);
        return RpcLibAdapatorsBase::CameraInfo(camera_info);
    });

    pimpl_->server.bind("setCameraOrientation", [&](int camera_id, const RpcLibAdapatorsBase::Quaternionr& orientation) -> void {
        getVehicleApi()->setCameraOrientation(camera_id, orientation.to());
    });

    pimpl_->server.bind("simGetCollisionInfo", [&]() -> RpcLibAdapatorsBase::CollisionInfo { 
        const auto& collision_info = getVehicleSimApi()->getCollisionInfo(); 
        return RpcLibAdapatorsBase::CollisionInfo(collision_info);
    });

    pimpl_->server.bind("simGetObjectPose", [&](const std::string& object_name) -> RpcLibAdapatorsBase::Pose { 
        const auto& pose = getWorldSimApi()->getObjectPose(object_name); 
        return RpcLibAdapatorsBase::Pose(pose);
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
