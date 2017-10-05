// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include precompiled header file first

#include "vehicles/car/api/CarRpcLibServer.hpp"


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
#include "vehicles/car/api/CarRpcLibAdapators.hpp"
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
#define check(expr) (static_cast<void>((expr)))
STRICT_MODE_ON


namespace msr { namespace airlib {

struct CarRpcLibServer::impl {
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

typedef msr::airlib_rpclib::CarRpcLibAdapators CarRpcLibAdapators;

CarRpcLibServer::CarRpcLibServer(CarApiBase* vehicle, string server_address, uint16_t port)
    : vehicle_(vehicle)
{
    if (server_address == "")
        pimpl_.reset(new impl(port));
    else
        pimpl_.reset(new impl(server_address, port));
    pimpl_->server.bind("ping", [&]() -> bool { return true; });


    //sim only
    pimpl_->server.bind("simSetPose", [&](const CarRpcLibAdapators::Pose &pose, bool ignore_collison) -> void { 
        vehicle_->simSetPose(pose.to(), ignore_collison); 
    });
    pimpl_->server.bind("simGetPose", [&]() ->
        CarRpcLibAdapators::Pose { return vehicle_->simGetPose(); 
    });
    pimpl_->server.bind("simGetImages", [&](const std::vector<CarRpcLibAdapators::ImageRequest>& request_adapter) -> vector<CarRpcLibAdapators::ImageResponse> { 
        const auto& response = vehicle_->simGetImages(CarRpcLibAdapators::ImageRequest::to(request_adapter)); 
        return CarRpcLibAdapators::ImageResponse::from(response);
    });
    pimpl_->server.bind("simGetImage", [&](uint8_t camera_id, VehicleCameraBase::ImageType type) -> vector<uint8_t> { 
        auto result = vehicle_->simGetImage(camera_id, type); 
        if (result.size() == 0) {
            // rpclib has a bug with serializing empty vectors, so we return a 1 byte vector instead.
            result.push_back(0);
        }
        return result;
    });

    pimpl_->server.bind("setCarControls", [&](const CarRpcLibAdapators::CarControls& controls) -> void {
        vehicle_->setCarControls(controls.to());
    });

    pimpl_->server.bind("reset", [&]() -> void {
        vehicle_->reset();
    });

    pimpl_->server.bind("getCarState", [&]() -> CarRpcLibAdapators::CarState {
        return CarRpcLibAdapators::CarState(vehicle_->getCarState());
    });

    pimpl_->server.bind("getHomeGeoPoint", [&]() -> CarRpcLibAdapators::GeoPoint { 
        return vehicle_->getHomeGeoPoint(); 
    });

    pimpl_->server.bind("enableApiControl", [&](bool is_enabled) -> void { vehicle_->enableApiControl(is_enabled); });
    pimpl_->server.bind("isApiControlEnabled", [&]() -> bool { return vehicle_->isApiControlEnabled(); });
    
    pimpl_->server.suppress_exceptions(true);
}

//required for pimpl
CarRpcLibServer::~CarRpcLibServer()
{
    stop();
    vehicle_ = nullptr;
}

void CarRpcLibServer::start(bool block)
{
    if (block)
        pimpl_->server.run();
    else
        pimpl_->server.async_run(4);   //4 threads
}

void CarRpcLibServer::stop()
{
    pimpl_->server.stop();
}

}} //namespace


#endif
#endif
