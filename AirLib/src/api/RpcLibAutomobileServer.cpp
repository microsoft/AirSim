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
#include "api/RpcLibAutomobileServer.hpp"

#include "common/Common.hpp"
#include "controllers/VehicleCameraBase.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
#undef check
#include <rpc/server.h>
#include "api/RpcLibAdapators.hpp"
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
STRICT_MODE_ON


namespace msr { namespace airlib {

    struct RpcLibAutomobileServer::impl {
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

    RpcLibAutomobileServer::RpcLibAutomobileServer(AutomobileSedanController* vechicle_controller, string server_address, uint16_t port)
        : vehicle_controller_(vechicle_controller)

    {
        if (server_address == "")
            pimpl_.reset(new impl(port));
        else
            pimpl_.reset(new impl(server_address, port));
        pimpl_->server.bind("ping", [&]() -> bool { return true; });
        pimpl_->server.bind("setVehicleControlSignals", [&](float steeringAngle, float throttlePercentage, float brakePercentage) -> bool {
            return setVehicleControlSignals(steeringAngle, throttlePercentage, brakePercentage);
        });

        pimpl_->server.bind("simGetImage", [&](uint8_t camera_id, VehicleCameraBase::ImageType_ type) -> vector<uint8_t> {
            return getImageFromCamera(camera_id, type);
        });

        pimpl_->server.suppress_exceptions(true);
    }

    //required for pimpl
    RpcLibAutomobileServer::~RpcLibAutomobileServer()
    {
        stop();
        vehicle_controller_ = nullptr;
    }

    void RpcLibAutomobileServer::start(bool block)
    {
        if (block)
            pimpl_->server.run();
        else
            pimpl_->server.async_run(4);   //4 threads
    }

    void RpcLibAutomobileServer::stop()
    {
        pimpl_->server.stop();
    }

    bool RpcLibAutomobileServer::setVehicleControlSignals(float steeringAngle, float throttlePercentage, float brakePercentage)
    { 
        vehicle_controller_->setVehicleControlSignals(static_cast<real_T>(steeringAngle), static_cast<real_T>(throttlePercentage), static_cast<real_T>(brakePercentage));
        return true;
    }

    vector<uint8_t> RpcLibAutomobileServer::getImageFromCamera(uint8_t camera_id, VehicleCameraBase::ImageType_ type)
    {
		auto result = vehicle_controller_->simGetImage(camera_id, type);
		if (result.size() == 0) {
			// rpclib has a bug with serializing empty vectors, so we return a 1 byte vector instead.
			result.push_back(0);
		}
		return result;
    }

}} //namespace

#endif
#endif
