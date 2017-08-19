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

namespace msr { namespace airlib {

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
        vehicle_controller_ = nullptr;
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
