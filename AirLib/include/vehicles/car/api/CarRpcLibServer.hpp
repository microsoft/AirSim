// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_CarRpcLibServer_hpp
#define air_CarRpcLibServer_hpp

#ifndef AIRLIB_NO_RPC

#include "common/Common.hpp"
#include <functional>
#include "api/RpcLibServerBase.hpp"
#include "vehicles/car/api/CarApiBase.hpp"

namespace msr
{
namespace airlib
{

    class CarRpcLibServer : public RpcLibServerBase
    {
    public:
        CarRpcLibServer(ApiProvider* api_provider, string server_address, uint16_t port = RpcLibPort);
        virtual ~CarRpcLibServer();

    protected:
        virtual CarApiBase* getVehicleApi(const std::string& vehicle_name) override
        {
            return static_cast<CarApiBase*>(RpcLibServerBase::getVehicleApi(vehicle_name));
        }
    };

#endif
}
} //namespace
#endif