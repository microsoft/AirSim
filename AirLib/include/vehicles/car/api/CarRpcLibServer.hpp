// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_CarRpcLibServer_hpp
#define air_CarRpcLibServer_hpp

#include "common/Common.hpp"
#include <functional>
#include "api/RpcLibServerBase.hpp"
#include "vehicles/car/api/CarApiBase.hpp"

namespace msr { namespace airlib {

class CarRpcLibServer : public RpcLibServerBase {
public:
    CarRpcLibServer(VehicleApiBase* vehicle_api, WorldSimApiBase* world_sim_api, string server_address, uint16_t port = 41451);
    virtual ~CarRpcLibServer();

    virtual const CarApiBase* getVehicleApi(const std::string& vehicle_name = "") const override
    {
        return vehicle_api_;
    }
    virtual CarApiBase* getVehicleApi(const std::string& vehicle_name = "") override
    {
        return const_cast<CarApiBase*>(getVehicleApi(vehicle_name));
    }
    virtual const WorldSimApiBase* getWorldSimApi() const override
    {
        return world_sim_api_;
    }
    virtual WorldSimApiBase* getWorldSimApi() override
    {
        return const_cast<WorldSimApiBase*>(getWorldSimApi());
    }

private:
    CarApiBase* vehicle_api_;
    WorldSimApiBase* world_sim_api_;
};

}} //namespace
#endif