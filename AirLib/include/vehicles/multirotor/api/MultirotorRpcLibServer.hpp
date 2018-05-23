// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_MultirotorRpcLibServer_hpp
#define air_MultirotorRpcLibServer_hpp

#include "common/Common.hpp"
#include <functional>
#include "api/RpcLibServerBase.hpp"
#include "vehicles/multirotor/api/MultirotorApiBase.h"


namespace msr { namespace airlib {

class MultirotorRpcLibServer : public RpcLibServerBase {
public:
    MultirotorRpcLibServer(MultirotorApiBase* vehicle_api, WorldSimApiBase* world_sim_api, string server_address, uint16_t port = 41451);
    virtual ~MultirotorRpcLibServer();

    virtual const MultirotorApiBase* getVehicleApi(const std::string& vehicle_name = "") const override
    {
        return vehicle_api_;
    }
    virtual MultirotorApiBase* getVehicleApi(const std::string& vehicle_name = "") override
    {
        return const_cast<MultirotorApiBase*>(getVehicleApi(vehicle_name));
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
    MultirotorApiBase* vehicle_api_;
    WorldSimApiBase* world_sim_api_;
};

}} //namespace
#endif
