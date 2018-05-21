// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_ApiServerBase_hpp
#define air_ApiServerBase_hpp

#include <functional>
#include "common/Common.hpp"
#include "VehicleApiBase.hpp"
#include "WorldSimApiBase.hpp"

namespace msr { namespace airlib {

class ApiServerBase {
public:
    virtual void start(bool block = false) = 0;
    virtual void stop() = 0;

    virtual const VehicleApiBase* getVehicleApi(const std::string& vehicle_name = "") const = 0;
    virtual VehicleApiBase* getVehicleApi(const std::string& vehicle_name = "")
    {
        return const_cast<VehicleApiBase*>(getVehicleApi(vehicle_name));
    }

    virtual const WorldSimApiBase* getWorldSimApi() const = 0;
    virtual WorldSimApiBase* getWorldSimApi()
    {
        return const_cast<WorldSimApiBase*>(getWorldSimApi());
    }

    virtual const VehicleSimApiBase* getVehicleSimApi(const std::string& vehicle_name = "") const = 0;
    virtual VehicleSimApiBase* getVehicleSimApi(const std::string& vehicle_name = "")
    {
        auto* world_sim_api = getWorldSimApi();
        if (world_sim_api) {
            auto* vehicle_sim_api = getWorldSimApi()->getVehicleSimApi(vehicle_name);
            return const_cast<VehicleSimApiBase*>(vehicle_sim_api);
        }
        return nullptr;
    }


    virtual ~ApiServerBase() = default;
};

}} //namespace
#endif
