// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_ApiProvider_hpp
#define air_ApiProvider_hpp

#include "VehicleApiBase.hpp"
#include "VehicleSimApiBase.hpp"
#include "WorldSimApiBase.hpp"
#include <map>


namespace msr { namespace airlib {

class ApiProvider {
public:
    ApiProvider(WorldSimApiBase * world_sim_api)
        : world_sim_api_(world_sim_api)
    {
    }

    //vehicle API
    virtual VehicleApiBase* getVehicleApi(const std::string& vehicle_name = "")
    {
        return Utils::findOrDefault(vehicle_apis_, vehicle_name, 
            static_cast<VehicleApiBase*>(nullptr));
    }

    //world simulation API
    virtual WorldSimApiBase* getWorldSimApi()
    {
        return world_sim_api_;
    }

    //vehicle simulation API
    virtual VehicleSimApiBase* getVehicleSimApi(const std::string& vehicle_name = "")
    {
        return Utils::findOrDefault(vehicle_sim_apis_, vehicle_name,
            static_cast<VehicleSimApiBase*>(nullptr));
    }

    size_t getVehicleCount() const
    {
        return vehicle_apis_.size();
    }
    void insert_or_assign(const std::string& vehicle_name, VehicleApiBase* vehicle_api, 
        VehicleSimApiBase* vehicle_sim_api)
    {
        vehicle_apis_[vehicle_name] = std::move(vehicle_api);
        vehicle_sim_apis_[vehicle_name] = std::move(vehicle_sim_api);
    }
    const std::map<std::string, VehicleApiBase*>& getVehicleApis()
    {
        return vehicle_apis_;
    }
    const std::map<std::string, VehicleSimApiBase*>& geVehicleSimApis()
    {
        return vehicle_sim_apis_;
    }

    bool hasDefaultVehicle() const
    {
        return Utils::findOrDefault<std::string, VehicleApiBase*>(vehicle_apis_, "") == nullptr &&
            Utils::findOrDefault<std::string, VehicleSimApiBase*>(vehicle_sim_apis_, "") == nullptr;
    }

    void makeDefaultVehicle(const std::string& vehicle_name)
    {
        vehicle_apis_[""] = vehicle_apis_[vehicle_name];
        vehicle_sim_apis_[""] = vehicle_sim_apis_[vehicle_name];
    }

private:
    WorldSimApiBase* world_sim_api_;

    std::map<std::string, VehicleApiBase*> vehicle_apis_;
    std::map<std::string, VehicleSimApiBase*> vehicle_sim_apis_;
};

}} //namespace
#endif