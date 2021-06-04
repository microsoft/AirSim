// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_ApiProvider_hpp
#define air_ApiProvider_hpp

#include "VehicleApiBase.hpp"
#include "VehicleSimApiBase.hpp"
#include "WorldSimApiBase.hpp"
#include <map>
#include "common/common_utils/UniqueValueMap.hpp"

namespace msr
{
namespace airlib
{

    class ApiProvider
    {
    public:
        ApiProvider(WorldSimApiBase* world_sim_api)
            : world_sim_api_(world_sim_api)
        {
        }
        virtual ~ApiProvider() = default;

        //vehicle API
        virtual VehicleApiBase* getVehicleApi(const std::string& vehicle_name)
        {
            return vehicle_apis_.findOrDefault(vehicle_name, nullptr);
        }

        //world simulation API
        virtual WorldSimApiBase* getWorldSimApi()
        {
            return world_sim_api_;
        }

        //vehicle simulation API
        virtual VehicleSimApiBase* getVehicleSimApi(const std::string& vehicle_name) const
        {
            return vehicle_sim_apis_.findOrDefault(vehicle_name, nullptr);
        }

        size_t getVehicleCount() const
        {
            return vehicle_apis_.valsSize();
        }
        void insert_or_assign(const std::string& vehicle_name, VehicleApiBase* vehicle_api,
                              VehicleSimApiBase* vehicle_sim_api)
        {
            vehicle_apis_.insert_or_assign(vehicle_name, vehicle_api);
            vehicle_sim_apis_.insert_or_assign(vehicle_name, vehicle_sim_api);
        }
        const common_utils::UniqueValueMap<std::string, VehicleApiBase*>& getVehicleApis()
        {
            return vehicle_apis_;
        }
        const common_utils::UniqueValueMap<std::string, VehicleSimApiBase*>& getVehicleSimApis()
        {
            return vehicle_sim_apis_;
        }
        bool hasDefaultVehicle() const
        {
            return !(vehicle_apis_.findOrDefault("", nullptr) == nullptr &&
                     vehicle_sim_apis_.findOrDefault("", nullptr) == nullptr);
        }

        void makeDefaultVehicle(const std::string& vehicle_name)
        {
            vehicle_apis_.insert_or_assign("", vehicle_apis_.at(vehicle_name));
            vehicle_sim_apis_.insert_or_assign("", vehicle_sim_apis_.at(vehicle_name));
        }

    private:
        WorldSimApiBase* world_sim_api_;

        common_utils::UniqueValueMap<std::string, VehicleApiBase*> vehicle_apis_;
        common_utils::UniqueValueMap<std::string, VehicleSimApiBase*> vehicle_sim_apis_;
    };
}
} //namespace
#endif