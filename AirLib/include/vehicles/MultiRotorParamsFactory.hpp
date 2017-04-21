// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_MultiRotorParamsFactory_hpp
#define msr_airlib_vehicles_MultiRotorParamsFactory_hpp

#include "vehicles/configs/RosFlightQuadX.hpp"
#include "controllers/MavLinkDroneController.hpp"
#include "controllers/Settings.hpp"
#include "vehicles/configs/PX4QuadX.hpp"


namespace msr { namespace airlib {

class MultiRotorParamsFactory {
public:
    static std::unique_ptr<MultiRotorParams> createConfig(const std::string& vehicle_name)
    {
        //read settings and override defaults
        Settings& settings = Settings::singleton();
        if (!settings.isLoadSuccess())
            throw std::runtime_error("Cannot load settings.json");

        Settings child;
        settings.getChild(vehicle_name, child);

        std::unique_ptr<MultiRotorParams> config;

        if (vehicle_name == "Pixhawk") {
            config.reset(new Px4QuadX(child));
        } else if (vehicle_name == "RosFlight") {
            config.reset(new RosFlightQuadX(child));
        } else
            throw std::runtime_error(Utils::stringf("Cannot create vehicle config because vehicle name '%s' is not recognized", vehicle_name.c_str()));

        return config;
    }
};

}} //namespace
#endif
