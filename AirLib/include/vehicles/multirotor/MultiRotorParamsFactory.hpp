// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_MultiRotorParamsFactory_hpp
#define msr_airlib_vehicles_MultiRotorParamsFactory_hpp

#include "vehicles/multirotor/configs/Px4MultiRotor.hpp"
#include "vehicles/multirotor/controllers/MavLinkDroneController.hpp"
#include "vehicles/multirotor/configs/RosFlightQuadX.hpp"
#include "vehicles/multirotor/configs/SimpleFlightQuadX.hpp"
#include "controllers/Settings.hpp"


namespace msr { namespace airlib {

class MultiRotorParamsFactory {
public:
    static std::unique_ptr<MultiRotorParams> createConfig(const std::string& vehicle_name)
    {
        //read settings and override defaults
        Settings& settings = Settings::singleton();
        Settings vehicle_config_settings;
        settings.getChild(vehicle_name, vehicle_config_settings);

        std::string firmware_name = vehicle_config_settings.getString("FirmwareName", vehicle_name);

        std::unique_ptr<MultiRotorParams> config;

        if (firmware_name == "PX4") {
            config.reset(new Px4MultiRotor(vehicle_config_settings));
        } else if (firmware_name == "RosFlight") {
            config.reset(new RosFlightQuadX(vehicle_config_settings));
        } else if (firmware_name == "SimpleFlight") {
            config.reset(new SimpleFlightQuadX(vehicle_config_settings));
        } else
            throw std::runtime_error(Utils::stringf("Cannot create vehicle config because vehicle name '%s' is not recognized", vehicle_name.c_str()));

        config->initialize();
        
        config->getParams().api_server_port = static_cast<uint16_t>(
            vehicle_config_settings.getInt("ApiServerPort", 41451));

        return config;
    }
};

}} //namespace
#endif
