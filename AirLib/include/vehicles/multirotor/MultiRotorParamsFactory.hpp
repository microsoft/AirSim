// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_MultiRotorParamsFactory_hpp
#define msr_airlib_vehicles_MultiRotorParamsFactory_hpp

#include "vehicles/multirotor/configs/Px4MultiRotor.hpp"
#include "vehicles/multirotor/controllers/MavLinkDroneController.hpp"
#include "vehicles/multirotor/configs/RosFlightQuadX.hpp"
#include "vehicles/multirotor/configs/SimpleFlightQuadX.hpp"
#include "common/AirSimSettings.hpp"


namespace msr { namespace airlib {

class MultiRotorParamsFactory {
public:
    static std::unique_ptr<MultiRotorParams> createConfig(const std::string& vehicle_name, std::shared_ptr<const SensorFactory> sensor_factory)
    {
        AirSimSettings::VehicleSettings vehicle_settings = 
            AirSimSettings::singleton().getVehicleSettings(vehicle_name);

        std::unique_ptr<MultiRotorParams> config;

        if (vehicle_settings.firmware_name == "PX4") {
            config.reset(new Px4MultiRotor(vehicle_settings, sensor_factory));
        } else if (vehicle_settings.firmware_name == "RosFlight") {
            config.reset(new RosFlightQuadX(vehicle_settings, sensor_factory));
        } else if (vehicle_settings.firmware_name == "SimpleFlight") {
            config.reset(new SimpleFlightQuadX(vehicle_settings, sensor_factory));
        } else
            throw std::runtime_error(Utils::stringf(
                "Cannot create vehicle config because vehicle name '%s' is not recognized", 
                vehicle_name.c_str()));

        config->initialize();
        
        config->getParams().api_server_port = static_cast<uint16_t>(vehicle_settings.server_port);

        return config;
    }
};

}} //namespace
#endif
