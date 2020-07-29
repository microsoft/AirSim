// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_Px4MultiRotor_hpp
#define msr_airlib_vehicles_Px4MultiRotor_hpp

#include "vehicles/multirotor/firmwares/mavlink/MavLinkMultirotorApi.hpp"
#include "common/AirSimSettings.hpp"
#include "sensors/SensorFactory.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"

namespace msr { namespace airlib {

class Px4MultiRotorParams : public MultiRotorParams {
public:
    Px4MultiRotorParams(const AirSimSettings::MavLinkVehicleSetting& vehicle_setting, std::shared_ptr<const SensorFactory> sensor_factory)
        : sensor_factory_(sensor_factory)
    {
        connection_info_ = getConnectionInfo(vehicle_setting);
    }

    virtual ~Px4MultiRotorParams() = default;

    virtual std::unique_ptr<MultirotorApiBase> createMultirotorApi() override
    {
        unique_ptr<MultirotorApiBase> api(new MavLinkMultirotorApi());
        auto api_ptr = static_cast<MavLinkMultirotorApi*>(api.get());
        api_ptr->initialize(connection_info_, &getSensors(), true);

        return api;
    }

    virtual void setupParams() override
    {
        auto& params = getParams();

        if (connection_info_.model == "Blacksheep") {
            setupFrameBlacksheep(params);
        }
        else if (connection_info_.model == "Flamewheel") {
            setupFrameFlamewheel(params);
        }
        else if (connection_info_.model == "FlamewheelFLA") {
            setupFrameFlamewheelFLA(params);
        }
        else if (connection_info_.model == "Hexacopter") {
            setupFrameGenericHex(params);
        }
        else //Generic
            setupFrameGenericQuad(params);
    }

protected:
    virtual const SensorFactory* getSensorFactory() const override
    {
        return sensor_factory_.get();
    }

private:
    static const AirSimSettings::MavLinkConnectionInfo& getConnectionInfo(const AirSimSettings::MavLinkVehicleSetting& vehicle_setting)
    {
        return vehicle_setting.connection_info;
    }


private:
    AirSimSettings::MavLinkConnectionInfo connection_info_;
    std::shared_ptr<const SensorFactory> sensor_factory_;

};

}} //namespace
#endif
