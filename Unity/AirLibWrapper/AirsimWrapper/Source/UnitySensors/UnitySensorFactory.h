#pragma once

#include "sensors/SensorFactory.hpp"
#include "../NedTransform.h"

using namespace msr::airlib;

class UnitySensorFactory : public SensorFactory
{
public:
    UnitySensorFactory(std::string vehicle_name, const NedTransform* ned_transform);
    void setActor(std::string vehicle_name, const NedTransform* ned_transform);
    virtual std::shared_ptr<msr::airlib::SensorBase> createSensorFromSettings(const AirSimSettings::SensorSetting* sensor_setting, const std::string& vehicle_type) const override;

private:
    std::string vehicle_name_;
    const NedTransform* ned_transform_;
};
