// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#include "UnrealSensorFactory.h"
#include "UnrealSensors/UnrealDistanceSensor.h"
#include "UnrealSensors/UnrealLidarSensor.h"

UnrealSensorFactory::UnrealSensorFactory(AActor* actor, const NedTransform* ned_transform)
{
    setActor(actor, ned_transform);
}

std::shared_ptr<msr::airlib::SensorBase> UnrealSensorFactory::createSensorFromSettings(
    const AirSimSettings::SensorSetting* sensor_setting,
    const std::string& vehicle_type) const
{
    using SensorBase = msr::airlib::SensorBase;

    switch (sensor_setting->sensor_type) {
    case SensorBase::SensorType::Distance:
        return std::make_shared<UnrealDistanceSensor>(
            *static_cast<const AirSimSettings::DistanceSetting*>(sensor_setting), actor_, ned_transform_, vehicle_type);
    case SensorBase::SensorType::Lidar:
        return std::make_shared<UnrealLidarSensor>(
            *static_cast<const AirSimSettings::LidarSetting*>(sensor_setting), actor_, ned_transform_, vehicle_type);
    default:
        return msr::airlib::SensorFactory::createSensorFromSettings(sensor_setting, vehicle_type);
    }
}

void UnrealSensorFactory::setActor(AActor* actor, const NedTransform* ned_transform)
{
    actor_ = actor;
    ned_transform_ = ned_transform;
}
