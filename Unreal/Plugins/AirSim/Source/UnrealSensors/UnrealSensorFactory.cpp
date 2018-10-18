// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#include "UnrealSensorFactory.h"
#include "UnrealSensors/UnrealDistanceSensor.h"
#include "UnrealSensors/UnrealLidarSensor.h"

UnrealSensorFactory::UnrealSensorFactory(AActor* actor, const NedTransform* ned_transform)
{
    setActor(actor, ned_transform);
}


std::unique_ptr<msr::airlib::SensorBase> UnrealSensorFactory::createSensor(msr::airlib::SensorBase::SensorType sensor_type) const
{
    using SensorBase = msr::airlib::SensorBase;

    switch (sensor_type) {
    case SensorBase::SensorType::Distance:
        return std::unique_ptr<UnrealDistanceSensor>(new UnrealDistanceSensor(NULL /*settings*/, actor_, ned_transform_));
    case SensorBase::SensorType::Lidar:
        return std::unique_ptr<UnrealLidarSensor>(new UnrealLidarSensor(NULL /*settings*/, actor_, ned_transform_));
    default:
        return msr::airlib::SensorFactory::createSensor(sensor_type);
    }
}

std::unique_ptr<msr::airlib::SensorBase> UnrealSensorFactory::createSensorFromSettings(
    AirSimSettings::SensorSetting* sensor_setting) const
{
    using SensorBase = msr::airlib::SensorBase;

    switch (sensor_setting->sensor_type) {
    case SensorBase::SensorType::Distance:
        return std::unique_ptr<UnrealDistanceSensor>(new UnrealDistanceSensor(sensor_setting, actor_, ned_transform_));
    case SensorBase::SensorType::Lidar:
        return std::unique_ptr<UnrealLidarSensor>(new UnrealLidarSensor(sensor_setting, actor_, ned_transform_));
    default:
        return msr::airlib::SensorFactory::createSensorFromSettings(sensor_setting);
    }
}

void UnrealSensorFactory::setActor(AActor* actor, const NedTransform* ned_transform)
{
    actor_ = actor;
    ned_transform_ = ned_transform;
}



