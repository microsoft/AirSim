// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "UnrealSensorFactory.h"
#include "UnrealSensors/UnrealDistanceSensor.h"


UnrealSensorFactory::UnrealSensorFactory(AActor* actor, const NedTransform* ned_transform)
{
    setActor(actor, ned_transform);
}


std::unique_ptr<msr::airlib::SensorBase> UnrealSensorFactory::createSensor(msr::airlib::SensorBase::SensorType sensor_type) const
{
    using SensorBase = msr::airlib::SensorBase;

    switch (sensor_type) {
    case SensorBase::SensorType::Distance:
        return std::unique_ptr<UnrealDistanceSensor>(new UnrealDistanceSensor(actor_, ned_transform_));
    default:
        return msr::airlib::SensorFactory::createSensor(sensor_type);
    }
}

void UnrealSensorFactory::setActor(AActor* actor, const NedTransform* ned_transform)
{
    actor_ = actor;
    ned_transform_ = ned_transform;
}



