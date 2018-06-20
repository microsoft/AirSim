// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "CoreMinimal.h"
#include "sensors/SensorFactory.hpp"
#include <memory>
#include "NedTransform.h"
#include "GameFramework/Actor.h"

class UnrealSensorFactory : public msr::airlib::SensorFactory {
public:
    UnrealSensorFactory(AActor* actor, const NedTransform* ned_transform);
    void setActor(AActor* actor, const NedTransform* ned_transform);
    virtual std::unique_ptr<msr::airlib::SensorBase> createSensor(msr::airlib::SensorBase::SensorType sensor_type) const override;

private:
    AActor* actor_;
    const NedTransform* ned_transform_;
};
