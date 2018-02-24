// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "sensors/SensorFactory.hpp"
#include <memory>
#include "NedTransform.h"
#include "GameFramework/Actor.h"


namespace msr { namespace airlib {


class UnrealSensorFactory : public msr::airlib::SensorFactory {
public:
    UnrealSensorFactory(AActor* actor, const NedTransform* ned_transform);
    void setActor(AActor* actor, const NedTransform* ned_transform);
    virtual std::unique_ptr<SensorBase> createSensor(SensorBase::SensorType sensor_type) const override;

private:
    AActor* actor_;
    const NedTransform* ned_transform_;
};


}} //namespace