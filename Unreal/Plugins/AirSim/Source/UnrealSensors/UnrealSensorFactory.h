// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "sensors/SensorFactory.hpp"
#include <memory>
#include "GameFramework/Actor.h"


namespace msr { namespace airlib {


class UnrealSensorFactory : public msr::airlib::SensorFactory {
public:
    UnrealSensorFactory(AActor* actor);
    void setActor(AActor* actor);
    virtual std::unique_ptr<SensorBase> createSensor(SensorBase::SensorType sensor_type) const override;

private:
    AActor* actor_;
};


}} //namespace