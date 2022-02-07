// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "common/Common.hpp"
#include "GameFramework/Actor.h"
#include "sensors/distance/DistanceSimple.hpp"
#include "NedTransform.h"

class UnrealDistanceSensor : public msr::airlib::DistanceSimple
{
public:
    typedef msr::airlib::AirSimSettings AirSimSettings;

public:
    UnrealDistanceSensor(const AirSimSettings::DistanceSetting& setting,
                         AActor* actor, const NedTransform* ned_transform,
                         const std::string& vehicle_type);

protected:
    virtual msr::airlib::real_T getRayLength(const msr::airlib::Pose& pose) override;

private:
    using Vector3r = msr::airlib::Vector3r;
    using VectorMath = msr::airlib::VectorMath;

private:
    AActor* actor_;
    const NedTransform* ned_transform_;
};