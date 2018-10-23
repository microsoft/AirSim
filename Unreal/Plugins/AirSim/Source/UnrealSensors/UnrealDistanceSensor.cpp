// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "UnrealDistanceSensor.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "NedTransform.h"

UnrealDistanceSensor::UnrealDistanceSensor(const AirSimSettings::DistanceSetting& setting,
    AActor* actor, const NedTransform* ned_transform)
    : DistanceSimple(setting), actor_(actor), ned_transform_(ned_transform)
{
}

msr::airlib::real_T UnrealDistanceSensor::getRayLength(const msr::airlib::Pose& pose)
{
    //update ray tracing
    Vector3r start = pose.position;
    Vector3r end = start + VectorMath::rotateVector(VectorMath::front(), pose.orientation, true) * getParams().max_distance;

    FHitResult dist_hit = FHitResult(ForceInit);
    bool is_hit = UAirBlueprintLib::GetObstacle(actor_, ned_transform_->fromLocalNed(start), ned_transform_->fromLocalNed(end), dist_hit);
    float distance = is_hit? dist_hit.Distance / 100.0f : getParams().max_distance;

    //FString hit_name = FString("None");
    //if (dist_hit.GetActor())
    //    hit_name=dist_hit.GetActor()->GetName();

    //UAirBlueprintLib::LogMessage(FString("Distance to "), hit_name+FString(": ")+FString::SanitizeFloat(distance), LogDebugLevel::Informational);

    return distance;
}
