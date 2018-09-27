// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "UnrealLidarSensor.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "NedTransform.h"
#include "DrawDebugHelpers.h"

// ctor
UnrealLidarSensor::UnrealLidarSensor(AActor* actor, const NedTransform* ned_transform)
    : actor_(actor), ned_transform_(ned_transform)
{
    CreateLasers();
}

// initializes information based on lidar configuration
void UnrealLidarSensor::CreateLasers()
{
    msr::airlib::LidarSimpleParams params = getParams();

    const auto numberOfLasers = params.number_of_channels;
    check(numberOfLasers > 0u);

    // calculate verticle angle distance between each laser
    const float dtAngle =
        (params.vertical_FOV_Upper - (params.vertical_FOV_Lower)) /
        static_cast<float>(numberOfLasers - 1);

    // store vertical angles for each laser
    laser_angles_.Empty(numberOfLasers);
    for (auto i = 0u; i < numberOfLasers; ++i)
    {
        const float verticalAngle = 15 - static_cast<float>(i) * dtAngle;
        laser_angles_.Emplace(verticalAngle);
    }
}

// returns a point-cloud for the tick
msr::airlib::vector<msr::airlib::real_T> UnrealLidarSensor::getPointCloud(const msr::airlib::Pose& pose, const float deltaTime)
{
    points_.clear();

    msr::airlib::LidarSimpleParams params = getParams();

    const auto numberOfLasers = params.number_of_channels;

    // calculate number of points needed for each laser/channel
    const uint32 pointsToScanWithOneLaser =
        FMath::RoundHalfFromZero(params.points_per_second * deltaTime / float(numberOfLasers));
    if (pointsToScanWithOneLaser <= 0)
    {
        //UAirBlueprintLib::LogMessageString("Lidar: ", "No points requested this frame", LogDebugLevel::Failure);
        return points_;
    }

    // calculate needed angle/distance between each point
    const float angleDistanceOfTick = params.horizontal_rotation_frequency * 360.0f * deltaTime;
    const float angleDistanceOfLaserMeasure = angleDistanceOfTick / pointsToScanWithOneLaser;

    // shoot lasers
    for (auto laser = 0u; laser < numberOfLasers; ++laser)
    {
        for (auto i = 0u; i < pointsToScanWithOneLaser; ++i)
        {
            Vector3r point;
            const float angle = currentHorizontalAngle_ + angleDistanceOfLaserMeasure * i;
            // shoot laser and get the impact point, if any
            if (ShootLaser(pose, laser, angle, params, point))
            {
                points_.emplace_back(point.x());
                points_.emplace_back(point.y());
                points_.emplace_back(point.z());
            }
        }
    }

    currentHorizontalAngle_ = std::fmod(currentHorizontalAngle_ + angleDistanceOfTick, 360.0f);

    return points_;
}

// simulate shooting a laser via Unreal ray-tracing.
bool UnrealLidarSensor::ShootLaser(const msr::airlib::Pose& pose, const uint32 laser, const float horizontalAngle, 
    msr::airlib::LidarSimpleParams params, Vector3r &point)
{
    const float verticalAngle = laser_angles_[laser];

    // start position
    FVector start = ned_transform_->fromLocalNed(pose.position);

    // calculate rotation needed for this laser point
    FRotator laserRot(verticalAngle /*pitch - rotation around Y axis*/,
        horizontalAngle             /*yaw   - rotation around Z axis*/, 
        0                           /*roll  - rotation around X axis*/);  

    // calculate final rotation for this laser point w.r.t. lidar pose
    // there might be a better way of achieving this via AirSim VectorMath but this implementation should work as well
    FQuat lidarOrientation(pose.orientation.x(), pose.orientation.y(), pose.orientation.z(), pose.orientation.w());
    FRotator lidarRot = lidarOrientation.Rotator();
    FRotator resultRot = UKismetMathLibrary::ComposeRotators(laserRot, lidarRot);

    // calculate end position (params.range is expected to be in meters)
    constexpr float TO_CENTIMETERS = 1e+2f;
    FVector end = (params.range * TO_CENTIMETERS) * UKismetMathLibrary::GetForwardVector(resultRot) + start;

    FHitResult hit_result = FHitResult(ForceInit);
    bool is_hit = UAirBlueprintLib::GetObstacle(actor_, start, end, hit_result);

    if (is_hit)
    {
        if (false)
        {
            // Debug code.
            // don't enable this for multirotors since drawing a debug point
            // outside of Unreal tick will cause exception
            DrawDebugPoint(
                actor_->GetWorld(),
                hit_result.ImpactPoint,
                5,                       //size
                FColor::Green,
                true,                    //persistent (never goes away)
                0.1                      //point leaves a trail on moving object
            );
        }

        point = ned_transform_->toLocalNed(hit_result.ImpactPoint);
       
        return true;
    }
    else 
    {
        return false;
    }
}
