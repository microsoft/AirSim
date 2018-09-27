// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "common/Common.hpp"
#include "GameFramework/Actor.h"
#include "sensors/Lidar/LidarSimple.hpp"
#include "NedTransform.h"

// UnrealLidarSensor implementation that uses Ray Tracing in Unreal
// The implementation uses a model similar to CARLA Lidar implementation.
// Thanks to CARLA folks for this.
class UnrealLidarSensor : public msr::airlib::LidarSimple {
public:
    UnrealLidarSensor(AActor* actor, const NedTransform* ned_transform);

protected:
    virtual msr::airlib::vector<msr::airlib::real_T> getPointCloud(const msr::airlib::Pose& pose, float deltaTime) override;

private:
    using Vector3r = msr::airlib::Vector3r;
    using VectorMath = msr::airlib::VectorMath;

    void CreateLasers();
    bool ShootLaser(const msr::airlib::Pose& pose, uint32 channel, float horizontalAngle, 
        msr::airlib::LidarSimpleParams params, Vector3r &point);

private:
    AActor* actor_;
    const NedTransform* ned_transform_;

    TArray<float> laser_angles_;
    float currentHorizontalAngle_;

    msr::airlib::vector<msr::airlib::real_T> points_;
};