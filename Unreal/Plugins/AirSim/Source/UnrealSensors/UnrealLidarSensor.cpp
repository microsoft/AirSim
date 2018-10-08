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
    createLasers();
}

// initializes information based on lidar configuration
void UnrealLidarSensor::createLasers()
{
    msr::airlib::LidarSimpleParams params = getParams();

    const auto number_of_lasers = params.number_of_channels;

    if (number_of_lasers <= 0)
        return;

    // calculate verticle angle distance between each laser
    const float delta_angle =
        (params.vertical_FOV_Upper - (params.vertical_FOV_Lower)) /
        static_cast<float>(number_of_lasers - 1);

    // store vertical angles for each laser
    laser_angles_.clear();
    for (auto i = 0u; i < number_of_lasers; ++i)
    {
        const float vertical_angle = params.vertical_FOV_Upper - static_cast<float>(i) * delta_angle;
        laser_angles_.emplace_back(vertical_angle);
    }
}

// returns a point-cloud for the tick
void UnrealLidarSensor::getPointCloud(const msr::airlib::Pose& lidar_pose, const msr::airlib::Pose& vehicle_pose,
    const msr::airlib::TTimeDelta delta_time, msr::airlib::vector<msr::airlib::real_T>& point_cloud)
{
    point_cloud.clear();

    msr::airlib::LidarSimpleParams params = getParams();
    const auto number_of_lasers = params.number_of_channels;

    // cap the points to scan via ray-tracing; this is currently needed for car/Unreal tick scenarios
    // since SensorBase mechanism uses the elapsed clock time instead of the tick delta-time.
    constexpr float MAX_POINTS_IN_SCAN = 1e+5f;
    uint32 total_points_to_scan = FMath::RoundHalfFromZero(params.points_per_second * delta_time);
    if (total_points_to_scan > MAX_POINTS_IN_SCAN)
    {
        total_points_to_scan = MAX_POINTS_IN_SCAN;
        UAirBlueprintLib::LogMessageString("Lidar: ", "Capping number of points to scan", LogDebugLevel::Failure);
    }

    // calculate number of points needed for each laser/channel
    const uint32 points_to_scan_with_one_laser = FMath::RoundHalfFromZero(total_points_to_scan / float(number_of_lasers));
    if (points_to_scan_with_one_laser <= 0)
    {
        //UAirBlueprintLib::LogMessageString("Lidar: ", "No points requested this frame", LogDebugLevel::Failure);
        return;
    }

    // calculate needed angle/distance between each point
    const float angle_distance_of_tick = params.horizontal_rotation_frequency * 360.0f * delta_time;
    const float angle_distance_of_laser_measure = angle_distance_of_tick / points_to_scan_with_one_laser;

    // shoot lasers
    for (auto laser = 0u; laser < number_of_lasers; ++laser)
    {
        for (auto i = 0u; i < points_to_scan_with_one_laser; ++i)
        {
            Vector3r point;
            const float angle = current_horizontal_angle_ + angle_distance_of_laser_measure * i;
            // shoot laser and get the impact point, if any
            if (shootLaser(lidar_pose, vehicle_pose, laser, angle, params, point))
            {
                point_cloud.emplace_back(point.x());
                point_cloud.emplace_back(point.y());
                point_cloud.emplace_back(point.z());
            }
        }
    }

    current_horizontal_angle_ = std::fmod(current_horizontal_angle_ + angle_distance_of_tick, 360.0f);

    return;
}

// simulate shooting a laser via Unreal ray-tracing.
bool UnrealLidarSensor::shootLaser(const msr::airlib::Pose& lidar_pose, const msr::airlib::Pose& vehicle_pose,
    const uint32 laser, const float horizontalAngle, msr::airlib::LidarSimpleParams params, Vector3r &point)
{
    const float vertical_angle = laser_angles_[laser];

    // start position
    Vector3r start = lidar_pose.position + vehicle_pose.position;

    // get ray quaternion in lidar frame (angles must be in radians)
    msr::airlib::Quaternionr ray_q_l = msr::airlib::VectorMath::toQuaternion(
        msr::airlib::Utils::degreesToRadians(vertical_angle),   //pitch - rotation around Y axis
        0,                                                      //roll  - rotation around X axis
        msr::airlib::Utils::degreesToRadians(horizontalAngle)); //yaw   - rotation around Z axis

    // get ray quaternion in body frame
    msr::airlib::Quaternionr ray_q_b = VectorMath::rotateQuaternion(ray_q_l, lidar_pose.orientation, true);

    // get ray quaternion in world frame
    msr::airlib::Quaternionr ray_q_w = VectorMath::rotateQuaternion(ray_q_b, vehicle_pose.orientation, true);
    
    // get ray vector (end position)
    Vector3r end = VectorMath::rotateVector(VectorMath::front(), ray_q_w, true) * params.range + start ;
   
    FHitResult hit_result = FHitResult(ForceInit);
    bool is_hit = UAirBlueprintLib::GetObstacle(actor_, ned_transform_->fromLocalNed(start), ned_transform_->fromLocalNed(end), hit_result);

    if (is_hit)
    {
        if (false && UAirBlueprintLib::IsInGameThread())
        {
            // Debug code for very specific cases.
            // Mostly shouldn't be needed. Use SimModeBase::drawLidarDebugPoints()
            DrawDebugPoint(
                actor_->GetWorld(),
                hit_result.ImpactPoint,
                5,                       //size
                FColor::Red,
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
