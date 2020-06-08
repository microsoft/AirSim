// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "UnrealLidarSensor.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "NedTransform.h"
#include "DrawDebugHelpers.h"

// ctor
UnrealLidarSensor::UnrealLidarSensor(const AirSimSettings::LidarSetting& setting,
    AActor* actor, const NedTransform* ned_transform)
    : LidarSimple(setting), actor_(actor), ned_transform_(ned_transform)
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
    float delta_angle = 0;
    if (number_of_lasers > 1)
        delta_angle = (params.vertical_FOV_upper - (params.vertical_FOV_lower)) /
            static_cast<float>(number_of_lasers - 1);

    // store vertical angles for each laser
    laser_angles_.clear();
    for (auto i = 0u; i < number_of_lasers; ++i)
    {
        const float vertical_angle = params.vertical_FOV_upper - static_cast<float>(i) * delta_angle;
        laser_angles_.emplace_back(vertical_angle);
    }
}

// returns a point-cloud for the tick
void UnrealLidarSensor::getPointCloud(const msr::airlib::Pose& lidar_pose, const msr::airlib::Pose& vehicle_pose,
    const msr::airlib::TTimeDelta delta_time, msr::airlib::vector<msr::airlib::real_T>& point_cloud, msr::airlib::vector<int>& segmentation_cloud)
{
    point_cloud.clear();
    segmentation_cloud.clear();

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

    // normalize FOV start/end
    const float laser_start = std::fmod(360.0f + params.horizontal_FOV_start, 360.0f);
    const float laser_end = std::fmod(360.0f + params.horizontal_FOV_end, 360.0f);

    // shoot lasers
    for (auto laser = 0u; laser < number_of_lasers; ++laser)
    {
        const float vertical_angle = laser_angles_[laser];

        for (auto i = 0u; i < points_to_scan_with_one_laser; ++i)
        {
            const float horizontal_angle = std::fmod(current_horizontal_angle_ + angle_distance_of_laser_measure * i, 360.0f);

            // check if the laser is outside the requested horizontal FOV
            if (!VectorMath::isAngleBetweenAngles(horizontal_angle, laser_start, laser_end))
                continue;
       
            Vector3r point;
            int segmentationID = -1;
            // shoot laser and get the impact point, if any
            if (shootLaser(lidar_pose, vehicle_pose, laser, horizontal_angle, vertical_angle, params, point, segmentationID))
            {
                point_cloud.emplace_back(point.x());
                point_cloud.emplace_back(point.y());
                point_cloud.emplace_back(point.z());
                segmentation_cloud.emplace_back(segmentationID);
            }
        }
    }

    current_horizontal_angle_ = std::fmod(current_horizontal_angle_ + angle_distance_of_tick, 360.0f);

    return;
}

// simulate shooting a laser via Unreal ray-tracing.
bool UnrealLidarSensor::shootLaser(const msr::airlib::Pose& lidar_pose, const msr::airlib::Pose& vehicle_pose,
    const uint32 laser, const float horizontal_angle, const float vertical_angle, 
    const msr::airlib::LidarSimpleParams params, Vector3r &point, int &segmentationID)
{
    // start position
    Vector3r start = VectorMath::add(lidar_pose, vehicle_pose).position;

    // We need to compose rotations here rather than rotate a vector by a quaternion
    // Hence using coordOrientationAdd(..) rather than rotateQuaternion(..)

    // get ray quaternion in lidar frame (angles must be in radians)
    msr::airlib::Quaternionr ray_q_l = msr::airlib::VectorMath::toQuaternion(
        msr::airlib::Utils::degreesToRadians(vertical_angle),   //pitch - rotation around Y axis
        0,                                                      //roll  - rotation around X axis
        msr::airlib::Utils::degreesToRadians(horizontal_angle));//yaw   - rotation around Z axis

    // get ray quaternion in body frame
    msr::airlib::Quaternionr ray_q_b = VectorMath::coordOrientationAdd(ray_q_l, lidar_pose.orientation);

    // get ray quaternion in world frame
    msr::airlib::Quaternionr ray_q_w = VectorMath::coordOrientationAdd(ray_q_b, vehicle_pose.orientation);

    // get ray vector (end position)
    Vector3r end = VectorMath::rotateVector(VectorMath::front(), ray_q_w, true) * params.range + start;
   
    FHitResult hit_result = FHitResult(ForceInit);
    bool is_hit = UAirBlueprintLib::GetObstacle(actor_, ned_transform_->fromLocalNed(start), ned_transform_->fromLocalNed(end), hit_result, actor_, ECC_Visibility);

    if (is_hit)
    {
        //Store the segmentation id of the hit object.
        auto hitActor = hit_result.GetActor();
        if (hitActor != nullptr)
        {
            TArray<UMeshComponent*> meshComponents;
            hitActor->GetComponents<UMeshComponent>(meshComponents);
            for (int i = 0; i < meshComponents.Num(); i++)
            {
                segmentationID = segmentationID == -1 ? meshComponents[i]->CustomDepthStencilValue : segmentationID;
            }
        }

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

        // decide the frame for the point-cloud
        if (params.data_frame == AirSimSettings::kVehicleInertialFrame) {
            // current detault behavior; though it is probably not very useful.
            // not changing the default for now to maintain backwards-compat.
            point = ned_transform_->toLocalNed(hit_result.ImpactPoint);
        }
        else if (params.data_frame == AirSimSettings::kSensorLocalFrame) {
            // point in vehicle intertial frame
            Vector3r point_v_i = ned_transform_->toLocalNed(hit_result.ImpactPoint);

            // tranform to lidar frame
            point = VectorMath::transformToBodyFrame(point_v_i, lidar_pose + vehicle_pose, true);

            // The above should be same as first transforming to vehicle-body frame and then to lidar frame
            //    Vector3r point_v_b = VectorMath::transformToBodyFrame(point_v_i, vehicle_pose, true);
            //    point = VectorMath::transformToBodyFrame(point_v_b, lidar_pose, true);

            // On the client side, if it is needed to transform this data back to the world frame,
            // then do the equivalent of following,
            //     Vector3r point_w = VectorMath::transformToWorldFrame(point, lidar_pose + vehicle_pose, true);
            // See SimModeBase::drawLidarDebugPoints()

            // TODO: Optimization -- instead of doing this for every point, it should be possible to do this
            // for the point-cloud together? Need to look into matrix operations to do this together for all points.
        }
        else 
            throw std::runtime_error("Unknown requested data frame");

        return true;
    }
    else 
    {
        return false;
    }
}
