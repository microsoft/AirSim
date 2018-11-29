// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_LidarSimpleParams_hpp
#define msr_airlib_LidarSimpleParams_hpp

#include "common/Common.hpp"
#include "common/AirSimSettings.hpp"

namespace msr { namespace airlib {

struct LidarSimpleParams {

    // Velodyne VLP-16 Puck config
    // https://velodynelidar.com/vlp-16.html

    // default settings
    // TODO: enable reading of these params from AirSim settings

    uint number_of_channels = 16; 
    real_T range = 10000.0f / 100;            // meters
    uint points_per_second = 100000;  
    uint horizontal_rotation_frequency = 10;  // rotations/sec
    real_T horizontal_FOV_start = 0;
    real_T horizontal_FOV_end = 359;
    real_T vertical_FOV_upper = -15;             // drones -15, car +10
    real_T vertical_FOV_lower = -45;             // drones -45, car -10

    Pose relative_pose {
        Vector3r(0,0,-1),                     // position - a little above vehicle (especially for cars) or Vector3r::Zero()
        Quaternionr::Identity()               // orientation - by default Quaternionr(1, 0, 0, 0) 
        };                       

    bool draw_debug_points = false;
    std::string data_frame = AirSimSettings::kVehicleInertialFrame;

    real_T update_frequency = 10;             // Hz
    real_T startup_delay = 0;                 // sec

    void initializeFromSettings(const AirSimSettings::LidarSetting& settings)
    {
        std::string simmode_name = AirSimSettings::singleton().simmode_name;

        number_of_channels = settings.number_of_channels;
        range = settings.range;
        points_per_second = settings.points_per_second;
        horizontal_rotation_frequency = settings.horizontal_rotation_frequency;

        horizontal_FOV_start = settings.horizontal_FOV_start;
        horizontal_FOV_end = settings.horizontal_FOV_end;

        // By default, for multirotors the lidars FOV point downwards;
        // for cars, the lidars FOV is more forward facing.
        vertical_FOV_upper = settings.vertical_FOV_upper;
        if (std::isnan(vertical_FOV_upper)) {
            if (simmode_name == "Multirotor")
                vertical_FOV_upper = -15;
            else
                vertical_FOV_upper = +10;
        }

        vertical_FOV_lower = settings.vertical_FOV_lower;
        if (std::isnan(vertical_FOV_lower)) {
            if (simmode_name == "Multirotor")
                vertical_FOV_lower = -45;
            else
                vertical_FOV_lower = -10;
        }

        relative_pose.position = settings.position;
        if (std::isnan(relative_pose.position.x()))
            relative_pose.position.x() = 0;
        if (std::isnan(relative_pose.position.y()))
            relative_pose.position.y() = 0;
        if (std::isnan(relative_pose.position.z())) {
            if (simmode_name == "Multirotor")
                relative_pose.position.z() = 0;
            else
                relative_pose.position.z() = -1;  // a little bit above for cars
        }

        float pitch, roll, yaw;
        pitch = !std::isnan(settings.rotation.pitch) ? settings.rotation.pitch : 0;
        roll = !std::isnan(settings.rotation.roll) ? settings.rotation.roll : 0;
        yaw = !std::isnan(settings.rotation.yaw) ? settings.rotation.yaw : 0;
        relative_pose.orientation = VectorMath::toQuaternion(
            Utils::degreesToRadians(pitch),   //pitch - rotation around Y axis
            Utils::degreesToRadians(roll),    //roll  - rotation around X axis
            Utils::degreesToRadians(yaw));    //yaw   - rotation around Z axis
           
        draw_debug_points = settings.draw_debug_points;
        data_frame = settings.data_frame;
    }
};

}} //namespace
#endif
