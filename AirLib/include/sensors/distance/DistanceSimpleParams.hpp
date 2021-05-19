// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_DistanceSimpleParams_hpp
#define msr_airlib_DistanceSimpleParams_hpp

#include "common/Common.hpp"
#include "common/AirSimSettings.hpp"

namespace msr
{
namespace airlib
{

    struct DistanceSimpleParams
    {
        real_T min_distance = 20.0f / 100; //m
        real_T max_distance = 4000.0f / 100; //m

        Pose relative_pose{
            Vector3r(0, 0, -1), // position - a little above vehicle (especially for cars) or Vector3r::Zero()
            Quaternionr::Identity() // orientation - by default Quaternionr(1, 0, 0, 0)
        };

        bool draw_debug_points = false;
        bool external_controller = true;

        /*
    Ref: A Stochastic Approach to Noise Modeling for Barometric Altimeters
     Angelo Maria Sabatini* and Vincenzo Genovese
     Sample values are from Table 1
     https://www.ncbi.nlm.nih.gov/pmc/articles/PMC3871085/
     This is however not used because numbers mentioned in paper doesn't match experiments.

     real_T correlated_noise_sigma = 0.27f;
     real_T correlated_noise_tau = 0.87f;
     real_T uncorrelated_noise_sigma = 0.24f;

*/
        //TODO: update sigma based on documentation, maybe as a function increasing with measured distance
        real_T uncorrelated_noise_sigma = 0.002f * 100;
        //jMavSim uses below
        //real_T uncorrelated_noise_sigma = 0.1f;

        //see PX4 param reference for EKF: https://dev.px4.io/en/advanced/parameter_reference.html
        real_T update_latency = 0.0f; //sec
        real_T update_frequency = 50; //Hz
        real_T startup_delay = 0; //sec

        void initializeFromSettings(const AirSimSettings::DistanceSetting& settings)
        {
            const auto& settings_json = settings.settings;
            min_distance = settings_json.getFloat("MinDistance", min_distance);
            max_distance = settings_json.getFloat("MaxDistance", max_distance);
            draw_debug_points = settings_json.getBool("DrawDebugPoints", draw_debug_points);
            external_controller = settings_json.getBool("ExternalController", external_controller);

            auto position = AirSimSettings::createVectorSetting(settings_json, VectorMath::nanVector());
            auto rotation = AirSimSettings::createRotationSetting(settings_json, AirSimSettings::Rotation::nanRotation());

            std::string simmode_name = AirSimSettings::singleton().simmode_name;

            relative_pose.position = position;
            if (std::isnan(relative_pose.position.x()))
                relative_pose.position.x() = 0;
            if (std::isnan(relative_pose.position.y()))
                relative_pose.position.y() = 0;
            if (std::isnan(relative_pose.position.z())) {
                if (simmode_name == AirSimSettings::kSimModeTypeMultirotor)
                    relative_pose.position.z() = 0;
                else
                    relative_pose.position.z() = -1; // a little bit above for cars
            }

            float pitch, roll, yaw;
            pitch = !std::isnan(rotation.pitch) ? rotation.pitch : 0;
            roll = !std::isnan(rotation.roll) ? rotation.roll : 0;
            yaw = !std::isnan(rotation.yaw) ? rotation.yaw : 0;
            relative_pose.orientation = VectorMath::toQuaternion(
                Utils::degreesToRadians(pitch), //pitch - rotation around Y axis
                Utils::degreesToRadians(roll), //roll  - rotation around X axis
                Utils::degreesToRadians(yaw)); //yaw   - rotation around Z axis
        }
    };
}
} //namespace
#endif
