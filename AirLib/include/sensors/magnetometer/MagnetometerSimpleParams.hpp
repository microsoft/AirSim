// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_MagnetometerSimpleParams_hpp
#define msr_airlib_MagnetometerSimpleParams_hpp

#include "common/Common.hpp"
#include "common/AirSimSettings.hpp"


namespace msr { namespace airlib {


struct MagnetometerSimpleParams {
    enum ReferenceSource {
        ReferenceSource_Constant,
        ReferenceSource_DipoleModel
    };

    Vector3r noise_sigma = Vector3r(0.005f, 0.005f, 0.005f); //5 mgauss as per specs sheet (RMS is same as stddev) https://goo.gl/UOz6FT
    real_T scale_factor = 1.0f;
    Vector3r noise_bias = Vector3r(0.0f, 0.0f, 0.0f); //no offset as per specsheet (zero gauss level) https://goo.gl/UOz6FT
    float ref_update_frequency = 0.2f;    //Hz

    //use dipole model if there is enough compute power available
    bool dynamic_reference_source = true;
    ReferenceSource ref_source = ReferenceSource::ReferenceSource_DipoleModel;
    //bool dynamic_reference_source = false;
    //ReferenceSource ref_source = ReferenceSource::ReferenceSource_Constant;

    //see PX4 param reference for EKF: https://dev.px4.io/en/advanced/parameter_reference.html
    real_T update_latency = 0.0f;    //sec: from PX4 doc
    real_T update_frequency = 50;    //Hz
    real_T startup_delay = 0;        //sec

    Pose relative_pose;

    void initializeFromSettings(const AirSimSettings::MagnetometerSetting& settings)
    {
        noise_sigma = settings.noise_sigma;
        noise_bias = settings.noise_bias;
        scale_factor = settings.scale_factor;
        dynamic_reference_source = settings.dynamic_reference_source;
        ref_update_frequency = settings.update_frequency;

        if (settings.ref_source == 0){
            ref_source = ReferenceSource::ReferenceSource_Constant;
        }

        update_latency = settings.update_latency;
        update_frequency = settings.update_frequency;
        startup_delay = settings.startup_delay;

        relative_pose.position = settings.position;
        if (std::isnan(relative_pose.position.x()))
            relative_pose.position.x() = 0;
        if (std::isnan(relative_pose.position.y()))
            relative_pose.position.y() = 0;
        if (std::isnan(relative_pose.position.z()))
	    relative_pose.position.z() = 0;

        float pitch, roll, yaw;
        pitch = !std::isnan(settings.rotation.pitch) ? settings.rotation.pitch : 0;
        roll = !std::isnan(settings.rotation.roll) ? settings.rotation.roll : 0;
        yaw = !std::isnan(settings.rotation.yaw) ? settings.rotation.yaw : 0;
        relative_pose.orientation = VectorMath::toQuaternion(
            Utils::degreesToRadians(pitch),   //pitch - rotation around Y axis
            Utils::degreesToRadians(roll),    //roll  - rotation around X axis
            Utils::degreesToRadians(yaw)      //yaw   - rotation around Z axis
        );
    }
};


}} //namespace
#endif
