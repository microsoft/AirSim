// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_GpsSimpleParams_hpp
#define msr_airlib_GpsSimpleParams_hpp

#include "common/Common.hpp"
#include "common/AirSimSettings.hpp"

namespace msr { namespace airlib {

struct GpsSimpleParams {
    real_T eph_time_constant = 0.9f, epv_time_constant = 0.9f;
    real_T eph_initial = 100.0f, epv_initial = 100.0f;   //initially fully diluted positions
    real_T eph_final = 0.3f, epv_final = 0.4f;
    real_T eph_min_3d = 3.0f, eph_min_2d = 4.0f;

    real_T update_latency = 0.2f;    //sec
    real_T update_frequency = 50;    //Hz
    real_T startup_delay = 1;        //sec

    Pose relative_pose;

    void initializeFromSettings(const AirSimSettings::GpsSetting& settings)
    {
	eph_time_constant = settings.eph_time_constant;
	epv_time_constant = settings.epv_time_constant;
	eph_initial = settings.eph_initial;
	epv_initial = settings.epv_initial;
	eph_final = settings.eph_final;
	epv_final = settings.epv_final;
	eph_min_3d = settings.eph_min_3d;
	eph_min_2d = settings.eph_min_2d;

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
            Utils::degreesToRadians(yaw));    //yaw   - rotation around Z axis
    }
};

}} //namespace
#endif
