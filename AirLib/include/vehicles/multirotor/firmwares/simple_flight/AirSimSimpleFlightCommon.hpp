// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_AirSimSimpleFlightCommon_hpp
#define msr_airlib_AirSimSimpleFlightCommon_hpp

#include "physics/Kinematics.hpp"
#include "common/Common.hpp"

namespace msr { namespace airlib {


class AirSimSimpleFlightCommon {
public:
    static simple_flight::Axis3r toAxis3r(const Vector3r& vec)
    {
        simple_flight::Axis3r conv;
        conv.x() = vec.x(); conv.y() = vec.y(); conv.z() = vec.z();

        return conv;
    }

    static Vector3r toVector3r(const simple_flight::Axis3r& vec)
    {
        Vector3r conv;
        conv.x() = vec.x(); conv.y() = vec.y(); conv.z() = vec.z();
        return conv;
    }

    static Kinematics::State toKinematicsState3r(const simple_flight::KinematicsState& state)
    {
        Kinematics::State state3r;
        state3r.pose.position = toVector3r(state.position);
        state3r.pose.orientation = toQuaternion(state.orientation);
        state3r.twist.linear = toVector3r(state.linear_velocity);
        state3r.twist.angular = toVector3r(state.angular_velocity);
        state3r.accelerations.linear = toVector3r(state.linear_acceleration);
        state3r.accelerations.angular = toVector3r(state.angular_acceleration);


        return state3r;
    }

    static simple_flight::Axis4r toAxis4r(const Quaternionr& q)
    {
        simple_flight::Axis4r conv;
        conv.x() = q.x(); conv.y() = q.y(); conv.z() = q.z();
        conv.val4() = q.w();

        return conv;
    }

    static Quaternionr toQuaternion(const simple_flight::Axis4r& q)
    {
        Quaternionr conv;
        conv.x() = q.x(); conv.y() = q.y(); conv.z() = q.z();
        conv.w() = q.val4();
        return conv;
    }

    static simple_flight::GeoPoint toSimpleFlightGeoPoint(const GeoPoint& geo_point)
    {
        simple_flight::GeoPoint conv;
        conv.latitude = geo_point.latitude;
        conv.longitude = geo_point.longitude;
        conv.altiude = geo_point.altitude;

        return conv;
    }

    static GeoPoint toGeoPoint(const simple_flight::GeoPoint& geo_point)
    {
        GeoPoint conv;
        conv.latitude = geo_point.latitude;
        conv.longitude = geo_point.longitude;
        conv.altitude = geo_point.altiude;

        return conv;
    }

    template<typename T> const T& makeConstant(T& _) 
    { 
        return const_cast<const T&>(_); 
    } 
    template<typename T> T& makeVariable(const T& _) 
    { 
        return const_cast<T&>(_); 
    }
};


}} //namespace
#endif
