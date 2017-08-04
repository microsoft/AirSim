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

    static simple_flight::Axis4r toAxis4r(const Quaternionr& q)
    {
        simple_flight::Axis4r conv;
        conv.axis3.x() = q.x(); conv.axis3.y() = q.y(); conv.axis3.z() = q.z();
        conv.val4 = q.w();

        return conv;
    }

    static Quaternionr toQuaternion(const simple_flight::Axis4r& q)
    {
        Quaternionr conv;
        conv.x() = q.axis3.x(); conv.y() = q.axis3.y(); conv.z() = q.axis3.z();
        conv.w() = q.val4;
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
};


}} //namespace
#endif
