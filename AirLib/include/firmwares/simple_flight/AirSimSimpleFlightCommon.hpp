// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_AirSimSimpleFlightCommon_hpp
#define msr_airlib_AirSimSimpleFlightCommon_hpp

#include "physics/Kinematics.hpp"
#include "common/Common.hpp"

namespace msr { namespace airlib {


class AirSimSimpleFlightCommon {
public:
    static simple_flight::Axis3r vector3rToAxis3r(const Vector3r& vec)
    {
        simple_flight::Axis3r conv;
        conv.x() = vec.x(); conv.y() = vec.y(); conv.z() = vec.z();

        return conv;
    }

    static Vector3r axis3rToVector3r(const simple_flight::Axis3r& vec)
    {
        Vector3r conv;
        conv.x() = vec.x(); conv.y() = vec.y(); conv.z() = vec.z();
        return conv;
    }

    static simple_flight::Axis4r quaternion3rToAxis4r(const Quaternionr& q)
    {
        simple_flight::Axis4r conv;
        conv.axis3.x() = q.x(); conv.axis3.y() = q.y(); conv.axis3.z() = q.z();
        conv.val4 = q.w();

        return conv;
    }

    static Quaternionr axis4rToQuaternionr(const simple_flight::Axis4r& q)
    {
        Quaternionr conv;
        conv.x() = q.axis3.x(); conv.y() = q.axis3.y(); conv.z() = q.axis3.z();
        conv.w() = q.val4;
        return conv;
    }
};


}} //namespace
#endif
