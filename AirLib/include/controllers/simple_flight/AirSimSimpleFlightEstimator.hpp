// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_AirSimSimpleFlightEstimator_hpp
#define msr_airlib_AirSimSimpleFlightEstimator_hpp

#include "firmware/CommonStructs.hpp"
#include "physics/Kinematics.hpp"
#include "common/Common.hpp"

namespace msr { namespace airlib {


class AirSimSimpleFlightEstimator : public simple_flight::IAngleEstimator {
public:
    //for now we don't do any state estimation and use ground truth (i.e. assume perfect sensors)
    void setKinematics(const Kinematics::State* kinematics)
    {
        kinematics_ = kinematics;
    }

    virtual simple_flight::Angles getAngles() const override
    {
        simple_flight::Angles angles;
        VectorMath::toEulerianAngle(kinematics_->pose.orientation.conjugate(),
            angles.pitch, angles.roll, angles.yaw);

        return angles;
    }

private:
    const Kinematics::State* kinematics_;
};


}} //namespace
#endif
