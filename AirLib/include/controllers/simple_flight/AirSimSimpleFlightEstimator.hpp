// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_AirSimSimpleFlightEstimator_hpp
#define msr_airlib_AirSimSimpleFlightEstimator_hpp

#include "firmware/interfaces/CommonStructs.hpp"
#include "physics/Kinematics.hpp"
#include "common/Common.hpp"

namespace msr { namespace airlib {


class AirSimSimpleFlightEstimator : public simple_flight::IStateEstimator {
public:
    //for now we don't do any state estimation and use ground truth (i.e. assume perfect sensors)
    void setKinematics(const Kinematics::State* kinematics)
    {
        kinematics_ = kinematics;
    }

    virtual simple_flight::Angles getAngles() const override
    {
        simple_flight::Angles angles;
        VectorMath::toEulerianAngle(kinematics_->pose.orientation,
            angles.pitch(), angles.roll(), angles.yaw());

        //Utils::log(Utils::stringf("Ang Est:\t(%f, %f, %f)", angles.pitch(), angles.roll(), angles.yaw()));

        return angles;
    }

    virtual simple_flight::Axis3r getAngulerVelocity() const override
    {
        const auto& anguler = kinematics_->twist.angular;

        simple_flight::Axis3r angular_vel;
        angular_vel.x() = anguler.x(); angular_vel.y() = anguler.y(); angular_vel.z() = anguler.z();

        return angular_vel;
    }


private:
    const Kinematics::State* kinematics_;
};


}} //namespace
#endif
