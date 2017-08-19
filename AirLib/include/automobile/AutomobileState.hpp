#ifndef air_automobilestate_hpp
#define air_automobilestate_hpp

#include "common/VectorMath.hpp"
#include "physics/Kinematics.hpp"

namespace msr { namespace airlib {
    class AutomobileState {

        public:
            AutomobileState()
            {
                this->SteeringAngle = 0.0f;
                this->ThrottlePercentage = 0.0f;
                this->BrakePercentage = 0.0f;
            }

            msr::airlib::Kinematics Kinematics;
            real_T SteeringAngle;
            real_T ThrottlePercentage;
            real_T BrakePercentage;

            Vector3r NormalizedHeading() const
            {
                return VectorMath::rotateVector(Vector3r(1, 0, 0),
                    this->Kinematics.getPose().orientation,
                    false); //TODO: Can we assume unit quat?
            }

            double ForwardAccelerationMagnitude() const
            {
                return this->NormalizedHeading()
                    .dot(this->Kinematics.getState().accelerations.linear);
            }
    };
}}
#endif