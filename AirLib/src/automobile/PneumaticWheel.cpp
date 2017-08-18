#ifdef AIRLIB_PCH
#include "AirSim.h"
#endif
#include "automobile/PneumaticWheel.hpp"
#include "automobile/Automobile.hpp"
#include "automobile/WheelParameters.hpp"
#include "common/Common.hpp"

#include <iostream>

namespace msr { namespace airlib {

    void PneumaticWheel::UpdateAngularVelocity(TTimeDelta timeSinceLastUpdate, real_T wheelNormalForce, real_T coefficientOfFrictionMultiplier)
    {
        WheelParameters* wheelParameters = this->GetWheelParameters();

        int currentAngularVelocityDirection = Utils::sgn(this->_angularVelocity);

        /*If we're stopped, default the next direction to forward.*/
        if (currentAngularVelocityDirection == 0)
        {
            currentAngularVelocityDirection = 1;
        }

        real_T throttleTorque = static_cast<real_T>(0.0f);
        if (this->IsPowered())
        {
            throttleTorque = this->_attachedAutomobile->GetEngineTorque(this->_angularVelocity) * currentAngularVelocityDirection;
        }

        real_T brakeTorque = static_cast<real_T>(0.0f);
        if (this->HasBrake())
        {
            brakeTorque = this->_attachedAutomobile->GetBrakingTorque() * -1.0f * currentAngularVelocityDirection;
        }

        real_T rollingFriction = wheelParameters->GetDryRollingFrictionCoefficient()
            * coefficientOfFrictionMultiplier
            * wheelNormalForce
            * -1.0f
            * currentAngularVelocityDirection;

        this->_angularAcceleration = (throttleTorque + brakeTorque + rollingFriction) / wheelParameters->GetAngularInertia();

        this->_angularVelocity = this->_angularVelocity + (this->_angularAcceleration * static_cast<real_T>(timeSinceLastUpdate));

        int nextAngularVelocityDirection = Utils::sgn(this->_angularVelocity);

        /*If the angular velocity direction is caused by a sign change, then it must be due to the throttle.*/
        /*Otherwise, set the angular velocity to zero.*/
        if ( (nextAngularVelocityDirection != currentAngularVelocityDirection) && (std::abs(throttleTorque) < (std::abs(brakeTorque) + std::abs(rollingFriction))))
        {
            this->_angularAcceleration = static_cast<real_T>(0.0f);
            this->_angularVelocity = static_cast<real_T>(0.0f);
        }
    }
}}
