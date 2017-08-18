#ifndef air_pneumaticwheel_hpp
#define air_pneumaticwheel_hpp

#include "common/Common.hpp"
#include "WheelParameters.hpp"

namespace msr { namespace airlib {

    class Automobile;

    class PneumaticWheel
    {
        public:
            virtual Vector3r GetForce(real_T wheelNormalForce, real_T directionalVelocity, real_T slipAngle, real_T coefficientOfFrictionMultiplier, TTimeDelta timeSinceLastUpdate) = 0;
            virtual WheelParameters* GetWheelParameters() const = 0;

            PneumaticWheel(bool isPowered, bool hasBrake, bool canSteer)
            {
                this->_isPowered = isPowered;
                this->_hasBrake = hasBrake;
                this->_canSteer = canSteer;
            }

            virtual void AttachAutomobile(Automobile* automobile)
            {
                this->_attachedAutomobile = automobile;
            }

            virtual bool IsPowered() const
            {
                return this->_isPowered;
            }

            virtual bool HasBrake() const
            {
                return this->_hasBrake;
            }

            virtual bool CanSteer() const
            {
                return this->_canSteer;
            }

            virtual void UpdateAngularVelocity(TTimeDelta timeSinceLastUpdate, real_T wheelNormalForce, real_T coefficientOfFrictionMultiplier);

        protected:
            Automobile* _attachedAutomobile;
            real_T _angularVelocity = static_cast<real_T>(0.0f);
            real_T _angularAcceleration = static_cast<real_T>(0.0f);
            bool _isPowered;
            bool _hasBrake;
            bool _canSteer;
            
    };
}}

#endif