#ifndef air_magicformulawheel_hpp
#define air_magicformulawheel_hpp

#include "common/Common.hpp"
#include "automobile/PneumaticWheel.hpp"
#include "automobile/Automobile.hpp"
#include "automobile/TirFileWheelParameters.hpp"

namespace msr { namespace airlib {

	class MagicFormulaWheel : public PneumaticWheel
	{
		public:
			MagicFormulaWheel(TirFileWheelParameters tirWheelParameters, bool isPowered, bool hasBrake, bool canSteer);
			virtual Vector3r GetForce(real_T wheelNormalForce, real_T directionalVelocity, real_T slipAngle, real_T coefficientOfFrictionMultiplier, TTimeDelta timeSinceLastUpdate) override;

			virtual WheelParameters* GetWheelParameters() const override
			{
				/*TODO: make this work with static_cast*/
				return (WheelParameters*)&this->_wheelParameters;
			}

		private:
			TirFileWheelParameters _wheelParameters;

			void ComputeMagicFormulaForces(real_T wheelNormalForce,
				real_T slipAngle,
				real_T directionalVelocity,
				real_T coefficientOfFrictionMultiplier,
				real_T &longitudinalForce,
				real_T &lateralForce);
	};
	
}}

#endif