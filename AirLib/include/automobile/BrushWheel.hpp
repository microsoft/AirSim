#ifndef air_brushwheel_hpp
#define air_brushwheel_hpp

#include "automobile/BrushWheelParameters.hpp"
#include "automobile/TirFileWheelParameters.hpp"
#include "automobile/PneumaticWheel.hpp"
#include "common/common_utils/Utils.hpp"
#include "common/Common.hpp"
#include "automobile/Automobile.hpp"

namespace msr { namespace airlib {

	class Automobile;

	class BrushWheel : public PneumaticWheel
	{
		public:
			BrushWheel(BrushWheelParameters wheelParameters, bool isPowered, bool hasBrake, bool canSteer);

			virtual Vector3r GetForce(real_T wheelNormalForce, real_T directionalVelocity, real_T slipAngle, real_T coefficientOfFrictionMultiplier, TTimeDelta timeSinceLastUpdate) override;

			virtual WheelParameters* GetWheelParameters() const override
			{
				/*TODO: make this work with static_cast*/
				return (WheelParameters*)&this->_wheelParameters;
			}

			std::vector<std::vector<real_T>> TestForces();

		private:
			BrushWheelParameters _wheelParameters;


			real_T ComputeContactPatchWidth(real_T weightOnWheel);
			real_T ComputeContactPatchLength(real_T directionalVelocity, real_T weightOnWheel);
			void ComputeBrushForces(real_T wheelNormalForce,
				real_T slipAngle,
				real_T directionalVelocity,
				real_T coefficientOfFrictionMultiplier,
				real_T contactPatchWidth,
				real_T contactPatchLength,
				real_T &longitudinalForce,
				real_T &lateralForce);

	};
}}

#endif