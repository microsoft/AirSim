#ifndef air_automobilehardcodedbrake_hpp
#define air_automobilehardcodedbrake_hpp

#include "common/Common.hpp"
#include "automobile/AutomobileBrake.hpp"
#include "automobile/AutomobileBrakeParameters.hpp"

namespace msr { namespace airlib {

	class AutomobileHardCodedBrake : public AutomobileBrake
	{
		public:
			AutomobileHardCodedBrake(AutomobileBrakeParameters *parameters)
				: AutomobileBrake(parameters) {};

			virtual real_T GetBrakeTorque(real_T brakePercentage) override
			{
				return this->_parameters->MaximumBrakeTorque * brakePercentage;
			}
	};
}}

#endif