#ifndef air_automobilebrake_hpp
#define air_automobilebrake_hpp

#include "common/Common.hpp"
#include "automobile/AutomobileBrakeParameters.hpp"

namespace msr { namespace airlib {
	class AutomobileBrake
	{
		public:
			AutomobileBrake(AutomobileBrakeParameters *parameters)
			{
				this->_parameters = parameters;
			}

			virtual real_T GetBrakeTorque(real_T brakePercentage) = 0;

		protected:
			AutomobileBrakeParameters *_parameters;
	};
}}

#endif