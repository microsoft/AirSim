#ifndef air_wheelparameters_hpp
#define air_wheelparameters_hpp

#include "common/Common.hpp"

namespace msr { namespace airlib {

	class WheelParameters 
	{
		public:
			virtual real_T GetNoLoadOuterDiameter() const = 0;
			virtual real_T GetDryRollingFrictionCoefficient() const = 0;
			virtual real_T GetAngularInertia() const = 0;
	};
}}

#endif