#ifndef air_automobilebrakeparameters_hpp
#define air_automobilebrakeparameters_hpp

#include "common/Common.hpp"

namespace msr { namespace airlib {
    class AutomobileBrakeParameters
    {
        public:
            AutomobileBrakeParameters(real_T maximumBrakeTorque)
            {
                this->MaximumBrakeTorque = maximumBrakeTorque;
            }

            real_T MaximumBrakeTorque;
    };
}}

#endif