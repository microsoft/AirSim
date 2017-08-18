#ifndef air_automobilestaticparams_hpp
#define air_automobilestaticparams_hpp

#include "common/Common.hpp"

namespace msr { namespace airlib {

    struct AutomobileStaticParams {
        real_T MaxSteeringAngleRadians;
        real_T MaximumBrakeTorque;
        real_T BrakeTorquePerRadPerSecond;
        real_T MaximumThrottleTorque;
        real_T DragCoefficient;
        real_T CrossSectionalArea;
        real_T Mass;
        real_T LongitudinalWheelBase;
        real_T LateralWheelBase;
    };

}}

#endif