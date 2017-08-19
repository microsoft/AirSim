#ifndef air_automobileengineparameters_hpp
#define air_automobileengineparameters_hpp

#include "common/Common.hpp"

namespace msr { namespace airlib {
    class AutomobileEngineParameters
    {
        public:
            AutomobileEngineParameters(real_T stallRadiansPerSecond, real_T redLineRadiansPerSecond, real_T maxTorque)
            {
                this->StallRadiansPerSecond = stallRadiansPerSecond;
                this->RedLineRadiansPerSecond = redLineRadiansPerSecond;
                this->MaxTorque = maxTorque;
            }

            /*TODO: Add constructor that reads from file*/

            real_T StallRadiansPerSecond;
            real_T RedLineRadiansPerSecond;

            /*TODO: This should be some sort of interpolation struct that can hold a curve*/
            /*this was coded with the HardCodedEngine in mind*/
            real_T MaxTorque;
    };
}}

#endif