#ifndef air_automobileengine_hpp
#define air_automobileengine_hpp

#include "common/Common.hpp"
#include "automobile/AutomobileEngineParameters.hpp"

namespace msr { namespace airlib {
    class AutomobileEngine
    {
        public:
            AutomobileEngine(AutomobileEngineParameters* parameters)
            {
                this->_parameters = parameters;
            };

            virtual real_T GetTorque(real_T throttlePercentage, real_T wheelAngularVelocity) = 0;

        protected:
            AutomobileEngineParameters *_parameters;

    };
}}

#endif