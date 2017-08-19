#ifndef air_automobilehardcodedengine_hpp
#define air_automobilehardcodedengine_hpp

#include "automobile/AutomobileEngine.hpp"
#include "automobile/AutomobileEngineParameters.hpp"
#include "common/Common.hpp"

#include <fstream>

namespace msr { namespace airlib {

    class AutomobileHardcodedEngine : public AutomobileEngine
    {
        public:
            AutomobileHardcodedEngine(AutomobileEngineParameters *parameters)
                : AutomobileEngine(parameters) {};

            virtual real_T GetTorque(real_T throttlePercentage, real_T wheelAngularVelocity) override
            {
                /*Really stupid model. Give full torque if engine is over */
                real_T requestedAngularVelocity = throttlePercentage * (this->_parameters->RedLineRadiansPerSecond - this->_parameters->StallRadiansPerSecond) + this->_parameters->StallRadiansPerSecond;
                real_T torque = static_cast<real_T>(0.0f);
                if (wheelAngularVelocity < this->_parameters->RedLineRadiansPerSecond && requestedAngularVelocity > wheelAngularVelocity)
                {
                    torque = std::max(throttlePercentage, 0.05f) * (this->_parameters->MaxTorque);
                }

                #if defined(PHYSICS_VERBOSE)
                    std::ofstream w;
                    w.open(PHYSICS_OUTPUT_DIR + "engine.tsv", std::ofstream::out | std::ofstream::app);
                    w << torque << "\t" << wheelAngularVelocity << "\t" << requestedAngularVelocity << std::endl;
                    w.close();
                #endif

                return torque;
            }

    };
}}


#endif