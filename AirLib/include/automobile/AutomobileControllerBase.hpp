#ifndef air_automobilecontrollerbase_hpp
#define air_automobilecontrollerbase_hpp


#include "controllers/VehicleControllerBase.hpp"
#include "automobile/Automobile.hpp"
#include "common/common_utils/WorkerThread.hpp"

namespace msr { namespace airlib {

    class AutomobileControllerBase : public VehicleControllerBase {

        public:

            virtual void setOffboardMode(bool is_set)
            {
                // NOT IMPLEMENTED
            }
            virtual void setSimulationMode(bool is_set)
            {
                // NOT IMPLEMENTED
            }
            virtual bool isOffboardMode()
            {
                return false;
            }
            virtual bool isSimulationMode()
            {
                return false;
            }

            //reset any state in the controller
            virtual void reset()
            {
            }
            virtual void update()
            {

            }

            //return 0 to 1 (corresponds to zero to full thrust)
            virtual real_T getVertexControlSignal(unsigned int rotor_index)
            {
                return real_T(0);
            }

            virtual size_t getVertexCount()
            {
                return 0;
            }

            /*TODO: GetState, SetControlSignals methods*/
    };




}}

#endif
