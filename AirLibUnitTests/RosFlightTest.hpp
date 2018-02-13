
#ifndef msr_AirLibUnitTests_RosFlightTest_hpp
#define msr_AirLibUnitTests_RosFlightTest_hpp

#include "vehicles/multirotor/MultiRotorParamsFactory.hpp"
#include "TestBase.hpp"
#include "sensors/SensorFactory.hpp"

namespace msr { namespace airlib {

class RosFlightTest : public TestBase {
public:
    virtual void run() override
    {
        auto rosFlight = MultiRotorParamsFactory::createConfig("RosFlight", std::make_shared<SensorFactory>());
        
        DroneControllerBase* controller = rosFlight->getController();
        testAssert(controller != nullptr, "Couldn't get controller");

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
};


}}
#endif