
#ifndef msr_AirLibUnitTests_RosFlightTest_hpp
#define msr_AirLibUnitTests_RosFlightTest_hpp

#include "vehicles/MultiRotorParamsFactory.hpp"
#include "TestBase.hpp"

namespace msr { namespace airlib {

class RosFlightTest : public TestBase {
public:
    virtual void run() override
    {
        auto rosFlight = MultiRotorParamsFactory::createConfig("RosFlight");
        rosFlight->initialize();

        DroneControllerBase* controller = rosFlight->getController();
        testAssert(controller != nullptr, "Couldn't get controller");

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
};


}}
#endif