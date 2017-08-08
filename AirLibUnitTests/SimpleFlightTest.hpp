
#ifndef msr_AirLibUnitTests_SimpleFlightTest_hpp
#define msr_AirLibUnitTests_SimpleFlightTest_hpp

#include "vehicles/MultiRotorParamsFactory.hpp"
#include "TestBase.hpp"

namespace msr { namespace airlib {

class SimpleFlightTest : public TestBase
{
public:
    virtual void run() override
    {
        auto simpleFlight = MultiRotorParamsFactory::createConfig("SimpleFlight");
        simpleFlight->initialize();

        DroneControllerBase* controller = simpleFlight->getController();
        testAssert(controller != nullptr, "Couldn't get controller");

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
};


}}
#endif