
#ifndef msr_AirLibUnitTests_VehicleTest_hpp
#define msr_AirLibUnitTests_VehicleTest_hpp

#include "vehicles/MultiRotorParamsFactory.hpp"
#include "TestBase.hpp"

namespace msr { namespace airlib {

class VehicleTest : public TestBase
{
public:
    virtual void run() override
    {
        auto rosFlight = MultiRotorParamsFactory::createConfig("RosFlight");
        rosFlight->initialize();

        // Test PX4 based drones
        auto pixhawk = MultiRotorParamsFactory::createConfig("Pixhawk");	
        pixhawk->initialize();
        
        DroneControllerBase* controller = pixhawk->getController();
        testAssert(controller != nullptr, "Couldn't get pixhawk controller");
        
        controller->start();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        controller->stop();
    }
};


}}
#endif