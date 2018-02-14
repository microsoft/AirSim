
#ifndef msr_AirLibUnitTests_PixhawkTest_hpp
#define msr_AirLibUnitTests_PixhawkTest_hpp

#include "vehicles/multirotor/MultiRotorParamsFactory.hpp"
#include "TestBase.hpp"

namespace msr { namespace airlib {

class PixhawkTest : public TestBase {
public:
    virtual void run() override
    {
        //Test PX4 based drones
        auto pixhawk = MultiRotorParamsFactory::createConfig("Pixhawk", std::make_shared<SensorFactory>());	
        
        DroneControllerBase* controller = pixhawk->getController();
        testAssert(controller != nullptr, "Couldn't get pixhawk controller");
        
        try {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        catch (std::domain_error& ex) {
            std::cout << ex.what() << std::endl;
        }
    }
};


}}
#endif