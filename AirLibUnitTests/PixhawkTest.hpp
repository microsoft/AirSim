
#ifndef msr_AirLibUnitTests_PixhawkTest_hpp
#define msr_AirLibUnitTests_PixhawkTest_hpp

#include "vehicles/MultiRotorParamsFactory.hpp"
#include "TestBase.hpp"

namespace msr { namespace airlib {

class PixhawkTest : public TestBase {
public:
    virtual void run() override
    {
        //Test PX4 based drones
        auto pixhawk = MultiRotorParamsFactory::createConfig("Pixhawk");	
        pixhawk->initialize();
        
        DroneControllerBase* controller = pixhawk->getController();
        testAssert(controller != nullptr, "Couldn't get pixhawk controller");
        
        try {
            controller->start();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            controller->stop();
        }
        catch (std::domain_error& ex) {
            std::cout << ex.what() << std::endl;
        }
    }
};


}}
#endif