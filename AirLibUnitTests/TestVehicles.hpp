
#ifndef msr_AirLibUnitTests_TestVehicles_hpp
#define msr_AirLibUnitTests_TestVehicles_hpp

#include "vehicles/MultiRotorParamsFactory.hpp"
#include <chrono>
#include <cassert>

using namespace msr::airlib;

class TestVehicles
{
public:
    void Run() {

        auto rosFlight = MultiRotorParamsFactory::createConfig("RosFlight");
        rosFlight->initialize();

        // Test PX4 based drones
        auto pixhawk = MultiRotorParamsFactory::createConfig("Pixhawk");	
        pixhawk->initialize();
        
        DroneControllerBase* controller = pixhawk->getController();
        assert(controller != nullptr);
        
        controller->start();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        controller->stop();


        return;
    }
};

#endif