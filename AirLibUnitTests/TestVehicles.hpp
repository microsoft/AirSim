
#ifndef msr_AirLibUnitTests_TestVehicles_hpp
#define msr_AirLibUnitTests_TestVehicles_hpp

#include <chrono>
#include <cassert>
#include "vehicles/configs/PX4ConfigCreator.hpp"
#include "vehicles/configs/RosFlightQuadX.hpp"

using namespace msr::airlib;

class TestVehicles
{
public:
	void Run() {

		// try the RosFlightQuadX.
		RosFlightQuadX rosFlight;
		rosFlight.initialize();

		// Test PX4 based drones
		std::unique_ptr<MultiRotorParams> params = PX4ConfigCreator::createConfig("Pixhawk");	
		params->initialize();
		
		DroneControllerBase* controller = params->getController();
		assert(controller != nullptr);
		
		controller->start();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		controller->stop();


		return;
	}
};

#endif