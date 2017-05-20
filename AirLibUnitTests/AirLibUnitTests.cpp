// AirLibUnitTests.cpp : Defines the entry point for the console application.
//

#include <cassert>
#include "TestVehicles.hpp"
#include "WorkerThreadTest.hpp"
#include <string>

using namespace msr::airlib;

int main()
{
	Settings& settings = Settings::loadJSonFile("settings.json");

	TestVehicles test1;
	test1.Run();

    WorkerThreadTest test2;
    test2.Run();
	
    return 0;
}

