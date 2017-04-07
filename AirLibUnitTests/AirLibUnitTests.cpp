// AirLibUnitTests.cpp : Defines the entry point for the console application.
//

#include <cassert>
#include "TestVehicles.hpp"

int main()
{
	Settings& settings = Settings::loadJSonFile("settings.json");

	TestVehicles test1;
	test1.Run();
	
    return 0;
}

