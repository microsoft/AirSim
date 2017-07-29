// AirLibUnitTests.cpp : Defines the entry point for the console application.
//

#include "VehicleTest.hpp"
#include "WorkerThreadTest.hpp"
#include "QuaternionTest.hpp"

int main()
{
    using namespace msr::airlib;

    QuaternionTest quaterion_test;
    quaterion_test.run();

    Settings& settings = Settings::loadJSonFile("settings.json");
    unused(settings);

    VehicleTest test1;
    test1.run();

    WorkerThreadTest test2;
    test2.run();
    
    return 0;
}

