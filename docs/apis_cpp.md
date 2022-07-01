# Using C++ APIs for AirSim

Please read [general API doc](apis.md) first if you haven't already. This document describes C++ examples and other C++ specific details.

## Quick Start

Fastest way to get started is to open AirSim.sln in Visual Studio 2019. You will see [Hello Car](https://github.com/Microsoft/AirSim/tree/main/HelloCar/) and [Hello Drone](https://github.com/Microsoft/AirSim/tree/main/HelloDrone/) examples in the solution. These examples will show you the include paths and lib paths you will need to setup in your VC++ projects. If you are using Linux then you will specify these paths either in your [cmake file](https://github.com/Microsoft/AirSim/tree/main/cmake//HelloCar/CMakeLists.txt) or on compiler command line.

#### Include and Lib Folders

* Include folders: `$(ProjectDir)..\AirLib\deps\rpclib\include;include;$(ProjectDir)..\AirLib\deps\eigen3;$(ProjectDir)..\AirLib\include`
* Dependencies: `rpc.lib`
* Lib folders: `$(ProjectDir)\..\AirLib\deps\MavLinkCom\lib\$(Platform)\$(Configuration);$(ProjectDir)\..\AirLib\deps\rpclib\lib\$(Platform)\$(Configuration);$(ProjectDir)\..\AirLib\lib\$(Platform)\$(Configuration)`
* References: Reference AirLib and MavLinkCom to the project references. (Right click your project then go to `References`, `Add reference...`, and then select AirLib and MavLinkCom)

## Hello Car

Here's how to use AirSim APIs using C++ to control simulated car (see also [Python example](apis.md#hello_car)):

```cpp

// ready to run example: https://github.com/Microsoft/AirSim/blob/main/HelloCar/main.cpp

#include <iostream>
#include "vehicles/car/api/CarRpcLibClient.hpp"

int main()
{
    msr::airlib::CarRpcLibClient client;
    client.enableApiControl(true); //this disables manual control
    CarControllerBase::CarControls controls;

    std::cout << "Press enter to drive forward" << std::endl; std::cin.get();
    controls.throttle = 1;
    client.setCarControls(controls);

    std::cout << "Press Enter to activate handbrake" << std::endl; std::cin.get();
    controls.handbrake = true;
    client.setCarControls(controls);

    std::cout << "Press Enter to take turn and drive backward" << std::endl; std::cin.get();
    controls.handbrake = false;
    controls.throttle = -1;
    controls.steering = 1;
    client.setCarControls(controls);

    std::cout << "Press Enter to stop" << std::endl; std::cin.get();
    client.setCarControls(CarControllerBase::CarControls());

    return 0;
}
```

## Hello Drone

Here's how to use AirSim APIs using C++ to control simulated quadrotor (see also [Python example](apis.md#hello_drone)):

```cpp

// ready to run example: https://github.com/Microsoft/AirSim/blob/main/HelloDrone/main.cpp

#include <iostream>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

int main()
{
    msr::airlib::MultirotorRpcLibClient client;

    std::cout << "Press Enter to enable API control\n"; std::cin.get();
    client.enableApiControl(true);

    std::cout << "Press Enter to arm the drone\n"; std::cin.get();
    client.armDisarm(true);

    std::cout << "Press Enter to takeoff\n"; std::cin.get();
    client.takeoffAsync(5)->waitOnLastTask();

    std::cout << "Press Enter to move 5 meters in x direction with 1 m/s velocity\n"; std::cin.get();
    auto position = client.getMultirotorState().getPosition(); // from current location
    client.moveToPositionAsync(position.x() + 5, position.y(), position.z(), 1)->waitOnLastTask();

    std::cout << "Press Enter to land\n"; std::cin.get();
    client.landAsync()->waitOnLastTask();

    return 0;
}
```

## See Also

* [Examples](https://github.com/microsoft/AirSim/tree/main/Examples) of how to use internal infrastructure in AirSim in your other projects
* [DroneShell](https://github.com/microsoft/AirSim/tree/main/DroneShell) app shows how to make simple interface using C++ APIs to control drones
* [HelloSpawnedDrones](https://github.com/microsoft/AirSim/tree/main/HelloSpawnedDrones) app shows how to make additional vehicles on the fly
* [Python APIs](apis.md)
