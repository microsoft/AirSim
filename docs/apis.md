## Introduction
AirSim offers APIs to interact with vehicles. You can use these APIs to retrieve images, get state, command the vehicle and so on. The APIs use [msgpack-rpc protocol](https://github.com/msgpack-rpc/msgpack-rpc) which has bindings available in variety of languages including C++, C#, Python, Java etc.

## Hello Drone
Here's very quick overview of how to use AirSim APIs using C++ (see also [Python doc](python.md)):
See also  if you prefer that language.

```
#include <iostream>
#include "control/RpcLibClient.hpp"

int main() 
{
    using namespace std;
    msr::airlib::RpcLibClient client;

    cout << "Press Enter to arm the drone" << endl; cin.get();
    client.armDisarm(true);

    cout << "Press Enter to takeoff" << endl; cin.get();
    client.takeoff(60);

    cout << "Press Enter to use offboard control" << endl; cin.get();
    client.requestControl();

    cout << "Press Enter to move 5 meters in x direction with 1 m/s velocity" << endl; cin.get();    
    auto position = client.getPosition(); // from current location
    client.moveToPosition(position.x() + 5, position.y(), position.z(), 1, 5*1);

    cout << "Press Enter to land" << endl; cin.get();
    client.land();

    return 0;
}

```

You can find a ready to run project in HelloDrone folder in the repository. Read more about [Hello Drone](hello_drone.md).

## Image / Computer Vision and Collision APIs
AirSim offers comprehensive images APIs to retrieve synchronized images from multiple cameras along with ground truth including depth and vision. You can set the resolution, FOV, motion blur etc parameters in [settings.json](settings.md). There is also API for detecting collison state. In addition, AirSim also includes complete examples of how to generate stereo images along with ground truth depth images.

More on [image APIs](image_apis.md).

## Note on Timing Related Parameters

Many API methods has parameters named `float duration` or: `float max_wait_seconds`.

Methods that take `float duration`, like `moveByVelocit`y return control immediately. So you can therefore choose to sleep for this duration, or you can change their mind and call something else which will automatically cancel the `moveByVelocity`.

Methods that take `float max_wait_seconds`, like `takeoff`, `land`, `moveOnPath`, `moveToPosition`, `moveToZ`, and so will block this amount of time waiting for command to be successfully completed. If the command completes before the max_wait_seconds they will return True, otherwise
if the `max_wait_seconds` times out they will return `false`.  If you want to wait for ever pass a big number. But if you want to be able to interrupt even these commands pass 0 and you can do something else or sleep in a loop while checking the drone position, etc. We would not recommend interrupting takeoff/land on a real drone, of course, as the results may be unpredictable.

## Using APIs on Real Vehicles
We want to be able to run *same code* that runs in simulation as on real vehicle. The AirLib is self-contained library that you can put on an offboard computing module such as the Gigabyte barebone Mini PC. This module then can talk to the flight controllers such as Pixhawk using exact same code and MavLink protocol (or DJI protocol). The code you write for testing in the simulator remains unchanged! 
See [AirLib on custom drones](https://github.com/Microsoft/AirSim/blob/master/docs/custom_drone.md).

## References and Examples

* AirSim APIs using [Python](python.md)
* [move on path](https://github.com/Microsoft/AirSim/wiki/moveOnPath-demo) demo showing video of fast flight through Modular Neighborhood environment
* [building a hexacopter](https://github.com/Microsoft/AirSim/wiki/hexacopter)
* [building point clouds](https://github.com/Microsoft/AirSim/wiki/Point-Clouds)