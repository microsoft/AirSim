## Introduction
This project includes a self-contained cross-platform library to retrieve data from the quadrotor and send the control commands. 
You can use this library for a simulated drone in Unreal engine or on a real quadrotor such as a MavLink based vehicle platform
(and very soon DJI quadrotors such as Matrice).

## Hello Drone
Here's the taste of how you can use our APIs in C++:
See also [Python API](python.md) if you prefer that language.

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

You can find a ready to run project in HelloDrone folder in the repository.

## How does Hello Drone work?
Hello Drone uses the RPC client to connect to the RPC server that is automatically started by the AirSim. 
The RPC server routes all the commands to a class that implements [DroneControlBase](https://github.com/Microsoft/AirSim/blob/master/AirLib/include/controllers/DroneControllerBase.hpp). 
In essence, DroneControlBase defines our abstract interface for getting data from the quadrotor and sending back commands. 
We currently have concrete implementation for DroneControlBase for MavLink based vehicles. The implementation for DJI drone 
platforms, specifically Matrice, is in works.

## Timing

Notice each method of DroneControlBase API takes one of two possible parameters: `float duration` or: `float max_wait_seconds`.

Methods that take `float duration`, like moveByVelocity return control immediately. So you can therefore choose to sleep for this duration, or you can change their mind and call something else which will automatically cancel the moveByVelocity.

Methods that take `float max_wait_seconds`, like takeoff, land, moveOnPath, moveToPosition, moveToZ, and so will block this amount of time waiting for command to be successfully completed. If the command
completes before the max_wait_seconds they will return True, otherwise
if the max_wait_seconds times out they will return False. 

If you want to wait for ever pass a big number. But if you want to be able to interrupt even these commands pass 0 and you can do something else or sleep in a loop while checking the drone position, etc. 

Note: We would not recommend interrupting takeoff/land on a real drone, of course, as the results may be unpredictable.


## How to get images from drone?
Here's a sample code to get a single image:

```
int playWithImages() 
{
    using namespace std;
    using namespace msr::airlib;
    
    msr::airlib::RpcLibClient client;

    vector<uint8_t> image = client.simGetImage(0, DroneControlBase::ImageType::Depth);
    //do something with images
}
```

You can also get multiple images using API `simGetImages` which is slighly more complex to use than `simGetImage`. For example, you can get left camera view, right camera view and depth image from left camera - all at once! For sample code please see [sample code in HelloDrone project](https://github.com/Microsoft/AirSim/blob/master/HelloDrone/main.cpp). We also have [complete code](https://github.com/Microsoft/AirSim/blob/master/Examples/StereoImageGenerator.hpp) that generates specified number of stereo images and ground truth depth with normalization to camera plan, computation of disparity image and saving it to pfm format.

Unlike `simGetImage`, the `simGetImages` API also allows you to get uncompressed images as well as floating point single channel images (instead of 3 channel (RGB), each 8 bit).

You can also use Python to get images. For sample code please see [PythonClient project](https://github.com/Microsoft/AirSim/tree/master/PythonClient) and [Python example doc](python.md).

Furthermore, if your work involves computer vision experiments and if you don't care about drone dynamics then you can use our so called "ComputerVision" mode. Please see next section for the details.

## Can I use AirSim just for computer vision? I don't care about drones, physics etc.
Yes, now you can! Simply go to settings.json that you can find in your Documents\AirSim folder (or ~/Documents/AirSim on Linux). Add following setting at root level:

```
{
  "FpvVehicleName": "SimpleFlight",
  "UsageScenario": "ComputerVision"
}
```

Now when you start AirSim, you won't be able to move drone using remote control, there is no drone dynamics and physics engine is disabled in this mode. Think of this mode as that justs you move around cameras, not drone. You can use keyboard to move around (use F1 to see help on keys) and call APIs to get images. You can also use two additional APIs `simSetPose` to set position and orientation of drone programatically (use nan to specify no change). Then use can image APIs as described in above section to get images for your desired pose. Please see [complete code](https://github.com/Microsoft/AirSim/blob/master/Examples/StereoImageGenerator.hpp) that generates specified number of stereo images and ground truth depth with normalization to camera plan, computation of disparity image and saving it to pfm format in this mode.

## Can I run above code on real quadrotors as well?
Absolutely! The AirLib is self-contained library that you can put on an offboard computing module such as the Gigabyte barebone Mini PC. 
This module then can talk to the flight controllers such as Pixhawk using exact same code and MavLink protocol (or DJI protocol). 
The code you write for testing in the simulator remains unchanged! 
See [AirLib on custom drones](https://github.com/Microsoft/AirSim/blob/master/docs/custom_drone.md).

## What else can I do ?

You can also program AirSim using [Python](python.md).

See [move on path](https://github.com/Microsoft/AirSim/wiki/moveOnPath-demo) demo showing video of fast flight through Modular Neighborhood environment.

See [building a hexacopter](https://github.com/Microsoft/AirSim/wiki/hexacopter).

See [building point clouds](https://github.com/Microsoft/AirSim/wiki/Point-Clouds).