## Introduction
This project includes a self-contained cross-platform library to retrieve data from the quadrotor and send the control commands. You can use this library for a simulated drone in Unreal engine or on a real quadrotor such as a MavLink based vehicle platform (and very soon DJI quadrotors such as Matrice).

## Hello Drone
Here's the taste of how you can use our APIs:

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
    client.takeoff(1000);

    cout << "Press Enter to use offboard control" << endl; cin.get();
    client.requestControl();

    cout << "Press Enter to move 5 meters in x direction with 1 m/s velocity" << endl; cin.get();    
    auto position = client.getPosition(); // from current location
    client.moveToPosition(position.x() + 5, position.y(), position.z(), 1);

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

## How to get images from drone?
Here's a sample code. For more information, please see [DroneControlBase](https://github.com/Microsoft/AirSim/blob/master/AirLib/include/controllers/DroneControllerBase.hpp) class.

```
int playWithImages() 
{
    using namespace std;
    using namespace msr::airlib;
    
    msr::airlib::RpcLibClient client;

    client.setImageTypeForCamera(0, DroneControlBase::ImageType::Depth);
    vector<uint8_t> image = client.getImageForCamera(0, DroneControlBase::ImageType::Depth);
    //do something with images
}
```

## Can I run above code on real quadrotors as well?
Absolutely! The AirLib is self-contained library that you can put on an offboard computing module such as the Gigabyte barebone Mini PC. 
This module then can talk to the flight controllers such as Pixhawk using exact same code and MavLink protocol (or DJI protocol). 
The code you write for testing in the simulator remains unchanged! We will post more detailed guide on how to do this soon.

## What else is in works?
We are working on enabling other RPC stack such as ZeroMQ over protobufs so we can enable more languages such as Python. 
We also hope to release ROS adapters for our APIs with Linux builds.
