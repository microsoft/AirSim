## Introduction
This project includes self-contained cross-platform library to retrieve data from the drone and send the control commands. You can use this library for simulated drone in Unreal engine or on real drones such as MavLink based vehicle platform (and very soon DJI drones such as Matrice).

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

    cout << "Press Enter to move 5 meters with 1 m/s velocity" << endl; cin.get();
    client.moveToPosition(5, 0, 2.5f, 1);

    cout << "Press Enter to land" << endl; cin.get();
    client.land();

    return 0;
}

```

You can find ready to run code in HelloDrone folder in the repository.

## How does Hello Drone work?
Above code uses the RPC client to connect to the RPC server that is automatically started by the AirSim. The RCP server routes all the commands to a class that implements [DroneControlBase](../AirLib/include/control/DroneControlBase.hpp). In essence, DroneControlBase defines our abstract interface for getting data from the drone and sending it commands. We currently have concrete implementation for DroneControlBase for MavLink based vehicles. The implementation for DJI drone platforms, specifically Matrice, is in works.

## How to get images from drone?
Here's sample code. For more information, please see [DroneControlBase](../AirLib/include/control/DroneControlBase.hpp) class.

```
int playWithImages() 
{
    using namespace std;
    using namespace msr::airlib;
    
    msr::airlib::RpcLibClient client;

    vector<uint8_t> image = client.getImageForCamera(0, DroneControlBase::ImageType::Depth);
    //do something with images
}
```

## Can I run above code on real drones as well?
Absolutely! The AirLib is self-contained library that you can put on offboard computing modules such as Gigabyte barebone Mini PC. This modules then can talk to flight controllers such as Pixhawk using exact same code and MavLink protocol (or DJI protocol). The code you write for testing in simulator remains unchanged! We will post more detailed guide on how to do this soon.

## What else is in works?
We are working on enabling other RPC stack such as ZeroMQ over protobufs so we can enable more languages such as Python. We also hope to release ROS adapters for our APIs with Linux builds.