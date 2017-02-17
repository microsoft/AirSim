## AirLib
Majority of the code is located in AirLib. This is a self-contained library that you should be able to compile with modern C++ compilers. The rpclib requires C++14 but everything else should compile with C++11.

AirLib consists of the following components:
1. *Physics engine:* This is header-only physics engine. It is designed to be fast and extensible to implement different vehicles.
2. *Sensor models:* This is header-only models for Barometer, IMU, GPS and Magnetometer
3. *Vehicle models:* This is header-only models for vehicle configurations and models. Currently we have implemented model for a MultiRotor and a configuration for PX4 QuadRotor in the X config.
4. *Control library:* This part of AirLib provides abstract base class for our APIs and concrete implementation for specific vehicle platforms such as MavLink. It also has classes for the RPC client and server.

## Unreal/Plugins/AirSim
This is the only portion of project which is dependent on Unreal engine. We have kept it isolated so we can implement simulator for other platforms as well (for example, Unity). The Unreal code takes advantage of its UObject based classes including Blueprints.
1. *SimMode_ classes*: We wish to support various simulator modes such as pure Computer Vision mode where there is no drone. The SimMode classes helps to implement many such different modes.
2. *VehiclePawnBase*: This is the base class for all vehicle pawn visualizations.
3. *VehicleBase*: This class provides abstract interface to implement combination of rendering component (i.e. Unreal pawn), physics component (i.e. MultiRotor) and controller (i.e. MavLinkHelper).

## MavLinkCom
This is the library developed by our own team member Chris Lovett that provides C++ classes to talk to the MavLink devices. This library is stand alone and can be used in any project.
See [MavLinkCom](../MavLinkCom/README.md) for more info.

## Sample Programs
We have created few sample program to demonstrate how to use the API. See HelloDrone and DroneServer. The DroneServer demonstrates how to connect to the simulator using UDP port for QGC.
