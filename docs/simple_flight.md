# simple_flight

 If you don't know what flight controller does than see [What is Flight Controller?](flight_controller.md). 
 
 AirSim has built-in flight controller called simple_flight and it is used by default. You don't need to do anything to use or configure it. AirSim also supports [PX4](px4_setup.md) as another flight controller for advanced users. In future, we also plan to support [ROSFlight](https://rosflight.org/) and [Hackflight](https://github.com/simondlevy/hackflight).

## Advantages

The advantage of using simple_flight is zero additional setup you need to do and it "just works". Also, simple_flight uses steppable clock which means you can pause the simulation and things are not at mercy of high variance low precision clock that operating system provides. Further, simple_flight is simple, cross platform and 100% header-only dependency-free C++ code which means you can literally step through from simulator to inside flight controller code within same code base!

## Design

Normally flight controllers are designed to run on actual hardware on vehicles and their support for running in simulator varies widely. They are often fairly difficult to configure for non-expert users and typically have complex build usually lacking cross platform support. All these problems have played significant part in design of simple_flight.

simple_flight is designed from ground up as library with clean interface that can work onboard the vehicle as well as simulator. The core principle is that flight controller has no way to specify special simulation mode and therefore it has no way to know if it is running under simulation or real vehicle. We thus view flight controller simply as collection of algorithms packaged in a library. Another key emphasis is to develop this code as dependency free header-only pure standard C++11 code. This means there is no special build required to compile simple_flight. You just copy its source code to any project you wish and it just works.

## Control

simple_flight can control vehicle by taking in desired input as angle rate, angle level, velocity or position. Each axis of control can be specified with one of these modes. Internally simple_flight uses cascade of PID controllers to finally generate actuator signals. This means position PID drives velocity PID which drives angle level PID which finally drives angle rate PID.

## State Estimation

In current release we are using ground truth from simulator for our state estimation. We plan to add complimentary filter based state estimation for angular velocity and orientation using 2 sensors (gyroscope, accelerometer) in near future. In more longer term, we plan to integrate another library to do velocity and position estimation using 4 sensors (gyroscope, accelerometer, magnetometer and barometer) using EKF. If you have experience this area than we encourage you to engage with us and contribute!

## Supported Boards

Currently we have implemented simple_flight interfaces for simulated board. We plan to implement it for Pixhawk V2 board and possibly Naze32 board. We expect all our code to remain unchanged and the implementation would mainly involve adding drivers for various sensors, handling ISRs and managing other board specific details. If you have experience this area than we encourage you to engage with us and contribute!

## Configuration

To have AirSim use simple_flight, you can specify it in [settings.json](settings.md) as shown below. Note that this is default so you don't have to do it explicitly.

```
"Vehicles": {
    "SimpleFlight": {
      "VehicleType": "SimpleFlight",
    }
}
```

By default, vehicle using simple_flight is already armed which is why you would see propellers spinning. However if you don't want that than set `DefaultVehicleState` to `Inactive` like this:

```
"Vehicles": {
    "SimpleFlight": {
      "VehicleType": "SimpleFlight",
      "DefaultVehicleState": "Inactive"
    }
}
```

In this case, you will need to either manually arm using RC sticks in down inward position or using APIs.

For safety reasons, flight controllers would disallow API control unless human operator has consented using a switch on RC. Also, when RC control is lost, vehicle should disable API control and enter hover mode for safety reasons. To simplify things a bit, simple_flight enables API control without human consent using RC and even when RC is not detected by default however you can change this using following setting:

```
"Vehicles": {
    "SimpleFlight": {
      "VehicleType": "SimpleFlight",

      "AllowAPIAlways": true,
      "RC": {
        "RemoteControlID": 0,      
        "AllowAPIWhenDisconnected": true
      }
    }
}
```

Finally, simple_flight uses steppable clock by default which means clock advances when simulator tells it to advance (unlike wall clock which advances strictly according to passage of time). This means clock can be paused, for example, if code hits the break point and there is zero variance in clock (clock APIs provides by operating system might have significant variance unless its "real time" OS). If you want simple_flight to use wall clock instead than use following settings:

```
  "ClockType": "ScalableClock"
```
