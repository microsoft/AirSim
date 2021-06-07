# simple_flight

If you don't know what the flight controller does, see [What is Flight Controller?](flight_controller.md). 
 
AirSim has a built-in flight controller called simple_flight and it is used by default. You don't need to do anything to use or configure it. AirSim also supports [PX4](px4_setup.md) as another flight controller for advanced users. In the future, we also plan to support [ROSFlight](https://rosflight.org/) and [Hackflight](https://github.com/simondlevy/hackflight).

## Advantages

The advantage of using simple_flight is zero additional setup you need to do and it "just works". Also, simple_flight uses a steppable clock which means you can pause the simulation and things are not at mercy of a high variance low precision clock that the operating system provides. Furthermore, simple_flight is simple, cross platform and consists of 100% header-only dependency-free C++ code which means you can literally switch between the simulator and the flight controller code within same code base!

## Design

Normally flight controllers are designed to run on actual hardware of vehicles and their support for running in simulator varies widely. They are often fairly difficult to configure for non-expert users and typically have a complex build, usually lacking cross platform support. All these problems have played a significant part in the design of simple_flight.

simple_flight is designed from ground up as library with clean a interface that can work onboard the vehicle as well as in the simulator. The core principle is that the flight controller has no way to specify a special simulation mode and therefore it has no way to know if it is running as a simulation or as a real vehicle. We thus view flight controllers simply as a collection of algorithms packaged in a library. Another key emphasis is to develop this code as dependency-free header-only pure standard C++11 code. This means there is no special build required to compile simple_flight. You just copy its source code to any project you wish and it just works.

## Control

simple_flight can control vehicles by taking in the desired input as angle rate, angle level, velocity or position. Each axis of control can be specified with one of these modes. Internally, simple_flight uses a cascade of PID controllers to finally generate actuator signals. This means that the position PID drives the velocity PID, which in turn drives the angle level PID which finally drives the angle rate PID.

## State Estimation

In the current release, we are using the ground truth from the simulator for our state estimation. We plan to add a complimentary filter-based state estimator for angular velocity and orientation using 2 sensors (gyroscope, accelerometer) in the near future. In a more longer term, we plan to integrate another library to perform velocity and position estimation using 4 sensors (gyroscope, accelerometer, magnetometer and barometer) using an Extended Kalman Filter (EKF). If you have experience in this area, we encourage you to engage with us and contribute!

## Supported Boards

Currently, we have implemented simple_flight interfaces for the simulated board. We plan to implement it for the Pixhawk V2 board and possibly the Naze32 board. We expect all our code to remain unchanged and the implementation would mainly involve adding drivers for various sensors, handling ISRs and managing other board specific details. If you have experience in this area, we encourage you to engage with us and contribute!

## Configuration

To have AirSim use simple_flight, you can specify it in [settings.json](settings.md) as shown below. Note that this is default, so you don't have to do it explicitly.

```
"Vehicles": {
    "SimpleFlight": {
      "VehicleType": "SimpleFlight",
    }
}
```

By default, a vehicle using simple_flight is already armed which is why you would see its propellers spinning. However, if you don't want that then set `DefaultVehicleState` to `Inactive` like this:

```
"Vehicles": {
    "SimpleFlight": {
      "VehicleType": "SimpleFlight",
      "DefaultVehicleState": "Inactive"
    }
}
```

In this case, you will need to either manually arm by placing the RC sticks in the down-inward position or using the APIs.

For safety reasons, flight controllers disallow API control unless a human operator has consented its use using a switch on his/her RC. Also, when RC control is lost, the vehicle should disable API control and enter hover mode for safety reasons. To simplify things a bit, simple_flight enables API control without human consent using RC and even when RC is not detected by default. However you can change this using the following setting:

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

Finally, simple_flight uses a steppable clock by default which means that the clock advances when the simulator tells it to advance (unlike the wall clock which advances strictly according to the passage of time). This means the clock can be paused, for example, if code hits a breakpoint and there is zero variance in the clock (clock APIs provided by operating systems might have significant variance unless it is a "real time" OS). If you want simple_flight to use a wall clock instead then use following settings:

```
  "ClockType": "ScalableClock"
```
