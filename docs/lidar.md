# How to Use Lidar in AirSim

AirSim supports Lidar for multirotors and cars. 

The enablement of lidar and the configuration currently requires manual changes to the code; but that will be fixed soon.

## Enabling lidar on a vehicle
For multirotors, please update the EnabledSensors struct in file [MultiRotorParams.hpp](../AirLib/include/vehicles/multirotor) and set lidar to `true`.

```cpp

// File location: https://github.com/Microsoft/AirSim/blob/master/AirLib/include/vehicles/multirotor/MultiRotor.hpp

struct EnabledSensors {
        bool imu = true;
        bool magnetometer = true;
        bool gps = true;
        bool barometer = true;
        bool distance = false; 
        bool lidar = false;     //set this to true 
    };
```

For cars, please update the EnabledSensors struct in file [CarApiBase.hpp](../AirLib/include/vehicles/car/api) and set lidar to `true`.

```cpp

// File location: https://github.com/Microsoft/AirSim/blob/master/AirLib/include/vehicles/car/api/CarApiBase.hpp

struct EnabledSensors {
        bool imu = false;
        bool magnetometer = false;
        bool gps = false;
        bool barometer = false;
        bool distance = false; 
        bool lidar = true;     //set this to true 
    };
```

## Lidar configuration
Update the Lidar configuration as needed in file [LidarSimpleParams.hpp](../AirLib/include/sensors/lidar).
The following parameters can be configured right now:

Parameter                      | Description
------------------------------ | ------------
number_of_channels             | Number of channels/lasers of the lidar
range                          | Range, in meters
points_per_second              | Number of points captured per second
horizontal_rotation_frequency  | Rotations per second
vertical_FOV_Upper             | Vertical FOV upper limit for the lidar, in degrees
vertical_FOV_Lower             | Vertical FOV lower limit for the lidar, in degrees
relative_pose                  | Position and rotation of the lidar relative to the vehicle                     

## Server side visualization for debugging
Be default, the lidar points are not drawn on the viewport. To enable the drawing of hit laser points on the viewport, please update the code in file [SimModeBase.h](../Unreal/Plugins/AirSim/Source/SimMode).
Set `draw_lidar_debug_points_` to true.

```cpp

bool draw_lidar_debug_points_ = false;    // set this to true

```

## Client API 
Use `getLidarData()` API to retrieve the Lidar data. 
* The API returns a Point-Cloud as a flat array of floats along with a timestamp of the capture.
* The floats represent [x,y,z] coordinate for each point hit within the range in the last scan.
* The coordinates are in the local vehicle NED like all other AirSim APIs.

### Python Examples
[drone_lidar.py](../PythonClient/multirotor)
[car_lidar.py](../PythonClient/car)

## Coming soon
* Configuration of lidar parameters via AirSim settings.
* Visualization of lidar data on client side.