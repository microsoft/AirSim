# How to Use Lidar in AirSim

AirSim supports Lidar for multirotors and cars. 

The enablement of lidar and the other lidar settings can be configured via AirSimSettings json.
Please see [general sensors](sensors.md) for information on configruation of general/shared sensor settings.

## Enabling lidar on a vehicle
* By default, lidars are not enabled. To enable lidar, set the SensorType and Enabled attributes in settings json.
```
        "Lidar1": { 
             "SensorType": 6,
             "Enabled" : true,
```
* Multiple lidars can be enabled on a vehicle.

## Lidar configuration
The following parameters can be configured right now via settings json.

Parameter                 | Description
--------------------------| ------------
NumberOfChannels          | Number of channels/lasers of the lidar
Range                     | Range, in meters
PointsPerSecond           | Number of points captured per second
RotationsPerSecond        | Rotations per second
VerticalFOVUpper          | Vertical FOV upper limit for the lidar, in degrees
VerticalFOVLower          | Vertical FOV lower limit for the lidar, in degrees
X Y Z                     | Position of the lidar relative to the vehicle (in NED, in meters)                     
Roll Pitch Yaw            | Roation of the lidar relative to the vehicle  (in degrees)

e.g.,
```
        "Lidar1": { 
             "SensorType": 6,
             "Enabled" : true,
             "NumberOfChannels": 16,
             "PointsPerSecond": 10000,
             "X": 0, "Y": 0, "Z": -1,
```

## Server side visualization for debugging
Be default, the lidar points are not drawn on the viewport. To enable the drawing of hit laser points on the viewport, please enable setting 'DrawDebugPoints' via settings json.
e.g.,
```
        "Lidar1": { 
             ...
             "DrawDebugPoints": true
        },
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
* Visualization of lidar data on client side.