# How to Use Lidar in AirSim

AirSim supports Lidar for multirotors and cars.

The enablement of lidar and the other lidar settings can be configured via AirSimSettings json.
Please see [general sensors](sensors.md) for information on configruation of general/shared sensor settings.

## Enabling lidar on a vehicle
* By default, lidars are not enabled. To enable lidar, set the SensorType and Enabled attributes in settings json.

```json
    "Lidar1": {
         "SensorType": 6,
         "Enabled" : true,
    }
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
HorizontalFOVStart        | Horizontal FOV start for the lidar, in degrees
HorizontalFOVEnd          | Horizontal FOV end for the lidar, in degrees
VerticalFOVUpper          | Vertical FOV upper limit for the lidar, in degrees
VerticalFOVLower          | Vertical FOV lower limit for the lidar, in degrees
X Y Z                     | Position of the lidar relative to the vehicle (in NED, in meters)
Roll Pitch Yaw            | Orientation of the lidar relative to the vehicle  (in degrees, yaw-pitch-roll order to front vector +X)
DataFrame                 | Frame for the points in output ("VehicleInertialFrame" or "SensorLocalFrame")

e.g.

```json
{
    "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings_json.md",
    "SettingsVersion": 1.2,

    "SimMode": "Multirotor",

     "Vehicles": {
		"Drone1": {
			"VehicleType": "simpleflight",
			"AutoCreate": true,
			"Sensors": {
			    "LidarSensor1": {
					"SensorType": 6,
					"Enabled" : true,
					"NumberOfChannels": 16,
					"RotationsPerSecond": 10,
					"PointsPerSecond": 100000,
					"X": 0, "Y": 0, "Z": -1,
					"Roll": 0, "Pitch": 0, "Yaw" : 0,
					"VerticalFOVUpper": -15,
					"VerticalFOVLower": -25,
					"HorizontalFOVStart": -20,
					"HorizontalFOVEnd": 20,
					"DrawDebugPoints": true,
					"DataFrame": "SensorLocalFrame"
				},
				"LidarSensor2": {
				   "SensorType": 6,
					"Enabled" : true,
					"NumberOfChannels": 4,
					"RotationsPerSecond": 10,
					"PointsPerSecond": 10000,
					"X": 0, "Y": 0, "Z": -1,
					"Roll": 0, "Pitch": 0, "Yaw" : 0,
					"VerticalFOVUpper": -15,
					"VerticalFOVLower": -25,
					"DrawDebugPoints": true,
					"DataFrame": "SensorLocalFrame"
				}
			}
		}
    }
}
```

## Server side visualization for debugging

By default, the lidar points are not drawn on the viewport. To enable the drawing of hit laser points on the viewport, please enable setting `DrawDebugPoints` via settings json.

```json
    "Lidar1": {
         ...
         "DrawDebugPoints": true
    },
```

**Note:** Enabling `DrawDebugPoints` can cause excessive memory usage and crash in releases `v1.3.1`, `v1.3.0`. This has been fixed in master and should work in later releases

## Client API

Use `getLidarData()` API to retrieve the Lidar data.

* The API returns a Point-Cloud as a flat array of floats along with the timestamp of the capture and lidar pose.
* Point-Cloud:
    * The floats represent [x,y,z] coordinate for each point hit within the range in the last scan.
    * The frame for the points in the output is configurable using "DataFrame" attribute -
        * "" or `VehicleInertialFrame` -- default; returned points are in vehicle inertial frame (in NED, in meters)
        * `SensorLocalFrame` -- returned points are in lidar local frame (in NED, in meters)
* Lidar Pose:
    * Lidar pose in the vehicle inertial frame (in NED, in meters)
    * Can be used to transform points to other frames.
* Segmentation: The segmentation of each lidar point's collided object

### Python Examples
- [drone_lidar.py](https://github.com/microsoft/AirSim/blob/master/PythonClient/multirotor/drone_lidar.py)
- [car_lidar.py](https://github.com/microsoft/AirSim/blob/master/PythonClient/car/car_lidar.py)

## Coming soon
* Visualization of lidar data on client side.
