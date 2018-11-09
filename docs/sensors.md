# Sensors in AirSim

AirSim currently supports the following sensors:
* Camera
* Imu
* Magnetometer
* Gps
* Barometer
* Distance
* Lidar

The cameras are currently configured a bit differently than other sensors. The camera configuration and apis are covered in other documents, e.g., [general settings](settings.md) and [image API](image_apis.md).

This document focuses on the configuration of other sensors.

## Default sensors

If not sensors are specified in the settings json, the the following sensors are enabled by default based on the simmode.

### Multirotor
* Imu
* Magnetometer
* Gps
* Barometer
### Car
* Gps
### ComputerVision
* None

Please see 'createDefaultSensorSettings' method in [AirSimSettings.hpp](https://github.com/Microsoft/AirSim/tree/master/AirLib//include/common/) 

## Configuration of Default Sensor list

A default sensor list can be configured in settings json.
e.g.,
```
    "DefaultSensors": {
        "Barometer": {
             "SensorType": 1,
             "Enabled" : true
        },
        "Gps": {
             "SensorType": 1,
             "Enabled" : true
        },
        "Lidar1": { 
             "SensorType": 6,
             "Enabled" : true,
             "NumberOfChannels": 16,
             "PointsPerSecond": 10000
        },
        "Lidar2": { 
             "SensorType": 6,
             "Enabled" : false,
             "NumberOfChannels": 4,
             "PointsPerSecond": 10000
        }
    },
```

## Configuration of vehicle specific sensor settings

A vehicle specific sensor list can be specified in the vehicle settings part of the json.
e.g.,

```
    "Vehicles": {

        "Drone1": {
            "VehicleType": "simpleflight",
            "AutoCreate": true,
            ...
            "Sensors": {
                "MyLidar1": { 
                    "SensorType": 6,
                    "Enabled" : true,
                    "NumberOfChannels": 16,
                    "PointsPerSecond": 10000,
                    "X": 0, "Y": 0, "Z": -1,
                    "DrawDebugPoints": true
                },
                "MyLidar2": { 
                    "SensorType": 6,
                    "Enabled" : true,
                    "NumberOfChannels": 4,
                    "PointsPerSecond": 10000,
                    "X": 0, "Y": 0, "Z": -1,
                    "DrawDebugPoints": true
                }
            }
        }
   }
```

If a vehicle provides its sensor list, it must provide the whole list. Selective add/remove/update of the default sensor list is NOT supported.

## Configuration of sensor settings

### Shared settings
There are two shared settings:
* SensorType
        An integer representing the sensor-type [SensorBase.hpp](https://github.com/Microsoft/AirSim/tree/master/AirLib//include/sensors/)
```
        enum class SensorType : uint {
            Barometer = 1,
            Imu = 2,
            Gps = 3,
            Magnetometer = 4,
            Distance = 5,
            Lidar = 6
        };
```
* Enabled
    Boolean

### Sensor specific settings
Each sensor-type has its own set of settings as well. Please see [lidar](lidar.md) for example of Lidar specific settings.

## Sensor APIs
Each sensor-type has its own set of APIs currently. Please see [lidar](lidar.md) for example of Lidar specific APIs.
