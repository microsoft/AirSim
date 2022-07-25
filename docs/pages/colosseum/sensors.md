# Sensors in AirSim

AirSim currently supports the following sensors.
Each sensor is associated with a integer enum specifying its sensor type.

* Camera
* Barometer = 1
* Imu = 2
* Gps = 3
* Magnetometer = 4
* Distance Sensor = 5
* Lidar = 6

**Note** :  Cameras are configured differently than the other sensors and do not have an enum associated with them.    Look at [general settings](settings.md) and [image API](image_apis.md) for camera config and API.

## Default sensors

If no sensors are specified in the `settings.json`, then the following sensors are enabled by default based on the sim mode.

### Multirotor
* Imu
* Magnetometer
* Gps
* Barometer

### Car
* Gps

### ComputerVision
* None

Behind the scenes, `createDefaultSensorSettings` method in [AirSimSettings.hpp](https://github.com/Microsoft/AirSim/blob/main/AirLib/include/common/AirSimSettings.hpp) sets up the above sensors with their default parameters, depending on the sim mode specified in the `settings.json` file.

## Configuring the default sensor list

The default sensor list can be configured in settings json:

```json
"DefaultSensors": {
    "Barometer": {
        "SensorType": 1,
        "Enabled" : true,
        "PressureFactorSigma": 0.001825,
        "PressureFactorTau": 3600,
        "UncorrelatedNoiseSigma": 2.7,
        "UpdateLatency": 0,
        "UpdateFrequency": 50,
        "StartupDelay": 0

    },
    "Imu": {
        "SensorType": 2,
        "Enabled" : true,
        "AngularRandomWalk": 0.3,
        "GyroBiasStabilityTau": 500,
        "GyroBiasStability": 4.6,
        "VelocityRandomWalk": 0.24,
        "AccelBiasStabilityTau": 800,
        "AccelBiasStability": 36
    },
    "Gps": {
        "SensorType": 3,
        "Enabled" : true,
        "EphTimeConstant": 0.9,
        "EpvTimeConstant": 0.9,
        "EphInitial": 25,
        "EpvInitial": 25,
        "EphFinal": 0.1,
        "EpvFinal": 0.1,
        "EphMin3d": 3,
        "EphMin2d": 4,
        "UpdateLatency": 0.2,
        "UpdateFrequency": 50,
        "StartupDelay": 1
    },
    "Magnetometer": {
        "SensorType": 4,
        "Enabled" : true,
        "NoiseSigma": 0.005,
        "ScaleFactor": 1,
        "NoiseBias": 0,
        "UpdateLatency": 0,
        "UpdateFrequency": 50,
        "StartupDelay": 0
    },
    "Distance": {
        "SensorType": 5,
        "Enabled" : true,
        "MinDistance": 0.2,
        "MaxDistance": 40,
        "X": 0, "Y": 0, "Z": -1,
        "Yaw": 0, "Pitch": 0, "Roll": 0,
        "DrawDebugPoints": false
    },
    "Lidar2": {
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
    }
},
```

## Configuring vehicle-specific sensor list

A vehicle can override a subset of the default sensors listed above. A Lidar and Distance sensor are
not added to a vehicle by default, so those you need to add this way. Each sensor must have a valid
"SensorType" and a subset of the properties can be defined that override the default values shown
above and you can set Enabled to false to disable a specific type of sensor.

```json
"Vehicles": {

    "Drone1": {
        "VehicleType": "SimpleFlight",
        "AutoCreate": true,
        ...
        "Sensors": {
            "Barometer":{
                "SensorType": 1,
                "Enabled": true,
                "PressureFactorSigma": 0.0001825
            },
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

### Sensor specific settings

For detailed information on the meaning of these sensor settings
see the following pages:

- [Lidar sensor settings](lidar.md)
- [Distance sensor settings](distance_sensor.md)

##### Server side visualization for debugging

Be default, the points hit by distance sensor are not drawn on the viewport. To enable the drawing of hit points on the viewport, please enable setting `DrawDebugPoints` via settings json. E.g. -

```json
"Distance": {
    "SensorType": 5,
    "Enabled" : true,
    ...
    "DrawDebugPoints": true
}
```

## Sensor APIs
Jump straight to [`hello_drone.py`](https://github.com/Microsoft/AirSim/blob/main/PythonClient/multirotor/hello_drone.py) or [`hello_drone.cpp`](https://github.com/Microsoft/AirSim/blob/main/HelloDrone/main.cpp) for example usage, or see follow below for the full API.

### Barometer
```cpp
msr::airlib::BarometerBase::Output getBarometerData(const std::string& barometer_name, const std::string& vehicle_name);
```

```python
barometer_data = client.getBarometerData(barometer_name = "", vehicle_name = "")
```

### IMU
```cpp
msr::airlib::ImuBase::Output getImuData(const std::string& imu_name = "", const std::string& vehicle_name = "");
```

```python
imu_data = client.getImuData(imu_name = "", vehicle_name = "")
```

### GPS
```cpp
msr::airlib::GpsBase::Output getGpsData(const std::string& gps_name = "", const std::string& vehicle_name = "");
```
```python
gps_data = client.getGpsData(gps_name = "", vehicle_name = "")
```

### Magnetometer
```cpp
msr::airlib::MagnetometerBase::Output getMagnetometerData(const std::string& magnetometer_name = "", const std::string& vehicle_name = "");
```
```python
magnetometer_data = client.getMagnetometerData(magnetometer_name = "", vehicle_name = "")
```

### Distance sensor
```cpp
msr::airlib::DistanceSensorData getDistanceSensorData(const std::string& distance_sensor_name = "", const std::string& vehicle_name = "");
```
```python
distance_sensor_data = client.getDistanceSensorData(distance_sensor_name = "", vehicle_name = "")
```

### Lidar
See the [lidar page](lidar.md) for Lidar API.
