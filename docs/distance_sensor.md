## Distance Sensor

By default, Distance Sensor points to the front of the vehicle. It can be pointed in any direction by modifying the settings

Configurable Parameters -

Parameter           | Description
--------------------|------------
X Y Z               | Position of the sensor relative to the vehicle (in NED, in meters) (Default (0,0,0)-Multirotor, (0,0,-1)-Car)
Yaw Pitch Roll      | Orientation of the sensor relative to the vehicle (degrees) (Default (0,0,0))
MinDistance         | Minimum distance measured by distance sensor (metres, only used to fill Mavlink message for PX4) (Default 0.2m)
MaxDistance         | Maximum distance measured by distance sensor (metres) (Default 40.0m)
ExternalController  | Whether data is to be sent to external controller such as ArduPilot or PX4 if being used (default `true`)

For example, to make the sensor point towards the ground (for altitude measurement similar to barometer), the orientation can be modified as follows -

```json
"Distance": {
    "SensorType": 5,
    "Enabled" : true,
    "Yaw": 0, "Pitch": -90, "Roll": 0
}
```

**Note:** For Cars, the sensor is placed 1 meter above the vehicle center by default. This is required since otherwise the sensor gives strange data due it being inside the vehicle. This doesn't affect the sensor values say when measuring the distance between 2 cars. See [`PythonClient/car/distance_sensor_multi.py`](https://github.com/Microsoft/AirSim/blob/main/PythonClient/car/distance_sensor_multi.py) for an example usage.