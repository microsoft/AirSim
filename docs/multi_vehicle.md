# Multiple Vehicles in AirSim

Since release 1.2, AirSim is fully enabled for multiple vehicles. This capability allows you to create multiple vehicles easily and use APIs to control them.

## Creating Multiple Vehicles
It's as easy as specifying them in [settings.json](settings.md). The `Vehicles` element allows you to specify list of vehicles you want to create along with their initial positions and orientations. The positions are specified in NED coordinates in SI units with origin set at Player Start component in Unreal environment. The orientation is specified as Yaw, Pitch and Roll in degrees.

### Creating Multiple Cars
```json
{
	"SettingsVersion": 1.2,
	"SimMode": "Car",
	
	"Vehicles": {
		"Car1": {
		  "VehicleType": "PhysXCar",
		  "X": 4, "Y": 0, "Z": -2
		},
		"Car2": {
		  "VehicleType": "PhysXCar",
		  "X": -4, "Y": 0, "Z": -2,
      "Yaw": 90
		}
  }
}
```

### Creating Multiple Drones
```json
{
	"SettingsVersion": 1.2,
	"SimMode": "Multirotor",
	
	"Vehicles": {
		"Drone1": {
		  "VehicleType": "SimpleFlight",
		  "X": 4, "Y": 0, "Z": -2,
      "Yaw": -180
		},
		"Drone2": {
		  "VehicleType": "SimpleFlight",
		  "X": 8, "Y": 0, "Z": -2
		}

    }
}
```

## Using APIs for Multiple Vehicles
The new APIs since AirSim 1.2 allows you to specify `vehicle_name`. This name corresponds to keys in json settings (for example, Car1 or Drone2 above). 

[Example code for cars](https://github.com/Microsoft/AirSim/tree/master/PythonClient//car/multi_agent_car.py)

[Example code for multirotors](https://github.com/Microsoft/AirSim/tree/master/PythonClient//multirotor/multi_agent_drone.py)

### Demo
[![AirSimMultiple Vehicles Demo Video](images/demo_multi_vehicles.png)](https://youtu.be/35dgcuLuF5M)
