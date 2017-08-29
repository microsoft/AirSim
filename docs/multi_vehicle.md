# Setting Up Multiple Vehicles

You will need some familiarity with Unreal environment to setup multiple vehicles. 

First Open up your environment in Unreal editor, go to [Plugin Contents view](working_with_plugin_contents.md) and drag and drop few Flying Pawn in to your environment. Typically you want to set their location close to ground. 

## IsFpvVehicle Property

If you click on Flying Pawn instances you dropped in environment you will see two properties under section "VehicleConfig". One is `IsFpvVehicle` which allows you to designate one of the pawns as "FPV". This means that when external camera will follow this specific drone. Also when you switch to FPV mode, you will see this drone's view. 

## VehicleConfigName Property

Another property that you can set is `VehicleConfigName`. This allows you to specify name of section in [settings.json](settings.md) that will have configuration for your vehicle. By default sections with names "PX4", "SimpleFlight" and "RosFlight" are automatically created and used if you had only one vehicle in the environment. However if you have multiple vehicles then you might want to create multiple sections, one for each vehicle. For example, if you have two PX4 vehicles in SITL, then you may create two section in settings.json like below. Notice that we are also specifying two different ports for RPC server so we can control vehicles independently. Now you can set `VehicleConfigName` for one of the FlyingPawn to "PX4_1" and other to "PX4_2".

```
{
  "PX4_1": {
    "FirmwareName": "PX4",
    "SitlIp": "127.0.0.1",
    "SitlPort": 14556,
    "UdpIp": "127.0.0.1",
    "UdpPort": 14560,
    "UseSerial": false,
    "ApiServerPort": 41451
  },
  "PX4_2": {
    "FirmwareName": "PX4",
    "SitlIp": "127.0.0.1",
    "SitlPort": 14557,
    "UdpIp": "127.0.0.1",
    "UdpPort": 14561,
    "UseSerial": false,
    "ApiServerPort": 41452
  }
}
```

## More Finer Control

If you are going deeper in to multi-vehicle scenarios, you may want to understand code more closely. Your starting point is function `ASimModeWorldMultiRotor::setupVehiclesAndCamera`. This function detects currently available pawns in the environment, initializes them and then
creates the "connector" object for each pawn. The "connector" object is simply and object that connects Unreal Engine's pawns to vehicles in Physics engine.

## Using simple_flight

AirSim uses "simple_flight" firmware by default which is built-it right in and thus eliminating any need for external SITL/HITL setups required by PX4. This might be better choice for multiple vehicles scenario. Also, we will be doing some significant refactoring of APIs so that there will be no need for individual RPC endpoint for each vehicle. In other words, one RPC endpoint would be sufficient to control 100s of vehicles in the environment.