# Unity AirSim Assets

Assets and settings required to start creating an *AirSim* simulation in Unity environment.

## Pre-requisites

- .Net 4.x framework
- *AirsimWrapper.dll* from [AirLibWrapper](https://gitlab.com/ankitdubeyryt/AirSim/tree/as-sept19/Unity/AirLibWrapper)

## Usage

There are couple of ways to start using this assets for a Unity project.

- Use this project as a base and add additional components\assets.
- Make a *unitypackage* by exporting all the assets in this project, also use the InputManger.asset, TagManager.asset and ProjectManager.asset from *ProjectSettings* folder in this project.

Once all the assets are included, follow the steps as mentioned below :

1. Add AirSimHUD prefab into the main scene.
2. Add vehicle model into the scene.
3. Add WindowCameras and CaptureCameras prefab as a child object of the vehicle model.
4. Add vehicle scripts to corresponding models :
    - Attach *Drone* script to the Drone model and assign the rotors as mentioned in **Rotor Poses** section. **Ensure the rotors count match the drone model**.
    - Attach *Car* Script to the car model, and assign wheel colliders along with wheel models in the *AirSimController* component. Configure car values based on car specifications. Default car configuration is mentioned in **Car Configuration** section.

## Prefabs

- ### AirSimHUD
    The HUD manager that sets up the window screens just like in Unreal. The initialization of `settings.json` is being done by this object.
    Failing to add this in the scene may cause unexpected behaviour.

- ### WindowCameras
    The windows at the bottom of the screen are being rendered by the cameras in this object. You may position the cameras as per
    your requirements.

- ### CaptureCameras
    The cameras needed to capture the data as per `setting.json` configurations. The cameras in the prefab can be positioned according to your
    requirements.

- ### RecorderCamera
    The camera component used in `CaptureCameras` for capturing/recording the vehicle data. This can be added as a part of `CaptureCameras` for
    additional data recording.

## Rotor Poses

>>>
        Quadcopter:
                x-axis

            (2)  |   (0)
                 |
            --------------      y-axis
                 |
            (1)  |   (3)

        Hexacopter:
                x-axis
            (2)      (4)
                \  /
                 \/
            (1)-----(0)         y-axis
                 /\
                /  \
            (5)     (3)
>>>

## settings.json - basic contents:
### For Car:
>>>
    {
    "SimMode": "Car",
    "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md",
    "SettingsVersion": 1.2
    }
>>>

### For Drone:
>>>
    {
    "SimMode": "Multirotor",
    "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md",
    "SettingsVersion": 1.2,
    "Vehicles": {
        "SimpleFlight": {
        "VehicleType": "SimpleFlight",
        "DefaultVehicleState": "Armed",
        "EnableCollisionPassthrogh": false,
        "EnableCollisions": true,
        "AllowAPIAlways": true,
        "RC": {
            "RemoteControlID": 0,
            "AllowAPIWhenDisconnected": false
        }
        }
    }
    }
>>>

## Car Configuration

**Note : This configuration can be adjusted based on *Car* model.**
>>>
    Car Drive Type : Four Wheel Drive
    Center Of Mass : 0, 0, 0
    Maximum Steer Angle : 25
    Steer Helper : 0.7
    Traction Control : 1
    Full Torque Over All Wheels : 2500
    Reverse Torque : 500
    Max Handbrake Torque : 100000000
    Downforce : 100
    Topspeed : 200
    No Of Gears : 5
    Rev Range Boundary : 1
    Slip Limit : 0.3
    Brake Torque : 20000
>>>
