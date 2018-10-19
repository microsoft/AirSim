# Unity AirSim Drone Demo

A Unity project to demonstrate the usage of [AirSimAssets](https://gitlab.com/prabhakarperigmail/AirSimUnityPort/tree/add-unity-modules/Unity/AirSimAssets) for *Drone* simulation.

## Usage

Open the *DroneDemo* scene in the project to test the simulation. To use the keyboard/Joystick please refer to **Controls** section.
Keys 1, 2, and 3 are used to toggle windows of different camera views.
Press *Record* button(Red button) located at the right bottom corner of the screen, to toggle recording of the simulation data. The recorded data can be found at **Documents\AirSim\(Date of recording)**

## Controls

To control the drone through keyboard\joystick please add the following block to `settings.json` file in *Documents/AirSim* folder.

>>>
    "DefaultVehicleConfig": "SimpleFlight",
    "SimMode": "",
    "SimpleFlight": {
        "FirmwareName": "SimpleFlight",
        "DefaultVehicleState": "Armed",
        "RC": {
          "RemoteControlID": 1,
          "AllowAPIWhenDisconnected": false,
          "AllowAPIAlways": true
        },
        "ApiServerPort": 41451
      }
>>>

### Keyboard Controls

>>>
    WASD\Arrow keys: Drone movement.
    Page Up : Elevate the drone.
    Page Down : Lower the drone.
    Q\E : Yaw of the drone.
>>>

### JoyStick Controls

Note : Tested with XBox 360 controller
>>>
    Left Stick : Drone movement
    Right Stick(Vertical) : Elevate\Lower the drone.
    Right Stick(Horizontal) : Yaw of the drone.
>>>
