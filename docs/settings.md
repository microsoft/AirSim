# AirSim Settings

## Where are Settings Stored?
Windows: `Documents\AirSim`
Linux: `~/Documents/AirSim`

The file is in usual [json format](https://en.wikipedia.org/wiki/JSON). On first startup AirSim would create `settings.json` file with no settings. To avoid problems, always use ASCII format to save json file.

## How to Chose Between Car and Multirotor?
The default is to use multirotor. To use car simple set `"SimMode": "Car"` like this:

```
{
  "SettingsVersion": 1.0,
  "SimMode": "Car"
}
```

To choose multirotor, set `"SimMode": ""`.

## Available Settings and Their Defaults
Below are complete list of settings available along with their default values. If any of the settings is missing from json file, then below default value is assumed. Please note that if setting has default value then its actual value may be chosen based on other settings. For example, ViewMode setting will have value "FlyWithMe" for drones and "SpringArmChase" for cars.

**WARNING:** Do not copy paste all of below in your settings.json. We stronly recommand leaving out any settings that you want to have default values from settings.json. Only copy settings that you want to *change* from default. Only required element is `"SettingsVersion": 1.0`.

````
{
  "DefaultVehicleConfig": "",
  "SimMode": "",
  "ClockType": "",
  "ClockSpeed": "1",
  "LocalHostIp": "127.0.0.1",
  "RecordUIVisible": true,
  "LogMessagesVisible": true,
  "ViewMode": "",
  "UsageScenario": "",
  "RpcEnabled": true,
  "EngineSound": true,
  "PhysicsEngineName": "",
  "EnableCollisionPassthrogh": false,
  "Recording": {
    "RecordOnMove": false,
    "RecordInterval": 0.05,
    "Cameras": [
		  { "CameraID": 0, "ImageType": 0, "PixelsAsFloat": false, "Compress": true }
	  ]
  },
  "CaptureSettings": [
    {
      "ImageType": 0,
      "Width": 256,
      "Height": 144,
      "FOV_Degrees": 90,
      "AutoExposureSpeed": 100,
      "AutoExposureBias": 0,
      "AutoExposureMaxBrightness": 0.64,
      "AutoExposureMinBrightness": 0.03,
      "MotionBlurAmount": 0,
      "TargetGamma": 1.0,
      "ProjectionMode": "",
      "OrthoWidth": 5.12,
      "Gimble": {
        "Stabilization": 0,
        "Pitch": NaN,
        "Roll": NaN,
        "Yaw": NaN
      }
    }
  ],
  "OriginGeopoint": {
    "Latitude": 47.641468,
    "Latitude": -122.140165,
    "Altitude": 122
  },
  "TimeOfDay": {
    "Enabled": false,
    "StartDateTime": "",
    "CelestialClockSpeed": 1,
    "StartDateTimeDst": false,
    "UpdateIntervalSecs": 60
  },
  "SubWindows": [
    {"WindowID": 0, "CameraID": 0, "ImageType": 3, "Visible": false},
    {"WindowID": 1, "CameraID": 0, "ImageType": 5, "Visible": false},
    {"WindowID": 2, "CameraID": 0, "ImageType": 0, "Visible": false}    
  ],
  "SimpleFlight": {
    "FirmwareName": "SimpleFlight",
    "DefaultVehicleState": "Armed",
    "RC": {
      "RemoteControlID": 0,
      "AllowAPIWhenDisconnected": false,
      "AllowAPIAlways": true
    },
    "ApiServerPort": 41451
  },
  "SegmentationSettings": {
    "InitMethod": "",
    "MeshNamingMethod": "",
    "OverrideExisting": false
  },
  "NoiseSettings": [
    {
      "Enabled": false,
      "ImageType": 0,

      "RandContrib": 0.2,
      "RandSpeed": 100000.0,
      "RandSize": 500.0,
      "RandDensity": 2,

      "HorzWaveContrib":0.03,			
      "HorzWaveStrength": 0.08,
      "HorzWaveVertSize": 1.0,
      "HorzWaveScreenSize": 1.0,
      
      "HorzNoiseLinesContrib": 1.0,
      "HorzNoiseLinesDensityY": 0.01,
      "HorzNoiseLinesDensityXY": 0.5,
      
      "HorzDistortionContrib": 1.0,
      "HorzDistortionStrength": 0.002
    }
  ],
  "AdditionalCameras": [
    { "X": 0.00, "Y": 0.5, "Z": 0.0, "Roll": 0.0, "Pitch": 0.0, "Yaw": 90.0 }
  ],
  "PX4": {
    "FirmwareName": "PX4",
    "LogViewerHostIp": "127.0.0.1",
    "LogViewerPort": 14388,
    "OffboardCompID": 1,
    "OffboardSysID": 134,
    "QgcHostIp": "127.0.0.1",
    "QgcPort": 14550,
    "SerialBaudRate": 115200,
    "SerialPort": "*",
    "SimCompID": 42,
    "SimSysID": 142,
    "SitlIp": "127.0.0.1",
    "SitlPort": 14556,
    "UdpIp": "127.0.0.1",
    "UdpPort": 14560,
    "UseSerial": true,
    "VehicleCompID": 1,
    "VehicleSysID": 135,
    "ApiServerPort": 41451
  }
}
````

## Image Capture Settings
The `CaptureSettings` determines how different image types such as scene, depth, disparity, surface normals and segmentation views are rendered. The Width, Height and FOV settings should be self explanatory. The AutoExposureSpeed decides how fast eye adaptation works. We set to generally high value such as 100 to avoid artifacts in image capture. Simplarly we set MotionBlurAmount to 0 by default to avoid artifacts in groung truth images. The `ProjectionMode` decides the projection used by the capture camera and can take value "perspective" (default) or "orthographic". If projection mode is "orthographic" then `OrthoWidth` determines width of projected area captured in meters.

For explanation of other settings, please see [this article](https://docs.unrealengine.com/latest/INT/Engine/Rendering/PostProcessEffects/AutomaticExposure/). 

The `ImageType` element determines which image type the settings applies to. The valid values are described in [ImageType section](image_apis.md#available-imagetype). In addition, we also support special value `ImageType: -1` to apply the settings to external camera (i.e. what you are looking at on the screen).

### Gimble
The `Gimble` element allows to freeze camera orientation for pitch, roll and/or yaw. This setting is ignored unless `ImageType` is -1. The `Stabilization` is defaulted to 0 meaning no gimble i.e. camera orientation changes with body orientation on all axis. The value of 1 means full stabilization. The value between 0 to 1 acts as a weight for fixed angles specified (in degrees, in world-frame) in `Pitch`, `Roll` and `Yaw` elements and orientation of the vehicle body. When any of the angles is ommitted from json or set to NaN, that angle is not stabilized (i.e. it moves along with vehicle body).

Note that `CaptureSettings` element is json array so you can add settings for multiple image types easily.

## Changing Flight Controller
The `DefaultVehicleConfig` decides which config settings will be used for your vehicles. By default we use [simple_flight](simple_flight.md) so you don't have to do separate HITL or SITL setups. We also support ["PX4"](px4_setup.md) for advanced users.

## PX4 and MavLink Related Settings
These settings define the Mavlink SystemId and ComponentId for the Simulator (SimSysID, SimCompID), and for an optional external renderer (ExtRendererSysID, ExtRendererCompID)
and the node that allows remote control of the drone from another app this is called the Air Control node (AirControlSysID, AirControlCompID).

If you want the simulator to also talk to your ground control app (like QGroundControl) you can also set the UDP address for that in case you want to run
that on a different machine (QgcHostIp,QgcPort).

You can connect the simulator to the LogViewer app, provided in this repo, by setting the UDP address for that (LogViewerHostIp,LogViewerPort).

And for each flying drone added to the simulator there is a named block of additional settings.  In the above you see the default name "PX4".   You can change this name from the Unreal Editor when you add a new BP_FlyingPawn asset.  You will see these properties grouped under the category "MavLink". The mavlink node for this pawn can be remote over UDP or it can be connected to a local serial port.  If serial then set UseSerial to true, otherwise set UseSerial to false and set the appropriate bard rate.  The default of 115200 works with Pixhawk version 2 over USB.

## Other Settings
### SimMode
Currently SimMode can be set to `""`, `"Multirotor"` or `"Car"`. The empty string value `""` means that use the default vehicle which is `"Multirotor"`. This determines which vehicle you would be using.

### PhysicsEngineName
For cars, we support only PhysX for now (regardless of value in this setting). For multirotors, we support `"FastPhysicsEngine"` only.

### ViewMode 
The ViewMode determines how you will view the vehicle. For multirotors, the default ViewMode is `"FlyWithMe"` while for cars the default ViewMode is `"SpringArmChase"`.

* FlyWithMe: Chase the vehicle from behind with 6 degrees of freedom
* GroundObserver: Chase the vehicle from 6' above the ground but with full freedome in XY plane.
* Fpv: View the scene from front camera of vehicle
* Manual: Don't move camera automatically. Use arrow keys and ASWD keys for move camera manually.
* SpringArmChase: Chase the vehicle with camera mounted on (invisible) arm that is attached to the vehicle via spring (so it has some latency in movement).
* NoDisplay: This will freeze rendering for main screen however rendering for subwindows, recording and APIs remain active. This mode is useful to save resources in "headless" mode where you are only interested in getting images and don't care about what gets rendered on main screen. This may also improve FPS for recording images.

### TimeOfDay
This section of settings controls the position of Sun in the environment. By default `Enabled` is false which means Sun's position is left at whatever was the default in the environment and it doesn't change over the time. If `Enabled` is true then Sun position is computed using longitude, latitude and altitude specified in `OriginGeopoint` section for the date specified in `StartDateTime` in the string format as `%Y-%m-%d %H:%M:%S`, for example, `2018-02-12 15:20:00`. If this string is empty then current date and time is used. If `StartDateTimeDst` is true then we adjust for day light savings time. The Sun's position is then contibuously updated at the interval specified in `UpdateIntervalSecs`. In some cases, it might be desirable to have celestial clock run faster or slower than simulation clock. This can be specified using `CelestialClockSpeed`, for example, value 100 means for every 1 second of simulation clock, Sun's position is advanced by 100 seconds so Sun will move in sky much faster.

### OriginGeopoint
This specifies the latitude, longitude and altitude of the coordinates (0, 0, 0) in Unreal Units in the Unreal Engine environment. The vehicle's home point is computed using this transformation. Note that all coordinates exposed via APIs are using NED system in SI units which means each vehicle starts at (0, 0, 0) in NED system. Time of Day settings are computed at (0, 0, 0) coordinates in Unreal Engine environment. This means that Sun position is computed for geographical coordinates specified in `OriginGeopoint` and time specified in `TimeOfDay` section in settings.json.

### EngineSound
To turn off the engine sound use [setting](settings.md) `"EngineSound": false`. Currently this setting applies only to car.

### SubWindows
This setting determines what is shown in each of 3 subwindows which are visible when you press 0 key. The WindowsID can be 0 to 2, CameraID is integer identifying camera number on the vehicle. ImageType integer value determines what kind of image gets shown according to [ImageType enum](image_apis.md#available-imagetype). For example, for car vehicles below shows driver view, front bumper view and rear view as scene, depth ans surface normals respectively.
```
  "SubWindows": [
    {"WindowID": 0, "ImageType": 0, "CameraID": 3, "Visible": true},
    {"WindowID": 1, "ImageType": 3, "CameraID": 0, "Visible": true},
    {"WindowID": 2, "ImageType": 6, "CameraID": 4, "Visible": true}
  ]
```
#### Recording
The recording feature allows you to record data such as position, orientation, velocity along with image get recorded in real time at given interval. You can start recording by pressing red Record button on lower right or R key. The data is recorded in `Documents\AirSim` folder, in a timestamped subfolder for each recording session, as csv file.

* RecordInterval: specifies minimal interval in seconds between capturing two images.
* RecordOnMove: specifies that do not record frame if there was vehicle's position or orientation hasn't changed.
* Cameras: this element controls which images gets recorded. By default scene image from camera 0 is recorded as compressed png format. This setting is json array so you can add more elements that allows to capture images from multiple cameras in different [image types](settings.md#image-capture-settings). When PixelsAsFloat is true, image is saved as [pfm](pfm.md) file instead of png file.

### ClockSpeed
Determines the speed of simulation clock with respect to wall clock. For example, value of 5.0 would mean simulation clock has 5 seconds elapsed when wall clock has 1 second elapsed (i.e. simulation is running faster). The value of 0.1 means that simulation clock is 10X slower than wall clock. The value of 1 means simulation is running in real time. It is important to realize that quality of simuation may decrease as the simulation clock runs faster. You might see artifacts like object moving past obstacles because collison is not detected. However slowing down simulation clock (i.e. values < 1.0) generally improves the quality of simulation.

### Segmentation Settings
The `InitMethod` determines how object IDs are initialized at startup to generate [segmentation](image_apis.md#segmentation). The value "" or "CommonObjectsRandomIDs" (default) means assign random IDs to each object at startup. This will generate segmentation view with random colors assign to each object. The value "None" means don't initialize object IDs. This will cause segmentation view to have single solid colors. This mode is useful if you plan to set up object IDs using [APIs](image_apis.md#segmentation) and it can save lot of delay at startup for large environments like CityEnviron.

 If `OverrideExisting` is false then initializtion does not alter non-zero object IDs already assigned otherwise it does.

 If `MeshNamingMethod` is "" or "OwnerName" then we use mesh's owner name to generate random hash as object IDs. If its "StaticMeshName" then we use static mesh's name to generate random hash as object IDs. Note that it is not possible to tell individual instances of the same static mesh apart this way, but the names are often more intuitive.

### LocalHostIp Setting
Now when connecting to remote machines you may need to pick a specific ethernet adapter to reach those machines, for example, it might be
over ethernet or over wifi, or some other special virtual adapter or a VPN.  Your PC may have multiple networks, and those networks might not
be allowed to talk to each other, in which case the UDP messages from one network will not get through to the others.

So the LocalHostIp allows you to configure how you are reaching those machines.  The default of 127.0.0.1 is not able to reach external machines, 
this default is only used when everything you are talking to is contained on a single PC.

### Image Noise Settings
The `NoiseSettings` allows to add noise to the specified image type with a goal of simulating camera sensor noise, interference and other artifacts. By default no noise is added, i.e., `Enabled: false`. If you set `Enabled: true` then following different types of noise and interference artifacts are enabled, each can be further tuned using setting. The noise effects are implemented as shader created as post processing material in Unreal Engine called [CameraSensorNoise](https://github.com/Microsoft/AirSim/blob/master/Unreal/Plugins/AirSim/Content/HUDAssets/CameraSensorNoise.uasset).

Demo of camera noise and inteference simulation:

[![AirSim Drone Demo Video](images/camera_noise_demo.png)](https://youtu.be/1BeCEZmQyp0)

#### Random noise
This adds random noise blobs with following parameters.
* `RandContrib`: This determines blend ratio of noise pixel with image pixel, 0 means no noise and 1 means only noise.
* `RandSpeed`: This determines how fast noise fluctuates, 1 means no fluctuation and higher values like 1E6 means full fluctuation.
* `RandSize`: This determines how coarse noise is, 1 means every pixel has its own noise while higher value means more than 1 pixels share same noise value.
* `RandDensity`: This determines how many pixels out of total will have noise, 1 means all pixels while higher value means lesser number of pixels (exponentially).

#### Horizontal bump distortion
This adds horizontal bumps / flickering / ghosting effect.
* `HorzWaveContrib`: This determines blend ratio of noise pixel with image pixel, 0 means no noise and 1 means only noise.
* `HorzWaveStrength`: This determines overall strength of the effect.
* `HorzWaveVertSize`: This determines how many vertical pixels would be effected by the effect.
* `HorzWaveScreenSize`: This determines how much of the screen is effected by the effect.

#### Horizontal noise lines
This adds regions of noise on horizontal lines.
* `HorzNoiseLinesContrib`: This determines blend ratio of noise pixel with image pixel, 0 means no noise and 1 means only noise.
* `HorzNoiseLinesDensityY`: This determines how many pixels in horizontal line gets affected.
* `HorzNoiseLinesDensityXY`: This determines how many lines on screen gets affected.

#### Horizontal line distortion
This adds fluctuations on horizontal line.
* `HorzDistortionContrib`: This determines blend ratio of noise pixel with image pixel, 0 means no noise and 1 means only noise.
* `HorzDistortionStrength`: This determines how large is the distortion.

### Additional Camera Settings
This allows to configure cameras in addition to the [standard ones](image_apis.md#available-cameras). This is only implemented in the multirotor drone at the moment.
The X, Y and Z fields specify the location of the new camera in the body frame, where X points forward, Y points to the right, and Z points downwards, and the values are given
in SI units (meters). Yaw, Pitch, and Roll specify the orientation of the camera, where Yaw denotes rotation around the Z axis, Pitch rotation around the Y axis and Roll rotation around the X axis.

This particular example adds a camera that is mounted on the right side of the drone, pointed to the right. The camera indices of the additional cameras are subsequent to the default ones, so camera index 5 is the first additional camera, camera index 6 the second additional camera, and so on.
