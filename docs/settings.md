# AirSim Settings

## Where are Settings Stored?
AirSim configuration is specified in `settings.json` file located in `Documents\AirSim` folder on Windows (`~/Documents/AirSim` folder on Linux). The file is in usual [json format](](https://en.wikipedia.org/wiki/JSON). 

On first startup AirSim would create `settings.json` file with no settings. To avoid problems, always use ASCII format to save json file.

## Available Settings and Their Defaults
Below are complete list of settings available along with their default values. If any of the settings is missing from json file, then below default value is assumed. 

````
{
  "LocalHostIp": "127.0.0.1",
  "RecordUIVisible": true,
  "LogMessagesVisible": true,
  "ViewMode": "FlyWithMe",
  "DefaultVehicleConfig": "Pixhawk",
  "UsageScenario": "",
  "RpcEnabled": true,
  "PhysicsEngineName": "FastPhysicsEngine",
  "EnableCollisionPassthrogh": false,
  "LogMessagesVisible": true,
  "RosFlight": {
    "RemoteControlID": 0
  },
  "Recording": {
    "RecordOnMove": false,
    "RecordInterval": 0.05f
  },
  "SceneCaptureSettings" : {
    "Width": 256,
	  "Height": 144,
    "FOV_Degrees": 90,
    "AutoExposureSpeed": 100,
    "AutoExposureBias": 0,
    "AutoExposureMaxBrightness": 0.64f,
    "AutoExposureMinBrightness": 0.03f,
    "MotionBlurAmount": 0,
    "TargetGamma": 1.0f
  },  
  "DepthCaptureSettings" : {
    "Width": 256,
	  "Height": 144,
    "FOV_Degrees": 90,
    "AutoExposureSpeed": 100,
    "AutoExposureBias": 0,
    "AutoExposureMaxBrightness": 0.64f,
    "AutoExposureMinBrightness": 0.03f,
    "MotionBlurAmount": 0,
    "TargetGamma": 1.0f
  },  
  "SegCaptureSettings" : {
    "Width": 256,
	  "Height": 144,
    "FOV_Degrees": 90,
    "AutoExposureSpeed": 100,
    "AutoExposureBias": 0,
    "AutoExposureMaxBrightness": 0.64f,
    "AutoExposureMinBrightness": 0.03f,
    "MotionBlurAmount": 0,
    "TargetGamma": 1.0f
  },  
  "SimpleFlight": {
    "FirmwareName": "SimpleFlight",
    "RemoteControlID": 0,
    "ApiServerPort": 41451
  },
  "Pixhawk": {
    "FirmwareName": "Pixhawk",
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
The `SceneCaptureSettings`, `DepthCaptureSettings` and `SegCaptureSettings` determines how scene view, depth view and segmentation views are captures. The Width, Height and FOV settings should be self explanatory. The AutoExposureSpeed decides how fast eye adaptation works. We set to generally high value such as 100 to avoid artifacts in image capture. Simplarly we set MotionBlurAmount to 0 by default to avoid artifacts in groung truth images. For explanation of other settings, please see [this article](https://docs.unrealengine.com/latest/INT/Engine/Rendering/PostProcessEffects/AutomaticExposure/).

## Changing Flight Controller
The `DefaultVehicleConfig` decides which config settings will be used for your vehicles. By default its "Pixhaw"k and we have "RosFlight" and "SimpleFlight" in development, both of which are designed so you don't have to do separate HITL or SITL setups. For ["Computer Vision" mode](image_apis.md), use "SimpleFlight" as described in doc.

## Pixhawk and MavLink Related Settings
These settings define the Mavlink SystemId and ComponentId for the Simulator (SimSysID, SimCompID), and for an optional external renderer (ExtRendererSysID, ExtRendererCompID)
and the node that allows remote control of the drone from another app this is called the Air Control node (AirControlSysID, AirControlCompID).

If you want the simulator to also talk to your ground control app (like QGroundControl) you can also set the UDP address for that in case you want to run
that on a different machine (QgcHostIp,QgcPort).

You can connect the simulator to the LogViewer app, provided in this repo, by setting the UDP address for that (LogViewerHostIp,LogViewerPort).

And for each flying drone added to the simulator there is a named block of additional settings.  In the above you see the default name "Pixhawk".  
You can change this name from the Unreal Editor when you add a new BP_FlyingPawn asset.  You will see these properties grouped under the category
"MavLink". The mavlink node for this pawn can be remote over UDP or it can be connected
to a local serial port.  If serial then set UseSerial to true, otherwise set UseSerial to false and set the appropriate bard rate.  The default
of 115200 works with Pixhawk version 2 over USB.

### LocalHostIp Setting
Now when connecting to remote machines you may need to pick a specific ethernet adapter to reach those machines, for example, it might be
over ethernet or over wifi, or some other special virtual adapter or a VPN.  Your PC may have multiple networks, and those networks might not
be allowed to talk to each other, in which case the UDP messages from one network will not get through to the others.

So the LocalHostIp allows you to configure how you are reaching those machines.  The default of 127.0.0.1 is not able to reach external machines, 
this default is only used when everything you are talking to is contained on a single PC.




