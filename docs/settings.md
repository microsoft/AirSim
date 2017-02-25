# settings.json

The AirSim Unreal plugin can use a settings.json file in the AirSim folder in your Documents folder if it exists.
Here's a [sample json file](https://en.wikipedia.org/wiki/JSON) file:

```
{
  "AirControlCompID": 1,
  "AirControlSysID": 134,
  "ExtRendererCompID": 1,
  "ExtRendererSysID": 167,
  "ExternalSimPort": 14588,
  "LocalHostIp": "127.0.0.1",
  "LogViewerHostIp": "127.0.0.1",
  "LogViewerPort": 14388,
  "Pixhawk": {
    "SerialBaudRate": 115200,
    "SerialPort": "*",
    "UdpIp": "127.0.0.1",
    "UdpPort": 14560,
    "UseSerial": true
  },
  "QgcHostIp": "127.0.0.1",
  "QgcPort": 14550,
  "SimCompID": 42,
  "SimSysID": 142
}

```

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

### LocalHostIp

Now when connecting to remote machines you may need to pick a specific ethernet adapter to reach those machines, for example, it might be
over ethernet or over wifi, or some other special virtual adapter or a VPN.  Your PC may have multiple networks, and those networks might not
be allowed to talk to each other, in which case the UDP messages from one network will not get through to the others.

So the LocalHostIp allows you to configure how you are reaching those machines.  The default of 127.0.0.1 is not able to reach external machines, 
this default is only used when everything you are talking to is contained on a single PC.




