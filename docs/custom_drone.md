# AirLib on a Real Drone

The AirLib library can be compiled and deployed on the companion computer on a real drone. For our testing, we mounted a Gigabyte Brix BXi7-5500 ultra compact PC on the drone connected to the Pixhawk flight controller over USB. The Gigabyte PC is running Ubuntu, so we are able to SSH into it over Wi-Fi: 

![Flamewheel](images/Flamewheel.png)

Once connected you can run MavLinkTest with this command line:
```
MavLinkTest -serial:/dev/ttyACM0,115200 -logdir:. 
```
And this will produce a log file of the flight which can then be used for [playback in the simulator](playback.md).

You can also add `-proxy:192.168.1.100:14550` to connect MavLinkTest to a remote computer where you can run QGroundControl or our 
[PX4 Log Viewer](log_viewer.md) which is another handy way to see what is going on with your drone.

MavLinkTest then has some simple commands for testing your drone, here's a simple example of some commands:

```
arm
takeoff 5
orbit 10 2
```

This will arm the drone, takeoff of 5 meters, then do an orbit pattern radius 10 meters, at 2 m/s.
Type '?' to find all available commands.

**Note:** Some commands (for example, `orbit`) are named differently and have different syntax in MavLinkTest and DroneShell (for example, `circlebypath -radius 10 -velocity 21`).

When you land the drone you can stop MavLinkTest and copy the *.mavlink log file that was generated.

# DroneServer and DroneShell

Once you are happy that the MavLinkTest is working, you can also run DroneServer and DroneShell as follows. First, run MavLinkTest with a local proxy to send everything to DroneServer:

```
MavLinkTest -serial:/dev/ttyACM0,115200 -logdir:. -proxy:127.0.0.1:14560
```
Change ~/Documents/AirSim/settings.json to say "serial":false, because we want DroneServer to look for this UDP connection.

```
DroneServer 0
```

Lastly, you can now connect DroneShell to this instance of DroneServer and use the DroneShell commands to fly your drone:

```
DroneShell
==||=>
        Welcome to DroneShell 1.0.
        Type ? for help.
        Microsoft Research (c) 2016.

Waiting for drone to report a valid GPS location...
==||=> requestcontrol
==||=> arm
==||=> takeoff
==||=> circlebypath -radius 10 -velocity 2
```

## PX4 Specific Tools
You can run the MavlinkCom library and MavLinkTest app to test the connection
between your companion computer and flight controller.  

## How Does This Work?
AirSim uses MavLinkCom component developed by @lovettchris. The MavLinkCom has a proxy architecture where you can open a connection to PX4 either using serial or UDP and then other components share this connection. When PX4 sends MavLink message, all components receive that message. If any component sends a message then it's received by PX4 only. This allows you to connect any number of components to PX4 [This code](https://github.com/Microsoft/AirSim/blob/master/AirLib/include/vehicles/multirotor/controllers/MavLinkDroneController.hpp#L793) opens a connection for LogViewer and QGC. You can add something more if you like.

If you want to use QGC + AirSim together than you will need QGC to let own the serial port. QGC opens up TCP connection that acts as a proxy so any other component can connect to QGC and send MavLinkMessage to QGC and then QGC forwards that message to PX4. So you tell AirSim to connect to QGC and let QGC own serial port.

For companion board, the way we did it earlier was to have Gigabyte Brix on the drone. This x86 full-fledged computer that will connect to PX4 through USB. We had Ubuntu on Brix and ran [DroneServer](https://github.com/Microsoft/AirSim/tree/master/DroneServer). The DroneServer created an API endpoint that we can talk to via C++ client code (or Python code) and it translated API calls to MavLink messages. That way you can write your code against the same API, test it in the simulator and then run the same code on an actual vehicle. So the companion computer has DroneServer running along with client code. 
