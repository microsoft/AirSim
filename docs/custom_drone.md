# Custom Drone

Parts of the AirSim stack can be used on a real drone.  For example, you can run the MavlinkCom library and MavLinkTest app to test the connection
between your `big brain` and `little brain`.  For our testing we mounted a Gigabyte Brix BXi7-5500 ultra compact PC on the drone connected to the Pixhawk flight controller over USB.  The Gigabyte PC is running Ubuntu, so we are able to SSH into it over Wifi: 

![Flamewheel](images/Flamewheel.png)

Once connected you can run MavLinkTest with this command line:
````
MavLinkTest -serial:/dev/ttyACM0,115200 -logdir:. 
````
And this will produce a log file of the flight which can then be used for [playback in the simulator](playback.md).

You can also add `-proxy:192.168.1.100:14550` to connect MavLinkTest to a remote computer where you can run QGroundControl or our 
[PX4 Log Viewer](log_viewer.md) which is another handy way to see what is going on with your drone.

MavLinkTest then has some simple commands for testing your drone, here's a simple example of some commands:

````
arm
takeoff 5
orbit 10 2
````

This will arm the drone, takeoff ot 5 meters, then do an orbit pattern radius 10 meters, at 2 m/s.
Type '?' to find all available commands.

# DroneServer and DroneShell

One the MavLinkConnection is working, you can also run DroneServer and DroneShell to control the drone that way.


