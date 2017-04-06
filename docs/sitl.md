# Why Flight Controller
"Wait!" you ask, "Why do you need *flight controller* for simulator?". Flight controller has two essential elements (among *many* others) called pose estimation and high speed real time controller loop that generates motor signals for the drone. We consume these signals directly in our physics engine so the simulation is as close to real thing as possible. This is known as "hardware-in-loop" OR HIL simulation. HIL simulation is preferred for testing 
new flight algorithms in the simulator because they have a better chance of working on the real drone when we are done testing.

If you do not want to spend money on this extra hardware then you can probably get away with "software-in-loop" simulation (SITL). 
In this case, the firmware runs in your computer as opposed to flight controller. One downside to SITL mode is that your PC usually has more power than 
the tiny flight controller on your drone, so you may not get the same flying performance when trying the same thing on a real drone.
But it's still useful when you don't have a flight controller handy. It is just a few more steps to set up as shown below.

## Software-In-Loop Simulation (SITL)

The [PX4](http://dev.px4.io) software provides a "software-in-loop" simulation (SITL) version of their stack that runs in Linux.
Sorry it doesn't run in Windows, but if you install [BashOnWindows](https://msdn.microsoft.com/en-us/commandline/wsl/install_guide)
you can build and run it there.

1. From your Linux bash terminal follow [these steps for Linux](http://dev.px4.io/starting-installing-linux.html)
and follow **all** the instructions under `NuttX based hardware` to install prerequisites.  We've also included
out own copy of the [PX4 setup instructions](px4.md) which is a bit more concise about what we need exactly.

3. Get the PX4 source code and build the posix SITL version of PX4:
```
git clone https://github.com/PX4/Firmware.git
cd Firmware
make posix_sitl_default
```
4. Use following command to start PX4 firmware in SITL mode:
```
./build_posix_sitl_default/src/firmware/posix/px4 ./posix-configs/SITL/init/ekf2/iris
```
5. You should see a message like this you `INFO  [simulator] Waiting for initial data on UDP port 14560` which means the SITL PX4 app is
waiting for someone to connect.
6. Now edit [settings file](settings.md) with `UdpIp` address 127.0.0.1 and `UdpPort` 14560, set `UseSerial` to false
and set `SitlIp` to 127.0.0.1 and `SitlPort` to 14556.
7. Run Unreal environment and it should connect to SITL via UDP.  You should see a bunch of messages from the SITL PX4 window from
things like `local_position_estimator` and `commander` and so on.
8. You should also be able to use QGroundControl just like with actual [flight controller harware](prereq.md). 
Note that as we don't have physical board, RC cannot be connected directly to it. 
So the alternatives are either use XBox 360 Controller or connect your RC using USB port if it has it 
(for example, in case of [FrSky Taranis X9D Plus](prereq.md)) or using trainer USB cable to PC. 
This makes your RC look like joystick. You will need to do extra set up in QGroundControl to use RC control as below.

## Setting GPS origin

PX4 SITL mode needs to be configured to get the home location correct.  Run the following in the PX4 console window
so that the origin matches that which is setup in AirSim AVehiclePawnBase::HomeLatitude and HomeLongitude.

````
param set LPE_LAT 47.641468
param set LPE_LON -122.140165
````

Now close Unreal app, restart `./build_posix_sitl_default/src/firmware/posix/px4` and re-start the unreal app.  

## Check the Home Position

If you are using DroneShell to execute commands (arm, takeoff, etc) then you should wait until the Home position is set.
You will see the PX4 SITL console output this message:

````
INFO  [commander] home: 47.6414680, -122.1401672, 119.99
INFO  [tone_alarm] home_set
````

Now DroneShell 'pos' command should report this position and the commands should be accepted by PX4.  If you attempt to
takeoff without a home position you will see the message:

````
WARN  [commander] Takeoff denied, disarm and re-try
````

After home position is set check the local position reported by 'pos' command :

````
Local position: x=-0.0326988, y=0.00656854, z=5.48506
````

If the z coordinate is large like this then takeoff might not work as expected.  Resetting the SITL and simulation 
should fix that problem.

## No Remote Control

If you plan to fly with no remote control, just using DroneShell commands for example, then you will need to set the
following parameters to stop the PX4 from triggering "failsafe mode on" every time a move command is finished.

````
param set NAV_RCL_ACT 0
param set NAV_DLL_ACT 0
````

NOTE: Do `NOT` do this on a real drone as it is too dangerous to fly without these failsafe measures.

## Using VirtualBox Ubuntu

If you want to run the above posix_sitl in a `VirtualBox Ubuntu` machine then it will have a different ip address from localhost.
So in this case you need to edit the [settings file](settings.md) and change the UdpIp and SitlIp to the ip address of your virtual machine
set the  LocalIpAddress to the address of your host machine running the Unreal engine. 

## Using Joystick/Gamepad (Alternative to RC)
Why do you need RC for simulator? Because usual joysticks are not very accurate and in fact very "noisy" for flying! 
We have tried it and gladly switched back to regular RC. But just in case you want to try it as well, here are the instructions.

1. Connect Joystick/Gamepad such as XBox 360 controller to PC. Start QGroundControl.
2. Click on the purple "Q" icon on left in tool bar to reveal the Preferences panel.
3. Go to General tab and check the Virtual Joystick checkbox.
4. Go back to settings screen (gears icon), click on Parameters tab, type `COM_RC_IN_MODE` in search box and change its value to either `Joystick/No RC Checks` or `Virtual RC by Joystick`.
5. You should now see a new tab "Joystick" where you can do the calibration.

A Playstation 3 controller is confirmed to work as an AirSim controller. On Windows, an emulator to make it look like an Xbox 360 controller, is required however. Many different solutions are available online, for example [x360ce Xbox 360 Controller Emulator](https://github.com/x360ce/x360ce).  `Note:` don't use x360ce if you have a real XBox controller.
