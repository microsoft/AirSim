# Setting up PX4 Software-in-Loop

The [PX4](http://dev.px4.io) software provides a "software-in-loop" simulation (SITL) version of their stack that runs in Linux. Sorry it doesn't run in Windows, but if you install [BashOnWindows](https://msdn.microsoft.com/en-us/commandline/wsl/install_guide)
you can build and run it there.

1. From your Linux bash terminal follow [these steps for Linux](http://dev.px4.io/starting-installing-linux.html) and follow **all** the instructions under `NuttX based hardware` to install prerequisites. We've also included out own copy of the [PX4 build instructions](px4_build.md) which is a bit more concise about what we need exactly.

2. Get the PX4 source code and build the posix SITL version of PX4:
    ```
    mkdir -p PX4
    cd PX4
    git clone https://github.com/PX4/Firmware.git
    cd Firmware
    make posix_sitl_default
    ```
3. Use following command to start PX4 firmware in SITL mode:
    ```
    ./build/posix_sitl_default/px4 ./posix-configs/SITL/init/ekf2/iris
    ```
4. You should see a message like this you `INFO  [simulator] Waiting for initial data on UDP port 14560` which means the SITL PX4 app is waiting for someone to connect.
5. Now edit [AirSim settings](settings.md) file to make sure you have followings:
    ```
    {
        "SettingsVersion": 1.2,
        "SimMode": "Multirotor",
        "Vehicles": {
            "PX4": {
                "VehicleType": "PX4Multirotor",
                "UseSerial": false
            }
        }
    }
    ```
6. Run Unreal environment and it should connect to SITL via UDP.  You should see a bunch of messages from the SITL PX4 window from things like `[mavlink]` and `[commander]` and so on.
7. You should also be able to use QGroundControl just like with flight controller hardware. Note that as we don't have physical board, RC cannot be connected directly to it. So the alternatives are either use XBox 360 Controller or connect your RC using USB (for example, in case of FrSky Taranis X9D Plus) or using trainer USB cable to PC. This makes your RC look like joystick. You will need to do extra set up in QGroundControl to use virtual joystick for RC control.

## Setting GPS origin

PX4 SITL mode needs to be configured to get the home location correct.  Run the following in the PX4 console window so that the origin matches that which is setup in AirSim AVehiclePawnBase::HomeLatitude and HomeLongitude.

````
param set LPE_LAT 47.641468
param set LPE_LON -122.140165
````

You might also want to set this one so that the drone automatically hovers after each offboard control command (the default setting is to land):

````
param set COM_OBL_ACT 1
````

Now close Unreal app, restart `./build_posix_sitl_default/src/firmware/posix/px4` and re-start the unreal app.  

## Check the Home Position

If you are using DroneShell to execute commands (arm, takeoff, etc) then you should wait until the Home position is set. You will see the PX4 SITL console output this message:

````
INFO  [commander] home: 47.6414680, -122.1401672, 119.99
INFO  [tone_alarm] home_set
````

Now DroneShell 'pos' command should report this position and the commands should be accepted by PX4.  If you attempt to takeoff without a home position you will see the message:

````
WARN  [commander] Takeoff denied, disarm and re-try
````

After home position is set check the local position reported by 'pos' command :

````
Local position: x=-0.0326988, y=0.00656854, z=5.48506
````

If the z coordinate is large like this then takeoff might not work as expected.  Resetting the SITL and simulation should fix that problem.

## No Remote Control

If you plan to fly with no remote control, just using DroneShell commands for example, then you will need to set the following parameters to stop the PX4 from triggering "failsafe mode on" every time a move command is finished.

````
param set NAV_RCL_ACT 0
param set NAV_DLL_ACT 0
````

NOTE: Do `NOT` do this on a real drone as it is too dangerous to fly without these failsafe measures.

## Using VirtualBox Ubuntu

If you want to run the above posix_sitl in a `VirtualBox Ubuntu` machine then it will have a different ip address from localhost. So in this case you need to edit the [settings file](settings.md) and change the UdpIp and SitlIp to the ip address of your virtual machine
set the  LocalIpAddress to the address of your host machine running the Unreal engine. 

## Remote Controller

There are several options for flying the simulated drone using a remote control or joystick like xbox gamepad. See [remote controllers](remote_control.md#RC_Setup_for_PX4)

