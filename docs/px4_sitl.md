# Setting up PX4 Software-in-Loop

The [PX4](http://dev.px4.io) software provides a "software-in-loop" simulation (SITL) version of their stack that runs in Linux. If you are on Windows then you must
use the [Cygwin Toolchain](https://dev.px4.io/master/en/setup/dev_env_windows_cygwin.html) as the [Bash On Windows](https://dev.px4.io/master/en/setup/dev_env_windows_bash_on_win.html) toolchain no longer works for SITL.

1. From your bash terminal follow [these steps for Linux](http://dev.px4.io/starting-installing-linux.html) and follow **all** the instructions under `NuttX based hardware` to install prerequisites. We've also included out own copy of the [PX4 build instructions](px4_build.md) which is a bit more concise about what we need exactly.

2. Get the PX4 source code and build the posix SITL version of PX4:
    ```
    mkdir -p PX4
    cd PX4
    git clone https://github.com/PX4/Firmware.git
    cd Firmware
    git checkout v1.9.2  # Pick a well known "good" release tag.
    ```
3. Use following command to build and start PX4 firmware in SITL mode:
    ```
    make px4_sitl_default none_iris
    ```
   If you are using older version v1.8.* use this command instead: `make posix_sitl_ekf2  none_iris`.

4. You should see a message like this you `INFO  [simulator] Waiting for initial data on UDP port 14560` which means the SITL PX4 app is waiting for someone to connect.
You will also see information about which ports are configured in your SITL app.
The default ports have changed recently.  You should see something like this:
    ```
    INFO  [simulator] Waiting for simulator to connect on UDP port 14560
    INFO  [init] Mixer: etc/mixers/quad_w.main.mix on /dev/pwm_output0
    INFO  [mavlink] mode: Normal, data rate: 4000000 B/s on udp port 14570 remote port 14550
    INFO  [mavlink] mode: Onboard, data rate: 4000000 B/s on udp port 14580 remote port 14540
    ```

5. Now edit [AirSim settings](settings.md) file to make sure you have matching UDP port settings:
    ```json
    {
        "SettingsVersion": 1.2,
        "SimMode": "Multirotor",
        "Vehicles": {
            "PX4": {
                "VehicleType": "PX4Multirotor",
                "UseSerial": false,
                "TcpIp": "",
                "UdpPort": 14560,
                "GroundControlPort": 14570
            }
        }
    }
    ```
    Notice the `[simulator]` is using UDP, which is why the TCP address needs
    to be disabled using: `"TcpIp": ""`.  Note also that on older versions of
    PX4 the ground control port is not printed, in that case set it to 14556.

6. Run Unreal environment and it should connect to SITL via UDP.  You should see a bunch of messages from the SITL PX4 window from things like `[mavlink]` and `[commander]` and so on.  The following messages tell you that AirSim is connected
properly:
    ```
    INFO  [simulator] Simulator connected on UDP port 14560
    INFO  [mavlink] partner IP: 127.0.0.1
    INFO  [ecl/EKF] EKF GPS checks passed (WGS-84 origin set)
    INFO  [ecl/EKF] EKF commencing GPS fusion
    ```

    If you do not see these messages then check your UDP port settings.

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

Now close Unreal app, restart the `px4` app and re-start the unreal app.  In fact, every time you stop the unreal app you have top restart the `px4` app.

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

