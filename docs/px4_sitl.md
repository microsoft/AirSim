# Setting up PX4 Software-in-Loop

The [PX4](http://dev.px4.io) software provides a "software-in-loop" simulation (SITL) version of
their stack that runs in Linux. If you are on Windows then you can use the [Cygwin
Toolchain](https://dev.px4.io/master/en/setup/dev_env_windows_cygwin.html) or you can use the
[Windows subsystem for Linux](https://docs.microsoft.com/en-us/windows/wsl/install-win10) and follow
the PX4 Linux toolchain setup.

If you are using WSL2 please read these [additional
instructions](px4_sitl_wsl2.md).

**Note** that every time you stop the unreal app you have to restart the `px4` app.

1. From your bash terminal follow [these steps for
   Linux](https://docs.px4.io/master/en/dev_setup/dev_env_linux.html) and follow **all** the
   instructions under `NuttX based hardware` to install prerequisites. We've also included our own
   copy of the [PX4 build instructions](px4_build.md) which is a bit more concise about what we need
   exactly.

2. Get the PX4 source code and build the posix SITL version of PX4:
    ```
    mkdir -p PX4
    cd PX4
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
    cd PX4-Autopilot
    ```
    And find the latest stable release from [https://github.com/PX4/PX4-Autopilot/releases](https://github.com/PX4/PX4-Autopilot/releases)
    and checkout the source code matching that release, for example:
    ```
    git checkout v1.11.3
    ```

3. Use following command to build and start PX4 firmware in SITL mode:
    ```
    make px4_sitl_default none_iris
    ```
   If you are using older version v1.8.* use this command instead: `make posix_sitl_ekf2 none_iris`.

4. You should see a message saying the SITL PX4 app is waiting for the simulator (AirSim) to connect.
You will also see information about which ports are configured for mavlink connection to the PX4 app.
The default ports have changed recently, so check them closely to make sure AirSim settings are correct.
    ```
    INFO  [simulator] Waiting for simulator to connect on TCP port 4560
    INFO  [init] Mixer: etc/mixers/quad_w.main.mix on /dev/pwm_output0
    INFO  [mavlink] mode: Normal, data rate: 4000000 B/s on udp port 14570 remote port 14550
    INFO  [mavlink] mode: Onboard, data rate: 4000000 B/s on udp port 14580 remote port 14540
    ```

    Note: this is also an interactive PX4 console, type `help` to see the
    list of commands you can enter here.  They are mostly low level PX4
    commands, but some of them can be useful for debugging.

5. Now edit [AirSim settings](settings.md) file to make sure you have matching UDP and TCP port settings:
    ```json
    {
        "SettingsVersion": 1.2,
        "SimMode": "Multirotor",
        "ClockType": "SteppableClock",
        "Vehicles": {
            "PX4": {
                "VehicleType": "PX4Multirotor",
                "UseSerial": false,
                "LockStep": true,
                "UseTcp": true,
                "TcpPort": 4560,
                "ControlPortLocal": 14540,
                "ControlPortRemote": 14580,
                "Sensors":{
                    "Barometer":{
                        "SensorType": 1,
                        "Enabled": true,
                        "PressureFactorSigma": 0.0001825
                    }
                },
                "Parameters": {
                    "NAV_RCL_ACT": 0,
                    "NAV_DLL_ACT": 0,
                    "COM_OBL_ACT": 1,
                    "LPE_LAT": 47.641468,
                    "LPE_LON": -122.140165
                }
            }
        }
    }
    ```
    Notice the PX4 `[simulator]` is using TCP, which is why we need to add: `"UseTcp": true,`.
    Notice we are also enabling `LockStep`, see [PX4 LockStep](px4_lockstep.md) for more
    information. The `Barometer` setting keeps PX4 happy because the default AirSim barometer has a
    bit too much noise generation.  This setting clamps that down a bit which allows PX4 to achieve
    GPS lock more quickly.

6. Open incoming TCP port 4560 and incoming UDP port 14540 using your firewall configuration.

7. Now run your Unreal AirSim environment and it should connect to SITL PX4 via TCP. You should see
   a bunch of messages from the SITL PX4 window. Specifically, the following messages tell you that
   AirSim is connected properly and GPS fusion is stable:
    ```
    INFO  [simulator] Simulator connected on UDP port 14560
    INFO  [mavlink] partner IP: 127.0.0.1
    INFO  [ecl/EKF] EKF GPS checks passed (WGS-84 origin set)
    INFO  [ecl/EKF] EKF commencing GPS fusion
    ```

    If you do not see these messages then check your port settings.

8. You should also be able to use QGroundControl with SITL mode.  Make sure there is no Pixhawk
   hardware plugged in, otherwise QGroundControl will choose to use that instead.  Note that as we
   don't have a physical board, an RC cannot be connected directly to it. So the alternatives are
   either use XBox 360 Controller or connect your RC using USB (for example, in case of FrSky
   Taranis X9D Plus) or using trainer USB cable to your PC. This makes your RC look like a joystick.
   You will need to do extra set up in QGroundControl to use virtual joystick for RC control.  You
   do not need to do this unless you plan to fly a drone manually in AirSim.  Autonomous flight
   using the Python API does not require RC, see `No Remote Control` below.

## Setting GPS origin

Notice the above settings are provided in the `params` section of the `settings.json` file:
```
    "LPE_LAT": 47.641468,
    "LPE_LON": -122.140165,
```

PX4 SITL mode needs to be configured to get the home location correct.
The home location needs to be set to the same coordinates defined in  [OriginGeopoint](settings.md#origingeopoint).

You can also run the following in the SITL PX4 console window to check
that these values are set correctly.

```
param show LPE_LAT
param show LPE_LON
```

## Smooth Offboard Transitions

Notice the above setting is provided in the `params` section of the `settings.json` file:
```
    "COM_OBL_ACT": 1
```

This tells the drone automatically hover after each offboard control command finishes (the default
setting is to land).  Hovering is a smoother transition between multiple offboard commands.  You can
check this setting by running the following PX4 console command:

```
param show COM_OBL_ACT
```

## Check the Home Position

If you are using DroneShell to execute commands (arm, takeoff, etc) then you should wait until the
Home position is set. You will see the PX4 SITL console output this message:

```
INFO  [commander] home: 47.6414680, -122.1401672, 119.99
INFO  [tone_alarm] home_set
```

Now DroneShell 'pos' command should report this position and the commands should be accepted by PX4.
If you attempt to takeoff without a home position you will see the message:

```
WARN  [commander] Takeoff denied, disarm and re-try
```

After home position is set check the local position reported by 'pos' command :

```
Local position: x=-0.0326988, y=0.00656854, z=5.48506
```

If the z coordinate is large like this then takeoff might not work as expected.  Resetting the SITL
and simulation should fix that problem.

## WSL 2

Windows Subsystem for Linux version 2 operates in a Virtual Machine.  This requires
additional setup - see [additional instructions](px4_sitl_wsl2.md).

## No Remote Control

Notice the above setting is provided in the `params` section of the `settings.json` file:
```
    "NAV_RCL_ACT": 0,
    "NAV_DLL_ACT": 0,
```

This is required if you plan to fly the SITL mode PX4 with no remote control, just using python
scripts, for example.  These parameters stop the PX4 from triggering "failsafe mode on" every time a
move command is finished.  You can use the following PX4 command to check these values are set
correctly:

```
param show NAV_RCL_ACT
param show NAV_DLL_ACT
```

NOTE: Do `NOT` do this on a real drone as it is too dangerous to fly without these failsafe measures.

## Manually set parameters

You can also run the following in the PX4 console to set all these parameters manually:

```
param set NAV_RCL_ACT 0
param set NAV_DLL_ACT 0
```

## Setting up multi-vehicle simulation

You can simulate multiple drones in SITL mode using AirSim. However, this requires setting up
multiple instances of the PX4 firmware simulator to be able to listen for each vehicle's connection
on a separate TCP port (4560, 4561, etc). Please see [this dedicated page](px4_multi_vehicle.md) for
instructions on setting up multiple instances of PX4 in SITL mode.

## Using VirtualBox Ubuntu

If you want to run the above posix_sitl in a `VirtualBox Ubuntu` machine then it will have a
different ip address from localhost. So in this case you need to edit the [settings
file](settings.md) and change the UdpIp and SitlIp to the ip address of your virtual machine set the
LocalIpAddress to the address of your host machine running the Unreal engine.

## Remote Controller

There are several options for flying the simulated drone using a remote control or joystick like
xbox gamepad. See [remote controllers](remote_control.md#rc-setup-for-px4)
