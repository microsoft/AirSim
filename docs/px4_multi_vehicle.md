# Setting up multi-vehicle PX4 simulation

The [PX4 SITL stack](px4_sitl.md) comes with a `sitl_multiple_run.sh` shell script that runs multiple instances of the PX4 binary. This would allow the SITL stack to listen to connections from multiple AirSim vehicles on multiple TCP ports starting from 4560.
However, the provided script does not let us view the PX4 console. If you want to run the instances manually while being able to view each instance's console (**Recommended**) see [this section](px4_multi_vehicle.md#starting-sitl-instances-with-px4-console) 

## Setting up multiple instances of PX4 Software-in-Loop

**Note** you have to build PX4 with `make px4_sitl_default none_iris` as shown [here](px4_sitl.md#setting-up-px4-software-in-loop) before trying to run multiple PX4 instances.

1. From your bash (or Cygwin) terminal go to the PX4 Firmware directory and run the `sitl_multiple_run.sh` script while specifying the number of vehicles you need
    ```
    cd PX4
    cd Firmware
    ./Tools/sitl_multiple_run.sh 2    # 2 here is the number of vehicles/instances 
    ```
    This starts multiple instances that listen to TCP ports 4560 to 4560+i where 'i' is the number of vehicles/instances specified

2. You should get a confirmation message that says that old instances have been stopped and new instances have been started
    ```
    killing running instances
    starting instance 0 in /cygdrive/c/PX4/home/PX4/Firmware/build/px4_sitl_default/instance_0
    starting instance 1 in /cygdrive/c/PX4/home/PX4/Firmware/build/px4_sitl_default/instance_1
    ```
3. Now edit [AirSim settings](settings.md) file to make sure you have matching TCP port settings for the set number of vehicles and to make sure that both vehicles do not spawn on the same point. 

    For example, these settings would spawn two PX4Multirotors where one of them would try to connect to PX4 SITL at port `4560` and the other at port `4561`. It also makes sure the vehicles spawn at `0,1,0` and `0,-1,0` to avoid collision:
    ```json
    {
        "SettingsVersion": 1.2,
        "SimMode": "Multirotor",
        "Vehicles": {
            "Drone1": {
                "VehicleType": "PX4Multirotor",
                "UseSerial": false,
                "UseTcp": true,
                "TcpPort": 4560,
                "ControlPort": 14580,
                "X": 0, "Y": 1, "Z": 0
            },
            "Drone2": {
                "VehicleType": "PX4Multirotor",
                "UseSerial": false,
                "UseTcp": true,
                "TcpPort": 4561,
                "ControlPort": 14580,          
                "X": 0, "Y": -1, "Z": 0
            }
        }
      }
    ```
    You can add more than two vehicles but you will need to make sure you adjust the TCP port for each (ie: vehicle 3's port would be `4562` and so on..) and adjust the spawn point.

4. Now run your Unreal AirSim environment and it should connect to SITL PX4 via TCP.
If you are running the instances with the [PX4 console visible](px4_multi_vehicle.md#Starting-sitl-instances-with-px4-console), you should see a bunch of messages from each SITL PX4 window.
Specifically, the following messages tell you that AirSim is connected properly and GPS fusion is stable:
    ```
    INFO  [simulator] Simulator connected on UDP port 14560
    INFO  [mavlink] partner IP: 127.0.0.1
    INFO  [ecl/EKF] EKF GPS checks passed (WGS-84 origin set)
    INFO  [ecl/EKF] EKF commencing GPS fusion
    ```

    If you do not see these messages then check your port settings.

5. You should also be able to use QGroundControl with SITL mode.  Make sure
there is no Pixhawk hardware plugged in, otherwise QGroundControl will choose
to use that instead.  Note that as we don't have a physical board, an RC cannot be connected directly to it. So the alternatives are either use XBox 360 Controller or connect your RC using USB (for example, in case of FrSky Taranis X9D Plus) or using trainer USB cable to your PC. This makes your RC look like a joystick. You will need to do extra set up in QGroundControl to use virtual joystick for RC control.  You do not need to do this unless you plan to fly a drone manually in AirSim.  Autonomous flight using the Python
API does not require RC, see [`No Remote Control`](px4_sitl.md#No-Remote-Control).

## Starting SITL instances with PX4 console

If you want to start your SITL instances while being able to view the PX4 console, you will need to run the shell scripts found [here](/PX4Scripts) rather than `sitl_multiple_run.sh`.
Here is how you would do so:

**Note** This script also assumes PX4 is built with `make px4_sitl_default none_iris` as shown [here](px4_sitl.md#setting-up-px4-software-in-loop) before trying to run multiple PX4 instances.

1. From your bash (or Cygwin) terminal go to the PX4 directory and get the scripts (place them in a subdirectory called Scripts win the PX4 directory as shown)
    ```
    cd PX4
    mkdir -p Scripts
    cd Scripts
    wget https://raw.githubusercontent.com/microsoft/AirSim/master/PX4Scripts/sitl_kill.sh
    wget https://raw.githubusercontent.com/microsoft/AirSim/master/PX4Scripts/run_airsim_sitl.sh
    ```
    **Note** the shell scripts expect the `Scripts` and `Firmware` directories to be within the same parent directory. Also, you may need to make the scripts executable by running `chmod +x sitl_kill.sh` and `chmod +x run_airsim_sitl.sh`.
2. Run the `sitl_kill.sh` script to kill all active PX4 SITL instances 
    ```
    ./sitl_kill.sh
    ```
    
3. Run the `run_airsim_sitl.sh` script while specifying which instance you would like to run in the current terminal window (the first instance would be numbered 0)
    ```
    ./run_airsim_sitl.sh 0 # first instance = 0
    ```
    
    You should see the PX4 instance starting and waiting for AirSim's connection as it would with a single instance.
    ```
    ______  __   __    ___
    | ___ \ \ \ / /   /   |
    | |_/ /  \ V /   / /| |
    |  __/   /   \  / /_| |
    | |     / /^\ \ \___  |
    \_|     \/   \/     |_/

    px4 starting.
    INFO  [px4] Calling startup script: /bin/sh /cygdrive/c/PX4/home/PX4/Firmware/etc/init.d-posix/rcS 0
    INFO  [dataman] Unknown restart, data manager file './dataman' size is 11798680 bytes
    INFO  [simulator] Waiting for simulator to connect on TCP port 4560
    ```
4. Open a new terminal and go to the Scripts directory and start the next instance
    ```
    cd PX4
    cd Scripts
    ./run_airsim_sitl.sh 1  # ,2,3,4,..,etc
    ```

5. Repeat step 4 for as many instances as you would like to start
 
6. Run your Unreal AirSim environment and it should connect to SITL PX4 via TCP (assuming your settings.json file has the right ports).
