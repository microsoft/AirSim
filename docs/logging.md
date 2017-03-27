# Logging

When reporting issues it is good to collect some logs so we can reproduce the problem
and then debug it.

These are the useful things you can do:

1. Add the following information to your issue report:
````
Operating System: Windows 10 64bit
CPU: Intel Core i7
GPU: Nvidia GTX 1080
RAM: 32 gb
Flight Controller: Pixhawk v2
Remote Control: Futaba
````

2. The following command will connect MavLinkTest app to the Simulator and enable logging
of all mavlink commands to and from the PX4.
````
MavLinkTest -server:127.0.0.1:14550 -logdir:d:\temp
````

You will then see log files organized by date in d:\temp\logs.  Please include the specific
*input.mavlink and *output.mavlink files in a zip file and attach those to the issue report.

3. If you have modified the default ~/Document/AirSim/settings.json, please include your
settings also.

4. If you are using SITL mode, please copy the log file that SITL produces when drone is armed.
Please zip it and attach it to the issue. The SITL terminal will contain the path to the log file,
it should look something like this `INFO  [logger] Opened log file: rootfs/fs/microsd/log/2017-03-27/20_02_49.ulg`.

5. If you are using Pixhawk hardware in HIL mode, then please setthis parameter: SYS_LOGGER=1
using QGroundControl, then after the experiment, download the log file using QGroundControl
and zip that and attach it to the issue.