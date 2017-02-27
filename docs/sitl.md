# Why Flight Controller
"Wait!" you ask, "Why do you need *flight controller* for simulator?". Flight controller has two essential elements (among *many* others) called pose estimation and high speed real time controller loop that generates motor signals for the drone. We consume these signals directly in our physics engine so the simulation is as close to real thing as possible. This is known as "hardware-in-loop" OR HIL simulation. HIL simulation is preferred for testing 
new flight algorithms in the simulator because they have a better chance of working on the real drone when we are done testing.

If you do not want to spend money on this extra hardware then you can probably get away with "software-in-loop" simulation (SITL). 
In this case, the firmware runs in your computer as opposed to flight controller. One downside to SITL mode is that your PC usually has more power than 
the tiny flight controller on your drone, so you may not get the same flying performance when trying the same thing on a real drone.
But it's still useful when you don't have a flight controller handy. It is just a few more steps to set up as shown below.

# Software-In-Loop Simulation (SITL)
1. Install [BashOnWindows](https://msdn.microsoft.com/en-us/commandline/wsl/install_guide).  You can also do this on an Ubuntu machine, 
but in that case you will have to use real ip addresses instead of localhost (See Using VirtualBox Ubuntu below).
2. Type `bash` at Windows command prompt. Follow [these steps for Linux](http://dev.px4.io/starting-installing-linux.html) 
and follow all the instructions under `NuttX based hardware` to install prerequisites.  Technically you don't need the full NuttX 
toolchain for cross-compiling ARM Cortex M4 code, but it won't hurt to get all that.  The PX4 make system probably checks that you have it all, 
even though you probably don't need that to build the posix-SITL version which will be running on your intel chipset.
3. Get the PX4 source code and build the posix SITL version of PX4:
```
git clone https://github.com/PX4/Firmware.git
cd Firmware
git submodule update --init --recursive
make posix_sitl_default
```
4. Use following command to start PX4 firmware in SITL mode:
```
./build_posix_sitl_default/src/firmware/posix/px4 ./posix-configs/SITL/init/lpe/iris
```
5. You should see a message like this you `INFO  [simulator] Waiting for initial data on UDP port 14560` which means the SITL PX4 app is
waiting for someone to connect.
6. Now edit [settings file](settings.md) with `UdpIp` address 127.0.0.1 and `UdpPort` 14560, set `UseSerial` to false
7. Run Unreal environment and it should connect to SITL via UDP.  You should see a bunch of messages from the SITL PX4 window from
things like `local_position_estimator` and `commander` and so on.
8. You should also be able to use QGroundControl just like with actual [flight controller harware](prereq.md). 
Note that as we don't have physical board, RC cannot be connected directly to it. 
So the alternatives are either use XBox 360 Controller or connect your RC using USB port if it has it 
(for example, in case of [FrSky Taranis X9D Plus](prereq.md)) or using trainer USB cable to PC. 
This makes your RC look like joystick. You will need to do extra set up in QGroundControl to use RC control as below.

# Using VirtualBox Ubuntu

If you want to run the above posix_sitl in a `VirtualBox Ubuntu` machine then it will have a different ip address from localhost.
So in this case you need to set the [settings file](settings.md) with the UdpIp address set to the ip address of your virtual machine
and the LocalIpAddress is the address of your host machine running the Unreal engine. 

# Using Joystick/Gamepad (Alternative to RC)
Why do you need RC for simulator? Because usual joysticks are not very accurate and in fact very "noisy" for flying! 
We have tried it and gladly switched back to regular RC. But just in case you want to try it as well, here are the instructions.

1. Connect Joystick/Gamepad such as XBox 360 controller to PC. Start QGroundControl.
2. Click on the purple "Q" icon on left in tool bar to reveal the Preferences panel.
3. Go to General tab and check the Virtual Joystick checkbox.
4. Go back to settings screen (gears icon), click on Parameters tab, type `COM_RC_IN_MODE` in search box and change its value to either `Joystick/No RC Checks` or `Virtual RC by Joystick`.
5. You should now see a new tab "Joystick" where you can do the calibration.
