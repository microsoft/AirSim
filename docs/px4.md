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
and follow all the instructions under `NuttX based hardware` to install prerequisites.  The full set of command then are as follows:
````
sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
sudo apt-get update
sudo apt-get install python-argparse git-core wget zip \
    python-empy qtcreator cmake build-essential genromfs -y
sudo apt-get install python-serial openocd \
    flex bison libncurses5-dev autoconf texinfo build-essential \
    libftdi-dev libtool zlib1g-dev \
    python-empy  -y
sudo apt-get remove gcc-arm-none-eabi gdb-arm-none-eabi binutils-arm-none-eabi gcc-arm-embedded
````
and the following command prompts for you to press ENTER so it must be run separately:
````
sudo add-apt-repository --remove ppa:team-gcc-arm-embedded/ppa
````
Then to finish up these:
````
pushd .
cd ~
wget https://launchpad.net/gcc-arm-embedded/5.0/5-2016-q2-update/+download/gcc-arm-none-eabi-5_4-2016q2-20160622-linux.tar.bz2
tar -jxf gcc-arm-none-eabi-5_4-2016q2-20160622-linux.tar.bz2
exportline="export PATH=$HOME/gcc-arm-none-eabi-5_4-2016q2/bin:\$PATH"
if grep -Fxq "$exportline" ~/.profile; then echo nothing to do ; else echo $exportline >> ~/.profile; fi
. ~/.profile
popd
sudo dpkg --add-architecture i386
sudo apt-get update
sudo apt-get install libc6:i386 libgcc1:i386 libstdc++5:i386 libstdc++6:i386
sudo apt-get install gcc-4.6-base:i386
````

So now you should be able to type this command `arm-none-eabi-gcc --version` and you should see:
````
arm-none-eabi-gcc (GNU Tools for ARM Embedded Processors) 5.4.1 20160609 (release) [ARM/embedded-5-branch revision 237715]
Copyright (C) 2015 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
````

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
and set `SitlIp` to 127.0.0.1 and `SitlPort` to 14556.
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

A Playstation 3 controller is confirmed to work as an AirSim controller. On Windows, an emulator to make it look like an Xbox 360 controller, is required however. Many different solutions are available online, the [x360ce](https://github.com/x360ce/x360ce) _Xbox 360 Controller Emulator_ is an excellent suggestion to get everything configured easily.
