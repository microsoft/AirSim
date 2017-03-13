# PX4 Development

PX4 is an open source flight controller stack from [dev.px4.io](http://dev.px4.io) that works on various hardware.
To get setup to build the PX4 stack, follow [these steps for Linux](http://dev.px4.io/starting-installing-linux.html) 
(or [BashOnWindows](https://msdn.microsoft.com/en-us/commandline/wsl/install_guide)) and follow all the instructions under `NuttX based hardware` to install prerequisites.  

We have copied the  full set of commands from those pages here:
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
Then to finish up, run these:
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
sudo apt-get install gcc-5.4-base:i386
````

So now you should be able to type this command `arm-none-eabi-gcc --version` and you should see:
````
arm-none-eabi-gcc (GNU Tools for ARM Embedded Processors) 5.4.1 20160609 (release) [ARM/embedded-5-branch revision 237715]
Copyright (C) 2015 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
````

Note: as of right now this version of arm-none-eabi-gcc compiler does not work in BashOnWindows because it is a 32 bit binary,
which BashOnWindows does not yet supporet on a 64 bit machine.  It is possible to download a 64 bit version of this compiler
but the steps are more involved.  But you don't actually need arm-none-eabi-gcc to build the SITL version of PX4.  The only
reason we recommend downloading all the tools is because the PX4 build system checks to make sure you have them.


3. Get the PX4 source code:
```
git clone https://github.com/PX4/Firmware.git
cd Firmware
````

4. Building the PX4 firmware for SITL mode execution:
````
make posix_sitl_default
````

5. Building the real firmware to run on Pixhawk hardware
```
make px4fmu-v2_default
```

Note: see note about about BashOnWindows.