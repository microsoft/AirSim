# Building PX4

## Source code

Getting the PX4 source code is easy:
```
sudo apt-get install git
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools
cd PX4-Autopilot
```

Now to build it you will need the right tools.

## PX4 Build tools

The full instructions are available on the [dev.px4.io](https://docs.px4.io/master/en/dev_setup/building_px4.html) website,
but we've copied the relevant subset of those instructions here for your convenience.

(Note that [BashOnWindows](https://msdn.microsoft.com/en-us/commandline/wsl/install_guide)) can be used to build
the PX4 firmware, just follow the BashOnWindows instructions at the bottom of this page) then proceed with the 
Ubuntu setup for PX4.

## Build SITL version

Now you can make the SITL version that runs in posix, from the Firmware folder you created above:
```
make px4_sitl_default none_iris
```

Note: this build system is quite special, it knows how to update git submodules (and there's a lot
of them), then it runs cmake (if necessary), then it runs the build itself. So in a way the root
Makefile is a meta-meta makefile :-)   You might see prompts like this:

```shell
 *******************************************************************************
 *   IF YOU DID NOT CHANGE THIS FILE (OR YOU DON'T KNOW WHAT A SUBMODULE IS):  *
 *   Hit 'u' and <ENTER> to update ALL submodules and resolve this.            *
 *   (performs git submodule sync --recursive                                  *
 *    and git submodule update --init --recursive )                            *
 *******************************************************************************
```
Every time you see this prompt type 'u' on your keyboard.

It shouldn't take long, about 2 minutes. If all succeeds, the last line will link the `px4` app,
which you can then run using the following:

```
make px4_sitl_default none_iris
```

And you should see output that looks like this:

```
creating new parameters file
creating new dataman file

______  __   __    ___ 
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting.

18446744073709551615 WARNING: setRealtimeSched failed (not run as root?)
ERROR [param] importing from 'rootfs/eeprom/parameters' failed (-1)
Command 'param' failed, returned 1
  SYS_AUTOSTART: curr: 0 -> new: 4010
  SYS_MC_EST_GROUP: curr: 2 -> new: 1
INFO  [dataman] Unkown restart, data manager file 'rootfs/fs/microsd/dataman' size is 11797680 bytes
  BAT_N_CELLS: curr: 0 -> new: 3
  CAL_GYRO0_ID: curr: 0 -> new: 2293768
  CAL_ACC0_ID: curr: 0 -> new: 1376264
  CAL_ACC1_ID: curr: 0 -> new: 1310728
  CAL_MAG0_ID: curr: 0 -> new: 196616

```

so this is good, first run sets up the px4 parameters for SITL mode. Second run has less output.
This app is also an interactive console where you can type commands. Type 'help' to see what they
are and just type ctrl-C to kill it. You can do that and restart it any time, that's a great way to
reset any wonky state if you need to (it's equivalent to a Pixhawk hardware reboot).

## ARM embedded tools

If you plan to build the PX4 firmware for real Pixhawk hardware then you will need the gcc
cross-compiler for ARM Cortex-M4 chipset. You can get this compiler by PX4 DevGuide, specifically
this is in their `ubuntu_sim_nuttx.sh` setup script.

After following those setup instructions you can verify the install by entering this command `arm-none-eabi-gcc --version`.  You should see the following output:

```
arm-none-eabi-gcc (GNU Tools for Arm Embedded Processors 7-2017-q4-major) 7.2.1 20170904 (release) [ARM/embedded-7-branch revision 255204]
Copyright (C) 2017 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```

## Build PX4 for ARM hardware

Now  you  can build the PX4 firmware for running on real pixhawk hardware:

```
make px4_fmu-v4
```

This build will take a little longer because it is building a lot more including the NuttX real time OS,
all the drivers for the sensors in the Pixhawk flight controller, and more.  It is also running the compiler
in super size-squeezing mode so it can fit all that in a 1 megabyte ROM !!

One nice tid bit is you can plug in your pixhawk USB, and type `make px4fmu-v2_default upload` to flash the
hardware with these brand new bits, so you don't need to use QGroundControl for that.

## Some Useful Parameters

PX4 has many customizable parameters (over 700 of them, in fact) and to get best results with AirSim we have
found the following parameters are handy:

```
// be sure to enable the new position estimator module:
param set SYS_MC_EST_GROUP 2

// increase default limits on cruise speed so you can move around a large map more quickly.
param MPC_XY_CRUISE 10             
param MPC_XY_VEL_MAX 10
param MPC_Z_VEL_MAX_DN 2

// increase timeout for auto-disarm on landing so that any long running app doesn't have to worry about it
param COM_DISARM_LAND 60

// make it possible to fly without radio control attached (do NOT do this one on a real drone)
param NAV_RCL_ACT 0

// enable new syslogger to get more information from PX4 logs
param set SYS_LOGGER 1
```

## Using BashOnWindows

See [Bash on Windows Toolchain](https://dev.px4.io/en/setup/dev_env_windows_bash_on_win.html).