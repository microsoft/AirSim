# Prerequisites

## What hardware do you need?
### Computer
It depends on how big your Unreal Envionment is. The Blocks environment that comes with AirSim is very basic and works on typical laptops. The [Modular Neighborhood Pack](https://www.unrealengine.com/marketplace/modular-neighborhood-pack) that we use ourselves for research requires GPUs with at least 4GB of RAM. The [Open World environment](https://www.unrealengine.com/marketplace/open-world-demo-collection) needs GPU with 8GB RAM. Our typical development machines have 32GB of RAM and NVidia TitanX and a [fast hard drive](hard_drive.md).

### Using AirSim Without PX4
Yes, now its possible to use AirSim without PX4. Please see the [instructions here](image_apis.md) for how to use so-called "Computer Vision" mode. If you don't need vehicle physics, we highly recommand to use this mode because setting up PX4 had been unreliable (and you can skip rest of the instructions on this page!). Due to bugs [such as this](https://github.com/PX4/Firmware/issues/7516) things haven't been working after PX4 release 1.5.5. With that release, you may still see wobbly drone sometimes. We are in process to integrate [ROSFlight](https://github.com/rosflight/firmware) to replace PX4 as our default flight controller. We are also making more head ways to develop our own [simple_flight](https://github.com/Microsoft/AirSim/tree/master/AirLib/include/controllers/simple_flight) which would be useful for folks working on re-inforcement learning.

### RC Transmitter and Receiver

This is the remote control that you usually use for real RC vehicles such as quadrotors. Our favorite is [FrSky Taranis X9D Plus](https://hobbyking.com/en_us/frsky-2-4ghz-accst-taranis-x9d-plus-and-x8r-combo-digital-telemetry-radio-system-mode-2.html) because it has built-in USB ports. 
We have also tested a Futaba 14SG, so if you have one of those already you should be fine.
You can also fly without a remote control.  See [flying SITL mode software in the loop flight controller](sitl.md).
See also [more information about remote control options](remote_controls.md) 

### Flight controller

Choose from several supported [PX4 flight controllers](px4.md).  Our code is tested with the [PX4 firmware](https://dev.px4.io/).  
We have not tested Arducopter or other mavlink implementations.  Some of the flight API's do use the
PX4 custom modes in the MAV_CMD_DO_SET_MODE messages (like PX4_CUSTOM_MAIN_MODE_AUTO)

You can also fly without flight controller hardware.  See [alternative to RC](sitl.md).

## Setup steps
1. Make sure your RC receiver is bound with its RC transmitter. Connect the RC trasmitter to the flight controller's RC port.
2. Download [QGroundControl](http://qgroundcontrol.com/), launch it and connect your flight controller to the USB port.
3. Install the PX4 firmware v1.5.5 from [github](https://github.com/PX4/Firmware/releases/tag/v1.5.5). You likely have v2 device and if so use file `nuttx-px4fmu-v2-default.px4`. To check if you really have v2 device you can use [Nirsoft USBDeview](http://www.nirsoft.net/utils/usb_devices_view.html) utility. Chris Lovett also has created [initial firmware setup video](https://dev.px4.io/starting-initial-config.html) that show how to install new firmware. Note that later releases of PX4 at least up to 1.6.3 has [issue](https://github.com/PX4/Firmware/issues/7516) where vehicle doesn't takeoff after arming.
4. In QGroundControl, configure your Pixhawk for HIL simulation by selecting the HIL Quadrocopter X airframe.  After PX4 reboots, check that "HIL Quadrocopter X" is indeed selected. You might also want to use QGroundControl to calibrate your RC and set up Flight Mode switch among other things (see below) so everything is green.

See [Walkthrough Demo Video](https://youtu.be/HNWdYrtw3f0) and  [Unreal AirSim Setup  Video](https://youtu.be/1oY8Qu5maQQ) that shows you all the setup steps in this document.

## Using the Remote Control
When using hardware-in-loop mode, you can use remote controls such as Fly Sky, Spektrum, Futaba etc.
1. Connect the receiver for your remote control to the Pixhawk board. Make sure your remote control is bound to that receiver.
2. In QGroundControl, go to Radio tab and calibrate (make sure, remote control is on and the receiver is showing the indicator for the binding). 
3. Go to the Flight Mode tab and chose one of the remote control switches as "Mode Channel". Then set (for example) Stabilized and Attitude flight modes for two positions of the switch.
4. Go to the Tuning section of QGroundControl and set appropriate values. For example, for Fly Sky's FS-TH9X remote control, following settings gives more realistic feel: Hover Throttle = mid+1 mark, Roll and pitch sensitivity = mid-3 mark, Altitude and position control sensitivity = mid-2 mark.
5. You can usually arm the vehicle by lowering and bringing two sticks of RC together (you don't need QGroundControl after the initial setup). Now you can use RC to fly in the simulator. Typically the Altitude mode gives better experience for beginners because this mode lets the autopilot automatically maintain an altitude when the throttle stick is approximately in the middle.
