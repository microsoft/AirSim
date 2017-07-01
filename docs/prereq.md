# Prerequisites

## What hardware do you need?
### Computer
Intel Core i7 or equivalent PC with 32GB of RAM and a very fast GPU with at least 4GB of RAM. Large environments in Unreal require a lot of RAM and a beefy GPU card. 
The typical computer hardware we use for development purposes usually have 6 to 12 cores and graphics card such as NVidia TitanX or NVidia GTX 1080 with 32GB to 64GB RAM and a [fast hard drive](hard_drive.md).

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
3. Install the PX4 firmware v1.5.5 from [github](https://github.com/PX4/Firmware/releases/tag/v1.5.5). 
See this [initial firmware setup video](https://dev.px4.io/starting-initial-config.html) that shows up to install new firmware. Note that later releases of PX4 has [issue](https://github.com/PX4/Firmware/issues/7516) where vehicle doesn't takeoff after arming.
4. In QGroundControl, configure your Pixhawk for HIL simulation by selecting the HIL Quadrocopter X airframe.  After PX4 reboots, check that "HIL Quadrocopter X" is indeed selected. You might also want to use QGroundControl to calibrate your RC and set up Flight Mode switch among other things (see below) so everything is green.

See [Walkthrough Demo Video](https://youtu.be/HNWdYrtw3f0) and  [Unreal AirSim Setup  Video](https://youtu.be/1oY8Qu5maQQ) that shows you all the setup steps in this document.

## Using the Remote Control
When using hardware-in-loop mode, you can use remote controls such as Fly Sky, Spektrum, Futaba etc.
1. Connect the receiver for your remote control to the Pixhawk board. Make sure your remote control is bound to that receiver.
2. In QGroundControl, go to Radio tab and calibrate (make sure, remote control is on and the receiver is showing the indicator for the binding). 
3. Go to the Flight Mode tab and chose one of the remote control switches as "Mode Channel". Then set (for example) Stabilized and Attitude flight modes for two positions of the switch.
4. Go to the Tuning section of QGroundControl and set appropriate values. For example, for Fly Sky's FS-TH9X remote control, following settings gives more realistic feel: Hover Throttle = mid+1 mark, Roll and pitch sensitivity = mid-3 mark, Altitude and position control sensitivity = mid-2 mark.
5. You can usually arm the vehicle by lowering and bringing two sticks of RC together (you don't need QGroundControl after the initial setup). Now you can use RC to fly in the simulator. Typically the Altitude mode gives better experience for beginners because this mode lets the autopilot automatically maintain an altitude when the throttle stick is approximately in the middle.
