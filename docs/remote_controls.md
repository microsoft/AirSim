# Remote Controllers

Whether you are flying Pixhawk in [HIL mode](px4.md) or using the [PX4 SITL mode](sitl.md) on Ubuntu, you will not be able to manually fly
the drone unless you have some sort of remote controller, either a real one, or a joystick that can be configured to
behave like one.

There are many remote control options that you can use with quadrotors. 
See [PX4 RC configuration](https://docs.px4.io/en/getting_started/rc_transmitter_receiver.html) for infomration on setting up
remote control with PX4.

We have used [FrSky Taranis X9D Plus](https://hobbyking.com/en_us/frsky-2-4ghz-accst-taranis-x9d-plus-and-x8r-combo-digital-telemetry-radio-system-mode-2.html) 
and Futaba 14SG with AirSim.

## Using Joystick/Gamepad (Alternative to RC)

You can also use an xbox controller, it just won't be as precise as a real RC controller.
See [xbox controller](xbox_controller.md) for details on how to set that up.


## Playstation 3 controller

A Playstation 3 controller is confirmed to work as an AirSim controller. On Windows, an emulator to make it look like an Xbox 360 controller, is required however. Many different solutions are available online, for example [x360ce Xbox 360 Controller Emulator](https://github.com/x360ce/x360ce).  `Note:` don't use x360ce if you have a real XBox controller.

## DJI Controller

Nils Tijtgat wrote an excellent blog on how to get the [DJI controller working with AirSim](https://timebutt.github.io/static/using-a-phantom-dji-controller-in-airsim/).
