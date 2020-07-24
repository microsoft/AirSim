# Remote Control

To fly manually, you need remote control or RC. If you don't have one then you can use [APIs](apis.md) to fly programmatically or use so-called [Computer Vision mode](image_apis.md) to move around using keyboard.

## RC Setup for Default Config

By default AirSim uses [simple_flight](simple_flight.md) as its flight controller which connects to RC via USB port to your computer.

You can either use XBox controller or [FrSky Taranis X9D Plus](https://hobbyking.com/en_us/frsky-2-4ghz-accst-taranis-x9d-plus-and-x8r-combo-digital-telemetry-radio-system-mode-2.html). Note that XBox 360 controller is not precise enough and is not recommended if you wanted more real world experience. See FAQ below if things are not working.

### Other Devices

AirSim can detect large variety of devices however devices other than above *might* need extra configuration. In future we will add ability to set this config through settings.json. For now, if things are not working then you might want to try workarounds such as [x360ce](http://www.x360ce.com/) or change code in [SimJoystick.cpp file](/Unreal/Plugins/AirSim/Source/SimJoyStick/SimJoyStick.cpp#L50).

### Note on FrSky Taranis X9D Plus

[FrSky Taranis X9D Plus](https://hobbyking.com/en_us/frsky-2-4ghz-accst-taranis-x9d-plus-and-x8r-combo-digital-telemetry-radio-system-mode-2.html) is real UAV remote control with an advantage that it has USB port so it can be directly connected to PC. You can [download AirSim config file](misc/AirSim_FrSkyTaranis.bin) and [follow this tutorial](https://www.youtube.com/watch?v=qe-13Gyb0sw) to import it in your RC. You should then see "sim" model in RC with all channels configured properly.

### Note on Linux
Currently default config on Linux is for using Xbox controller. This means other devices might not work properly. In future we will add ability to configure RC in settings.json but for now you *might* have to change  code in [SimJoystick.cpp file](/Unreal/Plugins/AirSim/Source/SimJoyStick/SimJoyStick.cpp#L340) to use other devices.

## RC Setup for PX4

AirSim supports PX4 flight controller however it requires different setup. There are many remote control options that you can use with quadrotors. We have successfully used FrSky Taranis X9D Plus, FlySky FS-TH9X and Futaba 14SG with AirSim. Following are the high level steps to configure your RC:

1. If you are going to use Hardware-in-Loop mode, you need transmitter for your specific brand of RC and bind it. You can find this information in RC's user guide. 
2. For Hardware-in-Loop mode, you connect transmitter to Pixhawk. Usually you can find online doc or YouTube video tutorial on how to do that.
3. [Calibrate your RC in QGroundControl](https://docs.qgroundcontrol.com/en/SetupView/Radio.html).

See [PX4 RC configuration](https://docs.px4.io/en/getting_started/rc_transmitter_receiver.html) and Please see [this guide](http://ardupilot.org/copter/docs/common-pixhawk-and-px4-compatible-rc-transmitter-and-receiver-systems.html) for more information. 

### Using XBox 360 USB Gamepad

You can also use an xbox controller in SITL mode, it just won't be as precise as a real RC controller.
See [xbox controller](xbox_controller.md) for details on how to set that up.

### Using Playstation 3 controller

A Playstation 3 controller is confirmed to work as an AirSim controller. On Windows, an emulator to make it look like an Xbox 360 controller, is required however. Many different solutions are available online, for example [x360ce Xbox 360 Controller Emulator](https://github.com/x360ce/x360ce).

### DJI Controller

Nils Tijtgat wrote an excellent blog on how to get the [DJI controller working with AirSim](https://timebutt.github.io/static/using-a-phantom-dji-controller-in-airsim/).

## FAQ

#### I'm using default config and AirSim says my RC is not detected on USB.

This typically happens if you have multiple RCs and or XBox/Playstation gamepads etc connected. In Windows, hit Windows+S key and search for "Set up USB Game controllers" (in older versions of Windows try "joystick"). This will show you all game controllers connected to your PC. If you don't see yours than Windows haven't detected it and so you need to first solve that issue. If you do see yours but not at the top of the list (i.e. index 0) than you need to tell AirSim because AirSim by default tries to use RC at index 0. To do this, navigate to your `~/Documents/AirSim` folder, open up `settings.json` and add/modify following setting. Below tells AirSim to use RC at index = 2.
```
{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "Vehicles": {
        "SimpleFlight": {
            "VehicleType": "SimpleFlight",
            "RC": {
              "RemoteControlID": 2
            }
        }
    }
}
```

#### Vehicle seems unstable when using XBox/PS3 controller

Regular gamepads are not very precise and have lot of random noise. Most of the times you may see significant offsets as well (i.e. output is not zero when sticks are at zero). So this behavior is expected.

#### Where is RC calibration in AirSim?

We haven't implemented it yet. This means your RC firmware will need to have a capability to do calibration for now.

#### My RC is not working with PX4 setup.

First you want to make sure your RC is working in [QGroundControl](https://docs.qgroundcontrol.com/en/SetupView/Radio.html). If it doesn't then it will sure not work in AirSim. The PX4 mode is suitable for folks who have at least intermediate level of experience to deal with various issues related to PX4 and we would generally refer you to get help from PX4 forums.
