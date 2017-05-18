# XBox Controller

To use an XBox Controller with AirSim follow these steps:

1. Connect XBox controller so it shows up in your PC Game Controllers:

![Gamecontrollers](images/game_controllers.png)

2. Launch PX4 SITL in a terminal, it will wait for a UDP connection on a certain port.

3. Launch Unreal Editor and open the project which has the AirSim plugin and an environemnt of your preference.

 __Note:__ Ensure that you have the gameMode as AirSimGameMode in the World Seetings
 
4. Press play so that the quad spawns into the scene at the player start actor. Once this is done, a connection would be made to px4 SITL which will unblock the autopilot and start executing. The log would appear similar to this:

```
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting.

INFO  [dataman] Unkown restart, data manager file 'rootfs/fs/microsd/dataman' size is 11797680 bytes
INFO  [platforms__posix__drivers__ledsim] LED::init
INFO  [platforms__posix__drivers__ledsim] LED::init
INFO  [simulator] Waiting for initial data on UDP port 14560. Please start the flight simulator to proceed..
INFO  [simulator] Got initial simulation data, running sim..
INFO  [pwm_out_sim] MODE_8PWM
INFO  [mavlink] mode: Normal, data rate: 4000000 B/s on udp port 14556 remote port 14550
INFO  [mavlink] mode: Onboard, data rate: 4000000 B/s on udp port 14557 remote port 14540
INFO  [mavlink] MAVLink only on localhost (set param MAV_BROADCAST = 1 to enable network)
INFO  [logger] logger started (mode=all)
pxh> INFO  [logger] Start file log
INFO  [logger] Opened log file: rootfs/fs/microsd/log/2017-05-16/02_57_46.ulg
INFO  [tone_alarm] startup
INFO  [mavlink] partner IP: 127.0.0.1
INFO  [lib__ecl] EKF aligned, (pressure height, IMU buf: 17, OBS buf: 16)
INFO  [lib__ecl] EKF GPS checks passed (WGS-84 origin set)
INFO  [lib__ecl] EKF commencing GPS fusion
INFO  [commander] home: 47.6414680, -122.1401669, 102.07
INFO  [tone_alarm] home_set
```

__Note:__ It is imporant to check the ~/Documents/AirSim/settings.json file and should be similar to the one shown [here](https://github.com/Microsoft/AirSim/blob/master/docs/settings.md).


5. Launch QGroundControl and follow the following steps
i. Click the white/purple Q in the top right to open the settings.
ii. Add a UDP comm link on port 14540 with the localhost ip (127.0.0.1)
iii. Connect using that link. You should see at the top the sattelite count become valid and the battery voltage too. If you don't see this, then stop now because the Mavlink connection is invalid.
iv. Click the gears icon next to the Q icon and you should get a bunch of menus including the joystick configuration menu as shown below.
v. Under the joystick mode, click calibrate and follow the instructions on the screen.

![Gamecontrollers](images/qgc_joystick.png)

Now calibrate the radio, and setup some handy button actions.  For example, I set mine so that 
the 'A' button arms the drone, 'B' put it in manual flight mode, 'X' puts it in altitude hold mode
and 'Y' puts it in position hold mode.  I also prefer the feel of the controller when I check the
box labelled "Use exponential curve on roll,pitch, yaw" because this gives me more sensitivity for
small movements.]

QGroundControl will find your Pixhawk via the UDP proxy port 14550 setup by MavLinkTest above.
AirSim will find your Pixhawk via the other UDP server port 14570 also setup by MavLinkTest above.
You can also use all the QGroundControl controls for autonomous flying at this point too.

## Advanced

If the Joystick tab doesn't show up in QGroundControl then Click on the purple "Q" icon on left in tool bar to reveal the Preferences panel.
Go to General tab and check the Virtual Joystick checkbox.  Go back to settings screen (gears icon), click on Parameters tab,
type `COM_RC_IN_MODE` in search box and change its value to either `Joystick/No RC Checks` or `Virtual RC by Joystick`.

## Playstation 3 controller

A Playstation 3 controller is confirmed to work as an AirSim controller. On Windows, an emulator to make it look like an Xbox 360 controller, is required however. Many different solutions are available online, for example [x360ce Xbox 360 Controller Emulator](https://github.com/x360ce/x360ce).  `Note:` don't use x360ce if you have a real XBox controller.
