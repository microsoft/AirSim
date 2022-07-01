# LockStep

The latest version of PX4 supports a new [lockstep
feature](https://docs.px4.io/master/en/simulation/#lockstep-simulation) when communicating with the
simulator over TCP.  Lockstep is an important feature because it synchronizes PX4 and the simulator
so they essentially use the same clock time.  This makes PX4 behave normally even during unusually
long delays in Simulator performance.

It is recommended that when you are running a lockstep enabled version of PX4 in SITL mode that you
tell AirSim to use a `SteppableClock`, and set `UseTcp` to `true` and `LockStep` to `true`.

```
    {
        "SettingsVersion": 1.2,
        "SimMode": "Multirotor",
        "ClockType": "SteppableClock",
        "Vehicles": {
            "PX4": {
                "VehicleType": "PX4Multirotor",
                "UseTcp": true,
                "LockStep": true,
                ...
```

This causes AirSim to not use a "realtime" clock, but instead it advances the clock in step which
each sensor update sent to PX4.  This way PX4 thinks time is progressing smoothly no matter how long
it takes AirSim to really process that update loop.

This has the following advantages:

- AirSim can be used on slow machines that cannot process updates quickly.
- You can debug AirSim and hit a breakpoint, and when you resume PX4 will behave normally.
- You can enable very slow sensors like the Lidar with large number of simulated points, and PX4
  will still behave normally.

There will be some side effects to `lockstep`, namely, slower update loops caused by running AirSim
on an underpowered machine or from expensive sensors (like Lidar) will create some visible jerkiness
in the simulated flight if you look at the updates on screen in realtime.

# Disabling LockStep

If you are running PX4 in cygwin, there is an [open issue with 
lockstep](https://github.com/microsoft/AirSim/issues/3415). PX4 is configured to use lockstep by 
default. To disable this feature, first [disable it in 
PX4](https://docs.px4.io/master/en/simulation/#disable-lockstep-simulation):

1. Navigate to `boards/px4/sitl/` in your local PX4 repository
1. Edit `default.cmake` and find the following line:
    ```
    set(ENABLE_LOCKSTEP_SCHEDULER yes)
    ```
1. Change this line to:
    ```
    set(ENABLE_LOCKSTEP_SCHEDULER no)
    ```
1. Disable it in AirSim by setting `LockStep` to `false` and either removing any `"ClockType": 
"SteppableClock"` setting or resetting `ClockType` back to default:
    ```
        {
            ...
            "ClockType": "",
            "Vehicles": {
                "PX4": {
                    "VehicleType": "PX4Multirotor",
                    "LockStep": false,
                    ...
    ```
1. Now you can run PX4 SITL as you normally would (`make px4_sitl_default none_iris`) and it will use 
the host system time without waiting on AirSim.
