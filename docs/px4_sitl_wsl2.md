# PX4 Software-in-Loop with WSL 2

The [Windows subsystem for Linux version
2](https://docs.microsoft.com/en-us/windows/wsl/install-win10) uses a Virtual Machine which has a
separate IP address from your Windows host machine. This means PX4 cannot find AirSim using
"localhost" which is the default behavior for PX4.

You will notice that on Windows `ipconfig` returns a new ethernet adapter for WSL like this (notice
the vEthernet has `(WSL)` in the name:

```plain
Ethernet adapter vEthernet (WSL):

   Connection-specific DNS Suffix  . :
   Link-local IPv6 Address . . . . . : fe80::1192:f9a5:df88:53ba%44
   IPv4 Address. . . . . . . . . . . : 172.31.64.1
   Subnet Mask . . . . . . . . . . . : 255.255.240.0
   Default Gateway . . . . . . . . . :
```

This address `172.31.64.1` is the address that WSL 2 can use to reach your Windows host machine.

Starting with this [PX4 Change
Request](https://github.com/PX4/PX4-Autopilot/commit/1719ff9892f3c3d034f2b44e94d15527ab09cec6)
(which correlates to version v1.12.0-beta1 or newer) PX4 in SITL mode can now connect to AirSim on a
different (remote) IP address.  To enable this make sure you have a version of PX4 containing this
fix and set the following environment variable in linux:

```shell
export PX4_SIM_HOST_ADDR=172.31.64.1
```

**Note:** Be sure to update the above address `172.31.64.1` to match what you see from your
`ipconfig` command.

Open incoming TCP port 4560 and incoming UDP port 14540 using your firewall configuration.

Now on the linux side run `ip address show` and copy the `eth0 inet` address, it should be something
like `172.31.66.156`.  This is the address Windows needs to know in order to find PX4.

Edit your [AirSim settings](settings.md) file and add `LocalHostIp` to tell AirSim to use the WSL
ethernet adapter address instead of the default `localhost`.  This will cause AirSim to open the TCP
port on that adapter which is the address that the PX4 app will be looking for.  Also tell AirSim
to connect the `ControlIp` UDP channel by setting `ControlIp` to the magic string `remote`.
This resolves to the WSL 2 remote ip address found in the TCP socket.

```json
{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "ClockType": "SteppableClock",
    "Vehicles": {
        "PX4": {
            "VehicleType": "PX4Multirotor",
            "UseSerial": false,
            "LockStep": true,
            "UseTcp": true,
            "TcpPort": 4560,
            "ControlIp": "remote",
            "ControlPortLocal": 14540,
            "ControlPortRemote": 14580,
            "LocalHostIp": "172.31.64.1",
            "Sensors":{
                "Barometer":{
                    "SensorType": 1,
                    "Enabled": true,
                    "PressureFactorSigma": 0.0001825
                }
            },
            "Parameters": {
                "NAV_RCL_ACT": 0,
                "NAV_DLL_ACT": 0,
                "COM_OBL_ACT": 1,
                "LPE_LAT": 47.641468,
                "LPE_LON": -122.140165
            }
        }
    }
}
```
See [PX4 LockStep](px4_lockstep.md) for more information.
The "Barometer" setting keeps PX4 happy because the default AirSim barometer has a bit too much
noise generation.  This setting clamps that down a bit.

If your local repo does not include [this PX4 
commit](https://github.com/PX4/PX4-Autopilot/commit/292a66ce417c9769e1a7845fbc9b8d5e68e1cf0b), 
please edit the Linux file in `ROMFS/px4fmu_common/init.d-posix/rcS` and make sure it is looking
for the `PX4_SIM_HOST_ADDR` environment variable and is passing that through to the PX4 
simulator like this:

```shell
# If PX4_SIM_HOST_ADDR environment variable is empty use localhost.
if [ -z "${PX4_SIM_HOST_ADDR}" ]; then
    echo "PX4 SIM HOST: localhost"
    simulator start -c $simulator_tcp_port
else
    echo "PX4 SIM HOST: $PX4_SIM_HOST_ADDR"
    simulator start -t $PX4_SIM_HOST_ADDR $simulator_tcp_port
fi
```

**Note:** this code might already be there depending on the version of PX4 you are using.

**Note:** please be patient when waiting for the message:

```
INFO  [simulator] Simulator connected on TCP port 4560.
```

It can take a little longer to establish the remote connection than it does with `localhost`.

Now you can proceed with the steps shown in [Setting up PX4 Software-in-Loop](px4_sitl.md).
