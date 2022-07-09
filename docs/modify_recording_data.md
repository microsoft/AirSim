# Modifying Recording Data

AirSim has a [Recording feature](settings.md#recording) to easily collect data and images. The [Recording APIs](apis.md#recording-apis) also allows starting and stopping the recording using API.

However, the data recorded by default might not be sufficient for your use cases, and it might be preferable to record additional data such as IMU, GPS sensors, Rotor speed for copters, etc. You can use the existing Python and C++ APIs to get the information and store it as required, especially for Lidar. Another option for adding small fields such as GPS or internal data such as Unreal position or something else is possible through modifying the recording methods inside AirSim. This page describes the specific methods which you might need to change.

The recorded data is written in a `airsim_rec.txt` file in a tab-separated format, with images in an `images/` folder. The entire folder is by default present in the `Documents` folder (or specified in settings) with the timestamp of when the recording started in `%Y-%M-%D-%H-%M-%S` format.

Car vehicle records the following fields -

```text
VehicleName TimeStamp   POS_X   POS_Y   POS_Z   Q_W Q_X Q_Y Q_Z Throttle    Steering    Brake   Gear    Handbrake   RPM Speed   ImageFile
```

For Multirotor -

```text
VehicleName TimeStamp   POS_X   POS_Y   POS_Z   Q_W Q_X Q_Y Q_Z ImageFile
```

## Code Changes

Note that this requires building and using AirSim from source. You can compile a binary yourself after modifying if needed.

The primary method which fills the data to be stored is [`PawnSimApi::getRecordFileLine`](https://github.com/microsoft/AirSim/blob/880c5541fd4824ee2cd9bb82ca5f611eb1ab236a/Unreal/Plugins/AirSim/Source/PawnSimApi.cpp#L544), it's the base method for all the vehicles, and Car overrides it to log additional data, as can be seen in [`CarPawnSimApi::getRecordFileLine`](https://github.com/microsoft/AirSim/blob/880c5541fd4824ee2cd9bb82ca5f611eb1ab236a/Unreal/Plugins/AirSim/Source/Vehicles/Car/CarPawnSimApi.cpp#L34).

To record additional data for multirotor, you can add a similar method in [MultirotorPawnSimApi.cpp/h](https://github.com/microsoft/AirSim/tree/main/Unreal/Plugins/AirSim/Source/Vehicles/Multirotor) files which overrides the base class implementation and append other data. The currently logged data can also be modified and removed as needed.

E.g. recording GPS, IMU and Barometer data also for multirotor -

```cpp
// MultirotorPawnSimApi.cpp
std::string MultirotorPawnSimApi::getRecordFileLine(bool is_header_line) const
{
    std::string common_line = PawnSimApi::getRecordFileLine(is_header_line);
    if (is_header_line) {
        return common_line +
               "Latitude\tLongitude\tAltitude\tPressure\tAccX\tAccY\tAccZ\t";
    }

    const auto& state = vehicle_api_->getMultirotorState();
    const auto& bar_data = vehicle_api_->getBarometerData("");
    const auto& imu_data = vehicle_api_->getImuData("");

    std::ostringstream ss;
    ss << common_line;
    ss << state.gps_location.latitude << "\t" << state.gps_location.longitude << "\t"
       << state.gps_location.altitude << "\t";

    ss << bar_data.pressure << "\t";

    ss << imu_data.linear_acceleration.x() << "\t" << imu_data.linear_acceleration.y() << "\t"
       << imu_data.linear_acceleration.z() << "\t";

    return ss.str();
}
```

```cpp
// MultirotorPawnSimApi.h
virtual std::string getRecordFileLine(bool is_header_line) const override;
```
