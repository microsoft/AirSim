# create PX4 settings.json
touch /home/$USER/Documents/AirSim/settings.json
> /home/$USER/Documents/AirSim/settings.json;
echo "{
    \"SettingsVersion\": 1.2,
    \"SimMode\": \"Multirotor\",
    \"Vehicles\": {
        \"PX4Vehicle\": {
            \"VehicleType\": \"PX4Multirotor\",
            \"UseSerial\": false,
            \"UseTcp\": true,
            \"TcpPort\": 4560,
            \"ControlPort\": 14580,
            \"params\": {
                \"NAV_RCL_ACT\": 0,
                \"NAV_DLL_ACT\": 0,
                \"LPE_LAT\": 47.641468,
                \"LPE_LON\": -122.140165,
                \"COM_OBL_ACT\": 1
            }
        }
    }
}" >> /home/$USER/Documents/AirSim/settings.json;

install PX4 prereqs
pip3 install --user empy toml;

# build PX4 from source # todo stable version tags?
mkdir PX4 && cd $_;
git clone https://github.com/PX4/Firmware.git;
cd Firmware;

# Run PX4 SITL # todo - track PID, run in background?
make px4_sitl_default none_iris;

# Run AirLib
./build_debug/output/bin/Test