#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream>
#include <chrono>
#include "common/AirSimSettings.hpp"

// a minimal airsim settings parser, adapted from Unreal/Plugins/AirSim/SimHUD/SimHUD.h
class AirSimSettingsParser
{
public:
    typedef msr::airlib::AirSimSettings AirSimSettings;
    typedef msr::airlib::AirSimSettings::VehicleSetting VehicleSetting;

public:
    AirSimSettingsParser();
    ~AirSimSettingsParser() {};

    bool success();

private:
    std::string getSimMode();
    bool readSettingsTextFromFile(std::string settingsFilepath, std::string& settingsText);
    bool getSettingsText(std::string& settingsText);
    bool initializeSettings();

    bool success_;
    std::string settingsText_;
};