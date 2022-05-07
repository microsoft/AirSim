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
    using AirSimSettings = msr::airlib::AirSimSettings;

public:
    AirSimSettingsParser(const std::string& host_ip);

    bool success();

private:
    std::string getSimMode();
    bool getSettingsText(std::string& settings_text) const;
    bool initializeSettings();

    bool success_;
    std::string settings_text_;
    std::string host_ip_;
};
