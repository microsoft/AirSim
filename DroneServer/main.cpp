// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <iostream>
#include <string>
#include "rpc/RpcLibServer.hpp"
#include "controllers/MavLinkDroneController.hpp"

using namespace std;
using namespace msr::airlib;

std::string server_address("127.0.0.1");

bool parseCommandLine(int argc, const char* argv[])
{
    // parse command line
    for (int i = 1; i < argc; i++)
    {
        const char* arg = argv[i];
        if (arg[0] == '-' || arg[0] == '/')
        {
            std::string name = arg + 1;
            if (name == "ipaddress" && i + 1 < argc)
            {
                server_address = argv[++i];
            }
            else {
                return false;
            }
        }
        else {
            return false;
        }
    }
    return true;
}

void printUsage() {
    cout << "Usage: DroneServer [-ipaddress 127.0.0.1]" << endl;
    cout << "The optional ipaddress argument specifies the RPC local IP address to use." << endl;
}

int main(int argc, const char* argv[])
{
    if (!parseCommandLine(argc, argv)) {
        printUsage();
        return 1;
    }

    MavLinkDroneController::ConnectionInfo connection_info;
    connection_info.use_serial = false;
    connection_info.ip_port = connection_info.qgc_ip_port;

    MavLinkDroneController mav_drone;
    mav_drone.initialize(connection_info, nullptr, true);   //TODO: need to review how is_simulation flag might affect here
    DroneControllerCancelable server_wrapper(&mav_drone);
    msr::airlib::RpcLibServer server(&server_wrapper, server_address);
    
    auto v = std::vector<msr::airlib::uint8_t>{ 5, 4, 3 };
    server_wrapper.setImageForCamera(3, DroneControllerBase::ImageType::Depth, v);
    server_wrapper.setImageForCamera(4, DroneControllerBase::ImageType::Scene, std::vector<msr::airlib::uint8_t>{6, 5, 4, 3, 2});
    
    std::cout << "Server connected to MavLink endpoint at " << server_address << ":" << connection_info.ip_port << std::endl;
    server.start(true);
    return 0;
}
