// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <iostream>
#include <string>
#include "control/RpcLibServer.hpp"
#include "control/MavLinkDroneControl.hpp"

using namespace std;
using namespace msr::airlib;

std::string server_address("127.0.0.1");

bool parseCommandLine(int argc, const char* argv[]) {
    // parse command line
    for (int i = 1; i < argc; i++) {
        const char* arg = argv[i];
        if (arg[0] == '-' || arg[0] == '/') {
            std::string name = arg + 1;
            if (name == "ipaddress" && i + 1 < argc) {
                server_address = argv[++i];
            } else {
                return false;
            }
        } else {
            return false;
        }
    }
    return true;
}

void printUsage() {
    cout << "Usage: DroneServer [-ipaddress 127.0.0.1]" << endl;
    cout << "The optional ipaddress argument specifies the RPC local IP address to use." << endl;
}

int main(int argc, const char* argv[]) {
    if (!parseCommandLine(argc, argv)) {
        printUsage();
        return 1;
    }

    MavLinkDroneControl::Parameters params;
    MavLinkDroneControl mav(params);
    DroneControlServer server_wrapper(&mav);
    msr::airlib::RpcLibServer server(&server_wrapper, server_address);

    auto v = std::vector<msr::airlib::uint8_t> { 5, 4, 3 };
    server_wrapper.setImageForCamera(3, DroneControlBase::ImageType::Depth, v);
    server_wrapper.setImageForCamera(4, DroneControlBase::ImageType::Scene, std::vector<msr::airlib::uint8_t> {6, 5, 4, 3, 2});

    std::cout << "Server started at " << server_address << ":" << params.udpPort << std::endl;
    server.start(true);
    return 0;
}
