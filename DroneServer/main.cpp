// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <iostream>
#include "control/RpcLibServer.hpp"
#include "control/MavLinkDroneControl.hpp"


int main() 
{
    using namespace msr::airlib;

    MavLinkDroneControl::Parameters params;
    MavLinkDroneControl mav(params);
    DroneControlServer server_wrapper(&mav);
    msr::airlib::RpcLibServer server(&server_wrapper);
    
    auto v = vector<msr::airlib::uint8_t>{ 5, 4, 3 };
    server_wrapper.setImageForCamera(3, DroneControlBase::ImageType::Depth, v);
    server_wrapper.setImageForCamera(4, DroneControlBase::ImageType::Scene, vector<msr::airlib::uint8_t>{6, 5, 4, 3, 2});
    
    std::cout << "Server started" << std::endl;
    server.start(true);
    return 0;
}
