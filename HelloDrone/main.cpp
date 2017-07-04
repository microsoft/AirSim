// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <iostream>
#include <chrono>
#include "rpc/RpcLibClient.hpp"
#include "controllers/DroneControllerBase.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON


int main() 
{
    using namespace msr::airlib;


    // This assumes you are running DroneServer already on the same machine.
    // DroneServer must be running first.
    msr::airlib::RpcLibClient client;

    try {
        client.confirmConnection();

        std::cout << "Press Enter to get FPV image" << std::endl; std::cin.get();
        auto image = client.getImageForCamera(0, VehicleCamera::ImageType::Scene);
        std::cout << "PNG images recieved bytes: " << image.size() << std::endl;
        std::cout << "Enter file name to save image (leave empty for no save)" << std::endl; 
        std::string filename;
        std::getline(std::cin, filename);

        if (filename != "") {
            std::ofstream file(filename, std::ios::binary);
            file.write((char*) image.data(), image.size());
            file.close();
        }

        std::cout << "Press Enter to arm the drone" << std::endl; std::cin.get();
        client.armDisarm(true);

        std::cout << "Press Enter to takeoff" << std::endl; std::cin.get();
        float takeoffTimeout = 5; 
        client.takeoff(takeoffTimeout);

        // switch to explicit hover mode so that this is the fallback when 
        // move* commands are finished.
        std::this_thread::sleep_for(std::chrono::duration<double>(5));
        client.hover();

        std::cout << "Press Enter to fly in a 10m box pattern at 1 m/s velocity" << std::endl; std::cin.get();
        // moveByVelocityZ is an offboard operation, so we need to set offboard mode.
        client.setOffboardMode(true); 
        auto position = client.getPosition();
        float z = position.z(); // current position (NED coordinate system).  
        const float speed = 1.0f;
        const float size = 10.0f; 
        const float duration = size / speed;
        DrivetrainType driveTrain = DrivetrainType::ForwardOnly;
        YawMode yaw(false, 0);
        std::cout << "moveByVelocityZ(" << speed << ", 0, " << z << "," << duration << ")" << std::endl;
        client.moveByVelocityZ(speed, 0, z, duration, driveTrain, yaw);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));
        std::cout << "moveByVelocityZ(0, " << speed << "," << z << "," << duration << ")" << std::endl;
        client.moveByVelocityZ(0, speed, z, duration, driveTrain, yaw);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));
        std::cout << "moveByVelocityZ(" << -speed << ", 0, " << z << "," << duration << ")" << std::endl;
        client.moveByVelocityZ(-speed, 0, z, duration, driveTrain, yaw);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));
        std::cout << "moveByVelocityZ(0, " << -speed << "," << z << "," << duration << ")" << std::endl;
        client.moveByVelocityZ(0, -speed, z, duration, driveTrain, yaw);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));

        client.hover();

        std::cout << "Press Enter to land" << std::endl; std::cin.get();
        client.land();

        std::cout << "Press Enter to disarm" << std::endl; std::cin.get();
        client.armDisarm(false);

    }
    catch (rpc::rpc_error&  e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
    }

    return 0;
}


int imageExample()
{
    using namespace std;
    using namespace msr::airlib;

    msr::airlib::RpcLibClient client;

    auto i1 = client.getImageForCamera(0, VehicleCamera::ImageType::Depth);
    std::cout << i1.size() << std::endl;

    auto i2 = client.getImageForCamera(3, VehicleCamera::ImageType::Depth);
    std::cout << i2.size() << " " << (i2.size() > 0 ? i2[0] : -1) << std::endl;

    auto i3 = client.getImageForCamera(4, VehicleCamera::ImageType::Scene);
    std::cout << i3.size() << " " << (i3.size() > 0 ? i3[0] : -1) << std::endl;


    return 0;
}