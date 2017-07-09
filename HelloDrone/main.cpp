// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <iostream>
#include <chrono>
#include "rpc/RpcLibClient.hpp"
#include "controllers/DroneControllerBase.hpp"
#include "common/common_utils/FileSystem.hpp"
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
    typedef DroneControllerBase::ImageRequest ImageRequest;
    typedef VehicleCameraBase::ImageResponse ImageResponse;
    typedef VehicleCameraBase::ImageType_ ImageType_;
    typedef common_utils::FileSystem FileSystem;
    
    try {
        client.confirmConnection();

        std::cout << "Press Enter to get FPV image" << std::endl; std::cin.get();
        vector<ImageRequest> request = { ImageRequest(0, ImageType_::Scene), ImageRequest(1, ImageType_::Depth) };
        const vector<ImageResponse>& response = client.simGetImages(request);
        std::cout << "# of images recieved: " << response.size() << std::endl;

        if (response.size() > 0) {
            std::cout << "Enter path with ending separator to save images (leave empty for no save)" << std::endl; 
            std::string path;
            std::getline(std::cin, path);

            for (const ImageResponse& image_info : response) {
                std::cout << "Image size: " << image_info.image_data.size() << std::endl;
                if (path != "") {
                    std::ofstream file(FileSystem::combine(path, std::to_string(image_info.time_stamp) + ".png"), std::ios::binary);
                    file.write((char*) image_info.image_data.data(), image_info.image_data.size());
                    file.close();
                }
            }
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
