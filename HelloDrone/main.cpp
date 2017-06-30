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
    using namespace std;
    using namespace msr::airlib;


    // This assumes you are running DroneServer already on the same machine.
    // DroneServer must be running first.
    msr::airlib::RpcLibClient client;

    try {
        cout << "Waiting for drone to get a GPS position..." << endl;
        GeoPoint pos = client.getGpsLocation();
        while (pos.altitude == 0 && pos.latitude == 0 && pos.longitude == 0)
        {
            std::this_thread::sleep_for(std::chrono::duration<double>(1)); 
            pos = client.getGpsLocation();
        }
        
        cout << "Great, we have a GPS position: lat=" << pos.latitude << ", lon=" << pos.longitude << ", alt=" << pos.altitude << endl;


        cout << "Press Enter to arm the drone" << endl; cin.get();
        client.armDisarm(true);

        cout << "Press Enter to takeoff" << endl; cin.get();
        float takeoffTimeout = 5; 
        client.takeoff(takeoffTimeout);

        // switch to explicit hover mode so that this is the fallback when 
        // move* commands are finished.
        std::this_thread::sleep_for(std::chrono::duration<double>(5));
        client.hover();


        cout << "Press Enter to fly in a 10m box pattern at 1 m/s velocity" << endl; cin.get();
        // moveByVelocityZ is an offboard operation, so we need to set offboard mode.
        client.setOffboardMode(true); 
        auto position = client.getPosition();
        float z = position.z(); // current position (NED coordinate system).  
        const float speed = 1.0f;
        const float size = 10.0f; 
        const float duration = size / speed;
        DrivetrainType driveTrain = DrivetrainType::ForwardOnly;
        YawMode yaw(false, 0);
        cout << "moveByVelocityZ(" << speed << ", 0, " << z << "," << duration << ")" << endl;
        client.moveByVelocityZ(speed, 0, z, duration, driveTrain, yaw);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));
        cout << "moveByVelocityZ(0, " << speed << "," << z << "," << duration << ")" << endl;
        client.moveByVelocityZ(0, speed, z, duration, driveTrain, yaw);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));
        cout << "moveByVelocityZ(" << -speed << ", 0, " << z << "," << duration << ")" << endl;
        client.moveByVelocityZ(-speed, 0, z, duration, driveTrain, yaw);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));
        cout << "moveByVelocityZ(0, " << -speed << "," << z << "," << duration << ")" << endl;
        client.moveByVelocityZ(0, -speed, z, duration, driveTrain, yaw);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));

        client.hover();

        cout << "Press Enter to land" << endl; cin.get();
        client.land();

        cout << "Press Enter to disarm" << endl; cin.get();
        client.armDisarm(false);

    }
    catch (rpc::rpc_error&  e) {
        std::string msg = e.get_error().as<std::string>();
        cout << "Exception raised by the API, something went wrong." << endl << msg << endl;
    }

    return 0;
}


int imageExample()
{
    using namespace std;
    using namespace msr::airlib;

    msr::airlib::RpcLibClient client;

    auto i1 = client.getImageForCamera(0, VehicleCamera::ImageType::Depth);
    cout << i1.size() << endl;

    auto i2 = client.getImageForCamera(3, VehicleCamera::ImageType::Depth);
    cout << i2.size() << " " << (i2.size() > 0 ? i2[0] : -1) << endl;

    auto i3 = client.getImageForCamera(4, VehicleCamera::ImageType::Scene);
    cout << i3.size() << " " << (i3.size() > 0 ? i3[0] : -1) << endl;


    return 0;
}