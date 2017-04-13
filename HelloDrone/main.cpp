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
        float takeoffTimeout = 10; 
        client.takeoff(takeoffTimeout);

        // switch to explicit hover mode so that this is the fallback when 
        // move* commands are finished.
        std::this_thread::sleep_for(std::chrono::duration<double>(5));
        client.hover();

        cout << "Press Enter to request offboard control" << endl; cin.get();
        client.setOffboardMode(true);

        cout << "Press Enter to fly in a 10m box pattern at 1 m/s velocity" << endl; cin.get();

        auto position = client.getPosition();
        float z = position.z() - 5.0f; // 5 meters above current position (NED coordinate system).  
        const float speed = 1.0f;
        const float size = 10.0f; 
        const float duration = size / speed;
        cout << "moveByVelocityZ(" << speed << ", 0, " << z << "," << duration << ")" << endl;
        client.moveByVelocityZ(speed, 0, z, duration);
        cout << "moveByVelocityZ(0, " << speed << "," << z << "," << duration << ")" << endl;
        client.moveByVelocityZ(0, speed, z, duration);
        cout << "moveByVelocityZ(" << -speed << ", 0, " << z << "," << duration << ")" << endl;
        client.moveByVelocityZ(-speed, 0, z, duration);
        cout << "moveByVelocityZ(0, " << -speed << "," << z << "," << duration << ")" << endl;
        client.moveByVelocityZ(0, -speed, z, duration);

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

    client.setImageTypeForCamera(4, DroneControllerBase::ImageType::Depth);

    //cout << (int) client.getImageTypeForCamera(4) <<endl;
    //cout << (int) client.getImageTypeForCamera(3) <<endl;
    //client.setImageTypeForCamera(3, DroneControllerBase::ImageType::Segmentation);
    //cout << (int) client.getImageTypeForCamera(3) <<endl;

    auto i1 = client.getImageForCamera(0, DroneControllerBase::ImageType::Depth);
    cout << i1.size() << endl;

    auto i2 = client.getImageForCamera(3, DroneControllerBase::ImageType::Depth);
    cout << i2.size() << " " << (i2.size() > 0 ? i2[0] : -1) << endl;

    auto i3 = client.getImageForCamera(4, DroneControllerBase::ImageType::Scene);
    cout << i3.size() << " " << (i3.size() > 0 ? i3[0] : -1) << endl;

    /*
    cout << "Press Enter to enable retrival of depth images" << endl; cin.get();
    client.setImageTypeForCamera(0, DroneControllerBase::ImageType::Segmentation);
    cout << "Press Enter to get depth image" << endl; cin.get();
    auto image = client.getImageForCamera(0, DroneControllerBase::ImageType::Segmentation);
    cout << "PNG images received bytes: " << image.size() << endl;
    cout << "Press Enter to save image" << endl; cin.get();
    ofstream file("c:\\temp\\depth.png", ios::binary);
    file.write((char*) image.data(), image.size());
    file.close();
    */

    return 0;
}