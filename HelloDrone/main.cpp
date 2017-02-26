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

int main() 
{
    using namespace std;
    using namespace msr::airlib;


    // This assumes you are running DroneServer already on the same machine.
    // DroneServer must be running first.
    msr::airlib::RpcLibClient client;

    try {

        cout << "Press Enter to arm the drone" << endl; cin.get();
        client.armDisarm(true);

        cout << "Press Enter to takeoff" << endl; cin.get();
        float takeoffTimeout = 1000; // wait 10 seconds to reach takeoff height.
        // drone will take off to whatever it thinks is a safe minimum altitude
        // so as to not get too much propellor backwash but also not too high.
        // usually 2-5 meters or thereabouts.
        client.takeoff(takeoffTimeout);

        cout << "Press Enter to request offboard control" << endl; cin.get();
        client.setOffboardMode(true);

        cout << "Press Enter to fly a 5 meters box at 2 m/s velocity" << endl; cin.get();        
        auto position = client.getPosition();
        float z = -3.0; // 3 meters above ground (NED coordinate system).
        float speed = 2.0f;
        float size = 5.0f; 
        client.moveToPosition(position.x() + size, position.y(), z, speed);
        client.moveToPosition(position.x() + size, position.y() + size, z, speed);
        client.moveToPosition(position.x(), position.y() + size, z, speed);
        client.moveToPosition(position.x(), position.y(), z, speed);

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
