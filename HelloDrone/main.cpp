// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <iostream>
#include "control/RpcLibClient.hpp"
#include "control/DroneControlBase.hpp"
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

    client.setImageTypeForCamera(4, DroneControlBase::ImageType::Depth);

    //cout << (int) client.getImageTypeForCamera(4) <<endl;
    //cout << (int) client.getImageTypeForCamera(3) <<endl;
    //client.setImageTypeForCamera(3, DroneControlBase::ImageType::Segmentation);
    //cout << (int) client.getImageTypeForCamera(3) <<endl;

    auto i1 = client.getImageForCamera(0, DroneControlBase::ImageType::Depth);
    cout << i1.size() << endl;

    auto i2 = client.getImageForCamera(3, DroneControlBase::ImageType::Depth);
    cout << i2.size() << " " << (i2.size() > 0 ? i2[0] : -1) << endl;

    auto i3 = client.getImageForCamera(4, DroneControlBase::ImageType::Scene);
    cout << i3.size() << " " << (i3.size() > 0 ? i3[0] : -1) << endl;
    return 0;
}

int main() 
{
    using namespace std;
    using namespace msr::airlib;

    msr::airlib::RpcLibClient client;

    cout << "Press Enter to enable retrival of depth images" << endl; cin.get();
    client.setImageTypeForCamera(0, DroneControlBase::ImageType::Segmentation);

    cout << "Press Enter to get depth image" << endl; cin.get();
    auto image = client.getImageForCamera(0, DroneControlBase::ImageType::Segmentation);
    cout << "PNG images recieved bytes: " << image.size() << endl;
    cout << "Press Enter to save image" << endl; cin.get();
    ofstream file("c:\\temp\\depth.png", ios::binary);
    file.write((char*) image.data(), image.size());
    file.close();

    cout << "Press Enter to arm the drone" << endl; cin.get();
    client.armDisarm(true);

    cout << "Press Enter to takeoff" << endl; cin.get();
    client.takeoff(1000);

    cout << "Press Enter to use offboard control" << endl; cin.get();
    client.requestControl();

    cout << "Press Enter to move 5 meters with 1 m/s velocity" << endl; cin.get();
    client.moveToPosition(5, 0, 2.5f, 1);

    cout << "Press Enter to land" << endl; cin.get();
    client.land();

    return 0;
}
