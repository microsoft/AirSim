// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "MavLinkConnection.hpp"
#include "Utils.hpp"

using namespace mavlinkcom;
using namespace mavlinkcom_impl;

std::vector<SerialPortInfo> MavLinkConnection::findSerialPorts(int vid, int pid)
{
    unused(vid); // avoid warning: unused parameter
    unused(pid); // avoid warning: unused parameter
    // todo: not implemented on Linux yet...  probably need to do an lstat on '/dev/serial/by-id' and find
    // something that looks like PX4 and return that name, or follow the symbolic link to /dev/ttyACM0...
    std::vector<SerialPortInfo> result;
    return result;
}
