// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "MavLinkConnection.hpp"

#ifdef ONECORE 
#include <Windows.h>
#include <cfgmgr32.h>
#include <propkey.h>
#include <string>

using namespace mavlinkcom;
using namespace mavlinkcom_impl;

// {4d36e978-e325-11ce-bfc1-08002be10318}
const GUID serialDeviceClass = { 0x4d36e978, 0xe325, 0x11ce, 0xBF, 0xC1, 0x08, 0x00, 0x2B, 0xE1, 0x03, 0x18 };


bool parseVidPid(std::wstring deviceId, int* vid, int* pid)
{
    const wchar_t* ptr = deviceId.c_str();
    const wchar_t* end = ptr + deviceId.size();
    // parse out the VID number
    const wchar_t* pos = wcsstr(ptr, L"VID_");
    if (pos == NULL) {
        return false;
    }
    wchar_t* numberEnd = NULL;
    long c = wcstol(pos + 4, &numberEnd, 16);
    *vid = (int)c;

    // now the PID 
    pos = wcsstr(numberEnd, L"PID_");
    if (pos == NULL) {
        return false;
    }

    numberEnd = NULL;
    c = wcstol(pos + 4, &numberEnd, 16);
    *pid = (int)c;

    return true;
}

void parseDisplayName(std::wstring displayName, SerialPortInfo* info)
{
    info->displayName = displayName;
    const wchar_t* ptr = displayName.c_str();
    // parse out the VID number
    const wchar_t* pos = wcsrchr(ptr, '(');
    if (pos == NULL) {
        return;
    }
    pos++; // skip '('
    const wchar_t* end = wcschr(pos, ')');
    if (end != NULL) {
        info->portName = std::wstring(pos, (size_t)(end - pos));
    }
}

std::vector<SerialPortInfo> ListDevices()
{
    std::vector<SerialPortInfo> list;

    ULONG length;
    CONFIGRET cr = CM_Get_Device_Interface_List_SizeW(
        &length,
        const_cast<GUID*>(&GUID_DEVINTERFACE_COMPORT),
        nullptr,        // pDeviceID
        CM_GET_DEVICE_INTERFACE_LIST_PRESENT);

    if ((cr != CR_SUCCESS) || (length == 0)) {
        return list;
    }

    std::vector<WCHAR> buf(length);
    cr = CM_Get_Device_Interface_ListW(
        const_cast<GUID*>(&GUID_DEVINTERFACE_COMPORT),
        nullptr,        // pDeviceID
        buf.data(),
        static_cast<ULONG>(buf.size()),
        CM_GET_DEVICE_INTERFACE_LIST_PRESENT);

    if ((cr != CR_SUCCESS) || (length == 0)) {
        return list;
    }

    if (!buf[0]) {
        return list;
    }

    *buf.rbegin() = UNICODE_NULL;

    ULONG index = 0;
    for (PCWSTR deviceInterface = buf.data();
        *deviceInterface;
        deviceInterface += wcslen(deviceInterface) + 1) {

        const DEVPROPKEY propkey = {
            PKEY_DeviceInterface_Serial_PortName.fmtid,
            PKEY_DeviceInterface_Serial_PortName.pid
        };
        DEVPROPTYPE propertyType;
        WCHAR portName[512];
        ULONG propertyBufferSize = sizeof(portName);
        cr = CM_Get_Device_Interface_PropertyW(
            deviceInterface,
            &propkey,
            &propertyType,
            reinterpret_cast<BYTE*>(&portName),
            &propertyBufferSize,
            0); // ulFlags


        SerialPortInfo info;
        info.pid = 0;
        info.vid = 0;
        info.portName = deviceInterface;
        int dvid = 0, dpid = 0;
        if (parseVidPid(info.portName, &dvid, &dpid))
        {
            info.pid = dpid;
            info.vid = dvid;
        }
        printf("Found: %S\n", deviceInterface);

        if ((cr == CR_SUCCESS) && (propertyType == DEVPROP_TYPE_STRING)) {
            info.displayName = portName;
        }

        list.push_back(info);
        ++index;
    }
    return list;
}


std::vector<SerialPortInfo> MavLinkConnection::findSerialPorts(int vid, int pid)
{
    std::vector<SerialPortInfo> list;
    std::vector<SerialPortInfo> result = ListDevices();

    for (auto ptr = result.begin(), end = result.end(); ptr != end; ptr++)
    {
        SerialPortInfo& item = *ptr;
        if ((vid == 0 && pid == 0) || (item.vid == vid && item.pid == pid)) {
            list.push_back(item);
        }
    }

    return list;
}
#elif defined _WIN32 || defined _WIN64
//suppress
//OneCoreFindSerialPorts.obj : warning LNK4221: This object file does not define any previously undefined public symbols, so it will not be used by any link operation that consumes this library
__declspec(dllexport) void getRidOfLNK4221() {}
#endif
