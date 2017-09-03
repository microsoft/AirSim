// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#ifndef ONECORE
#include "MavLinkConnection.hpp"
#include <Windows.h>
#include <ObjIdl.h>
#include <SetupAPI.h>
#include <Cfgmgr32.h>
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
    const wchar_t* pos = wcsstr(ptr, L"\\VID_");
    if (pos == NULL) {
        return false;
    }
    wchar_t* numberEnd = NULL;
    long c = wcstol(pos + 5, &numberEnd, 16);
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


std::vector<SerialPortInfo> MavLinkConnection::findSerialPorts(int vid, int pid)
{
    bool debug = false;
    std::vector<SerialPortInfo> result;

    HDEVINFO classInfo = SetupDiGetClassDevsEx(&serialDeviceClass, NULL, NULL, DIGCF_PRESENT, NULL, NULL, NULL);

    if (classInfo == INVALID_HANDLE_VALUE) {
        return result;
    }

    SP_DEVINFO_DATA deviceInfo;
    deviceInfo.cbSize = sizeof(SP_DEVINFO_DATA);
    for (int devIndex = 0; SetupDiEnumDeviceInfo(classInfo, devIndex, &deviceInfo); devIndex++)
    {
        ULONG size;
        HRESULT hr = CM_Get_Device_ID_Size(&size, deviceInfo.DevInst, 0);
        if (hr == CR_SUCCESS) {
            std::wstring buffer(size + 1, '\0');
            hr = CM_Get_Device_ID(deviceInfo.DevInst, (PWSTR)buffer.c_str(), size + 1, 0);

            // examples:
            // PX4: USB\VID_26AC&PID_0011\0
            // FTDI cable: FTDIBUS\VID_0403+PID_6001+FTUAN9UJA\0000"
            //printf("Found: %S\n", buffer.c_str());

            int dvid = 0, dpid = 0;
            if (parseVidPid(buffer, &dvid, &dpid) &&
                ((dvid == vid && dpid == pid) || (vid == 0 && pid == 0)))
            {
                DWORD keyCount = 0;
                if (!SetupDiGetDevicePropertyKeys(classInfo, &deviceInfo, NULL, 0, &keyCount, 0)) {
                    if (GetLastError() != ERROR_INSUFFICIENT_BUFFER) {
                        continue;
                    }
                }

                SerialPortInfo portInfo;
                portInfo.pid = dpid;
                portInfo.vid = dvid;

                DEVPROPKEY* keyArray = new DEVPROPKEY[keyCount];

                if (SetupDiGetDevicePropertyKeys(classInfo, &deviceInfo, keyArray, keyCount, &keyCount, 0)) {

                    for (DWORD j = 0; j < keyCount; j++)
                    {
                        DEVPROPKEY* key = &keyArray[j];
                        bool isItemNameProperty = (key->fmtid == PKEY_ItemNameDisplay.fmtid && key->pid == PKEY_ItemNameDisplay.pid);
                        if (isItemNameProperty) {
                            ULONG bufferSize = 0;
                            DEVPROPTYPE propertyType;
                            CM_Get_DevNode_Property(deviceInfo.DevInst, &keyArray[j], &propertyType, NULL, &bufferSize, 0);
                            if (bufferSize > 0) {
                                BYTE* propertyBuffer = new BYTE[bufferSize];
                                hr = CM_Get_DevNode_Property(deviceInfo.DevInst, &keyArray[j], &propertyType, propertyBuffer, &bufferSize, 0);
                                if (hr == CR_SUCCESS) {
                                    std::wstring displayName((WCHAR*)propertyBuffer);
                                    parseDisplayName(displayName, &portInfo);
                                }
                                delete[] propertyBuffer;
                            }
                        }
                    }
                }

                result.push_back(portInfo);

                delete[] keyArray;
            }
        }
    }

    return result;
}
#endif