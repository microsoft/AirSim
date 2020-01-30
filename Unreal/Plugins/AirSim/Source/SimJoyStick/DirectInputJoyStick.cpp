// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "DirectInputJoystick.h"
#include "Math/UnrealMathUtility.h"

#if defined _WIN32 || defined _WIN64

#ifndef DIRECTINPUT_VERSION
#define DIRECTINPUT_VERSION 0x0800
#endif

#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS 1
#endif

#include "common/common_utils/WindowsApisCommonPre.hpp"

#include "common/common_utils/MinWinDefines.hpp"
#include <windows.h>

#include <dinput.h>

#include <dinputd.h>
#include <ole2.h> //SysAllocString

// Stuff to filter out XInput devices
#ifndef FALSE
#define FALSE               0
#endif

#ifndef TRUE
#define TRUE                1
#endif
#include <wbemidl.h>

#include "common/common_utils/WindowsApisCommonPost.hpp"



//-----------------------------------------------------------------------------
// Defines, constants, and global variables
//-----------------------------------------------------------------------------
#define DIJT_SAFE_DELETE(p)  { if(p) { delete (p);     (p)=nullptr; } }
#define DIJT_SAFE_RELEASE(p) { if(p) { (p)->Release(); (p)=nullptr; } }

struct DirectInputJoyStick::impl {
private:
    struct XINPUT_DEVICE_NODE
    {
        DWORD dwVidPid;
        XINPUT_DEVICE_NODE* pNext;
    };

    struct DI_ENUM_CONTEXT
    {
        DIJOYCONFIG* pPreferredJoyCfg;
        bool bPreferredJoyCfgValid;
    };

    bool                    g_bFilterOutXinputDevices = false;
    XINPUT_DEVICE_NODE*     g_pXInputDeviceList = nullptr;

    LPDIRECTINPUT8          g_pDI = nullptr;
    LPDIRECTINPUTDEVICE8    g_pJoystick = nullptr;

    LPDIRECTINPUTEFFECT		g_pAutoCenterHandle = nullptr;
    DIEFFECT				g_sAutoCenterConfig;

    LPDIRECTINPUTEFFECT		g_pWheelRumbleHandle = nullptr;
    DIEFFECT				g_sWheelRumbleConfig;

    DIJOYCONFIG PreferredJoyCfg = { 0 };
    DI_ENUM_CONTEXT enumContext;

public:
    Capabilities caps;
    JoystickState state;
    JoystickInfo joystick_info;

    bool initialize(int joystick_index)
    {
        if (joystick_index < 0)
            return false;
        return InitDirectInput(joystick_index) == S_OK;
    }

    // Magnitude ranges from -1 to 1
    void setAutoCenterStrength(double magnitude)
    {
        DICONSTANTFORCE cf = { magnitude * 10000 };

        g_sAutoCenterConfig.cbTypeSpecificParams = sizeof(DICONSTANTFORCE);
        g_sAutoCenterConfig.lpvTypeSpecificParams = &cf;

        if (g_pAutoCenterHandle) {
            g_pAutoCenterHandle->SetParameters(&g_sAutoCenterConfig, DIEP_DIRECTION |
                DIEP_TYPESPECIFICPARAMS | DIEP_START);
        }

    }

#define FFWRMAX 0.08

    // Strength ranges from 0 to 1
    void setWheelRumbleStrength(double strength)
    {
        DIPERIODIC pf = { FFWRMAX * strength * 10000,0,0,0.06 * 1000000 };

        g_sWheelRumbleConfig.cbTypeSpecificParams = sizeof(DIPERIODIC);
        g_sWheelRumbleConfig.lpvTypeSpecificParams = &pf;

        if (g_pWheelRumbleHandle) {
            g_pWheelRumbleHandle->SetParameters(&g_sWheelRumbleConfig, DIEP_DIRECTION |
                DIEP_TYPESPECIFICPARAMS | DIEP_START);
        }
    }

    const JoystickState& getState(bool update_state = true)
    {
        if (update_state)
            UpdateInputState();

        return state;
    }

    const Capabilities& getCapabilities()
    {
        return caps;
    }

    const JoystickInfo& getJoystickInfo()
    {
        return joystick_info;
    }

    ~impl()
    {
        FreeDirectInput();
    }

private:
    std::string toString(const std::wstring& wstr)
    {
        return std::string(wstr.begin(), wstr.end());
    }

    DIGUID toDIGUID(const GUID& guid)
    {
        DIGUID g;
        g.Data1 = guid.Data1;
        g.Data2 = guid.Data2;
        g.Data3 = guid.Data3;
        std::copy(std::begin(guid.Data4), std::end(guid.Data4), g.Data4);

        return g;
    }

    HRESULT InitForceFeedback()
    {

        HRESULT hr;
        DWORD rgdwAxes[2] = { DIJOFS_X, DIJOFS_Y };
        LONG rglDirection[2] = { 0, 0 };

        if (FAILED(hr = g_pJoystick->SetCooperativeLevel(GetActiveWindow(), DISCL_EXCLUSIVE | DISCL_BACKGROUND)))
            return hr;

        if (FAILED(hr = g_pJoystick->Acquire()))
            return hr;

        // Autocenter
        ZeroMemory(&g_sAutoCenterConfig, sizeof(g_sAutoCenterConfig));

        g_sAutoCenterConfig.dwStartDelay = 0;

        g_sAutoCenterConfig.dwSize = sizeof(DIEFFECT);
        g_sAutoCenterConfig.dwFlags = DIEFF_CARTESIAN | DIEFF_OBJECTOFFSETS;
        g_sAutoCenterConfig.dwDuration = INFINITE;
        g_sAutoCenterConfig.dwSamplePeriod = 0;
        g_sAutoCenterConfig.dwGain = DI_FFNOMINALMAX;
        g_sAutoCenterConfig.dwTriggerButton = DIEB_NOTRIGGER;
        g_sAutoCenterConfig.dwTriggerRepeatInterval = 0;
        g_sAutoCenterConfig.cAxes = 1;
        g_sAutoCenterConfig.rgdwAxes = rgdwAxes;
        g_sAutoCenterConfig.rglDirection = rglDirection;

        g_sAutoCenterConfig.lpEnvelope = 0;
        g_sAutoCenterConfig.dwStartDelay = 0;

        DICONSTANTFORCE cf = { 0 };

        g_sAutoCenterConfig.cbTypeSpecificParams = sizeof(DICONSTANTFORCE);
        g_sAutoCenterConfig.lpvTypeSpecificParams = &cf;

        if (FAILED(hr = g_pJoystick->CreateEffect(GUID_ConstantForce, &g_sAutoCenterConfig, &g_pAutoCenterHandle, nullptr)))
            return hr;

        if (FAILED(hr = g_pAutoCenterHandle->Start(INFINITE, 0)))
            return hr;

        // Rumble
        ZeroMemory(&g_sWheelRumbleConfig, sizeof(g_sWheelRumbleConfig));

        g_sWheelRumbleConfig.dwStartDelay = 0;

        g_sWheelRumbleConfig.dwSize = sizeof(DIEFFECT);
        g_sWheelRumbleConfig.dwFlags = DIEFF_CARTESIAN | DIEFF_OBJECTOFFSETS;
        g_sWheelRumbleConfig.dwDuration = INFINITE;
        g_sWheelRumbleConfig.dwSamplePeriod = 0;
        g_sWheelRumbleConfig.dwGain = DI_FFNOMINALMAX;
        g_sWheelRumbleConfig.dwTriggerButton = DIEB_NOTRIGGER;
        g_sWheelRumbleConfig.dwTriggerRepeatInterval = 0;
        g_sWheelRumbleConfig.cAxes = 1;
        g_sWheelRumbleConfig.rgdwAxes = rgdwAxes;
        g_sWheelRumbleConfig.rglDirection = rglDirection;

        g_sWheelRumbleConfig.lpEnvelope = 0;
        g_sWheelRumbleConfig.dwStartDelay = 0;

        DIPERIODIC pf = { 0,0,0,0.08 };

        g_sWheelRumbleConfig.cbTypeSpecificParams = sizeof(DIPERIODIC);
        g_sWheelRumbleConfig.lpvTypeSpecificParams = &pf;

        if (FAILED(hr = g_pJoystick->CreateEffect(GUID_Sine, &g_sWheelRumbleConfig, &g_pWheelRumbleHandle, nullptr)))
            return hr;

        if (FAILED(hr = g_pWheelRumbleHandle->Start(INFINITE, 0)))
            return hr;

        return S_OK;
    }


    HRESULT InitDirectInput(unsigned int joystick_index)
    {
        HRESULT hr;

        state.is_initialized = false;

        // Register with the DirectInput subsystem and get a pointer
        // to a IDirectInput interface we can use.
        // Create a DInput object
        if (FAILED(hr = DirectInput8Create(GetModuleHandle(nullptr), DIRECTINPUT_VERSION,
            IID_IDirectInput8, (VOID**)&g_pDI, nullptr)))
            return hr;


        if (g_bFilterOutXinputDevices)
            SetupForIsXInputDevice();

        enumContext.pPreferredJoyCfg = &PreferredJoyCfg;
        enumContext.bPreferredJoyCfgValid = false;

        IDirectInputJoyConfig8* pJoyConfig = nullptr;
        if (FAILED(hr = g_pDI->QueryInterface(IID_IDirectInputJoyConfig8, (void**)&pJoyConfig))) {
            state.message = "QueryInterface on IID_IDirectInputJoyConfig8 failed";
            return hr;
        }

        PreferredJoyCfg.dwSize = sizeof(PreferredJoyCfg);
        if (SUCCEEDED(pJoyConfig->GetConfig(joystick_index, &PreferredJoyCfg, DIJC_GUIDINSTANCE))) { // This function is expected to fail if no joystick is attached
            enumContext.bPreferredJoyCfgValid = true;

            joystick_info.is_valid = true;
            joystick_info.instance_guide = toDIGUID(PreferredJoyCfg.guidInstance);
            joystick_info.pid_vid = toString(PreferredJoyCfg.wszType);
        }
        DIJT_SAFE_RELEASE(pJoyConfig);

        // Look for a simple joystick we can use for this sample program.
        if (FAILED(hr = g_pDI->EnumDevices(DI8DEVCLASS_GAMECTRL,
            DirectInputJoyStick::impl::EnumJoysticksCallback,
            this, DIEDFL_ATTACHEDONLY))) {

            state.message = "EnumDevices failed";
            return hr;
        }

        if (g_bFilterOutXinputDevices)
            CleanupForIsXInputDevice();

        // Make sure we got a joystick
        if (!g_pJoystick)
        {
            state.message = "Joystick at index " + std::to_string(joystick_index) + " is not available";
            return S_FALSE;
        }

        // Set the data format to "simple joystick" - a predefined data format 
        //
        // A data format specifies which controls on a device we are interested in,
        // and how they should be reported. This tells DInput that we will be
        // passing a DIJOYSTATE2 structure to IDirectInputDevice::GetDeviceState().
        if (FAILED(hr = g_pJoystick->SetDataFormat(&c_dfDIJoystick2)))
        {
            state.message = "Device does not support DIJOYSTATE2";
            return hr;
        }

        // Set the cooperative level to let DInput know how this device should
        // interact with the system and with other DInput applications.
        if (FAILED(hr = g_pJoystick->SetCooperativeLevel(NULL, DISCL_NONEXCLUSIVE | DISCL_BACKGROUND))) {

            state.message = "SetCooperativeLevel failed";
            return hr;
        }

        // Enumerate the joystick objects. The callback function enabled user
        // interface elements for objects that are found, and sets the min/max
        // values property for discovered axes.
        if (FAILED(hr = g_pJoystick->EnumObjects(DirectInputJoyStick::impl::EnumObjectsCallback,
            (VOID*) this, DIDFT_ALL))) {

            state.message = "EnumObjects failed";
            return hr;
        }

        InitForceFeedback();

        state.is_initialized = true;
        return S_OK;
    }

    //-----------------------------------------------------------------------------
    // Enum each PNP device using WMI and check each device ID to see if it contains 
    // "IG_" (ex. "VID_045E&PID_028E&IG_00").  If it does, then it's an XInput device
    // Unfortunately this information can not be found by just using DirectInput.
    // Checking against a VID/PID of 0x028E/0x045E won't find 3rd party or future 
    // XInput devices.
    //
    // This function stores the list of xinput devices in a linked list 
    // at g_pXInputDeviceList, and IsXInputDevice() searchs that linked list
    //-----------------------------------------------------------------------------
    HRESULT SetupForIsXInputDevice()
    {
        IWbemServices* pIWbemServices = nullptr;
        IEnumWbemClassObject* pEnumDevices = nullptr;
        IWbemLocator* pIWbemLocator = nullptr;
        IWbemClassObject* pDevices[20] = { 0 };
        BSTR bstrDeviceID = nullptr;
        BSTR bstrClassName = nullptr;
        BSTR bstrNamespace = nullptr;
        DWORD uReturned = 0;
        bool bCleanupCOM = false;
        UINT iDevice = 0;
        VARIANT var;
        HRESULT hr;

        // CoInit if needed
        hr = CoInitialize(nullptr);
        bCleanupCOM = SUCCEEDED(hr);

        // Create WMI
        hr = CoCreateInstance(__uuidof(WbemLocator),
            nullptr,
            CLSCTX_INPROC_SERVER,
            __uuidof(IWbemLocator),
            (LPVOID*)&pIWbemLocator);
        if (FAILED(hr) || pIWbemLocator == nullptr)
            goto LCleanup;

        // Create BSTRs for WMI
        bstrNamespace = SysAllocString(L"\\\\.\\root\\cimv2"); if (bstrNamespace == nullptr) goto LCleanup;
        bstrDeviceID = SysAllocString(L"DeviceID");           if (bstrDeviceID == nullptr)  goto LCleanup;
        bstrClassName = SysAllocString(L"Win32_PNPEntity");    if (bstrClassName == nullptr) goto LCleanup;

        // Connect to WMI 
        hr = pIWbemLocator->ConnectServer(bstrNamespace, nullptr, nullptr, 0L,
            0L, nullptr, nullptr, &pIWbemServices);
        if (FAILED(hr) || pIWbemServices == nullptr)
            goto LCleanup;

        // Switch security level to IMPERSONATE
        (void)CoSetProxyBlanket(pIWbemServices, RPC_C_AUTHN_WINNT, RPC_C_AUTHZ_NONE, nullptr,
            RPC_C_AUTHN_LEVEL_CALL, RPC_C_IMP_LEVEL_IMPERSONATE, nullptr, 0);

        // Get list of Win32_PNPEntity devices
        hr = pIWbemServices->CreateInstanceEnum(bstrClassName, 0, nullptr, &pEnumDevices);
        if (FAILED(hr) || pEnumDevices == nullptr)
            goto LCleanup;

        // Loop over all devices
        for (; ; )
        {
            // Get 20 at a time
            hr = pEnumDevices->Next(10000, 20, pDevices, &uReturned);
            if (FAILED(hr))
                goto LCleanup;
            if (uReturned == 0)
                break;

            for (iDevice = 0; iDevice < uReturned; iDevice++)
            {
                if (!pDevices[iDevice])
                    continue;

                // For each device, get its device ID
                hr = pDevices[iDevice]->Get(bstrDeviceID, 0L, &var, nullptr, nullptr);
                if (SUCCEEDED(hr) && var.vt == VT_BSTR && var.bstrVal != nullptr)
                {
                    // Check if the device ID contains "IG_".  If it does, then it's an XInput device
                    // Unfortunately this information can not be found by just using DirectInput 
                    if (wcsstr(var.bstrVal, L"IG_"))
                    {
                        // If it does, then get the VID/PID from var.bstrVal
                        DWORD dwPid = 0, dwVid = 0;
                        WCHAR* strVid = wcsstr(var.bstrVal, L"VID_");
                        if (strVid && swscanf(strVid, L"VID_%lX", &dwVid) != 1)
                            dwVid = 0;
                        WCHAR* strPid = wcsstr(var.bstrVal, L"PID_");
                        if (strPid && swscanf(strPid, L"PID_%lX", &dwPid) != 1)
                            dwPid = 0;

                        DWORD dwVidPid = MAKELONG(dwVid, dwPid);

                        // Add the VID/PID to a linked list
                        XINPUT_DEVICE_NODE* pNewNode = new XINPUT_DEVICE_NODE;
                        if (pNewNode)
                        {
                            pNewNode->dwVidPid = dwVidPid;
                            pNewNode->pNext = g_pXInputDeviceList;
                            g_pXInputDeviceList = pNewNode;
                        }
                    }
                }
                DIJT_SAFE_RELEASE(pDevices[iDevice]);
            }
        }

    LCleanup:
        if (bstrNamespace)
            SysFreeString(bstrNamespace);
        if (bstrDeviceID)
            SysFreeString(bstrDeviceID);
        if (bstrClassName)
            SysFreeString(bstrClassName);
        for (iDevice = 0; iDevice < 20; iDevice++)
            DIJT_SAFE_RELEASE(pDevices[iDevice]);
        DIJT_SAFE_RELEASE(pEnumDevices);
        DIJT_SAFE_RELEASE(pIWbemLocator);
        DIJT_SAFE_RELEASE(pIWbemServices);

        return hr;
    }


    //-----------------------------------------------------------------------------
    // Returns true if the DirectInput device is also an XInput device.
    // Call SetupForIsXInputDevice() before, and CleanupForIsXInputDevice() after
    //-----------------------------------------------------------------------------
    bool IsXInputDevice(const GUID* pGuidProductFromDirectInput)
    {
        // Check each xinput device to see if this device's vid/pid matches
        XINPUT_DEVICE_NODE* pNode = g_pXInputDeviceList;
        while (pNode)
        {
            if (pNode->dwVidPid == pGuidProductFromDirectInput->Data1)
                return true;
            pNode = pNode->pNext;
        }

        return false;
    }


    //-----------------------------------------------------------------------------
    // Cleanup needed for IsXInputDevice()
    //-----------------------------------------------------------------------------
    void CleanupForIsXInputDevice()
    {
        // Cleanup linked list
        XINPUT_DEVICE_NODE* pNode = g_pXInputDeviceList;
        while (pNode)
        {
            XINPUT_DEVICE_NODE* pDelete = pNode;
            pNode = pNode->pNext;
            DIJT_SAFE_DELETE(pDelete);
        }
    }

    //-----------------------------------------------------------------------------
    // Name: EnumJoysticksCallback()
    // Desc: Called once for each enumerated joystick. If we find one, create a
    //       device interface on it so we can play with it.
    //-----------------------------------------------------------------------------
    static BOOL CALLBACK EnumJoysticksCallback(const DIDEVICEINSTANCE* pdidInstance,
        VOID* pContext)
    {
        DirectInputJoyStick::impl *obj = reinterpret_cast<DirectInputJoyStick::impl *>(pContext);

        auto pEnumContext = &obj->enumContext;
        HRESULT hr;

        if (obj->g_bFilterOutXinputDevices && obj->IsXInputDevice(&pdidInstance->guidProduct))
            return DIENUM_CONTINUE;

        // Skip anything other than the perferred joystick device as defined by the control panel.  
        // Instead you could store all the enumerated joysticks and let the user pick.
        if (pEnumContext->bPreferredJoyCfgValid &&
            !IsEqualGUID(pdidInstance->guidInstance, pEnumContext->pPreferredJoyCfg->guidInstance))
            return DIENUM_CONTINUE;

        // Obtain an interface to the enumerated joystick.
        hr = obj->g_pDI->CreateDevice(pdidInstance->guidInstance, &obj->g_pJoystick, nullptr);

        // If it failed, then we can't use this joystick. (Maybe the user unplugged
        // it while we were in the middle of enumerating it.)
        if (FAILED(hr)) {
            obj->state.message = "CreateDevice failed";
            return DIENUM_CONTINUE;
        }

        // Stop enumeration. Note: we're just taking the first joystick we get. You
        // could store all the enumerated joysticks and let the user pick.
        return DIENUM_STOP;
    }

    //-----------------------------------------------------------------------------
    // Name: EnumObjectsCallback()
    // Desc: Callback function for enumerating objects (axes, buttons, POVs) on a 
    //       joystick. This function enables user interface elements for objects
    //       that are found to exist, and scales axes min/max values.
    //-----------------------------------------------------------------------------
    static BOOL CALLBACK EnumObjectsCallback(const DIDEVICEOBJECTINSTANCE* pdidoi,
        VOID* pContext)
    {
        DirectInputJoyStick::impl *obj = reinterpret_cast<DirectInputJoyStick::impl *>(pContext);

        static int nSliderCount = 0;  // Number of returned slider controls
        static int nPOVCount = 0;     // Number of returned POV controls

                                      // For axes that are returned, set the DIPROP_RANGE property for the
                                      // enumerated axis in order to scale min/max values.
        if (pdidoi->dwType & DIDFT_AXIS)
        {
            DIPROPRANGE diprg;
            diprg.diph.dwSize = sizeof(DIPROPRANGE);
            diprg.diph.dwHeaderSize = sizeof(DIPROPHEADER);
            diprg.diph.dwHow = DIPH_BYID;
            diprg.diph.dwObj = pdidoi->dwType; // Specify the enumerated axis
            diprg.lMin = -1000;
            diprg.lMax = +1000;

            // Set the range for the axis
            if (FAILED(obj->g_pJoystick->SetProperty(DIPROP_RANGE, &diprg.diph))) {
                obj->state.message = "setting range for axis failed";
                return DIENUM_STOP;
            }

        }


        // Set the UI to reflect what objects the joystick supports
        if (pdidoi->guidType == GUID_XAxis)
        {
            obj->caps.x_axis = true;
        }
        if (pdidoi->guidType == GUID_YAxis)
        {
            obj->caps.y_axis = true;
        }
        if (pdidoi->guidType == GUID_ZAxis)
        {
            obj->caps.z_axis = true;
        }
        if (pdidoi->guidType == GUID_RxAxis)
        {
            obj->caps.rx_axis = true;
        }
        if (pdidoi->guidType == GUID_RyAxis)
        {
            obj->caps.ry_axis = true;
        }
        if (pdidoi->guidType == GUID_RzAxis)
        {
            obj->caps.rz_axis = true;
        }
        if (pdidoi->guidType == GUID_Slider)
        {
            switch (nSliderCount++)
            {
            case 0: obj->caps.slider0 = true;
                break;

            case 1: obj->caps.slider1 = true;
                break;
            }
        }
        if (pdidoi->guidType == GUID_POV)
        {
            switch (nPOVCount++)
            {
            case 0:
                obj->caps.pov0 = true;
                break;

            case 1:
                obj->caps.pov1 = true;
                break;

            case 2:
                obj->caps.pov2 = true;
                break;

            case 3: obj->caps.pov3 = true;
                break;
            }
        }

        return DIENUM_CONTINUE;
    }

    //-----------------------------------------------------------------------------
    // Name: UpdateInputState()
    // Desc: Get the input device's state and display it.
    //-----------------------------------------------------------------------------
    HRESULT UpdateInputState()
    {
        HRESULT hr;
        DIJOYSTATE2 js;           // DInput joystick state 

        state.is_valid = false;

        if (!g_pJoystick) {
            state.message = "No device at index";
            return S_OK;
        }

        // Poll the device to read the current state
        hr = g_pJoystick->Poll();
        if (FAILED(hr))
        {
            state.message = "device stream interrupted";

            // DInput is telling us that the input stream has been
            // interrupted. We aren't tracking any state between polls, so
            // we don't have any special reset that needs to be done. We
            // just re-acquire and try again.
            hr = g_pJoystick->Acquire();
            //while (hr == DIERR_INPUTLOST)
            //    hr = g_pJoystick->Acquire();

            // hr may be DIERR_OTHERAPPHASPRIO or other errors.  This
            // may occur when the app is minimized or in the process of 
            // switching, so just try again later 
            return S_OK;
        }

        // Get the input's device state
        if (FAILED(hr = g_pJoystick->GetDeviceState(sizeof(DIJOYSTATE2), &js))) {
            state.message = "GetDeviceState failed";
            return hr; // The device should have been acquired during the Poll()
        }

        state.is_valid = true;
        state.message = "";

        // Axes
        state.x = js.lX; state.y = js.lY; state.z = js.lZ;
        state.rx = js.lRx; state.ry = js.lRy; state.rz = js.lRz;
        state.slider0 = js.rglSlider[0]; state.slider1 = js.rglSlider[1];
        // Slider controls
        state.slider0 = js.rglSlider[0]; state.slider1 = js.rglSlider[1];
        // Points of view
        state.pov0 = js.rgdwPOV[0]; state.pov1 = js.rgdwPOV[1];
        state.pov2 = js.rgdwPOV[2]; state.pov3 = js.rgdwPOV[3];

        // Buttons
        for (int i = 0; i < 128; i++)
        {
            state.buttons[i] = js.rgbButtons[i];
        }

        return S_OK;
    }

    //-----------------------------------------------------------------------------
    // Name: FreeDirectInput()
    // Desc: Initialize the DirectInput variables.
    //-----------------------------------------------------------------------------
    VOID FreeDirectInput()
    {
        // Unacquire the device one last time just in case 
        // the app tried to exit while the device is still acquired.
        if (g_pJoystick)
            g_pJoystick->Unacquire();

        // Release any DirectInput objects.
        DIJT_SAFE_RELEASE(g_pAutoCenterHandle);
        DIJT_SAFE_RELEASE(g_pWheelRumbleHandle);
        DIJT_SAFE_RELEASE(g_pJoystick);
        DIJT_SAFE_RELEASE(g_pDI);
    }


};

DirectInputJoyStick::DirectInputJoyStick()
{
    pimpl_.reset(new impl());
}

DirectInputJoyStick::~DirectInputJoyStick()
{
    //nop
}

bool DirectInputJoyStick::initialize(int joystick_index)
{
    return pimpl_->initialize(joystick_index);
}
void DirectInputJoyStick::setAutoCenter(double strength)
{
    pimpl_->setAutoCenterStrength(FMath::Clamp<double>(strength, -1.0, 1.0));
}
void DirectInputJoyStick::setWheelRumble(double strength)
{
    pimpl_->setWheelRumbleStrength(FMath::Clamp<double>(strength, 0.0, 1.0));
}
const DirectInputJoyStick::JoystickState& DirectInputJoyStick::getState(bool update_state)
{
    return pimpl_->getState(update_state);
}
const DirectInputJoyStick::Capabilities& DirectInputJoyStick::getCapabilities()
{
    return pimpl_->getCapabilities();
}

const DirectInputJoyStick::JoystickInfo& DirectInputJoyStick::getJoystickInfo()
{
    return pimpl_->getJoystickInfo();
}

#endif