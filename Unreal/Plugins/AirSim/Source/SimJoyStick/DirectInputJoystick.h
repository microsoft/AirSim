#pragma once

#if defined _WIN32 || defined _WIN64

#include <memory>
#include <string>

class DirectInputJoyStick
{
public:
    struct DIGUID
    {
        unsigned long Data1;
        unsigned short Data2;
        unsigned short Data3;
        unsigned char Data4[8];
    };

    struct Capabilities
    {
        bool x_axis = false, y_axis = false, z_axis = false;
        bool rx_axis = false, ry_axis = false, rz_axis = false;
        bool slider0 = false, slider1 = false;
        bool pov0 = false, pov1 = false, pov2 = false, pov3 = false;
    };

    struct JoystickInfo
    {
        DIGUID instance_guide;
        std::string pid_vid;
        bool is_valid = false;
    };

    struct JoystickState
    {
        long x = 0, y = 0, z = 0;
        long rx = 0, ry = 0, rz = 0;
        long slider0 = 0, slider1 = 0;
        unsigned long pov0 = 0, pov1 = 0, pov2 = 0, pov3 = 0;
        unsigned char buttons[128] = {};
        bool is_valid = false;
        bool is_initialized = false;
        std::string message;
    };

public:
    DirectInputJoyStick();
    ~DirectInputJoyStick();
    bool initialize(int joystick_index);
    // strength ranges from -1 to 1
    void setAutoCenter(double strength);

    // strength ranges from 0 to 1
    void setWheelRumble(double strength);
    const JoystickState& getState(bool update_state = true);
    const Capabilities& getCapabilities();
    const JoystickInfo& getJoystickInfo();

private: //pimpl
    struct impl;
    std::unique_ptr<impl> pimpl_;
};

#endif