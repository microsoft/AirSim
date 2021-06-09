// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "common/Common.hpp"

class SimJoyStick
{
public:
    struct AxisMap
    {
        enum class AxisType : int
        {
            Auto = 0,
            LeftX,
            LeftY,
            LeftZ,
            RightX,
            RightY,
            RightZ
        };
        enum class AxisDirection : int
        {
            Auto = 0,
            Normal,
            Reverse
        };

        AxisType rc_axis = AxisType::Auto;
        float min_val = -1000, max_val = 1000;
        AxisDirection direction = AxisDirection::Auto;
    };
    struct AxisMaps
    {
        AxisMap left_x, left_y, left_z, right_x, right_y, right_z;
    } axis_maps;

    struct State
    {
        float left_x, left_y, left_z;
        float right_x, right_y, right_z;

        float slider0, slider1;
        float pov0, pov1, pov2, pov3;

        uint16_t buttons;
        bool is_initialized = false;
        bool is_valid = false;

        std::string message;
        std::string pid_vid;
        unsigned long connection_error_code = std::numeric_limits<unsigned long>::max();
    };

    void getJoyStickState(int index, State& state) const;
    // strength ranges from -1 to 1
    void setAutoCenter(int index, double strength);

    // strength ranges from 0 to 1
    void setWheelRumble(int index, double strength);

    SimJoyStick();
    ~SimJoyStick(); //required for pimpl
private:
    static bool initialized_success_;
    //bool api_control_enabled_ = false;

    struct impl;
    std::unique_ptr<impl> pimpl_;
};
