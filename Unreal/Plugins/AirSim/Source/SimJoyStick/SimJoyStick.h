// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include <cstdint>
#include <memory>

class SimJoyStick
{
public:
    struct AxisMap {
        enum class AxisType : int {
            Auto = 0, LeftX, LeftY, LeftZ, RightX, RightY, RightZ
        };
        enum class AxisDirection : int {
            Auto = 0, Normal, Reverse
        };

        AxisType rc_axis = AxisType::Auto;
        float min_val = -1000, max_val = 1000;
        AxisDirection direction = AxisDirection::Auto;
    };
    struct AxisMaps {
        AxisMap left_x, left_y, left_z, right_x, right_y, right_z;
    } axis_maps;

    struct State {
        float left_x, left_y, left_z, right_x, right_y, right_z;
        uint16_t buttons;
        bool is_initialized = false;
        bool is_valid = false;

        std::string message;
        unsigned long connection_error_code = std::numeric_limits<unsigned long>::max();
    };

    void getJoyStickState(unsigned int index, State& state);

    SimJoyStick();
    ~SimJoyStick();    //required for pimpl
private:
    static bool initialized_success_;
    
    struct impl;
    std::unique_ptr<impl> pimpl_;
};

