// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include <cstdint>
#include <memory>

class SimJoyStick
{
public:
    struct State {
        int16_t left_x, left_y, right_x, right_y;
        bool left_trigger, right_trigger;
        uint16_t buttons;
        bool is_connected;
        std::string message;
        unsigned long connection_error_code = std::numeric_limits<unsigned long>::max();
    };

    static void setInitializedSuccess(bool success);
    static bool isInitializedSuccess();

    void getJoyStickState(unsigned int index, State& state);

    SimJoyStick();
    ~SimJoyStick();    //required for pimpl
private:
    static bool initialized_success_;
    
    struct impl;
    std::unique_ptr<impl> pimpl_;
};

