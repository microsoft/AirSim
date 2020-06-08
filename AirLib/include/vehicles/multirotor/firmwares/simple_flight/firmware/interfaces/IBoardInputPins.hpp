#pragma once

#include <cstdint>

namespace simple_flight {

class IBoardInputPins {
public:
    virtual float readChannel(uint16_t index) const = 0; //output -1 to 1
    virtual bool isRcConnected() const = 0;
    virtual float getAvgMotorOutput() const = 0;
};

}