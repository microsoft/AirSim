#pragma once

#include <cstdint>

namespace simple_flight {

class IBoardInputPins {
public:
    virtual float readChannel(uint8_t index) const = 0; //output -1 to 1
};

}