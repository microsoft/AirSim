#pragma once

#include <cstdint>

namespace simple_flight
{

class IBoardOutputPins
{
public:
    virtual void writeOutput(uint16_t index, float val) = 0; //val = -1 to 1 for reversible motors otherwise 0 to 1
    virtual void setLed(uint8_t index, int32_t color) = 0;

    virtual ~IBoardOutputPins() = default;
};
}
