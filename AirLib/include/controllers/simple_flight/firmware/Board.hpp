#pragma once

#include <cstdint>
#include "CommonStructs.hpp"

namespace simple_flight {

class Board : 
    public IBoardClock, 
    public IBoardInputPins, 
    public IBoardOutputPins,
    public IBoardSensors {
public:
    virtual void reset() = 0;
    virtual void update() = 0;
};


} //namespace