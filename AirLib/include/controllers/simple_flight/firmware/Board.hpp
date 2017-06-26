#pragma once

#include <cstdint>
#include "CommonStructs.hpp"

namespace simple_flight {

class Board : 
    public IBoardClock, 
    public IBoardInputPins, 
    public IBoardOutputPins {
public:
    virtual void readAccel(float accel[3]) const = 0; //accel in m/s^2
    virtual void readGyro(float gyro[3]) const = 0; //angular velocity vector rad/sec
    virtual void reset() = 0;
    virtual void update() = 0;
};


} //namespace