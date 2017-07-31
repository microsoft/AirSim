#pragma once

#include "CommonStructs.hpp"

namespace simple_flight {

class IStateEstimator {
public:
    virtual Angles getAngles() const = 0;
    virtual Axis3r getAngulerVelocity() const = 0;
};

}