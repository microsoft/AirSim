#pragma once

#include "CommonStructs.hpp"

namespace simple_flight {

class IStateEstimator {
public:
    virtual Angles getAngles() const = 0;
};

}