#pragma once

#include "IUpdatable.hpp"
#include "IGoalInput.hpp"
#include "IStateEstimator.hpp"


namespace simple_flight {

class IFirmware : public IUpdatable {
public:
    virtual IGoalInput& getGoalInput() = 0;
    virtual const IStateEstimator& getStateEstimator() = 0;
};

} //namespace