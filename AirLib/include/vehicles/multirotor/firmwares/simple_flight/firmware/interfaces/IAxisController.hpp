#pragma once

#include <cstdint>
#include "IUpdatable.hpp"
#include "IGoal.hpp"
#include "IStateEstimator.hpp"
#include "IBoardClock.hpp"

namespace simple_flight
{

class IAxisController : public IUpdatable
{
public:
    virtual void initialize(unsigned int axis, const IGoal* goal, const IStateEstimator* state_estimator) = 0;
    virtual TReal getOutput() = 0;

    virtual void reset() override
    {
        //disable checks for reset/update sequence because
        //this object may get created but not used
        clearResetUpdateAsserts();
        IUpdatable::reset();
    }
};

} //namespace