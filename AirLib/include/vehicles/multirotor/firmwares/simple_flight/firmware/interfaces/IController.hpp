#pragma once

#include <cstdint>
#include "IUpdatable.hpp"
#include "IGoal.hpp"
#include "IStateEstimator.hpp"
#include "IBoardClock.hpp"

namespace simple_flight
{

class IController : public IUpdatable
{
public:
    virtual void initialize(const IGoal* goal, const IStateEstimator* state_estimator) = 0;
    virtual const Axis4r& getOutput() = 0;
    virtual bool isLastGoalModeAllPassthrough() = 0;
};

} //namespace