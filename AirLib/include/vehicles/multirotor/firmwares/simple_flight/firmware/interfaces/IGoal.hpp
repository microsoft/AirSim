#pragma once

#include "IUpdatable.hpp"
#include "CommonStructs.hpp"

namespace simple_flight
{

class IGoal
{
public:
    virtual const Axis4r& getGoalValue() const = 0;
    virtual const GoalMode& getGoalMode() const = 0;
};

} //namespace