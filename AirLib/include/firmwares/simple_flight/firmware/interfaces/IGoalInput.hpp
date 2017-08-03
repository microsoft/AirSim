#pragma once

#include "IUpdatable.hpp"
#include "CommonStructs.hpp"

namespace simple_flight {

class IGoalInput : public IUpdatable {
public:
    virtual const Axis4r& getGoal() const = 0;
    virtual const GoalMode& getGoalMode() const = 0;
};

} //namespace