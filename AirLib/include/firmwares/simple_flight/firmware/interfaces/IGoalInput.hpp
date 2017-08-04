#pragma once

#include <string>
#include "CommonStructs.hpp"
#include "IGoal.hpp"

namespace simple_flight {

class IGoalInput : public IGoal {
public:
    virtual bool canRequestApiControl(std::string& message) = 0;
    virtual bool hasApiControl() = 0;
    virtual bool requestApiControl(std::string& message) = 0;
    virtual void releaseApiControl() = 0;
    virtual bool setGoalAndMode(const Axis4r* goal, const GoalMode* goal_mode, std::string& message) = 0;
};

} //namespace

