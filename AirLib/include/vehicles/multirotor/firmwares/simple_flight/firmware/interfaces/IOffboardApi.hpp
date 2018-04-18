#pragma once

#include <string>
#include "CommonStructs.hpp"
#include "IGoal.hpp"

namespace simple_flight {

class IOffboardApi : public IGoal {
public:
    virtual bool canRequestApiControl(std::string& message) = 0;
    virtual bool hasApiControl() = 0;
    virtual bool requestApiControl(std::string& message) = 0;
    virtual void releaseApiControl() = 0;
    virtual bool setGoalAndMode(const Axis4r* goal, const GoalMode* goal_mode, std::string& message) = 0;

    virtual bool arm(std::string& message) = 0;
    virtual bool disarm(std::string& message) = 0;
    virtual VehicleStateType getVehicleState() const = 0;

    virtual bool getLandedState() const = 0;

    virtual const IStateEstimator& getStateEstimator() = 0;
    virtual GeoPoint getHomeGeoPoint() const = 0;
    virtual GeoPoint getGeoPoint() const = 0;
};

} //namespace

