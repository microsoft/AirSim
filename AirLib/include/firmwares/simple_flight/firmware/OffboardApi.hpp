#pragma once

#include <cstdint>
#include "interfaces/ICommLink.hpp"
#include "interfaces/IGoal.hpp"
#include "interfaces/IOffboardApi.hpp"
#include "interfaces/IUpdatable.hpp"
#include "interfaces/CommonStructs.hpp"
#include "RemoteControl.hpp"
#include "Params.hpp"

namespace simple_flight {

class OffboardApi : 
    public IUpdatable,
    public IOffboardApi {
public:
    OffboardApi(const Params* params, const IBoardClock* clock, const IBoardInputPins* board_inputs, 
        const IStateEstimator* state_estimator, ICommLink* comm_link)
        : params_(params), rc_(params, clock, board_inputs, &vehicle_state_, state_estimator, comm_link), 
          state_estimator_(state_estimator), comm_link_(comm_link)
    {
    }

    virtual void reset() override
    {
        IUpdatable::reset();

        vehicle_state_.setState(params_->default_vehicle_state, state_estimator_->getGeoPoint());
        has_api_control_ = false;
        rc_.reset();
        updateGoalFromRc();
    }

    virtual void update() override
    {
        IUpdatable::update();

        if (! has_api_control_) {
            rc_.update();
            updateGoalFromRc();
        }
        //else leave the goal set by IOffboardApi API
    }

    /**************** IOffboardApi ********************/

    virtual const Axis4r& getGoalValue() const override
    {
        return goal_;
    }

    virtual const GoalMode& getGoalMode() const override
    {
        return goal_mode_;
    }


    virtual bool canRequestApiControl(std::string& message) override
    {
        if (rc_.allowApiControl())
            return true;
        else {
            message = "RemoteControl switch position disallows API control";
            comm_link_->log(message, ICommLink::kLogLevelError);
            return false;
        }
    }
    virtual bool hasApiControl() override
    {
        return has_api_control_;
    }
    virtual bool requestApiControl(std::string& message) override
    {
        if (canRequestApiControl(message)) {
            has_api_control_ = true;

            //initial value from RC for smooth transition
            updateGoalFromRc();

            return true;
        }
        return false;
    }
    virtual void releaseApiControl() override
    {
        has_api_control_ = false;
    }
    virtual bool setGoalAndMode(const Axis4r* goal, const GoalMode* goal_mode, std::string& message) override
    {
        if (has_api_control_) {
            if (goal != nullptr)
                goal_ = *goal;
            if (goal_mode != nullptr)
                goal_mode_ = *goal_mode;
            return true;
        }
        else {
            message = "requestControl() must be called before using API control";
            comm_link_->log(message, ICommLink::kLogLevelError);
            return false;
        }

    }

    virtual bool arm(std::string& message, const Axis4r& goal, const GoalMode& goal_mode) override
    {
        if (has_api_control_ && (vehicle_state_.getState() == VehicleStateType::Inactive
            || vehicle_state_.getState() == VehicleStateType::Disarmed
            || vehicle_state_.getState() == VehicleStateType::BeingDisarmed)) {

            vehicle_state_.setState(VehicleStateType::Armed, state_estimator_->getGeoPoint());
            goal_ = goal;
            goal_mode_ = goal_mode;

            message = "Vehicle is armed";
            comm_link_->log(message, ICommLink::kLogLevelInfo);
            return true;
        }
        else {
            message = "Vehicle cannot be armed because it is not in Inactive, Disarmed or BeingDisarmed state";
            comm_link_->log(message, ICommLink::kLogLevelError);
            return false;
        }
    }

    virtual bool disarm(std::string& message, const Axis4r& goal, const GoalMode& goal_mode) override
    {
        if (has_api_control_ && (vehicle_state_.getState() == VehicleStateType::Active
            || vehicle_state_.getState() == VehicleStateType::Armed
            || vehicle_state_.getState() == VehicleStateType::BeingArmed)) {

            vehicle_state_.setState(VehicleStateType::Disarmed);
            goal_ = goal;
            goal_mode_ = goal_mode;

            message = "Vehicle is disarmed";
            comm_link_->log(message, ICommLink::kLogLevelInfo);
            return true;
        }
        else {
            message = "Vehicle cannot be disarmed because it is not in Active, Armed or BeingArmed state";
            comm_link_->log(message, ICommLink::kLogLevelError);
            return false;
        }
    }

    virtual VehicleStateType getVehicleState() const override
    {
        return vehicle_state_.getState();
    }

    virtual const IStateEstimator& getStateEstimator() override
    {
        return *state_estimator_;
    }

    virtual GeoPoint getHomePoint() const override
    {
        return vehicle_state_.getHomePoint();
    }
    
private:
    void updateGoalFromRc()
    {
        goal_ = rc_.getGoalValue();
        goal_mode_ = rc_.getGoalMode();
    }

private:
    const Params* params_;
    RemoteControl rc_;
    const IStateEstimator* state_estimator_;
    ICommLink* comm_link_;

    VehicleState vehicle_state_;
    
    Axis4r goal_;
    GoalMode goal_mode_;

    bool has_api_control_;
};


} //namespace