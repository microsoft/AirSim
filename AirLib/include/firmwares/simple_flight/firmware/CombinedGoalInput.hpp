#pragma once

#include <cstdint>
#include "interfaces/ICommLink.hpp"
#include "interfaces/IGoal.hpp"
#include "interfaces/IGoalInput.hpp"
#include "interfaces/IUpdatable.hpp"
#include "interfaces/CommonStructs.hpp"
#include "RemoteControl.hpp"
#include "Params.hpp"

namespace simple_flight {

class CombinedGoalInput : 
    public IUpdatable,
    public IGoalInput {
public:
    CombinedGoalInput(const Params* params, const IBoardClock* clock, const IBoardInputPins* board_inputs, ICommLink* comm_link)
        : params_(params), rc_(params, clock, board_inputs, comm_link), comm_link_(comm_link)
    {
    }

    virtual void reset() override
    {
        IUpdatable::reset();

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
        //else leave the goal set by IGoalInput API
    }

    /**************** IGoalInput ********************/

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
            return false;
        }

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
    ICommLink* comm_link_;

    Axis4r goal_;
    GoalMode goal_mode_;

    bool has_api_control_;
};


} //namespace