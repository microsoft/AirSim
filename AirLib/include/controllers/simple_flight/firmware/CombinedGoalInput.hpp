#pragma once

#include <cstdint>
#include "interfaces/ICommLink.hpp"
#include "interfaces/IGoalInput.hpp"
#include "interfaces/CommonStructs.hpp"
#include "RemoteControl.hpp"
#include "Params.hpp"

namespace simple_flight {

class CombinedGoalInput : public IGoalInput {
public:
    CombinedGoalInput(const Params* params, ICommLink* comm_link, RemoteControl* rc)
        : params_(params), comm_link_(comm_link), rc_(rc)
    {
    }

    virtual void reset() override
    {
        IGoalInput::reset();

        has_api_control_ = false;
    }

    virtual void update() override
    {
        IGoalInput::update();
    }

    virtual const Axis4r& getGoal() const override
    {
        return goal_;
    }

    virtual const GoalMode& getGoalMode() const override
    {
        return goal_mode_;
    }


    bool canRequestApiControl(std::string& message)
    {
        if (rc_->allowApiControl)
            return true;
        else {
            message = "RemoteControl switch position disallows API control";
            return false;
        }
    }
    bool hasApiControl()
    {
        return has_api_control_;
    }
    bool requestApiControl(std::string& message)
    {
        if (canRequestApiControl(message)) {
            has_api_control_ = true;

            //initial value from RC for smooth transition
            goal_ = rc_->getGoal();
            goal_mode_ = rc_->getGoalMode();

            return true;
        }
        return false;
    }
    void releaseApiControl()
    {
        has_api_control_ = false;
    }
    bool setGoalAndMode(const Axis4r* goal, const GoalMode* goal_mode, std::string& message)
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
    const Params* params_;
    ICommLink* comm_link_;

    RemoteControl* rc_;

    Axis4r goal_;
    GoalMode goal_mode_;

    bool has_api_control_;
};


} //namespace