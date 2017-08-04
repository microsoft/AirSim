#pragma once

#include <string>
#include <exception>
#include "interfaces/IController.hpp"
#include "interfaces/IStateEstimator.hpp"
#include "interfaces/ICommLink.hpp"
#include "interfaces/IGoal.hpp"
#include "AngleRateController.hpp"
#include "AngleLevelController.hpp"

namespace simple_flight {

class CascadeController : public IController {
public:
    CascadeController(const Params* params, const IBoardClock* clock, ICommLink* comm_link)
        : params_(params), clock_(clock), comm_link_(comm_link)
    {
    }

    virtual void initialize(const IGoal* goal, const IStateEstimator* state_estimator) override
    {
        goal_ = goal;
        state_estimator_ = state_estimator;
    }

    virtual void reset() override
    {
        IController::reset();

        last_goal_mode_ = GoalMode::getUnknown();
        output_ = Axis4r();

        for (unsigned int axis = 0; axis < 3; ++axis) {
            if (axis_controllers_[axis] != nullptr)
                axis_controllers_[axis]->reset();
        }

    }

    virtual void update() override
    {
        IController::update();

        const auto& goal_mode = goal_->getGoalMode();

        //for now we set throttle to same as goal
        output_.throttle = goal_->getGoalValue().throttle;

        for (unsigned int axis = 0; axis < 3; ++axis) {
            //re-create axis controllers if goal mode was changed since last time
            if (goal_mode[axis] != last_goal_mode_[axis]) {
                switch (goal_mode[axis]) {
                case GoalModeType::AngleRate:
                    axis_controllers_[axis].reset(new AngleRateController(params_, clock_));
                    break;
                case GoalModeType::AngleLevel:
                    axis_controllers_[axis].reset(new AngleLevelController(params_, clock_));
                    break;
                default:
                    throw std::invalid_argument("Axis controller type is not yet implemented for axis " 
                        + std::to_string(axis));
                }

                //initialize axis controller
                axis_controllers_[axis]->initialize(axis, goal_, state_estimator_);
                axis_controllers_[axis]->reset();
            }

            //update axis controller
            if (axis_controllers_[axis] != nullptr) {
                axis_controllers_[axis]->update();
                output_.axis3[axis] = axis_controllers_[axis]->getOutput();
            }
            else
                comm_link_->log(std::string("Axis controller type is not set for axis ").append(std::to_string(axis)), ICommLink::kLogLevelError);
        }
    }

    virtual const Axis4r& getOutput() override
    {
        return output_;
    }


private:
    const Params* params_;
    const IBoardClock* clock_;

    const IGoal* goal_;
    const IStateEstimator* state_estimator_;
    ICommLink* comm_link_;

    Axis4r output_;

    GoalMode last_goal_mode_;

    std::unique_ptr<IAxisController> axis_controllers_[kAxisCount];
};

}