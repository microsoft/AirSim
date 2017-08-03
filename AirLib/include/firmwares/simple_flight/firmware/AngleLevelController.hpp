
#pragma once

#include "interfaces/IUpdatable.hpp"
#include "interfaces/IGoalInput.hpp"
#include "interfaces/IBoardClock.hpp"
#include "interfaces/CommonStructs.hpp"
#include "interfaces/IAxisController.hpp"
#include "AngleRateController.hpp"
#include "Params.hpp"
#include "PidController.hpp"
#include "common/common_utils/Utils.hpp"

namespace simple_flight {

class AngleLevelController : 
    public IAxisController,
    public IGoalInput  //for internal rate controller
{
public:
    AngleLevelController(const Params* params, const IBoardClock* clock = nullptr)
        : params_(params), clock_(clock)
    {
    }

    virtual void initialize(unsigned int axis, const IGoalInput* goal_input, const IStateEstimator* state_estimator) override
    {
        axis_ = axis;
        goal_input_ = goal_input;
        state_estimator_ = state_estimator;

        //initialize level PID
        pid_.reset(new PidController<float>(clock_,
            PidController<float>::Config(params_->angle_level_pid.p[axis], 0, 0)));

        //initialize rate controller
        rate_controller_.reset(new AngleRateController(params_, clock_));
        rate_controller_->initialize(axis, this, state_estimator_);

        //we will be setting goal for rate controller so we need these two things
        rate_mode_  = GoalMode::getUnknown();
        rate_mode_[axis] = GoalModeType::AngleRate;
    }

    virtual void reset() override
    {
        IAxisController::reset();
        IGoalInput::reset();

        pid_->reset();
        rate_controller_->reset();
        rate_goal_ = Axis4r();
        output_ = TReal();
    }

    virtual void update() override
    {
        IAxisController::update();
        IGoalInput::update();

        //get response of level PID
        const auto& level_goal = goal_input_->getGoal();
        pid_->setGoal(level_goal.axis3[axis_]);
        pid_->setMeasured(state_estimator_->getAngles()[axis_]);
        pid_->update();

        //use this to drive rate controller
        rate_goal_.throttle = level_goal.throttle;
        rate_goal_.axis3[axis_] = pid_->getOutput() * params_->angle_rate_pid.max_limit[axis_];
        rate_controller_->update();

        //rate controller's output is final output
        output_ = rate_controller_->getOutput();
    }

    virtual TReal getOutput() override
    {
        return output_;
    }

    /********************  IGoalInput ********************/
    virtual const Axis4r& getGoal() const override
    {
        return rate_goal_;
    }

    virtual const GoalMode& getGoalMode() const  override
    {
        return rate_mode_;
    }

private:
    unsigned int axis_;
    const IGoalInput* goal_input_;
    const IStateEstimator* state_estimator_;

    GoalMode rate_mode_;
    Axis4r rate_goal_;

    TReal output_;

    const Params* params_;
    const IBoardClock* clock_;
    std::unique_ptr<PidController<float>> pid_;
    std::unique_ptr<AngleRateController> rate_controller_;

};


} //namespace