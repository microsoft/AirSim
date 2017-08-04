
#pragma once

#include "interfaces/IUpdatable.hpp"
#include "interfaces/IGoal.hpp"
#include "interfaces/IBoardClock.hpp"
#include "interfaces/CommonStructs.hpp"
#include "interfaces/IAxisController.hpp"
#include "AngleLevelController.hpp"
#include "Params.hpp"
#include "PidController.hpp"
#include "common/common_utils/Utils.hpp"

namespace simple_flight {

class VelocityController : 
    public IAxisController,
    public IGoal  //for internal child controller
{
public:
    VelocityController(const Params* params, const IBoardClock* clock = nullptr)
        : params_(params), clock_(clock)
    {
    }

    virtual void initialize(unsigned int axis, const IGoal* goal, const IStateEstimator* state_estimator) override
    {
        axis_ = axis;
        goal_ = goal;
        state_estimator_ = state_estimator;

        //initialize level PID
        pid_.reset(new PidController<float>(clock_,
            PidController<float>::Config(params_->velocity_pid.p[axis], 0, 0)));

        //initialize child controller
        level_controller_.reset(new AngleLevelController(params_, clock_));
        level_controller_->initialize(axis, this, state_estimator_);

        //we will be setting goal for child controller so we need these two things
        level_mode_  = GoalMode::getUnknown();
        level_mode_[axis] = GoalModeType::AngleLevel;
    }

    virtual void reset() override
    {
        IAxisController::reset();

        pid_->reset();
        level_controller_->reset();
        level_goal_ = Axis4r();
        output_ = TReal();
    }

    virtual void update() override
    {
        IAxisController::update();

        const Axis4r& goal_velocity_world = goal_->getGoalValue();
        const Axis3r& goal_velocity_local = state_estimator_->transformToBodyFrame(goal_velocity_world.axis3);
        pid_->setGoal(goal_velocity_local[axis_]);

        const Axis3r& measured_velocity_world = state_estimator_->getLinearVelocity();
        const Axis3r& measured_velocity_local = state_estimator_->transformToBodyFrame(measured_velocity_world);
        pid_->setMeasured(measured_velocity_local[axis_]);
        pid_->update();

        //use this to drive child controller
        level_goal_.throttle() = goal_velocity_world.throttle();
        level_goal_.axis3[axis_] = pid_->getOutput() * params_->angle_level_pid.max_limit[axis_];
        level_controller_->update();

        //final output
        output_ = level_controller_->getOutput();
    }

    virtual TReal getOutput() override
    {
        return output_;
    }

    /********************  IGoal ********************/
    virtual const Axis4r& getGoalValue() const override
    {
        return level_goal_;
    }

    virtual const GoalMode& getGoalMode() const  override
    {
        return level_mode_;
    }

private:
    unsigned int axis_;
    const IGoal* goal_;
    const IStateEstimator* state_estimator_;

    GoalMode level_mode_;
    Axis4r level_goal_;

    TReal output_;

    const Params* params_;
    const IBoardClock* clock_;
    std::unique_ptr<PidController<float>> pid_;
    std::unique_ptr<AngleLevelController> level_controller_;

};


} //namespace