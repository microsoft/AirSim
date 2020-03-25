
#pragma once

#include "interfaces/IUpdatable.hpp"
#include "interfaces/IGoal.hpp"
#include "interfaces/IBoardClock.hpp"
#include "interfaces/CommonStructs.hpp"
#include "interfaces/IAxisController.hpp"
#include "VelocityController.hpp"
#include "Params.hpp"
#include "PidController.hpp"
#include "common/common_utils/Utils.hpp"

namespace simple_flight {

class PositionController : 
    public IAxisController,
    public IGoal  //for internal child controller
{
public:
    PositionController(Params* params, const IBoardClock* clock = nullptr)
        : params_(params), clock_(clock)
    {
    }

    virtual void initialize(unsigned int axis, const IGoal* goal, const IStateEstimator* state_estimator) override
    {
        if (axis == 2)
            throw std::invalid_argument("PositionController does not support yaw axis i.e. " + std::to_string(axis));

        axis_ = axis;
        goal_ = goal;
        state_estimator_ = state_estimator;

        //initialize parent PID
        pid_.reset(new PidController<float>(clock_,
            PidConfig<float>(params_->position_pid.p[axis], params_->position_pid.i[axis], params_->position_pid.d[axis])));

        //initialize child controller
        velocity_controller_.reset(new VelocityController(params_, clock_));
        velocity_controller_->initialize(axis, this, state_estimator_);

        //we will be setting goal for child controller so we need these two things
        velocity_mode_  = GoalMode::getUnknown();
        velocity_mode_[axis] = GoalModeType::VelocityWorld;
    }

    virtual void reset() override
    {
        IAxisController::reset();

        pid_->reset();
        velocity_controller_->reset();
        velocity_goal_ = Axis4r();
        output_ = TReal();
    }

    virtual void update() override
    {
        IAxisController::update();

        const Axis4r& goal_position_world = goal_->getGoalValue();
        pid_->setGoal(goal_position_world[axis_]);
        const Axis4r& measured_position_world = Axis4r::xyzToAxis4(
            state_estimator_->getPosition(), true);
        pid_->setMeasured(measured_position_world[axis_]);
        pid_->update();

        //use this to drive child controller
        velocity_goal_[axis_] = pid_->getOutput() * params_->velocity_pid.max_limit[axis_];
        velocity_controller_->update();

        //final output
        output_ = velocity_controller_->getOutput();
    }

    virtual TReal getOutput() override
    {
        return output_;
    }

    /********************  IGoal ********************/
    virtual const Axis4r& getGoalValue() const override
    {
        return velocity_goal_;
    }

    virtual const GoalMode& getGoalMode() const  override
    {
        return velocity_mode_;
    }

private:
    unsigned int axis_;
    const IGoal* goal_;
    const IStateEstimator* state_estimator_;

    GoalMode velocity_mode_;
    Axis4r velocity_goal_;

    TReal output_;

    Params* params_;
    const IBoardClock* clock_;
    std::unique_ptr<PidController<float>> pid_;
    std::unique_ptr<VelocityController> velocity_controller_;

};


} //namespace