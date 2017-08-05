
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

        //initialize parent PID
        pid_.reset(new PidController<float>(clock_,
            PidController<float>::Config(params_->velocity_pid.p[axis], 0, 0)));

        //we will be setting goal for child controller so we need these two things
        child_mode_  = GoalMode::getUnknown();
        switch (axis_) {
        case 0:
            child_controller_.reset(new AngleLevelController(params_, clock_));
            child_mode_.pitch() = GoalModeType::AngleLevel; //vx = - pitch
            break;
        case 1:
            child_controller_.reset(new AngleLevelController(params_, clock_));
            child_mode_.roll() = GoalModeType::AngleLevel; //vx = roll
            break;
        case 2:
            //we control yaw
            throw std::invalid_argument("axis must be 0, 1 or 3 but it was " + std::to_string(axis_) + " because yaw cannot be controlled by VelocityController");
            break;
        case 3:
            child_controller_.reset(new PassthroughController());
            child_mode_.throttle() = GoalModeType::Passthrough; //vx = roll
            break;
        default:
            throw std::invalid_argument("axis must be 0 to 2");
        }

        //initialize child controller
        child_controller_->initialize(axis, this, state_estimator_);
    }

    virtual void reset() override
    {
        IAxisController::reset();

        pid_->reset();
        child_controller_->reset();
        child_goal_ = Axis4r();
        output_ = TReal();
    }

    virtual void update() override
    {
        IAxisController::update();

        //First get PID output
        const Axis4r& goal_velocity_world = goal_->getGoalValue();
        const Axis3r& goal_velocity_local = state_estimator_->transformToBodyFrame(goal_velocity_world);
        pid_->setGoal(goal_velocity_local[axis_]);

        const Axis3r& measured_velocity_world = state_estimator_->getLinearVelocity();
        const Axis3r& measured_velocity_local = state_estimator_->transformToBodyFrame(measured_velocity_world);
        pid_->setMeasured(measured_velocity_local[axis_]);
        pid_->update();

        //use this to drive child controller
        switch (axis_)
        {
        case 0: //+vx is -ve pitch
            child_goal_.pitch() = -pid_->getOutput() * params_->angle_level_pid.max_limit.pitch();
            child_controller_->update();
            output_ = child_controller_->getOutput();
            break;
        case 1: //+vy is +ve roll
            child_goal_.roll() = pid_->getOutput() * params_->angle_level_pid.max_limit.roll();
            output_ = child_controller_->getOutput();
            break;
        case 3: //+vz is -ve throttle (NED coordinates)
            output_ = (-pid_->getOutput() + 1) / 2; //-1 to 1 --> 1 to 0
            output_ = std::max(output_, params_->velocity_pid.min_throttle);
            break;
        default:
            throw std::invalid_argument("axis must be 0, 1 or 3 for VelocityController");
        }
    }

    virtual TReal getOutput() override
    {
        return output_;
    }

    /********************  IGoal ********************/
    virtual const Axis4r& getGoalValue() const override
    {
        return child_goal_;
    }

    virtual const GoalMode& getGoalMode() const  override
    {
        return child_mode_;
    }

private:
    unsigned int axis_;
    const IGoal* goal_;
    const IStateEstimator* state_estimator_;

    GoalMode child_mode_;
    Axis4r child_goal_;

    TReal output_;

    const Params* params_;
    const IBoardClock* clock_;
    std::unique_ptr<PidController<float>> pid_;
    std::unique_ptr<IAxisController> child_controller_;

};


} //namespace