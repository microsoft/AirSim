
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
    VelocityController(Params* params, const IBoardClock* clock = nullptr)
        : params_(params), clock_(clock)
    {
    }

    virtual void initialize(unsigned int axis, const IGoal* goal, const IStateEstimator* state_estimator) override
    {
        axis_ = axis;
        goal_ = goal;
        state_estimator_ = state_estimator;

        PidConfig<float> pid_config(params_->velocity_pid.p[axis],
            params_->velocity_pid.i[axis], params_->velocity_pid.d[axis]);
        pid_config.iterm_discount = params_->velocity_pid.iterm_discount[axis];
        pid_config.output_bias = params_->velocity_pid.output_bias[axis];

        pid_.reset(new PidController<float>(clock_, pid_config));

        //we will be setting goal for child controller so we need these two things
        child_mode_  = GoalMode::getUnknown();
        switch (axis_) {
        case 0:
            child_controller_.reset(new AngleLevelController(params_, clock_));
            child_mode_[axis_] = GoalModeType::AngleLevel; //vy = roll
            break;
        case 1:
            child_controller_.reset(new AngleLevelController(params_, clock_));
            child_mode_[axis_] = GoalModeType::AngleLevel; //vx = - pitch
            break;
        case 2:
            //we control yaw
            throw std::invalid_argument("axis must be 0, 1 or 3 but it was " + std::to_string(axis_) + " because yaw cannot be controlled by VelocityController");
        case 3:
            //not really required
            //output of parent controller is -1 to 1 which
            //we will transform to 0 to 1
            child_controller_.reset(new PassthroughController());
            child_mode_[axis_] = GoalModeType::Passthrough;
            break;
        default:
            throw std::invalid_argument("axis must be 0 to 2");
        }

        //initialize child controller
        child_controller_->initialize(axis_, this, state_estimator_);
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
        const Axis3r& goal_velocity_world = Axis4r::axis4ToXyz(
            goal_->getGoalValue(), true);
        const Axis4r& goal_velocity_local = Axis4r::xyzToAxis4(
            state_estimator_->transformToBodyFrame(goal_velocity_world), true);
        pid_->setGoal(goal_velocity_local[axis_]);

        const Axis3r& measured_velocity_world = state_estimator_->getLinearVelocity();
        const Axis4r& measured_velocity_local = Axis4r::xyzToAxis4(
            state_estimator_->transformToBodyFrame(measured_velocity_world), true);
        pid_->setMeasured(measured_velocity_local[axis_]);
        pid_->update();

        //use this to drive child controller
        switch (axis_)
        {
        case 0: //+vy is +ve roll
            child_goal_[axis_] = pid_->getOutput() * params_->angle_level_pid.max_limit[axis_];
            child_controller_->update();
            output_ = child_controller_->getOutput();

            //if (std::abs(goal_velocity_local[axis_] - measured_velocity_local[axis_]) > 1)
            //    msr::airlib::Utils::log(msr::airlib::Utils::stringf("VC: %i\t%f\t%f\t%f",
            //        axis_, goal_velocity_local[axis_],
            //        measured_velocity_local[axis_], output_));

            break;
        case 1: //+vx is -ve pitch
            child_goal_[axis_] = - pid_->getOutput() * params_->angle_level_pid.max_limit[axis_];
            child_controller_->update();
            output_ = child_controller_->getOutput();
            break;
        case 3: //+vz is -ve throttle (NED coordinates)
            output_ = (-pid_->getOutput() + 1) / 2; //-1 to 1 --> 0 to 1
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

    Params* params_;
    const IBoardClock* clock_;
    std::unique_ptr<PidController<float>> pid_;
    std::unique_ptr<IAxisController> child_controller_;

};


} //namespace