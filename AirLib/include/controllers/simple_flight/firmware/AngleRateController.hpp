
#pragma once

#include "interfaces/CommonStructs.hpp"
#include "interfaces/IBoardClock.hpp"
#include "interfaces/IAxisController.hpp"
#include "Params.hpp"
#include "PidController.hpp"
#include "common/common_utils/Utils.hpp"
#include <memory>

namespace simple_flight {

class AngleRateController : public IAxisController {
public:
    AngleRateController(const Params* params, const IBoardClock* clock)
        : params_(params), clock_(clock)
    {
    }

    virtual void initialize(unsigned int axis, const IGoalInput* goal_input, const IStateEstimator* state_estimator)
    {
        axis_ = axis;
        goal_input_ = goal_input;
        state_estimator_ = state_estimator;

        pid_.reset(new PidController<float>(clock_,
            PidController<float>::Config(params_->pid_p_angle_rate[axis], 0, 0)));

        AngleRateController::reset();
    }

    virtual void reset() override
    {
        pid_->reset();
    }

    virtual void update() override
    {
        pid_->setGoal(goal_input_->getGoal().axis3[axis_]);
        pid_->setMeasured(state_estimator_->getAngulerVelocity()[axis_]);
        pid_->update();

        output_ = pid_->getOutput();
    }

    virtual TReal getOutput() override
    {
        return output_;
    }

private:
    unsigned int axis_;
    const IGoalInput* goal_input_;
    const IStateEstimator* state_estimator_;

    TReal output_;

    const Params* params_;
    const IBoardClock* clock_;
    std::unique_ptr<PidController<float>> pid_;
};


} //namespace