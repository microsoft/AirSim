
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

    virtual void initialize(unsigned int axis, const IGoal* goal, const IStateEstimator* state_estimator) override
    {
        axis_ = axis;
        goal_ = goal;
        state_estimator_ = state_estimator;

        pid_.reset(new PidController<float>(clock_,
            PidController<float>::Config(params_->angle_rate_pid.p[axis], 0, 0)));
    }

    virtual void reset() override
    {
        IAxisController::reset();

        pid_->reset();
        output_ = TReal();
    }

    virtual void update() override
    {
        IAxisController::update();

        pid_->setGoal(goal_->getGoalValue().axis3[axis_]);
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
    const IGoal* goal_;
    const IStateEstimator* state_estimator_;

    TReal output_;

    const Params* params_;
    const IBoardClock* clock_;
    std::unique_ptr<PidController<float>> pid_;
};


} //namespace