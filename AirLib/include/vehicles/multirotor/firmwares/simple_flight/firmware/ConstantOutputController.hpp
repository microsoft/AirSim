
#pragma once

#include "interfaces/CommonStructs.hpp"
#include "interfaces/IBoardClock.hpp"
#include "interfaces/IAxisController.hpp"
#include "common/common_utils//Utils.hpp"
#include "Params.hpp"
#include <memory>

namespace simple_flight {

class ConstantOutputController : public IAxisController {
public:
    ConstantOutputController(TReal update_output = TReal(), TReal reset_output = TReal())
        : update_output_(update_output), reset_output_(reset_output)
    {
    }

    virtual void initialize(unsigned int axis, const IGoal* goal, const IStateEstimator* state_estimator) override
    {
        axis_ = axis;
        unused(goal);
        unused(state_estimator);
    }

    virtual void reset() override
    {
        IAxisController::reset();
        output_ = reset_output_;
    }

    virtual void update() override
    {
        IAxisController::update();
        output_ = update_output_;
    }

    virtual TReal getOutput() override
    {
        return output_;
    }

private:
    unsigned int axis_;
    TReal update_output_;
    TReal reset_output_;
    TReal output_;
};


} //namespace