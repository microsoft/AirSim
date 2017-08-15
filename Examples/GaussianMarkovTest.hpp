#pragma once

#include "common/GaussianMarkov.hpp"
#include "common/SteppableClock.hpp"

namespace msr { namespace airlib {

class GaussianMarkovTest {
public:
    GaussianMarkovTest()
        : pressure_factor_(0.1f, 5.0f)
    {
        clock_ = std::make_shared<SteppableClock>(0.02f);
        ClockFactory::get(clock_);
        pressure_factor_.reset();
    }

    void run()
    {
        for (int i = 0; i < 100; ++i) {
            clock_->step();
            pressure_factor_.update();
            std::cout << pressure_factor_.getOutput() << std::endl;
        }
    }

private:
    GaussianMarkov pressure_factor_;
    std::shared_ptr<SteppableClock> clock_;
};

}}