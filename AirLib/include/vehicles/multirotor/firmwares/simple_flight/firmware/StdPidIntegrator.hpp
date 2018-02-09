#pragma once

#include "interfaces/CommonStructs.hpp"

namespace simple_flight {

template<typename T>
class StdPidIntegrator : public IPidIntegrator<T>  {
public:
    StdPidIntegrator(const PidConfig<T>& config)
        : config_(config)
    {
    }

    virtual void reset() override
    {
        iterm_int_ = T();
    }

    virtual void set(T val) override
    {
        iterm_int_ = val;
        clipIterm();
    }

    virtual void update(float dt, T error, uint64_t last_time) override
    {
        unused(last_time);

        //to supoort changes in ki at runtime, we accumulate iterm
        //instead of error
        iterm_int_ = iterm_int_ * config_.iterm_discount + dt * error * config_.ki;

        //don't let iterm grow beyond limits (integral windup)
        clipIterm();
    }

    virtual T getOutput() override
    {
        return iterm_int_;
    }

private:
    void clipIterm()
    {
        iterm_int_ = clip(iterm_int_, config_.min_output, config_.max_output);
    }

    //TODO: replace with std::clamp after moving to C++17
    static T clip(T val, T min_value, T max_value) 
    {
        return std::max(min_value, std::min(val, max_value));
    }

private:
    float iterm_int_;
    const PidConfig<T> config_;
};

} //namespace