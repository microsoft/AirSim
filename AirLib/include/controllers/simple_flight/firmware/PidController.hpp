#pragma once

#include <cstdlib>
#include <algorithm>
#include "interfaces/CommonStructs.hpp"

namespace simple_flight {

template<class T>
class PidController {
public:
    struct Config {
        Config(float kp_val = 0.01f, float ki_val = 0.0f, float kd_val = 0.0f,
            float min_output_val = -1.0f, float max_output_val = 1.0f,
            float time_scale_val = 1.0f / 1000,
            bool enabled_val = true)
            : kp(kp_val), ki(ki_val), kd(kd_val),
              time_scale(time_scale_val),
              min_output(min_output_val), max_output(max_output_val),
              enabled(enabled_val)
        {}

        float kp, ki, kd;
        float time_scale;
        float min_output, max_output;
        bool enabled;
    };

    PidController(const IBoardClock* clock = nullptr, const Config& config = Config())
        : clock_(clock), config_(config)
    {
    }

    void setGoal(const T& goal)
    {
        goal_ = goal;
    }
    const T& getGoal() const
    {
        return goal_;
    }

    void setMeasured(const T& measured)
    {
        measured_ = measured;
    }
    const T& getMeasured() const
    {
        return measured_;
    }

    const Config& getConfig() const
    {
        return config_;
    }

    //allow changing config at runtime
    void setConfig(const Config& config)
    {
        bool renabled = !config_.enabled && config.enabled;
        config_ = config;

        if (renabled) {
            last_goal_ = goal_;
            iterm_int_ = output_;
            clipIterm();
        }
    }

    T getOutput()
    {
        return output_;
    }

    void resetErrorIntegral()
    {
        iterm_int_ = T();
    }

    void reset()
    {
        goal_ = T();
        measured_ = T();
        last_time_ = clock_ == nullptr ? 0 : clock_->millis();
        iterm_int_ = 0;
        last_goal_ = goal_;
        min_dt_ = config_.time_scale * config_.time_scale;
    }

    void update()
    {
        if (!config_.enabled)
            return;

        const T error = goal_ - measured_;

        float dt = clock_ == nullptr ? 1 :
            (static_cast<float>(clock_->millis()) - last_time_)
                / config_.time_scale;

        float pterm = error * config_.kp;
        float dterm = 0;
        if (dt > min_dt_) {
            //to supoort changes in ki at runtime, we accumulate iterm
            //instead of error
            iterm_int_ += dt * error * config_.ki;

            //don't let iterm grow beyond limits (integral windup)
            clipIterm();

            //To eliminate "derivative kick", we assume goal was approximately
            //constant between successive calls. dE = dGoal - dInput = -dInput
            float error_der = - (goal_ - last_goal_) / dt;
            dterm = error_der * config_.kd;
            last_goal_ = goal_;
        }

        output_ = pterm + iterm_int_ + dterm;

        //limit final output
        output_ = clip(output_, config_.min_output, config_.max_output);

        last_time_ = clock_->millis();
    }

private:
    static T clip(T val, T min_value, T max_value) 
    {
        return std::max(min_value, std::min(val, max_value));
    }

    void clipIterm()
    {
        iterm_int_ = clip(iterm_int_, config_.min_output, config_.max_output);
    }

private:
    T goal_, measured_;
    T output_;
    const IBoardClock* clock_;
    Config config_;
    uint64_t last_time_;
    float min_dt_;
    float iterm_int_;
    float last_goal_;
};


} //namespace