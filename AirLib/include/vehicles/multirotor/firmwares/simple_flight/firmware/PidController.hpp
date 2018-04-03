#pragma once

#include <cstdlib>
#include <algorithm>
#include "interfaces/CommonStructs.hpp"
#include "interfaces/IPidIntegrator.hpp"
#include "StdPidIntegrator.hpp"
#include "RungKuttaPidIntegrator.hpp"

namespace simple_flight {

template<class T>
class PidController : public IUpdatable {
public:

    PidController(const IBoardClock* clock = nullptr, const PidConfig<T>& config = PidConfig<T>())
        : clock_(clock), config_(config)
    {
        switch (config.integrator_type) {
        case PidConfig<T>::IntegratorType::Standard:
            integrator = std::unique_ptr<StdPidIntegrator<T>>(new StdPidIntegrator<T>(config_));
            break;
        case PidConfig<T>::IntegratorType::RungKutta:
            integrator = std::unique_ptr<RungKuttaPidIntegrator<T>>(new RungKuttaPidIntegrator<T>(config_));
            break;
        default:
            throw std::invalid_argument("PID integrator type is not recognized");
        }
        
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

    const PidConfig<T>& getConfig() const
    {
        return config_;
    }

    //allow changing config at runtime
    void setConfig(const PidConfig<T>& config)
    {
        bool renabled = !config_.enabled && config.enabled;
        config_ = config;

        if (renabled) {
            last_goal_ = goal_;
            integrator->set(output_);
        }
    }

    T getOutput()
    {
        return output_;
    }

    virtual void reset() override
    {
        IUpdatable::reset();

        goal_ = T();
        measured_ = T();
        last_time_ = clock_ == nullptr ? 0 : clock_->millis();
        integrator->reset();
        last_goal_ = goal_;
        min_dt_ = config_.time_scale * config_.time_scale;
    }

    virtual void update() override
    {
        IUpdatable::update();

        if (!config_.enabled)
            return;

        const T error = goal_ - measured_;

        float dt = clock_ == nullptr ? 1 :
            (clock_->millis() - last_time_)
            * config_.time_scale;

        float pterm = error * config_.kp;
        float dterm = 0;
        if (dt > min_dt_) {
            integrator->update(dt, error, last_time_);

            //To eliminate "derivative kick", we assume goal was approximately
            //constant between successive calls. dE = dGoal - dInput = -dInput
            float error_der = - (goal_ - last_goal_) / dt;
            dterm = error_der * config_.kd;
            last_goal_ = goal_;
        }

        output_ = config_.output_bias + pterm + integrator->getOutput() + dterm;

        //limit final output
        output_ = clip(output_, config_.min_output, config_.max_output);

        last_time_ = clock_->millis();
    }

private:
    //TODO: replace with std::clamp after moving to C++17
    static T clip(T val, T min_value, T max_value) 
    {
        return std::max(min_value, std::min(val, max_value));
    }

private:
    T goal_, measured_;
    T output_;
    uint64_t last_time_;
    const IBoardClock* clock_;

    float last_goal_;
    float min_dt_;
    const PidConfig<T> config_;

    std::unique_ptr<IPidIntegrator<T>> integrator;
};


} //namespace
