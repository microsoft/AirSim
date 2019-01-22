#pragma once

#include "interfaces/CommonStructs.hpp"

namespace simple_flight {

template<typename T>
class RungKuttaPidIntegrator : public IPidIntegrator<T>  {
public:
    RungKuttaPidIntegrator(const PidConfig<T>& config)
        : config_(config)
    {
    }

    virtual ~RungKuttaPidIntegrator() {}

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
        unused(dt);
        
        error_int = error;
        y[0] = error_int;

        rungeKutta(y,yp,static_cast<float>(last_time),0.003f,length);

        //don't let iterm grow beyond limits (integral windup)
        clipIterm();
    }

    virtual T getOutput() override
    {
        return config_.ki * yp[0];
    }

private:
    void clipIterm()
    {
        yp[0] = clip(yp[0], config_.min_output, config_.max_output);
    }

    //TODO: replace with std::clamp after moving to C++17
    static T clip(T val, T min_value, T max_value) 
    {
        return std::max(min_value, std::min(val, max_value));
    }

    void model(float* y_val, float last_time_, float* y_out)
    {
        unused(last_time_);
        y_out[0] = y_val[0];
    }

    void rungeKutta(float* y_val, float* yp_val, float time, float dt,int size)
    {
        float k1, k2, k3, k4;
        float zero_vec[length] = { 0 };
        float* y_temp;
        float* y_out = zero_vec;
        for (int n = 0; n < size; n++)
        {
            y_temp = y_val;
            model(y_temp, time, y_out);
            k1 = dt*y_out[n];

            y_temp[n] = y_val[n] + k1 / 2;
            model(y_temp, time + dt / 2, y_out);
            k2 = dt*y_out[n];

            y_temp[n] = y_val[n] + k2 / 2;
            model(y_temp, time + dt / 2, y_out);
            k3 = dt*y_out[n];

            y_temp[n] = y_val[n] + k3;
            model(y_temp, time + dt, y_out);
            k4 = dt*y_out[n];

            yp_val[n] = y_val[n] + k1 / 6 + k2 / 3 + k3 / 4 + k4 / 6;
        }
    }

private:
    float iterm_int_;
    const PidConfig<T> config_;

    static constexpr int length = 1;
    float y_vec [length] = {};
    float* y = y_vec;
    float yp_vec[length] = {};
    float* yp = yp_vec;
    float error_int = 0;
};

} //namespace