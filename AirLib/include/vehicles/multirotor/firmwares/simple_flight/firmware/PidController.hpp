#pragma once

#include <cstdlib>
#include <algorithm>
#include "interfaces/CommonStructs.hpp"

const int length = 1;
float y_vec [length] = {};
float* y = y_vec;
float yp_vec[length] = {};
float* yp = yp_vec;
float error_int = 0;

namespace simple_flight {

template<class T>
class PidController : public IUpdatable {
public:
    //config params for PID controller
    struct Config {
        Config(float kp_val = 0.01f, float ki_val = 0.0f, float kd_val = 0.0f,
            T min_output_val = -1, T max_output_val = 1,
            float time_scale_val = 1.0f / 1000,
            bool enabled_val = true, T output_bias_val = T(), float iterm_discount_val = 1)
            : kp(kp_val), ki(ki_val), kd(kd_val),
              time_scale(time_scale_val),
              min_output(min_output_val), max_output(max_output_val),
              enabled(enabled_val), output_bias(output_bias_val), iterm_discount(iterm_discount_val)
        {}

        float kp, ki, kd;
        float time_scale;
        T min_output, max_output;
        bool enabled;
        T output_bias;
        float iterm_discount;
		
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

    virtual void reset() override
    {
        IUpdatable::reset();

        goal_ = T();
        measured_ = T();
        last_time_ = clock_ == nullptr ? 0 : clock_->millis();
        iterm_int_ = 0;
        last_goal_ = goal_;
        min_dt_ = config_.time_scale * config_.time_scale;
    }

	

    virtual void update() override
    {
        IUpdatable::update();

        if (!config_.enabled)
            return;

        const T error = goal_ - measured_;
		error_int = error;
		y[0] = error_int;
        float dt = clock_ == nullptr ? 1 :
            (static_cast<float>(clock_->millis()) - last_time_)
            * config_.time_scale;

        float pterm = error * config_.kp;
        float dterm = 0;
        if (dt > min_dt_) {
			rungeKutta(y,yp,last_time_,0.003,length);

            //don't let iterm grow beyond limits (integral windup)
            clipIterm(yp);

            //To eliminate "derivative kick", we assume goal was approximately
            //constant between successive calls. dE = dGoal - dInput = -dInput
            float error_der = - (goal_ - last_goal_) / dt;
            dterm = error_der * config_.kd;
            last_goal_ = goal_;
        }

        output_ = config_.output_bias + pterm + config_.ki * yp[0] + dterm;

        //limit final output
        output_ = clip(output_, config_.min_output, config_.max_output);

        last_time_ = clock_->millis();
    }

void model(float* y, uint64_t last_time_, float* y_out)
{
	y_out[0] = y[0];
}

void rungeKutta(float* y, float* yp, uint64_t time, float dt,int size)
{
	float k1, k2, k3, k4;
	float zero_vec[length] = { 0 };
	float* y_temp;
	float* y_out = zero_vec;
	for (int n = 0; n < size; n++)
	{
		y_temp = y;
		model(y_temp, time, y_out);
		k1 = dt*y_out[n];

		y_temp[n] = y[n] + k1 / 2;
		model(y_temp, time + dt / 2, y_out);
		k2 = dt*y_out[n];

		y_temp[n] = y[n] + k2 / 2;
		model(y_temp, time + dt / 2, y_out);
		k3 = dt*y_out[n];

		y_temp[n] = y[n] + k3;
		model(y_temp, time + dt, y_out);
		k4 = dt*y_out[n];

		yp[n] = y[n] + k1 / 6 + k2 / 3 + k3 / 4 + k4 / 6;
	}
}
private:
    static T clip(T val, T min_value, T max_value) 
    {
        return std::max(min_value, std::min(val, max_value));
    }

    void clipIterm(float* yp)
    {
        yp[0] = clip(yp[0], config_.min_output, config_.max_output);
    }

private:
    T goal_, measured_;
    T output_;
    uint64_t last_time_;
    const IBoardClock* clock_;
    float iterm_int_;
    float last_goal_;
    float min_dt_;
    Config config_;
};


} //namespace

