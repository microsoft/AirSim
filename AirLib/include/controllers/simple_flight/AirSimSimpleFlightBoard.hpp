// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_AirSimSimpleFlightBoard_hpp
#define msr_airlib_AirSimSimpleFlightBoard_hpp

#include <exception>
#include <vector>
#include "firmware/Board.hpp"
#include "firmware/Params.hpp"
#include "common/Common.hpp"
#include "common/ClockFactory.hpp"
#include "physics/Kinematics.hpp"


namespace msr { namespace airlib {

class AirSimSimpleFlightBoard : public simple_flight::Board {
public:
    AirSimSimpleFlightBoard(const simple_flight::Params* params)
        : params_(params)
    {
    }

    //interface for simulator --------------------------------------------------------------------------------
    //for now we don't do any state estimation and use ground truth (i.e. assume perfect sensors)
    void setKinematics(const Kinematics::State* kinematics)
    {
        kinematics_ = kinematics;
    }

    //called to get o/p motor signal as float value
    real_T getMotorControlSignal(uint index) const
    {
        //convert PWM to scalled 0 to 1 control signal
        return (motors_pwm_[index] - params_->min_pwm) / static_cast<float>(params_->max_pwm - params_->min_pwm);
    }

    //set current RC stick status
    void setInputChannel(uint channel, uint16_t val)
    {
        input_channels_[channel] = val;
    }

public:
    //Board interface implementation --------------------------------------------------------------------------

    virtual uint64_t micros() const override 
    {
        return clock()->nowNanos() / 1000;
    }

    virtual uint64_t millis() const override 
    {
        return clock()->nowNanos() / 1000000;
    }

    virtual uint16_t readChannel(int16_t channel) const override 
    {
        return static_cast<uint16_t>(input_channels_[channel]);
    }

    virtual void writeOutput(uint8_t index, uint16_t value) override 
    {
        if (index < params_->motor_count)
            motors_pwm_[index] = value;
        else
            throw std::runtime_error("cannot write motor output for index > motor count");
    }

    virtual void setLed(uint8_t index, int32_t color) override 
    {
        //TODO: implement this
        unused(index);
        unused(color);
    }

    virtual void readAccel(float accel[3]) const override 
    {
        const auto& linear_accel = kinematics_->accelerations.linear;
        accel[0] = linear_accel.x();
        accel[1] = linear_accel.y();
        accel[2] = linear_accel.z();
    }

    virtual void readGyro(float gyro[3]) const override 
    {
        const auto& angula_vel = kinematics_->twist.angular;
        gyro[0] = angula_vel.x();
        gyro[1] = angula_vel.y();
        gyro[2] = angula_vel.z();
    }

    virtual void delayMicros(uint32_t us) override 
    {
        sleep(us * 1E3f);
    }

    virtual void delayMillis(uint32_t ms) override 
    {
        sleep(static_cast<float>(ms));
    }

    virtual void reset() override 
    {
        motors_pwm_.assign(params_->motor_count, 1000);
        input_channels_.assign(params_->rc_channel_count, 1000);
    }

    virtual void update() override
    {
        //no op for now
    }

private:
    void sleep(double msec)
    {
        clock()->sleep_for(msec * 1000.0);
    }

    const ClockBase* clock() const
    {
        return ClockFactory::get();
    }

    ClockBase* clock()
    {
        return ClockFactory::get();
    }

private:
    //motor outputs
    std::vector<uint16_t> motors_pwm_;
    std::vector<uint16_t> input_channels_;

    const simple_flight::Params* params_;
    const Kinematics::State* kinematics_;
};

}} //namespace
#endif
