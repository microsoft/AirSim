// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_AirSimSimpleFlightBoard_hpp
#define msr_airlib_AirSimSimpleFlightBoard_hpp

#include <exception>
#include <vector>
#include "firmware/interfaces/IBoard.hpp"
#include "firmware/Params.hpp"
#include "common/Common.hpp"
#include "common/ClockFactory.hpp"
#include "physics/Kinematics.hpp"


namespace msr { namespace airlib {

class AirSimSimpleFlightBoard : public simple_flight::IBoard {
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
        return static_cast<float>(motor_output_[index]);
    }

    //set current RC stick status
    void setInputChannel(uint index, real_T val)
    {
        input_channels_[index] = static_cast<float>(val);
    }

    void setIsRcConnected(bool is_connected)
    {
        is_connected_ = is_connected;
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

    virtual float readChannel(uint16_t index) const override 
    {
        return input_channels_[index];
    }

    virtual bool isRcConnected() const override
    {
        return is_connected_;
    }

    virtual void writeOutput(uint16_t index, float value) override 
    {
        motor_output_[index] = value;
    }

    virtual void setLed(uint8_t index, int32_t color) override 
    {
        //TODO: implement this
        unused(index);
        unused(color);
    }

    virtual void readAccel(float accel[3]) const override 
    {
        const auto& linear_accel = VectorMath::transformToBodyFrame(kinematics_->accelerations.linear, kinematics_->pose.orientation);
        accel[0] = linear_accel.x();
        accel[1] = linear_accel.y();
        accel[2] = linear_accel.z();
    }

    virtual void readGyro(float gyro[3]) const override 
    {
        const auto angular_vel = kinematics_->twist.angular; //angular velocity is already in body frame
        gyro[0] = angular_vel.x();
        gyro[1] = angular_vel.y();
        gyro[2] = angular_vel.z();
    }

    virtual void reset() override 
    {
        IBoard::reset();

        motor_output_.assign(params_->motor.motor_count, 0);
        input_channels_.assign(params_->rc.channel_count, 0);
        is_connected_ = false;
    }

    virtual void update() override
    {
        IBoard::update();

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
    std::vector<float> motor_output_;
    std::vector<float> input_channels_;
    bool is_connected_;

    const simple_flight::Params* params_;
    const Kinematics::State* kinematics_;
};

}} //namespace
#endif
