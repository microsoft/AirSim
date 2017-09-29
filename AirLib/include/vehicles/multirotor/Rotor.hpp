// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.



#ifndef airsimcore_motor_hpp
#define airsimcore_motor_hpp

#include <limits>
#include "common/Common.hpp"
#include "physics/Environment.hpp"
#include "common/FirstOrderFilter.hpp"
#include "physics/PhysicsBodyVertex.hpp"
#include "RotorParams.hpp"

namespace msr { namespace airlib {

class Rotor : public PhysicsBodyVertex {
public: //types
    struct Output {
        real_T thrust;
        real_T torque_scaler;
        real_T speed;
        RotorTurningDirection turning_direction;
        real_T control_signal_filtered;
        real_T control_signal_input;
    };

public: //methods
    Rotor()
    {
        //allow default constructor with later call for initialize
    }
    Rotor(const Vector3r& position, const Vector3r& normal, RotorTurningDirection turning_direction, 
        const RotorParams& params, const Environment* environment, uint id = -1)
    {
        initialize(position, normal, turning_direction, params, environment, id);
    }
    void initialize(const Vector3r& position, const Vector3r& normal, RotorTurningDirection turning_direction, 
        const RotorParams& params, const Environment* environment, uint id = -1)
    {
        id_ = id;
        params_ = params;
        turning_direction_ = turning_direction;
        environment_ = environment;
        air_density_sea_level_ = EarthUtils::getAirDensity(0.0f);
        
        control_signal_filter_.initialize(params_.control_signal_filter_tc, 0, 0);
        
        PhysicsBodyVertex::initialize(position, normal);   //call base initializer
    }
    
    //0 to 1 - will be scalled to 0 to max_speed
    void setControlSignal(real_T control_signal)
    {
        control_signal_filter_.setInput(Utils::clip(control_signal, 0.0f, 1.0f));
    }

    Output getOutput() const
    {
        return output_;
    }

       
    //*** Start: UpdatableState implementation ***//
    virtual void reset() override
    {
        PhysicsBodyVertex::reset();

        //update environmental factors before we call base
        updateEnvironmentalFactors();

        control_signal_filter_.reset();

        setOutput(output_, params_, control_signal_filter_, turning_direction_);
    }

    virtual void update() override
    {
        //update environmental factors before we call base
        updateEnvironmentalFactors();

        //this will in turn call setWrench
        PhysicsBodyVertex::update();

        //update our state
        setOutput(output_, params_, control_signal_filter_, turning_direction_);

        //update filter - this should be after so that first output is same as initial
        control_signal_filter_.update();
    }

    virtual void reportState(StateReporter& reporter) override
    {
        reporter.writeValue("Dir", static_cast<int>(turning_direction_));
        reporter.writeValue("Ctrl-in", output_.control_signal_input);
        reporter.writeValue("Ctrl-fl", output_.control_signal_filtered);
        reporter.writeValue("speed", output_.speed);
        reporter.writeValue("thrust", output_.thrust);
        reporter.writeValue("torque", output_.torque_scaler);
    }
    //*** End: UpdatableState implementation ***//


protected:
    virtual void setWrench(Wrench& wrench) override
    {
        Vector3r normal = getNormal();
        //forces and torques are proportional to air density: http://physics.stackexchange.com/a/32013/14061
        wrench.force = normal * output_.thrust * air_density_ratio_;
        wrench.torque = normal * output_.torque_scaler * air_density_ratio_; //TODO: try using filtered control here
    }

private: //methods
    static void setOutput(Output& output, const RotorParams& params, const FirstOrderFilter<real_T>& control_signal_filter, RotorTurningDirection turning_direction)
    {
        output.control_signal_input = control_signal_filter.getInput();
        output.control_signal_filtered = control_signal_filter.getOutput();
        //see relationship of rotation speed with thrust: http://physics.stackexchange.com/a/32013/14061
        output.speed = sqrt(output.control_signal_filtered * params.max_speed_square);
        output.thrust = output.control_signal_filtered * params.max_thrust;
        output.torque_scaler = output.control_signal_input * params.max_torque * static_cast<int>(turning_direction);
        output.turning_direction = turning_direction;
    }

    void updateEnvironmentalFactors()
    {
        //update air density ration - this will affect generated force and torques by rotors
        air_density_ratio_ = environment_->getState().air_density / air_density_sea_level_;
    }


private: //fields
    uint id_; //only used for debug messages
    RotorTurningDirection turning_direction_;
    RotorParams params_;
    FirstOrderFilter<real_T> control_signal_filter_;
    const Environment* environment_ = nullptr;
    real_T air_density_sea_level_, air_density_ratio_;
    Output output_;
};


}} //namespace
#endif
