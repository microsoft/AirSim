// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_RosFlightQuadX_hpp
#define msr_airlib_vehicles_RosFlightQuadX_hpp

#include "vehicles/multirotor/firmwares/ros_flight/RosFlightDroneController.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "common/AirSimSettings.hpp"
#include "sensors/SensorFactory.hpp"


namespace msr { namespace airlib {

class RosFlightQuadX : public MultiRotorParams {
public:
    RosFlightQuadX(const AirSimSettings::VehicleSettings& vehicle_settings, std::shared_ptr<const SensorFactory> sensor_factory)
        : sensor_factory_(sensor_factory)
    {
        unused(vehicle_settings);
    }

protected:
    virtual void setupParams() override
    {
        auto& params = getParams();

        //set up arm lengths
        //dimensions are for F450 frame: http://artofcircuits.com/product/quadcopter-frame-hj450-with-power-distribution
        params.rotor_count = 4;
        std::vector<real_T> arm_lengths(params.rotor_count, 0.2275f);

        //set up mass
        params.mass = 2.856f;
        //real_T motor_assembly_weight = 0.0800f;  //weight for MT2212 motor for F450 frame
        //real_T box_mass = params.mass - params.rotor_count * motor_assembly_weight;

        //set up dimensions of core body box
        params.body_box.x() = 0.35f; params.body_box.y() = 0.20f; params.body_box.z() = 0.22f;

        //TODO: support ground effects https://github.com/byu-magicc/fcu_sim/blob/RC1.0/fcu_sim/agents/mikey/mikey.yaml

        //setup rotor params
        //TODO: support arbitrary force vector generation: https://github.com/byu-magicc/fcu_sim/blob/RC1.0/fcu_sim/agents/mikey/mikey.yaml
        params.rotor_params.max_thrust = 14;    // 9.00225f;
        params.rotor_params.max_torque = 0.12531f;
        params.rotor_params.control_signal_filter_tc = 0.2164f;

        //setup rotor poses
        params.rotor_poses.clear();
        params.rotor_poses.emplace_back(Vector3r(0.230f, 0.1926f,  -0.0762f), Vector3r(0.0223925f, -0.02674078f,  -0.99939157f), RotorTurningDirection::RotorTurningDirectionCCW);
        params.rotor_poses.emplace_back(Vector3r(-0.205f, -0.1907f, -0.0762f), Vector3r(-0.02375588f, 0.02553726f, -0.99939157f), RotorTurningDirection::RotorTurningDirectionCCW);
        params.rotor_poses.emplace_back(Vector3r(0.205f, -0.1907f, -0.0762f), Vector3r(0.02375588f, 0.02553726f, -0.99939157f), RotorTurningDirection::RotorTurningDirectionCW);
        params.rotor_poses.emplace_back(Vector3r(-0.230f, 0.1926f, -0.0762f), Vector3r(-0.0223925f,  -0.02674078f, -0.99939157f), RotorTurningDirection::RotorTurningDirectionCW);

        params.inertia = Matrix3x3r::Zero();
        params.inertia(0, 0) = 0.07f;
        params.inertia(1, 1) = 0.08f;
        params.inertia(2, 2) = 0.12f;

        //leave everything else to defaults
    }

    virtual std::unique_ptr<SensorBase> createSensor(SensorBase::SensorType sensor_type) override
    {
        return sensor_factory_->createSensor(sensor_type);
    }

    virtual std::unique_ptr<DroneControllerBase> createController() override
    {
        return std::unique_ptr<DroneControllerBase>(new RosFlightDroneController(& getSensors(), this));
    }

private:
    vector<unique_ptr<SensorBase>> sensor_storage_;
    std::shared_ptr<const SensorFactory> sensor_factory_;

};

}} //namespace
#endif
