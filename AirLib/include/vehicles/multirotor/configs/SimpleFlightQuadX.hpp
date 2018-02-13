// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_SimpleFlightQuadX_hpp
#define msr_airlib_vehicles_SimpleFlightQuadX_hpp

#include "vehicles/multirotor/firmwares/simple_flight/SimpleFlightDroneController.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "common/AirSimSettings.hpp"
#include "sensors/SensorFactory.hpp"


namespace msr { namespace airlib {

class SimpleFlightQuadX : public MultiRotorParams {
public:
    SimpleFlightQuadX(const AirSimSettings::VehicleSettings& vehicle_settings, std::shared_ptr<const SensorFactory> sensor_factory)
        : vehicle_settings_(vehicle_settings), sensor_factory_(sensor_factory)
    {
    }

    virtual ~SimpleFlightQuadX() = default;

protected:
    virtual void setupParams() override
    {
        auto& params = getParams();

        /******* Below is same config as PX4 generic model ********/

        //set up arm lengths
        //dimensions are for F450 frame: http://artofcircuits.com/product/quadcopter-frame-hj450-with-power-distribution
        params.rotor_count = 4;
        std::vector<real_T> arm_lengths(params.rotor_count, 0.2275f);

        //set up mass
        params.mass = 1.0f; //can be varied from 0.800 to 1.600
        real_T motor_assembly_weight = 0.055f;  //weight for MT2212 motor for F450 frame
        real_T box_mass = params.mass - params.rotor_count * motor_assembly_weight;

        // using rotor_param default, but if you want to change any of the rotor_params, call calculateMaxThrust() to recompute the max_thrust
        // given new thrust coefficients, motor max_rpm and propeller diameter.
        params.rotor_params.calculateMaxThrust();

        //set up dimensions of core body box or abdomen (not including arms).
        params.body_box.x() = 0.180f; params.body_box.y() = 0.11f; params.body_box.z() = 0.040f;
        real_T rotor_z = 2.5f / 100;

        //computer rotor poses
        initializeRotorQuadX(params.rotor_poses, params.rotor_count, arm_lengths.data(), rotor_z);

        //compute inertia matrix
        computeInertiaMatrix(params.inertia, params.body_box, params.rotor_poses, box_mass, motor_assembly_weight);

        //leave everything else to defaults
    }

    virtual std::unique_ptr<SensorBase> createSensor(SensorBase::SensorType sensor_type) override
    {
        return sensor_factory_->createSensor(sensor_type);
    }

    virtual std::unique_ptr<DroneControllerBase> createController() override
    {
        return std::unique_ptr<DroneControllerBase>(new SimpleFlightDroneController(this, vehicle_settings_));
    }


private:
    vector<unique_ptr<SensorBase>> sensor_storage_;
    AirSimSettings::VehicleSettings vehicle_settings_;
    std::shared_ptr<const SensorFactory> sensor_factory_;
};

}} //namespace
#endif
