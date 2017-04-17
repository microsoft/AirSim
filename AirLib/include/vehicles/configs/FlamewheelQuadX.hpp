// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_FlamewheelQuadX_hpp
#define msr_airlib_vehicles_FlamewheelQuadX_hpp

#include "vehicles/MultiRotorParams.hpp"
#include "controllers/MavLinkDroneController.hpp"



namespace msr { namespace airlib {

class FlamewheelQuadX : public MultiRotorParams {
public:
    FlamewheelQuadX(const MavLinkDroneController::ConnectionInfo& connection_info)
        : connection_info_(connection_info)
    {
    }

protected:
    virtual void setup(Params& params, SensorCollection& sensors, unique_ptr<DroneControllerBase>& controller) override
    {
        //set up arm lengths
        //dimensions are for F450 frame: http://artofcircuits.com/product/quadcopter-frame-hj450-with-power-distribution
        params.rotor_count = 4;
        std::vector<real_T> arm_lengths(params.rotor_count, 0.225f);

        //set up mass
        params.mass = 1.635f; 
        real_T motor_assembly_weight = 0.052f;  
        real_T box_mass = params.mass - params.rotor_count * motor_assembly_weight;

        params.rotor_params.C_T = 0.11f;
        params.rotor_params.C_P = 0.047f;
        params.rotor_params.max_rpm = 9500;
        params.rotor_params.calculateMaxThrust();
		params.linear_drag_coefficient *= 4; // make top speed more real.
		params.rotor_params.throttle_boost = 0;

        //set up dimensions of core body box or abdomen (not including arms).
        params.body_box.x = 0.16f; params.body_box.y = 0.10f; params.body_box.z = 0.14f;
        real_T rotor_z = 0.15f;

        //computer rotor poses
        initializeRotorQuadX(params.rotor_poses, params.rotor_count, arm_lengths.data(), rotor_z);
        //compute inertia matrix
        computeInertiaMatrix(params.inertia, params.body_box, params.rotor_poses, box_mass, motor_assembly_weight);
        //create sensors
        createStandardSensors(sensor_storage_, sensors, params.enabled_sensors);
        //create MavLink controller for PX4
        createController(controller, sensors);
    }

private:
    void createController(unique_ptr<DroneControllerBase>& controller, SensorCollection& sensors)
    {
        controller.reset(new MavLinkDroneController());
        auto mav_controller = static_cast<MavLinkDroneController*>(controller.get());
        mav_controller->initialize(connection_info_, &sensors, true);
    }


private:
    vector<unique_ptr<SensorBase>> sensor_storage_;
    MavLinkDroneController::ConnectionInfo connection_info_;
};

}} //namespace
#endif
