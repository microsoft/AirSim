// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_Px4HILQuadX_hpp
#define msr_airlib_vehicles_Px4HILQuadX_hpp

#include "vehicles/MultiRotorParams.hpp"
#include "controllers/MavLinkDroneController.hpp"



namespace msr { namespace airlib {

class Px4HILQuadX : public MultiRotorParams {
public:
    Px4HILQuadX(const MavLinkDroneController::ConnectionInfo& connection_info)
        : connection_info_(connection_info)
    {
    }

protected:
    virtual void setup(Params& params, SensorCollection& sensors, unique_ptr<DroneControllerBase>& controller) override
    {
        //set up arm lengths
        //dimensions are for F450 frame: http://artofcircuits.com/product/quadcopter-frame-hj450-with-power-distribution
        params.rotor_count = 4;
        std::vector<real_T> arm_lengths(params.rotor_count, 0.2275f);
		std::vector<real_T> arm_angles(params.rotor_count, 45);

		//set up mass
        params.mass = 1.0f; //can be varied from 0.800 to 1.600
        real_T motor_assembly_weight = 0.055f;  //weight for MT2212 motor for F450 frame
        real_T box_mass = params.mass - params.rotor_count * motor_assembly_weight;

        // using rotor_param default, but if you want to change any of the rotor_params, call calculateMaxThrust() to recompute the max_thrust
        // given new thrust coefficients, motor max_rpm and propeller diameter.
        params.rotor_params.calculateMaxThrust();

        //set up dimensions of core body box or abdomen (not including arms).
        params.body_box.x = 0.180f; params.body_box.y = 0.11f; params.body_box.z = 0.040f;
        real_T rotor_z = 2.5f / 100;

		//computer rotor poses
		initializeRotorQuadX(params.rotor_poses, params.rotor_count, arm_lengths.data(), arm_angles.data(), rotor_z);
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
