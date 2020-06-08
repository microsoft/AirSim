// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_Px4MultiRotor_hpp
#define msr_airlib_vehicles_Px4MultiRotor_hpp

#include "vehicles/multirotor/firmwares/mavlink/MavLinkMultirotorApi.hpp"
#include "common/AirSimSettings.hpp"
#include "sensors/SensorFactory.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"

namespace msr { namespace airlib {

class Px4MultiRotorParams : public MultiRotorParams {
public:
    Px4MultiRotorParams(const AirSimSettings::MavLinkVehicleSetting& vehicle_setting, std::shared_ptr<const SensorFactory> sensor_factory)
        : sensor_factory_(sensor_factory)
    {
        connection_info_ = getConnectionInfo(vehicle_setting);
    }

    virtual ~Px4MultiRotorParams() = default;

    virtual std::unique_ptr<MultirotorApiBase> createMultirotorApi() override
    {
        unique_ptr<MultirotorApiBase> api(new MavLinkMultirotorApi());
        auto api_ptr = static_cast<MavLinkMultirotorApi*>(api.get());
        api_ptr->initialize(connection_info_, &getSensors(), true);

        return api;
    }

    virtual void setupParams() override
    {
        auto& params = getParams();

        if (connection_info_.model == "Blacksheep") {
            setupFrameBlacksheep(params);
        }
        else if (connection_info_.model == "Flamewheel") {
            setupFrameFlamewheel(params);
        }
        else if (connection_info_.model == "FlamewheelFLA") {
            setupFrameFlamewheelFLA(params);
        }
        else if (connection_info_.model == "Hexacopter") {
            setupFrameGenericHex(params);
        }
        else //Generic
            setupFrameGenericQuad(params);
    }

protected:
    virtual const SensorFactory* getSensorFactory() const override
    {
        return sensor_factory_.get();
    }

private:
    void setupFrameGenericQuad(Params& params)
    {
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
    }

    void setupFrameGenericHex(Params& params)
    {
        //set up arm lengths
        //dimensions are for F450 frame: http://artofcircuits.com/product/quadcopter-frame-hj450-with-power-distribution
        params.rotor_count = 6;
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
        initializeRotorHexX(params.rotor_poses, params.rotor_count, arm_lengths.data(), rotor_z);

        //compute inertia matrix
        computeInertiaMatrix(params.inertia, params.body_box, params.rotor_poses, box_mass, motor_assembly_weight);
    }

    void setupFrameFlamewheel(Params& params)
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

        //set up dimensions of core body box or abdomen (not including arms).
        params.body_box.x() = 0.16f; params.body_box.y() = 0.10f; params.body_box.z() = 0.14f;
        real_T rotor_z = 0.15f;

        //computer rotor poses
        initializeRotorQuadX(params.rotor_poses, params.rotor_count, arm_lengths.data(), rotor_z);

        //compute inertia matrix
        computeInertiaMatrix(params.inertia, params.body_box, params.rotor_poses, box_mass, motor_assembly_weight);
    }

    void setupFrameFlamewheelFLA(Params& params)
    {
    //set up arm lengths
    //dimensions are for F450 frame: http://artofcircuits.com/product/quadcopter-frame-hj450-with-power-distribution
    params.rotor_count = 4;
    std::vector<real_T> arm_lengths(params.rotor_count, 0.225f);

    //set up mass
    params.mass = 2.25f;
    real_T motor_assembly_weight = 0.1f;
    real_T box_mass = params.mass - params.rotor_count * motor_assembly_weight;

    params.rotor_params.C_T = 0.2f;
    params.rotor_params.C_P = 0.1f;
    params.rotor_params.max_rpm = 9324;
    params.rotor_params.calculateMaxThrust();
    params.linear_drag_coefficient *= 4; // make top speed more real.

    //set up dimensions of core body box or abdomen (not including arms).
    params.body_box.x() = 0.16f; params.body_box.y() = 0.10f; params.body_box.z() = 0.14f;
    real_T rotor_z = 0.15f;

    //computer rotor poses
    initializeRotorQuadX(params.rotor_poses, params.rotor_count, arm_lengths.data(), rotor_z);

    //compute inertia matrix
    computeInertiaMatrix(params.inertia, params.body_box, params.rotor_poses, box_mass, motor_assembly_weight);
    }

    void setupFrameBlacksheep(Params& params)
    {
        /*
        Motor placement:
        x
        (2)  |   (0)
        |
        ------------ y
        |
        (1)  |   (3)
        |

        */
        //set up arm lengths
        //dimensions are for Team Blacksheep Discovery (http://team-blacksheep.com/products/product:98)
        params.rotor_count = 4;
        std::vector<real_T> arm_lengths;

        Vector3r unit_z(0, 0, -1);  //NED frame

        // relative to Forward vector in the order (0,3,1,2) required by quad X pattern
        // http://ardupilot.org/copter/_images/MOTORS_QuadX_QuadPlus.jpg
        arm_lengths.push_back(0.22f);
        arm_lengths.push_back(0.255f);
        arm_lengths.push_back(0.22f);
        arm_lengths.push_back(0.255f);

        // note: the Forward vector is actually the "x" axis, and the AngleAxisr rotation is pointing down and is left handed, so this means the rotation
        // is counter clockwise, so the vector (arm_lengths[i], 0) is the X-axis, so the CCW rotations to position each arm correctly are listed below:
        // See measurements here: http://diydrones.com/profiles/blogs/arducopter-tbs-discovery-style (angles reversed because we are doing CCW rotation)
        std::vector<real_T> arm_angles;
        arm_angles.push_back(-55.0f);
        arm_angles.push_back(125.0f);
        arm_angles.push_back(55.0f);
        arm_angles.push_back(-125.0f);

        // quad X pattern 
        std::vector<RotorTurningDirection> rotor_directions;
        rotor_directions.push_back(RotorTurningDirection::RotorTurningDirectionCCW);
        rotor_directions.push_back(RotorTurningDirection::RotorTurningDirectionCCW);
        rotor_directions.push_back(RotorTurningDirection::RotorTurningDirectionCW);
        rotor_directions.push_back(RotorTurningDirection::RotorTurningDirectionCW);

        // data from
        // http://dronesvision.net/team-blacksheep-750kv-motor-esc-set-for-tbs-discovery-fpv-quadcopter/
        //set up mass
        params.mass = 2.0f; //can be varied from 0.800 to 1.600
        real_T motor_assembly_weight = 0.052f;  // weight for TBS motors 
        real_T box_mass = params.mass - params.rotor_count * motor_assembly_weight;

        // the props we are using a E-Prop, which I didn't find in UIUC database, but this one is close:
        // http://m-selig.ae.illinois.edu/props/volume-2/plots/ef_130x70_static_ctcp.png
        params.rotor_params.C_T = 0.11f;
        params.rotor_params.C_P = 0.047f;
        params.rotor_params.max_rpm = 9500;
        params.rotor_params.calculateMaxThrust();

        //set up dimensions of core body box or abdomen (not including arms).
        params.body_box.x() = 0.20f; params.body_box.y() = 0.12f; params.body_box.z() = 0.04f;
        real_T rotor_z = 2.5f / 100;

        //computer rotor poses
        params.rotor_poses.clear();
        for (uint i = 0; i < 4; i++)
        {
            Quaternionr angle(AngleAxisr(arm_angles[i] * M_PIf / 180, unit_z));
            params.rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(arm_lengths[i], 0, rotor_z), angle, true), unit_z, rotor_directions[i]);
        };

        //compute inertia matrix
        computeInertiaMatrix(params.inertia, params.body_box, params.rotor_poses, box_mass, motor_assembly_weight);
    }


    static const AirSimSettings::MavLinkConnectionInfo& getConnectionInfo(const AirSimSettings::MavLinkVehicleSetting& vehicle_setting)
    {
        return vehicle_setting.connection_info;
    }


private:
    AirSimSettings::MavLinkConnectionInfo connection_info_;
    std::shared_ptr<const SensorFactory> sensor_factory_;

};

}} //namespace
#endif
