// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_air_copter_sim_MultiRotorParameters_hpp
#define msr_air_copter_sim_MultiRotorParameters_hpp

#include "common/Common.hpp"
#include "RotorParams.hpp"



namespace msr { namespace airlib {

//needs separate include because of circular reference in controller	
struct MultiRotorParams {
    struct RotorPose {
        Vector3r position;  //relative to center of gravity of vehicle body
        Vector3r normal;
        RotorTurningDirection direction;

        RotorPose()
        {}
        RotorPose(const Vector3r& position_val, const Vector3r& normal_val, RotorTurningDirection direction_val)
            : position(position_val), normal(normal_val), direction(direction_val)
        {}
    };

    vector<RotorPose> rotor_poses;
    
    real_T mass = 1;
    real_T motor_assembly_weight = 0.055f;  //weight for MT2212 motor for F450 frame
    Matrix3x3r inertia = Matrix3x3r::Identity();

    real_T linear_drag_coefficient = 1.3f / 4.0f; 
	//sample value 1.3 from http://klsin.bpmsg.com/how-fast-can-a-quadcopter-fly/, but divided by 4 to account
	// for nice streamlined frame design and allow higher top speed which is more fun.
    //angular coefficient is usually 10X smaller than linear, however we should replace this with exact number
    //http://physics.stackexchange.com/q/304742/14061
    real_T angular_drag_coefficient = 0.13f; 
    real_T restitution = 0.15f;
    real_T friction = 0.7f;

    struct dimensions {
        real_T x, y, z; //box in the center with dimensions x, y and z
    } dim;

    RotorParams rotor_params;
    
    struct EnabledSensors {
    	bool imu = true;
    	bool magnetometer = true;
    	bool gps = true;
        bool barometer = true;
    } enabled_sensors;

    void initializeRotorPoses(uint rotor_count, real_T arm_lengths[], real_T rotor_z /* z relative to center of gravity */)
    {
        Vector3r unit_z(0, 0, -1);  //NED frame
        if (rotor_count == 4) {
            rotor_poses.clear();
            uint rotor_index = 0;
            /*
                           ^ x
                      (2)  |   (0)
                           |
                    --------------- y
                           |
                      (1)  |   (3)
                           |

            */
            Quaternionr quadx_rot(AngleAxisr(M_PIf / 4, unit_z));
            rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(0, arm_lengths[0], rotor_z), quadx_rot, true),
                unit_z, getRotorTurningDirection(rotor_index++));
            rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(0, -arm_lengths[1], rotor_z), quadx_rot, true),
                unit_z, getRotorTurningDirection(rotor_index++));
            rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(arm_lengths[2], 0, rotor_z), quadx_rot, true), unit_z,
                getRotorTurningDirection(rotor_index++));
            rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(-arm_lengths[3], 0, rotor_z), quadx_rot, true), unit_z,
                getRotorTurningDirection(rotor_index++));
        } else
            throw std::invalid_argument("Rotor count other than 4 is not supported yet!");
    }

    static RotorTurningDirection getRotorTurningDirection(uint rotor_index)
    {
        return (rotor_index < 2) ? RotorTurningDirection::RotorTurningDirectionCCW : RotorTurningDirection::RotorTurningDirectionCW;
    }
};

}} //namespace
#endif
