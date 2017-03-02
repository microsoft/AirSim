// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_MultiRotorParameters_hpp
#define msr_airlib_MultiRotorParameters_hpp

#include "common/Common.hpp"
#include "RotorParams.hpp"
#include "sensors/SensorCollection.hpp"
#include "controllers/DroneControllerBase.hpp"

namespace msr { namespace airlib {

class MultiRotorParams {
//All units are SI
public: //types
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

    struct EnabledSensors {
        bool imu = true;
        bool magnetometer = true;
        bool gps = true;
        bool barometer = true;
    };

    //TODO: support arbitrary shapes for cor body via interfaces
    struct BoxDimensions {
        real_T x, y, z; //box in the center with dimensions x, y and z
    };

    struct Params {
        /*********** required parameters ***********/
        uint rotor_count;
        vector<RotorPose> rotor_poses;
        real_T mass;
        Matrix3x3r inertia;
        BoxDimensions body_box;

        /*********** optional parameters with defaults ***********/
        real_T linear_drag_coefficient = 1.3f / 4.0f; 
        //sample value 1.3 from http://klsin.bpmsg.com/how-fast-can-a-quadcopter-fly/, but divided by 4 to account
        // for nice streamlined frame design and allow higher top speed which is more fun.
        //angular coefficient is usually 10X smaller than linear, however we should replace this with exact number
        //http://physics.stackexchange.com/q/304742/14061
        real_T angular_drag_coefficient = 0.13f; 
        real_T restitution = 0.15f;
        real_T friction = 0.7f;
        EnabledSensors enabled_sensors;
        RotorParams rotor_params;
    };

public: //interface
    void initialize()
    {
        setup(params_, sensors_, controller_);
    }

    const Params& getParams() const
    {
        return params_;
    }

    SensorCollection& getSensors()
    {
        return sensors_;
    }

    //return pointer because we might have derived class
    DroneControllerBase* getController()
    {
        return controller_.get();
    }

protected: //must override by derived class
    //this method must clean up any previous initializations
    virtual void setup(Params& params, SensorCollection& sensors, unique_ptr<DroneControllerBase>& controller) = 0;

protected: //static utility functions for derived classes to use
    static void initializeRotorPoses(vector<RotorPose>& rotor_poses, uint rotor_count, real_T arm_lengths[], real_T rotor_z /* z relative to center of gravity */)
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

    static void computeInertiaMatrix(Matrix3x3r& inertia, const BoxDimensions& body_box, const vector<RotorPose>& rotor_poses,
        real_T box_mass, real_T motor_assembly_weight)
    {
        inertia = Matrix3x3r::Zero();

        //http://farside.ph.utexas.edu/teaching/336k/Newtonhtml/node64.html
        inertia(0, 0) = box_mass / 12.0f * (body_box.y*body_box.y + body_box.z*body_box.z); 
        inertia(1, 1) = box_mass / 12.0f * (body_box.x*body_box.x + body_box.z*body_box.z); 
        inertia(2, 2) = box_mass / 12.0f * (body_box.x*body_box.x + body_box.y*body_box.y); 

        for (size_t i = 0; i < rotor_poses.size(); ++i) {
            const auto& pos = rotor_poses.at(i).position;
            inertia(0, 0) += (pos.y()*pos.y() + pos.z()*pos.z()) * motor_assembly_weight;
            inertia(1, 1) += (pos.x()*pos.x() + pos.z()*pos.z()) * motor_assembly_weight;
            inertia(2, 2) += (pos.x()*pos.x() + pos.y()*pos.y()) * motor_assembly_weight;
        }
    }

private:
    Params params_;
    SensorCollection sensors_;
    std::unique_ptr<DroneControllerBase> controller_;
};

}} //namespace
#endif
