// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_air_copter_sim_vehicles_Pix4QuadX_hpp
#define msr_air_copter_sim_vehicles_Pix4QuadX_hpp

#include "vehicles/MultiRotorParams.hpp"

namespace msr { namespace airlib {

class Px4QuadX {
private:
    typedef msr::airlib::MultiRotorParams  MultiRotorParams;
    typedef MultiRotorParams::RotorPose RotorPose;
    MultiRotorParams params_;
public:
    Px4QuadX()
    {
        params_.mass = 1.0; //can be varied from 0.800 to 1.600
        real_T motor_assembly_weight = 0.055f;  //weight for MT2212 motor for F450 frame
        params_.inertia = Matrix3x3r::Zero();

        params_.dim.x = 0.180f; params_.dim.y = 0.11f; params_.dim.z = 0.040f;

        //dimensions are for F450 frame: http://artofcircuits.com/product/quadcopter-frame-hj450-with-power-distribution
        std::vector<real_T> arm_lengths{ 0.2275f, 0.2275f, 0.2275f, 0.2275f };
        params_.initializeRotorPoses(static_cast<uint>(arm_lengths.size()), arm_lengths.data(), 2.5f / 100);

        //http://farside.ph.utexas.edu/teaching/336k/Newtonhtml/node64.html
        real_T box_mass = params_.mass - params_.rotor_poses.size() * motor_assembly_weight;
        params_.inertia(0, 0) = box_mass / 12.0f * (params_.dim.y*params_.dim.y + params_.dim.z*params_.dim.z); 
        params_.inertia(1, 1) = box_mass / 12.0f * (params_.dim.x*params_.dim.x + params_.dim.z*params_.dim.z); 
        params_.inertia(2, 2) = box_mass / 12.0f * (params_.dim.x*params_.dim.x + params_.dim.y*params_.dim.y); 

        for (auto i = 0; i < params_.rotor_poses.size(); ++i) {
            const auto& pos = params_.rotor_poses.at(i).position;
            params_.inertia(0, 0) += (pos.y()*pos.y() + pos.z()*pos.z()) * motor_assembly_weight;
            params_.inertia(1, 1) += (pos.x()*pos.x() + pos.z()*pos.z()) * motor_assembly_weight;
            params_.inertia(2, 2) += (pos.x()*pos.x() + pos.y()*pos.y()) * motor_assembly_weight;
        }
    }

    const MultiRotorParams& getParams()
    {
        return params_;
    }

    static const MultiRotorParams& Params()
    {
        static Px4QuadX vehicle_;	
        return vehicle_.getParams();
    }

};

}} //namespace
#endif
