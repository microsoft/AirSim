// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_RotorParams_hpp
#define msr_airlib_RotorParams_hpp


#include "common/Common.hpp"

namespace msr { namespace airlib {


//In NED system, +ve torque would generate clockwise rotation
enum class RotorTurningDirection:int {
    RotorTurningDirectionCCW = -1,
    RotorTurningDirectionCW = 1
};

struct RotorParams {
    /*
    Ref: http://physics.stackexchange.com/a/32013/14061
    force in Newton = C_T * \rho * n^2 * D^4
    torque in N.m = C_P * \rho * n^2 * D^5 / (2*pi)
    where,
    \rho = air density (1.225 kg/m^3)
    n = revolusions per sec
    D = propeller diameter in meters
    C_T, C_P = dimensionless constants available at 
        propeller performance database http://m-selig.ae.illinois.edu/props/propDB.html

    We use values for GWS 9X5 propeller for which,
    C_T = 0.109919, C_P = 0.040164 @ 6396.667 RPM
    */
    real_T max_speed_square = pow(6396.667f / 60 * 2 * M_PIf, 2.0f);  //rad/sec, 6396.667 RPM
    real_T max_thrust = 4.179446268f; //computed from above formula
    real_T max_torque =  0.055562f; //computed from above formula
    real_T control_signal_filter_tc = 0.005f;    //time constant for low pass filter
    real_T propeller_radius = 0.2286f;   //diameter in meters, default is for DJI Phantom 2
    real_T propeller_height = 1 / 100.0f;   //height of cylinderical area when propeller rotates, 1 cm
};


}} //namespace
#endif
