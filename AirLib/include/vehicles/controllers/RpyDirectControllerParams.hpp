// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_air_copter_sim_RpyDirectControllerParams_hpp
#define msr_air_copter_sim_RpyDirectControllerParams_hpp


#include "common/Common.hpp"

namespace msr { namespace airlib {


struct RpyDirectControllerParams {
    real_T throttle_scale = 1, roll_scale = 20, pitch_scale = 20, yaw_scale = 2;
    uint rotor_count = 4;
};


}} //namespace
#endif
