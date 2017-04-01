// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_GpsSimpleParams_hpp
#define msr_airlib_GpsSimpleParams_hpp

#include "common/Common.hpp"


namespace msr { namespace airlib {

struct GpsSimpleParams {
    real_T eph_time_constant = 0.9f, epv_time_constant = 0.9f;
    real_T eph_initial = 100.0f, epv_initial = 100.0f;   //initially fully diluted positions
    real_T eph_final = 0.3f, epv_final = 0.4f;
    real_T eph_min_3d = 3.0f, eph_min_2d = 4.0f;

    real_T update_latency = 0.2f;    //sec
    real_T update_frequency = 50;    //Hz
    real_T startup_delay = 1;        //sec
};

}} //namespace
#endif
