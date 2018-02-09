// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_DistanceSimpleParams_hpp
#define msr_airlib_DistanceSimpleParams_hpp

#include "common/Common.hpp"


namespace msr { namespace airlib {


struct DistanceSimpleParams {
    real_T min_distance = 20.0f / 100; //m
    real_T max_distance = 4000.0f / 100; //m
    Pose relative_pose;

/*
    Ref: A Stochastic Approach to Noise Modeling for Barometric Altimeters
     Angelo Maria Sabatini* and Vincenzo Genovese
     Sample values are from Table 1
     https://www.ncbi.nlm.nih.gov/pmc/articles/PMC3871085/
     This is however not used because numbers mentioned in paper doesn't match experiments.

     real_T correlated_noise_sigma = 0.27f;
     real_T correlated_noise_tau = 0.87f;
     real_T unnorrelated_noise_sigma = 0.24f;

*/
    //TODO: update sigma based on documentation, maybe as a function increasing with measured distance
    real_T unnorrelated_noise_sigma = 0.002f * 100;
    //jMavSim uses below
    //real_T unnorrelated_noise_sigma = 0.1f;

    //see PX4 param reference for EKF: https://dev.px4.io/en/advanced/parameter_reference.html
    real_T update_latency = 0.0f;    //sec
    real_T update_frequency = 50;    //Hz
    real_T startup_delay = 0;        //sec
};


}} //namespace
#endif
