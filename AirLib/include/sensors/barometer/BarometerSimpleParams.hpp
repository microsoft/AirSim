// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_air_copter_sim_BarometerSimpleParams_hpp
#define msr_air_copter_sim_BarometerSimpleParams_hpp

#include "common/Common.hpp"


namespace msr {
namespace airlib {


struct BarometerSimpleParams {
    //user specified sea level pressure
    real_T qnh = EarthUtils::SeaLevelPressure / 100.0f; // hPa

    //sea level min,avh,max = 950,1013,1050 ie approx 3.65% variation
    //put this in 2 std deviations to get sigma = ~18 hPa
    //stddevs for various seasons are at
    //Mariner's Pressure Atlas, David Burch, 2014
    //https://www.starpath.com/ebooksamples/9780914025382_sample.pdf
    real_T pressure_factor_sigma = 0.0365f / 2;
    real_T pressure_factor_tau = 3600;

    /*
    Ref: A Stochastic Approach to Noise Modeling for Barometric Altimeters
         Angelo Maria Sabatini* and Vincenzo Genovese
         Sample values are from Table 1
         https://www.ncbi.nlm.nih.gov/pmc/articles/PMC3871085/
    */
    real_T correlated_noise_sigma = 0.27f;
    real_T correlated_noise_tau = 0.87f;
    real_T unnorrelated_noise_sigma = 0.24f;
};


}
} //namespace
#endif
