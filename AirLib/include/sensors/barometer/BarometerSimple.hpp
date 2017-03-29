// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_Barometer_hpp
#define msr_airlib_Barometer_hpp

#include <random>
#include "common/Common.hpp"
#include "common/EarthUtils.hpp"
#include "BarometerSimpleParams.hpp"
#include "BarometerBase.hpp"
#include "common/GaussianMarkov.hpp"


namespace msr { namespace airlib {

class BarometerSimple  : public BarometerBase {
public:
    BarometerSimple(const BarometerSimpleParams& params = BarometerSimpleParams())
        : params_(params)
    {
        pressure_factor.initialize(params_.pressure_factor_tau, params_.pressure_factor_sigma, Utils::nan<real_T>());

        uncorrelated_noise = RandomGeneratorGausianR(0.0f, params_.unnorrelated_noise_sigma);
        //correlated_noise.initialize(params_.correlated_noise_tau, params_.correlated_noise_sigma, 0.0f);
    }

    //*** Start: UpdatableState implementation ***//
    virtual void reset() override
    {
        updateOutput(0);
        pressure_factor.reset();
        correlated_noise.reset();
        uncorrelated_noise.reset();
    }

    virtual void update(real_T dt) override
    {
        updateOutput(dt);
    }
    //*** End: UpdatableState implementation ***//

    virtual ~BarometerSimple() = default;

private: //methods
    void updateOutput(real_T dt)
    {
        Output output;
        const GroundTruth& ground_truth = getGroundTruth();

        auto altitude = ground_truth.environment->getState().geo_point.altitude;
        auto pressure = EarthUtils::getStandardPressure(altitude);
        //add drift in pressure
        pressure_factor.update(dt);
        pressure += pressure * pressure_factor.getOutput();
        //add user specified offset
        pressure += EarthUtils::SeaLevelPressure - params_.qnh*100.0f;
        pressure += uncorrelated_noise.next();

        output.pressure = pressure;
        //apply altimeter formula
        //https://en.wikipedia.org/wiki/Pressure_altitude
        //TODO: use same formula as in driver code?
        output.altitude = (1 - pow(pressure / EarthUtils::SeaLevelPressure, 0.190284f)) * 145366.45f * 0.3048f;
        output.qnh = params_.qnh;

        setOutput(output);
    }

private:
    BarometerSimpleParams params_;

    GaussianMarkov pressure_factor;
    GaussianMarkov correlated_noise;
    RandomGeneratorGausianR uncorrelated_noise;
};

}} //namespace
#endif 
