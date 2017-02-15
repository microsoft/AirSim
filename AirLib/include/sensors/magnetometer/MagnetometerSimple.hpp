// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_air_copter_sim_MagnetometerSimple_hpp
#define msr_air_copter_sim_MagnetometerSimple_hpp

#include <random>
#include "common/Common.hpp"
#include "common/EarthUtils.hpp"
#include "MagnetometerSimpleParams.hpp"
#include "MagnetometerBase.hpp"
#include "common/FrequencyLimiter.hpp"


namespace msr { namespace airlib {

class MagnetometerSimple : public MagnetometerBase {
public: 
    MagnetometerSimple()
    {
        MagnetometerSimple::reset();
    }
    MagnetometerSimple(GroundTruth* ground_truth)
    {
        initialize(ground_truth);
    }
    void initialize(GroundTruth* ground_truth)
    {
        MagnetometerBase::initialize(ground_truth);
        
        noise_vec_ = RandomVectorGaussianR(Vector3r::Zero(), params_.noise_sigma);
        bias_vec_ = RandomVectorR(-params_.noise_bias, params_.noise_bias).next();

        MagnetometerSimple::reset();
    }


    //*** Start: UpdatableObject implementation ***//
    virtual void reset() override
    {
        updateReference(getGroundTruth());

        noise_vec_.reset();
        updateOutput(0);
    }

    virtual void update(real_T dt) override
    {
        updateOutput(dt);
    }
    //*** End: UpdatableObject implementation ***//

    virtual ~MagnetometerSimple() = default;

private: //methods
    void updateReference(const GroundTruth& ground_truth)
    {
        switch (params_.ref_source)
        {
        case MagnetometerSimpleParams::ReferenceSource::ReferenceSource_Constant:
            //constant from jMavSim that we know works well
            magnetic_field_true_ = Vector3r(0.44831f, 0.01664f, 0.89372f); //Tesla to Gauss
            break;
        case MagnetometerSimpleParams::ReferenceSource::ReferenceSource_DipoleModel:
            magnetic_field_true_ = EarthUtils::getMagField(ground_truth.environment->getState().geo_point) * 1E4f; //Tesla to Gauss
            break;
        default:
            throw std::invalid_argument("magnetic reference source type is not recognized");
        }
    }
    void updateOutput(real_T dt)
    {
        Output output;
        const GroundTruth& ground_truth = getGroundTruth();

        if (params_.dynamic_reference_source)
            updateReference(ground_truth); 

        // Calculate the magnetic field noise.
        // Calculate the magnetic field noise.
        output.magnetic_field_body = VectorMath::transformToBodyFrame(magnetic_field_true_, //Vector3r(0.44831f, 0.01664f, 0.89372f)
            ground_truth.kinematics->pose.orientation, true) * params_.scale_factor
            + noise_vec_.next()
            + bias_vec_;

        setOutput(output);
    }

private:
    RandomVectorGaussianR noise_vec_;
    Vector3r bias_vec_;

    Vector3r magnetic_field_true_;
    MagnetometerSimpleParams params_;
};

}} //namespace
#endif 
