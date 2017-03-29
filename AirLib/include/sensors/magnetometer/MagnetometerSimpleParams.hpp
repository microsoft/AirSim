// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_MagnetometerSimpleParams_hpp
#define msr_airlib_MagnetometerSimpleParams_hpp

#include "common/Common.hpp"


namespace msr { namespace airlib {


struct MagnetometerSimpleParams {
    enum ReferenceSource {
        ReferenceSource_Constant,
        ReferenceSource_DipoleModel
    };

    Vector3r noise_sigma = Vector3r(0.005f, 0.005f, 0.005f); //5 mgauss as per specs sheet (RMS is same as stddev) https://goo.gl/UOz6FT
    real_T scale_factor = 2.0f;
    Vector3r noise_bias = Vector3r(0.0f, 0.0f, 0.0f); //no offset as per specsheet (zero gauss level) https://goo.gl/UOz6FT
    float ref_update_frequency = 0.2f;    //Hz

    //use dipole model is there is enough compute power available
    bool dynamic_reference_source = true;
    ReferenceSource ref_source = ReferenceSource::ReferenceSource_DipoleModel;
    //bool dynamic_reference_source = false;
    //ReferenceSource ref_source = ReferenceSource::ReferenceSource_Constant;
};


}} //namespace
#endif
