// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_CarControllerBase_hpp
#define air_CarControllerBase_hpp

#include "controllers/VehicleCameraBase.hpp"

namespace msr { namespace airlib {

class CarControllerBase {
public:
    virtual vector<VehicleCameraBase::ImageResponse> simGetImages(const vector<VehicleCameraBase::ImageRequest>& request) = 0;
    virtual vector<uint8_t> simGetImage(uint8_t camera_id, VehicleCameraBase::ImageType image_type) = 0;
};


}} //namespace
#endif