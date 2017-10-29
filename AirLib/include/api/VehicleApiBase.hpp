// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_VehicleApiBase_hpp
#define air_VehicleApiBase_hpp

#include "common/CommonStructs.hpp"
#include "controllers/VehicleCameraBase.hpp"

namespace msr { namespace airlib {


class VehicleApiBase {
public:
    virtual GeoPoint getHomeGeoPoint() = 0;
    virtual void enableApiControl(bool is_enabled) = 0;
    virtual bool isApiControlEnabled() = 0;
    virtual void reset() = 0;

    virtual vector<VehicleCameraBase::ImageResponse> simGetImages(const vector<VehicleCameraBase::ImageRequest>& request) = 0;
    virtual vector<uint8_t> simGetImage(uint8_t camera_id, VehicleCameraBase::ImageType image_type) = 0;

    virtual void simSetPose(const Pose& pose, bool ignore_collison) = 0;
    virtual Pose simGetPose() = 0;

    virtual void simSetSegmentationObjectID(const std::string& mesh_name, int object_id, bool is_name_regex = false) = 0;
    virtual int simGetSegmentationObjectID(const std::string& mesh_name) = 0;

    virtual ~VehicleApiBase() = default;
};


}} //namespace
#endif
